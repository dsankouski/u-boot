// SPDX-License-Identifier: GPL-2.0+
/*
 * Qualcomm geni serial engine UART driver
 *
 * (C) Copyright 2020 Dzmitry Sankouski <dsankouski@gmail.com>
 *
 * Based on Linux driver.
 */
#define DEBUG

#include <log.h>
#include <common.h>
#include <clk.h>
#include <dm.h>
#include <errno.h>
#include <malloc.h>
#include <serial.h>
#include <watchdog.h>
#include <asm/io.h>
#include <linux/compiler.h>
#include <dm/pinctrl.h>

/* Serial registers - this driver works in uartdm mode*/

#define UART_OVERSAMPLING	(32)

#define GENI_SER_M_CLK_CFG		(0x48)
#define GENI_SER_S_CLK_CFG		(0x4C)
#define SE_GENI_M_CMD0			0x600
#define SE_GENI_M_IRQ_CLEAR		0x618
#define SE_GENI_S_IRQ_STATUS		(0x640)
#define SE_GENI_S_IRQ_CLEAR		(0x648)
#define SE_GENI_S_IRQ_EN		(0x644)
#define SE_GENI_M_IRQ_EN		(0x614)
#define SE_GENI_TX_FIFOn		0x700
#define SE_GENI_RX_FIFOn		0x780
#define SE_GENI_TX_FIFO_STATUS		0x800
#define SE_GENI_RX_FIFO_STATUS		0x804
#define SE_GENI_TX_WATERMARK_REG	(0x80C)
#define M_TX_FIFO_WATERMARK_EN	(BIT(30))
#define DEF_TX_WM		(2)
#define SE_UART_TX_TRANS_LEN 0x270
#define SE_GENI_TX_PACKING_CFG0 0x260
#define SE_GENI_TX_PACKING_CFG1 0x264
#define SE_GENI_RX_PACKING_CFG0		0x284
#define SE_GENI_RX_PACKING_CFG1		0x288
#define SE_UART_TX_STOP_BIT_LEN		0x26c
#define SE_UART_TX_WORD_LEN		0x268
#define SE_UART_RX_WORD_LEN		0x28c
#define SE_UART_RX_TRANS_CFG		0x280
#define SE_UART_RX_PARITY_CFG		0x2a8
#define SE_UART_TX_TRANS_CFG		0x25c
#define SE_UART_TX_PARITY_CFG		0x2a4


#define SE_GENI_S_CMD0			(0x630)
#define UART_START_READ		(0x1)


#define M_CMD_DONE_EN		(BIT(0))
#define M_CMD_DONE_DISABLE_MASK		~M_CMD_DONE_EN
#define SE_GENI_M_IRQ_STATUS		(0x610)

#define M_OPCODE_SHIFT		(27)
#define S_OPCODE_SHIFT		(27)
#define M_TX_FIFO_WATERMARK_EN	(BIT(30))
#define UART_START_TX		(0x1)
#define UART_CTS_MASK		BIT(1)
#define M_SEC_IRQ_EN		(BIT(31))
#define RX_FIFO_WC_MSK		(GENMASK(24, 0))

#define S_RX_FIFO_WATERMARK_EN	(BIT(26))
#define S_RX_FIFO_LAST_EN	(BIT(27))
#define M_RX_FIFO_WATERMARK_EN	(BIT(26))
#define M_RX_FIFO_LAST_EN	(BIT(27))


/* GENI_SER_M_CLK_CFG/GENI_SER_S_CLK_CFG */
#define SER_CLK_EN			(BIT(0))
#define CLK_DIV_MSK			(GENMASK(15, 4))
#define CLK_DIV_SHFT			(4)


DECLARE_GLOBAL_DATA_PTR;

struct msm_serial_data {
	phys_addr_t base;
	unsigned chars_cnt; /* number of buffered chars */
	uint32_t chars_buf; /* buffered chars */
	uint32_t clk_bit_rate; /* data mover mode bit rate register value */
};

static int get_clk_cfg(unsigned long clk_freq, unsigned long *ser_clk)
{
	unsigned long root_freq[] = {7372800, 14745600, 19200000, 29491200,
		32000000, 48000000, 64000000, 80000000, 96000000, 100000000};
	int i;
	int match = -1;

	for (i = 0; i < ARRAY_SIZE(root_freq); i++) {
		if (clk_freq > root_freq[i])
			continue;

		if (!(root_freq[i] % clk_freq)) {
			match = i;
			break;
		}
	}
	if (match != -1)
		*ser_clk = root_freq[match];
	else
		pr_err("clk_freq %ld\n", clk_freq);
	return match;
}

static int get_clk_div_rate(unsigned int baud, unsigned long *desired_clk_rate)
{
	unsigned long ser_clk;
	int dfs_index;
	int clk_div = 0;

	*desired_clk_rate = baud * UART_OVERSAMPLING;
	dfs_index = get_clk_cfg(*desired_clk_rate, &ser_clk);
	if (dfs_index < 0) {
		pr_err("%s: Can't find matching DFS entry for baud %d\n",
								__func__, baud);
		clk_div = -EINVAL;
		goto exit_get_clk_div_rate;
	}

	clk_div = ser_clk / *desired_clk_rate;
	*desired_clk_rate = ser_clk;
exit_get_clk_div_rate:
	return clk_div;
}

int msm_serial_setbrg(struct udevice *dev, int baudrate)
{
    struct msm_serial_data *priv = dev_get_priv(dev);

	u32 clk_div;
	u32 s_clk_cfg = 0;
	u64 clk_rate;

    clk_div = get_clk_div_rate(baudrate, &clk_rate);

    s_clk_cfg |= SER_CLK_EN;
    s_clk_cfg |= (clk_div << CLK_DIV_SHFT);

    writel(s_clk_cfg, priv->base + GENI_SER_M_CLK_CFG);
	writel(s_clk_cfg, priv->base + GENI_SER_S_CLK_CFG);

	return 0;
}


static void msm_geni_serial_setup_tx(struct udevice *dev,
				unsigned int xmit_size)
{
    struct msm_serial_data *priv = dev_get_priv(dev);
	u32 m_cmd = 0;

    writel(DEF_TX_WM, priv->base + SE_GENI_TX_WATERMARK_REG);
	writel(xmit_size, priv->base + SE_UART_TX_TRANS_LEN);
	m_cmd |= (UART_START_TX << M_OPCODE_SHIFT);
	writel(m_cmd, priv->base + SE_GENI_M_CMD0);
	mb();
    u32 watermark_irq;
    do {
        watermark_irq = readl(priv->base + SE_GENI_M_IRQ_STATUS);
    }
    while (!(watermark_irq & M_TX_FIFO_WATERMARK_EN));
    writel(M_TX_FIFO_WATERMARK_EN, priv->base + SE_GENI_M_IRQ_CLEAR);
	/*
	 * Writes to enable the primary sequencer should go through before
	 * exiting this function.
	 */
	mb();
}

static void msm_geni_serial_setup_rx(struct udevice *dev)
{
    u32 geni_s_irq_en;
	u32 geni_m_irq_en;
	struct msm_serial_data *priv = dev_get_priv(dev);

	u32 s_cmd;
	s_cmd = UART_START_READ << S_OPCODE_SHIFT;
	writel(s_cmd, priv->base + SE_GENI_S_CMD0);

    geni_s_irq_en = readl(priv->base + SE_GENI_S_IRQ_EN);
    geni_m_irq_en = readl(priv->base + SE_GENI_M_IRQ_EN);

    geni_s_irq_en |= S_RX_FIFO_WATERMARK_EN | S_RX_FIFO_LAST_EN;
    geni_m_irq_en |= M_RX_FIFO_WATERMARK_EN | M_RX_FIFO_LAST_EN;

    writel(geni_s_irq_en, priv->base + SE_GENI_S_IRQ_EN);
    writel(geni_m_irq_en, priv->base + SE_GENI_M_IRQ_EN);

    mb();
}

static int msm_serial_putc(struct udevice *dev, const char ch)
{
	struct msm_serial_data *priv = dev_get_priv(dev);

    u32 m_cmd = 0;
    writel(DEF_TX_WM, priv->base + SE_GENI_TX_WATERMARK_REG);
	writel(1, priv->base + SE_UART_TX_TRANS_LEN);
	m_cmd |= (UART_START_TX << M_OPCODE_SHIFT);
	writel(m_cmd, priv->base + SE_GENI_M_CMD0);
	mb();
    u32 watermark_irq;
    do {
        watermark_irq = readl(priv->base + SE_GENI_M_IRQ_STATUS);
    }
    while (!(watermark_irq & M_TX_FIFO_WATERMARK_EN));
    writel(ch, priv->base + SE_GENI_TX_FIFOn);
    writel(M_TX_FIFO_WATERMARK_EN, priv->base + SE_GENI_M_IRQ_CLEAR);

    mb();
    /* fix pending func, to remove this code */
    u32 irq_status;
    do {
        irq_status = readl(priv->base + SE_GENI_M_IRQ_STATUS);
    }
    while (!(irq_status & M_CMD_DONE_EN));
    u32 irq_clear = M_CMD_DONE_EN;
    writel(irq_clear, priv->base + SE_GENI_M_IRQ_CLEAR);
    mb();

	return 0;
}

static int msm_serial_getc(struct udevice *dev)
{
    struct msm_serial_data *priv = dev_get_priv(dev);
	char c;
	u32 rx_fifo;
    u32 m_irq_status;
    u32 s_irq_status;

    writel(1 << S_OPCODE_SHIFT, priv->base + SE_GENI_S_CMD0);


    while (!(readl(priv->base + SE_GENI_M_IRQ_STATUS) & M_SEC_IRQ_EN)) {}

    m_irq_status = readl(priv->base + SE_GENI_M_IRQ_STATUS);
    s_irq_status = readl(priv->base + SE_GENI_S_IRQ_STATUS);
    writel(m_irq_status, priv->base + SE_GENI_M_IRQ_CLEAR);
    writel(s_irq_status, priv->base + SE_GENI_S_IRQ_CLEAR);
    while (!(readl(priv->base + SE_GENI_RX_FIFO_STATUS) & RX_FIFO_WC_MSK)) {}

    mb();
	rx_fifo = readl(priv->base + SE_GENI_RX_FIFOn);
	rx_fifo &= 0xFF;
	return rx_fifo;
}

static int msm_serial_pending(struct udevice *dev, bool input)
{
    struct msm_serial_data *priv = dev_get_priv(dev);

    if (input)
		return readl(priv->base + SE_GENI_M_IRQ_STATUS) & M_CMD_DONE_EN == 0;
	else
		return readl(priv->base + SE_GENI_M_IRQ_STATUS) & M_CMD_DONE_EN == 0;

	return 0;
}

static const struct dm_serial_ops msm_serial_ops = {
	.putc = msm_serial_putc,
//	.pending = msm_serial_pending,
	.getc = msm_serial_getc,
	.setbrg = msm_serial_setbrg,
};

static int msm_uart_clk_init(struct udevice *dev)
{
//	uint clk_rate = 921600;
//	uint clkd[2]; /* clk_id and clk_no */
//	int clk_offset;
//	struct udevice *clk_dev;
//	struct clk clk;
//	int ret;
//
//	ret = fdtdec_get_int_array(gd->fdt_blob, dev_of_offset(dev), "clock",
//				   clkd, 2);
//	if (ret)
//		return ret;
//
//	clk_offset = fdt_node_offset_by_phandle(gd->fdt_blob, clkd[0]);
//	if (clk_offset < 0)
//		return clk_offset;
//
//	ret = uclass_get_device_by_of_offset(UCLASS_CLK, clk_offset, &clk_dev);
//	if (ret)
//		return ret;
//
//	clk.id = clkd[1];
//	ret = clk_request(clk_dev, &clk);
//	if (ret < 0)
//		return ret;
//
//	ret = clk_set_rate(&clk, clk_rate);
//	clk_free(&clk);
//	if (ret < 0)
//		return ret;

	return 0;
}

static int msm_serial_probe(struct udevice *dev)
{
	int ret;
	struct msm_serial_data *priv = dev_get_priv(dev);
	debug("Probing serial driver");
	debug("msm serial base addr: 0x%p", priv->base);

	/* No need to reinitialize the UART after relocation */
	if (gd->flags & GD_FLG_RELOC)
		return 0;

	msm_geni_serial_setup_rx(dev);
	
	u32 geni_s_irq_en;
    u32 geni_m_irq_en;
	geni_s_irq_en = readl(priv->base + SE_GENI_S_IRQ_EN);
    geni_m_irq_en = readl(priv->base + SE_GENI_M_IRQ_EN);

    		geni_s_irq_en |= S_RX_FIFO_WATERMARK_EN | S_RX_FIFO_LAST_EN;
    		geni_m_irq_en |= M_RX_FIFO_WATERMARK_EN | M_RX_FIFO_LAST_EN;

    		writel(geni_s_irq_en, priv->base + SE_GENI_S_IRQ_EN);
    		writel(geni_m_irq_en, priv->base + SE_GENI_M_IRQ_EN);

//	ret = msm_uart_clk_init(dev);
//	if (ret)
//		return ret;

//	pinctrl_select_state(dev, "uart");
//	uart_dm_init(priv);

	return 0;
}

static int msm_serial_ofdata_to_platdata(struct udevice *dev)
{
	struct msm_serial_data *priv = dev_get_priv(dev);

	priv->base = dev_read_addr(dev);
	if (priv->base == FDT_ADDR_T_NONE)
		return -EINVAL;

//	priv->clk_bit_rate = fdtdec_get_int(gd->fdt_blob, dev_of_offset(dev),
//							"bit-rate", UART_DM_CLK_RX_TX_BIT_RATE);

	return 0;
}

static const struct udevice_id msm_serial_ids[] = {
	{ .compatible = "qcom,msm-geni-uart" },
	{ }
};

U_BOOT_DRIVER(serial_msm) = {
	.name	= "serial_msm_geni",
	.id	= UCLASS_SERIAL,
	.of_match = msm_serial_ids,
	.ofdata_to_platdata = msm_serial_ofdata_to_platdata,
	.priv_auto_alloc_size = sizeof(struct msm_serial_data),
	.probe = msm_serial_probe,
	.ops	= &msm_serial_ops,
};

#ifdef CONFIG_DEBUG_UART_MSM

#include <debug_uart.h>

static inline void _debug_uart_init(void)
{
    u32 base_address = CONFIG_DEBUG_UART_BASE;
    u32 tx_trans_cfg;
	u32 tx_parity_cfg = 0;	/* Disable Tx Parity */
	u32 rx_trans_cfg = 0;
	u32 rx_parity_cfg = 0;	/* Disable Rx Parity */
	u32 stop_bit_len = 0;	/* Default stop bit length - 1 bit */
	u32 bits_per_char;

	/*
	 * Ignore Flow control.
	 * n = 8.
	 */
	tx_trans_cfg = UART_CTS_MASK;
	bits_per_char = BITS_PER_BYTE;

    u32 cfg0 = 0xf;
    u32 cfg1 = 0x0;

	/*
	 * Make an unconditional cancel on the main sequencer to reset
	 * it else we could end up in data loss scenarios.
	 */
//	qcom_geni_serial_poll_tx_done(uport);
//	qcom_geni_serial_abort_rx(uport);
//	geni_se_config_packing(&se, BITS_PER_BYTE, 1, false, true, false);
	writel(cfg0, base_address + SE_GENI_TX_PACKING_CFG0);
	writel(cfg1, base_address + SE_GENI_TX_PACKING_CFG1);

//    cfg0 = 0x4380e;
//    cfg1 = 0xc3e0e;
    writel(cfg0, base_address + SE_GENI_RX_PACKING_CFG0);
	writel(cfg1, base_address + SE_GENI_RX_PACKING_CFG1);

//	geni_se_init(&se, DEF_FIFO_DEPTH_WORDS / 2, DEF_FIFO_DEPTH_WORDS - 2);
//	geni_se_select_mode(&se, GENI_SE_FIFO);

	writel(tx_trans_cfg, base_address + SE_UART_TX_TRANS_CFG);
	writel(tx_parity_cfg, base_address + SE_UART_TX_PARITY_CFG);
	writel(rx_trans_cfg, base_address + SE_UART_RX_TRANS_CFG);
	writel(rx_parity_cfg, base_address + SE_UART_RX_PARITY_CFG);
	writel(bits_per_char, base_address + SE_UART_TX_WORD_LEN);
	writel(bits_per_char, base_address + SE_UART_RX_WORD_LEN);
	writel(stop_bit_len, base_address + SE_UART_TX_STOP_BIT_LEN);

	return 0;
}



static inline void _debug_uart_putc(int ch)
{
    u32 base = CONFIG_DEBUG_UART_BASE;
    u32 m_cmd = 0;
    u32 irq_clear = M_CMD_DONE_EN;

    writel(DEF_TX_WM, base + SE_GENI_TX_WATERMARK_REG);
	writel(1, base + SE_UART_TX_TRANS_LEN);
	m_cmd |= (UART_START_TX << M_OPCODE_SHIFT);
	writel(m_cmd, base + SE_GENI_M_CMD0);
	mb();
    u32 watermark_irq;
    do {
        watermark_irq = readl(base + SE_GENI_M_IRQ_STATUS);
    }
    while (!(watermark_irq & M_TX_FIFO_WATERMARK_EN));

	writel(ch, base + SE_GENI_TX_FIFOn);
	writel(M_TX_FIFO_WATERMARK_EN, base + SE_GENI_M_IRQ_CLEAR);
    mb();

//    writel(M_TX_FIFO_WATERMARK_EN, base + SE_GENI_M_IRQ_CLEAR);
//    mb();

    u32 irq_status;
    do {
        irq_status = readl(base + SE_GENI_M_IRQ_STATUS);
    }
    while (!(irq_status & M_CMD_DONE_EN));
    writel(irq_clear, base + SE_GENI_M_IRQ_CLEAR);

    mb();
}

DEBUG_UART_FUNCS

#endif
