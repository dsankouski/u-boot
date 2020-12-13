// SPDX-License-Identifier: GPL-2.0+
/*
 * Qualcomm geni serial engine UART driver
 *
 * (C) Copyright 2020 Dzmitry Sankouski <dsankouski@gmail.com>
 *
 * Based on Linux driver.
 */

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

#define SE_GENI_M_CMD0			0x600
#define SE_GENI_M_IRQ_CLEAR		0x618
#define SE_GENI_TX_FIFOn		0x700
#define SE_UART_TX_TRANS_LEN 0x270

#define M_OPCODE_SHIFT		(27)
#define M_TX_FIFO_WATERMARK_EN	(BIT(30))
#define UART_START_TX		(0x1)





#define UARTDM_DMRX             0x34 /* Max RX transfer length */
#define UARTDM_NCF_TX           0x40 /* Number of chars to TX */

#define UARTDM_RXFS             0x50 /* RX channel status register */
#define UARTDM_RXFS_BUF_SHIFT   0x7  /* Number of bytes in the packing buffer */
#define UARTDM_RXFS_BUF_MASK    0x7
#define UARTDM_MR1				 0x00
#define UARTDM_MR2				 0x04
#define UARTDM_CSR				 0xA0

#define UARTDM_SR                0xA4 /* Status register */
#define UARTDM_SR_RX_READY       (1 << 0) /* Word is the receiver FIFO */
#define UARTDM_SR_TX_EMPTY       (1 << 3) /* Transmitter underrun */
#define UARTDM_SR_UART_OVERRUN   (1 << 4) /* Receive overrun */

#define UARTDM_CR                         0xA8 /* Command register */
#define UARTDM_CR_CMD_RESET_ERR           (3 << 4) /* Clear overrun error */
#define UARTDM_CR_CMD_RESET_STALE_INT     (8 << 4) /* Clears stale irq */
#define UARTDM_CR_CMD_RESET_TX_READY      (3 << 8) /* Clears TX Ready irq*/
#define UARTDM_CR_CMD_FORCE_STALE         (4 << 8) /* Causes stale event */
#define UARTDM_CR_CMD_STALE_EVENT_DISABLE (6 << 8) /* Disable stale event */

#define UARTDM_IMR                0xB0 /* Interrupt mask register */
#define UARTDM_ISR                0xB4 /* Interrupt status register */
#define UARTDM_ISR_TX_READY       0x80 /* TX FIFO empty */

#define UARTDM_TF               0x100 /* UART Transmit FIFO register */
#define UARTDM_RF               0x140 /* UART Receive FIFO register */

#define UART_DM_CLK_RX_TX_BIT_RATE 0xCC
#define MSM_BOOT_UART_DM_8_N_1_MODE 0x34
#define MSM_BOOT_UART_DM_CMD_RESET_RX 0x10
#define MSM_BOOT_UART_DM_CMD_RESET_TX 0x20

DECLARE_GLOBAL_DATA_PTR;

struct msm_serial_data {
	phys_addr_t base;
	unsigned chars_cnt; /* number of buffered chars */
	uint32_t chars_buf; /* buffered chars */
	uint32_t clk_bit_rate; /* data mover mode bit rate register value */
};

static void msm_geni_serial_setup_tx(struct udevice *dev,
				unsigned int xmit_size)
{
    struct msm_serial_data *priv = dev_get_priv(dev);
	u32 m_cmd = 0;

	writel(xmit_size, priv->base + SE_UART_TX_TRANS_LEN);
	m_cmd |= (UART_START_TX << M_OPCODE_SHIFT);
	writel(m_cmd, priv->base + SE_GENI_M_CMD0);
	/*
	 * Writes to enable the primary sequencer should go through before
	 * exiting this function.
	 */
	mb();
}

static int msm_serial_putc(struct udevice *dev, const char ch)
{
	struct msm_serial_data *priv = dev_get_priv(dev);

    writel(ch, priv->base + SE_GENI_TX_FIFOn);
    mb();

	return 0;
}

static int msm_serial_getc(struct udevice *dev)
{


	return 0x42;
}

static int msm_serial_pending(struct udevice *dev, bool input)
{
	if (input) {
		if (msm_serial_fetch(dev))
			return 1;
	}

	return 0;
}

static const struct dm_serial_ops msm_serial_ops = {
	.putc = msm_serial_putc,
//	.pending = msm_serial_pending,
	.getc = msm_serial_getc,
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

	/* No need to reinitialize the UART after relocation */
	if (gd->flags & GD_FLG_RELOC)
		return 0;

//	ret = msm_uart_clk_init(dev);
	if (ret)
		return ret;

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

	priv->clk_bit_rate = fdtdec_get_int(gd->fdt_blob, dev_of_offset(dev), 
							"bit-rate", UART_DM_CLK_RX_TX_BIT_RATE);

	return 0;
}

static const struct udevice_id msm_serial_ids[] = {
	{ .compatible = "qcom,msm-geni-uart" },
	{ }
};

U_BOOT_DRIVER(serial_msm) = {
	.name	= "serial_geni_msm",
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
// dm init
//    writel(115200, CONFIG_DEBUG_UART_BASE + UARTDM_CSR);
//	writel(0x0, CONFIG_DEBUG_UART_BASE + UARTDM_MR1);
//	writel(MSM_BOOT_UART_DM_8_N_1_MODE, CONFIG_DEBUG_UART_BASE + UARTDM_MR2);
//	writel(MSM_BOOT_UART_DM_CMD_RESET_RX, CONFIG_DEBUG_UART_BASE + UARTDM_CR);
//	writel(MSM_BOOT_UART_DM_CMD_RESET_TX, CONFIG_DEBUG_UART_BASE + UARTDM_CR);
}

#define M_CMD_DONE_EN		(BIT(0))
#define SE_GENI_M_IRQ_STATUS		(0x610)


static inline void _debug_uart_putc(int ch)
{
    u32 base = CONFIG_DEBUG_UART_BASE;
    u32 m_cmd = 0;

	writel(1, base + SE_UART_TX_TRANS_LEN);
	m_cmd |= (UART_START_TX << M_OPCODE_SHIFT);
	writel(m_cmd, base + SE_GENI_M_CMD0);
	/*
	 * Writes to enable the primary sequencer should go through before
	 * exiting this function.
	 */
	mb();

	writel(ch, base + SE_GENI_TX_FIFOn);
    mb();

    writel(M_TX_FIFO_WATERMARK_EN, base + SE_GENI_M_IRQ_CLEAR);
    mb();

    int done = 0;
    unsigned int irq_clear = M_CMD_DONE_EN;
    while (readl(base + SE_GENI_M_IRQ_STATUS) & M_CMD_DONE_EN);
}

DEBUG_UART_FUNCS

#endif
