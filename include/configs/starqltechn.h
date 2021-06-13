/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Board configuration file for Samsung S9(SM-G9600)(starqltechn)
 *
 * (C) Copyright 2017 Jorge Ramirez-Ortiz <jorge.ramirez-ortiz@linaro.org>
 */

#ifndef __CONFIGS_STARQLTECHN_H
#define __CONFIGS_STARQLTECHN_H

#include <linux/sizes.h>
#include <asm/arch/sysmap-sdm845.h>

/* Physical Memory Map */

#define PHYS_SDRAM_SIZE			0xFE1BFFFF
#define PHYS_SDRAM_1			0x80000000
#define PHYS_SDRAM_1_SIZE		0x80000000
#define PHYS_SDRAM_2			0x100000000
#define PHYS_SDRAM_2_SIZE		0x7e1bffff

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_SDRAM_BASE + 0x7fff0)
#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE)
#define CONFIG_LNX_KRNL_IMG_TEXT_OFFSET_BASE    CONFIG_SYS_LOAD_ADDR
#define CONFIG_SYS_BOOTM_LEN		SZ_64M

#define CONFIG_SYS_BAUDRATE_TABLE	{ 115200, 230400, 460800, 921600 }

/* Generic Timer Definitions */
#define COUNTER_FREQUENCY		19000000

/* BOOTP options */
#define CONFIG_BOOTP_BOOTFILESIZE

//#ifndef CONFIG_SPL_BUILD
//#include <config_distro_bootcmd.h>
//#endif

#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 0)

#define EXTRA_ENV_SETTINGS \
	"loadaddr=0x80000000\0" \
	"fdt_high=0xffffffffffffffff\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"linux_image=uImage\0" \
	"kernel_addr_r=0x95000000\0"\
	"fdtfile=qcom/apq8096-db820c.dtb\0" \
	"fdt_addr_r=0x93000000\0"\
	"ramdisk_addr_r=0x91000000\0"\
	"scriptaddr=0x90000000\0"\
	"pxefile_addr_r=0x90100000\0"

#ifdef CONFIG_MUIC_MANUAL_SWITCH
#define MUIC_DEFINITIONS \
"I2C_MUIC_BUS_NUM=0\0" \
"I2C_MUIC_ADDR=0x66\0" \
"COM_UART=3\0" \
"S2MU004_REG_MUIC_SW_CTRL=0xCA\0" \
"MANUAL_SW_UART_VBUS_CLOSED=0x48\0" \
"MANUAL_SW_OPEN=0x00\0" \
"S2MU004_REG_MUIC_CTRL1=0xC7\0" \
"MUIC_CTRL1_RAW_DATA_MAN_SWITCH=0x12\0" \
"MUIC_CTRL1_INITIAL=0x17\0" \
"BYTE_COUNT=1\0"

#define MUIC_CONNECT_UART \
    "connect_uart_usb=" \
    "echo switching i2c pins to gpio mode; " \
    "mw 0x3921000 0 1" \
    "mw 0x3922000 0 1" \
    "echo connecting uart to usb signal lines...; " \
    "echo uart connected to usb signal lines;" \
    "echo CAUTON! Do NOT connect USB host before disconnecting uart ('run disconnect_uart_usb'), this will damage your device;\0"

#define MUIC_DISCONNECT_UART \
    "disconnect_uart_usb=" \
    "echo disconnecting uart from usb signal lines...; " \
    "echo uart disconnected from usb signal lines;\0"

#define CONFIG_EXTRA_ENV_SETTINGS \
    EXTRA_ENV_SETTINGS \
    MUIC_DEFINITIONS \
	MUIC_DISCONNECT_UART \
	MUIC_CONNECT_UART

#define CONFIG_PREBOOT \
"echo Check pressed buttons match uart on usb combination...;" \
"PRESSED=1;" \
"RELEASED=0;" \
"if test " \
"$key_vol_down -eq $PRESSED; " \
"then run connect_uart_usb; else " \
"echo pressed buttons does not match;" \
"fi;"

#else
#define CONFIG_EXTRA_ENV_SETTINGS EXTRA_ENV_SETTINGS
#endif


/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + SZ_8M)

/* Monitor Command Prompt */
#define CONFIG_SYS_CBSIZE		512
#define CONFIG_SYS_MAXARGS		64

#endif
