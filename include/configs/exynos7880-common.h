/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration settings for the EXYNOS 7880 boards.
 *
 * Copyright (c) 2020 Dzmitry Sankouski (dsankouski@gmail.com)
 * based on include/exynos7420-common.h
 * Copyright (C) 2016 Samsung Electronics
 * Thomas Abraham <thomas.ab@samsung.com>
 */

#ifndef __CONFIG_EXYNOS7880_COMMON_H
#define __CONFIG_EXYNOS7880_COMMON_H

/* High Level Configuration Options */
#define CONFIG_SAMSUNG			/* in a SAMSUNG core */
#define CONFIG_S5P

#include <asm/arch/cpu.h>		/* get chip and board defs */
#include <linux/sizes.h>

/* Size of malloc() pool before and after relocation */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + (80 << 20))

/* Miscellaneous configurable options */
#define CONFIG_SYS_CBSIZE		1024	/* Console I/O Buffer Size */
#define CONFIG_SYS_PBSIZE		1024	/* Print Buffer Size */

/* Boot Argument Buffer Size */
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE

/* Timer input clock frequency */
#define COUNTER_FREQUENCY		26000000

/* Device Tree */
#define CONFIG_DEVICE_TREE_LIST "exynos7880-a5y17lte"

#define CPU_RELEASE_ADDR		secondary_boot_addr

#define CONFIG_NR_DRAM_BANKS 12

#define PHYS_SDRAM_1		CONFIG_SYS_SDRAM_BASE
#define PHYS_SDRAM_1_SIZE	SDRAM_BANK_SIZE
#define PHYS_SDRAM_2		(CONFIG_SYS_SDRAM_BASE + SDRAM_BANK_SIZE)
#define PHYS_SDRAM_2_SIZE	SDRAM_BANK_SIZE
#define PHYS_SDRAM_3		(CONFIG_SYS_SDRAM_BASE + (2 * SDRAM_BANK_SIZE))
#define PHYS_SDRAM_3_SIZE	SDRAM_BANK_SIZE
#define PHYS_SDRAM_4		(CONFIG_SYS_SDRAM_BASE + (3 * SDRAM_BANK_SIZE))
#define PHYS_SDRAM_4_SIZE	SDRAM_BANK_SIZE
#define PHYS_SDRAM_5		(CONFIG_SYS_SDRAM_BASE + (4 * SDRAM_BANK_SIZE))
#define PHYS_SDRAM_5_SIZE	SDRAM_BANK_SIZE
#define PHYS_SDRAM_6		(CONFIG_SYS_SDRAM_BASE + (5 * SDRAM_BANK_SIZE))
#define PHYS_SDRAM_6_SIZE	SDRAM_BANK_SIZE
#define PHYS_SDRAM_7		(CONFIG_SYS_SDRAM_BASE + (6 * SDRAM_BANK_SIZE))
#define PHYS_SDRAM_7_SIZE	SDRAM_BANK_SIZE
#define PHYS_SDRAM_8		(CONFIG_SYS_SDRAM_BASE + (7 * SDRAM_BANK_SIZE))
#define PHYS_SDRAM_8_SIZE	SDRAM_BANK_SIZE
#define PHYS_SDRAM_9		(CONFIG_SYS_SDRAM_BASE + (8 * SDRAM_BANK_SIZE))
#define PHYS_SDRAM_9_SIZE	SDRAM_BANK_SIZE
#define PHYS_SDRAM_10		(CONFIG_SYS_SDRAM_BASE + (9 * SDRAM_BANK_SIZE))
#define PHYS_SDRAM_10_SIZE	SDRAM_BANK_SIZE
#define PHYS_SDRAM_11		(CONFIG_SYS_SDRAM_BASE + (10 * SDRAM_BANK_SIZE))
#define PHYS_SDRAM_11_SIZE	SDRAM_BANK_SIZE
#define PHYS_SDRAM_12		(CONFIG_SYS_SDRAM_BASE + (11 * SDRAM_BANK_SIZE))
#define PHYS_SDRAM_12_SIZE	SDRAM_BANK_SIZE

/* Configuration of ENV Blocks */

#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 1) \
	func(MMC, mmc, 0) \

//todo fix addresses
#ifndef MEM_LAYOUT_ENV_SETTINGS
#define MEM_LAYOUT_ENV_SETTINGS \
	"bootm_size=0x10000000\0" \
	"kernel_addr_r=0x42000000\0" \
	"fdt_addr_r=0x43000000\0" \
	"ramdisk_addr_r=0x43300000\0" \
	"scriptaddr=0x50000000\0" \
	"pxefile_addr_r=0x51000000\0"
#endif

#ifndef EXYNOS_DEVICE_SETTINGS
#define EXYNOS_DEVICE_SETTINGS \
	"stdin=serial\0" \
	"stdout=serial\0" \
	"stderr=serial\0"
#endif

#ifndef EXYNOS_FDTFILE_SETTING
#define EXYNOS_FDTFILE_SETTING
#endif

#define EXTRA_ENV_SETTINGS \
	EXYNOS_DEVICE_SETTINGS \
	EXYNOS_FDTFILE_SETTING \
	MEM_LAYOUT_ENV_SETTINGS

#ifdef CONFIG_MUIC_MANUAL_SWITCH
#define MUIC_DEFINITIONS \
"I2C_MUIC_BUS_NUM=0\0" \
"I2C_MUIC_ADDR=0x3D\0" \
"S2MU004_REG_MUIC_SW_CTRL=0xCA\0" \
"MANUAL_SW_UART_VBUS_CLOSED=0x48\0" \
"MANUAL_SW_OPEN=0x00\0" \
"S2MU004_REG_MUIC_CTRL1=0xC7\0" \
"MUIC_CTRL1_RAW_DATA_MAN_SWITCH=0x12\0" \
"MUIC_CTRL1_INITIAL=0x17\0" \
"BYTE_COUNT=1\0"

#define MUIC_CONNECT_UART \
"connect_uart_usb=" \
    "echo connecting uart to usb signal lines...; " \
    "i2c dev $I2C_MUIC_BUS_NUM; " \
    "i2c mw $I2C_MUIC_ADDR $S2MU004_REG_MUIC_SW_CTRL $MANUAL_SW_UART_VBUS_CLOSED $BYTE_COUNT; " \
    "i2c mw $I2C_MUIC_ADDR $S2MU004_REG_MUIC_CTRL1 $MUIC_CTRL1_RAW_DATA_MAN_SWITCH $BYTE_COUNT;" \
    "echo uart connected to usb signal lines;" \
    "echo CAUTON! Do NOT connect USB host before disconnecting uart ('run disconnect_uart_usb'), this will damage your device;\0"

#define MUIC_DISCONNECT_UART \
"disconnect_uart_usb=" \
    "echo disconnecting uart from usb signal lines...; " \
    "i2c dev $I2C_MUIC_BUS_NUM; " \
    "i2c mw $I2C_MUIC_ADDR $S2MU004_REG_MUIC_SW_CTRL $MANUAL_SW_OPEN $BYTE_COUNT; " \
    "i2c mw $I2C_MUIC_ADDR $S2MU004_REG_MUIC_CTRL1 $MUIC_CTRL1_INITIAL $BYTE_COUNT; " \
    "echo uart disconnected from usb signal lines;\0"
#define CONFIG_EXTRA_ENV_SETTINGS \
	EXTRA_ENV_SETTINGS \
    MUIC_DEFINITIONS \
	MUIC_DISCONNECT_UART \
	MUIC_CONNECT_UART
#else
#define CONFIG_EXTRA_ENV_SETTINGS \
	EXTRA_ENV_SETTINGS
#endif

#endif	/* __CONFIG_EXYNOS7880_COMMON_H */
