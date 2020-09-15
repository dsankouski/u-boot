/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration settings for the SAMSUNG A5 2017 (A520F)(a5y17lte) board.
 *
 * Copyright (c) 2020 Dzmitry Sankouski (dsankouski@gmail.com)
 * based on configs/espresso7420.h
 * Copyright (C) 2016 Samsung Electronics
 * Thomas Abraham <thomas.ab@samsung.com>
 */

#ifndef __CONFIG_A5Y17LTE_H
#define __CONFIG_A5Y17LTE_H

#include <configs/exynos7880-common.h>

#define CONFIG_BOARD_COMMON

#define CONFIG_SYS_SDRAM_BASE		0x40000000
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_TEXT_BASE + SZ_2M - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_LOAD_ADDR		0x40001000

/* select serial console configuration */
#define CONFIG_DEFAULT_CONSOLE	"console=ttySAC2,115200n8\0"

/* DRAM Memory Banks */
#define SDRAM_BANK_SIZE		(256UL << 20UL)	/* 256 MB */

#endif	/* __CONFIG_A5Y17LTE_H */
