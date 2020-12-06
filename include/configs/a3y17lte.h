/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration settings for the SAMSUNG A3 2017 (A320F)(a3y17lte) board.
 *
 * Copyright (c) 2020 Dzmitry Sankouski (dsankouski@gmail.com)
 * based on configs/espresso7420.h
 * Copyright (C) 2016 Samsung Electronics
 * Thomas Abraham <thomas.ab@samsung.com>
 */

#ifndef __CONFIG_A3Y17LTE_H
#define __CONFIG_A3Y17LTE_H

#include <configs/exynos7870-common.h>

#define CONFIG_BOARD_COMMON

#define CONFIG_SYS_SDRAM_BASE		0x40000000
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_TEXT_BASE + SZ_2M - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_LOAD_ADDR		0x40080000

/* select serial console configuration */
#define CONFIG_DEFAULT_CONSOLE	"console=ttySAC2,115200n8\0"
#define CONFIG_DEBUG_UART_CLOCK	132710400

/* DRAM Memory Banks */
#define SDRAM_BANK_SIZE		(256UL << 20UL)	/* 256 MB */

#ifdef CONFIG_MUIC_MANUAL_SWITCH
#define CONFIG_PREBOOT \
"echo Check pressed buttons match uart on usb combination...;" \
"KEY_POWER=gpa00;" \
"KEY_HOME=gpa17;" \
"KEY_VOLUMEUP=gpa20;" \
"KEY_VOLUMEDOWN=gpa21;" \
"PRESSED=0;" \
"RELEASED=1;" \
"if gpio input $KEY_VOLUMEUP; then setenv VOLUME_UP $PRESSED; else setenv VOLUME_UP $RELEASED; fi;" \
"if gpio input $KEY_VOLUMEDOWN; then setenv VOLUME_DOWN $PRESSED; else setenv VOLUME_DOWN $RELEASED; fi;" \
"if gpio input $KEY_HOME; then setenv HOME $PRESSED; else setenv HOME $RELEASED; fi;" \
"if gpio input $KEY_POWER; then setenv POWER $PRESSED; else setenv POWER $RELEASED; fi;" \
"if test " \
"$VOLUME_UP -eq $PRESSED " \
"&& test $VOLUME_DOWN -eq $PRESSED " \
"&& test $HOME -eq $RELEASED " \
"&& test $POWER -eq $RELEASED; " \
"then run connect_uart_usb; else " \
"echo pressed buttons does not match;" \
"fi;"
#endif

#endif	/* __CONFIG_A3Y17LTE_H */
