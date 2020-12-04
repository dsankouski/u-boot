// SPDX-License-Identifier: GPL-2.0+
/*
 * Board init file for Dragonboard 820C
 *
 * (C) Copyright 2017 Jorge Ramirez-Ortiz <jorge.ramirez-ortiz@linaro.org>
 */

#include <init.h>
#include <env.h>
#include <common.h>


DECLARE_GLOBAL_DATA_PTR;

int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

int board_init(void)
{
	return 0;
}

void reset_cpu(ulong addr)
{
//	psci_system_reset();
}