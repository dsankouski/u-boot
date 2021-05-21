// SPDX-License-Identifier: GPL-2.0+
/*
 * Board init file for Dragonboard 820C
 *
 * (C) Copyright 2017 Jorge Ramirez-Ortiz <jorge.ramirez-ortiz@linaro.org>
 */

#include <init.h>
#include <env.h>
#include <common.h>
#include <asm/gpio.h>
#include <dm.h>


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

/* Check for vol- button - if pressed - stop autoboot */
int misc_init_r(void)
{
    printf("misc_init_r\n");
	struct udevice *pon;
	struct gpio_desc resin;
	int node, ret;

	ret = uclass_get_device_by_name(UCLASS_GPIO, "pm8998_pon@800", &pon);
	if (ret < 0) {
		printf("Failed to find PMIC pon node. Check device tree\n");
		return 0;
	}

	node = fdt_subnode_offset(gd->fdt_blob, dev_of_offset(pon),
				  "key_vol_down");
	if (node < 0) {
		printf("Failed to find key_vol_down node. Check device tree\n");
		return 0;
	}
    printf("key_vol_down requested\n");
	if (gpio_request_by_name_nodev(offset_to_ofnode(node), "gpios", 0,
				       &resin, 0)) {
		printf("Failed to request key_vol_down button.\n");
		return 0;
	}
    printf("pm gpio requested\n");

	if (dm_gpio_get_value(&resin)) {
		env_set("bootdelay", "-1");
		env_set("key_vol_down", "1");
		printf("Power button pressed - dropping to console.\n");
	} else {
		env_set("key_vol_down", "0");
	}

	return 0;
}