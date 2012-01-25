/*
 * linux/arch/arm/mach-omap2/board-oma3logic-product-id.c
 *
 * Copyright (C) 2012 Logic Product Development, Inc.
 * Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>

#include <asm/mach-types.h>

#include <plat/board-omap3logic.h>
#include <plat/omap3logic-productid.h>
#include <plat/omap3logic-old-productid.h>
#include <plat/omap3logic-new-productid.h>


static int omap3logic_old_product_id_valid;
static int omap3logic_new_product_id_valid;

/* Return zero if product ID data is good. */
int omap3logic_extract_wifi_ethaddr(u8 *macaddr)
{
	/* Extract the MAC addr from the productID data */
	if (omap3logic_new_product_id_valid)
		return omap3logic_extract_new_wifi_ethaddr(macaddr);
	
	if (omap3logic_old_product_id_valid)
		return omap3logic_extract_old_wifi_ethaddr(macaddr);

	return -ENOENT;
}

/* DM37x Torpedo boards mute is gpio_17 since DSS uses 24 pins */
#define TWL4030_DM37X_TORPEDO_EXTERNAL_AUDIO_MUTE_GPIO		17

/* OMAP35x Torpedo boards mute is gpio_177 */
#define TWL4030_OMAP35X_TORPEDO_EXTERNAL_AUDIO_MUTE_GPIO	177

/* OMAP35x/DM37x SOM LV mute is GPIO_57 */
#define TWL4030_SOM_LV_EXTERNAL_AUDIO_MUTE_GPIO			57

int omap3logic_external_mute_gpio(void)
{
	if (machine_is_dm3730_torpedo())
		return TWL4030_DM37X_TORPEDO_EXTERNAL_AUDIO_MUTE_GPIO;

	if (machine_is_omap3_torpedo())
		return TWL4030_OMAP35X_TORPEDO_EXTERNAL_AUDIO_MUTE_GPIO;

	if (machine_is_dm3730_som_lv() || machine_is_omap3530_lv_som())
		return TWL4030_SOM_LV_EXTERNAL_AUDIO_MUTE_GPIO;

	return -EINVAL;
}

/* Create the sysfs files for product ID data */
int omap3logic_create_product_id_sysfs(void)
{
	if (omap3logic_new_product_id_valid)
		return omap3logic_create_new_product_id_sysfs();

	if (omap3logic_old_product_id_valid)
		return omap3logic_create_old_product_id_sysfs();

	return 0;
}

/* Check the SRAM for valid product_id data(put there by u-boot). */
int omap3logic_fetch_sram_product_id_data(void)
{
	int ret;

	ret = omap3logic_fetch_sram_new_product_id_data();
	if (!ret) {
		omap3logic_new_product_id_valid = 1;
	} else {
		ret = omap3logic_fetch_old_sram_product_id_data();
		if (!ret)
			omap3logic_old_product_id_valid = 1;
	}

	/* If ret is zero, we *know* there's valid product_id data.
	 * Therefore configure those bits that are specific to certain
	 * types of module. */
	if (!ret)
		omap3logic_init_productid_specifics();

	return ret;
}

/* Extract the NVS data for the wm12xx; if nvs_data is null then just set
 * *nvs_data_size appropriately and return success.  Else if *nvs_data_size
 * is large enough copy the data into nvs_data and update *nvs_data_size */
int omap3logic_extract_nvs_data(u8 *nvs_data, u32 *nvs_data_size)
{
	if (omap3logic_new_product_id_valid)
		return omap3logic_extract_new_nvs_data(nvs_data, nvs_data_size);

	if (omap3logic_old_product_id_valid)
		return omap3logic_extract_old_nvs_data(nvs_data, nvs_data_size);

	return -EINVAL;
}
EXPORT_SYMBOL(omap3logic_extract_nvs_data);
