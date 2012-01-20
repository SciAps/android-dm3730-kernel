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
#include <linux/errno.h>

#include <plat/omap3logic-productid.h>
#include <plat/omap3logic-old-productid.h>
#include <plat/omap3logic-new-productid.h>


static int omap3logic_old_product_id_valid;
static int omap3logic_new_product_id_valid;

/* Return zero if product ID data is not valid or if no wifi_macaddr */
int omap3logic_extract_wifi_ethaddr(u8 *macaddr)
{
	/* Extract the MAC addr from the productID data */
	if (omap3logic_new_product_id_valid)
		return omap3logic_extract_new_wifi_ethaddr(macaddr);
	
	if (omap3logic_old_product_id_valid)
		return omap3logic_extract_old_wifi_ethaddr(macaddr);

	return 0;
}

int omap3logic_extern_mute_gpio(void)
{
	printk("%s: return -EINVAL\n", __FUNCTION__);
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
		return ret;
	}
	ret = omap3logic_fetch_old_sram_product_id_data();
	if (!ret) {
		omap3logic_old_product_id_valid = 1;
		return ret;
	}
	return ret;
}

/* Should we look for product ID data if its not already in the SRAM??? */
