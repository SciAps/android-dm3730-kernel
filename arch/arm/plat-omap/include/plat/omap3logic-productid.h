/*
 * (C) Copyright 2008
 * Logic Produc Development, <www.logicpd.com>
 * Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef OMAP3LOGIC_PRODUCTID_H
#define OMAP3LOGIC_PRODUCTID_H

/* Empty for now to find all the functions needed */
extern int omap3logic_fetch_sram_product_id_data(void);
extern int omap3logic_extract_wifi_ethaddr(u8 *macaddr);
extern int omap3logic_external_mute_gpio(void);
extern int omap3logic_create_product_id_sysfs(void);
#endif
