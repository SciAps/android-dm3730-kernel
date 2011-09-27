/*
 * linux/arch/arm/plat-omap/include/plat/board-omap3logic.h
 *
 * Copyright (C) 2011 Logic Product Devleopment, Inc.
 *
 * Initial code: Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

extern volatile int omap3logic_hang;
extern int __init omap3logic_nor_init(u32 nor_cs_mask, unsigned long nor_size);
extern int __init omap3logic_nand_init(void);

#if 0
extern void dm3730logic_init_twl_external_mute(void);
extern void twl4030_set_ext_mute(int mute);
extern int twl4030_get_ext_mute(void);
extern void twl4030_set_path_mute(int mute);
#endif

