/*
 * linux/arch/arm/mach-omap2/board-omap3logic-audio.c
 *
 * Copyright (C) 2011 Logic Product Devleopment, Inc.
 *
 * Initial code: Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

extern void omap3logic_init_twl_audio(void);
extern void twl4030_set_ext_mute(int mute);
extern int twl4030_get_ext_mute(void);
extern void twl4030_set_path_mute(int mute);
