/*
 * linux/arch/arm/plat-omap/include/plat/board-omap3logic-display.h
 *
 * Copyright (C) 2011 Logic Product Devleopment, Inc.
 *
 * Initial code: Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

extern void omap3logic_lcd_init(void);
extern struct omap_dss_device omap3logic_lcd_device;
extern struct platform_device omap3logic_dss_device;

struct omap3logic_panel {
	char				*name;
	int				config;
	int				acb;
	char				data_lines;
	struct omap_video_timings	timing;
};
extern struct omap3logic_panel omap3logic_default_panel;
