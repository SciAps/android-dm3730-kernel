/*
 * LCD panel driver for Logic OMAP35x/DM37x reference boards
 *
 * Copyright (C) 2011 Logic Product Devleopment, Inc.
 * Author: Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>

#include <video/omapdss.h>
#include <plat/board-omap3logic-display.h>

#define DEBUG

#ifdef DEBUG
#define DPRINTK(fmt, args...) printk(fmt, ## args)
#else
#define DPRINTK(fmt, args ...)
#endif

static struct omap3logic_panel omap3logic_panels[] = {
	{
		.name	= "15",
		.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS
		| OMAP_DSS_LCD_ONOFF | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_IEO
				| OMAP_DSS_LCD_IHS,
		.acb	= 0x28,
		.timing = {
			/* 480 x 272, LQ043T1DG01 */
			.x_res		= 480,
			.y_res		= 272,
			.pixel_clock	= 9000,
			.hfp		= 3,
			.hsw		= 42,
			.hbp		= 2,
			.vfp		= 2,
			.vsw		= 11,
			.vbp		= 3,
		},
	},
	{
		.name	= "3",
		.config	= OMAP_DSS_LCD_TFT,
		.acb	= 0x28,
		.timing = {
			/* 320 x 240, LQ036Q1DA01 */
			.x_res		= 320,
			.y_res		= 240,
			.pixel_clock	= 24500,
			.hfp		= 20,
			.hsw		= 20,
			.hbp		= 20,
			.vfp		= 3,
			.vsw		= 3,
			.vbp		= 4,
		},
	},
	{
		.name	= "7",
		.config	= OMAP_DSS_LCD_TFT,
		.acb	= 0x28,
		.timing = {
			/* 640 x 480, LQ10D368 */
			.x_res		= 640,
			.y_res		= 480,
			.pixel_clock	= 27000,
			.hfp		= 24,
			.hsw		= 48,
			.hbp		= 135,
			.vfp		= 34,
			.vsw		= 1,
			.vbp		= 34,
		},
	},
	{
		.name	= "5",
		.config	= OMAP_DSS_LCD_TFT,
		.acb	= 0x28,
		.timing = {
			/* 640 x 240, LQ036Q1DA01 */
			.x_res		= 640,
			.y_res		= 480,
			.pixel_clock	= 27000,
			.hfp		= 24,
			.hsw		= 48,
			.hbp		= 135,
			.vfp		= 34,
			.vsw		= 1,
			.vbp		= 34,
		},
	},
	{
		.name	= "2",
		.config	= OMAP_DSS_LCD_TFT,
		.acb	= 0x28,
		.timing = {
			/* 800 x 600, LQ121S1DG31 */
			.x_res		= 800,
			.y_res		= 600,
			.pixel_clock	= 42000,
			.hfp		= 120,
			.hsw		= 5,
			.hbp		= 88-4-2,
			.vfp		= 100,
			.vsw		= 4,
			.vbp		= 21-1,
		},
	},
	{
		.name	= "vga",
		.config	= OMAP_DSS_LCD_TFT,
		.acb	= 0x28,
		.timing = {
			/* 640 x 480, VGA on DVI */
			.x_res		= 640,
			.y_res		= 480,
			.pixel_clock	= 24685,
			.hfp		= 16,
			.hsw		= 96,
			.hbp		= 48,
			.vfp		= 10,
			.vsw		= 2,
			.vbp		= 33,
		},
	},
	{
		.name	= "svga",
		.config	= OMAP_DSS_LCD_TFT,
		.acb	= 0x28,
		.timing = {
			/* 800 x 600, SVGA on DVI */
			.x_res		= 800,
			.y_res		= 600,
			.pixel_clock	= 42000,
			.hfp		= 120,
			.hsw		= 5,
			.hbp		= 88-4-2,
			.vfp		= 100,
			.vsw		= 4,
			.vbp		= 21-1,
		},
	},
	{
		.name	= "xga",
		.config	= OMAP_DSS_LCD_TFT,
		.acb	= 0x28,
		.timing = {
			/* 1024 x 769, XGA on DVI */
			.x_res		= 1024,
			.y_res		= 768,
			.pixel_clock	= 61714,
			.hfp		= 24,
			.hsw		= 41,
			.hbp		= 160,
			.vfp		= 3,
			.vsw		= 6,
			.vbp		= 29,
		},
	},
	{
		.name	= "sxga",
		.config	= OMAP_DSS_LCD_TFT,
		.acb	= 0x28,
		.timing = {
			/* 1280 x 1024, SXGA on DVI */
			.x_res		= 1280,
			.y_res		= 1024,
			.pixel_clock	= 108000,
			.hfp		= 81,
			.hsw		= 41,
			.hbp		= 209,
			.vfp		= 6,
			.vsw		= 6,
			.vbp		= 21,
		},
	},
	{
		.name	= "uxga",
		.config	= OMAP_DSS_LCD_TFT,
		.acb	= 0x28,
		.timing = {
			/* 1600 x 1200, UXGA on DVI */
			.x_res		= 1600,
			.y_res		= 1200,
			.pixel_clock	= 172800,
			.hfp		= 64,
			.hsw		= 192,
			.hbp		= 304,
			.vfp		= 46,
			.vsw		= 3,
			.vbp		= 1,
		},
	},
	{
		.name	= "720p",
		.config	= OMAP_DSS_LCD_TFT,
		.acb	= 0x28,
		.timing = {
			/* 1280 x 720, 720P on DVI */
			.x_res		= 1280,
			.y_res		= 720,
			.pixel_clock	= 72000,
			.hfp		= 110,
			.hsw		= 40,
			.hbp		= 220,
			.vfp		= 5,
			.vsw		= 5,
			.vbp		= 20,
		},
	},


};

struct omap3logic_panel omap3logic_default_panel;

static struct omap_custom_lcd_fields {
	char *field;
	char *format;
	void *ptr;
	int len;
} omap_custom_lcd_fields[] = {
	{ "xres", "%u", &omap3logic_default_panel.timing.x_res , 2},
	{ "yres", "%u", &omap3logic_default_panel.timing.y_res , 2},
	{ "left margin", "%u", &omap3logic_default_panel.timing.hbp, 2 },
	{ "right margin", "%u", &omap3logic_default_panel.timing.hfp, 2 },
	{ "top margin", "%u", &omap3logic_default_panel.timing.vbp, 2 },
	{ "bottom margin", "%u", &omap3logic_default_panel.timing.vfp, 2 },
	{ "hsync length", "%u", &omap3logic_default_panel.timing.hsw, 2 },
	{ "vsync length", "%u", &omap3logic_default_panel.timing.vsw, 2 },
	{ "pixclock", "%u", &omap3logic_default_panel.timing.pixel_clock, 4 },
	{ "config", "%u", &omap3logic_default_panel.config, 4 },
	{ "data_lines", "%u", &omap3logic_default_panel.data_lines, 1 },
};

/* !0 if we found a valid "display=" on the command line. */
// char *omap3logic_panel_name;

int omap3logic_display_selection(char *s)
{
	char *p, *q, *r;
	struct omap_custom_lcd_fields *f;
	struct omap3logic_panel *lcd;
	int err = 0, i;
	unsigned int val;
	int last;
	int data_lines;
	char panel_name[32];

	DPRINTK("%s:%d\n", __FUNCTION__, __LINE__);
	err = 0;
	if (strchr(s, ':')) {
		// Display is custom specification, not name, fill in defaults
		omap3logic_default_panel = omap3logic_panels[0];
		omap3logic_default_panel.name = "custom";
		omap3logic_default_panel.data_lines = 16;
		last = 0;
		p = s;
		for (i=0, f=omap_custom_lcd_fields; !last && i<ARRAY_SIZE(omap_custom_lcd_fields); ++i, ++f) {
			q = strchr(p, ':');
			if (q)
				*q = '\0';
			else
				last = 1;

			val = simple_strtoul(p, &r, 0);

			if (q && (r != q)) {
				printk(KERN_ERR "Custom display field '%s' value of '%s' invalid\n", f->field, p);
				err = 1;
				break;
			}
			switch(f->len) {
			case 1: {
					u8 *ptr = f->ptr;
					*ptr = val;
				}
				break;
			case 2: {
					u16 *ptr = f->ptr;
					*ptr = val;
				}
				break;
			default:
			case 4: {
					u32 *ptr = f->ptr;
					*ptr = val;
				}
				break;
			}
			p = q+1;
		}
		printk(KERN_INFO "Custom display=%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d\n",
			omap3logic_default_panel.timing.x_res, omap3logic_default_panel.timing.y_res,
			omap3logic_default_panel.timing.hbp, omap3logic_default_panel.timing.hfp,
			omap3logic_default_panel.timing.vbp, omap3logic_default_panel.timing.vfp,
			omap3logic_default_panel.timing.hsw, omap3logic_default_panel.timing.vsw,
			omap3logic_default_panel.timing.pixel_clock, omap3logic_default_panel.config,
			omap3logic_default_panel.data_lines);
	} else {
		/* Copy panel name and null-terminate it */
		strncpy(panel_name, s, sizeof(panel_name));
		panel_name[sizeof(panel_name)-1] = '\0';

		/* Search for trailing "-dvi" or "-hdmi", if found
		* set data_lines and strip off trailing specifier */
		data_lines = 16;
		if ((p = strrchr(panel_name, '-')) != NULL) {
			if (!strcmp(p+1, "dvi")) {
				data_lines = 16;
				*p='\0';
			} else if (!strcmp(p+1, "hdmi")) {
				data_lines = 24;
				*p='\0';
			}
		}

		for (i=0, lcd=omap3logic_panels; i<ARRAY_SIZE(omap3logic_panels); ++i, ++lcd) {
			if (!strcmp(lcd->name, panel_name)) {
				omap3logic_default_panel = *lcd;
				omap3logic_default_panel.data_lines = data_lines;
				printk(KERN_INFO "Standard display '%s' %d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d\n",
					s,
					omap3logic_default_panel.timing.x_res, omap3logic_default_panel.timing.y_res,
					omap3logic_default_panel.timing.hbp, omap3logic_default_panel.timing.hfp,
					omap3logic_default_panel.timing.vbp, omap3logic_default_panel.timing.vfp,
					omap3logic_default_panel.timing.hsw, omap3logic_default_panel.timing.vsw,
					omap3logic_default_panel.timing.pixel_clock,
					omap3logic_default_panel.config,
					omap3logic_default_panel.data_lines);
				break;
			}
		}
		if (i == ARRAY_SIZE(omap3logic_panels)) {
			printk(KERN_ERR "display='%s' specified on commandline not found\n", s);
			err = 1;
		}
	}

	if (!err) {
		printk("%s: found panel '%s'\n", __FUNCTION__, omap3logic_default_panel.name);
		// omap3logic_panel_name = omap3logic_default_panel.name;
	}
		

	return err;
}

__setup("display=", omap3logic_display_selection);

static int omap3logic_panel_probe(struct omap_dss_device *dssdev)
{
	if (!omap3logic_default_panel.name)
		return -ENODEV;
	dssdev->panel.config = omap3logic_default_panel.config;
	dssdev->panel.acb = 0x0;
	dssdev->panel.timings = omap3logic_default_panel.timing;

	return 0;
}

static void omap3logic_panel_remove(struct omap_dss_device *dssdev)
{
}

static int omap3logic_panel_pre_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	if (dssdev->platform_pre_enable)
		r = dssdev->platform_pre_enable(dssdev);

	return r;
}

static int omap3logic_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;


	if (dssdev->platform_enable)
		r = dssdev->platform_enable(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

err0:
	return r;
}

static void omap3logic_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	omapdss_dpi_display_disable(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int omap3logic_panel_suspend(struct omap_dss_device *dssdev)
{
	omap3logic_panel_disable(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	return 0;
}

static int omap3logic_panel_pre_resume(struct omap_dss_device *dssdev)
{
	int r;

	r = omap3logic_panel_enable(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return r;
}

static int omap3logic_panel_resume(struct omap_dss_device *dssdev)
{
	return omap3logic_panel_enable(dssdev);
}

static struct omap_dss_driver omap3logic_driver = {
	.probe		= omap3logic_panel_probe,
	.remove		= omap3logic_panel_remove,

	.pre_enable	= omap3logic_panel_pre_enable,
	.enable		= omap3logic_panel_enable,
	.disable	= omap3logic_panel_disable,
	.suspend	= omap3logic_panel_suspend,
	.pre_resume	= omap3logic_panel_pre_resume,
	.resume		= omap3logic_panel_resume,

	.driver         = {
		.name   = "omap3logic_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init omap3logic_panel_drv_init(void)
{
	return omap_dss_register_driver(&omap3logic_driver);
}

static void __exit omap3logic_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&omap3logic_driver);
}

module_init(omap3logic_panel_drv_init);
module_exit(omap3logic_panel_drv_exit);
MODULE_LICENSE("GPL");
