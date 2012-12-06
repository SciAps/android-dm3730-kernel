/*
 * linux/arch/arm/mach-omap2/board-omap3logic-display.c
 *
 * Copyright (C) 2011 Logic Product Development
 *
 * Modified from mach-omap2/board-omap3evm.c
 *
 * Initial code: Peter Barada
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/backlight.h>
#include <linux/regulator/machine.h>
#include <linux/workqueue.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>

//#include <plat/control.h>
#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>

#include <plat/board-omap3logic-display.h>
#include "board-omap3logic.h"
#include <plat/dmtimer.h>

#include "mux.h"
#include "mux34xx.h"

struct omap3logic_dss_board_info {
	int gpio_flag;
	int lcd_gpio_enable;
	int lcd_gpio_backlight;
	int lcd_enabled;
	int dvi_enabled;
	int syncs_as_gpio;   /* !0 if hsync/vsync are gpio pins */
	struct delayed_work work;
};

#if 1
#define LCDPRINTK(fmt, args...) printk(KERN_INFO fmt, ## args)
#else
#define LCDPRINTK(fmt, args...)
#endif

#if (defined(CONFIG_OMAP2_DSS) || defined(CONFIG_OMAP2_DSS_MODULE)) && defined(CONFIG_PANEL_OMAP3LOGIC)
static struct {
	struct mutex lock;
	int enabled;
	int intensity;
} omap3logic_bl_data;

static struct omap3logic_dss_board_info omap3logic_dss_lcd_data = {
	.gpio_flag = 0,
};

#define MIN_BRIGHTNESS 45
#define MAX_BRIGHTNESS 255

#define MAX_SOM_LV_BRIGHTNESS 127

struct omap_dss_device omap3logic_lcd_device;

/* SOM LV Backlight intensity function */

/*
 * PWMA/B register offsets (TWL4030_MODULE_PWMA)
 */
#define TWL_INTBR_PMBR1	0xD
#define TWL_INTBR_GPBR1	0xC
#define TWL_LED_PWMON	0x0
#define TWL_LED_PWMOFF	0x1

static struct generic_bl_info omap3logic_bl_info = {
	.name			= "omap3logic",
	.max_intensity		= MAX_BRIGHTNESS,
	.default_intensity	= (MAX_BRIGHTNESS * 7) / 10,
	.set_bl_intensity	= NULL,
};

static struct platform_device omap3logic_bl_device = {
	.name			= "generic-bl",
	.id			= 1,
	.dev = {
		.platform_data = &omap3logic_bl_info,
	},
};

static void dm3730_som_lv_bl_set_intensity(int level)
{
	struct omap3logic_dss_board_info *pdata;
	unsigned char c;

	u8 mux_pwm, enb_pwm;

	pdata = omap3logic_lcd_device.dev.platform_data;

	LCDPRINTK("%s: level %d\n", __FUNCTION__, level);
	if (level > MAX_BRIGHTNESS)
		return;

	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &mux_pwm, TWL_INTBR_PMBR1);
	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &enb_pwm, TWL_INTBR_GPBR1);

	if (level == 0) {
		/* disable pwm0 output and clock */
		enb_pwm = enb_pwm & 0xFA;
		/* change pwm0 pin to gpio pin */
		mux_pwm = mux_pwm & 0xF3;
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					enb_pwm, TWL_INTBR_GPBR1);
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					mux_pwm, TWL_INTBR_PMBR1);
		LCDPRINTK("%s: turn off GPIO_%d as backlight!\n", __FUNCTION__, omap3logic_dss_lcd_data.lcd_gpio_backlight);
		/* Turn off the backlight! */
		gpio_set_value(omap3logic_dss_lcd_data.lcd_gpio_backlight, 0);

		return;
	}

	LCDPRINTK("%s: enb_pwm %02x mux_pwm %02x\n", __FUNCTION__, enb_pwm, mux_pwm);
	if (((enb_pwm & 0x5) != 0x5) || ((mux_pwm & 0x0c) != 0x4)) {
		/* change gpio pin to pwm0 pin */
		mux_pwm = (mux_pwm & 0xc) | 0x04;
		/* enable pwm0 output and clock*/
		enb_pwm = (enb_pwm & 0x5) | 0x05;
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					mux_pwm, TWL_INTBR_PMBR1);
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					enb_pwm, TWL_INTBR_GPBR1);
		LCDPRINTK("%s: turn on GPIO_%d as backlight!\n", __FUNCTION__, omap3logic_dss_lcd_data.lcd_gpio_backlight);
		/* Turn on the backlight! */
	}

	/*
	 * Limit the minimum brightness. Otherwide, the unlock slide bar
	 * might not be easily clear to see under dimmer state, when
	 * brightness is the lowest.
	 */
	if(level < MIN_BRIGHTNESS) {
		level = MIN_BRIGHTNESS;
	}

	/* 255 -> 1, 1 -> 126 */
	c = (255 * 126 + (1 - 126) * level) / (255 - 1);

	LCDPRINTK("%s: c %d (%d%% on)\n", __FUNCTION__, c, (((MAX_BRIGHTNESS+1)-c) * 100)/(MAX_BRIGHTNESS+1));
	twl_i2c_write_u8(TWL4030_MODULE_PWM0, 0x7F, TWL_LED_PWMOFF);
	twl_i2c_write_u8(TWL4030_MODULE_PWM0, c, TWL_LED_PWMON);

	/* Turn on the backlight! */
	gpio_set_value(omap3logic_dss_lcd_data.lcd_gpio_backlight, 1);


	return;
}

static struct omap_dm_timer *intensity_timer;

/* TORPEDO Backlight intensity function */
static void dm3730_torpedo_bl_set_intensity(int level)
{
	LCDPRINTK("%s: level %d (%d%% on)\n", __FUNCTION__, level, (level * 100)/ MAX_BRIGHTNESS);

	if (level > MAX_BRIGHTNESS)
		return;

	if (level == 0) {
		LCDPRINTK("%s: turn off GPIO_%d as backlight!\n", __FUNCTION__, omap3logic_dss_lcd_data.lcd_gpio_backlight);
		/* Turn off the backlight! */
		gpio_set_value(omap3logic_dss_lcd_data.lcd_gpio_backlight, 0);
		omap_dm_timer_disable(intensity_timer);
	} else {
		LCDPRINTK("%s: turn on GPIO_%d as backlight!\n", __FUNCTION__, omap3logic_dss_lcd_data.lcd_gpio_backlight);
		/* Turn on the backlight! */
		gpio_set_value(omap3logic_dss_lcd_data.lcd_gpio_backlight, 1);

		omap_dm_timer_enable(intensity_timer);
		omap_dm_timer_set_source(intensity_timer, OMAP_TIMER_SRC_SYS_CLK);
		omap_dm_timer_set_load(intensity_timer, 1, 0xFFFF0000);
		omap_dm_timer_set_pwm(intensity_timer, 0, 1,
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);

		/*
		 * Limit the minimum brightness. Otherwide, the unlock slide bar
		 * might not be easily clear to see under dimmer state, when
		 * brightness is the lowest.
		 */
		if(level < MIN_BRIGHTNESS) {
			level = MIN_BRIGHTNESS;
		}

		omap_dm_timer_set_match(intensity_timer, 1,
					(0xffff0000) | (level<<8));
		omap_dm_timer_start(intensity_timer);
	}
}

static void omap3logic_do_update(int level)
{
	if (machine_is_dm3730_som_lv() || machine_is_omap3530_lv_som())
		dm3730_som_lv_bl_set_intensity(level);
	else
		dm3730_torpedo_bl_set_intensity(level);
}

/* Common function called to set the backlight; defer if need be since
 * omap3logic_bl_update() will handle it later */
static void omap3logic_do_bl_intensity(int level)
{
	mutex_lock(&omap3logic_bl_data.lock);
	omap3logic_bl_data.intensity = level;
	if (omap3logic_bl_data.enabled)
		omap3logic_do_update(level);
	mutex_unlock(&omap3logic_bl_data.lock);
}

static void omap3logic_enable_backlight(void)
{
	mutex_lock(&omap3logic_bl_data.lock);
	if (!omap3logic_bl_data.enabled) {
		omap3logic_bl_data.enabled = 1;
		if (omap3logic_bl_data.intensity) {
			/* Update the backlight if turned on */
			omap3logic_do_update(omap3logic_bl_data.intensity);
		}
	}
	mutex_unlock(&omap3logic_bl_data.lock);
}

static void omap3logic_disable_backlight(void)
{
	mutex_lock(&omap3logic_bl_data.lock);
	if (omap3logic_bl_data.enabled) {
		if (omap3logic_bl_data.intensity) {
			printk(KERN_ERR "%s: backlight (%d) is still enabled and non-zero!\n", __FUNCTION__, omap3logic_bl_data.intensity);
			omap3logic_do_update(0);
		}
		omap3logic_bl_data.enabled = 0;
	}
	mutex_unlock(&omap3logic_bl_data.lock);
}

static void omap3logic_panel_wait_vsyncs(int count)
{
	while(count--)
	{
		// Wait for a VSYNC for up to 50ms (or bail)
		if(omap_dispc_wait_for_irq_timeout(DISPC_IRQ_VSYNC, msecs_to_jiffies(50)) < 0)
		{
			printk(KERN_INFO "omap3logic_panel_wait_vsyncs: Failed to find VSYNC interrupt.\n");
		}
	}
}

static int omap3logic_panel_pre_enable_lcd(struct omap_dss_device *dssdev)
{
	// Pulse the LCD enable line momentarily.  This helps to reset some
	// of the internal logic of the 4.3" display.  We're doing this
	// with the VSYNC signals off so we are ensured that this won't
	// propogate to the LCD as a display enable through the flip-flops.
	struct omap3logic_dss_board_info *pdata = dssdev->dev.platform_data;
	gpio_set_value(pdata->lcd_gpio_enable, 1);
	msleep(5);
	gpio_set_value(pdata->lcd_gpio_enable, 0);
	return 0;
}

static int omap3logic_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	static int first_init = 1;
	struct omap3logic_dss_board_info *pdata = dssdev->dev.platform_data;

	// If this is the first init, ensure we fully reset the
	// LCD controller.  The Sharp LCD controller is a bit picky,
	// and needs clocks running for a minimum amount to reset.
	// Handle this on the first initialization, and subsequently
	// remove this extra time from other panel enables.
	if(first_init)
	{
		gpio_set_value(pdata->lcd_gpio_enable, 1);
		omap3logic_panel_wait_vsyncs(11);
		gpio_set_value(pdata->lcd_gpio_enable, 0);
		omap3logic_panel_wait_vsyncs(11);
		first_init = 0;
	}

	omap3logic_panel_wait_vsyncs(3);
	gpio_set_value(pdata->lcd_gpio_enable, 1);

	// Display controller needs 10 vsyncs (~200ms) before white
	// LCD screen is gone.
	queue_delayed_work(system_freezable_wq, &pdata->work, msecs_to_jiffies(200));

	return 0;
}

static void omap3logic_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	struct omap3logic_dss_board_info *pdata = dssdev->dev.platform_data;

	cancel_delayed_work_sync(&pdata->work);
	omap3logic_disable_backlight();
	gpio_set_value(pdata->lcd_gpio_enable, 0);
	// Display controller needs 5 vsyncs after disable to
	// properly go down.  Due to flip-flops in the path, we need
	// an additional vsync.
	omap3logic_panel_wait_vsyncs(6);

	return;
}

void omap3logic_enable_backlight_delayed(struct work_struct *work)
{
	omap3logic_enable_backlight();
	return;
}

struct omap_dss_device omap3logic_lcd_device = {
	.name			= "lcd",
	.driver_name		= "omap3logic_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= -EINVAL,
	.platform_pre_enable    = omap3logic_panel_pre_enable_lcd,
	.platform_enable	= omap3logic_panel_enable_lcd,
	.platform_disable	= omap3logic_panel_disable_lcd,
	.dev = {
		.platform_data = &omap3logic_dss_lcd_data,
	},
};

#ifdef CONFIG_OMAP2_DSS_VENC
static int omap3logic_enable_tv(struct omap_dss_device *dssdev)
{
	printk("%s:%d\n", __FUNCTION__, __LINE__);
	return 0;
}

static void omap3logic_disable_tv(struct omap_dss_device *dssdev)
{
	printk("%s:%d\n", __FUNCTION__, __LINE__);
}

static struct omap_dss_device omap3logic_tv_device = {
	.name			= "tv",
	.driver_name		= "venc",
	.type			= OMAP_DISPLAY_TYPE_VENC,
#if defined(CONFIG_OMAP2_VENC_OUT_TYPE_SVIDEO)
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
#elif defined(CONFIG_OMAP2_VENC_OUT_TYPE_COMPOSITE)
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_COMPOSITE,
#endif
	.platform_enable	= omap3logic_enable_tv,
	.platform_disable	= omap3logic_disable_tv,
};
#endif

static int omap3logic_enable_dvi(struct omap_dss_device *dssdev)
{
	struct omap3logic_dss_board_info *pdata;
	pdata = dssdev->dev.platform_data;

	printk("%s:%d\n", __FUNCTION__, __LINE__);
	if (pdata->lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}

	pdata->dvi_enabled = 1;
	return 0;
}

static void omap3logic_disable_dvi(struct omap_dss_device *dssdev)
{
	struct omap3logic_dss_board_info *pdata;
	pdata = dssdev->dev.platform_data;

	printk("%s:%d\n", __FUNCTION__, __LINE__);
	pdata->dvi_enabled = 0;
}

static struct panel_generic_dpi_data omap3logic_dvi_panel = {
	.name			= "generic",
	.platform_enable	= omap3logic_enable_dvi,
	.platform_disable	= omap3logic_disable_dvi,
};

static struct omap_dss_device omap3logic_dvi_device = {
	.name			= "dvi",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.driver_name		= "generic_dpi_panel",
	.phy.dpi.data_lines	= 24,
#if 1
	.data			= &omap3logic_dvi_panel,
#else
	.platform_enable	= omap3logic_enable_dvi,
	.platform_disable	= omap3logic_disable_dvi,
#endif
};

static struct omap_dss_device *omap3logic_dss_devices[] = {
	&omap3logic_lcd_device,
	&omap3logic_dvi_device,
#ifdef CONFIG_OMAP2_DSS_VENC
	&omap3logic_tv_device,
#endif
};

static struct omap_dss_board_info omap3logic_dss_data = {
	.num_devices	= ARRAY_SIZE(omap3logic_dss_devices),
	.devices	= omap3logic_dss_devices,
	.default_device	= &omap3logic_lcd_device,
};

#if 0
struct platform_device omap3logic_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data = &omap3logic_dss_data,
	},
};
#endif

static int force_old_dss_pinmux;
static int __init dm3730_force_old_dss_pinmux(char *str)
{
	force_old_dss_pinmux = 1;
	return 1;
}

__setup("old_dss_pinmux", dm3730_force_old_dss_pinmux);

static void omap3logic_lcd_set_panel_mux(void)
{
	struct omap3logic_dss_board_info *pdata;

	pdata = omap3logic_lcd_device.dev.platform_data;

	pdata->lcd_gpio_enable = 155;
	if (machine_is_dm3730_som_lv() || machine_is_omap3530_lv_som()) {
		pdata->lcd_gpio_backlight = 8;
	} else if (machine_is_dm3730_torpedo() || machine_is_omap3_torpedo()) {
		pdata->lcd_gpio_backlight = 154;
		omap_mux_init_signal("gpmc_ncs5.gpt10_pwm_evt", OMAP_PIN_INPUT_PULLDOWN);
	} else
		BUG();

	if (gpio_request(pdata->lcd_gpio_backlight, "LCD backlight"))
		pr_err("omap3logic: can't request GPIO %d for LCD backlight\n", pdata->lcd_gpio_backlight);
	gpio_direction_output(pdata->lcd_gpio_backlight, 0);
	omap_mux_init_gpio(pdata->lcd_gpio_backlight, OMAP_PIN_OUTPUT);
	if (gpio_request(pdata->lcd_gpio_enable, "LCD enable"))
		pr_err("omap3logic: can't request GPIO %d for LCD enable\n", pdata->lcd_gpio_enable);
	gpio_direction_output(pdata->lcd_gpio_enable, 0);
	omap_mux_init_gpio(pdata->lcd_gpio_enable, OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	pdata->gpio_flag = 1;

	/* Mux common DSS signals */
	omap_mux_init_signal("dss_pclk.dss_pclk", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_hsync.dss_hsync", OMAP_PIN_INPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_vsync.dss_vsync", OMAP_PIN_INPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_acbias.dss_acbias", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);

	/* Mux the differing DSS signals (DATA0-DATA5). We assume
	   the Torpedo uses the "new" mapping.  If overridden then use the
	   old mapping */
	if (force_old_dss_pinmux || machine_is_dm3730_som_lv()) {
		omap_mux_init_signal("dss_data0.dss_data0", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data1.dss_data1", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data2.dss_data2", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data3.dss_data3", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data4.dss_data4", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data5.dss_data5", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	} else {
		omap_mux_init_signal("dss_data18.dss_data0", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data19.dss_data1", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data20.dss_data2", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data21.dss_data3", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data22.dss_data4", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data23.dss_data5", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	}

	omap_mux_init_signal("dss_data6.dss_data6", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data7.dss_data7", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data8.dss_data8", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data9.dss_data9", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data10.dss_data10", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data11.dss_data11", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data12.dss_data12", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data13.dss_data13", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data14.dss_data14", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data15.dss_data15", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);

	/* OMAP35x SOM/Torpedo doesn't have HDMI, has to be 16 bit display */
	if (machine_is_omap3530_lv_som() || machine_is_omap3_torpedo()) {
		if (omap3logic_default_panel.data_lines != 16) {
			printk("%s: OMAP35x Torpedo/SOM LV is 16bpp display!\n", __FUNCTION__);
			omap3logic_default_panel.data_lines = 16;
		}
	}

	if (omap3logic_default_panel.data_lines == 16)
		return;

	/* DSS_DATA16/17 is common to SOM LV/Torpedo */
	omap_mux_init_signal("dss_data16.dss_data16", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data17.dss_data17", OMAP_PIN_OUTPUT);

	/* Mux the HDMI signals */
	if (machine_is_dm3730_som_lv()) {
		printk(KERN_ERR "Warning: muxing DSS18-23 for SOM LV!\n");
		omap_mux_init_signal("dss_data18.dss_data18", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("dss_data19.dss_data19", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("dss_data20.dss_data20", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("dss_data21.dss_data21", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("dss_data22.dss_data22", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("dss_data23.dss_data23", OMAP_PIN_OUTPUT);
	} else if (machine_is_dm3730_torpedo()) {
		omap_mux_init_signal("sys_boot0.dss_data18", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("sys_boot1.dss_data19", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("sys_boot3.dss_data20", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("sys_boot4.dss_data21", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("sys_boot5.dss_data22", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("sys_boot6.dss_data23", OMAP_PIN_OUTPUT);
	}
}

void omap3logic_lcd_init(void)
{
	int result;

	/* If no valid "display=" was specified then no video wanted */
	if (omap3logic_default_panel.name) {
		struct omap3logic_dss_board_info *pdata = omap3logic_lcd_device.dev.platform_data;

		omap3logic_lcd_set_panel_mux();

		printk("%s: name %s data_lines %d\n", __FUNCTION__, 
			omap3logic_default_panel.name,
			omap3logic_default_panel.data_lines);
		omap3logic_lcd_device.name = omap3logic_default_panel.name;
		omap3logic_lcd_device.phy.dpi.data_lines = omap3logic_default_panel.data_lines;
		result = omap_display_init(&omap3logic_dss_data);

		INIT_DELAYED_WORK(&pdata->work, omap3logic_enable_backlight_delayed);

		if (result)
			printk("%s: platform device register of DSS2 device failed: %d\n", __FUNCTION__, result);
		mutex_init(&omap3logic_bl_data.lock);
		if (machine_is_dm3730_som_lv() || machine_is_omap3530_lv_som()) {
			omap3logic_bl_info.set_bl_intensity = omap3logic_do_bl_intensity;
		} else if (machine_is_dm3730_torpedo() || machine_is_omap3_torpedo()) {
			intensity_timer = omap_dm_timer_request_specific(10);
			if (intensity_timer == NULL) {
				printk("%s: can't get backlight timer!\n", __FUNCTION__);
			} else {
				omap_dm_timer_disable(intensity_timer);
				omap3logic_bl_info.set_bl_intensity = omap3logic_do_bl_intensity;
			}
		} else
			BUG();

#if defined(CONFIG_BACKLIGHT_GENERIC)
		if (0 && machine_is_dm3730_som_lv()) {
			/* Override the default intensity for the SOM LV, since
			 * it uses a different range than the Torpedo.
			 */
			omap3logic_bl_info.default_intensity =
				(MAX_SOM_LV_BRIGHTNESS * 7) / 10;
		}
		result = platform_device_register(&omap3logic_bl_device);
		if (result)
			printk("%s: platform device register of backlight device failed: %d\n", __FUNCTION__, result);
#endif
	}
}
#else
void omap3logic_lcd_init(void)
{
}
#endif
