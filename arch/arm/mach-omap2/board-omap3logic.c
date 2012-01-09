/*
 * linux/arch/arm/mach-omap2/board-omap3logic.c
 *
 * Copyright (C) 2010 Li-Pro.Net
 * Stephan Linz <linz@li-pro.net>
 *
 * Copyright (C) 2010 Logic Product Development, Inc.
 * Peter Barada <peter.barada@logicpd.com>
 *
 * Modified from Beagle, EVM, and RX51
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
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>

#include <linux/i2c/twl.h>
#include <linux/i2c/tsc2004.h>
#include <linux/wl12xx.h>
#include <linux/mmc/host.h>
#include <linux/usb/isp1763.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "control.h"
#include "common-board-devices.h"

#include <plat/mux.h>
#include <plat/board.h>
#include <plat/usb.h>
#include <plat/common.h>
#include <video/omapdss.h>
#include <plat/gpmc-smsc911x.h>
#include <plat/gpmc.h>
#include <plat/sdrc.h>

#include <plat/board-omap3logic.h>
#include <plat/board-omap3logic-display.h>
#include <plat/omap3logic-new-productid.h>
#include <plat/omap3logic-productid.h>
#include <plat/omap3logic-cf.h>
// #include "board-omap3logic.h"

#ifdef CONFIG_PRINTK_DEBUG
#include <plat/printk-debug.h>
#endif

#define OMAP3LOGIC_SMSC911X_CS			1

#define OMAP3530_LV_SOM_MMC_GPIO_CD		110
#define OMAP3530_LV_SOM_MMC_GPIO_WP		126
#define OMAP3530_LV_SOM_SMSC911X_GPIO_IRQ	152

#define OMAP3_TORPEDO_MMC_GPIO_CD		127
#define OMAP3_TORPEDO_SMSC911X_GPIO_IRQ		129

static struct regulator_consumer_supply omap3logic_vmmc1_supply = {
	.supply			= "vmmc",
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data omap3logic_vmmc1 = {
	.constraints = {
		.name			= "VMMC1",
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &omap3logic_vmmc1_supply,
};

static struct regulator_consumer_supply omap3logic_vaux1_supply = {
	.supply			= "vaux1",
};

/* VAUX1 for touch/product ID chip */
static struct regulator_init_data omap3logic_vaux1 = {
	.constraints = {
		.min_uV		= 3000000,
		.max_uV		= 3000000,
		.apply_uV	= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
#if 1
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
#endif
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3logic_vaux1_supply,
};

static struct regulator_consumer_supply omap3logic_vaux3_supplies[] = {
	REGULATOR_SUPPLY("vmmc_aux", "omap_hsmmc.2"),
};

static struct regulator_init_data omap3logic_vaux3 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies		= ARRAY_SIZE(omap3logic_vaux3_supplies),
	.consumer_supplies		= omap3logic_vaux3_supplies,
};

static struct regulator_consumer_supply omap3logic_vmmc3_supply = {
	.supply			= "vmmc",
	.dev_name		= "omap_hsmmc.2",
};

static struct regulator_init_data omap3logic_vmmc3 = {
	.constraints = {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies = &omap3logic_vmmc3_supply,
};

#define OMAP3LOGIC_WLAN_SOM_LV_PMENA_GPIO 3
#define OMAP3LOGIC_WLAN_SOM_LV_IRQ_GPIO 2
#define OMAP3LOGIC_WLAN_TORPEDO_PMENA_GPIO 157
#define OMAP3LOGIC_WLAN_TORPEDO_IRQ_GPIO 152

static struct fixed_voltage_config omap3logic_vwlan = {
	.supply_name		= "vwl1271",
	.microvolts		= 1800000, /* 1.8V */
	.gpio			= -EINVAL,
	.startup_delay		= 70000, /* 70msec */
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &omap3logic_vmmc3,
};

static struct platform_device omap3logic_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data	= &omap3logic_vwlan,
	},
};

static struct wl12xx_platform_data omap3logic_wlan_data __initdata = {
	.irq = -EINVAL,
	.board_ref_clock = -EINVAL,
	.board_tcxo_clock = -EINVAL,
};

static int omap3logic_twl_gpio_base;	/* base GPIO of TWL4030 GPIO.0 */

#if defined(CONFIG_NEW_LEDS) || defined(CONFIG_NEW_LEDS_MODULE)
static struct gpio_led omap3logic_leds[] = {
	{
		.name			= "led1",	/* D1 on baseboard */
		.default_trigger	= "heartbeat",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= false,
	},
	{
		.name			= "led2",	/* D2 on baseboard */
		.default_trigger	= "none",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= false,
	},
	{
		.name			= "led3",	/* D1 on Torpedo module */
		.default_trigger	= "none",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
	},
};
 
static struct gpio_led_platform_data omap3logic_led_data = {
	.leds		= omap3logic_leds,
	.num_leds	= 0,	/* Initialized in omap3logic_led_init() */
 };
 
static struct platform_device omap3logic_led_device = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &omap3logic_led_data,
	},
};

#define GPIO_LED1_SOM_LV	133
#define GPIO_LED2_SOM_LV	11

#define GPIO_LED1_TORPEDO	180
#define GPIO_LED2_TORPEDO	179

static void omap3logic_led_init(void)
{
	int gpio_led1 = -EINVAL;
	int gpio_led2 = -EINVAL;

	if (machine_is_omap3_torpedo() || machine_is_dm3730_torpedo()) {
		if (!omap3logic_twl_gpio_base) {
			printk(KERN_ERR "Huh?!? twl4030_gpio_base not set!\n");
			return;
		}
		/* baseboard LEDs are MCSPIO2_SOMI, MCSPOI2_SIMO */
		gpio_led1 = GPIO_LED1_TORPEDO;
		gpio_led2 = GPIO_LED2_TORPEDO;

		/* twl4030 ledA is the LED on the module */
		omap3logic_leds[2].gpio = omap3logic_twl_gpio_base + TWL4030_GPIO_MAX + 0;
		omap3logic_led_data.num_leds = 3;
	} else if (machine_is_omap3530_lv_som() || machine_is_dm3730_som_lv()) {
		gpio_led1 = GPIO_LED1_SOM_LV;
		omap3logic_leds[0].active_low = true;
		gpio_led2 = GPIO_LED2_SOM_LV;
		omap3logic_leds[1].active_low = true;

		/* SOM has only two LEDs */
		omap3logic_led_data.num_leds = 2;
	}

	if (gpio_led1 < omap3logic_twl_gpio_base)
		omap_mux_init_gpio(gpio_led1, OMAP_PIN_OUTPUT);
	omap3logic_leds[0].gpio = gpio_led1;

	if (gpio_led2 < omap3logic_twl_gpio_base)
		omap_mux_init_gpio(gpio_led2, OMAP_PIN_OUTPUT);
	omap3logic_leds[1].gpio = gpio_led2;

	if (platform_device_register(&omap3logic_led_device) < 0)
		printk(KERN_ERR "Unable to register LED device\n");
}
#else
static void omap3logic_led_init(void)
{
}
#endif

static int omap3logic_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	omap3logic_twl_gpio_base = gpio;

	omap3logic_led_init();

	return 0;
}

static struct twl4030_gpio_platform_data omap3logic_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2)  | BIT(6)  | BIT(7)  | BIT(8)
			| BIT(13) | BIT(15) | BIT(16) | BIT(17),
	.setup		= omap3logic_twl_gpio_setup,
};

#if defined(CONFIG_OMAP2_DSS) || defined(CONFIG_OMAP2_DSS_MODULE)
static struct regulator_consumer_supply omap3logic_vdda_dac_supplies[] = {
	REGULATOR_SUPPLY("vdda_dac", "omapdss_venc"),
};

/* VDAC for DSS driving S-Video */
static struct regulator_init_data omap3logic_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies	= omap3logic_vdda_dac_supplies,
	.num_consumer_supplies	= ARRAY_SIZE(omap3logic_vdda_dac_supplies),
};

/* VPLL2 for digital video outputs */
static struct regulator_consumer_supply omap3logic_vpll2_supplies[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
	REGULATOR_SUPPLY("vpll2", NULL),
};

static struct regulator_init_data omap3logic_vpll2 = {
	.constraints = {
		.name			= "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(omap3logic_vpll2_supplies),
	.consumer_supplies	= omap3logic_vpll2_supplies,
};
#endif

static struct twl4030_usb_data omap3logic_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

#define STANDARD_OMAP	0
#define TEST_LOGIC	1

//#define TEST_GROUP	DEV_GRP_P1
#define TEST_GROUP	DEV_GRP_NULL

static struct twl4030_ins  sleep_on_seq[] = {
#if STANDARD_OMAP
	/* Broadcast message to put res to sleep (TYPE2 = 1, 2) */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_SLEEP), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_SLEEP), 2},
#else
	{MSG_SINGULAR(DEV_GRP_P1, RES_HFCLKOUT, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VDD1,     RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VDD2,     RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VPLL1,    RES_STATE_OFF), 2},
#if TEST_LOGIC
	{MSG_BROADCAST(TEST_GROUP, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_OFF), 2},
#endif
#endif
};

static struct twl4030_script sleep_on_script = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

static struct twl4030_ins wakeup_p12_seq[] = {
#if STANDARD_OMAP
	/*
	 * Broadcast message to put resources to active
	 *
	 * Since we are not using TYPE, resources which have TYPE2 configured
	 * as 1 will be targeted (VPLL1, VDD1, VDD2, REGEN, NRES_PWRON, SYSEN).
	 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_ACTIVE), 2},
#else
	{MSG_SINGULAR(DEV_GRP_P1, RES_HFCLKOUT, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VDD1,     RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VDD2,     RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VPLL1,    RES_STATE_ACTIVE), 2},
#if TEST_LOGIC
	{MSG_BROADCAST(TEST_GROUP, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_ACTIVE), 2},
#endif
#endif
};

static struct twl4030_script wakeup_p12_script = {
	.script	= wakeup_p12_seq,
	.size	= ARRAY_SIZE(wakeup_p12_seq),
	.flags	= TWL4030_WAKEUP12_SCRIPT,
};

static struct twl4030_ins wakeup_p3_seq[] = {
#if STANDARD_OMAP
	/*
	 * Broadcast message to put resources to active
	 *
	 * Since we are not using TYPE, resources which have TYPE2 configured
	 * as 2 will be targeted
	 * (VINTANA1, VINTANA2, VINTDIG, VIO, CLKEN, HFCLKOUT).
	 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_ACTIVE), 2},
#else
	{MSG_SINGULAR(DEV_GRP_P1, RES_HFCLKOUT, RES_STATE_ACTIVE), 2},
#endif
};

static struct twl4030_script wakeup_p3_script = {
	.script = wakeup_p3_seq,
	.size   = ARRAY_SIZE(wakeup_p3_seq),
	.flags  = TWL4030_WAKEUP3_SCRIPT,
};

static struct twl4030_ins wrst_seq[] = {
#if STANDARD_OMAP
	/*
	 * As a workaround for OMAP Erratum  (ID: i537 - OMAP HS devices are
	 * not recovering from warm reset while in OFF mode)
	 * NRESPWRON is toggled to force a power on reset condition to OMAP
	 */
	/* Trun OFF NRES_PWRON */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_NRES_PWRON, RES_STATE_OFF), 2},
	/* Reset twl4030 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_OFF), 2},
	/* Reset MAIN_REF */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_MAIN_REF, RES_STATE_WRST), 2},
	/* Reset All type2_group2 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_WRST), 2},
	/* Reset VUSB_3v1 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_VUSB_3V1, RES_STATE_WRST), 2},
	/* Reset All type2_group1 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_WRST), 2},
	/* Reset the Reset & Contorl_signals */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_RC, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_WRST), 2},
	/* Re-enable twl4030 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_ACTIVE), 2},
	/* Trun ON NRES_PWRON */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_NRES_PWRON, RES_STATE_ACTIVE), 2},
#else
	/*
	 * Reset twl4030.
	 * Reset VDD1 regulator.
	 * Reset VDD2 regulator.
	 * Reset VPLL1 regulator.
	 * Enable sysclk output.
	 * Reenable twl4030.
	 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET,    RES_STATE_OFF),    2},
	{MSG_SINGULAR(DEV_GRP_P1,   RES_VDD1,     RES_STATE_WRST),   15},
	{MSG_SINGULAR(DEV_GRP_P1,   RES_VDD2,     RES_STATE_WRST),   15},
	{MSG_SINGULAR(DEV_GRP_P1,   RES_VPLL1,    RES_STATE_WRST),   0x60},
	{MSG_SINGULAR(DEV_GRP_P1,   RES_HFCLKOUT, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET,    RES_STATE_ACTIVE), 2},
#endif
};
static struct twl4030_script wrst_script = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wrst_seq),
	.flags  = TWL4030_WRST_SCRIPT,
};

static struct twl4030_script *twl4030_scripts[] = {
	&wakeup_p12_script,
	&wakeup_p3_script,
	&sleep_on_script,
	&wrst_script,
};

static struct twl4030_resconfig twl4030_rconfig[] = {
#if STANDARD_OMAP
	{
		.resource = RES_NRES_PWRON,
		.devgroup = DEV_GRP_ALL,
		.type = 0,
		.type2 = 1,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_VINTANA2,
		.devgroup = DEV_GRP_ALL,
		.type = 0,
		.type2 = 2,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_HFCLKOUT,
		.devgroup = DEV_GRP_P3,
		.type = 0,
		.type2 = 2,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_VINTANA1,
		.devgroup = DEV_GRP_ALL,
		.type = 1,
		.type2 = 2,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_VINTDIG,
		.devgroup = DEV_GRP_ALL,
		.type = 1,
		.type2 = 2,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_REGEN,
		.devgroup = DEV_GRP_ALL,
		.type = 2,
		.type2 = 1,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_VIO,
		.devgroup = DEV_GRP_ALL,
		.type = 2,
		.type2 = 2,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_VPLL1,
		.devgroup = DEV_GRP_P1,
		.type = 3,
		.type2 = 1,
		.remap_sleep = RES_STATE_OFF
	},
	{
		.resource = RES_VDD2,
		.devgroup = DEV_GRP_P1,
		.type = 3,
		.type2 = 1,
		.remap_sleep = RES_STATE_OFF
	},
	{
		.resource = RES_CLKEN,
		.devgroup = DEV_GRP_ALL,
		.type = 3,
		.type2 = 2,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_VDD1,
		.devgroup = DEV_GRP_P1,
		.type = 4,
		.type2 = 1,
		.remap_sleep = RES_STATE_OFF
	},
	{
		.resource = RES_SYSEN,
		.devgroup = DEV_GRP_ALL,
		.type = 6,
		.type2 = 1,
		.remap_sleep = RES_STATE_SLEEP
	},
#else
	{
		.resource = RES_HFCLKOUT,
		.devgroup = DEV_GRP_P3,
		.type = -1,
		.type2 = -1
	},
	{
		.resource = RES_VDD1,
		.devgroup = DEV_GRP_P1,
		.type = -1,
		.type2 = -1
	},
	{
		.resource = RES_VDD2,
		.devgroup = DEV_GRP_P1,
		.type = -1,
		.type2 = -1
	},
#if TEST_LOGIC
	{
		.resource = RES_VAUX1,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VAUX2,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VAUX3,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VAUX4,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VMMC1,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VMMC2,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VSIM,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VDAC,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
#if 0
	// Disabling these seems to to hose up the warm reset.  The system will
	// still come up from a cold start.
	{
		.resource = RES_VINTANA1,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VINTANA2,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
#endif
	{
		.resource = RES_VUSB_1V5,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VUSB_1V8,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VUSB_3V1,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
#if 1
	// No effect on power consumption when the system is in suspend.
	{
		.resource = RES_VUSBCP,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
#endif
	{
		.resource = RES_SYSEN,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
#endif
#endif
	{ 0, 0},
};

static struct twl4030_power_data omap3logic_t2scripts_data __initdata = {
	.scripts        = twl4030_scripts,
	.num = ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

static struct twl4030_codec_audio_data omap3logic_audio_data;

static struct twl4030_codec_data omap3logic_codec_data = {
	.audio_mclk = 26000000,
	.audio = &omap3logic_audio_data,
};

static struct twl4030_platform_data omap3logic_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.usb		= &omap3logic_usb_data,
	.gpio		= &omap3logic_gpio_data,
	.power		= &omap3logic_t2scripts_data,
	.codec		= &omap3logic_codec_data,
	.vmmc1		= &omap3logic_vmmc1,
	.vaux1		= &omap3logic_vaux1,
	.vaux3		= &omap3logic_vaux3,
#if defined(CONFIG_OMAP2_DSS) || defined(CONFIG_OMAP2_DSS_MODULE)
	.vdac           = &omap3logic_vdac,
	.vpll2          = &omap3logic_vpll2,
#endif
};

#ifdef CONFIG_TOUCHSCREEN_TSC2004

#define	GPIO_TSC2004_IRQ	153

static int tsc2004_pre_init(struct tsc2004_platform_data *pdata)
{
	int err;

	pdata->regulator_name = "vaux1";
	pdata->regulator = regulator_get(NULL, "vaux1");
	if (IS_ERR(pdata->regulator)) {
		pr_err("%s: unable to get %s regulator\n", __FUNCTION__, pdata->regulator_name);
		return -1;
	}

	err = regulator_enable(pdata->regulator);
	if (err) {
		pr_err("%s: unable to enable %s regulator\n", __FUNCTION__, pdata->regulator_name);
		regulator_put(pdata->regulator);
		pdata->regulator = NULL;
		return err;
	}
	return 0;
}

static int tsc2004_init_irq(void)
{
	int ret = 0;

	omap_mux_init_gpio(GPIO_TSC2004_IRQ, OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);
	ret = gpio_request(GPIO_TSC2004_IRQ, "tsc2004-irq");
	if (ret < 0) {
		printk(KERN_WARNING "failed to request GPIO#%d: %d\n",
				GPIO_TSC2004_IRQ, ret);
		return ret;
	}

	if (gpio_direction_input(GPIO_TSC2004_IRQ)) {
		printk(KERN_WARNING "GPIO#%d cannot be configured as "
				"input\n", GPIO_TSC2004_IRQ);
		return -ENXIO;
	}

	gpio_set_debounce(GPIO_TSC2004_IRQ, 0xa);
	return ret;
}

static void tsc2004_exit_irq(void)
{
	gpio_free(GPIO_TSC2004_IRQ);
}

static void tsc2004_post_exit(struct tsc2004_platform_data *pdata)
{
	if (pdata->regulator && regulator_is_enabled(pdata->regulator)) {
		regulator_disable(pdata->regulator);
	}
}

static int tsc2004_get_irq_level(void)
{
	return gpio_get_value(GPIO_TSC2004_IRQ) ? 0 : 1;
}

struct tsc2004_platform_data omap3logic_tsc2004data = {
	.model = 2004,
	.x_plate_ohms = 180,
	.get_pendown_state = tsc2004_get_irq_level,
	.pre_init_platform_hw = tsc2004_pre_init,
	.init_platform_hw = tsc2004_init_irq,
	.exit_platform_hw = tsc2004_exit_irq,
	.post_exit_platform_hw = tsc2004_post_exit,
	.regulator_name = "vaux1",
};

#endif

static struct i2c_board_info __initdata omap3logic_i2c3_boardinfo[] = {
#ifdef CONFIG_TOUCHSCREEN_TSC2004
	{
		I2C_BOARD_INFO("tsc2004", 0x48),
		.type		= "tsc2004",
		.platform_data = &omap3logic_tsc2004data,
		.irq = OMAP_GPIO_IRQ(GPIO_TSC2004_IRQ),
	},
#endif
};

static int __init omap3logic_i2c_init(void)
{
	omap3_pmic_init("twl4030", &omap3logic_twldata);
	omap_register_i2c_bus(3, 400, omap3logic_i2c3_boardinfo,
			ARRAY_SIZE(omap3logic_i2c3_boardinfo));

	return 0;
}

static struct omap2_hsmmc_info __initdata board_mmc_info[] = {
	{
		.name		= "external",
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{
		.name		= "wl1271",
		.mmc		= 3,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
	},
	{}      /* Terminator */
};

static int __init board_wl12xx_init(void)
{
	unsigned char mac_addr[6];

	// Setup the mux for mmc3
	if (machine_is_dm3730_som_lv() || machine_is_omap3530_lv_som()) {
		omap_mux_init_signal("mcspi1_cs1.sdmmc3_cmd", OMAP_PIN_INPUT_PULLUP); /* McSPI1_CS1/ADPLLV2D_DITHERING_EN2/MMC3_CMD/GPIO_175 */
		omap_mux_init_signal("mcspi1_cs2.sdmmc3_clk", OMAP_PIN_INPUT_PULLUP); /* McSPI1_CS2/MMC3_CLK/GPIO_176 */
	} else if (machine_is_dm3730_torpedo()) {
		omap_mux_init_signal("etk_ctl.sdmmc3_cmd", OMAP_PIN_INPUT_PULLUP); /* ETK_CTL/MMC3_CMD/HSUSB1_CLK/HSUSB1_TLL_CLK/GPIO_13 */
		omap_mux_init_signal("etk_clk.sdmmc3_clk", OMAP_PIN_INPUT_PULLUP); /* ETK_CTL/McBSP5_CLKX/MMC3_CLK/HSUSB1_STP/MM1_RXDP/HSUSB1_TLL_STP/GPIO_12 */
	} else
		return -ENODEV;

	omap_mux_init_signal("sdmmc2_dat4.sdmmc3_dat0", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT4/MMC2_DIR_DAT0/MMC3_DAT0/GPIO_136 */
	omap_mux_init_signal("sdmmc2_dat5.sdmmc3_dat1", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT5/MMC2_DIR_DAT1/CAM_GLOBAL_RESET/MMC3_DAT1/HSUSB3_TLL_STP/MM3_RXDP/GPIO_137 */
	omap_mux_init_signal("sdmmc2_dat6.sdmmc3_dat2", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT6/MMC2_DIR_CMD/CAM_SHUTTER/MMC3_DAT2/HSUSB3_TLL_DIR/GPIO_138 */
	omap_mux_init_signal("sdmmc2_dat7.sdmmc3_dat3", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT7/MMC2_CLKIN/MMC3_DAT3/HSUSB3_TLL_NXT/MM3_RXDM/GPIO_139 */

	if (machine_is_omap3530_lv_som() || machine_is_dm3730_som_lv()) {
		omap_mux_init_gpio(OMAP3LOGIC_WLAN_SOM_LV_PMENA_GPIO, OMAP_PIN_OUTPUT);
		gpio_export(OMAP3LOGIC_WLAN_SOM_LV_PMENA_GPIO, 0);
		omap_mux_init_gpio(OMAP3LOGIC_WLAN_SOM_LV_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
		if (gpio_request_one(OMAP3LOGIC_WLAN_SOM_LV_IRQ_GPIO, GPIOF_IN, "wlan_irq") < 0) {
			printk(KERN_WARNING "Failed to gpio_request %d for wlan_irq\n", OMAP3LOGIC_WLAN_SOM_LV_IRQ_GPIO);
			return -ENODEV;
		}

		omap3logic_wlan_data.irq = OMAP_GPIO_IRQ(OMAP3LOGIC_WLAN_SOM_LV_IRQ_GPIO);
		omap3logic_vwlan.gpio = OMAP3LOGIC_WLAN_SOM_LV_PMENA_GPIO;
		/* wl1271 ref clock is 26 MHz */
		omap3logic_wlan_data.board_ref_clock = WL12XX_REFCLOCK_26;
	} else if (machine_is_dm3730_torpedo()) {
		omap_mux_init_gpio(OMAP3LOGIC_WLAN_TORPEDO_PMENA_GPIO, OMAP_PIN_OUTPUT);
		gpio_export( OMAP3LOGIC_WLAN_TORPEDO_PMENA_GPIO, 0 );
		omap_mux_init_gpio(OMAP3LOGIC_WLAN_TORPEDO_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
		if (gpio_request_one(OMAP3LOGIC_WLAN_TORPEDO_IRQ_GPIO, GPIOF_IN, "wlan_irq") < 0) {
			printk(KERN_WARNING "Failed to gpio_request %d for wlan_irq\n", OMAP3LOGIC_WLAN_TORPEDO_IRQ_GPIO);
			return -ENODEV;
		}
		omap3logic_wlan_data.irq = OMAP_GPIO_IRQ(OMAP3LOGIC_WLAN_TORPEDO_IRQ_GPIO);
		omap3logic_vwlan.gpio = OMAP3LOGIC_WLAN_TORPEDO_PMENA_GPIO;

		/* Pull BT_EN low */
		omap_mux_init_gpio(162, OMAP_PIN_OUTPUT);
		gpio_request_one(162, GPIOF_OUT_INIT_LOW, "bt_en");
		gpio_export(162, 0);
		/* wl128x ref clock is 26 MHz; torpedo TXCO clock is 26Mhz */
		omap3logic_wlan_data.board_ref_clock = WL12XX_REFCLOCK_26;
		omap3logic_wlan_data.board_tcxo_clock = WL12XX_TCXOCLOCK_26;
	} else
		return -ENODEV;

	/* Extract the MAC addr from the productID data */
	if (omap3logic_extract_new_wifi_ethaddr(mac_addr))
		memcpy(omap3logic_wlan_data.mac_addr, mac_addr, sizeof(mac_addr));
	else if (omap3logic_extract_old_wifi_ethaddr(mac_addr))
		memcpy(omap3logic_wlan_data.mac_addr, mac_addr, sizeof(mac_addr));


#ifdef CONFIG_WL12XX_PLATFORM_DATA
	/* WL12xx WLAN Init */
	if (wl12xx_set_platform_data(&omap3logic_wlan_data))
		pr_err("error setting wl12xx data\n");
	platform_device_register(&omap3logic_vwlan_device);
#endif

	return 0;
}

/* Fix the PBIAS voltage for Torpedo MMC1 pins that
 * are used for other needs (IRQs, etc). */
static void omap3torpedo_fix_pbias_voltage(void)
{
	u16 control_pbias_offset = OMAP343X_CONTROL_PBIAS_LITE;
	static int pbias_fixed = 0;
	unsigned long timeout;
	u32 reg;

	if (!pbias_fixed) {
		/* Set the bias for the pin */
		reg = omap_ctrl_readl(control_pbias_offset);

		reg &= ~OMAP343X_PBIASLITEPWRDNZ1;
		omap_ctrl_writel(reg, control_pbias_offset);

		/* 100ms delay required for PBIAS configuration */
		msleep(100);

		/* Set PBIASLITEVMODE1 appropriately */
		if (reg & OMAP343X_PBIASLITESUPPLY_HIGH1)
			reg |= OMAP343X_PBIASLITEVMODE1;
		else
			reg &= ~OMAP343X_PBIASLITEVMODE1;

		reg |= OMAP343X_PBIASLITEPWRDNZ1;

		omap_ctrl_writel(reg, control_pbias_offset);

		/* Wait for pbias to match up */
		timeout = jiffies + msecs_to_jiffies(5);
		do {
			reg = omap_ctrl_readl(control_pbias_offset);
			if (!(reg & OMAP343X_PBIASLITEVMODEERROR1))
				break;
		} while (!time_after(jiffies, timeout));
		if (reg & OMAP343X_PBIASLITEVMODEERROR1)
			printk("%s: Error - VMODE1 doesn't matchup to supply!\n", __FUNCTION__);

		/* For DM3730, turn on GPIO_IO_PWRDNZ to connect input pads*/
		if (cpu_is_omap3630()) {
			reg = omap_ctrl_readl(OMAP36XX_CONTROL_WKUP_CTRL);
			reg |= OMAP36XX_GPIO_IO_PWRDNZ;
			omap_ctrl_writel(reg, OMAP36XX_CONTROL_WKUP_CTRL);
			printk("%s:%d PKUP_CTRL %#x\n", __FUNCTION__, __LINE__, omap_ctrl_readl(OMAP36XX_CONTROL_WKUP_CTRL));
		}

		pbias_fixed = 1;
	}
}

static void __init board_mmc_init(void)
{
	int ret;

	omap3torpedo_fix_pbias_voltage();

	if (machine_is_omap3530_lv_som() || machine_is_dm3730_som_lv()) {
		/* OMAP35x/DM37x LV SOM board */
		board_mmc_info[0].gpio_cd = OMAP3530_LV_SOM_MMC_GPIO_CD;
		board_mmc_info[0].gpio_wp = OMAP3530_LV_SOM_MMC_GPIO_WP;
		/* gpio_cd for MMC wired to CAM_STROBE; cam_strobe and
		 * another pin share GPIO_126. Mux CAM_STROBE as GPIO. */
		omap_mux_init_signal("cam_strobe.gpio_126", OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP);
	} else if (machine_is_omap3_torpedo() || machine_is_dm3730_torpedo()) {
		/* OMAP35x/DM37x Torpedo board */
		board_mmc_info[0].gpio_cd = OMAP3_TORPEDO_MMC_GPIO_CD;
	} else {
		/* unsupported board */
		printk(KERN_ERR "%s(): unknown machine type\n", __func__);
		return;
	}

	/* Check the SRAM for valid product_id data(put there by u-boot). */
	ret = omap3logic_fetch_sram_new_product_id_data();
	if (ret)
		ret = omap3logic_fetch_sram_product_id_data();
	if (ret)
		printk(KERN_ERR "No valid product ID data found in SRAM\n");

	ret = board_wl12xx_init();
	if (ret) {
		/* No wifi configuration for this board */
		board_mmc_info[2].mmc = 0;
	}

	omap2_hsmmc_init(board_mmc_info);

	/* link regulators to MMC adapters */
	omap3logic_vmmc1_supply.dev = board_mmc_info[0].dev;
}

static struct omap_smsc911x_platform_data __initdata board_smsc911x_data = {
	.cs             = OMAP3LOGIC_SMSC911X_CS,
	.gpio_irq       = -EINVAL,
	.gpio_reset     = -EINVAL,
};


static inline void __init board_smsc911x_init(void)
{
	if (machine_is_omap3530_lv_som() || machine_is_dm3730_som_lv()) {
		/* OMAP3530 LV SOM board */
		board_smsc911x_data.gpio_irq =
					OMAP3530_LV_SOM_SMSC911X_GPIO_IRQ;
		omap_mux_init_gpio(OMAP3530_LV_SOM_SMSC911X_GPIO_IRQ, OMAP_PIN_INPUT);
	} else if (machine_is_omap3_torpedo() || machine_is_dm3730_torpedo()) {
		/* OMAP3 Torpedo board */
		board_smsc911x_data.gpio_irq = OMAP3_TORPEDO_SMSC911X_GPIO_IRQ;
		omap_mux_init_gpio(OMAP3_TORPEDO_SMSC911X_GPIO_IRQ, OMAP_PIN_INPUT);
	} else {
		/* unsupported board */
		printk(KERN_ERR "%s(): unknown machine type\n", __func__);
		return;
	}

	gpmc_smsc911x_init(&board_smsc911x_data);
}

#if defined(CONFIG_USB_ISP1763)
/* ISP1763 USB interrupt */
#define OMAP3TORPEDO_ISP1763_IRQ_GPIO          128

static struct isp1763_platform_data omap3logic_isp1763_pdata = {
       .bus_width_8            = 0,
       .port1_otg              = 0,
       .dack_polarity_high     = 0,
       .dreq_polarity_high     = 0,
       .intr_polarity_high     = 0,
       .intr_edge_trigger      = 0,
};

static struct resource omap3logic_isp1763_resources[] = {
       [0] = {
               .flags = IORESOURCE_MEM,
       },
       [1] = {
               .start = -EINVAL,
               .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
       },
};

static struct platform_device omap3logic_isp1763 = {
       .name           = "isp1763",
       .id             = -1,
       .dev            = {
               .platform_data  = &omap3logic_isp1763_pdata,
       },
       .num_resources = ARRAY_SIZE(omap3logic_isp1763_resources),
       .resource = omap3logic_isp1763_resources,
};


static int omap3logic_init_isp1763(void)
{
       unsigned long cs_mem_base;
       unsigned int irq_gpio;

       /* ISP1763 IRQ is an MMC1 data pin - need to update PBIAS
        * to get voltage to the device so the IRQ works correctly rather
        * than float below logic 1 and cause IRQ storm... */
       if (machine_is_dm3730_torpedo() || machine_is_omap3_torpedo())
               omap3torpedo_fix_pbias_voltage();
       else
               return -ENODEV;

       if (gpmc_cs_request(6, SZ_16M, &cs_mem_base) < 0) {
               printk(KERN_ERR "Failed to request GPMC mem for ISP1763\n");
               return -ENOMEM;
       }
       
       omap3logic_isp1763_resources[0].start = cs_mem_base;
       omap3logic_isp1763_resources[0].end = cs_mem_base + 0xffff;

       irq_gpio = OMAP3TORPEDO_ISP1763_IRQ_GPIO;
       omap_mux_init_gpio(irq_gpio, OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE);
       /* Setup ISP1763 IRQ pin as input */
       if (gpio_request(irq_gpio, "isp1763_irq") < 0) {
               printk(KERN_ERR "Failed to request GPIO%d for isp1763 IRQ\n",
               irq_gpio);
               return -EINVAL;
       }
       gpio_direction_input(irq_gpio);
       omap3logic_isp1763_resources[1].start = OMAP_GPIO_IRQ(irq_gpio);
       if (platform_device_register(&omap3logic_isp1763) < 0) {
               printk(KERN_ERR "Unable to register isp1763 device\n");
               gpio_free(irq_gpio);
               return -EINVAL;
       } else {
               pr_info("registered isp1763 platform_device\n");
       }
       return 0;
}
#else
static int omap3logic_init_isp1763(void)
{
	return -ENODEV;
}
#endif

#if defined(CONFIG_USB_MUSB_OMAP2PLUS)
static void omap3logic_musb_init(void)
{
	/* Set up the mux for musb */
	omap_mux_init_signal("hsusb0_clk", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_stp", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_dir", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_nxt", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data0", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data1", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data2", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data3", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data4", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data5", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data6", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data7", OMAP_PIN_INPUT);

	usb_musb_init(NULL);
}
#else
static void omap3logic_musb_init(void)
{
}
#endif

static void __init omap3logic_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static const struct usbhs_omap_board_data usbhs_bdata __initconst = {

	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = 4,
	.reset_gpio_port[2]  = -EINVAL
};

static void omap3logic_init_ehci(void)
{
	omap_mux_init_gpio(usbhs_bdata.reset_gpio_port[1], OMAP_PIN_OUTPUT);
	usbhs_init(&usbhs_bdata);
}

static void omap3logic_usb_init(void)
{
	if (machine_is_omap3530_lv_som() || machine_is_dm3730_som_lv())
		omap3logic_init_ehci();
	else
		omap3logic_init_isp1763();
}

#if defined(CONFIG_OMAP3LOGIC_COMPACT_FLASH) || defined(CONFIG_OMAP3LOGIC_COMPACT_FLASH_MODULE)

#define DM3730_SOM_LV_CF_RESET_GPIO 6
#define DM3730_SOM_LV_CF_EN_GPIO 128
#define DM3730_SOM_LV_CF_CD_GPIO 154

static struct resource omap3logic_som_lv_cf_resources[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = OMAP_GPIO_IRQ(DM3730_SOM_LV_CF_CD_GPIO),
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct omap3logic_cf_data cf_data = {
	.gpio_reset = DM3730_SOM_LV_CF_RESET_GPIO,
	.gpio_en = DM3730_SOM_LV_CF_EN_GPIO,
	.gpio_cd = DM3730_SOM_LV_CF_CD_GPIO,
};

static struct platform_device omap3logic_som_lv_cf = {
	.name		= "omap3logic-cf",
	.id		= 0,
	.dev		= {
		.platform_data	= &cf_data,
	},
	.num_resources = ARRAY_SIZE(omap3logic_som_lv_cf_resources),
	.resource = omap3logic_som_lv_cf_resources,
};

void omap3logic_cf_init(void)
{
	unsigned long cs_mem_base;
	int result;

	/* Only the LV SOM SDK has a CF interface */
	if (!machine_is_dm3730_som_lv())
		return;

	/* Fix PBIAS to get USIM enough voltage to power up */
	omap3torpedo_fix_pbias_voltage();

	if (gpmc_cs_request(3, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for CF\n");
		return;
	}

	omap3logic_som_lv_cf_resources[0].start = cs_mem_base;
	omap3logic_som_lv_cf_resources[0].end = cs_mem_base + 0x1fff;
	
	omap_mux_init_signal("gpmc_ncs3", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_io_dir", OMAP_PIN_OUTPUT);

	omap_mux_init_gpio(DM3730_SOM_LV_CF_CD_GPIO, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(DM3730_SOM_LV_CF_CD_GPIO, "CF card detect") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for CompactFlash CD IRQ\n",
		DM3730_SOM_LV_CF_CD_GPIO);
		return;
	}
	gpio_set_debounce(DM3730_SOM_LV_CF_CD_GPIO, 0xa);
	gpio_direction_input(DM3730_SOM_LV_CF_CD_GPIO);
	gpio_export(DM3730_SOM_LV_CF_CD_GPIO, 0);

	// Setup ComapctFlash Enable pin
	omap_mux_init_gpio(DM3730_SOM_LV_CF_EN_GPIO, OMAP_PIN_OUTPUT);
	if (gpio_request(DM3730_SOM_LV_CF_EN_GPIO, "CF enable") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for CompactFlash EN\n",
		DM3730_SOM_LV_CF_EN_GPIO);
		return;
	}
	gpio_direction_output(DM3730_SOM_LV_CF_EN_GPIO, 0);
	gpio_export(DM3730_SOM_LV_CF_EN_GPIO, 0);

	// Setup ComapctFlash Reset pin
	omap_mux_init_gpio(DM3730_SOM_LV_CF_RESET_GPIO, OMAP_PIN_OUTPUT);
	if (gpio_request(DM3730_SOM_LV_CF_RESET_GPIO, "CF reset") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for CompactFlash Reset\n",
		DM3730_SOM_LV_CF_RESET_GPIO);
		return;
	}
	gpio_direction_output(DM3730_SOM_LV_CF_RESET_GPIO, 0);
	gpio_export(DM3730_SOM_LV_CF_RESET_GPIO, 0);

	result = platform_device_register(&omap3logic_som_lv_cf);
	if (result)
		printk("%s: platform device register of CompactFlash device failed: %d\n", __FUNCTION__, result);
}
#else
static void omap3logic_cf_init(void)
{
}
#endif

/* Code that is invoked only after
 * the product ID data has been found; used for finer-grain
 * board configuration
 */
void omap3logic_init_productid_specifics(void)
{
	omap3logic_init_twl_audio();
}

#ifdef CONFIG_PRINTK_DEBUG
struct printk_debug *printk_debug;
static int __init printk_debug_setup(char *str)
{
	/* printk debug buffer is in start of DRAM; u-boot will look
	 * there for printk_buffer information */
	printk_debug = (void *)PAGE_OFFSET;
	printk_debug->tag = PRINTK_DEBUG_COOKIE;
	return 1;
}
__setup("printk-debug", printk_debug_setup);
#endif

static void __init omap3logic_init(void)
{
	struct omap_board_data bdata;

	/* hang on start if "hang" is on command line */
	while (omap3logic_hang)
		;

	/* Pick the right MUX table based on the machine */
	if (machine_is_dm3730_som_lv() || machine_is_dm3730_torpedo())
		omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);
	else if (machine_is_omap3530_lv_som() || machine_is_omap3_torpedo())
		omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
		
	omap3torpedo_fix_pbias_voltage();
	omap3logic_i2c_init();
	omap3logic_lcd_init();

#ifdef CONFIG_OMAP3LOGIC_UART_A
	printk(KERN_INFO "Setup pinmux and enable UART A\n");
	omap_mux_init_signal("uart1_tx.uart1_tx", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart1_rts.uart1_rts", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart1_cts.uart1_cts", OMAP_PIN_INPUT);
	omap_mux_init_signal("uart1_rx.uart1_rx", OMAP_PIN_INPUT);

	// Taken from serial.c:omap_serial_init()
	bdata.id = 0;
	bdata.flags = 0;
	bdata.pads = NULL;
	bdata.pads_cnt = 0;
	omap_serial_init_port(&bdata);
#endif

#ifdef CONFIG_OMAP3LOGIC_UART_B
	printk(KERN_INFO "Setup pinmux and enable UART B\n");
	omap_mux_init_signal("uart2_tx.uart2_tx", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart2_rts.uart2_rts", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart2_cts.uart2_cts", OMAP_PIN_INPUT);
	omap_mux_init_signal("uart2_rx.uart2_rx", OMAP_PIN_INPUT);

	// Taken from serial.c:omap_serial_init()
	bdata.id = 1;
	bdata.flags = 0;
	bdata.pads = NULL;
	bdata.pads_cnt = 0;
	omap_serial_init_port(&bdata);
#endif

#ifdef CONFIG_OMAP3LOGIC_UART_C
	printk(KERN_INFO "Setup pinmux and enable UART C\n");
	omap_mux_init_signal("uart3_tx_irtx.uart3_tx_irtx", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart3_rts_sd.uart3_rts_sd", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart3_cts_rctx.uart3_cts_rctx", OMAP_PIN_INPUT);
	omap_mux_init_signal("uart3_rx_irrx.uart3_rx_irrx", OMAP_PIN_INPUT);

	// Taken from serial.c:omap_serial_init()
	bdata.id = 2;
	bdata.flags = 0;
	bdata.pads = NULL;
	bdata.pads_cnt = 0;
	omap_serial_init_port(&bdata);
#endif

	board_mmc_init();
	board_smsc911x_init();

	/* Assume NOR is only on CS2 (if its there) */
	omap3logic_nor_init(1<<2, SZ_8M);
	printk("%s:%d\n", __FUNCTION__, __LINE__);
	omap3logic_nand_init();
	printk("%s:%d\n", __FUNCTION__, __LINE__);

	/* Initialixe EHCI port */
	omap3logic_usb_init();

	/* Initialise OTG MUSB port */
	omap3logic_musb_init();

	/* Initialise ComapctFlash */
	omap3logic_cf_init();

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);
}

MACHINE_START(OMAP3_TORPEDO, "Logic OMAP35x Torpedo board")
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap3logic_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap3logic_init,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(OMAP3530_LV_SOM, "Logic OMAP35x SOM LV board")
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap3logic_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap3logic_init,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(DM3730_SOM_LV, "Logic DM37x SOM LV board")
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap3logic_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap3logic_init,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(DM3730_TORPEDO, "Logic DM37x Torpedo board")
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap3logic_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap3logic_init,
	.timer		= &omap_timer,
MACHINE_END
