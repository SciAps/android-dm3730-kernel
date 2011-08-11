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
#include <linux/wl12xx.h>
#include <linux/mmc/host.h>

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
#include <plat/common.h>
#include <plat/gpmc-smsc911x.h>
#include <plat/gpmc.h>
#include <plat/sdrc.h>

#include "board-omap3logic.h"

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

#define OMAP3LOGIC_WLAN_PMENA_GPIO 3
#define OMAP3LOGIC_WLAN_IRQ_GPIO 2

static struct fixed_voltage_config omap3logic_vwlan = {
	.supply_name		= "vwl1271",
	.microvolts		= 1800000, /* 1.8V */
	.gpio			= OMAP3LOGIC_WLAN_PMENA_GPIO,
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
	.irq = OMAP_GPIO_IRQ(OMAP3LOGIC_WLAN_IRQ_GPIO),
	/* ZOOM ref clock is 26 MHz */
	.board_ref_clock = 1,
};

static struct twl4030_gpio_platform_data omap3logic_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2)  | BIT(6)  | BIT(7)  | BIT(8)
			| BIT(13) | BIT(15) | BIT(16) | BIT(17),
};

static struct twl4030_platform_data omap3logic_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.gpio		= &omap3logic_gpio_data,
	.vmmc1		= &omap3logic_vmmc1,
	.vaux3		= &omap3logic_vaux3,
};

static int __init omap3logic_i2c_init(void)
{
	omap3_pmic_init("twl4030", &omap3logic_twldata);
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

static void __init board_wl12xx_init(void)
{
	// Setup the mux for mmc3
	omap_mux_init_signal("mcspi1_cs1.sdmmc3_cmd", OMAP_PIN_INPUT_PULLUP); /* McSPI1_CS1/ADPLLV2D_DITHERING_EN2/MMC3_CMD/GPIO_175 */
	omap_mux_init_signal("mcspi1_cs2.sdmmc3_clk", OMAP_PIN_INPUT_PULLUP); /* McSPI1_CS2/MMC3_CLK/GPIO_176 */
	omap_mux_init_signal("sdmmc2_dat4.sdmmc3_dat0", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT4/MMC2_DIR_DAT0/MMC3_DAT0/GPIO_136 */
	omap_mux_init_signal("sdmmc2_dat5.sdmmc3_dat1", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT5/MMC2_DIR_DAT1/CAM_GLOBAL_RESET/MMC3_DAT1/HSUSB3_TLL_STP/MM3_RXDP/GPIO_137 */
	omap_mux_init_signal("sdmmc2_dat6.sdmmc3_dat2", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT6/MMC2_DIR_CMD/CAM_SHUTTER/MMC3_DAT2/HSUSB3_TLL_DIR/GPIO_138 */
	omap_mux_init_signal("sdmmc2_dat7.sdmmc3_dat3", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT7/MMC2_CLKIN/MMC3_DAT3/HSUSB3_TLL_NXT/MM3_RXDM/GPIO_139 */

	omap_mux_init_gpio(OMAP3LOGIC_WLAN_PMENA_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(OMAP3LOGIC_WLAN_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
}

static void __init board_mmc_init(void)
{
	if (machine_is_omap3530_lv_som() || machine_is_dm3730_som_lv()) {
		/* OMAP35x/DM37x LV SOM board */
		board_mmc_info[0].gpio_cd = OMAP3530_LV_SOM_MMC_GPIO_CD;
		board_mmc_info[0].gpio_wp = OMAP3530_LV_SOM_MMC_GPIO_WP;
		omap_mux_init_signal("gpio_110", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("cam_strobe.gpio_126", OMAP_PIN_OUTPUT);
	} else if (machine_is_omap3_torpedo() || machine_is_dm3730_torpedo()) {
		/* OMAP35x/DM37x Torpedo board */
		board_mmc_info[0].gpio_cd = OMAP3_TORPEDO_MMC_GPIO_CD;
		omap_mux_init_signal("gpio_127", OMAP_PIN_OUTPUT);
	} else {
		/* unsupported board */
		printk(KERN_ERR "%s(): unknown machine type\n", __func__);
		return;
	}

	omap2_hsmmc_init(board_mmc_info);
#ifdef CONFIG_WL12XX_PLATFORM_DATA
	/* WL12xx WLAN Init */
	if (wl12xx_set_platform_data(&omap3logic_wlan_data))
		pr_err("error setting wl12xx data\n");
	platform_device_register(&omap3logic_vwlan_device);
#endif
	/* link regulators to MMC adapters */
	omap3logic_vmmc1_supply.dev = board_mmc_info[0].dev;
}

static struct omap_smsc911x_platform_data __initdata board_smsc911x_data = {
	.cs             = OMAP3LOGIC_SMSC911X_CS,
	.gpio_irq       = -EINVAL,
	.gpio_reset     = -EINVAL,
};

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

		printk("%s:%d reg %08x\n", __FUNCTION__, __LINE__, reg);

		reg &= ~OMAP343X_PBIASLITEPWRDNZ1;
		omap_ctrl_writel(reg, control_pbias_offset);

		printk("%s:%d reg %08x\n", __FUNCTION__, __LINE__, reg);

		/* 100ms delay required for PBIAS configuration */
		msleep(100);

		/* Set PBIASLITEVMODE1 appropriately */
		if (reg & OMAP343X_PBIASLITESUPPLY_HIGH1)
			reg |= OMAP343X_PBIASLITEVMODE1;
		else
			reg &= ~OMAP343X_PBIASLITEVMODE1;

		reg |= OMAP343X_PBIASLITEPWRDNZ1;

		omap_ctrl_writel(reg, control_pbias_offset);

		printk("%s:%d reg %08x\n", __FUNCTION__, __LINE__, reg);

		/* Wait for pbias to match up */
		timeout = jiffies + msecs_to_jiffies(5);
		do {
			reg = omap_ctrl_readl(control_pbias_offset);
			if (!(reg & OMAP343X_PBIASLITEVMODEERROR1))
				break;
		} while (!time_after(jiffies, timeout));
		printk("%s:%d reg %08x\n", __FUNCTION__, __LINE__, reg);
		if (reg & OMAP343X_PBIASLITEVMODEERROR1)
			printk("%s: Error - VMODE1 doesn't matchup to supply!\n", __FUNCTION__);

		/* For DM3730, turn on GPIO_IO_PWRDNZ to connect input pads*/
		if (cpu_is_omap3630()) {
			reg = omap_ctrl_readl(OMAP36XX_CONTROL_WKUP_CTRL);
			printk("%s:%d reg %08x\n", __FUNCTION__, __LINE__, reg);
			reg |= OMAP36XX_GPIO_IO_PWRDNZ;
			omap_ctrl_writel(reg, OMAP36XX_CONTROL_WKUP_CTRL);
			printk("%s:%d PKUP_CTRL %#x\n", __FUNCTION__, __LINE__, omap_ctrl_readl(OMAP36XX_CONTROL_WKUP_CTRL));
		}

		pbias_fixed = 1;
	}
}

static inline void __init board_smsc911x_init(void)
{
	if (machine_is_omap3530_lv_som() || machine_is_dm3730_som_lv()) {
		/* OMAP3530 LV SOM board */
		board_smsc911x_data.gpio_irq =
					OMAP3530_LV_SOM_SMSC911X_GPIO_IRQ;
		omap_mux_init_signal("gpio_152", OMAP_PIN_INPUT);
	} else if (machine_is_omap3_torpedo() || machine_is_dm3730_torpedo()) {
		/* OMAP3 Torpedo board */
		board_smsc911x_data.gpio_irq = OMAP3_TORPEDO_SMSC911X_GPIO_IRQ;
		omap_mux_init_signal("gpio_129", OMAP_PIN_INPUT);
	} else {
		/* unsupported board */
		printk(KERN_ERR "%s(): unknown machine type\n", __func__);
		return;
	}

	gpmc_smsc911x_init(&board_smsc911x_data);
}

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

static void __init omap3logic_init(void)
{
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
	omap_serial_init();
	board_wl12xx_init();
	board_mmc_init();
	board_smsc911x_init();

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);
}

MACHINE_START(OMAP3_TORPEDO, "Logic OMAP35x Torpedo board")
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.init_early	= omap3logic_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap3logic_init,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(OMAP3530_LV_SOM, "Logic OMAP35x SOM LV board")
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.init_early	= omap3logic_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap3logic_init,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(DM3730_SOM_LV, "Logic DM37x SOM LV board")
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.init_early	= omap3logic_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap3logic_init,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(DM3730_TORPEDO, "Logic DM37x Torpedo board")
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.init_early	= omap3logic_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap3logic_init,
	.timer		= &omap_timer,
MACHINE_END
