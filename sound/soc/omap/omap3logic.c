/*
 * omap3logic.c  -- ALSA SoC support for OMAP35x/DM37x SOM LV/Torpedo
 *
 * Author: Peter Barada <peter.barada@logicpd.com>
 *
 * Based on sound/soc/omap/omap3evm.c by Anuj Aggarwal
 *
 * Copyright (C) 2011 Logic Product Development, Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"

static int omap3logic_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

#if 0
	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "Can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "Can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "Can't set codec system clock\n");
		return ret;
	}
#else
	unsigned int fmt;

    switch (params_channels(params)) {
    case 2: /* Stereo I2S mode */
        fmt =   SND_SOC_DAIFMT_I2S |
            SND_SOC_DAIFMT_NB_NF |
            SND_SOC_DAIFMT_CBM_CFM;
        break;
    case 4: /* Four channel TDM mode */
        fmt =   SND_SOC_DAIFMT_DSP_A |
            SND_SOC_DAIFMT_IB_NF |
            SND_SOC_DAIFMT_CBM_CFM;
        break;
    default:
        return -EINVAL;
    }   

    /* Set codec DAI configuration */
    ret = snd_soc_dai_set_fmt(codec_dai, fmt);
    if (ret < 0) {
        printk(KERN_ERR "can't set codec DAI configuration\n");
        return ret;
    }   

    /* Set cpu DAI configuration */
    ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
    if (ret < 0) {
        printk(KERN_ERR "can't set cpu DAI configuration\n");
        return ret;
    }   

    /* Set the codec system clock for DAC and ADC */
    ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
                     SND_SOC_CLOCK_IN);
    if (ret < 0) {
        printk(KERN_ERR "can't set codec system clock\n");
        return ret;
    }   

#endif
	return 0;
}

static struct snd_soc_ops omap3logic_ops = {
	.hw_params = omap3logic_hw_params,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link omap3logic_dai = {
	.name 		= "TWL4030",
	.stream_name 	= "TWL4030",
	.cpu_dai_name 	= "omap-mcbsp-dai.1",
	.codec_dai_name	= "twl4030-hifi",
	.codec_name	= "twl4030-codec",
	.ops 		= &omap3logic_ops,
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_omap3logic = {
	.name		= "omap3logic",
	.owner		= THIS_MODULE,
	.dai_link	= &omap3logic_dai,
	.num_links	= 1,
};

/* twl4030 setup */
/*
static struct twl4030_setup_data twl4030_setup = {
	.ramp_delay_value = 4,
	.sysclk = 26000,
};
*/

static struct platform_device *omap3logic_snd_device;

static int __init omap3logic_soc_init(void)
{
	int ret;

	if (!(machine_is_dm3730_som_lv() || machine_is_dm3730_torpedo()
			|| machine_is_omap3530_lv_som() || machine_is_omap3_torpedo())) {
		return -ENODEV;
	}
	pr_info("Omap3logic SoC init\n");

	omap3logic_snd_device = platform_device_alloc("soc-audio", -1);
	if (!omap3logic_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(omap3logic_snd_device, &snd_soc_omap3logic);

	ret = platform_device_add(omap3logic_snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(omap3logic_snd_device);

	return ret;
}

static void __exit omap3logic_soc_exit(void)
{
	platform_device_unregister(omap3logic_snd_device);
}

module_init(omap3logic_soc_init);
module_exit(omap3logic_soc_exit);

MODULE_AUTHOR("Peter Barada <peter.barada@logicpd.com>");
MODULE_DESCRIPTION("ALSA SoC OMAP3 Logic reference boards");
MODULE_LICENSE("GPL v2");
