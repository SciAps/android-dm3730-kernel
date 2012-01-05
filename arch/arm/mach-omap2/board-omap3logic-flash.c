/*
 * linux/arch/arm/mach-omap2/board-omap3logic-flash.c
 *
 * Copyright (C) 2011 Logic Product Development
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mtd/physmap.h>
#include <linux/io.h>
#include <plat/irqs.h>

#include <linux/mtd/physmap.h>
#include <linux/mtd/nand.h>

#include <plat/gpmc.h>
#include <plat/nand.h>

#include <plat/board-omap3logic.h>

static struct mtd_partition omap3logic_nor_partitions[] = {
	{
		.name	= "nor-flash",
		.offset	= 0,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct physmap_flash_data omap3logic_nor_data = {
	.width		= 2,
	.parts		= omap3logic_nor_partitions,
	.nr_parts	= ARRAY_SIZE(omap3logic_nor_partitions),
};

static int __init _nor_init(u32 id, u32 nor_cs, unsigned long cs_mem_size)
{
	struct platform_device *pdev;
	struct resource *res;
	unsigned long cs_mem_base;
	int err;

	printk("%s: id %d nor_cs %d cs_mem_size %08lx\n", __FUNCTION__,
		id, nor_cs, cs_mem_size);

	if ((err = gpmc_cs_request(nor_cs, cs_mem_size, &cs_mem_base)) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for NOR%d flash\n", id);
		return err;
	}

	pdev = kmalloc(sizeof(*pdev), GFP_KERNEL);
	res = kmalloc(sizeof(*res), GFP_KERNEL);
	if (!pdev || !res) {
		printk(KERN_ERR "Failed to allocate NOR%d resource/device\n", id);
		return -ENOMEM;
	}
	
	memset(res, 0, sizeof(*res));
	res->flags	= IORESOURCE_MEM;
	res->start	= cs_mem_base;
	res->end	= cs_mem_base + cs_mem_size;

	memset(pdev, 0, sizeof(*pdev));
	pdev->name		= "physmap-flash";
	pdev->dev.platform_data	= &omap3logic_nor_data;
	pdev->num_resources	= 1;
	pdev->resource		= res;

	if (platform_device_register(pdev) < 0) {
		printk(KERN_ERR "Unable to register NOR%d device\n", id);
		return -ENODEV;
	}
	return 0;
}

int __init omap3logic_nor_init(u32 nor_cs_mask, unsigned long nor_size)
{
	u32 config1, config7;
	int err;
	u32 id, nor_cs;

	for (id=0, nor_cs=0; nor_cs<GPMC_CS_NUM; ++nor_cs) {
		if (nor_cs_mask & (1 << nor_cs)) {
			config1 = gpmc_cs_read_reg(nor_cs, GPMC_CS_CONFIG1);
			config7 = gpmc_cs_read_reg(nor_cs, GPMC_CS_CONFIG7);
	
			/* Did the bootloader setup NOR CSs? */
			if ((config7 & GPMC_CONFIG7_CSVALID)) {
				if (GPMC_CONFIG1_EXTRACT_DEVICETYPE(config1) == GPMC_DEVICETYPE_NOR) {
					err = _nor_init(id, nor_cs, nor_size);
					if (err < 0)
						return err;
					id++;
				}
			}
		}
	}
	return 0;
}

static struct mtd_partition omap3logic_nand_partitions[] = {
	{
		.name	= "nand-flash",
		.offset	= 0,
		.size	= MTDPART_SIZ_FULL,
	},
};

#ifdef CONFIG_MTD_NAND_OMAP2
static int omap3logic_use_soft_bch;
static int __init omap3logic_soft_bch_option(char *str)
{
	omap3logic_use_soft_bch = 1;
	return 1;
}

__setup("soft-bch", omap3logic_soft_bch_option);

static int __init _nand_init(u32 id, u32 nand_cs)
{
	struct platform_device *pdev;
	struct omap_nand_platform_data *ndata;
	struct resource *res;
	unsigned long cs_mem_base;
	int err;

	printk("%s: id %d nand_cs %d\n", __FUNCTION__,
		id, nand_cs);

	if ((err = gpmc_cs_request(nand_cs, NAND_IO_SIZE, &cs_mem_base)) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for NAND%d flash\n", id);
		return err;
	}

	res = kmalloc(sizeof(*res), GFP_KERNEL);
	ndata = kmalloc(sizeof(*ndata), GFP_KERNEL);
	pdev = kmalloc(sizeof(*pdev), GFP_KERNEL);
	if (!res || !ndata || !pdev) {
		printk(KERN_ERR "Failed to allocate NAND%d resource/data/device\n", id);
		return -ENOMEM;
	}

	memset(ndata, 0, sizeof(*ndata));
	ndata->cs		= nand_cs;
	ndata->devsize		= NAND_BUSWIDTH_16 /* | NAND_SKIP_BBTSCAN */;
	ndata->xfer_type	= NAND_OMAP_PREFETCH_DMA;
	ndata->parts		= omap3logic_nand_partitions;
	ndata->nr_parts		= ARRAY_SIZE(omap3logic_nand_partitions);
	ndata->ecc_opt		= OMAP_ECC_HAMMING_CODE_HW_ROMCODE;
	if (omap3logic_use_soft_bch) {
		/* use software BCH ECC if new Micron NAND w/internal ECC */
		ndata->ecc_opt |= OMAP_ECC_BCH_NEW_MICRON;
	}
	ndata->gpmc_irq		= OMAP_GPMC_IRQ_BASE + nand_cs;
	ndata->nand_setup	= omap2_nand_gpmc_retime;
	ndata->phys_base	= cs_mem_base;

	if ((err = omap2_nand_gpmc_retime(ndata)) < 0) {
		printk(KERN_ERR "Unable to set GPMC timings for NAND%d\n", id);
		return err;
	}

#if 0
	/* Enable RD PIN Monitoring Reg */
	gpmc_cs_configure(cs, GPMC_CONFIG_RDY_BSY, 1);
#endif

	memset(res, 0, sizeof(*res));
	res->flags		= IORESOURCE_MEM;

	memset(pdev, 0, sizeof(*pdev));
	pdev->name		= "omap2-nand";
	pdev->id		= id;
	pdev->dev.platform_data	= ndata;
	pdev->num_resources	= 1;
	pdev->resource		= res;

	if (platform_device_register(pdev) < 0) {
		printk(KERN_ERR "Unable to register NAND%d device\n", id);
		return -ENODEV;
	}
	return 0;
}

int __init omap3logic_nand_init(void)
{
	u32 config1, config7;
	u32 nand_cs, id;
	int err;

	for (nand_cs=0, id=0; nand_cs<GPMC_CS_NUM; ++nand_cs) {
		config1 = gpmc_cs_read_reg(nand_cs, GPMC_CS_CONFIG1);
		config7 = gpmc_cs_read_reg(nand_cs, GPMC_CS_CONFIG7);
	
		/* Did the bootloader setup NAND CSs? */
		if ((config7 & GPMC_CONFIG7_CSVALID)) {
			if (GPMC_CONFIG1_EXTRACT_DEVICETYPE(config1) == GPMC_DEVICETYPE_NAND) {
				err = _nand_init(id, nand_cs);
				if (err < 0)
					return err;
				id++;
			}
		}
	}
	return 0;
}
#else
int __init omap3logic_nand_init(void)
{
}
#endif

