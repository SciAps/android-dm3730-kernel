/*
 * Copyright © 2011 Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This file is the header for the on-chip NAND ECC implementation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_disturb.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#ifdef CONFIG_DEBUG_FS

static int nand_disturb_debug;

static int nand_disturb_dbg_show(struct seq_file *s, void *unused)
{
	struct nand_chip *chip = s->private;
	struct nand_disturb_stats *p;
	int i;

	seq_printf(s, "chip: %p\n", chip);
	if (chip) {
		p = chip->disturb;
		seq_printf(s, "chip->disturb: %p\n", p);
		seq_printf(s, "num_blocks: %u read_limit %u erase_limit %u\n",
			p->num_blocks, p->read_limit, p->erase_limit);
		for (i=0; i<p->num_blocks; ++i)
			if (p->stats[i].read_count || p->stats[i].erase_count)
				seq_printf(s, "%u: [%u %u]\n", i,
					p->stats[i].read_count, p->stats[i].erase_count);
	}
	return 0;
}

static ssize_t nand_disturb_dbg_write(struct file *file,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	char buf[10];
	unsigned long val;
	int buf_size, ret;

	if (count > 10)
		return -EINVAL;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	ret = strict_strtoul(buf, 0x10, &val);
	if (ret < 0)
		return ret;
	nand_disturb_debug = val;

	return count;
}

static int nand_disturb_dbg_open(struct inode *inode, struct file *file)
{
	return single_open(file, nand_disturb_dbg_show, inode->i_private);
}

static const struct file_operations nand_disturb_dbg_fops = {
	.open		= nand_disturb_dbg_open,
	.read		= seq_read,
	.write		= nand_disturb_dbg_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static void nand_disturb_files(struct nand_chip *chip)
{
	(void)debugfs_create_file("nand-disturb", S_IWUSR, NULL, chip, &nand_disturb_dbg_fops);
}
#else
static void nand_disturb_files(struct nand chip *chip)
{
}
#endif

void nand_alloc_disturb(struct nand_chip *chip, uint32_t num_blocks, uint32_t erase_limit, uint32_t read_limit)
{
	struct nand_disturb_stats *p;
	uint32_t size;
	size = sizeof(struct nand_disturb_stats) + num_blocks * sizeof(struct nand_disturb);
	p = kzalloc(size, GFP_KERNEL);
	if (p) {
		p->num_blocks = num_blocks;
		p->erase_limit = erase_limit;
		p->read_limit = read_limit;
		if (nand_disturb_debug)
			printk("%s: p %p num_blocks %u erase_limit %u read_limit %u\n", 
				__FUNCTION__, p, num_blocks, erase_limit, read_limit);
		nand_disturb_files(chip);
	}
	chip->disturb = p;
}

int nand_disturb_incr_read_cnt(struct nand_chip *chip, uint32_t page)
{
	struct nand_disturb_stats *p = chip->disturb;
	uint32_t block = page >> (chip->phys_erase_shift - chip->page_shift);

	if (nand_disturb_debug)
		printk("%s: page %u block %u phys_erase_shift %u page_shift %u\n",
			__FUNCTION__, page, block, chip->phys_erase_shift, chip->page_shift);
	if (p) {
		WARN_ON(block > p->num_blocks);
		if (nand_disturb_debug)
			printk("%s: block %u read_count %d\n", __FUNCTION__, block, p->stats[block].read_count);
		p->stats[block].read_count++;
		if (p->stats[block].read_count > p->read_limit)
			return -ESTALE;
		else
			return 0;
	}
	return -ERANGE;
}

int nand_disturb_incr_erase_cnt(struct nand_chip *chip, uint32_t page)
{
	struct nand_disturb_stats *p = chip->disturb;
	uint32_t block = page >> (chip->phys_erase_shift - chip->page_shift);

	if (nand_disturb_debug)
		printk("%s: page %u block %u phys_erase_shift %u page_shift %u\n",
			__FUNCTION__, page, block, chip->phys_erase_shift, chip->page_shift);
	if (p) {
		WARN_ON(block > p->num_blocks);
		if (nand_disturb_debug)
			printk("%s: block %u erase_count %d\n", __FUNCTION__, block, p->stats[block].erase_count);
		p->stats[block].erase_count++;
		p->stats[block].read_count = 0;
		if (p->stats[block].erase_count > p->erase_limit)
			return -EUCLEAN;
		else
			return 0;
	}
	return -ERANGE;
}
