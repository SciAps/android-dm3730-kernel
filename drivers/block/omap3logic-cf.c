/*
 * OMAP SOM LV CF memory card device driver
 *
 * Copyright 2011 Logic Product Development, Inc.
 *
 * Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#define DEBUG
#define STATIC

#define DEBUG_CF_WAIT		0x001
#define DEBUG_CF_STATUS		0x002
#define DEBUG_CF_REQUEST	0x004
#define DEBUG_CF_BLOCK		0x008
#define DEBUG_CF_REG_ACCESS	0x010
#define DEBUG_CF_TRACE		0x020
#define DEBUG_CF_INTERRUPT	0x040
#define DEBUG_CF_ENTRY		0x080
#define DEBUG_CF_ID		0x100
#define DEBUG_CF_GENDISK	0x200		
#define DEBUG_CF_RW_THREAD	0x400
#define DEBUG_CF_ALL		0xfff

#define DEBUG_INIT_BITS DEBUG_CF_INTERRUPT | DEBUG_CF_REQUEST | DEBUG_CF_GENDISK | DEBUG_CF_RW_THREAD

static int omap3logic_cf_debug_bits = (DEBUG_INIT_BITS);
#ifdef DEBUG
#define DEBUG_BITS omap3logic_cf_debug_bits
#  define DPRINTK(flg, fmt, args...)					\
do {									\
	if ((flg) & omap3logic_cf_debug_bits)					\
		printk(KERN_INFO "%s:%d " fmt, __FUNCTION__, __LINE__, ## args); \
} while (0)
#else
#define DEBUG_BITS 0
#  define DPRINTK(flg, fmt, args...) \
do {	\
	; \
} while (0)
#endif

#include <linux/module.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#if 1
#include <linux/genhd.h>
#endif
#include <linux/slab.h>
#include <linux/blkdev.h>
#include <linux/ata.h>
#include <linux/hdreg.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/regulator/machine.h>

#include <plat/omap3logic-cf.h>

#define DRIVER_NAME "omap3logic-cf"

struct cf_device {
	/* driver state data */
	int id;
	int ejected;	/* !0 if disk is not in */
	int disk_in;	/* !0 if disk is in */

	struct regulator *cf_reg; /* CF used for pullup on baseboard */

	/* Transfer state/result, use for both id and block request */
	struct request *req;	/* request being processed */

	/* Details of hardware device */
	unsigned long physaddr;
	unsigned long physize;
	void __iomem *baseaddr;
	int irq;
	// int gpio;
	int gpio_reset, gpio_en, gpio_cd, gpio_cs;
	int bus_width;		/* 0 := 8 bit; 1 := 16 bit */

	/* Block device data structures */
	spinlock_t blk_lock;
	struct device *dev;
	struct request_queue *queue;
	struct gendisk *gd;

	/* Insertion IRQ thread */
	struct semaphore irq_sem;
	struct task_struct *irq_task;		// stack
	struct completion task_irq_completion;	// completion

	/* R/W thread */
	struct semaphore rw_sem;
	struct task_struct *rw_task;			// stack
	struct completion task_rw_completion;		// completion

	/* Inserted CF card parameters */
	int card_inserted;

	u16 cf_id[ATA_ID_WORDS];
};

static struct request *current_req;

#define CF_NUM_MINORS 16

static int cf_major;

#define ATAOFFSET 0x0

#define CB_STAT_BSY 0x8000
#define CB_STAT_DRQ 0x0800
#define CB_STAT_DRDY 0x4000
#define CB_STAT_DF 0x2000
#define CB_STAT_ERR 0x0100

#define CB_DC_HD15 0x08
#define CB_DC_NIEN 0x02

#define CMD_IDENTIFY_DEVICE 0xec
#define CMD_READ_SECTORS 0x20
#define CMD_WRITE_SECTORS 0x30

#define CB_CMD	0x100c	// write
#define CB_DATA 0x1000

// #define CB_STAT 0x101c  // status is in d15:8
#define CB_SECT_NUM_CNT 0x1004 // sector num:sector cnt
#define CB_CYL_HIGH_LOW 0x1008 // cylinder high:cylinder low
#define CB_CMD_DRV_HEAD 0x100c // cmd:drv head
#define CB_STATUS_ALT 0x100e // :alt status
#define CB_DEV_CTRL 0x0e // :dev ctrl

#define CB_LBA 0x40
#define CB_DH	0x06
#define CB_ERR  0x0d	// read
// #define CB_ALT_STAT 0x0e  // read
#define CB_DEV_CTRL 0x0e  // write


#define cf_read_reg16(cf, reg, dump) my_cf_read_reg16(__FUNCTION__, __LINE__, cf, reg, dump)
STATIC u16 my_cf_read_reg16(const char *func, int line, struct cf_device *cf, u32 reg, int dump)
{
	u32 address1, address2;
	u16 data;

	address1 = (u32)cf->baseaddr + ATAOFFSET;

	address2 = address1 + reg;

	data = __raw_readw(address2);

	if (dump)
		DPRINTK(DEBUG_CF_REG_ACCESS, "%s:%d %p+0x%x = %04x\n", func, line, (void *)address1, (address2-address1), data);

	return(data);
}


#define cf_write_reg16(cf, reg, data, dump) my_cf_write_reg16(__FUNCTION__, __LINE__, cf, reg, data, dump)
STATIC void my_cf_write_reg16(const char *func, int line, struct cf_device *cf, u32 reg, u16 data, int dump)
{
	u32 address1, address2;

	address1 = (u32)cf->baseaddr + ATAOFFSET;

	address2 = address1 + reg;

	if (dump)
		DPRINTK(DEBUG_CF_REG_ACCESS, "%s:%d %p+0x%x = %04x\n", func, line, (void *)address1, (address2-address1), data);

	__raw_writew(data, address2);
}

#define TIMEOUT 0x80000

// Wait for the previous command to finish
STATIC int cf_wait_fin(struct cf_device *cf)
{
	int timer;
	u32 reg, old_reg = ~0;

	DPRINTK(DEBUG_CF_WAIT, "\n");

	udelay(500);  // wait 500us

	for (timer = 0; timer < TIMEOUT; ++timer) {
		reg = cf_read_reg16(cf, CB_STATUS_ALT, 0);
		if (DEBUG_BITS & DEBUG_CF_STATUS) {
			if (reg != old_reg) {
				printk("%s: reg %04x\n", __FUNCTION__, reg);
				old_reg = reg;
			}
		}
		if (!(reg & CB_STAT_BSY))
			break;
		yield();
	}

	if (timer == TIMEOUT) {
		printk("%s:%d TIMEOUT! reg %04x\n", __FUNCTION__, __LINE__, reg);
		return -EIO;
	}

	DPRINTK(DEBUG_CF_WAIT, "\n");

	return 0;

}

// Wait for the previous command to finish
STATIC int cf_wait_ready(struct cf_device *cf)
{
	int timer;
	u32 reg, old_reg = ~0;

	DPRINTK(DEBUG_CF_WAIT, "\n");

	udelay(500);  // wait 500us

	for (timer = 0; timer < TIMEOUT; ++timer) {
		reg = cf_read_reg16(cf, CB_STATUS_ALT, 0);
		if (omap3logic_cf_debug_bits & (DEBUG_CF_STATUS)) {
			if (reg != old_reg) {
				printk("%s: reg %04x\n", __FUNCTION__, reg);
				old_reg = reg;
			}
		}
		if (reg & CB_STAT_DRDY)
			break;
		yield();
	}

	if (timer == TIMEOUT) {
		printk("%s:%d TIMEOUT! reg@%0x %04x\n", __FUNCTION__, __LINE__, ((u32)cf->baseaddr+ATAOFFSET+CB_STATUS_ALT), reg);
		return -EIO;
	}

	DPRINTK(DEBUG_CF_WAIT, "\n");

	return 0;

}

// Wait for the previous command to finish
STATIC int cf_wait_drq(struct cf_device *cf, const char *func, int line)
{
	int timer;
	u32 reg, old_reg = ~0;

	DPRINTK(DEBUG_CF_ENTRY, "called from %s:%d\n", func, line);
	udelay(500);  // wait 500us

	for (timer = 0; timer < TIMEOUT; ++timer) {
		reg = cf_read_reg16(cf, CB_STATUS_ALT, 0);
		if (omap3logic_cf_debug_bits & (DEBUG_CF_STATUS)) {
			if (reg != old_reg) {
				printk("%s: reg %04x\n", __FUNCTION__, reg);
				old_reg = reg;
			}
		}
		if (reg & CB_STAT_DRQ)
			break;
		yield();
	}

	if (timer == TIMEOUT) {
		printk("%s:%d TIMEOUT reg=%04x\n", __FUNCTION__, __LINE__, reg);
		return -EIO;
	}

	return 0;
}

int driveno = 0;

STATIC int cf_ident_card(struct cf_device *cf)
{
	int i, ret;
	u16 buf[256];
	unsigned short tmp;

	DPRINTK(DEBUG_CF_TRACE, "\n");

	/* Set disk_in to zero - successful ID command enables it */
	cf->disk_in = 0;

	memset(&cf->cf_id, 0, sizeof(cf->cf_id));

	cf_write_reg16(cf, CB_CMD, 0xa0 | (CMD_IDENTIFY_DEVICE << 8), 1);

	ret = cf_wait_drq(cf, __FUNCTION__, __LINE__);
	if (ret)
		return 1;

	for (i=0; i<256; ++i) {
		tmp = cf_read_reg16(cf, CB_DATA, 0);
		buf[i] = tmp;
	}
	memcpy(cf->cf_id, buf, sizeof(cf->cf_id));

	ata_id_to_hd_driveid(cf->cf_id);

	DPRINTK(DEBUG_CF_GENDISK, "H %d S %d C %d\n", cf->cf_id[ATA_ID_HEADS], cf->cf_id[ATA_ID_SECTORS], cf->cf_id[ATA_ID_CYLS]);

	cf->disk_in = 1;

	return 0;
}


/* ---------------------------------------------------------------------
 * Block ops
 */

// Return 0 for success, non-zero for error.

/* cf_read_sectors: handle a read request */
STATIC int cf_read_sectors(struct cf_device *cf, unsigned char *buffer,u_int block,u_int count)
{
	u_int i,j;
	u_int orig_block = block;
	u_int devHead, status;
	u_int head, sect, cyl_l, cyl_h;
	int ret;

	DPRINTK(DEBUG_CF_BLOCK, "cf %p buffer = %p, block = %u, count = %u\n", cf, buffer, block,count);

	if (count > 255) {
		printk("%s: count %u is too large\n", __FUNCTION__, count);
		return 1;
	}

	// translate into LBA
	sect = block & 0xff;
	block >>= 8;
	cyl_l = block & 0xff;
	block >>= 8;
	cyl_h = block & 0xff;
	block >>= 8;
	head = (block & 0x0f);

	devHead = head | CB_LBA | 0xa0;;

	ret = cf_wait_ready(cf);
	if (ret)
		return 1;

	cf_write_reg16(cf, CB_SECT_NUM_CNT, (sect << 8) | count, 1);
	cf_write_reg16(cf, CB_CYL_HIGH_LOW, cyl_l | (cyl_h << 8), 1);
	cf_write_reg16(cf, CB_CMD_DRV_HEAD, (CMD_READ_SECTORS << 8) | devHead, 1);

	udelay(10);
	for (j=0; j<count; ++j) {
		udelay(1);  // spin for a moment to allow the card to raise
		// busy
		ret = cf_wait_fin(cf);
		if (ret)
			return 1;
		ret = cf_wait_drq(cf, __FUNCTION__, __LINE__);
		if (ret)
			return 1;

		for (i=0; i<256; ++i) {
			u_int data;
			data = cf_read_reg16(cf, CB_DATA, 0);
			*buffer++ = data;
			*buffer++ = data>>8;
		}

		ret = cf_wait_fin(cf);
		if (ret)
			return 1;
		status = cf_read_reg16(cf, CB_STATUS_ALT, 1);
		if (status & (CB_STAT_DF | CB_STAT_ERR)) {
			printk("%s: error at block %u status %#x\n", __FUNCTION__, orig_block+j, status);
			return 1;
		}
	}

	return 0; // Success!
}

STATIC int cf_write_sectors(struct cf_device *cf, unsigned char *buffer,u_int block,u_int count)
{
	u_int i,j;
	u_int orig_block = block;
	u_int devHead, status;
	u_int head, sect, cyl_l, cyl_h;
	int ret;

	DPRINTK(DEBUG_CF_BLOCK, "cf %p buffer = %p, block = %u, count = %u\n", cf, buffer, block,count);

	if (count > 255) {
		printk("%s: count %u is too large\n", __FUNCTION__, count);
		return 1;
	}

	// translate into LBA
	sect = block & 0xff;
	block >>= 8;
	cyl_l = block & 0xff;
	block >>= 8;
	cyl_h = block & 0xff;
	block >>= 8;
	head = (block & 0x0f);

	devHead = head | CB_LBA | 0xa0;;

	ret = cf_wait_ready(cf);
	if (ret)
		return 1;

	cf_write_reg16(cf, CB_SECT_NUM_CNT, (sect << 8) | count, 1);
	cf_write_reg16(cf, CB_CYL_HIGH_LOW, cyl_l | (cyl_h << 8), 1);
	cf_write_reg16(cf, CB_CMD_DRV_HEAD, (CMD_WRITE_SECTORS << 8) | devHead, 1);

	udelay(10);
	for (j=0; j<count; ++j) {
		udelay(1);  // spin for a moment to allow the card to raise
		// busy
		ret = cf_wait_fin(cf);
		if (ret)
			return 1;
		ret = cf_wait_drq(cf, __FUNCTION__, __LINE__);
		if (ret)
			return 1;

		for (i=0; i<256; ++i) {
			u_int data;
			data = *buffer++;
			data |= (*buffer++ << 8);
			cf_write_reg16(cf, CB_DATA, data, 0);
		}

		ret = cf_wait_fin(cf);
		if (ret)
			return 1;
		status = cf_read_reg16(cf, CB_STATUS_ALT, 1);
		if (status & (CB_STAT_DF | CB_STAT_ERR)) {
			printk("%s: error at block %u status %#x\n", __FUNCTION__, orig_block+j, status);
			return 1;
		}
	}

	return 0;
}

STATIC void cf_request(struct request_queue *q)
{
	struct cf_device *cf;

	DPRINTK(DEBUG_CF_REQUEST, "q %p curret_req %p\n", q, current_req);

	BUG_ON(current_req);
	current_req = blk_fetch_request(q);

	DPRINTK(DEBUG_CF_REQUEST, "current_req %p\n", current_req);

	if (current_req == NULL)
		return;

	if (current_req->cmd_type != REQ_TYPE_FS) {
		DPRINTK(DEBUG_CF_REQUEST, "req %p not fs_request\n", current_req);
		__blk_end_request_all(current_req, -EIO);
		current_req = NULL;
		return;
	}

	cf = current_req->rq_disk->private_data;

	/* Due to changes found in 3.0 kernel, I/O requests that yeild can
	 * cause "inconsistent lock state" due to cfq_idle_slice_timer()
	 * trying to get the queue lock.  The only way I can figure out how
	 * to get around this is to stop the block queue while handling
	 * the request.  In cf_rw_task the queue is restarted when the
	 * request is done. */
	blk_stop_queue(q);

	up(&cf->rw_sem);
}

STATIC int cf_media_changed(struct gendisk *gd)
{
	struct cf_device *cf = gd->private_data;

	DPRINTK(DEBUG_CF_GENDISK, "ejected %d\n", cf->ejected);

	return cf->ejected;
}


STATIC int cf_revalidate_disk(struct gendisk *gd)
{
	struct cf_device *cf = gd->private_data;

	DPRINTK(DEBUG_CF_GENDISK, "cf_revalidate_disk: ejected %d disk_in %d\n", cf->ejected, cf->disk_in);

	return !cf->disk_in;
}

static int cf_open(struct block_device *bdev, fmode_t mode)
{
	DPRINTK(DEBUG_CF_GENDISK, "\n");

	check_disk_change(bdev);
	return 0;
}

static int cf_release(struct gendisk *disk, fmode_t mode)
{
	DPRINTK(DEBUG_CF_GENDISK, "cf_release:\n");

	return 0;
}

static int cf_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	struct cf_device *cf = bdev->bd_disk->private_data;

	geo->heads = cf->cf_id[ATA_ID_HEADS];
	geo->sectors = cf->cf_id[ATA_ID_SECTORS];
	geo->cylinders = cf->cf_id[ATA_ID_CYLS];

	DPRINTK(DEBUG_CF_GENDISK, "H %d S %d C %d\n", geo->heads, geo->sectors, geo->cylinders);

	return 0;
}

static struct block_device_operations cf_fops = {
	.owner = THIS_MODULE,
	.open = cf_open,
	.release = cf_release,
	.media_changed = cf_media_changed,
	.revalidate_disk = cf_revalidate_disk,
	.getgeo = cf_getgeo,
};

static irqreturn_t cf_interrupt(int irq, void *v)
{
	struct cf_device *cf = v;

	DPRINTK(DEBUG_CF_INTERRUPT, "\n");

	// On the first interrupt we want to change to rising/falling
	// so we see only the edges...
	irq_set_irq_type(cf->irq, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING);

	up(&cf->irq_sem);

	return IRQ_HANDLED;
}

static void omap3logic_cf_reset_card(struct cf_device *cf)
{
	DPRINTK(DEBUG_CF_TRACE, "GPIO%d(reset) lo/hi/lo\n", cf->gpio_reset);

	gpio_set_value(cf->gpio_reset, 0);
	msleep(10);
	gpio_set_value(cf->gpio_reset, 1);
	msleep(100);
	gpio_set_value(cf->gpio_reset, 0);
	msleep(100);
  
}

static int cf_create_disk(struct cf_device *cf);

static int cf_irq_thread(void *v)
{
	struct cf_device *cf = v;
	struct task_struct *tsk = current;
	int /* card_inserted = ~0, */ old_card_inserted = ~0;
	int ret;

	cf->irq_task = tsk;

	daemonize("omap-cf-irq");
	allow_signal(SIGKILL);

	DPRINTK(DEBUG_CF_INTERRUPT, "Entered\n");

	complete(&cf->task_irq_completion);

	cf->card_inserted = ~0;

	do {

		ret = down_interruptible(&cf->irq_sem);
		if (ret)
			break;

		DPRINTK(DEBUG_CF_INTERRUPT, "got irq_sem\n");

		/* Wait a moment and then read read the nDetect pin */
		msleep(100);

		old_card_inserted = cf->card_inserted;
		cf->card_inserted = !gpio_get_value(cf->gpio_cd);
	  

		if (cf->card_inserted != old_card_inserted) {
			DPRINTK(DEBUG_CF_INTERRUPT, "at %lu: pin %d old %d\n", jiffies, cf->card_inserted, old_card_inserted);
			if (cf->card_inserted) {

				/* Reset the card */
				omap3logic_cf_reset_card(cf);

				/* Ident the card */
				cf_ident_card(cf);

				cf->ejected = 0;

				if (cf->disk_in)
					cf_create_disk(cf);


			} else {
				/* Card removed */
				cf->disk_in = 0;
				cf->ejected = 1;

				if (cf->gd) {
					printk(KERN_INFO "%s: Card removed\n", cf->gd->disk_name);
					del_gendisk(cf->gd);
					put_disk(cf->gd);
					cf->gd = NULL;
				}

			}

			DPRINTK(DEBUG_CF_TRACE, "disk_in %d\n", cf->disk_in);
			// Save the state until next time
			old_card_inserted = cf->card_inserted;
		}

	} while (!signal_pending(tsk));

	DPRINTK(DEBUG_CF_TRACE, "\n");

	cf->irq_task = NULL;
	complete_and_exit(&cf->task_irq_completion, 0);
}

static int cf_rw_thread(void *v)
{
	struct cf_device *cf = v;
	struct task_struct *tsk = current;
	int err;
	unsigned block, count;
	int rw;
	unsigned long flags;

	cf->rw_task = tsk;

	daemonize("omap-cf-rw");
	allow_signal(SIGKILL);

	DPRINTK(DEBUG_CF_RW_THREAD, "Entered\n");

	complete(&cf->task_rw_completion);

	DPRINTK(DEBUG_CF_RW_THREAD, "\n");

	do {
		err = down_interruptible(&cf->rw_sem);
		if (err)
			break;

		DPRINTK(DEBUG_CF_RW_THREAD, "got rw_sem\n");

		BUG_ON(!current_req);
		block = blk_rq_pos(current_req);
		count = blk_rq_cur_sectors(current_req);
		rw = rq_data_dir(current_req);

		DPRINTK(DEBUG_CF_REQUEST, "req %p block %d count %d rw %c\n", current_req, block, count, (rw == READ)?'R':'W');

		if (block+count > get_capacity(current_req->rq_disk)) {
			printk("%u+%u is larger than %llu\n", block, count, get_capacity(current_req->rq_disk));
			err = -EIO;
			goto done;
		}

		if (rw == READ)
			err = cf_read_sectors(cf, current_req->buffer, block, count);
		else
			err = cf_write_sectors(cf, current_req->buffer, block, count);

	done:
		DPRINTK(DEBUG_CF_REQUEST, "__blk_end_request(%p, %d, %u)\n", current_req, err, count<<9);
		spin_lock_irqsave(cf->queue->queue_lock, flags);
		__blk_end_request_all(current_req, err);
		current_req = NULL;

		/* Restart the queue now that the request is complete */
		blk_start_queue(cf->queue);

		spin_unlock_irqrestore(cf->queue->queue_lock, flags);

		DPRINTK(DEBUG_CF_REQUEST, "end\n");


	} while (!signal_pending(tsk));

	DPRINTK(DEBUG_CF_TRACE, "\n");

	cf->rw_task = NULL;
	complete_and_exit(&cf->task_rw_completion, 0);
}


#define IRQ_TYPE IRQF_SAMPLE_RANDOM | IRQF_DISABLED | IRQF_TRIGGER_LOW | IRQF_TRIGGER_HIGH

/* --------------------------------------------------------------------
 * device setup/teardown code
 */
static int __devinit omap3logic_cf_setup(struct cf_device *cf)
{
	int rc;
	// int version;

	DPRINTK(DEBUG_CF_TRACE, "\n");

	DPRINTK(DEBUG_CF_ENTRY, "cf_setup(cf=0x%p)\n", cf);
	DPRINTK(DEBUG_CF_ENTRY, "physaddr=0x%lx physize=0x%lx irq=%i\n", cf->physaddr, cf->physize, cf->irq);

	/*
	 * Map the device
	 */
	cf->baseaddr = ioremap(cf->physaddr, cf->physize);
	if (!cf->baseaddr)
		goto err_ioremap;

	/* Now we can hook up the irq handler */
	if (cf->irq != NO_IRQ) {
		rc = request_irq(cf->irq, cf_interrupt, IRQ_TYPE, "omap CF", cf);
		if (rc) {
			/* Failure - fall back to polled mode */
			dev_err(cf->dev, "request_irq failed\n");
			cf->irq = NO_IRQ;
		}
	}

	DPRINTK(DEBUG_CF_ENTRY, "physaddr 0x%lx, mapped to 0x%p, irq=%i\n",
		cf->physaddr, cf->baseaddr, cf->irq);

	return 0;

err_ioremap:
	dev_info(cf->dev, "omap-cf: error initializing device at 0x%lx\n",
		cf->physaddr);
	return -ENOMEM;
}

static void __devexit omap3logic_cf_teardown(struct cf_device *cf)
{
	DPRINTK(DEBUG_CF_ENTRY, "omap3logic_cf_teardown(%p)\n", cf);

	if (cf->gd) {
		del_gendisk(cf->gd);
		put_disk(cf->gd);
		cf->gd = NULL;
	}

	if (cf->queue)
		blk_cleanup_queue(cf->queue);

	if (cf->irq != NO_IRQ)
		free_irq(cf->irq, cf);

	iounmap(cf->baseaddr);
}

static int cf_create_disk(struct cf_device *cf)
{
	struct gendisk *disk;

	DPRINTK(DEBUG_CF_GENDISK, "cf %p\n", cf);

	disk = alloc_disk(1 << 4);
	if (!disk) {
		DPRINTK(DEBUG_CF_TRACE, "\n");
		blk_cleanup_queue(cf->queue);
		cf->queue = NULL;
		return -ENOMEM;
	}
	cf->gd = disk;

	disk->major = cf_major;
	disk->first_minor = 0;
	disk->fops = &cf_fops;
	disk->queue = cf->queue;
	disk->private_data = cf;
	snprintf(disk->disk_name, 32, "cf%c", cf->id + 'a');

	disk->private_data = cf;

	set_capacity(disk, ata_id_u32(cf->cf_id, ATA_ID_LBA_CAPACITY));

	printk(KERN_INFO "%s: %u MiB\n", disk->disk_name, ata_id_u32(cf->cf_id, ATA_ID_LBA_CAPACITY) >> (20-9));

	add_disk(disk);
	return 0;
}

static int __devinit
omap3logic_cf_alloc(struct platform_device *pdev, int id, unsigned long physaddr,
		unsigned long physize, int irq, int gpio, int bus_width)
{
	struct device *dev = &pdev->dev;
	struct omap3logic_cf_data *cf_data = dev->platform_data;
	struct cf_device *cf;
	int rc;

	DPRINTK(DEBUG_CF_GENDISK, "dev %p\n", dev);

	if (!physaddr) {
		rc = -ENODEV;
		goto err_noreg;
	}

	/* Allocate and initialize the cf device structure */
	cf = kzalloc(sizeof(struct cf_device), GFP_KERNEL);
	if (!cf) {
		rc = -ENOMEM;
		goto err_alloc;
	}

	platform_set_drvdata(pdev, cf);

	cf->dev = dev;
	cf->id = id;
	cf->physaddr = physaddr;
	cf->physize = physize;
	cf->irq = irq;
	cf->gpio_cd = cf_data->gpio_cd;
	cf->gpio_reset = cf_data->gpio_reset;
	cf->gpio_en = cf_data->gpio_en;
	cf->bus_width = bus_width;

	/* Get/enable cf power supply for pullup on baseboard */
	cf->cf_reg = regulator_get(NULL, "vpll2");
	if (IS_ERR(cf->cf_reg)) {
		pr_err("Unable to get CF regulator\n");
		rc = -ENODEV;
		goto err_setup;
	}

	rc = regulator_enable(cf->cf_reg);
	if (rc < 0) {
		pr_err("Unable to enable CF regulator\n");
		goto err_setup;
	}

	/* We fake it as ejected to start with */
	cf->ejected = 1;

	DPRINTK(DEBUG_CF_GENDISK, "\n");
	spin_lock_init(&cf->blk_lock);
	cf->queue = blk_init_queue(cf_request, &cf->blk_lock);
	if (cf->queue == NULL) {
		DPRINTK(DEBUG_CF_TRACE, "\n");
		return -ENOMEM;
	}
	blk_queue_logical_block_size(cf->queue, 512);

	// Limit requests to simple contiguous ones
#if 1
	blk_queue_max_hw_sectors(cf->queue, 255);  /* max sectors in reqeust */
#else
	blk_queue_max_phys_segments(cf->queue, 1);
	blk_queue_max_hw_segments(cf->queue, 1);
#endif

	DPRINTK(DEBUG_CF_GENDISK, "\n");
	/* The IRQ semaphore is locked and only in the IRQ is it released */
	sema_init(&cf->irq_sem, 0);

	/* The RW semaphore to have only one call into either read/write
	 * at a time */
	sema_init(&cf->rw_sem, 0);


	DPRINTK(DEBUG_CF_TRACE, "\n");

	// Create the thread that sits and waits for an interrupt
	init_completion(&cf->task_irq_completion);
	rc = kernel_thread(cf_irq_thread, cf, CLONE_KERNEL);
	if (rc < 0) {
		printk("cf_irq_thread create fail! %d\n", rc);
		goto err_setup;
	} else {
		wait_for_completion(&cf->task_irq_completion);
	}

	DPRINTK(DEBUG_CF_GENDISK, "\n");

	/* Create the thread that handles R/W requests */
	init_completion(&cf->task_rw_completion);
	DPRINTK(DEBUG_CF_GENDISK, "\n");
	rc = kernel_thread(cf_rw_thread, cf, CLONE_KERNEL);
	DPRINTK(DEBUG_CF_GENDISK, "rc %d\n", rc);
	if (rc < 0) {
		printk("cf_rw_thread create fail! %d\n", rc);
		goto err_setup;
	} else {
		DPRINTK(DEBUG_CF_GENDISK, "\n");
		wait_for_completion(&cf->task_rw_completion);
	}

	DPRINTK(DEBUG_CF_GENDISK|DEBUG_CF_TRACE, "\n");

	/* Call the setup code */
	rc = omap3logic_cf_setup(cf);
	if (rc)
		goto err_setup;

	DPRINTK(DEBUG_CF_TRACE, "\n");

	dev_set_drvdata(dev, cf);


	DPRINTK(DEBUG_CF_TRACE, "\n");

	return 0;

err_setup:
	dev_set_drvdata(dev, NULL);
	kfree(cf);
err_alloc:
err_noreg:
	dev_err(dev, "could not initialize device, err=%i\n", rc);
	return rc;
}

static void __devexit omap3logic_cf_free(struct device *dev)
{
	struct cf_device *cf = dev_get_drvdata(dev);
	dev_dbg(dev, "omap3logic_cf_free(%p)\n", dev);

	if (cf) {
		omap3logic_cf_teardown(cf);
		dev_set_drvdata(dev, NULL);
		kfree(cf);
	}
}


/* ---------------------------------------------------------------------
 * Platform Bus Support
 */

#ifdef CONFIG_PM
int omap3logic_cf_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct cf_device *cf = platform_get_drvdata(pdev);

	// Put into reset
	gpio_set_value(cf->gpio_reset, 1);
	udelay(100);

	// disable power
	gpio_set_value(cf->gpio_en, 1);
	msleep(10);

	return 0;
}

static int omap3logic_cf_resume(struct platform_device *pdev)
{
	struct cf_device *cf = platform_get_drvdata(pdev);

	// enable power
	gpio_set_value(cf->gpio_en, 0);
	msleep(10);

	
	// If a card is inserted, then we need to ID it and see if it
	// matches.  If so, then we just reset the card and march forth
	if (!gpio_get_value(cf->gpio_cd)) {
		// card is inserted, reset it.

		omap3logic_cf_reset_card(cf);
	}
	return 0;
}
#endif

static int __devinit omap3logic_cf_probe(struct platform_device *pdev)
{
	unsigned long physaddr = 0;
	unsigned long physend = 1, physize;
	int bus_width = 16; /* FIXME: should not be hard coded */
	int id = pdev->id;
	int irq = NO_IRQ;
	int gpio = 0;
	int i;

	DPRINTK(DEBUG_CF_TRACE, "\n");

	dev_dbg(&pdev->dev, "omap3logic_cf_probe(%p)\n", &pdev->dev);

	for (i = 0; i < pdev->num_resources; i++) {
		if (pdev->resource[i].flags & IORESOURCE_MEM) {
			physaddr = pdev->resource[i].start;
			physend = pdev->resource[i].end;
		}
		if (pdev->resource[i].flags & IORESOURCE_IRQ) {
			irq = pdev->resource[i].start;
		}
	}

	DPRINTK(DEBUG_CF_TRACE, "\n");

	/* Call the bus-independant setup code */
	physize = physend - physaddr + 1;
	return omap3logic_cf_alloc(pdev, id, physaddr, physize, irq, gpio, bus_width);
}

/*
 * Platform bus remove() method
 */
static int __devexit omap3logic_cf_remove(struct platform_device *dev)
{
	omap3logic_cf_free(&dev->dev);
	return 0;
}

static struct platform_driver omap3logic_cf_platform_driver = {
	.probe = omap3logic_cf_probe,
#ifdef CONFIG_PM
	.suspend = omap3logic_cf_suspend,
	.resume = omap3logic_cf_resume,
#endif
	.remove = __devexit_p(omap3logic_cf_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
	},
};

#if defined(CONFIG_DEBUG_FS) && defined(DEBUG)
static int omap3logic_cf_debug_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "%#x\n", omap3logic_cf_debug_bits);

	return 0;
}

static ssize_t omap3logic_cf_debug_write(struct file *file,
					const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	char buf[10];
	unsigned long val;
	int buf_size, ret;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) - 1);

	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	omap3logic_cf_debug_bits = val;

	*ppos += count;
	return count;
}

static int omap3logic_cf_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, omap3logic_cf_debug_show, inode->i_private);
}

static const struct file_operations omap3logic_cf_debug_fops = {
	.open		= omap3logic_cf_debug_open,
	.read		= seq_read,
	.write		= omap3logic_cf_debug_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

/* ---------------------------------------------------------------------
 * Module init/exit routines
 */
static int __init omap3logic_cf_init(void)
{
	int rc;

	cf_major = register_blkdev(cf_major, "omap-cf");
	if (cf_major <= 0) {
		rc = -ENOMEM;
		goto err_blk;
	}

#if defined(CONFIG_DEBUG_FS) && defined(DEBUG)
	(void)debugfs_create_file("cf-debug", S_IRUGO | S_IFREG, NULL,
				NULL, &omap3logic_cf_debug_fops);
#endif

	pr_info("omap-cf: registering platform binding\n");
	rc = platform_driver_register(&omap3logic_cf_platform_driver);
	if (rc)
		goto err_plat;

	printk(KERN_INFO "DM3730 SOM LV memory CompactFlash device driver, major=%i\n", cf_major);

	return 0;

err_plat:
	unregister_blkdev(cf_major, "omap3logic-cf");
err_blk:
	printk(KERN_ERR "omap-cf: registration failed; err=%i\n", rc);
	return rc;
}

static void __exit omap3logic_cf_exit(void)
{
	pr_info("Unregistering DM3730 SOM LV memory CompactFlash  driver\n");
	platform_driver_unregister(&omap3logic_cf_platform_driver);
	unregister_blkdev(cf_major, "omap-cf");
}

module_init(omap3logic_cf_init);
module_exit(omap3logic_cf_exit);

MODULE_AUTHOR("Peter Barada <peter.barada@logicpd.com>");
MODULE_DESCRIPTION("OMAP3LOGIC SOM LV memory CompactFlash device driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
