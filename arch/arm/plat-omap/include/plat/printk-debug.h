/*
 * linux/arch/arm/plat-omap/include/printk-debug.h
 *
 * Copyright (C) 2011 Logic Product Development, Inc.
 * Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * The following structure is used by x-loader to see if there is
 * a printk buffer in memory, and if so, dump the last "ndump_chars"
 * of it (on a line boundary) */
#define PRINTK_DEBUG_COOKIE 0xfeedf001
struct printk_debug {
	unsigned int tag;
	char *log_buf_phys;
	unsigned log_size;
	unsigned log_start;
	unsigned log_end;
	unsigned ndump_chars;
};

extern struct printk_debug *printk_debug;

/* Offset from start of SDRAM to store the printk_debug record */
#define PRINTK_DEBUG_OFFSET 0x0
