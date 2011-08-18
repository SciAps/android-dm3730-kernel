/*
 * linux/arch/arm/mach-omap2/board-omap3logic.h
 *
 * Copyright (C) 2011 Logic Product Development
 *
 * Initial code: Peter Barada
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

extern volatile int omap3logic_hang;
extern void omap3logic_nor_init(u32 nor_cs_mask, unsigned long nor_size);
extern void omap3logic_nand_init(void);
