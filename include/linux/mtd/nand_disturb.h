/*
 * Copyright © 2011 Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This file is the header for the on-chip NAND ECC implementation.
 */

#ifndef __NAND_DISTURB_H__
#define __NAND_DISTURB_H__

struct nand_disturb {
	uint32_t erase_count;
	uint32_t read_count;
	uint32_t max_read_count;
};
struct nand_disturb_stats {
	uint32_t erase_limit;
	uint32_t read_limit;
	uint32_t num_blocks;
	struct nand_disturb stats[0];
};

extern void nand_alloc_disturb(struct nand_chip *chip, uint32_t num_blocks, uint32_t erase_limit, uint32_t read_limit);
extern int nand_disturb_incr_read_cnt(struct nand_chip *chip, uint32_t page);
extern int nand_disturb_incr_erase_cnt(struct nand_chip *chip, uint32_t page);
#endif
