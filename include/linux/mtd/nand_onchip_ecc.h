/*
 * Copyright © 2011 Jeff Cooper <jeff.cooper@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This file is the header for the on-chip NAND ECC implementation.
 */

#ifndef __MTD_NAND_ONCHIP_ECC_H__
#define __MTD_NAND_ONCHIP_ECC_H__

struct mtd_info;

#if defined(CONFIG_MTD_NAND_ONCHIP_ECC)
/*
 * Enable/Disable on-chip ECC.
 */
void nand_onchip_enable_ecc(struct mtd_info *mtd, int enable);

/*
 * Calculate on-chip ecc code
 */
int nand_onchip_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
			      u_char *ecc_code);

/*
 * Detect and correct bit errors
 */
int nand_onchip_correct_ecc(struct mtd_info *mtd, u_char *dat, u_char *read_ecc,
			    u_char *calc_ecc);

int nand_onchip_read_oob_ecc(struct mtd_info *mtd, struct nand_chip *chip,
			     int page, int sndcmd);

void nand_onchip_hwctl(struct mtd_info *mtd, int mode);

#else /* !CONFIG_MTD_NAND_ONCHIP_ECC */

static inline void nand_onchip_enable_ecc(struct mtd_info *mtd, int enable)
{
	return;
}

static inline int
nand_onchip_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
			  u_char *ecc_code)
{
	return -1;
}

static inline int
nand_onchip_correct_ecc(struct mtd_info *mtd, u_char *dat, u_char *read_ecc,
			u_char *calc_ecc)
{
	return -1;
}

static inline int
nand_onchip_read_oob_ecc(struct mtd_info *mtd, struct nand_chip *chip,
			 int page, int sndcmd)
{
	return -1;
}

static inline void nand_onchip_hwctl(struct mtd_info *mtd, int mode)
{
	return;
}

#endif /* CONFIG_MTD_NAND_ONCHIP_ECC */

#endif /* __MTD_NAND_NOCHIP_ECC_H__ */
