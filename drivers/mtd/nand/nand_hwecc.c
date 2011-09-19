#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>

int nand_chip_correct_hwecc(struct mtd_info *mtd, u_char *dat,
	u_char *read_ecc, u_char *calc_ecc)
{
	struct nand_chip *chip = mtd->priv;

	DEBUG(MTD_DEBUG_LEVEL3, "%s: ecc_status %02x\n",
				__func__, chip->ecc.status);

	/* We stored the read status in chip->ecc.status in the read.  If bit 0
	 * is set, then there was an uncorrectable ECC error.  If bit 3 is set,
	 * then there was a correctable error (up to four bits of correction).
	 */
	if (chip->ecc.status & 0x01)
		return -1;
	if (chip->ecc.status & 0x08)
		return 4;
	return 0;
}
EXPORT_SYMBOL(nand_chip_correct_hwecc);

int nand_chip_calculate_hwecc(struct mtd_info *mtd, const u_char *dat,
	u_char *ecc_code)
{
	DEBUG(MTD_DEBUG_LEVEL3, "%s:\n", __func__);
	return 0;
}
EXPORT_SYMBOL(nand_chip_calculate_hwecc);

void nand_chip_enable_hwecc(struct mtd_info *mtd, int mode)
{
	DEBUG(MTD_DEBUG_LEVEL3, "%s:\n", __func__);
}
EXPORT_SYMBOL(nand_chip_enable_hwecc);

/**
 * nand_chip_read_oob_hwecc - [REPLACABLE] OOB data read function for on chip HW
 *                         ECC.
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @page:	page number to read
 * @sndcmd:	flag whether to issue read command or not
 */
int nand_chip_read_oob_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
	int page, int sndcmd)
{
	DEBUG(MTD_DEBUG_LEVEL3, "%s: page = %d, len = %i\n",
				__func__, page, mtd->oobsize);

	if (sndcmd) {
		chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
		sndcmd = 0;
	}

	/* Send the status command */
	chip->cmd_ctrl(mtd, NAND_CMD_STATUS,
		NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
	/* Switch to data access */
	chip->cmd_ctrl(mtd, NAND_CMD_NONE,
		NAND_NCE | NAND_CTRL_CHANGE);
	chip->ecc.status = chip->read_byte(mtd);
	DEBUG(MTD_DEBUG_LEVEL3, "%s: ecc_status %02x\n",
				__func__, chip->ecc.status);
	if (chip->ecc.status& (0x8|0x1)) {
		DEBUG(MTD_DEBUG_LEVEL3, "%s:%d page %d ecc_status %02x\n",
					 __func__, __LINE__,
					page, chip->ecc.status);
		if (chip->ecc.status & 0x1)
		{
			mtd->ecc_stats.failed++;
			DEBUG(MTD_DEBUG_LEVEL3, "%s: ecc_stats.failed= %d\n",
				mtd->name, mtd->ecc_stats.failed);
		}
		else if (chip->ecc.status & 0x8)
		{
			mtd->ecc_stats.corrected += 4;
			DEBUG(MTD_DEBUG_LEVEL3, "%s: ecc_stats.corrected= %d\n",
				mtd->name, mtd->ecc_stats.corrected);
		}
	}

	/* Send the read prefix */
	chip->cmd_ctrl(mtd, NAND_CMD_READ0,
		NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
	/* Switch to data access */
	chip->cmd_ctrl(mtd, NAND_CMD_NONE,
		NAND_NCE | NAND_CTRL_CHANGE);
	
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
	return sndcmd;
}
EXPORT_SYMBOL(nand_chip_read_oob_hwecc);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jeff Cooper <jeff.cooper@logicpd.com>");
MODULE_DESCRIPTION("NAND on-chip ECC support");
