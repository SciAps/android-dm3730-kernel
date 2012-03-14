#include <linux/delay.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>

/* Extended commands for ONFI devices */
#define NAND_CMD_GET_FEATURES	0xee
#define NAND_CMD_SET_FEATURES	0xef

#define ONCHIP_MTD_DEBUG_LEVEL	MTD_DEBUG_LEVEL4

static void micron_set_features(struct mtd_info *mtd, uint8_t faddr,
				uint8_t *features)
{
	struct nand_chip *chip = mtd->priv;

	chip->select_chip(mtd, 0);

	/* Send the status command */
	chip->cmd_ctrl(mtd, NAND_CMD_SET_FEATURES,
		NAND_CTRL_CHANGE | NAND_CTRL_CLE);

	/* Send the feature address */
	chip->cmd_ctrl(mtd, faddr, NAND_CTRL_CHANGE | NAND_CTRL_ALE);

	/* Switch to data access */
	chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_CTRL_CHANGE | NAND_NCE);

	ndelay(100);

	DEBUG(ONCHIP_MTD_DEBUG_LEVEL, "%s: IO_ADDR_W %p\n",
		__func__, chip->IO_ADDR_W);

	if (chip->options & NAND_BUSWIDTH_16) {
		uint16_t ftrs16[4];
		int i;
		for (i=0; i<4; ++i)
			ftrs16[i] = features[i];
		chip->write_buf(mtd, (uint8_t *)ftrs16, sizeof(ftrs16));
	} else
		chip->write_buf(mtd, features, 4);

	udelay(2);

	DEBUG(ONCHIP_MTD_DEBUG_LEVEL, "%s: faddr %02x [%02x %02x %02x %02x]\n",
		__func__, faddr,
		features[0], features[1], features[2], features[3]);
}

static void micron_get_features(struct mtd_info *mtd, uint8_t faddr,
				uint8_t *features)
{
	struct nand_chip *chip = mtd->priv;

	chip->select_chip(mtd, 0);

	/* Send the status command */
	chip->cmd_ctrl(mtd, NAND_CMD_GET_FEATURES,
		NAND_CTRL_CHANGE | NAND_CTRL_CLE);

	/* Send the feature address */
	chip->cmd_ctrl(mtd, faddr, NAND_CTRL_CHANGE | NAND_CTRL_ALE);

	/* Switch to data access */
	chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_CTRL_CHANGE | NAND_NCE);

	ndelay(100);

	DEBUG(ONCHIP_MTD_DEBUG_LEVEL, "%s: IO_ADDR_R %p\n",
		__func__, chip->IO_ADDR_R);

	if (chip->options & NAND_BUSWIDTH_16) {
		uint16_t ftrs16[4];
		int i;
		chip->read_buf(mtd, (uint8_t*)ftrs16, sizeof(ftrs16));
		for (i=0; i<4; ++i)
			features[i] = ftrs16[i];
	} else
		chip->read_buf(mtd, features, 4);

	DEBUG(ONCHIP_MTD_DEBUG_LEVEL, "%s: faddr %02x [%02x %02x %02x %02x]\n",
		__func__, faddr,
		features[0], features[1], features[2], features[3]);
}

void nand_onchip_enable_ecc(struct mtd_info *mtd, int enable)
{
	uint8_t params[4];

	DEBUG(ONCHIP_MTD_DEBUG_LEVEL, "%s:%d enable %d\n",
		__func__, __LINE__, enable);

	memset(params, 0x00, sizeof(params));
	if (enable)
		params[0] = 0x08;

	micron_set_features(mtd, 0x90, params);
	micron_get_features(mtd, 0x90, params);

	DEBUG(ONCHIP_MTD_DEBUG_LEVEL, "%s: %02x %02x %02x %02x\n",
		__func__, params[0], params[1], params[2], params[3]);
}
EXPORT_SYMBOL(nand_onchip_enable_ecc);

int nand_onchip_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
			      u_char *ecc_code)
{
	DEBUG(ONCHIP_MTD_DEBUG_LEVEL, "%s:\n", __func__);
	return 0;
}
EXPORT_SYMBOL(nand_onchip_calculate_ecc);

int nand_onchip_correct_ecc(struct mtd_info *mtd, u_char *dat,
			    u_char *read_ecc, u_char *calc_ecc)
{
	struct nand_chip *chip = mtd->priv;

	DEBUG(ONCHIP_MTD_DEBUG_LEVEL, "%s: ecc_status %02x\n",
				__func__, chip->ecc.status);

	/* We stored the read status in chip->ecc.status in the read.  If bit 0
	 * is set, then there was an uncorrectable ECC error.  If bit 3 is set,
	 * then there was a correctable error (up to four bits of correction).
	 */
	if (chip->ecc.status & 0x01)
		return -1;
#if 0
	if (chip->ecc.status & 0x08)
		return 4;
#endif
	return 0;
}
EXPORT_SYMBOL(nand_onchip_correct_ecc);

/**
 * nand_chip_read_oob_hwecc - [REPLACABLE] OOB data read function for on chip HW
 *                         ECC.
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @page:	page number to read
 * @sndcmd:	flag whether to issue read command or not
 */
int nand_onchip_read_oob_ecc(struct mtd_info *mtd, struct nand_chip *chip,
			     int page, int sndcmd)
{
	DEBUG(ONCHIP_MTD_DEBUG_LEVEL, "%s: page = %d, len = %i\n",
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
	DEBUG(ONCHIP_MTD_DEBUG_LEVEL, "%s: ecc_status %02x\n",
				__func__, chip->ecc.status);
	if (chip->ecc.status& (0x8|0x1)) {
		DEBUG(ONCHIP_MTD_DEBUG_LEVEL, "%s:%d page %d ecc_status %02x\n",
					 __func__, __LINE__,
					page, chip->ecc.status);
		if (chip->ecc.status & 0x1)
		{
			mtd->ecc_stats.failed++;
			DEBUG(ONCHIP_MTD_DEBUG_LEVEL, "%s: ecc_stats.failed= %d\n",
				mtd->name, mtd->ecc_stats.failed);
		}
		else if (chip->ecc.status & 0x8)
		{
			mtd->ecc_stats.corrected += 4;
			DEBUG(ONCHIP_MTD_DEBUG_LEVEL, "%s: ecc_stats.corrected= %d\n",
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
EXPORT_SYMBOL(nand_onchip_read_oob_ecc);

void nand_onchip_hwctl(struct mtd_info *mtd, int mode)
{
	/* Nothing to do here since the on-chip ECC is only enabled once. */
	DEBUG(ONCHIP_MTD_DEBUG_LEVEL, "%s:\n", __func__);
}
EXPORT_SYMBOL(nand_onchip_hwctl);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jeff Cooper <jeff.cooper@logicpd.com>");
MODULE_DESCRIPTION("NAND on-chip ECC support");
