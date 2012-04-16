/*
 * Driver for ov7690 Image Sensor from OmniVision
 *
 * Copyright (C) 2012, Logic Produc Development

 * Based on  Driver for MT9P031 CMOS Image Sensor from Aptina
 * by Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-mediabus.h>

#include "ov7690.h"

#if 0
#define SENSDBG(fmt, ...) printk(KERN_INFO fmt, ## __VA_ARGS__)
#else
#define SENSDBG(fmt, ...)
#endif


#define DBG_ENTRY(x...) SENSDBG("%s:%d entry\n",__FUNCTION__, __LINE__)
#define DBG_ENTRYP(x) SENSDBG("%s:%d entry " #x "=%d \n" ,__FUNCTION__, __LINE__,x)

/*
 * Basic window sizes. 
 */
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define CIF_WIDTH	352
#define CIF_HEIGHT	288
#define QCIF_WIDTH	176
#define	QCIF_HEIGHT	144

/* Sensor pixel array is 492x656 */
#define OV7690_PIXEL_ARRAY_WIDTH 656
#define OV7690_PIXEL_ARRAY_HEIGHT 492

#define OV7690_WINDOW_MIN_WIDTH 40 
#define OV7690_WINDOW_MIN_HEIGHT 30
#define OV7690_WINDOW_MAX_WIDTH (VGA_WIDTH)
#define OV7690_WINDOW_MAX_HEIGHT (VGA_HEIGHT)
#define OV7690_VSTART_DEF 0xc
#define OV7690_HSTART_DEF 0x69
/*
 * The 7690 sits on i2c with ID 0x42
 */
#define OV7690_I2C_ADDR 0x42

/* Registers */
#define REG_GAIN	0x00	/* Gain lower 8 bits (rest in vref) */
#define REG_BGAIN	0x01	/* blue gain */
#define REG_RGAIN       0x02	/* red gain */
#define REG_GGAIN       0x03
#define REG_PID		0x0a	/* Product ID MSB */
#define REG_VER		0x0b	/* Product ID LSB */
#define REG_REG0C          0x0c    
#define   REG0C_VFLIP       0x80
#define   REG0C_MIRROR      0x40 
#define   REG0C_RB_SWAP     0x20  /* BR SWAP in RGB format */
#define   REG0C_UV_SWAP     0x10  /* YuYv swap in YUV mode */
#define   REG0C_DATAOUT_ENABLE 0x04  /* Data pins enable*/
#define   REG0C_CTLOUT_ENABLE 0x2 /* VSYNC, HREF, PCLK enable*/
#define   REG0C_COLOR_BAR 0x1 	/* Overlay color Bar */
#define REG_0D          0x0d    
#define REG_0E          0x0e    
#define REG_AECH	0xf	/* Automatic exposure control MSB */
#define REG_AECL	0x10	/* ... LSB */
#define REG_CLKRC	0x11	
#define   CLK_EXT	  0x40	  /* Use external clock directly */
#define   CLK_SCALE	  0x3f	  /* Mask for internal clock scale */
#define REG_REG12	0x12	
#define   REG12_RESET	  0x80	  /* Register reset */
#define REG_REG13	0x13	
#define REG_REG14	0x14	
#define REG_REG15	0x15	
#define REG_REG16	0x16	
#define REG_HSTART	0x17	
#define REG_HSIZE	0x18	
#define REG_VSTART	0x19	
#define REG_VSIZE	0x1a	
#define REG_SHFT	0x1b	
#define REG_MIDH	0x1c	/* Manuf. ID high */
#define REG_MIDL	0x1d	/* Manuf. ID low */ 
#define REG_REG1E	0x1e	
#define REG_REG1F	0x1f	
#define REG_REG20	0x20	
#define REG_AECGM	0x21	
#define REG_REG22	0x22	
#define REG_WPT		0x24	
#define REG_BPT		0x25	
#define REG_VPT		0x26	
#define REG_REG27	0x27	
#define REG_REG28	0x28	
#define REG_PLL		0x29	
#define REG_EXCHL	0x2a	
#define REG_EXCHH	0x2b	
#define REG_DM_LN	0x2c	
#define REG_ADVFL	0x2d	
#define REG_ADVFH	0x2e	
#define REG_StrobeADC	0x38	
//#define REG_REG39	0x39	
#define REG_REG3E	0x3e	
#define REG_REG3F	0x3f	
#define REG_ANA1	0x41	
#define REG_PWC0	0x49	
#define REG_BD50ST	0x50
#define REG_BD60ST	0x51
#define REG_UVCTR0	0x5a
#define REG_UVCTR1	0x5b
#define REG_UVCTR2	0x5c
#define REG_UVCTR3	0x5d
#define REG_REG62	0x62
#define REG_BLC8	0x68
#define REG_BLCOUT	0x6b
#define REG_6F		0x6f
#define REG_REG80	0x80	/* Enables */
#define REG_REG81	0x81	/* SDE: Special digital effects */
#define REG_REG82	0x82	
#define REG_LCC0	0x85	/* Lens control and correction*/
#define REG_LCC1	0x86	
#define REG_LCC2	0x87	
#define REG_LCC3	0x88	
#define REG_LCC4	0x89	
#define REG_LCC5	0x8a	
#define REG_LCC6	0x8b	
#define REG_AWB_BASE	0x8c 	/* AWB Control @0x8c:0xa2*/
#define REG_AWB(x)	((x)+REG_AWB_BASE)
#define REG_GAM_BASE	0xa2 	/* Gamma correction constants 1:15 */
#define REG_GAM(x)	((x)+REG_GAM_BASE)
#define REG_SLOPE	0xb2
#define REG_REGB4       0xb4
#define REG_REGB5       0xb5
#define REG_REGB6	0xb6
#define REG_REGB7       0xb7
#define REG_REGB8       0xb8
#define REG_REGB9       0xb9
#define REG_REGBA       0xba
#define REG_REGBB       0xbb
#define REG_REGBC       0xbc
#define REG_REGBD       0xbd
#define REG_REGBE       0xbe
#define REG_REGBF       0xbf
#define REG_REGC0	0xc0
#define REG_REGC1	0xc1
#define   CMATRIX_BASE  (REG_REGBB)
#define   CMATRIX_LEN	(1+REG_REGC1-CMATRIX_BASE)
#define REG_REGC2	0xc2
#define REG_REGC3	0xc3
#define REG_REGC4	0xc4
#define REG_REGC5	0xc5
#define REG_REGC6	0xc6
#define REG_REGC7	0xc7
#define REG_REGC8	0xc8
#define REG_REGC9	0xc9
#define REG_REGCA	0xca
#define REG_REGCB	0xcb
#define REG_REGCC	0xcc
#define REG_REGCD	0xcd
#define REG_REGCE	0xce
#define REG_REGCF	0xcf
#define REG_REGD0	0xd0
#define REG_REGD1	0xd1
#define REG_SDECTRL	0xd2	// Side control for sde
#define   SDECTRL_HUE_EN 0x1	// 0x1 - hue
#define   SDECTRL_SAT_EN 0x2	// 0x2 - saturation
#define   SDECTRL_CONT_EN 0x4	// 0x4 - contrast & brightness
#define   SDECTRL_FIX_UV 0x18   // fix U and V		
#define REG_REGD3	0xd3
#define REG_REGD4	0xd4
#define REG_REGD5 	0xd5
#define REG_REGD6 	0xd6	// Hue cos
#define REG_REGD7 	0xd7	// Hue sin
#define REG_REGD8 	0xd8
#define REG_REGD9 	0xd9
#define REG_REGDA 	0xda	//Ureg
#define REG_REGDB 	0xdb	//Vreg
#define REG_REGDC 	0xdc	// Signs for SDE controls
				// 0x33 hue:
				//   00xx01 - 0 <= theta < pi/2
				//   00xx10 - -pi/2 <= theta < 0
				//   11xx01 - pi/2 <= theta < pi
				//   11xx10 - -pi <= theta < -pi/2
				// 0x4 Contrast 
				// 0x8 Brightness
#define REG_REGDD 	0xdd
#define REG_REGDE 	0xde
#define REG_REGDF 	0xdf
#define REG_REGE0 	0xe0
#define REG_REGE1 	0xe1
/* Non-existent register number to terminate value lists */
#define REG_DUMMY       0xff
/* Forward declaration */
struct ov7690_format_struct; 

/* Structure defining element of register values array */
struct regval_list {
	unsigned char reg_num;
	unsigned char value;
	unsigned char mask;
};
#define REGVAL_LIST_END { REG_DUMMY, REG_DUMMY,}
/* Structure describing frame interval */
struct ov7690_interval 
{
	const struct regval_list *regs;
	struct v4l2_fract interval;
};

/*
 * Information we maintain about a known sensor.
 */

struct ov7690_info {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct ov7690_format_struct *fmt;  /* Descriptor of current format */
	struct v4l2_rect curr_crop;
	struct v4l2_ctrl_handler ctrls;
	const struct ov7690_interval *curr_fi;
	struct ov7690_platform_data *pdata;
	struct mutex power_lock;
	int power_count;
/* Brightness and contrast controls affect each other
 * To keep track driver holds on to a handle of brightness control
 * Editing values directly is a legitimate technique by 
 * Documentation/video4linux/v4l2-controls.txt
 */
	int setup; /* Flag to indicate that v4l2-ctl is replaying setup into driver on powerup vs runtime adjustments */
	int brightness; /* Cached value of actual brightness */
	struct	v4l2_ctrl *bright_ctrl;
	int streaming; /* flag to indicate if video stream is active */
};

static inline struct ov7690_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov7690_info, sd);
}

/*
 * The default register settings, as obtained from OmniVision.  There
 * is really no making sense of most of these - lots of "reserved" values
 * and such.
 *
 * These settings give VGA YUYV.
 */


static struct regval_list ov7690_default_regs[] = {
	{REG_REG12, REG12_RESET },
	{REG_PWC0, 0x0c} , // Power is at 1.8 v
/*
** Apparently soft reset reg_0x12.bit7 = 1 enables 
** both pll and rata/control outputs 
** It creates deadly consequences for omap isp.
** During init soft reset must be issued
** We try to follow reset with disabling outputs as closely in time
** as possible
*/
	{REG_REG0C,  REG0C_UV_SWAP |
	 (0 &  (REG0C_DATAOUT_ENABLE | REG0C_CTLOUT_ENABLE)) |
	 0},
	{0x48, 0x42 }, // Omnivision magic
	{REG_ANA1, 0x43 }, // More magic
	{0x4c, 0x73},
	{REG_REG81, 0xFF}, // Enable SDE functions
	{REG_AECGM, 0x44},
	{REG_REG16, 0x03},
	{0x39, 0x80},
	{REG_REG1E, 0xb1},
	/* Format */
	// YUV
	{REG_REG12, 0x00},
	{REG_REG82, 0x03}, // YUV422
	{REG_REGD0, 0x48}, // Boundary offset
	{REG_REG80, 0x7f}, 
	{REG_REG3E, 0x30}, // Double pclk for YUV format

	{REG_REG22, 0x00},

	/* Resolution */
	{REG_HSTART, 0x69},
	{REG_HSIZE, 0xa4},
	{REG_VSTART, 0x0c},
	{REG_VSIZE, 0xf6},

	{REG_REGC8, (VGA_WIDTH>>8)&3},  
	{REG_REGC9, VGA_WIDTH & 0xff}, //;ISP input hsize (640)
	{REG_REGCA, (VGA_HEIGHT >>8)&1},  
	{REG_REGCB, VGA_HEIGHT&0xff }, //;ISP input vsize (480)

	{REG_REGCC, (VGA_WIDTH>>8)&3},
	{REG_REGCD, VGA_WIDTH & 0xff}, //;ISP output hsize (640)
	{REG_REGCE, (VGA_HEIGHT >>8)&1}, 
	{REG_REGCF, VGA_HEIGHT&0xff }, //;ISP output vsize (480)

	/* Lens Correction */
	{REG_LCC0, 0x90},
	{REG_LCC1, 0x00},
	{REG_LCC2, 0x00},
	{REG_LCC3, 0x10},
	{REG_LCC4, 0x30},
	{REG_LCC5, 0x29},
	{REG_LCC6, 0x26},

	/* Color Matrix */
	{REG_REGBB, 0x80},
	{REG_REGBC, 0x62},
	{REG_REGBD, 0x1e},
	{REG_REGBE, 0x26},
	{REG_REGBF, 0x7b},
	{REG_REGC0, 0xac},
	{REG_REGC1, 0x1e},

	/* Edge + Denoise */
	{REG_REGB7, 0x05},
	{REG_REGB8, 0x09},
	{REG_REGB9, 0x00},
	{REG_REGBA, 0x18},

	/* UVAdjust */
	{REG_UVCTR0, 0x4A},
	{REG_UVCTR1, 0x9F},
	{REG_UVCTR2, 0x48},
	{REG_UVCTR3, 0x32},

	/* AEC/AGC target */
	{REG_WPT, 0x78},
	{REG_BPT, 0x68},
	{REG_VPT, 0xb3},

	/* Gamma */
	{REG_GAM(1), 0x0b},
	{REG_GAM(2), 0x15},
	{REG_GAM(3), 0x2a},
	{REG_GAM(4), 0x51},
	{REG_GAM(5), 0x63},
	{REG_GAM(6), 0x74},
	{REG_GAM(7), 0x83},
	{REG_GAM(8), 0x91},
	{REG_GAM(9), 0x9e},
	{REG_GAM(10), 0xaa},
	{REG_GAM(11), 0xbe},
	{REG_GAM(12), 0xce},
	{REG_GAM(13), 0xe5},
	{REG_GAM(14), 0xf3},
	{REG_GAM(15), 0xfb},
	{REG_SLOPE, 0x06},


	/* AWB */
	/* Simple */
//;42 8e 92 ; simple AWB
//;42 96 ff
//;42 97 00 ;unlimit AWB range.

	/* Advanced */
	{REG_AWB(0), 0x5d},
	{REG_AWB(1), 0x11},
	{REG_AWB(2), 0x12},
	{REG_AWB(3), 0x11},
	{REG_AWB(4), 0x50},
	{REG_AWB(5), 0x22},
	{REG_AWB(6), 0xd1},
	{REG_AWB(7), 0xa7},
	{REG_AWB(8), 0x23},
	{REG_AWB(9), 0x3b},
	{REG_AWB(10), 0xff},
	{REG_AWB(11), 0x00},
	{REG_AWB(12), 0x4a},
	{REG_AWB(13), 0x46},
	{REG_AWB(14), 0x3d},
	{REG_AWB(15), 0x3a},
	{REG_AWB(16), 0xf0},
	{REG_AWB(17), 0xf0},
	{REG_AWB(18), 0xf0},
	{REG_AWB(19), 0xff},
	{REG_AWB(20), 0x56},
	{REG_AWB(21), 0x55},
	{REG_AWB(22), 0x13},

	/* General Control */
	{REG_BD50ST, 0x9a},
	{REG_BD60ST, 0x80},
	{REG_AECGM, 0x23},

	{REG_REG14, 0x28},
	{REG_REG13, 0xf7},
/*
*	rate = clock(divider+1)
*	0 - 30 fps
*	1 - 15 fps
* 	2 - 10 fps
*	3 - 7.5 fps
*/
	{REG_CLKRC, 0x00},

	{REG_0E, 0x01,0x3},	// set drive strength to x2

	REGVAL_LIST_END	/* END MARKER */
};


/*
 * Output video format register settings
 * RGB656 and YUV422 come from OV
 */
static const struct regval_list ov7690_fmt_yuv422[] = {
	{REG_REG12, 0x00, 0x3f},
	{REG_REG82, 0x03, 0x03},
	{REG_REG3E, 0x10, 0x10},
	REGVAL_LIST_END
};

static const struct regval_list ov7690_fmt_rgb565[] = {
	{REG_REG12, 0x06, 0x3f},
	{REG_REG82, 0x03, 0x03},
	{REG_REG3E, 0x10, 0x10},
	REGVAL_LIST_END
};

/* 
 * Frame interval register settings 
 * values come from OmniVision.
 * Frame rates are more then just pll, AEC related values change as well
 */
static const struct regval_list ov7690_15fps[] = {
	{REG_BD50ST, 0x4c},
	{REG_BD60ST, 0x3f},
	{REG_AECGM, 0x57 },
	{REG_REG20, 0x0},
	{REG_CLKRC, 0x1, CLK_SCALE},
	REGVAL_LIST_END
};

static const struct regval_list ov7690_30fps[] = {
	{REG_BD50ST, 0x9a},
	{REG_BD60ST, 0x80},
	{REG_AECGM, 0x23 },
	{REG_REG20, 0x0},
	{REG_CLKRC, 0x0, CLK_SCALE},
	REGVAL_LIST_END
};

static const struct ov7690_interval ov7690_intervals[] = {
	{ov7690_30fps,{33300,1000000}}, /* 30fps */
	{ov7690_15fps,{15000,1000000}}, /* 15fps */
};

/*
 * Video format definitions
 */
static struct ov7690_format_struct {
	struct v4l2_mbus_framefmt format;
	const struct regval_list *regs;
} ov7690_formats[] = {
	{       
	  .format = { 
	    .width= VGA_WIDTH,
	    .height = VGA_HEIGHT,
	    .code	= V4L2_MBUS_FMT_YUYV8_2X8, //?1X8
	    .colorspace	= V4L2_COLORSPACE_JPEG,
	    .field =  V4L2_FIELD_NONE,
	  },
	  .regs 		= ov7690_fmt_yuv422,
	},
	{
	  .format = { 
	    .width= VGA_WIDTH,
	    .height = VGA_HEIGHT,
	    .code	=V4L2_MBUS_FMT_RGB565_2X8_BE,
	    .colorspace	= V4L2_COLORSPACE_SRGB,
	    .field =  V4L2_FIELD_NONE,
	  },
	  .regs		= ov7690_fmt_rgb565,
	},

};
#define N_OV7690_FMTS ARRAY_SIZE(ov7690_formats)

/*
 * Low-level register I/O.
 *
 * Note that there are two versions of these.  On the XO 1, the
 * i2c controller only does SMBUS, so that's what we use.  The
 * ov7690 is not really an SMBUS device, though, so the communication
 * is not always entirely reliable.
 */
static int ov7690_read(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret >= 0) {
		*value = (unsigned char)ret;
		ret = 0;
	}
	return ret;
}

static int ov7690_write(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = i2c_smbus_write_byte_data(client, reg, value);

	if ( reg == REG_REG12 && (value & REG12_RESET)) msleep(5);
	return ret;
}

static int ov7690_write_mask(struct v4l2_subdev *sd, unsigned char reg, unsigned char value, unsigned char mask)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) return ret;
	ret = (unsigned char) ret & (~mask);
	ret |= value & mask;

	return i2c_smbus_write_byte_data(client, reg, (unsigned char) ret);
}


/*
 * Write a list of register settings; ff/ff ends the list
 */
static int ov7690_write_array(struct v4l2_subdev *sd, const struct regval_list *vals)
{
	while (vals->reg_num != REG_DUMMY || vals->value != REG_DUMMY) {
		int ret;
		if (vals->mask)
			ret=ov7690_write_mask(sd, vals->reg_num, vals->value, vals->mask);
		else
			ret = ov7690_write(sd, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
	}
	return 0;
}

/*
 * Stuff that knows about the sensor.
 */
static int ov7690_init(struct v4l2_subdev *sd)
{
	return ov7690_write_array(sd, ov7690_default_regs);
}

static int ov7690_detect(struct v4l2_subdev *sd)
{
	unsigned char v;
	int ret;

	ret = ov7690_read(sd, REG_MIDH, &v);
	if (ret < 0)
		return ret;
	if (v != 0x7f) /* OV manuf. id. */
		return -ENODEV;
	ret = ov7690_read(sd, REG_MIDL, &v);
	if (ret < 0)
		return ret;
	if (v != 0xa2)
		return -ENODEV;
	/*
	 * OK, we know we have an OmniVision chip...but which one?
	 */
	ret = ov7690_read(sd, REG_PID, &v);
	if (ret < 0)
		return ret;
	if (v != 0x76)  /* PID + VER = 0x76 / 0x91 */
		return -ENODEV;
	ret = ov7690_read(sd, REG_VER, &v);
	if (ret < 0)
		return ret;
	if (v != 0x91)  /* PID + VER = 0x76 / 0x91 */
		return -ENODEV;
#if defined(notdef_LowLevelDebug)
	{ // Dump sensor registers - note: doig it at this point may be destructive for future operation
		int i;
		unsigned char v;
		unsigned char outbuf[128], *pbuf;
		
		printk(KERN_INFO "          00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F <-ov7690_regs\n");
		
		for(pbuf=outbuf,i=0; i < 0xe2;i++) {
			if (0 == i%16) pbuf = outbuf+sprintf(outbuf,"%08X:",i);
			ov7690_read(sd,i,&v);
			pbuf += sprintf(pbuf," %02X",v);
			if (15 == i%16) {
				printk(KERN_INFO "%s\n",outbuf); 
				pbuf = outbuf;
			}
		}
		if (pbuf != outbuf)  printk(KERN_INFO "%s\n",outbuf); 
	}
#endif
	/* It is right sensor, do things, which need to be done ASAP.
        ** Sensor comes out of both hard and soft reset with
	** enabled outputs and pll. 
	** To avoid causing grief to the host, disable outputs immediately.
	** Normal operation controls those at s_power and s_stream time as needed.
	*/
	ov7690_write_mask(sd, REG_REG0C, 0, ( REG0C_DATAOUT_ENABLE |  REG0C_CTLOUT_ENABLE));
	ov7690_write_mask(sd, REG_PLL, 0x8,0x8);
	return 0;
}

static int ov7690_set_scaling( struct v4l2_subdev *sd)
{
	struct ov7690_info *info = to_state(sd);
	int ret;
	ret =  ov7690_write(sd,REG_REGC8,(info->curr_crop.width >> 8) &3);
	if (ret >= 0) ret = ov7690_write(sd,REG_REGC9,info->curr_crop.width & 0xff);
	if (ret >= 0) ret = ov7690_write(sd,REG_REGCA,(info->curr_crop.height >> 8) & 3);
	if (ret >= 0) ret = ov7690_write(sd,REG_REGCB,info->curr_crop.height & 0xff);
	if (ret >= 0) ret = ov7690_write(sd,REG_REGCC,(info->fmt->format.width >> 8) & 3);
	if (ret >= 0) ret = ov7690_write(sd,REG_REGCD,info->fmt->format.width & 0xff);
	if (ret >= 0) ret = ov7690_write(sd,REG_REGCE,(info->fmt->format.height >> 8) & 3);
	if (ret >= 0) ret = ov7690_write(sd,REG_REGCF,info->fmt->format.height & 0xff);
	return ret;
}


/*
 * Code for dealing with controls.
 */

/* Low-level interface to clock/power */
static int ov7690_power_on(struct ov7690_info *info)
{
  if (info->pdata->s_xclk)
    info->pdata->s_xclk(&info->sd,1);
  msleep(1);
  return 0;
}

static int ov7690_power_off(struct ov7690_info *info)
{
	if (info->pdata->s_xclk)
		info->pdata->s_xclk(&info->sd,0);
	return 0;
}
static int __ov7690_set_power(struct ov7690_info *info, bool on)
{
	int ret;
	
	if (!on) {
		ov7690_power_off(info);
		return 0;
	}
	ret = ov7690_power_on(info);
	if (ret < 0)
		return ret;

	return ret;
}
/* -----------------------------------------------------------------------------
 * V4L2 subdev video operations
 */

static int ov7690_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov7690_info *info = to_state(sd);
	int ret;

	ret = 0;
	if (info->power_count == 0){
		return -EINVAL;
	}
	if (0 == enable) {
		info->streaming = 0;
		ret = ov7690_write_mask(sd,REG_REG0C,0 ,  (REG0C_DATAOUT_ENABLE  |  REG0C_CTLOUT_ENABLE ));
	} else {
		ret = ov7690_init(sd);
		if (ret >= 0) {
			/* set format, scaling and frame rate*/
			ret = ov7690_write_array(sd, info->fmt->regs);
			ov7690_set_scaling(sd);
			ret = ov7690_write_array(sd, info->curr_fi->regs);
		}
                info->setup = 1;
                ret=v4l2_ctrl_handler_setup(&info->ctrls);
                info->setup = 0;
		info->streaming = 1;
		ret = ov7690_write_mask(sd,REG_REG0C,
				      (REG0C_DATAOUT_ENABLE |  REG0C_CTLOUT_ENABLE) ,  
				      (REG0C_DATAOUT_ENABLE |  REG0C_CTLOUT_ENABLE));
	}
	return ret;
}

static int ov7690_g_frame_interval(struct v4l2_subdev *sd,
                                   struct v4l2_subdev_frame_interval *fi)
{
       
	struct ov7690_info *info = to_state(sd);
       
        fi->interval = info->curr_fi->interval;

        return 0;
}

static int ov7690_s_frame_interval(struct v4l2_subdev *sd,
                                   struct v4l2_subdev_frame_interval *fi)
{
     	struct ov7690_info *info = to_state(sd);
	int i, fr_time;
	unsigned int err, min_err = UINT_MAX;
	const struct ov7690_interval *fiv = &ov7690_intervals[0];
	
	if (fi->interval.denominator == 0)
                return -EINVAL;

	fr_time = fi->interval.numerator * 10000 / fi->interval.denominator;
        for (i = 0; i < ARRAY_SIZE(ov7690_intervals); i++) {
		const struct ov7690_interval *iv = &ov7690_intervals[i];
		err = abs((iv->interval.numerator * 10000/ iv->interval.denominator) - fr_time);
                if (err < min_err) {
                        fiv = iv;
                        min_err = err;
		}
	}
	
	info->curr_fi = fiv;

        return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev pad operations
 */

static int ov7690_enum_mbus_code(struct v4l2_subdev *sd,
                                  struct v4l2_subdev_fh *fh,
                                  struct v4l2_subdev_mbus_code_enum *code)
{
  
  if ((code->pad) || (code->index > N_OV7690_FMTS ))
      return -EINVAL;
  
  code->code = ov7690_formats[code->index].format.code;
  return 0;

}
static int ov7690_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	int i =  N_OV7690_FMTS;
	if (fse->index > 0) return -EINVAL;
	
	while(--i)
		if (fse->code == ov7690_formats[i].format.code)
			break;
	fse->code = ov7690_formats[i].format.code;

	fse->min_width = OV7690_WINDOW_MIN_WIDTH ;
	fse->max_width = OV7690_WINDOW_MAX_WIDTH ;
	fse->min_height = OV7690_WINDOW_MIN_HEIGHT; 
	fse->max_height = OV7690_WINDOW_MAX_HEIGHT;

  return 0;
}

static struct v4l2_mbus_framefmt *
__ov7690_get_pad_format(struct ov7690_info *info, struct v4l2_subdev_fh *fh,
                         unsigned int pad, u32 which)
{
        switch (which) {
        case V4L2_SUBDEV_FORMAT_TRY:
                return v4l2_subdev_get_try_format(fh, pad);
        case V4L2_SUBDEV_FORMAT_ACTIVE:
                return &info->fmt->format;
        default:
                return NULL;
        }
}

static struct v4l2_rect *
__ov7690_get_pad_crop(struct ov7690_info *info, struct v4l2_subdev_fh *fh,
		      unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &info->curr_crop;
	default:
		return NULL;
	}
}

static int ov7690_get_format(struct v4l2_subdev *sd,
                              struct v4l2_subdev_fh *fh,
                              struct v4l2_subdev_format *fmt)
{
	struct ov7690_info *info = to_state(sd);
	struct v4l2_mbus_framefmt *__format;

	__format = __ov7690_get_pad_format(info, fh, fmt->pad,
					   fmt->which);
	if (__format == NULL) return -EINVAL;
	fmt->format = *__format;
	return 0;
}
 
static int ov7690_set_format(struct v4l2_subdev *sd,
                              struct v4l2_subdev_fh *fh,
                              struct v4l2_subdev_format *format)
{  
	struct ov7690_info *info = to_state(sd);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	unsigned int width;
	unsigned int height;
	int i =  ARRAY_SIZE(ov7690_formats);

	if ((format->which == V4L2_SUBDEV_FORMAT_ACTIVE) &&
	    (info->streaming))
		return -EBUSY;

	/* match format against array of supported formats */
	while (--i)
		if (ov7690_formats[i].format.code == format->format.code) break;
	info->fmt =  &ov7690_formats[i];

	__crop = __ov7690_get_pad_crop(info, fh, format->pad, format->which);
	/* ov7690 can only scale size down, not up */
	width = clamp_t(unsigned int, ALIGN(format->format.width, 2),
                        OV7690_WINDOW_MIN_WIDTH, __crop->width);
	height = clamp_t(unsigned int, ALIGN(format->format.height, 2),
                        OV7690_WINDOW_MIN_HEIGHT, __crop->height);

	__format = __ov7690_get_pad_format(info, fh, format->pad,
					   format->which);
	if (__format == NULL) return -EINVAL;
	__format->width = width;
	__format->height = height;
	format->format = *__format;
	return 0;
}

static int ov7690_get_crop(struct v4l2_subdev *sd,
			   struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_crop *crop)
{
	struct ov7690_info *info = to_state(sd);
	struct v4l2_rect *rect;
	
	rect = __ov7690_get_pad_crop(info, fh, crop->pad,
				     crop->which);
	if (!rect)
		return -EINVAL;
	crop->rect = *rect;
	
	return 0;
}
static int ov7690_set_crop(struct v4l2_subdev *sd,
			   struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_crop *crop)
{
	struct ov7690_info *info = to_state(sd);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	struct v4l2_rect rect;
	/* It is unclear how to control left corner of crop rectangle */
	rect.left =0;
	rect.top = 0;
	rect.width = clamp(ALIGN(crop->rect.width, 2),
                           OV7690_WINDOW_MIN_WIDTH,
                           OV7690_WINDOW_MAX_WIDTH);
        rect.height = clamp(ALIGN(crop->rect.height, 2),
                            OV7690_WINDOW_MIN_HEIGHT,
                            OV7690_WINDOW_MAX_HEIGHT);
	__crop = __ov7690_get_pad_crop(info, fh, crop->pad, crop->which);

	if (rect.width != __crop->width || rect.height != __crop->height) {
		/* Reset the output image size if the crop rectangle size has
                 * been modified.
                 */
		__format = __ov7690_get_pad_format(info, fh, crop->pad,
						   crop->which);
                __format->width = rect.width;
                __format->height = rect.height;
        }

        *__crop = rect;
        crop->rect = rect;

	return 0;
}

static int ov7690_enum_frame_interval(struct v4l2_subdev *sd,
                              struct v4l2_subdev_fh *fh,
                              struct v4l2_subdev_frame_interval_enum *fie)
{
	
        if (fie->index > ARRAY_SIZE(ov7690_intervals))
                return -EINVAL;
	
        v4l_bound_align_image(&fie->width, OV7690_WINDOW_MIN_WIDTH,
                              OV7690_WINDOW_MAX_WIDTH, 1,
                              &fie->height, OV7690_WINDOW_MIN_HEIGHT,
                              OV7690_WINDOW_MAX_HEIGHT, 1, 0);

        fie->interval = ov7690_intervals[fie->index].interval;

        return 0;
}
  
/* -----------------------------------------------------------------------------
 * V4L2 subdev control operations
 */
/*
 * Hue also requires messing with the color matrix.  It also requires
 * trig functions, which tend not to be well supported in the kernel.
 * So here is a simple table of sine values, 0-90 degrees, in steps
 * of five degrees.  Values are multiplied by 1000.
 *
 * The following naive approximate trig functions require an argument
 * carefully limited to -180 <= theta <= 180.
 */
#define SIN_STEP 5
static const int ov7690_sin_table[] = {
	   0,	 87,   173,   258,   342,   422,
	 500,	573,   642,   707,   766,   819,
	 866,	906,   939,   965,   984,   996,
	1000
};

static int ov7690_sine(int theta)
{
	int chs = 1;
	int sine;

	if (theta < 0) {
		theta = -theta;
		chs = -1;
	}
	if (theta <= 90)
		sine = ov7690_sin_table[theta/SIN_STEP];
	else {
		theta = 180 - theta;
		sine = ov7690_sin_table[theta/SIN_STEP];
	}
	return sine*chs;
}

static int ov7690_cosine(int theta)
{
	theta = 90 - theta;
	if (theta > 180)
		theta -= 360;
	else if (theta < -180)
		theta += 360;
	return ov7690_sine(theta);
}

static int ov7690_s_hue(struct v4l2_subdev *sd, int value)
{
#define HUE_MIN -180
#define HUE_MAX 180
#define HUE_STEP 5
	int ret;
	int sinth, costh;
	unsigned char sign_hue;
	if (value < -180 || value > 180)
		return -ERANGE;
	// 1.7 fixpoint format for sin and cos
	sinth = ov7690_sine(value);
	costh = ov7690_cosine(value);
	if (sinth < 0) sinth = -sinth;
	if (costh < 0) costh = -costh;
	sinth = ((sinth << 4)+124) / 125;	// x = x*128/1000;
	costh = ((costh << 4)+124) / 125;	// x = x*128/1000;

	/* Sight bits go into bits 0:1:4:5 of reg 0xDC
	 * 0.1.4.5 
	 * 1.0.0.0 := 0 <= theta < pi/2
	 * 0.1.0.0 := -pi/2 <= theta < 0
	 * 1.0.1.1 := pi/2 < theta
	 * 0.1.1.1 := theta < -pi/2
	 */
   	if (value >= 0 ) {
		if (value < 90) sign_hue = 0x01;
		else sign_hue = 0x31;
	} else {
		if (value < -90) sign_hue = 0x32;
		else sign_hue = 0x02;
	}

	// Write values
	// enable SDE
	ret = ov7690_write_mask(sd, REG_REG81, 0x20, 0x20);
	// enable Hue side-control
	if (ret >= 0) ret = ov7690_write_mask(sd,REG_SDECTRL,SDECTRL_HUE_EN,SDECTRL_HUE_EN);
	// Write values and signs
	if (ret >= 0) ret = ov7690_write(sd,REG_REGD6, costh);	
	if (ret >= 0) ret = ov7690_write(sd,REG_REGD7, sinth);	
	if (ret >= 0) ret = ov7690_write_mask(sd,REG_REGDC,sign_hue,0x33);

	return ret;
}

static int ov7690_s_brightness(struct v4l2_subdev *sd, int value)
{
#define BRIGHTNESS_MIN -255
#define BRIGHTNESS_MAX 255
	int ret;
	unsigned char v, sign_bright;
	struct ov7690_info *info = to_state(sd);

	if (value < -255 || value > 255)
		return -EINVAL;
	// enable SDE
	ret = ov7690_write_mask(sd, REG_REG81, 0x20, 0x20);
	
	if (value < 0) {
		sign_bright = 0x8;
		v = (unsigned char) ((-value));
	} else {
		sign_bright = 0;
		v = (unsigned char) (value);
	}
	// Write value, side-control and sign
	if (ret >= 0) ret = ov7690_write(sd, REG_REGD3, v);
	if (0 == info->setup) 
		info->brightness = value;

	if (ret >= 0) ret = ov7690_write_mask(sd, REG_SDECTRL, SDECTRL_CONT_EN, SDECTRL_CONT_EN);
	if (ret >= 0) ret = ov7690_write_mask(sd, REG_REGDC, sign_bright, 0x8);

	return ret;
}

static int ov7690_s_contrast(struct v4l2_subdev *sd, int value)
{
#define CONTRAST_MIN -4
#define CONTRAST_MAX 4
	// For some reason setting contrast sets brightness as well
	// If brightnesss is undesirable, set it after setting contrast
	int ret;
	unsigned char v, sign_cont, imply_brightness;
	struct ov7690_info *info = to_state(sd);

	static unsigned char brightness_list[] =
		{ 0xd0, 0x80, 0x48, 0x20 };
	if (value < CONTRAST_MIN || value > CONTRAST_MAX)
		return -ERANGE;

	// enable SDE
	ret = ov7690_write_mask(sd, REG_REG81, 0x20, 0x20);
	if (value < 0) {
		sign_cont = 0x4;
		imply_brightness = 
			brightness_list[ARRAY_SIZE(brightness_list) + value];
	} else {
		sign_cont = 0x0;
		imply_brightness = 0;
	}
	v = 0x20 + (value*4);
	if (ret >= 0) ret = ov7690_write(sd, REG_REGD5, 0x20);
	if (ret >= 0) ret = ov7690_write(sd, REG_REGD4, v);
	if (ret >= 0) {
		if (0 == info->setup) {
			ret = ov7690_write_mask(sd, REG_REGDC, 0, 0x8);
			ret = ov7690_write(sd, REG_REGD3, imply_brightness);
			info->bright_ctrl->cur.val=imply_brightness;
			info->brightness = imply_brightness;
		} else {
			// brightness setup will take care of setting 
		}
	} 
	if (ret >= 0) ret = ov7690_write_mask(sd, REG_REGDC, sign_cont, 0x4);
	if (ret >= 0) ret = ov7690_write_mask(sd, REG_SDECTRL, SDECTRL_CONT_EN, SDECTRL_CONT_EN);

	return ret;
}


static int ov7690_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret;

	unsigned char v = (value) ? REG0C_MIRROR : 0;
	ret = ov7690_write_mask(sd,REG_REG0C, v, REG0C_MIRROR);
	return ret;
}

static int ov7690_s_vflip(struct v4l2_subdev *sd, int value)
{
	int ret;

	unsigned char v = (value) ? REG0C_VFLIP : 0;
	ret = ov7690_write_mask(sd,REG_REG0C, v, REG0C_VFLIP);
	return ret;
}
static int ov7690_s_sharpness(struct v4l2_subdev *sd, int value)
{
#define SHARPNESS_MIN -1
#define SHARPNESS_MAX 5
	int ret;
	if (-1 == value) { /* Sharpness off */
		ret = ov7690_write_mask(sd, REG_REGB4, 0x20,0x20);
		if (ret >= 0) ret = ov7690_write_mask(sd, REG_REGB6, 0,0x1f);
	} else if (0 == value) { /* Sharpness Auto */
		ret = ov7690_write_mask(sd, REG_REGB4, 0,0x20);
		if (ret >= 0) ret = ov7690_write_mask(sd, REG_REGB6, 2,0x1f);
		if (ret >= 0) ret = ov7690_write(sd, REG_REGB8, 9);
	} else {
		static unsigned char sharp_vals[SHARPNESS_MAX] =
			{ 1,2,3,5,8 };
		if (value > ARRAY_SIZE(sharp_vals)) return -ERANGE;
		ret = ov7690_write_mask(sd, REG_REGB4, 0x20,0x20);
		if (ret >= 0) ret = ov7690_write_mask(sd, REG_REGB6, sharp_vals[value-1],0x1f);
	}
	return ret;
}
static int ov7690_s_saturation(struct v4l2_subdev *sd, int value)
{
#define SATURATION_MIN -1
#define SATURATION_MAX 8
	int ret;
	if ((value < SATURATION_MIN) || (value > SATURATION_MAX)) return -ERANGE;
	if (-1 == value) {
		ret = ov7690_write_mask(sd, REG_SDECTRL, 0,SDECTRL_SAT_EN);
	} else {
		ret =  ov7690_write_mask(sd, REG_SDECTRL,SDECTRL_SAT_EN ,SDECTRL_SAT_EN);
		if (ret >= 0) {
			ret= ov7690_write(sd,REG_REGD8,value<<4);
			ret |= ov7690_write(sd,REG_REGD9,value<<4);
		}
	}
	return ret;
}
static int ov7690_s_exposure(struct v4l2_subdev *sd, int value)
{
#define EXPOSURE_MIN -5
#define EXPOSURE_MAX 5
        struct {
                unsigned char WPT; /* reg 0x24 */
                unsigned char BPT; /* reg 0x25 */
                unsigned char VPT; /* reg 0x26 */
        } exposure_avg[1 + EXPOSURE_MAX - EXPOSURE_MIN] = {
		/* -1.7EV */ { 0x50 , 0x40 , 0x63 }, 
		/* -1.3EV */ { 0x58 , 0x48 , 0x73 }, 
		/* -1.0EV */ { 0x60 , 0x50 , 0x83 },
		/* -0.7EV */ { 0x68 , 0x58 , 0x93 },
                /* -0.3EV */ { 0x70 , 0x60 , 0xa3 },
                /* default */ { 0x78 , 0x68 , 0xb3 },
                /* 0.3EV */ { 0x80 , 0x70 , 0xc3 },
                /* 0.7EV */ { 0x88 , 0x78 , 0xd3 },
                /* 1.0EV */ { 0x90 , 0x80 , 0xd3 },
		/* 1.3EV */ { 0x98 , 0x88 , 0xe3 }, 
		/* 1.7EV */ { 0xa0 , 0x90 , 0xe3 }, 
        };
	int ret;
	int	index = value - EXPOSURE_MIN;
	if ((index < 0) || (index > ARRAY_SIZE(exposure_avg) )) return -ERANGE;
	ret=ov7690_write(sd,REG_WPT,exposure_avg[index].WPT);
	ret=ov7690_write(sd,REG_BPT,exposure_avg[index].BPT);
	ret=ov7690_write(sd,REG_VPT,exposure_avg[index].VPT);
	return ret;
}

static int ov7690_s_colorfx(struct v4l2_subdev *sd, int value)
{
	int ret = -EINVAL;
	switch(value) {
	case V4L2_COLORFX_NONE:
		ret = ov7690_write_mask(sd,REG_REG28, 0, 0x80);
		ret |= ov7690_write_mask(sd, REG_SDECTRL, 0,SDECTRL_FIX_UV);
		break;
	case V4L2_COLORFX_BW:
		ret = ov7690_write_mask(sd,REG_REG28, 0, 0x80);
		ret |= ov7690_write_mask(sd, REG_SDECTRL, SDECTRL_FIX_UV,SDECTRL_FIX_UV);
		ret |= ov7690_write(sd,REG_REGDA, 0x80);
		ret |= ov7690_write(sd,REG_REGDB, 0x80);
		break;
	case V4L2_COLORFX_SEPIA:
		ret = ov7690_write_mask(sd,REG_REG28, 0, 0x80);
		ret |= ov7690_write_mask(sd, REG_SDECTRL, SDECTRL_FIX_UV,SDECTRL_FIX_UV);
		ret |= ov7690_write(sd,REG_REGDA, 0x40);
		ret |= ov7690_write(sd,REG_REGDB, 0xa0);
		break;
	case V4L2_COLORFX_NEGATIVE :
		ret = ov7690_write_mask(sd,REG_REG28, 0x80, 0x80);
		ret |= ov7690_write_mask(sd, REG_SDECTRL, 0,SDECTRL_FIX_UV);
		break;
	case V4L2_COLORFX_SKY_BLUE:
		ret = ov7690_write_mask(sd,REG_REG28, 0, 0x80);
		ret |= ov7690_write_mask(sd, REG_SDECTRL, SDECTRL_FIX_UV,SDECTRL_FIX_UV);
		ret |= ov7690_write(sd,REG_REGDA, 0xa0);
		ret |= ov7690_write(sd,REG_REGDB, 0x40);
		break;
	case V4L2_COLORFX_GRASS_GREEN:
		ret = ov7690_write_mask(sd,REG_REG28, 0, 0x80);
		ret |= ov7690_write_mask(sd, REG_SDECTRL, SDECTRL_FIX_UV,SDECTRL_FIX_UV);
		ret |= ov7690_write(sd,REG_REGDA, 0x60);
		ret |= ov7690_write(sd,REG_REGDB, 0x60);
		break;
	case V4L2_COLORFX_EMBOSS: /* Reddish */
		ret = ov7690_write_mask(sd,REG_REG28, 0, 0x80);
		ret |= ov7690_write_mask(sd, REG_SDECTRL, SDECTRL_FIX_UV,SDECTRL_FIX_UV);
		ret |= ov7690_write(sd,REG_REGDA, 0x80);
		ret |= ov7690_write(sd,REG_REGDB, 0xc0);
		break;
		
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int ov7690_test_pattern(struct v4l2_subdev *sd, int value)
{
/*
 * 0: None
 * 1: Color bar overlay
 * 2: Framecount intenciry color bar
 * 3: Solid color bar
 */
	switch(value) {
	case 0:
		ov7690_write_mask(sd, REG_REG82, 0x0, 0xc);
		return ov7690_write_mask(sd, REG_REG0C, 0, REG0C_COLOR_BAR);
	case 1:
		ov7690_write_mask(sd, REG_REG82, 0x0, 0xc);
		return ov7690_write_mask(sd, REG_REG0C, REG0C_COLOR_BAR, REG0C_COLOR_BAR);
	case 2:
		ov7690_write_mask(sd, REG_REG0C, 0, REG0C_COLOR_BAR);
		return ov7690_write_mask(sd, REG_REG82, 0x8, 0xc);
	case 3:
		ov7690_write_mask(sd, REG_REG0C, 0, REG0C_COLOR_BAR);
		return ov7690_write_mask(sd, REG_REG82, 0xc, 0xc);	
	}
	return -ERANGE;
}

static int ov7690_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov7690_info *info =
		container_of(ctrl->handler, struct ov7690_info, ctrls);
	struct v4l2_subdev *sd = &info->sd;

	switch(ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return ov7690_s_brightness(sd, ctrl->val);
	case V4L2_CID_CONTRAST:
		return ov7690_s_contrast(sd, ctrl->val);
	case V4L2_CID_HUE:
		return ov7690_s_hue(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return ov7690_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return ov7690_s_hflip(sd, ctrl->val);
	case V4L2_CID_SHARPNESS:
		return ov7690_s_sharpness(sd, ctrl->val);
	case V4L2_CID_SATURATION:
		return ov7690_s_saturation(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return ov7690_s_exposure(sd, ctrl->val);
	case V4L2_CID_COLORFX:
		return ov7690_s_colorfx(sd, ctrl->val);
	case V4L2_CID_TEST_PATTERN:
		return ov7690_test_pattern(sd, ctrl->val);
	default:
		return -EINVAL;  
	}

	SENSDBG("%s: did not honor entry CID= %x %s",__FUNCTION__,ctrl->id,
		v4l2_ctrl_get_name(ctrl->id) ?  v4l2_ctrl_get_name(ctrl->id) : "???");
	
	return -EINVAL;

}
static struct v4l2_ctrl_ops ov7690_ctrl_ops = {
        .s_ctrl = ov7690_s_ctrl,
};

static const char *ov7690_test_pattern_menu[] = {
	"Disabled",
	"Overlay Vertical Color Bars",
	"Gradual Vertical Color Bars",
	"Solid Vertical Color Bars ",
};

static const struct v4l2_ctrl_config ov7690_ctrls[] = {
        {
                .ops            = &ov7690_ctrl_ops,
                .id             = V4L2_CID_TEST_PATTERN,
                .type           = V4L2_CTRL_TYPE_MENU,
                .name           = "Test Pattern",
                .min            = 0,
                .max            = ARRAY_SIZE(ov7690_test_pattern_menu) - 1,
                .step           = 0,
                .def            = 0,
                .flags          = 0,
                .menu_skip_mask = 0,
                .qmenu          = ov7690_test_pattern_menu,
        }
};



/* -----------------------------------------------------------------------------
 * V4L2 subdev core operations
 */
 
static int ov7690_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov7690_info *info = to_state(sd);
	int ret = 0;
	
	mutex_lock(&info->power_lock);
	
	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (info->power_count == !on) {
		ret = __ov7690_set_power(info, !!on);
		if (ret < 0)
			goto out;
	}
	
	/* Update the power count. */
	info->power_count += on ? 1 : -1;
	WARN_ON(info->power_count < 0);
	
out:
	mutex_unlock(&info->power_lock);
	return ret;
}

static int ov7690_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
//	struct i2c_client *client = v4l2_get_subdevdata(sd);
	SENSDBG("%s id:%d none:%d\n",
	       __FUNCTION__, chip->ident, V4L2_IDENT_NONE);
//	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_OV7690, 0);
	chip->ident =  V4L2_IDENT_OV7690;
	chip->revision = 0;
	
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG

static int ov7690_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val = 0;
	int ret;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	if (!v4l2_chip_match_i2c_client(client, &reg->match)) 
		return -EINVAL;	
	ret = ov7690_read(sd, reg->reg & 0xff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int ov7690_s_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	ov7690_write(sd, reg->reg & 0xff, reg->val & 0xff);
	return 0;
}
#endif

/* fall-through for ioctls, which were not handled by the v4l2-subdev */
static   long ov7690_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg){
	switch (cmd) {
	case VIDIOC_QUERYCAP:
	{
		struct v4l2_capability *cap = (struct v4l2_capability *)arg;
		struct i2c_client *client = v4l2_get_subdevdata(sd);

		cap->version           = KERNEL_VERSION(0, 0, 1);
		strlcpy(cap->driver, "ov7690", sizeof(cap->driver));
		strlcpy(cap->card, "ov7690", sizeof(cap->card));
		snprintf(cap->bus_info, sizeof(cap->bus_info),
			 "media i2c 0x%02x",client->addr);
		cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
		
		break;
	}
	default:
		SENSDBG("%s rejecting ioctl 0x%08x\n",__FUNCTION__,cmd);
		return -EINVAL;
	}
	return 0;
}


/* -----------------------------------------------------------------------
 * V4L2 subdev internal operations
 */
static int ov7690_registered(struct v4l2_subdev *sd)
{
          struct i2c_client *client = v4l2_get_subdevdata(sd);
          struct ov7690_info *info = to_state(sd);
          int ret;

          ret = ov7690_power_on(info);
          if (ret < 0) {
                  dev_err(&client->dev, "OV7690 power up failed\n");
                  return ret;
          }
          /* Read out the chip version register */
	  ret = ov7690_detect(sd);
          if (ret < 0) {
                  dev_err(&client->dev, "OV7690 not detected, wrong manufacturer or version \n");
                  return -ENODEV;
          }

          ov7690_power_off(info);
  
          dev_info(&client->dev, "OV7690 detected at address 0x%02x\n",
                   client->addr);
          return ret;
}
  
static int ov7690_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;
	struct ov7690_info *info = to_state(sd);
	
	crop = v4l2_subdev_get_try_crop(fh, 0);
	crop->left = 0;
	crop->top = 0;
	crop->width = VGA_WIDTH;
	crop->height = VGA_HEIGHT;
	
	format = v4l2_subdev_get_try_format(fh, 0);
	format->width = VGA_WIDTH;
	format->height = VGA_HEIGHT;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_JPEG;;
	format->code =  V4L2_MBUS_FMT_YUYV8_2X8;
	
	return ov7690_s_power(sd, 1);
	
}

static int ov7690_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
  return ov7690_s_power(sd, 0);
}

/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops ov7690_subdev_core_ops = {
	.g_chip_ident = ov7690_g_chip_ident,
	.s_power = ov7690_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov7690_g_register,
	.s_register = ov7690_s_register,
#endif
	.ioctl = ov7690_ioctl,
	
};

static const struct v4l2_subdev_video_ops ov7690_subdev_video_ops = {
	.s_stream = ov7690_s_stream,
	.g_frame_interval = ov7690_g_frame_interval,
	.s_frame_interval = ov7690_s_frame_interval,
};

static struct v4l2_subdev_pad_ops ov7690_subdev_pad_ops = {
        .enum_mbus_code = ov7690_enum_mbus_code,
        .enum_frame_size = ov7690_enum_frame_size,
        .get_fmt = ov7690_get_format,
        .set_fmt = ov7690_set_format,
        .get_crop = ov7690_get_crop,
	.set_crop = ov7690_set_crop,
	.enum_frame_interval = ov7690_enum_frame_interval,
};

static const struct v4l2_subdev_ops ov7690_ops = {
	.core = &ov7690_subdev_core_ops,
	.video = &ov7690_subdev_video_ops,
	.pad = &ov7690_subdev_pad_ops,
};
static const struct v4l2_subdev_internal_ops ov7690_subdev_internal_ops = {
  .registered = ov7690_registered,
  .open = ov7690_open,
  .close = ov7690_close,
};
/* ----------------------------------------------------------------------- */

static int ov7690_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ov7690_info *info;
	int ret;
	int i;
	struct ov7690_platform_data *pdata = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	if (pdata == NULL) {
	  dev_err(&client->dev, "No platform data\n");
	}

	if ( !i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&adapter->dev,
			"I2C-Adapter doesn't support "
			"I2C_FUNC_SMBUS_BYTE_DATA\n");
		return -EIO;
	}

	info = kzalloc(sizeof(struct ov7690_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;

	info->pdata = pdata;

	v4l2_ctrl_handler_init(&info->ctrls,9 + ARRAY_SIZE(ov7690_ctrls));
	info->bright_ctrl = 
		v4l2_ctrl_new_std(&info->ctrls,&ov7690_ctrl_ops,
				  V4L2_CID_BRIGHTNESS,
				  BRIGHTNESS_MIN, BRIGHTNESS_MAX, 1, 0);
	v4l2_ctrl_new_std(&info->ctrls,&ov7690_ctrl_ops,
			  V4L2_CID_CONTRAST,
			  CONTRAST_MIN, CONTRAST_MAX, 1, 0);
	v4l2_ctrl_new_std(&info->ctrls,&ov7690_ctrl_ops,
			  V4L2_CID_VFLIP,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->ctrls,&ov7690_ctrl_ops,
			  V4L2_CID_HFLIP,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std(&info->ctrls,&ov7690_ctrl_ops,
			  V4L2_CID_HUE,
			  HUE_MIN, HUE_MAX, HUE_STEP, 0);
	v4l2_ctrl_new_std(&info->ctrls,&ov7690_ctrl_ops,
			  V4L2_CID_SHARPNESS,
			  SHARPNESS_MIN, SHARPNESS_MAX, 1, -1);
	v4l2_ctrl_new_std(&info->ctrls,&ov7690_ctrl_ops,
			  V4L2_CID_SATURATION,
			  SATURATION_MIN, SATURATION_MAX, 1, -1);
	v4l2_ctrl_new_std(&info->ctrls,&ov7690_ctrl_ops,
			  V4L2_CID_EXPOSURE,
			  EXPOSURE_MIN, EXPOSURE_MAX, 1, 0);
	v4l2_ctrl_new_std_menu(&info->ctrls,&ov7690_ctrl_ops,
			       V4L2_CID_COLORFX,
			       V4L2_COLORFX_GRASS_GREEN, 
			       (0xffffffffUL ^
				((1<< V4L2_COLORFX_NONE) |
				 (1<< V4L2_COLORFX_BW) |
				 (1<<V4L2_COLORFX_SEPIA) |
				 (1<<V4L2_COLORFX_NEGATIVE) |
				 (1<<V4L2_COLORFX_SKY_BLUE) |
				 (1<<V4L2_COLORFX_GRASS_GREEN) |
				 (1<<V4L2_COLORFX_EMBOSS)))
			       , 0);

	for (i = 0; i < ARRAY_SIZE(ov7690_ctrls); ++i)
		v4l2_ctrl_new_custom(&info->ctrls, &ov7690_ctrls[i], NULL);


	info->sd.ctrl_handler = &info->ctrls;

	mutex_init(&info->power_lock);
	v4l2_i2c_subdev_init(&info->sd, client, &ov7690_ops);


	info->sd.internal_ops = &ov7690_subdev_internal_ops;

	info->fmt = &ov7690_formats[0];

	info->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	info->curr_crop.width = VGA_WIDTH;
        info->curr_crop.height = VGA_HEIGHT;
        info->curr_crop.left = 0;
        info->curr_crop.top = 0;

	info->curr_fi = &ov7690_intervals[0];

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
#if defined(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_init(&info->sd.entity, 1, &info->pad, 0);
#endif
	if (ret < 0) {
	  v4l2_ctrl_handler_free(&info->ctrls);
	  media_entity_cleanup(&info->sd.entity);
	  kfree(info);
        }
	
	return ret;
}


static int ov7690_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov7690_info *info = to_state(sd);

	v4l2_ctrl_handler_free(&info->ctrls);
	v4l2_device_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id ov7690_id[] = {
	{ "ov7690", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov7690_id);

static struct i2c_driver ov7690_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ov7690",
	},
	.probe		= ov7690_probe,
	.remove		= ov7690_remove,
	.id_table	= ov7690_id,
};

static __init int init_ov7690(void)
{
	return i2c_add_driver(&ov7690_driver);
}

static __exit void exit_ov7690(void)
{
	i2c_del_driver(&ov7690_driver);
}

module_init(init_ov7690);
module_exit(exit_ov7690);

MODULE_AUTHOR("Michael Tsukernik <mike.tsukernik@logicpd.com>");
MODULE_DESCRIPTION("Driver for OmniVision ov7690 sensor");
MODULE_LICENSE("GPL");

