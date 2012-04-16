/*
 * A V4L2 driver for OmniVision OV7690 cameras.
 *
 * Copyright 2010 One Laptop Per Child
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */

#ifndef __OV7690_H
#define __OV7690_H
#include <media/v4l2-subdev.h>

struct ov7690_platform_data {
	int (*s_xclk) (struct v4l2_subdev *s, u32 on);
	int min_width;			/* Filter out smaller sizes */
	int min_height;			/* Filter out smaller sizes */
	int clock_speed;		/* External clock speed (MHz) */
};
#define V4L2_CID_TEST_PATTERN   (V4L2_CID_USER_BASE | 0x1001)

#endif
