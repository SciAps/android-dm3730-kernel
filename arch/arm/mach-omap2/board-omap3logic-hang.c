/*
 * linux/arch/arm/mach-omap2/board-dm3730omap3logic-hang.c
 *
 * Copyright (C) 2011 Logic Product Development
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include "board-omap3logic.h"

volatile int omap3logic_hang;

static int __init omap3logic_hang_onstart(char *str)
{
	omap3logic_hang = 1;
	return 1;
}

__setup("hang", omap3logic_hang_onstart);
