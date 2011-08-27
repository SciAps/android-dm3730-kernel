#ifndef __LINUX_I2C_TSC2004_H
#define __LINUX_I2C_TSC2004_H

/* linux/i2c/tsc2004.h */

struct tsc2004_platform_data {
	u16	model;				/* 2004. */
	u16	x_plate_ohms;

	int	(*get_pendown_state)(void);
	void	(*clear_penirq)(void);		/* If needed, clear 2nd level
						   interrupt source */
	int	(*pre_init_platform_hw)(struct tsc2004_platform_data *pdata);
	int	(*init_platform_hw)(void);
	void	(*exit_platform_hw)(void);
	void	(*post_exit_platform_hw)(struct tsc2004_platform_data *pdata);
	char	*regulator_name;
	struct regulator	*regulator;	/* If needed by tsc2004 */
};

#endif
