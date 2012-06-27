/*
 * drivers/input/touchscreen/tsc2004.c
 *
 * Copyright (C) 2009 Texas Instruments Inc
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 * Using code from:
 *  - tsc2007.c
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c/tsc2004.h>
#include <linux/kbd_kern.h>

#define TS_DEBOUNCE			50 /* ms delay for penup event */

/* Control byte 0 */
#define TSC2004_CMD0(addr, pnd, rw) ((addr<<3)|(pnd<<1)|rw)
/* Control byte 1 */
#define TSC2004_CMD1(cmd, mode, rst) ((1<<7)|(cmd<<4)|(mode<<2)|(rst<<1))

/* Command Bits */
#define READ_REG	1
#define WRITE_REG	0
#define SWRST_TRUE	1
#define SWRST_FALSE	0
#define PND0_TRUE	1
#define PND0_FALSE	0

/* Converter function mapping */
enum convertor_function {
	MEAS_X_Y_Z1_Z2,	/* Measure X,Y,z1 and Z2:	0x0 */
	MEAS_X_Y,	/* Measure X and Y only:	0x1 */
	MEAS_X,		/* Measure X only:		0x2 */
	MEAS_Y,		/* Measure Y only:		0x3 */
	MEAS_Z1_Z2,	/* Measure Z1 and Z2 only:	0x4 */
	MEAS_AUX,	/* Measure Auxillary input:	0x5 */
	MEAS_TEMP1,	/* Measure Temparature1:	0x6 */
	MEAS_TEMP2,	/* Measure Temparature2:	0x7 */
	MEAS_AUX_CONT,	/* Continuously measure Auxillary input: 0x8 */
	X_DRV_TEST,	/* X-Axis drivers tested 	0x9 */
	Y_DRV_TEST,	/* Y-Axis drivers tested 	0xA */
	/*Command Reserved*/
	SHORT_CKT_TST = 0xC,	/* Short circuit test:	0xC */
	XP_XN_DRV_STAT,	/* X+,Y- drivers status:	0xD */
	YP_YN_DRV_STAT,	/* X+,Y- drivers status:	0xE */
	YP_XN_DRV_STAT	/* Y+,X- drivers status:	0xF */
};

/* Register address mapping */
enum register_address {
	X_REG,		/* X register:		0x0 */
	Y_REG,		/* Y register:		0x1 */
	Z1_REG,		/* Z1 register:		0x2 */
	Z2_REG,		/* Z2 register:		0x3 */
	AUX_REG,	/* AUX register:	0x4 */
	TEMP1_REG,	/* Temp1 register:	0x5 */
	TEMP2_REG,	/* Temp2 register:	0x6 */
	STAT_REG,	/* Status Register:	0x7 */
	AUX_HGH_TH_REG,	/* AUX high threshold register:	0x8 */
	AUX_LOW_TH_REG,	/* AUX low threshold register:	0x9 */
	TMP_HGH_TH_REG,	/* Temp high threshold register:0xA */
	TMP_LOW_TH_REG,	/* Temp low threshold register:	0xB */
	CFR0_REG,	/* Configuration register 0:	0xC */
	CFR1_REG,	/* Configuration register 1:	0xD */
	CFR2_REG,	/* Configuration register 2:	0xE */
	CONV_FN_SEL_STAT	/* Convertor function select register:	0xF */
};

/* Supported Resolution modes */
enum resolution_mode {
	MODE_10BIT,	/* 10 bit resolution */
	MODE_12BIT		/* 12 bit resolution */
};

/* Configuraton register bit fields */
/* CFR0 */
#define PEN_STS_CTRL_MODE	(1<<15)
#define ADC_STS			(1<<14)
#define RES_CTRL		(1<<13)
#define ADC_CLK_4MHZ		(0<<11)
#define ADC_CLK_2MHZ		(1<<11)
#define ADC_CLK_1MHZ		(2<<11)
#define PANEL_VLTG_STB_TIME_0US		(0<<8)
#define PANEL_VLTG_STB_TIME_100US	(1<<8)
#define PANEL_VLTG_STB_TIME_500US	(2<<8)
#define PANEL_VLTG_STB_TIME_1MS		(3<<8)
#define PANEL_VLTG_STB_TIME_5MS		(4<<8)
#define PANEL_VLTG_STB_TIME_10MS	(5<<8)
#define PANEL_VLTG_STB_TIME_50MS	(6<<8)
#define PANEL_VLTG_STB_TIME_100MS	(7<<8)

/* CFR2 */
#define PINTS1			(1<<15)
#define PINTS0			(1<<14)
#define MEDIAN_VAL_FLTR_SIZE_1	(0<<12)
#define MEDIAN_VAL_FLTR_SIZE_3	(1<<12)
#define MEDIAN_VAL_FLTR_SIZE_7	(2<<12)
#define MEDIAN_VAL_FLTR_SIZE_15	(3<<12)
#define AVRG_VAL_FLTR_SIZE_1	(0<<10)
#define AVRG_VAL_FLTR_SIZE_3_4	(1<<10)
#define AVRG_VAL_FLTR_SIZE_7_8	(2<<10)
#define AVRG_VAL_FLTR_SIZE_16	(3<<10)
#define MAV_FLTR_EN_X		(1<<4)
#define MAV_FLTR_EN_Y		(1<<3)
#define MAV_FLTR_EN_Z		(1<<2)

#define	MAX_12BIT		((1 << 12) - 1)
#define MEAS_MASK		0xFFF

struct ts_event {
	u16	x;
	u16	y;
	u16	z1, z2;
};

struct tsc2004 {
	struct input_dev	*input;
	char			phys[32];
	struct delayed_work	work;

	struct i2c_client	*client;

	u16			model;
	u16			x_plate_ohms;
	u16			last_pressure;

	bool			pendown;
	int			irq;
#ifdef CONFIG_TOUCHSCREEN_TSC2004_POKE_CONSOLE
	unsigned long		last_poke;
#endif

	int			(*get_pendown_state)(void);
	void			(*clear_penirq)(void);

	// Exposed settings to userspace via sysfs
	int swapxy;
	int flipx;
	int flipy;
	int legacy_8bit;
};

static inline int tsc2004_read_word_data(struct tsc2004 *tsc, u8 cmd)
{
	s32 data;
	u16 val;

	data = i2c_smbus_read_word_data(tsc->client, cmd);
	if (data < 0) {
		dev_err(&tsc->client->dev, "i2c io (read) error: %d\n", data);
		return data;
	}

	/* The protocol and raw data format from i2c interface:
	 * S Addr Wr [A] Comm [A] S Addr Rd [A] [DataLow] A [DataHigh] NA P
	 * Where DataLow has [D11-D4], DataHigh has [D3-D0 << 4 | Dummy 4bit].
	 */
	val = swab16(data);

	dev_dbg(&tsc->client->dev, "data: 0x%x, val: 0x%x\n", data, val);

	return val;
}

static inline int tsc2004_write_word_data(struct tsc2004 *tsc, u8 cmd, u16 data)
{
	u16 val;

	val = swab16(data);
	return i2c_smbus_write_word_data(tsc->client, cmd, val);
}

static inline int tsc2004_write_cmd(struct tsc2004 *tsc, u8 value)
{
	return i2c_smbus_write_byte(tsc->client, value);
}

static inline int tsc2004_read_reg(struct tsc2004 *tsc, u8 reg)
{
	return tsc2004_read_word_data(tsc, TSC2004_CMD0(reg, PND0_FALSE, READ_REG));
}

static int tsc2004_prepare_for_reading(struct tsc2004 *ts)
{
	int err;
	int cmd, data;

	/* Reset the TSC, configure for 12 bit */
	cmd = TSC2004_CMD1(MEAS_X_Y_Z1_Z2, MODE_12BIT, SWRST_TRUE);
	err = tsc2004_write_cmd(ts, cmd);
	if (err < 0)
		return err;

	/* Enable interrupt for PENIRQ and DAV */
	cmd = TSC2004_CMD0(CFR2_REG, PND0_FALSE, WRITE_REG);
	data = PINTS0 | MEDIAN_VAL_FLTR_SIZE_15 |
		AVRG_VAL_FLTR_SIZE_7_8 | MAV_FLTR_EN_X | MAV_FLTR_EN_Y |
		MAV_FLTR_EN_Z;
	err = tsc2004_write_word_data(ts, cmd, data);
	if (err < 0)
		return err;

	/* Configure the TSC in TSMode 1 */
	cmd = TSC2004_CMD0(CFR0_REG, PND0_FALSE, WRITE_REG);
	data = PEN_STS_CTRL_MODE | ADC_CLK_2MHZ | PANEL_VLTG_STB_TIME_1MS;
	err = tsc2004_write_word_data(ts, cmd, data);
	if (err < 0)
		return err;

	/* Enable x, y, z1 and z2 conversion functions */
	cmd = TSC2004_CMD1(MEAS_X_Y_Z1_Z2, MODE_12BIT, SWRST_FALSE);
	err = tsc2004_write_cmd(ts, cmd);
	if (err < 0)
		return err;

	return 0;
}

static void tsc2004_work(struct work_struct *work)
{
	struct tsc2004 *tsc =
		container_of(to_delayed_work(work), struct tsc2004, work);
	struct input_dev *input = tsc->input;

	tsc->pendown = 0;
	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);

	dev_dbg(&tsc->client->dev, "UP\n");
}

static irqreturn_t tsc2004_irq(int irq, void *handle)
{
	struct tsc2004 *tsc = handle;
	struct input_dev *input = tsc->input;
	u16 status;
	u16 cfr0;
	u16 x = 0, y = 0, z1 = 0, z2 = 0;

	/* The work queue item is used to queue a touchscreen release;
	 * if we get here, we'll cut another check in 50 ms.  Also, this
	 * acts as a "mutex" of sorts to make sure the workqueue
	 * item doesn't conflict with the inputs in this handler.
	 *
	 * (The TSC2004 won't always issue a "data available" interrupt
	 *  when the user has let go of the screen.  The 50ms work queue
	 *  itme is a catch for this.)
	 *
	 * (This handler is run in a thread, as requested by
	 * "request_threaded_irq(...)" )
	 */
	cancel_delayed_work_sync(&tsc->work);

	cfr0 = tsc2004_read_reg(tsc, CFR0_REG);
	status = tsc2004_read_reg(tsc, STAT_REG);

	if (tsc->pendown != !!(cfr0 & 0x8000))
	{
		tsc->pendown = !!(cfr0 & 0x8000);
		input_report_key(input, BTN_TOUCH, tsc->pendown);

		dev_dbg(&tsc->client->dev, "%s\n", tsc->pendown ? "DOWN" : "UP");
	}

#ifdef CONFIG_TOUCHSCREEN_TSC2004_POKE_CONSOLE
	if(tsc->pendown && (!ts->last_poke || (jiffies - ts->last_poke) > 30 * HZ)) {
		do_poke_blanked_console = 1;
		schedule_console_callback();
		ts->last_poke = jiffies;
	}
#endif

	if(status & 0x8000)
	{
		u16 x1 = x = tsc2004_read_reg(tsc, X_REG) & MEAS_MASK;
		if((tsc->swapxy && tsc->flipy) || (!tsc->swapxy && tsc->flipx))
			x1 = 0xfff - x;
		if(tsc->legacy_8bit)
			x1 >>= 4;
		input_report_abs(input, tsc->swapxy ? ABS_Y : ABS_X, x1);
	}
	if(status & 0x4000)
	{
		// Don't modify the y value read for pressure detection below
		u16 y1 = y = tsc2004_read_reg(tsc, Y_REG) & MEAS_MASK;
		if((tsc->swapxy && tsc->flipx) || (!tsc->swapxy && tsc->flipy))
			y1 = 0xfff - y;
		if(tsc->legacy_8bit)
			y1 >>= 4;
		input_report_abs(input, (tsc->swapxy ? ABS_X : ABS_Y), y1);
	}
	if(status & 0x2000)
	{
		z1 = tsc2004_read_reg(tsc, Z1_REG);
	}
	if(status & 0x1000)
	{
		z2 = tsc2004_read_reg(tsc, Z2_REG);
	}
	// If we've read x, z1, and z2 this time around, we have the
	// info we need to calculate pressure (usually the tsc2004
	// has all the fields required delivered; if it didn't, this
	// is a failsafe check).
	if((status & 0xb000) == 0xb000)
	{
		unsigned int pressure = 0;

		if (likely(x && z1)) {
			/* compute touch pressure resistance using equation #1 */
			pressure = z2 - z1;
			pressure *= x;
			pressure *= tsc->x_plate_ohms;
			pressure /= z1;
			pressure = (pressure + 2047) >> 12;

			/* Do a sanity check on the result before reporting anything
			 * and flip the orientation of the reading (the formula
			 * in equation #1 gives a lower value for higher pressure
			 * which is opposite of what the Linux input system is
			 * supposed to report
			 */
			if(pressure < 512)
			{
				pressure = 511 - pressure;
				input_report_abs(input, ABS_PRESSURE, pressure);
			}
		}
	}

	input_sync(input);

	if(tsc->pendown)
		schedule_delayed_work(&tsc->work,
			msecs_to_jiffies(TS_DEBOUNCE));

	return IRQ_HANDLED;
}

static void tsc2004_free_irq(struct tsc2004 *ts)
{
	free_irq(ts->irq, ts);
	if (cancel_delayed_work_sync(&ts->work)) {
		/*
		 * Work was pending, therefore we need to enable
		 * IRQ here to balance the disable_irq() done in the
		 * interrupt handler.
		 */
		enable_irq(ts->irq);
	}
}

static int tsc2004_resume(struct i2c_client *client)
{
	struct tsc2004 *ts = i2c_get_clientdata(client);
	tsc2004_prepare_for_reading(ts);
	return 0;
}

struct tsc2004_attribute
{
	struct device_attribute attrib;
	int offset;
};

static ssize_t tsc2004_attr_show(struct device *dev,
                                   struct device_attribute *_attr,
                                   char *buf)
{
	struct tsc2004_attribute *attr = container_of(_attr, struct tsc2004_attribute, attrib);
	struct tsc2004 *ts = dev_get_drvdata(dev);
	int *attr_val = (int *)(((void *)ts) + attr->offset);
	return sprintf(buf, "%u\n", *attr_val);
}

static ssize_t tsc2004_attr_store(struct device *dev,
                                   struct device_attribute *_attr,
                                   const char *buf, size_t count)
{
	struct tsc2004_attribute *attr = container_of(_attr, struct tsc2004_attribute, attrib);
	struct tsc2004 *ts = dev_get_drvdata(dev);
	unsigned long val;
	int *attr_val = (int *)(((void *)ts) + attr->offset);
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	*attr_val = !!val;

	return count;
}

#define ABS_MASK_MAX 0x80000000

static ssize_t tsc2004_attr_abs_show(struct device *dev,
                                   struct device_attribute *_attr,
                                   char *buf)
{
	struct tsc2004_attribute *attr = container_of(_attr, struct tsc2004_attribute, attrib);
	struct tsc2004 *ts = dev_get_drvdata(dev);
	struct input_absinfo *abs = &ts->input->absinfo[attr->offset & ~ABS_MASK_MAX];

	if(attr->offset & ABS_MASK_MAX)
	{
		return sprintf(buf, "%u\n", abs->maximum);
	} else {
		return sprintf(buf, "%u\n", abs->minimum);
	}
}

static ssize_t tsc2004_attr_abs_store(struct device *dev,
                                   struct device_attribute *_attr,
                                   const char *buf, size_t count)
{
	struct tsc2004_attribute *attr = container_of(_attr, struct tsc2004_attribute, attrib);
	struct tsc2004 *ts = dev_get_drvdata(dev);
	struct input_absinfo *abs = &ts->input->absinfo[attr->offset & ~ABS_MASK_MAX];
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	if(attr->offset & ABS_MASK_MAX)
	{
		abs->maximum = val;
	} else {
		abs->minimum = val;
	}

	return count;
}

#define TSC2004_ATTRIBUTE(_name) \
struct tsc2004_attribute dev_attr_##_name = { \
	.attrib = __ATTR(_name, 0644, tsc2004_attr_show, tsc2004_attr_store), \
	.offset = offsetof(struct tsc2004, _name), }

#define TSC2004_ABS_ATTRIBUTE(_name, _offset) \
struct tsc2004_attribute dev_attr_##_name = { \
	.attrib = __ATTR(_name, 0644, tsc2004_attr_abs_show, tsc2004_attr_abs_store), \
	.offset = _offset, }

static TSC2004_ATTRIBUTE(swapxy);
static TSC2004_ATTRIBUTE(flipx);
static TSC2004_ATTRIBUTE(flipy);
static TSC2004_ATTRIBUTE(legacy_8bit);

static TSC2004_ABS_ATTRIBUTE(x_min, ABS_X);
static TSC2004_ABS_ATTRIBUTE(x_max, ABS_X | ABS_MASK_MAX);
static TSC2004_ABS_ATTRIBUTE(y_min, ABS_Y);
static TSC2004_ABS_ATTRIBUTE(y_max, ABS_Y | ABS_MASK_MAX);

static struct attribute *tsc2004_attributes[] = {
	&dev_attr_swapxy.attrib.attr,
	&dev_attr_flipx.attrib.attr,
	&dev_attr_flipy.attrib.attr,
	&dev_attr_legacy_8bit.attrib.attr,
	&dev_attr_x_min.attrib.attr,
	&dev_attr_x_max.attrib.attr,
	&dev_attr_y_min.attrib.attr,
	&dev_attr_y_max.attrib.attr,
	NULL
};

static const struct attribute_group tsc2004_attr_group = {
	.attrs = tsc2004_attributes,
};

static int __devinit tsc2004_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct tsc2004 *ts;
	struct tsc2004_platform_data *pdata = pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (pdata->pre_init_platform_hw)
		pdata->pre_init_platform_hw(pdata);

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	ts = kzalloc(sizeof(struct tsc2004), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	ts->irq = client->irq;
	ts->input = input_dev;
	INIT_DELAYED_WORK(&ts->work, tsc2004_work);

	ts->model             = pdata->model;
	ts->x_plate_ohms      = pdata->x_plate_ohms;
	ts->get_pendown_state = pdata->get_pendown_state;
	ts->clear_penirq      = pdata->clear_penirq;
	ts->legacy_8bit = 1;

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));

	input_dev->name = "TSC2004 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
        __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 400, 0, 0);

	if (pdata->init_platform_hw)
		pdata->init_platform_hw();

	err = request_threaded_irq(ts->irq, NULL, tsc2004_irq,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			client->dev.driver->name, ts);
	if (err < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}

	/* Prepare for touch readings */
	err = tsc2004_prepare_for_reading(ts);
	if (err < 0)
		goto err_free_irq;

	input_dev->dev.parent = &client->dev;

	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	i2c_set_clientdata(client, ts);

	err = sysfs_create_group(&client->dev.kobj, &tsc2004_attr_group);

	return 0;

 err_free_irq:
	tsc2004_free_irq(ts);
	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();
	if (pdata->post_exit_platform_hw)
		pdata->post_exit_platform_hw(pdata);
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int __devexit tsc2004_remove(struct i2c_client *client)
{
	struct tsc2004	*ts = i2c_get_clientdata(client);
	struct tsc2004_platform_data *pdata = client->dev.platform_data;

	sysfs_remove_group(&ts->input->dev.kobj, &tsc2004_attr_group);

	tsc2004_free_irq(ts);

	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();
	if (pdata->post_exit_platform_hw)
		pdata->post_exit_platform_hw(pdata);

	input_unregister_device(ts->input);
	kfree(ts);

	return 0;
}

static struct i2c_device_id tsc2004_idtable[] = {
	{ "tsc2004", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tsc2004_idtable);

static struct i2c_driver tsc2004_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tsc2004"
	},
	.id_table	= tsc2004_idtable,
	.probe		= tsc2004_probe,
	.remove		= __devexit_p(tsc2004_remove),
	.resume		= tsc2004_resume,
};

static int __init tsc2004_init(void)
{
	return i2c_add_driver(&tsc2004_driver);
}

static void __exit tsc2004_exit(void)
{
	i2c_del_driver(&tsc2004_driver);
}

module_init(tsc2004_init);
module_exit(tsc2004_exit);

MODULE_AUTHOR("Vaibhav Hiremath <hvaibhav@ti.com>");
MODULE_DESCRIPTION("TSC2004 TouchScreen Driver");
MODULE_LICENSE("GPL");
