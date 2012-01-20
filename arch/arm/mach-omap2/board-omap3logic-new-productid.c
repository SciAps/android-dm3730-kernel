/*
 * linux/arch/arm/mach-omap2/board-oma3logic-new-product-id.c
 *
 * Copyright (C) 2009 Logic Product Development, Inc.
 * Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>

#include <plat/hardware.h>

#include <plat/gpio.h>
#include <plat/mux.h>
#include <plat/board.h>
#include <plat/common.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/mach-types.h>
#include "control.h"
#include <plat/sram.h>

#include <plat/board-omap3logic.h>
#include <plat/omap3logic-new-productid.h>
#include "mux.h"

#undef DEBUG

#ifdef DEBUG
#define DPRINTF(fmt, args...) printk(KERN_DBG fmt, ## args)
#else
#define DPRINTF(fmt, ...)
#endif

/*
 * Header file that interfaces to environment to access data
 */

/* Create the enum list of keys, not strings! */
#undef ID_KEY_STRINGS
#define ID_KEY_ENUMS

#undef ID_KEY_START
#undef ID_KEY_ENTRY
#undef ID_KEY_END 

#if defined(ID_KEY_STRINGS)
/* This is the usage to build the keys for the compiler; we define
 * an array of strings whose index is the value */
#define ID_KEY_START static char *id_keys[] = {
#define ID_KEY_ENTRY(XX) #XX ,
#define ID_KEY_END };
#elif defined(ID_KEY_ENUMS)
/* This is the usage by people using the library to access the data */
#define ID_KEY_START typedef enum {
#define ID_KEY_ENTRY(XX) ID_KEY_ ## XX,
#define ID_KEY_END } id_keys_t;
#else
#error "Need either ID_KEY_INTERFACE or ID_KEY_COMPILER defined!"
#endif

/* There are some implied conventions here: */
/* - names of keys that contain other keys (dictionaries) end in "_group" */
/* - names of keys that provide a register setting end in "_reg"          */
/* - any keys that specify a unit of measure, include units in the name (ie. _mhz, _degf, _bytes) */

ID_KEY_START

/* Manufacturing unique data for each SOM */
ID_KEY_ENTRY(serialization_group)
ID_KEY_ENTRY(serial_number)
ID_KEY_ENTRY(wifi_ethaddr1)
ID_KEY_ENTRY(wifi_ethaddr2)
ID_KEY_ENTRY(wifi_ethaddr3)
ID_KEY_ENTRY(wifi_ethaddr4)
ID_KEY_ENTRY(nvs)

/* BOM Model number infromation */
ID_KEY_ENTRY(model_group)
ID_KEY_ENTRY(model_name)
ID_KEY_ENTRY(part_number)
ID_KEY_ENTRY(version_code)
ID_KEY_ENTRY(hardware_platform)

/* CPU specific information */
ID_KEY_ENTRY(cpu0_group)
ID_KEY_ENTRY(type)
ID_KEY_ENTRY(number)
ID_KEY_ENTRY(speed_mhz)
ID_KEY_ENTRY(temp_class)

/* CPU bus information */
ID_KEY_ENTRY(cpu0_bus_group)

/* DRAM bus information */
ID_KEY_ENTRY(dram_bus_group)
ID_KEY_ENTRY(sysconfig_reg)
ID_KEY_ENTRY(sharing_reg)
ID_KEY_ENTRY(dlla_ctrl_reg)
ID_KEY_ENTRY(cs_cfg_reg)
// ID_KEY_ENTRY(cs0_group) Used in the dram_bus_group, but key defined below after local_bus_group
// ID_KEY_ENTRY(cs1_group) Used in the dram_bus_group, but key defined below after local_bus_group
ID_KEY_ENTRY(mcfg_reg)
ID_KEY_ENTRY(mr_reg)
ID_KEY_ENTRY(rfr_ctrl_reg)
ID_KEY_ENTRY(emr2_reg)
ID_KEY_ENTRY(actim_ctrla_reg)
ID_KEY_ENTRY(actim_ctrlb_reg)
ID_KEY_ENTRY(power_reg)

/* GPMC keys */
ID_KEY_ENTRY(local_bus_group)
ID_KEY_ENTRY(cs0_group)
ID_KEY_ENTRY(cs1_group)
ID_KEY_ENTRY(cs2_group)
ID_KEY_ENTRY(cs3_group)
ID_KEY_ENTRY(cs4_group)
ID_KEY_ENTRY(cs5_group)
ID_KEY_ENTRY(cs6_group)
ID_KEY_ENTRY(config1_reg)
ID_KEY_ENTRY(config2_reg)
ID_KEY_ENTRY(config3_reg)
ID_KEY_ENTRY(config4_reg)
ID_KEY_ENTRY(config5_reg)
ID_KEY_ENTRY(config6_reg)
ID_KEY_ENTRY(config7_reg)

/* Manufacturing unique data for each SOM */
ID_KEY_ENTRY(lan_ethaddr1)
ID_KEY_ENTRY(lan_ethaddr2)
ID_KEY_ENTRY(lan_ethaddr3)
ID_KEY_ENTRY(lan_ethaddr4)

/* End of keys */
ID_KEY_END

typedef enum {
	/* Number */
	IDENUM_NEG_NUM = 0,
	IDENUM_POS_NUM,

	/* String/Hex String */
	IDENUM_STR,
	IDENUM_HEXSTR,

	/* Array */
	IDENUM_ARRAY,

	/* Dictionary */
	IDENUM_DICT,

	/* Key */
	IDENUM_KEY,

	/* Any string */
	IDENUM_ANY_STRING,

	/* Any number */
	IDENUM_ANY_NUMBER,

} idenum_t;

/* structure of builtin keys */
struct id_key {
	unsigned char *ptr;
	unsigned int size;
};

#define ID_EOK		0	/* Okay */
#define ID_ENOENT	2	/* No such key */
#define ID_ENOMEM	12	/* Out of memory */
#define ID_EACCES	13	/* Permission denied */
#define ID_ENODEV	19	/* No such device */
#define ID_EINVAL	22	/* Invalid arcument */
#define ID_EDOM		33	/* argument out of domain of func */
#define ID_ERANGE	34	/* Out of range */
#define	ID_EL2NSYNC	45	/* Level 2 not synchronized */
#define	ID_ENOMEDIUM	123	/* No medium found */


/*
 * return a byte from the ID data at offset 'offset' and set *oor to zero
 * if offset is in range of the device.  If offset is out of range then
 * set *oor to non-zero
 */

static unsigned char id_fetch_byte(int offset, int *oor)
{
	unsigned char *p = sram_get_base_va();
	if (offset < (32<<10)) {
		*oor = ID_EOK;
		return p[offset];
	}

	*oor = -ID_ERANGE;
	return 0;
}

struct id_data {
	unsigned char *mem_ptr;	/* pointer to memory to copy data into */
	unsigned int root_size;
	unsigned int root_offset;
};

/* Function to do the intial startup (i.e. figure out how much data, offset of
 * key table, etc */
static int id_startup(struct id_data *data);
/*
 * Functions provided back to callers for use in accessing data
 */

/* ID data "cookie" used to access data; ultimately this will be opaque
 * to the callers as they don't need to know whats in it, just pass it around
 */
struct id_cookie {
	unsigned int start_offset;	/* start offset from beginning of data */
	unsigned int size;		/* size of data in bytes */
	unsigned int offset;		/* current read offset */
};

/* Initialize the cookie to cover the whole root dictionary */
static int id_init_cookie(struct id_data *data, struct id_cookie *cookie);

/* What is the read pointer cookie is pointing at */
static int id_whatis(struct id_cookie *cookie, idenum_t *type);

/* User interface functions */

static int id_dict_find_key(struct id_cookie *cookie, id_keys_t key);
static int id_find_dict(struct id_cookie *cookie, id_keys_t key, idenum_t type);
static int id_find_string(struct id_cookie *cookie, id_keys_t key, unsigned char *str_ptr, unsigned int *str_size);
static int id_find_number(struct id_cookie *cookie, id_keys_t key, int *num);
static int id_find_numbers(struct id_cookie *cookie, id_keys_t *key, int key_size, int *nums);

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(var) sizeof(var)/sizeof((var)[0])
#endif

/*
 * Extract an unsigned packed number, first byte is in 'pack_bits'
 * of first byte, starting at offset 'offset' */
static unsigned int extract_unsigned_pnum(struct id_cookie *cookie, int pack_bits, int *err);
static int extract_signed_pnum(struct id_cookie *cookie, int pack_bits, int *err);



#define ID_MAX_KEY_SIZE 32

static int id_extract_size(struct id_cookie *cookie, int *err);



/* struct id_data id_data; */

struct __attribute__ ((packed)) id_header { 
	unsigned char signature[4];
	unsigned char id_fmt_ver;
	unsigned char unused0;
	unsigned short data_length;
} ;

struct __attribute__ ((packed)) id_checksums { 
	unsigned short header;
	unsigned short data;
} ;

/*
 * Calculate a CRC-15 of a data buffer passed in
 */

void crc_15_step(unsigned short *crc, unsigned char byte)
{
	int i;
	unsigned short crcnext;

	for (i=0; i<7; ++i) {
		crcnext = (byte & 1) ^ (*crc>>14);
		*crc = (*crc << 1) & 0x7fff;
		if (crcnext)
			*crc ^= 0x4599;
		byte >>= 1;
	}
}

unsigned short crc_15(void *buf, int len)
{
	unsigned char *p = buf;
	unsigned short xsum = 0;
	int i;

	for (i=0; i<len; ++i) {
		crc_15_step(&xsum, p[i]);
	}
	return xsum;
}

static int id_startup(struct id_data *data)
{
	int i, err;
	struct id_cookie cookie;
	unsigned char byte, *p;
	char *header_tag= "LpId";
	unsigned short xsum;
	struct id_header hdr;
	struct id_checksums xsums;
	unsigned char *mem_ptr = data->mem_ptr;

	cookie.offset = 0;
	/* Data starts with the header, should be 'LpId' */
	for (i=0; i<4; ++i) {
		byte = id_fetch_byte(cookie.offset, &err);
		if (mem_ptr)
			mem_ptr[cookie.offset] = byte;
		hdr.signature[i] = byte;
		cookie.offset++;
		if (err != ID_EOK) {
			printk(KERN_DEBUG "%s[%u]\n", __FILE__, __LINE__);
			goto err_ret;
		}
		if (hdr.signature[i] != header_tag[i]) {
			printk(KERN_DEBUG "%s[%u]\n", __FILE__, __LINE__);
			err = ID_ENODEV;
			goto err_ret;
		}
	}

	/* First LE 8-bit value is ID format version */
	byte = id_fetch_byte(cookie.offset, &err);
	if (mem_ptr)
		mem_ptr[cookie.offset] = byte;
	hdr.id_fmt_ver = byte;
	cookie.offset++;
	
	/* Second LE 8-bit value is currently not used */
	byte = id_fetch_byte(cookie.offset, &err);
	if (mem_ptr)
		mem_ptr[cookie.offset] = byte;
	hdr.unused0 = byte;
	cookie.offset++;
	
	/* Next LE 16-bit value is length of data */
	byte = id_fetch_byte(cookie.offset, &err);
	if (mem_ptr)
		mem_ptr[cookie.offset] = byte;
	hdr.data_length = byte;
	cookie.offset++;

	byte = id_fetch_byte(cookie.offset, &err);
	if (mem_ptr)
		mem_ptr[cookie.offset] = byte;
	hdr.data_length |= byte << 8;
	cookie.offset++;
	
	/* Next LE 16-bit value is xsum of header */
	byte = id_fetch_byte(cookie.offset, &err);
	if (mem_ptr)
		mem_ptr[cookie.offset] = byte;
	xsums.header = byte;
	cookie.offset++;

	byte = id_fetch_byte(cookie.offset, &err);
	if (mem_ptr)
		mem_ptr[cookie.offset] = byte;
	xsums.header |= byte << 8;
	cookie.offset++;

	/* Checksum the header */
	xsum = 0;
	p = (unsigned char *)&hdr;
	for (i = 0; i < sizeof(hdr); ++i)
		crc_15_step(&xsum, p[i]);

	if (xsum != xsums.header) {
		printk(KERN_DEBUG "%s[%u] xsum: 0x%04x, xsums.header: 0x%04x\n", 
		        __FILE__, __LINE__, xsum, xsums.header);
		err = -ID_EL2NSYNC;
		goto err_ret;
	}

	/* Next LE 16-bit value is xsum of data */
	byte = id_fetch_byte(cookie.offset, &err);
	if (mem_ptr)
		mem_ptr[cookie.offset] = byte;
	xsums.data = byte;
	cookie.offset++;

	byte = id_fetch_byte(cookie.offset, &err);
	if (mem_ptr)
		mem_ptr[cookie.offset] = byte;
	xsums.data |= byte << 8;
	cookie.offset++;

	/* Checksum the data (next id_len bytes), must match xsums.data */
	xsum = 0;
	for (i = 0; i < hdr.data_length; ++i) {
		byte = id_fetch_byte(cookie.offset + i, &err);
		if (mem_ptr)
			mem_ptr[cookie.offset + i] = byte;
		if (err != ID_EOK) {
			printk(KERN_DEBUG "%s[%u]\n", __FILE__, __LINE__);
			goto err_ret;
		}
		crc_15_step(&xsum, byte);
	}
	if (xsum != xsums.data) {
		printk(KERN_DEBUG "%s[%u] xsum: 0x%04x, xsums.data: 0x%04x\n", 
		        __FILE__, __LINE__, xsum, xsums.data);
		err = -ID_EL2NSYNC;
		goto err_ret;
	}

	/* offset is now at the first byte of the root dictionary which
	   contains its span */
	data->root_offset = cookie.offset;
	data->root_size = extract_unsigned_pnum(&cookie, 5, &err);
	if (err != ID_EOK) {
		printk(KERN_DEBUG "%s[%u]\n", __FILE__, __LINE__);
		goto err_ret;
	}

	data->root_size += cookie.offset - data->root_offset;

#if 0
	printk(KERN_DEBUG "Data format version: %u\n", hdr.id_fmt_ver);	
#endif	
	return ID_EOK;

err_ret:

	/* Error return - make sure signature in SRAM is invalid */
	if (mem_ptr)
		mem_ptr[0] = 0;

	return err;
}

/*
 * Reset the cookie to cover the whole root dictionary
 */
int id_init_cookie(struct id_data *data, struct id_cookie *cookie)
{
	if (!cookie)
		return -ID_EINVAL;
	cookie->start_offset = data->root_offset;
	cookie->size = data->root_size;
	cookie->offset = cookie->start_offset;
	return ID_EOK;
}

unsigned int extract_unsigned_pnum(struct id_cookie *cookie, int start_bit, int *err)
{
	unsigned int value=0;
	unsigned int bit_offset=0;
	unsigned char bits;
	unsigned char ch;
	int oor;

	*err = ID_EOK;
	for (;;) {
		ch = id_fetch_byte(cookie->offset++, &oor);
		if (oor != ID_EOK) {
			*err = oor;
			printk(KERN_ERR "extract runs oor");
			return 0;
		}
		if (ch & (1<<(start_bit-1))) {
			/* more to go, accumulate bits */
			bits = ch & ((1<<(start_bit - 1)) - 1);
			value |= (bits << bit_offset);
			bit_offset += start_bit-1;
			start_bit = 8;
		} else {
			/* last byte of number */
			bits = ch & ((1<<(start_bit - 1)) - 1);
			value |= (bits << bit_offset);
			break;
		}
	}
	return value;
}

int extract_signed_pnum(struct id_cookie *cookie, int start_bit, int *err)
{
	int value=0;
	unsigned int bit_offset=0;
	unsigned char bits;
	unsigned char ch;
	int oor;

	*err = ID_EOK;
	for (;;) {
		ch = id_fetch_byte(cookie->offset++, &oor);
		if (oor != ID_EOK) {
			*err = oor;
			printk(KERN_ERR "extract runs oor");
			return 0;
		}
		if (ch & (1<<(start_bit-1))) {
			/* more to go, accumulate bits */
			bits = ch & ((1<<(start_bit - 1)) - 1);
			value |= (bits << bit_offset);
			bit_offset += start_bit-1;
			start_bit = 8;
		} else {
			/* last byte of number */
			bits = ch & ((1<<(start_bit - 2)) - 1);
			value |= (bits << bit_offset);
			if (ch & (1<<(start_bit - 2)))
				value = -value;
			break;
		}
	}
	return value;
}

int id_whatis(struct id_cookie *cookie, idenum_t *type)
{
	unsigned char byte;
	int oor;
	if (!cookie)
		return -ID_EINVAL;

	byte = id_fetch_byte(cookie->offset, &oor);
	if (oor != ID_EOK)
		return -ID_ERANGE;

	byte >>= 5;
	*type = (idenum_t)byte;
			
	return ID_EOK;
}

int id_extract_size(struct id_cookie *cookie, int *err)
{
	idenum_t type;
	struct id_cookie s_cookie;
	int size;

	s_cookie = *cookie;

	*err = id_whatis(&s_cookie, &type);
	if (*err != ID_EOK)
		return *err;

	switch(type) {
	case IDENUM_DICT:
		size = extract_unsigned_pnum(&s_cookie, 5, err);
		size += (s_cookie.offset - cookie->offset);
		break;
	case IDENUM_ARRAY:
		size = extract_unsigned_pnum(&s_cookie, 5, err);
		size += (s_cookie.offset - cookie->offset);
		break;
	case IDENUM_STR:
	case IDENUM_HEXSTR:
		size = extract_unsigned_pnum(&s_cookie, 5, err);
		size += (s_cookie.offset - cookie->offset);
		break;
	case IDENUM_POS_NUM:
	case IDENUM_NEG_NUM:
		extract_signed_pnum(&s_cookie, 5, err);
		size = (s_cookie.offset - cookie->offset);
		break;
	case IDENUM_KEY:
		extract_unsigned_pnum(&s_cookie, 5, err);
		size = (s_cookie.offset - cookie->offset);
		break;
	default:
		*err = -ID_EDOM;
		size = 0;
		break;
	}
	if (*err != ID_EOK)
		return *err;

	return size;
}

#if 0
static int id_dict_size(struct id_data *data, struct id_cookie *cookie)
{
	idenum_t type;
	int err;
	int count = 0;
	unsigned int size, keyval;
	struct id_cookie d_cookie;

	d_cookie = *cookie;

	/* It has to be a dictionary */
	err = id_whatis(&d_cookie, &type);
	if (type != IDENUM_DICT)
		return -ID_EINVAL;

	size = extract_unsigned_pnum(&d_cookie, 5, &err);
	if (err != ID_EOK)
		return err;

	d_cookie.size = size;
	d_cookie.start_offset = d_cookie.offset;
	while (d_cookie.offset < d_cookie.start_offset+d_cookie.size) {
		/* It has to be a key */
		err = id_whatis(&d_cookie, &type);
		if (type != IDENUM_KEY)
			return -ID_EINVAL;
		keyval = extract_unsigned_pnum(&d_cookie, 5, &err);
		if (err != ID_EOK)
			return err;
		
		/* Get the size of the object */
		size = id_extract_size(&d_cookie, &err);
		if (err != ID_EOK)
			return err;

		/* Move the offset forward by the object size */
		d_cookie.offset += size;

		/* Increment the count */
		count++;
	}
	return count;
}
#endif

static int id_extract_key(struct id_cookie *cookie, id_keys_t *key)
{
	int err;
	id_keys_t keyval;

	keyval = (id_keys_t)extract_unsigned_pnum(cookie, 5, &err);
	if (err != ID_EOK)
		return err;
	*key = keyval;
	return ID_EOK;
}

/* in dictionary that cookie points to find key "key"; if found
 * update cookie to associated "key" entry and return ID_EOK;
 * else return -ID_ENOENT */
static int id_dict_find_key(struct id_cookie *cookie, id_keys_t key)
{
	int err;
	unsigned int size;
	id_keys_t d_key;
	idenum_t type;
	struct id_cookie d_cookie = *cookie;
	struct id_cookie t_cookie;

	err = id_whatis(cookie, &type);
	if (err != ID_EOK)
		return err;

	/* Header has to be a dictionary */
	if (type != IDENUM_DICT)
		return -ID_EINVAL;

	/* Extract size of dictionary */
	size = extract_unsigned_pnum(&d_cookie, 5, &err);
	if (err != ID_EOK)
		return err;

	d_cookie.size = size;
	d_cookie.start_offset = d_cookie.offset;

	/* cookie->offset is now at first key */
	while (d_cookie.offset < d_cookie.start_offset + d_cookie.size) {
		/* Extract the key and move the cookie over key */
		err = id_extract_key(&d_cookie, &d_key);
		if (err != ID_EOK)
			return err;
		t_cookie = d_cookie;
		/* move forward over the value */
		size = id_extract_size(&d_cookie, &err);
		if (err != ID_EOK)
			return err;
		if (key == d_key) {
			d_cookie.size = size;
			d_cookie.start_offset = t_cookie.offset;
			d_cookie.offset = t_cookie.offset;
			*cookie = d_cookie;
			return ID_EOK;
		}
		d_cookie.offset += size;
	}
	return -ID_ENOENT;
}

/* Are these two types a match? */
static int id_match_type(idenum_t type_a, idenum_t type_b)
{
	idenum_t tmp;

	if (type_a == type_b)
		return 1;

	/* Oder the types (so the "*ANY*" types are in type_b) */
	if ((int)type_a > (int)type_b) {
		tmp = type_a;
		type_a = type_b;
		type_b = tmp;
	}
	if (type_b == IDENUM_ANY_STRING && (type_a == IDENUM_STR || type_a == IDENUM_HEXSTR))
		return 1;

	if (type_b == IDENUM_ANY_NUMBER && (type_a == IDENUM_NEG_NUM || type_a == IDENUM_POS_NUM))
		return 1;

	return 0;
}

/* Find in dictionary (that cookie points to) key "key" that is type "type" */
static int id_find_dict(struct id_cookie *cookie, id_keys_t key, idenum_t type)
{
	int err;
	struct id_cookie d_cookie = *cookie;
	idenum_t l_type;

	err = id_dict_find_key(&d_cookie, key);
	if (err != ID_EOK)
		return err;
	err = id_whatis(&d_cookie, &l_type);
	if (err != ID_EOK)
		return err;
	if (!id_match_type(l_type, type))
		return -ID_EINVAL;
	*cookie = d_cookie;
	return ID_EOK;
}

/* in dictionary pointed at by cookie, find the key "key"; verify its a
 * string and copy its value */
static int id_find_string(struct id_cookie *cookie, id_keys_t key, unsigned char *str_ptr, unsigned int *str_size)
{
	int err, i;
	unsigned char byte;
	unsigned int size;
	struct id_cookie d_cookie = *cookie;

	err = id_find_dict(&d_cookie, key, IDENUM_ANY_STRING);

	if (err != ID_EOK)
		return err;
	/* Extract the string size */
	size = extract_unsigned_pnum(&d_cookie, 5, &err);
	if (err != ID_EOK)
		return err;

	if (size > *str_size)
		return -ID_ERANGE;

	for(i=0; i<size; ++i) {
		byte = id_fetch_byte(d_cookie.offset++, &err);
		if (err)
			return err;
		str_ptr[i] = byte;
	}
	*str_size = size;

	return ID_EOK;
}

/* in dictionary pointed at by cookie, find the key "key"; verify its a
 * number (either pos/neg) and return its value through *num */
static int id_find_number(struct id_cookie *cookie, id_keys_t key, int *num)
{
	int err;
	int l_num;
	idenum_t l_type;
	struct id_cookie d_cookie = *cookie;

	err = id_find_dict(&d_cookie, key, IDENUM_ANY_NUMBER);

	if (err != ID_EOK)
		return err;
	err = id_whatis(&d_cookie, &l_type);
	if (err != ID_EOK)
		return err;
	/* Extract the number size */
	l_num = extract_unsigned_pnum(&d_cookie, 5, &err);
	if (err != ID_EOK)
		return err;

	if (l_type == IDENUM_NEG_NUM)
		l_num = -l_num;

	*num = l_num;
	return ID_EOK;
}

/* in dictionary pointed at by cookie, find the list of keys; verify they are
 * numbers (either pos/neg) and return their value through *nums */
static int id_find_numbers(struct id_cookie *cookie, id_keys_t *keys, int key_size, int *nums)
{
	int i, err;
	struct id_cookie d_cookie;

	for (i=0;i<key_size; ++i) {
		d_cookie = *cookie;
		err = id_find_number(&d_cookie, keys[i], &nums[i]);
		if (err != ID_EOK)
			return err;
	}
	return ID_EOK;
}

/* --------------------------------------------------------- */

/*
 * Here down is the code to interface to the kernel to extract product
 * ID information from the SRAM/AT24 chip.
 */

struct id_data id_data;
static int found_id_data;
static struct id_cookie serialization_group_cookie;
static struct id_cookie model_group_cookie;

static int omap3logic_find_model_group_cookie(struct id_cookie *mg_cookie)
{
	int ret;
	struct id_cookie cookie;

	if (!found_id_data) {
		return -1;
	}

	if (model_group_cookie.offset) {
		*mg_cookie = model_group_cookie;
		return ID_EOK;
	}

	/* Reinitialise cookie back to the root */
	ret = id_init_cookie(&id_data, &cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	/* find /model_group from root */
	ret = id_find_dict(&cookie, ID_KEY_model_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	model_group_cookie = cookie;
	*mg_cookie = cookie;
	return ret;
}

static int omap3logic_find_serialization_cookie(struct id_cookie *s_cookie)
{
	int ret;
	struct id_cookie cookie;

	if (!found_id_data) {
		return -1;
	}

	if (serialization_group_cookie.offset) {
		*s_cookie = serialization_group_cookie;
		return ID_EOK;
	}

	ret = id_init_cookie(&id_data, &cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	/* find /serialization_group from root */
	ret = id_find_dict(&cookie, ID_KEY_serialization_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	serialization_group_cookie = cookie;
	*s_cookie = cookie;
	return ID_EOK;
}

int omap3logic_extract_new_part_number(u32 *part_number)
{
	int ret;
	struct id_cookie cookie;

	ret = omap3logic_find_model_group_cookie(&cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	/* Find part number */
	ret = id_find_number(&cookie, ID_KEY_part_number, part_number);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}
	return ret;
}

static int omap3logic_extract_new_model_name(char *model_name, u32 *model_name_size)
{
	int ret;
	struct id_cookie cookie;

	ret = omap3logic_find_model_group_cookie(&cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	ret = id_find_string(&cookie, ID_KEY_model_name, model_name, model_name_size);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}
	return ret;
}

int omap3logic_extract_new_serial_number(u8 *serial_number, u32 *serial_number_size)
{
	int ret;
	struct id_cookie cookie;

	ret = omap3logic_find_serialization_cookie(&cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	/* Find serial_number */
	ret = id_find_string(&cookie, ID_KEY_serial_number, serial_number, serial_number_size);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}
	return ret;
}

int omap3logic_extract_new_nvs_data(u8 *nvs_data, u32 *nvs_data_size)
{
	int ret;
	struct id_cookie cookie;

	ret = omap3logic_find_serialization_cookie(&cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	/* Find serial_number */
	ret = id_find_string(&cookie, ID_KEY_nvs, nvs_data, nvs_data_size);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}
	return ret;
}

static int valid_product_id_lan_ethaddr;  // !0 if LAN ethaddr is good
static int valid_product_id_wifi_ethaddr;  // !0 if LAN ethaddr is good
static int valid_product_id_has_wifi_config_data;  // !0 if has Murata

int omap3logic_extract_new_lan_ethaddr(u8 *ethaddr);
int omap3logic_extract_new_wifi_ethaddr(u8 *ethaddr);


int logic_dump_serialization_info(void)
{
	u8 ethaddr[6];
	int ret;
	struct id_cookie cookie;
	int part_number;
	u8 model_name[32];
	u32 model_name_size;
	u8 serial_number[10];
	u32 serial_number_size;

	ret = omap3logic_find_serialization_cookie(&cookie);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	serial_number_size = sizeof(serial_number);
	ret = omap3logic_extract_new_serial_number(serial_number, &serial_number_size);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	ret = omap3logic_extract_new_part_number(&part_number);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}


	/* Find model name */
	model_name_size = sizeof(model_name);
	ret = omap3logic_extract_new_model_name(model_name, &model_name_size);
	if (ret != ID_EOK) {
		printk(KERN_ERR "%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	if (omap3logic_wl12xx_exists()) {
		valid_product_id_has_wifi_config_data = 1;
	}

	printk(KERN_INFO "Part Number  : %u\n", part_number);
	printk(KERN_INFO "Model Name   : %.*s\n", model_name_size, model_name);
	printk(KERN_INFO "Serial Number: %.*s\n", serial_number_size, serial_number);
	ret = omap3logic_extract_new_lan_ethaddr(ethaddr);
	if (ret == ID_EOK) {
		printk(KERN_INFO "LAN ethaddr  : %02x:%02x:%02x:%02x:%02x:%02x\n",
			ethaddr[0], ethaddr[1], ethaddr[2],
			ethaddr[3], ethaddr[4], ethaddr[5]);
		valid_product_id_lan_ethaddr = 1;
	}
	ret = omap3logic_extract_new_wifi_ethaddr(ethaddr);
	if (ret == ID_EOK) {
		printk(KERN_INFO "WLAN ethaddr : %02x:%02x:%02x:%02x:%02x:%02x\n",
			ethaddr[0], ethaddr[1], ethaddr[2],
			ethaddr[3], ethaddr[4], ethaddr[5]);
		valid_product_id_wifi_ethaddr = 1;
	}
	return 0;
}

/* Extract GPMC timings for particular CS register */
id_keys_t gpmc_ncs_keys[] = {
	ID_KEY_cs0_group,
	ID_KEY_cs1_group,
	ID_KEY_cs2_group,
	ID_KEY_cs3_group,
	ID_KEY_cs4_group,
	ID_KEY_cs5_group,
	ID_KEY_cs6_group,
};

id_keys_t gpmc_config_reg_keys[] = {
	ID_KEY_config1_reg,
	ID_KEY_config2_reg,
	ID_KEY_config3_reg,
	ID_KEY_config4_reg,
	ID_KEY_config5_reg,
	ID_KEY_config6_reg,
	ID_KEY_config7_reg,
};

int logic_extract_gpmc_timing(int cs, int *config_regs)
{
	int ret;
	struct id_cookie cookie;
	// int gpmc_config_values[ARRAY_SIZE(gpmc_config_reg_keys)];

	if (!found_id_data)
		return -1;

	ret = id_init_cookie(&id_data, &cookie);
	if (ret != ID_EOK) {
		return ret;
	}

	/* find /cpu0_bus_group from root */
	ret = id_find_dict(&cookie, ID_KEY_cpu0_bus_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		return ret;
	}

	/* find /local_bus_group from /cpu0_bus_group */
	ret = id_find_dict(&cookie, ID_KEY_local_bus_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		return ret;
	}

	/* Now look for the particular chip select group */
	ret = id_find_dict(&cookie, gpmc_ncs_keys[cs], IDENUM_DICT);
	if (ret != ID_EOK) {
		return ret;
	}

	/* We have the group, now extract all the config registers */
	ret = id_find_numbers(&cookie, gpmc_config_reg_keys, ARRAY_SIZE(gpmc_config_reg_keys), config_regs);

	return ret;
}

/* Initialize the product ID data and return 0 if found */
static int product_id_init(void)
{
	int ret;

	memset(&id_data, 0, sizeof(id_data));

	ret = id_startup(&id_data);
	if (ret != ID_EOK) {
		return -1;
	}

	return 0;
}

int logic_has_new_product_id(void)
{
	if (!found_id_data) {
		if (!product_id_init()) {
			found_id_data = 1;
		}
	}
	return found_id_data;
}

int omap3logic_fetch_sram_new_product_id_data(void)
{
	if (!logic_has_new_product_id()) {
#if 0
		printk(KERN_INFO "U-boot provided product_id data (new format) is invalid\n");
#endif
		return -ENOENT;
	}

	printk(KERN_INFO "U-boot Production Data (new format) is valid\n");

	return logic_dump_serialization_info();
}

/* Extract the Wired LAN ethaddr, and return !0 if its valid */
int omap3logic_extract_new_lan_ethaddr(u8 *ethaddr)
{
	int ret;
	struct id_cookie cookie;
	int ethaddr_size;

	if (!found_id_data) {
		ret = -ENXIO;
		printk("%s:%d\n", __FUNCTION__, __LINE__);
		goto done;
	}

	ret = id_init_cookie(&id_data, &cookie);
	if (ret != ID_EOK) {
		printk("%s:%d\n", __FUNCTION__, __LINE__);
		goto done;
	}

	/* Find /serialization_group */
	ret = id_find_dict(&cookie, ID_KEY_serialization_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		printk("%s:%d\n", __FUNCTION__, __LINE__);
		goto done;
	}

	/* Find /lan_ethaddr1 */
	ethaddr_size = 6;
	ret = id_find_string(&cookie, ID_KEY_lan_ethaddr1, ethaddr, &ethaddr_size);
	if (ret != ID_EOK) {
		goto done;
	}
	if (ethaddr_size != 6) {
		ret = -E2BIG;
		printk("%s:%d ethaddr_size %u\n", __FUNCTION__, __LINE__, ethaddr_size);
		goto done;
	}
	ret = 0;
done:
	return ret;
}

/* Extract the WiFi ethaddr, and return !0 if its valid */
int omap3logic_extract_new_wifi_ethaddr(u8 *ethaddr)
{
	int ret;
	struct id_cookie cookie;
	int ethaddr_size;

	if (!found_id_data) {
		ret = -ENXIO;
		printk("%s:%d\n", __FUNCTION__, __LINE__);
		goto done;
	}

	ret = id_init_cookie(&id_data, &cookie);
	if (ret != ID_EOK) {
		printk("%s:%d\n", __FUNCTION__, __LINE__);
		goto done;
	}

	/* Find /serialization_group */
	ret = id_find_dict(&cookie, ID_KEY_serialization_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		printk("%s:%d\n", __FUNCTION__, __LINE__);
		goto done;
	}

	/* Find /lan_ethaddr2 */
	ethaddr_size = 6;
	ret = id_find_string(&cookie, ID_KEY_wifi_ethaddr1, ethaddr, &ethaddr_size);
	if (ret != ID_EOK) {
		goto done;
	}
	if (ethaddr_size != 6) {
		ret = -E2BIG;
		printk("%s:%d ethadr_size %d\n", __FUNCTION__, __LINE__, ethaddr_size);
		goto done;
	}

	ret = 0;
done:
	return ret;
}


static ssize_t product_id_show_wifi_macaddr(struct class *class, struct class_attribute *attr, char *buf)
{
	u8 macaddr[6];
	int ret;

	ret = omap3logic_extract_new_wifi_ethaddr(macaddr);
	if (!ret)
		return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
			macaddr[0], macaddr[1], macaddr[2],
			macaddr[3], macaddr[4], macaddr[5]);
	return ret;
}

static ssize_t product_id_show_lan_macaddr(struct class *class, struct class_attribute *attr, char *buf)
{
	u8 macaddr[6];
	int ret;

	ret = omap3logic_extract_new_lan_ethaddr(macaddr);
	if (!ret)
		return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
			macaddr[0], macaddr[1], macaddr[2],
			macaddr[3], macaddr[4], macaddr[5]);
	return ret;
}

static ssize_t product_id_show_part_number(struct class *class, struct class_attribute *attr, char *buf)
{
	u32 part_number;
	int len;
	omap3logic_extract_new_part_number(&part_number);

	len = sprintf(buf, "%d\n", part_number);
	return len;
}

static ssize_t product_id_show_model_name(struct class *class, struct class_attribute *attr, char *buf)
{
	u32 model_name_size = 128;
	int ret;

	ret = omap3logic_extract_new_model_name((u8 *)buf, &model_name_size);

	buf[model_name_size] = '\n';
	return model_name_size + 1;
}

static ssize_t product_id_show_serial_number(struct class *class, struct class_attribute *attr, char *buf)
{
	u32 serial_number_size = 128;

	omap3logic_extract_new_serial_number((u8 *)buf, &serial_number_size);
	buf[serial_number_size] = '\n';
	return serial_number_size + 1;
}

static ssize_t product_id_show_wifi_config_data(struct class *class, struct class_attribute *attr, char *buf)
{
	u32 wifi_config_size = PAGE_SIZE;
	int ret;

	ret = omap3logic_extract_new_nvs_data(buf, &wifi_config_size);

	if (ret == ID_EOK)
		return wifi_config_size;

	return ret;
}



#define DECLARE_CLASS_ATTR(_name,_mode,_show,_store)                  \
{                                                               \
	.attr   = { .name = __stringify(_name), .mode = _mode },	\
		.show   = _show,                                        \
		.store  = _store,				\
}

static struct {
	struct class_attribute attr;
	int *test_value;
} product_id_class_attributes[] = {
	{
		__ATTR(lan_macaddr, S_IRUGO, product_id_show_lan_macaddr, NULL),
		&valid_product_id_lan_ethaddr,
	},
	{
		__ATTR(wifi_macaddr, S_IRUGO, product_id_show_wifi_macaddr, NULL),
		&valid_product_id_wifi_ethaddr,
	},
	{
		__ATTR(part_number, S_IRUGO, product_id_show_part_number, NULL),
		NULL,
	},
	{
		__ATTR(model_name, S_IRUGO, product_id_show_model_name, NULL),
		NULL,
	},
	{
		__ATTR(serial_number, S_IRUGO, product_id_show_serial_number, NULL),
		NULL,
	},
	{
		__ATTR(wifi_config_data, S_IRUGO, product_id_show_wifi_config_data, NULL),
		&valid_product_id_has_wifi_config_data,
	},
};

static void product_id_dev_release(struct device *dev)
{
}

static struct class product_id_class = {
	.name = "product_id",
	.dev_release = product_id_dev_release,
};

int omap3logic_create_new_product_id_sysfs(void)
{
	int i, rc;

	rc = class_register(&product_id_class);
	if (rc != 0) {
		printk("%s: failed to register product_id class\n", __FUNCTION__);
		return rc;
	}

	for (i=0; i<ARRAY_SIZE(product_id_class_attributes); ++i) {
		if (!product_id_class_attributes[i].test_value || *product_id_class_attributes[i].test_value) {
			rc = class_create_file(&product_id_class, &product_id_class_attributes[i].attr);
			if (unlikely(rc)) {
				printk("%s: failed to create product_id class file\n", __FUNCTION__);
				while (--i >= 0) { 
					if (!product_id_class_attributes[i].test_value || *product_id_class_attributes[i].test_value) {
						class_remove_file(&product_id_class, &product_id_class_attributes[i].attr);
					}
				}
				class_unregister(&product_id_class);
				return -EPERM;
			}
		}
	}

	return 0;
}
