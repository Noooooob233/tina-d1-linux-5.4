/*
 * drivers/soc/sunxi-sid.c
 *
 * Copyright(c) 2014-2016 Allwinnertech Co., Ltd.
 *         http://www.allwinnertech.com
 *
 * Author: sunny <sunny@allwinnertech.com>
 * Author: superm <superm@allwinnertech.com>
 * Author: Matteo <duanmintao@allwinnertech.com>
 *
 * Allwinner sunxi soc chip version and chip id manager.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/err.h>
#include <linux/sunxi-smc.h>

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>

#include <linux/sunxi-sid.h>

#define SID_DBG(fmt, arg...) pr_debug("%s()%d - "fmt, __func__, __LINE__, ##arg)
#define SID_WARN(fmt, arg...) pr_warn("%s()%d - "fmt, __func__, __LINE__, ##arg)
#define SID_ERR(fmt, arg...) pr_err("%s()%d - "fmt, __func__, __LINE__, ##arg)

#if defined(CONFIG_ARCH_SUN50I) || defined(CONFIG_ARCH_SUN8IW6)
#define SUNXI_SECURITY_SUPPORT	1
#endif

#define SUNXI_VER_MAX_NUM	8
struct soc_ver_map {
	u32 id;
	u32 rev[SUNXI_VER_MAX_NUM];
};

#define SUNXI_SOC_ID_INDEX		1
#define SUNXI_SECURITY_ENABLE_INDEX	2
struct soc_ver_reg {
	s8 compatile[48];
	u32 offset;
	u32 mask;
	u32 shift;
	struct soc_ver_map ver_map;
};

#define SUNXI_SOC_ID_IN_SID

#if defined(CONFIG_ARCH_SUN50IW6)
#define TYPE_SB (0b001)
#define TYPE_NB (0b010)
#define TYPE_FB (0b011)
#else
#define TYPE_SB (0b001)
#define TYPE_NB (0b011)
#define TYPE_FB (0b111)
#endif

static unsigned int sunxi_soc_chipid[4];
static unsigned int sunxi_soc_ftzone[4];
static unsigned int sunxi_serial[4];
static int sunxi_soc_secure;
static unsigned int sunxi_soc_bin;
static unsigned int sunxi_soc_ver;
static unsigned int sunxi_soc_rotpk_status;

#ifndef CONFIG_SUNXI_SMC
u32 sunxi_smc_readl(phys_addr_t addr)
{
	void __iomem *vaddr = ioremap(addr, 4);
	u32 val;

	val = readl(vaddr);
	iounmap(vaddr);
	return val;
}
#endif

static s32 sid_get_vir_base(struct device_node **pnode, void __iomem **base,
		s8 *compatible)
{
	*pnode = of_find_compatible_node(NULL, NULL, compatible);
	if (IS_ERR_OR_NULL(*pnode)) {
		SID_ERR("Failed to find \"%s\" in dts.\n", compatible);
		return -ENXIO;
	}

	*base = of_iomap(*pnode, 0); /* reg[0] must be accessible. */
	if (*base == NULL) {
		SID_ERR("Unable to remap IO\n");
		return -ENXIO;
	}
	SID_DBG("Base addr of \"%s\" is %p\n", compatible, *base);
	return 0;
}

static s32  sid_get_phy_base(struct device_node **pnode, phys_addr_t **base,
		s8 *compatible)
{
	struct resource res = {0};
	int ret;
	*pnode = of_find_compatible_node(*pnode, NULL, compatible);
	if (IS_ERR_OR_NULL(*pnode)) {
		SID_ERR("Failed to find \"%s\" in dts.\n", compatible);
		return -ENXIO;
	}

	ret = of_address_to_resource(*pnode, 0, &res);
	if (ret) {
		SID_ERR("ret:%d Failed to get \"%s\"  base address\n", ret, compatible);
		return -ENXIO;
	}
	*base = (phys_addr_t *)res.start;
	SID_DBG("Base addr of \"%s\" is %p\n", compatible, (void *)*base);
	return 0;
}

static s32 sid_get_base(struct device_node **pnode,
		void __iomem **base, s8 *compatible, u32 sec)
{
	if (sec == 1)
		return sid_get_phy_base(pnode,
			(phys_addr_t **)base, compatible);
	else
		return sid_get_vir_base(pnode, base, compatible);
}

static void sid_put_base(struct device_node *pnode, void __iomem *base, u32 sec)
{
	SID_DBG("base = %p, Sec = %d\n", base, sec);
	if ((sec == 0) && (base != NULL))
		iounmap(base);
	if (pnode)
		of_node_put(pnode);
}

static u32 sid_readl(void __iomem *base, u32 sec)
{
	if (sec == 0)
		return readl(base);
	else
		return sunxi_smc_readl((phys_addr_t)base);
}

int get_key_map_info(s8 *name, u8 *compatile, u32 *offset, u32 *max_size)
{
	struct device_node *child_pnode;
	struct device_node *pnode = of_find_compatible_node(NULL, NULL, compatile);
	if (IS_ERR_OR_NULL(pnode)) {
		SID_ERR("Failed to find \"%s\" in dts.\n", compatile);
		return -ENXIO;
	}
	child_pnode = of_get_child_by_name(pnode, name);
	if (IS_ERR_OR_NULL(child_pnode)) {
		SID_ERR("Failed to find \"%s\" in dts.\n", name);
		return -ENXIO;
	}
	of_property_read_u32(child_pnode, "offset", offset);
	of_property_read_u32(child_pnode, "size", max_size);
	return 0;
}


static u32 sid_read_key(s8 *key_name, u32 *key_buf, u32 key_size, u32 sec)
{
	u32 i, offset = 0, max_size = 0;
	void __iomem *baseaddr = NULL;
	struct device_node *dev_node = NULL;

	if (sid_get_base(&dev_node, &baseaddr, EFUSE_SID_BASE, sec))
		return 0;

	get_key_map_info(key_name, EFUSE_SID_BASE, &offset, &max_size);
	SID_DBG("key_name:%s offset:0x%x max_size:0x%x\n", key_name, offset, max_size);
	if (key_size > max_size) {
		key_size = max_size;
	}
	for (i = 0; i < key_size; i += 4) {
		key_buf[i/4] = sid_readl(baseaddr + offset + i, sec);
	}

	sid_put_base(dev_node, baseaddr, sec);
	return 0;
}


static u32 sid_rd_bits(s8 *name, u32 offset, u32 shift, u32 mask, u32 sec)
{
	u32 value = 0;
	void __iomem *baseaddr = NULL;
	struct device_node *dev_node = NULL;

#ifdef SID_REG_READ
	return __sid_reg_read_key(offset);
#else
	if (sid_get_base(&dev_node, &baseaddr, name, sec))
		return 0;

	value = sid_readl(baseaddr + offset, sec);

	value = (value >> shift) & mask;
	SID_DBG("Read \"%s\" + %#x, shift %#x, mask %#x, return %#x, Sec %d\n",
			name, offset, shift, mask, value, sec);

	sid_put_base(dev_node, baseaddr, sec);
	return value;
#endif
}

int get_soc_ver_regs(u8 *name, u8 *compatile, struct soc_ver_reg *reg)
{
	struct device_node *child_pnode;
	struct device_node *pnode = of_find_compatible_node(NULL, NULL, compatile);
	if (IS_ERR_OR_NULL(pnode)) {
		SID_ERR("Failed to find \"%s\" in dts.\n", SRAM_CTRL_BASE);
		return -ENXIO;
	}
	child_pnode = of_get_child_by_name(pnode, name);
	if (IS_ERR_OR_NULL(child_pnode)) {
		SID_ERR("Failed to find \"%s\" in dts.\n", name);
		return -ENXIO;
	}

	of_property_read_u32(child_pnode, "offset", &reg->offset);
	of_property_read_u32(child_pnode, "shift", &reg->shift);
	of_property_read_u32(child_pnode, "mask", &reg->mask);
	of_property_read_u32(child_pnode, "ver_a", &reg->ver_map.rev[0]);
	of_property_read_u32(child_pnode, "ver_b", &reg->ver_map.rev[1]);
	return 0;
}

void sid_rd_ver_reg(u32 id)
{
	s32 i = 0;
	u32 ver = 0;
	static struct soc_ver_reg reg = {0};
	get_soc_ver_regs("soc_ver", SRAM_CTRL_BASE, &reg);
	ver = sid_rd_bits(SRAM_CTRL_BASE, reg.offset,
		reg.shift, reg.mask, 0);
	if (ver >= SUNXI_VER_MAX_NUM/2)
		SID_WARN("ver >= %d, soc ver:%d\n", SUNXI_VER_MAX_NUM/2, ver);

	sunxi_soc_ver = reg.ver_map.rev[0] + ver;

	SID_DBG("%d-%d: soc_ver %#x\n", i, ver, sunxi_soc_ver);
}

static s32 sid_rd_soc_ver_from_sid(void)
{
	u32 id = 0;
	static struct soc_ver_reg reg = {0};
	get_soc_ver_regs("soc_id", SRAM_CTRL_BASE, &reg);
	id = sid_rd_bits(EFUSE_SID_BASE, reg.offset, reg.shift, reg.mask, 0);
	sid_rd_ver_reg(id);

	return 0;
}

static void sid_soc_ver_init(void)
{
	static s32 init_flag;

	if (init_flag == 1) {
		SID_DBG("It's already inited.\n");
		return;
	}

	sid_rd_soc_ver_from_sid();

	SID_DBG("The SoC version: %#x\n", sunxi_soc_ver);
	init_flag = 1;
}


static void sid_chipid_init(void)
{
	u32 type = 0, offset = 0, max_size;
	static s32 init_flag;
	static struct soc_ver_reg reg = {0};

	if (init_flag == 1) {
		SID_DBG("It's already inited.\n");
		return;
	}
	sid_read_key("chipid", sunxi_soc_chipid, 16, sunxi_soc_is_secure());

	sunxi_serial[0] = sunxi_soc_chipid[3];
	sunxi_serial[1] = sunxi_soc_chipid[2];
	sunxi_serial[2] = (sunxi_soc_chipid[1] >> 16) & 0x0FFFF;

	get_key_map_info("chipid", EFUSE_SID_BASE, &offset, &max_size);
	get_soc_ver_regs("soc_bin", SRAM_CTRL_BASE, &reg);

	type = sid_rd_bits(EFUSE_SID_BASE, reg.offset + offset, reg.shift,
		reg.mask, sunxi_soc_is_secure());

	switch (type) {
	case 0b000001:
		sunxi_soc_bin = 1;
		break;
	case 0b000011:
		sunxi_soc_bin = 2;
		break;
	case 0b000111:
		sunxi_soc_bin = 3;
		break;
	default:
		break;
	}
	SID_DBG("soc bin: %d\n", sunxi_soc_bin);

	init_flag = 1;
}

void sid_ft_zone_init(void)
{
	static s32 init_flag;
	if (init_flag == 1) {
		SID_DBG("It's already inited.\n");
		return;
	}
	sid_read_key(EFUSE_FT_ZONE_NAME, sunxi_soc_ftzone, 0x10, sunxi_soc_is_secure());

	init_flag = 1;

}

void sid_rd_soc_secure_status(void)
{
#if defined(CONFIG_TEE) && \
	(defined(CONFIG_ARCH_SUN8IW7) || defined(CONFIG_ARCH_SUN8IW6))
	sunxi_soc_secure = 1;
#else
	static s32 init_flag;
	void __iomem *base = NULL;
	struct device_node *node = NULL;
	u32 offset = 0, max_size;

	if (init_flag == 1) {
		SID_DBG("It's already inited.\n");
		return;
	}

	if (sid_get_base(&node, &base, EFUSE_SID_BASE, 1))
		return;

	get_key_map_info("secure_status", EFUSE_SID_BASE, &offset, &max_size);

#ifdef CONFIG_ARCH_SUN20IW1
	sunxi_soc_secure = (((sunxi_smc_readl((phys_addr_t)(base + offset))) >> 31) & 0x1);
#else
	sunxi_soc_secure = ((sunxi_smc_readl((phys_addr_t)(base + offset))) & 0x1);
#endif

	sid_put_base(node, base, 1);
	init_flag = 1;
#endif
}

void sid_rotpk_status_init(void)
{
	static s32 init_flag;
	if (init_flag == 1) {
		SID_DBG("It's already inited.\n");
		return;
	}
	sid_read_key(EFUSE_ROTPK_NAME, &sunxi_soc_rotpk_status, 4, sunxi_soc_is_secure());

	init_flag = 1;

}

s32 sunxi_get_platform(s8 *buf, s32 size)
{
	return snprintf(buf, size, "%s", CONFIG_SUNXI_SOC_NAME);
}
EXPORT_SYMBOL(sunxi_get_platform);

/**
 * soc chipid:
 */
int sunxi_get_soc_chipid(u8 *chipid)
{
	sid_chipid_init();
	memcpy(chipid, sunxi_soc_chipid, 16);
	return 0;
}
EXPORT_SYMBOL(sunxi_get_soc_chipid);

/**
 * soc chipid serial:
 */
int sunxi_get_serial(u8 *serial)
{
	sid_chipid_init();
	memcpy(serial, sunxi_serial, 16);
	return 0;
}
EXPORT_SYMBOL(sunxi_get_serial);

/**
 * get module_param:
 * argc[0]---dst buf
 * argc[1]---the sid offset
 * argc[2]---len(btye)
 */
int sunxi_get_module_param_from_sid(u32 *dst, u32 offset, u32 len)
{
	void __iomem *baseaddr = NULL;
	struct device_node *dev_node = NULL;
	int i;

	if (dst == NULL) {
		pr_err("the dst buf is NULL\n");
		return -1;
	}

	if (len & 0x3) {
		pr_err("the len must be word algin\n");
		return -2;
	}

	if (sid_get_base(&dev_node, &baseaddr, EFUSE_SID_BASE, 0)) {
		pr_err("sid_get_base fail \n");
		return 0;
	}

	SID_DBG("baseaddr: 0x%p offset:0x%x len(word):0x%x\n", baseaddr, offset, len);

	for (i = 0; i < len; i += 4) {
		dst[i] = sid_readl(baseaddr + 0x200 + offset + i, 0);
	}

	sid_put_base(dev_node, baseaddr, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_get_module_param_from_sid);



/**
 * soc chipid str:
 */
int sunxi_get_soc_chipid_str(char *serial)
{
	size_t size;

	sid_chipid_init();
#if defined(CONFIG_ARCH_SUN50IW9) || defined(CONFIG_ARCH_SUN50IW10)
	size = sprintf(serial, "%08x", sunxi_soc_chipid[0] & 0xffff);
#else
	size = sprintf(serial, "%08x", sunxi_soc_chipid[0] & 0x0ff);
#endif
	return size;
}
EXPORT_SYMBOL(sunxi_get_soc_chipid_str);

/**
 * soc ft zone str:
 */
int sunxi_get_soc_ft_zone_str(char *serial)
{
	size_t size;

	sid_ft_zone_init();
	size = sprintf(serial, "%08x", (sunxi_soc_ftzone[0] & 0xff000000) >> 24);
	return size;
}
EXPORT_SYMBOL(sunxi_get_soc_ft_zone_str);

/**
 * soc rotpk status str:
 */
int sunxi_get_soc_rotpk_status_str(char *status)
{
	size_t size;

	sid_rotpk_status_init();
	size = sprintf(status, "%d", (sunxi_soc_rotpk_status & 0x3) >> 1);
	return size;
}
EXPORT_SYMBOL(sunxi_get_soc_rotpk_status_str);

/**
 * soc chipid:
 */
int sunxi_soc_is_secure(void)
{
	sid_rd_soc_secure_status();
	return sunxi_soc_secure;
}
EXPORT_SYMBOL(sunxi_soc_is_secure);

/**
 * get sunxi soc bin
 *
 * return: the bin of sunxi soc, like that:
 * 0 : fail
 * 1 : slow
 * 2 : normal
 * 3 : fast
 */
unsigned int sunxi_get_soc_bin(void)
{
	sid_chipid_init();
	return sunxi_soc_bin;
}
EXPORT_SYMBOL(sunxi_get_soc_bin);

unsigned int sunxi_get_soc_ver(void)
{
	sid_soc_ver_init();
	return sunxi_soc_ver;
}
EXPORT_SYMBOL(sunxi_get_soc_ver);

s32 sunxi_efuse_readn(s8 *key_name, void *buf, u32 n)
{
	char name[32] = {0};

	if ((key_name == NULL) || (*(s8 *)key_name == 0)
			|| (n == 0) || (buf == NULL)) {
		SID_ERR("Invalid parameter. name: %p, read_buf: %p, size: %d\n",
		key_name, buf, n);
		return -EINVAL;
	}
	WARN_ON(n < 4);

	strncpy(name, key_name, strlen(key_name) - 1);
	sid_read_key(name, buf, n, sunxi_soc_is_secure());
	return 0;
}
EXPORT_SYMBOL(sunxi_efuse_readn);

static int __init sunxi_sid_init(void)
{
	SID_WARN("insmod ok\n");
	return 0;
}

static void __exit sunxi_sid_exit(void)
{
	SID_WARN("rmmod ok\n");
}

module_init(sunxi_sid_init);
module_exit(sunxi_sid_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("weidonghui<weidonghui@allwinnertech.com>");
MODULE_DESCRIPTION("sunxi sid.");
