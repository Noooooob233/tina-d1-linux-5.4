// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020, allwinnertech.
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/clk/sunxi.h>
#include <linux/regulator/consumer.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <uapi/linux/dsp/dsp_debug.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/dma.h>

#define DSP_DEBUG_DEV_NAME	"dsp_debug"
#define DSP_DEV_NUM		(1)

#define DSP_ERR(fmt, arg...)	pr_err("[linux:dsp:err]- fun:%s() line:%d - "fmt, __func__, __LINE__, ##arg)
#define DSP_INFO(fmt, arg...)	pr_info("[linux:dsp:info]-"fmt, ##arg)

struct dsp_sharespace_t *dsp_sharespace;

static int32_t check_addr_valid(uint32_t addr)
{
	int ret = 0;
	if ((addr & (~0xFFFU)) != addr)
		ret = -1;

	return ret;
}

static int sharespace_check_addr(struct dsp_sharespace_t *msg)
{
	int ret = 1;
	ret = check_addr_valid(msg->dsp_write_addr);
	if (ret < 0) {
		DSP_ERR("dsp_write_addr fail to check\n");
		return ret;
	}

	ret = check_addr_valid(msg->dsp_write_addr + msg->dsp_write_size);
	if (ret < 0) {
		DSP_ERR("dsp_write_size fail to check\n");
		return ret;
	}

	ret = check_addr_valid(msg->arm_write_addr);
	if (ret < 0) {
		DSP_ERR("arm_write_addr fail to check\n");
		return ret;
	}

	ret = check_addr_valid(msg->arm_write_addr + msg->arm_write_size);
	if (ret < 0) {
		DSP_ERR("dsp_write_size fail to check\n");
		return ret;
	}

	ret = check_addr_valid(msg->dsp_log_addr);
	if (ret < 0) {
		DSP_ERR("dsp_log_addr fail to check\n");
		return ret;
	}

	ret = check_addr_valid(msg->dsp_log_addr + msg->dsp_log_size);
	if (ret < 0) {
		DSP_ERR("dsp_log_size fail to check\n");
		return ret;
	}

    return ret;
}

struct debug_dsp_dev_t{
	dev_t devid;
	struct cdev cdev;
	struct class *class;
	struct device *device;
	int major;
};

struct debug_dsp_dev_t *debug_dsp_dev;

static int debug_dsp_dev_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int debug_dsp_dev_mmap(struct file *file, struct vm_area_struct *vma)
{
	vma->vm_flags |= VM_IO;
	/* addr 4K align */
	vma->vm_pgoff = dsp_sharespace->mmap_phy_addr >> PAGE_SHIFT;
	if (remap_pfn_range(vma,
			vma->vm_start,
			vma->vm_pgoff,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot)) {
		DSP_ERR("sharespace mmap fail\n");
		return -EAGAIN;
	}
	return 0;
}

static long debug_dsp_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 1;
	switch (cmd) {
	case CMD_REFRESH_LOG_HEAD_ADDR:
		dsp_sharespace->value.log_head_addr = dsp_sharespace->dsp_log_addr;
		break;

	case CMD_READ_DEBUG_MSG:
		ret = copy_to_user((void __user *) arg, (void *)dsp_sharespace,\
					sizeof(struct dsp_sharespace_t));
		if (ret < 0) {
			DSP_ERR("copy dsp_sharespace msg to user fail\n");
			ret = -EFAULT;
		}
		break;

	case CMD_WRITE_DEBUG_MSG:
		ret = copy_from_user((void *)dsp_sharespace, (void __user *) arg,\
					sizeof(struct dsp_sharespace_t));
		if (ret < 0) {
			DSP_ERR("copy dsp_sharespace msg to user fail\n");
			return -EFAULT;
		}
		break;

	default:
		break;
	}
	return ret;
}

static struct file_operations debug_dsp_fops = {
	.owner = THIS_MODULE,
	.open = debug_dsp_dev_open,
	.mmap = debug_dsp_dev_mmap,
	.unlocked_ioctl = debug_dsp_dev_ioctl,
};

static int dsp_debug_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	uint32_t regdata[8];
	int ret = 0;

	dsp_sharespace = kzalloc(sizeof(struct dsp_sharespace_t), GFP_KERNEL);
	if (dsp_sharespace == NULL) {
		DSP_ERR("sharespace failed to alloc mem");
		goto err0;
	}

	debug_dsp_dev = kzalloc(sizeof(struct debug_dsp_dev_t), GFP_KERNEL);
	if (debug_dsp_dev == NULL) {
		DSP_ERR("debug_dsp_dev failed to alloc mem");
		goto err1;
	}

	/* copy msg from dts */
	if (np == NULL) {
		DSP_ERR("sharespace fail to get of_node\n");
		goto err2;
	}
	memset(regdata, 0, sizeof(regdata));
	ret = of_property_read_u32_array(np, "reg", regdata, 8);
	if (ret < 0) {
		DSP_ERR("reg property failed to read\n");
		goto err2;
	}

	dsp_sharespace->dsp_write_addr = regdata[0];
	dsp_sharespace->arm_write_addr = regdata[2];
	dsp_sharespace->dsp_log_addr = regdata[4];

	dsp_sharespace->dsp_write_size = regdata[1];
	dsp_sharespace->arm_write_size = regdata[3];
	dsp_sharespace->dsp_log_size = regdata[5];

	dsp_sharespace->arm_read_dsp_log_addr = dsp_sharespace->dsp_log_addr;

	/* check msg value valid */
	ret = sharespace_check_addr(dsp_sharespace);
	if (ret < 0) {
		DSP_ERR("sharespace addr failed to check\n");
		goto err2;
	}

	/* set dev mun */
	if (debug_dsp_dev->major) {
		debug_dsp_dev->devid = MKDEV(debug_dsp_dev->major, 0);
		register_chrdev_region(debug_dsp_dev->devid, DSP_DEV_NUM, DSP_DEBUG_DEV_NAME);
	} else {
		alloc_chrdev_region(&debug_dsp_dev->devid, 0, DSP_DEV_NUM, DSP_DEBUG_DEV_NAME);
		debug_dsp_dev->major = MAJOR(debug_dsp_dev->devid);
	}

	/* add dev  */
	cdev_init(&debug_dsp_dev->cdev, &debug_dsp_fops);
	cdev_add(&debug_dsp_dev->cdev, debug_dsp_dev->devid, DSP_DEV_NUM);

	/* create class */
	debug_dsp_dev->class = class_create(THIS_MODULE, DSP_DEBUG_DEV_NAME);
	if (IS_ERR(debug_dsp_dev->class)) {
		DSP_ERR("class failed to create\n");
		goto err3;
	}

	/* create device */
	debug_dsp_dev->device = device_create(debug_dsp_dev->class,\
							NULL, debug_dsp_dev->devid,\
							NULL, DSP_DEBUG_DEV_NAME);
	if (IS_ERR(debug_dsp_dev->device)) {
		DSP_ERR("device failed to create\n");
		goto err4;
	}

	return 0;
err4:
	class_destroy(debug_dsp_dev->class);
err3:
	unregister_chrdev_region(debug_dsp_dev->devid, DSP_DEV_NUM);
	cdev_del(&debug_dsp_dev->cdev);
err2:
	kfree(debug_dsp_dev);
err1:
	kfree(dsp_sharespace);
err0:
	return 0;
}

static int dsp_debug_remove(struct platform_device *pdev)
{

	cdev_del(&debug_dsp_dev->cdev);
	unregister_chrdev_region(debug_dsp_dev->devid, DSP_DEV_NUM);
	device_destroy(debug_dsp_dev->class, debug_dsp_dev->devid);
	class_destroy(debug_dsp_dev->class);
	kfree(debug_dsp_dev);
	kfree(dsp_sharespace);
	return 0;
}

static const struct of_device_id dsp_debug_match[] = {
	{ .compatible = "allwinner,sun8iw20p1-dsp-share-space" },
	{},
};

MODULE_DEVICE_TABLE(of, dsp_debug_match);

static struct platform_driver dsp_debug_driver = {
	.probe   = dsp_debug_probe,
	.remove  = dsp_debug_remove,
	.driver = {
		.name	= DSP_DEBUG_DEV_NAME,
		.owner	= THIS_MODULE,
		.pm		= NULL,
		.of_match_table = dsp_debug_match,
	},
};

static int __init dsp_debug_init(void)
{
	return platform_driver_register(&dsp_debug_driver);
}
fs_initcall_sync(dsp_debug_init);

static void __exit dsp_debug_exit(void)
{
	platform_driver_unregister(&dsp_debug_driver);
}


module_exit(dsp_debug_exit);
//module_param_named(debug, debug_mask, int, 0664);

MODULE_AUTHOR("wujiayi <wujiayi@allwinnertech.com>");
MODULE_VERSION("1.0");
MODULE_DESCRIPTION("dsp debug module");
MODULE_ALIAS("platform:"DSP_DEBUG_DEV_NAME);
MODULE_LICENSE("GPL v2");
