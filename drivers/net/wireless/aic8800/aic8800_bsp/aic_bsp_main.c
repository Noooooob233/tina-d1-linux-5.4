#include <linux/module.h>
#include <linux/inetdevice.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include "aic_bsp_driver.h"

#define DRV_DESCRIPTION       "AIC BSP"
#define DRV_COPYRIGHT         "Copyright(c) 2015-2020 AICSemi"
#define DRV_AUTHOR            "AICSemi"
#define DRV_VERS_MOD          "1.0"

static struct platform_device *aicbsp_pdev;

const struct aicbsp_firmware *aicbsp_firmware_list = fw_u02;

#if defined(AICWF_SDIO_SUPPORT)
const struct aicbsp_firmware fw_u02[] = {
	[AICBSP_CPMODE_WORK] = {
		.desc          = "normal work mode(sdio u02)",
		.bt_adid       = "fw_adid.bin",
		.bt_patch      = "fw_patch.bin",
		.bt_table      = "fw_patch_table.bin",
		.wl_fw         = "fmacfw.bin"
	},

	[AICBSP_CPMODE_TEST] = {
		.desc          = "rf test mode(sdio u02)",
		.bt_adid       = "fw_adid.bin",
		.bt_patch      = "fw_patch.bin",
		.bt_table      = "fw_patch_table.bin",
		.wl_fw         = "fmacfw_rf.bin"
	},
};

const struct aicbsp_firmware fw_u03[] = {
	[AICBSP_CPMODE_WORK] = {
		.desc          = "normal work mode(sdio u03/u04)",
		.bt_adid       = "fw_adid_u03.bin",
		.bt_patch      = "fw_patch_u03.bin",
		.bt_table      = "fw_patch_table_u03.bin",
		.wl_fw         = "fmacfw.bin"
	},

	[AICBSP_CPMODE_TEST] = {
		.desc          = "rf test mode(sdio u03/u04)",
		.bt_adid       = "fw_adid_u03.bin",
		.bt_patch      = "fw_patch_u03.bin",
		.bt_table      = "fw_patch_table_u03.bin",
		.wl_fw         = "fmacfw_rf.bin"
	},
};

#elif defined(AICWF_USB_SUPPORT)

const struct aicbsp_firmware fw_u02[] = {
	[AICBSP_CPMODE_WORK] = {
		.desc          = "normal work mode(usb u02)",
		.bt_adid       = "fw_adid.bin",
		.bt_patch      = "fw_patch.bin",
		.bt_table      = "fw_patch_table.bin",
		.wl_fw         = "fmacfw_usb.bin"
	},

	[AICBSP_CPMODE_TEST] = {
		.desc          = "rf test mode(usb u02)",
		.bt_adid       = "fw_adid.bin",
		.bt_patch      = "fw_patch.bin",
		.bt_table      = "fw_patch_table.bin",
		.wl_fw         = "fmacfw_rf_usb.bin"
	},
};

const struct aicbsp_firmware fw_u03[] = {
	[AICBSP_CPMODE_WORK] = {
		.desc          = "normal work mode(usb u03/u04)",
		.bt_adid       = "fw_adid_u03.bin",
		.bt_patch      = "fw_patch_u03.bin",
		.bt_table      = "fw_patch_table_u03.bin",
		.wl_fw         = "fmacfw_usb.bin"
	},

	[AICBSP_CPMODE_TEST] = {
		.desc          = "rf test mode(usb u03/u04)",
		.bt_adid       = "fw_adid_u03.bin",
		.bt_patch      = "fw_patch_u03.bin",
		.bt_table      = "fw_patch_table_u03.bin",
		.wl_fw         = "fmacfw_rf_usb.bin"
	},
};
#endif

struct aicbsp_info_t aicbsp_info = {
	.hwinfo_r = AICBSP_HWINFO_DEFAULT,
	.hwinfo   = AICBSP_HWINFO_DEFAULT,
	.cpmode   = AICBSP_CPMODE_DEFAULT,
};

struct mutex aicbsp_power_lock;

static void aicbsp_shutdown(struct platform_device *pdev)
{
	(void)pdev;
}

static struct platform_driver aicbsp_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "aic-bsp",
	},
	.shutdown = aicbsp_shutdown,
	//.probe = aicbsp_probe,
	//.remove = aicbsp_remove,
};

static ssize_t cpmode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	uint8_t i = 0;

	count += sprintf(&buf[count], "Support mode value:\n");

	for (i = 0; i < AICBSP_CPMODE_MAX; i++) {
		if (aicbsp_firmware_list[i].desc)
			count += sprintf(&buf[count], " %2d: %s\n", i, aicbsp_firmware_list[i].desc);
	}

	count += sprintf(&buf[count], "Current: %d, firmware info:\n", aicbsp_info.cpmode);
	count += sprintf(&buf[count], "  BT ADID : %s\n", aicbsp_firmware_list[aicbsp_info.cpmode].bt_adid);
	count += sprintf(&buf[count], "  BT PATCH: %s\n", aicbsp_firmware_list[aicbsp_info.cpmode].bt_patch);
	count += sprintf(&buf[count], "  BT TABLE: %s\n", aicbsp_firmware_list[aicbsp_info.cpmode].bt_table);
	count += sprintf(&buf[count], "  WIFI FW : %s\n", aicbsp_firmware_list[aicbsp_info.cpmode].wl_fw);
	return count;
}

static ssize_t cpmode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int err = kstrtoul(buf, 0, &val);
	if (err)
		return err;

	if (val >= AICBSP_CPMODE_MAX) {
		pr_err("mode value must less than %d\n", AICBSP_CPMODE_MAX);
		return -EINVAL;
	}

	aicbsp_info.cpmode = val;
	printk("%s, set mode to: %lu[%s] done\n", __func__, val, aicbsp_firmware_list[val].desc);

	return count;
}

static ssize_t hwinfo_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;

	count += sprintf(&buf[count], "chip hw rev: ");
	if (aicbsp_info.hwinfo_r < 0)
		count += sprintf(&buf[count], "-1(not avalible)\n");
	else
		count += sprintf(&buf[count], "0x%02X\n", aicbsp_info.chip_rev);

	count += sprintf(&buf[count], "hwinfo read: ");
	if (aicbsp_info.hwinfo_r < 0)
		count += sprintf(&buf[count], "%d(not avalible), ", aicbsp_info.hwinfo_r);
	else
		count += sprintf(&buf[count], "0x%02X, ", aicbsp_info.hwinfo_r);

	if (aicbsp_info.hwinfo < 0)
		count += sprintf(&buf[count], "set: %d(not avalible)\n", aicbsp_info.hwinfo);
	else
		count += sprintf(&buf[count], "set: 0x%02X\n", aicbsp_info.hwinfo);

	return count;
}

static ssize_t hwinfo_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int err = kstrtol(buf, 0, &val);

	if (err) {
		pr_err("invalid input\n");
		return err;
	}

	if ((val == -1) || (val >= 0 && val <= 0xFF)) {
		aicbsp_info.hwinfo = val;
	} else {
		pr_err("invalid values\n");
		return -EINVAL;
	}
	return count;
}

static ssize_t fwdebug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;

	count += sprintf(&buf[count], "fw log status: %s\n",
			aicbsp_info.fwlog_en ? "on" : "off");

	return count;
}

static ssize_t fwdebug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	int err = kstrtol(buf, 0, &val);

	if (err) {
		pr_err("invalid input\n");
		return err;
	}

	if (val > 1 || val < 0) {
		pr_err("must be 0 or 1\n");
		return err;
	}

	aicbsp_info.fwlog_en = val;
	return count;
}

static DEVICE_ATTR(cpmode, S_IRUGO | S_IWUSR,
		cpmode_show, cpmode_store);

static DEVICE_ATTR(hwinfo, S_IRUGO | S_IWUSR,
		hwinfo_show, hwinfo_store);

static DEVICE_ATTR(fwdebug, S_IRUGO | S_IWUSR,
		fwdebug_show, fwdebug_store);

static struct attribute *aicbsp_attributes[] = {
	&dev_attr_cpmode.attr,
	&dev_attr_hwinfo.attr,
	&dev_attr_fwdebug.attr,
	NULL,
};

static struct attribute_group aicbsp_attribute_group = {
	.name  = "aicbsp_info",
	.attrs = aicbsp_attributes,
};

static int aicbt_set_power(void *data, bool blocked)
{
	pr_info("%s: start_block=%d\n", __func__, blocked);
	if (!blocked) {
		aicbsp_set_subsys(AIC_BLUETOOTH, AIC_PWR_ON);
	} else {
		aicbsp_set_subsys(AIC_BLUETOOTH, AIC_PWR_OFF);
	}

	pr_info("%s: end_block=%d\n", __func__, blocked);
	return 0;
}

static struct rfkill_ops aicbt_rfkill_ops = {
	.set_block = aicbt_set_power,
};

static struct rfkill *aicbt_rfk;

static int aicbt_rfkill_init(struct platform_device *pdev)
{
	int rc = 0;

	pr_info("-->%s\n", __func__);
	aicbt_rfk = rfkill_alloc("bluetooth", &pdev->dev, RFKILL_TYPE_BLUETOOTH,
			&aicbt_rfkill_ops, NULL);
	if (!aicbt_rfk) {
		rc = -ENOMEM;
		goto err_rfkill_alloc;
	}
	/* userspace cannot take exclusive control */
	rfkill_init_sw_state(aicbt_rfk, true);
	rc = rfkill_register(aicbt_rfk);
	if (rc)
		goto err_rfkill_reg;

	pr_info("<--%s\n", __func__);

	return 0;

err_rfkill_reg:
	rfkill_destroy(aicbt_rfk);
err_rfkill_alloc:
	return rc;
}

static int aicbt_rfkill_remove(struct platform_device *dev)
{
	pr_info("-->%s\n", __func__);
	rfkill_unregister(aicbt_rfk);
	rfkill_destroy(aicbt_rfk);
	pr_info("<--%s\n", __func__);
	return 0;
}

static int __init aicbsp_init(void)
{
	int ret;
	printk("%s\n", __func__);

	mutex_init(&aicbsp_power_lock);
	ret = platform_driver_register(&aicbsp_driver);
	if (ret) {
		pr_err("register platform driver failed: %d\n", ret);
		goto err0;
	}

	aicbsp_pdev = platform_device_alloc("aic-bsp", -1);
	ret = platform_device_add(aicbsp_pdev);
	if (ret) {
		pr_err("register platform device failed: %d\n", ret);
		goto err1;
	}

	ret = sysfs_create_group(&(aicbsp_pdev->dev.kobj), &aicbsp_attribute_group);
	if (ret) {
		pr_err("register sysfs create group failed!\n");
		goto err2;
	}

	ret = aicbt_rfkill_init(aicbsp_pdev);
	if (ret) {
		pr_err("register rfkill fail\n");
		goto err3;
	}

	ret = aicbsp_device_init();
	if (ret) {
		pr_err("bsp subsys init fail!\n");
		goto err4;
	}

	return 0;

err4:
	aicbt_rfkill_remove(aicbsp_pdev);
err3:
	sysfs_remove_group(&(aicbsp_pdev->dev.kobj), &aicbsp_attribute_group);
err2:
	platform_device_del(aicbsp_pdev);
err1:
	platform_driver_unregister(&aicbsp_driver);
err0:
	mutex_destroy(&aicbsp_power_lock);
	return ret;
}

static void __exit aicbsp_exit(void)
{
	aicbsp_device_exit();
	aicbt_rfkill_remove(aicbsp_pdev);
	sysfs_remove_group(&(aicbsp_pdev->dev.kobj), &aicbsp_attribute_group);
	platform_device_del(aicbsp_pdev);
	platform_driver_unregister(&aicbsp_driver);
	mutex_destroy(&aicbsp_power_lock);
	printk("%s\n", __func__);
}

module_init(aicbsp_init);
module_exit(aicbsp_exit);

MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_VERSION(DRV_VERS_MOD);
MODULE_AUTHOR(DRV_COPYRIGHT " " DRV_AUTHOR);
MODULE_LICENSE("GPL");
