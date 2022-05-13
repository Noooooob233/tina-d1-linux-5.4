/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2017, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*****************************************************************************
*
* File Name: focaltech_core.h

* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

#ifndef __LINUX_FOCALTECH_CORE_H__
#define __LINUX_FOCALTECH_CORE_H__
/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mount.h>
#include <linux/netdevice.h>
#include <linux/unistd.h>
#include <linux/ioctl.h>
#include "ts_common.h"

#define TS_MAX_POINTS                      5
#define TS_KEY_WIDTH                       50
#define TS_ONE_TCH_LEN                     7
#define POINT_READ_BUF  (18 + TS_ONE_TCH_LEN * TS_MAX_POINTS)

#define TS_MAX_ID                          0x0F
#define TS_TOUCH_X_H_POS                   3
#define TS_TOUCH_X_L_POS                   4
#define TS_TOUCH_Y_H_POS                   5
#define TS_TOUCH_Y_L_POS                   6
#define TS_TOUCH_PRE_POS                   7
#define TS_TOUCH_AREA_POS                  8
#define TS_TOUCH_POINT_NUM                 2
#define TS_TOUCH_EVENT_POS                 3
#define TS_TOUCH_ID_POS                    5
#define TS_COORDS_ARR_SIZE                 4

#define TS_TOUCH_DOWN      0
#define TS_TOUCH_CONTACT   1
#define TS_TOUCH_UP        2

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/

struct ts_platform_data {
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 max_touch_number;
};

struct ts_event {
	u16 au16_x[TS_MAX_POINTS]; /*x coordinate */
	u16 au16_y[TS_MAX_POINTS]; /*y coordinate */
	u16 pressure[TS_MAX_POINTS];
	u8 au8_touch_event[TS_MAX_POINTS]; /* touch event: 0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[TS_MAX_POINTS];   /*touch ID */
	u8 area[TS_MAX_POINTS];
	u8 touch_point;
	u8 point_num;
};

struct ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	const struct ts_platform_data *pdata;

	struct work_struct  touch_event_work;
	struct workqueue_struct *ts_workqueue;
	spinlock_t irq_lock;
	struct mutex report_mutex;
	u16 addr;
	bool suspended;
	int touchs;
	int irq_disable;

	struct notifier_block fb_notif;
};

/*****************************************************************************
* Static variables
*****************************************************************************/
extern struct i2c_client *ts_i2c_client;
extern struct ts_data *ts_wq_data;
extern struct input_dev *ts_input_dev;

#endif /* __LINUX_FOCALTECH_CORE_H__ */
