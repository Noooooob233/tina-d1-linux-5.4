#include "ts_core.h"

#include <linux/notifier.h>
#include <linux/fb.h>

#define TS_DRIVER_NAME                     "cst128a_ts"

struct i2c_client *ts_i2c_client;
struct ts_data *ts_wq_data;
struct input_dev *ts_input_dev;

static void ts_release_all_finger(void);
static int ts_suspend(struct device *dev);
static int ts_resume(struct device *dev);

int ts_reset_proc(int hdelayms)
{
	gpio_direction_output(ts_wq_data->pdata->reset_gpio, 0);
	msleep(20);
	gpio_direction_output(ts_wq_data->pdata->reset_gpio, 1);
	msleep(hdelayms);

	return 0;
}

void ts_irq_disable(void)
{
	unsigned long irqflags;

	spin_lock_irqsave(&ts_wq_data->irq_lock, irqflags);

	if (!ts_wq_data->irq_disable) {
		disable_irq_nosync(ts_wq_data->client->irq);
		ts_wq_data->irq_disable = 1;
	}

	spin_unlock_irqrestore(&ts_wq_data->irq_lock, irqflags);
}

void ts_irq_enable(void)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&ts_wq_data->irq_lock, irqflags);

	if (ts_wq_data->irq_disable) {
		enable_irq(ts_wq_data->client->irq);
		ts_wq_data->irq_disable = 0;
	}

	spin_unlock_irqrestore(&ts_wq_data->irq_lock, irqflags);
}

static int ts_input_dev_init(struct i2c_client *client, struct ts_data *data,  struct input_dev *input_dev, struct ts_platform_data *pdata)
{
	int  err;

	TS_FUNC_ENTER();

	/* Init and register Input device */
	input_dev->name = TS_DRIVER_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#if TS_MT_PROTOCOL_B_EN
	input_mt_init_slots(input_dev, pdata->max_touch_number, INPUT_MT_DIRECT);
#else
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 0x0f, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min, pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min, pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
#if TS_REPORT_PRESSURE_EN
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
#endif

	err = input_register_device(input_dev);
	if (err) {
		TS_ERROR("Input device registration failed");
		goto free_inputdev;
	}

	TS_FUNC_EXIT();

	return 0;

free_inputdev:
	input_free_device(input_dev);
	TS_FUNC_EXIT();
	return err;

}

static void ts_release_all_finger(void)
{
#if TS_MT_PROTOCOL_B_EN
	unsigned int finger_count = 0;
#endif

	mutex_lock(&ts_wq_data->report_mutex);
#if TS_MT_PROTOCOL_B_EN
	for (finger_count = 0; finger_count < ts_wq_data->pdata->max_touch_number; finger_count++) {
		input_mt_slot(ts_input_dev, finger_count);
		input_mt_report_slot_state(ts_input_dev, MT_TOOL_FINGER, false);
	}
#else
	input_mt_sync(ts_input_dev);
#endif
	input_report_key(ts_input_dev, BTN_TOUCH, 0);
	input_sync(ts_input_dev);
	mutex_unlock(&ts_wq_data->report_mutex);
}

#if TS_MT_PROTOCOL_B_EN
static int ts_input_dev_report_b(struct ts_event *event, struct ts_data *data)
{
	int i = 0;
	int uppoint = 0;
	int touchs = 0;

	for (i = 0; i < event->touch_point; i++) {
		if (event->au8_finger_id[i] >= data->pdata->max_touch_number) {
			break;
		}
		input_mt_slot(data->input_dev, event->au8_finger_id[i]);

		if (event->au8_touch_event[i] == TS_TOUCH_DOWN || event->au8_touch_event[i] == TS_TOUCH_CONTACT) {
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);

#if TS_REPORT_PRESSURE_EN
			if (event->pressure[i] <= 0) {
				TS_ERROR("[B]Illegal pressure: %d", event->pressure[i]);
				event->pressure[i] = 1;
			}
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure[i]);
#endif

			if (event->area[i] <= 0) {
				TS_ERROR("[B]Illegal touch-major: %d", event->area[i]);
				event->area[i] = 1;
			}
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->area[i]);

			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
			touchs |= BIT(event->au8_finger_id[i]);
			data->touchs |= BIT(event->au8_finger_id[i]);

			TS_DEBUG("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!", event->au8_finger_id[i], event->au16_x[i],
				  event->au16_y[i], event->pressure[i], event->area[i]);
		} else {
			uppoint++;
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			data->touchs &= ~BIT(event->au8_finger_id[i]);
			TS_DEBUG("[B]P%d UP!", event->au8_finger_id[i]);
		}
	}

	if (unlikely(data->touchs ^ touchs)) {
		for (i = 0; i < data->pdata->max_touch_number; i++) {
			if (BIT(i) & (data->touchs ^ touchs)) {
				TS_DEBUG("[B]P%d UP!", i);
				input_mt_slot(data->input_dev, i);
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			}
		}
	}

	data->touchs = touchs;
	if (event->touch_point == uppoint) {
		TS_DEBUG("Points All Up!");
		input_report_key(data->input_dev, BTN_TOUCH, 0);
	} else {
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
	}

	input_sync(data->input_dev);

	return 0;

}

#else
static int ts_input_dev_report_a(struct ts_event *event, struct ts_data *data)
{
	int i = 0;
	int uppoint = 0;
	int touchs = 0;

	for (i = 0; i < event->touch_point; i++) {

		if (event->au8_touch_event[i] == TS_TOUCH_DOWN || event->au8_touch_event[i] == TS_TOUCH_CONTACT) {
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->au8_finger_id[i]);
#if TS_REPORT_PRESSURE_EN
			if (event->pressure[i] <= 0) {
				TS_ERROR("[B]Illegal pressure: %d", event->pressure[i]);
				event->pressure[i] = 1;
			}
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure[i]);
#endif

			if (event->area[i] <= 0) {
				TS_ERROR("[B]Illegal touch-major: %d", event->area[i]);
				event->area[i] = 1;
			}
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->area[i]);

			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);

			input_mt_sync(data->input_dev);

			TS_DEBUG("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!", event->au8_finger_id[i], event->au16_x[i],
				  event->au16_y[i], event->pressure[i], event->area[i]);
		} else {
			uppoint++;
		}
	}

	data->touchs = touchs;
	if (event->touch_point == uppoint) {
		TS_DEBUG("Points All Up!");
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		input_mt_sync(data->input_dev);
	} else {
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
	}

	input_sync(data->input_dev);

	return 0;
}
#endif

static int ts_read_touchdata(struct ts_data *data)
{
	u8 buf[POINT_READ_BUF] = { 0 };
	u16 point_x = 0, point_y = 0;
	int ret = -1;
	int i;
	struct ts_event *event = &(data->event);

	buf[0] = 0x00;
	ret = ts_i2c_read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		TS_ERROR("[B]Read touchdata failed, ret: %d", ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));
	event->point_num = buf[17] & 0x0F;
	if (event->point_num > data->pdata->max_touch_number) {
		event->point_num = data->pdata->max_touch_number;
	}
	event->touch_point = 0;

#if CST128A_DEBUG_EN 
	printk("-----buf----\n");
	printk("PointsNum: %d\n", buf[17]);
	for(i = 18; i < POINT_READ_BUF; i++){
		if((i - 18) % 7 == 0){
			printk("%d,", buf[i]);
		}
		else{
			printk(KERN_CONT "%d,", buf[i]);
		}
	}
#endif

	event->touch_point = buf[17];

	for(i = 0; i < event->touch_point; i++){
		event->au8_finger_id[i] = (buf[18 + 7 * i]);
		point_x = (s16)(buf[20 + 7 * i] & 0x0F) << 8 | (s16) buf[19 + 7 * i];
		point_x = point_x << 1;

		point_y = (s16)(buf[22 + 7 * i] & 0x0F) << 8 | (s16) buf[21 + 7 * i];
		point_y = (point_y << 1) + (point_y >> 1);

		if (1) {
			event->au16_x[i] = point_x;
			event->au16_y[i] = point_y;
		}
		else {
			event->au16_y[i] = point_x;
			event->au16_x[i] = point_y;
		}

		if(1){
			event->au16_x[i] = data->pdata->x_max - data->pdata->x_min - event->au16_x[i];
		}

		if(0){
			event->au16_y[i] = data->pdata->y_max - data->pdata->y_min - event->au16_y[i];
		}

		event->au8_touch_event[i] = buf[24 + 7 * i] >> 1;

		event->area[i] = 0x09;
		event->pressure[i] = 0x3f;
#if CST128A_DEBUG_EN 
		printk("i:%d, x:%d, y:%d, ev:%d, fi:%d, area:%d, pre:%d\n", 
		 		i, event->au16_x[i], event->au16_y[i], event->au8_touch_event[i], event->au8_finger_id[i], event->area[i], event->pressure[i]);
#endif
	}

	if (event->touch_point == 0) {
		return -1;
	}
	return 0;
}

static void ts_report_value(struct ts_data *data)
{
	struct ts_event *event = &data->event;

	TS_DEBUG("point number: %d, touch point: %d", event->point_num,
		  event->touch_point);

#if TS_MT_PROTOCOL_B_EN
	ts_input_dev_report_b(event, data);
#else
	ts_input_dev_report_a(event, data);
#endif

	return;
}

static irqreturn_t ts_interrupt(int irq, void *dev_id)
{
	struct ts_data *ts = dev_id;
	int ret = -1;

	if (!ts) {
		TS_ERROR("[INTR]: Invalid ts");
		return IRQ_HANDLED;
	}

	ret = ts_read_touchdata(ts_wq_data);

	if (ret == 0) {
		mutex_lock(&ts_wq_data->report_mutex);
		ts_report_value(ts_wq_data);
		mutex_unlock(&ts_wq_data->report_mutex);
	}

	return IRQ_HANDLED;
}

static int ts_gpio_configure(struct ts_data *data)
{
	int err = 0;

	TS_FUNC_ENTER();
	/* request irq gpio */
	if (gpio_is_valid(data->pdata->irq_gpio)) {
		err = gpio_request(data->pdata->irq_gpio, "ts_irq_gpio");
		if (err) {
			TS_ERROR("[GPIO]irq gpio request failed");
			goto err_irq_gpio_req;
		}

		err = gpio_direction_input(data->pdata->irq_gpio);
		if (err) {
			TS_ERROR("[GPIO]set_direction for irq gpio failed");
			goto err_irq_gpio_dir;
		}
	}
	/* request reset gpio */
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		err = gpio_request(data->pdata->reset_gpio, "ts_reset_gpio");
		if (err) {
			TS_ERROR("[GPIO]reset gpio request failed");
			goto err_irq_gpio_dir;
		}

		err = gpio_direction_output(data->pdata->reset_gpio, 1);
		if (err) {
			TS_ERROR("[GPIO]set_direction for reset gpio failed");
			goto err_reset_gpio_dir;
		}
	}

	TS_FUNC_EXIT();
	return 0;

err_reset_gpio_dir:
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_free(data->pdata->reset_gpio);
	}
err_irq_gpio_dir:
	if (gpio_is_valid(data->pdata->irq_gpio)) {
		gpio_free(data->pdata->irq_gpio);
	}
err_irq_gpio_req:
	TS_FUNC_EXIT();
	return err;
}

static int ts_get_dt_coords(struct device *dev, char *name,
			     struct ts_platform_data *pdata)
{
	u32 coords[TS_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop) {
		return -EINVAL;
	}
	if (!prop->value) {
		return -ENODATA;
	}

	coords_size = prop->length / sizeof(u32);
	if (coords_size != TS_COORDS_ARR_SIZE) {
		TS_ERROR("invalid %s", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		TS_ERROR("Unable to read %s", name);
		return rc;
	}

	if (!strcmp(name, "display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		TS_ERROR("unsupported property %s", name);
		return -EINVAL;
	}

	return 0;
}

static int ts_parse_dt(struct device *dev, struct ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;

	TS_FUNC_ENTER();

	rc = ts_get_dt_coords(dev, "display-coords", pdata);
	if (rc) {
		TS_ERROR("Unable to get display-coords");
	}

	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "reset-gpio", 0, NULL);
	if (pdata->reset_gpio < 0) {
		TS_ERROR("Unable to get reset_gpio");
	}
	pdata->irq_gpio_flags = 0;
	pdata->irq_gpio = of_get_named_gpio_flags(np, "irq-gpio", 0, NULL);

	if (pdata->irq_gpio < 0) {
		TS_ERROR("Unable to get irq_gpio");
	}

	rc = of_property_read_u32(np, "max-touch-number", &temp_val);
	if (!rc) {
		pdata->max_touch_number = temp_val;
		TS_DEBUG("max_touch_number=%d", pdata->max_touch_number);
	} else {
		TS_ERROR("Unable to get max-touch-number");
		pdata->max_touch_number = TS_MAX_POINTS;
	}

	TS_FUNC_EXIT();
	return 0;
}

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ts_data *ts_data =
		container_of(self, struct ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
	    ts_data && ts_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			ts_resume(&ts_data->client->dev);
		} else if (*blank == FB_BLANK_POWERDOWN) {
			ts_suspend(&ts_data->client->dev);
		}
	}

	return 0;
}

static int ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ts_platform_data *pdata;
	struct ts_data *data;
	struct input_dev *input_dev;
	int err;

	TS_FUNC_ENTER();
	/* 1. Get Platform data */
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct ts_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			TS_ERROR("[MEMORY]Failed to allocate memory");
			TS_FUNC_EXIT();
			return -ENOMEM;
		}
		err = ts_parse_dt(&client->dev, pdata);
		if (err) {
			TS_ERROR("[DTS]DT parsing failed");
		}
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		TS_ERROR("Invalid pdata");
		TS_FUNC_EXIT();
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TS_ERROR("I2C not supported");
		TS_FUNC_EXIT();
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev, sizeof(struct ts_data), GFP_KERNEL);
	if (!data) {
		TS_ERROR("[MEMORY]Failed to allocate memory");
		TS_FUNC_EXIT();
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		TS_ERROR("[INPUT]Failed to allocate input device");
		TS_FUNC_EXIT();
		return -ENOMEM;
	}

	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;

	ts_wq_data = data;
	ts_i2c_client = client;
	ts_input_dev = input_dev;

	spin_lock_init(&ts_wq_data->irq_lock);
	mutex_init(&ts_wq_data->report_mutex);

	ts_input_dev_init(client, data, input_dev, pdata);

	err = ts_gpio_configure(data);
	if (err < 0) {
		TS_ERROR("[GPIO]Failed to configure the gpios");
		goto free_gpio;
	}

	ts_reset_proc(200);

	if (!client->irq) {
		client->irq = gpio_to_irq(pdata->irq_gpio);
	}

	err = request_threaded_irq(client->irq, NULL, ts_interrupt,
				   pdata->irq_gpio_flags | IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
				   client->dev.driver->name, data);
	if (err) {
		TS_ERROR("Request irq failed!");
		goto free_gpio;
	}

	ts_irq_enable();

	data->fb_notif.notifier_call = fb_notifier_callback;
	err = fb_register_client(&data->fb_notif);
	if (err) {
		TS_ERROR("[FB]Unable to register fb_notifier: %d", err);
	}

	TS_FUNC_EXIT();
	return 0;

free_gpio:
	if (gpio_is_valid(pdata->reset_gpio)) {
		gpio_free(pdata->reset_gpio);
	}
	if (gpio_is_valid(pdata->irq_gpio)) {
		gpio_free(pdata->irq_gpio);
	}
	return err;

}

static int ts_remove(struct i2c_client *client)
{
	struct ts_data *data = i2c_get_clientdata(client);

	TS_FUNC_ENTER();
	cancel_work_sync(&data->touch_event_work);

	if (fb_unregister_client(&data->fb_notif)) {
		TS_ERROR("Error occurred while unregistering fb_notifier.");
	}

	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_free(data->pdata->reset_gpio);
	}

	if (gpio_is_valid(data->pdata->irq_gpio)) {
		gpio_free(data->pdata->irq_gpio);
	}

	input_unregister_device(data->input_dev);

	TS_FUNC_EXIT();
	return 0;
}

static int ts_suspend(struct device *dev)
{
	struct ts_data *data = dev_get_drvdata(dev);

	TS_FUNC_ENTER();
	if (data->suspended) {
		TS_INFO("Already in suspend state");
		TS_FUNC_EXIT();
		return -1;
	}

	ts_irq_disable();

	data->suspended = true;

	TS_FUNC_EXIT();

	return 0;
}

static int ts_resume(struct device *dev)
{
	struct ts_data *data = dev_get_drvdata(dev);

	TS_FUNC_ENTER();
	if (!data->suspended) {
		TS_DEBUG("Already in awake state");
		TS_FUNC_EXIT();
		return -1;
	}

	ts_release_all_finger();

	ts_reset_proc(200);

	data->suspended = false;

	ts_irq_enable();

	TS_FUNC_EXIT();
	return 0;
}

static const struct i2c_device_id ts_id[] = {

	{TS_DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ts_id);

static struct of_device_id ts_match_table[] = {

	{ .compatible = "hyn,cst128a", },
	{ },
};

static struct i2c_driver ts_driver = {

	.probe = ts_probe,
	.remove = ts_remove,
	.driver = {
		.name = TS_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ts_match_table,
	},
	.id_table = ts_id,
};

static int __init ts_init(void)
{
	int ret = 0;

	TS_FUNC_ENTER();
	ret = i2c_add_driver(&ts_driver);
	if (ret != 0) {
		TS_ERROR("Focaltech touch screen driver init failed!");
	}
	TS_FUNC_EXIT();
	return ret;
}

static void __exit ts_exit(void)
{
	i2c_del_driver(&ts_driver);
}

module_init(ts_init);
module_exit(ts_exit);

MODULE_AUTHOR("MajorTom");
MODULE_DESCRIPTION("CST128A CUSTOMIZE Touchscreen Driver");
MODULE_LICENSE("GPL v2");
