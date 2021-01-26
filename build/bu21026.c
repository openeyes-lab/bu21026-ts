/*
 * bu21026.c - Part of OPEN-EYES-II products, Linux kernel modules for
 * restistive touch screen.
 *
 * Author:
 * Massimiliano Negretti <massimiliano.negretti@open-eyes.it> 2020-07-12
 *
 * This driver handle irq from chip and convert A/D values into registered
 * input device parameters.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include "bu21026.h"

#define DRIVER_NAME		"bu21026"

/**
 * @brief function to write BU21026 command type 0
 * @return int 0 on success
 */
static int bu21026_write_command0(struct BU21026_ts_data *data,
                          bool touch_on, u8 code)
{
	char buf[2];
	int  ret;

  buf[0] = (touch_on) ? 0:0x04;

  buf[0] |= ((code<<4) | (data->resolution<<2));

	/* Send the command */
	ret = i2c_master_send(data->client, buf, 1);

	return (ret>0) ? 0 : ret;
}

/**
 * @brief function to write BU21026 setup (command of type 1)
 * @return int 0 on success
 */
static int bu21026_write_setup(struct BU21026_ts_data *data)
{
	char buf[1];
	int  ret;

  buf[0] = (data->filter) ? 0:0x02;

  buf[0] |= (BU21026_SETUP_CODE<<4);

	/* Send the command */
	ret = i2c_master_send(data->client, buf, 1);

	return (ret>0) ? 0 : ret;
}

/**
 * @brief function to write BU21026 reset (command of type 2)
 * @return int 0 on success
 */
static int bu21026_write_reset(struct BU21026_ts_data *data)
{
	char buf[1];
	int  ret;

  buf[0] = (BU21026_SW_RESET_CODE<<4);

	/* Send the command */
	ret = i2c_master_send(data->client, buf, 1);

	return (ret>0) ? 0 : ret;
}

static int bu21026_set_power(struct BU21026_ts_data *data)
{
	/* Send the command */
	int ret = bu21026_write_command0(data, true, BU21026_SET_POWER_CODE);

	return ret;
}

static int bu21026_measure(struct BU21026_ts_data *data, u8 measure)
{
  char buf[2];
	int  value;
  int  error;
	int  retry=3;
	u8   cmd;

	switch (measure) {
		case BU21026_MEASURE_X_CODE:
			cmd = BU21026_DRIVE_X_CODE;
			break;
		case BU21026_MEASURE_Y_CODE:
			cmd = BU21026_DRIVE_Y_CODE;
			break;
		case BU21026_MEASURE_Z1_CODE:
		case BU21026_MEASURE_Z2_CODE:
			cmd = BU21026_DRIVE_Z_CODE;
			break;
		default:
			return -EINVAL;
	}

	do {
		if (retry--==0)
			return error;

		/* Send the drive command */
		error = bu21026_write_command0(data, false, cmd);

		if (error>=0) {

			usleep_range(CONVERSION_DELAY_MIN_US,CONVERSION_DELAY_MAX_US);

			/* Send the measure command */
			error = bu21026_write_command0(data, false, measure);

			if (error>=0) {
  			error = i2c_master_recv(data->client, buf, 2);

				if (error>=0) {
					if (buf[0]==0 && buf[1]==0)
						error =-EIO;
				}
			}
		}

		value = (buf[0] << 4) | (buf[1] >> 4);

	} while (error<0);

  return value;
}

static void touch_release(struct BU21026_ts_data *data)
{
	input_report_abs(data->in_dev, ABS_PRESSURE, 0);
	input_report_key(data->in_dev, BTN_TOUCH, 0);
	input_sync(data->in_dev);
}

static void irq_restart_handler(struct work_struct *work)
{
	struct BU21026_ts_data *data = container_of(work,
															struct BU21026_ts_data, irq_restart_work.work);

	mutex_lock(&data->lock);
	enable_irq(data->client->irq);
	data->state = TS_IRQ_IDLE;
	mutex_unlock(&data->lock);

	touch_release(data);
}

/**
 * @brief Build report for touch point.
 * @param [in] data struct BU21026_ts_data pointer.
 * @return None.
 * @details Builds and generate event for the input device. This function is
 * called after an IRQ generated from ths chip.
 */
static void BU21026_touch_report(struct BU21026_ts_data *data)
{
	u32 rz;
	u32 x,y;
	s32 max_pressure = input_abs_get_max(data->in_dev, ABS_PRESSURE);


	if (data->ts.z1==0 || data->ts.z1>4095)
		return;

	if (data->ts.z2==0 || data->ts.z2>4095)
		return;

	/*
	 * Apply some Calibration
	 */

	x = (data->ts.x>TS_X_OFFSET) ? (data->ts.x-TS_X_OFFSET):0;
	x *= data->prop.max_x;
	x /= TS_X_RATIO;

	y = (data->ts.y>TS_Y_OFFSET) ? (data->ts.y-TS_Y_OFFSET):0;
	y *= data->prop.max_y;
	y /= TS_Y_RATIO;

	/*
	 * calculate Rz (pressure resistance value) by equation:
	 * Rz = Rx * (x/Q) * ((z2/z1) - 1), where
	 * Rx is x-plate resistance,
	 * Q  is the touch screen resolution (8bit = 256, 12bit = 4096)
	 * x, z1, z2 are the measured positions.
	 */

	//printk(KERN_ERR "Report x=%d y=%d z1=%d z2=%d",data->ts.x,data->ts.y,data->ts.z1,data->ts.z2);

	rz  = data->ts.z2 - data->ts.z1;
	rz *= data->ts.x;
	rz *= data->x_plate_ohms;
	rz /= data->ts.z1;
	rz  = DIV_ROUND_CLOSEST(rz, SCALE_12BIT);

	if (rz <= max_pressure) {
		touchscreen_report_pos(data->in_dev, &data->prop, x, y, false);
		input_report_abs(data->in_dev, ABS_PRESSURE, (max_pressure - rz) );
		input_report_key(data->in_dev, BTN_TOUCH, 1);
		input_sync(data->in_dev);
	}
}

/**
 * @brief Touch detected.
 * @param [in] work struct work_struct pointer.
 * @return None.
 * @details Measure all values recorded by the chip
 * called after an IRQ generated from ths chip and call the function that
 * generate the right events.
 */
static void touch_detect_work(struct work_struct *work)
{
	struct BU21026_ts_data *data = container_of(work,
																	struct BU21026_ts_data, irq_work);

	mutex_lock(&data->lock);

	if (data->state==TS_IRQ_TOUCH_DETECTED) {

		if (bu21026_write_setup(data)==0) {
			data->ts.x = bu21026_measure(data,BU21026_MEASURE_X_CODE);
			data->ts.y = bu21026_measure(data,BU21026_MEASURE_Y_CODE);
			data->ts.z1 = bu21026_measure(data,BU21026_MEASURE_Z1_CODE);
			data->ts.z2 = bu21026_measure(data,BU21026_MEASURE_Z2_CODE);

			bu21026_set_power(data);

			BU21026_touch_report(data);

			data->state = TS_IRQ_TOUCH_DELAY;

			schedule_delayed_work(&data->irq_restart_work,
																			msecs_to_jiffies(PEN_DOWN_WAIT_MS));
		}
	}

	mutex_unlock(&data->lock);

}

/**
 * @brief IRQ handler
 * @param [in] irq number of IRQ (unused).
 * @param [in] priv void pointer of device data.
 * @return Always IRQ_HANDLED.
 * @details Check if there isn't a work in progress (debounce) and schedule
 * to serve this IRQ out of the IRQ handler.
 */
static irqreturn_t touch_detect_irq(int irq, void *priv)
{
	struct BU21026_ts_data *data = priv;

	if (data->state==TS_IRQ_IDLE) {
		data->state = TS_IRQ_TOUCH_DETECTED;
		// schedule delayed work that measure x-y-z
		schedule_work(&data->irq_work);
		// Disable IRQ until measure is done
		disable_irq_nosync(data->client->irq);
	}

	return IRQ_HANDLED;
}

static void BU21026_put_chip_in_reset(struct BU21026_ts_data *data)
{
	bu21026_write_reset(data);
}

static int BU21026_start_chip(struct input_dev *dev)
{
	struct BU21026_ts_data *data = input_get_drvdata(dev);
	struct i2c_client *i2c = data->client;
	int error;

  error = regulator_enable(data->vdd);
	if (error) {
		dev_err(&i2c->dev, "failed to power up chip: %d", error);
		return error;
	}

  if (data->started)
    return 0;

	error = bu21026_set_power(data);
	if (error) {
		dev_err(&i2c->dev, "failed to set touch autodetect\n");
		goto err_out;
	}

  data->stopped = false;
  data->started = true;

	enable_irq(data->client->irq);
	return 0;

err_out:
	BU21026_put_chip_in_reset(data);
	regulator_disable(data->vdd);
	return error;
}

static void BU21026_stop_chip(struct input_dev *dev)
{
	struct BU21026_ts_data *data = input_get_drvdata(dev);

  if (data->stopped)
    return;

	disable_irq(data->client->irq);

	BU21026_put_chip_in_reset(data);
	regulator_disable(data->vdd);
  data->stopped = true;
  data->started = false;
}

static int BU21026_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct BU21026_ts_data *data;
	struct input_dev *in_dev;
	int error;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE |
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
		dev_err(&client->dev,
			"i2c support isn't enought\n");
		return -EIO;
	}

	data = devm_kzalloc(&client->dev, sizeof(struct BU21026_ts_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	error = device_property_read_u32(&client->dev, "rohm,x-plate-ohms",
					 &data->x_plate_ohms);
	if (error) {
		dev_err(&client->dev,
			"invalid 'x-plate-ohms' supplied: %d\n", error);
		return error;
	}

	data->vdd = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		error = PTR_ERR(data->vdd);
		if (error != -EPROBE_DEFER)
			dev_err(&client->dev,
				"failed to acquire 'vdd' supply: %d\n", error);
		return error;
	}

	in_dev = devm_input_allocate_device(&client->dev);
	if (!in_dev) {
		dev_err(&client->dev, "unable to allocate input device\n");
		return -ENOMEM;
	}

	data->client = client;
	data->in_dev = in_dev;
	data->filter = true;
	data->state = TS_IRQ_IDLE;
  data->started = false;
  data->stopped = false;

	in_dev->name		    = DRIVER_NAME;
	in_dev->id.bustype	= BUS_I2C;
	in_dev->open		    = BU21026_start_chip;
	in_dev->close		    = BU21026_stop_chip;

	input_set_capability(in_dev, EV_KEY, BTN_TOUCH);
	input_set_abs_params(in_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(in_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(in_dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);
	touchscreen_parse_properties(in_dev, false, &data->prop);

	input_set_drvdata(in_dev, data);

	irq_set_status_flags(client->irq, IRQ_NOAUTOEN);
	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, touch_detect_irq,
					  IRQF_ONESHOT, DRIVER_NAME, data);
	if (error) {
		dev_err(&client->dev,
			"unable to request touch irq: %d\n", error);
		return error;
	}

	error = input_register_device(in_dev);
	if (error) {
		dev_err(&client->dev,
			"unable to register input device: %d\n", error);
		return error;
	}

	i2c_set_clientdata(client, data);

  /*
	dev_info(&client->dev,"BU21026 correctly probed x=%d y=%d ix=%d iy=%d swap=%d plate=%d\n",
	   data->prop.max_x,data->prop.max_y,
     data->prop.invert_x,
     data->prop.invert_y,
     data->prop.swap_x_y,
     data->x_plate_ohms );
  */

  dev_info(&client->dev,"Start delayed work");
	INIT_WORK(&data->irq_work, touch_detect_work);
	INIT_DELAYED_WORK(&data->irq_restart_work, irq_restart_handler);

	return 0;
}

static int __maybe_unused BU21026_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct BU21026_ts_data *data = i2c_get_clientdata(i2c);

	cancel_delayed_work_sync(&data->irq_restart_work);
	cancel_work_sync(&data->irq_work);

	if (!device_may_wakeup(dev)) {
		mutex_lock(&data->in_dev->mutex);
		if (data->in_dev->users)
			BU21026_stop_chip(data->in_dev);
		mutex_unlock(&data->in_dev->mutex);
	}

	return 0;
}

static int __maybe_unused BU21026_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct BU21026_ts_data *data = i2c_get_clientdata(i2c);

	if (!device_may_wakeup(dev)) {
		mutex_lock(&data->in_dev->mutex);
		if (data->in_dev->users)
			BU21026_start_chip(data->in_dev);
		mutex_unlock(&data->in_dev->mutex);
	}

	return 0;
}
static SIMPLE_DEV_PM_OPS(BU21026_pm_ops, BU21026_suspend, BU21026_resume);

static const struct i2c_device_id BU21026_ids[] = {
	{ DRIVER_NAME, 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, BU21026_ids);

#ifdef CONFIG_OF
static const struct of_device_id BU21026_of_ids[] = {
	{ .compatible = "rohm,bu21026" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, BU21026_of_ids);
#endif

static struct i2c_driver BU21026_driver = {
	.driver	= {
		.name		= DRIVER_NAME,
		.of_match_table	= of_match_ptr(BU21026_of_ids),
		.pm		= &BU21026_pm_ops,
	},
	.id_table	= BU21026_ids,
	.probe		= BU21026_probe,
};
module_i2c_driver(BU21026_driver);

MODULE_AUTHOR("Massimiliano Negretti massimiliano.negretti@open-eyes.it>");
MODULE_DESCRIPTION("Rohm BU21026 touchscreen controller driver");
MODULE_LICENSE("GPL v2");
