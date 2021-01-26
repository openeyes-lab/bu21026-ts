/*
 * bu20026.h - Part of OPEN-EYES-II products, Linux kernel modules for
 * restistive touch screen.
 *
 * Author:
 * Massimiliano Negretti <massimiliano.negretti@open-eyes.it> 2020-07-12
 *
 * Include file of bu21026-ts Linux driver
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

#ifndef _BU21026_H
#define _BU21026_H

/*
 * BU21026 register address
 */
#define BU21026_SET_POWER_CODE          0x0
#define BU21026_SW_RESET_CODE           0x5
#define BU21026_DRIVE_X_CODE            0x8
#define BU21026_DRIVE_Y_CODE            0x9
#define BU21026_DRIVE_Z_CODE            0xA
#define BU21026_SETUP_CODE              0xB
#define BU21026_MEASURE_X_CODE          0xC
#define BU21026_MEASURE_Y_CODE          0xD
#define BU21026_MEASURE_Z1_CODE         0xE
#define BU21026_MEASURE_Z2_CODE         0xF

/*
 * Some timing value
 */
#define PEN_DOWN_WAIT_MS	              20

#define CONVERSION_DELAY_MIN_US	        3000
#define CONVERSION_DELAY_MAX_US	        8000

/*
 * A/D value range
 */
#define SCALE_12BIT		(1 << 12)
#define MAX_12BIT		((1 << 12) - 1)

/*
 * Calibration defines
 */
#define TS_X_OFFSET 240
#define TS_X_RATIO (SCALE_12BIT-376)

#define TS_Y_OFFSET 550
#define TS_Y_RATIO (SCALE_12BIT-1000)


enum BU21026_IRQ_STATE {
	TS_IRQ_IDLE,
	TS_IRQ_TOUCH_DETECTED,
	TS_IRQ_TOUCH_DELAY
};

struct touch_screen_data {
	s16    x;
	s16    y;
	s16    z1;
	s16    z2;
};

struct BU21026_ts_data {
	struct i2c_client             *client;
	struct input_dev		          *in_dev;
	struct regulator		          *vdd;
	struct mutex                  lock;
	u32                           x_plate_ohms;
	u32                           y_plate_ohms;
	struct work_struct            irq_work;
	struct delayed_work           irq_restart_work;
	bool                          resolution;
	bool                          filter;
	bool                          started;
	bool                          stopped;
	enum BU21026_IRQ_STATE        state;
	struct touch_screen_data      ts;
	struct touchscreen_properties prop;
};

#endif /* _BU21026_H */
