/*
 * Copyright 2021 The Chromium OS Authors
 * Copyright (c) 2021 Grinn
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_INA26X_INA260_H_
#define ZEPHYR_DRIVERS_SENSOR_INA26X_INA260_H_

#ifdef CONFIG_INA260_TRIGGER
#include <stdbool.h>
#endif
#include <stdint.h>

#include <zephyr/device.h>
#ifdef CONFIG_INA260_TRIGGER
#include <zephyr/drivers/gpio.h>
#endif
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#ifdef CONFIG_INA260_TRIGGER
#include <zephyr/kernel.h>
#endif

#define INA260_REG_CONFIG     0x00	//MKB confirmed correct
#define INA260_REG_CURRENT    0x01	//MKB confirmed correct
#define INA260_REG_BUS_VOLT   0x02	//MKB confirmed correct
#define INA260_REG_POWER      0x03	//MKB confirmed correct
//4 not used
//5 not used
#define INA260_REG_MASK       0x06	//MKB confirmed correct
#define INA260_REG_ALERT      0x07	//MKB confirmed correct
#define INA260_REG_MANU		  0xFE	//MKB confirmed correct
#define INA260_REG_DIE_ID	  0xFF	//MKB confirmed correct

//not used on the ina260
//#define INA260_REG_SHUNT_VOLT 0x01
//#define INA260_REG_CALIB      0x05

struct ina260_data {
	const struct device *dev;
	uint16_t current;
	uint16_t bus_voltage;
	uint16_t power;
#ifdef CONFIG_INA260_TRIGGER
	const struct device *gpio;
	struct gpio_callback gpio_cb;
	struct k_work work;
	sensor_trigger_handler_t handler_alert;
#endif  /* CONFIG_INA260_TRIGGER */
};

struct ina260_config {
	struct i2c_dt_spec bus;
	uint16_t config;
#ifdef CONFIG_INA260_TRIGGER
	bool trig_enabled;
	uint16_t mask;
	const struct gpio_dt_spec alert_gpio;
	uint16_t alert_limit;
#endif  /* CONFIG_INA260_TRIGGER */
};

int ina260_trigger_mode_init(const struct device *dev);
int ina260_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);

#endif /* ZEPHYR_DRIVERS_SENSOR_INA26X_INA260_H_ */
