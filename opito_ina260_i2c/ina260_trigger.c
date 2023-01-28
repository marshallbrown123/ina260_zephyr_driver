/*
 * Copyright 2021 The Chromium OS Authors
 * Copyright 2021 Grinn
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ina260.h"

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(INA260, CONFIG_SENSOR_LOG_LEVEL);

static void ina260_gpio_callback(const struct device *port,
				 struct gpio_callback *cb, uint32_t pin)
{
	struct sensor_trigger ina260_trigger;
	struct ina260_data *ina260 = CONTAINER_OF(cb, struct ina260_data, gpio_cb);
	const struct device *dev = (const struct device *)ina260->dev;

	ARG_UNUSED(port);
	ARG_UNUSED(pin);
	ARG_UNUSED(cb);

	if (ina260->handler_alert) {
		ina260_trigger.type = SENSOR_TRIG_DATA_READY;
		ina260->handler_alert(dev, &ina260_trigger);
	}
}

int ina260_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
	struct ina260_data *ina260 = dev->data;

	ARG_UNUSED(trig);

	ina260->handler_alert = handler;

	return 0;
}

int ina260_trigger_mode_init(const struct device *dev)
{
	struct ina260_data *ina260 = dev->data;
	const struct ina260_config *config = dev->config;
	int ret;

	/* setup alert gpio interrupt */
	if (!device_is_ready(config->alert_gpio.port)) {
		LOG_ERR("Alert GPIO device not ready");
		return -ENODEV;
	}

	ina260->dev = dev;

	ret = gpio_pin_configure_dt(&config->alert_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure gpio");
		return ret;
	}

	gpio_init_callback(&ina260->gpio_cb,
			   ina260_gpio_callback,
			   BIT(config->alert_gpio.pin));

	ret = gpio_add_callback(config->alert_gpio.port, &ina260->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Could not set gpio callback");
		return ret;
	}

	return gpio_pin_interrupt_configure_dt(&config->alert_gpio,
					       GPIO_INT_EDGE_BOTH);
}
