/*
 * Copyright 2021 The Chromium OS Authors
 * Copyright 2021 Grinn
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_ina260

#include "ina260.h"
#include "ina26x_common.h"

#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>


LOG_MODULE_REGISTER(ina260, CONFIG_SENSOR_LOG_LEVEL);

/** @brief The LSB value for the bus voltage register, in microvolts/LSB. */
#define INA260_BUS_VOLTAGE_UV_LSB 1250U //MKB confirmed correct 1.25mVLSB
#define INA260_BUS_CURRENT_UA_LSB 1250U //MKB confirmed correct 1.25mALSB

/** @brief The scaling for the power register. */
//#define INA260_POWER_SCALING 10 //MKB confirmed correct (10mW = LSB)
#define INA260_POWER_SCALING 10*1000 //MKB confirmed correct (10000uW = LSB)

static int ina260_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct ina260_data *data = dev->data;
	const struct ina260_config *const config = dev->config;
	uint32_t bus_uv, current_ua, power_uw;
	int32_t sign;

	switch (chan) {
	case SENSOR_CHAN_VOLTAGE:
		bus_uv = data->bus_voltage * INA260_BUS_VOLTAGE_UV_LSB;

		/* convert to fractional volts (units for voltage channel) */
		val->val1 = bus_uv / 1000000U;
		val->val2 = bus_uv % 1000000U;
		break;

	case SENSOR_CHAN_CURRENT:
		if (data->current & INA26X_CURRENT_SIGN_BIT) {
			//current_ua = ~data->current + 1U; //perhaps this would work if it was co-erced to (uint16_t) 
			//convert from 2's complement, with masking as we are going from 16bit to 32bit. 
			current_ua = data->current & 0x7FFF;
			current_ua = ~current_ua;
			current_ua &= 0x7FFF;
			sign = -1;
		} else {
			current_ua = data->current;
			sign = 1;
		}

		/* see datasheet "Programming" section for reference */
		current_ua = current_ua * INA260_BUS_CURRENT_UA_LSB;

		/* convert to fractional amperes */
		val->val1 = sign * (int32_t)(current_ua / 1000000U);
		val->val2 = sign * (int32_t)(current_ua % 1000000U);

		break;

	case SENSOR_CHAN_POWER:
		power_uw = data->power * INA260_POWER_SCALING;

		/* convert to fractional watts */
		val->val1 = (int32_t)(power_uw / 1000000U);
		val->val2 = (int32_t)(power_uw % 1000000U);

		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static int ina260_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct ina260_data *data = dev->data;
	const struct ina260_config *config = dev->config;
	int ret;

	if (chan != SENSOR_CHAN_ALL &&
	    chan != SENSOR_CHAN_VOLTAGE &&
	    chan != SENSOR_CHAN_CURRENT &&
	    chan != SENSOR_CHAN_POWER) {
		return -ENOTSUP;
	}

	if ((chan == SENSOR_CHAN_ALL) || (chan == SENSOR_CHAN_VOLTAGE)) {
		ret = ina26x_reg_read_16(&config->bus, INA260_REG_BUS_VOLT, &data->bus_voltage);
		if (ret < 0) {
			LOG_ERR("Failed to read bus voltage");
			return ret;
		}
	}

	if ((chan == SENSOR_CHAN_ALL) || (chan == SENSOR_CHAN_CURRENT)) {
		ret = ina26x_reg_read_16(&config->bus, INA260_REG_CURRENT, &data->current);
		if (ret < 0) {
			LOG_ERR("Failed to read current");
			return ret;
		}
	}

	if ((chan == SENSOR_CHAN_ALL) || (chan == SENSOR_CHAN_POWER)) {
		ret = ina26x_reg_read_16(&config->bus, INA260_REG_POWER, &data->power);
		if (ret < 0) {
			LOG_ERR("Failed to read power");
			return ret;
		}
	}

	return 0;
}

static int ina260_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	const struct ina260_config *config = dev->config;
	uint16_t data = val->val1;

	switch (attr) {
	case SENSOR_ATTR_CONFIGURATION:
		return ina26x_reg_write(&config->bus, INA260_REG_CONFIG, data);
	case SENSOR_ATTR_FEATURE_MASK:
		return ina26x_reg_write(&config->bus, INA260_REG_MASK, data);
	case SENSOR_ATTR_ALERT:
		return ina26x_reg_write(&config->bus, INA260_REG_ALERT, data);
	default:
		LOG_ERR("INA260 attribute not supported.");
		return -ENOTSUP;
	}
}

static int ina260_attr_get(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr,
			   struct sensor_value *val)
{
	const struct ina260_config *config = dev->config;
	uint16_t data;
	int ret;

	switch (attr) {
	case SENSOR_ATTR_CONFIGURATION:
		ret = ina26x_reg_read_16(&config->bus, INA260_REG_CONFIG, &data);
		if (ret < 0) {
			return ret;
		}
		break;
	case SENSOR_ATTR_FEATURE_MASK:
		ret = ina26x_reg_read_16(&config->bus, INA260_REG_MASK, &data);
		if (ret < 0) {
			return ret;
		}
		break;
	case SENSOR_ATTR_ALERT:
		ret = ina26x_reg_read_16(&config->bus, INA260_REG_ALERT, &data);
		if (ret < 0) {
			return ret;
		}
		break;
	default:
		LOG_ERR("INA260 attribute not supported.");
		return -ENOTSUP;
	}

	val->val1 = data;
	val->val2 = 0;

	return 0;
}

static int ina260_init(const struct device *dev)
{
	const struct ina260_config *const config = dev->config;
	int ret;

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C bus %s is not ready", config->bus.bus->name);
		return -ENODEV;
	}

	ret = ina26x_reg_write(&config->bus, INA260_REG_CONFIG, config->config);
	if (ret < 0) {
		LOG_ERR("Failed to write configuration register!");
		return ret;
	}


#ifdef CONFIG_INA260_TRIGGER
	if (config->trig_enabled) {
		ret = ina260_trigger_mode_init(dev);
		if (ret < 0) {
			LOG_ERR("Failed to init trigger mode\n");
			return ret;
		}

		ret = ina26x_reg_write(&config->bus, INA260_REG_ALERT,
				       config->alert_limit);
		if (ret < 0) {
			LOG_ERR("Failed to write alert register!");
			return ret;
		}

		ret = ina26x_reg_write(&config->bus, INA260_REG_MASK, config->mask);
		if (ret < 0) {
			LOG_ERR("Failed to write mask register!");
			return ret;
		}
	}
#endif  /* CONFIG_INA260_TRIGGER */

	return 0;
}

static const struct sensor_driver_api ina260_driver_api = {
	.attr_set = ina260_attr_set,
	.attr_get = ina260_attr_get,
#ifdef CONFIG_INA260_TRIGGER
	.trigger_set = ina260_trigger_set,
#endif
	.sample_fetch = ina260_sample_fetch,
	.channel_get = ina260_channel_get,
};

#ifdef CONFIG_INA260_TRIGGER
#define INA260_CFG_IRQ(inst)				\
	.trig_enabled = true,				\
	.mask = DT_INST_PROP(inst, mask),		\
	.alert_limit = DT_INST_PROP(inst, alert_limit),	\
	.alert_gpio = GPIO_DT_SPEC_INST_GET(inst, alert_gpios)
#else
#define INA260_CFG_IRQ(inst)
#endif /* CONFIG_INA260_TRIGGER */

#define INA260_DRIVER_INIT(inst)				    \
	static struct ina260_data drv_data_##inst;		    \
	static const struct ina260_config drv_config_##inst = {	    \
		.bus = I2C_DT_SPEC_INST_GET(inst),		    \
		.config = DT_INST_PROP(inst, config),		    \
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, alert_gpios),\
			    (INA260_CFG_IRQ(inst)), ())		    \
	};							    \
	SENSOR_DEVICE_DT_INST_DEFINE(inst,			    \
			      &ina260_init,			    \
			      NULL,				    \
			      &drv_data_##inst,			    \
			      &drv_config_##inst,		    \
			      POST_KERNEL,			    \
			      CONFIG_SENSOR_INIT_PRIORITY,	    \
			      &ina260_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INA260_DRIVER_INIT)
