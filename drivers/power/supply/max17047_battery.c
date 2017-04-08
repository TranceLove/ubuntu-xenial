/*
 * Maxim MAX17047 fuel gauge driver
 * Copyright (C) 2017 Hans de Goede <hdegoede@redhat.com>
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
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

#define MAX17047_STATUS					0x00
#define MAX17047_STATUS_BAT_NOT_PRESENT			BIT(3)
#define MAX17047_REM_CAP_REP				0x05
#define MAX17047_SOC_REP				0x06
#define MAX17047_VCELL					0x09
#define MAX17047_CURRENT				0x0a
#define MAX17047_AVG_CURRENT				0x0b
#define MAX17047_FULL_CAP				0x10
#define MAX17047_FULL_SOC_THR				0x13
#define MAX17047_DESIGN_CAP				0x18
#define MAX17047_AVG_VCELL				0x19
#define MAX17047_MAXMIN_VCELL				0x1b
#define MAX17047_VFOCV					0xfb

/* A-scales are based on the reference design Rsense = 0.010Ω. */
#define UAH_SCALE					500, false
#define UA_SCALE					156, true
#define UV_SCALE					(625 / 8), false

/* Consider REM_CAP_REP which is less then 10 units below FULL_CAP full */
#define FULL_THRESHOLD					10

struct max17047_fg_data {
	struct i2c_client *client;
	struct power_supply *battery;
};

static int max17047_get(struct max17047_fg_data *fg, u8 reg,
			union power_supply_propval *val,
			int scale, bool sign_extend)
{
	int ret;

	ret = i2c_smbus_read_word_data(fg->client, reg);
	if (ret < 0)
		return ret;

	if (sign_extend)
		ret = sign_extend32(ret, 15);

	val->intval = ret * scale;

	return 0;
}

static int max17047_get_status(struct max17047_fg_data *fg,
			       union power_supply_propval *val)
{
	int charge_full, charge_now;

	if (!power_supply_am_i_supplied(fg->battery)) {
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	/*
	 * The MAX17047 has builtin end-of-charge detection and will update
	 * FULL_CAP to match REM_CAP_REP when it detects end of charging.
	 *
	 * When this cycle the battery gets charged to a higher (calculated)
	 * capacity then the previous cycle then FULL_CAP will get updated
	 * contineously once end-of-charge detection kicks in, so allow the
	 * 2 to differ a bit.
	 */

	charge_full = i2c_smbus_read_word_data(fg->client, MAX17047_FULL_CAP);
	if (charge_full < 0)
		return charge_full;

	charge_now = i2c_smbus_read_word_data(fg->client, MAX17047_REM_CAP_REP);
	if (charge_now < 0)
		return charge_now;

	if ((charge_full - charge_now) <= FULL_THRESHOLD)
		val->intval = POWER_SUPPLY_STATUS_FULL;
	else
		val->intval = POWER_SUPPLY_STATUS_CHARGING;

	return 0;
}

static int max17047_get_present(struct max17047_fg_data *fg,
				union power_supply_propval *val)
{
	int ret;

	ret = i2c_smbus_read_word_data(fg->client, MAX17047_STATUS);
	if (ret < 0)
		return ret;

	if (ret & MAX17047_STATUS_BAT_NOT_PRESENT)
		val->intval = 0;
	else
		val->intval = 1;

	return 0;
}

static int max17047_get_min_max_volt(struct max17047_fg_data *fg,
				     union power_supply_propval *min_val,
				     union power_supply_propval *max_val)
{
	int ret;

	ret = i2c_smbus_read_word_data(fg->client, MAX17047_MAXMIN_VCELL);
	if (ret < 0)
		return ret;

	if (min_val)
		/* Lower byte contains min in 20mV units */
		min_val->intval = (ret & 0xff) * 20000;

	if (max_val)
		/* Upper byte contains max in 20mV units */
		max_val->intval = (ret >> 8) * 20000;

	return 0;
}

static int max17047_get_property(struct power_supply *psy,
	enum power_supply_property prop, union power_supply_propval *val)
{
	struct max17047_fg_data *fg = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = max17047_get_status(fg, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = max17047_get_present(fg, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		max17047_get_min_max_volt(fg, NULL, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		max17047_get_min_max_volt(fg, val, NULL);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = max17047_get(fg, MAX17047_VCELL, val, UV_SCALE);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = max17047_get(fg, MAX17047_AVG_VCELL, val, UV_SCALE);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		ret = max17047_get(fg, MAX17047_VFOCV, val, UV_SCALE);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = max17047_get(fg, MAX17047_CURRENT, val, UA_SCALE);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = max17047_get(fg, MAX17047_AVG_CURRENT, val, UA_SCALE);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = max17047_get(fg, MAX17047_DESIGN_CAP, val, UAH_SCALE);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = max17047_get(fg, MAX17047_FULL_CAP, val, UAH_SCALE);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = max17047_get(fg, MAX17047_REM_CAP_REP, val, UAH_SCALE);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = i2c_smbus_read_word_data(fg->client, MAX17047_SOC_REP);
		if (ret >= 0)
			val->intval = ret >> 8; /* Reg is in fixed 8.8 fmt */
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
	default:
		ret = -ENODATA;
	}

	return ret;
}

static void max17047_external_power_changed(struct power_supply *psy)
{
	power_supply_changed(psy);
}

static enum power_supply_property max17047_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_SCOPE,
};

static const struct power_supply_desc bat_desc = {
	/* .name must match chargers supplied_to setting, do not change */
	.name			= "main-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= max17047_properties,
	.num_properties		= ARRAY_SIZE(max17047_properties),
	.get_property		= max17047_get_property,
	.external_power_changed = max17047_external_power_changed,
};

static int max17047_probe(struct i2c_client *client,
			const struct i2c_device_id *i2c_id)
{
	struct device *dev = &client->dev;
	struct power_supply_config bat_cfg = {};
	struct max17047_fg_data *fg;
	int ret;

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->client = client;

	/*
	 * Enable End-of-Charge Detection when the voltage FG reports 95%
	 * or more, as recommend in the datasheet.
	 */
	ret = i2c_smbus_write_word_data(fg->client, MAX17047_FULL_SOC_THR,
					95 << 8);
	if (ret < 0) {
		dev_err(dev, "Error setting FULL_SOC_THR: %d\n", ret);
		return ret;
	}

	bat_cfg.drv_data = fg;
	fg->battery = devm_power_supply_register(dev, &bat_desc, &bat_cfg);
	if (IS_ERR(fg->battery))
		return PTR_ERR(fg->battery);

	return 0;
}

static const struct i2c_device_id max17047_i2c_id[] = {
	{ "max17047", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17047_i2c_id);

static struct i2c_driver max17047_driver = {
	.driver	= {
		.name	= "CHT Whiskey Cove PMIC Fuel Gauge",
	},
	.probe = max17047_probe,
	.id_table = max17047_i2c_id,
};

module_i2c_driver(max17047_driver);

MODULE_DESCRIPTION("Maxim MAX17047 fuel gauge driver");
MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_LICENSE("GPL");
