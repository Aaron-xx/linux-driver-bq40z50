/*
 * bq40z50 fuel gauge driver
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>

#include "bq40z50_fg.h"

struct bq40z50_device {
	struct i2c_client *client;
	struct device *dev;

	u8 recv_buf[40];
	u8 send_buf[40];

	u8 reg_addr;

	struct mutex i2c_rw_lock;
	struct mutex lock;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	int fw_ver;
	int df_ver;

	u8 chip;
	u8 regs;

	/* status tracking */

	bool batt_fc;
	bool batt_fd;	/* full depleted */

	bool batt_dsg;
	bool batt_rca;	/* remaining capacity alarm */

	int seal_state; /* 0 - Full Access, 1 - Unsealed, 2 - Sealed */
	int batt_tte;
	int batt_soc;
	int batt_fcc;	/* Full charge capacity */
	int batt_rm;	/* Remaining capacity */
	int batt_dc;	/* Design Capacity */
	int batt_volt;
	int batt_temp;
	int batt_curr;

	int batt_cyclecnt;	/* cycle count */

	int fake_soc;
	int fake_temp;

	struct	delayed_work monitor_work;

	struct power_supply		*charger;
};

enum bq_fg_device {
	BQ40Z50,
};

static const unsigned char *device2str[] = {
	"bq40z50",
};

static int fg_read_word(struct bq40z50_device *bq, u8 reg, u16 *val)
{
	int ret;

	mutex_lock(&bq->i2c_rw_lock);
	ret = i2c_smbus_read_word_data(bq->client, reg);
	if (ret < 0) {
		bq_err("i2c read word fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}
	*val = (u16)ret;

	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int fg_read_block(struct bq40z50_device *bq, u8 reg, u8 *buf)
{
	int ret;

	mutex_lock(&bq->i2c_rw_lock);
	ret = i2c_smbus_read_block_data(bq->client, reg, buf);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int fg_write_block(struct bq40z50_device *bq, u8 reg, u8 *buf, u8 len)
{
	int ret;

	mutex_lock(&bq->i2c_rw_lock);
	ret = i2c_smbus_write_block_data(bq->client, reg, len, buf);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int fg_mac_read_block(struct bq40z50_device *bq, u16 cmd, u8 *buf)
{
	int ret = -1;
	u8 t_buf[40] = {0};
	u8 t_len;
	int i;

	t_buf[0] = (u8)cmd;
	t_buf[1] = (u8)(cmd >> 8);

	ret = fg_write_block(bq, BQ40Z50_CMD_MANUFACTURER_BLOCK_ACCESS, t_buf, 2);
	if (ret < 0)
		return ret;

	msleep(10);

	ret = fg_read_block(bq, BQ40Z50_CMD_MANUFACTURER_BLOCK_ACCESS, t_buf);
	if (ret < 0)
		return ret;
	t_len = ret;

	for (i = 0; i < t_len - 2; i++)
		buf[i] = t_buf[i+2];

	return ret;
}

static int fg_mac_trigger(struct bq40z50_device *bq, u16 cmd)
{
	int ret;
	u8 t_buf[2];

	t_buf[0] = (u8)cmd;
	t_buf[1] = (u8)(cmd >> 8);

	ret = fg_write_block(bq, BQ40Z50_CMD_MANUFACTURER_BLOCK_ACCESS, t_buf, 2);
	if (ret < 0)
		return ret;

	return ret;
}

static void fg_read_fw_version(struct bq40z50_device *bq)
{

	int ret;
	u8 t_buf[36];

	ret = fg_mac_read_block(bq, BQ40Z50_MANU_ACCESS_FIRMWARE_VERSION, t_buf);
	if (ret < 0) {
		bq_err("Failed to read firmware version:%d\n", ret);
		return;
	}

	bq_log("FW Ver:%04X, Build:%04X\n",
		t_buf[2] << 8 | t_buf[3], t_buf[4] << 8 | t_buf[5]);
	bq_log("Ztrack Ver:%04X\n", t_buf[7] << 8 | t_buf[8]);
}

static int fg_read_status(struct bq40z50_device *bq)
{
	int ret;
	u16 flags;

	ret = fg_read_word(bq, BQ40Z50_CMD_BATTERY_STATUS, &flags);
	if (ret < 0)
		return ret;

	mutex_lock(&bq->lock);
	bq->batt_fc		= !!(flags & BQ40Z50_BATT_STATUS_FULLY_CHARGED);
	bq->batt_fd		= !!(flags & BQ40Z50_BATT_STATUS_FULLY_DISCHARGED);
	bq->batt_rca	= !!(flags & BQ40Z50_BATT_STATUS_RCA);
	bq->batt_dsg	= !!(flags & BQ40Z50_BATT_STATUS_DSG);
	mutex_unlock(&bq->lock);

	return 0;
}

static int fg_read_rsoc(struct bq40z50_device *bq)
{
	int ret;
	u16 soc = 0;

	ret = fg_read_word(bq, BQ40Z50_CMD_ABSOLUTE_STATE_OF_CHARGE, &soc);
	if (ret < 0) {
		bq_err("could not read RSOC, ret = %d\n", ret);
		return ret;
	}

	return soc;
}

static int fg_read_temperature(struct bq40z50_device *bq)
{
	int ret;
	u16 temp = 0;

	ret = fg_read_word(bq, BQ40Z50_CMD_TEMPERATURE, &temp);
	if (ret < 0) {
		bq_err("could not read temperature, ret = %d\n", ret);
		return ret;
	}

	return temp - 2731;
}

static int fg_read_volt(struct bq40z50_device *bq)
{
	int ret;
	u16 volt = 0;

	ret = fg_read_word(bq, BQ40Z50_CMD_VOLTAGE, &volt);
	if (ret < 0) {
		bq_err("could not read voltage, ret = %d\n", ret);
		return ret;
	}

	return volt;

}

static int fg_read_current(struct bq40z50_device *bq, int *curr)
{
	int ret;
	u16 avg_curr = 0;

	ret = fg_read_word(bq, BQ40Z50_CMD_CURRENT, &avg_curr);
	if (ret < 0) {
		bq_err("could not read current, ret = %d\n", ret);
		return ret;
	}
	*curr = (int)((s16)avg_curr);

	return ret;
}

static int fg_read_fcc(struct bq40z50_device *bq)
{
	int ret;
	u16 fcc;

	ret = fg_read_word(bq, BQ40Z50_CMD_FULL_CHARGE_CAPACITY, &fcc);
	if (ret < 0)
		bq_err("could not read FCC, ret=%d\n", ret);

	return fcc;
}

static int fg_read_dc(struct bq40z50_device *bq)
{

	int ret;
	u16 dc;

	ret = fg_read_word(bq, BQ40Z50_CMD_DESIGN_CAPACITY, &dc);
	if (ret < 0) {
		bq_err("could not read DC, ret=%d\n", ret);
		return ret;
	}

	return dc;
}

static int fg_read_rm(struct bq40z50_device *bq)
{
	int ret;
	u16 rm;

	ret = fg_read_word(bq, BQ40Z50_CMD_REMAINING_CAPACITY, &rm);
	if (ret < 0) {
		bq_err("could not read DC, ret=%d\n", ret);
		return ret;
	}

	return rm;
}

static int fg_read_cyclecount(struct bq40z50_device *bq)
{
	int ret;
	u16 cc;

	ret = fg_read_word(bq, BQ40Z50_CMD_CYCLE_COUNT, &cc);
	if (ret < 0) {
		bq_err("could not read Cycle Count, ret=%d\n", ret);
		return ret;
	}

	return cc;
}

static int fg_read_tte(struct bq40z50_device *bq)
{
	int ret;
	u16 tte;

	ret = fg_read_word(bq, BQ40Z50_CMD_AVERAGE_TIME_TO_EMPTY, &tte);
	if (ret < 0) {
		bq_err("could not read Time To Empty, ret=%d\n", ret);
		return ret;
	}

	if (tte == 0xFFFF)
		return -ENODATA;

	return tte;
}

static int fg_get_batt_status(struct bq40z50_device *bq)
{

	fg_read_status(bq);

	if (bq->batt_fc)
		return POWER_SUPPLY_STATUS_FULL;
	else if (bq->batt_dsg)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	else if (bq->batt_curr > 0)
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
}


static int fg_get_batt_capacity_level(struct bq40z50_device *bq)
{
	if (bq->batt_fc)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (bq->batt_rca)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (bq->batt_fd)
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
}

static enum power_supply_property bq40z50_power_supply_props[] = {
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
};

static char *bq40z50_charger_supplied_to[] = {
	"main-battery",
};

static int bq40z50_get_property(struct power_supply *psy, enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct bq40z50_device *bq = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = fg_get_batt_status(bq);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			ret = fg_read_volt(bq);
			mutex_lock(&bq->lock);
			if (ret >= 0)
				bq->batt_volt = ret;
			val->intval = bq->batt_volt * 1000;
			mutex_unlock(&bq->lock);

			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = 1;
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			mutex_lock(&bq->lock);
			fg_read_current(bq, &bq->batt_curr);
			val->intval = -bq->batt_curr * 1000;
			mutex_unlock(&bq->lock);
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			if (bq->fake_soc >= 0) {
				val->intval = bq->fake_soc;
				break;
			}
			ret = fg_read_rsoc(bq);
			mutex_lock(&bq->lock);
			if (ret >= 0)
				bq->batt_soc = ret;
			val->intval = bq->batt_soc;
			mutex_unlock(&bq->lock);
			break;

		case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
			val->intval = fg_get_batt_capacity_level(bq);
			break;

		case POWER_SUPPLY_PROP_TEMP:
			if (bq->fake_temp != -EINVAL) {
				val->intval = bq->fake_temp;
				break;
			}
			ret = fg_read_temperature(bq);
			mutex_lock(&bq->lock);
			if (ret > 0)
				bq->batt_temp = ret;
			val->intval = bq->batt_temp;
			mutex_unlock(&bq->lock);
			break;

		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
			ret = fg_read_tte(bq);
			mutex_lock(&bq->lock);
			if (ret >= 0)
				bq->batt_tte = ret;

			val->intval = bq->batt_tte;
			mutex_unlock(&bq->lock);
			break;

		case POWER_SUPPLY_PROP_CHARGE_FULL:
			ret = fg_read_fcc(bq);
			mutex_lock(&bq->lock);
			if (ret > 0)
				bq->batt_fcc = ret;
			val->intval = bq->batt_fcc * 1000;
			mutex_unlock(&bq->lock);
			break;

		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			ret = fg_read_dc(bq);
			mutex_lock(&bq->lock);
			if (ret > 0)
				bq->batt_dc = ret;
			val->intval = bq->batt_dc * 1000;
			mutex_unlock(&bq->lock);
			break;

		case POWER_SUPPLY_PROP_CYCLE_COUNT:
			ret = fg_read_cyclecount(bq);
			mutex_lock(&bq->lock);
			if (ret >= 0)
				bq->batt_cyclecnt = ret;
			val->intval = bq->batt_cyclecnt;
			mutex_unlock(&bq->lock);
			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
			break;

		default:
			return -EINVAL;
	}
	return 0;
}

static int bq40z50_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct bq40z50_device *bq = power_supply_get_drvdata(psy);

	switch (prop) {
		case POWER_SUPPLY_PROP_TEMP:
			bq->fake_temp = val->intval;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			bq->fake_soc = val->intval;
			power_supply_changed(bq->charger);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int bq40z50_property_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	int ret;

	switch (prop) {
		case POWER_SUPPLY_PROP_TEMP:
		case POWER_SUPPLY_PROP_CAPACITY:
			ret = 1;
			break;
		default:
			ret = 0;
			break;
	}
	return ret;
}

static const struct power_supply_desc bq40z50_power_supply_desc = {
	.name = "bq40z50-charger",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = bq40z50_power_supply_props,
	.num_properties = ARRAY_SIZE(bq40z50_power_supply_props),
	.get_property = bq40z50_get_property,
	.set_property = bq40z50_set_property,
	.property_is_writeable = bq40z50_property_is_writeable,
};

static void fg_psy_unregister(struct bq40z50_device *bq)
{
	power_supply_unregister(bq->charger);
}

static int fg_read_mac_status(struct bq40z50_device *bq)
{
	int ret;
	u8 *t_buf = bq->recv_buf;
	
	ret = fg_mac_read_block(bq, BQ40Z50_MANU_ACCESS_MANUFACTURING_STATUS, t_buf);
	if (ret < 0) {
		bq_err("Failed to read FET status:%d", ret);
		return ret;
	}

	return ret;
}

static int fg_read_register_status(struct bq40z50_device *bq, u8 reg)
{
	int ret;
	u8 *t_buf = bq->recv_buf;
	
	ret = fg_mac_read_block(bq, reg, t_buf);
	if (ret < 0) {
		bq_err("Failed to read FET status:%d", ret);
		return ret;
	}

	return ret;
}

static ssize_t fg_attr_store_fet_control(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq40z50_device *bq = i2c_get_clientdata(client);
	int ret = -1;
    int val;

	ret = sscanf(buf, "%d", &val); 
    if (ret < 0 || ret > 1)
	{
		dev_err(dev, "couldn't read val\n");
        return ret;
	}

	fg_read_mac_status(bq);

	if(val == 1)
	{
		if(bq->recv_buf[0] & BIT(4))
		{
			ret = fg_mac_trigger(bq, BQ40Z50_MANU_ACCESS_FET_CONTROL);
			if (ret < 0) {
				bq_err("Failed to toggle FET control:%d\n", ret);
				return ret;
			}
		}
		
		if(!(bq->recv_buf[0] & BIT(1)))
		{
			ret = fg_mac_trigger(bq, BQ40Z50_MANU_ACCESS_CHG_FET_TOGGLE);
			if (ret < 0) {
				bq_err("Failed to toggle CHG control:%d\n", ret);
				return ret;
			}
		}

		if(!(bq->recv_buf[0] & BIT(2)))
		{
			ret = fg_mac_trigger(bq, BQ40Z50_MANU_ACCESS_DSG_FET_TOGGLE);
			if (ret < 0) {
				bq_err("Failed to toggle DSG control:%d\n", ret);
				return ret;
			}
		}
	}

	return ret ? ret : count;
}

static ssize_t fg_attr_store_register_addr(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq40z50_device *bq = i2c_get_clientdata(client);
	u8 reg_addr;
	int value;

	if (sscanf(buf, "%x", &value) != 1)
		return -EINVAL;
	
	reg_addr = (u8)value;
	bq->reg_addr = reg_addr;
	
	return count;
}

static ssize_t fg_attr_show_register_addr(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq40z50_device *bq = i2c_get_clientdata(client);

    return sprintf(buf, "0x%02x\n", bq->reg_addr);
}

static ssize_t fg_attr_show_register_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq40z50_device *bq = i2c_get_clientdata(client);
	int count = 0, ret, i;

	ret = fg_read_register_status(bq, bq->reg_addr);
	if (ret <= 0) {
		return sprintf(buf, "Read error or no data\n");
	}

	bq_log("Read register 0x%02X, ret = %d\n", bq->reg_addr, ret);

	for (i = 0; i < ret; i++) {
		count += sprintf(buf + count, "%02x", bq->recv_buf[i]);
	}

	if (count > 0) {
		buf[count-1] = '\n';
	}

	return count;
}

static ssize_t fg_attr_store_register_value(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq40z50_device *bq = i2c_get_clientdata(client);
	int ret, value;
	u16 reg_addr;

	if (sscanf(buf, "%x", &value) != 1)
		return -EINVAL;
	
	reg_addr = (u16)value;

	ret = fg_mac_trigger(bq, reg_addr);
	if (ret < 0) {
		bq_err("Failed to write register 0x%02X, ret = %d\n", bq->reg_addr, ret);
		return ret;
	}

	return ret ? ret : count;
}

static DEVICE_ATTR(register_addr, S_IRUGO | S_IWUSR, 
	fg_attr_show_register_addr, fg_attr_store_register_addr);
static DEVICE_ATTR(register_value, S_IRUGO, fg_attr_show_register_value, NULL);
static DEVICE_ATTR(set_register_value, 0644, NULL, fg_attr_store_register_value);
static DEVICE_ATTR(fet_control, 0644, NULL, fg_attr_store_fet_control);

static struct attribute *fg_attributes[] = {
	&dev_attr_register_addr.attr,
	&dev_attr_register_value.attr,
	&dev_attr_set_register_value.attr,
	&dev_attr_fet_control.attr,
	NULL,
};

static const struct attribute_group fg_attr_group = {
	.attrs = fg_attributes,
};

static irqreturn_t fg_btp_irq_thread(int irq, void *dev_id)
{
	/*struct bq40z50_device *bq = dev_id;*/

	/* user can update btp trigger point here*/

	return IRQ_HANDLED;
}

static void fg_update_status(struct bq40z50_device *bq)
{

	mutex_lock(&bq->lock);

	bq->batt_soc = fg_read_rsoc(bq);
	bq->batt_volt = fg_read_volt(bq);
	fg_read_current(bq, &bq->batt_curr);
	bq->batt_temp = fg_read_temperature(bq);
	bq->batt_rm = fg_read_rm(bq);

	mutex_unlock(&bq->lock);
}

static void fg_monitor_workfunc(struct work_struct *work)
{
	struct bq40z50_device *bq = container_of(work, struct bq40z50_device, monitor_work.work);

	fg_update_status(bq);

	// schedule_delayed_work(&bq->monitor_work, 5 * HZ);
}

static void determine_initial_status(struct bq40z50_device *bq)
{
	fg_update_status(bq);

	fg_btp_irq_thread(bq->client->irq, bq);
}

static struct of_device_id bq_fg_match_table[] = {
	{.compatible = "ti,bq40z50",},
	{},
};
MODULE_DEVICE_TABLE(of, bq_fg_match_table);

static const struct i2c_device_id bq_fg_id[] = {
	{ "bq40z50", BQ40Z50 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq_fg_id);

static int bq40z50_power_supply_init(struct bq40z50_device *bq,
							struct device *dev)
{
	struct power_supply_config psy_cfg = { .drv_data = bq,
		.of_node = dev->of_node, };

	psy_cfg.supplied_to = bq40z50_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(bq40z50_charger_supplied_to);

	bq->charger = devm_power_supply_register(bq->dev,
						&bq40z50_power_supply_desc,
						&psy_cfg);
	if (IS_ERR(bq->charger))
	return -EINVAL;

	return 0;
}

static int bq_fg_probe(struct i2c_client *client)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(client);
	struct device *dev = &client->dev;
	struct bq40z50_device *bq;
	int ret;

	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->client = client;
	bq->dev = dev;
	bq->chip =  i2c_match_id(bq_fg_id, client)->driver_data;

	bq->batt_soc    = -ENODATA;
	bq->batt_fcc    = -ENODATA;
	bq->batt_rm     = -ENODATA;
	bq->batt_dc     = -ENODATA;
	bq->batt_volt   = -ENODATA;
	bq->batt_temp   = -ENODATA;
	bq->batt_curr   = -ENODATA;
	bq->batt_cyclecnt = -ENODATA;

	bq->fake_soc    = -EINVAL;
	bq->fake_temp   = -EINVAL;

	bq->reg_addr	= BQ40Z50_CMD_MANUFACTURING_STATUS;

	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->lock);

	bq->resume_completed = true;
	bq->irq_waiting = false;

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				fg_btp_irq_thread,
				IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT,
				dev_name(&client->dev), bq);
		if (ret < 0) {
			bq_err("request irq for irq=%d failed, ret = %d\n", client->irq, ret);
			return ret;
		}
		enable_irq_wake(client->irq);
	}

	device_init_wakeup(bq->dev, 1);

	fg_read_fw_version(bq);

	bq40z50_power_supply_init(bq, dev);

	ret = devm_device_add_group(bq->dev, &fg_attr_group);
	if (ret) {
		dev_err(dev, "Failed to register power supply\n");
		return ret;
	}
	determine_initial_status(bq);

	INIT_DELAYED_WORK(&bq->monitor_work, fg_monitor_workfunc);
	schedule_delayed_work(&bq->monitor_work, 5 * HZ);

	bq_log("bq fuel gauge probe successfully, %s\n",
			device2str[bq->chip]);

	return 0;
}

static void bq_fg_remove(struct i2c_client *client)
{
	struct bq40z50_device *bq = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&bq->monitor_work);
	mutex_destroy(&bq->lock);
	mutex_destroy(&bq->i2c_rw_lock);
	fg_psy_unregister(bq);
}

static void bq_fg_shutdown(struct i2c_client *client)
{
	pr_info("bq fuel gauge driver shutdown!\n");
}

static struct i2c_driver bq_fg_driver = {
	.driver = {
		.name   = "bq40z50",
		.owner  = THIS_MODULE,
		.of_match_table = bq_fg_match_table,
	},
	.probe    = bq_fg_probe,
	.remove   = bq_fg_remove,
	.shutdown = bq_fg_shutdown,
	.id_table = bq_fg_id,
};

module_i2c_driver(bq_fg_driver);

MODULE_DESCRIPTION("TI BQ40Z50 Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");