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

#define pr_fmt(fmt)	"[bq40z50] %s: " fmt, __func__
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>

#define bq_info	pr_info
#define bq_dbg	pr_debug
#define bq_err	pr_info
#define bq_log	pr_err

// 寄存器的无效地址定义（用于不支持的命令）
#define INVALID_REG_ADDR       0xFF

// --- 状态寄存器（0x16 Battery Status）标志位定义 ---
#define FG_FLAGS_FD             BIT(4)   // Fully Discharged (完全耗尽状态)
#define FG_FLAGS_FC             BIT(5)   // Fully Charged
#define FG_FLAGS_DSG            BIT(6)   // Discharging or Relax
#define FG_FLAGS_RCA            BIT(9)   // Remaining Capacity Alarm

// --- 寄存器索引枚举（与地址映射对应）---
enum bq_fg_reg_idx {
	BQ_FG_REG_MAC,			// ManufacturerAccess (控制命令接口)
	BQ_FG_REG_TEMP,			// 电池温度 (单位：0.1°K，代码中有转换)
	BQ_FG_REG_VOLT,			// 电池电压 (单位：mV)
	BQ_FG_REG_AI,			// 平均电流 (单位：mA，有符号)
	BQ_FG_REG_SOC,			// 相对电量百分比 (0~100%)
	BQ_FG_REG_RM ,			// 剩余容量 (单位：mAh/10mWh)
	BQ_FG_REG_FCC,			// 充满电容量 (Full Charge Capacity)
	BQ_FG_REG_TTE,			// 完全放电预估时间 (分钟)
	BQ_FG_REG_TTF,			// 充满电预估时间 (分钟)
	BQ_FG_REG_BATT_STATUS,	// 电池状态字 (包含上述FC/FD/DSG/RCA等标志位)
	BQ_FG_REG_CC,			// 循环次数计数器 (Cycle Count)
	BQ_FG_REG_DC,			// 设计容量 (Design Capacity)
	BQ_FG_REG_SOH,			// 电池健康度百分比 (State of Health)
	BQ_FG_REG_MBA,			// ManufacturerBlockAccess (数据闪存块操作接口)
	NUM_REGS,
};

// --- 制造商访问命令枚举 (通过BQ_FG_REG_MAC寄存器发送) ---
enum bq_fg_mac_cmd {
	FG_MAC_CMD_OP_STATUS    = 0x0000, // 读取操作状态字
	FG_MAC_CMD_DEV_TYPE     = 0x0001, // 获取设备类型标识
	FG_MAC_CMD_FW_VER       = 0x0002, // 读取固件版本（如代码中的FW Ver/Ztrack Ver）
	FG_MAC_CMD_HW_VER       = 0x0003, // 硬件版本号
	FG_MAC_CMD_IF_SIG       = 0x0004, // 接口签名验证
	FG_MAC_CMD_CHEM_ID      = 0x0006, // 读取电池化学ID (用于阻抗匹配)
	FG_MAC_CMD_CHG_FET      = 0x001F, // 充电FET控制（开/关）
	FG_MAC_CMD_DSG_FET      = 0x0020, // 放电FET控制（开/关）
	FG_MAC_CMD_GAUGING      = 0x0021, // 进入校准模式（如Qmax更新）
	FG_MAC_CMD_FET_EN       = 0x0022, // FET使能（开/关）
	FG_MAC_CMD_SEAL         = 0x0030, // SEAL/UNSEAL设备（安全权限控制）
	FG_MAC_CMD_DEV_RESET    = 0x0041, // 复位设备（需UNSEAL权限）
	FG_MAC_CMD_MANUFACTURE_STATUS = 0x0057, // 读取制造商状态（FET状态）
	FG_MAC_CMD_FET_CONTROL	= 0x0058, // FET控制（开/关）
};

// --- 设备安全密封状态枚举 ---
enum {
	SEAL_STATE_RSVED,      // 保留状态（未使用状态）
	SEAL_STATE_FA,         // Full Access（已验证安全密钥，最高权限）
	SEAL_STATE_UNSEALED,   // 未密封状态（允许写操作）
	SEAL_STATE_SEALED,     // 已密封（禁止写敏感寄存器）
};


enum bq_fg_device {
	BQ40Z50,
};

static const unsigned char *device2str[] = {
	"bq40z50",
};

static u8 bq40z50_regs[NUM_REGS] = {
	0x00,  // ManufacturerAccess (控制命令接口)
	0x08,  // 电池温度 (单位：0.1°K，代码中有转换)
	0x09,  // 电池电压 (单位：mV)
	0x0B,  // 平均电流 (单位：mA，有符号)
	0x0D,  // 相对电量百分比 (0~100%)
	0x0F,  // 剩余容量 (单位：mAh/10mWh)
	0x10,  // 充满电容量 (Full Charge Capacity)
	0x12,  // 完全放电预估时间 (分钟)
	0x13,  // 充满电预估时间 (分钟)
	0x16,  // 电池状态字 (包含上述FC/FD/DSG/RCA等标志位)
	0x17,  // 循环次数计数器 (Cycle Count)
	0x18,  // 设计容量 (Design Capacity)
	0x4F,  // 电池健康度百分比 (State of Health)
	0x44,  // ManufacturerBlockAccess (数据闪存块操作接口)
};

struct bq_fg_chip {
	struct device *dev;
	struct i2c_client *client;

	u8 recv_buf[40];
	u8 send_buf[40];

	struct mutex i2c_rw_lock;
	struct mutex data_lock;
	struct mutex irq_complete;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	int fw_ver;
	int df_ver;

	u8 chip;
	u8 regs[NUM_REGS];

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

	/* debug */
	int skip_reads;
	int skip_writes;

	int fake_soc;
	int fake_temp;

	struct	delayed_work monitor_work;

	struct power_supply *fg_psy;
	struct power_supply_desc fg_psy_d;
};


static int __fg_read_word(struct i2c_client *client, u8 reg, u16 *val)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		bq_err("i2c read word fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u16)ret;

	return 0;
}


static int __fg_write_word(struct i2c_client *client, u8 reg, u16 val)
{
	s32 ret;

	bq_log("__fg_write_word: reg=0x%02X, val=0x%02X\n", reg, val);

	ret = i2c_smbus_write_word_data(client, reg, val);
	if (ret < 0) {
		bq_err("i2c write word fail: can't write 0x%02X to reg 0x%02X\n",
				val, reg);
		return ret;
	}

	return 0;
}

static int __fg_read_block(struct i2c_client *client, u8 reg, u8 *buf, u8 len)
{

	int ret;

	/* len is ignored due to smbus block reading contains Num of bytes to be returned */
	ret = i2c_smbus_read_block_data(client, reg, buf);

	return ret;
}

static int __fg_write_block(struct i2c_client *client, u8 reg, u8 *buf, u8 len)
{
	int ret;

	ret = i2c_smbus_write_block_data(client, reg, len, buf);

	return ret;
}

static int fg_read_word(struct bq_fg_chip *bq, u8 reg, u16 *val)
{
	int ret;

	if (bq->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_read_word(bq->client, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

#if 0
static int fg_write_word(struct bq_fg_chip *bq, u8 reg, u16 val)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_write_word(bq->client, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}
#endif

static int fg_read_block(struct bq_fg_chip *bq, u8 reg, u8 *buf, u8 len)
{
	int ret;

	if (bq->skip_reads)
		return 0;
	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_read_block(bq->client, reg, buf, len);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;

}

static int fg_write_block(struct bq_fg_chip *bq, u8 reg, u8 *data, u8 len)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_write_block(bq->client, reg, data, len);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

#if 1
static void fg_print_buf(const char *msg, u8 *buf, u8 len)
{
	int i;
	int idx = 0;
	int num;
	u8 strbuf[128];

	bq_err("%s buf: ", msg);
	for (i = 0; i < len; i++) {
		num = sprintf(&strbuf[idx], "%02X ", buf[i]);
		idx += num;
	}
	bq_err("%s\n", strbuf);
}
#else
static void fg_print_buf(const char *msg, u8 *buf, u8 len)
{}
#endif

static int fg_mac_read_block(struct bq_fg_chip *bq, u16 cmd, u8 *buf, u8 len)
{
	int ret = -1;
	u8 t_buf[40] = {0};
	u8 t_len;
	int i;

	t_buf[0] = (u8)cmd;
	t_buf[1] = (u8)(cmd >> 8);

	ret = fg_write_block(bq, bq->regs[BQ_FG_REG_MBA], t_buf, 2);
	if (ret < 0)
		return ret;

	msleep(10);

	ret = fg_read_block(bq, bq->regs[BQ_FG_REG_MBA], t_buf, len + 2);
	if (ret < 0)
		return ret;
	t_len = ret;
	/* ret contains number of data bytes in gauge's response*/
	fg_print_buf("mac_read_block", t_buf, t_len);

	for (i = 0; i < t_len - 2; i++)
		buf[i] = t_buf[i+2];

	return ret;
}

static int fg_mac_write_block(struct bq_fg_chip *bq, u16 cmd, u8 *data, u8 len)
{
	int ret;
	u8 t_buf[40];
	int i;

	if (len > 32)
		return -1;

	t_buf[0] = (u8)cmd;
	t_buf[1] = (u8)(cmd >> 8);

	for (i = 0; i < len; i++)
		t_buf[i+2] = data[i];

	/*write command/addr, data*/
	ret = fg_write_block(bq, bq->regs[BQ_FG_REG_MBA], t_buf, len + 2);
	if (ret < 0)
		return ret;

	return ret;
}

static int fg_mac_trigger(struct bq_fg_chip *bq, u16 cmd)
{
	int ret;
	u8 t_buf[2];

	t_buf[0] = (u8)cmd;
	t_buf[1] = (u8)(cmd >> 8);

	ret = fg_write_block(bq, bq->regs[BQ_FG_REG_MBA], t_buf, 2);
	if (ret < 0)
		return ret;

	return ret;
}

static void fg_read_fw_version(struct bq_fg_chip *bq)
{

	int ret;
	u8 t_buf[36];

	ret = fg_mac_read_block(bq, FG_MAC_CMD_FW_VER, t_buf, 11);
	if (ret < 0) {
		bq_err("Failed to read firmware version:%d\n", ret);
		return;
	}

	bq_log("FW Ver:%04X, Build:%04X\n",
		t_buf[2] << 8 | t_buf[3], t_buf[4] << 8 | t_buf[5]);
	bq_log("Ztrack Ver:%04X\n", t_buf[7] << 8 | t_buf[8]);
}

static int fg_read_status(struct bq_fg_chip *bq)
{
	int ret;
	u16 flags;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_BATT_STATUS], &flags);
	if (ret < 0)
		return ret;

	mutex_lock(&bq->data_lock);
	bq->batt_fc		= !!(flags & FG_FLAGS_FC);
	bq->batt_fd		= !!(flags & FG_FLAGS_FD);
	bq->batt_rca		= !!(flags & FG_FLAGS_RCA);
	bq->batt_dsg		= !!(flags & FG_FLAGS_DSG);
	mutex_unlock(&bq->data_lock);

	return 0;
}


static int fg_read_rsoc(struct bq_fg_chip *bq)
{
	int ret;
	u16 soc = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_SOC], &soc);
	if (ret < 0) {
		bq_err("could not read RSOC, ret = %d\n", ret);
		return ret;
	}

	return soc;
}

static int fg_read_temperature(struct bq_fg_chip *bq)
{
	int ret;
	u16 temp = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_TEMP], &temp);
	if (ret < 0) {
		bq_err("could not read temperature, ret = %d\n", ret);
		return ret;
	}

	return temp - 2731;
}

static int fg_read_volt(struct bq_fg_chip *bq)
{
	int ret;
	u16 volt = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_VOLT], &volt);
	if (ret < 0) {
		bq_err("could not read voltage, ret = %d\n", ret);
		return ret;
	}

	return volt;

}

static int fg_read_current(struct bq_fg_chip *bq, int *curr)
{
	int ret;
	u16 avg_curr = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_AI], &avg_curr);
	if (ret < 0) {
		bq_err("could not read current, ret = %d\n", ret);
		return ret;
	}
	*curr = (int)((s16)avg_curr);

	return ret;
}

static int fg_read_fcc(struct bq_fg_chip *bq)
{
	int ret;
	u16 fcc;

	if (bq->regs[BQ_FG_REG_FCC] == INVALID_REG_ADDR) {
		bq_err("FCC command not supported!\n");
		return 0;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_FCC], &fcc);

	if (ret < 0)
		bq_err("could not read FCC, ret=%d\n", ret);

	return fcc;
}

static int fg_read_dc(struct bq_fg_chip *bq)
{

	int ret;
	u16 dc;

	if (bq->regs[BQ_FG_REG_DC] == INVALID_REG_ADDR) {
		bq_err("DesignCapacity command not supported!\n");
		return 0;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_DC], &dc);

	if (ret < 0) {
		bq_err("could not read DC, ret=%d\n", ret);
		return ret;
	}

	return dc;
}


static int fg_read_rm(struct bq_fg_chip *bq)
{
	int ret;
	u16 rm;

	if (bq->regs[BQ_FG_REG_RM] == INVALID_REG_ADDR) {
		bq_err("RemainingCapacity command not supported!\n");
		return 0;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_RM], &rm);

	if (ret < 0) {
		bq_err("could not read DC, ret=%d\n", ret);
		return ret;
	}

	return rm;
}

static int fg_read_cyclecount(struct bq_fg_chip *bq)
{
	int ret;
	u16 cc;

	if (bq->regs[BQ_FG_REG_CC] == INVALID_REG_ADDR) {
		bq_err("Cycle Count not supported!\n");
		return -1;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CC], &cc);

	if (ret < 0) {
		bq_err("could not read Cycle Count, ret=%d\n", ret);
		return ret;
	}

	return cc;
}

static int fg_read_tte(struct bq_fg_chip *bq)
{
	int ret;
	u16 tte;

	if (bq->regs[BQ_FG_REG_TTE] == INVALID_REG_ADDR) {
		bq_err("Time To Empty not supported!\n");
		return -1;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_TTE], &tte);

	if (ret < 0) {
		bq_err("could not read Time To Empty, ret=%d\n", ret);
		return ret;
	}

	if (ret == 0xFFFF)
		return -ENODATA;

	return tte;
}

static int fg_get_batt_status(struct bq_fg_chip *bq)
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


static int fg_get_batt_capacity_level(struct bq_fg_chip *bq)
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

static enum power_supply_property fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	/*POWER_SUPPLY_PROP_HEALTH,*//*implement it in battery power_supply*/
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static int fg_get_property(struct power_supply *psy, enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct bq_fg_chip *bq = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = fg_get_batt_status(bq);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			ret = fg_read_volt(bq);
			mutex_lock(&bq->data_lock);
			if (ret >= 0)
				bq->batt_volt = ret;
			val->intval = bq->batt_volt * 1000;
			mutex_unlock(&bq->data_lock);

			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = 1;
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			mutex_lock(&bq->data_lock);
			fg_read_current(bq, &bq->batt_curr);
			val->intval = -bq->batt_curr * 1000;
			mutex_unlock(&bq->data_lock);
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			if (bq->fake_soc >= 0) {
				val->intval = bq->fake_soc;
				break;
			}
			ret = fg_read_rsoc(bq);
			mutex_lock(&bq->data_lock);
			if (ret >= 0)
				bq->batt_soc = ret;
			val->intval = bq->batt_soc;
			mutex_unlock(&bq->data_lock);
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
			mutex_lock(&bq->data_lock);
			if (ret > 0)
				bq->batt_temp = ret;
			val->intval = bq->batt_temp;
			mutex_unlock(&bq->data_lock);
			break;

		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
			ret = fg_read_tte(bq);
			mutex_lock(&bq->data_lock);
			if (ret >= 0)
				bq->batt_tte = ret;

			val->intval = bq->batt_tte;
			mutex_unlock(&bq->data_lock);
			break;

		case POWER_SUPPLY_PROP_CHARGE_FULL:
			ret = fg_read_fcc(bq);
			mutex_lock(&bq->data_lock);
			if (ret > 0)
				bq->batt_fcc = ret;
			val->intval = bq->batt_fcc * 1000;
			mutex_unlock(&bq->data_lock);
			break;

		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			ret = fg_read_dc(bq);
			mutex_lock(&bq->data_lock);
			if (ret > 0)
				bq->batt_dc = ret;
			val->intval = bq->batt_dc * 1000;
			mutex_unlock(&bq->data_lock);
			break;

		case POWER_SUPPLY_PROP_CYCLE_COUNT:
			ret = fg_read_cyclecount(bq);
			mutex_lock(&bq->data_lock);
			if (ret >= 0)
				bq->batt_cyclecnt = ret;
			val->intval = bq->batt_cyclecnt;
			mutex_unlock(&bq->data_lock);
			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
			break;

		default:
			return -EINVAL;
	}
	return 0;
}

static int fg_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct bq_fg_chip *bq = power_supply_get_drvdata(psy);

	switch (prop) {
		case POWER_SUPPLY_PROP_TEMP:
			bq->fake_temp = val->intval;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			bq->fake_soc = val->intval;
			power_supply_changed(bq->fg_psy);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}


static int fg_prop_is_writeable(struct power_supply *psy,
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



static int fg_psy_register(struct bq_fg_chip *bq)
{
	struct power_supply_config fg_psy_cfg = {};

	bq->fg_psy_d.name = "bms";
	bq->fg_psy_d.type = POWER_SUPPLY_TYPE_BATTERY;
	bq->fg_psy_d.properties = fg_props;
	bq->fg_psy_d.num_properties = ARRAY_SIZE(fg_props);
	bq->fg_psy_d.get_property = fg_get_property;
	bq->fg_psy_d.set_property = fg_set_property;
	bq->fg_psy_d.property_is_writeable = fg_prop_is_writeable;

	fg_psy_cfg.drv_data = bq;
	fg_psy_cfg.num_supplicants = 0;
	bq->fg_psy = devm_power_supply_register(bq->dev,
			&bq->fg_psy_d,
			&fg_psy_cfg);
	if (IS_ERR(bq->fg_psy)) {
		bq_err("Failed to register fg_psy");
		return PTR_ERR(bq->fg_psy);
	}
	return 0;
}


static void fg_psy_unregister(struct bq_fg_chip *bq)
{

	power_supply_unregister(bq->fg_psy);
}

static int fg_read_mac_status(struct bq_fg_chip *bq) {
	int ret;
	u8 *t_buf = bq->recv_buf;
	
	ret = fg_mac_read_block(bq, FG_MAC_CMD_MANUFACTURE_STATUS, t_buf, 4);
	if (ret < 0) {
		bq_err("Failed to read FET status:%d", ret);
		return ret;
	}

	return 0;
}

static ssize_t fg_attr_store_fet_control(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);
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
			fg_mac_trigger(bq, FG_MAC_CMD_FET_EN);
			if (ret < 0) {
				bq_err("Failed to toggle FET control:%d\n", ret);
				return ret;
			}
		}
		
		if(!(bq->recv_buf[0] & BIT(1)))
		{
			ret = fg_mac_trigger(bq, FG_MAC_CMD_CHG_FET);
			if (ret < 0) {
				bq_err("Failed to toggle CHG control:%d\n", ret);
				return ret;
			}
		}

		if(!(bq->recv_buf[0] & BIT(2)))
		{
			ret = fg_mac_trigger(bq, FG_MAC_CMD_DSG_FET);
			if (ret < 0) {
				bq_err("Failed to toggle DSG control:%d\n", ret);
				return ret;
			}
		}
	}

	return ret;
}

static ssize_t fg_attr_show_mac_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);
	u8 *recv_buf = bq->recv_buf;
	int ret;

	ret = fg_read_mac_status(bq);
	if (ret < 0)
		return ret;

	return sprintf (buf, "\n \
		[CALTS=%d][LT_TS=%d][ResD=%d][ResC=%d][ResB=%d ][ResA=%d][LED=%d][FUSE=%d]\n \
		[BBR=%d  ][PF=%d   ][LF=%d  ][FET=%d ][GAUGE=%d][DSG=%d ][CHG=%d][PCHG=%d]\n",
		!!(recv_buf[1] & BIT(7)), !!(recv_buf[1] & BIT(6)), !!(recv_buf[1] & BIT(5)), !!(recv_buf[1] & BIT(4)),
		!!(recv_buf[1] & BIT(3)), !!(recv_buf[1] & BIT(2)), !!(recv_buf[1] & BIT(1)), !!(recv_buf[1] & BIT(0)),
		!!(recv_buf[0] & BIT(7)), !!(recv_buf[0] & BIT(6)), !!(recv_buf[0] & BIT(5)), !!(recv_buf[0] & BIT(4)),
		!!(recv_buf[0] & BIT(3)), !!(recv_buf[0] & BIT(2)), !!(recv_buf[0] & BIT(1)), !!(recv_buf[0] & BIT(0))
	);

	return ret;
}

static DEVICE_ATTR(mac_status, S_IRUGO, fg_attr_show_mac_status, NULL);
static DEVICE_ATTR(fet_control, 0644, NULL, fg_attr_store_fet_control);

static struct attribute *fg_attributes[] = {
	&dev_attr_mac_status.attr,
	&dev_attr_fet_control.attr,
	NULL,
};

static const struct attribute_group fg_attr_group = {
	.attrs = fg_attributes,
};

static irqreturn_t fg_btp_irq_thread(int irq, void *dev_id)
{
	/*struct bq_fg_chip *bq = dev_id;*/

	/* user can update btp trigger point here*/

	return IRQ_HANDLED;
}

static void fg_update_status(struct bq_fg_chip *bq)
{

	mutex_lock(&bq->data_lock);

	bq->batt_soc = fg_read_rsoc(bq);
	bq->batt_volt = fg_read_volt(bq);
	fg_read_current(bq, &bq->batt_curr);
	bq->batt_temp = fg_read_temperature(bq);
	bq->batt_rm = fg_read_rm(bq);

	mutex_unlock(&bq->data_lock);
}

static void fg_monitor_workfunc(struct work_struct *work)
{
	struct bq_fg_chip *bq = container_of(work, struct bq_fg_chip, monitor_work.work);

	fg_update_status(bq);

	schedule_delayed_work(&bq->monitor_work, 5 * HZ);
}

static void determine_initial_status(struct bq_fg_chip *bq)
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

static int bq_fg_probe(struct i2c_client *client)
{
	int ret;
	struct bq_fg_chip *bq;
	u8 *regs;

	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);

	if (!bq)
		return -ENOMEM;

	bq->dev = &client->dev;
	bq->client = client;
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

	if (bq->chip == BQ40Z50) {
		regs = bq40z50_regs;
	} else {
		bq_err("unexpected fuel gauge: %d\n", bq->chip);
		regs = bq40z50_regs;
	}

	memcpy(bq->regs, regs, NUM_REGS);

	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				fg_btp_irq_thread,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"bq fuel gauge irq", bq);
		if (ret < 0) {
			bq_err("request irq for irq=%d failed, ret = %d\n", client->irq, ret);
			goto err_1;
		}
		enable_irq_wake(client->irq);
	}

	device_init_wakeup(bq->dev, 1);

	fg_read_fw_version(bq);

	fg_psy_register(bq);

	ret = devm_device_add_group(bq->dev, &fg_attr_group);
	if (ret) {
		bq_err("Failed to register sysfs, err:%d\n", ret);
		goto err_2;
	}
	determine_initial_status(bq);

	INIT_DELAYED_WORK(&bq->monitor_work, fg_monitor_workfunc);
	// schedule_delayed_work(&bq->monitor_work, 5 * HZ);

	bq_log("bq fuel gauge probe successfully, %s\n",
			device2str[bq->chip]);

	return 0;
err_2:
	fg_psy_unregister(bq);
err_1:
	return ret;
}

static inline bool is_device_suspended(struct bq_fg_chip *bq)
{
	return !bq->resume_completed;
}

static int bq_fg_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);

	return 0;
}

static int bq_fg_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;

}

static int bq_fg_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		fg_btp_irq_thread(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

	power_supply_changed(bq->fg_psy);

	return 0;
}

static void bq_fg_remove(struct i2c_client *client)
{
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&bq->monitor_work);

	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->irq_complete);
}

static void bq_fg_shutdown(struct i2c_client *client)
{
	pr_info("bq fuel gauge driver shutdown!\n");
}

static const struct dev_pm_ops bq_fg_pm_ops = {
	.resume		= bq_fg_resume,
	.suspend_noirq = bq_fg_suspend_noirq,
	.suspend	= bq_fg_suspend,
};

static struct i2c_driver bq_fg_driver = {
	.driver = {
		.name   = "bq40z50",
		.owner  = THIS_MODULE,
		.of_match_table = bq_fg_match_table,
		.pm     = &bq_fg_pm_ops,
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