// SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
/*
 * MAX42500 - Industrial Power System Monitor
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/crc8.h>
#include <linux/gpio.h>

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define CRC8_PEC        0x07      /* Implements Polynomial X^8 + X^2 + X^1 +1 */

DECLARE_CRC8_TABLE(max42500_crc8);

#define MAX42500_REG_ID                 0x00
#define MAX42500_REG_CONFIG1            0x01
#define MAX42500_REG_CONFIG2            0x02
#define MAX42500_REG_VMON               0x03
#define MAX42500_REG_RSTMAP             0x04
#define MAX42500_REG_STATOV             0x05
#define MAX42500_REG_STATUV             0x06
#define MAX42500_REG_STATOFF            0x07
#define MAX42500_REG_VIN1               0x08
#define MAX42500_REG_VIN2               0x09
#define MAX42500_REG_VIN3               0x0A
#define MAX42500_REG_VIN4               0x0B
#define MAX42500_REG_VIN5               0x0C
#define MAX42500_REG_VINO6              0x0D
#define MAX42500_REG_VINU6              0x0E
#define MAX42500_REG_VINO7              0x0F
#define MAX42500_REG_VINU7              0x10
#define MAX42500_REG_OVUV1              0x11
#define MAX42500_REG_OVUV2              0x12
#define MAX42500_REG_OVUV3              0x13
#define MAX42500_REG_OVUV4              0x14
#define MAX42500_REG_OVUV5              0x15
#define MAX42500_REG_FPSSTAT1           0x16
#define MAX42500_REG_FPSCFG1            0x17
#define MAX42500_REG_UTIME1             0x18
#define MAX42500_REG_UTIME2             0x19
#define MAX42500_REG_UTIME3             0x1A
#define MAX42500_REG_UTIME4             0x1B
#define MAX42500_REG_UTIME5             0x1C
#define MAX42500_REG_UTIME6             0x1D
#define MAX42500_REG_UTIME7             0x1E
#define MAX42500_REG_DTIME1             0x1F
#define MAX42500_REG_DTIME2             0x20
#define MAX42500_REG_DTIME3             0x21
#define MAX42500_REG_DTIME4             0x22
#define MAX42500_REG_DTIME5             0x23
#define MAX42500_REG_DTIME6             0x24
#define MAX42500_REG_DTIME7             0x25
#define MAX42500_REG_WDSTAT             0x26
#define MAX42500_REG_WDCDIV             0x27
#define MAX42500_REG_WDCFG1             0x28
#define MAX42500_REG_WDCFG2             0x29
#define MAX42500_REG_WDKEY              0x2A
#define MAX42500_REG_WDLOCK             0x2B
#define MAX42500_REG_RSTCTRL            0x2C
#define MAX42500_REG_CID                0x2D

/** X is set based on the pull configuration of the ADDR pin */
#define MAX42500_ADDR(x)                (0x28 + (x))
#define MAX42500_SILICON_ID             (0x30)
#define MAX42500_I2C_WR_FRAME_SIZE      (4)
#define MAX42500_I2C_RD_FRAME_SIZE      (5)

/** MAX42500 Nominal voltage computation */
#define MAX42500_VNOM_MAX_VM1_VM4       3.6875
#define MAX42500_VNOM_MAX_VM5           5.6
#define MAX42500_MIN_VNOM               0.5
#define MAX42500_VNOM_STEP_VM1_VM4      0.0125
#define MAX42500_VNOM_STEP_VM5          0.02

/** MAX42500 Undervoltage/Overvoltage maximum and minimum thresholds*/
#define MAX42500_MAX_THRESH_VM1_VM5     10
#define MAX42500_MIN_THRESH_VM1_VM5     2.5
#define MAX42500_MAX_THRESH_VM6_V7      1.775
#define MAX42500_MIN_THRESH_VM6_V7      0.5

/* CONFIG1 bit masks */
#define MAX42500_CONFIG1_PECE_MASK  	BIT(0)
#define MAX42500_CONFIG1_MBST_MASK  	BIT(1)
#define MAX42500_CONFIG1_RR_MASK    	BIT(2)

/* VMON bit masks */
#define MAX42500_VMON_IN_MASK(bit)  	BIT(bit)
#define MAX42500_VMON_VMPD_MASK     	BIT(7)

/* RSTMAP bit masks */
#define MAX42500_RSTMAP_IN_MASK(bit)	BIT(bit)
#define MAX42500_RSTMAP_PARM_MASK       BIT(7)

/* WDCDIV bit masks */
#define MAX42500_WDCDIV_SWW_MASK    	BIT(6)
#define MAX42500_WDCDIV_WDIC_MASK   	(0x3F)

/* WDCFG2 bit masks */
#define MAX42500_WDCFG2_WDEN_MASK   	BIT(3)
#define MAX42500_WDCFG2_1UP_MASK    	(0x7)

/* WDLOCK bit masks */
#define MAX42500_WDLOCK_LOCK_MASK   	BIT(0)

/* RSTCTRL bit masks */
#define MAX42500_RSTCTRL_MR1_MASK   	BIT(2)
#define MAX42500_RSTCTRL_RHLD_MASK  	(0x3)

/* MAX42500 device state */
enum max42500_state {
	MAX42500_STATE_OFF,
	MAX42500_STATE_SLEEP,
	MAX42500_STATE_ON,
	MAX42500_STATE_MAX
};

/* MAX42500 voltage monitor input */
enum max42500_vm_input {
	MAX42500_VM1 = 1,
	MAX42500_VM2,
	MAX42500_VM3,
	MAX42500_VM4,
	MAX42500_VM5,
	MAX42500_VM6,
	MAX42500_VM7,
	MAX42500_VM_MAX
};

/* MAX42500 comparator status */
enum max42500_comp_stat {
	MAX42500_COMP_STAT_OFF,
	MAX42500_COMP_STAT_UV,
	MAX42500_COMP_STAT_OV,
	MAX42500_COMP_STAT_MAX
};

/* MAX42500 watchdog mode */
enum max42500_wd_mode {
	MAX42500_WD_MODE_CH_RESP,
	MAX42500_WD_MODE_SIMPLE,
	MAX42500_WD_MODE_MAX
};

/* MAX42500 reset hold/active timeout time. */
enum max42500_wd_rhld {
	MAX42500_WD_RHOLD_0_MS,
	MAX42500_WD_RHOLD_8_MS,
	MAX42500_WD_RHOLD_16_MS,
	MAX42500_WD_RHOLD_32_MS,
	MAX42500_WD_RHOLD_MAX
};

struct max42500_state {
	/* I2C */
	struct i2c_client *client;
	/* Packet error checking enable */
	uint8_t pece;
	/* Enabled voltage monitor inputs */
	uint8_t vmon_en;
	/* Voltage monitor power down enable */
	uint8_t vmon_vmpd;
	/* Enabled voltage monitor reset mapping */
	uint8_t reset_map;
	/* Watchdog mode */
	enum max42500_wd_mode wd_mode;
	/* Watchdog clock div */
	uint8_t wd_cdiv;
	/* Watchdog close window */
	uint8_t wd_close;
	/* Watchdog open window */
	uint8_t wd_open;
	/* Watchdog first update window */
	uint8_t wd_1ud;
	/* Watchdog enable */
	uint8_t wd_en;
	/* Register Map */
	struct regmap *regmap;
};

/************************ Functions Definitions *******************************/
/**
 * @brief 8-bit CRC computation
 * @return 8-bit CRC value
 */
static uint8_t crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    bool msb_set;

    for (uint8_t idx = 0; idx < len; idx++) {
        crc ^= data[idx];

        for (uint8_t loop=0; loop < 8; loop++) {
            msb_set = ((crc & 0x80) == 0x80);
            crc <<= 1;

            /* Divide by PEC if bit 8 was set */
            if (msb_set) {
                crc ^= CRC8_PEC;
            }
        }
    }

    return crc;
}

/**
 * @brief Read a raw value from a register.
 * @return 0 in case of success, error code otherwise.
 */
static int max42500_reg_read(struct max42500_state *st,
		      uint8_t reg_addr,
		      uint8_t *reg_data)
{
	int ret;
	uint8_t i2c_data[MAX42500_I2C_RD_FRAME_SIZE] = {0};
	uint8_t bytes_number;
	uint8_t crc;

	/* PEC is computed over entire I2C frame from the first START condition */
	i2c_data[0] = (st->client->addr << 1);
	i2c_data[1] = reg_addr;
	i2c_data[2] = (st->client->addr << 1) | 0x1;

	/* I2C write target address */
	bytes_number = 1;

	ret = regmap_bulk_write(st->regmap, reg_addr, &i2c_data[1], bytes_number);
	if (ret)
		return ret;

	/* Change read byte count if PECE is enabled (1-byte data. 1-byte PEC) */
	bytes_number = (st->pece) ? 2 : bytes_number;

	ret = regmap_bulk_read(st->regmap, reg_addr, &i2c_data[3], bytes_number);
	if (ret)
		return ret;

	if (st->pece) {
		/* Compute CRC over entire I2C frame */
        crc = crc8(i2c_data, (MAX42500_I2C_RD_FRAME_SIZE - 1));

		if (i2c_data[4] != crc)
			return -EIO;
	}

	*reg_data = i2c_data[3];

	return 0;
}

/**
 * @brief Write a raw value to a register.
 * @return 0 in case of success, negative error code otherwise.
 */
static int max42500_reg_write(struct max42500_state *st,
		       uint8_t reg_addr,
		       uint8_t data)
{
	uint8_t i2c_data[MAX42500_I2C_WR_FRAME_SIZE] = {0};
	uint8_t bytes_number, pece_value = 0;

	bytes_number = (st->pece) ? (MAX42500_I2C_WR_FRAME_SIZE - 1) : 2;
	i2c_data[0] = (st->client->addr << 1);
	i2c_data[1] = reg_addr;
	i2c_data[2] = (uint8_t)(data & 0xFF);

	if (st->pece)
	{
        pece_value = crc8(i2c_data, bytes_number);
	}

	i2c_data[0] = i2c_data[1];
	i2c_data[1] = i2c_data[2];
	i2c_data[2] = pece_value;

	return regmap_bulk_write(st->regmap, reg_addr, i2c_data, bytes_number);
}

/**
 * @brief Update a register's value based on a mask.
 * @return 0 in case of success, negative error code otherwise.
 */
static int max42500_reg_update(struct max42500_state *st,
			uint8_t reg_addr,
			uint8_t mask,
			uint8_t data)
{
	int ret;
	uint8_t reg_data;

	ret = max42500_reg_read(st, reg_addr, &reg_data);
	if (ret)
		return ret;

	reg_data &= ~mask;
	reg_data |= mask & data;

	return max42500_reg_write(st, reg_addr, reg_data);
}

/**
 * @brief Set nominal voltage for VM1 to VM5.
 * @return 0 in case of success, negative error code otherwise.
 */
static int max42500_set_nominal_voltage(struct max42500_state *st,
				 enum max42500_vm_input vm_in,
				 float voltage)
{
	uint8_t reg_val;
	uint8_t reg_addr;

	switch (vm_in) {
	case MAX42500_VM1:
	case MAX42500_VM2:
	case MAX42500_VM3:
	case MAX42500_VM4:
		if ((voltage < MAX42500_MIN_VNOM) ||
		    (voltage > MAX42500_VNOM_MAX_VM1_VM4))
			return -EINVAL;
		reg_val = (uint8_t)((voltage - MAX42500_MIN_VNOM) /
				    MAX42500_VNOM_STEP_VM1_VM4);
		reg_addr = MAX42500_REG_VIN1 + vm_in;
		break;
	case MAX42500_VM5:
		if ((voltage < MAX42500_MIN_VNOM) ||
		    (voltage > MAX42500_VNOM_MAX_VM5))
			return -EINVAL;
		reg_val = (uint8_t)((voltage - MAX42500_MIN_VNOM) /
				    MAX42500_VNOM_STEP_VM5);
		reg_addr = MAX42500_REG_VIN5;
		break;
	default:
		return -EINVAL;
	}

	return max42500_reg_write(st, reg_addr, reg_val);
}

/**
 * @brief Get the status of the voltage monitor input.
 * @return 0 in case of success, negative error code otherwise.
 */
static int max42500_get_comp_status(struct max42500_state *st,
			     enum max42500_vm_input vm_in,
			     uint8_t *status)
{
	int ret;
	uint8_t reg_addr;
	uint8_t vm_in_status;
	uint8_t comp_stat;

	comp_stat = vm_in % MAX42500_COMP_STAT_MAX;
	switch (comp_stat) {
	case MAX42500_COMP_STAT_OFF:
		reg_addr = MAX42500_REG_STATOFF;
		break;
	case MAX42500_COMP_STAT_UV:
		reg_addr = MAX42500_REG_STATUV;
		break;
	case MAX42500_COMP_STAT_OV:
		reg_addr = MAX42500_REG_STATOV;
		break;
	default:
		return -EINVAL;
	}

	ret = max42500_reg_read(st, reg_addr, &vm_in_status);
	if (ret)
		return ret;

	*status = (uint8_t)FIELD_GET((1 << (vm_in % MAX42500_VM_MAX)), &vm_in_status);

	return 0;
}


/**
 * @brief Set the overvoltage threshold of VM1 to VM5.
 * @return 0 in case of success, negative error code otherwise.
 */
static int max42500_set_ov_thresh1(struct max42500_state *st,
			    enum max42500_vm_input vm_in,
			    float thresh)
{
	uint8_t ov_val;

	if ((thresh < MAX42500_MIN_THRESH_VM1_VM5) ||
	    (thresh > MAX42500_MAX_THRESH_VM1_VM5))
		return -EINVAL;

	switch (vm_in) {
	case MAX42500_VM1:
	case MAX42500_VM2:
	case MAX42500_VM3:
	case MAX42500_VM4:
	case MAX42500_VM5:
		/* Compute the value of OV to be written in the register*/
		ov_val = (uint8_t)
			 DIV_ROUND_CLOSEST(((1 + (thresh / 100)) - 1.025),
						 0.005);
		return max42500_reg_update(st,
					   MAX42500_REG_OVUV1 + vm_in,
					   GENMASK(7,4),
					   FIELD_PREP(GENMASK(7,4),
							   ov_val));
	default:
		return -EINVAL;
	}
}

/**
 * @brief Set the overvoltage threshold of VM6 and VM7.
 * @return 0 in case of success, negative error code otherwise.
 */
static int max42500_set_ov_thresh2(struct max42500_state *st,
			    enum max42500_vm_input vm_in,
			    float thresh)
{
	uint8_t reg_addr;
	uint8_t ov_val;

	if ((thresh < MAX42500_MIN_THRESH_VM6_V7) ||
	    (thresh > MAX42500_MAX_THRESH_VM6_V7))
		return -EINVAL;

	switch (vm_in) {
	case MAX42500_VM6:
		reg_addr = MAX42500_REG_VINO6;
		break;
	case MAX42500_VM7:
		reg_addr = MAX42500_REG_VINO7;
		break;
	default:
		return -EINVAL;
	}

	ov_val = (uint8_t)DIV_ROUND_CLOSEST((thresh - 0.5), 0.005);

	return max42500_reg_write(st, reg_addr, ov_val);
}

/**
 * @brief Set the undervoltage threshold of VM1 to VM5.
 * @return 0 in case of success, negative error code otherwise.
 */
static int max42500_set_uv_thresh1(struct max42500_state *st,
			    enum max42500_vm_input vm_in,
			    float thresh)
{
	uint8_t uv_val;

	if ((thresh < MAX42500_MIN_THRESH_VM1_VM5) ||
	    (thresh > MAX42500_MAX_THRESH_VM1_VM5))
		return -EINVAL;

	switch (vm_in) {
	case MAX42500_VM1:
	case MAX42500_VM2:
	case MAX42500_VM3:
	case MAX42500_VM4:
	case MAX42500_VM5:
		uv_val = (uint8_t)
			 DIV_ROUND_CLOSEST(((1 - (thresh / 100)) - 0.975),
						 -0.005);
		return max42500_reg_update(st,
					   MAX42500_REG_OVUV1 + vm_in,
					   GENMASK(3,0),
					   uv_val);
	default:
		return -EINVAL;
	}
}

/**
 * @brief Set the undervoltage threshold of VM6 and VM7.
 * @return 0 in case of success, negative error code otherwise.
 */
static int max42500_set_uv_thresh2(struct max42500_state *st,
			    enum max42500_vm_input vm_in,
			    float thresh)
{
	uint8_t reg_addr;
	uint8_t uv_val;

	if ((thresh < MAX42500_MIN_THRESH_VM6_V7) ||
	    (thresh > MAX42500_MAX_THRESH_VM6_V7))
		return -EINVAL;

	switch (vm_in) {
	case MAX42500_VM6:
		reg_addr = MAX42500_REG_VINU6;
		break;
	case MAX42500_VM7:
		reg_addr = MAX42500_REG_VINU7;
		break;
	default:
		return -EINVAL;
	}

	uv_val = (uint8_t)DIV_ROUND_CLOSEST((thresh - 0.5), 0.005);

	return max42500_reg_write(st, reg_addr, uv_val);
}

/**
 * @brief Get the FPS clock divider value.
 * @return 0 in case of success, negative error code otherwise.
 */
static static int max42500_get_fps_clk_div(struct max42500_state *st,
				    uint8_t *fps_clk_div)
{
	int ret;
	uint8_t reg_val;

	ret = max42500_reg_read(desc, MAX42500_REG_FPSCFG1, &reg_val);
	if (ret)
		return ret;

	*fps_clk_div = (uint8_t)FIELD_GET(GENMASK(2,0), reg_val);

	return 0;
}

/**
 * @brief Get the power-up timestamp for a specified voltage monitor input.
 * @return 0 in case of success, negative error code otherwise.
 */
static int max42500_get_power_up_timestamp(struct max42500_state *st,
				    enum max42500_vm_input vm_in,
				    uint8_t *timestamp)
{
	int ret;
	uint8_t reg_val;
	uint8_t fps_clk_div;

	ret = max42500_reg_read(desc, MAX42500_REG_UTIME1 + vm_in, &reg_val);
	if (ret)
		return ret;

	// Check if the input voltage rose above the UV threshold
	if (reg_val == 0) {
		// Input voltage never rose above UV threshold
		*timestamp = 0;
		return 0;
	}

	ret = max42500_get_fps_clk_div(desc, &fps_clk_div);
	if (ret)
		return ret;

	*timestamp = (reg_val - 1) * 25 * (1 << fps_clk_div);

	return 0;
}

/**
 * @brief Get the power-down timestamp for a specified voltage monitor input.
 * @return 0 in case of success, negative error code otherwise.
 */
static int max42500_get_power_down_timestamp(struct max42500_state *st,
				      enum max42500_vm_input vm_in,
				      uint8_t *timestamp)
{
	int ret;
	uint8_t reg_val;
	uint8_t fps_clk_div;

	ret = max42500_reg_read(desc, MAX42500_REG_DTIME1 + vm_in, &reg_val);
	if (ret)
		return ret;

	// Check if the input voltage fell below the the OFF threshold
	if (reg_val == 0) {
		// Input voltage never fell below OFF threshold
		*timestamp = 0;
		return 0;
	}

	ret = max42500_get_fps_clk_div(desc, &fps_clk_div);
	if (ret)
		return ret;

	*timestamp = (reg_val - 1) * 25 * (1 << fps_clk_div);

	return 0;
}

/**
 * @brief Enable/Disable watchdog
 * @return 0 in case of success, error code otherwise
 */
static int max42500_set_watchdog_enable(struct max42500_state *st, bool wd_enable)
{
	int ret;
	uint8_t reg_val;

	ret = max42500_reg_read(desc, MAX42500_REG_WDCFG2, &reg_val);
	if (ret)
		return ret;

	if (wd_enable)
		reg_val |= BIT(3);
	else
		reg_val &= ~BIT(3);

	return max42500_reg_write(desc, MAX42500_REG_WDCFG2, reg_val);
}

/**
 * @brief 8-bit watchdog key computation.
 * @return 0 in case of success, negative error code otherwise.
 */
static int max42500_new_watchdog_key(struct max42500_state *st,
				     uint8_t *new_wd_key)
{
	int ret;
	uint8_t curr_wd_key;

	ret = max42500_reg_read(desc, MAX42500_REG_WDKEY, &curr_wd_key);
	if (ret)
		return ret;

	/* Calculate the new bit using the LFSR polynomial */
	uint8_t new_bit = ((curr_wd_key >> 7) ^
			   (curr_wd_key >> 5) ^
			   (curr_wd_key >> 4) ^
			   (curr_wd_key >> 3)) & 0x01;

	/* Shift existing bits upwards toward MSb and insert the new bit as LSb */
	*new_wd_key = (curr_wd_key << 1) | new_bit;

	return 0;
}

/**
 * @brief Update the watchdog key based on the mode and current value.
 * @return 0 in case of success, error code otherwise.
 */
static int max42500_set_watchdog_key(struct max42500_state *st)
{
	int ret;
	uint8_t reg_val;
	uint8_t wd_key;
	uint8_t wd_mode;

	ret = max42500_reg_read(desc, MAX42500_REG_WDKEY, &wd_key);
	if (ret)
		return ret;

	ret = max42500_reg_read(desc, MAX42500_REG_WDCDIV, &reg_val);
	if (ret)
		return ret;

	wd_mode = (uint8_t)FIELD_GET(BIT(6), reg_val);

	/* Compute new watchdog key for challenge/response mode */
	if (wd_mode == MAX42500_WD_MODE_CH_RESP)
		max42500_new_watchdog_key(desc, &wd_key);

	return max42500_reg_write(desc, MAX42500_REG_WDKEY, wd_key);
}

/** @brief Set watchdog reset hold time
 * @return 0 in case of success, error code otherwise
 */
static int max42500_set_watchdog_rhld(struct max42500_state *st,
			       enum max42500_wd_rhld rhld)
{
	return max42500_reg_update(desc,
				   MAX42500_REG_RSTCTRL,
				   GENMASK(1,0),
				   rhld);
}

static umode_t max42500_is_visible(const void *data, enum hwmon_sensor_types type, u32 attr,
				  int channel)
{
	const struct max42500_state *st = data;

	switch (type) {
	case hwmon_chip:
		switch (attr) {
		case hwmon_chip_temp_reset_history:
		case hwmon_chip_in_reset_history:
		case hwmon_chip_register_tz:
		case hwmon_chip_alarms:
			return 0644;
		case hwmon_chip_curr_reset_history:
		case hwmon_chip_power_reset_history:
		case hwmon_chip_update_interval:
			return 0200;
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
		case hwmon_in_lowest:
		case hwmon_in_highest:
		case hwmon_in_label:
			return 0444;
		case hwmon_in_min:
		case hwmon_in_max:
			return 0644;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int max42500_read_in(struct device *dev, u32 attr, int channel, long val)
{
	struct max42500_state *st = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_in_input:
		return max42500_set_nominal_voltage(st, channel, val);
	case hwmon_in_min:
		return max42500_set_uv_thresh1(st, channel, val);
	case hwmon_in_max:
		return max42500_set_uv_thresh2(st, channel, val);
	case hwmon_in_lowest:
		return max42500_set_ov_thresh1(st, channel, val);
	case hwmon_in_highest:
		return max42500_set_ov_thresh2(st, channel, val);
	case hwmon_in_label:
		return max42500_get_comp_status(st, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int max42500_read_chip(struct device *dev, u32 attr, int channel, long val)
{
	struct max42500_state *st = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_chip_temp_reset_history:
		return max42500_set_watchdog_rhld(st, val);
	case hwmon_chip_in_reset_history:
		return max42500_get_power_up_timestamp(st, channel, &val);
	case hwmon_chip_curr_reset_history:
		return max42500_get_power_down_timestamp(st, channel, &val);
	case hwmon_chip_power_reset_history:
		return max42500_new_watchdog_key(st, &val);
	case hwmon_chip_register_tz:
		return max42500_set_watchdog_key(st);
	case hwmon_chip_update_interval:
		return max42500_get_fps_clk_div(st, &val);
	case hwmon_chip_alarms:
		return max42500_set_watchdog_enable(st, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int max42500_read(struct device *dev, enum hwmon_sensor_types type, u32 attr, int channel,
			long *val)
{
	switch (type) {
	case hwmon_chip:
		return max42500_read_chip(dev, attr, channel, val);
	case hwmon_in:
		return max42500_read_in(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int max42500_write_in(struct device *dev, u32 attr, int channel, long val)
{
	struct max42500_state *st = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_in_input:
		return max42500_set_nominal_voltage(st, channel, val);
	case hwmon_in_min:
		return max42500_set_uv_thresh1(st, channel, val);
	case hwmon_in_max:
		return max42500_set_uv_thresh2(st, channel, val);
	case hwmon_in_lowest:
		return max42500_set_ov_thresh1(st, channel, val);
	case hwmon_in_highest:
		return max42500_set_ov_thresh2(st, channel, val);
	case hwmon_in_label:
		return max42500_get_comp_status(st, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int max42500_write_chip(struct device *dev, u32 attr, int channel, long val)
{
	struct max42500_state *st = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_chip_temp_reset_history:
		return max42500_set_watchdog_rhld(st, val);
	case hwmon_chip_in_reset_history:
		return max42500_get_power_up_timestamp(st, channel, &val);
	case hwmon_chip_curr_reset_history:
		return max42500_get_power_down_timestamp(st, channel, &val);
	case hwmon_chip_power_reset_history:
		return max42500_new_watchdog_key(st, &val);
	case hwmon_chip_register_tz:
		return max42500_set_watchdog_key(st);
	case hwmon_chip_update_interval:
		return max42500_get_fps_clk_div(st, &val);
	case hwmon_chip_alarms:
		return max42500_set_watchdog_enable(st, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int max42500_write(struct device *dev, enum hwmon_sensor_types type, u32 attr, int channel,
			 long val)
{
	switch (type) {
	case hwmon_chip:
		return max42500_write_chip(dev, attr, channel, val);
	case hwmon_in:
		return max42500_write_in(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static struct max42500_state max42500_config = {
	.client = NULL,
	.pece = true,
	.vmon_en = true,
	.vmon_vmpd = true,
	.reset_map = (MAX42500_RSTMAP_IN_MASK(MAX42500_VM2) | \
                  MAX42500_RSTMAP_IN_MASK(MAX42500_VM3) | \
                  MAX42500_RSTMAP_IN_MASK(MAX42500_VM4) | \
                  MAX42500_RSTMAP_PARM_MASK),
	.wd_mode = true,
	.wd_cdiv = 0x0,
	.wd_close =0x0,
	.wd_open =0x0,
	.wd_1ud = 0x0,
	.wd_en = true,
	.regmap = NULL,
};

static const struct hwmon_ops max42500_hwmon_ops = {
	.is_visible = max42500_is_visible,
	.read = max42500_read,
	.write = max42500_write,
};

static const enum max42500_vm_input set_nominal_voltage_config[] = {
	HWMON_I_INPUT,
	HWMON_I_INPUT,
	HWMON_I_INPUT,
	HWMON_I_INPUT,
	HWMON_I_INPUT,
	0,
};

static const struct hwmon_channel_info set_nominal_voltage = {
    .type = hwmon_in,
    .config = set_nominal_voltage_config,
};

static const enum max42500_vm_input get_comp_status_config[] = {
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	HWMON_I_LABEL,
	0,
};

static const struct hwmon_channel_info get_comp_status = {
    .type = hwmon_in,
    .config = get_comp_status_config,
};

static const enum max42500_vm_input set_ov_thresh1_config[] = {
	HWMON_I_LOWEST,
	HWMON_I_LOWEST,
	HWMON_I_LOWEST,
	HWMON_I_LOWEST,
	HWMON_I_LOWEST,
	0,
};

static const struct hwmon_channel_info set_ov_thresh1 = {
    .type = hwmon_in,
    .config = set_ov_thresh1_config,
};

static const enum max42500_vm_input set_ov_thresh2_config[] = {
	HWMON_I_HIGHEST,
	HWMON_I_HIGHEST,
	0,
};

static const struct hwmon_channel_info set_ov_thresh2 = {
    .type = hwmon_in,
    .config = set_ov_thresh2_config,
};

static const enum max42500_vm_input set_uv_thresh1_config[] = {
	HWMON_I_MIN,
	HWMON_I_MIN,
	HWMON_I_MIN,
	HWMON_I_MIN,
	HWMON_I_MIN,
	0,
};

static const struct hwmon_channel_info set_uv_thresh1 = {
    .type = hwmon_in,
    .config = set_uv_thresh1_config,
};

static const enum max42500_vm_input set_uv_thresh2_config[] = {
	HWMON_I_MAX,
	HWMON_I_MAX,
	0,
};

static const struct hwmon_channel_info set_uv_thresh2 = {
    .type = hwmon_in,
    .config = set_uv_thresh2_config,
};

static const enum max42500_vm_input get_power_up_timestamp_config[] = {
	HWMON_C_IN_RESET_HISTORY,
	HWMON_C_IN_RESET_HISTORY,
	HWMON_C_IN_RESET_HISTORY,
	HWMON_C_IN_RESET_HISTORY,
	HWMON_C_IN_RESET_HISTORY,
	HWMON_C_IN_RESET_HISTORY,
	HWMON_C_IN_RESET_HISTORY,
	0,
};

static const struct hwmon_channel_info get_power_up_timestamp = {
    .type = hwmon_in,
    .config = get_power_up_timestamp_config,
};

static const enum max42500_vm_input get_power_down_timestamp_config[] = {
	HWMON_C_CURR_RESET_HISTORY,
	HWMON_C_CURR_RESET_HISTORY,
	HWMON_C_CURR_RESET_HISTORY,
	HWMON_C_CURR_RESET_HISTORY,
	HWMON_C_CURR_RESET_HISTORY,
	HWMON_C_CURR_RESET_HISTORY,
	HWMON_C_CURR_RESET_HISTORY,
	0,
};

static const struct hwmon_channel_info get_power_down_timestamp = {
    .type = hwmon_in,
    .config = get_power_down_timestamp_config,
};

static const struct hwmon_channel_info *max42500_info[] = {
	HWMON_CHANNEL_INFO(chip,
			   HWMON_C_TEMP_RESET_HISTORY | &get_power_up_timestamp |
			   &get_power_down_timestamp | HWMON_C_POWER_RESET_HISTORY |
			   HWMON_C_REGISTER_TZ | HWMON_C_UPDATE_INTERVAL | HWMON_C_ALARMS,
			   HWMON_C_TEMP_RESET_HISTORY | &get_power_up_timestamp |
			   &get_power_down_timestamp | HWMON_C_POWER_RESET_HISTORY |
			   HWMON_C_REGISTER_TZ | HWMON_C_UPDATE_INTERVAL | HWMON_C_ALARMS),
	HWMON_CHANNEL_INFO(in,
			   &set_nominal_voltage | &set_ov_thresh1 | &set_ov_thresh2 | &set_uv_thresh1 |
			   &set_uv_thresh2 | &get_comp_status,
			   &set_nominal_voltage | &set_ov_thresh1 | &set_ov_thresh2 | &set_uv_thresh1 |
			   &set_uv_thresh2 | &get_comp_status),
	NULL
};

static const struct hwmon_chip_info max42500_chip_info = {
	.ops = &max42500_hwmon_ops,
	.info = max42500_info,
};

static const struct regmap_config max42500_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x2D
	
};

static int max42500_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *hwmon_dev;
	struct max42500_state *st;
	int ret;
	
	st = devm_kzalloc(&client->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->client = client;
	st->regmap = devm_regmap_init_i2c(client, &max42500_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	st->vmon_en = max42500_config.vmon_en;
	st->vmon_vmpd = max42500_config.vmon_vmpd;
	st->reset_map = max42500_config.reset_map;
	st->wd_mode = max42500_config.wd_mode;
	st->wd_cdiv = max42500_config.wd_cdiv;
	st->wd_close = max42500_config.wd_close;
	st->wd_open = max42500_config.wd_open;
	st->wd_1ud = max42500_config.wd_1ud;
	st->wd_en = max42500_config.wd_en;
	st->pece = max42500_config.pece;
	if (st->pece)
		crc8_populate_msb(max42500_crc8, CRC8_PEC);

	hwmon_dev = devm_hwmon_device_register_with_info(&client->dev, client->name, st,
							 &max42500_chip_info, NULL);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}}

static const struct of_device_id max42500_of_match[] = {
	{ .compatible = "adi,max42500" },
	{ }
};
MODULE_DEVICE_TABLE(of, max42500_of_match);

static const struct i2c_device_id max42500_id[] = {
	{ "max42500", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max42500_id);

static struct i2c_driver max42500_driver = {
	.driver = {
		.name	= "max42500",
		.of_match_table = max42500_of_match,
	},
	.probe  	= max42500_probe,
	.id_table	= max42500_id,
};

module_i2c_driver(max42500_driver);

MODULE_AUTHOR("Kent Libetario <kent.libetario@analog.com>");
MODULE_DESCRIPTION("Hwmon driver for Linear Technology 2992");
MODULE_LICENSE("Dual BSD/GPL");

