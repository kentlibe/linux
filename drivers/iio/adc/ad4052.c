// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD4052 SPI ADC driver
 *
 * Copyright 2024 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/units.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/pwm.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi-engine-ex.h>
#include <linux/iio/iio.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#define AD4052_REG_DEVICE_CONFIG	0x02
#define AD4052_REG_PROD_ID_1		0x05
#define AD4052_REG_VENDOR_H		0x0D
#define AD4052_REG_MODE_SET		0x20
#define AD4052_REG_ADC_MODES		0x21
#define AD4052_REG_AVG_CONFIG		0x23
#define AD4052_REG_GP_CONFIG		0x24
#define AD4052_REG_INTR_CONFIG		0x25
#define AD4052_REG_TIMER_CONFIG		0x27
#define AD4052_REG_MAX_LIMIT		0x29
#define AD4052_REG_MIN_LIMIT		0x2B
#define AD4052_REG_MAX_HYST		0x2C
#define AD4052_REG_MIN_HYST		0x2D
#define AD4052_REG_DEVICE_STATUS	0x41
/* GP_CONFIG */
#define AD4052_GP_MODE_MSK(x)		(GENMASK(2, 0) << (x) * 4)
#define AD4052_GP_MODE(x, y)		((y) << (x) * 4)
/* INTR_CONFIG */
#define AD4052_INTR_EN_MSK(x)		(GENMASK(1, 0) << (x) * 4)
#define AD4052_INTR_EN(x, y)		((y) << (x) * 4)
/* ADC_MODES */
#define AD4052_DATA_FORMAT		BIT(7)
#define AD4052_SIGN_EXTEND		BIT(6)
/* DEVICE_CONFIG */
#define AD4052_POWER_MODE_MSK		GENMASK(1, 0)
#define AD4052_LOW_POWER_MODE		3
/* DEVICE_STATUS */
#define AD4052_DEVICE_RESET		BIT(6)
#define AD4052_MAX_THRESH_INTR		BIT(3)
#define AD4052_MIN_THRESH_INTR		BIT(2)

#define AD4052_SPI_VENDOR		0x0456
#define AD4052_SPI_MAX_SCLK		25000000

enum ad4052_device_type {
	ID_AD4050,
	ID_AD4056,
	ID_AD4052,
	ID_AD4058,
};

enum ad4052_operation_mode {
	AD4052_SAMPLE_MODE = 0,
	AD4052_BURST_AVERAGING_MODE = 1,
	AD4052_AVERAGING_MODE = 2,
	AD4052_MONITOR_MODE = 3,
	AD4052_TRIGGER_MODE = 7,
	AD4052_CONFIG_MODE = 8,
};

enum ad4052_sample_rate {
	AD4052_2_MSPS,
	AD4052_1_MSPS,
	AD4052_333_KSPS,
	AD4052_100_KSPS,
	AD4052_33_KSPS,
	AD4052_10_KSPS,
	AD4052_3_KSPS,
	AD4052_1_KSPS,
	AD4052_500_SPS,
	AD4052_333_SPS,
	AD4052_250_SPS,
	AD4052_200_SPS,
	AD4052_166_SPS,
	AD4052_140_SPS,
	AD4052_125_SPS,
	AD4052_111_SPS
};

enum ad4052_gp_mode {
	AD4052_GP_DISABLED,
	AD4052_GP_INTR,
	AD4052_GP_DRDY
};

enum ad4052_interrupt_en {
	AD4052_INTR_EN_NEITHER,
	AD4052_INTR_EN_MIN,
	AD4052_INTR_EN_MAX,
	AD4052_INTR_EN_EITHER
};

struct ad4052_functional_mode {
	const struct iio_chan_spec channels[1];
};

struct ad4052_chip_info {
	const struct ad4052_functional_mode *modes;
	const char *name;
	int interface;
	u16 prod_id;
};

struct ad4052_state {
	const struct	ad4052_bus_ops *ops;
	const struct	ad4052_chip_info *chip;
	struct spi_device	*spi;
	struct spi_transfer	offload_xfer;
	struct spi_message	offload_msg;
	struct pwm_device	*cnv_pwm;
	struct gpio_desc	*cnv_gp;
	struct regmap		*regmap;
	enum ad4052_operation_mode	functional_mode;
	enum ad4052_operation_mode	current_mode;
	enum ad4052_sample_rate		rate;
	__be32 d32 ____cacheline_aligned;
	bool trigger_mode;
	u8 data_format;
};

static int ad4052_set_avg_filter(struct iio_dev *dev,
				 const struct iio_chan_spec *chan,
				    unsigned int val);
static int ad4052_get_avg_filter(struct iio_dev *dev,
				 const struct iio_chan_spec *chan);

static const struct iio_event_spec ad4052_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_shared_by_all = BIT(IIO_EV_INFO_ENABLE)
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_shared_by_all = BIT(IIO_EV_INFO_VALUE) |
				      BIT(IIO_EV_INFO_HYSTERESIS)
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_shared_by_all = BIT(IIO_EV_INFO_VALUE) |
				      BIT(IIO_EV_INFO_HYSTERESIS)
	}
};

static const char *const ad4052_avg_filter[] = {
	"2", "4", "8", "16", "32", "64", "128", "256", "512", "1024",
	"2048", "4096"
};

static const struct iio_enum ad4052_avg_filter_enum = {
	.items = ad4052_avg_filter,
	.num_items = ARRAY_SIZE(ad4052_avg_filter),
	.set = ad4052_set_avg_filter,
	.get = ad4052_get_avg_filter,
};

static const struct iio_chan_spec_ext_info ad4052_ext_info[] = {
	IIO_ENUM("averaging_filter", IIO_SHARED_BY_TYPE,
		 &ad4052_avg_filter_enum),
	IIO_ENUM_AVAILABLE("averaging_filter", IIO_SHARED_BY_TYPE,
			   &ad4052_avg_filter_enum),
	{}
};

#define AD4052_CHAN(_realbits, _ext_info) {				\
	.type = IIO_VOLTAGE,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.indexed = 1,							\
	.event_spec = ad4052_events,					\
	.num_event_specs = ARRAY_SIZE(ad4052_events),			\
	.ext_info = _ext_info,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = _realbits,					\
		.storagebits = 32					\
	},								\
}

static const struct ad4052_functional_mode ad4050_modes[] = {
	[AD4052_SAMPLE_MODE] = {
		.channels = {
			AD4052_CHAN(12, NULL)
		},
	},
	[AD4052_BURST_AVERAGING_MODE] = {
		.channels = {
			AD4052_CHAN(16, ad4052_ext_info)
		},
	},
};

static const struct ad4052_functional_mode ad4052_modes[] = {
	[AD4052_SAMPLE_MODE] = {
		.channels = {
			AD4052_CHAN(16, NULL)
		},
	},
	[AD4052_BURST_AVERAGING_MODE] = {
		.channels = {
			AD4052_CHAN(24, ad4052_ext_info)
		},
	},
};

const struct ad4052_chip_info ad4052_chip_info[] = {
	[ID_AD4050] = {
		.name = "ad4050",
		.modes = ad4050_modes,
		.prod_id = 0x70,
	},
	[ID_AD4052] = {
		.name = "ad4052",
		.modes = ad4052_modes,
		.prod_id = 0x72,
	},
	[ID_AD4056] = {
		.name = "ad4056",
		.modes = ad4050_modes,
		.prod_id = 0x70,
	},
	[ID_AD4058] = {
		.name = "ad4058",
		.modes = ad4052_modes,
		.prod_id = 0x72,
	},
};

static int ad4052_exit_command(struct ad4052_state *st)
{
	struct spi_device *spi = st->spi;
	const u8 buf = 0xA8;

	return spi_write(spi, &buf, 1);
}

static int ad4052_set_operation_mode(struct ad4052_state *st, enum ad4052_operation_mode mode)
{
	struct spi_device *spi = st->spi;
	u8 buf[2];
	int ret;

	if (mode == st->current_mode)
		return 0;

	if (st->current_mode != AD4052_CONFIG_MODE) {
		ret = ad4052_exit_command(st);
		if (ret)
			return ret;
	}

	if (mode != AD4052_CONFIG_MODE) {
		buf[0] = AD4052_REG_ADC_MODES;
		buf[1] = st->data_format | mode;
		ret = spi_write(spi, buf, 2);
		if (ret)
			return ret;

		buf[0] = AD4052_REG_MODE_SET;
		buf[1] = BIT(0);
		ret = spi_write(spi, buf, 2);
		if (ret)
			return ret;
	}

	st->current_mode = mode;

	return 0;
};

static int ad4052_set_avg_filter(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 unsigned int val)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	u8 buf = val;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	buf = val;
	ret = regmap_write(st->regmap, AD4052_REG_AVG_CONFIG, buf);

	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int ad4052_get_avg_filter(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int buf;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, AD4052_REG_AVG_CONFIG, &buf);
	iio_device_release_direct_mode(indio_dev);
	if (ret)
		return ret;

	return buf;
}

static int ad4052_assert(struct ad4052_state *st)
{
	__be16 value;
	int ret;

	ret = regmap_bulk_read(st->regmap, AD4052_REG_PROD_ID_1, &value, 2);
	value = be16_to_cpu(value);
	if (ret)
		return ret;

	if (value != st->chip->prod_id)
		return -ENODEV;

	ret = regmap_bulk_read(st->regmap, AD4052_REG_VENDOR_H, &value, 2);
	value = be16_to_cpu(value);
	if (ret)
		return ret;

	if (value != AD4052_SPI_VENDOR)
		return -ENODEV;

	return 0;
}

static int ad4052_config(struct ad4052_state *st)
{
	struct device *dev = &st->spi->dev;
	struct pwm_state conv_state;
	u8 realbits;
	int ret;

	ret = device_property_read_u32(dev, "adi,functional-mode", &st->functional_mode);
	if (!ret) {
		if (st->functional_mode > AD4052_BURST_AVERAGING_MODE)
			return dev_err_probe(dev, -EINVAL, "Invalid functional mode(%u)\n",
					     st->functional_mode);
	}
	/*
	 * Receive buffer needs to be non-zero for the SPI engine controller
	 * to mark the transfer as a read.
	 */
	realbits = st->chip->modes[st->functional_mode].channels[0].scan_type.realbits;
	/* Optimize message requires power of two */
	st->offload_xfer.len = roundup_pow_of_two(BITS_TO_BYTES(realbits));
	st->offload_xfer.speed_hz = AD4052_SPI_MAX_SCLK;
	st->offload_xfer.rx_buf = (void *)-1;
	st->offload_xfer.bits_per_word = 16;

	spi_message_init_with_transfers(&st->offload_msg, &st->offload_xfer, 1);

	/* Prepare PWM CNV */
	st->cnv_pwm = devm_pwm_get(dev, "cnv");
	if (IS_ERR(st->cnv_pwm)) {
		return dev_err_probe(dev, PTR_ERR(st->cnv_pwm),
				     "Failed to get cnv pwm\n");
	}
	pwm_init_state(st->cnv_pwm, &conv_state);
	conv_state.duty_cycle = 20;
	conv_state.period = 1000;
	conv_state.enabled = false;
	ret = pwm_apply_state(st->cnv_pwm, &conv_state);
	if (ret)
		return ret;

	/* Prepare GPIO CNV */
	st->cnv_gp = devm_gpiod_get_optional(dev, "cnv",
					     GPIOD_OUT_LOW);
	if (IS_ERR(st->cnv_gp)) {
		return dev_err_probe(dev, PTR_ERR(st->cnv_gp),
				    "Failed to get cnv gpio\n");
	}

	return 0;
}

static int ad4052_soft_reset(struct spi_device *spi)
{
	int ret;
	u8 buf[18] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE,
		      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE,
		      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE
		     };

	ret = spi_write(spi, buf, sizeof(buf));

	if (ret)
		return ret;

	/* Wait at least AD4052 Tswreset time - 500us */
	if (msleep_interruptible(1))
		return -ERESTARTSYS;

	return 0;
};

int ad4052_update_bits(struct ad4052_state *st, u8 reg, u8 mask, u8 val)
{
	int ret;
	int buf;

	ret = regmap_read(st->regmap, reg, &buf);
	if (ret)
		return ret;

	buf &= ~mask;
	buf |= val;

	return regmap_write(st->regmap, reg, buf);
};

static int ad4052_set_non_defaults(struct ad4052_state *st)
{
	struct iio_chan_spec chan = st->chip->modes[st->functional_mode].channels[0];
	u8 val = AD4052_GP_MODE(1, AD4052_GP_DRDY);
	int ret;

	ret = ad4052_update_bits(st, AD4052_REG_GP_CONFIG, AD4052_GP_MODE_MSK(1),
				 val);
	if (ret)
		return ret;

	val = AD4052_INTR_EN(0, AD4052_INTR_EN_EITHER) |
	      AD4052_INTR_EN(1, AD4052_INTR_EN_NEITHER);

	ret = ad4052_update_bits(st, AD4052_REG_INTR_CONFIG,
				 AD4052_INTR_EN_MSK(0) |
				 AD4052_INTR_EN_MSK(1), val);

	val = 0;
	if (chan.scan_type.sign == 's')
		val |= AD4052_DATA_FORMAT;

	if (st->offload_xfer.len == 4)
		val |= AD4052_SIGN_EXTEND;
	st->data_format = val;

	return regmap_write(st->regmap, AD4052_REG_ADC_MODES, val);
}

irqreturn_t ad4052_interrupt_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct ad4052_state *st = iio_priv(indio_dev);
	int buf = 0;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret || !st->trigger_mode)
		return IRQ_HANDLED;

	st->trigger_mode = false;
	/* Switched to ADC_MODE from NONPERSISTENT_AUTO_MODE on event */
	st->current_mode = AD4052_SAMPLE_MODE;
	regmap_read(st->regmap, AD4052_REG_DEVICE_STATUS, &buf);

	if (buf & AD4052_MAX_THRESH_INTR) {
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, 0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_RISING),
			       iio_get_time_ns(indio_dev));
	}

	if (buf & AD4052_MIN_THRESH_INTR) {
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, 0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_FALLING),
			       iio_get_time_ns(indio_dev));
	}

	st->trigger_mode = true;
	regmap_write(st->regmap, AD4052_REG_DEVICE_STATUS, buf);
	iio_device_release_direct_mode(indio_dev);

	return IRQ_HANDLED;
}

static int ad4052_request_irq(struct iio_dev *indio_dev)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	int irq;
	int ret;

	irq = of_irq_get(dev->of_node, 0);
	/* Optional */
	if (irq > 0) {
		ret = devm_request_threaded_irq(dev, irq,
						NULL, ad4052_interrupt_handler,
						IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						indio_dev->name, indio_dev);
		if (ret) {
			dev_err(dev, "irq request for gp0 error %d.\n", ret);
			return ret;
		}
	} else if (irq < 0) {
		dev_err(dev, "irq get for gp0 error %d.\n", irq);
		return irq;
	}

	return ret;
}

static int ad4052_set_sampling_freq(const struct ad4052_state *st, unsigned int freq)
{
	struct spi_device *spi = st->spi;
	struct pwm_state conv_state;

	if (freq > spi->max_speed_hz)
		return -EINVAL;

	pwm_get_state(st->cnv_pwm, &conv_state);
	conv_state.period = DIV_ROUND_CLOSEST(NSEC_PER_SEC, freq);
	return pwm_apply_state(st->cnv_pwm, &conv_state);
}

static void ad4052_get_sampling_freq(const struct ad4052_state *st, int *val)
{
	struct pwm_state conversion_state;

	pwm_get_state(st->cnv_pwm, &conversion_state);
	*val = DIV_ROUND_CLOSEST_ULL(NANO, conversion_state.period);
}

static int __ad4052_read_chan_raw(struct ad4052_state *st, int *val)
{
	struct spi_device *spi = st->spi;
	short size;
	int ret;

	size = st->offload_xfer.len;

	struct spi_transfer t[] = {
		{
			.rx_buf = &st->d32,
			.len = size,
		},
	};

	gpiod_set_value(st->cnv_gp, 1);
	/* Takes > 3us */
	ret = spi_sync_transfer(spi, t, ARRAY_SIZE(t));
	gpiod_set_value(st->cnv_gp, 0);
	if (ret)
		return ret;

	if (size == 2) {
		*val = be16_to_cpu(st->d32);
		if (st->data_format & AD4052_DATA_FORMAT)
			*val = sign_extend32(*val, 15);
	} else {
		*val = be32_to_cpu(st->d32);
	}

	return 0;
};

static int ad4052_read_chan_raw(struct iio_dev *indio_dev, int index, int *val)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int ret;

	ret = pm_runtime_resume_and_get(&st->spi->dev);
	if (ret)
		return ret;

	switch (index) {
	case 0:
		ret = ad4052_set_operation_mode(st, st->functional_mode);
		if (ret)
			return ret;
		break;
	default:
		return -ENOENT;
	}

	ret = __ad4052_read_chan_raw(st, val);
	if (ret)
		goto out_error;

	if (st->trigger_mode)
		ret = ad4052_set_operation_mode(st, AD4052_TRIGGER_MODE);

out_error:
	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);
	return ret;
}

static int ad4052_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
			ret = ad4052_read_chan_raw(indio_dev, chan->channel, val);
			if (ret < 0)
				return ret;

			return IIO_VAL_INT;
		}
		unreachable();
	case IIO_CHAN_INFO_SAMP_FREQ:
		ad4052_get_sampling_freq(st, val);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad4052_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val,
			    int val2, long info)
{
	const struct ad4052_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
			return ad4052_set_sampling_freq(st, val);
		}
		unreachable();
	default:
		return -EINVAL;
	}
}

static int ad4052_read_event_config(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int state;
	int ret;

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		switch (dir) {
		case IIO_EV_DIR_EITHER:
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
		ret = regmap_read(st->regmap, AD4052_REG_GP_CONFIG, &state);
		if (!ret)
			ret = state & AD4052_GP_MODE_MSK(0);
		return ret;
	}
	unreachable();
}

static int ad4052_write_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     int state)
{
	u8 val = state ? AD4052_GP_INTR : AD4052_GP_DISABLED;
	struct ad4052_state *st = iio_priv(indio_dev);
	int ret;

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		switch (dir) {
		case IIO_EV_DIR_EITHER:
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (st->trigger_mode == state)
		return 0;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	st->trigger_mode = false;
	val = AD4052_GP_MODE(0, val);
	ret = ad4052_update_bits(st, AD4052_REG_GP_CONFIG,
				 AD4052_GP_MODE_MSK(0),
				 val);
	if (ret)
		return ret;

	if (state) {
		ret = pm_runtime_resume_and_get(&st->spi->dev);
		if (ret)
			return ret;

		ret = ad4052_set_operation_mode(st, AD4052_TRIGGER_MODE);
		if (ret)
			return ret;
	} else {
		pm_runtime_mark_last_busy(&st->spi->dev);
		pm_runtime_put_autosuspend(&st->spi->dev);
	}
	st->trigger_mode = state;
	iio_device_release_direct_mode(indio_dev);

	return 0;
}

static int ad4052_read_event_value(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info, int *val, int *val2)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	u8 size = 1;
	int ret;
	u16 buf;
	u8 reg;

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		switch (dir) {
		case IIO_EV_DIR_RISING:
		case IIO_EV_DIR_FALLING:
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		switch (info) {
		case IIO_EV_INFO_VALUE:
			if (dir == IIO_EV_DIR_RISING)
				reg = AD4052_REG_MAX_LIMIT;
			else
				reg = AD4052_REG_MIN_LIMIT;
			size++;
			break;
		case IIO_EV_INFO_HYSTERESIS:
			if (dir == IIO_EV_DIR_RISING)
				reg = AD4052_REG_MAX_HYST;
			else
				reg = AD4052_REG_MIN_HYST;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
		ret = regmap_bulk_read(st->regmap, reg, &buf, size);
		if (ret)
			return ret;

		if (reg == AD4052_REG_MAX_LIMIT || reg == AD4052_REG_MIN_LIMIT) {
			*val = be16_to_cpu(buf);
			if (st->data_format & AD4052_DATA_FORMAT)
				*val = sign_extend32(*val, 11);
		} else {
			*val = (u8)buf;
		}
		return IIO_VAL_INT;
	}
	unreachable();
}

static int ad4052_write_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info, int val, int val2)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	u16 buf = cpu_to_be16(val);
	u8 size = 1;
	int ret;
	u8 reg;

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		switch (dir) {
		case IIO_EV_DIR_RISING:
		case IIO_EV_DIR_FALLING:
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		switch (info) {
		case IIO_EV_INFO_VALUE:
			if (st->data_format & AD4052_DATA_FORMAT) {
				if (val > 2047 || val < -2048)
					return -EINVAL;
			} else if (val > 4095 || val < 0) {
				return -EINVAL;
			}
			if (dir == IIO_EV_DIR_RISING)
				reg = AD4052_REG_MAX_LIMIT;
			else
				reg = AD4052_REG_MIN_LIMIT;
			size++;
			break;
		case IIO_EV_INFO_HYSTERESIS:
			if (val & BIT(7))
				return -EINVAL;
			if (dir == IIO_EV_DIR_RISING)
				reg = AD4052_REG_MAX_HYST;
			else
				reg = AD4052_REG_MIN_HYST;
			buf >>= 8;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
		ret = regmap_bulk_write(st->regmap, reg, &buf, size);
		return ret;
	}
	unreachable();
}

static int ad4052_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	struct spi_device *spi = st->spi;
	struct pwm_state conv_state;
	int ret;

	ret = pm_runtime_resume_and_get(&st->spi->dev);
	if (ret)
		return ret;

	pwm_get_state(st->cnv_pwm, &conv_state);
	conv_state.enabled = true;
	ret = pwm_apply_state(st->cnv_pwm, &conv_state);
	if (ret)
		goto out_error;

	ret = ad4052_set_operation_mode(st, st->functional_mode);
	if (ret)
		goto out_error;

	ret = spi_optimize_message(spi, &st->offload_msg);
	if (ret)
		goto out_error;

	spi_bus_lock(spi->controller);
	spi_engine_ex_offload_load_msg(spi, &st->offload_msg);
	spi_engine_ex_offload_enable(spi, true);

	return 0;
out_error:
	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);
	return ret;
}

static int ad4052_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	struct spi_device *spi = st->spi;
	struct pwm_state conv_state;
	int ret;

	spi_engine_ex_offload_enable(spi, false);
	spi_bus_unlock(spi->master);
	spi_unoptimize_message(&st->offload_msg);

	pwm_get_state(st->cnv_pwm, &conv_state);
	conv_state.enabled = false;
	ret = pwm_apply_state(st->cnv_pwm, &conv_state);

	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);
	return ret;
}

static int ad4052_spi_read(void *context, const void *reg, size_t reg_size,
			   void *val, size_t val_size)
{
	struct ad4052_state *st = context;
	int ret;
	struct spi_transfer t[] = {
		{
			.tx_buf = reg,
			.len = reg_size,
			.cs_change = 0
		}, {
			.rx_buf = val,
			.len = val_size,
		},
	};
	ret = ad4052_set_operation_mode(st, AD4052_CONFIG_MODE);
	if (ret)
		return ret;

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret)
		return ret;

	if (st->trigger_mode)
		ret = ad4052_set_operation_mode(st, AD4052_TRIGGER_MODE);

	return ret;
}

static int ad4052_spi_write(void *context, const void *data, size_t count)
{
	struct ad4052_state *st = context;
	int ret;

	ret = ad4052_set_operation_mode(st, AD4052_CONFIG_MODE);
	if (ret)
		return ret;

	ret = spi_write(st->spi, data, count);
	if (ret)
		return ret;

	if (st->trigger_mode)
		ret = ad4052_set_operation_mode(st, AD4052_TRIGGER_MODE);

	return ret;
}

static int ad4052_debugfs_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				     unsigned int writeval, unsigned int *readval)
{
	struct ad4052_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_buffer_setup_ops ad4052_buffer_setup_ops = {
	.preenable = &ad4052_buffer_preenable,
	.postdisable = &ad4052_buffer_postdisable,
};

static const struct iio_info ad4052_info = {
	.read_raw = ad4052_read_raw,
	.write_raw = ad4052_write_raw,
	.read_event_config = &ad4052_read_event_config,
	.write_event_config = &ad4052_write_event_config,
	.read_event_value = &ad4052_read_event_value,
	.write_event_value = &ad4052_write_event_value,
	.debugfs_reg_access = &ad4052_debugfs_reg_access,
};

static const struct regmap_bus ad4052_regmap_bus = {
	.read = ad4052_spi_read,
	.write = ad4052_spi_write,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
};

static const struct regmap_config ad4052_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static void ad4052_pm_disable(void *data)
{
	pm_runtime_disable(data);
}

static int ad4052_probe(struct spi_device *spi)
{
	const struct ad4052_chip_info *chip;
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad4052_state *st;
	int ret;
	u8 buf;

	chip = spi_get_device_match_data(spi);
	if (!chip)
		return dev_err_probe(dev, -ENODEV,
				     "Could not find chip info data\n");

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	spi_set_drvdata(spi, st);

	st->regmap = devm_regmap_init(&spi->dev, &ad4052_regmap_bus, st,
				      &ad4052_regmap_config);
	if (IS_ERR(st->regmap))
		dev_err_probe(&spi->dev,  PTR_ERR(st->regmap),
			      "Failed to initialize regmap\n");

	st->current_mode = AD4052_CONFIG_MODE;
	st->rate = AD4052_2_MSPS;
	st->trigger_mode = false;
	st->chip = chip;

	ret = ad4052_config(st);
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "Config failed: %d\n", ret);

	indio_dev->modes = INDIO_BUFFER_HARDWARE | INDIO_DIRECT_MODE;
	indio_dev->channels = chip->modes[st->functional_mode].channels;
	indio_dev->setup_ops = &ad4052_buffer_setup_ops;
	indio_dev->num_channels = 1;
	indio_dev->info = &ad4052_info;
	indio_dev->name = chip->name;

	ret = devm_iio_dmaengine_buffer_setup(dev, indio_dev, "rx",
					      IIO_BUFFER_DIRECTION_IN);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get DMA buffer\n");

	ret = ad4052_soft_reset(spi);
	if (ret) {
		dev_err(dev, "AD4052 failed to soft reset %d.\n", ret);
		return ret;
	}

	ret = ad4052_assert(st);
	if (ret) {
		dev_err(dev, "AD4052 fields assertions failed.\n");
		return ret;
	}

	ret = ad4052_set_non_defaults(st);
	if (ret)
		return ret;

	buf = AD4052_DEVICE_RESET;
	regmap_write(st->regmap, AD4052_REG_DEVICE_STATUS, buf);

	ret = ad4052_request_irq(indio_dev);
	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = devm_add_action_or_reset(dev, ad4052_pm_disable, &spi->dev);

	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static int ad4052_runtime_suspend(struct device *dev)
{
	u8 val = FIELD_PREP(AD4052_POWER_MODE_MSK, AD4052_LOW_POWER_MODE);
	struct ad4052_state *st = dev_get_drvdata(dev);

	return regmap_write(st->regmap, AD4052_REG_DEVICE_CONFIG, val);
}

static int ad4052_runtime_resume(struct device *dev)
{
	struct ad4052_state *st = dev_get_drvdata(dev);
	u8 val = FIELD_PREP(AD4052_POWER_MODE_MSK, 0);

	return regmap_write(st->regmap, AD4052_REG_DEVICE_CONFIG, val);
}

static const struct dev_pm_ops ad4052_pm_ops = {
	SET_RUNTIME_PM_OPS(ad4052_runtime_suspend, ad4052_runtime_resume, NULL)
};

static const struct spi_device_id ad4052_id_table[] = {
	{"ad4050", (kernel_ulong_t)&ad4052_chip_info[ID_AD4050] },
	{"ad4056", (kernel_ulong_t)&ad4052_chip_info[ID_AD4056] },
	{"ad4052", (kernel_ulong_t)&ad4052_chip_info[ID_AD4052] },
	{"ad4058", (kernel_ulong_t)&ad4052_chip_info[ID_AD4058] },
	{}
};
MODULE_DEVICE_TABLE(spi, ad4052_id_table);

static const struct of_device_id ad4052_of_match[] = {
	{ .compatible = "adi,ad4050", .data = &ad4052_chip_info[ID_AD4050] },
	{ .compatible = "adi,ad4052", .data = &ad4052_chip_info[ID_AD4052] },
	{ .compatible = "adi,ad4056", .data = &ad4052_chip_info[ID_AD4056] },
	{ .compatible = "adi,ad4058", .data = &ad4052_chip_info[ID_AD4058] },
	{}
};
MODULE_DEVICE_TABLE(of, ad4052_of_match);

static struct spi_driver ad4052_driver = {
	.driver = {
		.name = "ad4052",
		.of_match_table = ad4052_of_match,
		.pm = pm_ptr(&ad4052_pm_ops),
	},
	.probe = ad4052_probe,
	.id_table = ad4052_id_table,
};
module_spi_driver(ad4052_driver);

MODULE_AUTHOR("Jorge Marques <jorge.marques@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4052");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_AD4052);
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
