/*
 * drivers/input/touchscreen/tsc2008.c
 *
 * Copyright (c) 2010 Cyber Switching, Inc.
 *    Chris Verges <chrisv@cyberswitching.com>
 *    Robert Mehranfar <robertme@earthlink.net>
 *
 * Using code from:
 *  - tsc2008.c
 *      Copyright (c) 2008 MtekVision Co., Ltd.
 *	Kwangwoo Lee <kwlee@mtekvision.com>
 *  - ads7846.c
 *	Copyright (c) 2005 David Brownell
 *	Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc2008.h>

#define TS_POLL_DELAY			1 /* ms delay between samples */
#define TS_POLL_PERIOD			1 /* ms delay between samples */

#define TSC2008_START_CMD		(0x1 << 7)

#define TSC2008_MEASURE_TEMP1		(0x0 << 4)
#define TSC2008_MEASURE_Y		(0x1 << 4)
#define TSC2008_SETUP			(0x2 << 4)
#define TSC2008_MEASURE_Z1		(0x3 << 4)
#define TSC2008_MEASURE_Z2		(0x4 << 4)
#define TSC2008_MEASURE_X		(0x5 << 4)
#define TSC2008_MEASURE_AUX		(0x6 << 4)
#define TSC2008_MEASURE_TEMP2		(0x7 << 4)

#define TSC2008_USE_50K			(0x0 << 3)
#define TSC2008_USE_90K			(0x1 << 3)

#define TSC2008_ENABLE_MAV		(0x0 << 2)
#define TSC2008_DISABLE_MAV		(0x1 << 2)

#define TSC2008_SOFTWARE_RESET		(0x1 << 0)

#define TSC2008_POWER_OFF_IRQ_EN	(0x0 << 0)
#define TSC2008_ADC_ON_IRQ_DIS0		(0x1 << 0)
#define TSC2008_ADC_OFF_IRQ_EN		(0x2 << 0)
#define TSC2008_ADC_ON_IRQ_DIS1		(0x3 << 0)

#define TSC2008_12BIT			(0x0 << 1)
#define TSC2008_8BIT			(0x1 << 1)

#define	MAX_12BIT			(1 << 12)
#define MAX_8BIT			(1 << 8)

#define ADC_ON_12BIT	(TSC2008_START_CMD | TSC2008_12BIT | TSC2008_POWER_OFF_IRQ_EN)
#define ADC_ON_8BIT	(TSC2008_START_CMD | TSC2008_8BIT | TSC2008_POWER_OFF_IRQ_EN)

/* usec, SCLK=25MHz w/o MAV */
#define TX_DELAY	(40)
#define RX_DELAY	(40)

struct ts_event {
	uint16_t	x;
	uint16_t	y;
	uint16_t	z1, z2;
};

struct tsc2008 {
	struct input_dev	*input;
	char			phys[32];
	struct delayed_work	work;

	struct spi_device	*spi;
	struct spi_message	msg;
	struct spi_transfer	xfer[8];
	uint8_t			tx_buf[4];

	uint16_t		model;
	uint16_t		x_plate_ohms;
	uint16_t		max_bits;

	bool			pendown;
	int			irq;

	int			(*get_pendown_state)(void);
	void			(*clear_penirq)(void);
};

static void tsc2008_read_values(struct tsc2008 *tsc, struct ts_event *tc)
{
	/* Read the values from the TSC2008 */
	tsc->xfer[1].rx_buf = &tc->x;
	tsc->xfer[3].rx_buf = &tc->y;
	tsc->xfer[5].rx_buf = &tc->z1;
	tsc->xfer[7].rx_buf = &tc->z2;

	spi_sync(tsc->spi, &tsc->msg);

	dev_dbg(&tsc->spi->dev, "data: x  %05u 0x%08x\n", tc->x, tc->x);
	dev_dbg(&tsc->spi->dev, "data: y  %05u 0x%08x\n", tc->y, tc->y);
	dev_dbg(&tsc->spi->dev, "data: z1 %05u 0x%08x\n", tc->z1, tc->z1);
	dev_dbg(&tsc->spi->dev, "data: z2 %05u 0x%08x\n", tc->z2, tc->z2);

	if (tsc->max_bits == MAX_8BIT) {
		tc->x  = (tc->x  << 1) & 0x00FF;
		tc->y  = (tc->y  << 1) & 0x00FF;
		tc->z1 = (tc->z1 << 1) & 0x00FF;
		tc->z2 = (tc->z2 << 1) & 0x00FF;
	} else if (tsc->max_bits == MAX_12BIT) {
		tc->x  = (be16_to_cpu(tc->x)  << 1) >> 4 & 0x0FFF;
		tc->y  = (be16_to_cpu(tc->y)  << 1) >> 4 & 0x0FFF;
		tc->z1 = (be16_to_cpu(tc->z1) << 1) >> 4 & 0x0FFF;
		tc->z2 = (be16_to_cpu(tc->z2) << 1) >> 4 & 0x0FFF;
	} else {
		dev_err(&tsc->spi->dev, "invalid number of bits expected\n");
	}

	dev_dbg(&tsc->spi->dev, "data: x  %05u 0x%08x\n", tc->x, tc->x);
	dev_dbg(&tsc->spi->dev, "data: y  %05u 0x%08x\n", tc->y, tc->y);
	dev_dbg(&tsc->spi->dev, "data: z1 %05u 0x%08x\n", tc->z1, tc->z1);
	dev_dbg(&tsc->spi->dev, "data: z2 %05u 0x%08x\n", tc->z2, tc->z2);
}

static uint32_t tsc2008_calculate_pressure(struct tsc2008 *tsc, struct ts_event *tc)
{
	uint32_t rt = 0;

	/* range filtering */
	if (tc->x == (tsc->max_bits - 1))
		tc->x = 0;

	if (likely(tc->x && tc->z1)) {
		rt = (tc->z2 / tc->z1) - 1;
		rt *= tc->y * tsc->x_plate_ohms;
		rt /= tsc->max_bits;

		if (tsc->max_bits == MAX_8BIT)
			rt &= 0x00FF;
		else
			rt &= 0x0FFF;
	}

	dev_dbg(&tsc->spi->dev, "pressure: %05u 0x%08x\n", rt, rt);

	return rt;
}

static void tsc2008_send_up_event(struct tsc2008 *tsc)
{
	struct input_dev *input = tsc->input;

	dev_dbg(&tsc->spi->dev, "UP\n");

	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);
}

static void tsc2008_work(struct work_struct *work)
{
	struct tsc2008 *ts =
		container_of(to_delayed_work(work), struct tsc2008, work);
	struct ts_event tc;
	uint32_t rt;

	/*
	 * NOTE: We can't rely on the pressure to determine the pen down
	 * state, even though this controller has a pressure sensor.
	 * The pressure value can fluctuate for quite a while after
	 * lifting the pen and in some cases may not even settle at the
	 * expected value.
	 *
	 * The only safe way to check for the pen up condition is in the
	 * work function by reading the pen signal state (it's a GPIO
	 * and IRQ). Unfortunately such callback is not always available,
	 * in that case we have rely on the pressure anyway.
	 */
	if (ts->get_pendown_state) {
		if (unlikely(!ts->get_pendown_state())) {
			dev_dbg(&ts->spi->dev, "detected pen UP via GPIO read\n");

			tsc2008_send_up_event(ts);
			ts->pendown = false;
			goto out;
		}

		dev_dbg(&ts->spi->dev, "pen is still down\n");
	} else {
		dev_dbg(&ts->spi->dev, "get_pendown_state undefined, "
				"using pressure instead\n");
	}

	tsc2008_read_values(ts, &tc);

	rt = tsc2008_calculate_pressure(ts, &tc);
	if (rt >= ts->max_bits) {
		/*
		 * Sample found inconsistent by debouncing or pressure is
		 * beyond the maximum. Don't report it to user space,
		 * repeat at least once more the measurement.
		 */
		dev_dbg(&ts->spi->dev, "ignored pressure %d\n", rt);
		goto out;
	}

	if (rt) {
		struct input_dev *input = ts->input;

		if (!ts->pendown) {
			dev_dbg(&ts->spi->dev, "DOWN\n");

			input_report_key(input, BTN_TOUCH, 1);
			ts->pendown = true;
		}

		input_report_abs(input, ABS_X, tc.x);
		input_report_abs(input, ABS_Y, tc.y);
		input_report_abs(input, ABS_PRESSURE, rt);

		input_sync(input);

		dev_dbg(&ts->spi->dev, "point(%4d,%4d), pressure (%4u)\n",
			tc.x, tc.y, rt);
	} else if (!ts->get_pendown_state && ts->pendown) {
		/*
		 * We don't have callback to check pendown state, so we
		 * have to assume that since pressure reported is 0 the
		 * pen was lifted up.
		 */
		tsc2008_send_up_event(ts);
		ts->pendown = false;
	} else {
		dev_dbg(&ts->spi->dev, "no change in pen state detected\n");
	}

 out:
	if (ts->pendown)
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(TS_POLL_PERIOD));
	else
		enable_irq(ts->irq);
}

static irqreturn_t tsc2008_irq(int irq, void *handle)
{
	struct tsc2008 *ts = handle;

	if (!ts->get_pendown_state || likely(ts->get_pendown_state())) {
		disable_irq_nosync(ts->irq);
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(TS_POLL_DELAY));
	}

	if (ts->clear_penirq)
		ts->clear_penirq();

	return IRQ_HANDLED;
}

static void tsc2008_free_irq(struct tsc2008 *ts)
{
	free_irq(ts->irq, ts);
	if (cancel_delayed_work_sync(&ts->work)) {
		/*
		 * Work was pending, therefore we need to enable
		 * IRQ here to balance the disable_irq() done in the
		 * interrupt handler.
		 */
		enable_irq(ts->irq);
	}
}

static int __devinit tsc2008_probe(struct spi_device *spi)
{
	struct tsc2008 *ts;
	struct tsc2008_platform_data *pdata = spi->dev.platform_data;
	struct input_dev *input_dev;
	struct spi_transfer *x;
	uint8_t tx_buf;
	int num_bytes_rx;
	int err;

	if (!pdata) {
		dev_err(&spi->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!pdata->get_pendown_state) {
		dev_err(&spi->dev, "pendown function is required!\n");
		return -EINVAL;
	}

	if (!pdata->bit_mode) {
		dev_dbg(&spi->dev, "bit mode not specified, using 8-bit mode\n");
		pdata->bit_mode = 8;
	} else if (pdata->bit_mode != 8 && pdata->bit_mode != 12) {
		dev_err(&spi->dev, "invalid bit mode specified, 8-bit or 12-bit expected\n");
		return -EINVAL;
	}

	err = spi_setup(spi);
	if (err) {
		dev_err(&spi->dev, "unable to initialize spi\n");
		return -ENODEV;
	}

	ts = kzalloc(sizeof(struct tsc2008), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	dev_set_drvdata(&spi->dev, ts);
	ts->spi = spi;
	ts->irq = spi->irq;
	ts->input = input_dev;
	INIT_DELAYED_WORK(&ts->work, tsc2008_work);

	ts->model             = pdata->model;
	ts->x_plate_ohms      = pdata->x_plate_ohms;
	ts->max_bits          = 1 << pdata->bit_mode;	/* Match either MAX_8BIT or MAX_12BIT */
	ts->get_pendown_state = pdata->get_pendown_state;
	ts->clear_penirq      = pdata->clear_penirq;

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&spi->dev));

	input_dev->name = "TSC2008 Touchscreen";
	input_dev->dev.parent = &spi->dev;
	input_dev->phys = ts->phys;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_X, 0, ts->max_bits, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, ts->max_bits, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, ts->max_bits, 0, 0);

	if (pdata->init_platform_hw)
		pdata->init_platform_hw();

	err = request_irq(ts->irq, tsc2008_irq, pdata->irq_flags,
			spi->dev.driver->name, ts);
	if (err < 0) {
		dev_err(&spi->dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}

	/* Setup SPI variables */
	spi_message_init(&ts->msg);

	/*
	 * TSC2008_MEASURE_X and TSC2008_MEASURE_Y are intentionally
	 * swapped below.
	 */
	if (ts->max_bits == MAX_8BIT) {
		dev_dbg(&spi->dev, "using 8-bit mode\n");
		num_bytes_rx = 1;
		ts->tx_buf[0] = ADC_ON_8BIT | TSC2008_MEASURE_Y;
		ts->tx_buf[1] = ADC_ON_8BIT | TSC2008_MEASURE_X;
		ts->tx_buf[2] = ADC_ON_8BIT | TSC2008_MEASURE_Z1;
		ts->tx_buf[3] = ADC_ON_8BIT | TSC2008_MEASURE_Z2;
	} else {
		dev_dbg(&spi->dev, "using 12-bit mode\n");
		num_bytes_rx = 2;
		ts->tx_buf[0] = ADC_ON_12BIT | TSC2008_MEASURE_Y;
		ts->tx_buf[1] = ADC_ON_12BIT | TSC2008_MEASURE_X;
		ts->tx_buf[2] = ADC_ON_12BIT | TSC2008_MEASURE_Z1;
		ts->tx_buf[3] = ADC_ON_12BIT | TSC2008_MEASURE_Z2;
	}

	x = &ts->xfer[0];
	memset(x, 0, sizeof(*x));
	x->tx_buf = &ts->tx_buf[0];
	x->len = 1;
	x->delay_usecs = TX_DELAY;
	spi_message_add_tail(x, &ts->msg);

	x = &ts->xfer[1];
	memset(x, 0, sizeof(*x));
	x->len = num_bytes_rx;
	x->delay_usecs = RX_DELAY;
	spi_message_add_tail(x, &ts->msg);

	x = &ts->xfer[2];
	memset(x, 0, sizeof(*x));
	x->tx_buf = &ts->tx_buf[1];
	x->len = 1;
	x->delay_usecs = TX_DELAY;
	spi_message_add_tail(x, &ts->msg);

	x = &ts->xfer[3];
	memset(x, 0, sizeof(*x));
	x->len = num_bytes_rx;
	x->delay_usecs = RX_DELAY;
	spi_message_add_tail(x, &ts->msg);

	x = &ts->xfer[4];
	memset(x, 0, sizeof(*x));
	x->tx_buf = &ts->tx_buf[2];
	x->len = 1;
	x->delay_usecs = TX_DELAY;
	spi_message_add_tail(x, &ts->msg);

	x = &ts->xfer[5];
	memset(x, 0, sizeof(*x));
	x->len = num_bytes_rx;
	x->delay_usecs = RX_DELAY;
	spi_message_add_tail(x, &ts->msg);

	x = &ts->xfer[6];
	memset(x, 0, sizeof(*x));
	x->tx_buf = &ts->tx_buf[3];
	x->len = 1;
	x->delay_usecs = TX_DELAY;
	spi_message_add_tail(x, &ts->msg);

	x = &ts->xfer[7];
	memset(x, 0, sizeof(*x));
	x->len = num_bytes_rx;
	spi_message_add_tail(x, &ts->msg);

	/*
	 * Reset the controller once using the sequence described
	 * on page 29 of the TSC2008 manual (March 2009 revision)
	 */
	tx_buf = TSC2008_START_CMD
			| TSC2008_SETUP
			| TSC2008_SOFTWARE_RESET;
	err = spi_write(spi, &tx_buf, 1);
	if (err < 0)
		goto err_free_irq;
	udelay(1);

	/* Configure the pull-up resistor and MAV settings */
	tx_buf = TSC2008_START_CMD
			| TSC2008_SETUP
			| TSC2008_USE_90K
			| TSC2008_DISABLE_MAV;
	err = spi_write(spi, &tx_buf, 1);
	if (err < 0)
		goto err_free_irq;

	/* Finish input device registration */
	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	return 0;

 err_free_irq:
	tsc2008_free_irq(ts);
	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int __devexit tsc2008_remove(struct spi_device *spi)
{
	struct tsc2008 *ts = dev_get_drvdata(&spi->dev);
	struct tsc2008_platform_data *pdata = spi->dev.platform_data;

	tsc2008_free_irq(ts);

	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();

	input_unregister_device(ts->input);
	kfree(ts);

	dev_dbg(&spi->dev, "unregistered touchscreen\n");

	return 0;
}

static struct spi_driver tsc2008_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tsc2008",
		.bus	= &spi_bus_type,
	},
	.probe		= tsc2008_probe,
	.remove		= __devexit_p(tsc2008_remove),
};

static int __init tsc2008_init(void)
{
	return spi_register_driver(&tsc2008_driver);
}

static void __exit tsc2008_exit(void)
{
	spi_unregister_driver(&tsc2008_driver);
}

module_init(tsc2008_init);
module_exit(tsc2008_exit);

MODULE_AUTHOR("Chris Verges <chrisv@cyberswitching.com>, "
		"Robert Mehranfar <robertme@earthlink.net>");
MODULE_DESCRIPTION("TSC2008 Touchscreen Driver");
MODULE_LICENSE("GPL");
