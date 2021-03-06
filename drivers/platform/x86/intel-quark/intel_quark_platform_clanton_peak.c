/*
 * Copyright(c) 2013 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details
 */
/*
 * Clanton Peak board entry point
 *
 */

#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/spi/spi_gpio.h>
#include <linux/spi/pxa2xx_spi.h>

#define DRIVER_NAME		"ClantonPeakSVP"
#define GPIO_RESTRICT_NAME_NC	"gpio-restrict-nc"
#define GPIO_RESTRICT_NAME_SC	"gpio-restrict-sc"


/* GPIO connected to Test Equipment */
#define SUT_GPIO_NC_3				0x03
#define SUT_GPIO_NC_4				0x04
#define SUT_GPIO_NC_5				0x05
#define SUT_GPIO_NC_6				0x06
#define SUT_GPIO_SC_2				0x0A
#define SUT_GPIO_SC_3				0x0B
#define SUT_GPIO_SC_4				0x0C
#define SUT_GPIO_SC_5				0x0D

#define GPIO_NC_BITBANG_SPI_BUS			2
#define GPIO_SC_BITBANG_SPI_BUS			3

/* GPIO numbers to use for SPI chipselect */
#define SPI0_CS0_GPIO				8
#define SPI0_CS1_GPIO				9
#define SPI0_CS2_GPIO				10
#define SPI0_CS3_GPIO				11
#define SPI1_CS0_GPIO				12
#define SPI1_CS1_GPIO				13
#define SPI1_CS2_GPIO				14
#define SPI1_CS3_GPIO				15

static int nc_gpio_reg;
static int sc_gpio_reg;

static int spi_eight_dev = 0;
module_param(spi_eight_dev, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(spi_eight_dev,
	"8-slave SPI test. off=0, on=!0 (default: off)");

static int spi_bbang = 1;
module_param(spi_bbang, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(spi_bbang,
	"Register two gpio SPI masters. off=0, on=!0 (default: on)");

/* SPI Controller Data (gpio-to-chipselect assignment) */
static struct pxa2xx_spi_chip spi_0_cs_0 = {
	.gpio_cs = SPI0_CS0_GPIO,
};
static struct pxa2xx_spi_chip spi_0_cs_1 = {
	.gpio_cs = SPI0_CS1_GPIO,
};
static struct pxa2xx_spi_chip spi_0_cs_2 = {
	.gpio_cs = SPI0_CS2_GPIO,
};
static struct pxa2xx_spi_chip spi_0_cs_3 = {
	.gpio_cs = SPI0_CS3_GPIO,
};
static struct pxa2xx_spi_chip spi_1_cs_0 = {
	.gpio_cs = SPI1_CS0_GPIO,
};
static struct pxa2xx_spi_chip spi_1_cs_1 = {
	.gpio_cs = SPI1_CS1_GPIO,
};
static struct pxa2xx_spi_chip spi_1_cs_2 = {
	.gpio_cs = SPI1_CS2_GPIO,
};
static struct pxa2xx_spi_chip spi_1_cs_3 = {
	.gpio_cs = SPI1_CS3_GPIO,
};

/* Default onboard SPI device declaration */
static struct spi_board_info spi_onboard_devs[] = {
	{
		.modalias = "spidev",
		.chip_select = 0,
		.max_speed_hz = 50000000,
		.bus_num = 0,
	},
	{
		.modalias = "spidev",
		.chip_select = 0,
		.max_speed_hz = 50000000,
		.bus_num = 1,
	},

};

#define QRK_SPIDEV_DATA_GPIO_CS(bus, cs, ptr_ctrl_data)		\
	{							\
		.modalias = "spidev",				\
		.max_speed_hz = 50000000,			\
		.mode = SPI_MODE_0,				\
		.bus_num = bus,					\
		.chip_select = cs,				\
		.controller_data = ptr_ctrl_data,		\
	}

/* 8-slave onboard SPI device declaration */
static struct spi_board_info spi_onboard_eight_devs[] = {
	QRK_SPIDEV_DATA_GPIO_CS(0, 0, &spi_0_cs_0),
	QRK_SPIDEV_DATA_GPIO_CS(0, 1, &spi_0_cs_1),
	QRK_SPIDEV_DATA_GPIO_CS(0, 2, &spi_0_cs_2),
	QRK_SPIDEV_DATA_GPIO_CS(0, 3, &spi_0_cs_3),
	QRK_SPIDEV_DATA_GPIO_CS(1, 0, &spi_1_cs_0),
	QRK_SPIDEV_DATA_GPIO_CS(1, 1, &spi_1_cs_1),
	QRK_SPIDEV_DATA_GPIO_CS(1, 2, &spi_1_cs_2),
	QRK_SPIDEV_DATA_GPIO_CS(1, 3, &spi_1_cs_3),
};

/*
 * Define platform data for bitbanged SPI devices.
 * Assign GPIO to SCK/MOSI/MISO
 */
static struct spi_gpio_platform_data spi_gpio_nc_data = {
	.sck = SUT_GPIO_NC_3,
	.mosi = SUT_GPIO_NC_4,
	.miso = SUT_GPIO_NC_5,
	.num_chipselect = 1,
};
static struct spi_gpio_platform_data spi_gpio_sc_data = {
	.sck = SUT_GPIO_SC_2,
	.mosi = SUT_GPIO_SC_3,
	.miso = SUT_GPIO_SC_4,
	.num_chipselect = 1,
};

/*
 * Board information for bitbanged SPI devices.
 */
static const struct spi_board_info spi_gpio_nc_bi[] = {
	{
	.modalias	= "spidev",
	.max_speed_hz	= 1000,
	.bus_num	= GPIO_NC_BITBANG_SPI_BUS,
	.mode		= SPI_MODE_0,
	.platform_data	= &spi_gpio_nc_data,
	/* Assign GPIO to CS */
	.controller_data = (void *)SUT_GPIO_NC_6,
	},
};
static const struct spi_board_info spi_gpio_sc_bi[] = {
	{
	.modalias	= "spidev",
	.max_speed_hz	= 1000,
	.bus_num	= GPIO_SC_BITBANG_SPI_BUS,
	.mode		= SPI_MODE_0,
	.platform_data	= &spi_gpio_sc_data,
	/* Assign GPIO to CS */
	.controller_data = (void *)SUT_GPIO_SC_5,
	},
};

static struct platform_device spi_gpio_nc_pd = {
	.name	= "spi_gpio",
	.id	= GPIO_NC_BITBANG_SPI_BUS,
	.dev	= {
		.platform_data = &spi_gpio_nc_data,
	},
};

static struct platform_device spi_gpio_sc_pd = {
	.name	= "spi_gpio",
	.id	= GPIO_SC_BITBANG_SPI_BUS,
	.dev	= {
		.platform_data = &spi_gpio_sc_data,
	},
};

/**
 * intel_qrk_spi_add_onboard_devs
 *
 * @return 0 on success or standard errnos on failure
 *
 * Registers onboard SPI device(s) present on the Clanton Peak platform
 */
static int intel_qrk_spi_add_onboard_devs(void)
{
	return spi_register_board_info(spi_onboard_devs,
			ARRAY_SIZE(spi_onboard_devs));
}

static int intel_qrk_spi_add_onboard_eight_devs(void)
{
	return spi_register_board_info(spi_onboard_eight_devs,
			ARRAY_SIZE(spi_onboard_eight_devs));
}

static int register_bitbanged_spi(int nc)
{
	int ret = 0;

	ret = platform_device_register(nc ? &spi_gpio_nc_pd : &spi_gpio_sc_pd);
	if (ret)
		goto err;

	ret = spi_register_board_info(nc ? spi_gpio_nc_bi : spi_gpio_sc_bi,
				      nc ? ARRAY_SIZE(spi_gpio_nc_bi) :
					   ARRAY_SIZE(spi_gpio_sc_bi));
	if (ret)
		goto err_unregister;

	return 0;

err_unregister:
	platform_device_unregister(nc ? &spi_gpio_nc_pd : &spi_gpio_sc_pd);
err:
	return ret;
}

static int intel_qrk_gpio_restrict_probe_nc(struct platform_device *pdev)
{
	int ret = 0;
	nc_gpio_reg = 1;

	if (spi_eight_dev) {
		if (nc_gpio_reg == 1 && sc_gpio_reg == 1) {
			ret = intel_qrk_spi_add_onboard_eight_devs();
		}
	} else if (spi_bbang) {
		ret = register_bitbanged_spi(1);
	}
	return ret;
}

static int intel_qrk_gpio_restrict_probe_sc(struct platform_device *pdev)
{
	int ret = 0;
	sc_gpio_reg = 1;

	if (spi_eight_dev) {
		if (nc_gpio_reg == 1 && sc_gpio_reg == 1) {
			ret = intel_qrk_spi_add_onboard_eight_devs();
		}
	} else if (spi_bbang) {
		ret = register_bitbanged_spi(0);
	}
	return ret;
}

static struct platform_driver gpio_restrict_pdriver_nc = {
	.driver		= {
		.name	= GPIO_RESTRICT_NAME_NC,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_qrk_gpio_restrict_probe_nc,
};

static struct platform_driver gpio_restrict_pdriver_sc = {
	.driver		= {
		.name	= GPIO_RESTRICT_NAME_SC,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_qrk_gpio_restrict_probe_sc,
};

static int intel_qrk_plat_clanton_peak_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = platform_driver_register(&gpio_restrict_pdriver_nc);
	if (ret) {
		pr_err("%s: couldn't register %s platform driver\n",
		       __func__, gpio_restrict_pdriver_nc.driver.name);
	}

	ret = platform_driver_register(&gpio_restrict_pdriver_sc);
	if (ret) {
		pr_err("%s: couldn't register %s platform driver\n",
		       __func__, gpio_restrict_pdriver_sc.driver.name);
	}

	if (0 == spi_eight_dev) {
		ret = intel_qrk_spi_add_onboard_devs();
	}
	return ret;
}

static int intel_qrk_plat_clanton_peak_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver clanton_peak_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_qrk_plat_clanton_peak_probe,
	.remove		= intel_qrk_plat_clanton_peak_remove,
};

module_platform_driver(clanton_peak_driver);

MODULE_AUTHOR("Bryan O'Donoghue <bryan.odonoghue@intel.com>");
MODULE_DESCRIPTION("Clanton Peak BSP Data");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:"DRIVER_NAME);
