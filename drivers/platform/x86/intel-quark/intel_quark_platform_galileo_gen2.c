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
 * more details.
 */
/*
 * Intel Quark Legacy Platform Data accessor layer
 *
 * Simple Legacy SPI flash access layer
 *
 * Author : Bryan O'Donoghue <bryan.odonoghue@linux.intel.com> 2013
 */

#include <linux/dmi.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/pps-gpio.h> 
#include <linux/iio_mpu.h>
#include <linux/platform_data/at24.h>

/******************************************************************************
 *                        Intel Galileo Gen2
 ******************************************************************************/

#define DRIVER_NAME "GalileoGen2"

#define GYRO_ORIENTATION {  0,  1,  0,  1,  0,  0,  0,  0, -1 }
#define ACCEL_ORIENTATION { -1,  0,  0,  0,  1,  0,  0,  0, -1 }
#define COMPASS_ORIENTATION {  0,  0,  1,  0,  1,  0, -1,  0,  0 }
#define PRESSURE_ORIENTATION {  1,  0,  0,  0,  1,  0,  0,  0,  1 }

#define MPU_9250_IRQ            12

#define AT24_SIZE_BYTELEN       5
#define AT24_SIZE_FLAGS         8

#define AT24_BITMASK(x) (BIT(x) - 1)

/* create non-zero magic value for given eeprom parameters */
#define AT24_DEVICE_MAGIC(_len, _flags)         \
    ((1 << AT24_SIZE_FLAGS | (_flags))      \
            << AT24_SIZE_BYTELEN | ilog2(_len))

static struct pps_gpio_platform_data pps_gpio_info = {
    .assert_falling_edge = false,
    .capture_clear = false,
    .gpio_pin = 15,
    .gpio_label = "gpio15",
};


static struct platform_device pps_gpio_device = {
    .name = "pps-gpio",

    .id = -1,
    .dev = {
        .platform_data = &pps_gpio_info
    },
};

const unsigned int iAT24Config = AT24_DEVICE_MAGIC(2048 / 8, 0);

struct mpu_platform_data mpu_data =
{
    .int_config  = 0x10,
    .level_shifter = 0,
    .orientation = ACCEL_ORIENTATION,
    .sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS,
    .sec_slave_id   = COMPASS_ID_AK8963,
    .secondary_i2c_addr = 0x18 >> 1,
    .secondary_orientation = COMPASS_ORIENTATION,
    .power_supply = NULL,
    .secondary_power_supply = NULL,
    .gpio = MPU_9250_IRQ,
};

struct at24_platform_data at24_data = {
    .page_size = 8,
};

static struct i2c_board_info i2c0_board_info[] __initdata = {
{
        .type = "mpu9250",
        .addr = 0xD0 >> 1,
        .platform_data = &mpu_data,
},
{
        .type = "slb9645tt",
        .addr = 0x20,
},
{
        .type = "lm73",
        .addr = 0x48,
},
{
        .type = "ads1015",
        .addr = 0x4b,
},

{
        .type = "24c02",
        .addr = 0x50,
        .platform_data = &at24_data,
},

};

static int __init pcp_i2c_init(void)
{
    at24_data.byte_len = BIT(iAT24Config & AT24_BITMASK(AT24_SIZE_BYTELEN));
    at24_data.flags =  iAT24Config & AT24_BITMASK(AT24_SIZE_FLAGS);

    i2c0_board_info[0].irq = gpio_to_irq(MPU_9250_IRQ);

    if ( i2c_register_board_info(0, i2c0_board_info, ARRAY_SIZE(i2c0_board_info) ) != 0 )
    {
        printk( "Error registering board info\n" );
    }

    return 0;
}

static int intel_quark_platform_galileo_gen2_probe(struct platform_device *pdev)
{
    int err;

    pr_info("%s:\n",__func__);
    err = platform_device_register(&pps_gpio_device);
    if (err) {
        pr_err("%s: Could not register PPS_GPIO device\n",__func__);
    } else {
        pr_info("%s: PPS_GPIO device registered OK\n",__func__);
    }

	return 0;
}

static int intel_quark_platform_galileo_gen2_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver quark_galileo_platform_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_quark_platform_galileo_gen2_probe,
	.remove		= intel_quark_platform_galileo_gen2_remove,
};

module_platform_driver(quark_galileo_platform_driver);

MODULE_AUTHOR("Bryan O'Donoghue <bryan.odonoghue@intel.com>");
MODULE_DESCRIPTION("Intel Quark SPI Data API");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:"DRIVER_NAME);
