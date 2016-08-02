/*
 *  intel_i2cgpio.c - LPC interface for Intel Poulsbo SCH
 *
 *  LPC bridge function of the Intel SCH contains many other
 *  functional units, such as Interrupt controllers, Timers,
 *  Power Management, System Management, GPIO, RTC, and LPC
 *  Configuration Registers.
 *
 *  Copyright (c) 2010 CompuLab Ltd
 *  Copyright (c) 2014 Intel Corp.
 *  Author: Denis Turischev <denis@compulab.co.il>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License 2 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/acpi.h>
#include <linux/pci.h>
#include <linux/mfd/core.h>

#define SMBASE		0x40
#define SMBUS_IO_SIZE	64

#define GPIOBASE	0x44
#define GPIO_IO_SIZE	64
#define GPIO_IO_SIZE_CENTERTON	128

/* Intel Quark X1000 GPIO IRQ Number */
#define GPIO_IRQ_QUARK_X1000	9

#define WDTBASE		0x84
#define WDT_IO_SIZE	64

enum sch_chipsets {
	LPC_QUARK_X1000=0,	/* Intel Quark X1000 */
};

struct intel_i2cgpio_info {
	unsigned int io_size_smbus;
	unsigned int io_size_gpio;
	unsigned int io_size_wdt;
	int irq_gpio;
};

static struct intel_i2cgpio_info sch_chipset_info[] = {
	[LPC_QUARK_X1000] = {
		.io_size_gpio = GPIO_IO_SIZE,
		.irq_gpio = GPIO_IRQ_QUARK_X1000,
		.io_size_wdt = WDT_IO_SIZE,
	},
};

#define PCI_DEVICE_ID_INTEL_QUARK_X1000_I2CGPIO		0x0934

static const struct pci_device_id intel_i2cgpio_ids[] = {
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_QUARK_X1000_I2CGPIO), LPC_QUARK_X1000 },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, intel_i2cgpio_ids);

#define LPC_NO_RESOURCE		1
#define LPC_SKIP_RESOURCE	2

static int intel_i2cgpio_get_io(struct pci_dev *pdev, int where,
			  struct resource *res)
{
	int size;
	unsigned short base_addr;

	size = pci_resource_len(pdev, where);
	base_addr = pci_resource_start(pdev, where);

	res->start = base_addr;
	res->end = base_addr + size - 1;
	res->flags = IORESOURCE_IO;

	return 0;
}

static int intel_i2cgpio_populate_cell(struct pci_dev *pdev, int where,
				 const char *name, int size, int irq,
				 int id, struct mfd_cell *cell)
{
	struct resource *res;
	int ret;

	res = devm_kcalloc(&pdev->dev, 2, sizeof(*res), GFP_KERNEL);
	if (!res)
		return -ENOMEM;

	ret = intel_i2cgpio_get_io(pdev, where, name, res, size);
	if (ret)
		return ret;

	memset(cell, 0, sizeof(*cell));

	cell->name = name;
	cell->resources = res;
	cell->num_resources = 1;
	cell->ignore_resource_conflicts = true;
	cell->id = id;

	/* Check if we need to add an IRQ resource */
	if (irq < 0)
		return 0;

	res++;

	res->start = irq;
	res->end = irq;
	res->flags = IORESOURCE_IRQ;

	cell->num_resources++;

	return 0;
}

static int intel_i2cgpio_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	struct mfd_cell intel_i2cgpio_cells[3];
	struct intel_i2cgpio_info *info = &sch_chipset_info[id->driver_data];
	unsigned int cells = 0;
	int ret;

	ret = intel_i2cgpio_populate_cell(dev, 0, "intel_i2c",
				    info->io_size_gpio, -1,
				    id->device, &intel_i2cgpio_cells[cells]);
	if (ret < 0)
		return ret;
	if (ret == 0)
		cells++;

	ret = intel_i2cgpio_populate_cell(dev, 1, "intel_gpio",
				    info->io_size_smbus, info->irq_gpio,
				    id->device, &intel_i2cgpio_cells[cells]);
	if (ret < 0)
		return ret;
	if (ret == 0)
		cells++;

	if (cells == 0) {
		dev_err(&dev->dev, "All decode registers disabled.\n");
		return -ENODEV;
	}

	return mfd_add_devices(&dev->dev, 0, intel_i2cgpio_cells, cells, NULL, 0, NULL);
}

static void intel_i2cgpio_remove(struct pci_dev *dev)
{
	mfd_remove_devices(&dev->dev);
}

static struct pci_driver intel_i2cgpio_driver = {
	.name		= "intel_i2cgpio",
	.id_table	= intel_i2cgpio_ids,
	.probe		= intel_i2cgpio_probe,
	.remove		= intel_i2cgpio_remove,
};

module_pci_driver(intel_i2cgpio_driver);

MODULE_AUTHOR("Denis Turischev <denis@compulab.co.il>");
MODULE_DESCRIPTION("Interface for the Intel I2C/GPIO controller");
MODULE_LICENSE("GPL");
