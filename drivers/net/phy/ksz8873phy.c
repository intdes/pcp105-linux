/*
 * drivers/net/phy/ksz8873phy.c
 *
 * Copyright (c) 2010 SAGEMCOM
 *
 * Author: Karl Beldan <karl.beldan@sagemcom.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * PHY Driver for Micrel KSZ8873 serial switch
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/ethtool.h>
#include <linux/etherdevice.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/if_vlan.h>
#include <linux/crc32.h>
#include <linux/net_tstamp.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <net/tcp.h>
#include "ksz8873phy.h"


#define DRV_MDIONAME		"KSZ8873 MII bus"

int stmmac_mdio_read(struct mii_bus *bus, int phyaddr, int phyreg);
int stmmac_mdio_write(struct mii_bus *bus, int phyaddr, int phyreg,
                 u16 phydata);

void macb_handle_link_change(struct net_device *dev);
void stmmac_adjust_link(struct net_device *dev);

/* point to function for MDIO bus, SPI bus, or I2C bus */
static struct ksz8873_fns_t *ksz8873_fns;

/* point to function for platform dependent MDIO */
static struct ksz8873_fns_t *ksz8873_mii_fns;

/* point to function for i/o access on SPI, or I2C bus */
struct ksz8873_io_fns_t *ksz8873_io_fns;


static const char *bus_str[] = {
    /*  0 */  "unknown_bus",
    /*  1 */  "mdio_bus",
    /*  2 */  "spi_bus",
    /*  3 */  "i2c_bus",
    ""
};
static int bus_type;

int  ks8872_total_phy=KSZ8873_MIM_ETH;



static int ksz8873_config_init(struct phy_device *phydev)
{
    int val;
    u32 features;


    features = SUPPORTED_MII;

    /* Do we support autonegotiation? */
    val = phy_read(phydev, MII_BMSR);

    if (val < 0) {
        return val;
    }

    if (val & BMSR_ANEGCAPABLE)
        features |= SUPPORTED_Autoneg;

    if (val & BMSR_100FULL)
        features |= SUPPORTED_100baseT_Full;
    if (val & BMSR_100HALF)
        features |= SUPPORTED_100baseT_Half;
    if (val & BMSR_10FULL)
        features |= SUPPORTED_10baseT_Full;
    if (val & BMSR_10HALF)
        features |= SUPPORTED_10baseT_Half;

    if (val & BMSR_ESTATEN) {
        val = phy_read(phydev, MII_ESTATUS);

        if (val < 0)
            return val;

        if (val & ESTATUS_1000_TFULL)
            features |= SUPPORTED_1000baseT_Full;
        if (val & ESTATUS_1000_THALF)
            features |= SUPPORTED_1000baseT_Half;
    }

    phydev->supported = features;
    phydev->advertising = features;

    return 0;
}

#if defined (CONFIG_KSPHY_BUS_SPI) || defined (CONFIG_KSPHY_BUS_I2C)
static int ksz8873_ack_interrupt(struct phy_device *phydev)
{
    u8 spi_val;
    int ret;


    ret = ksz8873_io_fns->nread(LinkChangeInterrupt, &spi_val, 1);
    if (ret < 0)
        return ret;

#if defined (CONFIG_KSPHY_MULTI_ETH)
    /* The Switch is represented as multiple PHYs
     */
    spi_val &= (0x04 | (1 << (phydev->addr - 1)));
#endif
    return ksz8873_io_fns->nwrite(LinkChangeInterrupt, &spi_val, 1);
}

static int ksz8873_config_intr(struct phy_device *phydev)
{
    u8 spi_val;
    int phy_addr, i;
    int ret;


    ret = ksz8873_io_fns->nread(InterruptEnable, &spi_val, 1);
    if (ret < 0)
        return ret;

    if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
        for (i=0; i<ks8872_total_phy; i++) {
            phy_addr = phydev->addr + i;
            spi_val |= 1 << (phy_addr - 1);
        }
    } else {
        for (i=0; i<ks8872_total_phy; i++) {
            phy_addr = phydev->addr + i;
            spi_val &= ~(1 << (phy_addr - 1));
        }
    }

    return ksz8873_io_fns->nwrite(InterruptEnable, &spi_val, 1);
}

static int ksz8873_did_interrupt(struct phy_device *phydev)
{
    u8 spi_val;
    int ret;


    ret = ksz8873_io_fns->nread(LinkChangeInterrupt, &spi_val, 1);
    if (ret < 0)
        return ret;

#if !defined (CONFIG_KSPHY_MULTI_ETH)

    /* The Switch is represented as single PHY
     */
    if ( (spi_val & 0x80) != 0 )
        return (1);
    else
        return (0);
#else
    /* The Switch is represented as multiple PHYs
     */
    return spi_val & (1 << (phydev->addr - 1));
#endif
}

#endif /* #if defined (CONFIG_KSPHY_BUS_SPI) || defined (CONFIG_KSPHY_BUS_I2C) */


static struct phy_driver ksz8873_phy_driver = {
    .phy_id         = (KSZ8873_PHYID1 << 16) | KSZ8873_PHYID2,
    .phy_id_mask    = ~0x6,
    .name           = "Micrel KSZ8873",
    .config_init    = ksz8873_config_init,
    .features       = PHY_BASIC_FEATURES | SUPPORTED_Pause,
    .flags          = PHY_HAS_INTERRUPT,
    .config_aneg    = genphy_config_aneg,
    .read_status    = genphy_read_status,
#if defined (CONFIG_KSPHY_BUS_SPI) || defined (CONFIG_KSPHY_BUS_I2C)
    .ack_interrupt  = ksz8873_ack_interrupt,
    .config_intr    = ksz8873_config_intr,
    .did_interrupt  = ksz8873_did_interrupt,
#endif
    .driver         = {.owner= THIS_MODULE, },
};

static int __init ksz8873_phy_init(void)
{
    return phy_driver_register(&ksz8873_phy_driver);
}

static void __exit ksz8873_phy_exit(void)
{
    phy_driver_unregister(&ksz8873_phy_driver);
}


/*--------------------------------------------------------------------------*/

#ifdef PLATFORM_MDIO_FUNCTIONS
static int platform_mdio_read(struct mii_bus *bus, int phy_id, int regnum)
{
    printk(KERN_INFO "%s: bus=0x%x, phy_id=0x%x, regnum=0x%x\n", __func__,
           (int)bus, phy_id, regnum);
    return (0);
}

static int platform_mdio_write(struct mii_bus *bus, int phy_id, int regnum, u16 val)
{
    printk(KERN_INFO "%s: bus=0x%x, phy_id=0x%x, regnum=0x%x, val=0x%x\n", __func__,
           (int)bus, phy_id, regnum, val);
	macb_mdio_write(bus, phy_id, regnum, val);
    return (0);
}
#endif

int ksz8873_mii_read(struct mii_bus *bus, int phy_id, int regnum)
{
    int ret;

    /* KSZ8873 doesn't support broadcast PHY address, force it to PHY address 1 */
    if (phy_id == 0)
        phy_id = KSZ8873_PHY1_ADDR;

#if !defined (CONFIG_KSPHY_MULTI_ETH)

    /* The Switch is represented as single PHY
     */

    if ((regnum == MII_PHYSID1) || (regnum == MII_PHYSID2))
        return ksz8873_mii_fns->read(bus, phy_id, regnum);

    /* read phy 1 link status first */
    ret = ksz8873_mii_fns->read(bus, phy_id, MII_BMSR);

    /* Is it going to read phy link status? */
    if (regnum == MII_BMSR) {
        /* Yes, return link down if both Switch phy are link down,
           otherwise, linkup */
        if ((ret & BMSR_LSTATUS) == BMSR_LSTATUS)
            return (ret);
        else
            return ksz8873_mii_fns->read(bus, (phy_id+1), regnum);
    } else {
        /* No, return phy 1 register if phy 1 is linkup,
           otherwise, return phy 2 register status */
        if ((ret & BMSR_LSTATUS) == BMSR_LSTATUS)
            return ksz8873_mii_fns->read(bus, phy_id, regnum);
        else
            return ksz8873_mii_fns->read(bus, (phy_id+1), regnum);
    }
#else
    /* The Switch is represented as multiple PHYs
     */
    return ksz8873_mii_fns->read(bus, phy_id, regnum);
#endif

}

int ksz8873_mii_write(struct mii_bus *bus, int phy_id, int regnum, u16 val)
{
    int phy_addr, i;
    int ret=0;


    /* KSZ8873 doesn't support broadcast PHY address, force it to PHY address 1 */
    if (phy_id == 0)
        phy_id = KSZ8873_PHY1_ADDR;

    for (i=0; i<ks8872_total_phy; i++) {
        phy_addr = phy_id + i;
        ret = ksz8873_mii_fns->write(bus, phy_addr, regnum, val);
    }

    return ret;
}

void platform_mac_adjust_link(struct net_device *dev)
{
#if defined (CONFIG_NET_PEGASUS)
    ksz9692_mac_adjust_link(dev);
#elif defined (CONFIG_STMMAC_ETH)
	stmmac_adjust_link( dev );
#elif defined (CONFIG_MACB)
	macb_handle_link_change( dev );
#endif
}

struct phy_device * ksz8873_mii_connect(struct net_device *dev, u8 phy_addr, phy_interface_t phy_mode ) {
    struct phy_device *phydev = NULL;
    char phy_id[MII_BUS_ID_SIZE];



    snprintf(phy_id, MII_BUS_ID_SIZE, PHY_ID_FMT, "mdio", phy_addr);

    phydev = phy_connect(dev, phy_id,
                         platform_mac_adjust_link, phy_mode);

    if (phydev != NULL) {

        printk(KERN_INFO "%s: attached PHY driver [%s] (%s:phy_addr=%s, irq=%d)\n",
               dev->name, phydev->drv->name, bus_str[bus_type],
               dev_name(&phydev->dev), phydev->irq);

        /* mask with MAC supported features */
        phydev->supported &= (PHY_BASIC_FEATURES | SUPPORTED_Pause );

        phydev->supported |= (SUPPORTED_Pause);

        phydev->advertising = phydev->supported;
    } else {
        printk(KERN_INFO"%s: Could not connect to PHY, make a dummy phy device\n",
               dev->name);

        /* Make dummy phy device */
        phydev = kzalloc(sizeof(struct phy_device), GFP_KERNEL);
        phydev->link = 1;
        phydev->speed = 100;
        phydev->duplex = 1;
    }

    return (phydev);
}


static int ksz8873_mii_probe(void *priv, struct net_device *dev, struct mii_bus **pmii_bus)
{
    struct mii_bus *mii_bus;
    int *irqlist;
    int err = -ENXIO, i;

    if ((ksz8873_fns->read == NULL) || (ksz8873_fns->write == NULL)) {
        err = -ENOMEM;
        printk(KERN_ERR "%s: ksz8873_fns not available\n", __func__);
        goto err_out_1;
    }

#if defined (CONFIG_KSPHY_BUS_MDIO)
    if ((ksz8873_mii_fns->read == NULL) || (ksz8873_mii_fns->write == NULL)) {
        err = -ENOMEM;
        printk(KERN_ERR "%s: ksz8873_mii_fns not available\n", __func__);
        goto err_out_1;
    }
#endif

#if !defined (CONFIG_KSPHY_BUS_MDIO)
    if ((ksz8873_io_fns->nread == NULL) || (ksz8873_io_fns->nwrite == NULL) ||
        (ksz8873_io_fns->init == NULL) || (ksz8873_io_fns->exit == NULL)) {
        err = -ENOMEM;
        printk(KERN_ERR "%s: ksz8873_io_fns not available\n", __func__);
        goto err_out_1;
    }
#endif

    mii_bus = mdiobus_alloc();
    if (mii_bus==NULL) {
        err = -ENOMEM;
        printk(KERN_ERR "%s: mdiobus_alloc fail\n", __func__);
        goto err_out_1;
    }

    irqlist = kmalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
    if (!irqlist) {
        printk(KERN_ERR "%s: Error kmalloc irq\n", __func__);
        goto err_out_free_bus_2;
    }

    for (i = 0; i < PHY_MAX_ADDR; ++i)
        irqlist[i] = PHY_IGNORE_INTERRUPT;

#if !defined (CONFIG_KSPHY_BUS_MDIO)
    irqlist[KSZ8873_PHY1_ADDR] = (int)KSZ8873_IRQ;
#endif

    mii_bus->name = DRV_MDIONAME;
    snprintf(mii_bus->id, MII_BUS_ID_SIZE, "mdio");
    mii_bus->parent  = &(dev->dev);
    mii_bus->read = ksz8873_fns->read;
    mii_bus->write = ksz8873_fns->write;
    mii_bus->reset = NULL;
    mii_bus->irq = irqlist;
    mii_bus->priv = priv;
    mii_bus->phy_mask =  ~((KSZ8873_PHY1_ADDR | KSZ8873_PHY2_ADDR) << 1) ;

    if (mdiobus_register(mii_bus)) {
        printk(KERN_ERR "%s: Error registering mii bus\n", __func__);
        goto err_out_free_bus_2;
    }

	*pmii_bus = mii_bus;
//    platform_set_drvdata(pdev, mii_bus);

    return 0;


err_out_free_bus_2:
    mdiobus_free(mii_bus);
err_out_1:
    return err;
}

int ksz8873_mii_init(void *priv, struct net_device *dev, struct mii_bus **mii_bus)
{
    int ret = -1;


    /* point to mii bus access function dependent on MDIO, SPI, or I2C bus */
    ksz8873_fns = kzalloc(sizeof(*ksz8873_fns), GFP_KERNEL);
    if (!ksz8873_fns) {
        printk(KERN_ERR "%s:ksz8873_fns memory allocation failed\n", __func__);
        return -1;
    }

#if defined (CONFIG_KSPHY_BUS_MDIO)

    /* MIIM by MDIO/MDC bus */
    bus_type = KSZ8873_MIDO_BUS;
    ksz8873_fns->read  = ksz8873_mii_read;
    ksz8873_fns->write = ksz8873_mii_write;

    /* point to MDIO I2C access function dependent on MDIO master */
    ksz8873_mii_fns = kzalloc(sizeof(*ksz8873_mii_fns), GFP_KERNEL);
    if (!ksz8873_mii_fns) {
        printk(KERN_ERR "%s:ksz8873_mii_fns memory allocation failed\n", __func__);
        return -1;
    }
#if defined (CONFIG_NET_PEGASUS)
    ksz8873_mii_fns->read  = ksz9692_mdio_read;
    ksz8873_mii_fns->write = ksz9692_mdio_write;
#elif defined (CONFIG_STMMAC_ETH)
    ksz8873_mii_fns->read  = stmmac_mdio_read;
    ksz8873_mii_fns->write = stmmac_mdio_write;
#elif defined (CONFIG_MACB)
    ksz8873_mii_fns->read  = macb_mdio_read;
    ksz8873_mii_fns->write = macb_mdio_write;
#else
    ksz8873_mii_fns->read  = platform_mdio_read;
    ksz8873_mii_fns->write = platform_mdio_write;
#endif /* #if defined (CONFIG_NET_PEGASUS) */

#elif defined (CONFIG_KSPHY_BUS_SPI) || defined (CONFIG_KSPHY_BUS_I2C)

    ksz8873_fns->read  = ksz8873_spi_i2c_read;
    ksz8873_fns->write = ksz8873_spi_i2c_write;

    /* point to SPI or I2C access function */
    ksz8873_io_fns = kzalloc(sizeof(*ksz8873_io_fns), GFP_KERNEL);
    if (!ksz8873_io_fns) {
        printk(KERN_ERR "%s:ksz8873_io_fns memory allocation failed\n", __func__);
        return -1;
    }

#if defined (CONFIG_KSPHY_BUS_SPI)

    /* MIIM by SPI bus */
    bus_type = KSZ8873_SPI_BUS;
    ksz8873_io_fns->nread  = ksz8873_spi_nread;
    ksz8873_io_fns->nwrite = ksz8873_spi_nwrite;
    ksz8873_io_fns->init   = ksz8873_spibus_mod_init;
    ksz8873_io_fns->exit   = ksz8873_spibus_mod_exit;

#else /* #if defined (CONFIG_KSPHY_BUS_SPI) */

    /* MIIM by I2C bus */
    bus_type = KSZ8873_I2C_BUS;
    ksz8873_io_fns->nread  = ksz8873_i2c_nread;
    ksz8873_io_fns->nwrite = ksz8873_i2c_nwrite;
    ksz8873_io_fns->init   = ksz8873_i2cbus_mod_init;
    ksz8873_io_fns->exit   = ksz8873_i2cbus_mod_exit;

#endif /* #if defined (CONFIG_KSPHY_BUS_SPI) */

#else /* #if defined (CONFIG_KSPHY_BUS_MDIO) */

    /* unknow interface */
    bus_type = 0;
    ksz8873_fns->read  = NULL;
    ksz8873_fns->write = NULL;
    printk(KERN_ERR "%s: no mii bus define!\n", __func__);
    return -1;

#endif /* #if defined (CONFIG_KSPHY_BUS_MDIO)*/

#if !defined (CONFIG_KSPHY_MULTI_ETH)
    /* The KSZ8873 Switch as a single PHY */
    ks8872_total_phy = KSZ8873_MAX_ETH;
#endif

    printk(KERN_INFO "KSZ8873 PHY driver version %s on %s\n",
           KSZ8873_PHY_VER, bus_str[bus_type]);

    if ((bus_type == KSZ8873_SPI_BUS) || (bus_type == KSZ8873_I2C_BUS)) {
        u8  chip_id;

        /* init SPI/I2C bus mod */
        ret = ksz8873_io_fns->init();
        if (ret < 0)
            return ret;

        /* checking SPI/I2C interface */
        ret = ksz8873_io_fns->nread(ChipID0, &chip_id, 1);
        if (ret < 0) {
            printk(KERN_ERR "%s: %s access failed\n", __func__, bus_str[bus_type]);
            return ret;
        }
        if (chip_id != KSZ8873_FAMILY_ID) {
            printk(KERN_ERR "%s: warning Chip ID0 is not 0x%02x (0x%02x)\n",
                   __func__, KSZ8873_FAMILY_ID, chip_id);
            return -1;
        }
    }

    return ( ksz8873_mii_probe( priv, dev, mii_bus ) );
}

void ksz8873_mii_exit( void )
{

    if (ksz8873_io_fns->exit)
        ksz8873_io_fns->exit();

    if (ksz8873_fns)
        kfree(ksz8873_fns);

    if (ksz8873_mii_fns)
        kfree(ksz8873_fns);

    if (ksz8873_io_fns)
        kfree(ksz8873_io_fns);
}


/*--------------------------------------------------------------------------*/

module_init(ksz8873_phy_init);
module_exit(ksz8873_phy_exit);

MODULE_AUTHOR("karl.beldan@sagemcom.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Micrel KSZ8873 PHY driver");
