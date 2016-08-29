/*
 * drivers/net/phy/ksz8873phy.h
 *
 * PHY Driver header file for Micrel KSZ8873 serial switch
 *
 * Copyright (c) 2010 SAGEM Communications.
 *
 * Author: Karl Beldan <karl.beldan@sagemcom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef KSZ8873PHY_H
#define KSZ8873PHY_H

#include <linux/mii.h>
#if defined(CONFIG_ARCH_PEGASUS)
#include <mach/platform.h>
#endif

#define KSZ8873_PHY_VER   "1.0.1"

#define KSZ8873_PHY1_ADDR               1
#define KSZ8873_PHY2_ADDR               2

#define KSZ8873_SPI_CMD_READ            0x03
#define KSZ8873_SPI_CMD_WRITE           0x02

#define KSZ8873_FAMILY_ID               0x88
#define KSZ8873_CHIP_ID                 0x03
#define KSZ8873_PHYID1                  0x0022
#define KSZ8873_PHYID2                  0x1430
#define KSZ8873_PORTS_REGS_OFFSET       16

#define KSZ8873_PHY1_DFLT_ADDR          1
#define KSZ8873_PHY2_DFLT_ADDR          2

#define KSZ8873_MDIO_MIN                MII_BMCR
#define KSZ8873_MDIO_MAX                MII_LPA

#define KSZ8873_MIDO_BUS                1
#define KSZ8873_SPI_BUS                 2
#define KSZ8873_I2C_BUS                 3


/* max switch port support logical Network interface */
#define KSZ8873_MAX_ETH                 2
#define KSZ8873_MIM_ETH                 1

#if defined (CONFIG_NET_PEGASUS)
#define KSZ8873_IRQ                     (LOW_IRQS + KS8692_INT_EXT_INT0)
#endif

struct ksz8873_fns_t {
    int (*read)(struct mii_bus *, int, int);
    int (*write)(struct mii_bus *, int, int, u16);
};

struct ksz8873_io_fns_t {
    int (*nread)(u8, u8 *, int);
    int (*nwrite)(u8, u8 *, int);
    int (*init)(void);
    void (*exit)(void);
};
extern struct ksz8873_io_fns_t *ksz8873_io_fns;


/* enum from sources.blackfin.uclinux.org/net/dsa/ksz8893.h with ksz8863 bits */
enum switch_reg {
    /* Global Registers: 0-15 */
    ChipID0 = 0,
    ChipID1,
    GlobalControl0,
    GlobalControl1,
    GlobalControl2, /* 4 */
    GlobalControl3,
    GlobalControl4,
    GlobalControl5,
    GlobalControl6, /* 8 */
    GlobalControl7,
    GlobalControl8,
    GlobalControl9,
    GlobalControl10, /* 12 */
    GlobalControl11,
    GlobalControl12,
    GlobalControl13,

    /* Port Registers: 16-95 */
    Port1Control0 = 16,
    Port1Control1,
    Port1Control2,
    Port1Control3,
    Port1Control4, /* 20 */
    Port1Control5,
    Port1Control6,
    Port1Control7,
    Port1Control8, /* 24 */
    Port1Control9,
    Port1Control10,
    Port1Control11,
    Port1Control12, /* 28 */
    Port1Control13,
    Port1Status0,
    Port1Status1,

    Port2Control0, /* 32 */
    Port2Control1,
    Port2Control2,
    Port2Control3,
    Port2Control4, /* 36 */
    Port2Control5,
    Port2Control6,
    Port2Control7,
    Port2Control8, /* 40 */
    Port2Control9,
    Port2Control10,
    Port2Control11,
    Port2Control12, /* 44 */
    Port2Control13,
    Port2Status0,
    Port2Status1,

    Port3Control0, /* 48 */
    Port3Control1,
    Port3Control2,
    Port3Control3,
    Port3Control4, /* 52 */
    Port3Control5,
    Port3Control6,
    Port3Control7,
    Port3Control8, /* 56 */
    Port3Control9,
    Reservednotappliedtoport3, /* 58-62 */
    Port3Status1 = 63,

    /* ksz8863 specific */
    Reset = 67,

    TOSPriorityControlRegister0 = 96,
    TOSPriorityControlRegister1,
    TOSPriorityControlRegister2,
    TOSPriorityControlRegister3,
    TOSPriorityControlRegister4, /* 100 */
    TOSPriorityControlRegister5,
    TOSPriorityControlRegister6,
    TOSPriorityControlRegister7,
    TOSPriorityControlRegister8, /* 104 */
    TOSPriorityControlRegister9,
    TOSPriorityControlRegister10,
    TOSPriorityControlRegister11,
    TOSPriorityControlRegister12, /* 108 */
    TOSPriorityControlRegister13,
    TOSPriorityControlRegister14,
    TOSPriorityControlRegister15,

    IndirectAccessControl0 = 121,
    IndirectAccessControl1,
    IndirectDataRegister8,
    IndirectDataRegister7, /* 124 */
    IndirectDataRegister6,
    IndirectDataRegister5,
    IndirectDataRegister4,
    IndirectDataRegister3, /* 128 */
    IndirectDataRegister2,
    IndirectDataRegister1,
    IndirectDataRegister0,

    Port1TxqSplitForQ0 = 175,
    Port1TxqSplitForQ1,
    Port1TxqSplitForQ2,
    Port1TxqSplitForQ3,
    Port2TxqSplitForQ0,
    Port2TxqSplitForQ1, /* 180 */
    Port2TxqSplitForQ2,
    Port2TxqSplitForQ3,
    Port3TxqSplitForQ0,
    Port3TxqSplitForQ1, /* 184 */
    Port3TxqSplitForQ2,
    Port3TxqSplitForQ3,

    InterruptEnable = 187,
    LinkChangeInterrupt,

    InsertSrcPvid = 194,
    PowerManagementLedMode,

    ForwardInvalidVIDFrameandHostMode = 198,
};

/* Export functions */
#if defined (CONFIG_NET_PEGASUS)
extern int ksz9692_mdio_read(struct mii_bus *, int, int);
extern int ksz9692_mdio_write(struct mii_bus *, int, int, u16);
extern void ksz9692_mac_adjust_link(struct net_device *);
#endif

#if defined (CONFIG_KSPHY_BUS_SPI) || defined (CONFIG_KSPHY_BUS_I2C)
extern inline int ksz8873_spi_i2c_read(struct mii_bus *, int, int);
extern inline int ksz8873_spi_i2c_write(struct mii_bus *, int, int, u16);
#endif

#if defined (CONFIG_KSPHY_BUS_SPI)

extern int  ksz8873_spi_nread(u8, u8 *, int);
extern int  ksz8873_spi_nwrite(u8, u8 *, int);

extern int  ksz8873_spibus_mod_init(void);
extern void ksz8873_spibus_mod_exit(void);

extern struct spi_device *spi_ksz8873;

#elif defined (CONFIG_KSPHY_BUS_I2C)

extern int ksz8873_i2c_nread(u8, u8 *, int);
extern int ksz8873_i2c_nwrite(u8, u8 *, int);

extern int  ksz8873_i2cbus_mod_init(void);
extern void ksz8873_i2cbus_mod_exit(void);

extern struct i2c_adapter *i2c_ksz8873_adap;

#endif

#endif /* KSZ8873PHY_H */
