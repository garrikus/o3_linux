/*
 * linux/arch/arm/mach-omap2/board-orion2.c
 *
 * 
 *
 * Modified from mach-omap2/board-omap3evm.c
 *
 * 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/leds.h>
#include <linux/interrupt.h>
#include <linux/mtd/nand.h>

#include <linux/i2c/twl.h>

#include <linux/regulator/machine.h>
#include <linux/mmc/host.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/mcspi.h>
#include <plat/display.h>
#include <plat/omap-pm.h>
#include <plat/gpmc.h>

#include "mux.h"
#include "sdram-micron-mt46h64m32lfcm-5.h"
#include "hsmmc.h"
#include "board-flash.h"

static struct regulator_consumer_supply omap3evm_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply omap3evm_vaux1_supply = {
	.supply			= "vrpu",
};

static struct regulator_consumer_supply omap3_evm_vio_supply[] = {
	REGULATOR_SUPPLY("vcc", "spi1.0"),
	REGULATOR_SUPPLY("vio_1v8", NULL),
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data omap3evm_vmmc1 = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		=  REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3evm_vmmc1_supply,
};

/* VAUX1 for NAV-sensor */
static struct regulator_init_data omap3evm_vaux1 = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		=  REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3evm_vaux1_supply,
};

static struct regulator_init_data omap3_evm_vio = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(omap3_evm_vio_supply),
	.consumer_supplies      = omap3_evm_vio_supply,
};

static struct twl4030_clock_init_data omap3orion_clock_data = {
	.ck32k_lowpwr_enable	= false,
	/* .slicer_bypass  	= false,*/
};

static struct twl4030_platform_data omap3evm_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.clock          = &omap3orion_clock_data,
	/*.madc		= &omap3evm_madc_data,*/
	/*.usb		= &omap3evm_usb_data,*/
	/*.gpio		= &omap3evm_gpio_data,*/
	/*.codec		= &omap3evm_codec_data,*/
	/*.vdac		= &omap3_evm_vdac,*/
	/*.vpll2		= &omap3_evm_vpll2,*/
	/*.vaux2          = &omap3evm_vaux2,*/
	.vaux1          = &omap3evm_vaux1,
	.vio		= &omap3_evm_vio,
	/*.vaux3		= &omap3evm_vaux3,*/
};

static struct i2c_board_info __initdata omap3orion_i2c_boardinfo1[] = {
    /* PMIC. TPS65951 onboard, but using TPS65950 so far. */
	{
		I2C_BOARD_INFO("tps65950", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &omap3evm_twldata,
	},
};

static struct i2c_board_info __initdata omap3orion_i2c_boardinfo2[] = {
	{
		/* Compass */
		I2C_BOARD_INFO("hmc5843", 0x1e),
	},
	{
		/* Accelerometer */
		I2C_BOARD_INFO("mma8450q", 0x1d)
	},
	{
		/* Keypad */
		I2C_BOARD_INFO("tca8418", 0x68)
	},

};

static struct i2c_board_info __initdata omap3orion_i2c_boardinfo3[] = {
	{
		/* Charger */
		I2C_BOARD_INFO("ltc4155", 0x12)
	},
	{
		/* Fuel gauge */
		I2C_BOARD_INFO("bq27510", 0xaa)
	},
};

static int __init omap3_evm_i2c_init(void)
{
	/*
	 * REVISIT: These entries can be set in omap3evm_twl_data
	 * after a merge with MFD tree
	 */
	omap3evm_twldata.vmmc1 = &omap3evm_vmmc1;

	omap_register_i2c_bus(1, 2600, omap3orion_i2c_boardinfo1,
			ARRAY_SIZE(omap3orion_i2c_boardinfo1));

	/* Bus 2 on Orion board is used for Compass, G-sensor, FRAM */
	omap_register_i2c_bus(2, 400, omap3orion_i2c_boardinfo2,
			ARRAY_SIZE(omap3orion_i2c_boardinfo2));

	/* Bus 3. Charge control and Fuel Gauge */
	omap_register_i2c_bus(3, 400, omap3orion_i2c_boardinfo3,
		        ARRAY_SIZE(omap3orion_i2c_boardinfo3));
	return 0;
}

static struct omap_board_config_kernel omap3_evm_config[] __initdata = {
};

static void __init omap3_evm_init_irq(void)
{
	omap_board_config = omap3_evm_config;
	omap_board_config_size = ARRAY_SIZE(omap3_evm_config);
	omap2_init_common_infrastructure();

    omap2_init_common_devices(mt46h64m32lfcm5_sdrc_params, NULL);

	omap_init_irq();
	gpmc_init();
}

/*
 * NAND
 */
static struct mtd_partition omap3_evm_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader-NAND",
		.offset		= 0,
		.size		= 4 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 10 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "Boot Env-NAND",

		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x1c0000 */
		.size		= 6 * (64 * 2048),
	},
	{
		.name		= "Kernel-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 40 * (64 * 2048),
	},
	{
		.name		= "File System - NAND",
		.size		= MTDPART_SIZ_FULL,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x780000 */
	},
};

static void __init omap3_evm_init(void)
{

    /* TODO: Select one*/
#if 0
    omap3evm_twldata.vaux2 = &omap3evm_vaux2;
    omap3evm_twldata.vusb = &omap3_evm_vusb;
#endif

	/*
	 * O2 uses DM3730CUSA.
	 *
	 * Normally boolader should have done this. So this is normaly ineffective,
	 * since CONFIG_OMAP_MUX disable during kernel config
	 */
	omap3_mux_init(omap35x_board_mux, OMAP_PACKAGE_CUS);

	omap3_evm_i2c_init();

#if 0
	platform_add_devices(omap3_evm_devices, ARRAY_SIZE(omap3_evm_devices));

	spi_register_board_info(omap3evm_spi_board_info,
				ARRAY_SIZE(omap3evm_spi_board_info));
#endif

	omap_serial_init();

#if 0
	/* OMAP3EVM uses ISP1504 phy and so register nop transceiver */
	usb_nop_xceiv_register(0);

	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2) {
		/* enable EHCI VBUS using GPIO22 */
		omap_mux_init_gpio(22, OMAP_PIN_INPUT_PULLUP);
		gpio_request(OMAP3_EVM_EHCI_VBUS, "enable EHCI VBUS");
		gpio_direction_output(OMAP3_EVM_EHCI_VBUS, 0);
		gpio_set_value(OMAP3_EVM_EHCI_VBUS, 1);

		/* Select EHCI port on main board */
		omap_mux_init_gpio(61, OMAP_PIN_INPUT_PULLUP);
		gpio_request(OMAP3_EVM_EHCI_SELECT, "select EHCI port");
		gpio_direction_output(OMAP3_EVM_EHCI_SELECT, 0);
		gpio_set_value(OMAP3_EVM_EHCI_SELECT, 0);

		/* setup EHCI phy reset config */
		omap_mux_init_gpio(21, OMAP_PIN_INPUT_PULLUP);
		ehci_pdata.reset_gpio_port[1] = 21;

		/* EVM REV >= E can supply 500mA with EXTVBUS programming */
		musb_board_data.power = 500;
		musb_board_data.extvbus = 1;
	} else {
		/* setup EHCI phy reset on MDC */
		omap_mux_init_gpio(135, OMAP_PIN_OUTPUT);
		ehci_pdata.reset_gpio_port[1] = 135;
	}
	usb_musb_init(&musb_board_data);
	usb_ehci_init(&ehci_pdata);
	tsc2008_dev_init();
	omap3evm_init_smsc911x();
#endif

	/* NAND */
	board_nand_init(omap3_evm_nand_partitions,
			ARRAY_SIZE(omap3_evm_nand_partitions),
			0, NAND_BUSWIDTH_16);
}

/* Keep two MACHINE_START to keep U-Boot the same */

/* Much like OMAP3 EVM which it is derived from */
MACHINE_START(ORION2, "ORION2")
	/* Maintainer: Igor S.K.*/
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= omap3_evm_init_irq,
	.init_machine	= omap3_evm_init,
	.timer		= &omap_timer,
MACHINE_END

/* Much like OMAP3 EVM which it is derived from */
MACHINE_START(OMAP3ORION, "OMAP3 ORION")
	/* Maintainer: Igor S.K.*/
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= omap3_evm_init_irq,
	.init_machine	= omap3_evm_init,
	.timer		= &omap_timer,
MACHINE_END
