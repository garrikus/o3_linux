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

#include <linux/input/tca8418_keypad.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc2008.h>
#include <linux/i2c/twl.h>
#include <linux/usb/otg.h>

#include <linux/regulator/machine.h>
#include <linux/mmc/host.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/usb.h>
#include <plat/common.h>
#include <plat/mcspi.h>
#include <plat/display.h>
#include <plat/nokia-dsi-panel.h>
#include <plat/omap-pm.h>
#include <plat/gpmc.h>

#include "mux.h"
#include "sdram-micron-mt46h64m32lfcm-5.h"
#include "hsmmc.h"
#include "board-flash.h"

#define ORION_TS_GPIO          14
#define ORION2_KEYPAD_IRQGPIO  15

#define ORION2_DSIDBI_GPIO     181
#define ORION2_DSI_CMDVID_GPIO 178

static int lcd_enabled;

static void __init omap3_evm_display_init(void)
{
	int r;

	r = gpio_request(ORION2_DSIDBI_GPIO, "lcd_dsidbi");
	if (r) {
		printk(KERN_ERR "failed to get lcd_dsidbi\n");
		return;
	}
	gpio_direction_output(ORION2_DSIDBI_GPIO, 1);

	r = gpio_request(ORION2_DSI_CMDVID_GPIO, "lcd_dsi_cmdvid");
	if (r) {
		printk(KERN_ERR "failed to get lcd_dsi_cmdvid\n");
		goto err_1;
	}
	gpio_direction_output(ORION2_DSI_CMDVID_GPIO, 0);

	return;

err_1:
	gpio_free(ORION2_DSIDBI_GPIO);

}
static int omap3_evm_enable_lcd(struct omap_dss_device *dssdev)
{

	lcd_enabled = 1;
	return 0;
}

static void omap3_evm_disable_lcd(struct omap_dss_device *dssdev)
{

	lcd_enabled = 0;
}

static struct nokia_dsi_panel_data panel_data = {
    .name = "truly_panel",
    .reset_gpio = -1,
};

static struct omap_dss_device omap3_evm_lcd_device = {
	.name			= "truly_lcd",
	.driver_name		= "truly",
	.type			= OMAP_DISPLAY_TYPE_DSI,
	.phy.dsi = {
			.clk_lane   = 2,
			.clk_pol    = 0,
			.data1_lane = 1,
			.data1_pol  = 0,
			.data2_lane = 3,
			.data2_pol  = 0,

            .div = {
                    .regn  = 13,
                    .regm  = 138,
                    .regm3 = 6,
                    .regm4 = 6,

                    .lp_clk_div = 8,

                    .lck_div = 1,
                    .pck_div = 4,
            },
    },

    .panel = {
            .timings.x_res = 480,
            .timings.y_res = 800,
            .timings.pixel_clock = 23000,

            .config = OMAP_DSS_LCD_TFT,
    },

    .ctrl.pixel_size    = 24,
/*	.max_backlight_level	= 100,*/
	.platform_enable	= omap3_evm_enable_lcd,
	.platform_disable	= omap3_evm_disable_lcd,
/*	.set_backlight		= omap3evm_set_bl_intensity,*/
    .data               = &panel_data,
};

static struct omap_dss_device *omap3_evm_dss_devices[] = {
	&omap3_evm_lcd_device,
};

static struct omap_dss_board_info omap3_evm_dss_data = {
	.num_devices	= ARRAY_SIZE(omap3_evm_dss_devices),
	.devices	= omap3_evm_dss_devices,
	.default_device	= &omap3_evm_lcd_device,
};

static struct platform_device omap3_evm_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data = &omap3_evm_dss_data,
	},
};

static struct regulator_consumer_supply omap3evm_vaux3_supply = {
	.supply         = "lcd_2v8",
	.dev	    	= &omap3_evm_dss_device.dev,
};

/* VAUX3 for LCD */
static struct regulator_init_data omap3evm_vaux3 = {
	.constraints = {
		.min_uV                 = 2800000,
		.max_uV                 = 2800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &omap3evm_vaux3_supply,
};

static struct regulator_consumer_supply omap3evm_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply omap3evm_vaux1_supply = {
	.supply			= "vrpu",
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

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{}	/* Terminator */
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


static int omap3evm_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	omap2_hsmmc_init(mmc);

	/* link regulator to MMC adapters */
	omap3evm_vmmc1_supply.dev = mmc[0].dev;

	return 0;
}

static struct twl4030_gpio_platform_data omap3evm_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= false,
	.setup		= omap3evm_twl_gpio_setup,
};

static struct twl4030_usb_data omap3evm_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static uint32_t board_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_DOWN),
	KEY(0, 2, KEY_ENTER),
	KEY(0, 3, KEY_M),

	KEY(1, 0, KEY_RIGHT),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_I),
	KEY(1, 3, KEY_N),

	KEY(2, 0, KEY_A),
	KEY(2, 1, KEY_E),
	KEY(2, 2, KEY_J),
	KEY(2, 3, KEY_O),

	KEY(3, 0, KEY_B),
	KEY(3, 1, KEY_F),
	KEY(3, 2, KEY_K),
	KEY(3, 3, KEY_P)
};

struct matrix_keymap_data tca8418_keymap = {
	.keymap = board_keymap,
	.keymap_size = ARRAY_SIZE(board_keymap),
};

struct tca8418_keypad_platform_data tca8418_pdata = {
    .keymap_data = &tca8418_keymap,
    .rows = 8,
    .cols = 10,
    .rep = 1,
    .irq_is_gpio = 1,
};

static struct regulator_consumer_supply omap3_evm_vio_supply[] = {
	REGULATOR_SUPPLY("vcc", "spi1.0"),
	REGULATOR_SUPPLY("vio_1v8", NULL),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss"),
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
	.usb		= &omap3evm_usb_data,
	.gpio		= &omap3evm_gpio_data,
	/*.codec		= &omap3evm_codec_data,*/
	/*.vdac		= &omap3_evm_vdac,*/
	/*.vpll2		= &omap3_evm_vpll2,*/
	/*.vaux2          = &omap3evm_vaux2,*/
	.vaux1          = &omap3evm_vaux1,
	.vio		= &omap3_evm_vio,
	.vaux3		= &omap3evm_vaux3,
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
		I2C_BOARD_INFO("mma8450q", 0x1d),
	},
    {
        /* Keypad */
        I2C_BOARD_INFO("tca8418_keypad", 0x34),
        .irq = ORION2_KEYPAD_IRQGPIO,
        .platform_data = &tca8418_pdata,
    },
};

static struct i2c_board_info __initdata omap3orion_i2c_boardinfo3[] = {
	{
		/* Charger */
		I2C_BOARD_INFO("ltc4155", 0x12),
	},
	{
		/* Fuel gauge */
		I2C_BOARD_INFO("bq27510", 0xaa),
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

static void tsc2008_dev_init(void)
{
	if (gpio_request(ORION_TS_GPIO, "TSC2008 pendown") < 0)
		printk(KERN_ERR "can't get tsc2008 pen down GPIO\n");

	gpio_direction_input(ORION_TS_GPIO);
	gpio_set_debounce(ORION_TS_GPIO, 310);
}

static int tsc2008_get_pendown_state(void)
{
	return !gpio_get_value(ORION_TS_GPIO);
}

static struct tsc2008_platform_data tsc2008_config = {
	.x_plate_ohms        = 450,
	.bit_mode            = 12,
	.get_pendown_state   = tsc2008_get_pendown_state,
	.clear_penirq        = NULL,
	.init_platform_hw    = NULL,
	.exit_platform_hw    = NULL,
	.irq_flags           = IRQF_TRIGGER_FALLING,
};

static struct omap2_mcspi_device_config tsc2008_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info omap3evm_spi_board_info[] = {
#if 0
	[0] = {
		.modalias		= "nt39122b",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.mode                   = SPI_3WIRE,
		.controller_data	= &nt39122b_mcspi_config,
	},
#endif

    /* XXX: Take care of index!!! */
	/* No touchscreen so far... */
	[0] = {
		.modalias		= "tsc2008",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 25000000,
		.controller_data	= &tsc2008_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(ORION_TS_GPIO),
		.platform_data		= &tsc2008_config,
	},
};

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

static struct platform_device *omap3_evm_devices[] __initdata = {
	&omap3_evm_dss_device,
};
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

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 100,
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

	/*omap3_mux_init(omap35x_board_mux, OMAP_PACKAGE_CUS);*/

	omap3_evm_i2c_init();

	platform_add_devices(omap3_evm_devices, ARRAY_SIZE(omap3_evm_devices));

	spi_register_board_info(omap3evm_spi_board_info,
				ARRAY_SIZE(omap3evm_spi_board_info));

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
#endif
	
    usb_musb_init(&musb_board_data);
	tsc2008_dev_init();
#if 0
	omap3evm_init_smsc911x();
#endif
	omap3_evm_display_init();

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
