#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/mpu.h>

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("A simple Linux driver to bring up some particular i2c-devices");

#define ORION2_MPU9250_IRQGPIO 16

static const unsigned short mpu_i2c_addresses[] = {0x68, I2C_CLIENT_END};


static struct mpu_platform_data gyro_platform_data = {
        .int_config  = 0x00,
        .level_shifter = 0,
        .orientation = {   1,  0,  0,
                           0,  1,  0,
                           0,  0, 1 },
        .sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS,
        .sec_slave_id   = COMPASS_ID_AK8963,
        .secondary_i2c_addr = 0x0C,
        .secondary_orientation = { 0,  1,  0,
                                   -1, 0,  0,
                                   0,  0,  1 },
};

static int probe_mpu9250(void)
{
	struct i2c_adapter *i2c_adap;
	struct i2c_board_info i2c_info;
	struct i2c_client *i2c_client_ptr = NULL;

	i2c_adap = i2c_get_adapter(2);
	memset(&i2c_info, 0, sizeof(struct i2c_board_info));
	strlcpy(i2c_info.type, "mpu9250", I2C_NAME_SIZE);
        i2c_info.irq = ORION2_MPU9250_IRQGPIO;
        i2c_info.platform_data = &gyro_platform_data;
	i2c_client_ptr = i2c_new_probed_device(i2c_adap, &i2c_info, mpu_i2c_addresses, NULL);
	i2c_put_adapter(i2c_adap);

	if (i2c_client_ptr) {
		printk(KERN_INFO "MPU9250 detected\n");
		return 0;
	}
	
	return 1;
}

static const unsigned short compass_i2c_addresses[] = {0x1e, I2C_CLIENT_END};
static const unsigned short accel_i2c_addresses[] = {0x1d, I2C_CLIENT_END};
static const unsigned short baro_i2c_addresses[] = {0x77, I2C_CLIENT_END};

static int probe_old_devices(void)
{
	struct i2c_adapter *i2c_adap;
	struct i2c_board_info i2c_info;
	struct i2c_client *i2c_client_ptr = NULL;

	printk(KERN_INFO "Installing old i2c device set\n");

	i2c_adap = i2c_get_adapter(2);

	/* Compass */
	memset(&i2c_info, 0, sizeof(struct i2c_board_info));
	strlcpy(i2c_info.type, "hmc5843", I2C_NAME_SIZE);
	i2c_client_ptr = i2c_new_probed_device(i2c_adap, &i2c_info, compass_i2c_addresses, NULL);

	/* Accelerometer */
	memset(&i2c_info, 0, sizeof(struct i2c_board_info));
	strlcpy(i2c_info.type, "mma8450q", I2C_NAME_SIZE);
	i2c_client_ptr = i2c_new_probed_device(i2c_adap, &i2c_info, accel_i2c_addresses, NULL);

	/* Baro */
	memset(&i2c_info, 0, sizeof(struct i2c_board_info));
	strlcpy(i2c_info.type, "bmp085", I2C_NAME_SIZE);
	i2c_client_ptr = i2c_new_probed_device(i2c_adap, &i2c_info, baro_i2c_addresses, NULL);

	i2c_put_adapter(i2c_adap);

	return 0;
}

static int orion2_board_rev = 0;
static int __devinit orion2_i2c_init(void)
{
	int ret;

	ret = probe_mpu9250();

	if (ret) {
		probe_old_devices();
	} else {
		orion2_board_rev = 20160;
	}

	return 0;
}

int orion2_board_rev_get(void)
{
	return orion2_board_rev;
}
EXPORT_SYMBOL(orion2_board_rev_get);

static void __devexit orion2_i2c_exit(void)
{
}

module_init(orion2_i2c_init);
module_exit(orion2_i2c_exit);
