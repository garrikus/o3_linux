#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/regulator/consumer.h>

struct pwm_dev {
	dev_t devt;
	struct cdev cdev;
	struct device *device;
	struct semaphore sem;
	int id;
	u32 current_val;

	struct regulator * reg;
};

static struct pwm_dev pwm_dev;

/* only one class */
struct class *nvs_class;

static ssize_t pwm_read(struct file *filp, char __user *buff, size_t count,
		loff_t *offp)
{
	size_t len;
	ssize_t status;
	struct pwm_dev *pd = filp->private_data;
	char temp[16];
	int enabled;

	if (!buff)
		return -EFAULT;

	/* for user progs like cat that will keep asking forever */
	if (*offp > 0)
		return 0;

	if (down_interruptible(&pd->sem))
		return -ERESTARTSYS;

	enabled = regulator_is_enabled(pd->reg);
	if (enabled)
		enabled = 1;
	len = sprintf(temp, "%u\n", enabled);

	if (len + 1 < count)
		count = len + 1;

	if (copy_to_user(buff, temp, count))  {
		status = -EFAULT;
	}
	else {
		*offp += count;
		status = count;
	}

	up(&pd->sem);

	return status;
}

static ssize_t pwm_write(struct file *filp, const char __user *buff,
		size_t count, loff_t *offp)
{
	size_t len;
	u32 val;
	ssize_t status = 0;
	char temp[16];
	int enabled;

	struct pwm_dev *pd = filp->private_data;

	if (down_interruptible(&pd->sem))
		return -ERESTARTSYS;

	if (!buff || count < 1) {
		status = -EINVAL;
		goto pwm_write_done;
	}

	if (count > 8)
		len = 8;
	else
		len = count;

	memset(temp, 0, 16);

	if (copy_from_user(temp, buff, len)) {
		status = -EFAULT;
		goto pwm_write_done;
	}

	val = simple_strtoul(temp, NULL, 0);

	enabled = regulator_is_enabled(pd->reg);

	if (val && !enabled)
		status = regulator_enable(pd->reg);
	else if (!val && enabled)
		status = regulator_disable(pd->reg);

	*offp += count;

	if (!status)
		status = count;

pwm_write_done:

	up(&pd->sem);

	return status;
}

static int pwm_open(struct inode *inode, struct file *filp)
{
	struct pwm_dev *pd = &pwm_dev;
	filp->private_data = pd;

	return 0;
}

static struct file_operations pwm_fops = {
	.owner = THIS_MODULE,
	.read = pwm_read,
	.write = pwm_write,
	.open = pwm_open,
};

static int __init pwm_init_cdev(struct pwm_dev *pd)
{
	int error;

	error = alloc_chrdev_region(&pd->devt, pd->id, 1, "nvs");

	if (error) {
		pd->devt = 0;
		return error;
	}

	cdev_init(&pd->cdev, &pwm_fops);
	pd->cdev.owner = THIS_MODULE;

	error = cdev_add(&pd->cdev, pd->devt, 1);

	if (error) {
		unregister_chrdev_region(pd->devt, 1);
		pd->devt = 0;
		return error;
	}

	return 0;
}

static int __init pwm_init_class(struct pwm_dev *pd)
{
	int ret;

	if (!nvs_class) {
		nvs_class = class_create(THIS_MODULE, "nvs");

		if (IS_ERR(nvs_class)) {
			ret = PTR_ERR(nvs_class);
			nvs_class = 0;
			return ret;
		}
	}

	pd->device = device_create(nvs_class, NULL, pd->devt, NULL, "nvs%d",
			MINOR(pd->devt));

	if (IS_ERR(pd->device)) {
		ret = PTR_ERR(pd->device);
		pd->device = 0;
		return ret;
	}

	return 0;
}

static int __init pwm_get_regulator(struct pwm_dev *pd)
{
	int ret = 0;

	pd->reg  = regulator_get(pd->device, "vrpu");

	if (IS_ERR(pd->reg)) {
		printk(KERN_ERR "%s: cant get vrpu\n", __func__);
		ret = PTR_ERR(pd->reg);
		goto pgr_out;
	}

pgr_out:
	return ret;
}

static void pwm_dev_cleanup(void)
{
	regulator_put(pwm_dev.reg);

	if (pwm_dev.device)
		device_destroy(nvs_class, pwm_dev.devt);

	if (nvs_class)
		class_destroy(nvs_class);

	cdev_del(&pwm_dev.cdev);
	unregister_chrdev_region(pwm_dev.devt, 1);
}

static int __init nvs_init(void)
{
	printk(KERN_INFO "Starting nvs power\n");

	sema_init(&pwm_dev.sem, 1);

	if (pwm_init_cdev(&pwm_dev))
		goto init_fail;

	if (pwm_init_class(&pwm_dev))
		goto init_fail;

	if (pwm_get_regulator(&pwm_dev))
		goto init_fail;

	return 0;

init_fail:
	pwm_dev_cleanup();

	return -1;
}
module_init(nvs_init);

static void __exit nvs_exit(void)
{
	pwm_dev_cleanup();
}
module_exit(nvs_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Igor S.K.");
MODULE_DESCRIPTION("User space power supply control for nav receiver.");
