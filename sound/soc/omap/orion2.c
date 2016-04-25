#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"

#include <linux/i2c/twl.h>

#define TWL4030_INTBR_PMBR1 0x0D

#define ORION2_SPK_AMP_GPIO 205

static int orion2_hw_params_codec(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "Can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "Can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "Can't set codec system clock\n");
		return ret;
	}

	return 0;
}

/* HACK: use hardcoded values from board init files */
extern int orion2_board_rev_get(void);
static int is_codec_amp_control(void)
{
	int revision;
	int ret;

	ret = 0;

	revision = orion2_board_rev_get();
	switch (revision) {
		case 20160:
			ret = 1;
			break;
		default:
			ret = 0;
			break;
	}

	return ret;
}

static int orion2_codec_amp_start(struct snd_pcm_substream *substream)
{
	printk("AMP Start\n");
	if (is_codec_amp_control())
		gpio_direction_output(ORION2_SPK_AMP_GPIO, 1);
	return 0;
}

static void orion2_codec_amp_shutdown(struct snd_pcm_substream *substream)
{
	printk("AMP Shutdown\n");
	if (is_codec_amp_control())
		gpio_direction_output(ORION2_SPK_AMP_GPIO, 0);
}

static int orion2_hw_params_modem(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_DSP_A |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "Can't set codec DAI configuration for modem\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_DSP_A |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "Can't set cpu DAI configuration for modem\n");
		return ret;
	}

	/* Set the modem system clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 1000000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "Can't set codec system clock for modem\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops orion2_codec_ops = {
	.hw_params = orion2_hw_params_codec,
	.startup = orion2_codec_amp_start,
	.shutdown = orion2_codec_amp_shutdown,
};

static struct snd_soc_ops orion2_modem_ops = {
	.hw_params = orion2_hw_params_modem,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link orion2_dai[] = {
	{
		.name 		= "TPS65951",
		.stream_name 	= "TPS65951",
		.cpu_dai_name = "omap-mcbsp-dai.1",
		.codec_dai_name = "twl4030-hifi",
		.platform_name = "omap-pcm-audio",
		.codec_name = "twl4030-codec",
		.ops 		= &orion2_codec_ops,
	}, {
		.name 		= "Modem",
		.stream_name 	= "GSM Modem",
		.cpu_dai_name = "omap-mcbsp-dai.2",
		.codec_dai_name = "ws6318-hifi",
		.platform_name = "omap-pcm-audio",
		.codec_name = "ws6318-codec",
		.ops 		= &orion2_modem_ops,
	}
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_orion2 = {
	.name = "Orion2",
	.dai_link = orion2_dai,
	.num_links = ARRAY_SIZE(orion2_dai),
};

static struct platform_device *orion2_snd_device;

static int __init orion2_soc_init(void)
{
	int ret;
	u8 pin_mux;

	pr_info("Orion2 SoC Audio init\n");

	printk("GPIO-205 request\n");
	if (gpio_request(ORION2_SPK_AMP_GPIO, "codec-amp-power"))
		printk(KERN_ERR "failed to get codec-amp-power (gpio-205)\n");

	orion2_snd_device = platform_device_alloc("soc-audio", -1);
	if (!orion2_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	/* Set TWL4030 GPIO16 as Digimic CLK */
	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &pin_mux, TWL4030_INTBR_PMBR1);
	pin_mux |= (0x3 << 6);
	twl_i2c_write_u8(TWL4030_MODULE_INTBR, pin_mux, TWL4030_INTBR_PMBR1);

	platform_set_drvdata(orion2_snd_device, &snd_soc_orion2);
	ret = platform_device_add(orion2_snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(orion2_snd_device);

	return ret;
}

static void __exit orion2_soc_exit(void)
{
	platform_device_unregister(orion2_snd_device);
}

module_init(orion2_soc_init);
module_exit(orion2_soc_exit);

MODULE_AUTHOR("Orion");
MODULE_DESCRIPTION("ALSA SoC Orion2");
MODULE_LICENSE("GPL v2");
