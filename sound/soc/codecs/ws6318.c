#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>

static int ws6318_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{
	return 0;
}

static int ws6318_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	if (freq != 1000000) {
		dev_err(codec->dev, "Unsupported APLL mclk: %u\n", freq);
		return -EINVAL;
	}

	return 0;
}

static int ws6318_set_dai_fmt(struct snd_soc_dai *codec_dai,
			unsigned int fmt)
{
	return 0;
}

static struct snd_soc_dai_ops ws6318_dai_hifi_ops = {
	.hw_params	= ws6318_hw_params,
	.set_sysclk	= ws6318_set_dai_sysclk,
	.set_fmt	= ws6318_set_dai_fmt,
};

static struct snd_soc_dai_driver ws6318_dai = {
	.name = "ws6318-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE, },
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE, },
	.ops = &ws6318_dai_hifi_ops,
};

static struct snd_soc_codec_driver soc_codec_dev_ws6318;

static int ws6318_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_ws6318, &ws6318_dai, 1);
}

static int __devexit ws6318_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver ws6318_codec_driver = {
	.probe = ws6318_probe,
	.remove = __devexit_p(ws6318_remove),
	.driver = {
			.name = "ws6318-codec",
			.owner = THIS_MODULE,
	},
};

static int __init ws6318_init(void)
{
	return platform_driver_register(&ws6318_codec_driver);
}
module_init(ws6318_init);

static void __exit ws6318_exit(void)
{
	platform_driver_unregister(&ws6318_codec_driver);
}
module_exit(ws6318_exit);

MODULE_DESCRIPTION("ASoC WS6318 driver");
MODULE_AUTHOR("Orion");
MODULE_LICENSE("GPL");
