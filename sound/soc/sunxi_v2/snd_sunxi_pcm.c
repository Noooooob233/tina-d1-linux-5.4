/*
 * sound\soc\sunxi\snd_sunxi_pcm.c
 * (C) Copyright 2021-2025
 * AllWinner Technology Co., Ltd. <www.allwinnertech.com>
 * Dby <dby@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#include "snd_sunxi_log.h"
#include "snd_sunxi_pcm.h"

#define HLOG				"PCM"
#define SUNXI_DMAENGINE_PCM_DRV_NAME	"sunxi_dmaengine_pcm"

static u64 g_sunxi_pcm_mask = DMA_BIT_MASK(32);

static struct snd_pcm_hardware sunxi_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED
				| SNDRV_PCM_INFO_BLOCK_TRANSFER
				| SNDRV_PCM_INFO_MMAP
				| SNDRV_PCM_INFO_MMAP_VALID
				| SNDRV_PCM_INFO_PAUSE
				| SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S8
				| SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S20_3LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE,
	.rates			= SNDRV_PCM_RATE_8000_192000
				| SNDRV_PCM_RATE_KNOT,
	.rate_min		= 8000,
	.rate_max		= 192000,
	.channels_min		= 1,
	.channels_max		= 8,
	/* value must be (2^n)Kbyte */
	.buffer_bytes_max	= SUNXI_AUDIO_CMA_MAX_BYTES,
	.period_bytes_min	= 256,
	.period_bytes_max	= SUNXI_AUDIO_CMA_MAX_BYTES / 2,
	.periods_min		= 1,
	.periods_max		= 8,
	.fifo_size		= 128,
};

static const char * const dmaengine_pcm_dma_channel_names[] = {
	[SNDRV_PCM_STREAM_PLAYBACK] = "tx",
	[SNDRV_PCM_STREAM_CAPTURE] = "rx",
};

static int sunxi_pcm_open(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, SUNXI_DMAENGINE_PCM_DRV_NAME);
	struct device *dev = component->dev;
	struct sunxi_dma_params *dma_params = NULL;
	struct dma_chan *chan;

	SND_LOG_DEBUG(HLOG, "\n");

	/* Set HW params now that initialization is complete */
	dma_params = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	sunxi_pcm_hardware.buffer_bytes_max = dma_params->cma_kbytes * SUNXI_AUDIO_CMA_BLOCK_BYTES;
	sunxi_pcm_hardware.period_bytes_max = sunxi_pcm_hardware.buffer_bytes_max / 2;
	sunxi_pcm_hardware.fifo_size	    = dma_params->fifo_size;
	snd_soc_set_runtime_hwparams(substream, &sunxi_pcm_hardware);
	ret = snd_pcm_hw_constraint_integer(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		SND_LOG_ERR(HLOG, "constraint_integer failed, err %d\n", ret);
		return ret;
	}

	chan = dma_request_chan(dev, dmaengine_pcm_dma_channel_names[substream->stream]);
	if (IS_ERR(chan)) {
		SND_LOG_ERR(HLOG, "DMA channels request %s failed, err -> %d.\n",
			    dmaengine_pcm_dma_channel_names[substream->stream],
			    IS_ERR(chan));
		return -EINVAL;
	}

	ret = snd_dmaengine_pcm_open(substream, chan);
	if (ret < 0) {
		SND_LOG_ERR(HLOG, "dmaengine pcm open failed with err %d\n", ret);
		return ret;
	}

	return 0;
}

static int sunxi_pcm_close(struct snd_pcm_substream *substream)
{
	SND_LOG_DEBUG(HLOG, "\n");

	return snd_dmaengine_pcm_close_release_chan(substream);
}


static int sunxi_pcm_ioctl(struct snd_pcm_substream *substream,
			   unsigned int cmd, void *arg)
{
	SND_LOG_DEBUG(HLOG, "cmd -> %u\n", cmd);

	return snd_pcm_lib_ioctl(substream, cmd, arg);
}

static int sunxi_pcm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params)
{
	struct sunxi_dma_params *dma_params;
	struct dma_slave_config slave_config;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct dma_chan *chan;
	int ret;

	SND_LOG_DEBUG(HLOG, "\n");

	dma_params = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	ret = snd_hwparams_to_dma_slave_config(substream, params, &slave_config);
	if (ret) {
		SND_LOG_ERR(HLOG, "hw params config failed, err %d\n", ret);
		return ret;
	}

	slave_config.dst_maxburst = dma_params->dst_maxburst;
	slave_config.src_maxburst = dma_params->src_maxburst;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		slave_config.dst_addr = dma_params->dma_addr;
		slave_config.src_addr_width = slave_config.dst_addr_width;
	} else {
		slave_config.src_addr =	dma_params->dma_addr;
		slave_config.dst_addr_width = slave_config.src_addr_width;
	}

	chan = snd_dmaengine_pcm_get_chan(substream);
	if (chan == NULL) {
		SND_LOG_ERR(HLOG, "dma pcm get chan failed! chan is NULL\n");
		return -EINVAL;
	}
	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret < 0) {
		SND_LOG_ERR(HLOG, "dma slave config failed, err %d\n", ret);
		return ret;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	return 0;
}

static int sunxi_pcm_hw_free(struct snd_pcm_substream *substream)
{
	SND_LOG_DEBUG(HLOG, "\n");

	snd_pcm_set_runtime_buffer(substream, NULL);

	return 0;
}

static int sunxi_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	SND_LOG_DEBUG(HLOG, "cmd -> %d\n", cmd);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			snd_dmaengine_pcm_trigger(substream,
						  SNDRV_PCM_TRIGGER_START);
		break;
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			snd_dmaengine_pcm_trigger(substream,
						  SNDRV_PCM_TRIGGER_STOP);
		break;
		default:
			SND_LOG_ERR(HLOG, "unsupport trigger -> %d\n", cmd);
			return -1;
		break;
		}
	} else {
		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			snd_dmaengine_pcm_trigger(substream,
						  SNDRV_PCM_TRIGGER_START);
		break;
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			snd_dmaengine_pcm_trigger(substream,
						  SNDRV_PCM_TRIGGER_STOP);
		break;
		default:
			SND_LOG_ERR(HLOG, "unsupport trigger -> %d\n", cmd);
			return -1;
		break;
		}
	}

	return 0;
}

snd_pcm_uframes_t sunxi_pcm_pointer(struct snd_pcm_substream *substream)
{
	return snd_dmaengine_pcm_pointer(substream);
}

static int sunxi_pcm_mmap(struct snd_pcm_substream *substream,
			  struct vm_area_struct *vma)
{
	SND_LOG_DEBUG(HLOG, "\n");

	if (substream->runtime == NULL) {
		SND_LOG_ERR(HLOG, "substream->runtime is null\n");
		return -EFAULT;
	}

	return dma_mmap_wc(substream->pcm->card->dev, vma,
			   substream->runtime->dma_area,
			   substream->runtime->dma_addr,
			   substream->runtime->dma_bytes);
}

static struct snd_pcm_ops sunxi_pcm_ops = {
	.open		= sunxi_pcm_open,
	.close		= sunxi_pcm_close,
	.ioctl		= sunxi_pcm_ioctl,
	.hw_params	= sunxi_pcm_hw_params,
	.hw_free	= sunxi_pcm_hw_free,
	.trigger	= sunxi_pcm_trigger,
	.pointer	= sunxi_pcm_pointer,
	.mmap		= sunxi_pcm_mmap,
};

static int sunxi_pcm_preallocate_dma_buffer(struct snd_pcm *pcm,
					    int stream,
					    size_t buffer_bytes_max)
{
	struct snd_dma_buffer *buf = NULL;
	struct snd_pcm_str *streams = NULL;
	struct snd_pcm_substream *substream = NULL;

	SND_LOG_DEBUG(HLOG, "\n");

	streams = &pcm->streams[stream];
	if (IS_ERR_OR_NULL(streams)) {
		SND_LOG_ERR(HLOG, "stream=%d streams is null!\n", stream);
		return -EFAULT;
	}
	substream = pcm->streams[stream].substream;
	if (IS_ERR_OR_NULL(substream)) {
		SND_LOG_ERR(HLOG, "stream=%d substreams is null!\n", stream);
		return -EFAULT;
	}

	buf = &substream->dma_buffer;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	if (buffer_bytes_max > SUNXI_AUDIO_CMA_MAX_BYTES) {
		buffer_bytes_max = SUNXI_AUDIO_CMA_MAX_BYTES;
		SND_LOG_WARN(HLOG, "buffer_bytes_max too max, set %lu\n",
			     buffer_bytes_max);
	}
	if (buffer_bytes_max < SUNXI_AUDIO_CMA_MIN_BYTES) {
		buffer_bytes_max = SUNXI_AUDIO_CMA_MIN_BYTES;
		SND_LOG_WARN(HLOG, "buffer_bytes_max too min, set %lu\n",
			     buffer_bytes_max);
	}

	buf->area = dma_alloc_coherent(pcm->card->dev, buffer_bytes_max,
				       &buf->addr, GFP_KERNEL);
	if (!buf->area) {
		SND_LOG_ERR(HLOG, "dmaengine alloc coherent failed.\n");
		return -ENOMEM;
	}
	buf->bytes = buffer_bytes_max;

	return 0;
}

static void sunxi_pcm_free_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_dma_buffer *buf;
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;

	SND_LOG_DEBUG(HLOG, "\n");

	if (IS_ERR_OR_NULL(substream)) {
		SND_LOG_WARN(HLOG, "stream=%d streams is null!\n", stream);
		return;
	}

	buf = &substream->dma_buffer;
	if (!buf->area) {
		SND_LOG_WARN(HLOG, "stream=%d buf->area is null!\n", stream);
		return;
	}

	dma_free_coherent(pcm->card->dev, buf->bytes, buf->area, buf->addr);
	buf->area = NULL;
}

static int sunxi_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	int ret;

	struct snd_pcm *pcm = rtd->pcm;
	struct snd_card *card = rtd->card->snd_card;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;

	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct sunxi_dma_params *capture_dma_data  = cpu_dai->capture_dma_data;
	struct sunxi_dma_params *playback_dma_data = cpu_dai->playback_dma_data;
	size_t capture_cma_bytes  = SUNXI_AUDIO_CMA_BLOCK_BYTES;
	size_t playback_cma_bytes = SUNXI_AUDIO_CMA_BLOCK_BYTES;

	SND_LOG_DEBUG(HLOG, "\n");

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &g_sunxi_pcm_mask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;

	if (!IS_ERR_OR_NULL(capture_dma_data))
		capture_cma_bytes *= capture_dma_data->cma_kbytes;
	if (!IS_ERR_OR_NULL(playback_dma_data))
		playback_cma_bytes *= playback_dma_data->cma_kbytes;

	if (dai_link->capture_only) {
		ret = sunxi_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_CAPTURE, capture_cma_bytes);
		if (ret) {
			SND_LOG_ERR(HLOG, "pcm new capture failed, err=%d\n", ret);
			return ret;
		}
	} else if (dai_link->playback_only) {
		ret = sunxi_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_PLAYBACK, playback_cma_bytes);
		if (ret) {
			SND_LOG_ERR(HLOG, "pcm new playback failed, err=%d\n", ret);
			return ret;
		}
	} else {
		ret = sunxi_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_CAPTURE, capture_cma_bytes);
		if (ret) {
			SND_LOG_ERR(HLOG, "pcm new capture failed, err=%d\n", ret);
			goto err_pcm_prealloc_capture_buffer;
		}
		ret = sunxi_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_PLAYBACK, playback_cma_bytes);
		if (ret) {
			SND_LOG_ERR(HLOG, "pcm new playback failed, err=%d\n", ret);
			goto err_pcm_prealloc_playback_buffer;
		}
	}

	return 0;

err_pcm_prealloc_playback_buffer:
	sunxi_pcm_free_dma_buffer(pcm, SNDRV_PCM_STREAM_CAPTURE);
err_pcm_prealloc_capture_buffer:
	return ret;
}

static void sunxi_pcm_free(struct snd_pcm *pcm)
{
	int stream;

	SND_LOG_DEBUG(HLOG, "\n");

	for (stream = 0; stream < SNDRV_PCM_STREAM_LAST; stream++) {
		sunxi_pcm_free_dma_buffer(pcm, stream);
	}
}

static struct snd_soc_component_driver sunxi_soc_platform = {
	.name		= SUNXI_DMAENGINE_PCM_DRV_NAME,
	.ops		= &sunxi_pcm_ops,
	.pcm_new	= sunxi_pcm_new,
	.pcm_free	= sunxi_pcm_free,
};

int snd_soc_sunxi_dma_platform_register(struct device *dev)
{
	SND_LOG_DEBUG(HLOG, "\n");
	return snd_soc_register_component(dev, &sunxi_soc_platform, NULL, 0);
}
EXPORT_SYMBOL_GPL(snd_soc_sunxi_dma_platform_register);

void snd_soc_sunxi_dma_platform_unregister(struct device *dev)
{
	SND_LOG_DEBUG(HLOG, "\n");
	snd_soc_unregister_component(dev);
}
EXPORT_SYMBOL_GPL(snd_soc_sunxi_dma_platform_unregister);

MODULE_AUTHOR("Dby@allwinnertech.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sunxi ASoC DMA driver");
