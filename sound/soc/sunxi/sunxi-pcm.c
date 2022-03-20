/*
 * sound\soc\sunxi\sunxi-pcm.c
 * (C) Copyright 2014-2018
 * AllWinner Technology Co., Ltd. <www.allwinnertech.com>
 * wolfgang huang <huangjinhui@allwinnertech.com>
 * yumingfeng <yumingfeng@allwinnertech.com>
 *
 * some simple description for this code
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/simple_card.h>
#include <sound/dmaengine_pcm.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <asm/dma.h>
#include "sunxi-pcm.h"

static int raw_flag = 1;
static dma_addr_t hdmiraw_dma_addr;
static dma_addr_t hdmipcm_dma_addr;
static unsigned char *hdmiraw_dma_area;	/* DMA area */
static unsigned int channel_status[192];

static u64 sunxi_pcm_mask = DMA_BIT_MASK(32);

struct headbpcuv {
	unsigned char other:3;
	unsigned char V:1;
	unsigned char U:1;
	unsigned char C:1;
	unsigned char P:1;
	unsigned char B:1;
};

union head61937 {
	struct headbpcuv head0;
	unsigned char head1;
} head;

union word {
	struct {
		unsigned int bit0:1;
		unsigned int bit1:1;
		unsigned int bit2:1;
		unsigned int bit3:1;
		unsigned int bit4:1;
		unsigned int bit5:1;
		unsigned int bit6:1;
		unsigned int bit7:1;
		unsigned int bit8:1;
		unsigned int bit9:1;
		unsigned int bit10:1;
		unsigned int bit11:1;
		unsigned int bit12:1;
		unsigned int bit13:1;
		unsigned int bit14:1;
		unsigned int bit15:1;
		unsigned int rsvd:16;
	} bits;
	unsigned int wval;
} wordformat;

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
	.buffer_bytes_max	= SUNXI_AUDIO_CMA_MAX_BYTES_MAX,
	.period_bytes_min	= 256,
	.period_bytes_max	= SUNXI_AUDIO_CMA_MAX_BYTES_MAX / 2,
	.periods_min		= 1,
	.periods_max		= 8,
	.fifo_size		= 128,
};

int sunxi_get_rawflag(void)
{
	return raw_flag;
}

int hdmi_transfer_format_61937_to_60958(int *out, short *temp,
					int samples, int rate)
{
	int ret = 0;
	int i;
	static int numtotal;
	union word w1;

	samples >>= 1;
	head.head0.other = 0;
	head.head0.B = 1;
	head.head0.P = 0;
	head.head0.C = 0;
	head.head0.U = 0;
	head.head0.V = 1;

	for (i = 0; i < 192; i++)
		channel_status[i] = 0;

	channel_status[1] = 1;
	/* sample rates */
	if (rate == 32000) {
		channel_status[24] = 1;
		channel_status[25] = 1;
		channel_status[26] = 0;
		channel_status[27] = 0;
	} else if (rate == 44100) {
		channel_status[24] = 0;
		channel_status[25] = 0;
		channel_status[26] = 0;
		channel_status[27] = 0;
	} else if (rate == 48000) {
		channel_status[24] = 0;
		channel_status[25] = 1;
		channel_status[26] = 0;
		channel_status[27] = 0;
	} else if (rate == (32000*4)) {
		channel_status[24] = 1;
		channel_status[25] = 0;
		channel_status[26] = 0;
		channel_status[27] = 0;
	} else if (rate == (44100*4)) {
		channel_status[24] = 0;
		channel_status[25] = 0;
		channel_status[26] = 1;
		channel_status[27] = 1;
	} else if (rate == (48000*4)) {
		channel_status[24] = 0;
		channel_status[25] = 1;
		channel_status[26] = 1;
		channel_status[27] = 1;
		if (raw_flag == 12 || raw_flag == 11) {
			channel_status[24] = 1;
			channel_status[25] = 0;
			channel_status[26] = 0;
			channel_status[27] = 1;
		}
	} else {
		channel_status[24] = 0;
		channel_status[25] = 1;
		channel_status[26] = 0;
		channel_status[27] = 0;
	}

	for (i = 0; i < samples; i++, numtotal++) {
		if ((numtotal % 384 == 0) || (numtotal % 384 == 1))
			head.head0.B = 1;
		else
			head.head0.B = 0;

		head.head0.C = channel_status[(numtotal % 384)/2];

		if (numtotal % 384 == 0)
			numtotal = 0;

		w1.wval = (*temp) & (0xffff);

		head.head0.P = w1.bits.bit15 ^ w1.bits.bit14 ^ w1.bits.bit13 ^
			       w1.bits.bit12 ^ w1.bits.bit11 ^ w1.bits.bit10 ^
			       w1.bits.bit9 ^ w1.bits.bit8 ^ w1.bits.bit7 ^
			       w1.bits.bit6 ^ w1.bits.bit5 ^ w1.bits.bit4 ^
			       w1.bits.bit3 ^ w1.bits.bit2 ^ w1.bits.bit1 ^
			       w1.bits.bit0;

		ret = (int)(head.head1) << 24;
		/* 11 may can be replace by 8 or 12 */
		ret |= (int)((w1.wval)&(0xffff)) << 11;
		*out = ret;
		out++;
		temp++;
	}

	return 0;
}

static int sunxi_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct sunxi_dma_params *dmap;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->dev;
	struct dma_chan *chan = snd_dmaengine_pcm_get_chan(substream);
	struct dma_slave_config slave_config;
	int ret;

	dmap = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	ret = snd_hwparams_to_dma_slave_config(substream, params,
			&slave_config);
	if (ret) {
		dev_err(dev, "hw params config failed with err %d\n", ret);
		return ret;
	}

	slave_config.dst_maxburst = dmap->dst_maxburst;
	slave_config.src_maxburst = dmap->src_maxburst;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		slave_config.dst_addr = dmap->dma_addr;
		slave_config.src_addr_width = slave_config.dst_addr_width;
	} else {
		slave_config.src_addr =	dmap->dma_addr;
		slave_config.dst_addr_width = slave_config.src_addr_width;
	}

	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret < 0) {
		dev_err(dev, "dma slave config failed with err %d\n", ret);
		return ret;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	return 0;
}


static int sunxi_pcm_hdmi_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct sunxi_dma_params *dmap;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->dev;
	struct dma_chan *chan = snd_dmaengine_pcm_get_chan(substream);
	struct dma_slave_config slave_config;
	struct asoc_simple_priv *sndhdmi_priv = snd_soc_card_get_drvdata(rtd->card);
	int ret;

	//TODO:adapt hdmi mode
	raw_flag = sndhdmi_priv->hdmi_format;
	pr_info("raw_flag value is %u\n", raw_flag);
	dmap = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	ret = snd_hwparams_to_dma_slave_config(substream, params,
			&slave_config);
	if (ret) {
		dev_err(dev, "hw params config failed with err %d\n", ret);
		return ret;
	}

	slave_config.dst_maxburst = dmap->dst_maxburst;
	slave_config.src_maxburst = dmap->src_maxburst;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		slave_config.dst_addr =	dmap->dma_addr;
		slave_config.src_addr_width = slave_config.dst_addr_width;
	} else {
		slave_config.src_addr =	dmap->dma_addr;
		slave_config.dst_addr_width = slave_config.src_addr_width;
	}

	/*raw_flag>1. rawdata*/
	if (raw_flag > 1) {
		slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

		if (!dev->dma_mask)
			dev->dma_mask = &sunxi_pcm_mask;
		if (!dev->coherent_dma_mask)
			dev->coherent_dma_mask = 0xffffffff;

		/* note: change the follow function
		 * sound/core/pcm_dmaengine.c -> dmaengine_pcm_prepare_and_submit()
		 * snd_pcm_lib_buffer_bytes(substream) * 2 when rawflag > 1
		 */
		hdmiraw_dma_area = dma_alloc_coherent(dev,
				(2 * params_buffer_bytes(params)),
				&hdmiraw_dma_addr, GFP_KERNEL);
		if (hdmiraw_dma_area == NULL) {
			pr_err("hdmi:raw:get mem failed...\n");
			return -ENOMEM;
		}
		hdmipcm_dma_addr = substream->dma_buffer.addr;
		substream->dma_buffer.addr = (dma_addr_t)hdmiraw_dma_addr;
	}

	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret < 0) {
		dev_err(dev, "dma slave config failed with err %d\n", ret);
		return ret;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	return 0;
}

static int sunxi_pcm_hdmi_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->dev;

	if (snd_pcm_lib_buffer_bytes(substream) && (raw_flag > 1)) {
		dma_free_coherent(dev,
				(2 * snd_pcm_lib_buffer_bytes(substream)),
				hdmiraw_dma_area, hdmiraw_dma_addr);
		substream->dma_buffer.addr = hdmipcm_dma_addr;
		hdmiraw_dma_area = NULL;
	}
	snd_pcm_set_runtime_buffer(substream, NULL);

	return 0;
}

static int sunxi_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);

	return 0;
}

static int sunxi_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
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
		}
	}
	return 0;
}

static const char * const dmaengine_pcm_dma_channel_names[] = {
	[SNDRV_PCM_STREAM_PLAYBACK] = "tx",
	[SNDRV_PCM_STREAM_CAPTURE] = "rx",
};

static int sunxi_pcm_open(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, SUNXI_DMAENGINE_PCM_DRV_NAME);
	struct device *dev = component->dev;
	struct sunxi_dma_params *dma_params = NULL;
	struct dma_chan *chan;

	dma_params = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	/* Set HW params now that initialization is complete */
	sunxi_pcm_hardware.buffer_bytes_max = dma_params->cma_kbytes *
		SUNXI_AUDIO_CMA_BLOCK_BYTES;
	sunxi_pcm_hardware.period_bytes_max = sunxi_pcm_hardware.buffer_bytes_max / 2;
	sunxi_pcm_hardware.fifo_size = dma_params->fifo_size;
	snd_soc_set_runtime_hwparams(substream, &sunxi_pcm_hardware);
	ret = snd_pcm_hw_constraint_integer(substream->runtime,
			SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	chan = dma_request_chan(dev, dmaengine_pcm_dma_channel_names[substream->stream]);
	if (IS_ERR(chan)) {
		dev_err(dev, "DMA channels request %s failed.\n",
				dmaengine_pcm_dma_channel_names[substream->stream]);
		return -EINVAL;
	}

	ret = snd_dmaengine_pcm_open(substream, chan);
	if (ret < 0) {
		dev_err(dev, "dmaengine pcm open failed with err %d\n", ret);
		return ret;
	}

	return 0;
}

static int sunxi_pcm_mmap(struct snd_pcm_substream *substream,
		struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = NULL;

	if (substream->runtime != NULL) {
		runtime = substream->runtime;

		return dma_mmap_wc(substream->pcm->card->dev, vma,
				runtime->dma_area,
				runtime->dma_addr,
				runtime->dma_bytes);
	} else {
		return -1;
	}

}

static int sunxi_pcm_copy(struct snd_pcm_substream *substream, int channel,
			  unsigned long hwoff, void __user *buf,
			  unsigned long bytes)
{
	int ret = 0;
	char *hwbuf;
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		hwbuf = runtime->dma_area + hwoff;
		if (copy_from_user(hwbuf, buf, bytes))
			return -EFAULT;
		if (raw_flag > 1) {
			char *hdmihw_area = hdmiraw_dma_area + 2 * hwoff;
			hdmi_transfer_format_61937_to_60958((int *)hdmihw_area,
							    (short *)hwbuf,
							    bytes,
							    runtime->rate);
		}
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		hwbuf = runtime->dma_area + hwoff;
		if (copy_to_user(buf, hwbuf, bytes))
			return -EFAULT;
	}

	return ret;
}

/* For passthrough mode: using no_residue */
snd_pcm_uframes_t sunxi_dmaengine_pcm_pointer(struct snd_pcm_substream *substream)
{
	if (raw_flag > 1)
		return snd_dmaengine_pcm_pointer_no_residue(substream);
	else
		return snd_dmaengine_pcm_pointer(substream);
}

static struct snd_pcm_ops sunxi_pcm_ops = {
	.open		= sunxi_pcm_open,
	.close		= snd_dmaengine_pcm_close_release_chan,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= sunxi_pcm_hw_params,
	.hw_free	= sunxi_pcm_hw_free,
	.trigger	= sunxi_pcm_trigger,
	.pointer	= snd_dmaengine_pcm_pointer,
	.mmap		= sunxi_pcm_mmap,
};

static struct snd_pcm_ops sunxi_pcm_ops_no_residue = {
	.open		= sunxi_pcm_open,
	.close		= snd_dmaengine_pcm_close_release_chan,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= sunxi_pcm_hdmi_hw_params,
	.hw_free	= sunxi_pcm_hdmi_hw_free,
	.trigger	= sunxi_pcm_trigger,
	.pointer	= sunxi_dmaengine_pcm_pointer,
	.mmap		= sunxi_pcm_mmap,
	.copy_user	= sunxi_pcm_copy,
};

static int sunxi_pcm_preallocate_stream_dma_buffer(struct snd_pcm *pcm,
		int stream, size_t buffer_bytes_max)
{
	struct snd_pcm_str *streams = NULL;
	struct snd_pcm_substream *substream = NULL;
	struct snd_dma_buffer *buf = NULL;

	streams = &pcm->streams[stream];
	if (IS_ERR_OR_NULL(streams)) {
		pr_err("[%s] stream=%d streams is null!\n", __func__, stream);
		return -EFAULT;
	}
	substream = pcm->streams[stream].substream;
	if (IS_ERR_OR_NULL(substream)) {
		pr_err("[%s] stream=%d substream is null!\n", __func__, stream);
		return -EFAULT;
	}
	buf = &substream->dma_buffer;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	if (buffer_bytes_max > SUNXI_AUDIO_CMA_MAX_BYTES_MAX)
		buffer_bytes_max = SUNXI_AUDIO_CMA_MAX_BYTES_MAX;
	if (buffer_bytes_max < SUNXI_AUDIO_CMA_MAX_BYTES_MIN)
		buffer_bytes_max = SUNXI_AUDIO_CMA_MAX_BYTES_MIN;

	buf->area = dma_alloc_coherent(pcm->card->dev, buffer_bytes_max,
			&buf->addr, GFP_KERNEL);
	if (!buf->area) {
		dev_err(pcm->card->dev, "dmaengine alloc coherent failed.\n");
		return -ENOMEM;
	}
	buf->bytes = buffer_bytes_max;

	return 0;
}

static void sunxi_pcm_free_stream_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf;

	if (IS_ERR_OR_NULL(substream)) {
		pr_err("[%s] stream=%d substream is null!\n", __func__, stream);
		return;
	}

	buf = &substream->dma_buffer;
	if (!buf->area) {
		pr_err("[%s] stream=%d buf->area is null!\n", __func__, stream);
		return;
	}

	dma_free_coherent(pcm->card->dev, buf->bytes,
			buf->area, buf->addr);
	buf->area = NULL;
}

static void sunxi_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_coherent(pcm->card->dev, buf->bytes,
				buf->area, buf->addr);
		buf->area = NULL;
	}
}

static int sunxi_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	struct snd_pcm *pcm = rtd->pcm;
	struct device *dev = rtd->dev;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct sunxi_dma_params *playback_dma_data = cpu_dai->playback_dma_data;
	struct sunxi_dma_params *capture_dma_data = cpu_dai->capture_dma_data;
	size_t capture_cma_bytes = SUNXI_AUDIO_CMA_BLOCK_BYTES;
	size_t playback_cma_bytes = SUNXI_AUDIO_CMA_BLOCK_BYTES;
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &sunxi_pcm_mask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;

	if (!IS_ERR_OR_NULL(playback_dma_data))
		playback_cma_bytes *= playback_dma_data->cma_kbytes;
	if (!IS_ERR_OR_NULL(capture_dma_data))
		capture_cma_bytes *= capture_dma_data->cma_kbytes;

	if (dai_link->playback_only) {
		ret = sunxi_pcm_preallocate_stream_dma_buffer(pcm,
				SNDRV_PCM_STREAM_PLAYBACK, playback_cma_bytes);
		if (ret) {
			dev_err(dev, "pcm new playback failed with err=%d\n", ret);
			return ret;
		}
	} else if (dai_link->capture_only) {
		ret = sunxi_pcm_preallocate_stream_dma_buffer(pcm,
				SNDRV_PCM_STREAM_CAPTURE, capture_cma_bytes);
		if (ret) {
			dev_err(dev, "pcm new capture failed with err=%d\n", ret);
			return ret;
		}
	} else {
		ret = sunxi_pcm_preallocate_stream_dma_buffer(pcm,
				SNDRV_PCM_STREAM_PLAYBACK, playback_cma_bytes);
		if (ret) {
			dev_err(dev, "pcm new playback failed with err=%d\n", ret);
			goto err_pcm_prealloc_playback_buffer;
		}
		ret = sunxi_pcm_preallocate_stream_dma_buffer(pcm,
				SNDRV_PCM_STREAM_CAPTURE, capture_cma_bytes);
		if (ret) {
			dev_err(dev, "pcm new capture failed with err=%d\n", ret);
			goto err_pcm_prealloc_capture_buffer;
		}
	}

	return 0;

err_pcm_prealloc_capture_buffer:
	sunxi_pcm_free_stream_dma_buffer(pcm, SNDRV_PCM_STREAM_PLAYBACK);
err_pcm_prealloc_playback_buffer:
	return ret;
}

static struct snd_soc_component_driver sunxi_soc_platform = {
	.name		= SUNXI_DMAENGINE_PCM_DRV_NAME,
	.ops		= &sunxi_pcm_ops,
	.pcm_new	= sunxi_pcm_new,
	.pcm_free	= sunxi_pcm_free_dma_buffers,
};

static const struct snd_soc_component_driver sunxi_soc_platform_no_residue = {
	.name		= SUNXI_DMAENGINE_PCM_DRV_NAME,
	.ops		= &sunxi_pcm_ops_no_residue,
	.pcm_new	= sunxi_pcm_new,
	.pcm_free	= sunxi_pcm_free_dma_buffers,
};

int asoc_dma_platform_register(struct device *dev, unsigned int flags)
{
	/*
	 * FIXME, Don't try to request the DMA channels through devicetree.
	 * in sunxi famaily, using HDMI, will decodec rawdata, should be using
	 * self defined copy function, so using this flag just mark diff with
	 * normal audio platform copy function, no relation with devicetree
	 */
	if (flags & SND_DMAENGINE_PCM_FLAG_NO_DT)
		return snd_soc_register_component(dev,
				&sunxi_soc_platform_no_residue, NULL, 0);
	else
		return snd_soc_register_component(dev,
				&sunxi_soc_platform, NULL, 0);
}
EXPORT_SYMBOL_GPL(asoc_dma_platform_register);

void asoc_dma_platform_unregister(struct device *dev)
{
	snd_soc_unregister_component(dev);
}
EXPORT_SYMBOL_GPL(asoc_dma_platform_unregister);

MODULE_AUTHOR("huangxin, liushaohua");
MODULE_DESCRIPTION("sunxi ASoC DMA Driver");
MODULE_LICENSE("GPL");

