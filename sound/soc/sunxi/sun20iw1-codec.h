/*
 * sound\soc\sunxi\sun20iw1-codec.h
 * (C) Copyright 2021-2026
 * allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * yumingfeng <yumingfeng@allwinnertech.com>
 * luguofang <luguofang@allwinnertech.com>
 * Dby <Dby@allwinnertech.com>
 *
 * some simple description for this code
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */
#ifndef _SUN20IW1_CODEC_H
#define _SUN20IW1_CODEC_H

#include "sunxi-pcm.h"

#define SUNXI_DAC_DPC		0x00
#define SUNXI_DAC_VOL_CTRL	0x04
#define SUNXI_DAC_FIFOC		0x10
#define SUNXI_DAC_FIFOS		0x14

#define SUNXI_DAC_TXDATA	0X20
#define SUNXI_DAC_CNT		0x24
#define SUNXI_DAC_DG		0x28

#define	SUNXI_ADC_FIFOC		0x30
#define	SUNXI_ADC_VOL_CTRL	0x34
#define SUNXI_ADC_FIFOS		0x38
#define SUNXI_ADC_RXDATA	0x40
#define SUNXI_ADC_CNT		0x44
#define SUNXI_ADC_DG		0x4C
#define SUNXI_ADC_DIG_CTRL	0x50
#define SUNXI_VRA1SPEEDUP_DOWN_CTRL	0x54

#define SUNXI_DAC_DAP_CTL	0xF0
#define SUNXI_ADC_DAP_CTL	0xF8

#define SUNXI_DAC_DRC_HHPFC	0x100
#define SUNXI_DAC_DRC_LHPFC	0x104
#define SUNXI_DAC_DRC_CTRL	0x108
#define SUNXI_DAC_DRC_LPFHAT	0x10C
#define SUNXI_DAC_DRC_LPFLAT	0x110
#define SUNXI_DAC_DRC_RPFHAT	0x114
#define SUNXI_DAC_DRC_RPFLAT	0x118
#define SUNXI_DAC_DRC_LPFHRT	0x11C
#define SUNXI_DAC_DRC_LPFLRT	0x120
#define SUNXI_DAC_DRC_RPFHRT	0x124
#define SUNXI_DAC_DRC_RPFLRT	0x128
#define SUNXI_DAC_DRC_LRMSHAT	0x12C
#define SUNXI_DAC_DRC_LRMSLAT	0x130
#define SUNXI_DAC_DRC_RRMSHAT	0x134
#define SUNXI_DAC_DRC_RRMSLAT	0x138
#define SUNXI_DAC_DRC_HCT	0x13C
#define SUNXI_DAC_DRC_LCT	0x140
#define SUNXI_DAC_DRC_HKC	0x144
#define SUNXI_DAC_DRC_LKC	0x148
#define SUNXI_DAC_DRC_HOPC	0x14C
#define SUNXI_DAC_DRC_LOPC	0x150
#define SUNXI_DAC_DRC_HLT	0x154
#define SUNXI_DAC_DRC_LLT	0x158
#define SUNXI_DAC_DRC_HKI	0x15C
#define SUNXI_DAC_DRC_LKI	0x160
#define SUNXI_DAC_DRC_HOPL	0x164
#define SUNXI_DAC_DRC_LOPL	0x168
#define SUNXI_DAC_DRC_HET	0x16C
#define SUNXI_DAC_DRC_LET	0x170
#define SUNXI_DAC_DRC_HKE	0x174
#define SUNXI_DAC_DRC_LKE	0x178
#define SUNXI_DAC_DRC_HOPE	0x17C
#define SUNXI_DAC_DRC_LOPE	0x180
#define SUNXI_DAC_DRC_HKN	0x184
#define SUNXI_DAC_DRC_LKN	0x188
#define SUNXI_DAC_DRC_SFHAT	0x18C
#define SUNXI_DAC_DRC_SFLAT	0x190
#define SUNXI_DAC_DRC_SFHRT	0x194
#define SUNXI_DAC_DRC_SFLRT	0x198
#define SUNXI_DAC_DRC_MXGHS	0x19C
#define SUNXI_DAC_DRC_MXGLS	0x1A0
#define SUNXI_DAC_DRC_MNGHS	0x1A4
#define SUNXI_DAC_DRC_MNGLS	0x1A8
#define SUNXI_DAC_DRC_EPSHC	0x1AC
#define SUNXI_DAC_DRC_EPSLC	0x1B0
#define SUNXI_DAC_DRC_OPT	0x1B4
#define SUNXI_DAC_DRC_HPFHGAIN	0x1B8
#define SUNXI_DAC_DRC_HPFLGAIN	0x1BC

#define SUNXI_ADC_DRC_HHPFC	0x200
#define SUNXI_ADC_DRC_LHPFC	0x204
#define SUNXI_ADC_DRC_CTRL	0x208
#define SUNXI_ADC_DRC_LPFHAT	0x20C
#define SUNXI_ADC_DRC_LPFLAT	0x210
#define SUNXI_ADC_DRC_RPFHAT	0x214
#define SUNXI_ADC_DRC_RPFLAT	0x218
#define SUNXI_ADC_DRC_LPFHRT	0x21C
#define SUNXI_ADC_DRC_LPFLRT	0x220
#define SUNXI_ADC_DRC_RPFHRT	0x224
#define SUNXI_ADC_DRC_RPFLRT	0x228
#define SUNXI_ADC_DRC_LRMSHAT	0x22C
#define SUNXI_ADC_DRC_LRMSLAT	0x230
#define SUNXI_ADC_DRC_RRMSHAT	0x234
#define SUNXI_ADC_DRC_RRMSLAT	0x238
#define SUNXI_ADC_DRC_HCT	0x23C
#define SUNXI_ADC_DRC_LCT	0x240
#define SUNXI_ADC_DRC_HKC	0x244
#define SUNXI_ADC_DRC_LKC	0x248
#define SUNXI_ADC_DRC_HOPC	0x24C
#define SUNXI_ADC_DRC_LOPC	0x250
#define SUNXI_ADC_DRC_HLT	0x254
#define SUNXI_ADC_DRC_LLT	0x258
#define SUNXI_ADC_DRC_HKI	0x25C
#define SUNXI_ADC_DRC_LKI	0x260
#define SUNXI_ADC_DRC_HOPL	0x264
#define SUNXI_ADC_DRC_LOPL	0x268
#define SUNXI_ADC_DRC_HET	0x26C
#define SUNXI_ADC_DRC_LET	0x270
#define SUNXI_ADC_DRC_HKE	0x274
#define SUNXI_ADC_DRC_LKE	0x278
#define SUNXI_ADC_DRC_HOPE	0x27C
#define SUNXI_ADC_DRC_LOPE	0x280
#define SUNXI_ADC_DRC_HKN	0x284
#define SUNXI_ADC_DRC_LKN	0x288
#define SUNXI_ADC_DRC_SFHAT	0x28C
#define SUNXI_ADC_DRC_SFLAT	0x290
#define SUNXI_ADC_DRC_SFHRT	0x294
#define SUNXI_ADC_DRC_SFLRT	0x298
#define SUNXI_ADC_DRC_MXGHS	0x29C
#define SUNXI_ADC_DRC_MXGLS	0x2A0
#define SUNXI_ADC_DRC_MNGHS	0x2A4
#define SUNXI_ADC_DRC_MNGLS	0x2A8
#define SUNXI_ADC_DRC_EPSHC	0x2AC
#define SUNXI_ADC_DRC_EPSLC	0x2B0
#define SUNXI_ADC_DRC_OPT	0x2B4
#define SUNXI_ADC_DRC_HPFHGAIN	0x2B8
#define SUNXI_ADC_DRC_HPFLGAIN	0x2BC

#define SUNXI_AC_VERSION	0x2C0

/* Analog register */
#define SUNXI_ADC1_REG		0x300
#define SUNXI_ADC2_REG		0x304
#define SUNXI_ADC3_REG		0x308
#define SUNXI_DAC_REG		0x310
#define SUNXI_MICBIAS_REG	0x318
#define SUNXI_RAMP_REG		0x31C
#define SUNXI_BIAS_REG		0x320
#define SUNXI_HMIC_CTRL		0x328
#define SUNXI_HMIC_STS		0x32C
#define SUNXI_HP2_REG		0x340
#define SUNXI_POWER_REG		0x348
#define SUNXI_ADC_CUR_REG	0x34C

/* SUNXI_DAC_DPC:0x00 */
#define EN_DAC			31
#define MODQU			25
#define DWA_EN			24
#define HPF_EN			18
#define DVOL			12
#define DAC_HUB_EN		0

/* SUNXI_DAC_VOL_CTRL:0x04 */
#define DAC_VOL_SEL		16
#define DAC_VOL_L		8
#define DAC_VOL_R		0

/* SUNXI_DAC_FIFOC:0x10 */
#define DAC_FS			29
#define FIR_VER			28
#define SEND_LASAT		26
#define FIFO_MODE		24
#define DAC_DRQ_CLR_CNT		21
#define TX_TRIG_LEVEL		8
#define DAC_MONO_EN		6
#define TX_SAMPLE_BITS		5
#define DAC_DRQ_EN		4
#define DAC_IRQ_EN		3
#define FIFO_UNDERRUN_IRQ_EN	2
#define FIFO_OVERRUN_IRQ_EN	1
#define FIFO_FLUSH		0

/* SUNXI_DAC_FIFOS:0x14 */
#define	TX_EMPTY		23
#define	DAC_TXE_CNT		8
#define	DAC_TXE_INT		3
#define	DAC_TXU_INT		2
#define	DAC_TXO_INT		1

/* SUNXI_DAC_DG:0x28 */
#define	DAC_MODU_SEL		11
#define	DAC_PATTERN_SEL		9
#define	DAC_CODEC_CLK_SEL	8
#define	DAC_SWP			6
#define	ADDA_LOOP_MODE		0

/* SUNXI_ADC_FIFOC:0x30 */
#define ADC_FS			29
#define EN_AD			28
#define ADCFDT			26
#define ADCDFEN			25
#define RX_FIFO_MODE		24
#define RX_SYNC_EN_STA		21
#define RX_SYNC_EN		20
#define RX_SAMPLE_BITS		16
#define RX_FIFO_TRG_LEVEL	4
#define ADC_DRQ_EN		3
#define ADC_IRQ_EN		2
#define ADC_OVERRUN_IRQ_EN	1
#define ADC_FIFO_FLUSH		0

/* SUNXI_ADC_VOL_CTRL:0x34 */
#define ADC3_VOL		16
#define ADC2_VOL		8
#define ADC1_VOL		0

/* SUNXI_ADC_FIFOS:0x38 */
#define	RXA			23
#define	ADC_RXA_CNT		8
#define	ADC_RXA_INT		3
#define	ADC_RXO_INT		1

/* SUNXI_ADC_DG:0x4C */
#define	AD_SWP2			25
#define	AD_SWP1			24

/* SUNXI_ADC_DIG_CTRL:0x50 */
#define	ADC3_VOL_EN		17
#define	ADC1_2_VOL_EN		16
#define	ADC3_CHANNEL_EN		2
#define	ADC2_CHANNEL_EN		1
#define	ADC1_CHANNEL_EN		0
#define	ADC_CHANNEL_EN		0

/* SUNXI_VRA1SPEEDUP_DOWN_CTRL:0x54 */
#define	VRA1SPEEDUP_DOWN_STATE		4
#define	VRA1SPEEDUP_DOWN_CTRL		1
#define	VRA1SPEEDUP_DOWN_RST_CTRL	0

/* SUNXI_DAC_DAP_CTL:0xf0 */
#define	DDAP_EN			31
#define	DDAP_DRC_EN		29
#define	DDAP_HPF_EN		28

/* SUNXI_ADC_DAP_CTL:0xf8 */
#define	ADC_DAP0_EN		31
#define	ADC_DRC0_EN		29
#define	ADC_HPF0_EN		28
#define	ADC_DAP1_EN		27
#define	ADC_DRC1_EN		25
#define	ADC_HPF1_EN		24

/* SUNXI_DAC_DRC_HHPFC : 0x100*/
#define DAC_HHPF_CONF		0

/* SUNXI_DAC_DRC_LHPFC : 0x104*/
#define DAC_LHPF_CONF		0

/* SUNXI_DAC_DRC_CTRL : 0x108*/
#define DAC_DRC_DELAY_OUT_STATE		15
#define DAC_DRC_SIGNAL_DELAY		8
#define DAC_DRC_DELAY_BUF_EN		7
#define DAC_DRC_GAIN_MAX_EN		6
#define DAC_DRC_GAIN_MIN_EN		5
#define DAC_DRC_NOISE_DET_EN		4
#define DAC_DRC_SIGNAL_SEL		3
#define DAC_DRC_DELAY_EN		2
#define DAC_DRC_LT_EN			1
#define DAC_DRC_ET_EN			0

/* SUNXI_ADC_DRC_HHPFC : 0x200*/
#define ADC_HHPF_CONF		0

/* SUNXI_ADC_DRC_LHPFC : 0x204*/
#define ADC_LHPF_CONF		0

/* SUNXI_ADC_DRC_CTRL : 0x208*/
#define ADC_DRC_DELAY_OUT_STATE		15
#define ADC_DRC_SIGNAL_DELAY		8
#define ADC_DRC_DELAY_BUF_EN		7
#define ADC_DRC_GAIN_MAX_EN		6
#define ADC_DRC_GAIN_MIN_EN		5
#define ADC_DRC_NOISE_DET_EN		4
#define ADC_DRC_SIGNAL_SEL		3
#define ADC_DRC_DELAY_EN		2
#define ADC_DRC_LT_EN			1
#define ADC_DRC_ET_EN			0

/* SUNXI_ADC1_REG : 0x300 */
#define ADC1_EN			31
#define MIC1_PGA_EN		30
#define ADC1_DITHER_CTRL	29
#define MIC1_SIN_EN		28
#define FMINLEN			27
#define FMINLG			26
#define ADC1_DSM_DITHER_LVL	24
#define LINEINLEN		23
#define LINEINLG		22
#define ADC1_IOPBUFFER		20
#define ADC1_PGA_CTRL_RCM	18
#define ADC1_PGA_IN_VCM_CTRL	16
#define IOPADC			14
#define ADC1_PGA_GAIN_CTRL	8
#define ADC1_IOPAAF		6
#define ADC1_IOPSDM1		4
#define ADC1_IOPSDM2		2
#define ADC1_IOPMIC		0

/* SUNXI_ADC2_REG : 0x304 */
#define ADC2_EN			31
#define MIC2_PGA_EN		30
#define ADC2_DITHER_CTRL	29
#define MIC2_SIN_EN		28
#define FMINREN			27
#define FMINRG			26
#define ADC2_DSM_DITHER_LVL	24
#define LINEINREN		23
#define LINEINRG		22
#define ADC2_IOPBUFFER		20
#define ADC2_PGA_CTRL_RCM	18
#define ADC2_PGA_IN_VCM_CTRL	16
#define ADC2_PGA_GAIN_CTRL	8
#define ADC2_IOPAAF		6
#define ADC2_IOPSDM1		4
#define ADC2_IOPSDM2		2
#define ADC2_IOPMIC		0

/* SUNXI_ADC3_REG : 0x308 */
#define ADC3_EN			31
#define MIC3_PGA_EN		30
#define ADC3_DITHER_CTRL	29
#define MIC3_SIN_EN		28
#define ADC3_DSM_DITHER_LVL	24
#define ADC3_IOPBUFFER		20
#define ADC3_PGA_CTRL_RCM	18
#define ADC3_PGA_IN_VCM_CTRL	16
#define ADC3_PGA_GAIN_CTRL	8
#define ADC3_IOPAAF		6
#define ADC3_IOPSDM1		4
#define ADC3_IOPSDM2		2
#define ADC3_IOPMIC		0

/* SUNXI_DAC_REG : 0x310 */
#define CURRENT_TEST_SELECT	23
#define	VRA2_IOPVRS		20
#define	ILINEOUTAMPS		18
#define IOPDACS			16
#define DACLEN			15
#define DACREN			14
#define LINEOUTLEN		13
#define DACLMUTE		12
#define LINEOUTREN		11
#define DACRMUTE		10
#define LINEOUTLDIFFEN		6
#define LINEOUTRDIFFEN		5
#define LINEOUT_VOL		0

/* SUNXI_MICBIAS_REG : 0x318 */
#define SELDETADCFS		28
#define SELDETADCDB		26
#define SELDETADCBF		24
#define JACKDETEN		23
#define SELDETADCDY		21
#define MICADCEN		20
#define POPFREE			19
#define DETMODE			18
#define AUTOPLEN		17
#define MICDETPL		16
#define HMICBIASEN		15
#define HBIASSEL		13
#define	HMICBIAS_CHOP_EN	12
#define HMICBIAS_CHOP_CLK_SEL	10
#define MMICBIASEN		7
#define	MBIASSEL		5
#define	MMICBIAS_CHOP_EN	4
#define MMICBIAS_CHOP_CLK_SEL	2

/* SUNXI_RAMP_REG : 0x31C */
#define RAMP_RISE_INT_EN	31
#define RAMP_RISE_INT		30
#define RAMP_FALL_INT_EN	29
#define RAMP_FALL_INT		28
#define RAMP_SRST		24
#define RAMP_CLK_DIV_M		16
#define HP_PULL_OUT_EN		15
#define RAMP_HOLD_STEP		12
#define GAP_STEP		8
#define RAMP_STEP		4
#define RMD_EN			3
#define RMU_EN			2
#define RMC_EN			1
#define RD_EN			0

/* SUNXI_BIAS_REG : 0x320 */
#define AC_BIASDATA		0

/* SUNXI_HMIC_CTRL : 0x328 */
#define HMIC_SAMPLE_SEL		21
#define MDATA_THRESHOLD		16
#define HMIC_SF			14
#define HMIC_M			10
#define HMIC_N			6
#define MDATA_THRESHOLD_DB	3
#define JACK_OUT_IRQ_EN		2
#define JACK_IN_IRQ_EN		1
#define MIC_DET_IRQ_EN		0

/* SUNXI_HMIC_STS : 0x32C */
#define MDATA_DISCARD		13
#define	HMIC_DATA		8
#define JACK_DET_OUT_ST		4
#define JACK_DET_OIRQ		4
#define JACK_DET_IIN_ST 	3
#define JACK_DET_IIRQ		3
#define MIC_DET_ST		0

/* SUNXI_HP2_REG	:0x340 */
#define HPFB_BUF_EN		31
#define HEADPHONE_GAIN		28
#define HPFB_RES		26
#define OPDRV_CUR		24
#define IOPHP			22
#define HP_DRVEN		21
#define HP_DRVOUTEN		20
#define RSWITCH			19
#define RAMPEN			18
#define HPFB_IN_EN		17
#define RAMP_FINAL_CTRL		16
#define RAMP_OUT_EN		15
#define RAMP_FINAL_STATE_RES	13
#define FB_BUF_OUTPUT_CURRENT	8

/* SUNXI_POWER_REG      :0x348 */
#define ALDO_EN			31
#define HPLDO_EN		30
#define VAR1SPEEDUP_DOWN_FURTHER_CTRL	29
#define AVCCPOR			16
#define ALDO_OUTPUT_VOLTAGE	12
#define HPLDO_OUTPUT_VOLTAGE	8
#define BG_TRIM			0

/* SUNXI_ADC_CUR_REG    :0x34C */
#define ADC3_IOPMIC2		20
#define ADC3_OP_MIC1_CUR	18
#define ADC3_OP_MIC2_CUR	16
#define ADC2_IOPMIC2		12
#define ADC2_OP_MIC1_CUR	10
#define ADC2_OP_MIC2_CUR	8
#define ADC1_IOPMIC2		4
#define ADC1_OP_MIC1_CUR	2
#define ADC1_OP_MIC2_CUR	0

#define CODEC_TX_FIFO_SIZE	128
#define CODEC_RX_FIFO_SIZE	256

/* gain select */
#define MIC1_GAIN_SHIFT		1
#define MIC2_GAIN_SHIFT		2
#define MIC3_GAIN_SHIFT		3

#define HP_GAIN_SHIFT		4

/*125ms * (HP_DEBOUCE_TIME+1)*/
#define HP_DEBOUCE_TIME	0x3

#define REG_LABEL(constant)	{#constant, constant, 0}
#define REG_LABEL_END           {NULL, 0, 0}

/* SUNXI_CODEC_DAP_ENABLE: Whether to use the adc/dac drc/hpf function */
#define SUNXI_CODEC_DAP_ENABLE

/* SUNXI_CODEC_HUB_ENABLE: Whether to use the hub mode */
#define SUNXI_CODEC_HUB_ENABLE

/* SUNXI_CODEC_ADCSWAP_ENABLE: Whether to open the adc swap func controls */
#define SUNXI_CODEC_ADCSWAP_ENABLE

struct reg_label {
	const char *name;
	const unsigned int address;
	unsigned int value;
};

struct codec_hw_config {
	u32 adcdrc_cfg:8;
	u32 dacdrc_cfg:8;
	u32 adchpf_cfg:8;
	u32 dachpf_cfg:8;
};

struct codec_spk_config {
	u32 spk_gpio;
	u32 pa_msleep;
	bool used;
	bool pa_level;
};

struct sample_rate {
	unsigned int samplerate;
	unsigned int rate_bit;
};

struct voltage_supply {
	struct regulator *avcc;
	struct regulator *hpvcc;
};

struct codec_dap {
	int drc_enable;
	int hpf_enable;
};

struct sunxi_codec_info {
	struct device *dev;
	struct regmap *regmap;
	void __iomem *digital_base;
	struct clk *pllaudio0;
	struct clk *pllaudio1_div5;
	struct clk *dacclk;
	struct clk *adcclk;
	struct clk *codec_clk_bus;
	struct reset_control *codec_clk_rst;

	/* regulator about */
	struct voltage_supply vol_supply;

	/* for dap function */
	struct codec_dap dac_dap;
	struct codec_dap adc_dap;
	int dac_dap_enable;
	int adc_dap_enable;

	/* self user config params */
	u32 digital_vol;
	u32 lineout_vol;
	u32 dac_digital_vol;

	bool mic1gain_now;
	bool mic2gain_now;
	bool mic3gain_now;
	bool hpgain_now;
	u32 mic1gain;
	u32 mic2gain;
	u32 mic3gain;
	u32 headphonegain;

	u32 rx_sync_en;
	int rx_sync_id;
	rx_sync_domain_t rx_sync_domain;

	struct codec_spk_config spk_config;
	struct codec_spk_config spk_pwr_config;
	struct codec_hw_config hw_config;
};

#endif
