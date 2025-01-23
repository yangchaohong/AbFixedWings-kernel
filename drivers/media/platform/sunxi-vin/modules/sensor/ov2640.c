/*
 * A V4L2 driver for ov2640 YUV cameras.
 *
 * Copyright (c) 2017 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Authors:  Zhao Wei <zhaowei@allwinnertech.com>
 *    Yang Feng <yangfeng@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>

#include "camera.h"
#include "sensor_helper.h"


MODULE_AUTHOR("raymonxiu");
MODULE_DESCRIPTION("A low-level driver for ov2640 sensors");
MODULE_LICENSE("GPL");

#define AF_WIN_NEW_COORD

#define MCLK              (24*1000*1000)
int MCLK_DIV = 1;
#ifdef CONFIG_ARCH_SUN9IW1P1
int A80_VERSION;
#endif

#define VREF_POL	V4L2_MBUS_VSYNC_ACTIVE_HIGH	// V4L2_MBUS_VSYNC_ACTIVE_LOW 
#define HREF_POL	V4L2_MBUS_HSYNC_ACTIVE_HIGH	// V4L2_MBUS_HSYNC_ACTIVE_LOW
#define CLK_POL     V4L2_MBUS_PCLK_SAMPLE_RISING //V4L2_MBUS_PCLK_SAMPLE_RISING


#define V4L2_IDENT_SENSOR 0x2642

#define SENSOR_NAME "ov2640"

#ifdef _FLASH_FUNC_
#include "../modules/flash/flash.h"
unsigned int to_flash;
static unsigned int flash_auto_level = 0x1c;
#endif
#define CONTINUEOUS_AF

#define DENOISE_LV_AUTO
#define SHARPNESS 0x18

#ifdef AUTO_FPS

#endif

#ifndef DENOISE_LV_AUTO
#define DENOISE_LV 0x8
#endif

#define AE_CW 1

unsigned int night_mode;
unsigned int Nfrms = 1;
unsigned int cap_manual_gain = 0x10;
#define CAP_GAIN_CAL 0
#define CAP_MULTI_FRAMES
#ifdef CAP_MULTI_FRAMES
#define MAX_FRM_CAP 4
#else
#define MAX_FRM_CAP 1
#endif

/*
 * Our nominal (default) frame rate.
 */
#define SENSOR_FRAME_RATE 15

/*
 * The ov2640 sits on i2c with ID 0x78
 */
//#define I2C_ADDR 0x78 /*wutianhao*/

#define I2C_ADDR 0x60 /*wutianhao*/

struct cfg_array {		/* coming later */
	struct regval_list *regs;
	int size;
};


 /*以下ov2640设置*/
/*
 * The default register settings
 *
 */

#define VAL_SET(x, mask, rshift, lshift)  \
		((((x) >> rshift) & mask) << lshift)
/*
 * DSP registers
 * register offset for BANK_SEL == BANK_SEL_DSP
 */
#define R_BYPASS    0x05 /* Bypass DSP */
#define   R_BYPASS_DSP_BYPAS    0x01 /* Bypass DSP, sensor out directly */
#define   R_BYPASS_USE_DSP      0x00 /* Use the internal DSP */
#define QS          0x44 /* Quantization Scale Factor */
#define CTRLI       0x50
#define   CTRLI_LP_DP           0x80
#define   CTRLI_ROUND           0x40
#define   CTRLI_V_DIV_SET(x)    VAL_SET(x, 0x3, 0, 3)
#define   CTRLI_H_DIV_SET(x)    VAL_SET(x, 0x3, 0, 0)
#define HSIZE       0x51 /* H_SIZE[7:0] (real/4) */
#define   HSIZE_SET(x)          VAL_SET(x, 0xFF, 2, 0)
#define VSIZE       0x52 /* V_SIZE[7:0] (real/4) */
#define   VSIZE_SET(x)          VAL_SET(x, 0xFF, 2, 0)
#define XOFFL       0x53 /* OFFSET_X[7:0] */
#define   XOFFL_SET(x)          VAL_SET(x, 0xFF, 0, 0)
#define YOFFL       0x54 /* OFFSET_Y[7:0] */
#define   YOFFL_SET(x)          VAL_SET(x, 0xFF, 0, 0)
#define VHYX        0x55 /* Offset and size completion */
#define   VHYX_VSIZE_SET(x)     VAL_SET(x, 0x1, (8+2), 7)
#define   VHYX_HSIZE_SET(x)     VAL_SET(x, 0x1, (8+2), 3)
#define   VHYX_YOFF_SET(x)      VAL_SET(x, 0x3, 8, 4)
#define   VHYX_XOFF_SET(x)      VAL_SET(x, 0x3, 8, 0)
#define DPRP        0x56
#define TEST        0x57 /* Horizontal size completion */
#define   TEST_HSIZE_SET(x)     VAL_SET(x, 0x1, (9+2), 7)
#define ZMOW        0x5A /* Zoom: Out Width  OUTW[7:0] (real/4) */
#define   ZMOW_OUTW_SET(x)      VAL_SET(x, 0xFF, 2, 0)
#define ZMOH        0x5B /* Zoom: Out Height OUTH[7:0] (real/4) */
#define   ZMOH_OUTH_SET(x)      VAL_SET(x, 0xFF, 2, 0)
#define ZMHH        0x5C /* Zoom: Speed and H&W completion */
#define   ZMHH_ZSPEED_SET(x)    VAL_SET(x, 0x0F, 0, 4)
#define   ZMHH_OUTH_SET(x)      VAL_SET(x, 0x1, (8+2), 2)
#define   ZMHH_OUTW_SET(x)      VAL_SET(x, 0x3, (8+2), 0)
#define BPADDR      0x7C /* SDE Indirect Register Access: Address */
#define BPDATA      0x7D /* SDE Indirect Register Access: Data */
#define CTRL2       0x86 /* DSP Module enable 2 */
#define   CTRL2_DCW_EN          0x20
#define   CTRL2_SDE_EN          0x10
#define   CTRL2_UV_ADJ_EN       0x08
#define   CTRL2_UV_AVG_EN       0x04
#define   CTRL2_CMX_EN          0x01
#define CTRL3       0x87 /* DSP Module enable 3 */
#define   CTRL3_BPC_EN          0x80
#define   CTRL3_WPC_EN          0x40
#define SIZEL       0x8C /* Image Size Completion */
#define   SIZEL_HSIZE8_11_SET(x) VAL_SET(x, 0x1, 11, 6)
#define   SIZEL_HSIZE8_SET(x)    VAL_SET(x, 0x7, 0, 3)
#define   SIZEL_VSIZE8_SET(x)    VAL_SET(x, 0x7, 0, 0)
#define HSIZE8      0xC0 /* Image Horizontal Size HSIZE[10:3] */
#define   HSIZE8_SET(x)         VAL_SET(x, 0xFF, 3, 0)
#define VSIZE8      0xC1 /* Image Vertical Size VSIZE[10:3] */
#define   VSIZE8_SET(x)         VAL_SET(x, 0xFF, 3, 0)
#define CTRL0       0xC2 /* DSP Module enable 0 */
#define   CTRL0_AEC_EN       0x80
#define   CTRL0_AEC_SEL      0x40
#define   CTRL0_STAT_SEL     0x20
#define   CTRL0_VFIRST       0x10
#define   CTRL0_YUV422       0x08
#define   CTRL0_YUV_EN       0x04
#define   CTRL0_RGB_EN       0x02
#define   CTRL0_RAW_EN       0x01
#define CTRL1       0xC3 /* DSP Module enable 1 */
#define   CTRL1_CIP          0x80
#define   CTRL1_DMY          0x40
#define   CTRL1_RAW_GMA      0x20
#define   CTRL1_DG           0x10
#define   CTRL1_AWB          0x08
#define   CTRL1_AWB_GAIN     0x04
#define   CTRL1_LENC         0x02
#define   CTRL1_PRE          0x01
/*      REG 0xC7 (unknown name): affects Auto White Balance (AWB)
 *	  AWB_OFF            0x40
 *	  AWB_SIMPLE         0x10
 *	  AWB_ON             0x00	(Advanced AWB ?) */
#define R_DVP_SP    0xD3 /* DVP output speed control */
#define   R_DVP_SP_AUTO_MODE 0x80
#define   R_DVP_SP_DVP_MASK  0x3F /* DVP PCLK = sysclk (48)/[6:0] (YUV0);
				   *          = sysclk (48)/(2*[6:0]) (RAW);*/
#define IMAGE_MODE  0xDA /* Image Output Format Select */
#define   IMAGE_MODE_Y8_DVP_EN   0x40
#define   IMAGE_MODE_JPEG_EN     0x10
#define   IMAGE_MODE_YUV422      0x00
#define   IMAGE_MODE_RAW10       0x04 /* (DVP) */
#define   IMAGE_MODE_RGB565      0x08
#define   IMAGE_MODE_HREF_VSYNC  0x02 /* HREF timing select in DVP JPEG output
				       * mode (0 for HREF is same as sensor) */
#define   IMAGE_MODE_LBYTE_FIRST 0x01 /* Byte swap enable for DVP
				       *    1: Low byte first UYVY (C2[4] =0)
				       *        VYUY (C2[4] =1)
				       *    0: High byte first YUYV (C2[4]=0)
				       *        YVYU (C2[4] = 1) */
#define OV2640_RESET       0xE0 /* Reset */
#define   RESET_MICROC       0x40
#define   RESET_SCCB         0x20
#define   RESET_JPEG         0x10
#define   RESET_DVP          0x04
#define   RESET_IPU          0x02
#define   RESET_CIF          0x01
#define REGED       0xED /* Register ED */
#define   REGED_CLK_OUT_DIS  0x10
#define MS_SP       0xF0 /* SCCB Master Speed */
#define SS_ID       0xF7 /* SCCB Slave ID */
#define SS_CTRL     0xF8 /* SCCB Slave Control */
#define   SS_CTRL_ADD_AUTO_INC  0x20
#define   SS_CTRL_EN            0x08
#define   SS_CTRL_DELAY_CLK     0x04
#define   SS_CTRL_ACC_EN        0x02
#define   SS_CTRL_SEN_PASS_THR  0x01
#define MC_BIST     0xF9 /* Microcontroller misc register */
#define   MC_BIST_RESET           0x80 /* Microcontroller Reset */
#define   MC_BIST_BOOT_ROM_SEL    0x40
#define   MC_BIST_12KB_SEL        0x20
#define   MC_BIST_12KB_MASK       0x30
#define   MC_BIST_512KB_SEL       0x08
#define   MC_BIST_512KB_MASK      0x0C
#define   MC_BIST_BUSY_BIT_R      0x02
#define   MC_BIST_MC_RES_ONE_SH_W 0x02
#define   MC_BIST_LAUNCH          0x01
#define BANK_SEL    0xFF /* Register Bank Select */
#define   BANK_SEL_DSP     0x00
#define   BANK_SEL_SENS    0x01

/*
 * Sensor registers
 * register offset for BANK_SEL == BANK_SEL_SENS
 */
#define GAIN        0x00 /* AGC - Gain control gain setting */
#define COM1        0x03 /* Common control 1 */
#define   COM1_1_DUMMY_FR          0x40
#define   COM1_3_DUMMY_FR          0x80
#define   COM1_7_DUMMY_FR          0xC0
#define   COM1_VWIN_LSB_UXGA       0x0F
#define   COM1_VWIN_LSB_SVGA       0x0A
#define   COM1_VWIN_LSB_CIF        0x06
#define REG04       0x04 /* Register 04 */
#define   REG04_DEF             0x20 /* Always set */
#define   REG04_HFLIP_IMG       0x80 /* Horizontal mirror image ON/OFF */
#define   REG04_VFLIP_IMG       0x40 /* Vertical flip image ON/OFF */
#define   REG04_VREF_EN         0x10
#define   REG04_HREF_EN         0x08
#define   REG04_AEC_SET(x)      VAL_SET(x, 0x3, 0, 0)
#define REG08       0x08 /* Frame Exposure One-pin Control Pre-charge Row Num */
#define COM2        0x09 /* Common control 2 */
#define   COM2_SOFT_SLEEP_MODE  0x10 /* Soft sleep mode */
				     /* Output drive capability */
#define   COM2_OCAP_Nx_SET(N)   (((N) - 1) & 0x03) /* N = [1x .. 4x] */
#define PID         0x0A /* Product ID Number MSB */
#define VER         0x0B /* Product ID Number LSB */
#define COM3        0x0C /* Common control 3 */
#define   COM3_BAND_50H        0x04 /* 0 For Banding at 60H */
#define   COM3_BAND_AUTO       0x02 /* Auto Banding */
#define   COM3_SING_FR_SNAPSH  0x01 /* 0 For enable live video output after the
				     * snapshot sequence*/
#define AEC         0x10 /* AEC[9:2] Exposure Value */
#define CLKRC       0x11 /* Internal clock */
#define   CLKRC_EN             0x80
#define   CLKRC_DIV_SET(x)     (((x) - 1) & 0x1F) /* CLK = XVCLK/(x) */
#define COM7        0x12 /* Common control 7 */
#define   COM7_SRST            0x80 /* Initiates system reset. All registers are
				     * set to factory default values after which
				     * the chip resumes normal operation */
#define   COM7_RES_UXGA        0x00 /* Resolution selectors for UXGA */
#define   COM7_RES_SVGA        0x40 /* SVGA */
#define   COM7_RES_CIF         0x20 /* CIF */
#define   COM7_ZOOM_EN         0x04 /* Enable Zoom mode */
#define   COM7_COLOR_BAR_TEST  0x02 /* Enable Color Bar Test Pattern */
#define COM8        0x13 /* Common control 8 */
#define   COM8_DEF             0xC0
#define   COM8_BNDF_EN         0x20 /* Banding filter ON/OFF */
#define   COM8_AGC_EN          0x04 /* AGC Auto/Manual control selection */
#define   COM8_AEC_EN          0x01 /* Auto/Manual Exposure control */
#define COM9        0x14 /* Common control 9
			  * Automatic gain ceiling - maximum AGC value [7:5]*/
#define   COM9_AGC_GAIN_2x     0x00 /* 000 :   2x */
#define   COM9_AGC_GAIN_4x     0x20 /* 001 :   4x */
#define   COM9_AGC_GAIN_8x     0x40 /* 010 :   8x */
#define   COM9_AGC_GAIN_16x    0x60 /* 011 :  16x */
#define   COM9_AGC_GAIN_32x    0x80 /* 100 :  32x */
#define   COM9_AGC_GAIN_64x    0xA0 /* 101 :  64x */
#define   COM9_AGC_GAIN_128x   0xC0 /* 110 : 128x */
#define COM10       0x15 /* Common control 10 */
#define   COM10_PCLK_HREF      0x20 /* PCLK output qualified by HREF */
#define   COM10_PCLK_RISE      0x10 /* Data is updated at the rising edge of
				     * PCLK (user can latch data at the next
				     * falling edge of PCLK).
				     * 0 otherwise. */
#define   COM10_HREF_INV       0x08 /* Invert HREF polarity:
				     * HREF negative for valid data*/
#define   COM10_VSINC_INV      0x02 /* Invert VSYNC polarity */
#define HSTART      0x17 /* Horizontal Window start MSB 8 bit */
#define HEND        0x18 /* Horizontal Window end MSB 8 bit */
#define VSTART      0x19 /* Vertical Window start MSB 8 bit */
#define VEND        0x1A /* Vertical Window end MSB 8 bit */
#define MIDH        0x1C /* Manufacturer ID byte - high */
#define MIDL        0x1D /* Manufacturer ID byte - low  */
#define AEW         0x24 /* AGC/AEC - Stable operating region (upper limit) */
#define AEB         0x25 /* AGC/AEC - Stable operating region (lower limit) */
#define VV          0x26 /* AGC/AEC Fast mode operating region */
#define   VV_HIGH_TH_SET(x)      VAL_SET(x, 0xF, 0, 4)
#define   VV_LOW_TH_SET(x)       VAL_SET(x, 0xF, 0, 0)
#define REG2A       0x2A /* Dummy pixel insert MSB */
#define FRARL       0x2B /* Dummy pixel insert LSB */
#define ADDVFL      0x2D /* LSB of insert dummy lines in Vertical direction */
#define ADDVFH      0x2E /* MSB of insert dummy lines in Vertical direction */
#define YAVG        0x2F /* Y/G Channel Average value */
#define REG32       0x32 /* Common Control 32 */
#define   REG32_PCLK_DIV_2    0x80 /* PCLK freq divided by 2 */
#define   REG32_PCLK_DIV_4    0xC0 /* PCLK freq divided by 4 */
#define ARCOM2      0x34 /* Zoom: Horizontal start point */
#define REG45       0x45 /* Register 45 */
#define FLL         0x46 /* Frame Length Adjustment LSBs */
#define FLH         0x47 /* Frame Length Adjustment MSBs */
#define COM19       0x48 /* Zoom: Vertical start point */
#define ZOOMS       0x49 /* Zoom: Vertical start point */
#define COM22       0x4B /* Flash light control */
#define COM25       0x4E /* For Banding operations */
#define   COM25_50HZ_BANDING_AEC_MSBS_MASK      0xC0 /* 50Hz Bd. AEC 2 MSBs */
#define   COM25_60HZ_BANDING_AEC_MSBS_MASK      0x30 /* 60Hz Bd. AEC 2 MSBs */
#define   COM25_50HZ_BANDING_AEC_MSBS_SET(x)    VAL_SET(x, 0x3, 8, 6)
#define   COM25_60HZ_BANDING_AEC_MSBS_SET(x)    VAL_SET(x, 0x3, 8, 4)
#define BD50        0x4F /* 50Hz Banding AEC 8 LSBs */
#define   BD50_50HZ_BANDING_AEC_LSBS_SET(x)     VAL_SET(x, 0xFF, 0, 0)
#define BD60        0x50 /* 60Hz Banding AEC 8 LSBs */
#define   BD60_60HZ_BANDING_AEC_LSBS_SET(x)     VAL_SET(x, 0xFF, 0, 0)
#define REG5A       0x5A /* 50/60Hz Banding Maximum AEC Step */
#define   BD50_MAX_AEC_STEP_MASK         0xF0 /* 50Hz Banding Max. AEC Step */
#define   BD60_MAX_AEC_STEP_MASK         0x0F /* 60Hz Banding Max. AEC Step */
#define   BD50_MAX_AEC_STEP_SET(x)       VAL_SET((x - 1), 0x0F, 0, 4)
#define   BD60_MAX_AEC_STEP_SET(x)       VAL_SET((x - 1), 0x0F, 0, 0)
#define REG5D       0x5D /* AVGsel[7:0],   16-zone average weight option */
#define REG5E       0x5E /* AVGsel[15:8],  16-zone average weight option */
#define REG5F       0x5F /* AVGsel[23:16], 16-zone average weight option */
#define REG60       0x60 /* AVGsel[31:24], 16-zone average weight option */
#define HISTO_LOW   0x61 /* Histogram Algorithm Low Level */
#define HISTO_HIGH  0x62 /* Histogram Algorithm High Level */

/*初始化寄存器*/

//#define ENDMARKER { 0xff, 0xff }

static struct regval_list sensor_default_regs[] = {
	{ BANK_SEL, BANK_SEL_DSP },
	{ 0x2c,   0xff },
	{ 0x2e,   0xdf },
	{ BANK_SEL, BANK_SEL_SENS },
	{ 0x3c,   0x32 },
	{ CLKRC,  CLKRC_DIV_SET(1) },
	{ COM2,   COM2_OCAP_Nx_SET(3) },
	{ REG04,  REG04_DEF | REG04_HREF_EN },
	{ COM8,   COM8_DEF | COM8_BNDF_EN | COM8_AGC_EN | COM8_AEC_EN },
	{ COM9,   COM9_AGC_GAIN_8x | 0x08},
	{ 0x2c,   0x0c },
	{ 0x33,   0x78 },
	{ 0x3a,   0x33 },
	{ 0x3b,   0xfb },
	{ 0x3e,   0x00 },
	{ 0x43,   0x11 },
	{ 0x16,   0x10 },
	{ 0x39,   0x02 },
	{ 0x35,   0x88 },
	{ 0x22,   0x0a },
	{ 0x37,   0x40 },
	{ 0x23,   0x00 },
	{ ARCOM2, 0xa0 },
	{ 0x06,   0x02 },
	{ 0x06,   0x88 },
	{ 0x07,   0xc0 },
	{ 0x0d,   0xb7 },
	{ 0x0e,   0x01 },
	{ 0x4c,   0x00 },
	{ 0x4a,   0x81 },
	{ 0x21,   0x99 },
	{ AEW,    0x40 },
	{ AEB,    0x38 },
	{ VV,     VV_HIGH_TH_SET(0x08) | VV_LOW_TH_SET(0x02) },
	{ 0x5c,   0x00 },
	{ 0x63,   0x00 },
	{ FLL,    0x22 },  /*wutianhao*/
//	{ FLL,    0x00 },
	{ COM3,   0x38 | COM3_BAND_AUTO },
	{ REG5D,  0x55 },
	{ REG5E,  0x7d },
	{ REG5F,  0x7d },
	{ REG60,  0x55 },
	{ HISTO_LOW,   0x70 },
	{ HISTO_HIGH,  0x80 },
	{ 0x7c,   0x05 },
	{ 0x20,   0x80 },
	{ 0x28,   0x30 },
	{ 0x6c,   0x00 },
	{ 0x6d,   0x80 },
	{ 0x6e,   0x00 },
	{ 0x70,   0x02 },
	{ 0x71,   0x94 },
	{ 0x73,   0xc1 },
	{ 0x3d,   0x34 },
	{ COM7,   COM7_RES_UXGA | COM7_ZOOM_EN  },
	{ REG5A,  BD50_MAX_AEC_STEP_SET(6)
		   | BD60_MAX_AEC_STEP_SET(8) },		/* 0x57 */
	{ COM25,  COM25_50HZ_BANDING_AEC_MSBS_SET(0x0bb)
		   | COM25_60HZ_BANDING_AEC_MSBS_SET(0x09c) },	/* 0x00 */
	{ BD50,   BD50_50HZ_BANDING_AEC_LSBS_SET(0x0bb) },	/* 0xbb */
	{ BD60,   BD60_60HZ_BANDING_AEC_LSBS_SET(0x09c) },	/* 0x9c */
	{ BANK_SEL,  BANK_SEL_DSP },
	{ 0xe5,   0x7f },
	{ MC_BIST,  MC_BIST_RESET | MC_BIST_BOOT_ROM_SEL },
	{ 0x41,   0x24 },
	{ OV2640_RESET,  RESET_JPEG | RESET_DVP },
	{ 0x76,   0xff },
	{ 0x33,   0xa0 },
	{ 0x42,   0x20 },
	{ 0x43,   0x18 },
	{ 0x4c,   0x00 },
	{ CTRL3,  CTRL3_BPC_EN | CTRL3_WPC_EN | 0x10 },
	{ 0x88,   0x3f },
	{ 0xd7,   0x03 },
	{ 0xd9,   0x10 },
	{ R_DVP_SP,  R_DVP_SP_AUTO_MODE | 0x2 },
	{ 0xc8,   0x08 },
	{ 0xc9,   0x80 },
	{ BPADDR, 0x00 },
	{ BPDATA, 0x00 },
	{ BPADDR, 0x03 },
	{ BPDATA, 0x48 },
	{ BPDATA, 0x48 },
	{ BPADDR, 0x08 },
	{ BPDATA, 0x20 },
	{ BPDATA, 0x10 },
	{ BPDATA, 0x0e },
	{ 0x90,   0x00 },
	{ 0x91,   0x0e },
	{ 0x91,   0x1a },
	{ 0x91,   0x31 },
	{ 0x91,   0x5a },
	{ 0x91,   0x69 },
	{ 0x91,   0x75 },
	{ 0x91,   0x7e },
	{ 0x91,   0x88 },
	{ 0x91,   0x8f },
	{ 0x91,   0x96 },
	{ 0x91,   0xa3 },
	{ 0x91,   0xaf },
	{ 0x91,   0xc4 },
	{ 0x91,   0xd7 },
	{ 0x91,   0xe8 },
	{ 0x91,   0x20 },
	{ 0x92,   0x00 },
	{ 0x93,   0x06 },
	{ 0x93,   0xe3 },
	{ 0x93,   0x03 },
	{ 0x93,   0x03 },
	{ 0x93,   0x00 },
	{ 0x93,   0x02 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x93,   0x00 },
	{ 0x96,   0x00 },
	{ 0x97,   0x08 },
	{ 0x97,   0x19 },
	{ 0x97,   0x02 },
	{ 0x97,   0x0c },
	{ 0x97,   0x24 },
	{ 0x97,   0x30 },
	{ 0x97,   0x28 },
	{ 0x97,   0x26 },
	{ 0x97,   0x02 },
	{ 0x97,   0x98 },
	{ 0x97,   0x80 },
	{ 0x97,   0x00 },
	{ 0x97,   0x00 },
	{ 0xa4,   0x00 },
	{ 0xa8,   0x00 },
	{ 0xc5,   0x11 },
	{ 0xc6,   0x51 },
	{ 0xbf,   0x80 },
	{ 0xc7,   0x10 },	/* simple AWB */
	{ 0xb6,   0x66 },
	{ 0xb8,   0xA5 },
	{ 0xb7,   0x64 },
	{ 0xb9,   0x7C },
	{ 0xb3,   0xaf },
	{ 0xb4,   0x97 },
	{ 0xb5,   0xFF },
	{ 0xb0,   0xC5 },
	{ 0xb1,   0x94 },
	{ 0xb2,   0x0f },
	{ 0xc4,   0x5c },
	{ 0xa6,   0x00 },
	{ 0xa7,   0x20 },
	{ 0xa7,   0xd8 },
	{ 0xa7,   0x1b },
	{ 0xa7,   0x31 },
	{ 0xa7,   0x00 },
	{ 0xa7,   0x18 },
	{ 0xa7,   0x20 },
	{ 0xa7,   0xd8 },
	{ 0xa7,   0x19 },
	{ 0xa7,   0x31 },
	{ 0xa7,   0x00 },
	{ 0xa7,   0x18 },
	{ 0xa7,   0x20 },
	{ 0xa7,   0xd8 },
	{ 0xa7,   0x19 },
	{ 0xa7,   0x31 },
	{ 0xa7,   0x00 },
	{ 0xa7,   0x18 },
	{ 0x7f,   0x00 },
	{ 0xe5,   0x1f },
	{ 0xe1,   0x77 },
	{ 0xdd,   0x7f },
	{ CTRL0,  CTRL0_YUV422 | CTRL0_YUV_EN | CTRL0_RGB_EN },
//	ENDMARKER,
};

/*设置分辨率*/

#define PER_SIZE_REG_SEQ(x, y, v_div, h_div, pclk_div)	\
	{ CTRLI, CTRLI_LP_DP | CTRLI_V_DIV_SET(v_div) |	\
		 CTRLI_H_DIV_SET(h_div)},		\
	{ ZMOW, ZMOW_OUTW_SET(x) },			\
	{ ZMOH, ZMOH_OUTH_SET(y) },			\
	{ ZMHH, ZMHH_OUTW_SET(x) | ZMHH_OUTH_SET(y) },	\
	{ R_DVP_SP, pclk_div | R_DVP_SP_AUTO_MODE },				\
	{ OV2640_RESET, 0x00}

static struct regval_list sensor_vga_regs[] = {
	PER_SIZE_REG_SEQ(VGA_WIDTH, VGA_HEIGHT, 0, 0, 63),
//	ENDMARKER,
};




static const struct regval_list ov2640_yuyv_regs[] = {
	{ IMAGE_MODE, IMAGE_MODE_YUV422 },
	{ 0xd7, 0x03 },
	{ 0x33, 0xa0 },
	{ 0xe5, 0x1f },
	{ 0xe1, 0x67 },
	{ OV2640_RESET,  0x00 },
	{ R_BYPASS, R_BYPASS_USE_DSP },
//	ENDMARKER,
};


static const struct regval_list ov2640_size_change_preamble_regs[] = {
	{ BANK_SEL, BANK_SEL_DSP },
	{ OV2640_RESET, RESET_DVP },
	{ SIZEL, SIZEL_HSIZE8_11_SET(UXGA_WIDTH) |
		 SIZEL_HSIZE8_SET(UXGA_WIDTH) |
		 SIZEL_VSIZE8_SET(UXGA_HEIGHT) },
	{ HSIZE8, HSIZE8_SET(UXGA_WIDTH) },
	{ VSIZE8, VSIZE8_SET(UXGA_HEIGHT) },
	{ CTRL2, CTRL2_DCW_EN | CTRL2_SDE_EN |
		 CTRL2_UV_AVG_EN | CTRL2_CMX_EN | CTRL2_UV_ADJ_EN },
	{ HSIZE, HSIZE_SET(UXGA_WIDTH) },
	{ VSIZE, VSIZE_SET(UXGA_HEIGHT) },
	{ XOFFL, XOFFL_SET(0) },
	{ YOFFL, YOFFL_SET(0) },
	{ VHYX, VHYX_HSIZE_SET(UXGA_WIDTH) | VHYX_VSIZE_SET(UXGA_HEIGHT) |
		VHYX_XOFF_SET(0) | VHYX_YOFF_SET(0)},
	{ TEST, TEST_HSIZE_SET(UXGA_WIDTH) },
//	ENDMARKER,
};


static const struct regval_list ov2640_format_change_preamble_regs[] = {
	{ BANK_SEL, BANK_SEL_DSP },
	{ R_BYPASS, R_BYPASS_USE_DSP },
//	ENDMARKER,
};

static int ov2640_reset(struct v4l2_subdev *sd)
{
	int ret;
	static const struct regval_list reset_seq[] = {
		{BANK_SEL, BANK_SEL_SENS},
		{COM7, COM7_SRST},
	//	ENDMARKER,
	};

	ret = sensor_write_array(sd, reset_seq,ARRAY_SIZE(reset_seq));
	if (ret==-1)
		goto err;

	msleep(5);
err:
	printk("%s: (ret %d)", __func__, ret);
	return ret;
}


/*ov2640上电不需要太多操作*/
static int sensor_power(struct v4l2_subdev *sd, int on)
{
	int ret = 0;
#ifdef _FLASH_FUNC_
	struct modules_config *modules = sd_to_modules(sd);
#endif
	switch (on) {
	case STBY_ON:
		printk("STBY_ON\r\n");
		sensor_dbg("STBY_ON!\n");
// #ifdef _FLASH_FUNC_
// 		io_set_flash_ctrl(modules->modules.flash.sd,
// 				  SW_CTRL_FLASH_OFF);
// #endif
		// sensor_s_release_af(sd);
		// ret =
		//     sensor_write_array(sd, sensor_sw_stby_on_regs,
		// 		       ARRAY_SIZE(sensor_sw_stby_on_regs));
		// if (ret < 0)
		// 	sensor_err("soft stby falied!\n");
		usleep_range(10000, 12000);
		sensor_print("disalbe oe!\n");
		// ret =
		//     sensor_write_array(sd, sensor_oe_disable_regs,
		// 		       ARRAY_SIZE(sensor_oe_disable_regs));
		// if (ret < 0)
		// 	sensor_err("disalbe oe falied!\n");

		cci_lock(sd);
		vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
		cci_unlock(sd);
		vin_set_mclk(sd, OFF);
		break;
	case STBY_OFF:
		printk("STBY_OFF\r\n");
		sensor_dbg("STBY_OFF!\n");
		cci_lock(sd);
		vin_set_mclk_freq(sd, MCLK / MCLK_DIV);
		vin_set_mclk(sd, ON);
		usleep_range(10000, 12000);
		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		usleep_range(10000, 12000);
		cci_unlock(sd);
		sensor_print("enable oe!\n");
		// ret =
		//     sensor_write_array(sd, sensor_oe_enable_regs,
		// 		       ARRAY_SIZE(sensor_oe_enable_regs));
		// if (ret < 0)
		// 	sensor_err("enable oe falied!\n");
		// ret =
		//     sensor_write_array(sd, sensor_sw_stby_off_regs,
		// 		       ARRAY_SIZE(sensor_sw_stby_off_regs));
		// if (ret < 0)
		// 	sensor_err("soft stby off falied!\n");
		usleep_range(10000, 12000);
		break;
	case PWR_ON:
		sensor_print("PWR_ON!\n");
		cci_lock(sd);
		vin_set_pmu_channel(sd, CAMERAVDD, ON);
		vin_gpio_set_status(sd, PWDN, 1);
		vin_gpio_set_status(sd, RESET, 1);
		vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		usleep_range(1000, 1200);
		vin_set_mclk_freq(sd, MCLK / MCLK_DIV);
		vin_set_mclk(sd, ON);
		usleep_range(10000, 12000);
		vin_gpio_write(sd, POWER_EN, CSI_GPIO_HIGH);
		vin_set_pmu_channel(sd, IOVDD, ON);
		vin_set_pmu_channel(sd, AVDD, ON);
		vin_set_pmu_channel(sd, DVDD, ON);
		vin_set_pmu_channel(sd, AFVDD, ON);
		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		usleep_range(10000, 12000);
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		usleep_range(30000, 31000);
		cci_unlock(sd);
		break;
	case PWR_OFF:
		sensor_print("PWR_OFF!\n");
		cci_lock(sd);
		vin_set_mclk(sd, OFF);
		vin_gpio_write(sd, POWER_EN, CSI_GPIO_LOW);
		vin_set_pmu_channel(sd, AFVDD, OFF);
		vin_set_pmu_channel(sd, DVDD, OFF);
		vin_set_pmu_channel(sd, AVDD, OFF);
		vin_set_pmu_channel(sd, IOVDD, OFF);
		vin_set_pmu_channel(sd, CAMERAVDD, OFF);
		usleep_range(10000, 12000);
		vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		vin_gpio_set_status(sd, RESET, 0);
		vin_gpio_set_status(sd, PWDN, 0);
		cci_unlock(sd);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	// switch (val) {
	// case 0:
	// 	vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
	// 	usleep_range(10000, 12000);
	// 	break;
	// case 1:
	// 	vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
	// 	usleep_range(10000, 12000);
	// 	break;
	// default:
	// 	return -EINVAL;
	// }
	sensor_print("sensor_reset \n"); /*重启设备*/
	return 0;
}


static int sensor_detect(struct v4l2_subdev *sd)
{
	unsigned int SENSOR_ID = 0;
	data_type val;
	int cnt = 0;
	unsigned char pid, midh, midl;
	sensor_read(sd, 0x0a, &val);
	SENSOR_ID |= (val << 8);
	sensor_read(sd, 0x0b, &val);
	SENSOR_ID |= (val);
	sensor_print("wutianhao V4L2_IDENT_SENSOR = %x\n", SENSOR_ID);
	return 0;

}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	ret = sensor_detect(sd);
	if (ret) {
		sensor_err("chip found is not an target chip.\n");
		return ret;
	}
	//ret =sensor_write_array(sd, sensor_default_regs,ARRAY_SIZE(sensor_default_regs));
	if (ret < 0) {
		sensor_err("write sensor_default_regs error\n");
		return ret;
	}
	return 0;
}



static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct sensor_info *info = to_state(sd);

	switch (cmd) {
	case GET_CURRENT_WIN_CFG:
		if (info->current_wins) {
			memcpy(arg, info->current_wins,
				sizeof(struct sensor_win_size));
			ret = 0;
		} else {
			sensor_err("empty wins!\n");
			ret = -1;
		}
		break;
	case GET_SENSOR_EXIF:
		//sensor_g_exif(sd, (struct sensor_exif_attribute *)arg);
		break;
	case SET_AUTO_FOCUS_WIN:
		//sensor_s_af_win(sd, (struct v4l2_win_setting *)arg);
		break;
	case SET_AUTO_EXPOSURE_WIN:
		//sensor_s_ae_win(sd, (struct v4l2_win_setting *)arg);
		break;
	case VIDIOC_VIN_SENSOR_EXP_GAIN:
		//sensor_s_exp_gain(sd, (struct sensor_exp_gain *)arg);
		break;
	case VIDIOC_VIN_SENSOR_CFG_REQ:
		//sensor_cfg_req(sd, (struct sensor_config *)arg);
		break;

	default:
		return -EINVAL;
	}
	return ret;
}

/*
 * Store information about the video data format.
 */
static struct sensor_format_struct sensor_formats[] = {
	{
		.desc = "YUYV 4:2:2",
		.mbus_code = MEDIA_BUS_FMT_YUYV8_2X8,
		.regs = ov2640_yuyv_regs,
		.regs_size = ARRAY_SIZE(ov2640_yuyv_regs),
		.bpp = 2, /*修改每个像素几个数据*/
	}, 
};

#define N_FMTS ARRAY_SIZE(sensor_formats)

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */

static struct sensor_win_size sensor_win_sizes[] = {
	{

	.width = VGA_WIDTH,
	.height = VGA_HEIGHT,
	.hoffset = 0,
	.voffset = 0,
	// .hts = 640,
	// .vts = 480,
	//.pclk = 9216 * 1000,
	//.pclk = 6144*1000,
	// .fps_fixed = 1,
	// .bin_factor = 1,
	// .intg_min = 1,
	// .intg_max = 480 << 4,
	// .gain_min = 1 << 4,
	// .gain_max = 10 << 4,
	.regs = sensor_vga_regs,
	.regs_size = ARRAY_SIZE(sensor_vga_regs),
	.set_size = NULL,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static int sensor_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_PARALLEL;
	cfg->flags = V4L2_MBUS_MASTER | VREF_POL | HREF_POL | CLK_POL;

	return 0;
}

static int ov2640_mask_set(struct v4l2_subdev *sd,u8  reg, u8  mask, u8  set)
{
	data_type val;
	sensor_read(sd, reg, &val);
	val &= ~mask;
	val |= set & mask;
	printk("wutianhao val num %x\r\n",val);
	return sensor_write(sd, reg, val);
}


static int ov2640_set_params(struct v4l2_subdev *sd, u32 code)
{
	const struct regval_list *selected_cfmt_regs;
	u8 val;
	int ret;

	/* reset hardware */
	ov2640_reset(sd);
	//ret =sensor_write_array(sd, sensor_default_regs,ARRAY_SIZE(sensor_default_regs));
	ret = sensor_write_array(sd, sensor_default_regs,ARRAY_SIZE(sensor_default_regs));
	if (ret < 0)
		goto err;

	/* select preamble */
	ret = sensor_write_array(sd, ov2640_size_change_preamble_regs,ARRAY_SIZE(ov2640_size_change_preamble_regs));
	if (ret < 0)
		goto err;

	/* set size win */
	ret = sensor_write_array(sd, sensor_vga_regs,ARRAY_SIZE(sensor_vga_regs)); /*固定分辨率*/
	if (ret < 0)
		goto err;
	//ov2640_mask_set(sd, COM7, COM7_COLOR_BAR_TEST, COM7_COLOR_BAR_TEST);

	/* cfmt preamble */
	ret = sensor_write_array(sd, ov2640_format_change_preamble_regs,ARRAY_SIZE(ov2640_format_change_preamble_regs));
	if (ret < 0)
		goto err;
	/* set cfmt */
	ret = sensor_write_array(sd, ov2640_yuyv_regs,ARRAY_SIZE(ov2640_yuyv_regs));
	if (ret < 0)
		goto err;
	val = (code == MEDIA_BUS_FMT_YVYU8_2X8)|| (code == MEDIA_BUS_FMT_VYUY8_2X8) ? CTRL0_VFIRST : 0x00;
	
	ret = ov2640_mask_set(sd, CTRL0, CTRL0_VFIRST, val);
	if (ret < 0)
		goto err;

	return 0;

err:
	printk("%s: Error %d\r\n", __func__, ret);
	ov2640_reset(sd);

	return ret;
}

// static int ov2640_s_stream(struct v4l2_subdev *sd, int on)
// {
// 	struct i2c_client *client = v4l2_get_subdevdata(sd);
// 	struct ov2640_priv *priv = to_ov2640(client);
// 	int ret = 0;

// 	mutex_lock(&priv->lock);
// 	if (priv->streaming == !on) {
// 		if (on) {
// 			ret = ov2640_set_params(client, priv->win,
// 						priv->cfmt_code);
// 			if (!ret)
// 				ret = __v4l2_ctrl_handler_setup(&priv->hdl);
// 		}
// 	}
// 	if (!ret)
// 		priv->streaming = on;
// 	mutex_unlock(&priv->lock);

// 	return ret;
// }


/*sensor start stream */
static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sensor_info *info = to_state(sd);
	if(enable == 1)
	{
		ov2640_set_params(sd,MEDIA_BUS_FMT_YUYV8_2X8);
		sensor_print("sensor  start\r\n");
	}else
	{
		sensor_print("sensor stop\r\n");
	}
	return 0;
}

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sensor_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_stream = sensor_s_stream,
	.g_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_mbus_code = sensor_enum_mbus_code,
	.enum_frame_size = sensor_enum_frame_size,
	.get_fmt = sensor_get_fmt,
	.set_fmt = sensor_set_fmt,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
	.pad = &sensor_pad_ops,
};


/* ----------------------------------------------------------------------- */
static struct cci_driver cci_drv = {
	.name = SENSOR_NAME,
	.addr_width = CCI_BITS_8, /*wutianhao 修改地址宽度*/ 
	.data_width = CCI_BITS_8, /*wutianhao*/
};

/*挂载设备*/
static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
	printk("wutianhao %s\r\n",__FUNCTION__);
	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	cci_dev_probe_helper(sd, client, &sensor_ops, &cci_drv); /*初始化v4l2相关东西 这里面还包含初始化芯片等操作*/

	mutex_init(&info->lock);
#ifdef CONFIG_SAME_I2C
	info->sensor_i2c_addr = I2C_ADDR >> 1;
#endif
	info->fmt = &sensor_formats[0];
	info->fmt_pt = &sensor_formats[0];
	info->win_pt = &sensor_win_sizes[0];
	info->fmt_num = N_FMTS;
	info->win_size_num = N_WIN_SIZES;
	info->sensor_field = V4L2_FIELD_NONE;
	info->af_first_flag = 0;
	info->auto_focus = 0;
	return 0;
}

static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd;

	sd = cci_dev_remove_helper(client, &cci_drv);
	pr_info("sensor_remove ov2640 sd = %p!\n", sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{SENSOR_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sensor_id);
static struct i2c_driver sensor_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = SENSOR_NAME,
		   },
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};
/*对ℹi2c设备进行初始化*/
static __init int init_sensor(void)
{
#ifdef CONFIG_ARCH_SUN9IW1P1
	A80_VERSION = sunxi_get_soc_ver();
	if (A80_VERSION >= SUN9IW1P1_REV_B)
		MCLK_DIV = 1;
	else
		MCLK_DIV = 2;
	pr_info("A80_VERSION = %d , SUN9IW1P1_REV_B = %d, MCLK_DIV = %d\n",
	       A80_VERSION, SUN9IW1P1_REV_B, MCLK_DIV);
#else
	MCLK_DIV = 1;
#endif
	printk("wutianhao %s %d\r\n",__FUNCTION__,MCLK_DIV);
	return cci_dev_init_helper(&sensor_driver);
}

static __exit void exit_sensor(void)
{
	cci_dev_exit_helper(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);