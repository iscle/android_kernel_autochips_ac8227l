/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   RAW
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   Leo Lee
 *
 *============================================================================
 *             HISTORY
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 09 26  2014
 * Lanking Zhou
 * Beta1.0.0
 * First release MT6589 GC2375_MIPI_RAW driver Version 1.0
 *
 *------------------------------------------------------------------------------
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc2375_mipi_raw_Sensor.h"
#include "gc2375_mipi_raw_Camera_Sensor_para.h"
#include "gc2375_mipi_raw_CameraCustomized.h"

#ifdef GC2375_MIPI_RAW_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif


#define GC2375_MIPI_RAW_TEST_PATTERN_CHECKSUM 0x54ddf19b

#if defined(MT6577)||defined(MT6589)
static DEFINE_SPINLOCK(gc2375_mipi_raw_drv_lock);
#endif

//#define IMAGE_NORMAL_MIRROR
//#define IMAGE_H_MIRROR
//#define IMAGE_V_MIRROR
#define IMAGE_HV_MIRROR

#ifdef IMAGE_NORMAL_MIRROR
#define MIRROR 0xd4
#define STARTY 0x01
#define STARTX 0x01
#define BLK_Select1_H 0x00
#define BLK_Select1_L 0x3c
#define BLK_Select2_H 0x00
#define BLK_Select2_L 0x03
#endif

#ifdef IMAGE_H_MIRROR
#define MIRROR 0xd5
#define STARTY 0x01
#define STARTX 0x00
#define BLK_Select1_H 0x00
#define BLK_Select1_L 0x3c
#define BLK_Select2_H 0x00
#define BLK_Select2_L 0x03
#endif

#ifdef IMAGE_V_MIRROR
#define MIRROR 0xd6
#define STARTY 0x02
#define STARTX 0x01
#define BLK_Select1_H 0x3c
#define BLK_Select1_L 0x00
#define BLK_Select2_H 0xc0
#define BLK_Select2_L 0x00
#endif

#ifdef IMAGE_HV_MIRROR
#define MIRROR 0xd7
#define STARTY 0x02
#define STARTX 0x00
#define BLK_Select1_H 0x3c
#define BLK_Select1_L 0x00
#define BLK_Select2_H 0xc0
#define BLK_Select2_L 0x00
#endif

//#define GC2375_MIPI_USE_OTP

//static kal_uint16 Half_iShutter=0;
//static kal_bool b_Enable_Half = false;
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);


static GC2375_MIPI_RAW_sensor_struct GC2375_MIPI_RAW_sensor =
{
	.eng_info =
	{
		.SensorId = 128,
		.SensorType = CMOS_SENSOR,
		.SensorOutputDataFormat = GC2375_MIPI_RAW_COLOR_FORMAT,
	},
	.Mirror = GC2375_MIPI_RAW_IMAGE_V_MIRROR,
	.shutter = 0x20,
	.gain = 0x20,
	.pclk = GC2375_MIPI_RAW_PREVIEW_CLK,
	.frame_height = GC2375_MIPI_RAW_PV_PERIOD_LINE_NUMS,
	.line_length = GC2375_MIPI_RAW_PV_PERIOD_PIXEL_NUMS,
};


/*************************************************************************
* FUNCTION
*    gc2375_mipi_raw_write_cmos_sensor
*
* DESCRIPTION
*    This function wirte data to CMOS sensor through I2C
*
* PARAMETERS
*    addr: the 16bit address of register
*    para: the 8bit value of register
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void gc2375_mipi_raw_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
kal_uint8 out_buff[2];

    out_buff[0] = addr;
    out_buff[1] = para;

    iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), GC2375_MIPI_RAW_WRITE_ID);
}

/*************************************************************************
* FUNCTION
*    GC2035_read_cmos_sensor
*
* DESCRIPTION
*    This function read data from CMOS sensor through I2C.
*
* PARAMETERS
*    addr: the 16bit address of register
*
* RETURNS
*    8bit data read through I2C
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint8 gc2375_mipi_raw_read_cmos_sensor(kal_uint8 addr)
{
  kal_uint8 in_buff[1] = {0xFF};
  kal_uint8 out_buff[1];

  out_buff[0] = addr;

    if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16) sizeof(in_buff), GC2375_MIPI_RAW_WRITE_ID)) {
        SENSORDB("ERROR: gc2375_mipi_raw_read_cmos_sensor \n");
    }
  return in_buff[0];
}


#ifdef GC2375_MIPI_USE_OTP

typedef struct otp_gc2375{
	kal_uint16 wb_flag;
	kal_uint16 r_gain;
	kal_uint16 g_gain;
	kal_uint16 b_gain;
	kal_uint16 g_r_gain_l;
	kal_uint16 b_g_gain_l;
	kal_uint16 ob_flag;
	kal_uint16 g1_slope;
	kal_uint16 r1_slope;
	kal_uint16 b2_slope;
	kal_uint16 g2_slope;
	kal_uint16 ob_slope;
}gc2375_otp;

static gc2375_otp gc2375_otp_info;

typedef enum{
	otp_close=0,
	otp_open,
}otp_state;

static kal_uint8 gc2375_read_otp(kal_uint8 addr)
{
	kal_uint8 value;

	gc2375_mipi_raw_write_cmos_sensor(0xfe,0x00);
	gc2375_mipi_raw_write_cmos_sensor(0xd5,addr);
	gc2375_mipi_raw_write_cmos_sensor(0xf3,0x20);
	value = read_cmos_sensor(0xd7);

	return value;
}

static void gc2375_gcore_read_otp_info(void)
{
	kal_uint8 flag;
	memset(&gc2375_otp_info,0,sizeof(gc2375_otp));

	flag = gc2375_read_otp(0x00);
	printk("GC2375_OTP : flag = 0x%x\n",flag);

	gc2375_otp_info.wb_flag = flag&0x03;
	gc2375_otp_info.ob_flag = (flag>>2)&0x03;

	/*WB*/
	if(0x01==gc2375_otp_info.wb_flag)
	{
		printk("GC2375_OTP_WB:Valid");
		/*
		gc2375_otp_info.r_gain = (gc2375_read_otp(0x08)<<3)|(gc2375_read_otp(0x20)&0x07);
		gc2375_otp_info.g_gain = (gc2375_read_otp(0x10)<<3)|((gc2375_read_otp(0x20)>>4)&0x07);
		gc2375_otp_info.b_gain = (gc2375_read_otp(0x18)<<3)|(gc2375_read_otp(0x28)&0x07);
		*/
		gc2375_otp_info.r_gain = gc2375_read_otp(0x08);
		gc2375_otp_info.g_gain = gc2375_read_otp(0x10);
		gc2375_otp_info.b_gain = gc2375_read_otp(0x18);
		gc2375_otp_info.g_r_gain_l= gc2375_read_otp(0x20);
		gc2375_otp_info.b_g_gain_l= gc2375_read_otp(0x28);

		printk("GC2375_OTP_WB:r_gain=0x%x\n",gc2375_otp_info.r_gain);
		printk("GC2375_OTP_WB:g_gain=0x%x\n",gc2375_otp_info.g_gain);
		printk("GC2375_OTP_WB:b_gain=0x%x\n",gc2375_otp_info.b_gain);
		printk("GC2375_OTP_WB:g_r_gain_l=0x%x\n",gc2375_otp_info.g_r_gain_l);
		printk("GC2375_OTP_WB:b_g_gain_l=0x%x\n",gc2375_otp_info.b_g_gain_l);
	}
	else
	{
		printk("GC2375_OTP_WB:Invalid!");
	}

	/*OB*/
	if(0x01==gc2375_otp_info.ob_flag)
	{
		printk("GC2375_OTP_OB:Valid");
		/*
		gc2375_otp_info.g1_slope = ((gc2375_read_otp(0x50)&0x01)<<8)|(gc2375_read_otp(0x30));
		gc2375_otp_info.r1_slope = ((gc2375_read_otp(0x50)&0x02)<<7)|(gc2375_read_otp(0x38));
		gc2375_otp_info.b2_slope = ((gc2375_read_otp(0x50)&0x04)<<6)|(gc2375_read_otp(0x40));
		gc2375_otp_info.g2_slope = ((gc2375_read_otp(0x50)&0x08)<<5)|(gc2375_read_otp(0x48));
		*/
		gc2375_otp_info.g1_slope = gc2375_read_otp(0x30);
		gc2375_otp_info.r1_slope = gc2375_read_otp(0x38);
		gc2375_otp_info.b2_slope = gc2375_read_otp(0x40);
		gc2375_otp_info.g2_slope =  gc2375_read_otp(0x48);
		gc2375_otp_info.ob_slope =  gc2375_read_otp(0x50);

		printk("GC2375_OTP_OB:g1_slope=0x%x\n",gc2375_otp_info.g1_slope);
		printk("GC2375_OTP_OB:r1_slope=0x%x\n",gc2375_otp_info.r1_slope);
		printk("GC2375_OTP_OB:b2_slope=0x%x\n",gc2375_otp_info.b2_slope);
		printk("GC2375_OTP_OB:g2_slope=0x%x\n",gc2375_otp_info.g2_slope);
		printk("GC2375_OTP_OB:ob_slope=0x%x\n",gc2375_otp_info.ob_slope);
	}
	else
	{
		printk("GC2375_OTP_OB:Invalid!");
	}

}

static void gc2375_gcore_update_wb(void)
{
	if(0x01==gc2375_otp_info.wb_flag)
	{
		gc2375_mipi_raw_write_cmos_sensor(0xfe,0x00);
		gc2375_mipi_raw_write_cmos_sensor(0xb8,gc2375_otp_info.g_gain);
		gc2375_mipi_raw_write_cmos_sensor(0xb9,gc2375_otp_info.r_gain);
		gc2375_mipi_raw_write_cmos_sensor(0xba,gc2375_otp_info.b_gain);
		gc2375_mipi_raw_write_cmos_sensor(0xbb,gc2375_otp_info.g_gain);
		gc2375_mipi_raw_write_cmos_sensor(0xbe,gc2375_otp_info.g_r_gain_l);
		gc2375_mipi_raw_write_cmos_sensor(0xbf,gc2375_otp_info.b_g_gain_l);
		gc2375_mipi_raw_write_cmos_sensor(0xfe,0x00);
	}
}

static void gc2375_gcore_update_ob(void)
{
	if(0x01==gc2375_otp_info.ob_flag)
	{
		gc2375_mipi_raw_write_cmos_sensor(0xfe,0x01);
		gc2375_mipi_raw_write_cmos_sensor(0x41,gc2375_otp_info.g1_slope);
		gc2375_mipi_raw_write_cmos_sensor(0x42,gc2375_otp_info.r1_slope);
		gc2375_mipi_raw_write_cmos_sensor(0x43,gc2375_otp_info.b2_slope);
		gc2375_mipi_raw_write_cmos_sensor(0x44,gc2375_otp_info.g2_slope);
		gc2375_mipi_raw_write_cmos_sensor(0x45,gc2375_otp_info.ob_slope);
		gc2375_mipi_raw_write_cmos_sensor(0xfe,0x00);
	}

}

static void gc2375_gcore_update_otp(void)
{
	gc2375_gcore_update_wb();
	gc2375_gcore_update_ob();
}

static void gc2375_gcore_enable_otp(otp_state state)
{
	kal_uint8 otp_clk,otp_en;
	otp_clk = gc2375_mipi_raw_read_cmos_sensor(0xfc);
	otp_en= gc2375_mipi_raw_read_cmos_sensor(0xd4);
	if(state)
	{
		otp_clk = otp_clk | 0x10;
		otp_en = otp_en | 0x80;
		mdelay(5);
		gc2375_mipi_raw_write_cmos_sensor(0xfc,otp_clk);	// 0xfc[4]:OTP_CLK_en
		gc2375_mipi_raw_write_cmos_sensor(0xd4,otp_en);	// 0xd4[7]:OTP_en

		printk("GC2375_OTP: Enable OTP!\n");
	}
	else
	{
		otp_en = otp_en & 0x7f;
		otp_clk = otp_clk & 0xef;
		mdelay(5);
		gc2375_mipi_raw_write_cmos_sensor(0xd4,otp_en);
		gc2375_mipi_raw_write_cmos_sensor(0xfc,otp_clk);

		printk("GC2375_OTP: Disable OTP!\n");
	}

}

void gc2375_gcore_identify_otp(void)
{
	gc2375_mipi_raw_write_cmos_sensor(0xfe, 0x00);
	gc2375_mipi_raw_write_cmos_sensor(0xfe, 0x00);
	gc2375_mipi_raw_write_cmos_sensor(0xfe, 0x00);
	gc2375_mipi_raw_write_cmos_sensor(0xf7, 0x01);
	gc2375_mipi_raw_write_cmos_sensor(0xf8, 0x0c);
	gc2375_mipi_raw_write_cmos_sensor(0xf9, 0x42);
	gc2375_mipi_raw_write_cmos_sensor(0xfa, 0x88);
	gc2375_mipi_raw_write_cmos_sensor(0xfc, 0x8e);
	gc2375_gcore_enable_otp(otp_open);
	gc2375_gcore_read_otp_info();
	gc2375_gcore_enable_otp(otp_close);
}

#endif

/*************************************************************************
* FUNCTION
*	GC2375_MIPI_RAW_SetShutter
*
* DESCRIPTION
*	This function set e-shutter of GC2375_MIPI_RAW to change exposure time.
*
* PARAMETERS
*   iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void GC2375_MIPI_RAW_set_shutter(kal_uint16 iShutter)
{
#if defined(MT6577)||defined(MT6589)
	spin_lock(&gc2375_mipi_raw_drv_lock);
#endif
	GC2375_MIPI_RAW_sensor.shutter = iShutter;
#if defined(MT6577)||defined(MT6589)
	spin_unlock(&gc2375_mipi_raw_drv_lock);
#endif

#ifdef GC2375_MIPI_RAW_DRIVER_TRACE
	SENSORDB("GC2375_MIPI_RAW_set_shutter iShutter = %d \n",iShutter);
#endif
	if(iShutter < 1) iShutter = 1;
	if(iShutter > 16383) iShutter = 16383;//2^14

	if (iShutter == (GC2375_MIPI_RAW_PV_PERIOD_LINE_NUMS-1)) //add 20160527
	iShutter +=1;

	//Update Shutter
	gc2375_mipi_raw_write_cmos_sensor(0x04, (iShutter) & 0xFF);
	gc2375_mipi_raw_write_cmos_sensor(0x03, (iShutter >> 8) & 0x3F);
}   /*  Set_GC2375_MIPI_RAW_Shutter */

/*************************************************************************
* FUNCTION
*	GC2375_MIPI_RAW_SetGain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*   iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/

#define ANALOG_GAIN_1 64   // 1.00x
#define ANALOG_GAIN_2 92   // 1.43x
#define ANALOG_GAIN_3 128  // 2.00x
#define ANALOG_GAIN_4 182  // 2.84x
#define ANALOG_GAIN_5 254  // 3.97x
#define ANALOG_GAIN_6 363  // 5.68x
#define ANALOG_GAIN_7 521  // 8.14x
#define ANALOG_GAIN_8 725  // 11.34x
#define ANALOG_GAIN_9 1038 // 16.23x

kal_uint16  GC2375_MIPI_RAW_SetGain(kal_uint16 iGain)
{
	kal_uint16 iReg,temp;

	#ifdef GC2375_MIPI_RAW_DRIVER_TRACE
	SENSORDB("GC2375_MIPI_RAW_SetGain iGain = %d \n",iGain);
	#endif

	iReg = iGain;

	if(iReg < 0x40)
		iReg = 0x40;

	if((ANALOG_GAIN_1<= iReg)&&(iReg < ANALOG_GAIN_2))
	{
		gc2375_mipi_raw_write_cmos_sensor(0x20,0x0b);
		gc2375_mipi_raw_write_cmos_sensor(0x22,0x0c);
		gc2375_mipi_raw_write_cmos_sensor(0x26,0x0e);
		//analog gain
		gc2375_mipi_raw_write_cmos_sensor(0xb6,0x00);
		temp = iReg;
		gc2375_mipi_raw_write_cmos_sensor(0xb1,temp>>6);
		gc2375_mipi_raw_write_cmos_sensor(0xb2,(temp<<2)&0xfc);
		printk("GC2375_MIPI_RAW analogic gain 1x, GC2375_MIPI_RAW add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_2<= iReg)&&(iReg < ANALOG_GAIN_3))
	{
		gc2375_mipi_raw_write_cmos_sensor(0x20,0x0c);
		gc2375_mipi_raw_write_cmos_sensor(0x22,0x0e);
		gc2375_mipi_raw_write_cmos_sensor(0x26,0x0e);
		//analog gain
		gc2375_mipi_raw_write_cmos_sensor(0xb6,0x01);
		temp = 64*iReg/ANALOG_GAIN_2;
		gc2375_mipi_raw_write_cmos_sensor(0xb1,temp>>6);
		gc2375_mipi_raw_write_cmos_sensor(0xb2,(temp<<2)&0xfc);
		printk("GC2375_MIPI_RAW analogic gain 1.375x , GC2375_MIPI_RAW add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_3<= iReg)&&(iReg < ANALOG_GAIN_4))
	{
		gc2375_mipi_raw_write_cmos_sensor(0x20,0x0c);
		gc2375_mipi_raw_write_cmos_sensor(0x22,0x0e);
		gc2375_mipi_raw_write_cmos_sensor(0x26,0x0e);
		//analog gain
		gc2375_mipi_raw_write_cmos_sensor(0xb6,0x02);
		temp = 64*iReg/ANALOG_GAIN_3;
		gc2375_mipi_raw_write_cmos_sensor(0xb1,temp>>6);
		gc2375_mipi_raw_write_cmos_sensor(0xb2,(temp<<2)&0xfc);
		printk("GC2375_MIPI_RAW analogic gain 1.891x , GC2375_MIPI_RAW add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_4<= iReg)&&(iReg < ANALOG_GAIN_5))
	{
		gc2375_mipi_raw_write_cmos_sensor(0x20,0x0c);
		gc2375_mipi_raw_write_cmos_sensor(0x22,0x0e);
		gc2375_mipi_raw_write_cmos_sensor(0x26,0x0e);
		//analog gain
		gc2375_mipi_raw_write_cmos_sensor(0xb6,0x03);
		temp = 64*iReg/ANALOG_GAIN_4;
		gc2375_mipi_raw_write_cmos_sensor(0xb1,temp>>6);
		gc2375_mipi_raw_write_cmos_sensor(0xb2,(temp<<2)&0xfc);
		printk("GC2375_MIPI_RAW analogic gain 2.625x , GC2375_MIPI_RAW add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_5<= iReg)&&(iReg < ANALOG_GAIN_6))
	{
		gc2375_mipi_raw_write_cmos_sensor(0x20,0x0c);
		gc2375_mipi_raw_write_cmos_sensor(0x22,0x0e);
		gc2375_mipi_raw_write_cmos_sensor(0x26,0x0e);
		//analog gain
		gc2375_mipi_raw_write_cmos_sensor(0xb6,0x04);
		temp = 64*iReg/ANALOG_GAIN_5;
		gc2375_mipi_raw_write_cmos_sensor(0xb1,temp>>6);
		gc2375_mipi_raw_write_cmos_sensor(0xb2,(temp<<2)&0xfc);
		printk("GC2375_MIPI_RAW analogic gain 3.734x , GC2375_MIPI_RAW add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_6<= iReg)&&(iReg < ANALOG_GAIN_7))
	{
		gc2375_mipi_raw_write_cmos_sensor(0x20,0x0e);
		gc2375_mipi_raw_write_cmos_sensor(0x22,0x0e);
		gc2375_mipi_raw_write_cmos_sensor(0x26,0x0e);
		//analog gain
		gc2375_mipi_raw_write_cmos_sensor(0xb6,0x05);
		temp = 64*iReg/ANALOG_GAIN_6;
		gc2375_mipi_raw_write_cmos_sensor(0xb1,temp>>6);
		gc2375_mipi_raw_write_cmos_sensor(0xb2,(temp<<2)&0xfc);
		printk("GC2375_MIPI_RAW analogic gain 5.250x , GC2375_MIPI_RAW add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_7<= iReg)&&(iReg < ANALOG_GAIN_8))
	{
		gc2375_mipi_raw_write_cmos_sensor(0x20,0x0c);
		gc2375_mipi_raw_write_cmos_sensor(0x22,0x0c);
		gc2375_mipi_raw_write_cmos_sensor(0x26,0x0e);
		//analog gain
		gc2375_mipi_raw_write_cmos_sensor(0xb6,0x06);
		temp = 64*iReg/ANALOG_GAIN_7;
		gc2375_mipi_raw_write_cmos_sensor(0xb1,temp>>6);
		gc2375_mipi_raw_write_cmos_sensor(0xb2,(temp<<2)&0xfc);
		printk("GC2375_MIPI_RAW analogic gain 3.734x , GC2375_MIPI_RAW add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_8<= iReg)&&(iReg < ANALOG_GAIN_9))
	{
		gc2375_mipi_raw_write_cmos_sensor(0x20,0x0e);
		gc2375_mipi_raw_write_cmos_sensor(0x22,0x0e);
		gc2375_mipi_raw_write_cmos_sensor(0x26,0x0e);
		//analog gain
		gc2375_mipi_raw_write_cmos_sensor(0xb6,0x07);
		temp = 64*iReg/ANALOG_GAIN_8;
		gc2375_mipi_raw_write_cmos_sensor(0xb1,temp>>6);
		gc2375_mipi_raw_write_cmos_sensor(0xb2,(temp<<2)&0xfc);
		printk("GC2375_MIPI_RAW analogic gain 5.250x , GC2375_MIPI_RAW add pregain = %d\n",temp);
	}
	else
	{
		gc2375_mipi_raw_write_cmos_sensor(0x20,0x0c);
		gc2375_mipi_raw_write_cmos_sensor(0x22,0x0e);
		gc2375_mipi_raw_write_cmos_sensor(0x26,0x0e);
		//analog gain
		gc2375_mipi_raw_write_cmos_sensor(0xb6,0x08);
		temp = 64*iReg/ANALOG_GAIN_9;
		gc2375_mipi_raw_write_cmos_sensor(0xb1,temp>>6);
		gc2375_mipi_raw_write_cmos_sensor(0xb2,(temp<<2)&0xfc);
		printk("GC2375_MIPI_RAW analogic gain 7.516x , GC2375_MIPI_RAW add pregain = %d\n",temp);
	}

	return iGain;
}
/*************************************************************************
* FUNCTION
*	GC2375_MIPI_RAW_NightMode
*
* DESCRIPTION
*	This function night mode of GC2375_MIPI_RAW.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void GC2375_MIPI_RAW_night_mode(kal_bool enable)
{
/*No Need to implement this function*/
#if 0
	const kal_uint16 dummy_pixel = GC2375_MIPI_RAW_sensor.line_length - GC2375_MIPI_RAW_PV_PERIOD_PIXEL_NUMS;
	const kal_uint16 pv_min_fps =  enable ? GC2375_MIPI_RAW_sensor.night_fps : GC2375_MIPI_RAW_sensor.normal_fps;
	kal_uint16 dummy_line = GC2375_MIPI_RAW_sensor.frame_height - GC2375_MIPI_RAW_PV_PERIOD_LINE_NUMS;
	kal_uint16 max_exposure_lines;
	printk("[GC2375_MIPI_RAW_night_mode]enable=%d",enable);
	if (!GC2375_MIPI_RAW_sensor.video_mode) return;
	max_exposure_lines = GC2375_MIPI_RAW_sensor.pclk * GC2375_MIPI_RAW_FPS(1) / (pv_min_fps * GC2375_MIPI_RAW_sensor.line_length);
	if (max_exposure_lines > GC2375_MIPI_RAW_sensor.frame_height) /* fix max frame rate, AE table will fix min frame rate */
//	{
//	  dummy_line = max_exposure_lines - GC2375_MIPI_RAW_PV_PERIOD_LINE_NUMS;
//	}
#endif
}   /*  GC2375_MIPI_RAW_NightMode    */


/* write camera_para to sensor register */
static void GC2375_MIPI_RAW_camera_para_to_sensor(void)
{
  kal_uint32 i;
#ifdef GC2375_MIPI_RAW_DRIVER_TRACE
	 SENSORDB("GC2375_MIPI_RAW_camera_para_to_sensor\n");
#endif
  for (i = 0; 0xFFFFFFFF != GC2375_MIPI_RAW_sensor.eng.reg[i].Addr; i++)
  {
    gc2375_mipi_raw_write_cmos_sensor(GC2375_MIPI_RAW_sensor.eng.reg[i].Addr, GC2375_MIPI_RAW_sensor.eng.reg[i].Para);
  }
  for (i = GC2375_MIPI_RAW_FACTORY_START_ADDR; 0xFFFFFFFF != GC2375_MIPI_RAW_sensor.eng.reg[i].Addr; i++)
  {
    gc2375_mipi_raw_write_cmos_sensor(GC2375_MIPI_RAW_sensor.eng.reg[i].Addr, GC2375_MIPI_RAW_sensor.eng.reg[i].Para);
  }
  GC2375_MIPI_RAW_SetGain(GC2375_MIPI_RAW_sensor.gain); /* update gain */
}

/* update camera_para from sensor register */
static void GC2375_MIPI_RAW_sensor_to_camera_para(void)
{
  kal_uint32 i,temp_data;
#ifdef GC2375_MIPI_RAW_DRIVER_TRACE
   SENSORDB("GC2375_MIPI_RAW_sensor_to_camera_para\n");
#endif
  for (i = 0; 0xFFFFFFFF != GC2375_MIPI_RAW_sensor.eng.reg[i].Addr; i++)
  {
  	temp_data = gc2375_mipi_raw_read_cmos_sensor(GC2375_MIPI_RAW_sensor.eng.reg[i].Addr);
#if defined(MT6577)||defined(MT6589)
	spin_lock(&gc2375_mipi_raw_drv_lock);
#endif
    GC2375_MIPI_RAW_sensor.eng.reg[i].Para = temp_data;
#if defined(MT6577)||defined(MT6589)
	spin_unlock(&gc2375_mipi_raw_drv_lock);
#endif
  }
  for (i = GC2375_MIPI_RAW_FACTORY_START_ADDR; 0xFFFFFFFF != GC2375_MIPI_RAW_sensor.eng.reg[i].Addr; i++)
  {
  	temp_data = gc2375_mipi_raw_read_cmos_sensor(GC2375_MIPI_RAW_sensor.eng.reg[i].Addr);
#if defined(MT6577)||defined(MT6589)
	spin_lock(&gc2375_mipi_raw_drv_lock);
#endif
    GC2375_MIPI_RAW_sensor.eng.reg[i].Para = temp_data;
#if defined(MT6577)||defined(MT6589)
	spin_unlock(&gc2375_mipi_raw_drv_lock);
#endif
  }
}

/* ------------------------ Engineer mode ------------------------ */
inline static void GC2375_MIPI_RAW_get_sensor_group_count(kal_int32 *sensor_count_ptr)
{
#ifdef GC2375_MIPI_RAW_DRIVER_TRACE
   SENSORDB("GC2375_MIPI_RAW_get_sensor_group_count\n");
#endif
  *sensor_count_ptr = GC2375_MIPI_RAW_GROUP_TOTAL_NUMS;
}

inline static void GC2375_MIPI_RAW_get_sensor_group_info(MSDK_SENSOR_GROUP_INFO_STRUCT *para)
{
#ifdef GC2375_MIPI_RAW_DRIVER_TRACE
   SENSORDB("GC2375_MIPI_RAW_get_sensor_group_info\n");
#endif
  switch (para->GroupIdx)
  {
  case GC2375_MIPI_RAW_PRE_GAIN:
    sprintf(para->GroupNamePtr, "CCT");
    para->ItemCount = 5;
    break;
  case GC2375_MIPI_RAW_CMMCLK_CURRENT:
    sprintf(para->GroupNamePtr, "CMMCLK Current");
    para->ItemCount = 1;
    break;
  case GC2375_MIPI_RAW_FRAME_RATE_LIMITATION:
    sprintf(para->GroupNamePtr, "Frame Rate Limitation");
    para->ItemCount = 2;
    break;
  case GC2375_MIPI_RAW_REGISTER_EDITOR:
    sprintf(para->GroupNamePtr, "Register Editor");
    para->ItemCount = 2;
    break;
  default:
    ASSERT(0);
  }
}

inline static void GC2375_MIPI_RAW_get_sensor_item_info(MSDK_SENSOR_ITEM_INFO_STRUCT *para)
{

     const static kal_char *cct_item_name[] = {"SENSOR_BASEGAIN", "Pregain-R", "Pregain-Gr", "Pregain-Gb", "Pregain-B"};
     const static kal_char *editer_item_name[] = {"REG addr", "REG value"};

#ifdef GC2375_MIPI_RAW_DRIVER_TRACE
     SENSORDB("GC2375_MIPI_RAW_get_sensor_item_info\n");
#endif
  switch (para->GroupIdx)
  {
  case GC2375_MIPI_RAW_PRE_GAIN:
    switch (para->ItemIdx)
    {
    case GC2375_MIPI_RAW_SENSOR_BASEGAIN:
    case GC2375_MIPI_RAW_PRE_GAIN_R_INDEX:
    case GC2375_MIPI_RAW_PRE_GAIN_Gr_INDEX:
    case GC2375_MIPI_RAW_PRE_GAIN_Gb_INDEX:
    case GC2375_MIPI_RAW_PRE_GAIN_B_INDEX:
      break;
    default:
      ASSERT(0);
    }
    sprintf(para->ItemNamePtr, cct_item_name[para->ItemIdx - GC2375_MIPI_RAW_SENSOR_BASEGAIN]);
    para->ItemValue = GC2375_MIPI_RAW_sensor.eng.cct[para->ItemIdx].Para * 1000 / BASEGAIN;
    para->IsTrueFalse = para->IsReadOnly = para->IsNeedRestart = KAL_FALSE;
    para->Min = GC2375_MIPI_RAW_MIN_ANALOG_GAIN * 1000;
    para->Max = GC2375_MIPI_RAW_MAX_ANALOG_GAIN * 1000;
    break;
  case GC2375_MIPI_RAW_CMMCLK_CURRENT:
    switch (para->ItemIdx)
    {
    case 0:
      sprintf(para->ItemNamePtr, "Drv Cur[2,4,6,8]mA");
      switch (GC2375_MIPI_RAW_sensor.eng.reg[GC2375_MIPI_RAW_CMMCLK_CURRENT_INDEX].Para)
      {
      case ISP_DRIVING_2MA:
        para->ItemValue = 2;
        break;
      case ISP_DRIVING_4MA:
        para->ItemValue = 4;
        break;
      case ISP_DRIVING_6MA:
        para->ItemValue = 6;
        break;
      case ISP_DRIVING_8MA:
        para->ItemValue = 8;
        break;
      default:
        ASSERT(0);
      }
      para->IsTrueFalse = para->IsReadOnly = KAL_FALSE;
      para->IsNeedRestart = KAL_TRUE;
      para->Min = 2;
      para->Max = 8;
      break;
    default:
      ASSERT(0);
    }
    break;
  case GC2375_MIPI_RAW_FRAME_RATE_LIMITATION:
    switch (para->ItemIdx)
    {
    case 0:
      sprintf(para->ItemNamePtr, "Max Exposure Lines");
      para->ItemValue = 5998;
      break;
    case 1:
      sprintf(para->ItemNamePtr, "Min Frame Rate");
      para->ItemValue = 5;
      break;
    default:
      ASSERT(0);
    }
    para->IsTrueFalse = para->IsNeedRestart = KAL_FALSE;
    para->IsReadOnly = KAL_TRUE;
    para->Min = para->Max = 0;
    break;
  case GC2375_MIPI_RAW_REGISTER_EDITOR:
    switch (para->ItemIdx)
    {
    case 0:
    case 1:
      sprintf(para->ItemNamePtr, editer_item_name[para->ItemIdx]);
      para->ItemValue = 0;
      para->IsTrueFalse = para->IsReadOnly = para->IsNeedRestart = KAL_FALSE;
      para->Min = 0;
      para->Max = (para->ItemIdx == 0 ? 0xFFFF : 0xFF);
      break;
    default:
      ASSERT(0);
    }
    break;
  default:
    ASSERT(0);
  }
}

inline static kal_bool GC2375_MIPI_RAW_set_sensor_item_info(MSDK_SENSOR_ITEM_INFO_STRUCT *para)
{
  kal_uint16 temp_para;
#ifdef GC2375_MIPI_RAW_DRIVER_TRACE
   SENSORDB("GC2375_MIPI_RAW_set_sensor_item_info\n");
#endif
  switch (para->GroupIdx)
  {
  case GC2375_MIPI_RAW_PRE_GAIN:
    switch (para->ItemIdx)
    {
    case GC2375_MIPI_RAW_SENSOR_BASEGAIN:
    case GC2375_MIPI_RAW_PRE_GAIN_R_INDEX:
    case GC2375_MIPI_RAW_PRE_GAIN_Gr_INDEX:
    case GC2375_MIPI_RAW_PRE_GAIN_Gb_INDEX:
    case GC2375_MIPI_RAW_PRE_GAIN_B_INDEX:
#if defined(MT6577)||defined(MT6589)
		spin_lock(&gc2375_mipi_raw_drv_lock);
#endif
      GC2375_MIPI_RAW_sensor.eng.cct[para->ItemIdx].Para = para->ItemValue * BASEGAIN / 1000;
#if defined(MT6577)||defined(MT6589)
	  spin_unlock(&gc2375_mipi_raw_drv_lock);
#endif
      GC2375_MIPI_RAW_SetGain(GC2375_MIPI_RAW_sensor.gain); /* update gain */
      break;
    default:
      ASSERT(0);
    }
    break;
  case GC2375_MIPI_RAW_CMMCLK_CURRENT:
    switch (para->ItemIdx)
    {
    case 0:
      switch (para->ItemValue)
      {
      case 2:
        temp_para = ISP_DRIVING_2MA;
        break;
      case 3:
      case 4:
        temp_para = ISP_DRIVING_4MA;
        break;
      case 5:
      case 6:
        temp_para = ISP_DRIVING_6MA;
        break;
      default:
        temp_para = ISP_DRIVING_8MA;
        break;
      }
      //GC2375_MIPI_RAW_set_isp_driving_current(temp_para);
#if defined(MT6577)||defined(MT6589)
      spin_lock(&gc2375_mipi_raw_drv_lock);
#endif
      GC2375_MIPI_RAW_sensor.eng.reg[GC2375_MIPI_RAW_CMMCLK_CURRENT_INDEX].Para = temp_para;
#if defined(MT6577)||defined(MT6589)
	  spin_unlock(&gc2375_mipi_raw_drv_lock);
#endif
      break;
    default:
      ASSERT(0);
    }
    break;
  case GC2375_MIPI_RAW_FRAME_RATE_LIMITATION:
    ASSERT(0);
    break;
  case GC2375_MIPI_RAW_REGISTER_EDITOR:
    switch (para->ItemIdx)
    {
      static kal_uint32 fac_sensor_reg;
    case 0:
      if (para->ItemValue < 0 || para->ItemValue > 0xFFFF) return KAL_FALSE;
      fac_sensor_reg = para->ItemValue;
      break;
    case 1:
      if (para->ItemValue < 0 || para->ItemValue > 0xFF) return KAL_FALSE;
      gc2375_mipi_raw_write_cmos_sensor(fac_sensor_reg, para->ItemValue);
      break;
    default:
      ASSERT(0);
    }
    break;
  default:
    ASSERT(0);
  }
  return KAL_TRUE;
}

void GC2375_MIPI_RAW_SetMirrorFlip(GC2375_MIPI_RAW_IMAGE_MIRROR Mirror)
{}

static void GC2375_MIPI_RAW_Sensor_Init(void)
{
	/*System*/
	gc2375_mipi_raw_write_cmos_sensor(0xfe,0x00);
	gc2375_mipi_raw_write_cmos_sensor(0xfe,0x00);
	gc2375_mipi_raw_write_cmos_sensor(0xfe,0x00);
	gc2375_mipi_raw_write_cmos_sensor(0xf7,0x01);
	gc2375_mipi_raw_write_cmos_sensor(0xf8,0x0c);
	gc2375_mipi_raw_write_cmos_sensor(0xf9,0x42);
	gc2375_mipi_raw_write_cmos_sensor(0xfa,0x88);
	gc2375_mipi_raw_write_cmos_sensor(0xfc,0x8e);
	gc2375_mipi_raw_write_cmos_sensor(0xfe,0x00);
	gc2375_mipi_raw_write_cmos_sensor(0x88,0x03);

	/*Analog*/
	gc2375_mipi_raw_write_cmos_sensor(0x03,0x04);
	gc2375_mipi_raw_write_cmos_sensor(0x04,0x65);
	gc2375_mipi_raw_write_cmos_sensor(0x05,0x02);
	gc2375_mipi_raw_write_cmos_sensor(0x06,0x5a);
	gc2375_mipi_raw_write_cmos_sensor(0x07,0x00);
	gc2375_mipi_raw_write_cmos_sensor(0x08,0x10);
	gc2375_mipi_raw_write_cmos_sensor(0x09,0x00);
	gc2375_mipi_raw_write_cmos_sensor(0x0a,0x08);
	gc2375_mipi_raw_write_cmos_sensor(0x0b,0x00);
	gc2375_mipi_raw_write_cmos_sensor(0x0c,0x18);
	gc2375_mipi_raw_write_cmos_sensor(0x0d,0x04);
	gc2375_mipi_raw_write_cmos_sensor(0x0e,0xb8);
	gc2375_mipi_raw_write_cmos_sensor(0x0f,0x06);
	gc2375_mipi_raw_write_cmos_sensor(0x10,0x48);
	gc2375_mipi_raw_write_cmos_sensor(0x17,MIRROR);
	gc2375_mipi_raw_write_cmos_sensor(0x1c,0x10);
	gc2375_mipi_raw_write_cmos_sensor(0x1d,0x13);
	gc2375_mipi_raw_write_cmos_sensor(0x20,0x0b);
	gc2375_mipi_raw_write_cmos_sensor(0x21,0x6d);
	gc2375_mipi_raw_write_cmos_sensor(0x22,0x0c);
	gc2375_mipi_raw_write_cmos_sensor(0x25,0xc1);
	gc2375_mipi_raw_write_cmos_sensor(0x26,0x0e);
	gc2375_mipi_raw_write_cmos_sensor(0x27,0x22);
	gc2375_mipi_raw_write_cmos_sensor(0x29,0x5f);
	gc2375_mipi_raw_write_cmos_sensor(0x2b,0x88);
	gc2375_mipi_raw_write_cmos_sensor(0x2f,0x12);
	gc2375_mipi_raw_write_cmos_sensor(0x38,0x86);
	gc2375_mipi_raw_write_cmos_sensor(0x3d,0x00);
	gc2375_mipi_raw_write_cmos_sensor(0xcd,0xa3);
	gc2375_mipi_raw_write_cmos_sensor(0xce,0x57);
	gc2375_mipi_raw_write_cmos_sensor(0xd0,0x09);
	gc2375_mipi_raw_write_cmos_sensor(0xd1,0xca);
	gc2375_mipi_raw_write_cmos_sensor(0xd2,0x34);
	gc2375_mipi_raw_write_cmos_sensor(0xd3,0xbb);
	gc2375_mipi_raw_write_cmos_sensor(0xd8,0x60);
	gc2375_mipi_raw_write_cmos_sensor(0xe0,0x08);
	gc2375_mipi_raw_write_cmos_sensor(0xe1,0x1f);
	gc2375_mipi_raw_write_cmos_sensor(0xe4,0xf8);
	gc2375_mipi_raw_write_cmos_sensor(0xe5,0x0c);
	gc2375_mipi_raw_write_cmos_sensor(0xe6,0x10);
	gc2375_mipi_raw_write_cmos_sensor(0xe7,0xcc);
	gc2375_mipi_raw_write_cmos_sensor(0xe8,0x02);
	gc2375_mipi_raw_write_cmos_sensor(0xe9,0x01);
	gc2375_mipi_raw_write_cmos_sensor(0xea,0x02);
	gc2375_mipi_raw_write_cmos_sensor(0xeb,0x01);

	/*Crop*/
	gc2375_mipi_raw_write_cmos_sensor(0x90,0x01);
	gc2375_mipi_raw_write_cmos_sensor(0x92,STARTY);
	gc2375_mipi_raw_write_cmos_sensor(0x94,STARTX);
	gc2375_mipi_raw_write_cmos_sensor(0x95,0x04);
	gc2375_mipi_raw_write_cmos_sensor(0x96,0xb0);
	gc2375_mipi_raw_write_cmos_sensor(0x97,0x06);
	gc2375_mipi_raw_write_cmos_sensor(0x98,0x40);

	/*BLK*/
	gc2375_mipi_raw_write_cmos_sensor(0x18,0x02);
	gc2375_mipi_raw_write_cmos_sensor(0x1a,0x18);
	gc2375_mipi_raw_write_cmos_sensor(0x28,0x00);
	gc2375_mipi_raw_write_cmos_sensor(0x3f,0x40);
	gc2375_mipi_raw_write_cmos_sensor(0x40,0x26);
	gc2375_mipi_raw_write_cmos_sensor(0x41,0x00);
	gc2375_mipi_raw_write_cmos_sensor(0x43,0x03);
	gc2375_mipi_raw_write_cmos_sensor(0x4a,0x00);
	gc2375_mipi_raw_write_cmos_sensor(0x4e,BLK_Select1_H);
	gc2375_mipi_raw_write_cmos_sensor(0x4f,BLK_Select1_L);
	gc2375_mipi_raw_write_cmos_sensor(0x66,BLK_Select2_H);
	gc2375_mipi_raw_write_cmos_sensor(0x67,BLK_Select2_L);

	/*Dark sun*/
	gc2375_mipi_raw_write_cmos_sensor(0x68,0x00);

	/*Gain*/
	gc2375_mipi_raw_write_cmos_sensor(0xb0,0x58);
	gc2375_mipi_raw_write_cmos_sensor(0xb1,0x01);
	gc2375_mipi_raw_write_cmos_sensor(0xb2,0x00);
	gc2375_mipi_raw_write_cmos_sensor(0xb6,0x00);

	/*MIPI*/
	gc2375_mipi_raw_write_cmos_sensor(0xef,0x90);
	gc2375_mipi_raw_write_cmos_sensor(0xfe,0x03);
	gc2375_mipi_raw_write_cmos_sensor(0x01,0x03);
	gc2375_mipi_raw_write_cmos_sensor(0x02,0x33);
	gc2375_mipi_raw_write_cmos_sensor(0x03,0x90);
	gc2375_mipi_raw_write_cmos_sensor(0x04,0x04);
	gc2375_mipi_raw_write_cmos_sensor(0x05,0x00);
	gc2375_mipi_raw_write_cmos_sensor(0x06,0x80);
	gc2375_mipi_raw_write_cmos_sensor(0x11,0x2b);
	gc2375_mipi_raw_write_cmos_sensor(0x12,0xd0);
	gc2375_mipi_raw_write_cmos_sensor(0x13,0x07);
	gc2375_mipi_raw_write_cmos_sensor(0x15,0x00);
	gc2375_mipi_raw_write_cmos_sensor(0x21,0x08);
	gc2375_mipi_raw_write_cmos_sensor(0x22,0x05);
	gc2375_mipi_raw_write_cmos_sensor(0x23,0x13);
	gc2375_mipi_raw_write_cmos_sensor(0x24,0x02);
	gc2375_mipi_raw_write_cmos_sensor(0x25,0x13);
	gc2375_mipi_raw_write_cmos_sensor(0x26,0x08);
	gc2375_mipi_raw_write_cmos_sensor(0x29,0x06);
	gc2375_mipi_raw_write_cmos_sensor(0x2a,0x08);
	gc2375_mipi_raw_write_cmos_sensor(0x2b,0x08);
	gc2375_mipi_raw_write_cmos_sensor(0xfe,0x00);

}   /*  GC2375_MIPI_RAW_Sensor_Init  */

/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	GC2375_MIPI_RAWOpen
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

UINT32 GC2375_MIPI_RAWOpen(void)
{
	kal_uint16 sensor_id=0;

	// check if sensor ID correct
	sensor_id=((gc2375_mipi_raw_read_cmos_sensor(0xf0) << 8) | gc2375_mipi_raw_read_cmos_sensor(0xf1));
#ifdef GC2375_MIPI_RAW_DRIVER_TRACE
	SENSORDB("GC2375_MIPI_RAWOpen, sensor_id:%x \n",sensor_id);
#endif
	if (sensor_id != GC2375_MIPI_RAW_SENSOR_ID)
	return ERROR_SENSOR_CONNECT_FAIL;

#ifdef GC2375_MIPI_USE_OTP
    gc2375_gcore_identify_otp();
#endif
	/* initail sequence write in  */
	GC2375_MIPI_RAW_Sensor_Init();

#ifdef GC2375_MIPI_USE_OTP
    gc2375_gcore_update_otp();
#endif
//	GC2375_MIPI_RAW_SetMirrorFlip(GC2375_MIPI_RAW_sensor.Mirror);

	return ERROR_NONE;
}   /* GC2375_MIPI_RAWOpen  */

/*************************************************************************
* FUNCTION
*   GC2375_MIPI_RAWGetSensorID
*
* DESCRIPTION
*   This function get the sensor ID
*
* PARAMETERS
*   *sensorID : return the sensor ID
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 GC2375_MIPI_RAWGetSensorID(UINT32 *sensorID)
{
	// check if sensor ID correct
	*sensorID=((gc2375_mipi_raw_read_cmos_sensor(0xf0) << 8) | gc2375_mipi_raw_read_cmos_sensor(0xf1));
#ifdef GC2375_MIPI_RAW_DRIVER_TRACE
	SENSORDB("GC2375_MIPI_RAWGetSensorID:%x \n",*sensorID);
#endif

	if (*sensorID != GC2375_MIPI_RAW_SENSOR_ID) {
		*sensorID = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
   return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	GC2375_MIPI_RAWClose
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 GC2375_MIPI_RAWClose(void)
{
#ifdef GC2375_MIPI_RAW_DRIVER_TRACE
   SENSORDB("GC2375_MIPI_RAWClose\n");
#endif
  //kdCISModulePowerOn(SensorIdx,currSensorName,0,mode_name);
//	DRV_I2CClose(GC2375_MIPI_RAWhDrvI2C);
	return ERROR_NONE;
}   /* GC2375_MIPI_RAWClose */

/*************************************************************************
* FUNCTION
* GC2375_MIPI_RAWPreview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 GC2375_MIPI_RAWPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

#if defined(MT6577)||defined(MT6589)
	spin_lock(&gc2375_mipi_raw_drv_lock);
#endif
	GC2375_MIPI_RAW_sensor.pv_mode = KAL_TRUE;
	//GC2375_MIPI_RAW_set_mirror(sensor_config_data->SensorImageMirror);
	switch (sensor_config_data->SensorOperationMode)
	{
	  case MSDK_SENSOR_OPERATION_MODE_VIDEO:
		GC2375_MIPI_RAW_sensor.video_mode = KAL_TRUE;
	  default: /* ISP_PREVIEW_MODE */
		GC2375_MIPI_RAW_sensor.video_mode = KAL_FALSE;
	}
	GC2375_MIPI_RAW_sensor.line_length = GC2375_MIPI_RAW_PV_PERIOD_PIXEL_NUMS;
	GC2375_MIPI_RAW_sensor.frame_height = GC2375_MIPI_RAW_PV_PERIOD_LINE_NUMS;
	GC2375_MIPI_RAW_sensor.pclk = GC2375_MIPI_RAW_PREVIEW_CLK;
#if defined(MT6577)||defined(MT6589)
	spin_unlock(&gc2375_mipi_raw_drv_lock);
#endif
	//GC2375_MIPI_RAW_Write_Shutter(GC2375_MIPI_RAW_sensor.shutter);
	return ERROR_NONE;
}   /*  GC2375_MIPI_RAWPreview   */

/*************************************************************************
* FUNCTION
*	GC2375_MIPI_RAWCapture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 GC2375_MIPI_RAWCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
//	kal_uint32 shutter = (kal_uint32)GC2375_MIPI_RAW_sensor.shutter;
#if defined(MT6577)||defined(MT6589)
	spin_lock(&gc2375_mipi_raw_drv_lock);
#endif
	GC2375_MIPI_RAW_sensor.video_mode = KAL_FALSE;
		GC2375_MIPI_RAW_sensor.pv_mode = KAL_FALSE;
#if defined(MT6577)||defined(MT6589)
		spin_unlock(&gc2375_mipi_raw_drv_lock);
#endif
	return ERROR_NONE;
}   /* GC2375_MIPI_RAW_Capture() */

UINT32 GC2375_MIPI_RAWGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth=GC2375_MIPI_RAW_IMAGE_SENSOR_FULL_WIDTH;
	pSensorResolution->SensorFullHeight=GC2375_MIPI_RAW_IMAGE_SENSOR_FULL_HEIGHT;
	pSensorResolution->SensorPreviewWidth=GC2375_MIPI_RAW_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight=GC2375_MIPI_RAW_IMAGE_SENSOR_PV_HEIGHT;
	pSensorResolution->SensorVideoWidth=GC2375_MIPI_RAW_IMAGE_SENSOR_VIDEO_WIDTH;
	pSensorResolution->SensorVideoHeight=GC2375_MIPI_RAW_IMAGE_SENSOR_VIDEO_HEIGHT;
	return ERROR_NONE;
}	/* GC2375_MIPI_RAWGetResolution() */

UINT32 GC2375_MIPI_RAWGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	pSensorInfo->SensorPreviewResolutionX=GC2375_MIPI_RAW_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY=GC2375_MIPI_RAW_IMAGE_SENSOR_PV_HEIGHT;
	pSensorInfo->SensorFullResolutionX=GC2375_MIPI_RAW_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY=GC2375_MIPI_RAW_IMAGE_SENSOR_FULL_HEIGHT;

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=TRUE; //low active
	pSensorInfo->SensorResetDelayCount=5;
	pSensorInfo->SensorOutputDataFormat=GC2375_MIPI_RAW_COLOR_FORMAT;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

	pSensorInfo->CaptureDelayFrame = 2;
	pSensorInfo->PreviewDelayFrame = 1;
	pSensorInfo->VideoDelayFrame = 1;

	pSensorInfo->SensorMasterClockSwitch = 0;
	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
	pSensorInfo->AEShutDelayFrame =0;		   /* The frame of setting shutter default 0 for TG int */
	pSensorInfo->AESensorGainDelayFrame = 0;   /* The frame of setting sensor gain */
	pSensorInfo->AEISPGainDelayFrame = 2;

	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
	pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;

	pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	pSensorInfo->SensorWidthSampling = 0;
	pSensorInfo->SensorHightSampling = 0;
	pSensorInfo->SensorPacketECCOrder = 1;
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = GC2375_MIPI_RAW_PV_X_START;
			pSensorInfo->SensorGrabStartY = GC2375_MIPI_RAW_PV_Y_START;

		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = GC2375_MIPI_RAW_VIDEO_X_START;
			pSensorInfo->SensorGrabStartY = GC2375_MIPI_RAW_VIDEO_Y_START;

		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount= 3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = GC2375_MIPI_RAW_FULL_X_START;
			pSensorInfo->SensorGrabStartY = GC2375_MIPI_RAW_FULL_Y_START;
		break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = GC2375_MIPI_RAW_PV_X_START;
			pSensorInfo->SensorGrabStartY = GC2375_MIPI_RAW_PV_Y_START;
		break;
	}
	memcpy(pSensorConfigData, &GC2375_MIPI_RAW_sensor.cfg_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
  return ERROR_NONE;
}	/* GC2375_MIPI_RAWGetInfo() */


UINT32 GC2375_MIPI_RAWControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

#ifdef GC2375_DRIVER_TRACE
	SENSORDB("GC2375_MIPI_RAWControl ScenarioId = %d \n",ScenarioId);
#endif
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			GC2375_MIPI_RAWPreview(pImageWindow, pSensorConfigData);
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			GC2375_MIPI_RAWCapture(pImageWindow, pSensorConfigData);
		break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* GC2375_MIPI_RAWControl() */


#if 0
UINT32 GC2375_MIPI_RAWSetVideoMode(UINT16 u2FrameRate)
{};
#endif
UINT32 GC2375_MIPI_RAWSetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("[GC2375SetTestPatternMode]test pattern bEnable:=%d\n",bEnable);
    if(bEnable)
    {
	gc2375_mipi_raw_write_cmos_sensor(0xfe, 0x00);
	gc2375_mipi_raw_write_cmos_sensor(0x8c, 0x11);
    }
    else
    {
	gc2375_mipi_raw_write_cmos_sensor(0xfe, 0x00);
	gc2375_mipi_raw_write_cmos_sensor(0x8c, 0x10);
    }
    return ERROR_NONE;
}

UINT32 GC2375_MIPI_RAWFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	//UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	//UINT32 GC2375_MIPI_RAWSensorRegNumber;
	//UINT32 i;
	//PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
	//MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	//MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
	//MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
	//MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=GC2375_MIPI_RAW_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=GC2375_MIPI_RAW_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:	/* 3 */
			*pFeatureReturnPara16++=GC2375_MIPI_RAW_sensor.line_length;
			*pFeatureReturnPara16=GC2375_MIPI_RAW_sensor.frame_height;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:  /* 3 */
			*pFeatureReturnPara32 = GC2375_MIPI_RAW_sensor.pclk;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:	/* 4 */
			GC2375_MIPI_RAW_set_shutter(*pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			GC2375_MIPI_RAW_night_mode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:	/* 6 */
			GC2375_MIPI_RAW_SetGain((UINT16) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			gc2375_mipi_raw_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = gc2375_mipi_raw_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
			memcpy(&GC2375_MIPI_RAW_sensor.eng.cct, pFeaturePara, sizeof(GC2375_MIPI_RAW_sensor.eng.cct));
			break;
		break;
		case SENSOR_FEATURE_GET_CCT_REGISTER:	/* 12 */
			if (*pFeatureParaLen >= sizeof(GC2375_MIPI_RAW_sensor.eng.cct) + sizeof(kal_uint32))
			{
			  *((kal_uint32 *)pFeaturePara++) = sizeof(GC2375_MIPI_RAW_sensor.eng.cct);
			  memcpy(pFeaturePara, &GC2375_MIPI_RAW_sensor.eng.cct, sizeof(GC2375_MIPI_RAW_sensor.eng.cct));
			}
			break;
		case SENSOR_FEATURE_SET_ENG_REGISTER:
			memcpy(&GC2375_MIPI_RAW_sensor.eng.reg, pFeaturePara, sizeof(GC2375_MIPI_RAW_sensor.eng.reg));
			break;
		case SENSOR_FEATURE_GET_ENG_REGISTER:	/* 14 */
			if (*pFeatureParaLen >= sizeof(GC2375_MIPI_RAW_sensor.eng.reg) + sizeof(kal_uint32))
			{
			  *((kal_uint32 *)pFeaturePara++) = sizeof(GC2375_MIPI_RAW_sensor.eng.reg);
			  memcpy(pFeaturePara, &GC2375_MIPI_RAW_sensor.eng.reg, sizeof(GC2375_MIPI_RAW_sensor.eng.reg));
			}
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
			((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->Version = NVRAM_CAMERA_SENSOR_FILE_VERSION;
			((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorId = GC2375_MIPI_RAW_SENSOR_ID;
			memcpy(((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorEngReg, &GC2375_MIPI_RAW_sensor.eng.reg, sizeof(GC2375_MIPI_RAW_sensor.eng.reg));
			memcpy(((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorCCTReg, &GC2375_MIPI_RAW_sensor.eng.cct, sizeof(GC2375_MIPI_RAW_sensor.eng.cct));
			*pFeatureParaLen = sizeof(NVRAM_SENSOR_DATA_STRUCT);
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pFeaturePara, &GC2375_MIPI_RAW_sensor.cfg_data, sizeof(GC2375_MIPI_RAW_sensor.cfg_data));
			*pFeatureParaLen = sizeof(GC2375_MIPI_RAW_sensor.cfg_data);
			break;
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		     GC2375_MIPI_RAW_camera_para_to_sensor();
			break;
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
			GC2375_MIPI_RAW_sensor_to_camera_para();
			break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
			GC2375_MIPI_RAW_get_sensor_group_count((kal_uint32 *)pFeaturePara);
			*pFeatureParaLen = 4;
			break;
		case SENSOR_FEATURE_GET_GROUP_INFO:
			GC2375_MIPI_RAW_get_sensor_group_info((MSDK_SENSOR_GROUP_INFO_STRUCT *)pFeaturePara);
			*pFeatureParaLen = sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_GET_ITEM_INFO:
			GC2375_MIPI_RAW_get_sensor_item_info((MSDK_SENSOR_ITEM_INFO_STRUCT *)pFeaturePara);
			*pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_SET_ITEM_INFO:
			GC2375_MIPI_RAW_set_sensor_item_info((MSDK_SENSOR_ITEM_INFO_STRUCT *)pFeaturePara);
			*pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_GET_ENG_INFO:
			memcpy(pFeaturePara, &GC2375_MIPI_RAW_sensor.eng_info, sizeof(GC2375_MIPI_RAW_sensor.eng_info));
			*pFeatureParaLen = sizeof(GC2375_MIPI_RAW_sensor.eng_info);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		   //GC2375_MIPI_RAWSetVideoMode(*pFeatureData16);
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			GC2375_MIPI_RAWGetSensorID(pFeatureReturnPara32);
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
		       GC2375_MIPI_RAWSetTestPatternMode((BOOL)*pFeatureData16);
		       break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		       *pFeatureReturnPara32=GC2375_MIPI_RAW_TEST_PATTERN_CHECKSUM;
		       *pFeatureParaLen=4;
		       break;
		default:
			break;
	}
	return ERROR_NONE;
}	/* GC2375_MIPI_RAWFeatureControl() */

SENSOR_FUNCTION_STRUCT	SensorFuncGC2375_MIPI_RAW=
{
	GC2375_MIPI_RAWOpen,
	GC2375_MIPI_RAWGetInfo,
	GC2375_MIPI_RAWGetResolution,
	GC2375_MIPI_RAWFeatureControl,
	GC2375_MIPI_RAWControl,
	GC2375_MIPI_RAWClose
};

UINT32 GC2375_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncGC2375_MIPI_RAW;

	return ERROR_NONE;
}	/* SensorInit() */
