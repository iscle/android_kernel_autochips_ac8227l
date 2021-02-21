/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
/*****************************************************************************

 ****************************************************************************/
 
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>


#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "sp2509_rearmipiraw_Sensor.h"
#include "sp2509_rearmipiraw_Camera_Sensor_para.h"
#include "sp2509_rearmipiraw_CameraCustomized.h"

kal_bool  SP2509_REARMIPI_MPEG4_encode_mode = KAL_FALSE;
kal_bool SP2509_REARMIPI_Auto_Flicker_mode = KAL_FALSE;
//static kal_bool ONLINE_DEBUG_BZW = KAL_TRUE;


kal_uint8 SP2509_REARMIPI_sensor_write_I2C_address = SP2509_REARMIPI_WRITE_ID;
kal_uint8 SP2509_REARMIPI_sensor_read_I2C_address = SP2509_REARMIPI_READ_ID;

//#define ONLINE_DEBUG //����adb����
#define DEBUG_SENSOR //T������
#ifdef DEBUG_SENSOR
	#define SP2509_REARMIPI_OP_CODE_INI		0x00		/* Initial value. */
	#define SP2509_REARMIPI_OP_CODE_REG		0x01		/* Register */
	#define SP2509_REARMIPI_OP_CODE_DLY		0x02		/* Delay */
	#define SP2509_REARMIPI_OP_CODE_END		0x03		/* End of initial setting. */
	

		typedef struct
	{
		u16 init_reg;
		u16 init_val;	/* Save the register value and delay tick */
		u8 op_code;		/* 0 - Initial value, 1 - Register, 2 - Delay, 3 - End of setting. */
	} SP2509_REARMIPI_initial_set_struct;

	SP2509_REARMIPI_initial_set_struct SP2509_REARMIPI_Init_Reg[1000];
	static UINT32 fromsd;//gpwdebug
	
 static kal_uint32 strtol(const char *nptr, u8 base)
{
	u8 ret;
	if(!nptr || (base!=16 && base!=10 && base!=8))
	{
		printk("%s(): NULL pointer input\n", __FUNCTION__);
		return -1;
	}
	for(ret=0; *nptr; nptr++)
	{
		if((base==16 && *nptr>='A' && *nptr<='F') || 
			(base==16 && *nptr>='a' && *nptr<='f') || 
			(base>=10 && *nptr>='0' && *nptr<='9') ||
			(base>=8 && *nptr>='0' && *nptr<='7') )
		{
			ret *= base;
			if(base==16 && *nptr>='A' && *nptr<='F')
				ret += *nptr-'A'+10;
			else if(base==16 && *nptr>='a' && *nptr<='f')
				ret += *nptr-'a'+10;
			else if(base>=10 && *nptr>='0' && *nptr<='9')
				ret += *nptr-'0';
			else if(base>=8 && *nptr>='0' && *nptr<='7')
				ret += *nptr-'0';
		}
		else
			return ret;
	}
	return ret;
}

 kal_uint8 SP2509_REARMIPI_Initialize_from_T_Flash(void)
{
	//FS_HANDLE fp = -1;				/* Default, no file opened. */
	//u8 *data_buff = NULL;
	u8 *curr_ptr = NULL;
	u32 file_size = 0;
	//u32 bytes_read = 0;
	u32 i = 0, j = 0;
	u8 func_ind[4] = {0};	/* REG or DLY */

	
	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static u8 data_buff[10*1024] ;
 
	fp = filp_open("/mnt/sdcard/sp2509_rear_sd", O_RDONLY , 0); 
	if (IS_ERR(fp)) 
	{ 
		printk("create file error\n"); 
		return -1; 
	} 
	fs = get_fs(); 
	set_fs(KERNEL_DS); 

	file_size = vfs_llseek(fp, 0, SEEK_END);
	vfs_read(fp, data_buff, file_size, &pos); 
	//printk("%s %d %d\n", buf,iFileLen,pos); 
	filp_close(fp, NULL); 
	set_fs(fs);
	
	/* Start parse the setting witch read from t-flash. */
	curr_ptr = data_buff;
	while (curr_ptr < (data_buff + file_size))
	{
		while ((*curr_ptr == ' ') || (*curr_ptr == '\t'))/* Skip the Space & TAB */
			curr_ptr++;				

		if (((*curr_ptr) == '/') && ((*(curr_ptr + 1)) == '*'))
		{
			while (!(((*curr_ptr) == '*') && ((*(curr_ptr + 1)) == '/')))
			{
				curr_ptr++;		/* Skip block comment code. */
			}

			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */
			
			continue ;
		}
		
		if (((*curr_ptr) == '/') || ((*curr_ptr) == '{') || ((*curr_ptr) == '}'))		/* Comment line, skip it. */
		{
			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}
		/* This just content one enter line. */
		if (((*curr_ptr) == 0x0D) && ((*(curr_ptr + 1)) == 0x0A))
		{
			curr_ptr += 2;
			continue ;
		}
		//printk(" curr_ptr1 = %s\n",curr_ptr);
		memcpy(func_ind, curr_ptr, 3);
	
						
		if (strcmp((const char *)func_ind, "REG") == 0)		/* REG */
		{
			curr_ptr += 6;				/* Skip "REG(0x" or "DLY(" */
			SP2509_REARMIPI_Init_Reg[i].op_code = SP2509_REARMIPI_OP_CODE_REG;
			
			SP2509_REARMIPI_Init_Reg[i].init_reg = strtol((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */
		
			SP2509_REARMIPI_Init_Reg[i].init_val = strtol((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */
		
		}
		else									/* DLY */
		{
			/* Need add delay for this setting. */
			curr_ptr += 4;	
			SP2509_REARMIPI_Init_Reg[i].op_code = SP2509_REARMIPI_OP_CODE_DLY;
			
			SP2509_REARMIPI_Init_Reg[i].init_reg = 0xFF;
			SP2509_REARMIPI_Init_Reg[i].init_val = strtol((const char *)curr_ptr,  10);	/* Get the delay ticks, the delay should less then 50 */
		}
		i++;
		

		/* Skip to next line directly. */
		while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
		{
			curr_ptr++;
		}
		curr_ptr += 2;
	}

	/* (0xFFFF, 0xFFFF) means the end of initial setting. */
	SP2509_REARMIPI_Init_Reg[i].op_code = SP2509_REARMIPI_OP_CODE_END;
	SP2509_REARMIPI_Init_Reg[i].init_reg = 0xFF;
	SP2509_REARMIPI_Init_Reg[i].init_val = 0xFF;
	i++;
	//for (j=0; j<i; j++)
		//printk(" %x  ==  %x\n",SP2509_REARMIPI_Init_Reg[j].init_reg, SP2509_REARMIPI_Init_Reg[j].init_val);

	/* Start apply the initial setting to sensor. */
	#if 1
	for (j=0; j<i; j++)
	{
		if (SP2509_REARMIPI_Init_Reg[j].op_code == SP2509_REARMIPI_OP_CODE_END)	/* End of the setting. */
		{
			break ;
		}
		else if (SP2509_REARMIPI_Init_Reg[j].op_code == SP2509_REARMIPI_OP_CODE_DLY)
		{
			msleep(SP2509_REARMIPI_Init_Reg[j].init_val);		/* Delay */
		}
		else if (SP2509_REARMIPI_Init_Reg[j].op_code == SP2509_REARMIPI_OP_CODE_REG)
		{
		
			SP2509_REARMIPI_write_cmos_sensor(SP2509_REARMIPI_Init_Reg[j].init_reg, SP2509_REARMIPI_Init_Reg[j].init_val);
			printk("%x = %x\n",SP2509_REARMIPI_Init_Reg[j].init_reg,SP2509_REARMIPI_Init_Reg[j].init_val);
		}
		else
		{
			printk("REG ERROR!\n");
		}
	}
#endif

	return 1;	
}

#endif

#ifdef ONLINE_DEBUG//for debug online
static u32 cur_reg=0;
static u8 cur_val;
static u32 debug_id = 0;

static ssize_t fcam_show(struct device *dev, struct device_attribute *attr, char *_buf)
{
	return sprintf(_buf, "0x%02x=0x%02x id = %x\n", cur_reg, cur_val,debug_id);
}
#ifndef DEBUG_SENSOR
static u32 strtol(const char *nptr, int base)
{
	u32 ret;
	if(!nptr || (base!=16 && base!=10 && base!=8))
	{
		printk("%s(): NULL pointer input\n", __FUNCTION__);
		return -1;
	}
	for(ret=0; *nptr; nptr++)
	{
		if((base==16 && *nptr>='A' && *nptr<='F') || 
			(base==16 && *nptr>='a' && *nptr<='f') || 
			(base>=10 && *nptr>='0' && *nptr<='9') ||
			(base>=8 && *nptr>='0' && *nptr<='7') )
		{
			ret *= base;
			if(base==16 && *nptr>='A' && *nptr<='F')
				ret += *nptr-'A'+10;
			else if(base==16 && *nptr>='a' && *nptr<='f')
				ret += *nptr-'a'+10;
			else if(base>=10 && *nptr>='0' && *nptr<='9')
				ret += *nptr-'0';
			else if(base>=8 && *nptr>='0' && *nptr<='7')
				ret += *nptr-'0';
		}
		else
			return ret;
	}
	return ret;
}
#endif

static ssize_t fcam_store(struct device *dev,
					struct device_attribute *attr,
					const char *_buf, size_t _count)
{
	const char * p=_buf;
	uint32_t reg;
	uint16_t val;
	uint16_t tmp;

	if(!strncmp(_buf, "get", strlen("get")))
	{
		p+=strlen("get");
		cur_reg=(u32)strtol(p, 16);
		
		//msm_camera_i2c_read(ov7695_s_ctrl.sensor_i2c_client, cur_reg, &val,MSM_CAMERA_I2C_BYTE_DATA);
		val=SP2509_REARMIPI_read_cmos_sensor(cur_reg);//hanlei
		
		printk("%s(): get 0x%02x=0x%02x\n", __FUNCTION__, cur_reg, val);
		cur_val=val;
	}
	else if(!strncmp(_buf, "put", strlen("put")))
	{
		p+=strlen("put");
		reg=strtol(p, 16);
	
		p=strchr(_buf, '=');
		if(p)
		{
			++ p;
			val=strtol(p, 16);
			
			//msm_camera_i2c_write(ov7695_s_ctrl.sensor_i2c_client, reg, val,MSM_CAMERA_I2C_BYTE_DATA);
			SP2509_REARMIPI_write_cmos_sensor(reg,val);//hanlei
			//msm_camera_i2c_read(ov7695_s_ctrl.sensor_i2c_client, reg, &tmp,MSM_CAMERA_I2C_BYTE_DATA);
			tmp=SP2509_REARMIPI_read_cmos_sensor(reg);
			
			printk("%s(): set 0x%02x=0x%02x==(0x%02x)\n", __FUNCTION__, reg, val,tmp);
		}
		else
			printk("%s(): Bad string format input!\n", __FUNCTION__);
	}
	else
		printk("%s(): Bad string format input!\n", __FUNCTION__);
	
	return _count;
} 

static ssize_t currreg_show(struct device *dev, struct device_attribute *attr, char *_buf)
{
    strcpy(_buf, "SP2509_REAR");
    return 4;
}

static struct device *fcam_dev = NULL;
static struct class *  fcam_class = NULL;
static DEVICE_ATTR(fcam, 0666, fcam_show, fcam_store);
static DEVICE_ATTR(currreg, 0666, currreg_show, NULL);
#endif



//#define SP2509_REARMIPI_USE_OTP

#ifdef SP2509_REARMIPI_USE_OTP
static uint16_t used_otp = 0;
static uint16_t ret = -1;
extern int sp2509_rear_update_otp_wb(void);
extern int sp2509_rear_update_awb_gain(void);
extern int sp2509_rear_check_mid(uint mid);
#endif
/*DTS2016121903793  by yangyale/ywx429347 begin */
static struct SP2509_REARMIPI_sensor_STRUCT SP2509_REARMIPI_sensor={SP2509_REARMIPI_WRITE_ID,SP2509_REARMIPI_READ_ID,KAL_TRUE,KAL_FALSE,KAL_TRUE,KAL_FALSE,
KAL_FALSE,KAL_FALSE,KAL_FALSE,40000000,40000000,40000000,1,1,1,10,10,10,SP2509_REARMIPI_PV_LINE_LENGTH_PIXELS,SP2509_REARMIPI_PV_FRAME_LENGTH_LINES,
SP2509_REARMIPI_VIDEO_LINE_LENGTH_PIXELS,SP2509_REARMIPI_VIDEO_FRAME_LENGTH_LINES,SP2509_REARMIPI_FULL_LINE_LENGTH_PIXELS,SP2509_REARMIPI_FULL_FRAME_LENGTH_LINES,0,1,0,1,0,1,30};
static MSDK_SCENARIO_ID_ENUM CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;	
/*DTS2016121903793  by yangyale/ywx429347 end */
kal_uint16	SP2509_REARMIPI_sensor_gain_base=0x10;//0x00//hanlei
/* MAX/MIN Explosure Lines Used By AE Algorithm */
kal_uint16 SP2509_REARMIPI_MAX_EXPOSURE_LINES = SP2509_REARMIPI_PV_FRAME_LENGTH_LINES-5;//3;//650;//5//hanlei
kal_uint8  SP2509_REARMIPI_MIN_EXPOSURE_LINES = 2;//1//2//hanlei
kal_uint32 SP2509_REARMIPI_isp_master_clock;
static DEFINE_SPINLOCK(sp2509_rear_drv_lock);

#define SENSORDB(fmt, arg...) printk( "[SP2509_REARMIPIRaw] "  fmt, ##arg)
#define RETAILMSG(x,...)
#define TEXT
UINT8 SP2509_REARMIPIPixelClockDivider=0;
kal_uint16 SP2509_REARMIPI_sensor_id=0;
MSDK_SENSOR_CONFIG_STRUCT SP2509_REARMIPISensorConfigData;
kal_uint32 SP2509_REARMIPI_FAC_SENSOR_REG;
kal_uint16 SP2509_REARMIPI_sensor_flip_value; 
/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT SP2509_REARMIPISensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT SP2509_REARMIPISensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
//#define SP2509_REARMIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, SP2509_REARMIPI_WRITE_ID)

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);


kal_uint16 SP2509_REARMIPI_read_cmos_sensor(kal_uint32 addr);

/*kal_uint16 SP2509_REARMIPI_read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,SP2509_REARMIPI_WRITE_ID);
    return get_byte;
}*/

//for test 2509
kal_uint16 SP2509_REARMIPI_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char puSendCmd[2] = { (char)(addr & 0xFF) ,(char)(para & 0xFF)};

	//SENSORDB("HCY IIC 8 8");  
	
	iWriteRegI2C(puSendCmd , 2,0x7a);
    SENSORDB("gpw write 0x%x  =%x \n" , addr,SP2509_REARMIPI_read_cmos_sensor(addr) );
	
	return TRUE;

}

kal_uint16 SP2509_REARMIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	char puSendCmd[1] = { (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 1, (u8*)&get_byte,1,0x7a);
	return get_byte;
}
//end for test 2509

void SP2509_REARMIPI_write_shutter(kal_uint16 shutter)
{
	SENSORDB("[SP2509_REARMIPI]enter SP2509_REARMIPI_write_shutter function\n"); 
//	unsigned long flags;	
#if 0
SP2509_REARMIPI_write_cmos_sensor(0xfd, 0x01);
SP2509_REARMIPI_write_cmos_sensor(0x03, (2098 >> 8) & 0xFF);
SP2509_REARMIPI_write_cmos_sensor(0x04, 2098  & 0xFF);

SP2509_REARMIPI_write_cmos_sensor(0x24,0xa0);

SP2509_REARMIPI_write_cmos_sensor(0x01, 0x01); 

#endif

	

	 //shutter=0x5300;



	 SP2509_REARMIPI_write_cmos_sensor(0xfd, 0x01);
	 SP2509_REARMIPI_write_cmos_sensor(0x03, (shutter >> 8) & 0xFF);
	 SP2509_REARMIPI_write_cmos_sensor(0x04, shutter  & 0xFF);
	 SP2509_REARMIPI_write_cmos_sensor(0x01, 0x01); 
		
	 
	
	 SENSORDB("hcy  SP2509_REARMIPI_write_shutter  =%d \n" , shutter );
}   /* write_SP2509_REARMIPI_shutter */

static kal_uint16 SP2509_REARMIPIReg2Gain(const kal_uint8 iReg)
{
	SENSORDB("[SP2509_REARMIPI]enter SP2509_REARMIPIReg2Gain function\n");

	SENSORDB("[SP2509_REARMIPI]exit SP2509_REARMIPIReg2Gain function\n");
	
       return 0;
}
static kal_uint8 SP2509_REARMIPIGain2Reg(const kal_uint16 iGain)
{
	SENSORDB("[SP2509_REARMIPI]enter SP2509_REARMIPIGain2Reg function\n");
	SENSORDB("[SP2509_REARMIPI]exit SP2509_REARMIPIGain2Reg function\n");
	
       return  0;
}

/*************************************************************************
* FUNCTION
*    SP2509_REARMIPI_SetGain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    gain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/

//test gain


void SP2509_REARMIPI_SetGain(UINT16 iGain)
{   
   

   	 kal_uint16 iReg;
	 
   	 SENSORDB("hcy  SP2509_REARMIPI_SetGain function=%d\n",iGain);  

	 if(iGain >= BASEGAIN && iGain <= 15*BASEGAIN)
   	 {   
   	    	 iReg = 0x10 * iGain/BASEGAIN ;        //change mtk gain base to aptina gain base
           //  iReg+= (iGain%BASEGAIN)/(0x10/BASEGAIN);

   	    	 if(iReg<=0x10)
   	    	 {
   	    	    	 SP2509_REARMIPI_write_cmos_sensor(0xfd, 0x01);
   	    	    	 SP2509_REARMIPI_write_cmos_sensor(0x24, 0x10);//0x23
   	    	    	 SP2509_REARMIPI_write_cmos_sensor(0x01, 0x01);
   	    	    	 SENSORDB("SP2509_REAR reg_gain =%d, SP2509_REARMIPI igain = %d", iReg, iGain);
   	    	 }
   	    	 else if(iReg>= 0x60)//gpw
   	    	 {
   	    	    	 SP2509_REARMIPI_write_cmos_sensor(0xfd, 0x01);
   	    	    	 SP2509_REARMIPI_write_cmos_sensor(0x24,0x60);
   	    	    	 SP2509_REARMIPI_write_cmos_sensor(0x01, 0x01);
   	    	    	 SENSORDB("SP2509_REARMIPI reg_gain =%d, SP2509_REARMIPI igain = %d", iReg, iGain);
	        }        	
   	    	 else
   	    	 {
   	    	    	 SP2509_REARMIPI_write_cmos_sensor(0xfd, 0x01);
   	    	    	 SP2509_REARMIPI_write_cmos_sensor(0x24, (kal_uint8)iReg);
   	    	    	 SP2509_REARMIPI_write_cmos_sensor(0x01, 0x01);
	       }	
   	 }	
   	 else
   	    	 SENSORDB("error gain setting");

}   /*  SP2509_REARMIPI_SetGain_SetGain  */


/*************************************************************************
* FUNCTION
*    read_SP2509_REARMIPI_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 read_SP2509_REARMIPI_gain(void)
{  
   	 SENSORDB("[SP2509_REARMIPI]enter read_SP2509_REARMIPI_gain function\n");
   	 SP2509_REARMIPI_write_cmos_sensor(0xfd, 0x01);
   	 return (kal_uint16)(SP2509_REARMIPI_read_cmos_sensor(0x24)) ;	//  N*4 //hanlei
   	 
}  /* read_SP2509_REARMIPI_gain */

void write_SP2509_REARMIPI_gain(kal_uint16 gain)
{
   	 SP2509_REARMIPI_SetGain(gain);
}


void SP2509_REARMIPI_camera_para_to_sensor(void)
{
   	 kal_uint32    i;
   	 SENSORDB("[SP2509_REARMIPI]enter SP2509_REARMIPI_camera_para_to_sensor function\n");
   	 for(i=0; 0xFFFFFFFF!=SP2509_REARMIPISensorReg[i].Addr; i++)
   	 {
   	    	 SP2509_REARMIPI_write_cmos_sensor(SP2509_REARMIPISensorReg[i].Addr, SP2509_REARMIPISensorReg[i].Para);
   	 }
   	 for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=SP2509_REARMIPISensorReg[i].Addr; i++)
   	 {
   	    	 SP2509_REARMIPI_write_cmos_sensor(SP2509_REARMIPISensorReg[i].Addr, SP2509_REARMIPISensorReg[i].Para);
   	 }
   	 for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
   	 {
   	    	 SP2509_REARMIPI_write_cmos_sensor(SP2509_REARMIPISensorCCT[i].Addr, SP2509_REARMIPISensorCCT[i].Para);
   	 }
	 
   	 SENSORDB("[SP2509_REARMIPI]exit SP2509_REARMIPI_camera_para_to_sensor function\n");
}


/*************************************************************************
* FUNCTION
*    SP2509_REARMIPI_sensor_to_camera_para
*
* DESCRIPTION
*    // update camera_para from sensor register
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
void SP2509_REARMIPI_sensor_to_camera_para(void)
{
   //	 SENSORDB("[SP2509_REARMIPI]enter SP2509_REARMIPI_sensor_to_camera_para function\n");
	 
   	 kal_uint16    i,temp_data;
	
   	 for(i=0; 0xFFFFFFFF!=SP2509_REARMIPISensorReg[i].Addr; i++)
   	 {
   	    	 temp_data=SP2509_REARMIPI_read_cmos_sensor(SP2509_REARMIPISensorReg[i].Addr);
   	    	 spin_lock(&sp2509_rear_drv_lock);
   	    	 SP2509_REARMIPISensorReg[i].Para = temp_data;
   	    	 spin_unlock(&sp2509_rear_drv_lock);
   	 }
   	 for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=SP2509_REARMIPISensorReg[i].Addr; i++)
   	 {
   	    	 temp_data=SP2509_REARMIPI_read_cmos_sensor(SP2509_REARMIPISensorReg[i].Addr);
   	    	 spin_lock(&sp2509_rear_drv_lock);
   	    	 SP2509_REARMIPISensorReg[i].Para = temp_data;
   	    	 spin_unlock(&sp2509_rear_drv_lock);
   	 }
	 
   	 SENSORDB("[SP2509_REARMIPI]exit SP2509_REARMIPI_sensor_to_camera_para function\n");
}

/*************************************************************************
* FUNCTION
*    SP2509_REARMIPI_get_sensor_group_count
*
* DESCRIPTION
*    //
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_int32  SP2509_REARMIPI_get_sensor_group_count(void)
{
   	 return GROUP_TOTAL_NUMS;
}

void SP2509_REARMIPI_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   	 switch (group_idx)
   	 {
   	    	 case PRE_GAIN:
   	    	    	 sprintf((char *)group_name_ptr, "CCT");
   	    	    	 *item_count_ptr = 2;
   	    	    	 break;
   	    	 case CMMCLK_CURRENT:
   	    	    	 sprintf((char *)group_name_ptr, "CMMCLK Current");
   	    	    	 *item_count_ptr = 1;
   	    	    	 break;
   	    	 case FRAME_RATE_LIMITATION:
   	    	    	 sprintf((char *)group_name_ptr, "Frame Rate Limitation");
   	    	    	 *item_count_ptr = 2;
   	    	    	 break;
   	    	 case REGISTER_EDITOR:
   	    	    	 sprintf((char *)group_name_ptr, "Register Editor");
   	    	    	 *item_count_ptr = 2;
   	    	    	 break;
   	    	 default:
   	    	    	 ASSERT(0);
   	 }
}

void SP2509_REARMIPI_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
   	 kal_int16 temp_reg=0;
   	 kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;
    
   	 switch (group_idx)
   	 {
   	     case PRE_GAIN:
   	    	   switch (item_idx)
   	    	   {
                 	  case 0:
                   	      sprintf((char *)info_ptr->ItemNamePtr,"Pregain-R");
                           temp_addr = PRE_GAIN_R_INDEX;
                 	    	break;
                 	  case 1:
                   	      sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gr");
                   	      temp_addr = PRE_GAIN_Gr_INDEX;
                   	      break;
                 	  case 2:
                   	      sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gb");
                   	      temp_addr = PRE_GAIN_Gb_INDEX;
                   	      break;
                 	  case 3:
                   	      sprintf((char *)info_ptr->ItemNamePtr,"Pregain-B");
                   	      temp_addr = PRE_GAIN_B_INDEX;
                   	      break;
                 	  case 4:
                   	      sprintf((char *)info_ptr->ItemNamePtr,"SENSOR_BASEGAIN");
                   	      temp_addr = SENSOR_BASEGAIN;
                   	      break;
                 	  default:
                   	      SENSORDB("[IMX105MIPI][Error]get_sensor_item_info error!!!\n");
   	    	   }
			   
   	    	   spin_lock(&sp2509_rear_drv_lock);    
   	    	   temp_para=SP2509_REARMIPISensorCCT[temp_addr].Para;	
   	    	   spin_unlock(&sp2509_rear_drv_lock);
   	    	   temp_gain = SP2509_REARMIPIReg2Gain(temp_para);
   	    	   temp_gain=(temp_gain*1000)/BASEGAIN; //hanlei?
   	    	   info_ptr->ItemValue=temp_gain;
   	    	   info_ptr->IsTrueFalse=KAL_FALSE;
   	    	   info_ptr->IsReadOnly=KAL_FALSE;
   	    	   info_ptr->IsNeedRestart=KAL_FALSE;
   	    	   info_ptr->Min=1000;//hanlei ?
   	    	   info_ptr->Max=15875;
   	    	   break;
   	     case CMMCLK_CURRENT:
   	    	   switch (item_idx)
   	    	   {
                 	  case 0:
                   	      sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");
                
                   	      //temp_reg=SP2509_REARMIPISensorReg[CMMCLK_CURRENT_INDEX].Para;
                   	      temp_reg = ISP_DRIVING_2MA;
                   	      if(temp_reg==ISP_DRIVING_2MA)
                   	      {
                   	      		info_ptr->ItemValue=2;
                   	      }
                   	      else if(temp_reg==ISP_DRIVING_4MA)
                   	      {
                   	      		info_ptr->ItemValue=4;
                   	      }
                   	      else if(temp_reg==ISP_DRIVING_6MA)
                   	      {
                   	      		info_ptr->ItemValue=6;
                   	      }
                   	      else if(temp_reg==ISP_DRIVING_8MA)
                   	      {
                   	      		info_ptr->ItemValue=8;
                   	      }
                
                   	      info_ptr->IsTrueFalse=KAL_FALSE;
                   	      info_ptr->IsReadOnly=KAL_FALSE;
                   	      info_ptr->IsNeedRestart=KAL_TRUE;
                   	      info_ptr->Min=2;
                   	      info_ptr->Max=8;
                   	      break;
                 	  default:
                   	      ASSERT(0);
   	    	   }
   	    	   break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue=SP2509_REARMIPI_MAX_EXPOSURE_LINES;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue=12;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}
kal_bool SP2509_REARMIPI_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
   kal_uint16 temp_addr=0, temp_para=0;

   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
              case 0:
                temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 SENSORDB("[IMX105MIPI][Error]set_sensor_item_info error!!!\n");
          }
            temp_para = SP2509_REARMIPIGain2Reg(ItemValue);
            spin_lock(&sp2509_rear_drv_lock);    
            SP2509_REARMIPISensorCCT[temp_addr].Para = temp_para;
			spin_unlock(&sp2509_rear_drv_lock);
		

		SP2509_REARMIPI_write_cmos_sensor(0xfd, 0x01);

		SP2509_REARMIPI_write_cmos_sensor(SP2509_REARMIPISensorCCT[temp_addr].Addr,temp_para);//hanlei?
		
			temp_para=read_SP2509_REARMIPI_gain();	
            spin_lock(&sp2509_rear_drv_lock);    
            SP2509_REARMIPI_sensor_gain_base=temp_para;
			spin_unlock(&sp2509_rear_drv_lock);

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    if(ItemValue==2)
                    {			
                        spin_lock(&sp2509_rear_drv_lock);    
                        SP2509_REARMIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_2MA;
                        spin_unlock(&sp2509_rear_drv_lock);
                        //SP2509_REARMIPI_set_isp_driving_current(ISP_DRIVING_2MA);
                    }
                    else if(ItemValue==3 || ItemValue==4)
                    {
                        spin_lock(&sp2509_rear_drv_lock);    
                        SP2509_REARMIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_4MA;
                        spin_unlock(&sp2509_rear_drv_lock);
                        //SP2509_REARMIPI_set_isp_driving_current(ISP_DRIVING_4MA);
                    }
                    else if(ItemValue==5 || ItemValue==6)
                    {
                        spin_lock(&sp2509_rear_drv_lock);    
                        SP2509_REARMIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_6MA;
                        spin_unlock(&sp2509_rear_drv_lock);
                        //SP2509_REARMIPI_set_isp_driving_current(ISP_DRIVING_6MA);
                    }
                    else
                    {
                    	    spin_lock(&sp2509_rear_drv_lock);    
                        SP2509_REARMIPISensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_8MA;
                    	    spin_unlock(&sp2509_rear_drv_lock);
                        //SP2509_REARMIPI_set_isp_driving_current(ISP_DRIVING_8MA);
                    }
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            ASSERT(0);
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
					spin_lock(&sp2509_rear_drv_lock);    
                    SP2509_REARMIPI_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&sp2509_rear_drv_lock);
                    break;
                case 1:
                    SP2509_REARMIPI_write_cmos_sensor(SP2509_REARMIPI_FAC_SENSOR_REG,ItemValue);
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

static void SP2509_REARMIPI_SetDummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
    SENSORDB("[SP2509_REARMIPI]enter SP2509_REARMIPI_SetDummy function\n");
    SP2509_REARMIPI_write_cmos_sensor(0xfd, 0x01);
	  SP2509_REARMIPI_write_cmos_sensor(0x05, (SP2509_REARMIPI_sensor.pv_dummy_lines >> 8) & 0xFF);
	  SP2509_REARMIPI_write_cmos_sensor(0x06, SP2509_REARMIPI_sensor.pv_dummy_lines & 0xFF);
	  SP2509_REARMIPI_write_cmos_sensor(0x01, 0x01);
  

}   /*  SP2509_REARMIPI_SetDummy */

static void SP2509_REARMIPI_Sensor_Init(void)
{
   #ifdef DEBUG_SENSOR
	if(fromsd == 1)//�Ƿ��SD��ȡ//gepeiwei   120903
		SP2509_REARMIPI_Initialize_from_T_Flash();//��SD����ȡ����Ҫ����
       else
   #endif
 {
 /*DTS2016121903793  by yangyale/ywx429347 begin */
  SP2509_REARMIPI_write_cmos_sensor(0xfd,0x00);
  SP2509_REARMIPI_write_cmos_sensor(0x2f,0x46);//80m  lixl
  SP2509_REARMIPI_write_cmos_sensor(0x34,0x00);
  SP2509_REARMIPI_write_cmos_sensor(0x35,0x21);//wu
  SP2509_REARMIPI_write_cmos_sensor(0x30,0x1d);
  SP2509_REARMIPI_write_cmos_sensor(0x33,0x05);//09 20151127 gh
  SP2509_REARMIPI_write_cmos_sensor(0xfd,0x01);
  SP2509_REARMIPI_write_cmos_sensor(0x44,0x00);
  SP2509_REARMIPI_write_cmos_sensor(0x2a,0x4c);//9.29canshu 7c
  SP2509_REARMIPI_write_cmos_sensor(0x2b,0x1e);
  SP2509_REARMIPI_write_cmos_sensor(0x2c,0x60);
  SP2509_REARMIPI_write_cmos_sensor(0x25,0x11);
  SP2509_REARMIPI_write_cmos_sensor(0x03,0x04);
  SP2509_REARMIPI_write_cmos_sensor(0x04,0x74);
  SP2509_REARMIPI_write_cmos_sensor(0x09,0x00);
  SP2509_REARMIPI_write_cmos_sensor(0x0a,0x6a);//8d  zhanghua
  SP2509_REARMIPI_write_cmos_sensor(0x06,0x0a);
  SP2509_REARMIPI_write_cmos_sensor(0x24,0x60);
  SP2509_REARMIPI_write_cmos_sensor(0x3f,0x03);
  SP2509_REARMIPI_write_cmos_sensor(0x01,0x01);
  SP2509_REARMIPI_write_cmos_sensor(0xfb,0x73);//9.29canshu 63
  SP2509_REARMIPI_write_cmos_sensor(0xfd,0x01);  
  SP2509_REARMIPI_write_cmos_sensor(0x16,0x04);
  SP2509_REARMIPI_write_cmos_sensor(0x1c,0x09);
  SP2509_REARMIPI_write_cmos_sensor(0x21,0x46);
  SP2509_REARMIPI_write_cmos_sensor(0x6c,0x00);
  SP2509_REARMIPI_write_cmos_sensor(0x6b,0x00);
  SP2509_REARMIPI_write_cmos_sensor(0x84,0x00);
  SP2509_REARMIPI_write_cmos_sensor(0x85,0x10);
  SP2509_REARMIPI_write_cmos_sensor(0x86,0x10);
  SP2509_REARMIPI_write_cmos_sensor(0x12,0x04);
  SP2509_REARMIPI_write_cmos_sensor(0x13,0x40);
  SP2509_REARMIPI_write_cmos_sensor(0x11,0x20);
  SP2509_REARMIPI_write_cmos_sensor(0x33,0x40);
  SP2509_REARMIPI_write_cmos_sensor(0xd0,0x03);
  SP2509_REARMIPI_write_cmos_sensor(0xd1,0x01);
  SP2509_REARMIPI_write_cmos_sensor(0xd2,0x00);
  SP2509_REARMIPI_write_cmos_sensor(0xd3,0x01);
  SP2509_REARMIPI_write_cmos_sensor(0xd4,0x20);
  SP2509_REARMIPI_write_cmos_sensor(0x51,0x16);//14;lixl
  SP2509_REARMIPI_write_cmos_sensor(0x52,0x11);//11;lixl
  SP2509_REARMIPI_write_cmos_sensor(0x55,0x35);//30;lixl
  SP2509_REARMIPI_write_cmos_sensor(0x58,0x11);//10;lixl
  SP2509_REARMIPI_write_cmos_sensor(0x71,0x11);//10;lixl
  SP2509_REARMIPI_write_cmos_sensor(0x6f,0x47);//40;lixl
  SP2509_REARMIPI_write_cmos_sensor(0x75,0x6a);//60;lixl
  SP2509_REARMIPI_write_cmos_sensor(0x76,0x11);//10;lixl
  SP2509_REARMIPI_write_cmos_sensor(0x8a,0x22);
  SP2509_REARMIPI_write_cmos_sensor(0x8b,0x22);
  SP2509_REARMIPI_write_cmos_sensor(0x19,0x71);
  SP2509_REARMIPI_write_cmos_sensor(0x29,0x01);
  SP2509_REARMIPI_write_cmos_sensor(0xfd,0x01);
  /*DTS2017011004761  by yangyale/ywx429347 begin */
  SP2509_REARMIPI_write_cmos_sensor(0x9d,0x96);//96
  /*DTS2017011004761  by yangyale/ywx429347 end */
  SP2509_REARMIPI_write_cmos_sensor(0xa0,0x29);//05
  SP2509_REARMIPI_write_cmos_sensor(0xa1,0x04);//
  SP2509_REARMIPI_write_cmos_sensor(0xad,0x62);//
  SP2509_REARMIPI_write_cmos_sensor(0xae,0x00);//
  SP2509_REARMIPI_write_cmos_sensor(0xaf,0x85);//
  SP2509_REARMIPI_write_cmos_sensor(0xb1,0x01);
  SP2509_REARMIPI_write_cmos_sensor(0xac,0x01);
  SP2509_REARMIPI_write_cmos_sensor(0xfd,0x01);
  SP2509_REARMIPI_write_cmos_sensor(0xfc,0x10); // 9.29wu
  SP2509_REARMIPI_write_cmos_sensor(0xfe,0x10); // 9.29wu 
  SP2509_REARMIPI_write_cmos_sensor(0xf9,0x00); // 9.29wu
  SP2509_REARMIPI_write_cmos_sensor(0xfa,0x00); // 9.29wu
  /*DTS2016121903793  by yangyale/ywx429347 end */
  #if 0
  SP2509_REARMIPI_write_cmos_sensor(0x8e,0x06);
  SP2509_REARMIPI_write_cmos_sensor(0x8f,0x40);
  SP2509_REARMIPI_write_cmos_sensor(0x90,0x04);
  SP2509_REARMIPI_write_cmos_sensor(0x91,0xb0);
  SP2509_REARMIPI_write_cmos_sensor(0x45,0x01);
  SP2509_REARMIPI_write_cmos_sensor(0x46,0x00);
  SP2509_REARMIPI_write_cmos_sensor(0x47,0x6c);
  SP2509_REARMIPI_write_cmos_sensor(0x48,0x03);
  SP2509_REARMIPI_write_cmos_sensor(0x49,0x8b);
  SP2509_REARMIPI_write_cmos_sensor(0x4a,0x00);
  SP2509_REARMIPI_write_cmos_sensor(0x4b,0x07);
  SP2509_REARMIPI_write_cmos_sensor(0x4c,0x04);
  SP2509_REARMIPI_write_cmos_sensor(0x4d,0xb7);
  #else
  SP2509_REARMIPI_write_cmos_sensor(0x8e,0x06);
  SP2509_REARMIPI_write_cmos_sensor(0x8f,0x50);
  SP2509_REARMIPI_write_cmos_sensor(0x90,0x04);
  SP2509_REARMIPI_write_cmos_sensor(0x91,0xc0); 
  SP2509_REARMIPI_write_cmos_sensor(0x3f,0x00);
  SP2509_REARMIPI_write_cmos_sensor(0xac,0x01);
  #endif

}
   // The register only need to enable 1 time.    
   spin_lock(&sp2509_rear_drv_lock);  
   SP2509_REARMIPI_Auto_Flicker_mode = KAL_FALSE;	  // reset the flicker status	 
   spin_unlock(&sp2509_rear_drv_lock);
   SENSORDB("[SP2509_REARMIPI]exit SP2509_REARMIPI_Sensor_Init function\n");

}   /*  SP2509_REARMIPI_Sensor_Init  */

static void  VideoFullSizeSetting(void)//16:9   6M
{
	SENSORDB("[SP2509_REARMIPI]enter VideoFullSizeSetting function\n");

       #ifdef SP2509_REARMIPI_USE_OTP
       if(ret == 0)
       {
	    SENSORDB("[SP2509_REARMIPI_USE_OTP]VideoFullSizeSetting function,sp2509_rear_update_awb_gain\n");
	    sp2509_rear_update_awb_gain();
       }
       #endif
       SENSORDB("[SP2509_REARMIPI]exit VideoFullSizeSetting function\n");
       return;
}
static void  PreviewSetting(void)
{
	//preview setting 

	//1296x972

	/*SP2509_REARMIPI_write_cmos_sensor(0xfd,0x06);
	SP2509_REARMIPI_write_cmos_sensor(0x40,0x06);
	SP2509_REARMIPI_write_cmos_sensor(0x42,0x06);//b
	SP2509_REARMIPI_write_cmos_sensor(0x46,0x00);
	SP2509_REARMIPI_write_cmos_sensor(0x48,0x00);//r
	SP2509_REARMIPI_write_cmos_sensor(0x4c,0x06);
	SP2509_REARMIPI_write_cmos_sensor(0x4e,0x06);//gb
	SP2509_REARMIPI_write_cmos_sensor(0x52,0x04);
	SP2509_REARMIPI_write_cmos_sensor(0x54,0x04);//gr*/
	
	return ;
}

static void SP2509_REARMIPI_set_5M(void)
{	//77 capture setting
	SENSORDB("[SP2509_REARMIPI]exit SP2509_REARMIPI_set_8M function\n"); 
	//capture setting
 
	//2592x1944
return ;

}
/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*   SP2509_REARMIPIOpen
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

UINT32 SP2509_REARMIPIOpen(void)
{
  //  SENSORDB("[hcy SP2509_REARMIPI]enter SP2509_REARMIPIOpen function\n");
	
  //  kal_uint32 rc = 0;
	
    #ifdef SP2509_REARMIPI_USE_OTP
    if(0 == used_otp){
	printk("[SP2509_REARMIPI_USE_OTP] before update otp wb...........................................\n");
	printk("[SP2509_REARMIPI_USE_OTP] before update otp wb...........................................\n");
	printk("[SP2509_REARMIPI_USE_OTP] before update otp wb...........................................\n");
	ret = sp2509_rear_update_otp_wb();

	used_otp =1;
	printk("[SP2509_REARMIPI_USE_OTP] after update otp wb............................................\n");
	printk("[SP2509_REARMIPI_USE_OTP] after update otp wb............................................\n");
	printk("[SP2509_REARMIPI_USE_OTP] after update otp wb............................................\n");
    }
    #endif
	
    int  retry = 0; 
    kal_uint16 sensorid;
 //   kal_uint16 sensorIDH;
   // kal_uint16 sensorIDL;
    //kal_uint16 sensorID;
    // check if sensor ID correct
    retry = 3; 

    do {
        
       	SP2509_REARMIPI_write_cmos_sensor(0xfd,0x00); 
	   sensorid = (SP2509_REARMIPI_read_cmos_sensor(0x02) << 8) | SP2509_REARMIPI_read_cmos_sensor(0x03) ;//0x02
       sensorid +=1;
	   SENSORDB("gpw sensorIDL =  0x%04x\n", sensorid);
	   spin_lock(&sp2509_rear_drv_lock);    
	   SP2509_REARMIPI_sensor_id =sensorid;
	   spin_unlock(&sp2509_rear_drv_lock);
		if (SP2509_REARMIPI_sensor_id == SP2509_REARMIPI_SENSOR_ID)
			break; 
		retry--; 

		
	    }
	while (retry > 0);
    SENSORDB("Read Sensor ID = 0x%04x\n", SP2509_REARMIPI_sensor_id);
	//SP2509_REARMIPI_sensor_id = SP2509_REARMIPI_SENSOR_ID;
	
    if (SP2509_REARMIPI_sensor_id != SP2509_REARMIPI_SENSOR_ID)
       return ERROR_SENSOR_CONNECT_FAIL;

#if  0  //gepeiwei   120903
					//�ж��ֻ���ӦĿ¼���Ƿ�����Ϊsp2509_rear_sd ���ļ�,û��Ĭ�ϲ���

					//���ڸ���ԭ�򣬱��汾��ʼ��������_s_fmt�С�
 struct file *fp; 
    mm_segment_t fs; 
    loff_t pos = 0; 
	static char buf[10*1024] ;
 
    fp = filp_open("/mnt/sdcard/sp2509_rear_sd", O_RDONLY , 0); 
    if (IS_ERR(fp)) { 
		fromsd = 0;   
		printk("gpw open file error\n");
		//return 0;
    } 
	
	else 
		{
		fromsd = 1;
	//SP2509_REARMIPI_Initialize_from_T_Flash();
	printk("gpw read ok!\n");

	filp_close(fp, NULL); 
    set_fs(fs);
		}
#endif
    
//    SP2509_REARMIPI_write_cmos_sensor(0xfd,0x01);
 //  SP2509_REARMIPI_write_cmos_sensor(0xfd,0x01);
  //	SP2509_REARMIPI_write_cmos_sensor(0xfd,0x00); 
	//   sensorid = SP2509_REARMIPI_read_cmos_sensor(0x01) ;

	//   SENSORDB("gpw2 sensorIDL =  0x%04x\n", sensorid);

	
    SP2509_REARMIPI_Sensor_Init();

	sensorid=read_SP2509_REARMIPI_gain();
	spin_lock(&sp2509_rear_drv_lock);	
    SP2509_REARMIPI_sensor_gain_base = sensorid;
	spin_unlock(&sp2509_rear_drv_lock);
	SENSORDB("[SP2509_REARMIPI]exit SP2509_REARMIPIOpen function\n");
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   SP2509_REARMIPIGetSensorID
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
UINT32 SP2509_REARMIPIGetSensorID(UINT32 *sensorID) 
{
    int  retry = 3; 
	SENSORDB("[SP2509_REARMIPI]enter SP2509_REARMIPIGetSensorID function\n");

  
 

//test 2509 sensor id for iic
    do {
	//int  retry = 3; 
        
        SP2509_REARMIPI_write_cmos_sensor(0xfd,0x00); 
	   *sensorID = (SP2509_REARMIPI_read_cmos_sensor(0x02) << 8) | SP2509_REARMIPI_read_cmos_sensor(0x03) ;//0x02
       
	  // *sensorID = SP2509_REARMIPI_SENSOR_ID;
	  *sensorID +=1;
	   SENSORDB("gpw1 Read Sensor ID Fail = 0x%04x\n", *sensorID); 
        if (*sensorID == SP2509_REARMIPI_SENSOR_ID)
            break;
		 msleep(20);
       // SENSORDB("hcy Read Sensor ID Fail = 0x%04x  reg0x04=%d\n", *sensorID,SP2509_REARMIPI_read_cmos_sensor(0x03)); 
        retry--; 
		
    } while (retry > 0);

//end test
#ifdef ONLINE_DEBUG
debug_id = *sensorID;
#endif


    if (*sensorID != SP2509_REARMIPI_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }
 
    
//   *sensorID = SP2509_REARMIPI_SENSOR_ID;
        SENSORDB("gpw2 Read Sensor ID   = 0x%04x\n", *sensorID); 
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   SP2509_REARMIPI_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of SP2509_REARMIPI to change exposure time.
*
* PARAMETERS
*   shutter : exposured lines
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void SP2509_REARMIPI_SetShutter(kal_uint16 iShutter)
{unsigned long flags;
	SENSORDB("GuoJinHui[SP2509_REARMIPI_SetShutter]%s():shutter=%d\n",__FUNCTION__,iShutter);
       if (iShutter < 13)
          iShutter = 13;
	else if(iShutter > 0xffff)
	   iShutter = 0xffff;
//	unsigned long flags;
	spin_lock_irqsave(&sp2509_rear_drv_lock,flags);
       SP2509_REARMIPI_sensor.pv_shutter = iShutter;	
	spin_unlock_irqrestore(&sp2509_rear_drv_lock,flags);
       SP2509_REARMIPI_write_shutter(iShutter);
	SENSORDB("[SP2509_REARMIPI]exit SP2509_REARMIPIGetSensorID function\n");
}   /*  SP2509_REARMIPI_SetShutter   */



/*************************************************************************
* FUNCTION
*   SP2509_REARMIPI_read_shutter
*
* DESCRIPTION
*   This function to  Get exposure time.
*
* PARAMETERS
*   None
*
* RETURNS
*   shutter : exposured lines
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT16 SP2509_REARMIPI_read_shutter(void)
{

    SP2509_REARMIPI_write_cmos_sensor(0xfd, 0x01);

    return (UINT16)( (SP2509_REARMIPI_read_cmos_sensor(0x03)<<8) | SP2509_REARMIPI_read_cmos_sensor(0x04) );
}

/*************************************************************************
* FUNCTION
*   SP2509_REARMIPI_night_mode
*
* DESCRIPTION
*   This function night mode of SP2509_REARMIPI.
*
* PARAMETERS
*   none
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void SP2509_REARMIPI_NightMode(kal_bool bEnable)
{
	SENSORDB("[SP2509_REARMIPI]enter SP2509_REARMIPI_NightMode function\n");
#if 0
    /************************************************************************/
    /*                      Auto Mode: 30fps                                                                                          */
    /*                      Night Mode:15fps                                                                                          */
    /************************************************************************/
    if(bEnable)
    {
        if(OV5642_MPEG4_encode_mode==KAL_TRUE)
        {
            OV5642_MAX_EXPOSURE_LINES = (kal_uint16)((OV5642_sensor_pclk/15)/(OV5642_PV_PERIOD_PIXEL_NUMS+OV5642_PV_dummy_pixels));
            OV5642_SP2509_REARMIPI_write_cmos_sensor(0x350C, (OV5642_MAX_EXPOSURE_LINES >> 8) & 0xFF);
            OV5642_SP2509_REARMIPI_write_cmos_sensor(0x350D, OV5642_MAX_EXPOSURE_LINES & 0xFF);
            OV5642_CURRENT_FRAME_LINES = OV5642_MAX_EXPOSURE_LINES;
            OV5642_MAX_EXPOSURE_LINES = OV5642_CURRENT_FRAME_LINES - OV5642_SHUTTER_LINES_GAP;
        }
    }
    else// Fix video framerate 30 fps
    {
        if(OV5642_MPEG4_encode_mode==KAL_TRUE)
        {
            OV5642_MAX_EXPOSURE_LINES = (kal_uint16)((OV5642_sensor_pclk/30)/(OV5642_PV_PERIOD_PIXEL_NUMS+OV5642_PV_dummy_pixels));
            if(OV5642_pv_exposure_lines < (OV5642_MAX_EXPOSURE_LINES - OV5642_SHUTTER_LINES_GAP)) // for avoid the shutter > frame_lines,move the frame lines setting to shutter function
            {
                OV5642_SP2509_REARMIPI_write_cmos_sensor(0x350C, (OV5642_MAX_EXPOSURE_LINES >> 8) & 0xFF);
                OV5642_SP2509_REARMIPI_write_cmos_sensor(0x350D, OV5642_MAX_EXPOSURE_LINES & 0xFF);
                OV5642_CURRENT_FRAME_LINES = OV5642_MAX_EXPOSURE_LINES;
            }
            OV5642_MAX_EXPOSURE_LINES = OV5642_MAX_EXPOSURE_LINES - OV5642_SHUTTER_LINES_GAP;
        }
    }
	
#endif	
	SENSORDB("[SP2509_REARMIPI]exit SP2509_REARMIPI_NightMode function\n");
}/*	SP2509_REARMIPI_NightMode */



/*************************************************************************
* FUNCTION
*   SP2509_REARMIPIClose
*
* DESCRIPTION
*   This function is to turn off sensor module power.
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 SP2509_REARMIPIClose(void)
{
    //SP2509_REARMIPI_write_cmos_sensor(0x0100,0x00);
    return ERROR_NONE;
}	/* SP2509_REARMIPIClose() */

void SP2509_REARMIPISetFlipMirror(kal_int32 imgMirror)
{
    kal_uint8  iTemp; 
	return;
	SENSORDB("[SP2509_REARMIPI]enter SP2509_REARMIPISetFlipMirror function\n");

	    SP2509_REARMIPI_write_cmos_sensor(0xfd, 0x01);


    iTemp = SP2509_REARMIPI_read_cmos_sensor(0x3f) & 0x10;	//Clear the mirror and flip bits.
    switch (imgMirror)
    {
        case IMAGE_NORMAL:
            SP2509_REARMIPI_write_cmos_sensor(0x3f, 0x10);	//Set normal
            break;
        case IMAGE_V_MIRROR:
            SP2509_REARMIPI_write_cmos_sensor(0x3f, iTemp | 0x40);	//Set flip
            break;
        case IMAGE_H_MIRROR:
            SP2509_REARMIPI_write_cmos_sensor(0x3f, iTemp | 0x20);	//Set mirror
            break;
        case IMAGE_HV_MIRROR:
            SP2509_REARMIPI_write_cmos_sensor(0x3f, 0x70);	//Set mirror and flip
            break;
    }
	SENSORDB("[SP2509_REARMIPI]exit SP2509_REARMIPISetFlipMirror function\n");
}


/*************************************************************************
* FUNCTION
*   SP2509_REARMIPIPreview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 SP2509_REARMIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 iStartX = 0, iStartY = 0;	
	SENSORDB("[SP2509_REARMIPI]enter SP2509_REARMIPIPreview function\n");
	spin_lock(&sp2509_rear_drv_lock);    
	SP2509_REARMIPI_MPEG4_encode_mode = KAL_FALSE;
	SP2509_REARMIPI_sensor.video_mode=KAL_FALSE;
	SP2509_REARMIPI_sensor.pv_mode=KAL_TRUE;
	SP2509_REARMIPI_sensor.capture_mode=KAL_FALSE;
	spin_unlock(&sp2509_rear_drv_lock);

	PreviewSetting();
        SP2509_REARMIPISetFlipMirror(IMAGE_NORMAL);//hanlei

	iStartX += SP2509_REARMIPI_IMAGE_SENSOR_PV_STARTX;
	iStartY += SP2509_REARMIPI_IMAGE_SENSOR_PV_STARTY;
	spin_lock(&sp2509_rear_drv_lock);

	//SP2509_REARMIPI_sensor.pv_line_length = SP2509_REARMIPI_PV_LINE_LENGTH_PIXELS+SP2509_REARMIPI_sensor.pv_dummy_pixels; 
	//SP2509_REARMIPI_sensor.pv_frame_length = SP2509_REARMIPI_PV_FRAME_LENGTH_LINES+SP2509_REARMIPI_sensor.pv_dummy_lines;
	spin_unlock(&sp2509_rear_drv_lock);
	   
	SP2509_REARMIPI_SetDummy(SP2509_REARMIPI_sensor.pv_dummy_pixels,SP2509_REARMIPI_sensor.pv_dummy_lines);
	SP2509_REARMIPI_SetShutter(SP2509_REARMIPI_sensor.pv_shutter);
	spin_lock(&sp2509_rear_drv_lock);	
	memcpy(&SP2509_REARMIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&sp2509_rear_drv_lock);
	
	image_window->GrabStartX= iStartX;
	image_window->GrabStartY= iStartY;
	image_window->ExposureWindowWidth= SP2509_REARMIPI_REAL_PV_WIDTH ;
	image_window->ExposureWindowHeight= SP2509_REARMIPI_REAL_PV_HEIGHT ; 
	SENSORDB("hcy [SP2509_REARMIPI]eXIT SP2509_REARMIPIPreview function\n"); 
	
	return ERROR_NONE;
}	/* SP2509_REARMIPIPreview() */

/*************************************************************************
* FUNCTION
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 SP2509_REARMIPIVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	//SENSORDB("[SP2509_REARMIPI]enter SP2509_REARMIPIVideo function\n"); 
	
	kal_uint16 iStartX = 0, iStartY = 0;

	spin_lock(&sp2509_rear_drv_lock);    
       SP2509_REARMIPI_MPEG4_encode_mode = KAL_TRUE;  
	SP2509_REARMIPI_sensor.video_mode=KAL_TRUE;
	SP2509_REARMIPI_sensor.pv_mode=KAL_FALSE;
	SP2509_REARMIPI_sensor.capture_mode=KAL_FALSE;
	spin_unlock(&sp2509_rear_drv_lock);
	VideoFullSizeSetting();

	SP2509_REARMIPISetFlipMirror(IMAGE_NORMAL);	//add by lishengli 20130614 hanlei
	iStartX += SP2509_REARMIPI_IMAGE_SENSOR_VIDEO_STARTX;
	iStartY += SP2509_REARMIPI_IMAGE_SENSOR_VIDEO_STARTY;
	spin_lock(&sp2509_rear_drv_lock);

	//SP2509_REARMIPI_sensor.video_line_length = SP2509_REARMIPI_VIDEO_LINE_LENGTH_PIXELS+SP2509_REARMIPI_sensor.video_dummy_pixels; 
	//SP2509_REARMIPI_sensor.video_frame_length = SP2509_REARMIPI_VIDEO_FRAME_LENGTH_LINES+SP2509_REARMIPI_sensor.video_dummy_lines;
	spin_unlock(&sp2509_rear_drv_lock);

	SP2509_REARMIPI_SetDummy(SP2509_REARMIPI_sensor.video_dummy_pixels,SP2509_REARMIPI_sensor.video_dummy_lines);
	SP2509_REARMIPI_SetShutter(SP2509_REARMIPI_sensor.video_shutter);
	spin_lock(&sp2509_rear_drv_lock);	
	memcpy(&SP2509_REARMIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&sp2509_rear_drv_lock);
	image_window->GrabStartX= iStartX;
	image_window->GrabStartY= iStartY;    
    SENSORDB("[SP2509_REARMIPI]eXIT SP2509_REARMIPIVideo function\n"); 
	return ERROR_NONE;
}	/* SP2509_REARMIPIPreview() */

UINT32 SP2509_REARMIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
//       SENSORDB("[SP2509_REARMIPI]enter SP2509_REARMIPICapture function\n");
	  
	kal_uint16 iStartX = 0, iStartY = 0;

	spin_lock(&sp2509_rear_drv_lock);	
	SP2509_REARMIPI_sensor.video_mode=KAL_FALSE;
	SP2509_REARMIPI_sensor.pv_mode=KAL_FALSE;
	SP2509_REARMIPI_sensor.capture_mode=KAL_TRUE;
	SP2509_REARMIPI_MPEG4_encode_mode = KAL_FALSE; 
	SP2509_REARMIPI_Auto_Flicker_mode = KAL_FALSE;    
	spin_unlock(&sp2509_rear_drv_lock);
    
		SP2509_REARMIPI_set_5M();
		SP2509_REARMIPISetFlipMirror(IMAGE_NORMAL);
		spin_lock(&sp2509_rear_drv_lock);

		//SP2509_REARMIPI_sensor.cp_line_length=SP2509_REARMIPI_FULL_LINE_LENGTH_PIXELS+SP2509_REARMIPI_sensor.cp_dummy_pixels;
		//SP2509_REARMIPI_sensor.cp_frame_length=SP2509_REARMIPI_FULL_FRAME_LENGTH_LINES+SP2509_REARMIPI_sensor.cp_dummy_lines;
		spin_unlock(&sp2509_rear_drv_lock);
		iStartX = SP2509_REARMIPI_IMAGE_SENSOR_CAP_STARTX;
		iStartY = SP2509_REARMIPI_IMAGE_SENSOR_CAP_STARTY;
		image_window->GrabStartX=iStartX;
		image_window->GrabStartY=iStartY;
		image_window->ExposureWindowWidth=SP2509_REARMIPI_REAL_CAP_WIDTH ;
		image_window->ExposureWindowHeight=SP2509_REARMIPI_REAL_CAP_HEIGHT;
		SP2509_REARMIPI_SetDummy(SP2509_REARMIPI_sensor.cp_dummy_pixels, SP2509_REARMIPI_sensor.cp_dummy_lines);   
	
	
	spin_lock(&sp2509_rear_drv_lock);	
	memcpy(&SP2509_REARMIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	spin_unlock(&sp2509_rear_drv_lock);
	
	SENSORDB("[SP2509_REARMIPI]exit SP2509_REARMIPICapture function\n");
	return ERROR_NONE;
}	/* SP2509_REARMIPICapture() */

UINT32 SP2509_REARMIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    SENSORDB("[SP2509_REARMIPI]eXIT SP2509_REARMIPIGetResolution function\n");
    pSensorResolution->SensorPreviewWidth	= SP2509_REARMIPI_REAL_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= SP2509_REARMIPI_REAL_PV_HEIGHT;
    pSensorResolution->SensorFullWidth		= SP2509_REARMIPI_REAL_CAP_WIDTH;
    pSensorResolution->SensorFullHeight		= SP2509_REARMIPI_REAL_CAP_HEIGHT;
    pSensorResolution->SensorVideoWidth		= SP2509_REARMIPI_REAL_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = SP2509_REARMIPI_REAL_VIDEO_HEIGHT;
    SENSORDB("SP2509_REARMIPIGetResolution :8-14");    

    return ERROR_NONE;
}   /* SP2509_REARMIPIGetResolution() */

UINT32 SP2509_REARMIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{ 
	SENSORDB("[SP2509_REARMIPI]enter SP2509_REARMIPIGetInfo function\n");
	
      pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;//SENSOR_OUTPUT_FORMAT_RAW_B;hanlei
      pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW; 
      
      pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
      pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;

      pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;//hanlei//SENSOR_INTERFACE_TYPE_PARALLEL;
			
    pSensorInfo->CaptureDelayFrame = 1; //2//hanlei140401
      pSensorInfo->PreviewDelayFrame =  1; //old 2
      /*< DTS2016112302816 yangyale/lwx352794 20161123 begin */
      pSensorInfo->VideoDelayFrame = 0;
      /* DTS2016112302816 yangyale/lwx352794 20161123 end > */

      pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA; //hanlei dri     
      pSensorInfo->AEShutDelayFrame = 0;	//  2   //0//hanlei	    /* The frame of setting shutter default 0 for TG int */
      pSensorInfo->AESensorGainDelayFrame =  0;  // 2  //0//hanlei   /* The frame of setting sensor gain */
      pSensorInfo->AEISPGainDelayFrame =  2;
	   
      switch (ScenarioId)
      {
          case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
               pSensorInfo->SensorClockFreq=24;

               pSensorInfo->SensorClockRisingCount= 0;

               pSensorInfo->SensorGrabStartX = SP2509_REARMIPI_IMAGE_SENSOR_PV_STARTX; 
               pSensorInfo->SensorGrabStartY = SP2509_REARMIPI_IMAGE_SENSOR_PV_STARTY;           		 
               pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
               pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
               pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
               pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
               pSensorInfo->SensorPacketECCOrder = 1;
			
               break;	
          case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
               pSensorInfo->SensorClockFreq=24;
               pSensorInfo->SensorClockRisingCount= 0;

               pSensorInfo->SensorGrabStartX = SP2509_REARMIPI_IMAGE_SENSOR_VIDEO_STARTX; 
               pSensorInfo->SensorGrabStartY = SP2509_REARMIPI_IMAGE_SENSOR_VIDEO_STARTY;				   
               pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;		   
               pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
               pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
               pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
               pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
               pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
               pSensorInfo->SensorPacketECCOrder = 1;
			   
               break;
          case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	   case MSDK_SCENARIO_ID_CAMERA_ZSD:
               pSensorInfo->SensorClockFreq=24;
               pSensorInfo->SensorClockRisingCount= 0;
               
               pSensorInfo->SensorGrabStartX = SP2509_REARMIPI_IMAGE_SENSOR_CAP_STARTX;//	//2*SP2509_REARMIPI_IMAGE_SENSOR_PV_STARTX; 
               pSensorInfo->SensorGrabStartY = SP2509_REARMIPI_IMAGE_SENSOR_CAP_STARTY;//	//2*SP2509_REARMIPI_IMAGE_SENSOR_PV_STARTY;          			
               pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;				
               pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
               pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
               pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;            
               pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
               pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x
               pSensorInfo->SensorPacketECCOrder = 1;

               break;
          default:
               pSensorInfo->SensorClockFreq=24;
               pSensorInfo->SensorClockRisingCount= 0;
               pSensorInfo->SensorGrabStartX = SP2509_REARMIPI_IMAGE_SENSOR_PV_STARTX; 
               pSensorInfo->SensorGrabStartY = SP2509_REARMIPI_IMAGE_SENSOR_PV_STARTY; 				 
               pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;		 
               pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
               pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
               pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;		  	 
               pSensorInfo->SensorWidthSampling = 0;	// 0 is default 1x
               pSensorInfo->SensorHightSampling = 0;	 // 0 is default 1x 
               pSensorInfo->SensorPacketECCOrder = 1;
			
               break;
      }
	
      spin_lock(&sp2509_rear_drv_lock);	
      SP2509_REARMIPIPixelClockDivider=pSensorInfo->SensorPixelClockCount;
      memcpy(pSensorConfigData, &SP2509_REARMIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
      spin_unlock(&sp2509_rear_drv_lock);
      SENSORDB("[hcy SP2509_REARMIPI]exit SP2509_REARMIPIGetInfo function\n");
      return ERROR_NONE;
}   /* SP2509_REARMIPIGetInfo() */


UINT32 SP2509_REARMIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{    
		spin_lock(&sp2509_rear_drv_lock);	
		CurrentScenarioId = ScenarioId;
		spin_unlock(&sp2509_rear_drv_lock);
		SENSORDB("[SP2509_REARMIPI]enter SP2509_REARMIPIControl function\n");
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            SP2509_REARMIPIPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			SP2509_REARMIPIVideo(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	      case MSDK_SCENARIO_ID_CAMERA_ZSD:
            SP2509_REARMIPICapture(pImageWindow, pSensorConfigData);//hhl 2-28
            break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
	SENSORDB("[SP2509_REARMIPI]exit SP2509_REARMIPIControl function\n");
    return ERROR_NONE;
} /* SP2509_REARMIPIControl() */

UINT32 SP2509_REARMIPISetVideoMode(UINT16 u2FrameRate)
{
//    SENSORDB("[SP2509_REARMIPISetVideoMode] frame rate = %d\n", u2FrameRate);
//	kal_uint16 SP2509_REARMIPI_Video_Max_Expourse_Time = 0;
	SENSORDB("[SP2509_REARMIPI]%s():fix_frame_rate=%d\n",__FUNCTION__,u2FrameRate);
	spin_lock(&sp2509_rear_drv_lock);
	SP2509_REARMIPI_sensor.fix_video_fps = KAL_TRUE;
	spin_unlock(&sp2509_rear_drv_lock);
	u2FrameRate=u2FrameRate*10;//10*FPS
	SENSORDB("[SP2509_REARMIPI][Enter Fix_fps func] SP2509_REARMIPI_Fix_Video_Frame_Rate = %d\n", u2FrameRate/10);
	//SP2509_REARMIPI_Video_Max_Expourse_Time = (kal_uint16)((SP2509_REARMIPI_sensor.video_pclk*10/u2FrameRate)/SP2509_REARMIPI_sensor.video_line_length);
	#if 0
	if (SP2509_REARMIPI_Video_Max_Expourse_Time > SP2509_REARMIPI_VIDEO_FRAME_LENGTH_LINES/*SP2509_REARMIPI_sensor.pv_frame_length*/) 
	{
		spin_lock(&sp2509_rear_drv_lock);    
		SP2509_REARMIPI_sensor.video_frame_length = SP2509_REARMIPI_Video_Max_Expourse_Time;
		SP2509_REARMIPI_sensor.video_dummy_lines = SP2509_REARMIPI_sensor.video_frame_length-SP2509_REARMIPI_VIDEO_FRAME_LENGTH_LINES;
		spin_unlock(&sp2509_rear_drv_lock);
		
		SP2509_REARMIPI_SetDummy(SP2509_REARMIPI_sensor.video_dummy_pixels,SP2509_REARMIPI_sensor.video_dummy_lines);
	}
	#endif
	spin_lock(&sp2509_rear_drv_lock);    
	SP2509_REARMIPI_MPEG4_encode_mode = KAL_TRUE; 
	spin_unlock(&sp2509_rear_drv_lock);
	SENSORDB("[SP2509_REARMIPI]exit SP2509_REARMIPISetVideoMode function\n");


	
	return ERROR_NONE;
}

UINT32 SP2509_REARMIPISetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	kal_uint32 pv_max_frame_rate_lines=0;

	if(SP2509_REARMIPI_sensor.pv_mode==TRUE)
	pv_max_frame_rate_lines=SP2509_REARMIPI_PV_FRAME_LENGTH_LINES;
	else
    pv_max_frame_rate_lines=SP2509_REARMIPI_VIDEO_FRAME_LENGTH_LINES	;
    SENSORDB("[SP2509_REARMIPISetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);
    if(bEnable) 
	{   // enable auto flicker   
    	spin_lock(&sp2509_rear_drv_lock);    
        SP2509_REARMIPI_Auto_Flicker_mode = KAL_TRUE; 
		spin_unlock(&sp2509_rear_drv_lock);
        if(SP2509_REARMIPI_MPEG4_encode_mode == KAL_TRUE) 
		{ // in the video mode, reset the frame rate
            pv_max_frame_rate_lines = SP2509_REARMIPI_MAX_EXPOSURE_LINES + (SP2509_REARMIPI_MAX_EXPOSURE_LINES>>7);            
           // SP2509_REARMIPI_write_cmos_sensor(0x0104, 1);        
            //SP2509_REARMIPI_write_cmos_sensor(0x0340, (pv_max_frame_rate_lines >>8) & 0xFF);
            //SP2509_REARMIPI_write_cmos_sensor(0x0341, pv_max_frame_rate_lines & 0xFF);	
            //SP2509_REARMIPI_write_cmos_sensor(0x0104, 0);        	
        }
    } 
	else 
	{
    	spin_lock(&sp2509_rear_drv_lock);    
        SP2509_REARMIPI_Auto_Flicker_mode = KAL_FALSE; 
		spin_unlock(&sp2509_rear_drv_lock);
        if(SP2509_REARMIPI_MPEG4_encode_mode == KAL_TRUE) 
		{    // in the video mode, restore the frame rate
            //SP2509_REARMIPI_write_cmos_sensor(0x0104, 1);        
            //SP2509_REARMIPI_write_cmos_sensor(0x0340, (SP2509_REARMIPI_MAX_EXPOSURE_LINES >>8) & 0xFF);
            //SP2509_REARMIPI_write_cmos_sensor(0x0341, SP2509_REARMIPI_MAX_EXPOSURE_LINES & 0xFF);	
           // SP2509_REARMIPI_write_cmos_sensor(0x0104, 0);        	
        }
        printk("Disable Auto flicker\n");    
    }
    return ERROR_NONE;
}
UINT32 SP2509_REARMIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
//	kal_uint16 lineLength,frameHeight;	
	SENSORDB("SP2509_REARMIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk =SP2509_REARMIPI_sensor.pv_pclk;//24000000;//hanlei
			//lineLength = SP2509_REARMIPI_PV_LINE_LENGTH_PIXELS;
			//lineLength = SP2509_REARMIPI_PV_LINE_LENGTH_PIXELS+SP2509_REARMIPI_sensor.pv_dummy_pixels;
			//frameHeight = (10 * pclk)/frameRate/lineLength;
			//dummyLine = frameHeight - SP2509_REARMIPI_PV_FRAME_LENGTH_LINES;

			
			if(dummyLine<0)
				dummyLine = 0;
			
			spin_lock(&sp2509_rear_drv_lock);	
			SP2509_REARMIPI_sensor.pv_mode=TRUE;
			spin_unlock(&sp2509_rear_drv_lock);
			SP2509_REARMIPI_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = SP2509_REARMIPI_sensor.video_pclk;//24000000;//hanlei
			//lineLength = SP2509_REARMIPI_VIDEO_LINE_LENGTH_PIXELS;
			//lineLength = SP2509_REARMIPI_VIDEO_LINE_LENGTH_PIXELS+SP2509_REARMIPI_sensor.video_dummy_pixels;//copy hailei
			//frameHeight = (10 * pclk)/frameRate/lineLength;
			//dummyLine = frameHeight - SP2509_REARMIPI_VIDEO_FRAME_LENGTH_LINES;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&sp2509_rear_drv_lock);	
			SP2509_REARMIPI_sensor.pv_mode=TRUE;
			spin_unlock(&sp2509_rear_drv_lock);
			SP2509_REARMIPI_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			//hanlei add
			////////////////////////////////////////////////////
			pclk = SP2509_REARMIPI_sensor.cp_pclk;//18000000;
			//lineLength = SP2509_REARMIPI_FULL_LINE_LENGTH_PIXELS+SP2509_REARMIPI_sensor.cp_dummy_pixels;//copy hanlei
			//frameHeight = (10 * pclk)/frameRate/lineLength;
			//dummyLine = frameHeight - SP2509_REARMIPI_FULL_FRAME_LENGTH_LINES;
			if(dummyLine<0)
				dummyLine = 0;
			
			spin_lock(&sp2509_rear_drv_lock);	
			SP2509_REARMIPI_sensor.pv_mode=FALSE;
			spin_unlock(&sp2509_rear_drv_lock);
			SP2509_REARMIPI_SetDummy(0, dummyLine);	
			////////////////////////////////////////////////////
			break;
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = SP2509_REARMIPI_sensor.cp_pclk;//12000000;//hanlei
			//lineLength = SP2509_REARMIPI_FULL_LINE_LENGTH_PIXELS+SP2509_REARMIPI_sensor.cp_dummy_pixels;//copy hanlei
			//frameHeight = (10 * pclk)/frameRate/lineLength;
			//dummyLine = frameHeight - SP2509_REARMIPI_FULL_FRAME_LENGTH_LINES;
			if(dummyLine<0)
				dummyLine = 0;
			
			spin_lock(&sp2509_rear_drv_lock);	
			SP2509_REARMIPI_sensor.pv_mode=FALSE;
			spin_unlock(&sp2509_rear_drv_lock);
			SP2509_REARMIPI_SetDummy(0, dummyLine);			
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			break;		
		default:
			break;
	}
	SENSORDB("[SP2509_REARMIPI]exit SP2509_REARMIPISetMaxFramerateByScenario function\n");
	return ERROR_NONE;
}
UINT32 SP2509_REARMIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			 *pframeRate = 180;//120; //hanlei add
			 break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 180;//150; //hanlei
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 60;//80 ;//hanlei
			break;		//hhl 2-28
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 180;//150 ;//hanlei
			break;		
		default:
			break;
	}

	return ERROR_NONE;
}
UINT32 SP2509_REARMIPISetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("[SP2509_REARMIPISetTestPatternMode] Test pattern enable:%d\n", bEnable);
    
    if(bEnable) {   // enable color bar   
        //SP2509_REARMIPI_write_cmos_sensor(0x30D8, 0x10);  // color bar test pattern
        //SP2509_REARMIPI_write_cmos_sensor(0x0600, 0x00);  // color bar test pattern
        //SP2509_REARMIPI_write_cmos_sensor(0x0601, 0x02);  // color bar test pattern 
    } else {
        //SP2509_REARMIPI_write_cmos_sensor(0x30D8, 0x00);  // disable color bar test pattern
    }
    return ERROR_NONE;
}

UINT32 SP2509_REARMIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++=SP2509_REARMIPI_REAL_CAP_WIDTH;
            *pFeatureReturnPara16=SP2509_REARMIPI_REAL_CAP_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
        		switch(CurrentScenarioId)
        		{
        			case MSDK_SCENARIO_ID_CAMERA_ZSD:
        		    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
 		            *pFeatureReturnPara16++=SP2509_REARMIPI_sensor.cp_line_length;  
 		            *pFeatureReturnPara16=SP2509_REARMIPI_sensor.cp_frame_length;
		            SENSORDB("Sensor period:%d %d\n",SP2509_REARMIPI_sensor.cp_line_length, SP2509_REARMIPI_sensor.cp_frame_length); 
		            *pFeatureParaLen=4;        				
        				break;
        			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara16++=SP2509_REARMIPI_sensor.video_line_length;  
					*pFeatureReturnPara16=SP2509_REARMIPI_sensor.video_frame_length;
					 SENSORDB("Sensor period:%d %d\n", SP2509_REARMIPI_sensor.video_line_length, SP2509_REARMIPI_sensor.video_frame_length); 
					 *pFeatureParaLen=4;
						break;
        			default:	
					*pFeatureReturnPara16++=SP2509_REARMIPI_sensor.pv_line_length;  
					*pFeatureReturnPara16=SP2509_REARMIPI_sensor.pv_frame_length;
		            SENSORDB("Sensor period:%d %d\n", SP2509_REARMIPI_sensor.pv_line_length, SP2509_REARMIPI_sensor.pv_frame_length); 
		            *pFeatureParaLen=4;
	            break;
          	}
          	break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        		switch(CurrentScenarioId)
        		{
        			case MSDK_SCENARIO_ID_CAMERA_ZSD:
        			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		            *pFeatureReturnPara32 = SP2509_REARMIPI_sensor.cp_pclk; 
		            *pFeatureParaLen=4;		         	
					
		            SENSORDB("Sensor CPCLK:%dn",SP2509_REARMIPI_sensor.cp_pclk); 
		         		break; //hhl 2-28
					case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
						*pFeatureReturnPara32 = SP2509_REARMIPI_sensor.video_pclk;
						*pFeatureParaLen=4;
						SENSORDB("Sensor videoCLK:%d\n",SP2509_REARMIPI_sensor.video_pclk); 
						break;
		         		default:
		            *pFeatureReturnPara32 = SP2509_REARMIPI_sensor.pv_pclk;
		            *pFeatureParaLen=4;
					SENSORDB("Sensor pvclk:%d\n",SP2509_REARMIPI_sensor.pv_pclk); 
		            break;
		         }
		         break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            SP2509_REARMIPI_SetShutter(*pFeatureData16); 
            break;
		case SENSOR_FEATURE_SET_SENSOR_SYNC: 
			break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            SP2509_REARMIPI_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
           SP2509_REARMIPI_SetGain((UINT16) *pFeatureData16); 
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			spin_lock(&sp2509_rear_drv_lock);    
            SP2509_REARMIPI_isp_master_clock=*pFeatureData32;
			spin_unlock(&sp2509_rear_drv_lock);
            break;
        case SENSOR_FEATURE_SET_REGISTER:
			SP2509_REARMIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = SP2509_REARMIPI_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&sp2509_rear_drv_lock);    
                SP2509_REARMIPISensorCCT[i].Addr=*pFeatureData32++;
                SP2509_REARMIPISensorCCT[i].Para=*pFeatureData32++; 
				spin_unlock(&sp2509_rear_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=SP2509_REARMIPISensorCCT[i].Addr;
                *pFeatureData32++=SP2509_REARMIPISensorCCT[i].Para; 
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {	spin_lock(&sp2509_rear_drv_lock);    
                SP2509_REARMIPISensorReg[i].Addr=*pFeatureData32++;
                SP2509_REARMIPISensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&sp2509_rear_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=SP2509_REARMIPISensorReg[i].Addr;
                *pFeatureData32++=SP2509_REARMIPISensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=SP2509_REARMIPI_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, SP2509_REARMIPISensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, SP2509_REARMIPISensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &SP2509_REARMIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            SP2509_REARMIPI_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            SP2509_REARMIPI_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=SP2509_REARMIPI_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            SP2509_REARMIPI_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            SP2509_REARMIPI_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            SP2509_REARMIPI_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

      case SENSOR_FEATURE_INITIALIZE_AF:
            break;
        case SENSOR_FEATURE_CONSTANT_AF:
            break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            SP2509_REARMIPISetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            SP2509_REARMIPIGetSensorID(pFeatureReturnPara32); 
            break;             
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            SP2509_REARMIPISetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));            
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            SP2509_REARMIPISetTestPatternMode((BOOL)*pFeatureData16);        	
            break;
	 case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			SP2509_REARMIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
	 case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			SP2509_REARMIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
        default:
            break;
    }
    return ERROR_NONE;
}	/* SP2509_REARMIPIFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncSP2509_REARMIPI=
{
    SP2509_REARMIPIOpen,
    SP2509_REARMIPIGetInfo,
    SP2509_REARMIPIGetResolution,
    SP2509_REARMIPIFeatureControl,
    SP2509_REARMIPIControl,
    SP2509_REARMIPIClose
};

UINT32 SP2509_REAR_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */


#ifdef  ONLINE_DEBUG//for online debug
      if(ONLINE_DEBUG_BZW==KAL_TRUE)
      	{
	  fcam_class = class_create(THIS_MODULE, "fcam");
		if (IS_ERR(fcam_class)) 
		{
			printk("Create class fcam.\n");
			return -ENOMEM;
		}
		fcam_dev = device_create(fcam_class, NULL, MKDEV(0, 1), NULL, "dev");
		 device_create_file(fcam_dev, &dev_attr_fcam);
		 device_create_file(fcam_dev, &dev_attr_currreg);
	       ONLINE_DEBUG_BZW=KAL_FALSE;
       }

#endif


	
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncSP2509_REARMIPI;
    return ERROR_NONE;
}   /* SensorInit() */

