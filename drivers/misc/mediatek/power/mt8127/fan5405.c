#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mt-plat/charging.h>
#include "fan5405.h"
//#include  "../../i2c/mt8127/mt_i2c.h"
#define fan5405_SLAVE_ADDR_WRITE   0xD6
#define fan5405_SLAVE_ADDR_READ    0xD7

static struct i2c_client *new_client;
static const struct i2c_device_id fan5405_i2c_id[] = { {"fan5405", 0}, {} };
/*<DTS2017010500156 rentianzhi/WX430340 20170105 begin*/
/*add for control reg dump output */
static int Is_Dump_Reg = 0;
module_param(Is_Dump_Reg, int, S_IRUSR);
/*DTS2017010500156 rentianzhi/WX430340 20170105 end>*/

kal_bool chargin_hw_init_done = KAL_FALSE;
static int fan5405_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

#ifdef CONFIG_OF
static const struct of_device_id fan5405_of_match[] = {
	{.compatible = "fan5405",},
	{},
};

MODULE_DEVICE_TABLE(of, fan5405_of_match);
#endif

static struct i2c_driver fan5405_driver = {
	.driver = {
		   .name = "fan5405",
#ifdef CONFIG_OF
		   .of_match_table = fan5405_of_match,
#endif
	},
	.probe = fan5405_driver_probe,
	.id_table = fan5405_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char fan5405_reg[fan5405_REG_NUM] = { 0 };

static DEFINE_MUTEX(fan5405_i2c_access);

int g_fan5405_hw_exist = 0;

/**********************************************************
  *
  *   [I2C Function For Read/Write fan5405]
  *
  *********************************************************/
int fan5405_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char     readData = 0;
	int      ret = 0;
	struct i2c_msg msg[2];
	struct i2c_adapter *adap = new_client->adapter;

	mutex_lock(&fan5405_i2c_access);
	msg[0].addr = new_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &cmd;

	msg[1].addr = new_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &readData;

	ret = i2c_transfer(adap, msg, 2);
	if (ret < 0) {
		mutex_unlock(&fan5405_i2c_access);
		return 0;
	}
	*returnData = readData;

	mutex_unlock(&fan5405_i2c_access);
	return 1;
}

int fan5405_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;
	struct i2c_msg msg;
	struct i2c_adapter *adap = new_client->adapter;

	mutex_lock(&fan5405_i2c_access);
	write_data[0] = cmd;
	write_data[1] = writeData;
	msg.addr = new_client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = (char *)write_data;

	ret = i2c_transfer(adap, &msg, 1);
	if (ret < 0) {
		mutex_unlock(&fan5405_i2c_access);
		return 0;
	}

	mutex_unlock(&fan5405_i2c_access);
	return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int fan5405_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char fan5405_reg = 0;
	int ret = 0;

	ret = fan5405_read_byte(RegNum, &fan5405_reg);

	battery_log(BAT_LOG_FULL, "[fan5405_read_interface] Reg[%x]=0x%x\n", RegNum, fan5405_reg);

	fan5405_reg &= (MASK << SHIFT);
	*val = (fan5405_reg >> SHIFT);

	battery_log(BAT_LOG_FULL, "[fan5405_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int fan5405_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char fan5405_reg = 0;
	int ret = 0;

	ret = fan5405_read_byte(RegNum, &fan5405_reg);
	/*<DTS2017010500156 rentianzhi/WX430340 20170105 begin*/
	battery_log(BAT_LOG_FULL, "[fan5405_config_interface] Reg[%x]=0x%x\n", RegNum, fan5405_reg);
	/*DTS2017010500156 rentianzhi/WX430340 20170105 end>*/
	fan5405_reg &= ~(MASK << SHIFT);
	fan5405_reg |= (val << SHIFT);

	if (RegNum == fan5405_CON4 && val == 1 && MASK == CON4_RESET_MASK
	    && SHIFT == CON4_RESET_SHIFT) {
		/* RESET bit */
	} else if (RegNum == fan5405_CON4) {
		fan5405_reg &= ~0x80;	/* RESET bit read returs 1, so clear it */
	}

	ret = fan5405_write_byte(RegNum, fan5405_reg);
	/*<DTS2017010500156 rentianzhi/WX430340 20170105 begin*/
	battery_log(BAT_LOG_FULL, "[fan5405_config_interface] write Reg[%x]=0x%x\n", RegNum,
		    fan5405_reg);
	/*DTS2017010500156 rentianzhi/WX430340 20170105 end>*/

	return ret;
}

/* write one register directly */
unsigned int fan5405_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	int ret = 0;
	

	fan5405_read_byte(RegNum, &fan5405_reg[RegNum]);
	/*<DTS2017010500156 rentianzhi/WX430340 20170105 begin*/
	battery_log(BAT_LOG_FULL, "RegNum ==%d,  val ==%x \n",RegNum,val);
	/*DTS2017010500156 rentianzhi/WX430340 20170105 end>*/
	fan5405_dump_register();
	ret = fan5405_write_byte(RegNum, val);
	fan5405_dump_register();

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0 */

void fan5405_set_tmr_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_TMR_RST_MASK),
				       (unsigned char) (CON0_TMR_RST_SHIFT)
	    );
}

unsigned int fan5405_get_otg_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON0),
				     (&val), (unsigned char) (CON0_OTG_MASK),
				     (unsigned char) (CON0_OTG_SHIFT)
	    );
	return val;
}

void fan5405_set_en_stat(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_STAT_MASK),
				       (unsigned char) (CON0_EN_STAT_SHIFT)
	    );
//	fan5405_read_byte(fan5405_CON0, &fan5405_reg[fan5405_CON0]);
	
}

unsigned int fan5405_get_chip_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON0),
				     (&val), (unsigned char) (CON0_STAT_MASK),
				     (unsigned char) (CON0_STAT_SHIFT)
	    );
	return val;
}

unsigned int fan5405_get_boost_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON0),
				     (&val), (unsigned char) (CON0_BOOST_MASK),
				     (unsigned char) (CON0_BOOST_SHIFT)
	    );
	return val;
}

unsigned int fan5405_get_fault_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON0),
				     (&val), (unsigned char) (CON0_FAULT_MASK),
				     (unsigned char) (CON0_FAULT_SHIFT)
	    );
	return val;
}

/* CON1 */

void fan5405_set_input_charging_current(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_LIN_LIMIT_MASK),
				       (unsigned char) (CON1_LIN_LIMIT_SHIFT)
	    );
	//fan5405_read_byte(fan5405_CON0, &fan5405_reg[fan5405_CON0]);
	
}

void fan5405_set_v_low(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_LOW_V_MASK),
				       (unsigned char) (CON1_LOW_V_SHIFT)
	    );
}

void fan5405_set_te(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_TE_MASK),
				       (unsigned char) (CON1_TE_SHIFT)
	    );
}

void fan5405_set_ce(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_CE_MASK),
				       (unsigned char) (CON1_CE_SHIFT)
	    );
}

void fan5405_set_hz_mode(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_HZ_MODE_MASK),
				       (unsigned char) (CON1_HZ_MODE_SHIFT)
	    );
}

void fan5405_set_opa_mode(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OPA_MODE_MASK),
				       (unsigned char) (CON1_OPA_MODE_SHIFT)
	    );
}

/* CON2 */

void fan5405_set_oreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OREG_MASK),
				       (unsigned char) (CON2_OREG_SHIFT)
	    );
}

void fan5405_set_otg_pl(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OTG_PL_MASK),
				       (unsigned char) (CON2_OTG_PL_SHIFT)
	    );
}

void fan5405_set_otg_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OTG_EN_MASK),
				       (unsigned char) (CON2_OTG_EN_SHIFT)
	    );
}

/* CON3 */

unsigned int fan5405_get_vender_code(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON3),
				     (&val), (unsigned char) (CON3_VENDER_CODE_MASK),
				     (unsigned char) (CON3_VENDER_CODE_SHIFT)
	    );
	return val;
}

unsigned int fan5405_get_pn(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON3),
				     (&val), (unsigned char) (CON3_PIN_MASK),
				     (unsigned char) (CON3_PIN_SHIFT)
	    );
	return val;
}

unsigned int fan5405_get_revision(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON3),
				     (&val), (unsigned char) (CON3_REVISION_MASK),
				     (unsigned char) (CON3_REVISION_SHIFT)
	    );
	return val;
}

/* CON4 */

void fan5405_set_reset(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_RESET_MASK),
				       (unsigned char) (CON4_RESET_SHIFT)
	    );
}

void fan5405_set_iocharge(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_I_CHR_MASK),
				       (unsigned char) (CON4_I_CHR_SHIFT)
	    );
}

void fan5405_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_I_TERM_MASK),
				       (unsigned char) (CON4_I_TERM_SHIFT)
	    );
}

/* CON5 */

void fan5405_set_dis_vreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_DIS_VREG_MASK),
				       (unsigned char) (CON5_DIS_VREG_SHIFT)
	    );
}

void fan5405_set_io_level(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_IO_LEVEL_MASK),
				       (unsigned char) (CON5_IO_LEVEL_SHIFT)
	    );
}

unsigned int fan5405_get_sp_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON5),
				     (&val), (unsigned char) (CON5_SP_STATUS_MASK),
				     (unsigned char) (CON5_SP_STATUS_SHIFT)
	    );
	return val;
}

unsigned int fan5405_get_en_level(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface((unsigned char) (fan5405_CON5),
				     (&val), (unsigned char) (CON5_EN_LEVEL_MASK),
				     (unsigned char) (CON5_EN_LEVEL_SHIFT)
	    );
	return val;
}

void fan5405_set_vsp(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_VSP_MASK),
				       (unsigned char) (CON5_VSP_SHIFT)
	    );
}

/* CON6 */

void fan5405_set_i_safe(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_ISAFE_MASK),
				       (unsigned char) (CON6_ISAFE_SHIFT)
	    );
}

void fan5405_set_v_safe(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan5405_config_interface((unsigned char) (fan5405_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VSAFE_MASK),
				       (unsigned char) (CON6_VSAFE_SHIFT)
	    );
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
void fan5405_hw_component_detect(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan5405_read_interface(0x03, &val, 0xFF, 0x0);

	if (val == 0)
		g_fan5405_hw_exist = 0;
	else
		g_fan5405_hw_exist = 1;

	battery_log(BAT_LOG_CRTI, "[fan5405_hw_component_detect] exist=%d, Reg[03]=0x%x\n", g_fan5405_hw_exist, val);
}

int is_fan5405_exist(void)
{
	battery_log(BAT_LOG_CRTI, "[is_fan5405_exist] g_fan5405_hw_exist=%d\n", g_fan5405_hw_exist);

	return g_fan5405_hw_exist;
}

void fan5405_dump_register(void)
{
	int i = 0;
	/*<DTS2017010500156 rentianzhi/WX430340 20170105 begin*/
	if(Is_Dump_Reg)
	{
		for (i = 0; i < fan5405_REG_NUM; i++) {
			fan5405_read_byte(i, &fan5405_reg[i]);
			battery_log(BAT_LOG_CRTI, "[fan5405] [0x%x]=0x%x \n", i, fan5405_reg[i]);
		}
	}
	/*DTS2017010500156 rentianzhi/WX430340 20170105 end>*/
}

static int fan5405_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	new_client = client;

	fan5405_hw_component_detect();
/*	R06 and R02 were initialized in lk.
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)||defined(CONFIG_HIGH_BATTERY_VOLTAGE_SUPPORT)
	fan5405_reg_config_interface(0x06,0x77); // ISAFE = 1250mA, VSAFE = 4.34V
	fan5405_reg_config_interface(0x02,0xaa); //4.34
#else
	fan5405_reg_config_interface(0x06,0x70);
#endif
*/
	fan5405_dump_register();
	chargin_hw_init_done = KAL_TRUE;

	return 0;
}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_fan5405 = 0;
static ssize_t show_fan5405_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "[show_fan5405_access] 0x%x\n", g_reg_value_fan5405);
	return sprintf(buf, "%u\n", g_reg_value_fan5405);
}

static ssize_t store_fan5405_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	battery_log(BAT_LOG_CRTI, "[store_fan5405_access]\n");

	if (buf != NULL && size != 0) {

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			/*<DTS2017040707821 liumaoxin WX422269 20170411 begin */
			if (!addr)
				return -EINVAL;
			/*DTS2017040707821 liumaoxin WX422269 20170411 end> */
			ret = kstrtou32(addr, 16, (unsigned int *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16, (unsigned int *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			/*<DTS2017040707821 liumaoxin WX422269 20170411 begin */
			if (!val)
				return -EINVAL;
			/*DTS2017040707821 liumaoxin WX422269 20170411 end> */
			ret = kstrtou32(val, 16, (unsigned int *)&reg_value);

			battery_log(BAT_LOG_CRTI,
			    "[store_fan5405_access] write fan5405 reg 0x%x with value 0x%x !\n",
			     reg_address, reg_value);
			ret = fan5405_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = fan5405_read_interface(reg_address, &g_reg_value_fan5405, 0xFF, 0x0);
			battery_log(BAT_LOG_CRTI,
			    "[store_fan5405_access] read fan5405 reg 0x%x with value 0x%x !\n",
			     reg_address, g_reg_value_fan5405);
			battery_log(BAT_LOG_CRTI,
			    "[store_fan5405_access] Please use \"cat fan5405_access\" to get value\r\n");
		}
	}
	return size;
}

static DEVICE_ATTR(fan5405_access, 0664, show_fan5405_access, store_fan5405_access);	/* 664 */

static int fan5405_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	battery_log(BAT_LOG_CRTI, "******** fan5405_user_space_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_fan5405_access);

	return 0;
}

struct platform_device fan5405_user_space_device = {
	.name = "fan5405-user",
	.id = -1,
};

static struct platform_driver fan5405_user_space_driver = {
	.probe = fan5405_user_space_probe,
	.driver = {
		   .name = "fan5405-user",
	},
};

static int __init fan5405_init(void)
{
	int ret = 0;

	if (i2c_add_driver(&fan5405_driver) != 0) {
		battery_log(BAT_LOG_CRTI,
			    "[fan5405_init] failed to register fan5405 i2c driver.\n");
	} else {
		battery_log(BAT_LOG_CRTI,
			    "[fan5405_init] Success to register fan5405 i2c driver.\n");
	}

	/* fan5405 user space access interface */
	ret = platform_device_register(&fan5405_user_space_device);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[fan5405_init] Unable to device register(%d)\n",
			    ret);
		return ret;
	}
	ret = platform_driver_register(&fan5405_user_space_driver);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[fan5405_init] Unable to register driver (%d)\n",
			    ret);
		return ret;
	}

	return 0;
}

static void __exit fan5405_exit(void)
{
	i2c_del_driver(&fan5405_driver);
}

subsys_initcall(fan5405_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C fan5405 Driver");
MODULE_AUTHOR("James Lo<james.lo@mediatek.com>");