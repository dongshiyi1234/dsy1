/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

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
#include "bq25601.h"

/**********************************************************
  *
  *   [I2C Slave Setting]
  *
  *********************************************************/
#define bq25601_SLAVE_ADDR_WRITE   0xD6
#define bq25601_SLAVE_ADDR_READ    0xD7

static struct i2c_client *new_client;
static const struct i2c_device_id bq25601_i2c_id[] = { {"bq25601", 0}, {} };

kal_bool chargin_hw_init_done = KAL_FALSE;
static int bq25601_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

#ifdef CONFIG_OF
static const struct of_device_id bq25601_of_match[] = {
	{.compatible = "bq25601",},
	{},
};

MODULE_DEVICE_TABLE(of, bq25601_of_match);
#endif

static struct i2c_driver bq25601_driver = {
	.driver = {
		   .name = "bq25601",
#ifdef CONFIG_OF
		   .of_match_table = bq25601_of_match,
#endif
		   },
	.probe = bq25601_driver_probe,
	.id_table = bq25601_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char bq25601_reg[bq25601_REG_NUM] = { 0 };

static DEFINE_MUTEX(bq25601_i2c_access);

int g_bq25601_hw_exist = 0;

/**********************************************************
  *
  *   [I2C Function For Read/Write bq25601]
  *
  *********************************************************/
int bq25601_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char     readData = 0;
	int      ret = 0;
	struct i2c_msg msg[2];
	struct i2c_adapter *adap = new_client->adapter;

	mutex_lock(&bq25601_i2c_access);
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
		mutex_unlock(&bq25601_i2c_access);
		return 0;
	}
	*returnData = readData;

	mutex_unlock(&bq25601_i2c_access);
	return 1;
}

int bq25601_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;
	struct i2c_msg msg;
	struct i2c_adapter *adap = new_client->adapter;

	mutex_lock(&bq25601_i2c_access);
	write_data[0] = cmd;
	write_data[1] = writeData;
	msg.addr = new_client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = (char *)write_data;

	ret = i2c_transfer(adap, &msg, 1);
	if (ret < 0) {
		mutex_unlock(&bq25601_i2c_access);
		return 0;
	}

	mutex_unlock(&bq25601_i2c_access);
	return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int bq25601_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char bq25601_reg = 0;
	int ret = 0;

	battery_log(BAT_LOG_FULL, "--------------------------------------------------\n");

	ret = bq25601_read_byte(RegNum, &bq25601_reg);

	battery_log(BAT_LOG_FULL, "[bq25601_read_interface] Reg[%x]=0x%x\n", RegNum, bq25601_reg);

	bq25601_reg &= (MASK << SHIFT);
	*val = (bq25601_reg >> SHIFT);

	battery_log(BAT_LOG_FULL, "[bq25601_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int bq25601_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				    unsigned char SHIFT)
{
	unsigned char bq25601_reg = 0;
	int ret = 0;

	battery_log(BAT_LOG_FULL, "--------------------------------------------------\n");

	ret = bq25601_read_byte(RegNum, &bq25601_reg);
	battery_log(BAT_LOG_FULL, "[bq25601_config_interface] Reg[%x]=0x%x\n", RegNum, bq25601_reg);

	bq25601_reg &= ~(MASK << SHIFT);
	bq25601_reg |= (val << SHIFT);

	ret = bq25601_write_byte(RegNum, bq25601_reg);
	battery_log(BAT_LOG_FULL, "[bq25601_config_interface] write Reg[%x]=0x%x\n", RegNum, bq25601_reg);

	/* Check */
	/* bq25601_read_byte(RegNum, &bq25601_reg); */
	/* battery_log(BAT_LOG_FULL, "[bq25601_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq25601_reg); */

	return ret;
}

/* write one register directly */
unsigned int bq25601_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	unsigned int ret = 0;

	ret = bq25601_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0---------------------------------------------------- */
void bq25601_set_en_hiz(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_config_interface((unsigned char) (bq25601_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_HIZ_MASK),
				       (unsigned char) (CON0_EN_HIZ_SHIFT)
	    );
}

void bq25601_set_ichg_mon(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_config_interface((unsigned char) (bq25601_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_ICHG_MON_MASK),
				       (unsigned char) (CON0_ICHG_MON_SHIFT)
	    );
}

void bq25601_set_iindpm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_config_interface((unsigned char) (bq25601_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_IINDPM_MASK),
				       (unsigned char) (CON0_IINDPM_SHIFT)
	    );
}

/* CON1---------------------------------------------------- */
void bq25601_set_pfm_dis(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON1), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON1_PFM_DIS_MASK),
                                    (unsigned char)(CON1_PFM_DIS_SHIFT)
                                    );
}

void bq25601_set_wdt_rst(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON1), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON1_WDT_RST_MASK),
                                    (unsigned char)(CON1_WDT_RST_SHIFT)
                                    );
}

void bq25601_set_otg_config(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON1), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON1_OTG_CONFIG_MASK),
                                    (unsigned char)(CON1_OTG_CONFIG_SHIFT)
                                    );
}

void bq25601_set_chg_config(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON1), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON1_CHG_CONFIG_MASK),
                                    (unsigned char)(CON1_CHG_CONFIG_SHIFT)
                                    );
}

void bq25601_set_sys_min(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON1), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON1_SYS_MIN_MASK),
                                    (unsigned char)(CON1_SYS_MIN_SHIFT)
                                    );
}

void bq25601_set_min_vbat_sel(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON1), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON1_MIN_VBAT_SEL_MASK),
                                    (unsigned char)(CON1_MIN_VBAT_SEL_SHIFT)
                                    );
}

/* CON2---------------------------------------------------- */

void bq25601_set_boost_lim(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON2), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON2_BOOST_LIM_MASK),
                                    (unsigned char)(CON2_BOOST_LIM_SHIFT)
                                    );
}

void bq25601_set_q1_fullon(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON2), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON2_Q1_FULLON_MASK),
                                    (unsigned char)(CON2_Q1_FULLON_SHIFT)
                                    );
}

void bq25601_set_ichg(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON2), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON2_ICHG_MASK),
                                    (unsigned char)(CON2_ICHG_SHIFT)
                                    );
}

/* CON3---------------------------------------------------- */

void bq25601_set_iprechg(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON3), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON3_IPRECHG_MASK),
                                    (unsigned char)(CON3_IPRECHG_SHIFT)
                                    );
}

void bq25601_set_iterm(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON3), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON3_ITERM_MASK),
                                    (unsigned char)(CON3_ITERM_SHIFT)
                                    );
}

/* CON4---------------------------------------------------- */

void bq25601_set_vreg(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON4), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON4_VREG_MASK),
                                    (unsigned char)(CON4_VREG_SHIFT)
                                    );
}

void bq25601_set_topoff_timer(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON4), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON4_TOPOFF_TIMER_MASK),
                                    (unsigned char)(CON4_TOPOFF_TIMER_SHIFT)
                                    );
}

void bq25601_set_vrechg(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON4), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON4_VRECHG_MASK),
                                    (unsigned char)(CON4_VRECHG_SHIFT)
                                    );
}

/* CON5---------------------------------------------------- */

void bq25601_set_en_term(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON5), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON5_EN_TERM_MASK),
                                    (unsigned char)(CON5_EN_TERM_SHIFT)
                                    );
}

void bq25601_set_watchdog(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON5), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON5_WATCHDOG_MASK),
                                    (unsigned char)(CON5_WATCHDOG_SHIFT)
                                    );
}

void bq25601_set_en_timer(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON5), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON5_EN_TIMER_MASK),
                                    (unsigned char)(CON5_EN_TIMER_SHIFT)
                                    );
}

void bq25601_set_chg_timer(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON5), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON5_CHG_TIMER_MASK),
                                    (unsigned char)(CON5_CHG_TIMER_SHIFT)
                                    );
}

void bq25601_set_treg(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON5), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON5_TREG_MASK),
                                    (unsigned char)(CON5_TREG_SHIFT)
                                    );
}

void bq25601_set_jeita_iset(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON5), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON5_JEITA_ISET_MASK),
                                    (unsigned char)(CON5_JEITA_ISET_SHIFT)
                                    );
}

/* CON6---------------------------------------------------- */

void bq25601_set_ovp(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON6), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON6_OVP_MASK),
                                    (unsigned char)(CON6_OVP_SHIFT)
                                    );
}

void bq25601_set_boostv(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON6), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON6_BOOSTV_MASK),
                                    (unsigned char)(CON6_BOOSTV_SHIFT)
                                    );
}

void bq25601_set_vindpm(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON6), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON6_VINDPM_MASK),
                                    (unsigned char)(CON6_VINDPM_SHIFT)
                                    );
}

/* CON7---------------------------------------------------- */

void bq25601_set_iindet_en(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON7), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON7_IINDET_EN_MASK),
                                    (unsigned char)(CON7_IINDET_EN_SHIFT)
                                    );
}
void bq25601_set_tmr2x_en(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON7), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON7_TMR2X_EN_MASK),
                                    (unsigned char)(CON7_TMR2X_EN_SHIFT)
                                    );
}

void bq25601_set_batfet_disable(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON7), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON7_BATFET_Disable_MASK),
                                    (unsigned char)(CON7_BATFET_Disable_SHIFT)
                                    );
}

void bq25601_set_jeita_vset(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON7), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON7_JEITA_VSET_MASK),
                                    (unsigned char)(CON7_JEITA_VSET_SHIFT)
                                    );
}

void bq25601_set_batfet_dly(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON7), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON7_BATFET_DLY_MASK),
                                    (unsigned char)(CON7_BATFET_DLY_SHIFT)
                                    );
}

void bq25601_set_batfet_rst_en(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON7), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON7_BATFET_RST_EN_MASK),
                                    (unsigned char)(CON7_BATFET_RST_EN_SHIFT)
                                    );
}

void bq25601_set_vdpm_bat_track(unsigned int val)
{
    unsigned int ret=0;    

    ret=bq25601_config_interface(   (unsigned char)(bq25601_CON7), 
                                    (unsigned char)(val),
                                    (unsigned char)(CON7_VDPM_BAT_TRACK_MASK),
                                    (unsigned char)(CON7_VDPM_BAT_TRACK_SHIFT)
                                    );
}

/* CON8---------------------------------------------------- */

unsigned int bq25601_get_system_status_con8(void)
{
    unsigned int ret=0;
    unsigned char val=0;

    ret=bq25601_read_interface(     (unsigned char)(bq25601_CON8), 
                                    (&val),
                                    (unsigned char)(0xFF),
                                    (unsigned char)(0x0)
                                    );
	
    return val;
}

unsigned int bq25601_get_vbus_stat(void)
{
    unsigned int ret=0;
    unsigned char val=0;

    ret=bq25601_read_interface(     (unsigned char)(bq25601_CON8), 
                                    (&val),
                                    (unsigned char)(CON8_VBUS_STAT_MASK),
                                    (unsigned char)(CON8_VBUS_STAT_SHIFT)
                                    );
	
    return val;
}

unsigned int bq25601_get_chrg_stat(void)
{
    unsigned int ret=0;
    unsigned char val=0;

    ret=bq25601_read_interface(     (unsigned char)(bq25601_CON8), 
                                    (&val),
                                    (unsigned char)(CON8_CHRG_STAT_MASK),
                                    (unsigned char)(CON8_CHRG_STAT_SHIFT)
                                    );
	
    return val;
}

unsigned int bq25601_get_pg_stat(void)
{
    unsigned int ret=0;
    unsigned char val=0;

    ret=bq25601_read_interface(     (unsigned char)(bq25601_CON8), 
                                    (&val),
                                    (unsigned char)(CON8_PG_STAT_MASK),
                                    (unsigned char)(CON8_PG_STAT_SHIFT)
                                    );
    
    return val;
}

unsigned int bq25601_get_therm_stat(void)
{
    unsigned int ret=0;
    unsigned char val=0;

    ret=bq25601_read_interface(     (unsigned char)(bq25601_CON8), 
                                    (&val),
                                    (unsigned char)(CON8_THERM_STAT_MASK),
                                    (unsigned char)(CON8_THERM_STAT_SHIFT)
                                    );
    
    return val;
}


unsigned int bq25601_get_vsys_stat(void)
{
    unsigned int ret=0;
    unsigned char val=0;

    ret=bq25601_read_interface(     (unsigned char)(bq25601_CON8), 
                                    (&val),
                                    (unsigned char)(CON8_VSYS_STAT_MASK),
                                    (unsigned char)(CON8_VSYS_STAT_SHIFT)
                                    );
	
    return val;
}

/* CON9---------------------------------------------------- */

unsigned int bq25601_get_system_status_con9(void)
{
    unsigned int ret=0;
    unsigned char val=0;

    ret=bq25601_read_interface(     (unsigned char)(bq25601_CON9), 
                                    (&val),
                                    (unsigned char)(0xFF),
                                    (unsigned char)(0x0)
                                    );
    
    return val;
}

/* CON10---------------------------------------------------- */

unsigned int bq25601_get_system_status_con10(void)
{
    unsigned int ret=0;
    unsigned char val=0;

    ret=bq25601_read_interface(     (unsigned char)(bq25601_CON10), 
                                    (&val),
                                    (unsigned char)(0xFF),
                                    (unsigned char)(0x0)
                                    );
    
    return val;
}

void bq25601_set_vindpm_int_mask(unsigned int val)
{
    unsigned int ret=0;

    ret=bq25601_config_interface(     (unsigned char)(bq25601_CON10), 
                                    (val),
                                    (unsigned char)(CON10_VINDPM_INT_MASK),
                                    (unsigned char)(CON10_VINDPM_INT_SHIFT)
                                    );
    
}

void bq25601_set_iindpm_int_mask(unsigned int val)
{
    unsigned int ret=0;

    ret=bq25601_config_interface(     (unsigned char)(bq25601_CON10), 
                                    (val),
                                    (unsigned char)(CON10_IINDPM_INT_MASK),
                                    (unsigned char)(CON10_IINDPM_INT_SHIFT)
                                    );
    
}

/* CON11---------------------------------------------------- */
unsigned int bq25601_get_system_status_con11(void)
{
    unsigned int ret=0;
    unsigned char val=0;

    ret=bq25601_read_interface(     (unsigned char)(bq25601_CON11), 
                                    (&val),
                                    (unsigned char)(0xFF),
                                    (unsigned char)(0x0)
                                    );
    
    return val;
}

void bq25601_set_reg_rst(unsigned int val)
{
    unsigned int ret=0;

    ret=bq25601_config_interface(     (unsigned char)(bq25601_CON11), 
                                    (val),
                                    (unsigned char)(CON11_REG_RST_MASK),
                                    (unsigned char)(CON11_REG_RST_SHIFT)
                                    );
    
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
void bq25601_hw_component_detect(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq25601_read_interface(0x0B, &val, 0xF, 0x3);

	if (val == 0)
		g_bq25601_hw_exist = 0;
	else
		g_bq25601_hw_exist = 1;

	battery_log(BAT_LOG_CRTI, "[bq25601_hw_component_detect] exist=%d, Reg[0x0b][3-6]=0x%x\n", g_bq25601_hw_exist, val);
}

int is_bq25601_exist(void)
{
	battery_log(BAT_LOG_CRTI, "[is_bq25601_exist] g_bq25601_hw_exist=%d\n", g_bq25601_hw_exist);

	return g_bq25601_hw_exist;
}

void bq25601_dump_register(void)
{
	int i = 0;

	battery_log(BAT_LOG_FULL, "[bq25601] ");
	for (i = 0; i < bq25601_REG_NUM; i++) {
		bq25601_read_byte(i, &bq25601_reg[i]);
		battery_log(BAT_LOG_CRTI, "bq25601_reg[0x%x]=0x%x\n", i, bq25601_reg[i]);
	}
	battery_log(BAT_LOG_FULL, "\n");
}

#if 0
static void elink_charge_init(struct i2c_client *client)
{
#ifdef CONFIG_OF
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_drvvbus_low;

	/* check customer setting */
	pinctrl = devm_pinctrl_get(&new_client->dev);
	if (IS_ERR(pinctrl)) {
		battery_log(BAT_LOG_CRTI, "[%s]Cannot find drvvbus pinctrl, err=%d\n",
			__func__, (int)PTR_ERR(pinctrl));
		return;
	}

	pinctrl_drvvbus_low = pinctrl_lookup_state(pinctrl, "drvvbus_low");
	if (IS_ERR(pinctrl_drvvbus_low)) {
		battery_log(BAT_LOG_CRTI, "[%s]Cannot find drvvbus_low state, err=%d\n",
			__func__, (int)PTR_ERR(pinctrl_drvvbus_low));
		return;
	}

	pinctrl_select_state(pinctrl, pinctrl_drvvbus_low);
	
	devm_pinctrl_put(pinctrl);

	battery_log(BAT_LOG_FULL, "[%s]pinctrl_select_state success\n", __func__);
#endif
}
#endif

static void bq25601_parse_customer_setting(void)
{
#ifdef CONFIG_OF
    struct pinctrl *pinctrl;
    struct pinctrl_state *ptrl_chren_low;

    /* check customer setting */
    pinctrl = devm_pinctrl_get(&new_client->dev);
    if (IS_ERR(pinctrl)) {
        battery_log(BAT_LOG_CRTI, "[%s]Cannot find drvvbus pinctrl, err=%d\n",
            __func__, (int)PTR_ERR(pinctrl));
        return;
    }

    ptrl_chren_low = pinctrl_lookup_state(pinctrl, "chren_low");
    if (IS_ERR(ptrl_chren_low)) {
        battery_log(BAT_LOG_CRTI, "[%s]Cannot find chren_low state, err=%d\n",
            __func__, (int)PTR_ERR(ptrl_chren_low));
        return;
    }
    pinctrl_select_state(pinctrl, ptrl_chren_low);

    devm_pinctrl_put(pinctrl);

    battery_log(BAT_LOG_CRTI, "[%s]pinctrl_select_state success\n", __func__);
#endif
}

static int bq25601_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	battery_log(BAT_LOG_CRTI, "[bq25601_driver_probe]\n");
	
	client->addr = 0x6b;

	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);

	if (!new_client) {
		err = -ENOMEM;
		goto exit;
	}
	memset(new_client, 0, sizeof(struct i2c_client));

	new_client = client;
	
	//elink_charge_init(client);

	/* --------------------- */
	bq25601_hw_component_detect();
	bq25601_dump_register();
	chargin_hw_init_done = KAL_TRUE;

    bq25601_parse_customer_setting();

	return 0;

exit:
	return err;

}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_bq25601 = 0;
static ssize_t show_bq25601_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "[show_bq25601_access] 0x%x\n", g_reg_value_bq25601);
	return sprintf(buf, "%u\n", g_reg_value_bq25601);
}

static ssize_t store_bq25601_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	battery_log(BAT_LOG_CRTI, "[store_bq25601_access]\n");

	if (buf != NULL && size != 0) {
		battery_log(BAT_LOG_CRTI, "[store_bq25601_access] buf is %s and size is %zu\n", buf, size);
		/*reg_address = kstrtoul(buf, 16, &pvalue);*/

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16, (unsigned int *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16, (unsigned int *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (unsigned int *)&reg_value);

			battery_log(BAT_LOG_CRTI,
			    "[store_bq25601_access] write bq25601 reg 0x%x with value 0x%x !\n",
			     reg_address, reg_value);
			ret = bq25601_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = bq25601_read_interface(reg_address, &g_reg_value_bq25601, 0xFF, 0x0);
			battery_log(BAT_LOG_CRTI,
			    "[store_bq25601_access] read bq25601 reg 0x%x with value 0x%x !\n",
			     reg_address, g_reg_value_bq25601);
			battery_log(BAT_LOG_CRTI,
			    "[store_bq25601_access] Please use \"cat bq25601_access\" to get value\r\n");
		}
	}
	return size;
}

static DEVICE_ATTR(bq25601_access, 0664, show_bq25601_access, store_bq25601_access);	/* 664 */

static int bq25601_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	battery_log(BAT_LOG_CRTI, "******** bq25601_user_space_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq25601_access);

	return 0;
}

struct platform_device bq25601_user_space_device = {
	.name = "bq25601-user",
	.id = -1,
};

static struct platform_driver bq25601_user_space_driver = {
	.probe = bq25601_user_space_probe,
	.driver = {
		   .name = "bq25601-user",
		   },
};

static int __init bq25601_subsys_init(void)
{
	int ret = 0;

	if (i2c_add_driver(&bq25601_driver) != 0)
		battery_log(BAT_LOG_CRTI, "[bq24261_init] failed to register bq24261 i2c driver.\n");
	else
		battery_log(BAT_LOG_CRTI, "[bq24261_init] Success to register bq24261 i2c driver.\n");

	/* bq25601 user space access interface */
	ret = platform_device_register(&bq25601_user_space_device);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[bq25601_init] Unable to device register(%d)\n", ret);
		return ret;
	}
	ret = platform_driver_register(&bq25601_user_space_driver);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[bq25601_init] Unable to register driver (%d)\n", ret);
		return ret;
	}

	return 0;
}

static void __exit bq25601_exit(void)
{
	i2c_del_driver(&bq25601_driver);
}

/* module_init(bq25601_init); */
/* module_exit(bq25601_exit); */
subsys_initcall(bq25601_subsys_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq25601 Driver");
MODULE_AUTHOR("Matt yin<yingen@elinktek.com>");
