
/*****************************************************************************
*
* Filename:
* ---------
*   bq2601.h
*
* Project:
* --------
*   Android
*
* Description:
* ------------
*   bq25601 header file
*
* Author: Matt(yingen@elinktek.com)
* -------
*
****************************************************************************/

#ifndef _bq25601_SW_H_
#define _bq25601_SW_H_


#define bq25601_CON0      0x00
#define bq25601_CON1      0x01
#define bq25601_CON2      0x02
#define bq25601_CON3      0x03
#define bq25601_CON4      0x04
#define bq25601_CON5      0x05
#define bq25601_CON6      0x06
#define bq25601_CON7      0x07
#define bq25601_CON8      0x08
#define bq25601_CON9      0x09
#define bq25601_CON10      0x0A
#define bq25601_CON11      0x0B
#define bq25601_REG_NUM 12

/**********************************************************
  *
  *   [MASK/SHIFT] 
  *
  *********************************************************/
//CON0
#define CON0_EN_HIZ_MASK   0x01
#define CON0_EN_HIZ_SHIFT  7

#define CON0_ICHG_MON_MASK       0x03
#define CON0_ICHG_MON_SHIFT      5

#define CON0_IINDPM_MASK       0x1F
#define CON0_IINDPM_SHIFT      0

//CON1
#define CON1_PFM_DIS_MASK     0x01
#define CON1_PFM_DIS_SHIFT    7

#define CON1_WDT_RST_MASK     0x01
#define CON1_WDT_RST_SHIFT    6

#define CON1_OTG_CONFIG_MASK        0x01
#define CON1_OTG_CONFIG_SHIFT       5

#define CON1_CHG_CONFIG_MASK        0x01
#define CON1_CHG_CONFIG_SHIFT       4

#define CON1_SYS_MIN_MASK        0x07
#define CON1_SYS_MIN_SHIFT       1

#define CON1_MIN_VBAT_SEL_MASK   0x01
#define CON1_MIN_VBAT_SEL_SHIFT  0

/* CON2 */
#define CON2_BOOST_LIM_MASK    0x01
#define CON2_BOOST_LIM_SHIFT   7

#define CON2_Q1_FULLON_MASK    0x01
#define CON2_Q1_FULLON_SHIFT   6

#define CON2_ICHG_MASK    0x3F
#define CON2_ICHG_SHIFT   0

//CON3
#define CON3_IPRECHG_MASK   0x0F
#define CON3_IPRECHG_SHIFT  4

#define CON3_ITERM_MASK           0x0F
#define CON3_ITERM_SHIFT          0

//CON4
#define CON4_VREG_MASK     0x1F
#define CON4_VREG_SHIFT    3

#define CON4_TOPOFF_TIMER_MASK     0x03
#define CON4_TOPOFF_TIMER_SHIFT    1

#define CON4_VRECHG_MASK    0x01
#define CON4_VRECHG_SHIFT   0

//CON5
#define CON5_EN_TERM_MASK      0x01
#define CON5_EN_TERM_SHIFT     7

#define CON5_WATCHDOG_MASK     0x03
#define CON5_WATCHDOG_SHIFT    4

#define CON5_EN_TIMER_MASK      0x01
#define CON5_EN_TIMER_SHIFT     3

#define CON5_CHG_TIMER_MASK           0x01
#define CON5_CHG_TIMER_SHIFT          2

#define CON5_TREG_MASK           0x01
#define CON5_TREG_SHIFT          1

#define CON5_JEITA_ISET_MASK           0x01
#define CON5_JEITA_ISET_SHIFT          0

//CON6
#define CON6_OVP_MASK     0x03
#define CON6_OVP_SHIFT    6

#define CON6_BOOSTV_MASK     0x03
#define CON6_BOOSTV_SHIFT    4

#define CON6_VINDPM_MASK     0x0F
#define CON6_VINDPM_SHIFT    0

//CON7
#define CON7_IINDET_EN_MASK      0x01
#define CON7_IINDET_EN_SHIFT     7

#define CON7_TMR2X_EN_MASK      0x01
#define CON7_TMR2X_EN_SHIFT     6

#define CON7_BATFET_Disable_MASK      0x01
#define CON7_BATFET_Disable_SHIFT     5

#define CON7_JEITA_VSET_MASK      0x01
#define CON7_JEITA_VSET_SHIFT     4

#define CON7_BATFET_DLY_MASK      0x01
#define CON7_BATFET_DLY_SHIFT     3

#define CON7_BATFET_RST_EN_MASK      0x01
#define CON7_BATFET_RST_EN_SHIFT     2

#define CON7_VDPM_BAT_TRACK_MASK     0x03
#define CON7_VDPM_BAT_TRACK_SHIFT    0

//CON8
#define CON8_VBUS_STAT_MASK      0x07
#define CON8_VBUS_STAT_SHIFT     5

#define CON8_CHRG_STAT_MASK           0x03
#define CON8_CHRG_STAT_SHIFT          3

#define CON8_PG_STAT_MASK           0x01
#define CON8_PG_STAT_SHIFT          2

#define CON8_THERM_STAT_MASK           0x01
#define CON8_THERM_STAT_SHIFT          1

#define CON8_VSYS_STAT_MASK           0x01
#define CON8_VSYS_STAT_SHIFT          0

//CON9
#define CON9_WATCHDOG_FAULT_MASK      0x01
#define CON9_WATCHDOG_FAULT_SHIFT     7

#define CON9_OTG_FAULT_MASK           0x01
#define CON9_OTG_FAULT_SHIFT          6

#define CON9_CHRG_FAULT_MASK           0x03
#define CON9_CHRG_FAULT_SHIFT          4

#define CON9_BAT_FAULT_MASK           0x01
#define CON9_BAT_FAULT_SHIFT          3

#define CON9_NTC_FAULT_MASK           0x07
#define CON9_NTC_FAULT_SHIFT          0

//CON10
#define CON10_VBUS_GD_MASK      0x01
#define CON10_VBUS_GD_SHIFT     7

#define CON10_VINDPM_STAT_MASK      0x01
#define CON10_VINDPM_STAT_SHIFT     6

#define CON10_IINDPM_STAT_MASK      0x01
#define CON10_IINDPM_STAT_SHIFT     5

#define CON10_TOPOFF_ACTIVE_MASK      0x01
#define CON10_TOPOFF_ACTIVE_SHIFT     3

#define CON10_ACOV_STAT_MASK      0x01
#define CON10_ACOV_STAT_SHIFT     2

#define CON10_VINDPM_INT_MASK      0x01
#define CON10_VINDPM_INT_SHIFT     1

#define CON10_IINDPM_INT_MASK      0x01
#define CON10_IINDPM_INT_SHIFT     0

//CON11
#define CON11_REG_RST_MASK      0x01
#define CON11_REG_RST_SHIFT     7

#define CON11_PN_MASK      0x0F
#define CON11_PN_SHIFT     3

#define CON11_DEV_REV_MASK      0x03
#define CON11_DEV_REV_SHIFT     0

/**********************************************************
  *
  *   [Extern Function] 
  *
  *********************************************************/
//CON0----------------------------------------------------
extern void bq25601_set_en_hiz(unsigned int val);
extern void bq25601_set_ichg_mon(unsigned int val);
extern void bq25601_set_iindpm(unsigned int val);
//CON1----------------------------------------------------
extern void bq25601_set_pfm_dis(unsigned int val);
extern void bq25601_set_wdt_rst(unsigned int val);
extern void bq25601_set_otg_config(unsigned int val);
extern void bq25601_set_chg_config(unsigned int val);
extern void bq25601_set_sys_min(unsigned int val);
extern void bq25601_set_min_vbat_sel(unsigned int val);
//CON2----------------------------------------------------
extern void bq25601_set_boost_lim(unsigned int val);
extern void bq25601_set_q1_fullon(unsigned int val);
extern void bq25601_set_ichg(unsigned int val);
//CON3----------------------------------------------------
extern void bq25601_set_iprechg(unsigned int val);
extern void bq25601_set_iterm(unsigned int val);
//CON4----------------------------------------------------
extern void bq25601_set_vreg(unsigned int val);
extern void bq25601_set_topoff_timer(unsigned int val);
extern void bq25601_set_vrechg(unsigned int val);
//CON5----------------------------------------------------
extern void bq25601_set_en_term(unsigned int val);
extern void bq25601_set_watchdog(unsigned int val);
extern void bq25601_set_en_timer(unsigned int val);
extern void bq25601_set_chg_timer(unsigned int val);
extern void bq25601_set_treg(unsigned int val);
extern void bq25601_set_jeita_iset(unsigned int val);
//CON6----------------------------------------------------
extern void bq25601_set_ovp(unsigned int val);
extern void bq25601_set_boostv(unsigned int val);
extern void bq25601_set_vindpm(unsigned int val);
//CON7----------------------------------------------------
extern void bq25601_set_iindet_en(unsigned int val);
extern void bq25601_set_tmr2x_en(unsigned int val);
extern void bq25601_set_batfet_disable(unsigned int val);
extern void bq25601_set_jeita_vset(unsigned int val);
extern void bq25601_set_batfet_dly(unsigned int val);
extern void bq25601_set_batfet_rst_en(unsigned int val);
extern void bq25601_set_vdpm_bat_track(unsigned int val);
//CON8----------------------------------------------------
extern unsigned int bq25601_get_system_status_con8(void);
extern unsigned int bq25601_get_vbus_stat(void);
extern unsigned int bq25601_get_chrg_stat(void);
extern unsigned int bq25601_get_pg_stat(void);
extern unsigned int bq25601_get_therm_stat(void);
extern unsigned int bq25601_get_vsys_stat(void);
//CON9----------------------------------------------------
extern unsigned int bq25601_get_system_status_con9(void);
//CON10---------------------------------------------------
extern unsigned int bq25601_get_system_status_con10(void);
extern void bq25601_set_vindpm_int_mask(unsigned int val);
extern void bq25601_set_iindpm_int_mask(unsigned int val);
//CON11---------------------------------------------------
extern unsigned int bq25601_get_system_status_con11(void);
extern void bq25601_set_reg_rst(unsigned int val);
//---------------------------------------------------------

extern void bq25601_dump_register(void);
extern unsigned int bq25601_reg_config_interface(unsigned char RegNum, unsigned char val);

extern unsigned int bq25601_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
					 unsigned char SHIFT);
extern unsigned int bq25601_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
					   unsigned char SHIFT);
#endif // _bq25601_SW_H_

