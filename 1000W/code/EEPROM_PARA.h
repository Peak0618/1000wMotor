#ifndef __EEPROM_PARA_H__
#define __EEPROM_PARA_H__
//-----------------------------------------------------------------------------
#include "user_typedefine.h"
//-----------------------------------------------------------------------------
#define   VAL_ROW        0
#define   MAX_ROW        1
#define   MIN_ROW        2
#define   KEYRATE_ROW    3
#define   DISPRATE_ROW   4
//-----------------------------------------------------------------------------
//定义定义小数点位置的常量
#define   NOT_DOT        0    //无小数点
#define   DOT_AT_0D1     1    //小数点在十位  0.1
#define   DOT_AT_0D01    2    //小数点在百位  0.01
#define   DOT_AT_0D001   3    //小数点在千位  0.001
#define   DOT_AT_0D0001  4    //小数点在万位  0.0001
//------------------------------------
//定义校验字
#define   VERIFY_WORD    0x55AA
//-----------------------------------------------------------------------------
//定义最大参数数量
#define   MAX_PARA_NUMBER		110//debug 
//-----------------------------------------------------------------------------
#define   num_pr_set_method             0
#define   num_cmd_source                1
#define   num_ref_freq                  2
#define   num_direction                 3
//-----------------------------------------------------------------------------
#define   num_accel_time                4
#define   num_decel_time                5
//-----------------------------------------------------------------------------
#define   num_min_freq                  6
#define   num_max_freq                  7
 
#define   num_begin_freq                8
#define   num_shut_freq                 9
//-----------------------------------------------------------------------------
#define   num_trip_stop_cnt             10
#define   num_stop_space_delaytime      11
#define   num_trip_lock_delaytime       12
//-----------------------------------------------------------------------------
#define   num_Iout_CT_zero_AD           13
#define   num_Iout_CT_zero_margin       14

#define   num_THM_lowlim_AD             15
#define   num_THM_uplim_AD              16
//-----------------------------------------------------------------------------
#define   num_prot_ALM_cnt              17

#define   num_prot_Iout_POC             18
#define   num_prot_Iout_POC_delaycnt    19

#define   num_Iout_rate                 20

#define   num_prot_Iout_imbal           21
#define   num_prot_Iout_imbal_delaytime 22

#define   num_prot_Iout_NC              23
#define   num_prot_Iout_NC_delaytime    24

#define   num_prot_Iout_OC              25
#define   num_prot_Iout_OC_delaytime    26

#define   num_prot_Iout_OL              27
#define   num_prot_Iout_OL_delaytime    28
#define   num_prot_Iout_OL_enter        29

#define   num_prot_Iout_decfreq_enter             30
#define   num_prot_Iout_decfreq_enter_delaytime   31
#define   num_prot_Iout_decfreq_deltfreq          32
#define   num_prot_Iout_decfreq_quit              33
#define   num_prot_Iout_decfreq_quit_delaytime    34

#define   num_prot_Iout_stall_enter               35
#define   num_prot_Iout_stall_enter_delaytime     36
#define   num_prot_Iout_stall_quit                37
#define   num_prot_Iout_stall_quit_delaytime      38
#define   num_prot_Iout_stall_amp                 39 
//------------------------------------------------------------------------------
#define   num_Tm_enbale                           40

#define   num_prot_Tm_OH                          41
#define   num_prot_Tm_OH_delaytime                42

#define   num_prot_Tm_decfreq_enter               43
#define   num_prot_Tm_decfreq_delaytime           44
#define   num_prot_Tm_decfreq_deltfreq            45
#define   num_prot_Tm_decfreq_quit                46
#define   num_prot_Tm_decfreq_quit_delaytime      47
 
#define   num_prot_Tm_stall_enter                 48
#define   num_prot_Tm_stall_enter_delaytime       49
#define   num_prot_Tm_stall_quit                  50
#define   num_prot_Tm_stall_quit_delaytime        51
#define   num_prot_Tm_stall_amp                   52
//------------------------------------------------------------------------------
#define   num_prot_Udc_POU              53
#define   num_prot_Udc_POU_delaycnt     54

#define   num_prot_Udc_OU_enter         55
#define   num_prot_Udc_OU_quit          56

#define   num_prot_Udc_stall_enter      57
#define   num_prot_Udc_stall_quit       58

#define   num_prot_Udc_LU_enter         59
#define   num_prot_Udc_LU_quit          60

#define   num_prot_Udc_stall_amp        61
//------------------------------------------------------------------------------
#define   num_motor_overspeed           62

#define   num_motor_stall               63
#define   num_motor_stall_delaytime     64
//------------------------------------------------------------------------------
#define   num_Iout_gain                 65
#define   num_Iout_gain_amp             66
#define   num_Iout_ref                  67
#define   num_Iout_ref_AD               68

#define   num_Udc_gain                  69
#define   num_Udc_gain_amp              70
#define   num_Udc_ref                   71
#define   num_Udc_ref_AD                72

#define   num_THM_ref                   73
#define   num_THF_ref                   74
//------------------------------------------------------------------------------
#define   num_Trip_clear                75
#define   num_Trip_cause1               76
#define   num_Trip_cause2               77
#define   num_Trip_cause3               78
#define   num_Trip_cause4               79
#define   num_Trip_cause5               80
//------------------------------------------------------------------------------
#define   num_drive_mode                81
//------------------------------------------------------------------------------
#define   num_max_k_set                 82
#define   num_pwm_carrier               83
#define   num_IPM_min_time              84
#define   num_dead_time                 85
#define   num_DTrepair                  86
#define   num_ADrepair_time             87
#define   num_AD_SH_time                88
//------------------------------------------------------------------------------
#define   num_Uin_rate                  89
//------------------------------------------------------------------------------
#define   num_PIctrl_delaytime          90
#define   num_Ksp                       91
#define   num_Ksi                       92
#define   num_Integral_initval          93
#define   num_V_max                     94
//------------------------------------------------------------------------------
#define   num_PhaseAdvStartFreq         95
#define   num_PhaseAdvSlope             96
#define   num_phase_max_degree          97

//------------------------------------------------------------------------------
#define   num_MOTOR_p                   98
#define   num_MOTOR_R                   99
#define   num_MOTOR_Ld                  100
#define   num_MOTOR_Lq                  101
#define   num_MOTOR_EMF                 102
#define   num_MOTOR_max_n               103
#define   num_MOTOR_CCW                 104
#define   num_MOTOR_CW                  105

#define   num_selftest                  106
#define   num_App_ID                    107
#define   num_soft_ver                  108
#define   num_drive_letter              109
#define   num_verify_word               110
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
extern const int16_t EEPROM_InitData[MAX_PARA_NUMBER + 1][5];

extern int16_t ram_para[MAX_PARA_NUMBER + 1]; 


//------------------------------------------------------------------------------
/*/应用ID参数定义
#define   MOTOR_accel_time     0           
#define   MOTOR_decel_time     1           		
#define   MOTOR_min_freq   	   2
#define   MOTOR_max_freq   		 3
#define   MOTOR_begin_freq     4     
#define   MOTOR_shut_freq      5
#define   MOTOR_trip_stop_cnt  6       	
#define   MOTOR_stop_space_delaytime  7 
#define   MOTOR_Iout_rate             8    
//-------------------模块温度保护参数-----------------------------------------
#define   MOTOR_prot_Tm_OH            9     //THM过热保护进入温度点    unit: 1℃   
#define   MOTOR_prot_Tm_decfreq_enter 10    //THM过热降频进入温度点   unit: 1℃   
#define   MOTOR_prot_Tm_decfreq_quit  11    //THM过热降频退出温度点   unit: 1℃  	
#define   MOTOR_prot_Tm_stall_enter   12    //THM过热失速进入温度点   unit: 1℃   	
#define   MOTOR_prot_Tm_stall_quit    13    //THM过热失速退出温度点   unit: 1℃  
//-------------------电机保护参数------------------------------------------
#define   MOTOR_motor_overspeed       14    //电机超速百分比 unit: 0.1% 
//-------------------驱动控制方法相关的参数--------------------------------
#define   MOTOR_drive_mode            15    //驱动方式选择  0:SVPWM    1:台形波  
#define   MOTOR_pwm_carrier           16    //PWM载波频率(精确到0.1kHz)  
//-------------------电机转速PI控制-----------------------------------------
#define   MOTOR_PIctrl_delaytime      17    //PI控制周期  //EE 
#define   MOTOR_Ksp                   18    //转速环比例系数(单位：0.001V/0.01Hz/s)扩大2^15*2^8   
#define   MOTOR_Ksi                   19    //转速环积分系数(单位：0.001V/0.01Hz/s)扩大2^15*2^8  
#define   MOTOR_Integral_initval      20    //积分项初值(0.1V)  //EE
#define   MOTOR_V_max                 21    //输出线电压最大有效值(0.1V) (作积分限值)  
//-------------------相位超前控制-------------------------------------------
#define   MOTOR_PhaseAdvStartFreq     22
#define   MOTOR_PhaseAdvSlope         23 
#define   MOTOR_phase_max_degree      24 

//--------------------------------------------------------------------------
#define   MOTOR_p               25 
#define   MOTOR_R               26
#define   MOTOR_Ld              27
#define   MOTOR_Lq              28
#define   MOTOR_EMF             29  
#define   MOTOR_max_n           30
#define   MOTOR_CCW             31 
#define   MOTOR_CW              32*/
//--------------------------------------
//------------------------------------------------------------------------------
#define  APP_ID_MIN      32
#define  APP_ID_MAX      40

#define  APP_ID_OFFSET    APP_ID_MIN    //ID号寻址偏移量

#define APP_PARA_CNT     33       //共33个需重写的参数
//------------------------------------------------------------------------------
extern const uint16_t App_index[];

extern const int16_t App_para[][APP_PARA_CNT];

//------------------------------------------------------------------------------
#endif     /*__ROM_PARA_H__ */
