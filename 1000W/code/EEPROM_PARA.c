#ifndef __ROM_PARA_C__
#define __ROM_PARA_C__
//------------------------------------------------------------------------------
#include "user_typedefine.h"
#include "EEPROM_PARA.h"
//------------------------------------------------------------------------------
const int16_t EEPROM_InitData[MAX_PARA_NUMBER+1][5]=           //出厂参数
{
//    val     max     min    keyrate    disprate
 {     0,      2,      0,       1,       NOT_DOT},     //No.000  num_pr_set_method  peak 1-->0
 {     1,      1,      0,       1,       NOT_DOT},     //No.001  num_cmd_source
 {  2000,  30000,      0,     100,   DOT_AT_0D01},     //No.002  num_ref_freq
 {     0,      1,      0,       1,       NOT_DOT},     //No.003  num_direction  0：逆时针  1：顺时针 peak 1-->0应该是对的
   
 {   150,   6000,      1,       1,    DOT_AT_0D1},     //No.004  num_accel_time (用于开机时的加速时间)  //EE
 {   150,   6000,      1,       1,    DOT_AT_0D1},     //No.005  num_decel_time (用于停机时的减速减速)  //EE
//----------------------------------------------------------------------------------------------------------------------
//频率参数
//    val     max     min    keyrate    disprate
 {  1000,  30000,      0,      10,   DOT_AT_0D01},     //No.006  num_min_freq  //EE
 { 15000,  15000,      0,      10,   DOT_AT_0D01},     //No.007  num_max_freq  //EE
 
 {     0,  30000,      0,      10,   DOT_AT_0D01},     //No.008  num_begin_freq  //EE
 //
 { 15000,  30000,      0,      10,   DOT_AT_0D01},     //No.009  num_shut_freq   //EE
//----------------------------------------------------------------------------------------------------------------------
//保护停机参数
//    val     max     min    keyrate    disprate
 {     6,    100,      0,       1,       NOT_DOT},     //No.010  num_trip_stop_cnt        保护重启次数  //EE

 {    30,   6000,      0,        1,     NOT_DOT},   //No.011  num_stop_space_delaytime 上电或重启延时s  //EE
 {    60,    600,      3,       1,       NOT_DOT},     //No.012  num_trip_lock_delaytime  故障锁定延时时间min
//----------------------------------------------------------------------------------------------------------------------
//检测电路保护参数
//    val     max     min    keyrate    disprate
 {   512,   1023,      0,       1,       NOT_DOT},     //No.013  num_Iout_CT_zero_AD      //输出电流Iout的CT零点对应的AD值
 {    50,    512,      0,       1,       NOT_DOT},     //No.014  num_Iout_CT_zero_margin  //输出电流Iout的CT零点的偏移范围

 {     0,   1023,      0,       1,       NOT_DOT},     //No.015  num_THM_lowlim_AD        //散热器温度检测下限AD值
 {  1010,   1023,      0,       1,       NOT_DOT},     //No.016  num_THM_uplim_AD         //散热器温度检测上限AD值
//----------------------------------------------------------------------------------------------------------------------
//输出电流保护参数
//   val     max     min    keyrate    disprate
 {     0,    100,      0,       1,       NOT_DOT},     //No.017  num_prot_ALM_cnt               //输出过流短路保护检查次数

 {  1500,   2000,      0,       1,    DOT_AT_0D1},     //No.018  num_prot_Iout_POC              //输出电流Iout软件瞬时过流保护峰值 Unit:0.01A
 {     3,    100,      0,       1,       NOT_DOT},     //No.019  num_prot_Iout_POC_delaycnt     //输出电流Iout瞬时过流保护检查次数

 {   900,   1000,      0,       1,    DOT_AT_0D1},     //No.020  num_Iout_rate                  //额定输出电流 Unit:0.01A //EE

 {   100,   1000,      0,      10,    DOT_AT_0D1},     //No.021  num_prot_Iout_imbal            //输出三相不平衡率  unit: 0.1%
 {    50,   3000,      0,       1,    DOT_AT_0D1},     //No.022  num_prot_Iout_imbal_delaytime  //输出三相不平持续时间 unit: 0.1S

 {     0,   1000,      0,      10,    DOT_AT_0D1},     //No.023  num_prot_Iout_NC               //输出缺相率 unit: 0.1%
 {    30,   3000,      0,       1,    DOT_AT_0D1},     //No.024  num_prot_Iout_NC_delaytime     //输出缺相持续时间 unit: 0.1S

 {  1100,   2000,      0,      10,    DOT_AT_0D1},     //No.025  num_prot_Iout_OC               //输出过流保护的百分比 unit: 0.1%
 {    30,   3000,      0,       1,    DOT_AT_0D1},     //No.026  num_prot_Iout_OC_delaytime     //输出过流保护持续时间 unit: 0.1S

 {  1100,   2000,      0,      10,    DOT_AT_0D1},     //No.027  num_prot_Iout_OL               //输出过载百分比   unit: 0.1%
 {   300,   3000,      0,       1,    DOT_AT_0D1},     //No.028  num_prot_Iout_OL_delaytime     //输出过载持续时间 unit: 0.1S
 {  1040,   2000,      0,      10,    DOT_AT_0D1},     //No.029  num_prot_Iout_OL_enter         //开始输出过载百分比   unit: 0.1%

 {  1000,   2000,      0,      10,    DOT_AT_0D1},     //No.030  num_prot_Iout_decfreq_enter              //输出过流降频进入百分比   unit: 0.1%
 {    10,   3000,      0,       1,    DOT_AT_0D1},     //No.031  num_prot_Iout_decfreq_enter_delaytime    //输出过流降频持续时间     unit: 0.1S
 {   400,    500,      0,      10,   DOT_AT_0D01},     //No.032  num_prot_Iout_decfreq_deltfreq           //输出过流降频步长         unit: 0.01Hz 
 {   920,   2000,      0,      10,    DOT_AT_0D1},     //No.033  num_prot_Iout_decfreq_quit               //输出过流降频退出百分比   unit: 0.1%
 {    30,   3000,      0,       1,    DOT_AT_0D1},     //No.034  num_prot_Iout_decfreq_quit_delaytime     //输出过流降频退出持续时间 unit: 0.1S

 {   900,   2000,      0,      10,    DOT_AT_0D1},     //No.035  num_prot_Iout_stall_enter                //输出过流失速进入百分比   unit: 0.1%
 {    10,   3000,      0,       1,    DOT_AT_0D1},     //No.036  num_prot_Iout_stall_enter_delaytime      //输出过流失速进入持续时间 unit: 0.1S
 {   860,   2000,      0,      10,    DOT_AT_0D1},     //No.037  num_prot_Iout_stall_quit                 //输出过流失速退出百分比   unit: 0.1%
 {    20,   3000,      0,       1,    DOT_AT_0D1},     //No.038  num_prot_Iout_stall_quit_delaytime       //输出过流失速退出持续时间 unit: 0.1S
 {     3,      4,      0,       1,       NOT_DOT},     //No.039  num_prot_Iout_stall_amp                  //输出过流失速倍率         unit: 2^n次幂

//----------------------------------------------------------------------------------------------------------------------
//模块温度保护参数
//   val     max     min    keyrate    disprate
 {     1,      1,      0,       1,       NOT_DOT},     //No.040  num_Tm_enbale                 //THM温度传感器使能选择      0：不使能  1：使能

 {    95,    140,    -20,       1,       NOT_DOT},     //No.041  num_prot_Tm_OH                //THM过热保护进入温度点    unit: 1℃  //EE
 {    10,   3000,      0,       1,    DOT_AT_0D1},     //No.042  num_prot_Tm_OH_delaytime      //THM过热保护进入持续时间  unit: 0.1S 
 {    90,    140,    -20,       1,       NOT_DOT},     //No.043  num_prot_Tm_decfreq_enter     //THM过热降频进入温度点   unit: 1℃  //EE
 {    30,   3000,      0,       1,    DOT_AT_0D1},     //No.044  num_prot_Tm_decfreq_delaytime //THM过热降频持续时间     unit: 0.1S
 {   200,   8000,      0,      10,   DOT_AT_0D01},     //No.045  num_prot_Tm_decfreq_deltfreq  //THM过热降频步长         unit: 0.01Hz 
 {    85,    140,    -20,       1,       NOT_DOT},     //No.046  num_prot_Tm_decfreq_quit      //THM过热降频退出温度点   unit: 1℃  //EE
 {    20,   3000,      0,       1,    DOT_AT_0D1},     //No.047  num_prot_Tm_decfreq_quit_delaytime //THM过热降频退出持续时间 unit: 0.1S
 {    85,    140,    -20,       1,       NOT_DOT},     //No.048  num_prot_Tm_stall_enter       //THM过热失速进入温度点   unit: 1℃  //EE
 {    20,   3000,      0,       1,    DOT_AT_0D1},     //No.049  num_prot_Tm_stall_enter_delaytime  //THM过热失速进入持续时间 unit: 0.1S
 {    80,    140,    -20,       1,       NOT_DOT},     //No.050  num_prot_Tm_stall_quit        //THM过热失速退出温度点   unit: 1℃  //EE
 {    20,   3000,      0,       1,    DOT_AT_0D1},     //No.051  num_prot_Tm_stall_quit_delaytime   //THM过热失速退出持续时间 unit: 0.1S
 {     3,      4,      0,       1,       NOT_DOT},     //No.052  num_prot_Tm_stall_amp         //THM过热失速倍率         unit: 2^n次幂

//----------------------------------------------------------------------------------------------------------------------
//母线电压保护参数
//   val     max     min    keyrate    disprate
 {   410,   1000,      0,       1,       NOT_DOT},     //No.053  num_prot_Udc_POU              //母线电压软件瞬时过压保护峰值
 {     3,    100,      0,       1,       NOT_DOT},     //No.054  num_prot_Udc_POU_delaycnt     //母线电压瞬时过压保护检查次数

 {   400,   1000,      0,       1,       NOT_DOT},     //No.055  num_prot_Udc_OU_enter         //母线电压过压进入电压值  unit: 1V
 {   376,   1000,      0,       1,       NOT_DOT},     //No.056  num_prot_Udc_OU_quit          //母线电压过压退出电压值  unit: 1V

 {   383,   1000,      0,       1,       NOT_DOT},     //No.057  num_prot_Udc_stall_enter      //母线电压失速进入电压值   unit: 1V
 {   373,   1000,      0,       1,       NOT_DOT},     //No.058  num_prot_Udc_stall_quit       //母线电压失速退出电压值   unit: 1V

 {   194,   1000,      0,       1,       NOT_DOT},     //No.059  num_prot_Udc_LU_enter         //母线电压欠压进入电压值   unit: 1V
 {   218,   1000,      0,       1,       NOT_DOT},     //No.060  num_prot_Udc_LU_quit          //母线电压欠压退出电压值   unit: 1V

 {     1,      4,      0,       1,       NOT_DOT},     //No.061  num_prot_Udc_stall_amp        //母线电压失速倍率         unit: 2^n次幂
//----------------------------------------------------------------------------------------------------------------------
//电机保护参数
 { 1100,  2000,        0,       1,       NOT_DOT},     //No.062  num_motor_overspeed           //电机超速百分比 unit: 0.1%  //EE
 //50 
 {  100,   100,        0,       1,       NOT_DOT},     //No.063  num_motor_stall               //电机失速百分比 unit: 1%    
 //500
 { 6000,  6000,        0,       1,       NOT_DOT},     //No.064  num_motor_stall_delaytime     //电机失速延时时间 unit:0.01s
//----------------------------------------------------------------------------------------------------------------------
//AD计算参数
 //   val     max     min    keyrate    disprate
 {  1251,  16384,      0,       1,       NOT_DOT},     //No.065  num_Iout_gain       输出电流Iout增益系数 扩大2^Iout_gain_amp次幂后的值 0.01A
 {     8,     16,      8,       1,       NOT_DOT},     //No.066  num_Iout_gain_amp   输出电流Iout增益系数的放大倍率   unit: 2^n次幂
 {     0,   1000,      0,       1,       NOT_DOT},     //No.067  num_Iout_ref        输出电流Iout参考值  unit: 0.01A 
 {     0,   1023,  -1023,       1,       NOT_DOT},     //No.068  num_Iout_ref_AD     输出电流Iout参考值对应的AD值与零点AD的差值
 
 {   259,  16384,      0,       1,       NOT_DOT},     //No.069  num_Udc_gain        母线电压Udc增益系数 扩大2^Udc_gain_amp次幂后的值  1V
 {     9,     16,      8,       1,       NOT_DOT},     //No.070  num_Udc_gain_amp    母线电压Udc增益系数的放大倍率   unit: 2^n次幂
 {     0,   1000,      0,       1,       NOT_DOT},     //No.071  num_Udc_ref         母线电压Udc的参考值  unit: 1V
 {     0,   1023,  -1023,       1,       NOT_DOT},     //No.072  num_Udc_ref_AD      母线电压Udc的参考值对应AD值与零点AD的差值

 {     0,    100,   -100,       1,       NOT_DOT},     //No.073  num_THM_ref         THM温度参考值 
 {     3,    100,   -100,       1,       NOT_DOT},     //No.074  num_THF_ref         THF温度参考值 
//----------------------------------------------------------------------------------------------------------------------
//跳停原因记录参数
//   val     max     min    keyrate    disprate
 {     0,      1,      0,       1,       NOT_DOT},     //No.075  num_Trip_clear      跳停原因清零使能 0:no Clear 1:Clear
 {     0,    255,      0,       0,       NOT_DOT},     //No.076  num_Trip_cause1     五个跳停原因记录
 {     0,    255,      0,       0,       NOT_DOT},     //No.077  num_Trip_cause2
 {     0,    255,      0,       0,       NOT_DOT},     //No.078  num_Trip_cause3
 {     0,    255,      0,       0,       NOT_DOT},     //No.079  num_Trip_cause4
 {     0,    255,      0,       0,       NOT_DOT},     //No.080  num_Trip_cause5
//----------------------------------------------------------------------------------------------------------------------
 {     0,      1,      0,       1,       NOT_DOT},     //No.081  num_drive_mode   驱动方式选择  0:SVPWM    1:台形波  //EE
//----------------------------------------------------------------------------------------------------------------------
//驱动控制方法相关的参数
//   val     max     min    keyrate    disprate
 {     1,      1,      0,       1,       NOT_DOT},     //No.082  num_max_k_set       最大调制率设定
 {    55,    150,     20,       1,    DOT_AT_0D1},     //No.083  num_pwm_carrier     PWM载波频率(精确到0.1kHz)  //EE
 {    30,    150,      0,       1,    DOT_AT_0D1},     //No.084  num_IPM_min_time    IPM最小导通时间(精确到0.1us)
 {    15,    150,     10,       1,    DOT_AT_0D1},     //No.085  num_dead_time       死区时间(精确到0.1us)
 {     0,    150,      0,       1,    DOT_AT_0D1},     //No.086  num_DTrepair        死区补偿时间(精确到0.1us) (采用只对某一相进行补偿)
 {    40,    150,      0,       1,    DOT_AT_0D1},     //No.087  num_ADrepair_time   AD转换的补偿时间(精确到0.1us)
 {     5,    150,      0,       1,    DOT_AT_0D1},     //No.088  num_AD_SH_time      AD转换的补偿时间(精确到0.1us)
//----------------------------------------------------------------------------------------------------------------------
 //   val     max     min    keyrate    disprate
 {  2200,  10000,     50,       1,    DOT_AT_0D1},     //No.089  num_Uin_rate        输入额定电压 Unit:0.1V
//----------------------------------------------------------------------------------------------------------------------
//电机转速PI控制
 {   100,   3000,      0,       1,       NOT_DOT},     //No.090  num_PIctrl_delaytime     PI控制周期  //EE
 {   456,  30000,      0,       1,       NOT_DOT},     //No.091  num_Ksp        转速环比例系数(单位：0.001V/0.01Hz/s)扩大2^15*2^8  //EE
 {    38,  30000,      0,       1,       NOT_DOT},     //No.092  num_Ksi        转速环积分系数(单位：0.001V/0.01Hz/s)扩大2^15*2^8  //EE
 {    10,   4000,      0,       1,    DOT_AT_0D1},     //No.093  num_Integral_initval     积分项初值(0.1V)  //EE
 {  2200,   4000,    700,      10,    DOT_AT_0D1},     //No.094  num_V_max      输出线电压最大有效值(0.1V) (作积分限值)  //EE
 //-------------------相位超前控制---------------------------
 {  1000,   8000,    500,     100,   DOT_AT_0D01},     //No.095  num_PhaseAdvStartFreq  //EE
 {   300,   1800,      0,      10,    DOT_AT_0D1},     //No.096  num_PhaseAdvSlope      //EE
 {   580,   1200,      0,       1,    DOT_AT_0D1},     //No.097  num_phase_max_degree      //EE
 
 
 //-------------------电机参数--------------------------------------
 //上祺1000W=4对极  大洋1000W=5对极 
 {     4,   100,       0,       1,       NOT_DOT},     //No.098  num_MOTOR_p
 {     0,   100,       0,       1,       NOT_DOT},     //No.099  num_MOTOR_R
 {     0,   100,       0,       1,       NOT_DOT},     //No.100  num_MOTOR_Ld
 {     0,   100,       0,       1,       NOT_DOT},     //No.101  num_MOTOR_Lq
 {     0,   100,       0,       1,       NOT_DOT},     //No.102  num_MOTOR_EMF
 {  1800, 30000,       0,       1,       NOT_DOT},     //No.103  num_MOTOR_max_n  单位：rpm
 {  3300, 30000,       0,      10,    DOT_AT_0D1},     //No.104  num_MOTOR_CCW    单位：0.1度
 {  3300, 30000,       0,      10,    DOT_AT_0D1},     //No.105  num_MOTOR_CW     单位：0.1度
 
//----------------------自检参数-----------------------------------
 {     0,      3,      0,       1,       NOT_DOT},     //No.106  num_selftest
//-----------------软件版本及型号信息-------------------------------
 {     0,    255,      0,       0,       NOT_DOT},     //No.107  num_App_ID
 {     0,    999,      0,       0,   DOT_AT_0D01},     //No.108  num_soft_ver
 {  3103,  19999,      0,       0,   DOT_AT_0D01},     //No.109  num_drive_letter  
 {0x55AA, 0x55AA, 0x55AA,       0,       NOT_DOT},     //No.110  num_Verify_word
};
//-----------------------------------------------------------------------
int16_t ram_para[MAX_PARA_NUMBER + 1];
//-----------------------------------------------------------------------
//------------------------------------------------------------------------------
const uint16_t App_index[]=
{
    num_accel_time,          
    num_decel_time,          		
    num_min_freq,
    num_max_freq,   		 
    num_begin_freq,           
    num_shut_freq,       
    num_trip_stop_cnt,          	
    num_stop_space_delaytime,    
    num_Iout_rate,                  
    //-------------------模块温度保护参数-----------------------------------------
    num_prot_Tm_OH,                  //THM过热保护进入温度点    unit: 1℃   
    num_prot_Tm_decfreq_enter,      //THM过热降频进入温度点   unit: 1℃   
    num_prot_Tm_decfreq_quit,       //THM过热降频退出温度点   unit: 1℃  	
    num_prot_Tm_stall_enter,        //THM过热失速进入温度点   unit: 1℃   	
    num_prot_Tm_stall_quit,         //THM过热失速退出温度点   unit: 1℃  
    //-------------------电机保护参数------------------------------------------
    num_motor_overspeed,            //电机超速百分比 unit: 0.1% 
    //-------------------驱动控制方法相关的参数--------------------------------
    num_drive_mode,                 //驱动方式选择  0:SVPWM    1:台形波  
    num_pwm_carrier,                //PWM载波频率(精确到0.1kHz)  
    //-------------------电机转速PI控制-----------------------------------------
    num_PIctrl_delaytime,           //PI控制周期  //EE 
    num_Ksp,                        //转速环比例系数(单位：0.001V/0.01Hz/s)扩大2^15*2^8   
    num_Ksi,                        //转速环积分系数(单位：0.001V/0.01Hz/s)扩大2^15*2^8  
    num_Integral_initval,           //积分项初值(0.1V)  //EE
    num_V_max,                      //输出线电压最大有效值(0.1V) (作积分限值)  
    //-------------------相位超前控制-------------------------------------------
    num_PhaseAdvStartFreq,      
    num_PhaseAdvSlope,           
    num_phase_max_degree,        

    //--------------------------------------------------------------------------
    num_MOTOR_p,                 
    num_MOTOR_R,                
    num_MOTOR_Ld,               
    num_MOTOR_Lq,               
    num_MOTOR_EMF,                
    num_MOTOR_max_n,            
    num_MOTOR_CCW,               
    num_MOTOR_CW               
};

//-------------------------------------
const int16_t App_para[][APP_PARA_CNT] =
{
	//ID=32，电机参数（大洋ZWK702B506049-375W）
    {
        150,   //No.004  num_accel_time                //(用于开机时的加速时间)  //EE
        150,   //No.005  num_decel_time                //(用于停机时的减速减速)  //EE		
        1000,  //No.006  num_min_freq   	 
        15000, //No.007  num_max_freq   		 
        0,     //No.008  num_begin_freq  // debug 
        15000,  //No.009  num_shut_freq    
        6,     //No.010  num_trip_stop_cnt         	
        30,    //No.011  num_stop_space_delaytime  
        //-------------------额定输出电流-----------------------------------------		
        900,   //No.020  num_Iout_rate                 //额定输出电流 Unit:0.01A  
        //-------------------模块温度保护参数-----------------------------------------
        95,    //No.041  num_prot_Tm_OH                //THM过热保护进入温度点    unit: 1℃   
        90,    //No.043  num_prot_Tm_decfreq_enter     //THM过热降频进入温度点   unit: 1℃   
        85,    //No.046  num_prot_Tm_decfreq_quit      //THM过热降频退出温度点   unit: 1℃  	
        85,    //No.048  num_prot_Tm_stall_enter       //THM过热失速进入温度点   unit: 1℃   	
        80,    //No.050  num_prot_Tm_stall_quit        //THM过热失速退出温度点   unit: 1℃  
        //-------------------电机保护参数------------------------------------------
        1100,  //No.062  num_motor_overspeed           //电机超速百分比 unit: 0.1% 
        //-------------------驱动控制方法相关的参数--------------------------------
        0,     //No.081  num_drive_mode                //驱动方式选择  0:SVPWM    1:台形波  
        55,    //No.083  num_pwm_carrier               //PWM载波频率(精确到0.1kHz)  
        //-------------------电机转速PI控制-----------------------------------------
        100,   //No.090  num_PIctrl_delaytime          //PI控制周期  //EE 
        456,   //No.091  num_Ksp                       //转速环比例系数(单位：0.001V/0.01Hz/s)扩大2^15*2^8   
        38,    //No.092  num_Ksi                       //转速环积分系数(单位：0.001V/0.01Hz/s)扩大2^15*2^8  
        10,    //No.093  num_Integral_initval          //积分项初值(0.1V)  //EE
        2200,  //No.094  num_V_max                     //输出线电压最大有效值(0.1V) (作积分限值)  
        //-------------------相位超前控制-------------------------------------------
        1000,  //No.095  num_PhaseAdvStartFreq   
        300,   //No.096  num_PhaseAdvSlope 
        580,   //No.097  num_phase_max_degree      //EE  
        //--------------------------------------------------------------------------
        4,     //No.098  num_MOTOR_p
        0,     //No.099  num_MOTOR_R
        0,     //No.100  num_MOTOR_Ld
        0,     //No.101  num_MOTOR_Lq
        0,     //No.102  num_MOTOR_EMF
        1800,  //No.103  num_MOTOR_max_n  单位：rpm
        3300,  //No.104  num_MOTOR_CCW    起始相位角 单位：0.1度 
        3300,  //No.105  num_MOTOR_CW     起始相位角 单位：0.1度  
    },
    
	//ID=33，电机参数（大洋ZWK702B506049-375W）
    {
        150,   //No.004  num_accel_time                //(用于开机时的加速时间)  //EE
        150,   //No.005  num_decel_time                //(用于停机时的减速减速)  //EE		
        1000,  //No.006  num_min_freq   	 
        15000, //No.007  num_max_freq   		 
        0,     //No.008  num_begin_freq  // debug 
        15000,  //No.009  num_shut_freq    
        6,     //No.010  num_trip_stop_cnt         	
        30,    //No.011  num_stop_space_delaytime  
        //-------------------额定输出电流-----------------------------------------		
        900,   //No.020  num_Iout_rate                 //额定输出电流 Unit:0.01A  
        //-------------------模块温度保护参数-----------------------------------------
        95,    //No.041  num_prot_Tm_OH                //THM过热保护进入温度点    unit: 1℃   
        90,    //No.043  num_prot_Tm_decfreq_enter     //THM过热降频进入温度点   unit: 1℃   
        85,    //No.046  num_prot_Tm_decfreq_quit      //THM过热降频退出温度点   unit: 1℃  	
        85,    //No.048  num_prot_Tm_stall_enter       //THM过热失速进入温度点   unit: 1℃   	
        80,    //No.050  num_prot_Tm_stall_quit        //THM过热失速退出温度点   unit: 1℃  
        //-------------------电机保护参数------------------------------------------
        1100,  //No.062  num_motor_overspeed           //电机超速百分比 unit: 0.1% 
        //-------------------驱动控制方法相关的参数--------------------------------
        0,     //No.081  num_drive_mode                //驱动方式选择  0:SVPWM    1:台形波  
        55,    //No.083  num_pwm_carrier               //PWM载波频率(精确到0.1kHz)  
        //-------------------电机转速PI控制-----------------------------------------
        100,   //No.090  num_PIctrl_delaytime          //PI控制周期  //EE 
        456,   //No.091  num_Ksp                       //转速环比例系数(单位：0.001V/0.01Hz/s)扩大2^15*2^8   
        38,    //No.092  num_Ksi                       //转速环积分系数(单位：0.001V/0.01Hz/s)扩大2^15*2^8  
        10,    //No.093  num_Integral_initval          //积分项初值(0.1V)  //EE
        2200,  //No.094  num_V_max                     //输出线电压最大有效值(0.1V) (作积分限值)  
        //-------------------相位超前控制-------------------------------------------
        1000,  //No.095  num_PhaseAdvStartFreq   
        300,   //No.096  num_PhaseAdvSlope 
        580,   //No.097  num_phase_max_degree      //EE  
        //--------------------------------------------------------------------------
        4,     //No.098  num_MOTOR_p
        0,     //No.099  num_MOTOR_R
        0,     //No.100  num_MOTOR_Ld
        0,     //No.101  num_MOTOR_Lq
        0,     //No.102  num_MOTOR_EMF
        1800,  //No.103  num_MOTOR_max_n  单位：rpm
        3300,  //No.104  num_MOTOR_CCW     起始相位角 单位：0.1度 
        3300,  //No.105  num_MOTOR_CW     起始相位角 单位：0.1度  
    },	
};
//-----------------------------------------------------------------------
#endif    /*__ROM_PARA_C__*/
