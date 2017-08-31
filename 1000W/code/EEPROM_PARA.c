#ifndef __ROM_PARA_C__
#define __ROM_PARA_C__
//------------------------------------------------------------------------------
#include "user_typedefine.h"
#include "EEPROM_PARA.h"
//------------------------------------------------------------------------------
const int16_t EEPROM_InitData[MAX_PARA_NUMBER+1][5]=           //��������
{
//    val     max     min    keyrate    disprate
 {     0,      2,      0,       1,       NOT_DOT},     //No.000  num_pr_set_method  peak 1-->0
 {     1,      1,      0,       1,       NOT_DOT},     //No.001  num_cmd_source
 {  2000,  30000,      0,     100,   DOT_AT_0D01},     //No.002  num_ref_freq
 {     0,      1,      0,       1,       NOT_DOT},     //No.003  num_direction  0����ʱ��  1��˳ʱ�� peak 1-->0Ӧ���ǶԵ�
   
 {   150,   6000,      1,       1,    DOT_AT_0D1},     //No.004  num_accel_time (���ڿ���ʱ�ļ���ʱ��)  //EE
 {   150,   6000,      1,       1,    DOT_AT_0D1},     //No.005  num_decel_time (����ͣ��ʱ�ļ��ټ���)  //EE
//----------------------------------------------------------------------------------------------------------------------
//Ƶ�ʲ���
//    val     max     min    keyrate    disprate
 {  1000,  30000,      0,      10,   DOT_AT_0D01},     //No.006  num_min_freq  //EE
 { 15000,  15000,      0,      10,   DOT_AT_0D01},     //No.007  num_max_freq  //EE
 
 {     0,  30000,      0,      10,   DOT_AT_0D01},     //No.008  num_begin_freq  //EE
 //
 { 15000,  30000,      0,      10,   DOT_AT_0D01},     //No.009  num_shut_freq   //EE
//----------------------------------------------------------------------------------------------------------------------
//����ͣ������
//    val     max     min    keyrate    disprate
 {     6,    100,      0,       1,       NOT_DOT},     //No.010  num_trip_stop_cnt        ������������  //EE

 {    30,   6000,      0,        1,     NOT_DOT},   //No.011  num_stop_space_delaytime �ϵ��������ʱs  //EE
 {    60,    600,      3,       1,       NOT_DOT},     //No.012  num_trip_lock_delaytime  ����������ʱʱ��min
//----------------------------------------------------------------------------------------------------------------------
//����·��������
//    val     max     min    keyrate    disprate
 {   512,   1023,      0,       1,       NOT_DOT},     //No.013  num_Iout_CT_zero_AD      //�������Iout��CT����Ӧ��ADֵ
 {    50,    512,      0,       1,       NOT_DOT},     //No.014  num_Iout_CT_zero_margin  //�������Iout��CT����ƫ�Ʒ�Χ

 {     0,   1023,      0,       1,       NOT_DOT},     //No.015  num_THM_lowlim_AD        //ɢ�����¶ȼ������ADֵ
 {  1010,   1023,      0,       1,       NOT_DOT},     //No.016  num_THM_uplim_AD         //ɢ�����¶ȼ������ADֵ
//----------------------------------------------------------------------------------------------------------------------
//���������������
//   val     max     min    keyrate    disprate
 {     0,    100,      0,       1,       NOT_DOT},     //No.017  num_prot_ALM_cnt               //���������·����������

 {  1500,   2000,      0,       1,    DOT_AT_0D1},     //No.018  num_prot_Iout_POC              //�������Iout���˲ʱ����������ֵ Unit:0.01A
 {     3,    100,      0,       1,       NOT_DOT},     //No.019  num_prot_Iout_POC_delaycnt     //�������Iout˲ʱ��������������

 {   900,   1000,      0,       1,    DOT_AT_0D1},     //No.020  num_Iout_rate                  //�������� Unit:0.01A //EE

 {   100,   1000,      0,      10,    DOT_AT_0D1},     //No.021  num_prot_Iout_imbal            //������಻ƽ����  unit: 0.1%
 {    50,   3000,      0,       1,    DOT_AT_0D1},     //No.022  num_prot_Iout_imbal_delaytime  //������಻ƽ����ʱ�� unit: 0.1S

 {     0,   1000,      0,      10,    DOT_AT_0D1},     //No.023  num_prot_Iout_NC               //���ȱ���� unit: 0.1%
 {    30,   3000,      0,       1,    DOT_AT_0D1},     //No.024  num_prot_Iout_NC_delaytime     //���ȱ�����ʱ�� unit: 0.1S

 {  1100,   2000,      0,      10,    DOT_AT_0D1},     //No.025  num_prot_Iout_OC               //������������İٷֱ� unit: 0.1%
 {    30,   3000,      0,       1,    DOT_AT_0D1},     //No.026  num_prot_Iout_OC_delaytime     //���������������ʱ�� unit: 0.1S

 {  1100,   2000,      0,      10,    DOT_AT_0D1},     //No.027  num_prot_Iout_OL               //������ذٷֱ�   unit: 0.1%
 {   300,   3000,      0,       1,    DOT_AT_0D1},     //No.028  num_prot_Iout_OL_delaytime     //������س���ʱ�� unit: 0.1S
 {  1040,   2000,      0,      10,    DOT_AT_0D1},     //No.029  num_prot_Iout_OL_enter         //��ʼ������ذٷֱ�   unit: 0.1%

 {  1000,   2000,      0,      10,    DOT_AT_0D1},     //No.030  num_prot_Iout_decfreq_enter              //���������Ƶ����ٷֱ�   unit: 0.1%
 {    10,   3000,      0,       1,    DOT_AT_0D1},     //No.031  num_prot_Iout_decfreq_enter_delaytime    //���������Ƶ����ʱ��     unit: 0.1S
 {   400,    500,      0,      10,   DOT_AT_0D01},     //No.032  num_prot_Iout_decfreq_deltfreq           //���������Ƶ����         unit: 0.01Hz 
 {   920,   2000,      0,      10,    DOT_AT_0D1},     //No.033  num_prot_Iout_decfreq_quit               //���������Ƶ�˳��ٷֱ�   unit: 0.1%
 {    30,   3000,      0,       1,    DOT_AT_0D1},     //No.034  num_prot_Iout_decfreq_quit_delaytime     //���������Ƶ�˳�����ʱ�� unit: 0.1S

 {   900,   2000,      0,      10,    DOT_AT_0D1},     //No.035  num_prot_Iout_stall_enter                //�������ʧ�ٽ���ٷֱ�   unit: 0.1%
 {    10,   3000,      0,       1,    DOT_AT_0D1},     //No.036  num_prot_Iout_stall_enter_delaytime      //�������ʧ�ٽ������ʱ�� unit: 0.1S
 {   860,   2000,      0,      10,    DOT_AT_0D1},     //No.037  num_prot_Iout_stall_quit                 //�������ʧ���˳��ٷֱ�   unit: 0.1%
 {    20,   3000,      0,       1,    DOT_AT_0D1},     //No.038  num_prot_Iout_stall_quit_delaytime       //�������ʧ���˳�����ʱ�� unit: 0.1S
 {     3,      4,      0,       1,       NOT_DOT},     //No.039  num_prot_Iout_stall_amp                  //�������ʧ�ٱ���         unit: 2^n����

//----------------------------------------------------------------------------------------------------------------------
//ģ���¶ȱ�������
//   val     max     min    keyrate    disprate
 {     1,      1,      0,       1,       NOT_DOT},     //No.040  num_Tm_enbale                 //THM�¶ȴ�����ʹ��ѡ��      0����ʹ��  1��ʹ��

 {    95,    140,    -20,       1,       NOT_DOT},     //No.041  num_prot_Tm_OH                //THM���ȱ��������¶ȵ�    unit: 1��  //EE
 {    10,   3000,      0,       1,    DOT_AT_0D1},     //No.042  num_prot_Tm_OH_delaytime      //THM���ȱ����������ʱ��  unit: 0.1S 
 {    90,    140,    -20,       1,       NOT_DOT},     //No.043  num_prot_Tm_decfreq_enter     //THM���Ƚ�Ƶ�����¶ȵ�   unit: 1��  //EE
 {    30,   3000,      0,       1,    DOT_AT_0D1},     //No.044  num_prot_Tm_decfreq_delaytime //THM���Ƚ�Ƶ����ʱ��     unit: 0.1S
 {   200,   8000,      0,      10,   DOT_AT_0D01},     //No.045  num_prot_Tm_decfreq_deltfreq  //THM���Ƚ�Ƶ����         unit: 0.01Hz 
 {    85,    140,    -20,       1,       NOT_DOT},     //No.046  num_prot_Tm_decfreq_quit      //THM���Ƚ�Ƶ�˳��¶ȵ�   unit: 1��  //EE
 {    20,   3000,      0,       1,    DOT_AT_0D1},     //No.047  num_prot_Tm_decfreq_quit_delaytime //THM���Ƚ�Ƶ�˳�����ʱ�� unit: 0.1S
 {    85,    140,    -20,       1,       NOT_DOT},     //No.048  num_prot_Tm_stall_enter       //THM����ʧ�ٽ����¶ȵ�   unit: 1��  //EE
 {    20,   3000,      0,       1,    DOT_AT_0D1},     //No.049  num_prot_Tm_stall_enter_delaytime  //THM����ʧ�ٽ������ʱ�� unit: 0.1S
 {    80,    140,    -20,       1,       NOT_DOT},     //No.050  num_prot_Tm_stall_quit        //THM����ʧ���˳��¶ȵ�   unit: 1��  //EE
 {    20,   3000,      0,       1,    DOT_AT_0D1},     //No.051  num_prot_Tm_stall_quit_delaytime   //THM����ʧ���˳�����ʱ�� unit: 0.1S
 {     3,      4,      0,       1,       NOT_DOT},     //No.052  num_prot_Tm_stall_amp         //THM����ʧ�ٱ���         unit: 2^n����

//----------------------------------------------------------------------------------------------------------------------
//ĸ�ߵ�ѹ��������
//   val     max     min    keyrate    disprate
 {   410,   1000,      0,       1,       NOT_DOT},     //No.053  num_prot_Udc_POU              //ĸ�ߵ�ѹ���˲ʱ��ѹ������ֵ
 {     3,    100,      0,       1,       NOT_DOT},     //No.054  num_prot_Udc_POU_delaycnt     //ĸ�ߵ�ѹ˲ʱ��ѹ����������

 {   400,   1000,      0,       1,       NOT_DOT},     //No.055  num_prot_Udc_OU_enter         //ĸ�ߵ�ѹ��ѹ�����ѹֵ  unit: 1V
 {   376,   1000,      0,       1,       NOT_DOT},     //No.056  num_prot_Udc_OU_quit          //ĸ�ߵ�ѹ��ѹ�˳���ѹֵ  unit: 1V

 {   383,   1000,      0,       1,       NOT_DOT},     //No.057  num_prot_Udc_stall_enter      //ĸ�ߵ�ѹʧ�ٽ����ѹֵ   unit: 1V
 {   373,   1000,      0,       1,       NOT_DOT},     //No.058  num_prot_Udc_stall_quit       //ĸ�ߵ�ѹʧ���˳���ѹֵ   unit: 1V

 {   194,   1000,      0,       1,       NOT_DOT},     //No.059  num_prot_Udc_LU_enter         //ĸ�ߵ�ѹǷѹ�����ѹֵ   unit: 1V
 {   218,   1000,      0,       1,       NOT_DOT},     //No.060  num_prot_Udc_LU_quit          //ĸ�ߵ�ѹǷѹ�˳���ѹֵ   unit: 1V

 {     1,      4,      0,       1,       NOT_DOT},     //No.061  num_prot_Udc_stall_amp        //ĸ�ߵ�ѹʧ�ٱ���         unit: 2^n����
//----------------------------------------------------------------------------------------------------------------------
//�����������
 { 1100,  2000,        0,       1,       NOT_DOT},     //No.062  num_motor_overspeed           //������ٰٷֱ� unit: 0.1%  //EE
 //50 
 {  100,   100,        0,       1,       NOT_DOT},     //No.063  num_motor_stall               //���ʧ�ٰٷֱ� unit: 1%    
 //500
 { 6000,  6000,        0,       1,       NOT_DOT},     //No.064  num_motor_stall_delaytime     //���ʧ����ʱʱ�� unit:0.01s
//----------------------------------------------------------------------------------------------------------------------
//AD�������
 //   val     max     min    keyrate    disprate
 {  1251,  16384,      0,       1,       NOT_DOT},     //No.065  num_Iout_gain       �������Iout����ϵ�� ����2^Iout_gain_amp���ݺ��ֵ 0.01A
 {     8,     16,      8,       1,       NOT_DOT},     //No.066  num_Iout_gain_amp   �������Iout����ϵ���ķŴ���   unit: 2^n����
 {     0,   1000,      0,       1,       NOT_DOT},     //No.067  num_Iout_ref        �������Iout�ο�ֵ  unit: 0.01A 
 {     0,   1023,  -1023,       1,       NOT_DOT},     //No.068  num_Iout_ref_AD     �������Iout�ο�ֵ��Ӧ��ADֵ�����AD�Ĳ�ֵ
 
 {   259,  16384,      0,       1,       NOT_DOT},     //No.069  num_Udc_gain        ĸ�ߵ�ѹUdc����ϵ�� ����2^Udc_gain_amp���ݺ��ֵ  1V
 {     9,     16,      8,       1,       NOT_DOT},     //No.070  num_Udc_gain_amp    ĸ�ߵ�ѹUdc����ϵ���ķŴ���   unit: 2^n����
 {     0,   1000,      0,       1,       NOT_DOT},     //No.071  num_Udc_ref         ĸ�ߵ�ѹUdc�Ĳο�ֵ  unit: 1V
 {     0,   1023,  -1023,       1,       NOT_DOT},     //No.072  num_Udc_ref_AD      ĸ�ߵ�ѹUdc�Ĳο�ֵ��ӦADֵ�����AD�Ĳ�ֵ

 {     0,    100,   -100,       1,       NOT_DOT},     //No.073  num_THM_ref         THM�¶Ȳο�ֵ 
 {     3,    100,   -100,       1,       NOT_DOT},     //No.074  num_THF_ref         THF�¶Ȳο�ֵ 
//----------------------------------------------------------------------------------------------------------------------
//��ͣԭ���¼����
//   val     max     min    keyrate    disprate
 {     0,      1,      0,       1,       NOT_DOT},     //No.075  num_Trip_clear      ��ͣԭ������ʹ�� 0:no Clear 1:Clear
 {     0,    255,      0,       0,       NOT_DOT},     //No.076  num_Trip_cause1     �����ͣԭ���¼
 {     0,    255,      0,       0,       NOT_DOT},     //No.077  num_Trip_cause2
 {     0,    255,      0,       0,       NOT_DOT},     //No.078  num_Trip_cause3
 {     0,    255,      0,       0,       NOT_DOT},     //No.079  num_Trip_cause4
 {     0,    255,      0,       0,       NOT_DOT},     //No.080  num_Trip_cause5
//----------------------------------------------------------------------------------------------------------------------
 {     0,      1,      0,       1,       NOT_DOT},     //No.081  num_drive_mode   ������ʽѡ��  0:SVPWM    1:̨�β�  //EE
//----------------------------------------------------------------------------------------------------------------------
//�������Ʒ�����صĲ���
//   val     max     min    keyrate    disprate
 {     1,      1,      0,       1,       NOT_DOT},     //No.082  num_max_k_set       ���������趨
 {    55,    150,     20,       1,    DOT_AT_0D1},     //No.083  num_pwm_carrier     PWM�ز�Ƶ��(��ȷ��0.1kHz)  //EE
 {    30,    150,      0,       1,    DOT_AT_0D1},     //No.084  num_IPM_min_time    IPM��С��ͨʱ��(��ȷ��0.1us)
 {    15,    150,     10,       1,    DOT_AT_0D1},     //No.085  num_dead_time       ����ʱ��(��ȷ��0.1us)
 {     0,    150,      0,       1,    DOT_AT_0D1},     //No.086  num_DTrepair        ��������ʱ��(��ȷ��0.1us) (����ֻ��ĳһ����в���)
 {    40,    150,      0,       1,    DOT_AT_0D1},     //No.087  num_ADrepair_time   ADת���Ĳ���ʱ��(��ȷ��0.1us)
 {     5,    150,      0,       1,    DOT_AT_0D1},     //No.088  num_AD_SH_time      ADת���Ĳ���ʱ��(��ȷ��0.1us)
//----------------------------------------------------------------------------------------------------------------------
 //   val     max     min    keyrate    disprate
 {  2200,  10000,     50,       1,    DOT_AT_0D1},     //No.089  num_Uin_rate        ������ѹ Unit:0.1V
//----------------------------------------------------------------------------------------------------------------------
//���ת��PI����
 {   100,   3000,      0,       1,       NOT_DOT},     //No.090  num_PIctrl_delaytime     PI��������  //EE
 {   456,  30000,      0,       1,       NOT_DOT},     //No.091  num_Ksp        ת�ٻ�����ϵ��(��λ��0.001V/0.01Hz/s)����2^15*2^8  //EE
 {    38,  30000,      0,       1,       NOT_DOT},     //No.092  num_Ksi        ת�ٻ�����ϵ��(��λ��0.001V/0.01Hz/s)����2^15*2^8  //EE
 {    10,   4000,      0,       1,    DOT_AT_0D1},     //No.093  num_Integral_initval     �������ֵ(0.1V)  //EE
 {  2200,   4000,    700,      10,    DOT_AT_0D1},     //No.094  num_V_max      ����ߵ�ѹ�����Чֵ(0.1V) (��������ֵ)  //EE
 //-------------------��λ��ǰ����---------------------------
 {  1000,   8000,    500,     100,   DOT_AT_0D01},     //No.095  num_PhaseAdvStartFreq  //EE
 {   300,   1800,      0,      10,    DOT_AT_0D1},     //No.096  num_PhaseAdvSlope      //EE
 {   580,   1200,      0,       1,    DOT_AT_0D1},     //No.097  num_phase_max_degree      //EE
 
 
 //-------------------�������--------------------------------------
 //����1000W=4�Լ�  ����1000W=5�Լ� 
 {     4,   100,       0,       1,       NOT_DOT},     //No.098  num_MOTOR_p
 {     0,   100,       0,       1,       NOT_DOT},     //No.099  num_MOTOR_R
 {     0,   100,       0,       1,       NOT_DOT},     //No.100  num_MOTOR_Ld
 {     0,   100,       0,       1,       NOT_DOT},     //No.101  num_MOTOR_Lq
 {     0,   100,       0,       1,       NOT_DOT},     //No.102  num_MOTOR_EMF
 {  1800, 30000,       0,       1,       NOT_DOT},     //No.103  num_MOTOR_max_n  ��λ��rpm
 {  3300, 30000,       0,      10,    DOT_AT_0D1},     //No.104  num_MOTOR_CCW    ��λ��0.1��
 {  3300, 30000,       0,      10,    DOT_AT_0D1},     //No.105  num_MOTOR_CW     ��λ��0.1��
 
//----------------------�Լ����-----------------------------------
 {     0,      3,      0,       1,       NOT_DOT},     //No.106  num_selftest
//-----------------����汾���ͺ���Ϣ-------------------------------
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
    //-------------------ģ���¶ȱ�������-----------------------------------------
    num_prot_Tm_OH,                  //THM���ȱ��������¶ȵ�    unit: 1��   
    num_prot_Tm_decfreq_enter,      //THM���Ƚ�Ƶ�����¶ȵ�   unit: 1��   
    num_prot_Tm_decfreq_quit,       //THM���Ƚ�Ƶ�˳��¶ȵ�   unit: 1��  	
    num_prot_Tm_stall_enter,        //THM����ʧ�ٽ����¶ȵ�   unit: 1��   	
    num_prot_Tm_stall_quit,         //THM����ʧ���˳��¶ȵ�   unit: 1��  
    //-------------------�����������------------------------------------------
    num_motor_overspeed,            //������ٰٷֱ� unit: 0.1% 
    //-------------------�������Ʒ�����صĲ���--------------------------------
    num_drive_mode,                 //������ʽѡ��  0:SVPWM    1:̨�β�  
    num_pwm_carrier,                //PWM�ز�Ƶ��(��ȷ��0.1kHz)  
    //-------------------���ת��PI����-----------------------------------------
    num_PIctrl_delaytime,           //PI��������  //EE 
    num_Ksp,                        //ת�ٻ�����ϵ��(��λ��0.001V/0.01Hz/s)����2^15*2^8   
    num_Ksi,                        //ת�ٻ�����ϵ��(��λ��0.001V/0.01Hz/s)����2^15*2^8  
    num_Integral_initval,           //�������ֵ(0.1V)  //EE
    num_V_max,                      //����ߵ�ѹ�����Чֵ(0.1V) (��������ֵ)  
    //-------------------��λ��ǰ����-------------------------------------------
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
	//ID=32���������������ZWK702B506049-375W��
    {
        150,   //No.004  num_accel_time                //(���ڿ���ʱ�ļ���ʱ��)  //EE
        150,   //No.005  num_decel_time                //(����ͣ��ʱ�ļ��ټ���)  //EE		
        1000,  //No.006  num_min_freq   	 
        15000, //No.007  num_max_freq   		 
        0,     //No.008  num_begin_freq  // debug 
        15000,  //No.009  num_shut_freq    
        6,     //No.010  num_trip_stop_cnt         	
        30,    //No.011  num_stop_space_delaytime  
        //-------------------��������-----------------------------------------		
        900,   //No.020  num_Iout_rate                 //�������� Unit:0.01A  
        //-------------------ģ���¶ȱ�������-----------------------------------------
        95,    //No.041  num_prot_Tm_OH                //THM���ȱ��������¶ȵ�    unit: 1��   
        90,    //No.043  num_prot_Tm_decfreq_enter     //THM���Ƚ�Ƶ�����¶ȵ�   unit: 1��   
        85,    //No.046  num_prot_Tm_decfreq_quit      //THM���Ƚ�Ƶ�˳��¶ȵ�   unit: 1��  	
        85,    //No.048  num_prot_Tm_stall_enter       //THM����ʧ�ٽ����¶ȵ�   unit: 1��   	
        80,    //No.050  num_prot_Tm_stall_quit        //THM����ʧ���˳��¶ȵ�   unit: 1��  
        //-------------------�����������------------------------------------------
        1100,  //No.062  num_motor_overspeed           //������ٰٷֱ� unit: 0.1% 
        //-------------------�������Ʒ�����صĲ���--------------------------------
        0,     //No.081  num_drive_mode                //������ʽѡ��  0:SVPWM    1:̨�β�  
        55,    //No.083  num_pwm_carrier               //PWM�ز�Ƶ��(��ȷ��0.1kHz)  
        //-------------------���ת��PI����-----------------------------------------
        100,   //No.090  num_PIctrl_delaytime          //PI��������  //EE 
        456,   //No.091  num_Ksp                       //ת�ٻ�����ϵ��(��λ��0.001V/0.01Hz/s)����2^15*2^8   
        38,    //No.092  num_Ksi                       //ת�ٻ�����ϵ��(��λ��0.001V/0.01Hz/s)����2^15*2^8  
        10,    //No.093  num_Integral_initval          //�������ֵ(0.1V)  //EE
        2200,  //No.094  num_V_max                     //����ߵ�ѹ�����Чֵ(0.1V) (��������ֵ)  
        //-------------------��λ��ǰ����-------------------------------------------
        1000,  //No.095  num_PhaseAdvStartFreq   
        300,   //No.096  num_PhaseAdvSlope 
        580,   //No.097  num_phase_max_degree      //EE  
        //--------------------------------------------------------------------------
        4,     //No.098  num_MOTOR_p
        0,     //No.099  num_MOTOR_R
        0,     //No.100  num_MOTOR_Ld
        0,     //No.101  num_MOTOR_Lq
        0,     //No.102  num_MOTOR_EMF
        1800,  //No.103  num_MOTOR_max_n  ��λ��rpm
        3300,  //No.104  num_MOTOR_CCW    ��ʼ��λ�� ��λ��0.1�� 
        3300,  //No.105  num_MOTOR_CW     ��ʼ��λ�� ��λ��0.1��  
    },
    
	//ID=33���������������ZWK702B506049-375W��
    {
        150,   //No.004  num_accel_time                //(���ڿ���ʱ�ļ���ʱ��)  //EE
        150,   //No.005  num_decel_time                //(����ͣ��ʱ�ļ��ټ���)  //EE		
        1000,  //No.006  num_min_freq   	 
        15000, //No.007  num_max_freq   		 
        0,     //No.008  num_begin_freq  // debug 
        15000,  //No.009  num_shut_freq    
        6,     //No.010  num_trip_stop_cnt         	
        30,    //No.011  num_stop_space_delaytime  
        //-------------------��������-----------------------------------------		
        900,   //No.020  num_Iout_rate                 //�������� Unit:0.01A  
        //-------------------ģ���¶ȱ�������-----------------------------------------
        95,    //No.041  num_prot_Tm_OH                //THM���ȱ��������¶ȵ�    unit: 1��   
        90,    //No.043  num_prot_Tm_decfreq_enter     //THM���Ƚ�Ƶ�����¶ȵ�   unit: 1��   
        85,    //No.046  num_prot_Tm_decfreq_quit      //THM���Ƚ�Ƶ�˳��¶ȵ�   unit: 1��  	
        85,    //No.048  num_prot_Tm_stall_enter       //THM����ʧ�ٽ����¶ȵ�   unit: 1��   	
        80,    //No.050  num_prot_Tm_stall_quit        //THM����ʧ���˳��¶ȵ�   unit: 1��  
        //-------------------�����������------------------------------------------
        1100,  //No.062  num_motor_overspeed           //������ٰٷֱ� unit: 0.1% 
        //-------------------�������Ʒ�����صĲ���--------------------------------
        0,     //No.081  num_drive_mode                //������ʽѡ��  0:SVPWM    1:̨�β�  
        55,    //No.083  num_pwm_carrier               //PWM�ز�Ƶ��(��ȷ��0.1kHz)  
        //-------------------���ת��PI����-----------------------------------------
        100,   //No.090  num_PIctrl_delaytime          //PI��������  //EE 
        456,   //No.091  num_Ksp                       //ת�ٻ�����ϵ��(��λ��0.001V/0.01Hz/s)����2^15*2^8   
        38,    //No.092  num_Ksi                       //ת�ٻ�����ϵ��(��λ��0.001V/0.01Hz/s)����2^15*2^8  
        10,    //No.093  num_Integral_initval          //�������ֵ(0.1V)  //EE
        2200,  //No.094  num_V_max                     //����ߵ�ѹ�����Чֵ(0.1V) (��������ֵ)  
        //-------------------��λ��ǰ����-------------------------------------------
        1000,  //No.095  num_PhaseAdvStartFreq   
        300,   //No.096  num_PhaseAdvSlope 
        580,   //No.097  num_phase_max_degree      //EE  
        //--------------------------------------------------------------------------
        4,     //No.098  num_MOTOR_p
        0,     //No.099  num_MOTOR_R
        0,     //No.100  num_MOTOR_Ld
        0,     //No.101  num_MOTOR_Lq
        0,     //No.102  num_MOTOR_EMF
        1800,  //No.103  num_MOTOR_max_n  ��λ��rpm
        3300,  //No.104  num_MOTOR_CCW     ��ʼ��λ�� ��λ��0.1�� 
        3300,  //No.105  num_MOTOR_CW     ��ʼ��λ�� ��λ��0.1��  
    },	
};
//-----------------------------------------------------------------------
#endif    /*__ROM_PARA_C__*/
