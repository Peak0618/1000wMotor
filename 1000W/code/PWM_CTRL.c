#ifndef _PWM_CTRL_C_
#define _PWM_CTRL_C_
//------------------------------------------------------------------------------
#include "srMN103SFJ9D.h"
#include "user_typedefine.h"
#include "comm_variable.h"
#include "user_asmfunc.h"

#include "EEPROM_PARA.h"
#include "PWM_CTRL.h"
#include "AD_Convert.h"
#include "SIN_TABLE.h"
#include "protect_deal.h"
#include "Hallsignal_deal.h"
#include "user_math.h"
//------------------------------------------------------------------------------
void PWM_configure(void);     //PWMģ�����ó���

void freq_rate_init(void);    //��ʼ��1msƵ���������ĳ���

void open_pwm(void);          //����PWM�ĳ����ڿ��������е���

void shut_pwm(void);          //�ر�PWM�ĳ�����ͣ������򱣻�ͣ�������е���

void bootstrap_charger_delaytime(void);  //�Ծٳ����ʱ������1ms��ʱ�����е���

void check_freq_updata(void);      //��ʱ����Ƶ�ʸ��µĳ���(ÿ1ms���1��)

void Sensored_cale_Vout(void);     //��λ�ô��������Ƽ��������ѹ�ĳ���

void calc_percent_theta(void);     //���ڼ��㵱ǰ�����ʺ���λ�ǲ����ĳ���

void calc_comp_val(void);          //������һ����������ıȽϼ���ֵ,�ڲ��η����ж��е���

int16_t get_sin_value(uint16_t now_sin_index);    //��sin��ĺ���

uint16_t get_tai_comp_value(uint16_t now_index);  //̨�β�����ʱ����PWM�������ֵ�ĳ���

uint16_t get_svpwm_comp_value(uint16_t now_index);//���β�����ʱ����PWM�������ֵ�ĳ���

void PWM_Reload_INT(void);    //���η����ж�

void comp_val_order(void);    //������Ƚ�ֵ��������

//------------------------------------------------------------------------------
//��־����
flag_type flg_PWM;
//----------------------------------------------------------
//��������
//----------------------------------------------------------
//����һ��������ڼ���Ƶ�ʼӼ��ٱ����Ľṹ��
struct  
{
    uint32_t normal_accel;    //�����׶ε�ÿ1ms���ٲ��� (������2^24)
    uint32_t normal_decel;    //�����׶ε�ÿ1ms���ٲ��� (������2^24)
    
    lword_type sum_accel;     //���ٲ���β���ۼƼĴ���
    lword_type sum_decel;     //���ٲ���β���ۼƼĴ���
}freq_rate;

uint16_t  gus_stall_amp_accel;//���ٹ��̵�ʧ�ٱ��ʼĴ���
uint16_t  gus_stall_amp_decel;//���ٹ��̵�ʧ�ٱ��ʼĴ���

uint16_t  gus_Td;             //����ʱ�����ֵ�Ĵ���

uint16_t  gus_half_pwm;       //pwm����ֵ��һ�룬���ز��ķ�ֵ
uint16_t  gus_peak_pwm;       //pwm����ֵ�����ز��Ĳ��嵽����ֵ

uint16_t  gus_centi_half_pwm; //�ٷ�֮һ��half_pwm; (�����Ծٳ��)

uint16_t  gus_val_a;
uint16_t  gus_val_b;
uint16_t  gus_val_c;

int16_t  gss_max_pwm;
int16_t  gss_min_pwm;

uint32_t  gul_percent_pwm_unit;    //��λ��ѹ(0.1V)�����ʼ���ֵ�Ĵ���(2^16��)

uint16_t  gus_max_percent;    //���������������ʵ�4096��(����̨�β�����)
uint16_t  gus_min_percent;    //��С������������ʵ�4096��(����̨�β�����)
uint16_t  gus_max_percent_gu; //���������������ʵ�4096��(���ڹ��β�����)

uint16_t  gus_percent_k;      //ʵ�ʵ����ʵ�4096��

uint16_t  gus_percent_pwm;    //�����ʶ�Ӧ��pwm����ֵ�Ĵ���
uint16_t  gus_percent_pwm_buf;//�����ʶ�Ӧ��pwm����ֵ�Ĵ����Ļ���Ĵ���

uint16_t  gus_rate_per;       //��СƵ�ʵ�λ(0.01Hz)��Ӧ�ĵ�λ���������ڵ���λ��
lword_type rate_theta;        //��ǰƵ���µ���λ�����Ĵ���
lword_type rate_theta_buf;    //��ǰƵ���µ���λ�����Ĵ����Ļ���Ĵ���

lword_type gul_theta;
lword_type theta;             //A����Ʋ�����λ�ǼĴ���
lword_type theta_buf;         //A����Ʋ�����λ�ǻ���Ĵ��� (���ڼ�����λ��ǰ�ǶȺ���)

uint16_t  gus_max_comp_val;   //��ǰPWM����Ƚ�ֵ�����ֵ
uint16_t  gus_mid_comp_val;   //��ǰPWM����Ƚ�ֵ���м�ֵ
uint16_t  gus_min_comp_val;   //��ǰPWM����Ƚ�ֵ����Сֵ

uint16_t  gus_val_abc_order;  //���ڼ�¼abc���������С�ļĴ���

//ʵ����������ADת�������Ĵ�����ֵ���ݴ���
uint16_t  gus_TUP_ADtrigger;       //�ز�������AD����ʱ���ݴ���
uint16_t  gus_TDW_ADtrigger;       //�ز��½���AD����ʱ���ݴ���   

uint16_t  gus_ADtrigger_repair_val;//AD����ʱ�̵Ĳ���ֵ�Ĵ���

uint16_t  gus_AD_SH_val;      //AD��������ʱ��ļĴ���

uint16_t  gus_now_theta;      //��ǰ�������ڶ�Ӧ����λ��
uint16_t  gus_next_theta;     //�¸��������ڶ�Ӧ����λ��

int16_t   gss_bootstrap_charger_delaytimer;  //�Ծٳ�������
uint16_t  gus_freq_set;       //�趨Ƶ��
uint16_t  gus_MAX_setfreq; //����趨Ƶ��
//uint16_t  gus_freq_out_prev;  //�ϴ����Ƶ��
uint16_t  gus_freq_out;       //���Ƶ��
uint16_t  gus_freq_real;      //��ʵ���Ƶ�� (ʵ��Ƶ��)

uint16_t  gus_speed_out;      //���ת��
uint16_t  gus_speed_real;     //��ʵ���ת�� (ʵ��ת��)  

uint16_t  gus_Uout_real;      //��ǰ��ʵ�����ѹ     (0.1V)
uint16_t  gus_Uout_for_disp;

uint16_t  gus_PIctrl_delaytimer;
//int16_t   gss_PI_DeltSpeed;
int32_t   gsl_FreqPI_integral;
int32_t   gsl_PI_initval;     //���ֳ�ֵ
//------------------------------------------------------------------------------
//ָ��ABC��������Ƚ�ֵ���Ӵ�С�����ָʾ��־��
#define   ORDER_ABC      0
#define   ORDER_ACB      1
#define   ORDER_CAB      2
#define   ORDER_BAC      3
#define   ORDER_BCA      4
#define   ORDER_CBA      5
//------------------------------------------------------------------------------
void PWM_configure(void)      //PWMģ�����ó���
{
    uint32_t lul_tmp;
    
    //------------------------------------------------------
    //���¼�������PMWclock=MCLKʱ�ļ���(PWMMD0A.CLKSEL0=1)  
    //����peak_pwm��half_pwm, pwm_carrier�ľ���Ϊ0.1khz 
    //peak_pwmΪ�ز����ֵ, half_pwmΪ�ز����ֵ��1/2
    //gus_peak_pwm = ([1 / (pwm_carrier * 1000 / 10)] / [1 / 40M]) / 2 - 1 
    //��gus_peak_pwm = (200000) / pwm_carrier - 1  
    gus_peak_pwm = 200000 / ram_para[num_pwm_carrier] - 1;
    gus_half_pwm = (gus_peak_pwm >> 1);
    gus_centi_half_pwm = (gus_half_pwm / 100) + 1;    //�ٷ�֮һ��half_pwm
    //------------------------------------------------------
    //������С��ͨʱ������ͨʱ��
    //��С��ͨʱ��=(Td+Tmin)/2
    //���ͨʱ�䣨��С�ض�ʱ�䣩= gus_peak_pwm - ��С��ͨʱ��
    //��ΪTd��Tmin����Ϊ0.1us ��xΪx/[(10^6)*10] s
    //����(((ram_para[num_dead_time]+ram_para[num_IPM_min_time])/[(10^6)*10])/2)*40M -1
    //��min_pwm=3*(ram_para[num_dead_time]+ram_para[num_IPM_min_time]) -1
    gss_min_pwm = 2 * (ram_para[num_dead_time] + ram_para[num_IPM_min_time]) - 1;  //��С��ͨʱ�����ֵ                      
    gss_max_pwm = gus_peak_pwm - gss_min_pwm;      //���ͨʱ�����ֵ(��С�ض�ʱ��)
    //------------------------------------------------------
    //������������(4096��)����С������(4096��)
    //��֤��С�����ʲ���С��3Td/peak���������ʲ��ܴ���(peak-3Td)/peak��
    //gus_max_percent=0x1000-[(ram_para[num_dead_time]+ram_para[num_IPM_min_time])*2*ram_para[num_pwm_carrier]*4096/100000] //��������
    //��gus_max_percent=0x1000-2*(ram_para[num_dead_time]+ram_para[num_IPM_min_time])*ram_para[num_pwm_carrier]*128*32/3125*32;
    lul_tmp = ram_para[num_dead_time] + ram_para[num_IPM_min_time];
    lul_tmp *= ram_para[num_pwm_carrier];
    lul_tmp *= 128;
    lul_tmp /= 3125;
    gus_min_percent = lul_tmp * 2;
    if (ram_para[num_max_k_set] == 0)
    {
        gus_max_percent = 0x1000 - lul_tmp;
        gus_max_percent_gu = 0x1000-2*lul_tmp;
    }
    else if (ram_para[num_max_k_set] == 1)       //���������
    {
    	  gus_max_percent = 0x1000;
    	  gus_max_percent_gu = 0x1000;
    }
    //-------------------------------------------------------
    //���㵥λ�����ʳ�ֵ
    //��λ��ѹ�����ʣ�[1/(ֱ����ѹ/sqrt(2))],�õ���λ�����ʵ�*(2^16)*4096��,��ѹ���ȵ�0.1V
    //percent_pwr_unit = 4096*(2^16)/[(Udc_val)/sqrt(2)] 
    //����Ϊ�ϵ�ʱ Udc=Uin*sqrt(2),����percent_pwr_unit�ĳ�ֵΪ 4096*(2^16)/Uin
    gul_percent_pwm_unit = 268435456 / ram_para[num_Uin_rate];
    //------------------------------------------------------
    //��������ʱ�����ֵgus_Td, Td����Ϊ0.1us
    //Td * (10^-6) / 10 = (2 * gus_Td + 1) / 40M
    //gus_Td = 2 * Td - (1/2)
    //gus_Td = 2 * Td    //(1/2)����
    gus_Td =(2 * ram_para[num_dead_time]);
    //------------------------------------------------------
    //�򿪻�ʱҪ�����Ծٳ��
    //��Ϊ�±����ϱۼ����෴�������±�ռ�ձ�0����50��ʱ����Ӧռ�ձȴ�100������50��   
    gus_val_a = gus_peak_pwm + 1;
    gus_val_b = gus_peak_pwm + 1;
    gus_val_c = gus_peak_pwm + 1;  //���������ֵ����ֵ
    //------------------------------------------------------
    //��ʼ��PWM����Ƚ�ֵ������Ĵ���   
    gus_max_comp_val = gus_half_pwm;    //��ǰPWM����Ƚ�ֵ�����ֵ�Ĵ�������ֵ
    gus_mid_comp_val = gus_half_pwm;    //��ǰPWM����Ƚ�ֵ���м�ֵ�Ĵ�������ֵ
    gus_min_comp_val = gus_half_pwm;    //��ǰPWM����Ƚ�ֵ����Сֵ�Ĵ�������ֵ
    //------------------------------------------------------
    //����rate_per��ֵ{��СƵ�ʵ�λ(0.01Hz)��Ӧ�ĵ�λ���������ڵ���λ��}
    lul_tmp = SIN_TAB_LENGTH_4;   //ÿ0.1��ȡһ���㣬��һ����������SIN_TAB_LENGTH_4=3600����
    lul_tmp <<= 16;      //gus_rate_per=��2^16��*(SIN_TAB_LENGTH_4)/(10000*f_carrier);  0.01Hz��һ���ز������ڶ�Ӧ�Ķ���(��λ0.1��)
    lul_tmp /= 10000;
    lul_tmp /= ram_para[num_pwm_carrier];    //ÿ���ز����ڷ���һ��PWM�ж�
    gus_rate_per = lul_tmp;
    //------------------------------------------------------
    //��ʼ��A�����λ�ǼĴ����Լ������ʼĴ����������ʼ���ֵ�Ĵ���
    theta.ulword = 0;       //ѡ��A����Ʋ�����λ��Ϊ0
    gus_percent_k = 0;       //�����ʼĴ�����0
    gus_percent_pwm = 0;    //�����ʼ���ֵ�Ĵ�����0
    
    gus_now_theta = 0;
    gus_next_theta = 0;
    //------------------------------------------------------
    //ѡ��TM20��ʱ��ԴΪMCLK
    //����AD����ʱ�̵Ĳ���ʱ�� (T/(1/40M)=(X/10)*(10^-6)*40*(10^6)=X*4
    lul_tmp = ram_para[num_ADrepair_time];
    lul_tmp *= 4;
    gus_ADtrigger_repair_val = lul_tmp;  
    //------------------------------------------------------
    //ѡ��TM20��ʱ��ԴΪMCLK
    //����AD�Ĳ�������ʱ�� (T/(1/40M)=(X/10)*(10^-6)*40*(10^6)=X*4
    lul_tmp = ram_para[num_AD_SH_time];
    lul_tmp *=6;
    gus_AD_SH_val = lul_tmp;
    //------------------------------------------------------
    gus_TUP_ADtrigger = gus_half_pwm;
    gus_TDW_ADtrigger = gus_half_pwm;
    
    //PWM�����ж�ʹ��ʱ,����TM20����AD����ʱ�̼�ʱ
    TM20CB = gus_TUP_ADtrigger;
    TM20CA = (2*gus_peak_pwm - gus_TDW_ADtrigger);
    
    //--------------------------------------------------------------------------
    //G16ICR����16�жϿ��ƼĴ���(����PWM0���������硢�����ж�)
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  LV2 |  LV1 |  LV0 |  -   |  -   |  IE1 |  IE0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | IR1  | IR1  |  -   |  -   | ID1  | ID0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //LV2~LV0 : �ж����ȼ�����, 0��7(0���  7��ֹ)
    //IE1~IE0 : �ж�ʹ�ܱ�־
    //IR1~IR0 : �ж������־
    //ID1~ID0 : �жϼ���־     
    //--------------------------------------------------------------------------
    G16ICR = 0x3000;    //����PWM���硢������ж����ȼ�Ϊ3 (������ALM��Hall��AD)
    //--------------------------------------------------------------------------
    
    //PWMMD0A:PWM0ģʽ���ƼĴ���
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12   |  11    |  10    |  9    |  8    | 
    // flag    |  -   |  -   |SFTEN0|CLKSEL0|TMSTAEN0|TMSTBEN0|SDSELA0|SDSELB0| 
    //at reset |  0   |  0   |  0   |  0    |  0     |  0     |  0    |  0    |
    //--------------------------------------------------------------------------
    //reg_bit  |  7    |  6    |  5    |  4    |  3   |  2   |  1   |  0    |  
    // flag    |PCRAEN0|PCRBEN0|INTAEN0|INTBEN0|DTEN0 |ORMD0 |TCEN0 |WAVEMD0| 
    //at reset |  0    |  0    |  0    |  0    |  0   |  0   |  0   |  0    |
    //--------------------------------------------------------------------------
    //SFTEN0: ���ʵʱ�������ѡ��   0=��ֹ    1=ʹ��
    //CLKSEL0: ����ʱ��ѡ��      0=IOCLK   1=MCLK
    //TMSTAEN0: ͨ��PWM0���������缤��TM20��AD0��AD1��AD2ѡ��   0=��ֹ    1=ʹ��
    //TMSTBEN0: ͨ��PWM0���������缤��TM20��AD0��AD1��AD2ѡ��   0=��ֹ    1=ʹ��
    //SDSELA0:  OUTMD0������ģʽѡ��    0=������ģʽ     1=˫����ģʽ
    //SDSELB0:  PWMSEL0������ģʽѡ��   0=������ģʽ     1=˫����ģʽ
    //PCRAEN0:  ˫�������ʱ��ѡ��(PWM0����������)   0=��ֹ    1=ʹ��
    //PCRBEN0:  ˫�������ʱ��ѡ��(PWM0����������)   0=��ֹ    1=ʹ��
    //INTAEN0:  ��ʱ���ж�ʱ��ѡ��(PWM0����������)   0=��ֹ    1=ʹ��
    //INTBEN0:  ��ʱ���ж�ʱ��ѡ��(PWM0����������)   0=��ֹ    1=ʹ��
    //DTEN0:    ����ʱ�����ѡ��   0=������ʱ��      1=������ʱ��
    //ORMD0:    ����ʱ������߼�   0=���߼�(L��Ч)   1=���߼�(H��Ч)
    //TCEN0:    PWM��������ʹ��    0=��ֹ    1=ʹ��
    //WAVEMD0:  PWM�ز�ģʽ        0=���ǲ�  1=��ݲ�
    //--------------------------------------------------------------------------
    PWMMD0A = 0x0000;
    
    PWMMD0A |= 0x1000;   //ѡ��PWM����ʱ��ΪMCLK
    PWMMD0A |= 0x0800;   //ͨ��PWM0���������缤��TM20��AD0��AD1��AD2��ʹ�� (����AD����)
    PWMMD0A |= 0x0100;   //PWMSEL0������ģʽѡ��˫����ģʽ
    PWMMD0A |= 0x0080;   //����˫����ʹ��
    PWMMD0A |= 0x0020;   //��ʱ�������ж�ʹ��
    PWMMD0A &= ~0x0004;  //����ʱ�����Ϊ���߼�
    
    PWMMD0A &= 0xFFFC;   //PWM�ز�Ϊ���ǲ�����ֹPWM0����������
    
    if(gus_Td != 0)        //�������ʱ�䲻Ϊ�㣬������Ϊ������ʱ�����
    {
        PWMMD0A |= 0x0008;   //������ʱ�����
    }
    else
    {
        PWMMD0A &= 0xFFF7;   //������ʱ�����   	  
    }
    //--------------------------------------------------------------------------
    //OUTMD0:PWM0������Կ��ƼĴ���
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   |  NW  |  W   |  NV  |  V   |  NU  |  U   | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //PXDTNx0 : 0=Negative phase  1=Positive phase
    //PXDTx0  : 0=Positive phase  1=Negative phase
    //--------------------------------------------------------------------------
    OUTMD0 = 0x003F;
    
    //PWMSEL0��PWM0������ƼĴ���
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   | PSEL | PSEL | PSEL | PSEL |
    // flag    |  -   |  -   |  -   |  -   | N02  |  02  | N01  |  01  |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   | 
    // flag    | PSEL | PSEL | OTLV | OTLV | OTLV | OTLV | OTLV | OTLV | 
    // flag    | N00  |  00  | N02  |  02  | N01  |  01  | N00  |  00  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //PSELN0x��PSEL0x :  0=PWM output       1=H/L level output
    //OTLVN0x��OTLV0x �� 0=L level output   1=H level output
    //--------------------------------------------------------------------------
    //PWMSEL0 = 0x0FFF;   //all pin for H  level output
    
    //PWMSET0: PWM0�����趨�Ĵ���
    PWMSET0 = gus_peak_pwm;
    
    //TCMP0x: PWM0ռ�ձ��趨�Ĵ���
    TCMP0A = gus_val_a;  //PWM00ռ�ձ��趨  W��
    TCMP0B = gus_val_b;  //PWM01ռ�ձ��趨  V��
    TCMP0C = gus_val_c;  //PWM02ռ�ձ��趨  U��
    
    //DTMSET0A: PWM0����ʱ���趨�Ĵ���A
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |  -   |  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |                 DTST0A7~DTST0A0                       | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //DTST0A7~DTST0A0 �� (Dead_time*clock_cycle)/2 -1
    //--------------------------------------------------------------------------
    DTMSET0A = gus_Td;
    
    //DTMSET0A: PWM0����ʱ���趨�Ĵ���B
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  -   |  -   |  -   |DT0BEN| 
    //at reset |  -   |  -   |  -   |  -   |  -   |  -   |  -   |  0   | 
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |                 DTST0B7~DTST0B0                       | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //DT0BEN :  0=Same dead time value is inserted at PWM0 output. 
    //          1=Another dead time values are inserted at PWM0 output
    //DTST0B7~DTST0B0 �� (Dead_time*clock_cycle)/2 -1
    //--------------------------------------------------------------------------
    DTMSET0B = (gus_Td | 0x0100);
    
    //PWMBC0: PWM0����ֵ���Ĵ���
    //PWMBC0  (only read)
    
    //BCSTR0: PWM0����ֵ״̬�Ĵ���
    //BCSTR0  (only read) bit0: PWM0STR 0=Down-count  1=Up-count (atreset 1)
    
    //--------------------------------------------------------------------------
    //G24ICR����24�жϿ��ƼĴ���(���ж�IRQ3)
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  LV2 |  LV1 |  LV0 |  -   |  -   |  -   |  IE0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | -    | IR0  |  -   |  -   | -    | ID0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //LV2~LV0 : �ж����ȼ�����, 0��7(0���  7��ֹ)
    //IE1~IE0 : �ж�ʹ�ܱ�־
    //IR1~IR0 : �ж������־
    //ID1~ID0 : �жϼ���־     
    //--------------------------------------------------------------------------
    G24ICR = 0x0000;       //���жϵ����ȼ�����Ϊ0 (��߼�)
    
    //PWMIRQCNT0: PWM0�ж�������ƼĴ���
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  IRQBSCNT03~IRQBSCNT00    |   IRQBCNT03~IRQBCNT00     |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   | 
    // flag    |  IRQASCNT03~IRQASCNT00    |   IRQACNT03~IRQACNT00     | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //IRQBSCNT03~IRQBSCNT00: ��PWM0���������缸�κ�,�����״������ж� 
    //IRQBCNT03~IRQBCNT00:  ��PWM0���������缸�κ�,�������������ж�
    //IRQASCNT03~IRQASCNT00: ��PWM0���������缸�κ�,�����״������ж�
    //IRQACNT03~IRQACNT00:  ��PWM0���������缸�κ�,�������������ж�
    //--------------------------------------------------------------------------
    PWMIRQCNT0 = 0x0000;      //ÿ�ζ��ж�
    
    //PWMOFF0A: PWM0���ű������ƼĴ���A
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  PRTNW01~00 |  PRTW01~00  |  PRTNV01~00 |  PRTV01~00  |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   | 
    // flag    |  PRTNU01~00 |  PRTU01~00  |  -   | IRQSEL01~00 |OUTEN0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //PRTNW01~00,PRTW01~00: output pin protection function
    //                      00=PWM output (protection disabled)   01=Hi-Z output 
    //                      10=Inactive output                    11=Unused         
    //IRQSEL01~00: PWM0 pin protection External interrupt selection
    //                      00=IRQ00   01=IRQ01  10=IRQ02   11=IRQ03                  
    //OUTEN0:  PWM0pin output enable   0=Disabled   1=Enabled
    //--------------------------------------------------------------------------
    //��������ʱ���ϱ������Ч��ƽ���±۳���Ч��ƽ�����ж�ѡ��ΪIRQ03,�����ʹ�� 
    PWMOFF0A = 0x0000;    //δ��
    
    //PWMOFF0A1: PWM0���ű������ƼĴ���A1
    //--------------------------------------------------------------------------
    //reg_bit  |  15                ~                      1    |  0   | 
    // flag    |  --------------------------------------------  |NMIEN0|
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //NMIEN0:  PWM0 pin output protection function by NMI enable
    //         0=Disabled   1=Enabled
    //--------------------------------------------------------------------------
    PWMOFF0A1 = 0x0001;
    
    //PWMOFF0B: PWM0���ű������ƼĴ���B   
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |   9    |    8    | 
    // flag    |   PRTBCNT03 ~PRTBCNT00    |  -   |  -   |PRTBBKA0|PRTBBKB0 |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |   0    |    0    |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4    |  3  |  2   |  1   |  0    | 
    // flag    |  -   | PRTBMD01~00 |IRQPHA0|  -  | IRQSELB01~00|PRTBEN0|
    //at reset |  0   |  0   |  0   |  0    |  0  |  0   |  0   |  0    |
    //--------------------------------------------------------------------------
    //PRTBCNT03 ~PRTBCNT00: ���ñ��������жϲ�������,�����жϴ����ﵽ�趨ֵ�󱣻�
    //PRTBBKA0: ͨ��PWMBC0��������������ѡ��,0=Disabled   1=Enabled
    //PRTBBKB0: ͨ��PWMBC0��������������ѡ��,0=Disabled   1=Enabled
    //PRTBMD01~00: PWM0���ű���B����ѡ��,
    //             00=PWM output(protection disabled),01=All pin is inactive output
    //             10=Only PWM00,01,02 is inactive output 11=The pin is automatically controlled by a state
    //IRQPHA0:  ���жϵ�ƽѡ�� 0=L   1=H
    //IRQSELB01~00: PWM0���ű���B���ж�ѡ�� 00=IRQ00   01=IRQ01  10=IRQ02   11=IRQ03 
    //PRTBEN0:  PWM0���ű���B����ʹ��ѡ�� 0=Disabled   1=Enabled
    //--------------------------------------------------------------------------
    //����Ϊÿ�����ж϶�����,����������PWM����,���жϵ�ƽΪL,�������ж�ΪIRQ03,������ʹ��
    PWMOFF0B = 0x0000;   //������ʹ��
    
    //PWMOFF0B1: PWM0���ű������ƼĴ���B1
    //--------------------------------------------------------------------------
    //reg_bit  |  15                ~                      1    |  0    | 
    // flag    |  --------------------------------------------  |PRTBST0|
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0    |
    //--------------------------------------------------------------------------
    //PRTBST0:  State of the PWM0 pin protection B operation  0=Disabled   1=Enabled
    //--------------------------------------------------------------------------
    //PWMOFF0B1 =0x0000;
    //��ֹ�ϵ�ʱ�󱣻��������Ծ�ǰ��������
    /*if (ram_para[num_prot_ALM_cnt] == 0) //��������趨��������Ϊ0
    {
   	    PWMOFF0B = 0xF026;
   	    
   	    PWMOFF0B1 &=~0x0001;      //��PRTBST0λ���ͷ�PWM0����Ӳ������B����
   	    PWMOFF0B |= 0x0001;
    }
    else
    {
   	    PWMOFF0B = 0x0000;      //����������Ϊ0ʱ������Ӳ������
    }*/
    
    //--------------------------------------------------------------------------
    ///P0MD = 0x08;    //P03 is TM5IO pin/IRQ03 pin(0=IO 1=Special function)
    
    //IRQ03 32���������˲��������˲�ʹ��
    NFCLK0 |= 0x0080;//0x00C0;//
    NFCNT0 |= 0x0008;
    
    //IRQ03 �½��ش����ж�
    EXTMD0 &= ~0x00C0;
    EXTMD0 |= 0x0040;
    
    //--------------------------------------------------------------------------
    //G24ICR����16�жϿ��ƼĴ���(���ж�IRQ3)
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  LV2 |  LV1 |  LV0 |  -   |  -   |  -   |  IE0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | -    | IR0  |  -   |  -   | -    | ID0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //LV2~LV0 : �ж����ȼ�����, 0��7(0���  7���)
    //IE1~IE0 : �ж�ʹ�ܱ�־
    //IR1~IR0 : �ж������־
    //ID1~ID0 : �жϼ���־     
    //--------------------------------------------------------------------------
    //����IRQ03���жϣ�����ALM����
    *(unsigned char *) & G24ICR = 0x01; //���IRQ03���ж������־
    //G24ICR |= 0x0100;       //��IRQ03���ж�ʹ�ܱ�־   //��ֹ�ϵ�ʱ�󱣻��������Ծ�ǰ��������
    
    //--------------------------------------------------------------------------
    //PWMDCNT0A: PWM0���ʱ����ƼĴ���A (����PWM00 and NPWM00)
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |   9  |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  -   |  -   |SDIRU0|SENU0 |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |   0  |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4    |  3  |  2   |  1   |  0    | 
    // flag    |                STIMU07 ~ STIMU00                       |
    //at reset |  0   |  0   |  0   |  0    |  0  |  0   |  0   |  0    |
    //--------------------------------------------------------------------------
    //SDIRU0:  PWM00 and NPWM00�ƶ��������  0=��ǰ��   1=�����
    //SENU0��  PWM00 and NPWM00�����ƶ�����  0=��ֹ     1=ʹ��
    //STIMU07 ~ STIMU00:  PWM00 and NPWM00����ƶ�����
    //--------------------------------------------------------------------------
    PWMDCNT0A = 0x0000;
    
    //PWMDCNT0B: PWM0���ʱ����ƼĴ���B (����PWM01 and NPWM01,��PWMDCNT0A��������)
    PWMDCNT0B = 0x0000;
    
    //PWMDCNT0C: PWM0���ʱ����ƼĴ���C (����PWM02 and NPWM02,��PWMDCNT0A��������)
    PWMDCNT0C = 0x0000;
    
    //--------------------------------------------------------------------------
    *(unsigned char *) & G16ICR = 0x03; //���PWM���硢���������־
    G16ICR |= 0x0100;    //��PWM����ʹ�ܱ�־   
    //------------------------------------------------------
    PWMSEL0 = 0x0FC0;    //PWM�������������,�ϱ�,�±�ȫ��Ч
    PWMOFF0A |= 0x0001;  //PWM���ʹ��
    //------------------------------------------------------
    //����P80~P85ΪPWM����
    P8MD |= 0x3F;  //P80~P85 Special function is PWM pin  (0=IO 1=PWM) 
    //------------------------------------------------------
    PWMMD0A |= 0x0002;   //PWM��������ʹ��
    //------------------------------------------------------
}
//-----------------------------------------------------------------------------------
void freq_rate_init(void)  //��ʼ��1msƵ���������ĳ��� �Ӽ����������ó����ڿ��Ƴ�ʼ�������е���
{
    lword_type ltmp;
    //------------------------------------------------------
    //��Ϊ�Ӽ���ʱ��Ķ���Ϊÿ50.00Hz��Ҫ�೤ʱ�䣬�����÷�ΧΪ0.1~600.0s
    //�����ڼ�������ʱ����2^24
    //50.00/(0.1~600.0)*1000ms=5000/(1~6000)*100=50/(1~6000)��1ms���Ӷ��ٸ�0.01Hz
    ltmp.ulword = 50;
    ltmp.ulword <<= 24;
    
    freq_rate.normal_accel = (ltmp.ulword / ram_para[num_accel_time]);
    freq_rate.normal_decel = (ltmp.ulword / ram_para[num_decel_time]);
    
    freq_rate.sum_accel.ulword = 0;
    freq_rate.sum_decel.ulword = 0;
    //------------------------------------------------------
}
//------------------------------------------------------------------------------
void open_pwm(void)   //����PWM�ĳ����ڿ��������е���
{
    //------------------------------------------------------
    gus_max_comp_val = gus_half_pwm;    //��ǰPWM����Ƚ�ֵ�����ֵ�Ĵ�������ֵ
    gus_mid_comp_val = gus_half_pwm;    //��ǰPWM����Ƚ�ֵ���м�ֵ�Ĵ�������ֵ
    gus_min_comp_val = gus_half_pwm;    //��ǰPWM����Ƚ�ֵ����Сֵ�Ĵ�������ֵ
    //------------------------------------------------------
    //ռ�ձȻ���Ĵ�������ֵ
    gus_val_a = gus_half_pwm;
    gus_val_b = gus_half_pwm;
    gus_val_c = gus_half_pwm;
    //------------------------------------------------------
    //��ʼ��TCMP0x: PWM0ռ�ձ��趨�Ĵ���
    TCMP0A = gus_val_a;     //PWM00ռ�ձ��趨  W��
    TCMP0B = gus_val_b;     //PWM01ռ�ձ��趨  V��
    TCMP0C = gus_val_c;     //PWM02ռ�ձ��趨  U��
    //-------------------------------------------------------
    //PWMSEL0��PWM0������ƼĴ���
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   | PSEL | PSEL | PSEL | PSEL |
    // flag    |  -   |  -   |  -   |  -   | N02  |  02  | N01  |  01  |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   | 
    // flag    | PSEL | PSEL | OTLV | OTLV | OTLV | OTLV | OTLV | OTLV | 
    // flag    | N00  |  00  | N02  |  02  | N01  |  01  | N00  |  00  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //PSELN0x��PSEL0x :  0=PWM output       1=H/L level output
    //OTLVN0x��OTLV0x �� 0=L level output   1=H level output
    //------------------------------------------------------
    PWMSEL0 = 0x003F;    //PWM������PWM���ܿ���
    //------------------------------------------------------
}
//------------------------------------------------------------------------------
void shut_pwm(void)      //�ر�PWM�ĳ�����ͣ������򱣻�ͣ�������е���
{
    //------------------------------------------------------
    PWMSEL0 = 0x0FC0;    //PWM������������ƣ�ȫ��Ч
    //------------------------------------------------------
    gus_val_a = gus_peak_pwm + 1;
    gus_val_b = gus_peak_pwm + 1;
    gus_val_c = gus_peak_pwm + 1;  //���������ֵ����ֵ

    TCMP0A = gus_val_a;  //PWM00ռ�ձ��趨  W��
    TCMP0B = gus_val_b;  //PWM01ռ�ձ��趨  V��
    TCMP0C = gus_val_c;  //PWM02ռ�ձ��趨  U��
    //------------------------------------------------------
    //��ʼ��WM����Ƚ�ֵ������Ĵ���   
    gus_max_comp_val = gus_half_pwm;    //��ǰPWM����Ƚ�ֵ�����ֵ�Ĵ�������ֵ
    gus_mid_comp_val = gus_half_pwm;    //��ǰPWM����Ƚ�ֵ���м�ֵ�Ĵ�������ֵ
    gus_min_comp_val = gus_half_pwm;    //��ǰPWM����Ƚ�ֵ����Сֵ�Ĵ�������ֵ

    rate_theta_buf.ulword = 0;
    rate_theta.ulword = 0;
    theta.ulword = 0; 
    
    gus_now_theta = 0;
    gus_next_theta = 0;
         
    gus_percent_k =0;
    
    gus_percent_pwm_buf = 0;
    gus_percent_pwm = 0;
    
    gus_freq_set = 0;
    //gus_freq_out_prev = 0;    
    gus_freq_out = 0;
    
    //bflg_theta_ctrl1 = 0;
    //bflg_theta_ctrl2 = 0;
    //gus_freq_real = 0;
    
    //gus_speed_real = 0;
    gus_Uout_for_disp = 0;
    gus_Uout_real = 0;
    gsl_FreqPI_integral = 0;
}
//------------------------------------------------------------------------------
void bootstrap_charger_delaytime(void)  //�Ծٳ����ʱ������1ms��ʱ�����е���
{
	  if (bflg_bootstrap_charger_delaytime == 1)
	  {
	  	  gss_bootstrap_charger_delaytimer++;
	  	  
	  	  if (gss_bootstrap_charger_delaytimer > 700)
	  	  {
	  	  	  bflg_bootstrap_charger_delaytime = 0; //500��ʱ�������Ծٵ�·����־
	  	  	  gss_bootstrap_charger_delaytimer = 0; //���ʱ��
	  	  }
	  	  else if (gss_bootstrap_charger_delaytimer > 400)
	  	  {
	  	  	  gus_val_a = gus_half_pwm;
	  	  	  gus_val_b = gus_half_pwm;
	  	  	  gus_val_c = gus_half_pwm;
	  	  	  
	  	  	  TCMP0A = gus_val_a;
	  	  	  TCMP0B = gus_val_b;
	  	  	  TCMP0C = gus_val_c;
	  	  	  
	  	  	  PWMSEL0 = 0x0540;
	  	  }
	  	  else if (gss_bootstrap_charger_delaytimer > 300)
	  	  {
	  	  	  gus_val_a = gus_half_pwm;
            gus_val_b = gus_half_pwm;
            gus_val_c -= gus_centi_half_pwm;
            
            TCMP0A = gus_val_a;
	  	  	  TCMP0B = gus_val_b;
	  	  	  TCMP0C = gus_val_c;
	  	  	  
	  	  	  PWMSEL0 = 0x0540; //100ms��W�±�Ҳ��ʼPWM���
	  	  }
	  	  else if (gss_bootstrap_charger_delaytimer > 200)
	  	  {
	  	  	  gus_val_a = gus_half_pwm;
            gus_val_b -= gus_centi_half_pwm;
            gus_val_c = gus_peak_pwm + 1;
            
            TCMP0A = gus_val_a;
	  	  	  TCMP0B = gus_val_b;
	  	  	  TCMP0C = gus_val_c;
	  	  	  
	  	  	  PWMSEL0 = 0x0D40; //100ms��V�±�Ҳ��ʼPWM���
	  	  }
	  	  else if (gss_bootstrap_charger_delaytimer > 100)
	  	  {
	  	  	  gus_val_a -= gus_centi_half_pwm;
            gus_val_b = gus_peak_pwm + 1;
            gus_val_c = gus_peak_pwm + 1;
            
            TCMP0A = gus_val_a;
	  	  	  TCMP0B = gus_val_b;
	  	  	  TCMP0C = gus_val_c;
	  	  	  
	  	  	  PWMSEL0 = 0x0F40; //U�±��ȿ�ʼPWM���
	  	  }
	  }
}
//------------------------------------------------------------------------------
void check_freq_updata(void)  //��ʱ����Ƶ�ʸ��µĳ���(ÿ1ms�ж��м��1��)
{
    uint16_t lus_tmp;
    uint16_t lus_stall_amp_accel;
    
    if (bflg_running == 1)    //����ʱ������
    {
        //gus_freq_out_prev = gus_freq_out;    //������һ�ε����Ƶ��
        //--------------------------------------------------
        if (bflg_current_direction == bflg_actual_direction)   //���ʵ��ת����Ҫ��ת��һ��
        {
		        if (gus_freq_set > gus_freq_out)     //����ʱ������
		        {
		            freq_rate.sum_decel.ulword = 0;  //����ʱ������ٲ���β���ۼƼĴ���
		            //bflg_ask_for_speeddown = 0;      //���Ҫ����ٱ�־λ
		            //bflg_ask_for_speedup = 1;        //��Ҫ����ٱ�־λ
		            //----------------------------------------------
		            //�õ���ǰ����ʱ��ʧ������
		            lus_stall_amp_accel = gus_prot_Iout_stall_amp > gus_prot_Tm_stall_amp ? gus_prot_Iout_stall_amp : gus_prot_Tm_stall_amp;           
		            gus_stall_amp_accel = lus_stall_amp_accel > gus_prot_Udc_stall_amp ? lus_stall_amp_accel : gus_prot_Udc_stall_amp;           
		            //----------------------------------------------
		            freq_rate.sum_accel.ulword += (freq_rate.normal_accel >> gus_stall_amp_accel);
		            //----------------------------------------------
		            //hz_out�����޲��ô���600.0Hz
		            lus_tmp = gus_freq_out;
		            lus_tmp += freq_rate.sum_accel.ubyte.HH;  //ֻȡ���ֵĸ��ֽ�(����С2^24��)
		            freq_rate.sum_accel.ubyte.HH = 0;          //����ֵĸ��ֽڣ����������ֽڣ��Ա㱣���ӷ���β��         
		            //----------------------------------------------
		            if(gus_freq_set < lus_tmp)
		            {   
		               lus_tmp = gus_freq_set;
		            }
		            //----------------------------------------------
		            //����Ƶ���ж�
		            if (lus_tmp < ram_para[num_begin_freq])
		            {
		            	  lus_tmp = ram_para[num_begin_freq];
		            }
		            //----------------------------------------------
		            gus_freq_out = lus_tmp;
		            //----------------------------------------------
		        }
		        else if (gus_freq_set < gus_freq_out)//����ʱ������
		        {
		            freq_rate.sum_accel.ulword = 0;  //����ʱ������ٲ���β���ۼƼĴ���
		            //bflg_ask_for_speedup = 0;        //���Ҫ����ٱ�־λ
		            //bflg_ask_for_speeddown = 1;      //��Ҫ����ٱ�־λ
		            //----------------------------------------------
		            //�õ���ǰ����ʱ��ʧ������
		            gus_stall_amp_decel = gus_prot_Udc_stall_amp;
		            //----------------------------------------------
		            freq_rate.sum_decel.ulword += (freq_rate.normal_decel >> gus_stall_amp_decel);
		            //----------------------------------------------
		            lus_tmp = gus_freq_out;
		            //----------------------------------------------
		            if (lus_tmp > freq_rate.sum_decel.ubyte.HH)
		            {
		                lus_tmp -= freq_rate.sum_decel.ubyte.HH;   //ֻȡ���ֵĸ��ֽ�(����С2^24��)
		            }
		            else
		            {
		                lus_tmp = 0;
		            }
		            //----------------------------------------------
		            freq_rate.sum_decel.ubyte.HH = 0; //����ֵĸ��ֽڣ����������ֽڣ��Ա㱣���ӷ���β��            
		            //----------------------------------------------
		            if(gus_freq_set > lus_tmp)
		            {
		                lus_tmp = gus_freq_set;
		            } 
		            //----------------------------------------------
		            gus_freq_out = lus_tmp;
		            //----------------------------------------------
		        }
		        else
		        {            
		            //bflg_ask_for_speedup = 0;        //���Ҫ����ٱ�־λ
		            //bflg_ask_for_speeddown = 0;      //���Ҫ����ٱ�־λ
		            freq_rate.sum_decel.ulword = 0;  //����ٲ���β���ۼƼĴ���
		            freq_rate.sum_accel.ulword = 0;  //����ٲ���β���ۼƼĴ���
		            //----------------------------------------------
		            gus_freq_out = gus_freq_set;
		            //----------------------------------------------
		            gus_stall_amp_accel = 0;
		            gus_stall_amp_decel = 0;
		            //----------------------------------------------
		        }
        }
        else
        {
        	  //gus_freq_out = 0;
        	  
        	      freq_rate.sum_accel.ulword = 0;  //����ʱ������ٲ���β���ۼƼĴ���
		            //bflg_ask_for_speedup = 0;        //���Ҫ����ٱ�־λ
		            //bflg_ask_for_speeddown = 1;      //��Ҫ����ٱ�־λ
		            //----------------------------------------------
		            //�õ���ǰ����ʱ��ʧ������
		            gus_stall_amp_decel = gus_prot_Udc_stall_amp;
		            //----------------------------------------------
		            freq_rate.sum_decel.ulword += (freq_rate.normal_decel >> (gus_stall_amp_decel << 1));
		            //----------------------------------------------
		            lus_tmp = gus_freq_out;
		            //----------------------------------------------
		            if (lus_tmp > freq_rate.sum_decel.ubyte.HH)
		            {
		                lus_tmp -= freq_rate.sum_decel.ubyte.HH;   //ֻȡ���ֵĸ��ֽ�(����С2^24��)
		            }
		            else
		            {
		                lus_tmp = 0;
		            }
		            //----------------------------------------------
		            freq_rate.sum_decel.ubyte.HH = 0;   //����ֵĸ��ֽڣ����������ֽڣ��Ա㱣���ӷ���β��            
		            //----------------------------------------------
		            /*if(gus_freq_set > lus_tmp)
		            {
		                lus_tmp = gus_freq_set;
		            }*/
		            //----------------------------------------------
		            gus_freq_out = lus_tmp;
		            //----------------------------------------------
        }
        //--------------------------------------------------
        //�������ת�� r=(f/100)*60/P
        /*lus_tmp = gus_freq_out;
        lus_tmp *= 3;
        lus_tmp /= motor_para[guc_motor_ID][MOTOR_p];
        lus_tmp /= 5;
        gus_speed_out = lus_tmp;*/
        //--------------------------------------------------
        /*if (bflg_startup_time == 1)     //����������׶Σ������
        {
            if (gus_freq_out >= ram_para[num_base_freq])    //���Ƶ�ʴ��ڵ��ڻ�׼Ƶ�ʣ����˳������׶�
            {
                bflg_startup_time = 0;  //�������׶α�־
            }
        }*/
    }
}
//------------------------------------------------------------------------------
//����ʵ�ʼ��Ƶ�����趨Ƶ�ʵĲ�ֵPI��ʵ�ʵ������ѹ
//------------------------------------------------------------------------------
void Sensored_cale_Vout(void)      //��λ�ô��������Ƽ��������ѹ�ĳ���
{
	  int32_t lsl_tmp;
	  //------------------------------------------------------
	  gus_PIctrl_delaytimer++;
	  if (gus_PIctrl_delaytimer >= ram_para[num_PIctrl_delaytime]) //PI��������
	  {
	  	  gus_PIctrl_delaytimer = 0;
	  	  //--------------------------------------------------
	  	  //Ksp�Ŵ�(2^15)*(2^8)��,Ksi�Ŵ�(2^15)*(2^8)��
	  	  //--------------------------------------------------
	  	  if (bflg_current_direction == bflg_actual_direction)
	  	  {
	  	  	  lsl_tmp = gus_freq_out;
	  	      lsl_tmp -= gus_Hall_freq;//gus_avr_realfreq;
	  	  }
	  	  else
	  	  {
	  	  	  lsl_tmp = gus_Hall_freq;//gus_avr_realfreq;
	  	      lsl_tmp -= gus_freq_out;
	  	  }
	  	  //lsl_tmp = gus_freq_out;
	  	  //lsl_tmp -= gus_avr_realfreq;
	  	  lsl_tmp *= ram_para[num_Ksi];
	  	  gsl_FreqPI_integral += lsl_tmp;
	  	  if (gsl_FreqPI_integral < 0)
	  	  {
	  	  	  gsl_FreqPI_integral = 0;
	  	  }
	  	  else if (gsl_FreqPI_integral > 8388608)//32768 * 256)//debug
	  	  {
	  	  	  gsl_FreqPI_integral = 8388608;//32768 * 256;
	  	  }
	  	  //--------------------------------------------------
	  	  //lsl_tmp = gss_PI_DeltSpeed;
	  	  if (bflg_current_direction == bflg_actual_direction)
	  	  {
	  	  	  lsl_tmp = gus_freq_out;
	  	      lsl_tmp -= gus_Hall_freq;//gus_avr_realfreq;
	  	  }
	  	  else
	  	  {
	  	  	  lsl_tmp = gus_Hall_freq;//gus_avr_realfreq;
	  	      lsl_tmp -= gus_freq_out;
	  	  }
	  	  //lsl_tmp = gus_freq_out;
	  	  //lsl_tmp -= gus_avr_realfreq;
	  	  lsl_tmp *= ram_para[num_Ksp];
	  	  lsl_tmp += gsl_FreqPI_integral;    //���ֻ��ڵ��ӵ�����������
	  	  //--------------------------------------------------
	  	  if (lsl_tmp < 0)
	  	  {
	  	  	  lsl_tmp = 0; //��СΪ��
	  	  }
	  	  else if (lsl_tmp > 8388608)//32768 * 256)
	  	  {
	  	  	  lsl_tmp = 8388608;//32768 * 256;  //����ܳ������ĵ�����
	  	  }
	  	  //--------------------------------------------------
	  	  //��С32768����Ϊʵ�ʵ�����,�ٳ����������ߵ�ѹ����Чֵ���õ�Ҫ��������ߵ�ѹ
	  	  lsl_tmp >>= 8;
	  	  lsl_tmp *= ram_para[num_V_max];
	  	  lsl_tmp >>= 15;       //�����������С(2^15)��
	  	  //--------------------------------------------------
	  	  gus_Uout_real = (uint16_t) lsl_tmp;   //�õ�Ҫ��������ߵ�ѹ�����ȵ�0.1V�����ڼ��������
	  }
}
//-------------------------------------------------------------------------------
void calc_percent_theta(void)      //���ڼ��㵱ǰ�����ʺ���λ�ǲ����ĳ���
{
    lword_type lul_tmp;
    int32_t lsl_tmp;
    //------------------------------------------------------
    lsl_tmp = gus_Udc_ad;
    lsl_tmp -= ram_para[num_Udc_ref_AD];
    lsl_tmp *= 10;                                //����10����ѹ��������0.1V
    lsl_tmp *= ram_para[num_Udc_gain];
    lsl_tmp >>= ram_para[num_Udc_gain_amp];
    lsl_tmp += (ram_para[num_Udc_ref]*10);        //Udc��ѹ����Ϊ0.1V
    
    if (lsl_tmp < 1000) lsl_tmp = 1000; //��ֹ���
    
    gus_Udc_tmp = lsl_tmp;
    
    //��λ��ѹ�����ʣ�[1/(ֱ����ѹ/sqrt(2))],�õ���λ�����ʵ�*(2^16)*4096��,��ѹ���ȵ�0.1V
    //percent_pwr_unit = 4096*(2^16)/[(Udc_val)/sqrt(2)]
    gul_percent_pwm_unit = 379625062 / gus_Udc_tmp;
    //------------------------------------------------------
    //���㵱ǰ������
    lul_tmp.ulword = gul_percent_pwm_unit;
    lul_tmp.ulword *= gus_Uout_real;
    //------------------------------------------------------
    if (ram_para[num_drive_mode] == 0)  //���������̨�β�����ģʽ�������
    {
        bflg_taiwave_ctrl = 0; 
    }       
    else  //�������̨�Ͳ�
    {
        //�����Ƶ�ʴ����л�Ƶ�ʣ���ʼ����̨�β��������
        if (bflg_taiwave_ctrl == 0)
        {
            //���ﵽ�л�Ƶ�ʺ��ҵ����ʴ����л������ʣ��л�Ϊ̨�β�����
            if (lul_tmp.uword.high > gus_max_percent_gu)
            {
                bflg_askfor_guwave_ctrl = 0;      //�����������β����Ʊ�־����֤����
                bflg_askfor_taiwave_ctrl = 1;     //���������̨�β�������־
            }
        }
        else
        {
            //�������С��̨�β���С�����ʣ����л��ع��β�����
            if (lul_tmp.uword.high < (gus_max_percent_gu - 200))
            {
                bflg_askfor_taiwave_ctrl = 0;     //���������̨�β����Ʊ�־����֤����
                bflg_askfor_guwave_ctrl = 1;      //�����������β����Ʊ�־
            }
        }
    }
    //------------------------------------------------------
    if (((bflg_taiwave_ctrl == 1) && (bflg_askfor_guwave_ctrl == 0)) || (bflg_askfor_taiwave_ctrl == 1))
    {   //�������̨�Ͳ�
    	  //̨�β�����ʱΪ4K*gus_peak_pwm
    	  if (lul_tmp.uword.high > gus_max_percent) 
        {
            lul_tmp.uword.high = gus_max_percent;
        }
        //--------------------------------------------------
        if (lul_tmp.uword.high < gus_min_percent)
        {
            lul_tmp.uword.high = gus_min_percent; //̨�β�����ʱ�������ʲ���С����С����
        }
        //--------------------------------------------------
        gus_percent_k = lul_tmp.uword.high;       //ʵ�ʵ����ʵ�4096��
        //--------------------------------------------------
        lul_tmp.ulword = lul_tmp.uword.high;
        lul_tmp.ulword *= gus_peak_pwm;
        lul_tmp.ulword >>= 10;
        gus_percent_pwm_buf = lul_tmp.ulword;     //��С1024��������С4����Ϊ�˸�sin���ֵ��˺�ȡ����
    }
    else
    {
    	  //���β�����ʱ��Ϊ4K*half
    	  if (lul_tmp.uword.high > gus_max_percent_gu) 
        {
            lul_tmp.uword.high = gus_max_percent_gu;
        }
        //--------------------------------------------------
        gus_percent_k = lul_tmp.uword.high;       //ʵ�ʵ����ʵ�4096��
        //--------------------------------------------------
        lul_tmp.ulword = lul_tmp.uword.high;
        lul_tmp.ulword *= gus_half_pwm;
        lul_tmp.ulword >>= 10;
        gus_percent_pwm_buf = lul_tmp.ulword;   //��С1024��������С4����Ϊ�˸�sin���ֵ��˺�ȡ����
    }
    //------------------------------------------------------
    //����ʵ�������ѹ��ʾֵ   Udc*percent_K/4096/(2^0.5)
    lul_tmp.ulword = gus_Udc_for_disp;
    lul_tmp.ulword *= gus_percent_k;
    lul_tmp.ulword *= 181;
    lul_tmp.ulword >>= 12;
    lul_tmp.ulword >>= 8;
    gus_Uout_for_disp = lul_tmp.ulword; 
    //------------------------------------------------------
    //����rate_thetaֵ;rate_per�ļ����Ѿ��ڳ�ʼ���׶���ɣ�����ֻҪ��һ���˷��Ϳ�����
    //if (bflg_actual_direction != bflg_current_direction)
    //{
    //	  rate_theta_buf.ulword = 0;
    //}
    //else
    //{
    	  if (bflg_theta_ctrl2 == 0)///
    	  {
    	  	  rate_theta_buf.ulword = 0;
    	  }
    	  else
    	  {
    	  	  rate_theta_buf.ulword = gus_Hall_freq;
            rate_theta_buf.ulword *= gus_rate_per;
    	  }
    //}
    //rate_theta_buf.ulword = gus_Hall_freq;
    //rate_theta_buf.ulword *= gus_rate_per;
    //------------------------------------------------------
    bflg_pwm_allow_updata = 1;//��pwm������±�־
}
//------------------------------------------------------------------------------
void calc_comp_val(void)      //������һ����������ıȽϼ���ֵ���ڲ��η����ж��е���  
{
    //��ڲ���gus_percent_pwm��rate_theta
    //���ڲ���theta��gus_val_a,gus_val_b,gus_val_c
    
    uint16_t lus_next_theta;  //��һ���ز����ڵ�A����λ��
    lword_type lul_theta;

    //������������Ƚϼ���ֵ
    //------------------------------------------------------
    theta.ulword += rate_theta.ulword;
    while (theta.uword.high >= SIN_TAB_LENGTH_4)
    {
        theta.uword.high -= SIN_TAB_LENGTH_4;
    }
    //------------------------------------------------------
    lul_theta.ulword = theta.ulword;
    lul_theta.uword.high += gus_phase_adv_degree;
    while (lul_theta.uword.high >= SIN_TAB_LENGTH_4)
    {
        lul_theta.uword.high -= SIN_TAB_LENGTH_4;
    }
    //------------------------------------------------------
    if (bflg_taiwave_ctrl == 1)    //�����̨�β����ƣ������
    {
        lus_next_theta = lul_theta.uword.high;
        gus_val_a = get_tai_comp_value(lus_next_theta);
        //--------------------------------------------------
        lus_next_theta = lul_theta.uword.high + SIN_TAB_LENGTH_8_3;
        gus_val_b = get_tai_comp_value(lus_next_theta);
        //--------------------------------------------------
        lus_next_theta = lul_theta.uword.high + SIN_TAB_LENGTH_4_3;
        gus_val_c = get_tai_comp_value(lus_next_theta);
    }
    else       //����ǹ��β����ƣ������
    {
        lus_next_theta = lul_theta.uword.high;
        gus_val_a = get_svpwm_comp_value(lus_next_theta);
        //--------------------------------------------------
        lus_next_theta = lul_theta.uword.high + SIN_TAB_LENGTH_8_3;
        gus_val_b = get_svpwm_comp_value(lus_next_theta);
        //--------------------------------------------------
        lus_next_theta = lul_theta.uword.high + SIN_TAB_LENGTH_4_3;
        gus_val_c = get_svpwm_comp_value(lus_next_theta);  
    }
}
//------------------------------------------------------------------------------
int16_t get_sin_value(uint16_t now_sin_index)     //��sin��ĺ���
{
    short lsi_sin_val;
    
    while (now_sin_index >= SIN_TAB_LENGTH_4)
    {
        now_sin_index -= SIN_TAB_LENGTH_4;
    }
    //----------------------------------
    if (now_sin_index < SIN_TAB_LENGTH)
    {
        lsi_sin_val = sin_table[now_sin_index];
    }
    else if (now_sin_index < SIN_TAB_LENGTH_2)
    {
        now_sin_index = SIN_TAB_LENGTH_2 - now_sin_index;
        lsi_sin_val = sin_table[now_sin_index];
    }
    else if (now_sin_index < SIN_TAB_LENGTH_3)
    {  
        now_sin_index -= SIN_TAB_LENGTH_2;
        lsi_sin_val = -sin_table[now_sin_index];
    }
    else
    {
        now_sin_index = SIN_TAB_LENGTH_4 - now_sin_index;
        lsi_sin_val = -sin_table[now_sin_index];
    }
    
    return   lsi_sin_val; 
}
//------------------------------------------------------------------------------
uint16_t get_tai_comp_value(uint16_t now_index)   //̨�β�����ʱ����PWM�������ֵ�ĳ���
{
    lword_type lsl_tai_val;
    
    while (now_index >= SIN_TAB_LENGTH_4)
    {
        now_index -= SIN_TAB_LENGTH_4;
    } 
    //----------------------------------
    if (now_index < SIN_TAB_LENGTH_2_3)
    {
        now_index += SIN_TAB_LENGTH_1_3;
        lsl_tai_val.lword = get_sin_value(now_index);
        lsl_tai_val.lword *= gus_percent_pwm;
        
        if (lsl_tai_val.word.high < gss_min_pwm)//debug
        {
        	  lsl_tai_val.word.high = gss_min_pwm;
        }
       	if (lsl_tai_val.word.high > gss_max_pwm)
       	{
       		  lsl_tai_val.word.high = gss_max_pwm;
       	}
    }
    else if (now_index < SIN_TAB_LENGTH_4_3)
    {
        lsl_tai_val.word.high = gus_peak_pwm + 1; 
    }
    else if (now_index < SIN_TAB_LENGTH_2)
    {
        now_index -= SIN_TAB_LENGTH_1_3;
        lsl_tai_val.lword = get_sin_value(now_index);
        lsl_tai_val.lword *= gus_percent_pwm;
        
        if (lsl_tai_val.word.high < gss_min_pwm)//debug
        {
        	  lsl_tai_val.word.high = gss_min_pwm;
        }
       	if (lsl_tai_val.word.high > gss_max_pwm)
       	{
       		  lsl_tai_val.word.high = gss_max_pwm;
       	}
    }  
    else if (now_index < SIN_TAB_LENGTH_8_3)
    {
        now_index += SIN_TAB_LENGTH_1_3;
        lsl_tai_val.lword = get_sin_value(now_index);
        lsl_tai_val.lword *= gus_percent_pwm;
        lsl_tai_val.word.high += gus_peak_pwm; 
        
        if (lsl_tai_val.word.high < gss_min_pwm)//debug
        {
        	  lsl_tai_val.word.high = gss_min_pwm;
        }
       	if (lsl_tai_val.word.high > gss_max_pwm)
       	{
       		  lsl_tai_val.word.high = gss_max_pwm;
       	}
    }
    else if (now_index < SIN_TAB_LENGTH_10_3)
    {
        lsl_tai_val.word.high = 0;
    }
    else if (now_index < SIN_TAB_LENGTH_4)
    {
        now_index -= SIN_TAB_LENGTH_1_3;
        lsl_tai_val.lword = get_sin_value(now_index);
        lsl_tai_val.lword *= gus_percent_pwm;
        lsl_tai_val.word.high += gus_peak_pwm;
        
        if (lsl_tai_val.word.high < gss_min_pwm)//debug
        {
        	  lsl_tai_val.word.high = gss_min_pwm;
        }
       	if (lsl_tai_val.word.high > gss_max_pwm)
       	{
       		  lsl_tai_val.word.high = gss_max_pwm;
       	}
    }
    
    return   lsl_tai_val.word.high;  
}
//------------------------------------------------------------------------------
uint16_t get_svpwm_comp_value(uint16_t now_index) //���β�����ʱ����PWM�������ֵ�ĳ���
{
    lword_type lsl_svpwm_val;
    
    while (now_index >= SIN_TAB_LENGTH_4)
    {
       now_index -= SIN_TAB_LENGTH_4;
    } 
    //----------------------------------
    if (now_index < SIN_TAB_LENGTH_1_3)
    {
        lsl_svpwm_val.lword = gus_percent_pwm;  
        lsl_svpwm_val.lword *= 14189;      //(3^0.5)ԼΪ 14189/2^13
        lsl_svpwm_val.lword >>= 13;
        lsl_svpwm_val.lword *= get_sin_value(now_index);
        lsl_svpwm_val.word.high += gus_half_pwm;
    }
    else if (now_index < SIN_TAB_LENGTH)
    {
        now_index += SIN_TAB_LENGTH_1_3;
        lsl_svpwm_val.lword = get_sin_value(now_index);
        lsl_svpwm_val.lword *= gus_percent_pwm;
        lsl_svpwm_val.word.high += gus_half_pwm;
    }
    else if (now_index < SIN_TAB_LENGTH_5_3)
    {  
        now_index -= SIN_TAB_LENGTH_1_3;
        lsl_svpwm_val.lword = get_sin_value(now_index);
        lsl_svpwm_val.lword *= gus_percent_pwm;
        lsl_svpwm_val.word.high += gus_half_pwm;
    }
    else if (now_index < SIN_TAB_LENGTH_7_3)
    { 
        lsl_svpwm_val.lword = gus_percent_pwm;  
        lsl_svpwm_val.lword *= 14189;      //(3^0.5)ԼΪ 14189/2^13
        lsl_svpwm_val.lword >>= 13;     
        lsl_svpwm_val.lword *= get_sin_value(now_index);
        lsl_svpwm_val.word.high += gus_half_pwm;
    }
    else if (now_index < SIN_TAB_LENGTH_3)
    { 
        now_index += SIN_TAB_LENGTH_1_3;
        lsl_svpwm_val.lword = get_sin_value(now_index);
        lsl_svpwm_val.lword *= gus_percent_pwm;
        lsl_svpwm_val.word.high += gus_half_pwm;      
    }
    else if (now_index < SIN_TAB_LENGTH_11_3)
    { 
        now_index -= SIN_TAB_LENGTH_1_3;
        lsl_svpwm_val.lword = get_sin_value(now_index);
        lsl_svpwm_val.lword *= gus_percent_pwm;
        lsl_svpwm_val.word.high += gus_half_pwm;
    }
    else if (now_index < SIN_TAB_LENGTH_4)
    { 
        lsl_svpwm_val.lword = gus_percent_pwm;  
        lsl_svpwm_val.lword *= 14189;      //(3^0.5)ԼΪ 14189/2^13
        lsl_svpwm_val.lword >>= 13;
        lsl_svpwm_val.lword *= get_sin_value(now_index);
        lsl_svpwm_val.word.high += gus_half_pwm;
    }   
    
    if (lsl_svpwm_val.word.high < gss_min_pwm)
    {
    	  lsl_svpwm_val.word.high = gss_min_pwm;
    }
    if (lsl_svpwm_val.word.high > gss_max_pwm)
    {
    	  lsl_svpwm_val.word.high = gss_max_pwm;//debug 
    }
    
    return lsl_svpwm_val.word.high;               
}
//------------------------------------------------------------------------------
void PWM_Reload_INT(void)     //���η����ж�
{
    if (bflg_running == 1)
    {
        //--------------------------------------------------
        //�����¸����ڵļ���ֵ
        if (bflg_current_direction == 0)//���Ҫ����ʱ����ת�������
        {
            //ռ�ձȻ���Ĵ�����ֵ
            //��U��Ϊ��׼
            TCMP0C = gus_val_a;    //PWM00ռ�ձ��趨  u��
            TCMP0B = gus_val_c;    //PWM01ռ�ձ��趨  w��
            TCMP0A = gus_val_b;    //PWM02ռ�ձ��趨  v��
        }
        else                            //���Ҫ��˳ʱ����ת�������
        {
            //ռ�ձȻ���Ĵ�����ֵ  
            //��U��Ϊ��׼
            TCMP0C = gus_val_a;    //PWM02ռ�ձ��趨  u��
            TCMP0B = gus_val_b;    //PWM01ռ�ձ��趨  v��
            TCMP0A = gus_val_c;    //PWM00ռ�ձ��趨  w��
        }
        //--------------------------------------------------
        //�жϱ������ܷ�ȡ����ADֵ
        if (bflg_taiwave_ctrl == 0)     //�������̨�Ͳ�
        {
        	  if (((gus_max_comp_val - gus_mid_comp_val) >= (gus_ADtrigger_repair_val + gus_AD_SH_val))
       	     || ((gus_mid_comp_val - gus_min_comp_val) >= (gus_ADtrigger_repair_val + gus_AD_SH_val)))
       	    {
       	    	  bflg_allow_get_Idc = 1;      //����ɼ�����AD
       	    	  
       	    	  if ((gus_max_comp_val - gus_mid_comp_val) > (gus_mid_comp_val - gus_min_comp_val))
       	    	  {
       	    	  	  bflg_up_down_get_Idc = 0;     //�½��زɼ�
       	    	  }
       	    	  else
       	    	  {
       	    	  	  bflg_up_down_get_Idc = 1;     //�����زɼ�
       	    	  }
       	    }
       	    else
       	    {
       	    	  bflg_allow_get_Idc = 0;      //������ɼ�����AD
       	    }
        }
        else
        {
        	  bflg_allow_get_Idc = 1;          //����ɼ�����AD
        	  if (gus_max_comp_val == gus_peak_pwm + 1)
        	  {
        	  	  bflg_up_down_get_Idc = 0;    //�½��زɼ�
        	  }
        	  else if (gus_min_comp_val == 0)
        	  {
        	  	  bflg_up_down_get_Idc = 1;    //�����زɼ�
        	  }
        	  else
        	  {
        	  	  bflg_allow_get_Idc = 0;      //������ɼ�����AD
        	  }
        }
        //--------------------------------------------------
        //���������������ֵ��������
        comp_val_order();
        //--------------------------------------------------
        //ȷ�������������������½�������AD0����ʱ��         
        //PWM��������ʱ��ȡmid��min֮��Ĳ�����
        //PWM�½�����ʱ��ȡmax��mid֮��Ĳ�����
        if (bflg_taiwave_ctrl == 0)     //�������̨�Ͳ�
        {
        	  gus_TUP_ADtrigger = (gus_min_comp_val + gus_mid_comp_val + gus_ADtrigger_repair_val - gus_AD_SH_val) >> 1;
            gus_TDW_ADtrigger = (gus_max_comp_val + gus_mid_comp_val + gus_AD_SH_val - gus_ADtrigger_repair_val) >> 1;
        }
        else        //�����̨�Ͳ�
        {
        	  gus_TUP_ADtrigger = gus_ADtrigger_repair_val;
            gus_TDW_ADtrigger = gus_peak_pwm - gus_TUP_ADtrigger;
        }
        
        TM20CB = gus_TUP_ADtrigger;
        TM20CA = ((gus_peak_pwm << 1) - gus_TDW_ADtrigger);
        //--------------------------------------------------
        //���µ����ʺ���λ����
        if (bflg_pwm_allow_updata == 1) //�������PWM���и��£��������µ����ʺ���λ����
        {
            bflg_pwm_allow_updata = 0;  //������PWM���±�־
            //----------------------------------------------
            //Ϊ�˱�֤��־λ�ͼ���ֵ��һ���� //ֻ�����������ص���Ӧ�ļ���ֵ��
            //��ʼ�������ֵʱ������̨�β�������־
            //----------------------------------------------
            if (bflg_askfor_taiwave_ctrl == 1)    //������������̨�β����ƣ������
            {
                bflg_taiwave_ctrl = 1;            //��̨�β����Ʊ�־
                bflg_askfor_taiwave_ctrl = 0;     //������̨�β����Ʊ�־
            }
            else if (bflg_askfor_guwave_ctrl == 1)//�����������й��β����ƣ������
            {
                bflg_taiwave_ctrl = 0;            //��̨�β����Ʊ�־
                bflg_askfor_guwave_ctrl = 0;      //��������β����Ʊ�־                      
            }
            //����������ֵд�������ļĴ�����
            gus_percent_pwm = gus_percent_pwm_buf;
            rate_theta.ulword = rate_theta_buf.ulword;
        }
        //--------------------------------------------------
        //��������������λ�ǡ�����ֵ
        calc_comp_val(); //��������������������ıȽϼ���ֵ,�ڲ��η����ж��е���
        //--------------------------------------------------
    }
    else
    {
        bflg_allow_get_Idc = 1;         //����ɼ�����AD
        
        gus_TUP_ADtrigger = gus_half_pwm;
        gus_TDW_ADtrigger = gus_peak_pwm;
        
        TM20CB = gus_TUP_ADtrigger;
        TM20CA = ((gus_peak_pwm << 1) - gus_TDW_ADtrigger);
    }
    //------------------------------------------------------
    get_Udc_ad();        //�õ�����Udc�����ADֵ
    //------------------------------------------------------
    prot_ALM_deal();      //ALM�����������
}
//------------------------------------------------------------------------------
void comp_val_order(void)     //������Ƚ�ֵ��������
{
    gus_now_theta = gus_next_theta;     //�õ���ǰ�������ڵ���λ��
    gus_next_theta = theta.uword.high;  //�õ��¸��������ڵ���λ��
    //------------------------------------------------------
    if (gus_val_a > gus_val_b)
    {
        if (gus_val_a > gus_val_c)
        {
            if (gus_val_b > gus_val_c)
            {
                gus_max_comp_val = gus_val_a;
                gus_mid_comp_val = gus_val_b;
                gus_min_comp_val = gus_val_c;
                
                gus_val_abc_order = ORDER_ABC;
            }
            else
            {
                gus_max_comp_val = gus_val_a;
                gus_mid_comp_val = gus_val_c;
                gus_min_comp_val = gus_val_b;   
                
                gus_val_abc_order = ORDER_ACB;                    
            }
        }
        else
        {
            gus_max_comp_val = gus_val_c;
            gus_mid_comp_val = gus_val_a;
            gus_min_comp_val = gus_val_b;
            
            gus_val_abc_order = ORDER_CAB;
        }
    }
    else
    {
        if (gus_val_b > gus_val_c)
        {
            if (gus_val_a > gus_val_c)
            {
                gus_max_comp_val = gus_val_b;
                gus_mid_comp_val = gus_val_a;
                gus_min_comp_val = gus_val_c;
                
                gus_val_abc_order = ORDER_BAC;
            }
            else
            {
                gus_max_comp_val = gus_val_b;
                gus_mid_comp_val = gus_val_c;
                gus_min_comp_val = gus_val_a;           
                
                gus_val_abc_order = ORDER_BCA;
            }
        }
        else
        {
            gus_max_comp_val = gus_val_c;
            gus_mid_comp_val = gus_val_b;
            gus_min_comp_val = gus_val_a;
            
            gus_val_abc_order = ORDER_CBA;
        }        
    }                  
}
//------------------------------------------------------------------------------
#endif            //end 
