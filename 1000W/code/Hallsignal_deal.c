#ifndef	_HALLSIGNAL_DEAL_C_
#define _HALLSIGNAL_DEAL_C_
//------------------------------------------------------------------------------
#include "srMN103SFJ9D.h"
#include "user_typedefine.h"
#include "comm_variable.h"

#include "EEPROM_PARA.h"
#include "SIN_TABLE.h"
#include "user_math.h"

#include "PWM_CTRL.h"
#include "protect_deal.h"
#include "Hallsignal_deal.h"
//------------------------------------------------------------------------------
void Hall_configure(void);         //�����źſ����ó���

void motor_var_init(void);         //�����ر������򻯣���Hall��ʼ����ͨѶID��������е���

void Hall_input_int(void);         //Hall�ź������жϷ������

void Hall_updata_deal(void);       //�����źŸ��´������,����ѭ���е���

void Hall_fault_delaytime(void);   //�����źŹ�����ʱ������10ms��ʱ�����е���

void Hall_error_delaytime(void);   //�����źŴ���ʱ���򣬼���10ms��ʱ������

void Hall_direction_delaytime(void);      //�����źŷ������ʱ���򣬼���10ms��ʱ������

void Hall_off_delaytime(void);     //�����ź�ֹͣ��ʱ������10ms��ʱ�����е���
//------------------------------------------------------------------------------
flag_type flg_Hall;
//------------------------------------------------------------------------------
uint16_t  PhaseValues_CCW[6]; //��ʱ����ת�����Ҳ�����ʱ�������Ӧ����ʼ��λ��
uint16_t  PhaseValues_CW[6];  //˳ʱ����ת�����Ҳ�����ʱ�������Ӧ����ʼ��λ��
//------------------------------------------------------------------------------
const int8_t gsc_sector_table[] = {-1, 0, 2, 1, 4, 5, 3, -1};    //�����ź������

lword_type Hall_trigger_time;      //�����źŴ���ʱ�̼Ĵ���

uint32_t  gul_Hall_trigger;        //�����źŴ�����
uint32_t  gul_Hall_past_trigger;   //�ϴλ����źŴ�����

uint32_t  gul_Hall_delt_trigger;
uint32_t  gul_Hall_past_delt_trigger;
uint32_t  gul_Hall_delt_trigger_buffer; //�����ź�ʱ������

uint16_t  gus_Hall_value;          //�����źŴ���ֵ
uint16_t  gus_Hall_past_value;     //�ϴλ����źŴ���ֵ
uint16_t  gss_Hall_value_buffer;   //�����źŴ���ֵ������

int8_t    gsc_Hall_sector;         //�����ź�����ֵ
int8_t    gsc_Hall_past_sector;    //�ϴλ����ź�����ֵ

uint16_t  gus_same_direction_cnt;  //ת����ͬ������
uint16_t  gus_dif_direction_cnt;   //ת��ͬ������

uint16_t  gus_Hall_watchdog;       //�����źſ��Ź�
//----------------------------------------------------------
uint16_t  gus_same_HU_cnt;
uint16_t  gus_same_HV_cnt;
uint16_t  gus_same_HW_cnt;
uint16_t  gus_Hall_same_cnt;

uint16_t  gus_stall_rate;

//f=1/T=1/6*detl_t=1/(6*delt_cnt/20M)=(20M/6)/delt_cnt  (Hz)=[(20M/6)/delt_cnt]*100 (0.01Hz)
#define   FREQ_FACTOR    ((20000000 / 6) * 100)    //���㵱ǰƵ�ʵļ������� (��ȷ��0.01Hz)

uint16_t  gus_Hall_freq;
uint16_t  gus_Hall_freq_mod;

uint16_t  gus_Hall_speed;

uint16_t  gus_sensor_startup_timer; //�������ٽ׶μ�ʱ��
//------------------------------------------------------------------------------
uint16_t  gus_avr_realfreq_cnt;
uint32_t  gul_avr_realfreq_sum;
uint16_t  gus_avr_realfreq;

uint16_t  gus_freq_array_len; //ʵ��Ƶ�����鳤��

uint16_t  gus_realfreq_array[50]; //���ڼ���Ƶ��ƽ��ֵ������
//------------------------------------------------------------------------------
uint16_t  gus_motor_overspeed;     //�������Ƶ��

int16_t   gss_Hall_fault_delaytimer;

uint16_t  gus_phase_adv_degree;
//------------------------------------------------------------------------------
uint8_t   guc_motor_ID;
//------------------------------------------------------------------------------
void Hall_configure(void)     //�����źſ�����
{
    //------------------------------------------------------
    //��8bit������timer4~7����Ϊ32bit�����������ڻ����źŲ��� 
    //------------------------------------------------------
    //G5ICR����5�жϿ��ƼĴ���
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  LV2 |  LV1 |  LV0 |  -   |  -   |  IE1 |  IE0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | IR1  | IR1  |  -   |  -   | ID1  | ID0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    G5ICR = 0x0000;      //��ֹtimer5,timer4�����ж�
    G6ICR = 0x0000;      //��ֹtimer7,timer6�����ж�
    
    TM7MD &= 0x7F;       //��ֹTM7BC����
    TM6MD &= 0x7F;       //��ֹTM6BC����
    TM5MD &= 0x7F;       //��ֹTM5BC����
    TM4MD &= 0x7F;       //��ֹTM4BC����  
    
    //TM47PSC��Ԥ��Ƶ���ƼĴ���1
    //--------------------------------------------------------------------------
    //reg_bit  |   7    |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |TMPSCNE1|  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |   0    |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //TMPSCNE1:Ԥ��Ƶ����ʹ��ѡ��  0=��ֹ  1=ʹ��
    //--------------------------------------------------------------------------
    TM47PSC = 0x00;
    
    //TM47EXPSC���ⲿԤ��Ƶ���ƼĴ���1
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0    |  
    // flag    |TM7IN |TM6IN |TM5IN |TM4IN |  -   |  -   |  -   |EXPSCNE1| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0    |
    //--------------------------------------------------------------------------
    //TM7IN:T7����ʱ��Դѡ��  0=TM7IO pin input  1=IOCLK/128
    //TM6IN:T6����ʱ��Դѡ��  0=TM6IO pin input  1=IOCLK/128
    //TM5IN:T5����ʱ��Դѡ��  0=TM5IO pin input  1=IOCLK/128
    //TM4IN:T4����ʱ��Դѡ��  0=TM4IO pin input  1=IOCLK/128
    //EXPSCNE1:Ԥ��Ƶ(1/128)����ʹ��ѡ��  0=��ֹ  1=ʹ��
    //--------------------------------------------------------------------------
    TM47EXPSC = 0x00;   
    
    //TM4BR: T4�����Ĵ��� (TM7BR,TM6BR,TM5BR��TM4BR����)
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM4BR7|TM4BR6|TM4BR5|TM4BR4|TM4BR3|TM4BR2|TM4BR1|TM4BR0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM4BR7~0: ������TM4BC����ʱ,���м����ĳ�ֵ��	TM4BC�����,��TM4BR���ص�TM4BC�С�
    //--------------------------------------------------------------------------
    TM4BR = 0xFF;
    TM5BR = 0xFF;
    TM6BR = 0xFF;
    TM7BR = 0xFF;
    
    //TM4MD: T4ģʽ�Ĵ���
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM4CNE|TM4LDE|  -   |  -   |  -   |TM4CK2|TM4CK1|TM4CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM4CNE: ��������תʹ��ѡ��  0=����ת  1=��ת
    //TM4LDE: ��������ʼ��   0=������ת   1=��ʼ��
    //TM4CK2~0: ����ʱ��Դѡ��  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=��ֹ����   
    //          100=��ֹ����   101=T5����    110=T6����     111=TM4IO pin����(������),IOCLK/128
    //--------------------------------------------------------------------------
    TM4MD = 0x00;
     
    //TM5MD: T5ģʽ�Ĵ���
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM5CNE|TM5LDE|  -   |  -   |  -   |TM5CK2|TM5CK1|TM5CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM5CNE: ��������תʹ��ѡ��  0=����ת  1=��ת
    //TM5LDE: ��������ʼ��   0=������ת   1=��ʼ��
    //TM5CK2~0: ����ʱ��Դѡ��  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=��T4����   
    //          100=T4����   101=��ֹ����    110=T6����     111=TM5IO pin����(������),IOCLK/128
    //--------------------------------------------------------------------------
    TM5MD = 0x03;
    
    //TM6MD: T6ģʽ�Ĵ���
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM6CNE|TM6LDE|  -   |  -   |  -   |TM6CK2|TM6CK1|TM6CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM6CNE: ��������תʹ��ѡ��  0=����ת  1=��ת
    //TM6LDE: ��������ʼ��   0=������ת   1=��ʼ��
    //TM6CK2~0: ����ʱ��Դѡ��  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=��T5����   
    //          100=T4����   101=T5����    110=��ֹ����     111=TM6IO pin����(������),IOCLK/128
    //--------------------------------------------------------------------------
    TM6MD = 0x03;
    
    //TM7MD: T7ģʽ�Ĵ���
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM7CNE|TM7LDE|  -   |  -   |  -   |TM7CK2|TM7CK1|TM7CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM7CNE: ��������תʹ��ѡ��  0=����ת  1=��ת
    //TM7LDE: ��������ʼ��   0=������ת   1=��ʼ��
    //TM7CK2~0: ����ʱ��Դѡ��  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=��T6����   
    //             100=T4����   101=T5����    110=T6����     111=TM7IO pin����(������),IOCLK/128
    //--------------------------------------------------------------------------
    TM7MD = 0x03;   
    //----------------------------------
    TM4MD |= 0x40;  //��ʼ��TM0BC
    TM5MD |= 0x40;  //��ʼ��TM1BC
    TM6MD |= 0x40;  //��ʼ��TM2BC
    TM7MD |= 0x40;  //��ʼ��TM3BC
    
    TM4MD &= 0xBF;  //���ʼ��TM0BC��־
    TM5MD &= 0xBF;  //���ʼ��TM1BC��־
    TM6MD &= 0xBF;  //���ʼ��TM2BC��־
    TM7MD &= 0xBF;  //���ʼ��TM3BC��־   
    
    TM7MD |= 0x80;    //����TM3BC����
    TM6MD |= 0x80;    //����TM3BC����
    TM5MD |= 0x80;    //����TM3BC����
    TM4MD |= 0x80;    //����TM3BC����   
    //----------------------------------
    //��ֹ�����ź��ж�
    G29ICR = 0x0000;
    G30ICR = 0x0000;
    //--------------------------------------------------------------------------
    //NFCLK2: ����ʱ�����üĴ���2
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | -    |  -  |  P41NFCK1~0 |  P40NFCK1~0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    NFCLK2 = 0x000F;     //P40,P41 ѡ�� 1/32 IOCLK
    
    //--------------------------------------------------------------------------
    //NFCLK3: ����ʱ�����üĴ���3
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  P61NFCK1~0 |  P60NFCK1~0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | -    |   -  |  P51NFCK1~0 |  P50NFCK1~0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    NFCLK3 = 0x0003;     //P50 ѡ�� 1/32 IOCLK
    
    //--------------------------------------------------------------------------
    //NFCNT1: �˲����ƼĴ���1
    //----------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |    13   |    12   |  11  |  10  |   9    |   8    | 
    // flag    |  -   |  -   | P61NFCNT|P60NFCNT |  -   |  -   |P51NFCNT|P50NFCNT| 
    //at reset |  0   |  0   |    0    |    0    |  0   |  0   |   0    |   0    |
    //-------------------------------------------------------------------
    //reg_bit  |  7   |  6   |    5    |    4    |  3   |  2   |   1    |   0    |  
    // flag    |  -   |  -   |   -     |    -    |  -   |   -  |P41NFCNT|P40NFCNT| 
    //at reset |  0   |  0   |    0    |    0    |  0   |  0   |   0    |   0    |
    //----------------------------------------------------------------------------  
    NFCNT1 = 0x0103;     //P40 P41 P50 noise filter enable
    
    //--------------------------------------------------------------------------
    IRQEDGESEL |= 0x0700;     //IRQ10,IRQ9,IRQ8�����ش���ʹ��  
    //--------------------------------------------------------------------------
    //EXTMD1:�ⲿ�ж������趨�Ĵ���
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  IR11TG1~0  |  IR10TG1~0  |  IR09TG1~0  |  IR08TG1~0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //IRxTG1~0 : IRQx���Ŵ��������趨, 00=�����ش��� 01=�½��ش��� 10=H��ƽ���� 11=L��ƽ����
    //--------------------------------------------------------------------------
    EXTMD1 = 0x0000;
    //--------------------------------------------------------------------------
    //Configure P40,P41  (P40 is HW,P41 is HV)
    P4MD1 |= 0x03;  //P40,P41 is TMnIO pin/IRQ pin (0=IO 1=Special function) 
    P4MD2 &= ~0x03; //selected by P4MD1 register
    
    //Configure P50,P51  (P50 is HU,P51 is NC)
    P5MD1 |= 0x01;  //P50 is TMnIO pin/IRQ pin,P51 is IO (0=IO 1=Special function) 
    P5MD2 &= ~0x01; //selected by P5MD1 register
    //--------------------------------------------------------------------------
    //���ж������־
    *(unsigned char *)&G29ICR = 0x03;  //������ж�IRQ09��IRQ08�����־
    *(unsigned char *)&G30ICR = 0x01;  //������ж�IRQ10�����־   
    //--------------------------------------------------------------------------
    //G29ICR����29�жϿ��ƼĴ���(�������ж�IRQ09��IRQ08)
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
    //IE2~IE0 : �ж�ʹ�ܱ�־     IE1=IRQ09   IE0=IRQ08
    //IR2~IR0 : �ж������־
    //ID2~ID0 : �жϼ���־     
    //--------------------------------------------------------------------------
    G29ICR = 0x1000;     //�����źŵ��ж����ȼ���Ϊ1 (������ALM)
    //--------------------------------------------------
    //G30ICR����30�жϿ��ƼĴ���(�������ж�IRQ11��IRQ10)
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
    //IE1~IE0 : �ж�ʹ�ܱ�־  IE1=IRQ11   IE0=IRQ10
    //IR1~IR0 : �ж������־
    //ID1~ID0 : �жϼ���־     
    //--------------------------------------------------------------------------
    G30ICR = 0x1000;     //�����źŵ��ж����ȼ���Ϊ1 (������ALM)
	  //--------------------------------------------------------------------------
	  guc_motor_ID = 0;    //��ʼ�����ID��Ϊ0
	  motor_var_init();    //�����ر�������
}
//------------------------------------------------------------------------------
void motor_var_init(void)     //�����ر������򻯣���Hall��ʼ����ͨѶID��������е���
{
	  uint32_t lul_tmp;
    
    //--------------------------------------------------------------------------
    //ȷ�����Ҳ�������ʱ����תÿ�������ź��������ʼ��λ��
    PhaseValues_CCW[0] = ram_para[num_MOTOR_CCW];//motor_para[guc_motor_ID][MOTOR_CCW];
    PhaseValues_CCW[1] = ((PhaseValues_CCW[0] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CCW[2] = ((PhaseValues_CCW[1] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CCW[3] = ((PhaseValues_CCW[2] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CCW[4] = ((PhaseValues_CCW[3] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CCW[5] = ((PhaseValues_CCW[4] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    //--------------------------------------------------------------------------
    //ȷ�����Ҳ�����˳ʱ����תÿ�������ź��������ʼ��λ��
    PhaseValues_CW[0]= ram_para[num_MOTOR_CW];//motor_para[guc_motor_ID][MOTOR_CW];
    PhaseValues_CW[5]= ((PhaseValues_CW[0] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CW[4]= ((PhaseValues_CW[5] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CW[3]= ((PhaseValues_CW[4] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CW[2]= ((PhaseValues_CW[3] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CW[1]= ((PhaseValues_CW[2] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);	 
	  //--------------------------------------------------------------------------
	  //Ƶ��ƽ��������=���������*������Hall��*Ȧ��
	  gus_freq_array_len = ram_para[num_MOTOR_p] * 6 * 1;//motor_para[guc_motor_ID][MOTOR_p]
	  for (gus_avr_realfreq_cnt = 0; gus_avr_realfreq_cnt < 50; gus_avr_realfreq_cnt++)
	  {
	  	   gus_realfreq_array[gus_avr_realfreq_cnt] = 0;
	  }
	  gus_avr_realfreq_cnt = 0;
	  //--------------------------------------------------------------------------
	  //Hall���ϼ���ź���ͬ����=���������*������Hall��*Ȧ��
	  gus_Hall_same_cnt = ram_para[num_MOTOR_p] * 6 * 20;//10;//2;//debug��תʱ��̫��
	  //--------------------------------------------------------------------------
	  //�õ��������Ƶ��
	  lul_tmp = ram_para[num_MOTOR_max_n];//motor_para[guc_motor_ID][MOTOR_max_n];
	  lul_tmp *= ram_para[num_motor_overspeed];
	  lul_tmp /= 1000;
	  gus_motor_overspeed = (uint16_t)lul_tmp;
	  //--------------------------------------------------------------------------
}
//------------------------------------------------------------------------------
void Hall_input_int(void)     //Hall�ź������жϷ������
{
      //------------------------------------------------------
      //����ǰ�жϴ�����ʱ��
    TM4MD &= 0x7F;  //ֹͣTM4BC~TM7BC 32bit����
    Hall_trigger_time.uword.low = TM45BC;
    Hall_trigger_time.uword.high = TM67BC;
    TM4MD |= 0x80;  //����TM4BC~TM7BC 32bit����
    //------------------------------------------------------
    if (bflg_hall_int == 0)   //������״ν�������ж�
    {
    	  bflg_hall_int = 1;    //���״ν�������жϱ�־
    	  
    	  gul_Hall_trigger = Hall_trigger_time.ulword;   //�õ�����ʱ��
    	  //gul_Hall_past_trigger = gul_Hall_trigger;
    	  
    	  gus_Hall_value = HALL_PIN;                     //�õ������ź�ֵ
    	  gus_Hall_past_value = gus_Hall_value;
    	  
    	  gsc_Hall_sector = gsc_sector_table[gus_Hall_value]; //�õ���������ֵ
    	  gsc_Hall_past_sector = gsc_Hall_sector;
    	  
    	  //theta.word.high = gul_theta.word.high + 600;   //��ǰ��60����λ
        //theta.word.low  = 0;
        
        //gul_theta.word.high = theta.word.high;
    	  if (bflg_current_direction == 0)    //���Ҫ��ת��Ϊ��ʱ��
    	  {
    	      gul_theta.uword.high = PhaseValues_CCW[gsc_Hall_sector];
            gul_theta.uword.low  = 0;
                                
            //gul_theta.word.high = theta.word.high;
    	  }
        else  //���ʵ��ת��Ϊ˳ʱ��
    	  {
    	  	  gul_theta.uword.high = PhaseValues_CW[gsc_Hall_sector];
            gul_theta.uword.low  = 0;
                                
            //gul_theta.word.high = theta.word.high;
    	  }    
    	      
    	  if (bflg_theta_ctrl1 == 0)///
        {
            gul_theta.uword.high += 300;
            theta.ulword = gul_theta.ulword;
            bflg_theta_ctrl2 = 0;
        }
        else
        {
            theta.ulword = gul_theta.ulword;
            bflg_theta_ctrl2 = 1;
        }        
        //??gul_theta.word.high = theta.word.high;
    }
    else
    {
    	  //�������Ϊ�ݼ����������Դ˴�ʹ���ϴμ���ֵ�����μ���ֵ
    	  gul_Hall_delt_trigger_buffer = gul_Hall_trigger - Hall_trigger_time.ulword;
    	  //if (gul_Hall_delt_trigger_buffer > 40000)  //������ֵС��40000ʱ����Ч����ʵ��Ƶ�ʴ���83.33Hz��
    	  //if (gul_Hall_delt_trigger_buffer > 35000)  //������ֵС��35000ʱ����Ч����ʵ��Ƶ�ʴ���95.24Hz��1429rpm���� 
    	  //if (gul_Hall_delt_trigger_buffer > 27500)  //������ֵС��27500ʱ����Ч����ʵ��Ƶ�ʴ���120Hz��1818rpm����     	  
    	  if (gul_Hall_delt_trigger_buffer > 27500)  //������ֵС��27500ʱ����Ч����ʵ��Ƶ�ʴ���120Hz��1818rpm��1000W-4�Լ����ת��1800rpm=60*120/������4����     	      	  
    	  {
    	  	  //gul_Hall_past_trigger = gul_Hall_trigger;
    	  	  gul_Hall_trigger = Hall_trigger_time.ulword;
    	  	  //----------------------------------------------
    	  	if (gss_Hall_value_buffer != HALL_PIN)
    	  	{
    	  	  gss_Hall_value_buffer = HALL_PIN;
    	  	  //----------------------------------------------
    	  	  if ((gss_Hall_value_buffer != 0) && (gss_Hall_value_buffer != 7))
    	  	  {
    	  	  	  if ((bflg_hall_update == 0) && (bflg_no_direction == 0))   //����޻����źŸ��±�־������ת�����־
    	  	  	  {
    	  	  	  	  gul_Hall_past_delt_trigger = gul_Hall_delt_trigger;    //������һ�ε�ʱ����
    	  	  	  	  gul_Hall_delt_trigger = gul_Hall_delt_trigger_buffer;  //��¼���ε�ʱ����
    	  	  	  	  
    	  	  	  	  bflg_hall_update = 1;
    	  	  	  }
    	  	  	  //--------------------------------------------------------------
    	  	  	  gus_Hall_past_value = gus_Hall_value;       //�����ϴλ����ź�ֵ
    	  	  	  gus_Hall_value = gss_Hall_value_buffer;     //��¼���λ����ź�ֵ
    	  	  	  
    	  	  	  gsc_Hall_past_sector = gsc_Hall_sector;     //�����ϴ�����ֵ
    	  	  	  gsc_Hall_sector = gsc_sector_table[gus_Hall_value];   //��¼��������ֵ
    	  	  	  //--------------------------------------------------------------
    	  	  	  bflg_past_actual_direction = bflg_actual_direction;   //�����ϴ�ʵ��ת���־
    	  	  	  //--------------------------------------------------------------
    	  	  	  //�ж�ʵ����ת����
    	  	  	  if (gsc_Hall_sector == ((gsc_Hall_past_sector + 1) % 6))
    	  	  	  {
    	  	  	  	  bflg_actual_direction = 0;    //ʵ��ת��Ϊ˳ʱ��
    	  	  	  	  bflg_no_direction = 0;        //���޷��ж���ת�����־

    	  	  	  }
    	  	  	  else if (gsc_Hall_sector == ((gsc_Hall_past_sector + 5) % 6))
    	  	  	  {
    	  	  	  	  bflg_actual_direction = 1;    //ʵ��ת��Ϊ��ʱ��
    	  	  	  	  bflg_no_direction = 0;        //���޷��ж���ת�����־
    	  	  	  }
    	  	  	  else
    	  	  	  {
    	  	  	  	  bflg_no_direction = 1;        //���޷��ж���ת�����־
    	  	  	  }
    	  	  	  //--------------------------------------------------------------
    	  	  	  if (bflg_running == 1)
    	  	  	  {
    	  	  	      //if (bflg_current_direction == bflg_actual_direction)   //���ʵ��ת����Ҫ��ת��һ��
    	  	  	      //{
    	  	  	  	      if (bflg_no_direction == 0)    //��������ҿ����ж�ת��
    	  	  	  	      {
										    	  if (bflg_current_direction == 0)    //���Ҫ��ת��Ϊ��ʱ��
										    	  {
										    	      gul_theta.uword.high = PhaseValues_CCW[gsc_Hall_sector];
										            gul_theta.uword.low  = 0;
										                                
										            //gul_theta.word.high = theta.word.high;
										    	  }
										        else  //���ʵ��ת��Ϊ˳ʱ��
										    	  {
										    	  	  gul_theta.uword.high = PhaseValues_CW[gsc_Hall_sector];
										            gul_theta.uword.low  = 0;
										                                
										            //gul_theta.word.high = theta.word.high;
										    	  }    
										    	      
											    	if (bflg_theta_ctrl1 == 0)///
											      {
											          gul_theta.uword.high += 300;
											          theta.ulword = gul_theta.ulword;
											          bflg_theta_ctrl2 = 0;
											      }
											      else
											      {
											          theta.ulword = gul_theta.ulword;
											          bflg_theta_ctrl2 = 1;
											      }        
       
										        //??gul_theta.word.high = theta.word.high;
    	  	  	  	      }
    	  	  	  	      //------------------------------------------------------
    	  	  	      //}
    	  	  	      //else
    	  	  	      if ((bflg_current_direction != bflg_actual_direction)   //���ʵ��ת����Ҫ��ת��һ��
    	  	  	         || (bflg_no_direction == 1))
    	  	  	      {
    	  	  	  	      //theta.word.high = gul_theta.word.high + 600;
    	  	  	  	      //theta.word.low  = 0;
    	  	  	  	      //gul_theta.word.high = theta.word.high;
    	  	  	  	      
    	  	  	  	      gus_same_direction_cnt = 0;
    	  	  	  	      if (bflg_prot_Hall_direction == 0)
    	  	  	  	      {
    	  	  	  	  	      gus_dif_direction_cnt++;
    	  	  	  	  	      /*if (gus_dif_direction_cnt >= 120)
    	  	  	  	  	      {
    	  	  	  	  	      	  gus_dif_direction_cnt = 0;
    	  	  	  	  	      	  bflg_prot_Hall_direction = 1;
    	  	  	  	  	      	  trip_stop_deal(Hall_direct_FAULT_CODE);
    	  	  	  	  	      }*/
    	  	  	  	      }
    	  	  	      } 
    	  	  	      else 
    	  	  	      {
    	  	  	  	      if (gus_dif_direction_cnt != 0)
    	  	  	  	      {
    	  	  	  	  	      gus_same_direction_cnt++;
    	  	  	  	  	      if (gus_same_direction_cnt >= 12)
    	  	  	  	  	      {
    	  	  	  	  	  	      gus_same_direction_cnt = 0;
    	  	  	  	  	  	      gus_dif_direction_cnt = 0;
    	  	  	  	  	      }
    	  	  	  	      }    	  	  	      	
    	  	  	      }
    	  	  	  }
    	  	  }
    	  	}
    	  }
    }
    //------------------------------------------------------
    gus_Hall_watchdog = 0;    //������źſ��Ź� 
    //------------------------------------------------------
}
//------------------------------------------------------------------------------
void Hall_updata_deal(void)        //�����źŸ��´������
{
	  uint16_t lus_index;
	  lword_type ltmp1,ltmp2;
	  
	  if (bflg_hall_update == 1)     //���������»����ź�λ�ã������
	  {
        //----------------------------------------------------------------------
        //���ڻ����ź�W���ϵ��ж�
        if ((gus_Hall_past_value & 0x0001) == (gus_Hall_value & 0x0001))
        {
        	  gus_same_HW_cnt++;     //�����ź�W��ͬ����������
        }
        else
        {
        	  gus_same_HW_cnt = 0;   //������ź�W��ͬ������
        }
        //--------------------------------------------------
        //���ڻ����ź�V���ϵ��ж�
        if ((gus_Hall_past_value & 0x0002) == (gus_Hall_value & 0x0002))
        {
        	  gus_same_HV_cnt++;     //�����ź�V��ͬ����������
        }
        else
        {
        	  gus_same_HV_cnt = 0;   //������ź�V��ͬ������
        }
        //--------------------------------------------------
        //���ڻ����ź�U���ϵ��ж�
        if ((gus_Hall_past_value & 0x0004) == (gus_Hall_value & 0x0004))
        {
        	  gus_same_HU_cnt++;     //�����ź�U��ͬ����������
        }
        else
        {
        	  gus_same_HU_cnt = 0;   //������ź�U��ͬ������
        }
        //----------------------------------------------------------------------
        if (bflg_current_direction == bflg_actual_direction)     //ʵ��ת����Ҫ��ת��һ��
        {
            if (bflg_running == 1)
            {
                //------------------------------------------
                //������ʧ����
                ltmp1.ulword = gul_Hall_past_delt_trigger;
                ltmp1.ulword *= 100;
                ltmp1.ulword /= gul_Hall_delt_trigger;
                ltmp2.lword = 100 - ltmp1.ulword;
                if (ltmp2.lword < 0)
                {
                	  ltmp2.lword = -ltmp2.lword;
                }
                gus_stall_rate = ltmp2.lword;
            }	          	  
        }
        //----------------------------------------------------------------------
        //����ʵ��ת��
        ltmp1.ulword = FREQ_FACTOR;
        ltmp1.ulword /= gul_Hall_delt_trigger;     //�õ�����ʵ��Ƶ��
        
        bflg_hall_update = 0;       //��������»����ź�λ�ñ�־
        //--------------------------------------------------
        gus_realfreq_array[gus_avr_realfreq_cnt] = ltmp1.uword.low;
        gus_avr_realfreq_cnt++;
        if (gus_avr_realfreq_cnt >= gus_freq_array_len)
        {
        	  gus_avr_realfreq_cnt = 0;
        } 
        //--------------------------------------------------
        for (lus_index = 0; lus_index < gus_freq_array_len; lus_index++)
        {
         	  gul_avr_realfreq_sum += gus_realfreq_array[lus_index];
        }
        //--------------------------------------------------
        ltmp2.ulword = gul_avr_realfreq_sum;
        ltmp2.ulword /= gus_freq_array_len;
        gul_avr_realfreq_sum = 0;
        gus_avr_realfreq = ltmp2.uword.low;
        //--------------------------------------------------
        //��Hallfreq����1/4��ľ�˲�
        ltmp2.ulword = lowpass_filter(gus_Hall_freq, gus_Hall_freq_mod, ltmp1.uword.low, 2);
        gus_Hall_freq = ltmp2.uword.high;
        gus_Hall_freq_mod = ltmp2.uword.low;
        //--------------------------------------------------
        if (bflg_running == 1)///
        {
        	  if ((gus_Hall_freq > 500)&&(bflg_actual_direction == bflg_current_direction))
        	  {
        	  	  if (bflg_theta_ctrl1 == 0)
        	  	  {
        	  	      bflg_theta_ctrl1 = 1;
        	      } 
        	  }        	  	
        }
        else
        {
        	  bflg_theta_ctrl1 = 0;
        	  bflg_theta_ctrl2 = 0;
        }
        //--------------------------------------------------
        gus_freq_real = gus_avr_realfreq;
        //--------------------------------------------------
        //r = (f / 100) * 60 / P
        ltmp1.ulword = gus_Hall_freq;
        ltmp1.ulword *= 6;
        ltmp1.ulword /= 10;
        ltmp1.ulword /= ram_para[num_MOTOR_p];//motor_para[guc_motor_ID][MOTOR_p];
        gus_Hall_speed = ltmp1.ulword;//���ڵ��������ʾ
        //----------------------------------------------
        //r = (f / 100) * 60 / P
        ltmp1.ulword = gus_avr_realfreq;//gus_Hall_freq;
        ltmp1.ulword *= 6;
        ltmp1.ulword /= 10;
        ltmp1.ulword /= ram_para[num_MOTOR_p];//motor_para[guc_motor_ID][MOTOR_p];
        gus_speed_real = ltmp1.ulword;//������λ����ʾ        
        
        //gus_speed_real = gus_Hall_speed;
        //----------------------------------------------
        //������λ��ǰ��
        if (gus_Hall_freq > ram_para[num_PhaseAdvStartFreq])
        {
        	  ltmp1.ulword = gus_Hall_freq;
        	  ltmp1.ulword -= ram_para[num_PhaseAdvStartFreq];
        	  ltmp1.ulword *= ram_para[num_PhaseAdvSlope];
        	  ltmp1.ulword /= 5000;
        	  gus_phase_adv_degree = ltmp1.ulword;
        }
        else
        {
        	  gus_phase_adv_degree = 0;
        }
        
        if (gus_phase_adv_degree >= ram_para[num_phase_max_degree])
        {
        	  gus_phase_adv_degree = ram_para[num_phase_max_degree];
        }
        //--------------------------------------------------
        /*if (bflg_sensor_startup == 1)
        {
            gus_sensor_startup_timer = 0;     //�忪�����ٽ׶μ�ʱ��
            //-----------------------------------------------
            if (gus_freq_real <= ram_para[num_min_freq])
            {
                //�ڿ������ٽ׶Σ��Ѿ��������ת�٣����忪�����ٽ׶α�־
                bflg_sensor_startup = 0;     //�ÿ������ٽ׶α�־
                sensored_start_init();       //��λ�ô���������ʱ�Ŀ�����ʼ������
            }                        	   
        }*/
        //--------------------------------------------------
	  }
}
//------------------------------------------------------------------------------
void Hall_fault_delaytime(void)    //�����źŹ�����ʱ������10ms��ʱ�����е���
{
	  if ((HALL_PIN == 0) || (HALL_PIN == 7))
	  {
	  	  if (bflg_Hall_fault == 0)
	  	  {
	  	  	  gss_Hall_fault_delaytimer++;
	  	  	  if (gss_Hall_fault_delaytimer >= 100)   //1s
	  	  	  {
	  	  	  	  gss_Hall_fault_delaytimer = 0;
	  	  	  	  bflg_Hall_fault = 1;
	  	  	  	  trip_stop_deal(Hall_FAULT_CODE);
	  	  	  }
	  	  }
	  }
	  else
	  {
        gss_Hall_fault_delaytimer = 0;

        if (bflg_running == 0) //����ͣ����־�������������ϱ�־
        {
            bflg_Hall_fault = 0;   
        }	  	  
     }
}
//------------------------------------------------------------------------------
void Hall_error_delaytime(void)    //�����źŴ���ʱ���򣬼���10ms��ʱ������
{
	  if (bflg_running == 1)
	  {
	  	  if ((gus_same_HW_cnt >= gus_Hall_same_cnt) || (gus_same_HV_cnt >= gus_Hall_same_cnt) || (gus_same_HU_cnt >= gus_Hall_same_cnt))
	  	  {
	  	  	  gus_same_HW_cnt = 0;
	  	  	  gus_same_HV_cnt = 0;
	  	  	  gus_same_HU_cnt = 0;
	  	  	  
	  	  	  if (bflg_Hall_error == 0)
	  	  	  {
	  	  	  	  bflg_Hall_error = 1;
	  	  	  	  trip_stop_deal(Hall_FAULT_CODE);
	  	  	  }
	  	  }
	  }
	  else
	  {
	  	  gus_same_HW_cnt = 0;
	  	  gus_same_HV_cnt = 0;
	  	  gus_same_HU_cnt = 0;
	  	  
    	  if (bflg_running == 0) //����ͣ����־�������������ϱ�־
    	  {
    	       bflg_Hall_error = 0;
        }	  	    
	  }
}
//------------------------------------------------------------------------------
void Hall_direction_delaytime(void)     //�����źŷ������ʱ���򣬼���10ms��ʱ������
{
	  if (bflg_running == 1)
	  {
	  	  if (gus_dif_direction_cnt >= gus_Hall_same_cnt)
    	  {
    	  	  gus_dif_direction_cnt = 0;
    	  	  
    	  	  if (bflg_prot_Hall_direction == 0)
    	  	  {
    	  	      bflg_prot_Hall_direction = 1;
    	  	      trip_stop_deal(Hall_direct_FAULT_CODE);
    	  	  }    
    	  }
	  }
	  else
	  {
	  	  gus_dif_direction_cnt = 0;
	  	  
    	  if (bflg_running == 0) //����ͣ����־�������������ϱ�־
    	  {
    	      bflg_prot_Hall_direction = 0;    
        }	  	    
	  }
}
//------------------------------------------------------------------------------
void Hall_off_delaytime(void)      //�����ź�ֹͣ��ʱ������10ms��ʱ�����е���
{
    gus_Hall_watchdog++;
    if (gus_Hall_watchdog >= 50)//debug
    {
    	  gus_Hall_watchdog = 0;
    	  
    	  gus_same_HW_cnt = 0;
    	  gus_same_HV_cnt = 0;
    	  gus_same_HU_cnt = 0;
    	  //--------------------------------------------------
    	  for (gus_avr_realfreq_cnt = 0; gus_avr_realfreq_cnt < gus_freq_array_len; gus_avr_realfreq_cnt++)
	      {
	  	      gus_realfreq_array[gus_avr_realfreq_cnt] = 0;
	      }
	      gus_avr_realfreq_cnt = 0;
	      //--------------------------------------------------
	      gus_avr_realfreq = 0;
	      gus_Hall_freq = 0;
	      gus_Hall_freq_mod = 0;
	      gus_freq_real = 0;
	      
	      gus_Hall_speed = 0;
	      gus_speed_real = 0;
	      
	      bflg_hall_int = 0;      //������ź��жϱ�־
	      //--------------------------------------------------
	      //ȷ��Hall����
	      gus_Hall_value = HALL_PIN; //�õ������ź�ֵ
	      gsc_Hall_sector = gsc_sector_table[gus_Hall_value];
	      //--------------------------------------------------
	      //gus_freq_out = 0;
        
        //gus_Uout_real = 0;
        //gsl_FreqPI_integral = 0;
        
    	  if (bflg_current_direction == 0)    //���Ҫ��ת��Ϊ��ʱ��
    	  {
    	      gul_theta.uword.high = PhaseValues_CCW[gsc_Hall_sector];
            gul_theta.uword.low  = 0;
                                
            //gul_theta.word.high = theta.word.high;
    	  }
        else  //���ʵ��ת��Ϊ˳ʱ��
    	  {
    	  	  gul_theta.uword.high = PhaseValues_CW[gsc_Hall_sector];
            gul_theta.uword.low  = 0;
                                
            //gul_theta.word.high = theta.word.high;
    	  }    
    	  //----------------------------
    	  if (bflg_theta_ctrl1 == 0)///debug
        {
            gul_theta.uword.high += 300;
            theta.ulword = gul_theta.ulword;
            bflg_theta_ctrl2 = 0;
        }
        else
        {
            theta.ulword = gul_theta.ulword;
            bflg_theta_ctrl2 = 1;
        }      	      
        //theta.ulword = gul_theta.ulword;
	  	  //--------------------------------------
	  	  //�õ����ֳ�ֵ
	  	  //gsl_FreqPI_integral = gsl_PI_initval;
        
        if (bflg_actual_direction != bflg_current_direction)
        {
        	  gus_freq_out = 0;
        }
        
        bflg_actual_direction = bflg_current_direction;
    }
}
//------------------------------------------------------------------------------
#endif
