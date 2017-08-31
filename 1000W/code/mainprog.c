//------------------------------------------------------------------------------
//�����ļ�
#include "srMN103SFJ9D.h"
#include "user_typedefine.h"
#include "comm_variable.h"

#include "EEPROM_PARA.h"
#include "SIN_TABLE.h"

#include "EEPROM_RW.h"
#include "SWI2C_EEPROM_Drive.h"

#include "PWM_CTRL.h"
#include "AD_Convert.h"
#include "Hallsignal_deal.h"
#include "protect_deal.h"
#include "Host_comm.h"
#include "panel_comm.h"
//------------------------------------------------------------------------------
//��������
void main(void);              //���������

void ports_configure(void);   //�˿����ó���

void read_para_RomtoRam(void);//��rom�ж�ȡ������ram

void control_init(void);      //���ϵ�׶Σ����ڳ�ʼ��һЩ�������ĳ���

void TM23_configure(void);    //TMR23ģ�����ó���

void TM23_1MS_INT(void);      //1ms��ʱ�жϷ������

void main_loop_deal(void);    //��ѭ���������

void timer_op(void);          //��ʱ�����ӳ������ڴ������еĶ�ʱ��ʱ����  

void fan_ctrl_deal(void);     //ѹ�������Ƴ���

void freq_ctrl_deal(void);    //Ƶ�ʿ��Ƴ���

uint16_t get_min_decfreq_deal(void);    //��������Ƶ�����򲢵õ���С��Ϊ0�ı���Ƶ�ʵĳ�����Ƶ���趨�����е���

void check_powerup_delaytime(void);     //�ϵ�����ʱ������1ms��ʱ�����е���

void check_run_stop_deal(void);         //��鿪�ػ�����ĵĳ���

void run_key(void);                     //���м�����

void stop_key(void);

//void check_auto_start(void);            //���ۺ���ԭ��ֻҪҪ�����������ô˳���

void check_auto_stop(void);             //����ͣ��ʱ�����õĳ���

//void check_free_stop(void); 

//void set_freq_op(void); 

void check_spry_ctrl_term(void);        //���spry���������ĳ�����100ms��ʱ�����е���

void check_spry_delaytime(void);        //���̵������ϺͶϿ�����ʱ��ʱ������1ms��ʱ�����е���

void stop_space_delaytime(void);        //ͣ�������ʱ������1s��ʱ�����е���

void trip_lampblink_deal(void);         //���ϵ���˸����,��10ms��ʱ�����е���

//------------------------------------------------------------------------------
flag_type flg_ctrl;
flag_type flg_delaytime;
flag_type flg_theta;
flag_type flg_SCI1;

int16_t   gss_timer_10ms;     //10ms�����Ĵ���
int16_t   gss_timer_100ms;    //100ms�����Ĵ���
int16_t   gss_timer_1s;       //1s�����Ĵ���
int16_t   gss_timer_1min;     //1min�����Ĵ���  


int16_t   gss_powerup_delaytimer;       //�ϵ���ʱ��ʱ��
int16_t   gss_spry_delaytimer;//spry��ʱ��ʱ��
//----------------------------------------------------------
uint16_t  main_loop_cnt;
//----------------------------------------------------------
uint16_t  gus_trip_flash_count;    //���ϵ���˸�����Ĵ���
uint16_t  gus_trip_flash_timer;    //���ϵ���˸��ʱ�Ĵ���
//------------------------------------------------------------------------------  
//------------------------------------------------------------------------------
void main(void)
{
    //------------------------------------------------------
    disable_irq();            //�����ж�
    set_imask(0);             //��ֹ��Ӧ�κ����ȼ����ж�
    //watchdog_disable();     //�ؿ��Ź�
    watchdog_enable();        //�������Ź�
    //----------------------------------
    ports_configure();        //�˿����ó���
    //----------------------------------
    delay_5us(20000);         //��ʱ100ms
    
    if (read_para_eeptoram() != 0)        //��eeprom�ж�ȡ������ram-peak����
    {
        bflg_trip_stop = 1;               //�ù���ͣ����־ 
        bflg_eeprom_change = 1;           //��EEPROM�����־
        bflg_eeprom_error = 1;            //��EEPROM�����־
        guc_trip_code = EEPROM_ERR_CODE;  //��EEPROM��
        guc_lamp_trip_code = LAMP_EEPROM_ERR_CODE;

        read_para_RomtoRam();             //��rom�ж�ȡ������ram
    }
    
    //read_para_RomtoRam();     //��rom�ж�ȡ������ram
    //ID������ȷ��
    if ((ram_para[num_App_ID] < APP_ID_MIN) || (ram_para[num_App_ID] > APP_ID_MAX))
    {
    	  ram_para[num_App_ID] = APP_ID_MIN;
    }	 
    //----------------------------------
    TM23_configure();         //TM16���ó���,����200us��ʱ
    //----------------------------------
    Hall_configure();         //�����ź����ó���
    //----------------------------------
    ADC_configure();          //ADת�����ó���
    //----------------------------------
    PWM_configure();          //PWMģ�����ó���
    //----------------------------------
    SCI0_configure();         //����SCI0����ͨѶ�ӿ�
   
    SCI1_configure();         //����SCI1����ͨѶ�ӿ�
    //----------------------------------
    control_init();           //���ϵ�׶Σ����ڳ�ʼ��һЩ�������ĳ���
    //----------------------------------
    //----------------------------------  
    //----------------------------------
    set_imask(7);             //������Ӧ�������ȼ����ж�
    enable_irq();             //�����ж�
    //watchdog_enable();      //�������Ź� 
    //----------------------------------
    while(1)
    {
        main_loop_deal();
        clear_watchdog();
    }
}
//------------------------------------------------------------------------------
void ports_configure(void)   //�˿����ó���
{
    //Configure P00~P03 (P00 is ���̵���RY, P01 is NC, P02 is EEPROM-WP, P03 is ALM)
    P0PLU = 0x00;   //P00~P03 Pull-up resistor not added
    P0OUT = 0x00;  //P00=1;
    P0ODC = 0x05;   //P00/P01/P03=0 P02=1;(0=Push-pull output 1=Nch open-drain output)  
    P0DIR = 0x05;   //P00=RY=input mode,P01,P02,P03 is input mode (0=input mode 1=output mode)
    //ģʽ�Ĵ����ھ��幦��������
    P0MD = 0x08;    //P00,P01,P02 is IO, P03 is TM5IO pin/IRQ03 pin(0=IO 1=Special function)
    
    //Configure P20~P25 (P20=TXD, P21=EEPROM-SCL, P22=RXD, P23=���ͨѶTXD, P24=EEPROM-SDA, P25=���ͨѶRXD)
    P2PLU = 0x20;  //not Pull-up
    P2OUT = 0x3F;  //P20~P25=1;
    P2ODC = 0x12;  //P21/P24=1;(0=Push-pull output 1=Nch open-drain output)
    P2DIR = 0x1B;  // (0=input mode 1=output mode)
    //ģʽ�Ĵ����ھ��幦��������
    P2MD = 0x2D;   //(0=IO 1=Special function)
    
    //Configure P40,P41(P40 is HW, P41 is HV)
    P4PLU = 0x00;   //P40,P41 not Pull-up
    P4OUT |= 0x00;  //P40,P41 input
    P4DIR = 0x00;   //P40,P41 is input mode (0=input mode 1=output mode)
    //ģʽ�Ĵ����ھ��幦��������
    P4MD1 = 0x00;   //P40,P41 is TMnIO pin/IRQ pin (0=IO 1=Special function)
    P4MD2 = 0x00;   //selected by P4MD1 register   
    
    //Configure P50,P51 (P50 is HU, P51 is LED)
    P5PLU = 0x00;   //P50,P51 not Pull-up
    P5OUT |= 0x02; //P51 output H
    P5DIR = 0x02;   //P50 is input mode, P51 is output mode (0=input mode 1=output mode)
    //ģʽ�Ĵ����ھ��幦��������
    P5MD1 = 0x00;   //P50 is TMnIO pin/IRQ pin ,P51 is IO (0=IO 1=Special function)
    P5MD2 = 0x00;   //selected by P5MD1 register
    
    //Configure P60,P61 (P60=��ַ����1,P61=��ַ����2)
    P6PLU = 0x00;   //P60,P61 Pull-up resistor not added 
    P6OUT |= 0x00;  //P60,P61 input
    P6DIR = 0x00;   //P60,P61 is input mode (0=input mode 1=output mode)
    //ģʽ�Ĵ����ھ��幦��������
    P6MD1 = 0x00;   //P60,P61 is IO (0=IO 1=Special function)
    P6MD2 = 0x00;   //selected by P6MD1 register
    
    //Configure P80~P85 (P80~P85 is PWM pin)
    P8PLU = 0x00;   //P80~P85 not Pull-up
    P8OUT &= ~0x3F; //P80~P85 out L (invalid level is L)
    P8DIR = 0x3F;   //P80~P85 is output mode  (0=input mode 1=output mode)
    //ģʽ�Ĵ����ھ��幦��������
    P8MD = 0x00;    //P80~P85 Special function is PWM pin  (0=IO 1=PWM)
    
    //Configure PC0~PC7 (PC0 is Idc, PC1 is THM, PC2 is Vdc, PC3~PC7 is NC)
    PCPLU = 0x00;   //PC0~PC7 not Pull-up
    PCOUT |= 0x00;  //PC0~PC7 input
    PCDIR = 0x00;   //PC0~PC7 is input mode  (0=input mode 1=output mode)
    //ģʽ�Ĵ����ھ��幦��������
    PCMD = 0x00;    //PC0~PC7 is IO (0=IO 1=AD)
    
    //Configure P20,P21,P22,P02,P03 Input voltage level
    SWCNT = 0x00;  //P20,P21,P22 Input voltage level is 0.8Vdd ,P02,P03 Input voltage level is 0.8Vdd 
}
//------------------------------------------------------------------------------
void read_para_RomtoRam(void)      //��rom�ж�ȡ������ram
{
    uint16_t i;
    
    for (i = 0; i < (MAX_PARA_NUMBER + 1); i++)
    {
        ram_para[i] = EEPROM_InitData[i][VAL_ROW];   //��EEP��������RAM��
        clear_watchdog();     //�忴�Ź�
    }
}
//------------------------------------------------------------------------------
void control_init(void)       //���ϵ�׶Σ����ڳ�ʼ��һЩ�������ĳ���
{
    uint16_t lus_tmp = 0;    
    //------------------------------------------------------
    if (ram_para[num_cmd_source] == 1)  //�������λ�����ƣ������
    {
        bflg_host_ctrl = 1;   //����λ�����Ʊ�־
    }
    else
    {
        bflg_host_ctrl = 0;   //����λ�����Ʊ�־����LCD������
        
        if (ram_para[num_direction] == 1)
        {
            bflg_current_direction = 1; //��˳ʱ����ת��־
        }
        else
        {
            bflg_current_direction = 0; //����ʱ����ת��־
        }        
    }
    //------------------------------------------------------
    guc_comm_address = COMM_ADDRESS;
    //guc_comm_address += 1;
    //------------------------------------------------------
    //�ϵ�Ƿѹ��ʱ500ms
    bflg_powerup_delaytime = 1;    //�ϵ���ó�ʼ�ϵ���ʱ��־
    gss_powerup_delaytimer = 0;    //���ʼ�ϵ���ʱ��ʱ��
    //------------------------------------------------------
    //ͣ�������ʼ��
    bflg_stop_space_delaytime = 1;
    gss_stop_space_delaytimer = 5;//ram_para[num_stop_space_delaytime];
    //------------------------------------------------------
    Iout_prot_variable_init();     //��Iout�����йصı�����ʼ������
    Udc_prot_variable_init();      //ĸ�ߵ�ѹ�����йر�����ʼ������
    //------------------------------------------------------
    freq_rate_init();         //Ƶ�������ٳ�ʼ������
    //------------------------------------------------------
    //�õ�������Ļ������ֵ
    gsl_PI_initval = ram_para[num_Integral_initval];
    gsl_PI_initval *= 32768;
    gsl_PI_initval /= ram_para[num_V_max];
    gsl_PI_initval *= 256;
    //------------------------------------------------------
    SPRY_PIN_OUTPUT_L;
    //------------------------------------------------------
    lus_tmp = ram_para[num_MOTOR_max_n];
	lus_tmp *= ram_para[num_MOTOR_p];//motor_para[guc_motor_ID][MOTOR_p];
	lus_tmp *= 5;
	lus_tmp /= 3;    
    gus_MAX_setfreq = lus_tmp; //�õ�����趨Ƶ��ֵ     
}
//------------------------------------------------------------------------------
void TM23_configure(void)     //TMR23ģ�����ó���
{
	  //--------------------------------------------------------------------------
    //��8bit������timer2��timer3���ó�16bit��ʱ��ʱ��������1ms�ж�
    //����1ms��ʱ�Ĳ���
    //------------------------------------------------------
    //G4ICR����4�жϿ��ƼĴ���
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  LV2 |  LV1 |  LV0 |  -   |  IE2 |  IE1 |  IE0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   | IR2  | IR1  | IR1  |  -   | ID2  | ID1  | ID0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //LV2~LV0 : �ж����ȼ�����, 0��7(0���  7��ֹ)
    //IE2~IE0 : �ж�ʹ�ܱ�־  IE2=TM16 compare/capture B IE1=TM16 compare/capture A  IE0=TM16 overflow/underflow
    //IR2~IR0 : �ж������־
    //ID2~ID0 : �жϼ���־     
    //--------------------------------------------------------------------------
    G4ICR = 0x4000; //��ֹTM3�ж�,�����ж����ȼ���Ϊ4(0���  7��ֹ)
    
    TM3MD &= 0x7F;  //��ֹTM3BC����
    TM2MD &= 0x7F;  //��ֹTM2BC����
    
    //TM03PSC��Ԥ��Ƶ���ƼĴ���0
    //--------------------------------------------------------------------------
    //reg_bit  |   7    |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |TMPSCNE0|  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |   0    |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //TMPSCNE0:Ԥ��Ƶ����ʹ��ѡ��  0=��ֹ  1=ʹ��
    //--------------------------------------------------------------------------
    TM03PSC = 0x00;
    
    //TM03EXPSC���ⲿԤ��Ƶ���ƼĴ���0
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0    |  
    // flag    |TM3IN |TM2IN |TM1IN |TM0IN |  -   |  -   |  -   |EXPSCNE0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0    |
    //--------------------------------------------------------------------------
    //TM3IN:T3����ʱ��Դѡ��  0=TM3IO pin input  1=IOCLK/128
    //TM2IN:T2����ʱ��Դѡ��  0=TM2IO pin input  1=IOCLK/128
    //TM1IN:T1����ʱ��Դѡ��  0=TM1IO pin input  1=IOCLK/128
    //TM0IN:T0����ʱ��Դѡ��  0=TM0IO pin input  1=IOCLK/128
    //EXPSCNE0:Ԥ��Ƶ(1/128)����ʹ��ѡ��  0=��ֹ  1=ʹ��
    //--------------------------------------------------------------------------
    TM03EXPSC = 0x00;
    
    //TM3BR: T3�����Ĵ��� (TM2BR��TM3BR����)
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM3BR7|TM3BR6|TM3BR5|TM3BR4|TM3BR3|TM3BR2|TM3BR1|TM3BR0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM3BR7~0: ������TM3BC����ʱ,���м����ĳ�ֵ��	TM3BC�����,��TM3BR���ص�TM3BC�С�
    //--------------------------------------------------------------------------
    TM3BR = 0xFF;
    TM2BR = 0xFF;
    
    //TM3MD: T3ģʽ�Ĵ���
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM3CNE|TM3LDE|  -   |  -   |  -   |TM3CK2|TM3CK1|TM3CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM3CNE: ��������תʹ��ѡ��  0=����ת  1=��ת
    //TM3LDE: ��������ʼ��   0=������ת   1=��ʼ��
    //TM3CK2~0: ����ʱ��Դѡ��  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=��T2����   
    //          100=T0����      101=T1����    110=T2����     111=TM3IO pin����(������),IOCLK/128
    //--------------------------------------------------------------------------  
    TM3MD = 0x03;
    
    //TM2MD: T2ģʽ�Ĵ���
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM2CNE|TM2LDE|  -   |  -   |  -   |TM2CK2|TM2CK1|TM2CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM2CNE: ��������תʹ��ѡ��  0=����ת  1=��ת
    //TM2LDE: ��������ʼ��   0=������ת   1=��ʼ��
    //TM2CK2~0: ����ʱ��Դѡ��  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=��T1����   
    //          100=T0����      101=T1����    110=��ֹ����   111=TM2IO pin����(������),IOCLK/128
    //--------------------------------------------------------------------------
    TM2MD = 0x00;
    
    //1000us / (1 / 20M) - 1 = 19999 = 0x4E1F
    TM3BR = 0x4E;
    TM2BR = 0x1F;
    
    TM3MD |= 0x40;       //��ʼ��TM3BC
    TM2MD |= 0x40;       //��ʼ��TM2BC
    
    TM3MD &= ~0x40;      //���ʼ��TM3BC��־
    TM2MD &= ~0x40;      //���ʼ��TM2BC��־
    
    G4ICR &= ~0x0020;    //������ж�TM3�ж������־
    G4ICR |= 0x0200;     //��TM3�����ж�ʹ�ܱ�־   
    //------------------------------------------------------
    gss_timer_10ms = 10;      //��ʼ��10ms�����Ĵ���
    gss_timer_100ms = 100;    //��ʼ��10ms�����Ĵ���
    gss_timer_1s = 1000;      //��ʼ��1s�����Ĵ���
    gss_timer_1min = 60;      //��ʼ��1min�����Ĵ���  
    //------------------------------------------------------
    TM3MD |= 0x80;       //����TM3BC����
    TM2MD |= 0x80;       //����TM2BC����
}
//------------------------------------------------------------------------------
void TM23_1MS_INT(void)  //1ms��ʱ�жϷ������
{     
    //------------------------------------------------------
	  bflg_1ms_reach = 1;  //��1ms��ʱ��־λ
    //------------------------------------------------------
    gss_timer_10ms--;
    if (gss_timer_10ms <= 0)  //10ms��ʱ��
    {
        bflg_10ms_reach = 1;
        gss_timer_10ms = 10;
    }
    //------------------------------------------------------
    gss_timer_100ms--;
    if (gss_timer_100ms <= 0) //100ms��ʱ��
    {
        bflg_100ms_reach = 1;
        gss_timer_100ms = 100;
    }
    //------------------------------------------------------
    gss_timer_1s--;
    if (gss_timer_1s <= 0)    //1s��ʱ��
    {
        bflg_1s_reach = 1;
        gss_timer_1s = 1000;
    }
    //------------------------------------------------------
    if (bflg_running == 1)    //���������ģʽ
    {       
        //------------------------------
        if (bflg_askfor_calc_outpercent == 0)
        {
    	      check_freq_updata();   //��ʱ����Ƶ�ʸ��µĳ���(ÿ1ms���1��)
    	      //----------------------------------------------
    	      bflg_askfor_calc_outpercent = 1; //�����������������ʱ�־
        }
    }
}
//------------------------------------------------------------------------------
void main_loop_deal(void)     //��ѭ���������
{
    timer_op();     //��ʱ�����ӳ���
    //--------------------------------------------------------------------------
    Hall_updata_deal();  //�����źŸ��´������     
    //--------------------------------------------------------------------------
    switch (main_loop_cnt)
    {
        case 0:     //ADת���������ж�
            //----------------------------------------------
            if (bflg_allow_calc_Iout == 1)  
            {
	              calc_Iout_deal();  	    //����Iout����
                bflg_allow_calc_Iout = 0;
            }        	   
        	  //----------------------------------------------
            if (bflg_allow_calc_Udc == 1)
            {
   	            calc_Udc_deal();       //����Udc����
   	            bflg_allow_calc_Udc = 0;
            }       	   
        	  //----------------------------------------------
            if (bflg_allow_calc_Tm == 1)
            {
        	      calc_Tm_deal();         //����Tm����
        	      bflg_allow_calc_Tm = 0;
        	  }
        	  //----------------------------------------------
            if (bflg_powerup_delaytime == 0)//�ϵ���ʱ�׶Σ����α������ϼ��
            {              	  
        	      prot_Udc_delaytime();//Udc��ѹ������ʱ����
            }	  
            //----------------------------------------------
            //trip_code_deal();     //���ϴ��봦�����
            //----------------------------------------------
            //state_code_deal();    //״̬���봦�����
            //----------------------------------------------
            break;
             
        case 1:     //ͨѶ���ݴ������ػ��ж�
            //----------------------------------------------
            if (bflg_SCI0_allow_rx == 1)
            {
            	  bflg_SCI0_allow_rx = 0;
            	  SCI0_receive_int();     //SCI0ͨѶ���ճ�ʼ������
            }
            //----------------------------------------------
            if (bflg_SCI0_rx_ok == 1)
            {
            	  bflg_SCI0_rx_ok = 0;
            	  SCI0_rx_data_deal();    //ͨѶ�������ݴ������
            }
            //----------------------------------------------
            if (bflg_SCI0_allow_tx == 1)
            {
            	  bflg_SCI0_allow_tx = 0;
            	  SCI0_send_init();       //SCI0ͨѶ���ͳ�ʼ������
            }
            //----------------------------------------------
            //sci0rx_data_deal();    //sci0�������ݴ������
            //----------------------------------------------
            //get_host_setfreq();    //�õ���λ���趨Ƶ�ʵĳ���
            //----------------------------------------------
            trip_code_deal();     //���ϴ��봦�����
            //----------------------------------------------
            state_code_deal();    //״̬���봦�����            
            //----------------------------------------------
            fan_ctrl_deal();       //ѹ�����Ƴ���
            //----------------------------------------------
            freq_ctrl_deal();      //Ƶ�ʿ��Ƴ���
            //----------------------------------------------
            SCI1rx_data_deal();     //SCI1�������ݴ������,����ѭ���е���             
            
            check_run_stop_deal();      //��鿪�ػ�����ĵĳ��� 
            //----------------------------------------------

            //----------------------------------------------
            //----------------------------------------------
            break;
        case 2:
            //------------------------------------------------------------------
            if ((bflg_running == 1) && (bflg_askfor_calc_outpercent == 1))
            {
                bflg_askfor_calc_outpercent = 0;  //��Ҫ�������������ʱ�־
                //--------------------------------------------------------------
                Sensored_cale_Vout();        //��λ�ô��������Ƽ��������ѹ�ĳ���
                //--------------------------------------------------------------
                calc_percent_theta();   //���ڼ��㵱ǰ�����ʺ���λ�ǲ����ĳ���
            }

        	  //---------------------------------------------
            break;         	 
       	 default:       	 	  
	      	  break;	          	  
    }
    //--------------------------------------------------------------------------
    main_loop_cnt++;
    if (main_loop_cnt >= 3)
    {
        main_loop_cnt = 0;
    }     
    //--------------------------------------------------------------------------
}
//------------------------------------------------------------------------------
void timer_op(void) //��ʱ�����ӳ������ڴ������еĶ�ʱ��ʱ����
{
    if (bflg_1ms_reach == 1)
    {
        bflg_1ms_reach = 0;
        //---------------------------1ms��ʱ����--------------------------------
        check_powerup_delaytime();      //�ϵ�����ʱ����
        
        check_spry_delaytime();         //���̵������ϺͶϿ�����ʱ��ʱ����
        
        bootstrap_charger_delaytime();  //�Ծٳ����ʱ����
        
        //check_sensored_startup_deal();  //��λ�ô���������ʱ�Ŀ������ٽ׶εĴ������
        //----------------------------------------------------------------------
        SCI0_tx_delaytime();            //SCI0ͨѶ������ʱ����
        
        SCI0_rx_delaytime();            //SCI0ͨѶ������ʱ����
        
        SCI0_rx_end_delaytime();        //SCI0ͨѶ���պ����ʱ����
        
        SCI1rx_on_dealtime();           //��ʱ���ռ�ʱ����
        
        SCI1rx_end_delaytimer();        //ͨѶ���պ����ʱ����
        
        SCI1tx_on_dealtime();           //��ʱ���ͼ�ʱ����
        
        //----------------------------------------------------------------------
        calc_Iout_delaytime();          //����Iout��ʱ����
    }
    //--------------------------------------------------------------------------
    if (bflg_10ms_reach == 1)
    {
        bflg_10ms_reach = 0;
        
        bflg_allow_calc_Tm = 1;
        //---------------------------10ms��ʱ����-------------------------------
        if (bflg_powerup_delaytime == 0)     //�ϵ���ʱ�׶Σ����α������ϼ��
        {
        	  Idc_zero_adjust();          //�������Idc���У׼����
        }
        //----------------------------------------------------------------------
        Hall_fault_delaytime();    //�����źŹ�����ʱ����
        
        Hall_error_delaytime();    //�����źŴ���ʱ����
        
        Hall_direction_delaytime();     //�����źŷ������ʱ����
        
        Hall_off_delaytime();      //�����ź�ֹͣ��ʱ����
        //----------------------------------------------------------------------
        prot_ALM_delaytime();           //���ͣ��ʱALM������ʱ����
        
        prot_motor_stall_delaytime();   //���ʧ�ٱ�����ʱ����
        
        prot_motor_block_delaytime();   //�����ת������ʱ����
        //----------------------------------------------------------------------
        //debug
        trip_lampblink_deal();     //���ϵ���˸����
        //----------------------------------------------------------------------
    }
    //--------------------------------------------------------------------------
    if (bflg_100ms_reach == 1)
    {
        bflg_100ms_reach = 0;
        //-------------------------100ms��ʱ����--------------------------------
        if (bflg_powerup_delaytime == 0)     //�ϵ���ʱ�׶Σ����α������ϼ��
        {       
            prot_Iout_delaytime();      //Iout����������ʱ����
            
            //prot_Udc_delaytime();       //Udc��ѹ������ʱ����
            
            prot_Tm_delaytime();        //Tmģ���¶ȱ���������
            
            prot_motor_overspeed_delaytime();//������ٱ�����ʱ����
            
            check_spry_ctrl_term();     //���spry���������ĳ���
        }

    }
    //--------------------------------------------------------------------------
    if (bflg_1s_reach == 1)
    {
        bflg_1s_reach = 0;
        gss_timer_1min--;
        //---------------------------1s��ʱ����---------------------------------
        SCI0_check_delaytime();
        
        SCI0_fault_delaytime();    //SCI0ͨѶ������ʱ����     
        //----------------------------------------------------------------------
        stop_space_delaytime();    //ͣ�������ʱ����
        
        SCI1comm_abend_dealtime();      //SCI1ͨѶ�쳣��ʱ������1S��ʱ�����е���  
        SCI1rx_ctrl_abend_dealtime();   //SCI1���տ�����Ϣ�쳣�������,��1S��ʱ�����е���
        
        //----------------------------------------------------------------------
    }
    //-------------------------
    if (gss_timer_1min <= 0)
    {
        gss_timer_1min=60;
        //--------------------------1��������-----------------------------------
        trip_lock_delaytime();     //����������ʱ����
    }
    //--------------------------------------------------------------------------
}
//------------------------------------------------------------------------------
void fan_ctrl_deal(void)     //ѹ�������Ƴ���
{
    if (bflg_running == 0)    //�����ͣ��״̬
    {
        if ((bflg_powerup_delaytime == 0)&&(bflg_stop_space_delaytime == 0)
        && (bflg_trip_stop == 0)&&(bflg_trip_lock == 0)&& (SPRY_PIN_LEVEL == 1))    //���ͣ����ʱ�����޹��ϣ������̵�������
        {
            if (bflg_askfor_run == 1)   //����������У����趨Ƶ�ʲ�Ϊ0
            {
                if (bflg_bootstrap_charger == 0)  //���δ�Ծٳ��
                {
                    bflg_bootstrap_charger = 1;   //���Ծٳ���־
                    //----------------------------------------------------------
                    //Ӳ����������
                    if (ram_para[num_prot_ALM_cnt] == 0)    //��������趨��������Ϊ0
                    {
                        PWMOFF0B = 0x7026;//0xF026;//

                        PWMOFF0B1 &= ~0x0001;     //��PRTBST0λ���ͷ�PWM0����Ӳ������B����
                        PWMOFF0B |= 0x0001;
                    }
                    else
                    {
                        PWMOFF0B = 0x0000;        //����������Ϊ0ʱ������Ӳ������
                    }
                    //----------------------------------------------------------
                    //����IRQ03���жϣ�����ALM����
                    *(unsigned char *)&G24ICR = 0x01;  //���IRQ03���ж������־
                    G24ICR |= 0x0100;        //��IRQ03���ж�ʹ�ܱ�־
                    //----------------------------------------------------------
                    //�Ծٳ���ʼ��
                    bflg_bootstrap_charger_delaytime = 1;
                    gss_bootstrap_charger_delaytimer = 0;
                    //----------------------------------------------------------
                    //�����ź���ش���

                    //----------------------------------------------------------
                    //���������ź��ж�
                    *(unsigned char *)&G29ICR = 0x03;  //������ж�IRQ09��IRQ08�����־
                    *(unsigned char *)&G30ICR = 0x01;  //������ж�IRQ10�����־

                    G29ICR |= 0x0300;        //�����ж�IRQ09��IRQ08ʹ�ܱ�־
                    G30ICR |= 0x0100;        //�����ж�IRQ10ʹ�ܱ�־
                    //----------------------------------------------------------
                }
                else if (bflg_bootstrap_charger_delaytime == 0)  //����Ծٳ�����
                {
                    bflg_bootstrap_charger = 0;   //���Ծٳ���־
                    //--------------------------------------
                    bflg_fan_running = 1;   //�÷�����б�־
                    bflg_running = 1;       //�����б�־
                    bflg_askfor_run = 0;

                    bflg_theta_ctrl1 = 0;
                    bflg_theta_ctrl2 = 0;
                    //----------------------------------------------------------
                    //ȷ����ʼ�Ƕ�
                    gus_Hall_value = HALL_PIN;    //�õ������ź�ֵ
                    gsc_Hall_sector = gsc_sector_table[gus_Hall_value];    //�õ�Hall����

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
                    gul_theta.uword.high += 300;
                    theta.ulword = gul_theta.ulword;

                    //gus_Hall_watchdog = 0;//debug
                    //--------------------------------------
                    //�õ����ֳ�ֵ
                    gsl_FreqPI_integral = gsl_PI_initval;
                    //----------------------------------------------------------
                    bflg_askfor_calc_outpercent = 1; //�����������������ʱ�־
                    //----------------------------------------------------------
                    open_pwm();             //��ʼPWM���
                    //--------------------------------------	  	  	  	      
                    gus_freq_out = gus_freq_real;
                }
            }
        }
    }
    else  //����ǿ���״̬
    {
        if (bflg_hostComm_shutdown == 1)//��λ������ͣ������
        {
            bflg_fan_running = 0;  //�������б�־

            bflg_running = 0;      //������б�־
            bflg_askfor_stop = 0;  //������ֹͣ��־

            shut_pwm();            //�ر�PWM   	  	

            bflg_hostComm_shutdown = 0;
            bflg_stop_space_delaytime = 1;
            gss_stop_space_delaytimer = ram_para[num_stop_space_delaytime];		  	  	
        }
        else if (bflg_trip_stop == 1)   //����ǹ���ͣ��
        {
            bflg_fan_running = 0;  //�������б�־

            bflg_running = 0;      //������б�־
            bflg_askfor_stop = 0;  //������ֹͣ��־

            ///shut_pwm();            //�ر�PWM(bflg_trip_stop��1�������Ѿ��ر�PWM)

            //-----------------------------------
            if ((bflg_prot_Iout_OL == 1)||(bflg_prot_Iout_POC == 1) || (bflg_prot_Iout_OC == 1))
            {
                guc_Iout_fault_cnt++;
                if (guc_Iout_fault_cnt >= 5) 
                {
                    bflg_stop_space_delaytime = 1;
                    gss_stop_space_delaytimer = ram_para[num_stop_space_delaytime];	  	  	  	
                }
                else
                {
                    bflg_trip_clear = 1;
                    bflg_stop_space_delaytime = 1;
                    gss_stop_space_delaytimer = 2;//ͣ��2s����������	  	  	  	
                }		  	      	 
            }
            else if (bflg_prot_ALM == 1)
            {
                guc_ALM_fault_cnt++;
                if (guc_ALM_fault_cnt >= 3)
                {
                    bflg_stop_space_delaytime = 1;
                    gss_stop_space_delaytimer = ram_para[num_stop_space_delaytime];	  	  	  	
                }
                else
                {
                    bflg_trip_clear = 1;
                    bflg_stop_space_delaytime = 1;
                    gss_stop_space_delaytimer = 5;//ͣ��2s����������	  	  	  	
                }		  	      	   
            }	  	      
            else if (bflg_prot_motor_block == 1)
            {
                guc_motor_block_cnt++;
                if (guc_motor_block_cnt >= 5)
                {
                    bflg_stop_space_delaytime = 1;
                    gss_stop_space_delaytimer = ram_para[num_stop_space_delaytime];	  	  	  	
                }
                else
                {
                    bflg_trip_clear = 1;
                    bflg_stop_space_delaytime = 1;
                    gss_stop_space_delaytimer = 2;//ͣ��2s����������	  	  	  	
                }		  	      	   
            }
            else
            {
                bflg_stop_space_delaytime = 1;
                gss_stop_space_delaytimer = ram_para[num_stop_space_delaytime];	  	      	
            } 
        }
        else if (bflg_askfor_stop == 1) //���������ֹͣ��־
        {
            bflg_fan_running = 0;      //�������б�־

            if (gus_freq_real <= ram_para[num_shut_freq])   //����Ƶ�� debug
            {
                bflg_running = 0;       //������б�־
                bflg_askfor_stop = 0;   //������ֹͣ��־

                shut_pwm();             //�ر�PWM

                bflg_stop_space_delaytime = 1;
                gss_stop_space_delaytimer = ram_para[num_stop_space_delaytime];
            }
        }
        else 
        {
            if ((bflg_current_direction == bflg_actual_direction)
            && (gus_freq_out >= 500)&&(gus_freq_real >= 500))//ram_para[num_min_freq]))
            {
                guc_motor_block_cnt = 5;
                guc_Iout_fault_cnt = 5;
                guc_ALM_fault_cnt = 3;	
            } 	  	  	
        }
    }
}
//------------------------------------------------------------------------------
void freq_ctrl_deal(void)     //Ƶ�ʿ��Ƴ���
{
	  uint16_t lus_freq_set;
	  uint16_t lus_min_protfreq = 0;
	  
	  if (bflg_running == 1)    //��������н׶�
	  {
	  	  if (bflg_fan_running == 0)      //������ֹͣ
	  	  {
	  	  	  lus_freq_set = 0;
	  	  	  //lus_freq_set = ram_para[num_shut_freq];
	  	  }
	  	  else
	  	  {
	  	  	  if (bflg_host_ctrl == 1)
	  	  	  {
	  	  	  	  lus_freq_set = gus_host_setfreq;
	  	  	  }
	  	  	  else 
	  	  	  {
	  	  	  	  lus_freq_set = gus_lcd_setfreq;
	  	  	  }
	  	  	  
	  	  }
	  	  //--------------------------------------------------
        if ((bflg_prot_Iout_decfreq == 1) || (bflg_prot_Tm_decfreq == 1))
        {
            //��������Ƶ�����򲢵õ���С��Ϊ0�ı���Ƶ�ʵĳ���
            lus_min_protfreq = get_min_decfreq_deal();
            //----------------------------------------------
            if (lus_freq_set > lus_min_protfreq)
            {
                lus_freq_set = lus_min_protfreq;
            }
        }
	  	  //--------------------------------------------------
	  	  if (lus_freq_set < ram_para[num_min_freq])
	  	  {
	  	  	  lus_freq_set = ram_para[num_min_freq];
	  	  }
	  	  
	  	  if (lus_freq_set > ram_para[num_max_freq])
	  	  {
	  	  	  lus_freq_set = ram_para[num_max_freq];
	  	  }
	  	  //--------------------------------------------------
	  	  gus_freq_set = lus_freq_set;
	  }
	  else  //�����ͣ��
	  {
	  	  gus_freq_set = 0;
	  }
}
//------------------------------------------------------------------------------
void check_run_stop_deal(void)     //��鿪�ػ�����ĵĳ���
{
    if (bflg_host_ctrl == 1)  //�������λ�����ƣ������
    {
        //��λ��Ҫ��������Ƶ�ʲ�Ϊ�����޽���ͣ����־������п����Ĵ���
        if ((bflg_host_running == 1) && (gus_host_setfreq > 0))
        {
            run_key();        //���м�����      
        }
        else if ((bflg_host_running == 0) || (gus_host_setfreq == 0))    //ͣ���Ĵ���
        {
            stop_key();       //ͣ��������
        }
    }
    else       //�����LCD�����ƣ������
    {
        if (gus_lcd_setfreq > 0)   //�����Ĵ���
        {
            run_key();   //���м����� 
        }
        else
        {
            stop_key();  //ͣ��������
        }    	  
    }   
}
//------------------------------------------------------------------------------
void run_key(void)       //���м�����
{
    if (bflg_eeprom_change == 0)   //�����eeprom�����־�����˳�
    {  
        if ((bflg_running == 0) && (bflg_askfor_run == 0)) //��������״̬��δ���տ����ź�,�����
        {  
            bflg_askfor_stop = 0;      //���Ҫ��ͣ����־λ
            bflg_askfor_run = 1;       //��Ҫ�����б�־
        } 
    } 
}
//------------------------------------------------------------------------------
void stop_key(void)      //ͣ��������
{
    if (bflg_askfor_stop == 0)    //���û��Ҫ��ͣ����־�������
    {         
        if (bflg_running == 1)
        {
            bflg_askfor_stop = 1; //��Ҫ��ͣ����־
        }
        else if (bflg_askfor_run == 1)
        {
            bflg_askfor_run = 0;  //�����������״̬������ֻ��Ҫ�����б�־��������ñ�־
            //----------------------------------
            PWMSEL0 = 0x0FC0;      //PWM�������������,ȫ��Ч
            
            gus_val_a = gus_peak_pwm + 1;
            gus_val_b = gus_peak_pwm + 1;
            gus_val_c = gus_peak_pwm + 1;    //���������ֵ����ֵ
            
            TCMP0A = gus_val_a;    //PWM00ռ�ձ��趨  U��
            TCMP0B = gus_val_b;    //PWM01ռ�ձ��趨  V��
            TCMP0C = gus_val_c;    //PWM02ռ�ձ��趨  W�� 
        }  
    } 
}
//------------------------------------------------------------------------------
uint16_t get_min_decfreq_deal(void)     //��������Ƶ�����򲢵õ���С��Ϊ0�ı���Ƶ�ʵĳ�����Ƶ���趨�����е���
{
	  uint16_t max_freq;
	  uint16_t min_freq;
    uint16_t tmp_freq;	  
    
    max_freq = gus_prot_Iout_freq;
    min_freq = gus_prot_Tm_freq;
    //----------------------------------
    if (max_freq < min_freq)
    {
    	  tmp_freq = max_freq;
    	  max_freq = min_freq;
    	  min_freq = tmp_freq;
    }
    //----------------------------------
    if (min_freq != 0)
    {
    	  tmp_freq = min_freq;
    }
    else if (max_freq != 0)
    {
    	  tmp_freq = max_freq;
    }
    else
    {
    	  tmp_freq = ram_para[num_min_freq];
    }
    //----------------------------------
    return tmp_freq;
}
//------------------------------------------------------------------------------
void check_powerup_delaytime(void)      //�ϵ�����ʱ������1ms��ʱ�����е���
{
    if (bflg_powerup_delaytime == 1)    //�ϵ�Ƿѹ��ʱ��ʱ����
    {
        gss_powerup_delaytimer++;
        if (gss_powerup_delaytimer >= 1500)
        {
            bflg_powerup_delaytime = 0;
            gss_powerup_delaytimer = 0;
        }
    }
}
//------------------------------------------------------------------------------
void check_spry_ctrl_term(void)    //���spry���������ĳ�����100ms��ʱ�����е���
{
    if ((bflg_prot_Udc_LU == 1) && (SPRY_PIN_LEVEL == 1))   //Ƿѹ��SPRY����״̬,�����
    {
        if (bflg_spry_off_delaytime == 0)
        {
            bflg_spry_off_delaytime = 1;     //�öϿ����̵�������ʱ��־
            bflg_spry_on_delaytime = 0;      //���������̵�������ʱ��־
            gss_spry_delaytimer = 0;         //�����̵�����ʱ��ʱ��  
        }
    }
    else if ((bflg_prot_Udc_LU == 0) && (SPRY_PIN_LEVEL == 0))   //��Ƿѹ��SPRY�Ͽ�״̬,�����
    {
        if (bflg_spry_on_delaytime == 0)
        {
            bflg_spry_off_delaytime = 0;     //��Ͽ����̵�������ʱ��־
            bflg_spry_on_delaytime = 1;      //���������̵�������ʱ��־
            gss_spry_delaytimer = 0;         //�����̵�����ʱ��ʱ��  
        }
    }
}
//------------------------------------------------------------------------------
void check_spry_delaytime(void)         //���̵������ϺͶϿ�����ʱ��ʱ������1ms��ʱ�����е���
{
    if (bflg_spry_off_delaytime == 1)   //���̵����Ͽ���ʱ
    { 
        gss_spry_delaytimer++;
        if (gss_spry_delaytimer >= 300)
        {
            gss_spry_delaytimer = 0;
            bflg_spry_off_delaytime = 0;
            
            SPRY_PIN_OUTPUT_L;     //�Ͽ����̵���
        }
    }
    else if (bflg_spry_on_delaytime == 1)    //���̵���������ʱ
    {
        gss_spry_delaytimer++;
        if (gss_spry_delaytimer >= 20)
        {
            gss_spry_delaytimer = 0;
            bflg_spry_on_delaytime = 0;
            
            SPRY_PIN_OUTPUT_H;     //ʱ�䵽���������̵���
        }
    }
}
//------------------------------------------------------------------------------
void stop_space_delaytime(void)    //ͣ�������ʱ������s��ʱ�����е���
{
	  if (bflg_stop_space_delaytime == 1)
	  {
	  	  gss_stop_space_delaytimer--;
	  	  if (gss_stop_space_delaytimer <= 0)
	  	  {
	  	  	  gss_stop_space_delaytimer = 0;
	  	  	  bflg_stop_space_delaytime = 0;
	  	  }
	  }
}
//------------------------------------------------------------------------------
void trip_lampblink_deal(void)     //���ϵ���˸����,��10ms��ʱ�����е���
{
    uint8_t luc_tmp;
    
    //���ֺ�����ͨ�ù��ϵ���˸����
    if (guc_LED_trip_code != 0)
    {
    	  luc_tmp = guc_LED_trip_code;
    }
    else
    {
    	  luc_tmp = guc_lamp_trip_code;
    }
    
    if (bflg_trip_stop == 1)  //���������ͣ����ת
    {
        if (gus_trip_flash_count > 0)   //�����˸�����Ѿ������㣬��ʼ3������ʱ
        {
            gus_trip_flash_timer++;
            if (gus_trip_flash_timer >= 30)  //ÿ0.30����˸һ�� //PEAK ԭ��30
            {
                gus_trip_flash_timer = 0;
                NOT_TRIP_LAMP_PIN;
                gus_trip_flash_count--;
            }
        }
        else
        {
            SET_TRIP_LAMP_PIN;
            gus_trip_flash_timer++;
            if (gus_trip_flash_timer >= 120) //1.5������ʱ ʵ���趨Ϊ1.5-0.3��
            {
                gus_trip_flash_timer = 0;
                //gus_trip_flash_count = guc_lamp_trip_code;
                gus_trip_flash_count = luc_tmp;
                gus_trip_flash_count <<= 1;
            }
        }
    }
    else
    {
        if (bflg_running == 1)     //����ʱ���ȼ����˸
        {
             gus_trip_flash_timer++;
             if(gus_trip_flash_timer >= 10)     //ÿ0.1����˸һ��
             {
                 gus_trip_flash_timer = 0;
                 NOT_TRIP_LAMP_PIN;
             }
        }
        else
        {
            SET_TRIP_LAMP_PIN;      //����ͣ��ʱ����£����ϵƣ����Դ�ƣ�����
            gus_trip_flash_timer = 0;
            gus_trip_flash_timer = 0;
        }
    }
}

//------------------------------------------------------------------------------
