#ifndef	_HOST_COMM_C_
#define _HOST_COMM_C_
//------------------------------------------------------------------------------
#include "srMN103SFJ9D.h"
#include "user_typedefine.h"
#include "comm_variable.h"

#include "EEPROM_PARA.h"
#include "EEPROM_RW.h"

#include "AD_Convert.h"
#include "PWM_CTRL.h"
#include "protect_deal.h"
#include "Hallsignal_deal.h"
#include "Host_comm.h"
//------------------------------------------------------------------------------
void SCI0_configure(void);    //����SCI0����ͨѶ�ӿ�

void SCI0comm_reset(void);

void SCI0_check_delaytime(void);

void SCI0_send_init(void);    //SCI0ͨѶ���ͳ�ʼ������

void SCI0_receive_int(void);  //SCI0ͨѶ���ճ�ʼ������

void SCI0_rx_data_deal(void); //ͨѶ�������ݴ������

void SCI0_tx_delaytime(void); //SCI0ͨѶ������ʱ������1ms��ʱ�����е���

void SCI0_rx_delaytime(void); //SCI0ͨѶ������ʱ������1ms��ʱ�����е���

void SCI0_rx_end_delaytime(void);  //SCI0ͨѶ���պ����ʱ������1ms��ʱ�����е���

void SCI0_fault_delaytime(void);   //SCI0ͨѶ������ʱ������1s��ʱ�����е���

void SCI0_tx_int(void);       //SCI0�����жϳ���

void SCI0_rx_int(void);       //SCI0�����жϳ���

uint16_t CRC16(uint8_t *puchmsg, uint16_t usDataLen);  //����CRCУ��ĳ���

void sci0_Renesas_tx_data_load(void);   //sci0ͨѶRenesas�������ݼ��س���

void sci0_receive_int(void);  //sci0ͨѶ���ճ�ʼ������

void Renesas_rx_data_deal(void);    //RenesasЭ��������ݴ������

uint8_t Ascii_to_data(uint8_t luc_Ascii_tmp);      //Ascii��ת��ֵ����

uint8_t data_to_Ascii(uint8_t luc_data_tmp);       //��ֵתAscii��
//uint8_t guc_uart_text;
//------------------------------------------------------------------------------
//��־����
flag_type flg_SCI0;
//------------------------------------------------------------------------------
//��������
#define   SCI0_TX_LEN    20//16//12
#define   SCI0_RX_LEN    20//10//8

uint8_t   guc_SCI0_tx_buffer[SCI0_TX_LEN];
uint8_t   guc_SCI0_rx_buffer[SCI0_RX_LEN];

#define   SCI0_TX_ORDER       0xAA
#define   SCI0_RX_MIN_ORDER   0x55
#define   SCI0_RX_MAX_ORDER   0x59

#define   SCI0_Renesas_RX_ORDER    0x3E
#define   SCI0_Renesas_TX_ORDER    0x3C

#define   SCI0_Renesas_RX_BUF_LEN       12
#define   SCI0_Renesas_TX_BUF_LEN       20

//uint16_t  gus_Renesas_tx_buffer[20];
//uint16_t  gus_Renesas_rx_buffer[12];

//uint8_t guc_Renesas_trip_code;
//uint8_t guc_Renesas_lamp_trip_code;

uint16_t  gus_SCI0_check_delaytimer;
int16_t   gss_SCI0_init_delaytimer;

uint8_t   guc_SCI0_tx_len;
uint8_t   guc_SCI0_rx_len;
uint8_t   guc_SCI0_tx_point;
uint8_t   guc_SCI0_rx_point;

int16_t   gss_SCI0_tx_delaytimer;
int16_t   gss_SCI0_rx_delaytimer;

int16_t   gss_SCI0_rx_end_delaytimer;
int16_t   gss_SCI0_fault_delaytimer;

int16_t   gss_trip_clear_cnt;
//------------------------------------------------------------------------------
uint16_t  gus_host_speed_set;
uint16_t  gus_host_setfreq;
//------------------------------------------------------------------------------
uint8_t   guc_comm_address;   //ͨѶ��ַ
uint8_t   guc_motor_ID_error_cnt;  //ID�Ŵ����

uint8_t   guc_SCI0_type_cnt;

uint8_t   guc_SCI0_trip_code;
uint8_t   guc_LED_trip_code;

uint16_t gus_host_test;
//------------------------------------------------------------------------------
void SCI0_configure(void)     //����SCI0����ͨѶ�ӿ�
{
    //G25ICR����25�жϿ��ƼĴ���
    //-------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  LV2 |  LV1 |  LV0 |  -   |  -   |  IE1 |  IE0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //-------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | IR1  | IR1  |  -   |  -   | ID1  | ID0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //-------------------------------------------------------------------   
    //LV2~LV0 : �ж����ȼ�����, 0��7(0���  7��ֹ)
    //IE2~IE0 : �ж�ʹ�ܱ�־  (IE1=SCI0ͨѶ��/�������ж�  IE0=SCI0�������ж�)
    //IR2~IR0 : �ж������־
    //ID2~ID0 : �жϼ���־     
    //-------------------------------------------------------------------
    G25ICR = 0x5000;     //�жϲ�ʹ��,�ж����ȼ�Ϊ5
    
    //SC0RB: SCI0�������ݻ�����
    //SC0TB: SCI0�������ݻ�����
    
    //SIFCLK:����ʱ��ѡ��Ĵ���
    //--------------------------------------------------------------------------
    //reg_bit  |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |  
    // flag    |   -   |   -   |   -   |   -   |SC1CKS1|SC1CKS0|SC0CKS1|SC0CKS0| 
    //at reset |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |
    //--------------------------------------------------------------------------
    //SC1CKS1~0:SCI1ʱ��Դ   00=T0���� 01=T1���� 10=T2���� 11=T3����
    //SC0CKS1~0:SCI0ʱ��Դ   00=T0���� 01=T1���� 10=T2���� 11=T3����
    //--------------------------------------------------------------------------
    SIFCLK &= ~0x03;     //ѡ��T0������Ϊ�����ʼ�ʱ  
    
    //SC0CTR3: SCI0ģʽ�Ĵ���3
    //------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5  |  4  |   3   |  2  |  1  |  0  |  
    // flag    |  SC0FDC1~0  |  -  |  -  |SC0PSCE|    SC0PSC2~0    | 
    //at reset |  0   |  0   |  0  |  0  |   0   |  0  |   0 |   0 |
    //------------------------------------------------------------------------- 
    //SC0FDC1~0: SBO0������ݴ��������ѡ��   00=�̶����"1"  01=�̶����"0"  x1=���һλ����
    //SC0PSCE: Ԥ��Ƶʹ��ѡ��  0=��ֹ  1=ʹ��
    //SC0PSC2~0:   ʱ��ѡ�� (ͬ��ģʽ 000=timer�����1/2   001=timer�����1/4  010=timer�����1/16)
    //                      (ͬ��ģʽ 011=timer�����1/64  100=IOCLK/2         101=IOCLK/4)
    //                      (ͬ��ģʽ 110,111=��ֹ����)
    //                      (�첽ģʽ 000=timer�����1/32   001=timer�����1/64 010=timer�����1/256)
    //                      (�첽ģʽ 011=timer�����1/1024 100=IOCLK/2         101=IOCLK/4)
    //                      (�첽ģʽ 110,111=��ֹ����)
    //-------------------------------------------------------------------------   
    SC0CTR3 = 0x08;      //ʱ��ѡ���첽ģʽ��timer�����1/32
    
    //��������
    P2MD |= 0x05;   //P20 is SBO0 pin�� P22 is SBI0 pin , other is IO (0=IO 1=Special function) 
    
    //SC0CTR0: SCI0ģʽ�Ĵ���0	
    //------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2    |  1    |  0    |  
    // flag    |SC0CE1|SC0SSC|  -   |SC0DIR|SC0STE|SC0LNG2|SC0LNG1|SC0LNG0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0    |  0    |  0    |
    //------------------------------------------------------------------------- 
    //SC0CE1: ������Ч����ѡ��  (���ͣ�0=�½���  1=������)(���գ�0=������  1=�½���)
    //SC0SSC: ͬ��ͨѶ��ģʽʱ�ڵ�·ʱ��Դѡ��  0=�������ͨѶʱ��ͬ��  1=�ڲ�����ͨѶʱ��ͬ�� 
    //SC0DIR: ��λ��������ѡ��  0=�Ӹ�λ�׷�   1=�ӵ�λ�׷�   
    //SC0STE: ��������ѡ��  0=�ⲿ   1=�ڲ�   
    //SC0LNG2~0: ͬ�����д���λ��   000=1bit  ...  111=8bit         
    //-------------------------------------------------------------------------
    SC0CTR0 = 0x18;      //�ӵ�λ�׷�,�ڲ���������
    
    //SC0CTR1: SCI0ģʽ�Ĵ���1
    //------------------------------------------------------------------------
    //reg_bit  |  7   |   6   |   5   |   4   |  3   |  2   |  1   |  0   |  
    // flag    |SC0IOM|SC0SBTS|SC0SBIS|SC0SBOS|SC0CKM|SC0MST|  -   |SC0CMD| 
    //at reset |  0   |   0   |   0   |   0   |  0   |  0   |  0   |  0   |
    //------------------------------------------------------------------------- 
    //SC0IOM: ����������������ѡ��  0=SBI0  1=SBO0
    //SC0SBTS: SBT0���Ź���ѡ��  0=IO�˿�  1=����ʱ��IO 
    //SC0SBIS: ���������������ѡ��  0=�̶�"1"����  1=������������
    //SC0SBOS: SBO0���Ź���ѡ��  0=IO�˿�  1=�����������
    //SC0CKM: ����ʱ�ӵ�1/16��Ƶѡ��  0=����Ƶ  1=��Ƶ
    //SC0MST:  ����ʱ��ģʽѡ��   0=ʱ�Ӵ�ģʽ   1=ʱ����ģʽ
    //SC0CMD:  ͬ��ͨѶ/ȫ˫���첽ͨѶѡ��  0=ͬ��ͨѶ   1=ȫ˫���첽ͨѶ      
    //-------------------------------------------------------------------------
    SC0CTR1 = 0x3D; //SBI0Ϊ��������,SBT0Ϊͨ��IO,SBO0Ϊ�������,1/16��Ƶ,ȫ˫���첽ͨѶ
    
    //SC0CTR2: SCI0ģʽ�Ĵ���2
    //------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2  |   1   |   0   |  
    // flag    |SC0FM1|SC0FM0|SC0PM1|SC0PM0|SC0NPE|  -  |SC0BRKF|SC0BRKE| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0  |   0   |   0   |
    //------------------------------------------------------------------------- 
    //SC0FM1~0: ����֡ģʽѡ��   00=7bit data + 1bit stop  01=7bit data + 2bit stop
    //                           10=8bit data + 1bit stop  11=8bit data + 2bit stop
    //SC0PM1~0: ����λ˵��  (���ͣ�00="0"����  01="1"����  10=����鸽��  11=ż���鸽��)
    //                      (���գ�00="0"���  01="1"���  10=�������  11=ż������)
    //SC0NPE:   ��ż����ʹ��ѡ��   0=����ʹ��   1=���鲻ʹ��
    //SC0BRKF:  ��״̬���ռ��   0=��������   1=���տ�״̬
    //SC0BRKE:  ��״̬���Ϳ���   0=��������   1=���Ϳ�״̬
    //-------------------------------------------------------------------------
    SC0CTR2 = 0x98; //����Ϊ8bit data  1bit stop ������ �����ͺͽ��տ�״̬
//SC0CTR2 = 0xB0;    
    //SC0STR: SCI0״̬�Ĵ���
    //------------------------------------------------------------------------
    //reg_bit  |   7   |   6   |    5  |    4  |   3  |   2  |   1  |   0  |  
    // flag    |SC0TBSY|SC0RBSY|SC0TEMP|SC0REMP|SC0FEF|SC0PEK|SC0ORE|SC0ERE| 
    //at reset |   0   |   0   |    0  |    0  |   0   |  0  |   0  |   0  |
    //------------------------------------------------------------------------- 
    //SC0TBSY:����æ   0=��æ  1=æ
    //SC0RBSY:����æ   0=��æ  1=æ
    //SC0TEMP:���ͻ������ձ�־  0=��   1=��
    //SC0REMP:���ջ������ձ�־  0=��   1=��
    //SC0FEF:֡������   0=�޴���   1=�д��� 
    //SC0PEK:��ż������   0=�޴���   1=�д���     
    //SC0ORE:���������   0=�޴���   1=�д���     
    //SC0ERE:��ش�����   0=�޴���   1=�д���     
    //--------------------------------------------------------------------------
    
    //G3ICR����3�жϿ��ƼĴ���
    //-------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  LV2 |  LV1 |  LV0 |  -   |  -   |  IE1 |  IE0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //-------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | IR1  | IR1  |  -   |  -   | ID1  | ID0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //------------------------------------------------------------------- 
    G3ICR &= ~0x0100;    //��ֹtimer0�����ж�
    
    TM0MD &= ~0x80;      //��ֹTM0BC����
    
    //TM03PSC��Ԥ��Ƶ���ƼĴ���0
    //--------------------------------------------------------------------
    //reg_bit  |   7    |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |TMPSCNE0|  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |   0    |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------
    //TMPSCNE0:Ԥ��Ƶ����ʹ��ѡ��  0=��ֹ  1=ʹ��
    //--------------------------------------------------------------------   
    TM03PSC = 0x80;
    
    //TM03EXPSC���ⲿԤ��Ƶ���ƼĴ���0
    //--------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0    |  
    // flag    |TM3IN |TM2IN |TM1IN |TM0IN |  -   |  -   |  -   |EXPSCNE0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0    |
    //--------------------------------------------------------------------
    //TM3IN:T3����ʱ��Դѡ��  0=TM3IO pin input  1=IOCLK/128
    //TM2IN:T2����ʱ��Դѡ��  0=TM2IO pin input  1=IOCLK/128
    //TM1IN:T1����ʱ��Դѡ��  0=TM1IO pin input  1=IOCLK/128
    //TM0IN:T0����ʱ��Դѡ��  0=TM0IO pin input  1=IOCLK/128
    //EXPSCNE0:Ԥ��Ƶ(1/128)����ʹ��ѡ��  0=��ֹ  1=ʹ��
    //--------------------------------------------------------------------------
    TM03EXPSC = 0x00;   
    
    //TM0MD: T0ģʽ�Ĵ���
    //------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM0CNE|TM0LDE|  -   |  -   |  -   |TM0CK2|TM0CK1|TM0CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //------------------------------------------------------------------   	
    //TM0CNE: ��������תʹ��ѡ��  0=����ת  1=��ת
    //TM0LDE: ��������ʼ��   0=������ת   1=��ʼ��
    //TM0CK2~0: ����ʱ��Դѡ��  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=��ֹ����   
    //          100=��ֹ����   101=T1����    110=T2����     111=TM0IO pin����(������),IOCLK/128
    //--------------------------------------------------------------------------
    TM0MD = 0x01;   //IOCLK/8
    
    //TM0BR: T0�����Ĵ��� (TM3BR,TM2BR,TM1BR��TM0BR����)
    //--------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM0BR7|TM0BR6|TM0BR5|TM0BR4|TM0BR3|TM0BR2|TM0BR1|TM0BR0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------   	
    //TM0BR7~0: ������TM0BC����ʱ,���м����ĳ�ֵ��	TM0BC�����,��TM0BR���ص�TM0BC�С�
    //--------------------------------------------------------------------------
    TM0BR = 0x41;   //1200bps  (T0����,ʱ��ѡ���첽ģʽ��timer�����1/32,T0����ʱ��ΪIOCLK/8)
    //------------------------------------------------------
    TM0MD |= 0x40;       //��ʼ��TM0BC
    
    TM0MD &= ~0x40;      //���ʼ��TM0BC��־
    
    TM0MD |= 0x80;       //����TM3BC����
    //------------------------------------------------------
    bflg_SCI0_tx_delaytime = 0;
    gss_SCI0_tx_delaytimer = 0;
    
    bflg_SCI0_rx_delaytime = 1;
    gss_SCI0_rx_delaytimer = 5;
    //------------------------------------------------------
    bflg_SCI0_allow_tx = 0;
    bflg_SCI0_allow_rx = 0;
    //------------------------------------------------------
    bflg_SCI0_type_ok = 0;
    
    guc_SCI0_rx_len = 8; //�������ݳ��ȳ�ʼ��ΪĬ�ϳ���    
//    guc_SCI0_rx_len = 12; //�������ݳ��ȳ�ʼ��ΪĬ�ϳ���
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void SCI0comm_reset(void)     //sci0ͨѶ��λ�����������
{
    uint16_t i;
    
    //��ͨѶ�ж�
    *(unsigned char *)&G25ICR = 0x02;   //���sci0�����ж������־
    G25ICR &= ~0x0200;                  //��sci0�����ж�ʹ�ܱ�־
    
    *(unsigned char *)&G25ICR = 0x01;   //���sci0�����ж������־            
    G25ICR &= ~0x0100;                  //��sci0�����ж�ʹ�ܱ�־
    
    //��������У��λ
    if (bflg_SCI0_type_ok == 0)    //���δͨѶ����δȷ��
    {
    	  if (bflg_SCI0_parity_type == 0) //���֮ǰ����У��
    	  {
    	  	  bflg_SCI0_parity_type = 1;  //����У���־
    	  	  SC0CTR2 = 0xB0; //����Ϊ8bit data  1bit stop żУ�� �����ͺͽ��տ�״̬
    	  }
    	  else        //���֮ǰ��żУ��
    	  {
    	  	  bflg_SCI0_parity_type = 0;  //����У���־
    	  	  SC0CTR2 = 0x88; //����Ϊ8bit data  1bit stop ��У�� �����ͺͽ��տ�״̬
    	  }
    }
    
    //------���ͨѶ��־λ-------
    bflg_SCI0_rx_busy = 0;
    bflg_SCI0_rx_ok = 0;
    bflg_SCI0_allow_tx = 0;
    bflg_SCI0_tx_busy = 0;
    
//    bflg_SCI0_fan_tx = 0;
          
    for (i = 0; i < 20; i++)
    {
        guc_SCI0_rx_buffer[i] = 0;
    }
    //----------------------------------
    for (i = 0; i < 20; i++)
    {
        guc_SCI0_tx_buffer[i] = 0;
    }
    //----------------------------------
    bflg_SCI0_tx_delaytime = 0;
    gss_SCI0_tx_delaytimer = 0;
        
    bflg_SCI0_rx_delaytime = 1;
    gss_SCI0_rx_delaytimer = 5;
    
}
//------------------------------------------------------------------------------
void SCI0_check_delaytime(void)    //ͨѶ�����ʱ������1s��ʱ�����е���
{
	  if ((bflg_host_ctrl == 1) && (bflg_SCI0_type_ok == 0))  //�������λ�����ƣ���ͨѶ����δȷ��
	  {
	  	  gus_SCI0_check_delaytimer++;
	  	  if (gus_SCI0_check_delaytimer >= 5)  //5s
	  	  {
	  	  	  gus_SCI0_check_delaytimer = 0;
	  	  	  SCI0comm_reset();      //sci0ͨѶ��λ�����������
	  	  }
	  }
}
//------------------------------------------------------------------------------
void SCI0_send_init(void)     //SCI0ͨѶ���ͳ�ʼ������
{
	  uint8_t luc_tmp = 0;
	  uint8_t luc_tmp1 = 0;
	  uint16_t lus_tmp = 0;
	  word_type rs485_crc;
	  
	  if ((guc_SCI0_rx_buffer[0] == SCI0_RX_MIN_ORDER) && (guc_SCI0_rx_len == 4)) //����Ǻ���Э��
	  {
	  	  guc_SCI0_tx_buffer[0] = SCI0_RX_MIN_ORDER;
	  	  //--------------------------------------------------
	      guc_SCI0_tx_buffer[1] = guc_SCI0_rx_buffer[1]; //ȡ���ֽ�
	      guc_SCI0_tx_buffer[2] = guc_SCI0_rx_buffer[2];   //ȡ���ֽ�
	      //--------------------------------------------------
	      lus_tmp = gus_speed_real;
	      guc_SCI0_tx_buffer[3] = (uint8_t) (lus_tmp & 0xFF); //ȡ���ֽ�
	      guc_SCI0_tx_buffer[4] = (uint8_t) (lus_tmp >> 8);   //ȡ���ֽ�
	      //--------------------------------------------------
	      switch(guc_trip_code)
	      {
	      	  case 0:
	      	  	   guc_LED_trip_code = 0;
	      	  	   guc_SCI0_trip_code = 0;
	      	  	   break;
	      	  	   //--------------------------
	      	  case MOTOR_BLOCK_PROT_CODE:
	      	  	   guc_LED_trip_code = 1;
	      	  	   if (guc_motor_block_cnt >= 5)//����ʱ��ת3�κ�����λ�������ϴ���
	      	  	   {                            ///������ת��ʵ��Ƶ�ʴ��ڵ�����СƵ�ʣ�ʱ����ת1�κ�ͷ����ϴ���
	      	  	   	   guc_SCI0_trip_code = 0x01;
	      	  	   }	
	      	  	   break;	      	  	   
	      	  	   //--------------------------	      	  	   
	      	  case DCT_FAULT_CODE:
	      	  	   guc_LED_trip_code = 2;
	      	  	   guc_SCI0_trip_code = 0x02;
	      	  	         	  	   
	      	  	   break;
	      	       //--------------------------
	      	  case Iout_OC_PROT_CODE:
	      	  	   guc_LED_trip_code = 2;
	      	  	   if (guc_Iout_fault_cnt >= 5)
	      	  	   {
	      	  	   	  guc_SCI0_trip_code = 0x02;
	      	  	   }
	      	  	   break;	      	  	   	      	  	   
                 //--------------------------
	      	  case Iout_OL_PROT_CODE:
	      	  	   guc_LED_trip_code = 2;
	      	  	   if (guc_Iout_fault_cnt >= 5)
	      	  	   {
	      	  	   	  guc_SCI0_trip_code = 0x02;
	      	  	   }
	      	  	   break;	
	      	  	   //--------------------------
	      	  case Udc_OU_PROT_CODE:
	      	  	   guc_LED_trip_code = 3;
	      	  	   guc_SCI0_trip_code = 0x04;
	      	  	   break;	      	  	   
	      	  	   //--------------------------
	      	  case Udc_LU_PROT_CODE:
	      	  	   guc_LED_trip_code = 4;
	      	  	   guc_SCI0_trip_code = 0x08;
	      	  	   break;
                //--------------------------	 
	      	  case MOTOR_STALL_PROT_CODE:
	      	  	   guc_LED_trip_code = 5;
	      	  	   guc_SCI0_trip_code = 0x10;
	      	  	   break;                
                 //--------------------------
	      	  case Hall_FAULT_CODE:
	      	  	   guc_LED_trip_code = 5;
	      	  	   guc_SCI0_trip_code = 0x10;
	      	  	   break;	
	      	  	   //--------------------------
	      	  case ALM_PROT_CODE:
	      	  	   guc_LED_trip_code = 6;
	      	  	   
	      	  	   if (guc_ALM_fault_cnt >= 3)
	      	  	   {
	      	  	   	  guc_SCI0_trip_code = 0x20;
	      	  	   }	      	  	   	      	  	   
	      	  	   break;	      	  	   
	      	  	   //--------------------------
	      	  case Tm_OH_PROT_CODE:
	      	  	   guc_LED_trip_code = 6;
	      	  	   guc_SCI0_trip_code = 0x20;
	      	  	   break;	      	  	         	  	   
	      	  	   //--------------------------	      	  	   
	      	  case Tm_FAULT_CODE:
	      	  	   guc_LED_trip_code = 6;
	      	  	   guc_SCI0_trip_code = 0x20;
	      	  	   break;
	      	  	   //--------------------------
	      	  case MOTOR_OVERSPEEED_PROT_CODE:
	      	  	   guc_LED_trip_code = 7;
	      	  	   guc_SCI0_trip_code = 0x40;
	      	  	   break;
                 //--------------------------
	      	  case Hall_direct_FAULT_CODE:
	      	  	   guc_LED_trip_code = 7;
	      	  	   guc_SCI0_trip_code = 0x40;
	      	  	   break;
	      	  case EEPROM_ERR_CODE:
	      	  		 guc_LED_trip_code = 8;
	      	  		 guc_SCI0_trip_code = 0x80;
	      	  		   
	      	  	   break;
	      	  	   //--------------------------
	      	  default:
	      	  	   guc_LED_trip_code = 0;
	      	       guc_SCI0_trip_code = 0;    
	      	  	   break;	    
	      }
	      
	      guc_SCI0_tx_buffer[5] = guc_SCI0_trip_code;    //�õ����ϴ���
	      //--------------------------------------------------
	      luc_tmp = guc_SCI0_tx_buffer[0];
	      luc_tmp += guc_SCI0_tx_buffer[1];
	      luc_tmp += guc_SCI0_tx_buffer[2];
	      luc_tmp += guc_SCI0_tx_buffer[3];
	      luc_tmp += guc_SCI0_tx_buffer[4];
	      luc_tmp += guc_SCI0_tx_buffer[5];
	      luc_tmp = -luc_tmp;
	      
	      guc_SCI0_tx_buffer[6] = luc_tmp;
	  }
	  else if ((guc_SCI0_rx_buffer[0] == SCI0_RX_MIN_ORDER) && (guc_SCI0_rx_len == 8)) //�����ͨ��Э��
	  {
	  	  guc_SCI0_tx_buffer[0] = SCI0_TX_ORDER;
	      //--------------------------------------------------
	      lus_tmp = gus_speed_real;
	      guc_SCI0_tx_buffer[1] = (uint8_t) (lus_tmp & 0xFF); //ȡ���ֽ�
	      guc_SCI0_tx_buffer[2] = (uint8_t) (lus_tmp >> 8);   //ȡ���ֽ�
	      
	      if (bflg_actual_direction == 0) //�������ʱ��
	      {
	      	  guc_SCI0_tx_buffer[2] &= ~0x80;
	      }
	      else  //�����˳ʱ��
	      {
	  	      guc_SCI0_tx_buffer[2] |= 0x80;
	      }
	      //--------------------------------------------------
	      guc_SCI0_tx_buffer[3] = guc_SCI0_rx_buffer[3]; //ID����ͬ
	      //--------------------------------------------------
	      guc_SCI0_tx_buffer[4] = (uint8_t) gus_Iout_peak;    //���������ֵ
	      //--------------------------------------------------

	      if (bflg_trip_stop == 0)   //����޹��ϣ�����״̬����
	      {
	  	      luc_tmp = guc_state_code;
	      }
	      else if (guc_trip_code != EEPROM_CHANGE_CODE) //&& (guc_trip_code != CLEAR_TRIP_CODE))      //����й���
	      {
	  	      luc_tmp = 0;
	  	      if ((guc_trip_code == Iout_OC_PROT_CODE)
	  	      	   || (guc_trip_code == Iout_OL_PROT_CODE))
	  	      {
	  	      	  if (guc_Iout_fault_cnt >= 5)
	  	      	  {
					  	      luc_tmp = guc_trip_code;
					  	      if (bflg_trip_lock == 1)
					  	      {
					  	  	      luc_tmp |= 0x80;
					  	      }	  	      	  	  
	  	      	  }	
	  	      }	  	      
	  	      else if (guc_trip_code == ALM_PROT_CODE) 
	  	      {
	  	      	  if (guc_ALM_fault_cnt >= 3)
	  	      	  {
					  	      luc_tmp = guc_trip_code;
					  	      if (bflg_trip_lock == 1)
					  	      {
					  	  	      luc_tmp |= 0x80;
					  	      }	  	      	  	  
	  	      	  }	
	  	      }			  	       		  	      
	  	      else if (guc_trip_code == MOTOR_BLOCK_PROT_CODE) 
	  	      {
	  	      	  if (guc_motor_block_cnt >= 5)
	  	      	  {
					  	      luc_tmp = guc_trip_code;
					  	      if (bflg_trip_lock == 1)
					  	      {
					  	  	      luc_tmp |= 0x80;
					  	      }	  	      	  	  
	  	      	  }	
	  	      }		  	      
            else 
            {
					      luc_tmp = guc_trip_code;
					      if (bflg_trip_lock == 1)
					      {
					  	      luc_tmp |= 0x80;
					  	  }            	
            }
	      }
	      else
	      {
	      	  luc_tmp = 0;
	      }
	      guc_SCI0_tx_buffer[5] = luc_tmp;
	      //--------------------------------------------------
	      rs485_crc.uword = CRC16(guc_SCI0_tx_buffer, guc_SCI0_tx_len - 2);
	      guc_SCI0_tx_buffer[guc_SCI0_tx_len - 2] = rs485_crc.ubyte.low;
        guc_SCI0_tx_buffer[guc_SCI0_tx_len - 1] = rs485_crc.ubyte.high;
	  }
	  else if ((guc_SCI0_rx_buffer[0] == SCI0_Renesas_RX_ORDER) && (guc_SCI0_rx_len == 12)) //���������Э��
	  {
	  	  sci0_Renesas_tx_data_load();    //sci0ͨѶRenesas�������ݼ��س���
	  }
	  else  //�������Э��
	  {
	  	  guc_SCI0_tx_buffer[0] = 0xC0 + guc_comm_address;//  	  
	  	  guc_SCI0_tx_buffer[1] = guc_motor_ID;
	  	  //--------------------------------------------------
	  	  lus_tmp = gus_speed_real;
	      guc_SCI0_tx_buffer[2] = (uint8_t) (lus_tmp & 0xFF); //ȡ���ֽ�
	      guc_SCI0_tx_buffer[3] = (uint8_t) (lus_tmp >> 8);   //ȡ���ֽ�
	      
	      if (bflg_actual_direction == 0)  
	      {
	      	  guc_SCI0_tx_buffer[3] &= ~0x80;//ʵ����ת����Ϊ˳ʱ��  ��bit7=0
	      }
	      else  
	      {
	  	      guc_SCI0_tx_buffer[3] |= 0x80;//ʵ����ת����Ϊ��ʱ��  ��bit7=1
	      }
	      //--------------------------------------------------
	      
	  	  lus_tmp = gus_Iout_for_disp;//���������Чֵ
	      guc_SCI0_tx_buffer[4] = (uint8_t) (lus_tmp & 0xFF); //ȡ���ֽ�
	      guc_SCI0_tx_buffer[5] = (uint8_t) (lus_tmp >> 8);   //ȡ���ֽ�	      
	      
	      guc_SCI0_tx_buffer[6] = 0;//Ԥ��
	      guc_SCI0_tx_buffer[7] = 0;//Ԥ��
	      //--------------------------------------------------
	      guc_SCI0_tx_buffer[8] = (uint8_t) (gss_Tm + 0x40);  //ģ���¶�
	      //--------------------------------------------------
	  	  if (bflg_trip_stop == 0)   //����޹��ϣ�����״̬����
	      {
	  	      luc_tmp = guc_state_code;
	      }
	      else if (guc_trip_code != EEPROM_CHANGE_CODE) //&& (guc_trip_code != CLEAR_TRIP_CODE))
	      {
	  	      luc_tmp = 0;
	  	      if ((guc_trip_code == Iout_OC_PROT_CODE)||(guc_trip_code == Iout_OL_PROT_CODE))   
	  	      {
	  	      	  if (guc_Iout_fault_cnt >= 5)
	  	      	  {
					  	      luc_tmp = guc_trip_code;
					  	      if (bflg_trip_lock == 1)
					  	      {
					  	  	      luc_tmp |= 0x80;
					  	      }	  	      	  	  
	  	      	  }	
	  	      }		  	      
	  	      else if (guc_trip_code == ALM_PROT_CODE) 
	  	      {
	  	      	  if (guc_ALM_fault_cnt >= 3)
	  	      	  {
					  	      luc_tmp = guc_trip_code;
					  	      if (bflg_trip_lock == 1)
					  	      {
					  	  	      luc_tmp |= 0x80;
					  	      }	  	      	  	  
	  	      	  }	
	  	      }	  	      	  	      
	  	      else if (guc_trip_code == MOTOR_BLOCK_PROT_CODE)
	  	      {
	  	      	  if (guc_motor_block_cnt >= 5)
	  	      	  {
					  	      luc_tmp = guc_trip_code;
					  	      if (bflg_trip_lock == 1)
					  	      {
					  	  	      luc_tmp |= 0x80;
					  	      }	  	      	  	  
	  	      	  }	
	  	      }		  	      
            else 
            {
					      luc_tmp = guc_trip_code;
					      if (bflg_trip_lock == 1)
					      {
					  	      luc_tmp |= 0x80;
					  	  }            	
            }
	      }
	      else 
	      {
	      	  luc_tmp = 0;
	      }
	      guc_SCI0_tx_buffer[9] = luc_tmp;//״̬����
	      //--------------------------------------------------
	      /*luc_tmp = SOFT_VERSION;
	      luc_tmp <<= 2;//������λ
	      luc_tmp |= gus_Udc_for_disp >> 14;
	       
	      guc_SCI0_tx_buffer[10] = luc_tmp;//
	      guc_SCI0_tx_buffer[11] = (uint8_t) gus_Udc_for_disp;//ȡ���ֽ�*/
	      
	      //[10] = D7D6:�汾��λ; D5D4D3D2:�汾��λ
	      //[11] = [10]��D1D0��+[11]��ֱ����ѹ
	      luc_tmp = SOFT_VERSION / 100;
	      luc_tmp <<= 6;
	      luc_tmp1 = SOFT_VERSION % 100;
	      luc_tmp1 <<= 2;
	      luc_tmp |= luc_tmp1;
	      
	      luc_tmp1 = (uint8_t) (gus_Udc_for_disp >> 8);
	      luc_tmp1 &= 0x03;
	      luc_tmp |= luc_tmp1; 
	      
	      guc_SCI0_tx_buffer[10] = luc_tmp;//
	      guc_SCI0_tx_buffer[11] = (uint8_t) (gus_Udc_for_disp & 0xFF);//ֱ����ѹ
	      
	      guc_SCI0_tx_buffer[12] = 0;//Ԥ��
	      guc_SCI0_tx_buffer[13] = 0;//Ԥ�� 
	      
	      //--------------------------------------------------
	      rs485_crc.uword = CRC16(guc_SCI0_tx_buffer, guc_SCI0_tx_len - 2);
	      guc_SCI0_tx_buffer[guc_SCI0_tx_len - 2] = rs485_crc.ubyte.low;
        guc_SCI0_tx_buffer[guc_SCI0_tx_len - 1] = rs485_crc.ubyte.high;
	  }	  
    //------------------------------------------------------
    bflg_SCI0_tx_busy = 1;
    guc_SCI0_tx_point = 1;
    //------------------------------------------------------
    *(unsigned char *) & G25ICR = 0x01; //��������жϱ�־
    G25ICR &= ~0x0100;                  //������ж�ʹ�ܱ�־
    //----------------------------------
    SC0TB = guc_SCI0_tx_buffer[0];
    //----------------------------------
    *(unsigned char *) & G25ICR = 0x02; //��������ж������־
    G25ICR |= 0x0200;                   //�÷����ж�ʹ�ܱ�־
}
//------------------------------------------------------------------------------
void sci0_Renesas_tx_data_load(void)    //sci0ͨѶRenesas�������ݼ��س���
{
	  uint8_t i;
	  uint16_t lus_tmp;
	  uint32_t lul_tmp;
	  
	  uint8_t luc_tmp1;
	  uint8_t luc_tmp2;
	  uint8_t luc_tmp3;
	  
	  uint16_t lus_tmp1;
	  uint16_t lus_tmp2;
	  uint16_t lus_tmp3;
	  uint16_t lus_tmp4;
	  
	  guc_SCI0_tx_buffer[0] = 0x3C;
	  guc_SCI0_tx_buffer[1] = 0x30;
	  //----------------------------------
	  //�������Ƶ��
	  lus_tmp = gus_speed_real;//gus_freq_out;
//	  lus_tmp /= ram_para[num_MOTOR_p];   //�õ���еƵ��
	  lus_tmp /= 6; 
	  
	  lus_tmp1 = lus_tmp;
	  lus_tmp1 &= 0xF000;
	  lus_tmp1 >>= 12;
	  
	  lus_tmp2 = lus_tmp;
	  lus_tmp2 &= 0x0F00;
	  lus_tmp2 >>= 8;
	  
	  lus_tmp3 = lus_tmp;
	  lus_tmp3 &= 0x00F0;
	  lus_tmp3 >>= 4;
	  
	  lus_tmp4 = lus_tmp;
	  lus_tmp4 &= 0x000F;
	  
	  guc_SCI0_tx_buffer[2] = data_to_Ascii(lus_tmp1);//debug
	  guc_SCI0_tx_buffer[3] = data_to_Ascii(lus_tmp2);
	  guc_SCI0_tx_buffer[4] = data_to_Ascii(lus_tmp3);
	  guc_SCI0_tx_buffer[5] = data_to_Ascii(lus_tmp4);//guc_sci0_rx_buffer[5];//
	  //------------------------------------------------------
	  //ֱ����ѹ��ADֵ
	  lul_tmp = gus_Udc_for_disp;
	  lul_tmp *= 76;//debug
	  lul_tmp >>= 6;
	  lus_tmp = lul_tmp;
	  
	  lus_tmp1 = lus_tmp;
	  lus_tmp1 &= 0xF000;
	  lus_tmp1 >>= 12;
	  
	  lus_tmp2 = lus_tmp;
	  lus_tmp2 &= 0x0F00;
	  lus_tmp2 >>= 8;
	  
	  lus_tmp3 = lus_tmp;
	  lus_tmp3 &= 0x00F0;
	  lus_tmp3 >>= 4;
	  
	  lus_tmp4 = lus_tmp;
	  lus_tmp4 &= 0x000F;
	  
	  guc_SCI0_tx_buffer[6] = data_to_Ascii(lus_tmp1);
	  guc_SCI0_tx_buffer[7] = data_to_Ascii(lus_tmp2);
	  guc_SCI0_tx_buffer[10] = data_to_Ascii(lus_tmp3);
	  guc_SCI0_tx_buffer[11] = data_to_Ascii(lus_tmp4);
	  //------------------------------------------------------
	  //�������
	  lus_tmp = gus_Iout_for_disp;
	  lus_tmp >>= 1;  //��λ��0.2A
	  
	  luc_tmp1 = lus_tmp;
	  luc_tmp1 &= 0x00F0;
	  luc_tmp1 >>= 4;
	  
	  luc_tmp2 = lus_tmp;
	  luc_tmp2 &= 0x000F;
	  
	  guc_SCI0_tx_buffer[8] = data_to_Ascii(luc_tmp1);
	  guc_SCI0_tx_buffer[9] = data_to_Ascii(luc_tmp2);
	  //------------------------------------------------------
	  if (gss_Tm < 0)
	  {
	  	  lus_tmp = 0;
	  }
	  else
	  {
	  	  lus_tmp = gss_Tm;
	  }
	  
	  luc_tmp1 = lus_tmp;
	  luc_tmp1 &= 0x00F0;
	  luc_tmp1 >>= 4;
	  
	  luc_tmp2 = lus_tmp;
	  luc_tmp2 &= 0x000F;
	  
	  guc_SCI0_tx_buffer[12] = data_to_Ascii(luc_tmp1);
	  guc_SCI0_tx_buffer[13] = data_to_Ascii(luc_tmp2);
	  //------------------------------------------------------
	  //���ϴ���
	  switch (guc_trip_code)
	  {
	  	  case Normal_STATE_CODE:
	  	       guc_LED_trip_code = 0;
	  	       guc_SCI0_trip_code = 0;
	  	  	   break;
    	  case MOTOR_BLOCK_PROT_CODE:
    	  	   guc_LED_trip_code = 11;
    	  	   if (guc_motor_block_cnt >= 5)//����ʱ��ת3�κ�����λ�������ϴ���
    	  	   {                            ///������ת��ʵ��Ƶ�ʴ��ڵ�����СƵ�ʣ�ʱ����ת1�κ�ͷ����ϴ���
    	  	   	   guc_SCI0_trip_code = 0x38;
    	  	   }	
    	  	   break;	  	  	   	  	  	   
	  	  case DCT_FAULT_CODE: //      
	  	       guc_SCI0_trip_code = 0x20;
	  	       guc_LED_trip_code = 8;
	  	  	   break;	 
	  	  case Iout_OC_PROT_CODE:
    	  	   if (guc_Iout_fault_cnt >= 5)//����ʱ��ת3�κ�����λ�������ϴ���
    	  	   {                            ///������ת��ʵ��Ƶ�ʴ��ڵ�����СƵ�ʣ�ʱ����ת1�κ�ͷ����ϴ���
    	  	   	   guc_SCI0_trip_code = 0x08;
    	  	   }	  	  	
	  	       guc_LED_trip_code = 2;
	  	  	   break;	
	  	  case Iout_OL_PROT_CODE:
    	  	   if (guc_Iout_fault_cnt >= 5)//����ʱ��ת3�κ�����λ�������ϴ���
    	  	   {                            ///������ת��ʵ��Ƶ�ʴ��ڵ�����СƵ�ʣ�ʱ����ת1�κ�ͷ����ϴ���
    	  	   	   guc_SCI0_trip_code = 0x10;
    	  	   }	  	  	
	  	       guc_LED_trip_code = 4;
	  	  	   break;	
	  	  case Udc_OU_PROT_CODE:
	  	  	   guc_SCI0_trip_code = 0x18;
	  	       guc_LED_trip_code = 6;
	  	  	   break;	  	  	   
	  	  case Udc_LU_PROT_CODE:
	  	  	   guc_SCI0_trip_code = 0x14;
	  	       guc_LED_trip_code = 5;
	  	  	   break;
            //--------------------------	 
    	  case MOTOR_STALL_PROT_CODE:
    	  	   guc_LED_trip_code = 11;
    	       guc_SCI0_trip_code = 0x38;
    	  	   break;              
             //--------------------------
    	  case Hall_FAULT_CODE:
    	  	   guc_LED_trip_code = 11;
    	  	   guc_SCI0_trip_code = 0x38;
    	  	   break;	
    	  	   //--------------------------
	  	  case ALM_PROT_CODE:	  	  	
	      	   if (guc_ALM_fault_cnt >= 3)
	      	   {	  	  	
			  	  	   guc_SCI0_trip_code = 0x04;
			  	       guc_LED_trip_code = 1;
	  	       }
	  	  	   break;	  	  	     	  	     	  	    	  	   	  	  	   
	  	  case Tm_OH_PROT_CODE:
	  	  	   guc_SCI0_trip_code = 0x0C;
	  	       guc_LED_trip_code = 3;
	  	  	   break;
	  	  case Tm_FAULT_CODE:
	  	       guc_SCI0_trip_code = 0x40;
	  	       guc_LED_trip_code = 12;
	  	  	   break;	 
	  	  case MOTOR_OVERSPEEED_PROT_CODE:
	  	  	   guc_LED_trip_code = 11;
	  	  	   guc_SCI0_trip_code = 0x38;
	  	  	   break;	  	  	   
    	  case Hall_direct_FAULT_CODE:
	  	  	   guc_LED_trip_code = 11;
	  	  	   guc_SCI0_trip_code = 0x38;
    	  	   break;	  	  	   
	  	  case EEPROM_ERR_CODE:
	  	  	   guc_LED_trip_code = 0x2C;
	  	       guc_SCI0_trip_code = 11;
	  	  	   break;
	  	  case MOTOR_ID_ERR_CODE:
	  	  	   guc_LED_trip_code = 0x2C;
	  	       guc_SCI0_trip_code = 11;
	  	  	   break;
    	  default:
    	  	   guc_LED_trip_code = 0;
    	       guc_SCI0_trip_code = 0;    
    	  	   break;
	  }
	  luc_tmp1 = guc_SCI0_trip_code;
	  luc_tmp1 &= 0x00F0;
	  luc_tmp1 >>= 4;
	  
	  luc_tmp2 = guc_SCI0_trip_code;
	  luc_tmp2 &= 0x000F;
	  
	  guc_SCI0_tx_buffer[14] = data_to_Ascii(luc_tmp1);
	  guc_SCI0_tx_buffer[15] = data_to_Ascii(luc_tmp2);
	  //------------------------------------------------------
	  //״̬����
	  //guc_SCI0_tx_buffer[16] = 0x30;
	  //guc_SCI0_tx_buffer[17] = 0x30;
	  //��Ƶ���������
	  lus_tmp = 0;//gus_Iin_for_disp;//debug
	  lus_tmp >>= 1;  //��λ��0.2A
	  
	  luc_tmp1 = lus_tmp;
	  luc_tmp1 &= 0x00F0;
	  luc_tmp1 >>= 4;
	  
	  luc_tmp2 = lus_tmp;
	  luc_tmp2 &= 0x000F;
	  
	  guc_SCI0_tx_buffer[16] = data_to_Ascii(luc_tmp1);
	  guc_SCI0_tx_buffer[17] = data_to_Ascii(luc_tmp2);
	  //------------------------------------------------------
	  //����У���
	  luc_tmp3 = 0xFF;
	  for (i = 1; i < 18; i++)
	  {
	  	  luc_tmp3 ^= guc_SCI0_tx_buffer[i];
	  }
	  
	  luc_tmp1 = luc_tmp3;
	  luc_tmp1 &= 0x00F0;
	  luc_tmp1 >>= 4;
	  
	  luc_tmp2 = luc_tmp3;
	  luc_tmp2 &= 0x000F;
	  
	  guc_SCI0_tx_buffer[18] = data_to_Ascii(luc_tmp1);
	  guc_SCI0_tx_buffer[19] = data_to_Ascii(luc_tmp2);
	  //------------------------------------------------------
	  guc_SCI0_tx_len = 20;
}
//------------------------------------------------------------------------------
void SCI0_receive_int(void)   //SCI0ͨѶ���ճ�ʼ������
{
	  uint8_t i;
	  
	  for (i = 0; i < 20; i++)
    {
        guc_SCI0_rx_buffer[i] = 0;
    }
    //------------------------------------------------------
    *(unsigned char *) & G25ICR = 0x02; //��������ж������־
    G25ICR &= ~0x0200;                  //�巢���ж�ʹ�ܱ�־
    //----------------------------------
    *(unsigned char *) & G25ICR = 0x01; //��������жϱ�־
    G25ICR |= 0x0100;                   //�ý����ж�ʹ�ܱ�־
    //------------------------------------------------------
}
//------------------------------------------------------------------------------
void SCI0_rx_data_deal(void)  //ͨѶ�������ݴ������
{
    uint8_t luc_tmp = 0;
    uint8_t i,luc_tmp1,luc_tmp2;
    uint16_t lus_tmp = 0;
    word_type rs485_crc;
    lword_type ltmp1;    
    //------------------------------------------------------
    if (guc_SCI0_rx_buffer[0] == SCI0_RX_MIN_ORDER)    //�������Э��
    {
    	  if (guc_SCI0_rx_len == 8)  //�����ͨ��Э��
    	  {
    	  	  rs485_crc.uword = CRC16(guc_SCI0_rx_buffer, guc_SCI0_rx_len - 2);
    	      
    	      if ((rs485_crc.ubyte.low == guc_SCI0_rx_buffer[guc_SCI0_rx_len - 2])
             && (rs485_crc.ubyte.high == guc_SCI0_rx_buffer[guc_SCI0_rx_len - 1]))
            {
        	      guc_SCI0_type_cnt = 0;
        	      bflg_SCI0_type_ok = 1;
		        	  gus_SCI0_check_delaytimer = 0;
		        	  gss_SCI0_init_delaytimer = 0; 
		        	          	      
        	      bflg_SCI0_tx_delaytime = 1;
	  	  	      gss_SCI0_tx_delaytimer = 10;
	  	  	      
	  	  	      bflg_SCI0_rx_delaytime = 0;
	  	  	      gss_SCI0_rx_delaytimer = 0;
	  	  	      
	  	  	      bflg_SCI0_fault = 0;
	  	  	      gss_SCI0_fault_delaytimer = 0;
	  	  	      //------------------------------------------
	  	  	      //ȷ���������ݳ���
	  	  	      guc_SCI0_tx_len = 8;
	  	  	      
	  	  	      if (bflg_host_ctrl == 1)
	  	  	      {	
			  	  	      //ȷ��ID��
			  	  	      luc_tmp = guc_SCI0_rx_buffer[3];
			  	  	      if ((luc_tmp >= APP_ID_MIN)&&(luc_tmp <= APP_ID_MAX))       
			  	  	      {
			  	  	  	      guc_motor_ID_error_cnt = 0;
			  	  	  	      bflg_motor_ID_error = 0;
			  	  	  	      
			  	  	  	      guc_motor_ID = 0;   //��Э��Ĭ��Ϊ0
			  	  	  	      //--------------------------------------
			  	  	  	      //�õ���λ���趨ת��
			  	  	          lus_tmp = guc_SCI0_rx_buffer[2];
			  	  	          lus_tmp &= 0x7F;
			  	  	          lus_tmp <<= 8;
			  	  	          lus_tmp += guc_SCI0_rx_buffer[1];
			  	  	          
			  	  	          if (lus_tmp > ram_para[num_MOTOR_max_n])//motor_para[guc_motor_ID][MOTOR_max_n])
			  	  	          {
			  	  	          	  lus_tmp = ram_para[num_MOTOR_max_n];//motor_para[guc_motor_ID][MOTOR_max_n];
			  	  	          }
			  	  	          
			  	  	          gus_host_speed_set = lus_tmp;
			  	  	  	      //--------------------------------------
			  	  	  	      if (gus_host_speed_set == 0)
			  	  	          {
			  	  	  	          //bflg_askfor_run = 0;
			  	  	  	          //bflg_askfor_stop = 1;	  	  	  	         
			  	  	  	          bflg_host_running = 0;
			  	  	  	          
			  	  	  	          motor_var_init();    //ͣ��ʱ�������ر�������
			  	  	  	          
			  	  	  	          gus_host_setfreq = 0;
			  	  	  	          
			  	  	  	          guc_Iout_fault_cnt = 0;
			  	  	  	          guc_motor_block_cnt = 0;	 
			  	  	  	          guc_ALM_fault_cnt = 0; 	  	  	  	          
			  	  	  	          
			  	  	  	          if ((bflg_trip_stop == 1) && (bflg_trip_clear == 0))    //����ǹ��Ͻ׶Σ���������ϱ�־
			  	  	  	          {
			  	  	  	  	          gss_trip_clear_cnt++;
			  	  	  	  	          if (gss_trip_clear_cnt >= 10)
			  	  	  	  	          {
			  	  	  	  	  	          gss_trip_clear_cnt = 0;
			  	  	  	  	  	          bflg_trip_clear = 1;  	  	  	  	  	          
			  	  	  	  	          }
			  	  	  	          }
			  	  	          }
			  	  	          else
			  	  	          {
			  	  	  	          //bflg_askfor_run = 1;
			  	  	  	          //bflg_askfor_stop = 0;
			  	  	  	          bflg_host_running = 1;
			  	  	  	          
			  	  	  	          lus_tmp = gus_host_speed_set;
			  	  	  	          lus_tmp *= ram_para[num_MOTOR_p];//motor_para[guc_motor_ID][MOTOR_p];
			  	  	              lus_tmp *= 5;
			  	  	              lus_tmp /= 3;
			  	  	              
			  	  	              gus_host_setfreq = lus_tmp;
			  	  	              		  	  	               
			  	  	              if (gus_host_setfreq > gus_MAX_setfreq)
			  	  	              {
			  	  	              	  gus_host_setfreq = gus_MAX_setfreq;
			  	  	              }				  	  	              
			  	  	          }
			                  //--------------------------------------
			                  //��ת�����ж�
			                  luc_tmp = guc_SCI0_rx_buffer[2];
			  	  	          luc_tmp &= 0x80;
			  	  	          
			  	  	          if (bflg_running == 0)   //�����ǰδ����
			  	  	          {
			  	  	  	          if (luc_tmp == 0)    // 0
			  	  	              {
			  	  	  	              bflg_current_direction = 0;     //˳ʱ����ת��־
			  	  	              }
			  	  	              else
			  	  	              {
			  	  	  	              bflg_current_direction = 1;     //��ʱ����ת��־
			  	  	              }
			  	  	          }
			  	  	          else    //�����ǰ����
			  	  	          {
			  	  	  	          if (gus_host_speed_set != 0)
			  	  	  	          {
			  	  	  	  	          if ((luc_tmp == 0) && (bflg_current_direction == 1))
			  	  	  	  	          {
			  	  	  	  	  	          //bflg_askfor_run = 0;
			  	  	  	                  //bflg_askfor_stop = 1;
			  	  	  	  	  	          bflg_host_running = 0;
			  	  	  	  	  	          
			  	  	  	  	  	          gus_host_speed_set = 0;
			  	  	  	  	  	          gus_host_setfreq = 0;
			  	  	  	  	          }
			  	  	  	  	          else if ((luc_tmp != 0) && (bflg_current_direction == 0))
			  	  	  	              {
			  	  	  	  	  	          //bflg_askfor_run = 0;
			  	  	  	                  //bflg_askfor_stop = 1;
			  	  	  	  	  	          bflg_host_running = 0;
			  	  	  	  	  	          
			  	  	  	  	  	          gus_host_speed_set = 0;
			  	  	  	  	  	          gus_host_setfreq = 0;	
			  	  	  	              	
			  	  	  	              }
			  	  	  	          }
			  	  	          }
			  	  	      }
			  	  	      else //���ID�ų���
			  	  	      {
			  	  	  	      guc_motor_ID_error_cnt++;
			  	  	  	      if (guc_motor_ID_error_cnt >= 10)  //��10��
			  	  	  	      {
			  	  	  	  	      guc_motor_ID_error_cnt = 0;
			  	  	  	  	      
			  	  	  	  	      if (bflg_motor_ID_error == 0)
			  	  	  	          {
			  	  	  	  	          bflg_motor_ID_error = 1;   //��ID�Ŵ��־
			  	  	  	  	          trip_stop_deal(MOTOR_ID_ERR_CODE);
			  	  	  	          }
			  	  	  	      }			  	  	      	
			  	  	      }
			  	  	  }  
            }
            else   //���У��Ͳ���
            {
        	      if (bflg_SCI0_type_ok == 0)  //���δȷ��Э��
        	      {
        	  	      guc_SCI0_type_cnt++;
        	  	      if (guc_SCI0_type_cnt >= 5)  //�������10У�����
        	  	      {
        	  	      	  guc_SCI0_type_cnt = 0;
        	  	      	  bflg_SCI0_type_ok = 1;
        	  	      	  
        	  	      	  guc_SCI0_rx_len = 4;      //�����ֽڳ��ȸ�Ϊ4
        	  	      }
        	      }
            }
    	  }
    	  else if (guc_SCI0_rx_len == 4)  //����Ǻ���Э��
    	  {
    	  	  luc_tmp = guc_SCI0_rx_buffer[0];
    	  	  luc_tmp += guc_SCI0_rx_buffer[1];
    	  	  luc_tmp += guc_SCI0_rx_buffer[2];
    	  	  luc_tmp = -luc_tmp;
    	  	  
    	  	  if (luc_tmp == guc_SCI0_rx_buffer[3]) //���У�����ȷ
    	  	  {
    	  	  	  guc_SCI0_type_cnt = 0;
        	      bflg_SCI0_type_ok = 1;
		        	  gus_SCI0_check_delaytimer = 0;
		        	  gss_SCI0_init_delaytimer = 0; 
		        	          	      
        	      bflg_SCI0_tx_delaytime = 1;
	  	  	      gss_SCI0_tx_delaytimer = 10;
	  	  	      
	  	  	      bflg_SCI0_rx_delaytime = 0;
	  	  	      gss_SCI0_rx_delaytimer = 0;
	  	  	      
	  	  	      bflg_SCI0_fault = 0;
	  	  	      gss_SCI0_fault_delaytimer = 0;
	  	  	      
	  	  	      guc_motor_ID = 0;   //����Э��Ĭ��Ϊ0
	  	  	      //------------------------------------------
	  	  	      //ȷ���������ݳ���
	  	  	      guc_SCI0_tx_len = 7;
	  	  	      //------------------------------------------
	  	  	      //�õ���λ���趨ת��
	  	  	      lus_tmp = guc_SCI0_rx_buffer[2];
	  	  	      lus_tmp <<= 8;
	  	  	      lus_tmp += guc_SCI0_rx_buffer[1];
	  	  	      
	  	  	      if (lus_tmp > ram_para[num_MOTOR_max_n])//motor_para[guc_motor_ID][MOTOR_max_n])
	  	  	      {
	  	  	          lus_tmp = ram_para[num_MOTOR_max_n];//motor_para[guc_motor_ID][MOTOR_max_n];
	  	  	      }
	  	  	      
	  	  	      gus_host_speed_set = lus_tmp;
	  	  	      //------------------------------------------
	  	  	      if (gus_host_speed_set == 0)
	  	  	      {
	  	  	  	      //bflg_askfor_run = 0;
	  	  	  	      //bflg_askfor_stop = 1;
	  	  	  	      bflg_host_running = 0;
	  	  	  	      gus_host_setfreq = 0;
	  	  	  	      
	  	  	  	      guc_Iout_fault_cnt = 0;
	  	  	  	      guc_motor_block_cnt = 0;	 
	  	  	  	      guc_ALM_fault_cnt = 0; 	  	  	  	      
	  	  	  	      
	  	  	  	      if ((bflg_trip_stop == 1) && (bflg_trip_clear == 0))    //����ǹ��Ͻ׶Σ���������ϱ�־
	  	  	  	      {
	  	  	  	          gss_trip_clear_cnt++;
	  	  	  	          if (gss_trip_clear_cnt >= 10)
	  	  	  	          {
	  	  	  	  	  	      gss_trip_clear_cnt = 0;
	  	  	  	  	  	      bflg_trip_clear = 1;  
	  	  	  	  	      }
	  	  	  	      }
	  	  	      }
	  	  	      else
	  	  	      {
	  	  	  	      //bflg_askfor_run = 1;
	  	  	  	      //bflg_askfor_stop = 0;
	  	  	  	      bflg_host_running = 1;
	  	  	  	      
	  	  	  	      lus_tmp = gus_host_speed_set;
	  	  	  	      lus_tmp *= ram_para[num_MOTOR_p];//motor_para[guc_motor_ID][MOTOR_p];
	  	  	          lus_tmp *= 5;
	  	  	          lus_tmp /= 3;
	  	  	          gus_host_setfreq = lus_tmp;
			  	  	               
			  	  	      if (gus_host_setfreq > gus_MAX_setfreq)
			  	  	      {
			  	           	  gus_host_setfreq = gus_MAX_setfreq;
			  	  	      }		  	  	            	  	          
	  	  	      }
	  	  	      //------------------------------------------
    	  	  }
    	  }
    }
	  else if (guc_SCI0_rx_buffer[0] == SCI0_Renesas_RX_ORDER)
	  {
	  	  luc_tmp = 0xFF;
	  	  for (i = 1; i < 10; i++)
	  	  {
	  	  	  luc_tmp ^= guc_SCI0_rx_buffer[i];
	  	  }
	  	  
	  	  luc_tmp1 = Ascii_to_data(guc_SCI0_rx_buffer[10]);
	  	  luc_tmp2 = Ascii_to_data(guc_SCI0_rx_buffer[11]);
	  	  luc_tmp1 <<= 4;
	  	  luc_tmp1 += luc_tmp2;
	  	  
	  	  
	  	  if (luc_tmp == luc_tmp1)    //���У������
	  	  {
	  	  	  if (bflg_SCI0_type_ok == 0)  //���ͨѶ������δȷ��
	  	  	  {
	  	  	  	  bflg_SCI0_type_ok = 1;
	  	  	  	  guc_SCI0_type_cnt = 0;
	  	  	  	  bflg_SCI0_parity_type = 1;    //����У���־
	  	          SC0CTR2 = 0xB0;               //����Ϊ8bit data  1bit stop żУ�� �����ͺͽ��տ�״̬
	  	  	  }

        	  gus_SCI0_check_delaytimer = 0;
        	  gss_SCI0_init_delaytimer = 0;
        	  
    	      bflg_SCI0_tx_delaytime = 1;
	  	      gss_SCI0_tx_delaytimer = 10;
	  	      
	  	      bflg_SCI0_rx_delaytime = 0;
	  	      gss_SCI0_rx_delaytimer = 0;
	  	      
	  	      bflg_SCI0_fault = 0;
	  	      gss_SCI0_fault_delaytimer = 0;
        	  //--------------------------------------------------------------
        	  Renesas_rx_data_deal();
	  	  }
        else   //���У��Ͳ���
        {
    	      if (bflg_SCI0_type_ok == 0)  //���δȷ��Э��
    	      {
    	  	      guc_SCI0_type_cnt++;
    	  	      if (guc_SCI0_type_cnt >= 5)  //�������10У�����
    	  	      {
    	  	      	  guc_SCI0_type_cnt = 0;
    	  	      	  bflg_SCI0_type_ok = 1;
			  	  	  	  bflg_SCI0_parity_type = 1;    //����У���־
			  	          SC0CTR2 = 0xB0;               //����Ϊ8bit data  1bit stop żУ�� �����ͺͽ��տ�״̬
    	  	      	  
    	  	      	  guc_SCI0_rx_len = 12;      //�����ֽڳ��ȸ�Ϊ12
    	  	      }
    	      }
        }	  	  
	  }    
    else//Э�飨new��
    {
    	  if ((guc_SCI0_rx_buffer[0] - 0x30) == guc_comm_address)//     //�����ַ��ȷ
    	  {
    	  	  rs485_crc.uword = CRC16(guc_SCI0_rx_buffer, guc_SCI0_rx_len - 2);
    	      
    	      if ((rs485_crc.ubyte.low == guc_SCI0_rx_buffer[guc_SCI0_rx_len - 2])
             && (rs485_crc.ubyte.high == guc_SCI0_rx_buffer[guc_SCI0_rx_len - 1]))
            {
            	  guc_SCI0_type_cnt = 0;
        	      bflg_SCI0_type_ok = 1;
		        	  gus_SCI0_check_delaytimer = 0;
		        	  gss_SCI0_init_delaytimer = 0; 
		        	             	  
            	  bflg_SCI0_tx_delaytime = 1;
	  	  	      gss_SCI0_tx_delaytimer = 10;
	  	  	      
	  	  	      bflg_SCI0_rx_delaytime = 0;
	  	  	      gss_SCI0_rx_delaytimer = 0;
	  	  	      
	  	  	      bflg_SCI0_fault = 0;
	  	  	      gss_SCI0_fault_delaytimer = 0;
	  	  	      //--------------------------------------------------------------
	  	  	      //ȷ���������ݳ���
	  	  	      guc_SCI0_tx_len = 16;    //10
	  	  	      
            	  if (bflg_host_ctrl == 1)
                {
                	  if (bflg_running == 0)
                	  {	
		                	  guc_motor_ID = guc_SCI0_rx_buffer[1];  //ȷ��ID��  guc_motor_ID
		                	  if ((guc_motor_ID >= APP_ID_MIN) && (guc_motor_ID <= APP_ID_MAX))
		                	  {
			  	  	      	      guc_motor_ID_error_cnt = 0;
			  	  	  	          bflg_motor_ID_error = 0;
			  	  	  	          
			  	  	  	          if (ram_para[num_App_ID] != guc_motor_ID)   //�������ID������յ���ID�Ų�һ�£�����д
                            {    
			  	  	  	              disable_irq();   //�����ж�
			  	  	  	          
								            	  //--------------------------------------------------
								            	  //����У���
								                //ram_para[num_check_sum] -= ram_para[num_App_ID];
								                //ram_para[num_check_sum] += guc_swuart_rx_AppID;
								                
								                ram_para[num_App_ID] = guc_motor_ID;    //����Ӧ��ID��д��RAM
								                writeword_to_eeprom(&(ram_para[num_App_ID]), num_App_ID, 1);  //����дEEPROM�ĳ���
								                //--------------------------------------------------
								                clear_watchdog();
								                //--------------------------------------------------
								                //��дEEP�еĹؼ�����
								                for (i = 0; i < APP_PARA_CNT; i++)
								                {
								                    //ram_para[num_check_sum] -= ram_para[App_index[i]];
								                    //ram_para[num_check_sum] += App_para[ram_para[num_App_ID]-APP_ID_OFFSET][i];
								                    
								                    ram_para[App_index[i]] = App_para[ram_para[num_App_ID]-APP_ID_OFFSET][i];
								                    writeword_to_eeprom(&(ram_para[App_index[i]]), App_index[i], 1);     //����дEEPROM�ĳ���
								                    clear_watchdog();
								                }
								                //--------------------------------------------------
								                //writeword_to_eeprom(&(ram_para[num_check_sum]), num_check_sum, 1);      //дУ���
								                //--------------------------------------------------
								                reset_dealprog();			  	  	  	          
											  	  }					  	  	      	 	  	  	      	              	  	
		                	  }
		                	  else 
		                	  {
					  	  	      	  guc_motor_ID_error_cnt++;
					  	  	  	      if (guc_motor_ID_error_cnt >= 10)  //��10��
					  	  	  	      {
					  	  	  	  	      guc_motor_ID_error_cnt = 0;
					  	  	  	  	      
					  	  	  	  	      if (bflg_motor_ID_error == 0)
					  	  	  	          {
					  	  	  	  	          bflg_motor_ID_error = 1;   //��ID�Ŵ��־
					  	  	  	  	          trip_stop_deal(MOTOR_ID_ERR_CODE);
					  	  	  	          }
					  	  	  	      }		 	  	                        	  	
		                	  } 
		                	  //------------------------------------------------------
	  	  	  	          lus_tmp = guc_SCI0_rx_buffer[5]*10;//��λ��ǰ�ǵ��趨
	  	  	  	          if ((lus_tmp >= EEPROM_InitData[num_PhaseAdvSlope][MIN_ROW])
	  	  	  	          	 &&(lus_tmp <= EEPROM_InitData[num_PhaseAdvSlope][MAX_ROW]))
	  	  	  	          {
	  	  	  	          	   if (ram_para[num_PhaseAdvSlope] != lus_tmp)//��λ��ǰ�ǵ��趨
	  	  	  	          	   {
	  	  	  	          	   	    //disable_irq();   //�����ж�
	  	  	  	          	   	    
	  	  	  	          	   	    ram_para[num_PhaseAdvSlope] = lus_tmp;
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
													        //------------------------------
													        if (gus_phase_adv_degree >= ram_para[num_phase_max_degree])
													        {
													        	  gus_phase_adv_degree = ram_para[num_phase_max_degree];
													        }												        	  	  	          	   	    
	  	  	  	          	   	    //writeword_to_eeprom(&(ram_para[num_PhaseAdvSlope]), num_PhaseAdvSlope, 1);  //����дEEPROM�ĳ���
	  	  	  	          	   	    //clear_watchdog();
	  	  	  	          	   	    
	  	  	  	          	   	    //reset_dealprog();	  	  	  	          	   	
	  	  	  	          	   }
	  	  	  	          }	                	  		                	  
                	  }
	  	  	  	      //--------------------------------------
	  	  	  	      //�õ���λ���趨ת��
	  	  	          lus_tmp = guc_SCI0_rx_buffer[3];
	  	  	          lus_tmp &= 0x7F;
	  	  	          lus_tmp <<= 8;
	  	  	          lus_tmp += guc_SCI0_rx_buffer[2];
	  	  	          
	  	  	          if (lus_tmp > ram_para[num_MOTOR_max_n])//motor_para[guc_motor_ID][MOTOR_max_n])
	  	  	          {
	  	  	      	     lus_tmp = ram_para[num_MOTOR_max_n];//motor_para[guc_motor_ID][MOTOR_max_n];
	  	  	          }
	  	  	          
	  	  	          gus_host_speed_set = lus_tmp;//�õ���λ��ת��ֵ
	  	  	          //--------------------------------------
	  	  	          if (guc_SCI0_rx_buffer[4]&0x02) //����ͣ���ж�
	  	  	          {
	  	  	              bflg_hostComm_shutdown = 1;
	  	  	          }
	  	  	          else 
	  	  	          {
	  	  	          	 bflg_hostComm_shutdown = 0;
	  	  	          }
	  	  	          //----------------------------------------
	  	  	          if ((bflg_hostComm_shutdown == 0)//�޽���ͣ��
	  	  	          	&& (guc_SCI0_rx_buffer[4]&0x01)//�п�������
	  	  	          	&& (gus_host_speed_set != 0)) //ת������
	  	  	          {
			  	  	  	          bflg_host_running = 1;
		                        
		                          /*   n=60*f/p            //��λ��HZ
		                               f*100=n*p*(100/60)   //��λ��0.01HZ
		                               f*100=n*p*(5/3)     */
		                          
			  	  	  	          lus_tmp *= ram_para[num_MOTOR_p];//motor_para[guc_motor_ID][MOTOR_p];
			  	  	              lus_tmp *= 5;
			  	  	              lus_tmp /= 3;
			  	  	              
			  	  	              gus_host_setfreq = lus_tmp;	 
			  	  	               
			  	  	              if (gus_host_setfreq > gus_MAX_setfreq)
			  	  	              {
			  	  	              	  gus_host_setfreq = gus_MAX_setfreq;
			  	  	              }		  	  	              	 	  	          
	  	  	          }
	  	  	          else
	  	  	          {
	  	  	  	          bflg_host_running = 0;
	  	  	  	          
	  	  	  	          motor_var_init();    //ͣ��ʱ�������ر�������
	  	  	  	          
	  	  	  	          gus_host_setfreq = 0;
	  	  	  	          guc_Iout_fault_cnt = 0;
	  	  	  	          guc_motor_block_cnt = 0;	 
	  	  	  	          guc_ALM_fault_cnt = 0; 	  
	  	  	          }		
	  	  	          //----------------------------------
	  	              if ((bflg_trip_stop == 1)
	  	              	&&(guc_SCI0_rx_buffer[4]&0x04) 
	  	              	&& (bflg_trip_clear == 0))    //����ǹ��Ͻ׶Σ���������ϱ�־
	  	              {
	  	  	              //gss_trip_clear_cnt++;
	  	  	              //if (gss_trip_clear_cnt >= 10)
	  	  	              //{
	  	  	  	              //gss_trip_clear_cnt = 0;
	  	  	  	              bflg_trip_clear = 1;//����������
	  	  	  	          //}  	  	  	  	  	          
	  	  	          }	  	  	          
	  	  	          
	  	  	          //------------------------------
	                  //��ת�����ж�
	                  luc_tmp = guc_SCI0_rx_buffer[3];
	  	  	          luc_tmp &= 0x80;	  	  	          
                	  if (bflg_running == 0) 
                	  {
	  	  	  	          if (luc_tmp == 0)
	  	  	              {
	  	  	  	              bflg_current_direction = 0;    //������ת�����־ 0=��ת��1=��ת
	  	  	              }                                  //bflg_actual_direction ʵ����ת���� 0=��ת��1=��ת            
	  	  	              else
	  	  	              {
	  	  	  	              bflg_current_direction = 1;     
	  	  	              }	                	  	
                	  }	
                	  else //����ing
                	  {                	  	
			  	  	  	          if (gus_host_speed_set != 0)
			  	  	  	          {
			  	  	  	  	          if ((luc_tmp == 0) && (bflg_current_direction == 1))
			  	  	  	  	          {
			  	  	  	  	  	          //bflg_askfor_run = 0;
			  	  	  	                  //bflg_askfor_stop = 1;
			  	  	  	  	  	          bflg_host_running = 0;
			  	  	  	  	  	          
			  	  	  	  	  	          gus_host_speed_set = 0;
			  	  	  	  	  	          gus_host_setfreq = 0;
			  	  	  	  	          }
			  	  	  	  	          else if ((luc_tmp != 0) && (bflg_current_direction == 0))
			  	  	  	              {
			  	  	  	  	  	          //bflg_askfor_run = 0;
			  	  	  	                  //bflg_askfor_stop = 1;
			  	  	  	  	  	          bflg_host_running = 0;
			  	  	  	  	  	          
			  	  	  	  	  	          gus_host_speed_set = 0;
			  	  	  	  	  	          gus_host_setfreq = 0;	
			  	  	  	              	
			  	  	  	              }
			  	  	  	          }               	  	
                	  }  	  	            	  	  	           	  	          
	  	  	      }    
	  	  	      //-----------------------------------------
	  	  	      //guc_SCI0_rx_buffer[6]//Ԥ��
	  	  	      //guc_SCI0_rx_buffer[7]//Ԥ��     
            }
            else   //���У��Ͳ���
            {
        	      if (bflg_SCI0_type_ok == 0)  //���δȷ��Э��
        	      {
        	  	      guc_SCI0_type_cnt++;
        	  	      if (guc_SCI0_type_cnt >= 5)  //�������10У�����
        	  	      {
        	  	      	  guc_SCI0_type_cnt = 0;
        	  	      	  bflg_SCI0_type_ok = 1;
        	  	      	  
        	  	      	  guc_SCI0_rx_len = 10;      //�����ֽڳ��ȸ�Ϊ10
        	  	      }
        	      }
            }            
    	  } 	
    } 
}
//------------------------------------------------------------------------------
void Renesas_rx_data_deal(void)    //RenesasЭ��������ݴ������
{
	  //uint8_t i;
	  
	  uint16_t lus_tmp1;
	  uint16_t lus_tmp2;
	  uint16_t lus_tmp3;
	  uint16_t lus_tmp4;
	  
	  uint16_t lus_tmp;
	  //--------------------------------------------
	  guc_motor_ID = 32;   //Ĭ��ID
    bflg_current_direction = 0; //Ĭ��˳ʱ����ת��־----peakԭ����1

    //ȷ���������ݳ���
    guc_SCI0_tx_len = 20;	
    //�õ���λ���趨ת��  
	  lus_tmp1 = Ascii_to_data(guc_SCI0_rx_buffer[2]);
	  lus_tmp1 <<= 12;
	  
	  lus_tmp2 = Ascii_to_data(guc_SCI0_rx_buffer[3]);
	  lus_tmp2 <<= 8;
	  
	  lus_tmp3 = Ascii_to_data(guc_SCI0_rx_buffer[4]);
	  lus_tmp3 <<= 4;
	  
	  lus_tmp4 = Ascii_to_data(guc_SCI0_rx_buffer[5]);
	  
	  lus_tmp = lus_tmp1;
	  lus_tmp += lus_tmp2;
	  lus_tmp += lus_tmp3;
	  lus_tmp += lus_tmp4;//ת��ָ��
	  
	  lus_tmp *= 6; //(Э�鵥λ��0.1rps)
	  
    if (lus_tmp > ram_para[num_MOTOR_max_n])//motor_para[guc_motor_ID][MOTOR_max_n])
    {
    	  lus_tmp = ram_para[num_MOTOR_max_n];//motor_para[guc_motor_ID][MOTOR_max_n];
    }
    
    gus_host_speed_set = lus_tmp;	
  
    //------------------------------------------
    if (gus_host_speed_set == 0)
    {
	      //bflg_askfor_run = 0;
	      //bflg_askfor_stop = 1;
	      bflg_host_running = 0;
	      
	      motor_var_init();    //ͣ��ʱ�������ر�������
	      
	      gus_host_setfreq = 0;
	      
	      guc_Iout_fault_cnt = 0;
	      guc_motor_block_cnt = 0;	 
	      guc_ALM_fault_cnt = 0; 	  	  	  	      
	      
	      if ((bflg_trip_stop == 1) && (bflg_trip_clear == 0))    //����ǹ��Ͻ׶Σ���������ϱ�־
	      {
	          gss_trip_clear_cnt++;
	          if (gss_trip_clear_cnt >= 10)
	          {
	  	  	      gss_trip_clear_cnt = 0;
	  	  	      bflg_trip_clear = 1;  
	  	      }
	      }
    }
    else
    {
	      //bflg_askfor_run = 1;
	      //bflg_askfor_stop = 0;
	      bflg_host_running = 1;
	      
	      lus_tmp = gus_host_speed_set;
	      lus_tmp *= ram_para[num_MOTOR_p];//motor_para[guc_motor_ID][MOTOR_p];
        lus_tmp *= 5;
        lus_tmp /= 3;
        gus_host_setfreq = lus_tmp;
	               
	      if (gus_host_setfreq > gus_MAX_setfreq)
	      {
         	  gus_host_setfreq = gus_MAX_setfreq;
	      }		  	  	            	  	          
    }
}
//------------------------------------------------------------------------------
void SCI0_tx_delaytime(void)  //SCI0ͨѶ������ʱ������1ms��ʱ�����е���
{
	  if (bflg_SCI0_tx_delaytime == 1)
	  {
	  	  gss_SCI0_tx_delaytimer--;
	  	  if (gss_SCI0_tx_delaytimer <= 0)
	  	  {
	  	  	  gss_SCI0_tx_delaytimer = 0;
	  	  	  bflg_SCI0_tx_delaytime = 0;
	  	  	  
	  	  	  bflg_SCI0_allow_tx = 1;
	  	  }
	  }
}
//------------------------------------------------------------------------------
void SCI0_rx_delaytime(void)  //SCI0ͨѶ������ʱ������1ms��ʱ�����е���
{
	  if (bflg_SCI0_rx_delaytime == 1)
	  {
	  	  gss_SCI0_rx_delaytimer--;
	  	  if (gss_SCI0_rx_delaytimer <= 0)
	  	  {
	  	  	  gss_SCI0_rx_delaytimer = 0;
	  	  	  bflg_SCI0_rx_delaytime = 0;
	  	  	  
	  	  	  bflg_SCI0_allow_rx = 1;
	  	  }
	  }
}
//------------------------------------------------------------------------------
void SCI0_rx_end_delaytime(void)   //SCI0ͨѶ���պ����ʱ������1ms��ʱ�����е���
{
    if (bflg_SCI0_rx_busy == 1)   //�����ǰ����æ
    {
        gss_SCI0_rx_end_delaytimer++;
        if (gss_SCI0_rx_end_delaytimer >= 15)
        {
            gss_SCI0_rx_end_delaytimer = 0;  //���������ʱ��ʱ��
            bflg_SCI0_rx_busy = 0;           //�����æ��־
            
            G25ICR &= ~0x0100;               //������ж�ʹ�ܱ�־
            
            bflg_SCI0_rx_delaytime = 1;
	  	  	  gss_SCI0_rx_delaytimer = 5;
	  	  	  
	  	  	  if (bflg_SCI0_type_ok == 0) //���δȷ��Э��
        	  {
        	  	  guc_SCI0_type_cnt++;
        	  	  if (guc_SCI0_type_cnt >= 5) //�������10У�����
        	  	  {
        	  	      guc_SCI0_type_cnt = 0;
        	  	      bflg_SCI0_type_ok = 1;
        	  	      
        	  	      guc_SCI0_rx_len = 4;     //�����ֽڳ��ȸ�Ϊ4
        	  	  }
        	  }
        }
    }
}
//------------------------------------------------------------------------------
void SCI0_fault_delaytime(void)    //SCI0ͨѶ������ʱ������1s��ʱ�����е���
{
	  if ((bflg_host_ctrl == 1)&&(bflg_SCI0_fault == 0))
	  {
	  	  gss_SCI0_fault_delaytimer++;
	  	  if (gss_SCI0_fault_delaytimer >= 30)//
	  	  {
	  	  	  gss_SCI0_fault_delaytimer = 0;
	  	  	  
	  	  	  bflg_SCI0_fault = 1;
	  	  	  trip_stop_deal(COMM_FAULT_CODE);
	  	  }
	  }
}
//------------------------------------------------------------------------------
void SCI0_tx_int(void)        //SCI0�����жϳ���
{
	  if (bflg_SCI0_tx_busy == 1)    //�������æ
	  {
	  	  if (guc_SCI0_tx_point < guc_SCI0_tx_len)       //�����û������
	  	  {
	  	  	  SC0TB = guc_SCI0_tx_buffer[guc_SCI0_tx_point++];     //��������
	  	  }
	  	  else
	  	  {
	  	  	  guc_SCI0_tx_point = 0;
	  	  	  bflg_SCI0_tx_busy = 0;
	  	  	  //----------------------------------------------
	  	  	  G25ICR &= ~0x0200;     //�巢���ж�ʹ�ܱ�־
	  	  	  //----------------------------------------------
	  	  	  bflg_SCI0_rx_delaytime = 1;
	  	  	  gss_SCI0_rx_delaytimer = 5;
	  	  }
	  }
}
//------------------------------------------------------------------------------
void SCI0_rx_int(void)        //SCI0�����жϳ���
{
	  uint8_t luc_SCI0_rx_byte;
	  
	  gss_SCI0_rx_end_delaytimer = 0;
	  //----------------------------------
	  luc_SCI0_rx_byte = SC0RB;
	  
	  if ((bflg_SCI0_rx_busy == 0) 
	  	&& (((luc_SCI0_rx_byte >= SCI0_RX_MIN_ORDER) && (luc_SCI0_rx_byte <= SCI0_RX_MAX_ORDER))
	  	   ||((luc_SCI0_rx_byte >= 0x30) && (luc_SCI0_rx_byte <= 0x33)))
	  	   || (luc_SCI0_rx_byte == SCI0_Renesas_RX_ORDER))   //������ǽ���æ������������
	  {
	  	  bflg_SCI0_rx_busy = 1;
	  	  guc_SCI0_rx_buffer[0] = luc_SCI0_rx_byte;
        guc_SCI0_rx_point = 1;
	  }
	  else if (bflg_SCI0_rx_busy == 1)
	  {
	  	  guc_SCI0_rx_buffer[guc_SCI0_rx_point++] = luc_SCI0_rx_byte;
	  	  
	  	  if (guc_SCI0_rx_point >= guc_SCI0_rx_len)
	  	  {
	  	  	  G25ICR &= ~0x0100;     //������ж�ʹ�ܱ�־
	  	  	  
	  	  	  guc_SCI0_rx_point = 0;
	  	  	  bflg_SCI0_rx_busy = 0;
	  	  	  bflg_SCI0_rx_ok = 1;
	  	  	  
	  	  	  bflg_SCI0_rx_delaytime = 1;
	  	  	  gss_SCI0_rx_delaytimer = 50;
	  	  }
	  }
}
//------------------------------------------------------------------------------
uint16_t CRC16(uint8_t *puchmsg, uint16_t usDataLen)  //����CRCУ��ĳ���
{
    uint16_t uchCRC;
    uint16_t uIndex_x,uIndex_y;
    
    uchCRC = 0xFFFF;  
    
    if (usDataLen > 0)
    {
        for (uIndex_x = 0; uIndex_x <= (usDataLen-1); uIndex_x++)
        {
            uchCRC = uchCRC ^ (uint16_t)(*puchmsg);
            puchmsg++;
            for (uIndex_y = 0; uIndex_y <= 7; uIndex_y++)
            {
                if ((uchCRC&0x0001) != 0)
                {
                    uchCRC = (uchCRC >> 1) ^ 0xA001;
                }
                else
                {
                    uchCRC = uchCRC >> 1;
                }
            }
        }
    }
    else
    {
        uchCRC = 0;
    }
        
    return uchCRC;
}
//------------------------------------------------------------------------------
uint8_t Ascii_to_data(uint8_t luc_Ascii_tmp)      //Ascii��ת��ֵ����
{
	  uint8_t luc_data_tmp;
	  
	  if ((luc_Ascii_tmp >= 0x30) && (luc_Ascii_tmp <= 0x39))
	  {
	  	  luc_data_tmp = luc_Ascii_tmp - 0x30;
	  }
	  else if ((luc_Ascii_tmp >= 0x41) && (luc_Ascii_tmp <= 0x46))
	  {
	  	  luc_data_tmp = luc_Ascii_tmp - 0x37;
	  }
	  
	  return luc_data_tmp;
}
//------------------------------------------------------------------------------
uint8_t data_to_Ascii(uint8_t luc_data_tmp)       //��ֵתAscii��
{
	  uint8_t luc_Ascii_tmp;
	  
	  if ((luc_data_tmp >= 0) && (luc_data_tmp <= 9))
	  {
	  	  luc_Ascii_tmp = luc_data_tmp + 0x30;
	  }
	  else if ((luc_data_tmp >= 10) && (luc_data_tmp <= 15))
	  {
	  	  luc_Ascii_tmp = luc_data_tmp + 0x37;
	  }
	  
	  return luc_Ascii_tmp;
}
//------------------------------------------------------------------------------
#endif
