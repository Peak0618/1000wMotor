#ifndef	_PANEL_COMM_C_
#define _PANEL_COMM_C_
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *    ��Ȩ˵����Copyright(C) 2007-2012 �ൺ�߿Ʊ�Ƶ�������޹�˾              *
// *    �ļ�����  panel_comm.c                                                 * 
// *    ����:     �ְ���                                                       *
// *    �ļ��汾: 1.00                                                         *
// *    ��������: 2009��9��15��                                                *
// *    ������    ���ļ���Ҫ���ڽ�����LCD������ͨѶ�Ĵ������                *
// *    �汾��Ϣ:                                                              *
// *        ����������ļ�:  p30f3010.gld, libpic30.a                          *
// *        ʹ�õĹ���:      MPLAB IDE -> 8.00                                 *
// *                         Compiler  -> 3.02                                 *
// *                         Assembler -> 2.00                                 *
// *                         Linker    -> 2.00                                 *
// *        ֧�ֵ�оƬ:      dsPIC30F3010                                      *
// *    �����б�:                                                              *
// *            1.                                                             * 
// *    ��ʷ�޸ļ�¼:                                                          *
// *            1. ���ߣ�                                                      *
// *               ���ڣ�                                                      *
// *               �汾��                                                      *
// *               ������                                                      *
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *****************************************************************************
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
void configure_SCI1(void);      //����SCI1����ͨѶ�ӿ�

void SCI1comm_reset(void);      //SCI1ͨѶ��λ�����������

void SCI1rx_on_dealtime(void);  //��ʱ���ռ�ʱ������1ms��ʱ�����е���

void SCI1_rx_int(void);         //SCI1�����ж�

//void SCI1_rx_err_int(void);    //SCI1���մ����ж�

void SCI1rx_end_delaytimer(void);    //ͨѶ���պ����ʱ���򣬼���1ms��ʱ������

void SCI1comm_abend_dealtime(void);  //����ͨѶ�쳣�������,��1S��ʱ�����е���

void SCI1rx_ctrl_abend_dealtime(void);   //����ͨѶ���տ�����Ϣ�쳣�������,��1S��ʱ�����е���

void SCI1rx_data_deal(void);     //SCI1�������ݴ������

void SCI1_handshake_info_deal(void);   //���յ�������Ϣ�������

void SCI1_wrcmd_info_deal(void);     //���յ�д������Ϣ�������

void SCI1_rdcmd_info_deal(void);     //���յĶ�������Ϣ�������

void SCI1_ctrl_info_deal(void);     //���յĿ���������Ϣ�������

void SCI1tx_on_dealtime(void);  //��ʱ���ͼ�ʱ������1ms��ʱ�����е���

void SCI1_tx_int(void);     //SCI1�����ж�

void SCI1_txend_int(void);  //SCI1�������ж�
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  ��������:  signed int read_eeprom(unsigned int eep_num)                  *
// *  ��������:  ���ݲ�����Ŵ�EEPROM�ж�ȡ�����ĳ���                          *
// *  ���뺯��:  //�����������õĺ����嵥                                      *
// *  �ȳ�����:  //���ñ������ĺ����嵥                                        *
// *             void read_para_eeptoram(void)                                 *
// *             void init_all_param(void)                                     *
// *  �������:  eep_num  //�������   eep_num����С��511                      *
// *  �������:  ��                                                            *
// *  ȫ�ֱ����� gsi_eeprom_para[x]                                            *
// *  ��������ֵ: lsi_eedata_val   //EEP����ֵ                                 *
// *  ����˵��:   ��                                                           *
// *                                                                           *
// *****************************************************************************
//-------------------------------------------------------------------------------	 
   uint16_t gus_SCI1rx_delay_timer;          //��ʱ���ռ�ʱ��
   uint16_t gus_SCI1tx_delay_timer;          //��ʱ���ͼ�ʱ��

   uint16_t gus_SCI1comm_abend_timer;        //ͨѶ�쳣��ʱ��
   uint16_t gus_SCI1rx_ctrl_abend_timer;     //���տ�����Ϣ�쳣��ʱ��
   uint16_t gus_SCI1rx_end_timer;
   
   uint16_t gus_SCI1tx_cnt;
   uint16_t gus_SCI1rx_cnt;   
   
   uint8_t guc_SCI1rx_byte;	
   //--------------------------------------------
   uint8_t guc_SCI1tx_buffer[17];
 
   uint8_t guc_SCI1rx_buffer[8];
   //--------------------------------------------
   uint16_t gus_lcd_setfreq;	
	 //-----------------------------------------------------------
	 uint8_t gus_tmppin;    //�������ţ�Ϊ�Ժ���չΪRS485ͨѶ����
	 
   #define SCI1_RS485SEL_PIN	gus_tmppin
	
	 #define SCI1_RS485SEL_IS_REC     0
	 #define SCI1_RS485SEL_IS_TANS	  1
	
//------------------------------------------------------------------------------
void SCI1_configure(void)   //����SCI1����ͨѶ�ӿ�1
{
    //G26ICR����26�жϿ��ƼĴ���
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
    //IE2~IE0 : �ж�ʹ�ܱ�־  (IE1=SCI1ͨѶ��/�������ж�  IE0=SCI1�������ж�)
    //IR2~IR0 : �ж������־
    //ID2~ID0 : �жϼ���־     
    //-------------------------------------------------------------------
    G26ICR = 0x5000;     //�жϲ�ʹ��,�ж����ȼ�Ϊ5
    
    //SC1RB: SCI1�������ݻ�����
    //SC1TB: SCI1�������ݻ�����
    
    //SIFCLK:����ʱ��ѡ��Ĵ���
    //--------------------------------------------------------------------------
    //reg_bit  |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |  
    // flag    |   -   |   -   |   -   |   -   |SC1CKS1|SC1CKS0|SC1CKS1|SC1CKS0| 
    //at reset |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |
    //--------------------------------------------------------------------------
    //SC1CKS1~0:SCI1ʱ��Դ   00=T0���� 01=T1���� 10=T2���� 11=T3����
    //SC1CKS1~0:SCI0ʱ��Դ   00=T0���� 01=T1���� 10=T2���� 11=T3����
    //--------------------------------------------------------------------------
    SIFCLK &= ~0x0C;
    SIFCLK |= 0x04;      //ѡ��T1������Ϊ�����ʼ�ʱ
    
    //SC1CTR3: SCI0ģʽ�Ĵ���3
    //------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5  |  4  |   3   |  2  |  1  |  0  |  
    // flag    |  SC1FDC1~0  |  -  |  -  |SC1PSCE|    SC1PSC2~0    | 
    //at reset |  0   |  0   |  0  |  0  |   0   |  0  |   0 |   0 |
    //------------------------------------------------------------------------- 
    //SC1FDC1~0: SBO0������ݴ��������ѡ��   00=�̶����"1"  10=�̶����"0"  x1=���һλ����
    //SC1PSCE: Ԥ��Ƶʹ��ѡ��  0=��ֹ  1=ʹ��
    //SC1PSC2~0:   ʱ��ѡ�� (ͬ��ģʽ 000=timer�����1/2   001=timer�����1/4  010=timer�����1/16)
    //                      (ͬ��ģʽ 011=timer�����1/64  100=IOCLK/2         101=IOCLK/4)
    //                      (ͬ��ģʽ 110,111=��ֹ����)
    //                      (�첽ģʽ 000=timer�����1/32   001=timer�����1/64 010=timer�����1/256)
    //                      (�첽ģʽ 011=timer�����1/1024 100=IOCLK/2         101=IOCLK/4)
    //                      (�첽ģʽ 110,111=��ֹ����)
    //-------------------------------------------------------------------------   
    SC1CTR3 = 0x08;      //ʱ��ѡ���첽ģʽ��timer�����1/32
    
    //SC1CTR0: SCI0ģʽ�Ĵ���0	
    //------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2    |  1    |  0    |  
    // flag    |SC1CE1|SC1SSC|  -   |SC1DIR|SC1STE|SC1LNG2|SC1LNG1|SC1LNG0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0    |  0    |  0    |
    //------------------------------------------------------------------------- 
    //SC1CE1: ������Ч����ѡ��  (���ͣ�0=�½���  1=������)(���գ�0=������  1=�½���)
    //SC1SSC: ͬ��ͨѶ��ģʽʱ�ڵ�·ʱ��Դѡ��  0=�������ͨѶʱ��ͬ��  1=�ڲ�����ͨѶʱ��ͬ�� 
    //SC1DIR: ��λ��������ѡ��  0=�Ӹ�λ�׷�   1=�ӵ�λ�׷�   
    //SC1STE: ��������ѡ��  0=�ⲿ   1=�ڲ�   
    //SC1LNG2~0: ͬ�����д���λ��   000=1bit  ...  111=8bit         
    //-------------------------------------------------------------------------
    SC1CTR0 = 0x18;      //�ӵ�λ�׷�,�ڲ���������
    
    //SC1CTR1: SCI0ģʽ�Ĵ���1
    //------------------------------------------------------------------------
    //reg_bit  |  7   |   6   |   5   |   4   |  3   |  2   |  1   |  0   |  
    // flag    |SC1IOM|SC1SBTS|SC1SBIS|SC1SBOS|SC1CKM|SC1MST|  -   |SC1CMD| 
    //at reset |  0   |   0   |   0   |   0   |  0   |  0   |  0   |  0   |
    //------------------------------------------------------------------------- 
    //SC1IOM: ����������������ѡ��  0=SBI0  1=SBO0
    //SC1SBTS: SBT0���Ź���ѡ��  0=IO�˿�  1=����ʱ��IO 
    //SC1SBIS: ���������������ѡ��  0=�̶�"1"����  1=������������
    //SC1SBOS: SBO0���Ź���ѡ��  0=IO�˿�  1=�����������
    //SC1CKM: ����ʱ�ӵ�1/16��Ƶѡ��  0=����Ƶ  1=��Ƶ
    //SC1MST:  ����ʱ��ģʽѡ��   0=ʱ�Ӵ�ģʽ   1=ʱ����ģʽ
    //SC1CMD:  ͬ��ͨѶ/ȫ˫���첽ͨѶѡ��  0=ͬ��ͨѶ   1=ȫ˫���첽ͨѶ      
    //-------------------------------------------------------------------------
    SC1CTR1 = 0x3D; //SBI0Ϊ��������,SBT0Ϊͨ��IO,SBO0Ϊ�������,1/16��Ƶ,ȫ˫���첽ͨѶ
    
    //SC1CTR2: SCI0ģʽ�Ĵ���2
    //------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2  |   1   |   0   |  
    // flag    |SC1FM1|SC1FM0|SC1PM1|SC1PM0|SC1NPE|  -  |SC1BRKF|SC1BRKE| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0  |   0   |   0   |
    //------------------------------------------------------------------------- 
    //SC1FM1~0: ����֡ģʽѡ��   00=7bit data + 1bit stop  01=7bit data + 2bit stop
    //                           10=8bit data + 1bit stop  11=8bit data + 2bit stop
    //SC1PM1~0: ����λ˵��  (���ͣ�00="0"����  01="1"����  10=����鸽��  11=ż���鸽��)
    //                      (���գ�00="0"���  01="1"���  10=�������  11=ż������)
    //SC1NPE:   ��ż����ʹ��ѡ��   0=����ʹ��   1=���鲻ʹ��
    //SC1BRKF:  ��״̬���ռ��   0=��������   1=���տ�״̬
    //SC1BRKE:  ��״̬���Ϳ���   0=��������   1=���Ϳ�״̬
    //-------------------------------------------------------------------------
    SC1CTR2 = 0xB0; //����Ϊ8bit data  1bit stop żУ�� �����ͺͽ��տ�״̬
    
    //SC1STR: SCI0״̬�Ĵ���
    //------------------------------------------------------------------------
    //reg_bit  |   7   |   6   |    5  |    4  |   3  |   2  |   1  |   0  |  
    // flag    |SC1TBSY|SC1RBSY|SC1TEMP|SC1REMP|SC1FEF|SC1PEK|SC1ORE|SC1ERE| 
    //at reset |   0   |   0   |    0  |    0  |   0   |  0  |   0  |   0  |
    //------------------------------------------------------------------------- 
    //SC1TBSY:����æ   0=��æ  1=æ
    //SC1RBSY:����æ   0=��æ  1=æ
    //SC1TEMP:���ͻ������ձ�־  0=��   1=��
    //SC1REMP:���ջ������ձ�־  0=��   1=��
    //SC1FEF:֡������   0=�޴���   1=�д��� 
    //SC1PEK:��ż������   0=�޴���   1=�д���     
    //SC1ORE:���������   0=�޴���   1=�д���     
    //SC1ERE:��ش�����   0=�޴���   1=�д���     
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
    G3ICR &= ~0x0200;    //��ֹtimer1�����ж�
    
    TM1MD &= ~0x80;      //��ֹTM1BC����
    
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
    
    //TM1MD: T1ģʽ�Ĵ���
    //------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM1CNE|TM1LDE|  -   |  -   |  -   |TM1CK2|TM1CK1|TM1CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //------------------------------------------------------------------   	
    //TM1CNE: ��������תʹ��ѡ��  0=����ת  1=��ת
    //TM1LDE: ��������ʼ��   0=������ת   1=��ʼ��
    //TM1CK2~0: ����ʱ��Դѡ��  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=��ֹ����   
    //          100=��ֹ����   101=T1����    110=T2����     111=TM1IO pin����(������),IOCLK/128
    //--------------------------------------------------------------------------
    TM1MD = 0x01;   //IOCLK/8
    
    //TM1BR: T0�����Ĵ��� (TM3BR,TM2BR,TM1BR��TM0BR����)
    //--------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM1BR7|TM1BR6|TM1BR5|TM1BR4|TM1BR3|TM1BR2|TM1BR1|TM1BR0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------   	
    //TM1BR7~0: ������TM1BC����ʱ,���м����ĳ�ֵ��	TM1BC�����,��TM1BR���ص�TM1BC�С�
    //--------------------------------------------------------------------------
    TM1BR = 0x0f;//   20M//4800bps  (T1����,ʱ��ѡ���첽ģʽ��timer�����1/32,T1����ʱ��ΪIOCLK/8)
    //------------------------------------------------------
    TM1MD |= 0x40;       //��ʼ��TM1BC
    
    TM1MD &= ~0x40;      //���ʼ��TM1BC��־
    
    TM1MD |= 0x80;       //����TM1BC����   
    
    //-----------------------------------------------------------------------
    SCI1comm_reset();    //uart1ͨѶ��λ�����������
}
//---------------------------------------------------------------------------
void SCI1comm_reset(void)    //SCI1ͨѶ��λ�����������
{
    uint16_t i;
    
    bflg_SCI1_rx_busy = 0;
    bflg_SCI1_rx_ok = 0;
    bflg_SCI1_allow_tx = 0;
    bflg_SCI1_tx_busy = 0;
   
    gus_SCI1tx_delay_timer=0;
    gus_SCI1tx_cnt=0;
    gus_SCI1rx_cnt=0;
    for(i=0;i<8;i++)
    {
       guc_SCI1rx_buffer[i]=0;
    }
    //------------
    for(i=0;i<17;i++)
    {
       guc_SCI1tx_buffer[i]=0;
    }
    //-------------
    gus_lcd_setfreq=0;     //�������趨Ƶ�ʼĴ���
    //------------- 
    bflg_SCI1_allow_rx=1;         //��������ձ�־
    gus_SCI1rx_delay_timer=2;     //��ʱ���ռ�ʱ������ֵ	  
}
//---------------------------------------------------------------------------
void SCI1rx_on_dealtime(void)  //��ʱ���ռ�ʱ������1ms��ʱ�����е���
{
    if(bflg_SCI1_allow_rx==1)
    {
        gus_SCI1rx_delay_timer--;
        if(gus_SCI1rx_delay_timer<=0)
        {
            bflg_SCI1_allow_rx=0;
            gus_SCI1rx_delay_timer=0;
            //--------------------------------------
            *(unsigned char *)&G26ICR = 0x02;  //���SCI1�����ж������־
            G26ICR &= ~0x0200;    //��SCI1�����ж�
            //----���������ʱ�������ж�------
            //guc_sci0rx_byte=SC1RB;  //�ٶ� ��ǰ��Ӱ��           
            *(unsigned char *)&G26ICR = 0x01;  //���SCI1�����ж������־            
            G26ICR |= 0x0100;    //��SCI1�����ж�ʹ�ܱ�־
        }
    }
}
//--------------------------------------------------------------------------
void SCI1_rx_int(void)     //SCI1�����ж�
{
	 	guc_SCI1rx_byte=SC1RB;    //�ӽ��ռĴ����ж�����
	 	//-------------------------
    gus_SCI1rx_end_timer=0;     //��������ʱ��
    
    if((bflg_SCI1_rx_busy==0)&&(guc_SCI1rx_byte==0x55))
    {
        gus_SCI1rx_cnt=0;
        bflg_SCI1_rx_busy=1;              //�ý���æ��־
        guc_SCI1rx_buffer[0]=guc_SCI1rx_byte;
        gus_SCI1rx_cnt++;
    }
    else if(bflg_SCI1_rx_busy==1)   //�����SCI1����æ��־�������
    {
        if(gus_SCI1rx_cnt<3)
        {
            guc_SCI1rx_buffer[gus_SCI1rx_cnt]=guc_SCI1rx_byte;
            gus_SCI1rx_cnt++; 
        }
        else if((gus_SCI1rx_cnt<(guc_SCI1rx_buffer[2]+3))&&(gus_SCI1rx_cnt<7))
        {
            guc_SCI1rx_buffer[gus_SCI1rx_cnt]=guc_SCI1rx_byte;
            gus_SCI1rx_cnt++;
        }
        else
        {
            guc_SCI1rx_buffer[gus_SCI1rx_cnt]=guc_SCI1rx_byte;
            gus_SCI1rx_cnt=0;
            bflg_SCI1_rx_busy=0;     //�����æ��־
            bflg_SCI1_rx_ok=1;       //�ý��ճɹ���־            
        }
    }     	 	  
}
//---------------------------------------------------------------------------
void SCI1rx_end_delaytimer(void)      //ͨѶ���պ����ʱ���򣬼���1ms��ʱ������
{
    if(bflg_SCI1_rx_busy==1)
    {
        gus_SCI1rx_end_timer++;
        if(gus_SCI1rx_end_timer>=5)
        {
            gus_SCI1rx_cnt=0;
            bflg_SCI1_rx_busy=0;     //�����æ��־
            bflg_SCI1_rx_ok=0;       //����ճɹ���־
            
            //--------------------------------
		        *(unsigned char *)&G26ICR = 0x02;     //���SCI1�����ж������־
            //-----------------------
            *(unsigned char *)&G26ICR = 0x01;     //���SCI1�����ж������־                
            G26ICR |= 0x0100;      //��SCI1�����ж�ʹ�ܱ�־
        }
    }	
}
//---------------------------------------------------------------------------
void SCI1comm_abend_dealtime(void)     //����ͨѶ�쳣�������,��1S��ʱ�����е���
{
     gus_SCI1comm_abend_timer++;
     if(gus_SCI1comm_abend_timer>30)
     {
        gus_SCI1comm_abend_timer = 0;
        SCI1comm_reset();               //SCI1ͨѶ��λ����
        //stop_key();                   //ͣ��������
        //bflg_SCI1comm_err_prot = 1;   //��SCI1ͨѶ���ϱ�����־
     }
}
//---------------------------------------------------------------------------
void SCI1rx_ctrl_abend_dealtime(void)     //����ͨѶ���տ�����Ϣ�쳣�������,��1S��ʱ�����е���
{
     gus_SCI1rx_ctrl_abend_timer++;
     if(gus_SCI1rx_ctrl_abend_timer>30)
     {
          gus_SCI1rx_ctrl_abend_timer=0;
          SCI1comm_reset();                //SCI1ͨѶ��λ����         
          //stop_key();                    //ͣ��������
          //bflg_SCI1comm_err_prot = 1;    //��uart1ͨѶ���ϱ�����־
     }
}
//---------------------------------------------------------------------------
/*
void SCI1comm_err_trip_deal(void)    //ͨѶ���ϴ������,����ѭ���е���
{
     if(bflg_SCI1comm_err_prot==1)
     {
         bflg_SCI1comm_err_prot = 0;     //��ͨѶ���ϱ�����־
         if(bflg_trip_stop==0)   //����Ѿ����˹���ͣ����־����ת
         {
             lamp_trip_code=Lamp_CommErr_TRIP_CODE;
             trip_code=CommErr_TRIP_CODE; 
             trip_stop_deal_op();    //���ù���ͣ���������
         }          
     }
}
*/
//---------------------------------------------------------------------------
void SCI1rx_data_deal(void)     //SCI1�������ݴ������,����ѭ���е���
{
    if(bflg_SCI1_rx_ok==1)    //����ǽ���ok,�����
    {        
        bflg_SCI1_rx_ok=0;
        
        if(guc_SCI1rx_buffer[0]==0x55)
        {
             uint8_t rx_byte_sum=0;
             uint8_t i;
             uint8_t rx_data_sum=0;
             uint8_t rx_data_len=0;
             
             rx_data_len=guc_SCI1rx_buffer[2]+3;
             
             for(i=0;i<rx_data_len;i++)
             {
                rx_byte_sum+=guc_SCI1rx_buffer[i];
             }
             rx_byte_sum=~rx_byte_sum;
             
             rx_data_sum=guc_SCI1rx_buffer[rx_data_len];
             
             if(rx_byte_sum==rx_data_sum)
             {
                gus_SCI1comm_abend_timer=0;  //��ͨѶ�쳣��ʱ��  //������ȷ����ͨѶ�쳣������
                //-------------------------------------                
                switch(guc_SCI1rx_buffer[1])
                {
                   case 0xA1:
                            SCI1_handshake_info_deal();   //���յ�������Ϣ�������
                            break;
                   case 0xB1:
                            SCI1_wrcmd_info_deal();       //���յ�д������Ϣ�������
                            break;
                   case 0xB2:
                            SCI1_rdcmd_info_deal();       //���յĶ�������Ϣ�������
                            break;
                   case 0xC1:
                            SCI1_ctrl_info_deal();        //���յĿ�����Ϣ�������
                            break;
                }
                //-------------------------------------
                bflg_SCI1_allow_tx=1;    //�������ͱ�־
                gus_SCI1tx_delay_timer=20;    //��ʱ20ms����                
                //-------------------------------------
             }
        }
    }
}
//------------------------------------------------------------
void SCI1_handshake_info_deal(void)   //���յ�������Ϣ�������
{ 
     uint8_t tx_byte_sum=0;
     uint8_t i;
     
     guc_SCI1tx_buffer[0]=0xAA;
     guc_SCI1tx_buffer[1]=0xA1;
     guc_SCI1tx_buffer[2]=0x02;
     guc_SCI1tx_buffer[3]=(MAX_PARA_NUMBER%256);
     guc_SCI1tx_buffer[4]=(MAX_PARA_NUMBER/256);
     //------------------------              
     for(i=0;i<5;i++)
     {
         tx_byte_sum+=guc_SCI1tx_buffer[i];
     }
     guc_SCI1tx_buffer[5]=~tx_byte_sum;       
}
//------------------------------------------------------------
void SCI1_wrcmd_info_deal(void)     //���յ�д������Ϣ�������
{
     uint8_t tx_byte_sum=0;
     uint8_t i;  
       
     word_type  para_addr;
     word_type  para_val;
     lword_type tmp;
     
     para_addr.ubyte.low  = guc_SCI1rx_buffer[3];
     para_addr.ubyte.high = guc_SCI1rx_buffer[4];
     para_val.ubyte.low   = guc_SCI1rx_buffer[5];
     para_val.ubyte.high  = guc_SCI1rx_buffer[6];
     
     if((para_addr.uword == 0xFFFF)&&(para_val.uword == 0xFFFF))  //�����޸���Ϻ󣬳�����и�λ
     {
         if((bflg_running==0)&&((bflg_eeprom_change==1)||(bflg_trip_lock==1)))    
         { //ֻ����ͣ��״̬�����ǲ��������־���ǹ���������־�����͸�λ��Ϣ��CPU�Ÿ�λ
             reset_dealprog();    //оƬ��λ�������
         }
     }
     else
     {
         if(para_addr.uword<MAX_PARA_NUMBER)
         {             
             //����32λ��� ��16λ�������  ��16λ�������
             tmp.lword = get_eeppara_limit(para_addr.uword);    //�õ����������޵ĳ���
             if(para_val.word<tmp.word.low)
             {
                  para_val.word=tmp.word.low;
             }
             else if(para_val.word>tmp.word.high)
             {
                  para_val.word=tmp.word.high;
             }
             //---------------------------------------
             if((bflg_running==0)&&(bflg_askfor_run==0))
             {
                  if((para_addr.word==num_pr_set_method)&&(para_val.word==2))   //���Ҫ��ָ����������������
                  {
     	   	            tmp.word.low = init_all_para();   //��ʼ��EEPROM����
                      //------------------
     	   	            bflg_trip_stop = 1;     //�ù���ͣ����־ 
     	   	            bflg_eeprom_change = 1;  //��EEPROM�����־
     	   	            if(tmp.word.low!=0)
     	   	            {
     	   	               bflg_eeprom_error = 1;     //��EEPROM�����־
     	   	               guc_trip_code = EEPROM_ERR_CODE;   //��EEPROM��
     	   	               guc_lamp_trip_code = LAMP_EEPROM_ERR_CODE;
     	   	            }
     	   	            else
     	   	            {
     	   	               bflg_eeprom_error = 0;     //��EEPROM�����־
     	   	               guc_trip_code = EEPROM_CHANGE_CODE;   //�������
     	   	               guc_lamp_trip_code = LAMP_EEPROM_CHANGE_CODE;     	   	             	  
     	   	            }
                  }
                  else if((para_addr.word==num_Trip_clear)&&(para_val.word==1))  //���Ҫ��������ϴ��룬�����
                  {
                      clear_trip_cause();   //�����ͣԭ��
                      
                      bflg_trip_stop = 1;     //�ù���ͣ����־ 
     	   	            bflg_eeprom_change = 1;  //��EEPROM�����־
     	   	            guc_trip_code = EEPROM_CHANGE_CODE;   //�������
     	   	            guc_lamp_trip_code = LAMP_EEPROM_CHANGE_CODE;
                  }
                  else if(ram_para[para_addr.word]!=para_val.word)
                  {
                      //-------------------------------------
                      //������У���
                      //ram_para[num_check_sum] -= ram_para[para_addr.word];
                      //ram_para[num_check_sum] += para_val.word;
                      //-------------------------------------
                      ram_para[para_addr.word]=para_val.word;      //������д��RAM��EEP������
                      //------------------------------------
                      tmp.ulword =  ram_para[para_addr.word];
                      //tmp.ulword *= EEPROM_InitData[para_addr.word].multiple;
                      ram_para[para_addr.word] = (uint16_t) tmp.ulword;  //������д��RAM��ʹ�ò�����
                      //------------------------------------
                      //tmp.uword.low = write_to_eeprom(&(para_val.word),para_addr.word,2,0);  
	                    tmp.word.low = writeword_to_eeprom(&ram_para[para_addr.word],para_addr.word,1);   //����дEEPROM�ĳ���
                      //-----------------------------
                      //��У���д��EEPROM
                      //tmp.word.high = writeword_to_eeprom(&ram_para[num_check_sum],num_check_sum,1);   //дУ���
                      //-----------------------------
	                    bflg_trip_stop = 1;     //�ù���ͣ����־ 
     	   	            bflg_eeprom_change = 1;  //��EEPROM�����־
	                    if((tmp.uword.low!=0)||(tmp.word.high!=0))
     	   	            {
     	   	               bflg_eeprom_error = 1;     //��EEPROM�����־
     	   	               guc_trip_code = EEPROM_ERR_CODE;   //��EEPROM��
     	   	               guc_lamp_trip_code = LAMP_EEPROM_ERR_CODE;
     	   	            }
     	   	            else
     	   	            {
     	   	               bflg_eeprom_error = 0;     //��EEPROM�����־
     	   	               guc_trip_code = EEPROM_CHANGE_CODE;   //�������
     	   	               guc_lamp_trip_code = LAMP_EEPROM_CHANGE_CODE;
     	   	            }
     	   	            //---------------------              
                  }
             }
             //----------------------------------------
             para_val.word=ram_para[para_addr.word];
             //----------------------------------------           
         } 
     }
     //------------------------------------
     guc_SCI1tx_buffer[0]=0xAA;
     guc_SCI1tx_buffer[1]=0xB1;
     guc_SCI1tx_buffer[2]=0x04;
     guc_SCI1tx_buffer[3]=para_addr.byte.low;
     guc_SCI1tx_buffer[4]=para_addr.byte.high;
     guc_SCI1tx_buffer[5]=para_val.byte.low;
     guc_SCI1tx_buffer[6]=para_val.byte.high;

     for(i=0;i<7;i++)
     {
        tx_byte_sum+=guc_SCI1tx_buffer[i];
     }
     
     guc_SCI1tx_buffer[7]=~tx_byte_sum;
}
//------------------------------------------------------------
void SCI1_rdcmd_info_deal(void)    //���յĶ�������Ϣ�������
{
     uint8_t tx_byte_sum=0;
     uint8_t i;
     
     word_type  para_addr;
     word_type  para_val;
     
     lword_type tmp;
     
      para_addr.byte.high = guc_SCI1rx_buffer[4];
      para_addr.byte.low  = guc_SCI1rx_buffer[3];
     
      if(para_addr.word<MAX_PARA_NUMBER)
      {     
         para_val.word=ram_para[para_addr.word];

         guc_SCI1tx_buffer[0]=0xAA;
         guc_SCI1tx_buffer[1]=0xB2;
         guc_SCI1tx_buffer[2]=0x0A;
     
         guc_SCI1tx_buffer[3]=para_addr.byte.low;
         guc_SCI1tx_buffer[4]=para_addr.byte.high;
     
         guc_SCI1tx_buffer[5]=para_val.byte.low;
         guc_SCI1tx_buffer[6]=para_val.byte.high;
      
         if (para_addr.word==num_ref_freq)//20150303 guan
         {   //��������ǻ�׼Ƶ�ʲ��������趨Ƶ����������No.006��No.007�����޶������������ԡ�
             tmp.word.high=ram_para[num_max_freq];
             tmp.word.low=ram_para[num_min_freq]; 	   
         }
         else 
         {
         	   tmp.lword = get_eeppara_limit(para_addr.uword);    //�õ����������޵ĳ��� 
         }
             
         guc_SCI1tx_buffer[7]=tmp.ubyte.HL;
         guc_SCI1tx_buffer[8]=tmp.ubyte.HH;               //��������

         guc_SCI1tx_buffer[9]=tmp.ubyte.LL;
         guc_SCI1tx_buffer[10]=tmp.ubyte.LH;              //��������
         
         guc_SCI1tx_buffer[11]=EEPROM_InitData[para_addr.word][KEYRATE_ROW];
          
         guc_SCI1tx_buffer[12]=EEPROM_InitData[para_addr.word][DISPRATE_ROW];

      }
      else
      {
         guc_SCI1tx_buffer[0]=0xAA;
         guc_SCI1tx_buffer[1]=0xB2;
         guc_SCI1tx_buffer[2]=0x0A;  
         guc_SCI1tx_buffer[3]=0xFF;
         guc_SCI1tx_buffer[4]=0xFF;
         guc_SCI1tx_buffer[5]=0xFF;
         guc_SCI1tx_buffer[6]=0xFF;
         guc_SCI1tx_buffer[7]=0xFF;
         guc_SCI1tx_buffer[8]=0xFF;
         guc_SCI1tx_buffer[9]=0xFF;
         guc_SCI1tx_buffer[10]=0xFF;
         guc_SCI1tx_buffer[11]=0xFF; 
         guc_SCI1tx_buffer[12]=0xFF;    
      }

     for(i=0;i<13;i++)
     {
        tx_byte_sum+=guc_SCI1tx_buffer[i];
     }
     guc_SCI1tx_buffer[13]=~tx_byte_sum;
}
//------------------------------------------------------------------------------
void SCI1_ctrl_info_deal(void)     //���յĿ���������Ϣ�������
{
     uint8_t tx_byte_sum=0;
     uint8_t i;
     word_type  para_val;

     gus_SCI1rx_ctrl_abend_timer=0;     //����տ�����Ϣ�쳣��ʱ��
          
     para_val.ubyte.high = guc_SCI1rx_buffer[4];
     para_val.ubyte.low  = guc_SCI1rx_buffer[3];
     
     gus_lcd_setfreq=para_val.uword;        //Ƶ�ʾ���Ϊ0.01Hz
		
		 if (gus_lcd_setfreq > gus_MAX_setfreq)
		 {
			    gus_lcd_setfreq = gus_MAX_setfreq;
		 }	     			  	  	                        
     //-------------------------------------------
     if((bflg_host_ctrl==0)&&(gus_lcd_setfreq==0))  //������λ������(��������)����Ƶ��ָ��Ϊ�㣬�����
     {
		    if(bflg_trip_stop==1)   //�������ת��Ϊ�����������ڹ���״̬�£������
        {
             gus_trip_release_cnt++;
             if(gus_trip_release_cnt>=10)
             {
                 gus_trip_release_cnt=0;          //������ͷż�����
                 ///bflg_askfor_release_tirp=0;      //��Ҫ������ͷű�־λ
                 bflg_trip_clear = 1;//debug
             }
        }
     }
     //------------------------------------
     guc_SCI1tx_buffer[0]=0xAA;
     guc_SCI1tx_buffer[1]=0xC1;
     guc_SCI1tx_buffer[2]=0x0D;
     //---------------------
     para_val.uword=gus_freq_real;      //0.1Hz
     guc_SCI1tx_buffer[3]=para_val.ubyte.low;
     guc_SCI1tx_buffer[4]=para_val.ubyte.high;
     //---------------------
     //debug
     //para_val.uword = gss_Tf;
     para_val.uword=gus_Uout_real;//gus_Uout_for_disp;  //1V
     guc_SCI1tx_buffer[5]=para_val.ubyte.low;
     guc_SCI1tx_buffer[6]=para_val.ubyte.high;
     //---------------------
     para_val.uword=gus_Udc_for_disp;   //1V
     guc_SCI1tx_buffer[7]=para_val.ubyte.low;
     guc_SCI1tx_buffer[8]=para_val.ubyte.high;
     //---------------------
     para_val.uword=gus_Iout_for_disp;
     //para_val.uword=gus_Iout;
     guc_SCI1tx_buffer[9]=para_val.ubyte.low;
     guc_SCI1tx_buffer[10]=para_val.ubyte.high;
     //---------------------
     //debug
     //para_val.uword=gss_delta_V;
     para_val.uword=gus_phase_adv_degree;//gus_Hall_speed;//gus_speed_real;//gus_Iin_for_disp;//debug 
     guc_SCI1tx_buffer[11]=para_val.ubyte.low;
     guc_SCI1tx_buffer[12]=para_val.ubyte.high;
     //----------------------
     if (ram_para[num_Tm_enbale] == 1)
     {
     	   para_val.uword = gss_Tm + 0x40;
     }
     //else if (ram_para[num_THM_enbale] == 1)  //���ɢ�����¶���Ч
     //{
     //	   para_val.uword = gss_Tm + 0x40;
     //}
     guc_SCI1tx_buffer[13]=para_val.ubyte.low;
     //----------------------
     if(bflg_trip_stop==1)    //�������ͣ״̬�������
     {   
        guc_SCI1tx_buffer[14] = guc_trip_code;
        if(bflg_trip_lock==1) guc_SCI1tx_buffer[12]|=0x80;
     }
     else   //���������ͣ״̬����ȥ����״̬����
     {
        guc_SCI1tx_buffer[14]= guc_state_code;
     }  
     //----------------------
     guc_SCI1tx_buffer[15] = (uint8_t)gss_stop_space_delaytimer;        
     //---------------------
     for(i=0;i<16;i++)
     {
        tx_byte_sum+=guc_SCI1tx_buffer[i];
     }
     guc_SCI1tx_buffer[16]=~tx_byte_sum;
}
//------------------------------------------------------------------------------
void SCI1tx_on_dealtime(void)  //��ʱ���ͼ�ʱ������1ms��ʱ�����е���
{
     if(bflg_SCI1_rx_busy==0)
     {
        if(bflg_SCI1_allow_tx==1)
        {
             gus_SCI1tx_delay_timer--;
             if(gus_SCI1tx_delay_timer <= 0)
             {
                bflg_SCI1_allow_tx = 0;
                gus_SCI1tx_delay_timer = 0;
                bflg_SCI1_tx_busy = 1;
                gus_SCI1tx_cnt = 0;
                //----����ʱ�رս���------
                *(unsigned char *)&G26ICR = 0x01; //���SCI1�����ж������־  
                G26ICR &= ~0x0100;                //��SCI1�����ж�ʹ�ܱ�־
		            //------------------------             
                *(unsigned char *)&G26ICR = 0x02; //���SCI1�����ж������־
                G26ICR |= 0x0200;                 //��SCI1�����ж�ʹ�ܱ�־
                gus_SCI1tx_cnt++;
                SC1TB = guc_SCI1tx_buffer[0];     //��������                                           
                //-----------------------
             }
        }
     }
}
//---------------------------------------------------------------------------
void SCI1_tx_int(void)     //SCI1�����ж�
{
    gus_SCI1comm_abend_timer = 0;  //��ͨѶ�쳣��ʱ��
    
    if(bflg_SCI1_tx_busy==1)
    {
        if(gus_SCI1tx_cnt<(guc_SCI1tx_buffer[2]+4))  //���û������ȥ����һ��
        {
            SC1TB = guc_SCI1tx_buffer[gus_SCI1tx_cnt];      //��������
            gus_SCI1tx_cnt++;    //�����ֽڼ���
        }
        else
        {
            bflg_SCI1_tx_busy=0;     //�������æ��־
            gus_SCI1tx_cnt=0;        //�����ֽڼ����� 
            //--------------------------------------
            //*(unsigned char *)&G26ICR = 0x02;   //���SCI1�����ж������־
            G26ICR &= ~0x0200;          //��SCI1�����ж�ʹ�ܱ�־
            
            bflg_SCI1_allow_rx = 1;     //��������ձ�־
            gus_SCI1rx_delay_timer = 2; //��ʱ���ռ�ʱ������ֵ
            //--------------------------------------
        }
    }
}
//-------------------------------------------------------------------------------

#endif 						//end

