#ifndef _SWI2C_EEPROM_DRIVER_C_
#define _SWI2C_EEPROM_DRIVER_C_
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *    ��Ȩ˵����Copyright(C) 2007-2012 �ൺ�߿Ʊ�Ƶ�������޹�˾              *
// *    �ļ�����  SWI2C_EEPROM_Drive.c                                         * 
// *    ����:     �ְ���                                                       *
// *    �ļ��汾: 1.00                                                         *
// *    ��������: 2009��4��14��                                                *
// *    ������    ���ģ��I2C��дEEPROM����������                              *
// *    �汾��Ϣ:                                                              *
// *        ����������ļ�:  SWI2C_EEPROM_Drive.h                              *
// *        ʹ�õĹ���:                                                        *
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *        ֧�ֵ�оƬ:                                                        *
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
#include "comm_variable.h"
#include "SWI2C_EEPROM_Drive.h"    
//------------------------------------------------------------------------------
void delay_5us(uint16_t us_cnt);    //5us��ʱ����

void I2C_start(void);     //I2C������������

void I2C_stop(void);      //I2C����ֹͣ����

uint8_t I2C_RecACK(void);   //I2C ���Ӧ����Ӧ����  ����ֵ 1����Ӧ��  0����Ӧ��

void I2C_ACK(void);      //I2C���߲���Ӧ���źŲ���

void I2C_NoACK(void);    //I2C���߲�����Ӧ���źŲ���

void I2C_send_byte(uint8_t Byte_Val);   //I2C���߷���һ���ֽڵĲ���

uint8_t I2C_receive_byte(void);   //I2C���߽���һ���ֽڵĲ���
//------------------------------------------------------------------------------
//����һ������EEPROM�ͺŵ�ö������
enum  eepromtype  {M24C01,M24C02,M24C04,M24C08,M24C16,M24C32,M24C64,M24C128,M24C256,M24C512};
  //------------------------------------------------------------------------------
//  |bit7 |bit6 |bit5 |bit4 |bit3 |bit2 |bit1 |bit0 |
//  |1    |0    |1    |0    |A2   |A1   |A0   |R/W  | AT24C01/AT24C02������   (Ƭ�ڵ�ַ 1Byte)
//  |1    |0    |1    |0    |A2   |A1   |a8   |R/W  | AT24C04������           (Ƭ�ڵ�ַ 1Byte)
//  |1    |0    |1    |0    |A2   |a9   |a8   |R/W  | AT24C08������           (Ƭ�ڵ�ַ 1Byte)
//  |1    |0    |1    |0    |a10  |a9   |a8   |R/W  | AT24C016������          (Ƭ�ڵ�ַ 1Byte)

//  |1    |0    |1    |0    |A2   |A1   |A0   |R/W  | AT24C32/AT24C64������   (Ƭ�ڵ�ַ 2Byte)
//  |1    |0    |1    |0    |0    |A1   |A0   |R/W  | AT24C128/AT24C256������ (Ƭ�ڵ�ַ 2Byte)
//  |1    |0    |1    |0    |A2   |A1   |A0   |R/W  | AT24C512������          (Ƭ�ڵ�ַ 2Byte)
//  Ax:Ϊ�����ܽ�λ   ax: Ϊ������ַ�ĸ�λ
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
#ifdef I2C_DRIVER_IS_EEP24C01

      #define MAX_SIZE      128
      #define PAGE_SIZE     8
      enum  eepromtype EepromType=M24C01;
      
#endif
//-----------------------------------------
#ifdef I2C_DRIVER_IS_EEP24C02

      #define MAX_SIZE      256
      #define PAGE_SIZE     8
      enum  eepromtype EepromType=M24C02;
      
#endif
//-----------------------------------------
#ifdef I2C_DRIVER_IS_EEP24C04

      #define MAX_SIZE      512
      #define PAGE_SIZE     16
      enum  eepromtype EepromType=M24C04;
      
#endif
//-----------------------------------------
#ifdef I2C_DRIVER_IS_EEP24C08

      #define MAX_SIZE      1024
      #define PAGE_SIZE     16
      enum  eepromtype EepromType=M24C08;
      
#endif
//-----------------------------------------
#ifdef I2C_DRIVER_IS_EEP24C16

      #define MAX_SIZE      2048
      #define PAGE_SIZE     16
      enum  eepromtype EepromType=M24C16;
      
#endif
//-----------------------------------------
#ifdef I2C_DRIVER_IS_EEP24C32

      #define MAX_SIZE      4096
      #define PAGE_SIZE     32
      enum  eepromtype EepromType=M24C32;
      
#endif
//-----------------------------------------
#ifdef I2C_DRIVER_IS_EEP24C64

      #define MAX_SIZE      8192
      #define PAGE_SIZE     32
      enum  eepromtype EepromType=M24C64;
      
#endif
//-----------------------------------------
#ifdef I2C_DRIVER_IS_EEP24C128

      #define MAX_SIZE      16384
      #define PAGE_SIZE     64
      enum  eepromtype EepromType=M24C128;
      
#endif
//-----------------------------------------
#ifdef I2C_DRIVER_IS_EEP24C256

      #define MAX_SIZE      32768
      #define PAGE_SIZE     64
      enum  eepromtype EepromType=M24C256;
      
#endif
//-----------------------------------------
#ifdef I2C_DRIVER_IS_EEP24C512

      #define MAX_SIZE      65536
      #define PAGE_SIZE     128
      enum  eepromtype EepromType=M24C512;  
      
#endif
//-----------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
const uint8_t  bit_table[8]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};  
//#define     SET_BIT(varible,set_bit)    (varible|=bit_table[set_bit])
//#define     CLR_BIT(varible,set_bit)    (varible&=~bit_table[set_bit])
//#define     TEST_BIT(varible,set_bit)   (varible&bit_table[set_bit])
//------------------------------------------------------------------------------
    uint8_t guc_eep_op_eercode;        //�����쳣����

    //0�����������������
    //1����������쳣���� (��д���ݳ���)
    //2����������������
    
    uint8_t guc_eep_driver_addr;        //�豸��ַ

    uint16_t gus_eep_start_page;        //��ʼҳ
    uint16_t gus_eep_end_page;          //����ҳ
    uint16_t gus_eep_now_page;          //��ǰҳ
    uint16_t gus_eep_now_byteaddr;      //��ǰд��ҳ��ʼ�ֽڵĵ�ַ;
    
    uint16_t gus_eep_start_index;       //��ʼָ������
    uint16_t gus_eep_end_index;         //����ָ������
    uint16_t gus_eep_now_index;         //��ǰд����ָ��
  
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  ��������:  void delay_5us(uint16_t us_cnt)                               *
// *  ��������:  δ���ж�ʱ����ʱ5us�ĳ���                                     *
// *  ���뺯��:  //�����������õĺ����嵥                                      *
// *             ��                                                            *
// *  �ȳ�����:  //���ñ������ĺ����嵥                                        *
// *             I2C_start();                                                  *
// *             I2C_stop();                                                   *
// *             I2C_RecACK();                                                 *
// *             I2C_ACK();                                                    *
// *             I2C_NoACK();                                                  *
// *             I2C_send_byte();                                              *
// *             I2C_receive_byte();                                           *
// *  �������:    us_cnt   ������ʱ����5us�ļ���ֵ                            *
// *  �������:    ��                                                          *
// *  ȫ�ֱ�����   ��                                                          *
// *  ��������ֵ:  ��                                                          *
// *  ����˵��:    ��                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
void delay_5us(uint16_t us_cnt)    //5us��ʱ����
{
   uint16_t cnt_5us,j;
   while(us_cnt--)
   {
       cnt_5us = TIM_5US_CNT;   
       while(cnt_5us--)
       {
            j=1;
       }
       //-------------------
       clear_watchdog();       //�忴�Ź�   
   } 
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  ��������:  void I2C_start(void)                                          *
// *  ��������:  ���ڲ���I2C���������źŵĳ���                                 *
// *  ���뺯��:  //�����������õĺ����嵥                                      *
// *             delay_5us();                                                  *
// *  �ȳ�����:  //���ñ������ĺ����嵥                                        *
// *             read_from_eeprom();                                           *
// *             write_to_eeprom();                                            *
// *  �������:    ��                                                          *
// *  �������:    ��                                                          *
// *  ȫ�ֱ�����   ��                                                          *
// *  ��������ֵ:  ��                                                          *
// *  ����˵��:    ��                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
void I2C_start(void)
{
     //------------
     I2C_WP_OFF       //I2Cд������ֹ���������ݴ���
     //------------
     I2C_SCL_PIN_L    //�Ƚ�SCL���ͣ�ʹSDA��������Ч
     delay_5us(1); 
     I2C_SDA_OUTPIN_H    //�Ƚ�SDA���ߣ��Ա����Start�ź�
     //------------
     I2C_SCL_OUTPUT   //�����żĴ��������ֵ�����������ŷ���
     I2C_SDA_OUTPUT
     //------------
     delay_5us(1);     
     I2C_SCL_PIN_H    //��SCL�øߣ�ʹSDA������Ч
     delay_5us(1);
     I2C_SDA_OUTPIN_L    //��SCLΪ��ʱ��SDA�ɸ߱�ͣ�����start�ź�
     delay_5us(1);
     //------------    
     I2C_SCL_PIN_L    //start�źŸ���������SCL��ʹSDA������Ч
     delay_5us(1);
     I2C_SDA_OUTPIN_H    //����Ч״̬�£�SDA�ָ�Ϊ��
     //------------
    //�ڳ��򷵻�ʱ����֤SCL��SDA��Ϊ�����SCL=0��SDA=1
     //------------
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  ��������:  void I2C_stop(void)                                           *
// *  ��������:  ���ڲ���I2C����ֹͣ�źŵĳ���                                 *
// *  ���뺯��:  //�����������õĺ����嵥                                      *
// *             delay_5us();                                                  *
// *  �ȳ�����:  //���ñ������ĺ����嵥                                        *
// *             read_from_eeprom();                                           *
// *             write_to_eeprom();                                            *
// *  �������:    ��                                                          *
// *  �������:    ��                                                          *
// *  ȫ�ֱ�����   ��                                                          *
// *  ��������ֵ:  ��                                                          *
// *  ����˵��:    ��                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
void I2C_stop(void)
{
    //I2C_SCL_OUTPUT    //�����żĴ��������ֵ�����������ŷ���
    I2C_SDA_OUTPUT  
    //---------------
    I2C_SCL_PIN_L     //�Ƚ�SCL���ͣ�ʹSDA��������Ч
    delay_5us(1); 
    I2C_SDA_OUTPIN_L     //�Ƚ�SDA���ͣ��Ա����Stop�ź�
    //---------------
    delay_5us(1); 
    I2C_SCL_PIN_H     //��SCL�øߣ�ʹSDA������Ч
    delay_5us(1);
    I2C_SDA_OUTPIN_H     //��SCLΪ��ʱ��SDA�ɵͱ�ߣ�����stop�ź�
    delay_5us(1);
    I2C_SCL_PIN_L     //stop�źŸ���������SCL��ʹSDA������Ч
    delay_5us(1);    
    //--------------
    I2C_WP_ON        //I2Cд����ʹ�ܣ���ֹ���ݴ���    
    //--------------
    //�ڳ��򷵻�ʱ����֤SCL��SDA��Ϊ�����SCL=0��SDA=1
    //--------------
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  ��������:  uint8_t I2C_RecACK(void)                                      *
// *  ��������:  ���ڼ��I2C�������Ƿ���Ӧ���źŵĳ���                         *
// *  ���뺯��:  //�����������õĺ����嵥                                      *
// *             delay_5us();                                                  *
// *  �ȳ�����:  //���ñ������ĺ����嵥                                        *
// *             read_from_eeprom();                                           *
// *             write_to_eeprom();                                            *
// *  �������:    ��                                                          *
// *  �������:    ��                                                          *
// *  ȫ�ֱ�����   ��                                                          *
// *  ��������ֵ:  ��                                                          *
// *  ����˵��:    ��                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
uint8_t I2C_RecACK(void)   //�����ӦӦ��
{
    //---------------
    I2C_SCL_PIN_L     //�Ƚ�SCL���ͣ�ʹSDA��������Ч
    //I2C_SDA_OUTPIN_H     //�Ƚ�SDA����
    //---------------
    //I2C_SCL_OUTPUT    //�����żĴ��������ֵ�����������ŷ���
    //I2C_SDA_INPUT
    //---------------
    delay_5us(1); 
    I2C_SCL_PIN_H     //��SCL�øߣ�ʹSDA������Ч
    delay_5us(1);    
    if(I2C_SDA_INPIN==0)   //��SDA����״̬
    {
        I2C_SCL_PIN_L    //���������SCL��ʹSDA������Ч
        delay_5us(1);
        //I2C_SDA_OUTPUT   //��SDA����Ϊ���
        //I2C_SDA_OUTPIN_H    //����Ч״̬������SDA����
        return 0;
    }
    else
    {
        I2C_SCL_PIN_L    //���������SCL��ʹSDA������Ч
        delay_5us(1);
        //I2C_SDA_OUTPUT   //��SDA����Ϊ���
        //I2C_SDA_OUTPIN_H    //����Ч״̬������SDA����
        return 1;
    }
    //�ڳ��򷵻�ʱ����֤SCL��SDA��Ϊ�����SCL=0��SDA=1    
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  ��������:  void I2C_ACK(void)                                            *
// *  ��������:  ���ڲ���Ӧ���źŵĳ���                                        *
// *  ���뺯��:  //�����������õĺ����嵥                                      *
// *             delay_5us();                                                  *
// *  �ȳ�����:  //���ñ������ĺ����嵥                                        *
// *             read_from_eeprom();                                           *
// *             write_to_eeprom();                                            *
// *  �������:    ��                                                          *
// *  �������:    ��                                                          *
// *  ȫ�ֱ�����   ��                                                          *
// *  ��������ֵ:  ��                                                          *
// *  ����˵��:    ��                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
void I2C_ACK(void)
{
    //I2C_SCL_OUTPUT    //�����żĴ��������ֵ�����������ŷ���
    I2C_SDA_OUTPUT
    //---------------
    I2C_SCL_PIN_L     //�Ƚ�SCL���ͣ�ʹSDA��������Ч
    delay_5us(1);    
    I2C_SDA_OUTPIN_L     //�Ƚ�SDA����
    delay_5us(1);
    I2C_SCL_PIN_H     //��SCL�øߣ�ʹSDA������Ч,SDA=0����Ӧ���ź�
    delay_5us(1);       
    I2C_SCL_PIN_L     //�źŸ���������SCL��ʹSDA������Ч
    delay_5us(1);
    I2C_SDA_OUTPIN_H    //����Ч״̬�£�SDA�ָ�Ϊ��
    //---------------
    //�ڳ��򷵻�ʱ����֤SCL��SDA��Ϊ�����SCL=0��SDA=1    
    //---------------       
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  ��������:  void I2C_NoACK(void)                                          *
// *  ��������:  ���ڲ�����Ӧ���źŵĳ���                                      *
// *  ���뺯��:  //�����������õĺ����嵥                                      *
// *             delay_5us();                                                  *
// *  �ȳ�����:  //���ñ������ĺ����嵥                                        *
// *             read_from_eeprom();                                           *
// *             write_to_eeprom();                                            *
// *  �������:    ��                                                          *
// *  �������:    ��                                                          *
// *  ȫ�ֱ�����   ��                                                          *
// *  ��������ֵ:  ��                                                          *
// *  ����˵��:    ��                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
void I2C_NoACK(void)
{
    //I2C_SCL_OUTPUT    //�����żĴ��������ֵ�����������ŷ���
    I2C_SDA_OUTPUT
    //---------------
    I2C_SCL_PIN_L     //�Ƚ�SCL���ͣ�ʹSDA��������Ч
    delay_5us(1);    
    I2C_SDA_OUTPIN_H     //�Ƚ�SDA����
    delay_5us(1);
    I2C_SCL_PIN_H     //��SCL�øߣ�ʹSDA������Ч,SDA=1������Ӧ���ź�  
    delay_5us(1);       
    I2C_SCL_PIN_L     //�źŸ���������SCL��ʹSDA������Ч
    delay_5us(1);
    //---------------
    //�ڳ��򷵻�ʱ����֤SCL��SDA��Ϊ�����SCL=0��SDA=1    
    //--------------- 
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  ��������:  void I2C_send_byte(uint8_t byte_Val)                          *
// *  ��������:  ����I2C�����Ϸ���һ���ֽڵĳ���                               *
// *  ���뺯��:  //�����������õĺ����嵥                                      *
// *             delay_5us();                                                  *
// *  �ȳ�����:  //���ñ������ĺ����嵥                                        *
// *             read_from_eeprom();                                           *
// *             write_to_eeprom();                                            *
// *  �������:    Byte_Val   //�����ֽڵ�ֵ                                   *
// *  �������:    ��                                                          *
// *  ȫ�ֱ�����   ��                                                          *
// *  ��������ֵ:  ��                                                          *
// *  ����˵��:    ��                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
void I2C_send_byte(uint8_t byte_val)
{ 
    uint8_t bit_index;
    
    //------------    
    I2C_SCL_PIN_L     //�Ƚ�SCL���ͣ�ʹSDA��������Ч
    //------------
    //I2C_SCL_OUTPUT    
    //------------
    I2C_SDA_OUTPUT
    //------------
    for(bit_index=8;bit_index>0;bit_index--)
    {
        I2C_SCL_PIN_L
        delay_5us(1);
        if(TEST_BIT(byte_val,bit_index-1)==0)
        {
           I2C_SDA_OUTPIN_L 
        }
        else
        {
           I2C_SDA_OUTPIN_H
        }
        delay_5us(1);
        I2C_SCL_PIN_H
        delay_5us(1);        
    }
    I2C_SCL_PIN_L           //SCL��Ч
    delay_5us(1);
    I2C_SDA_OUTPIN_H           
    //-------------------------------
    //�ڳ��򷵻�ʱ����֤SCL��SDA��Ϊ�����SCL=0��SDA=1    
    //-------------------------------
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  ��������:  uint8_t I2C_receive_byte(void)                                *
// *  ��������:  ��I2C�����Ͻ���һ���ֽڵĳ���                                 *
// *  ���뺯��:  //�����������õĺ����嵥                                      *
// *             delay_5us();                                                  *
// *  �ȳ�����:  //���ñ������ĺ����嵥                                        *
// *             read_from_eeprom();                                           *
// *             write_to_eeprom();                                            *
// *  �������:    ��                                                          *
// *  �������:    ��                                                          *
// *  ȫ�ֱ�����   ��                                                          *
// *  ��������ֵ:  Byte_Val  ���ؽ����ֽڵ�ֵ                                  *
// *  ����˵��:    ��                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
uint8_t I2C_receive_byte(void)
{
   uint8_t bit_index;
   uint8_t byte_val=0;
   //------------
   //I2C_SCL_OUTPUT
   //------------
   I2C_SDA_INPUT
   //------------
   for(bit_index=8;bit_index>0;bit_index--)
   {
      I2C_SCL_PIN_H
      delay_5us(1);
      if(I2C_SDA_INPIN==0)
      {
         CLR_BIT(byte_val,bit_index-1);
      }
      else
      {
         SET_BIT(byte_val,bit_index-1);
      }
      delay_5us(1);
      I2C_SCL_PIN_L
      delay_5us(1);
   }
   //--------------------
   I2C_SCL_PIN_L
   delay_5us(1);
   I2C_SDA_OUTPUT
   I2C_SDA_OUTPIN_H   
   //--------------------
   //�ڳ��򷵻�ʱ����֤SCL��SDA��Ϊ�����SCL=0��SDA=1   
   //--------------------
   return byte_val;
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  ��������:  uint8_t read_from_eeprom(                                     *
// *                                 uint16_t *result,                         *
// *                                 uint16_t byte_address,                    *
// *                                 uint16_t byte_counter,                    *
// *                                 const uint8_t CS_A2A1A0 )                 *
// *  ��������:  ��eeprom��ȡ�ֲ����ĳ���                                      *
// *  ���뺯��:  //�����������õĺ����嵥                                      *
// *             delay_5us();                                                  *
// *             I2C_start();                                                  *
// *             I2C_stop();                                                   *
// *             I2C_RecACK();                                                 *
// *             I2C_ACK();                                                    *
// *             I2C_NoACK();                                                  *
// *             I2C_send_byte();                                              *
// *             I2C_receive_byte();                                           *
// *  �ȳ�����:  //���ñ������ĺ����嵥                                        *
// *             δ֪                                                          *
// *  �������:  byte_address  Ҫ��ȡ�����ݵ�eeprom��ַ                        *
// *             byte_counter  Ҫ��ȡ�����ݵĸ���                              *
// *             CS_A2A1A0     ����ȡ���ݵ�eepromƬѡ��ַ                      *
// *  �������:  *result    ָ��������ݼĴ����׵�ַ��ָ��                     *
// *  ȫ�ֱ����� EepromType  ȫ��ö�ٱ��� ����eeprom���ͺ�                     *
// *  ��������ֵ:   0�����������������                                        *
// *                1����������쳣����                                        *
// *                2����������������                                      *
// *  ����˵��:    ��                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
uint8_t read_from_eeprom(uint8_t *result, uint16_t  byte_address,uint16_t byte_counter,const uint8_t CS_A2A1A0)
{
    uint8_t luc_tmp;
    uint8_t luc_read_cnt;
    uint8_t *now_result;
    uint16_t luc_byte_counter;
    //------------------------------------------------------------------------
    //guc_eep_op_eercode = 0;
    //------------------------------------------------------------------------
    //������������Ҫ���ʵĵ�ַ��������������ַ�����˳��������ض�д�������
    if((byte_address + byte_counter) > MAX_SIZE) 
    {
    	  return 2;
    }
    //------------------------------------------------------------------------
    //�õ���ȡ���ݵ���ʼ��ַ
    gus_eep_now_byteaddr = byte_address;
    //------------------------------------------------------------------------
    //�õ��豸��ַ
    if(EepromType > M24C16)
    {
     	 guc_eep_driver_addr = (0xA0 | ((CS_A2A1A0<<1) & 0x0E));
    }
    else
    {
     	 luc_tmp =((gus_eep_now_byteaddr>>8)<<1)&0x0E;
       guc_eep_driver_addr = (0xA0 |((CS_A2A1A0<<1) & 0x0E) | luc_tmp);            
    }
    //--------------------------------------
    luc_read_cnt = 3;        //�����ظ�������
    //-------------------------------------------------------------------------
    do
    {
       	     luc_byte_counter = byte_counter;
       	     guc_eep_op_eercode = 0;
       	     now_result=result;
       	     //-------------------
       	     I2C_start();
             I2C_send_byte(guc_eep_driver_addr&0xFE);       //�����豸��ַ��Ϊдָ��
             luc_tmp=0;
             while(I2C_RecACK() == 1)
             {
             	  luc_tmp++;
             	  if(luc_tmp>=3)
             	  {
             	  	 guc_eep_op_eercode=1;
             	  	 break;
             	  }  
             }
             if(EepromType > M24C16)
             {
                 luc_tmp =(gus_eep_now_byteaddr>>8);
                 I2C_send_byte(luc_tmp);
                 luc_tmp=0;
                 while(I2C_RecACK() == 1)
                 {
             	     luc_tmp++;
             	     if(luc_tmp>=3)
             	     {
             	  	   guc_eep_op_eercode=1;
             	  	   break;
             	     }  
                 }  
             }
             luc_tmp =(gus_eep_now_byteaddr&0x00FF);
             I2C_send_byte(luc_tmp);
             luc_tmp=0;
             while(I2C_RecACK() == 1)
             {
             	  luc_tmp++;
             	  if(luc_tmp>=3)
             	  {
             	  	 guc_eep_op_eercode=1;
             	  	 break;
             	  }  
             }                 	  
       	     //----------------------------------
       	     I2C_start();
             I2C_send_byte(guc_eep_driver_addr|0x01);       //�����豸��ַ��Ϊ��ָ��       	     
             luc_tmp=0;
             while(I2C_RecACK() == 1)
             {
             	  luc_tmp++;
             	  if(luc_tmp>=3)
             	  {
             	  	 guc_eep_op_eercode=1;
             	  	 break;
             	  }  
             }
             //----------------------------------
             while(luc_byte_counter--)
             {
             	  *now_result = I2C_receive_byte();
             	   if(luc_byte_counter > 0)
             	   {
             	   	   I2C_ACK();
             	   	   now_result++;
             	   }
             	   else
             	   {
             	   	   I2C_NoACK();
             	   }
             	   	
             }
             //----------------------------------
             I2C_stop();
       	     //----------------------------------
       	     if(guc_eep_op_eercode==0)
       	     {   	         
       	          break;
       	     }
    }while(luc_read_cnt--);             
    //-------------------------------------------------------------------------
    return  guc_eep_op_eercode;  	
}
//--------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  ��������:  uint8_t write_to_eeprom(                                      *
// *                                           uint16_t *value,                *
// *                                           uint16_t byte_address,          *
// *                                           uint16_t byte_counter,          *
// *                                           const uint8_t CS_A2A1A0 )       *
// *  ��������:  ��eepromд���ֲ����ĳ���                                      *
// *  ���뺯��:  //�����������õĺ����嵥                                      *
// *             delay_5us();                                                  *
// *             I2C_start();                                                  *
// *             I2C_stop();                                                   *
// *             I2C_RecACK();                                                 *
// *             I2C_ACK();                                                    *
// *             I2C_NoACK();                                                  *
// *             I2C_send_byte();                                              *
// *  �ȳ�����:  //���ñ������ĺ����嵥                                        *
// *             δ֪                                                          *
// *  �������:  *value        ָ��д��eeprom���ֽ�����Դ�Ĵ����׵�ַ��ָ��    *
// *             byte_address  Ҫд���ֽ����ݵ�eeprom��ַ                      *
// *             byte_counter  Ҫд���ֽ����ݵĸ���                            *
// *             CS_A2A1A0     ��д�����ݵ�eepromƬѡ��ַ                      *
// *  �������:    ��                                                          *
// *  ȫ�ֱ�����   EepromType  ȫ��ö�ٱ��� ����eeprom���ͺ�                   *
// *  ��������ֵ:   0�����������������                                        *
// *                1����������쳣���� (д���ַ����)                         *
// *                2����������쳣���� (д�����ݳ���)                         *
// *                3����������������                                      *
// *  ����˵��:    ��                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
uint8_t write_to_eeprom(uint8_t *value,uint16_t byte_address,uint16_t byte_counter,const uint8_t CS_A2A1A0)
{
    uint8_t luc_tmp;
    uint8_t luc_write_cnt;
    uint8_t *now_value;
    //------------------------------------------------------------------------
    //guc_eep_op_eercode = 0;
    //------------------------------------------------------------------------
    //������������Ҫ���ʵĵ�ַ��������������ַ�����˳��������ض�д�������
    if((byte_address + byte_counter) > MAX_SIZE) 
    {
    	  return 3;
    }
    //------------------------------------------------------------------------
    //�õ���ʼҳ�ͽ���ҳ       
    gus_eep_start_page = byte_address/PAGE_SIZE;
    gus_eep_end_page = (byte_address+byte_counter-1)/PAGE_SIZE;
    //------------------------------------------------------------------------	
    for(gus_eep_now_page = gus_eep_start_page; gus_eep_now_page <= gus_eep_end_page; gus_eep_now_page++)
    {
          //--------------------------------------
          if(gus_eep_start_page == gus_eep_end_page)
          {
           	   gus_eep_now_byteaddr = byte_address;
           	   gus_eep_start_index = byte_address%PAGE_SIZE;
           	   gus_eep_end_index = (byte_address+byte_counter-1)%PAGE_SIZE;
           	   now_value = value;
          }
          else if(gus_eep_now_page == gus_eep_start_page)
          {
               gus_eep_now_byteaddr = byte_address;
               gus_eep_start_index = byte_address%PAGE_SIZE;
               gus_eep_end_index = PAGE_SIZE-1;
               now_value = value;
          }
          else if(gus_eep_now_page == gus_eep_end_page)
          {
          	   gus_eep_now_byteaddr = gus_eep_now_page*PAGE_SIZE;
               gus_eep_start_index = 0;
               gus_eep_end_index = (byte_address+byte_counter-1)%PAGE_SIZE;
               now_value = value - byte_address + gus_eep_now_byteaddr;
          }
          else
          {
          	   gus_eep_now_byteaddr = gus_eep_now_page*PAGE_SIZE;
               gus_eep_start_index = 0;
               gus_eep_end_index = PAGE_SIZE-1;
               now_value = value - byte_address + gus_eep_now_byteaddr;
          } 
          //--------------------------------------
          //�õ��豸��ַ
          if(EepromType > M24C16)
          {
          	 guc_eep_driver_addr = (0xA0 | ((CS_A2A1A0<<1) & 0x0E));
          }
          else
          {
          	 luc_tmp =((gus_eep_now_byteaddr>>8)<<1)&0x0E;
          	 guc_eep_driver_addr = (0xA0 |((CS_A2A1A0<<1) & 0x0E) | luc_tmp);          
          }                       	         
       	  //--------------------------------------
       	  luc_write_cnt=3;        //д��ַ�����ظ�д����
       	  //----------------------------------
       	  do
       	  {
       	     guc_eep_op_eercode = 0;
             //-------------------------
       	     I2C_start();
             I2C_send_byte(guc_eep_driver_addr&0xFE);       //�����豸��ַ��Ϊдָ��
             luc_tmp=0;
             while(I2C_RecACK() == 1)
             {
             	  luc_tmp++;
             	  if(luc_tmp>=3)
             	  {
             	  	 guc_eep_op_eercode=1;
             	  	 break;
             	  }  
             }
             if(EepromType > M24C16)
             {
                 luc_tmp =(gus_eep_now_byteaddr>>8);
                 I2C_send_byte(luc_tmp);
                 luc_tmp=0;
                 while(I2C_RecACK() == 1)
                 {
             	      luc_tmp++;
             	      if(luc_tmp>=3)
             	      {
             	  	     guc_eep_op_eercode=1;
             	  	     break;
             	      }  
                 }      
             }
             luc_tmp =(gus_eep_now_byteaddr&0x00FF);
             I2C_send_byte(luc_tmp);
             luc_tmp=0;
             while(I2C_RecACK() == 1)
             {
                luc_tmp++;
                if(luc_tmp>=3)
                {
             	     guc_eep_op_eercode=1;
             	     break;
                }  
             }                	  
       	     //----------------------------------
       	     for(gus_eep_now_index = gus_eep_start_index; gus_eep_now_index <= gus_eep_end_index; gus_eep_now_index++ )
       	     {
                    //---------------------
                    I2C_send_byte(*now_value);
                    luc_tmp=0;
                    while(I2C_RecACK() == 1)
                    {
                       luc_tmp++;
                       if(luc_tmp>=3)
                       {
             	            guc_eep_op_eercode=1;
             	            break;
                       }  
                    }    
                    //---------------------
                    now_value++;	 
       	     }
       	     //----------------------------------
       	     I2C_stop();
       	     //----------------------------------
             delay_5us(2000);     //д��һҳ֮ǰ�����㹻����ʱ 10ms 
             clear_watchdog();       //�忴�Ź�	      	         
       	     //----------------------------------
       	     if(guc_eep_op_eercode==0)
       	     {
       	     	  break;
       	     }
       	  }while(luc_write_cnt--);
       	  //--------------------------------------
       	  if(guc_eep_op_eercode!=0)
       	  {
       	   	  break;
       	  }       	  
       	  //--------------------------------------     	    	 
    }
    //------------------------------------------------------------------------	
    return  guc_eep_op_eercode;
}

//------------------------------------------------------------------------------
#endif            //end 
