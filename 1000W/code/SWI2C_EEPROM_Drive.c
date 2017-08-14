#ifndef _SWI2C_EEPROM_DRIVER_C_
#define _SWI2C_EEPROM_DRIVER_C_
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *    版权说明：Copyright(C) 2007-2012 青岛斑科变频技术有限公司              *
// *    文件名：  SWI2C_EEPROM_Drive.c                                         * 
// *    作者:     林爱军                                                       *
// *    文件版本: 1.00                                                         *
// *    生成日期: 2009年4月14日                                                *
// *    描述：    软件模拟I2C读写EEPROM的驱动程序                              *
// *    版本信息:                                                              *
// *        必需的其他文件:  SWI2C_EEPROM_Drive.h                              *
// *        使用的工具:                                                        *
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *        支持的芯片:                                                        *
// *    函数列表:                                                              *
// *            1.                                                             * 
// *    历史修改记录:                                                          *
// *            1. 作者：                                                      *
// *               日期：                                                      *
// *               版本：                                                      *
// *               描述：                                                      *
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
void delay_5us(uint16_t us_cnt);    //5us延时程序

void I2C_start(void);     //I2C总线启动操作

void I2C_stop(void);      //I2C总线停止操作

uint8_t I2C_RecACK(void);   //I2C 检查应答响应操作  返回值 1：无应答  0：有应答

void I2C_ACK(void);      //I2C总线产生应答信号操作

void I2C_NoACK(void);    //I2C总线产生无应答信号操作

void I2C_send_byte(uint8_t Byte_Val);   //I2C总线发送一个字节的操作

uint8_t I2C_receive_byte(void);   //I2C总线接收一个字节的操作
//------------------------------------------------------------------------------
//定义一个代表EEPROM型号的枚举类型
enum  eepromtype  {M24C01,M24C02,M24C04,M24C08,M24C16,M24C32,M24C64,M24C128,M24C256,M24C512};
  //------------------------------------------------------------------------------
//  |bit7 |bit6 |bit5 |bit4 |bit3 |bit2 |bit1 |bit0 |
//  |1    |0    |1    |0    |A2   |A1   |A0   |R/W  | AT24C01/AT24C02控制字   (片内地址 1Byte)
//  |1    |0    |1    |0    |A2   |A1   |a8   |R/W  | AT24C04控制字           (片内地址 1Byte)
//  |1    |0    |1    |0    |A2   |a9   |a8   |R/W  | AT24C08控制字           (片内地址 1Byte)
//  |1    |0    |1    |0    |a10  |a9   |a8   |R/W  | AT24C016控制字          (片内地址 1Byte)

//  |1    |0    |1    |0    |A2   |A1   |A0   |R/W  | AT24C32/AT24C64控制字   (片内地址 2Byte)
//  |1    |0    |1    |0    |0    |A1   |A0   |R/W  | AT24C128/AT24C256控制字 (片内地址 2Byte)
//  |1    |0    |1    |0    |A2   |A1   |A0   |R/W  | AT24C512控制字          (片内地址 2Byte)
//  Ax:为器件管脚位   ax: 为器件地址的高位
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
    uint8_t guc_eep_op_eercode;        //操作异常代码

    //0：代表操作正常结束
    //1：代表操作异常结束 (读写数据出错)
    //2：输入参数错误，溢出
    
    uint8_t guc_eep_driver_addr;        //设备地址

    uint16_t gus_eep_start_page;        //起始页
    uint16_t gus_eep_end_page;          //结束页
    uint16_t gus_eep_now_page;          //当前页
    uint16_t gus_eep_now_byteaddr;      //当前写入页起始字节的地址;
    
    uint16_t gus_eep_start_index;       //起始指针索引
    uint16_t gus_eep_end_index;         //结束指针索引
    uint16_t gus_eep_now_index;         //当前写数据指针
  
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  函数名称:  void delay_5us(uint16_t us_cnt)                               *
// *  功能描述:  未开中断时的延时5us的程序                                     *
// *  扇入函数:  //被本函数调用的函数清单                                      *
// *             无                                                            *
// *  扇出函数:  //调用本函数的函数清单                                        *
// *             I2C_start();                                                  *
// *             I2C_stop();                                                   *
// *             I2C_RecACK();                                                 *
// *             I2C_ACK();                                                    *
// *             I2C_NoACK();                                                  *
// *             I2C_send_byte();                                              *
// *             I2C_receive_byte();                                           *
// *  输入参数:    us_cnt   用于延时几个5us的计数值                            *
// *  输出参数:    无                                                          *
// *  全局变量：   无                                                          *
// *  函数返回值:  无                                                          *
// *  其他说明:    无                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
void delay_5us(uint16_t us_cnt)    //5us延时程序
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
       clear_watchdog();       //清看门狗   
   } 
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  函数名称:  void I2C_start(void)                                          *
// *  功能描述:  用于产生I2C总线启动信号的程序                                 *
// *  扇入函数:  //被本函数调用的函数清单                                      *
// *             delay_5us();                                                  *
// *  扇出函数:  //调用本函数的函数清单                                        *
// *             read_from_eeprom();                                           *
// *             write_to_eeprom();                                            *
// *  输入参数:    无                                                          *
// *  输出参数:    无                                                          *
// *  全局变量：   无                                                          *
// *  函数返回值:  无                                                          *
// *  其他说明:    无                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
void I2C_start(void)
{
     //------------
     I2C_WP_OFF       //I2C写保护禁止，允许数据传输
     //------------
     I2C_SCL_PIN_L    //先将SCL拉低，使SDA上数据无效
     delay_5us(1); 
     I2C_SDA_OUTPIN_H    //先将SDA拉高，以便给出Start信号
     //------------
     I2C_SCL_OUTPUT   //在引脚寄存器给完初值后，再设置引脚方向
     I2C_SDA_OUTPUT
     //------------
     delay_5us(1);     
     I2C_SCL_PIN_H    //将SCL置高，使SDA数据有效
     delay_5us(1);
     I2C_SDA_OUTPIN_L    //在SCL为高时，SDA由高变低，给出start信号
     delay_5us(1);
     //------------    
     I2C_SCL_PIN_L    //start信号给出后，拉低SCL，使SDA数据无效
     delay_5us(1);
     I2C_SDA_OUTPIN_H    //在无效状态下，SDA恢复为高
     //------------
    //在程序返回时，保证SCL、SDA都为输出，SCL=0，SDA=1
     //------------
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  函数名称:  void I2C_stop(void)                                           *
// *  功能描述:  用于产生I2C总线停止信号的程序                                 *
// *  扇入函数:  //被本函数调用的函数清单                                      *
// *             delay_5us();                                                  *
// *  扇出函数:  //调用本函数的函数清单                                        *
// *             read_from_eeprom();                                           *
// *             write_to_eeprom();                                            *
// *  输入参数:    无                                                          *
// *  输出参数:    无                                                          *
// *  全局变量：   无                                                          *
// *  函数返回值:  无                                                          *
// *  其他说明:    无                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
void I2C_stop(void)
{
    //I2C_SCL_OUTPUT    //在引脚寄存器给完初值后，再设置引脚方向
    I2C_SDA_OUTPUT  
    //---------------
    I2C_SCL_PIN_L     //先将SCL拉低，使SDA上数据无效
    delay_5us(1); 
    I2C_SDA_OUTPIN_L     //先将SDA拉低，以便给出Stop信号
    //---------------
    delay_5us(1); 
    I2C_SCL_PIN_H     //将SCL置高，使SDA数据有效
    delay_5us(1);
    I2C_SDA_OUTPIN_H     //在SCL为高时，SDA由低变高，给出stop信号
    delay_5us(1);
    I2C_SCL_PIN_L     //stop信号给出后，拉低SCL，使SDA数据无效
    delay_5us(1);    
    //--------------
    I2C_WP_ON        //I2C写保护使能，禁止数据传输    
    //--------------
    //在程序返回时，保证SCL、SDA都为输出，SCL=0，SDA=1
    //--------------
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  函数名称:  uint8_t I2C_RecACK(void)                                      *
// *  功能描述:  用于检查I2C总线上是否有应答信号的程序                         *
// *  扇入函数:  //被本函数调用的函数清单                                      *
// *             delay_5us();                                                  *
// *  扇出函数:  //调用本函数的函数清单                                        *
// *             read_from_eeprom();                                           *
// *             write_to_eeprom();                                            *
// *  输入参数:    无                                                          *
// *  输出参数:    无                                                          *
// *  全局变量：   无                                                          *
// *  函数返回值:  无                                                          *
// *  其他说明:    无                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
uint8_t I2C_RecACK(void)   //检查响应应答
{
    //---------------
    I2C_SCL_PIN_L     //先将SCL拉低，使SDA上数据无效
    //I2C_SDA_OUTPIN_H     //先将SDA拉高
    //---------------
    //I2C_SCL_OUTPUT    //在引脚寄存器给完初值后，再设置引脚方向
    //I2C_SDA_INPUT
    //---------------
    delay_5us(1); 
    I2C_SCL_PIN_H     //将SCL置高，使SDA数据有效
    delay_5us(1);    
    if(I2C_SDA_INPIN==0)   //读SDA引脚状态
    {
        I2C_SCL_PIN_L    //读完后，拉低SCL，使SDA数据无效
        delay_5us(1);
        //I2C_SDA_OUTPUT   //将SDA重设为输出
        //I2C_SDA_OUTPIN_H    //在无效状态，并将SDA拉高
        return 0;
    }
    else
    {
        I2C_SCL_PIN_L    //读完后，拉低SCL，使SDA数据无效
        delay_5us(1);
        //I2C_SDA_OUTPUT   //将SDA重设为输出
        //I2C_SDA_OUTPIN_H    //在无效状态，并将SDA拉高
        return 1;
    }
    //在程序返回时，保证SCL、SDA都为输出，SCL=0，SDA=1    
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  函数名称:  void I2C_ACK(void)                                            *
// *  功能描述:  用于产生应答信号的程序                                        *
// *  扇入函数:  //被本函数调用的函数清单                                      *
// *             delay_5us();                                                  *
// *  扇出函数:  //调用本函数的函数清单                                        *
// *             read_from_eeprom();                                           *
// *             write_to_eeprom();                                            *
// *  输入参数:    无                                                          *
// *  输出参数:    无                                                          *
// *  全局变量：   无                                                          *
// *  函数返回值:  无                                                          *
// *  其他说明:    无                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
void I2C_ACK(void)
{
    //I2C_SCL_OUTPUT    //在引脚寄存器给完初值后，再设置引脚方向
    I2C_SDA_OUTPUT
    //---------------
    I2C_SCL_PIN_L     //先将SCL拉低，使SDA上数据无效
    delay_5us(1);    
    I2C_SDA_OUTPIN_L     //先将SDA拉低
    delay_5us(1);
    I2C_SCL_PIN_H     //将SCL置高，使SDA数据有效,SDA=0给出应答信号
    delay_5us(1);       
    I2C_SCL_PIN_L     //信号给出后，拉低SCL，使SDA数据无效
    delay_5us(1);
    I2C_SDA_OUTPIN_H    //在无效状态下，SDA恢复为高
    //---------------
    //在程序返回时，保证SCL、SDA都为输出，SCL=0，SDA=1    
    //---------------       
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  函数名称:  void I2C_NoACK(void)                                          *
// *  功能描述:  用于产生不应答信号的程序                                      *
// *  扇入函数:  //被本函数调用的函数清单                                      *
// *             delay_5us();                                                  *
// *  扇出函数:  //调用本函数的函数清单                                        *
// *             read_from_eeprom();                                           *
// *             write_to_eeprom();                                            *
// *  输入参数:    无                                                          *
// *  输出参数:    无                                                          *
// *  全局变量：   无                                                          *
// *  函数返回值:  无                                                          *
// *  其他说明:    无                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
void I2C_NoACK(void)
{
    //I2C_SCL_OUTPUT    //在引脚寄存器给完初值后，再设置引脚方向
    I2C_SDA_OUTPUT
    //---------------
    I2C_SCL_PIN_L     //先将SCL拉低，使SDA上数据无效
    delay_5us(1);    
    I2C_SDA_OUTPIN_H     //先将SDA拉高
    delay_5us(1);
    I2C_SCL_PIN_H     //将SCL置高，使SDA数据有效,SDA=1给出无应答信号  
    delay_5us(1);       
    I2C_SCL_PIN_L     //信号给出后，拉低SCL，使SDA数据无效
    delay_5us(1);
    //---------------
    //在程序返回时，保证SCL、SDA都为输出，SCL=0，SDA=1    
    //--------------- 
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  函数名称:  void I2C_send_byte(uint8_t byte_Val)                          *
// *  功能描述:  用于I2C总线上发送一个字节的程序                               *
// *  扇入函数:  //被本函数调用的函数清单                                      *
// *             delay_5us();                                                  *
// *  扇出函数:  //调用本函数的函数清单                                        *
// *             read_from_eeprom();                                           *
// *             write_to_eeprom();                                            *
// *  输入参数:    Byte_Val   //发送字节的值                                   *
// *  输出参数:    无                                                          *
// *  全局变量：   无                                                          *
// *  函数返回值:  无                                                          *
// *  其他说明:    无                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
void I2C_send_byte(uint8_t byte_val)
{ 
    uint8_t bit_index;
    
    //------------    
    I2C_SCL_PIN_L     //先将SCL拉低，使SDA上数据无效
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
    I2C_SCL_PIN_L           //SCL无效
    delay_5us(1);
    I2C_SDA_OUTPIN_H           
    //-------------------------------
    //在程序返回时，保证SCL、SDA都为输出，SCL=0，SDA=1    
    //-------------------------------
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  函数名称:  uint8_t I2C_receive_byte(void)                                *
// *  功能描述:  从I2C总线上接收一个字节的程序                                 *
// *  扇入函数:  //被本函数调用的函数清单                                      *
// *             delay_5us();                                                  *
// *  扇出函数:  //调用本函数的函数清单                                        *
// *             read_from_eeprom();                                           *
// *             write_to_eeprom();                                            *
// *  输入参数:    无                                                          *
// *  输出参数:    无                                                          *
// *  全局变量：   无                                                          *
// *  函数返回值:  Byte_Val  返回接收字节的值                                  *
// *  其他说明:    无                                                          *
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
   //在程序返回时，保证SCL、SDA都为输出，SCL=0，SDA=1   
   //--------------------
   return byte_val;
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  函数名称:  uint8_t read_from_eeprom(                                     *
// *                                 uint16_t *result,                         *
// *                                 uint16_t byte_address,                    *
// *                                 uint16_t byte_counter,                    *
// *                                 const uint8_t CS_A2A1A0 )                 *
// *  功能描述:  从eeprom读取字参数的程序                                      *
// *  扇入函数:  //被本函数调用的函数清单                                      *
// *             delay_5us();                                                  *
// *             I2C_start();                                                  *
// *             I2C_stop();                                                   *
// *             I2C_RecACK();                                                 *
// *             I2C_ACK();                                                    *
// *             I2C_NoACK();                                                  *
// *             I2C_send_byte();                                              *
// *             I2C_receive_byte();                                           *
// *  扇出函数:  //调用本函数的函数清单                                        *
// *             未知                                                          *
// *  输入参数:  byte_address  要读取字数据的eeprom地址                        *
// *             byte_counter  要读取字数据的个数                              *
// *             CS_A2A1A0     被读取数据的eeprom片选地址                      *
// *  输出参数:  *result    指向接收数据寄存器首地址的指针                     *
// *  全局变量： EepromType  全局枚举变量 表明eeprom的型号                     *
// *  函数返回值:   0：代表操作正常结束                                        *
// *                1：代表操作异常结束                                        *
// *                2：输入参数错误，溢出                                      *
// *  其他说明:    无                                                          *
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
    //如果输入参数所要访问的地址大于器件的最大地址，则退出，并返回读写溢出代码
    if((byte_address + byte_counter) > MAX_SIZE) 
    {
    	  return 2;
    }
    //------------------------------------------------------------------------
    //得到读取数据的起始地址
    gus_eep_now_byteaddr = byte_address;
    //------------------------------------------------------------------------
    //得到设备地址
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
    luc_read_cnt = 3;        //允许重复读次数
    //-------------------------------------------------------------------------
    do
    {
       	     luc_byte_counter = byte_counter;
       	     guc_eep_op_eercode = 0;
       	     now_result=result;
       	     //-------------------
       	     I2C_start();
             I2C_send_byte(guc_eep_driver_addr&0xFE);       //发送设备地址且为写指令
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
             I2C_send_byte(guc_eep_driver_addr|0x01);       //发送设备地址且为读指令       	     
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
// *  函数名称:  uint8_t write_to_eeprom(                                      *
// *                                           uint16_t *value,                *
// *                                           uint16_t byte_address,          *
// *                                           uint16_t byte_counter,          *
// *                                           const uint8_t CS_A2A1A0 )       *
// *  功能描述:  向eeprom写入字参数的程序                                      *
// *  扇入函数:  //被本函数调用的函数清单                                      *
// *             delay_5us();                                                  *
// *             I2C_start();                                                  *
// *             I2C_stop();                                                   *
// *             I2C_RecACK();                                                 *
// *             I2C_ACK();                                                    *
// *             I2C_NoACK();                                                  *
// *             I2C_send_byte();                                              *
// *  扇出函数:  //调用本函数的函数清单                                        *
// *             未知                                                          *
// *  输入参数:  *value        指向写入eeprom的字节数据源寄存器首地址的指针    *
// *             byte_address  要写入字节数据的eeprom地址                      *
// *             byte_counter  要写入字节数据的个数                            *
// *             CS_A2A1A0     被写入数据的eeprom片选地址                      *
// *  输出参数:    无                                                          *
// *  全局变量：   EepromType  全局枚举变量 表明eeprom的型号                   *
// *  函数返回值:   0：代表操作正常结束                                        *
// *                1：代表操作异常结束 (写入地址出错)                         *
// *                2：代表操作异常结束 (写入数据出错)                         *
// *                3：输入参数错误，溢出                                      *
// *  其他说明:    无                                                          *
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
    //如果输入参数所要访问的地址大于器件的最大地址，则退出，并返回读写溢出代码
    if((byte_address + byte_counter) > MAX_SIZE) 
    {
    	  return 3;
    }
    //------------------------------------------------------------------------
    //得到起始页和结束页       
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
          //得到设备地址
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
       	  luc_write_cnt=3;        //写地址允许重复写次数
       	  //----------------------------------
       	  do
       	  {
       	     guc_eep_op_eercode = 0;
             //-------------------------
       	     I2C_start();
             I2C_send_byte(guc_eep_driver_addr&0xFE);       //发送设备地址且为写指令
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
             delay_5us(2000);     //写下一页之前需有足够的延时 10ms 
             clear_watchdog();       //清看门狗	      	         
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
