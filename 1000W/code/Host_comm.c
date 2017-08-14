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
void SCI0_configure(void);    //配置SCI0串行通讯接口

void SCI0comm_reset(void);

void SCI0_check_delaytime(void);

void SCI0_send_init(void);    //SCI0通讯发送初始化程序

void SCI0_receive_int(void);  //SCI0通讯接收初始化程序

void SCI0_rx_data_deal(void); //通讯接收数据处理程序

void SCI0_tx_delaytime(void); //SCI0通讯发送延时程序，在1ms定时程序中调用

void SCI0_rx_delaytime(void); //SCI0通讯接收延时程序，在1ms定时程序中调用

void SCI0_rx_end_delaytime(void);  //SCI0通讯接收后的延时程序，在1ms定时程序中调用

void SCI0_fault_delaytime(void);   //SCI0通讯故障延时程序，在1s定时程序中调用

void SCI0_tx_int(void);       //SCI0发送中断程序

void SCI0_rx_int(void);       //SCI0接收中断程序

uint16_t CRC16(uint8_t *puchmsg, uint16_t usDataLen);  //进行CRC校验的程序

void sci0_Renesas_tx_data_load(void);   //sci0通讯Renesas发送数据加载程序

void sci0_receive_int(void);  //sci0通讯接收初始化程序

void Renesas_rx_data_deal(void);    //Renesas协议接收数据处理程序

uint8_t Ascii_to_data(uint8_t luc_Ascii_tmp);      //Ascii码转数值函数

uint8_t data_to_Ascii(uint8_t luc_data_tmp);       //数值转Ascii码
//uint8_t guc_uart_text;
//------------------------------------------------------------------------------
//标志定义
flag_type flg_SCI0;
//------------------------------------------------------------------------------
//变量定义
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
uint8_t   guc_comm_address;   //通讯地址
uint8_t   guc_motor_ID_error_cnt;  //ID号错次数

uint8_t   guc_SCI0_type_cnt;

uint8_t   guc_SCI0_trip_code;
uint8_t   guc_LED_trip_code;

uint16_t gus_host_test;
//------------------------------------------------------------------------------
void SCI0_configure(void)     //配置SCI0串行通讯接口
{
    //G25ICR：组25中断控制寄存器
    //-------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  LV2 |  LV1 |  LV0 |  -   |  -   |  IE1 |  IE0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //-------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | IR1  | IR1  |  -   |  -   | ID1  | ID0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //-------------------------------------------------------------------   
    //LV2~LV0 : 中断优先级设置, 0到7(0最高  7禁止)
    //IE2~IE0 : 中断使能标志  (IE1=SCI0通讯完/发送完中断  IE0=SCI0接收完中断)
    //IR2~IR0 : 中断请求标志
    //ID2~ID0 : 中断检测标志     
    //-------------------------------------------------------------------
    G25ICR = 0x5000;     //中断不使能,中断优先级为5
    
    //SC0RB: SCI0接收数据缓冲器
    //SC0TB: SCI0发送数据缓冲器
    
    //SIFCLK:串行时钟选择寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |  
    // flag    |   -   |   -   |   -   |   -   |SC1CKS1|SC1CKS0|SC0CKS1|SC0CKS0| 
    //at reset |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |
    //--------------------------------------------------------------------------
    //SC1CKS1~0:SCI1时钟源   00=T0下溢 01=T1下溢 10=T2下溢 11=T3下溢
    //SC0CKS1~0:SCI0时钟源   00=T0下溢 01=T1下溢 10=T2下溢 11=T3下溢
    //--------------------------------------------------------------------------
    SIFCLK &= ~0x03;     //选择T0下溢作为波特率计时  
    
    //SC0CTR3: SCI0模式寄存器3
    //------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5  |  4  |   3   |  2  |  1  |  0  |  
    // flag    |  SC0FDC1~0  |  -  |  -  |SC0PSCE|    SC0PSC2~0    | 
    //at reset |  0   |  0   |  0  |  0  |   0   |  0  |   0 |   0 |
    //------------------------------------------------------------------------- 
    //SC0FDC1~0: SBO0最后数据传输完输出选择   00=固定输出"1"  01=固定输出"0"  x1=最后一位保持
    //SC0PSCE: 预分频使能选择  0=禁止  1=使能
    //SC0PSC2~0:   时钟选择 (同步模式 000=timer下溢的1/2   001=timer下溢的1/4  010=timer下溢的1/16)
    //                      (同步模式 011=timer下溢的1/64  100=IOCLK/2         101=IOCLK/4)
    //                      (同步模式 110,111=禁止设置)
    //                      (异步模式 000=timer下溢的1/32   001=timer下溢的1/64 010=timer下溢的1/256)
    //                      (异步模式 011=timer下溢的1/1024 100=IOCLK/2         101=IOCLK/4)
    //                      (异步模式 110,111=禁止设置)
    //-------------------------------------------------------------------------   
    SC0CTR3 = 0x08;      //时钟选择异步模式的timer下溢的1/32
    
    //引脚配置
    P2MD |= 0x05;   //P20 is SBO0 pin， P22 is SBI0 pin , other is IO (0=IO 1=Special function) 
    
    //SC0CTR0: SCI0模式寄存器0	
    //------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2    |  1    |  0    |  
    // flag    |SC0CE1|SC0SSC|  -   |SC0DIR|SC0STE|SC0LNG2|SC0LNG1|SC0LNG0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0    |  0    |  0    |
    //------------------------------------------------------------------------- 
    //SC0CE1: 数据有效边沿选择  (发送：0=下降沿  1=上升沿)(接收：0=上升沿  1=下降沿)
    //SC0SSC: 同步通讯主模式时内电路时钟源选择  0=引脚输出通讯时钟同步  1=内部产生通讯时钟同步 
    //SC0DIR: 首位发送数据选择  0=从高位首发   1=从低位首发   
    //SC0STE: 启动条件选择  0=外部   1=内部   
    //SC0LNG2~0: 同步串行传输位数   000=1bit  ...  111=8bit         
    //-------------------------------------------------------------------------
    SC0CTR0 = 0x18;      //从低位首发,内部启动条件
    
    //SC0CTR1: SCI0模式寄存器1
    //------------------------------------------------------------------------
    //reg_bit  |  7   |   6   |   5   |   4   |  3   |  2   |  1   |  0   |  
    // flag    |SC0IOM|SC0SBTS|SC0SBIS|SC0SBOS|SC0CKM|SC0MST|  -   |SC0CMD| 
    //at reset |  0   |   0   |   0   |   0   |  0   |  0   |  0   |  0   |
    //------------------------------------------------------------------------- 
    //SC0IOM: 串行数据输入引脚选择  0=SBI0  1=SBO0
    //SC0SBTS: SBT0引脚功能选择  0=IO端口  1=发送时钟IO 
    //SC0SBIS: 串行数据输入控制选择  0=固定"1"输入  1=串行数据输入
    //SC0SBOS: SBO0引脚功能选择  0=IO端口  1=串行数据输出
    //SC0CKM: 传输时钟的1/16分频选择  0=不分频  1=分频
    //SC0MST:  主从时钟模式选择   0=时钟从模式   1=时钟主模式
    //SC0CMD:  同步通讯/全双工异步通讯选择  0=同步通讯   1=全双工异步通讯      
    //-------------------------------------------------------------------------
    SC0CTR1 = 0x3D; //SBI0为串行输入,SBT0为通用IO,SBO0为串行输出,1/16分频,全双工异步通讯
    
    //SC0CTR2: SCI0模式寄存器2
    //------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2  |   1   |   0   |  
    // flag    |SC0FM1|SC0FM0|SC0PM1|SC0PM0|SC0NPE|  -  |SC0BRKF|SC0BRKE| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0  |   0   |   0   |
    //------------------------------------------------------------------------- 
    //SC0FM1~0: 数据帧模式选择   00=7bit data + 1bit stop  01=7bit data + 2bit stop
    //                           10=8bit data + 1bit stop  11=8bit data + 2bit stop
    //SC0PM1~0: 附加位说明  (发送：00="0"附加  01="1"附加  10=奇较验附加  11=偶较验附加)
    //                      (接收：00="0"检查  01="1"检查  10=奇较验检查  11=偶较验检查)
    //SC0NPE:   奇偶较验使能选择   0=较验使能   1=较验不使能
    //SC0BRKF:  空状态接收监控   0=接收数据   1=接收空状态
    //SC0BRKE:  空状态发送控制   0=发送数据   1=发送空状态
    //-------------------------------------------------------------------------
    SC0CTR2 = 0x98; //设置为8bit data  1bit stop 不较验 不发送和接收空状态
//SC0CTR2 = 0xB0;    
    //SC0STR: SCI0状态寄存器
    //------------------------------------------------------------------------
    //reg_bit  |   7   |   6   |    5  |    4  |   3  |   2  |   1  |   0  |  
    // flag    |SC0TBSY|SC0RBSY|SC0TEMP|SC0REMP|SC0FEF|SC0PEK|SC0ORE|SC0ERE| 
    //at reset |   0   |   0   |    0  |    0  |   0   |  0  |   0  |   0  |
    //------------------------------------------------------------------------- 
    //SC0TBSY:发送忙   0=不忙  1=忙
    //SC0RBSY:接收忙   0=不忙  1=忙
    //SC0TEMP:发送缓冲器空标志  0=空   1=满
    //SC0REMP:接收缓冲器空标志  0=空   1=满
    //SC0FEF:帧错误检测   0=无错误   1=有错误 
    //SC0PEK:奇偶错误检测   0=无错误   1=有错误     
    //SC0ORE:溢出错误检测   0=无错误   1=有错误     
    //SC0ERE:监控错误检测   0=无错误   1=有错误     
    //--------------------------------------------------------------------------
    
    //G3ICR：组3中断控制寄存器
    //-------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  LV2 |  LV1 |  LV0 |  -   |  -   |  IE1 |  IE0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //-------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | IR1  | IR1  |  -   |  -   | ID1  | ID0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //------------------------------------------------------------------- 
    G3ICR &= ~0x0100;    //禁止timer0下溢中断
    
    TM0MD &= ~0x80;      //禁止TM0BC计数
    
    //TM03PSC：预分频控制寄存器0
    //--------------------------------------------------------------------
    //reg_bit  |   7    |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |TMPSCNE0|  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |   0    |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------
    //TMPSCNE0:预分频操作使能选择  0=禁止  1=使能
    //--------------------------------------------------------------------   
    TM03PSC = 0x80;
    
    //TM03EXPSC：外部预分频控制寄存器0
    //--------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0    |  
    // flag    |TM3IN |TM2IN |TM1IN |TM0IN |  -   |  -   |  -   |EXPSCNE0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0    |
    //--------------------------------------------------------------------
    //TM3IN:T3计数时钟源选择  0=TM3IO pin input  1=IOCLK/128
    //TM2IN:T2计数时钟源选择  0=TM2IO pin input  1=IOCLK/128
    //TM1IN:T1计数时钟源选择  0=TM1IO pin input  1=IOCLK/128
    //TM0IN:T0计数时钟源选择  0=TM0IO pin input  1=IOCLK/128
    //EXPSCNE0:预分频(1/128)操作使能选择  0=禁止  1=使能
    //--------------------------------------------------------------------------
    TM03EXPSC = 0x00;   
    
    //TM0MD: T0模式寄存器
    //------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM0CNE|TM0LDE|  -   |  -   |  -   |TM0CK2|TM0CK1|TM0CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //------------------------------------------------------------------   	
    //TM0CNE: 计数器运转使能选择  0=不运转  1=运转
    //TM0LDE: 计数器初始化   0=正常运转   1=初始化
    //TM0CK2~0: 计数时钟源选择  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=禁止设置   
    //          100=禁止设置   101=T1下溢    110=T2下溢     111=TM0IO pin输入(上升沿),IOCLK/128
    //--------------------------------------------------------------------------
    TM0MD = 0x01;   //IOCLK/8
    
    //TM0BR: T0基本寄存器 (TM3BR,TM2BR,TM1BR与TM0BR类似)
    //--------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM0BR7|TM0BR6|TM0BR5|TM0BR4|TM0BR3|TM0BR2|TM0BR1|TM0BR0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------   	
    //TM0BR7~0: 用于在TM0BC下溢时,进行计数的初值。	TM0BC下溢后,从TM0BR加载到TM0BC中。
    //--------------------------------------------------------------------------
    TM0BR = 0x41;   //1200bps  (T0下溢,时钟选择异步模式的timer下溢的1/32,T0计数时钟为IOCLK/8)
    //------------------------------------------------------
    TM0MD |= 0x40;       //初始化TM0BC
    
    TM0MD &= ~0x40;      //清初始化TM0BC标志
    
    TM0MD |= 0x80;       //启动TM3BC计数
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
    
    guc_SCI0_rx_len = 8; //接收数据长度初始化为默认长度    
//    guc_SCI0_rx_len = 12; //接收数据长度初始化为默认长度
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void SCI0comm_reset(void)     //sci0通讯复位操作处理程序
{
    uint16_t i;
    
    //关通讯中断
    *(unsigned char *)&G25ICR = 0x02;   //清除sci0发送中断请求标志
    G25ICR &= ~0x0200;                  //清sci0发送中断使能标志
    
    *(unsigned char *)&G25ICR = 0x01;   //清除sci0接收中断请求标志            
    G25ICR &= ~0x0100;                  //清sci0接收中断使能标志
    
    //重新设置校验位
    if (bflg_SCI0_type_ok == 0)    //如果未通讯类型未确定
    {
    	  if (bflg_SCI0_parity_type == 0) //如果之前是无校验
    	  {
    	  	  bflg_SCI0_parity_type = 1;  //置有校验标志
    	  	  SC0CTR2 = 0xB0; //设置为8bit data  1bit stop 偶校验 不发送和接收空状态
    	  }
    	  else        //如果之前是偶校验
    	  {
    	  	  bflg_SCI0_parity_type = 0;  //清有校验标志
    	  	  SC0CTR2 = 0x88; //设置为8bit data  1bit stop 不校验 不发送和接收空状态
    	  }
    }
    
    //------清除通讯标志位-------
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
void SCI0_check_delaytime(void)    //通讯检测延时程序，在1s定时程序中调用
{
	  if ((bflg_host_ctrl == 1) && (bflg_SCI0_type_ok == 0))  //如果是上位机控制，且通讯类型未确定
	  {
	  	  gus_SCI0_check_delaytimer++;
	  	  if (gus_SCI0_check_delaytimer >= 5)  //5s
	  	  {
	  	  	  gus_SCI0_check_delaytimer = 0;
	  	  	  SCI0comm_reset();      //sci0通讯复位操作处理程序
	  	  }
	  }
}
//------------------------------------------------------------------------------
void SCI0_send_init(void)     //SCI0通讯发送初始化程序
{
	  uint8_t luc_tmp = 0;
	  uint8_t luc_tmp1 = 0;
	  uint16_t lus_tmp = 0;
	  word_type rs485_crc;
	  
	  if ((guc_SCI0_rx_buffer[0] == SCI0_RX_MIN_ORDER) && (guc_SCI0_rx_len == 4)) //如果是海尔协议
	  {
	  	  guc_SCI0_tx_buffer[0] = SCI0_RX_MIN_ORDER;
	  	  //--------------------------------------------------
	      guc_SCI0_tx_buffer[1] = guc_SCI0_rx_buffer[1]; //取低字节
	      guc_SCI0_tx_buffer[2] = guc_SCI0_rx_buffer[2];   //取高字节
	      //--------------------------------------------------
	      lus_tmp = gus_speed_real;
	      guc_SCI0_tx_buffer[3] = (uint8_t) (lus_tmp & 0xFF); //取低字节
	      guc_SCI0_tx_buffer[4] = (uint8_t) (lus_tmp >> 8);   //取高字节
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
	      	  	   if (guc_motor_block_cnt >= 5)//启动时赌转3次后向上位机发故障代码
	      	  	   {                            ///正常运转（实际频率大于等于最小频率）时，赌转1次后就发故障代码
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
	      
	      guc_SCI0_tx_buffer[5] = guc_SCI0_trip_code;    //得到故障代码
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
	  else if ((guc_SCI0_rx_buffer[0] == SCI0_RX_MIN_ORDER) && (guc_SCI0_rx_len == 8)) //如果是通用协议
	  {
	  	  guc_SCI0_tx_buffer[0] = SCI0_TX_ORDER;
	      //--------------------------------------------------
	      lus_tmp = gus_speed_real;
	      guc_SCI0_tx_buffer[1] = (uint8_t) (lus_tmp & 0xFF); //取低字节
	      guc_SCI0_tx_buffer[2] = (uint8_t) (lus_tmp >> 8);   //取高字节
	      
	      if (bflg_actual_direction == 0) //如果是逆时针
	      {
	      	  guc_SCI0_tx_buffer[2] &= ~0x80;
	      }
	      else  //如果是顺时针
	      {
	  	      guc_SCI0_tx_buffer[2] |= 0x80;
	      }
	      //--------------------------------------------------
	      guc_SCI0_tx_buffer[3] = guc_SCI0_rx_buffer[3]; //ID号相同
	      //--------------------------------------------------
	      guc_SCI0_tx_buffer[4] = (uint8_t) gus_Iout_peak;    //输出电流峰值
	      //--------------------------------------------------

	      if (bflg_trip_stop == 0)   //如果无故障，则发生状态代码
	      {
	  	      luc_tmp = guc_state_code;
	      }
	      else if (guc_trip_code != EEPROM_CHANGE_CODE) //&& (guc_trip_code != CLEAR_TRIP_CODE))      //如果有故障
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
	  else if ((guc_SCI0_rx_buffer[0] == SCI0_Renesas_RX_ORDER) && (guc_SCI0_rx_len == 12)) //如果是瑞萨协议
	  {
	  	  sci0_Renesas_tx_data_load();    //sci0通讯Renesas发送数据加载程序
	  }
	  else  //如果是新协议
	  {
	  	  guc_SCI0_tx_buffer[0] = 0xC0 + guc_comm_address;//  	  
	  	  guc_SCI0_tx_buffer[1] = guc_motor_ID;
	  	  //--------------------------------------------------
	  	  lus_tmp = gus_speed_real;
	      guc_SCI0_tx_buffer[2] = (uint8_t) (lus_tmp & 0xFF); //取低字节
	      guc_SCI0_tx_buffer[3] = (uint8_t) (lus_tmp >> 8);   //取高字节
	      
	      if (bflg_actual_direction == 0)  
	      {
	      	  guc_SCI0_tx_buffer[3] &= ~0x80;//实际旋转方向为顺时针  发bit7=0
	      }
	      else  
	      {
	  	      guc_SCI0_tx_buffer[3] |= 0x80;//实际旋转方向为逆时针  发bit7=1
	      }
	      //--------------------------------------------------
	      
	  	  lus_tmp = gus_Iout_for_disp;//输出电流有效值
	      guc_SCI0_tx_buffer[4] = (uint8_t) (lus_tmp & 0xFF); //取低字节
	      guc_SCI0_tx_buffer[5] = (uint8_t) (lus_tmp >> 8);   //取高字节	      
	      
	      guc_SCI0_tx_buffer[6] = 0;//预留
	      guc_SCI0_tx_buffer[7] = 0;//预留
	      //--------------------------------------------------
	      guc_SCI0_tx_buffer[8] = (uint8_t) (gss_Tm + 0x40);  //模块温度
	      //--------------------------------------------------
	  	  if (bflg_trip_stop == 0)   //如果无故障，则发生状态代码
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
	      guc_SCI0_tx_buffer[9] = luc_tmp;//状态代码
	      //--------------------------------------------------
	      /*luc_tmp = SOFT_VERSION;
	      luc_tmp <<= 2;//左移两位
	      luc_tmp |= gus_Udc_for_disp >> 14;
	       
	      guc_SCI0_tx_buffer[10] = luc_tmp;//
	      guc_SCI0_tx_buffer[11] = (uint8_t) gus_Udc_for_disp;//取低字节*/
	      
	      //[10] = D7D6:版本高位; D5D4D3D2:版本低位
	      //[11] = [10]（D1D0）+[11]：直流电压
	      luc_tmp = SOFT_VERSION / 100;
	      luc_tmp <<= 6;
	      luc_tmp1 = SOFT_VERSION % 100;
	      luc_tmp1 <<= 2;
	      luc_tmp |= luc_tmp1;
	      
	      luc_tmp1 = (uint8_t) (gus_Udc_for_disp >> 8);
	      luc_tmp1 &= 0x03;
	      luc_tmp |= luc_tmp1; 
	      
	      guc_SCI0_tx_buffer[10] = luc_tmp;//
	      guc_SCI0_tx_buffer[11] = (uint8_t) (gus_Udc_for_disp & 0xFF);//直流电压
	      
	      guc_SCI0_tx_buffer[12] = 0;//预留
	      guc_SCI0_tx_buffer[13] = 0;//预留 
	      
	      //--------------------------------------------------
	      rs485_crc.uword = CRC16(guc_SCI0_tx_buffer, guc_SCI0_tx_len - 2);
	      guc_SCI0_tx_buffer[guc_SCI0_tx_len - 2] = rs485_crc.ubyte.low;
        guc_SCI0_tx_buffer[guc_SCI0_tx_len - 1] = rs485_crc.ubyte.high;
	  }	  
    //------------------------------------------------------
    bflg_SCI0_tx_busy = 1;
    guc_SCI0_tx_point = 1;
    //------------------------------------------------------
    *(unsigned char *) & G25ICR = 0x01; //清除接收中断标志
    G25ICR &= ~0x0100;                  //清接收中断使能标志
    //----------------------------------
    SC0TB = guc_SCI0_tx_buffer[0];
    //----------------------------------
    *(unsigned char *) & G25ICR = 0x02; //清除发送中断请求标志
    G25ICR |= 0x0200;                   //置发送中断使能标志
}
//------------------------------------------------------------------------------
void sci0_Renesas_tx_data_load(void)    //sci0通讯Renesas发送数据加载程序
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
	  //加载输出频率
	  lus_tmp = gus_speed_real;//gus_freq_out;
//	  lus_tmp /= ram_para[num_MOTOR_p];   //得到机械频率
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
	  //直流电压的AD值
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
	  //输出电流
	  lus_tmp = gus_Iout_for_disp;
	  lus_tmp >>= 1;  //单位：0.2A
	  
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
	  //故障代码
	  switch (guc_trip_code)
	  {
	  	  case Normal_STATE_CODE:
	  	       guc_LED_trip_code = 0;
	  	       guc_SCI0_trip_code = 0;
	  	  	   break;
    	  case MOTOR_BLOCK_PROT_CODE:
    	  	   guc_LED_trip_code = 11;
    	  	   if (guc_motor_block_cnt >= 5)//启动时赌转3次后向上位机发故障代码
    	  	   {                            ///正常运转（实际频率大于等于最小频率）时，赌转1次后就发故障代码
    	  	   	   guc_SCI0_trip_code = 0x38;
    	  	   }	
    	  	   break;	  	  	   	  	  	   
	  	  case DCT_FAULT_CODE: //      
	  	       guc_SCI0_trip_code = 0x20;
	  	       guc_LED_trip_code = 8;
	  	  	   break;	 
	  	  case Iout_OC_PROT_CODE:
    	  	   if (guc_Iout_fault_cnt >= 5)//启动时赌转3次后向上位机发故障代码
    	  	   {                            ///正常运转（实际频率大于等于最小频率）时，赌转1次后就发故障代码
    	  	   	   guc_SCI0_trip_code = 0x08;
    	  	   }	  	  	
	  	       guc_LED_trip_code = 2;
	  	  	   break;	
	  	  case Iout_OL_PROT_CODE:
    	  	   if (guc_Iout_fault_cnt >= 5)//启动时赌转3次后向上位机发故障代码
    	  	   {                            ///正常运转（实际频率大于等于最小频率）时，赌转1次后就发故障代码
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
	  //状态代码
	  //guc_SCI0_tx_buffer[16] = 0x30;
	  //guc_SCI0_tx_buffer[17] = 0x30;
	  //变频器输出功率
	  lus_tmp = 0;//gus_Iin_for_disp;//debug
	  lus_tmp >>= 1;  //单位：0.2A
	  
	  luc_tmp1 = lus_tmp;
	  luc_tmp1 &= 0x00F0;
	  luc_tmp1 >>= 4;
	  
	  luc_tmp2 = lus_tmp;
	  luc_tmp2 &= 0x000F;
	  
	  guc_SCI0_tx_buffer[16] = data_to_Ascii(luc_tmp1);
	  guc_SCI0_tx_buffer[17] = data_to_Ascii(luc_tmp2);
	  //------------------------------------------------------
	  //计算校验和
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
void SCI0_receive_int(void)   //SCI0通讯接收初始化程序
{
	  uint8_t i;
	  
	  for (i = 0; i < 20; i++)
    {
        guc_SCI0_rx_buffer[i] = 0;
    }
    //------------------------------------------------------
    *(unsigned char *) & G25ICR = 0x02; //清除发送中断请求标志
    G25ICR &= ~0x0200;                  //清发送中断使能标志
    //----------------------------------
    *(unsigned char *) & G25ICR = 0x01; //清除接收中断标志
    G25ICR |= 0x0100;                   //置接收中断使能标志
    //------------------------------------------------------
}
//------------------------------------------------------------------------------
void SCI0_rx_data_deal(void)  //通讯接收数据处理程序
{
    uint8_t luc_tmp = 0;
    uint8_t i,luc_tmp1,luc_tmp2;
    uint16_t lus_tmp = 0;
    word_type rs485_crc;
    lword_type ltmp1;    
    //------------------------------------------------------
    if (guc_SCI0_rx_buffer[0] == SCI0_RX_MIN_ORDER)    //如果是老协议
    {
    	  if (guc_SCI0_rx_len == 8)  //如果是通用协议
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
	  	  	      //确定发送数据长度
	  	  	      guc_SCI0_tx_len = 8;
	  	  	      
	  	  	      if (bflg_host_ctrl == 1)
	  	  	      {	
			  	  	      //确定ID号
			  	  	      luc_tmp = guc_SCI0_rx_buffer[3];
			  	  	      if ((luc_tmp >= APP_ID_MIN)&&(luc_tmp <= APP_ID_MAX))       
			  	  	      {
			  	  	  	      guc_motor_ID_error_cnt = 0;
			  	  	  	      bflg_motor_ID_error = 0;
			  	  	  	      
			  	  	  	      guc_motor_ID = 0;   //老协议默认为0
			  	  	  	      //--------------------------------------
			  	  	  	      //得到上位机设定转速
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
			  	  	  	          
			  	  	  	          motor_var_init();    //停机时，电机相关变量程序化
			  	  	  	          
			  	  	  	          gus_host_setfreq = 0;
			  	  	  	          
			  	  	  	          guc_Iout_fault_cnt = 0;
			  	  	  	          guc_motor_block_cnt = 0;	 
			  	  	  	          guc_ALM_fault_cnt = 0; 	  	  	  	          
			  	  	  	          
			  	  	  	          if ((bflg_trip_stop == 1) && (bflg_trip_clear == 0))    //如果是故障阶段，且无清故障标志
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
			                  //旋转方向判断
			                  luc_tmp = guc_SCI0_rx_buffer[2];
			  	  	          luc_tmp &= 0x80;
			  	  	          
			  	  	          if (bflg_running == 0)   //如果当前未运行
			  	  	          {
			  	  	  	          if (luc_tmp == 0)    // 0
			  	  	              {
			  	  	  	              bflg_current_direction = 0;     //顺时针旋转标志
			  	  	              }
			  	  	              else
			  	  	              {
			  	  	  	              bflg_current_direction = 1;     //逆时针旋转标志
			  	  	              }
			  	  	          }
			  	  	          else    //如果当前运行
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
			  	  	      else //如果ID号超限
			  	  	      {
			  	  	  	      guc_motor_ID_error_cnt++;
			  	  	  	      if (guc_motor_ID_error_cnt >= 10)  //错10次
			  	  	  	      {
			  	  	  	  	      guc_motor_ID_error_cnt = 0;
			  	  	  	  	      
			  	  	  	  	      if (bflg_motor_ID_error == 0)
			  	  	  	          {
			  	  	  	  	          bflg_motor_ID_error = 1;   //置ID号错标志
			  	  	  	  	          trip_stop_deal(MOTOR_ID_ERR_CODE);
			  	  	  	          }
			  	  	  	      }			  	  	      	
			  	  	      }
			  	  	  }  
            }
            else   //如果校验和不对
            {
        	      if (bflg_SCI0_type_ok == 0)  //如果未确定协议
        	      {
        	  	      guc_SCI0_type_cnt++;
        	  	      if (guc_SCI0_type_cnt >= 5)  //如果超过10校验错误
        	  	      {
        	  	      	  guc_SCI0_type_cnt = 0;
        	  	      	  bflg_SCI0_type_ok = 1;
        	  	      	  
        	  	      	  guc_SCI0_rx_len = 4;      //接收字节长度改为4
        	  	      }
        	      }
            }
    	  }
    	  else if (guc_SCI0_rx_len == 4)  //如果是海尔协议
    	  {
    	  	  luc_tmp = guc_SCI0_rx_buffer[0];
    	  	  luc_tmp += guc_SCI0_rx_buffer[1];
    	  	  luc_tmp += guc_SCI0_rx_buffer[2];
    	  	  luc_tmp = -luc_tmp;
    	  	  
    	  	  if (luc_tmp == guc_SCI0_rx_buffer[3]) //如果校验和正确
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
	  	  	      
	  	  	      guc_motor_ID = 0;   //海尔协议默认为0
	  	  	      //------------------------------------------
	  	  	      //确定发送数据长度
	  	  	      guc_SCI0_tx_len = 7;
	  	  	      //------------------------------------------
	  	  	      //得到上位机设定转速
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
	  	  	  	      
	  	  	  	      if ((bflg_trip_stop == 1) && (bflg_trip_clear == 0))    //如果是故障阶段，且无清故障标志
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
	  	  
	  	  
	  	  if (luc_tmp == luc_tmp1)    //如果校验和相等
	  	  {
	  	  	  if (bflg_SCI0_type_ok == 0)  //如果通讯类型尚未确定
	  	  	  {
	  	  	  	  bflg_SCI0_type_ok = 1;
	  	  	  	  guc_SCI0_type_cnt = 0;
	  	  	  	  bflg_SCI0_parity_type = 1;    //置有校验标志
	  	          SC0CTR2 = 0xB0;               //设置为8bit data  1bit stop 偶校验 不发送和接收空状态
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
        else   //如果校验和不对
        {
    	      if (bflg_SCI0_type_ok == 0)  //如果未确定协议
    	      {
    	  	      guc_SCI0_type_cnt++;
    	  	      if (guc_SCI0_type_cnt >= 5)  //如果超过10校验错误
    	  	      {
    	  	      	  guc_SCI0_type_cnt = 0;
    	  	      	  bflg_SCI0_type_ok = 1;
			  	  	  	  bflg_SCI0_parity_type = 1;    //置有校验标志
			  	          SC0CTR2 = 0xB0;               //设置为8bit data  1bit stop 偶校验 不发送和接收空状态
    	  	      	  
    	  	      	  guc_SCI0_rx_len = 12;      //接收字节长度改为12
    	  	      }
    	      }
        }	  	  
	  }    
    else//协议（new）
    {
    	  if ((guc_SCI0_rx_buffer[0] - 0x30) == guc_comm_address)//     //如果地址正确
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
	  	  	      //确定发送数据长度
	  	  	      guc_SCI0_tx_len = 16;    //10
	  	  	      
            	  if (bflg_host_ctrl == 1)
                {
                	  if (bflg_running == 0)
                	  {	
		                	  guc_motor_ID = guc_SCI0_rx_buffer[1];  //确定ID号  guc_motor_ID
		                	  if ((guc_motor_ID >= APP_ID_MIN) && (guc_motor_ID <= APP_ID_MAX))
		                	  {
			  	  	      	      guc_motor_ID_error_cnt = 0;
			  	  	  	          bflg_motor_ID_error = 0;
			  	  	  	          
			  	  	  	          if (ram_para[num_App_ID] != guc_motor_ID)   //如果已有ID号与接收到的ID号不一致，则重写
                            {    
			  	  	  	              disable_irq();   //关总中断
			  	  	  	          
								            	  //--------------------------------------------------
								            	  //计算校验和
								                //ram_para[num_check_sum] -= ram_para[num_App_ID];
								                //ram_para[num_check_sum] += guc_swuart_rx_AppID;
								                
								                ram_para[num_App_ID] = guc_motor_ID;    //将新应用ID号写入RAM
								                writeword_to_eeprom(&(ram_para[num_App_ID]), num_App_ID, 1);  //按字写EEPROM的程序
								                //--------------------------------------------------
								                clear_watchdog();
								                //--------------------------------------------------
								                //重写EEP中的关键参数
								                for (i = 0; i < APP_PARA_CNT; i++)
								                {
								                    //ram_para[num_check_sum] -= ram_para[App_index[i]];
								                    //ram_para[num_check_sum] += App_para[ram_para[num_App_ID]-APP_ID_OFFSET][i];
								                    
								                    ram_para[App_index[i]] = App_para[ram_para[num_App_ID]-APP_ID_OFFSET][i];
								                    writeword_to_eeprom(&(ram_para[App_index[i]]), App_index[i], 1);     //按字写EEPROM的程序
								                    clear_watchdog();
								                }
								                //--------------------------------------------------
								                //writeword_to_eeprom(&(ram_para[num_check_sum]), num_check_sum, 1);      //写校验和
								                //--------------------------------------------------
								                reset_dealprog();			  	  	  	          
											  	  }					  	  	      	 	  	  	      	              	  	
		                	  }
		                	  else 
		                	  {
					  	  	      	  guc_motor_ID_error_cnt++;
					  	  	  	      if (guc_motor_ID_error_cnt >= 10)  //错10次
					  	  	  	      {
					  	  	  	  	      guc_motor_ID_error_cnt = 0;
					  	  	  	  	      
					  	  	  	  	      if (bflg_motor_ID_error == 0)
					  	  	  	          {
					  	  	  	  	          bflg_motor_ID_error = 1;   //置ID号错标志
					  	  	  	  	          trip_stop_deal(MOTOR_ID_ERR_CODE);
					  	  	  	          }
					  	  	  	      }		 	  	                        	  	
		                	  } 
		                	  //------------------------------------------------------
	  	  	  	          lus_tmp = guc_SCI0_rx_buffer[5]*10;//相位超前角的设定
	  	  	  	          if ((lus_tmp >= EEPROM_InitData[num_PhaseAdvSlope][MIN_ROW])
	  	  	  	          	 &&(lus_tmp <= EEPROM_InitData[num_PhaseAdvSlope][MAX_ROW]))
	  	  	  	          {
	  	  	  	          	   if (ram_para[num_PhaseAdvSlope] != lus_tmp)//相位超前角的设定
	  	  	  	          	   {
	  	  	  	          	   	    //disable_irq();   //关总中断
	  	  	  	          	   	    
	  	  	  	          	   	    ram_para[num_PhaseAdvSlope] = lus_tmp;
													        //计算相位超前角
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
	  	  	  	          	   	    //writeword_to_eeprom(&(ram_para[num_PhaseAdvSlope]), num_PhaseAdvSlope, 1);  //按字写EEPROM的程序
	  	  	  	          	   	    //clear_watchdog();
	  	  	  	          	   	    
	  	  	  	          	   	    //reset_dealprog();	  	  	  	          	   	
	  	  	  	          	   }
	  	  	  	          }	                	  		                	  
                	  }
	  	  	  	      //--------------------------------------
	  	  	  	      //得到上位机设定转速
	  	  	          lus_tmp = guc_SCI0_rx_buffer[3];
	  	  	          lus_tmp &= 0x7F;
	  	  	          lus_tmp <<= 8;
	  	  	          lus_tmp += guc_SCI0_rx_buffer[2];
	  	  	          
	  	  	          if (lus_tmp > ram_para[num_MOTOR_max_n])//motor_para[guc_motor_ID][MOTOR_max_n])
	  	  	          {
	  	  	      	     lus_tmp = ram_para[num_MOTOR_max_n];//motor_para[guc_motor_ID][MOTOR_max_n];
	  	  	          }
	  	  	          
	  	  	          gus_host_speed_set = lus_tmp;//得到上位机转速值
	  	  	          //--------------------------------------
	  	  	          if (guc_SCI0_rx_buffer[4]&0x02) //紧急停机判断
	  	  	          {
	  	  	              bflg_hostComm_shutdown = 1;
	  	  	          }
	  	  	          else 
	  	  	          {
	  	  	          	 bflg_hostComm_shutdown = 0;
	  	  	          }
	  	  	          //----------------------------------------
	  	  	          if ((bflg_hostComm_shutdown == 0)//无紧急停机
	  	  	          	&& (guc_SCI0_rx_buffer[4]&0x01)//有开机命令
	  	  	          	&& (gus_host_speed_set != 0)) //转速命令
	  	  	          {
			  	  	  	          bflg_host_running = 1;
		                        
		                          /*   n=60*f/p            //单位：HZ
		                               f*100=n*p*(100/60)   //单位：0.01HZ
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
	  	  	  	          
	  	  	  	          motor_var_init();    //停机时，电机相关变量程序化
	  	  	  	          
	  	  	  	          gus_host_setfreq = 0;
	  	  	  	          guc_Iout_fault_cnt = 0;
	  	  	  	          guc_motor_block_cnt = 0;	 
	  	  	  	          guc_ALM_fault_cnt = 0; 	  
	  	  	          }		
	  	  	          //----------------------------------
	  	              if ((bflg_trip_stop == 1)
	  	              	&&(guc_SCI0_rx_buffer[4]&0x04) 
	  	              	&& (bflg_trip_clear == 0))    //如果是故障阶段，且无清故障标志
	  	              {
	  	  	              //gss_trip_clear_cnt++;
	  	  	              //if (gss_trip_clear_cnt >= 10)
	  	  	              //{
	  	  	  	              //gss_trip_clear_cnt = 0;
	  	  	  	              bflg_trip_clear = 1;//允许故障清除
	  	  	  	          //}  	  	  	  	  	          
	  	  	          }	  	  	          
	  	  	          
	  	  	          //------------------------------
	                  //旋转方向判断
	                  luc_tmp = guc_SCI0_rx_buffer[3];
	  	  	          luc_tmp &= 0x80;	  	  	          
                	  if (bflg_running == 0) 
                	  {
	  	  	  	          if (luc_tmp == 0)
	  	  	              {
	  	  	  	              bflg_current_direction = 0;    //控制旋转方向标志 0=正转；1=反转
	  	  	              }                                  //bflg_actual_direction 实际旋转方向 0=正转；1=反转            
	  	  	              else
	  	  	              {
	  	  	  	              bflg_current_direction = 1;     
	  	  	              }	                	  	
                	  }	
                	  else //运行ing
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
	  	  	      //guc_SCI0_rx_buffer[6]//预留
	  	  	      //guc_SCI0_rx_buffer[7]//预留     
            }
            else   //如果校验和不对
            {
        	      if (bflg_SCI0_type_ok == 0)  //如果未确定协议
        	      {
        	  	      guc_SCI0_type_cnt++;
        	  	      if (guc_SCI0_type_cnt >= 5)  //如果超过10校验错误
        	  	      {
        	  	      	  guc_SCI0_type_cnt = 0;
        	  	      	  bflg_SCI0_type_ok = 1;
        	  	      	  
        	  	      	  guc_SCI0_rx_len = 10;      //接收字节长度改为10
        	  	      }
        	      }
            }            
    	  } 	
    } 
}
//------------------------------------------------------------------------------
void Renesas_rx_data_deal(void)    //Renesas协议接收数据处理程序
{
	  //uint8_t i;
	  
	  uint16_t lus_tmp1;
	  uint16_t lus_tmp2;
	  uint16_t lus_tmp3;
	  uint16_t lus_tmp4;
	  
	  uint16_t lus_tmp;
	  //--------------------------------------------
	  guc_motor_ID = 32;   //默认ID
    bflg_current_direction = 0; //默认顺时针旋转标志----peak原来是1

    //确定发送数据长度
    guc_SCI0_tx_len = 20;	
    //得到上位机设定转速  
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
	  lus_tmp += lus_tmp4;//转速指令
	  
	  lus_tmp *= 6; //(协议单位：0.1rps)
	  
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
	      
	      motor_var_init();    //停机时，电机相关变量程序化
	      
	      gus_host_setfreq = 0;
	      
	      guc_Iout_fault_cnt = 0;
	      guc_motor_block_cnt = 0;	 
	      guc_ALM_fault_cnt = 0; 	  	  	  	      
	      
	      if ((bflg_trip_stop == 1) && (bflg_trip_clear == 0))    //如果是故障阶段，且无清故障标志
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
void SCI0_tx_delaytime(void)  //SCI0通讯发送延时程序，在1ms定时程序中调用
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
void SCI0_rx_delaytime(void)  //SCI0通讯接收延时程序，在1ms定时程序中调用
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
void SCI0_rx_end_delaytime(void)   //SCI0通讯接收后的延时程序，在1ms定时程序中调用
{
    if (bflg_SCI0_rx_busy == 1)   //如果当前接收忙
    {
        gss_SCI0_rx_end_delaytimer++;
        if (gss_SCI0_rx_end_delaytimer >= 15)
        {
            gss_SCI0_rx_end_delaytimer = 0;  //清接收完延时计时器
            bflg_SCI0_rx_busy = 0;           //清接收忙标志
            
            G25ICR &= ~0x0100;               //清接收中断使能标志
            
            bflg_SCI0_rx_delaytime = 1;
	  	  	  gss_SCI0_rx_delaytimer = 5;
	  	  	  
	  	  	  if (bflg_SCI0_type_ok == 0) //如果未确定协议
        	  {
        	  	  guc_SCI0_type_cnt++;
        	  	  if (guc_SCI0_type_cnt >= 5) //如果超过10校验错误
        	  	  {
        	  	      guc_SCI0_type_cnt = 0;
        	  	      bflg_SCI0_type_ok = 1;
        	  	      
        	  	      guc_SCI0_rx_len = 4;     //接收字节长度改为4
        	  	  }
        	  }
        }
    }
}
//------------------------------------------------------------------------------
void SCI0_fault_delaytime(void)    //SCI0通讯故障延时程序，在1s定时程序中调用
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
void SCI0_tx_int(void)        //SCI0发送中断程序
{
	  if (bflg_SCI0_tx_busy == 1)    //如果发送忙
	  {
	  	  if (guc_SCI0_tx_point < guc_SCI0_tx_len)       //如果还没发送完
	  	  {
	  	  	  SC0TB = guc_SCI0_tx_buffer[guc_SCI0_tx_point++];     //发送数据
	  	  }
	  	  else
	  	  {
	  	  	  guc_SCI0_tx_point = 0;
	  	  	  bflg_SCI0_tx_busy = 0;
	  	  	  //----------------------------------------------
	  	  	  G25ICR &= ~0x0200;     //清发送中断使能标志
	  	  	  //----------------------------------------------
	  	  	  bflg_SCI0_rx_delaytime = 1;
	  	  	  gss_SCI0_rx_delaytimer = 5;
	  	  }
	  }
}
//------------------------------------------------------------------------------
void SCI0_rx_int(void)        //SCI0接收中断程序
{
	  uint8_t luc_SCI0_rx_byte;
	  
	  gss_SCI0_rx_end_delaytimer = 0;
	  //----------------------------------
	  luc_SCI0_rx_byte = SC0RB;
	  
	  if ((bflg_SCI0_rx_busy == 0) 
	  	&& (((luc_SCI0_rx_byte >= SCI0_RX_MIN_ORDER) && (luc_SCI0_rx_byte <= SCI0_RX_MAX_ORDER))
	  	   ||((luc_SCI0_rx_byte >= 0x30) && (luc_SCI0_rx_byte <= 0x33)))
	  	   || (luc_SCI0_rx_byte == SCI0_Renesas_RX_ORDER))   //如果不是接收忙，且是命令字
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
	  	  	  G25ICR &= ~0x0100;     //清接收中断使能标志
	  	  	  
	  	  	  guc_SCI0_rx_point = 0;
	  	  	  bflg_SCI0_rx_busy = 0;
	  	  	  bflg_SCI0_rx_ok = 1;
	  	  	  
	  	  	  bflg_SCI0_rx_delaytime = 1;
	  	  	  gss_SCI0_rx_delaytimer = 50;
	  	  }
	  }
}
//------------------------------------------------------------------------------
uint16_t CRC16(uint8_t *puchmsg, uint16_t usDataLen)  //进行CRC校验的程序
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
uint8_t Ascii_to_data(uint8_t luc_Ascii_tmp)      //Ascii码转数值函数
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
uint8_t data_to_Ascii(uint8_t luc_data_tmp)       //数值转Ascii码
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
