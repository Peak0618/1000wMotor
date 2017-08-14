#ifndef	_PANEL_COMM_C_
#define _PANEL_COMM_C_
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *    版权说明：Copyright(C) 2007-2012 青岛斑科变频技术有限公司              *
// *    文件名：  panel_comm.c                                                 * 
// *    作者:     林爱军                                                       *
// *    文件版本: 1.00                                                         *
// *    生成日期: 2009年9月15日                                                *
// *    描述：    本文件主要用于建立与LCD监控面板通讯的处理程序                *
// *    版本信息:                                                              *
// *        必需的其他文件:  p30f3010.gld, libpic30.a                          *
// *        使用的工具:      MPLAB IDE -> 8.00                                 *
// *                         Compiler  -> 3.02                                 *
// *                         Assembler -> 2.00                                 *
// *                         Linker    -> 2.00                                 *
// *        支持的芯片:      dsPIC30F3010                                      *
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
void configure_SCI1(void);      //配置SCI1串行通讯接口

void SCI1comm_reset(void);      //SCI1通讯复位操作处理程序

void SCI1rx_on_dealtime(void);  //延时接收计时程序，在1ms定时程序中调用

void SCI1_rx_int(void);         //SCI1接收中断

//void SCI1_rx_err_int(void);    //SCI1接收错误中断

void SCI1rx_end_delaytimer(void);    //通讯接收后的延时程序，加入1ms定时程序中

void SCI1comm_abend_dealtime(void);  //串口通讯异常处理程序,在1S定时程序中调用

void SCI1rx_ctrl_abend_dealtime(void);   //串口通讯接收控制信息异常处理程序,在1S定时程序中调用

void SCI1rx_data_deal(void);     //SCI1接收数据处理程序

void SCI1_handshake_info_deal(void);   //接收的握手信息处理程序

void SCI1_wrcmd_info_deal(void);     //接收的写命令信息处理程序

void SCI1_rdcmd_info_deal(void);     //接收的读命令信息处理程序

void SCI1_ctrl_info_deal(void);     //接收的控制命令信息处理程序

void SCI1tx_on_dealtime(void);  //延时发送计时程序，在1ms定时程序中调用

void SCI1_tx_int(void);     //SCI1发送中断

void SCI1_txend_int(void);  //SCI1发送完中断
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  函数名称:  signed int read_eeprom(unsigned int eep_num)                  *
// *  功能描述:  根据参数编号从EEPROM中读取参数的程序                          *
// *  扇入函数:  //被本函数调用的函数清单                                      *
// *  扇出函数:  //调用本函数的函数清单                                        *
// *             void read_para_eeptoram(void)                                 *
// *             void init_all_param(void)                                     *
// *  输入参数:  eep_num  //参数编号   eep_num必须小于511                      *
// *  输出参数:  无                                                            *
// *  全局变量： gsi_eeprom_para[x]                                            *
// *  函数返回值: lsi_eedata_val   //EEP参数值                                 *
// *  其他说明:   无                                                           *
// *                                                                           *
// *****************************************************************************
//-------------------------------------------------------------------------------	 
   uint16_t gus_SCI1rx_delay_timer;          //延时接收计时器
   uint16_t gus_SCI1tx_delay_timer;          //延时发送计时器

   uint16_t gus_SCI1comm_abend_timer;        //通讯异常计时器
   uint16_t gus_SCI1rx_ctrl_abend_timer;     //接收控制信息异常计时器
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
	 uint8_t gus_tmppin;    //虚拟引脚，为以后扩展为RS485通讯留用
	 
   #define SCI1_RS485SEL_PIN	gus_tmppin
	
	 #define SCI1_RS485SEL_IS_REC     0
	 #define SCI1_RS485SEL_IS_TANS	  1
	
//------------------------------------------------------------------------------
void SCI1_configure(void)   //配置SCI1串行通讯接口1
{
    //G26ICR：组26中断控制寄存器
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
    //IE2~IE0 : 中断使能标志  (IE1=SCI1通讯完/发送完中断  IE0=SCI1接收完中断)
    //IR2~IR0 : 中断请求标志
    //ID2~ID0 : 中断检测标志     
    //-------------------------------------------------------------------
    G26ICR = 0x5000;     //中断不使能,中断优先级为5
    
    //SC1RB: SCI1接收数据缓冲器
    //SC1TB: SCI1发送数据缓冲器
    
    //SIFCLK:串行时钟选择寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |  
    // flag    |   -   |   -   |   -   |   -   |SC1CKS1|SC1CKS0|SC1CKS1|SC1CKS0| 
    //at reset |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |
    //--------------------------------------------------------------------------
    //SC1CKS1~0:SCI1时钟源   00=T0下溢 01=T1下溢 10=T2下溢 11=T3下溢
    //SC1CKS1~0:SCI0时钟源   00=T0下溢 01=T1下溢 10=T2下溢 11=T3下溢
    //--------------------------------------------------------------------------
    SIFCLK &= ~0x0C;
    SIFCLK |= 0x04;      //选择T1下溢作为波特率计时
    
    //SC1CTR3: SCI0模式寄存器3
    //------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5  |  4  |   3   |  2  |  1  |  0  |  
    // flag    |  SC1FDC1~0  |  -  |  -  |SC1PSCE|    SC1PSC2~0    | 
    //at reset |  0   |  0   |  0  |  0  |   0   |  0  |   0 |   0 |
    //------------------------------------------------------------------------- 
    //SC1FDC1~0: SBO0最后数据传输完输出选择   00=固定输出"1"  10=固定输出"0"  x1=最后一位保持
    //SC1PSCE: 预分频使能选择  0=禁止  1=使能
    //SC1PSC2~0:   时钟选择 (同步模式 000=timer下溢的1/2   001=timer下溢的1/4  010=timer下溢的1/16)
    //                      (同步模式 011=timer下溢的1/64  100=IOCLK/2         101=IOCLK/4)
    //                      (同步模式 110,111=禁止设置)
    //                      (异步模式 000=timer下溢的1/32   001=timer下溢的1/64 010=timer下溢的1/256)
    //                      (异步模式 011=timer下溢的1/1024 100=IOCLK/2         101=IOCLK/4)
    //                      (异步模式 110,111=禁止设置)
    //-------------------------------------------------------------------------   
    SC1CTR3 = 0x08;      //时钟选择异步模式的timer下溢的1/32
    
    //SC1CTR0: SCI0模式寄存器0	
    //------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2    |  1    |  0    |  
    // flag    |SC1CE1|SC1SSC|  -   |SC1DIR|SC1STE|SC1LNG2|SC1LNG1|SC1LNG0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0    |  0    |  0    |
    //------------------------------------------------------------------------- 
    //SC1CE1: 数据有效边沿选择  (发送：0=下降沿  1=上升沿)(接收：0=上升沿  1=下降沿)
    //SC1SSC: 同步通讯主模式时内电路时钟源选择  0=引脚输出通讯时钟同步  1=内部产生通讯时钟同步 
    //SC1DIR: 首位发送数据选择  0=从高位首发   1=从低位首发   
    //SC1STE: 启动条件选择  0=外部   1=内部   
    //SC1LNG2~0: 同步串行传输位数   000=1bit  ...  111=8bit         
    //-------------------------------------------------------------------------
    SC1CTR0 = 0x18;      //从低位首发,内部启动条件
    
    //SC1CTR1: SCI0模式寄存器1
    //------------------------------------------------------------------------
    //reg_bit  |  7   |   6   |   5   |   4   |  3   |  2   |  1   |  0   |  
    // flag    |SC1IOM|SC1SBTS|SC1SBIS|SC1SBOS|SC1CKM|SC1MST|  -   |SC1CMD| 
    //at reset |  0   |   0   |   0   |   0   |  0   |  0   |  0   |  0   |
    //------------------------------------------------------------------------- 
    //SC1IOM: 串行数据输入引脚选择  0=SBI0  1=SBO0
    //SC1SBTS: SBT0引脚功能选择  0=IO端口  1=发送时钟IO 
    //SC1SBIS: 串行数据输入控制选择  0=固定"1"输入  1=串行数据输入
    //SC1SBOS: SBO0引脚功能选择  0=IO端口  1=串行数据输出
    //SC1CKM: 传输时钟的1/16分频选择  0=不分频  1=分频
    //SC1MST:  主从时钟模式选择   0=时钟从模式   1=时钟主模式
    //SC1CMD:  同步通讯/全双工异步通讯选择  0=同步通讯   1=全双工异步通讯      
    //-------------------------------------------------------------------------
    SC1CTR1 = 0x3D; //SBI0为串行输入,SBT0为通用IO,SBO0为串行输出,1/16分频,全双工异步通讯
    
    //SC1CTR2: SCI0模式寄存器2
    //------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2  |   1   |   0   |  
    // flag    |SC1FM1|SC1FM0|SC1PM1|SC1PM0|SC1NPE|  -  |SC1BRKF|SC1BRKE| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0  |   0   |   0   |
    //------------------------------------------------------------------------- 
    //SC1FM1~0: 数据帧模式选择   00=7bit data + 1bit stop  01=7bit data + 2bit stop
    //                           10=8bit data + 1bit stop  11=8bit data + 2bit stop
    //SC1PM1~0: 附加位说明  (发送：00="0"附加  01="1"附加  10=奇较验附加  11=偶较验附加)
    //                      (接收：00="0"检查  01="1"检查  10=奇较验检查  11=偶较验检查)
    //SC1NPE:   奇偶较验使能选择   0=较验使能   1=较验不使能
    //SC1BRKF:  空状态接收监控   0=接收数据   1=接收空状态
    //SC1BRKE:  空状态发送控制   0=发送数据   1=发送空状态
    //-------------------------------------------------------------------------
    SC1CTR2 = 0xB0; //设置为8bit data  1bit stop 偶校验 不发送和接收空状态
    
    //SC1STR: SCI0状态寄存器
    //------------------------------------------------------------------------
    //reg_bit  |   7   |   6   |    5  |    4  |   3  |   2  |   1  |   0  |  
    // flag    |SC1TBSY|SC1RBSY|SC1TEMP|SC1REMP|SC1FEF|SC1PEK|SC1ORE|SC1ERE| 
    //at reset |   0   |   0   |    0  |    0  |   0   |  0  |   0  |   0  |
    //------------------------------------------------------------------------- 
    //SC1TBSY:发送忙   0=不忙  1=忙
    //SC1RBSY:接收忙   0=不忙  1=忙
    //SC1TEMP:发送缓冲器空标志  0=空   1=满
    //SC1REMP:接收缓冲器空标志  0=空   1=满
    //SC1FEF:帧错误检测   0=无错误   1=有错误 
    //SC1PEK:奇偶错误检测   0=无错误   1=有错误     
    //SC1ORE:溢出错误检测   0=无错误   1=有错误     
    //SC1ERE:监控错误检测   0=无错误   1=有错误     
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
    G3ICR &= ~0x0200;    //禁止timer1下溢中断
    
    TM1MD &= ~0x80;      //禁止TM1BC计数
    
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
    
    //TM1MD: T1模式寄存器
    //------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM1CNE|TM1LDE|  -   |  -   |  -   |TM1CK2|TM1CK1|TM1CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //------------------------------------------------------------------   	
    //TM1CNE: 计数器运转使能选择  0=不运转  1=运转
    //TM1LDE: 计数器初始化   0=正常运转   1=初始化
    //TM1CK2~0: 计数时钟源选择  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=禁止设置   
    //          100=禁止设置   101=T1下溢    110=T2下溢     111=TM1IO pin输入(上升沿),IOCLK/128
    //--------------------------------------------------------------------------
    TM1MD = 0x01;   //IOCLK/8
    
    //TM1BR: T0基本寄存器 (TM3BR,TM2BR,TM1BR与TM0BR类似)
    //--------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM1BR7|TM1BR6|TM1BR5|TM1BR4|TM1BR3|TM1BR2|TM1BR1|TM1BR0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------   	
    //TM1BR7~0: 用于在TM1BC下溢时,进行计数的初值。	TM1BC下溢后,从TM1BR加载到TM1BC中。
    //--------------------------------------------------------------------------
    TM1BR = 0x0f;//   20M//4800bps  (T1下溢,时钟选择异步模式的timer下溢的1/32,T1计数时钟为IOCLK/8)
    //------------------------------------------------------
    TM1MD |= 0x40;       //初始化TM1BC
    
    TM1MD &= ~0x40;      //清初始化TM1BC标志
    
    TM1MD |= 0x80;       //启动TM1BC计数   
    
    //-----------------------------------------------------------------------
    SCI1comm_reset();    //uart1通讯复位操作处理程序
}
//---------------------------------------------------------------------------
void SCI1comm_reset(void)    //SCI1通讯复位操作处理程序
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
    gus_lcd_setfreq=0;     //清除面板设定频率寄存器
    //------------- 
    bflg_SCI1_allow_rx=1;         //置允许接收标志
    gus_SCI1rx_delay_timer=2;     //延时接收计时器赋初值	  
}
//---------------------------------------------------------------------------
void SCI1rx_on_dealtime(void)  //延时接收计时程序，在1ms定时程序中调用
{
    if(bflg_SCI1_allow_rx==1)
    {
        gus_SCI1rx_delay_timer--;
        if(gus_SCI1rx_delay_timer<=0)
        {
            bflg_SCI1_allow_rx=0;
            gus_SCI1rx_delay_timer=0;
            //--------------------------------------
            *(unsigned char *)&G26ICR = 0x02;  //清除SCI1发送中断请求标志
            G26ICR &= ~0x0200;    //清SCI1发送中断
            //----发送完毕延时开接收中断------
            //guc_sci0rx_byte=SC1RB;  //假读 当前无影响           
            *(unsigned char *)&G26ICR = 0x01;  //清除SCI1接收中断请求标志            
            G26ICR |= 0x0100;    //置SCI1接收中断使能标志
        }
    }
}
//--------------------------------------------------------------------------
void SCI1_rx_int(void)     //SCI1接收中断
{
	 	guc_SCI1rx_byte=SC1RB;    //从接收寄存器中读数据
	 	//-------------------------
    gus_SCI1rx_end_timer=0;     //清接收完计时器
    
    if((bflg_SCI1_rx_busy==0)&&(guc_SCI1rx_byte==0x55))
    {
        gus_SCI1rx_cnt=0;
        bflg_SCI1_rx_busy=1;              //置接收忙标志
        guc_SCI1rx_buffer[0]=guc_SCI1rx_byte;
        gus_SCI1rx_cnt++;
    }
    else if(bflg_SCI1_rx_busy==1)   //如果是SCI1接收忙标志，则进入
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
            bflg_SCI1_rx_busy=0;     //清接收忙标志
            bflg_SCI1_rx_ok=1;       //置接收成功标志            
        }
    }     	 	  
}
//---------------------------------------------------------------------------
void SCI1rx_end_delaytimer(void)      //通讯接收后的延时程序，加入1ms定时程序中
{
    if(bflg_SCI1_rx_busy==1)
    {
        gus_SCI1rx_end_timer++;
        if(gus_SCI1rx_end_timer>=5)
        {
            gus_SCI1rx_cnt=0;
            bflg_SCI1_rx_busy=0;     //清接收忙标志
            bflg_SCI1_rx_ok=0;       //清接收成功标志
            
            //--------------------------------
		        *(unsigned char *)&G26ICR = 0x02;     //清除SCI1发送中断请求标志
            //-----------------------
            *(unsigned char *)&G26ICR = 0x01;     //清除SCI1接收中断请求标志                
            G26ICR |= 0x0100;      //置SCI1接收中断使能标志
        }
    }	
}
//---------------------------------------------------------------------------
void SCI1comm_abend_dealtime(void)     //串口通讯异常处理程序,在1S定时程序中调用
{
     gus_SCI1comm_abend_timer++;
     if(gus_SCI1comm_abend_timer>30)
     {
        gus_SCI1comm_abend_timer = 0;
        SCI1comm_reset();               //SCI1通讯复位操作
        //stop_key();                   //停机键处理
        //bflg_SCI1comm_err_prot = 1;   //置SCI1通讯故障保护标志
     }
}
//---------------------------------------------------------------------------
void SCI1rx_ctrl_abend_dealtime(void)     //串口通讯接收控制信息异常处理程序,在1S定时程序中调用
{
     gus_SCI1rx_ctrl_abend_timer++;
     if(gus_SCI1rx_ctrl_abend_timer>30)
     {
          gus_SCI1rx_ctrl_abend_timer=0;
          SCI1comm_reset();                //SCI1通讯复位操作         
          //stop_key();                    //停机键处理
          //bflg_SCI1comm_err_prot = 1;    //置uart1通讯故障保护标志
     }
}
//---------------------------------------------------------------------------
/*
void SCI1comm_err_trip_deal(void)    //通讯故障处理程序,在主循环中调用
{
     if(bflg_SCI1comm_err_prot==1)
     {
         bflg_SCI1comm_err_prot = 0;     //清通讯故障保护标志
         if(bflg_trip_stop==0)   //如果已经置了故障停机标志，则转
         {
             lamp_trip_code=Lamp_CommErr_TRIP_CODE;
             trip_code=CommErr_TRIP_CODE; 
             trip_stop_deal_op();    //调用故障停机处理程序
         }          
     }
}
*/
//---------------------------------------------------------------------------
void SCI1rx_data_deal(void)     //SCI1接收数据处理程序,在主循环中调用
{
    if(bflg_SCI1_rx_ok==1)    //如果是接收ok,则进入
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
                gus_SCI1comm_abend_timer=0;  //清通讯异常计时器  //接收正确后，清通讯异常计数器
                //-------------------------------------                
                switch(guc_SCI1rx_buffer[1])
                {
                   case 0xA1:
                            SCI1_handshake_info_deal();   //接收的握手信息处理程序
                            break;
                   case 0xB1:
                            SCI1_wrcmd_info_deal();       //接收的写命令信息处理程序
                            break;
                   case 0xB2:
                            SCI1_rdcmd_info_deal();       //接收的读命令信息处理程序
                            break;
                   case 0xC1:
                            SCI1_ctrl_info_deal();        //接收的控制信息处理程序
                            break;
                }
                //-------------------------------------
                bflg_SCI1_allow_tx=1;    //置允许发送标志
                gus_SCI1tx_delay_timer=20;    //延时20ms后发送                
                //-------------------------------------
             }
        }
    }
}
//------------------------------------------------------------
void SCI1_handshake_info_deal(void)   //接收的握手信息处理程序
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
void SCI1_wrcmd_info_deal(void)     //接收的写命令信息处理程序
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
     
     if((para_addr.uword == 0xFFFF)&&(para_val.uword == 0xFFFF))  //参数修改完毕后，程序进行复位
     {
         if((bflg_running==0)&&((bflg_eeprom_change==1)||(bflg_trip_lock==1)))    
         { //只有在停机状态下且是参数变更标志或是故障锁定标志，发送复位信息，CPU才复位
             reset_dealprog();    //芯片复位处理程序
         }
     }
     else
     {
         if(para_addr.uword<MAX_PARA_NUMBER)
         {             
             //返回32位结果 高16位存放上限  低16位存放下限
             tmp.lword = get_eeppara_limit(para_addr.uword);    //得到参数上下限的程序
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
                  if((para_addr.word==num_pr_set_method)&&(para_val.word==2))   //如果要求恢复出厂参数，则进入
                  {
     	   	            tmp.word.low = init_all_para();   //初始化EEPROM程序
                      //------------------
     	   	            bflg_trip_stop = 1;     //置故障停机标志 
     	   	            bflg_eeprom_change = 1;  //置EEPROM变更标志
     	   	            if(tmp.word.low!=0)
     	   	            {
     	   	               bflg_eeprom_error = 1;     //置EEPROM出错标志
     	   	               guc_trip_code = EEPROM_ERR_CODE;   //读EEPROM错
     	   	               guc_lamp_trip_code = LAMP_EEPROM_ERR_CODE;
     	   	            }
     	   	            else
     	   	            {
     	   	               bflg_eeprom_error = 0;     //清EEPROM出错标志
     	   	               guc_trip_code = EEPROM_CHANGE_CODE;   //参数变更
     	   	               guc_lamp_trip_code = LAMP_EEPROM_CHANGE_CODE;     	   	             	  
     	   	            }
                  }
                  else if((para_addr.word==num_Trip_clear)&&(para_val.word==1))  //如果要求清除故障代码，则进入
                  {
                      clear_trip_cause();   //清除跳停原因
                      
                      bflg_trip_stop = 1;     //置故障停机标志 
     	   	            bflg_eeprom_change = 1;  //置EEPROM变更标志
     	   	            guc_trip_code = EEPROM_CHANGE_CODE;   //参数变更
     	   	            guc_lamp_trip_code = LAMP_EEPROM_CHANGE_CODE;
                  }
                  else if(ram_para[para_addr.word]!=para_val.word)
                  {
                      //-------------------------------------
                      //计算新校验和
                      //ram_para[num_check_sum] -= ram_para[para_addr.word];
                      //ram_para[num_check_sum] += para_val.word;
                      //-------------------------------------
                      ram_para[para_addr.word]=para_val.word;      //将参数写入RAM的EEP参数区
                      //------------------------------------
                      tmp.ulword =  ram_para[para_addr.word];
                      //tmp.ulword *= EEPROM_InitData[para_addr.word].multiple;
                      ram_para[para_addr.word] = (uint16_t) tmp.ulword;  //将参数写入RAM的使用参数区
                      //------------------------------------
                      //tmp.uword.low = write_to_eeprom(&(para_val.word),para_addr.word,2,0);  
	                    tmp.word.low = writeword_to_eeprom(&ram_para[para_addr.word],para_addr.word,1);   //按字写EEPROM的程序
                      //-----------------------------
                      //将校验和写入EEPROM
                      //tmp.word.high = writeword_to_eeprom(&ram_para[num_check_sum],num_check_sum,1);   //写校验和
                      //-----------------------------
	                    bflg_trip_stop = 1;     //置故障停机标志 
     	   	            bflg_eeprom_change = 1;  //置EEPROM变更标志
	                    if((tmp.uword.low!=0)||(tmp.word.high!=0))
     	   	            {
     	   	               bflg_eeprom_error = 1;     //置EEPROM出错标志
     	   	               guc_trip_code = EEPROM_ERR_CODE;   //读EEPROM错
     	   	               guc_lamp_trip_code = LAMP_EEPROM_ERR_CODE;
     	   	            }
     	   	            else
     	   	            {
     	   	               bflg_eeprom_error = 0;     //清EEPROM出错标志
     	   	               guc_trip_code = EEPROM_CHANGE_CODE;   //参数变更
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
void SCI1_rdcmd_info_deal(void)    //接收的读命令信息处理程序
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
         {   //如果读的是基准频率参数，则设定频率上下限用No.006、No.007参数限定，用于面板调试。
             tmp.word.high=ram_para[num_max_freq];
             tmp.word.low=ram_para[num_min_freq]; 	   
         }
         else 
         {
         	   tmp.lword = get_eeppara_limit(para_addr.uword);    //得到参数上下限的程序 
         }
             
         guc_SCI1tx_buffer[7]=tmp.ubyte.HL;
         guc_SCI1tx_buffer[8]=tmp.ubyte.HH;               //参数上限

         guc_SCI1tx_buffer[9]=tmp.ubyte.LL;
         guc_SCI1tx_buffer[10]=tmp.ubyte.LH;              //参数下限
         
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
void SCI1_ctrl_info_deal(void)     //接收的控制命令信息处理程序
{
     uint8_t tx_byte_sum=0;
     uint8_t i;
     word_type  para_val;

     gus_SCI1rx_ctrl_abend_timer=0;     //清接收控制信息异常计时器
          
     para_val.ubyte.high = guc_SCI1rx_buffer[4];
     para_val.ubyte.low  = guc_SCI1rx_buffer[3];
     
     gus_lcd_setfreq=para_val.uword;        //频率精度为0.01Hz
		
		 if (gus_lcd_setfreq > gus_MAX_setfreq)
		 {
			    gus_lcd_setfreq = gus_MAX_setfreq;
		 }	     			  	  	                        
     //-------------------------------------------
     if((bflg_host_ctrl==0)&&(gus_lcd_setfreq==0))  //不是上位机控制(即面板控制)，且频率指令为零，则进入
     {
		    if(bflg_trip_stop==1)   //如果接收转速为零且驱动板在故障状态下，则进入
        {
             gus_trip_release_cnt++;
             if(gus_trip_release_cnt>=10)
             {
                 gus_trip_release_cnt=0;          //清故障释放计数器
                 ///bflg_askfor_release_tirp=0;      //清要求故障释放标志位
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
     //else if (ram_para[num_THM_enbale] == 1)  //如果散热器温度有效
     //{
     //	   para_val.uword = gss_Tm + 0x40;
     //}
     guc_SCI1tx_buffer[13]=para_val.ubyte.low;
     //----------------------
     if(bflg_trip_stop==1)    //如果是跳停状态，则进入
     {   
        guc_SCI1tx_buffer[14] = guc_trip_code;
        if(bflg_trip_lock==1) guc_SCI1tx_buffer[12]|=0x80;
     }
     else   //如果不是跳停状态，则去发送状态代码
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
void SCI1tx_on_dealtime(void)  //延时发送计时程序，在1ms定时程序中调用
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
                //----发送时关闭接收------
                *(unsigned char *)&G26ICR = 0x01; //清除SCI1接收中断请求标志  
                G26ICR &= ~0x0100;                //清SCI1接收中断使能标志
		            //------------------------             
                *(unsigned char *)&G26ICR = 0x02; //清除SCI1发送中断请求标志
                G26ICR |= 0x0200;                 //置SCI1发送中断使能标志
                gus_SCI1tx_cnt++;
                SC1TB = guc_SCI1tx_buffer[0];     //发送数据                                           
                //-----------------------
             }
        }
     }
}
//---------------------------------------------------------------------------
void SCI1_tx_int(void)     //SCI1发送中断
{
    gus_SCI1comm_abend_timer = 0;  //清通讯异常计时器
    
    if(bflg_SCI1_tx_busy==1)
    {
        if(gus_SCI1tx_cnt<(guc_SCI1tx_buffer[2]+4))  //如果没发送完去发下一个
        {
            SC1TB = guc_SCI1tx_buffer[gus_SCI1tx_cnt];      //发送数据
            gus_SCI1tx_cnt++;    //发送字节计数
        }
        else
        {
            bflg_SCI1_tx_busy=0;     //清除发送忙标志
            gus_SCI1tx_cnt=0;        //发送字节计数减 
            //--------------------------------------
            //*(unsigned char *)&G26ICR = 0x02;   //清除SCI1发送中断请求标志
            G26ICR &= ~0x0200;          //清SCI1发送中断使能标志
            
            bflg_SCI1_allow_rx = 1;     //置允许接收标志
            gus_SCI1rx_delay_timer = 2; //延时接收计时器赋初值
            //--------------------------------------
        }
    }
}
//-------------------------------------------------------------------------------

#endif 						//end

