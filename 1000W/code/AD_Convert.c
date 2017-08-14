#ifndef	_AD_CONVERT_C_
#define _AD_CONVERT_C_
//------------------------------------------------------------------------------
#include "srMN103SFJ9D.h"
#include "user_typedefine.h"
#include "comm_variable.h"
//#include "ROM_PARA.h"
#include "EEPROM_PARA.h"
#include "AD_Convert.h"
#include "PWM_CTRL.h"
#include "protect_deal.h"
#include "Tsink_tab.h"
//------------------------------------------------------------------------------
//函数声明
void ADC_configure(void);     //AD转换配置程序

void AD_variable_init(void);  //与AD转换相关的变量的初始化程序,在AD初始化配置程序中调用

void AD1_Complete_INT(void);  //AD0转换结束中断处理程序

void get_Idc_ad(void);        //POC的保护、得到Idc的AD值的程序，在AD1中断中调用

void get_Udc_ad(void);        //得到计算Udc的相关AD值

void calc_Iout_delaytime(void);    //计算I呕吐延时程序，在1ms定时程序中调用

void calc_Iout_deal(void);    //每100ms计算一次, 计算三相peak值和Iout值及三相不平衡率,在主循环中调用

void calc_Udc_deal(void);     //计算直流电压,在主循环中调用

void calc_Tm_deal(void);      //计算IPM模块温度的程序，每10ms调用一次

int16_t acquire_Tm(uint16_t ad_value);  //根据得到的a/d转换值查表得到相应的温度值
//------------------------------------------------------------------------------
//标志定义
flag_type flg_ADC;  //AD转换标志位定义
//----------------------------------------------------------
//变量定义
//----------------------------------------------------------
uint16_t  gus_25ms_pwm_cnt;   //25ms对应的PWM计数值 (AD0的转换周期，即一个载波周期计一次数)
//----------------------------------------------------------
//电流计算相关变量
uint16_t  gus_Idc_ad_up;      //PWM上升计数时的Idc AD转换值寄存器
uint16_t  gus_Idc_ad_dw;      //PWM下降计数时的Idc AD转换值寄存器

uint16_t  gus_Idc_ad;         //Idc AD转换值寄存器

int16_t   gss_Idc_ad_max;
uint16_t  gus_Idc_ad_max_sum;
uint16_t  gus_Idc_ad_max_cnt;

int16_t   gss_calc_Iout_delaytimer;

uint16_t  gus_Iout_peak;

uint16_t  gus_Iout;
uint16_t  gus_Iout_for_disp;
//----------------------------------------------------------
//电压计算相关变量
uint16_t  gus_Udc_ad;

uint16_t  gus_Udc_ad_cnt;
uint16_t  gus_Udc_ad_max_buf;
uint16_t  gus_Udc_ad_min_buf;
uint16_t  gus_Udc_ad_max;
uint16_t  gus_Udc_ad_min;

uint32_t  gul_Udc_ad_sum_buf;
uint32_t  gul_Udc_ad_sum;

uint16_t  gus_Udc;            //Udc的电压值(用于内部计算 单位0.1V)
uint16_t  gus_Udc_for_disp;   //用于显示的Udc电压值(用于显示 单位1V)
uint16_t  gus_Udc_tmp;

uint32_t  gul_Udc_ad_maxsum;
uint32_t  gul_Udc_ad_minsum;

uint16_t  gus_Udc_adlim_cnt;
uint16_t  gus_Udc_ad_max_arv;
uint16_t  gus_Udc_ad_min_arv;

uint16_t  gus_Udc_max;
uint16_t  gus_Udc_min;
//----------------------------------------------------------
//温度计算相关参数
uint16_t  gus_Tm_ad;
int16_t   gss_Tm_fault_delaytimer;
uint32_t  gul_Tm_ad_sum;
uint16_t  gus_Tm_ad_cnt;
int16_t   gss_Tm;
//------------------------------------------------------------------------------
void ADC_configure(void)    //AD转换配置程序
{
    //因ANIN00(Idc)  ANIN01(Tm)  ANIN02(Udc)
    
    //AD1选择触发源为A/B交替触发
    //触发B(TM20 compareB),单次多通道转换ANIN00、ANIN01
    //触发A(TM20 compareA),单次多通道转换ANIN00、ANIN02
    //因TM20与TM20CA匹配后，计数器停止，
    //所以在PWM0上升计数时，用TM20CB比较触发AD1;在PWM0下降计数时，用TM20CA比较触发AD1
    
    //TM20的相关寄存器配置
    //--------------------------------------------------------------------------
    //G11ICR：组11中断控制寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  LV2 |  LV1 |  LV0 |  -   |  IE2 |  IE1 |  IE0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  IR2 | IR1  | IR1  |  -   | ID2  | ID1  | ID0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //LV2~LV0 : 中断优先级设置, 0到7(0最高  7禁止)
    //IE2~IE0 : 中断使能标志
    //IR2~IR0 : 中断请求标志
    //ID2~ID0 : 中断检测标志     
    //--------------------------------------------------------------------------
    G11ICR = 0x0000;     //设置TM20的中断优先级为0
    
    //TM20PSC: TM20预分频控制寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |    7    |  6   | 5    |  4   |  3   |  2   |  1   |  0   |  
    // flag    |TMPSCNE20|  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |    0    |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //TMPSCNE20: 预分频操作选择   0=禁止    1=使能
    //--------------------------------------------------------------------------
    TM20PSC=0x00;
    
    //TM20CLKSEL: TM20时钟源选择控制寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |    7  |  6   | 5    |  4   |  3   |  2   |  1   |  0   |  
    // flag    |TM20CLK|  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |    0  |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //TM20CLK: TM20时钟源选择   0=IOCLK    1=MCLK
    //--------------------------------------------------------------------------
    TM20CLKSEL=0x80;     //选择TM20的时钟源为MCLK
    
    //TM20MD: TM20模式寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   |
    // flag    | TMXF |  -   |TMTGE |TMONE |TMCLE |TMCGE |   TMUD1~0   |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |
    // flag    |TMCNE |TMLDE |  -   |  -   |  -   |       TMCK2~0      |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //TMXF:  TM20运转状态显示   0=计数停止    1=计数运转中
    //TMTGE: TM20外部触发使能选择  0=通过PWM0禁止TM20运转   1=通过PWM0激活TM20运转 (必须使TM20MDA.TMAEG=1)
    //TMONE: TM20单次运转使能选择  0=禁止(TM20计数器不停止) 1=使能(当TM20CA和TM20CB比较到停止)
    //       注意：只要跟TM20CA匹配，则计数器停止计数
    //TMCLE: TM20计数清除操作使能选择  0=禁止清除运转   1=使能清除运转
    //       注意：选1时,只要跟TM20CA匹配，则计数器清零   
    //TMCGE: 保留 一直为零
    //TMUD1~0: TM20上/下计数选择   00=向上计数  01=向下计数  10,11=禁止设置
    //TMCNE: TM20运转旋转   0=运转禁止   1=运转使能
    //TMLDE: TM20初始化    0=正常运转    1=初始化(TM20BC=0x0000) 
    //(1:当TM20CA和TM20CB设置为双缓冲,从buffer传输到比较寄存器时，这个值被加载，引脚被初始化)
    //TMCK2~0: 时钟计数源选择     000=IOCLK, MCLK         001=IOCLK/8, MCLK/8 (选择此项时，TM20PSC必须使能)
    //                            010=Timer 6 underflow   011=Timer 7 underflow  其他禁止
    //--------------------------------------------------------------------------
    TM20MD = 0x3800;     //通过PWM0激活TM20运转，当TM20CA和TM20CB比较到停止，向上计数，运转使能
    
    //TM20MDA: TM20比较A模式寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   | 5    |  4   |  3   |  2   |  1   |  0   |  
    // flag    |   TMAM1~0   |TMAEG |TMACE |  -   |  -   |  TMA01~00   | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //TMAM1~0: TM20比较/捕获A模式选择   00=比较寄存器(双缓冲)  01=比较寄存器(单缓冲)  10,11=禁止设置
    //TMAEG: TM20比较Apin极性选择    0=下降沿  1=上升沿   (通过PWM0激活TM20时，必须设置为1)
    //TMACE:  保留 为0
    //TMA01~00: 保留 为0
    //--------------------------------------------------------------------------
    TM20MDA = 0x20;
    
    //TM20MDB: TM20比较B模式寄存器 (与TM20MDA类似)
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   | 5    |  4   |  3   |  2   |  1   |  0   |  
    // flag    |   TMBM1~0   |TMBEG |TMBCE |  -   |  -   |  TMB01~00   | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //TMAM1~0: TM20比较/捕获B模式选择   00=比较寄存器(双缓冲)  01=比较寄存器(单缓冲)  10,11=禁止设置
    //TMAEG:  保留 为0
    //TMACE:  保留 为0
    //TMA01~00: 保留 为0
    //--------------------------------------------------------------------------
    TM20MDB = 0x00;
    
    //TM20CA: TM20比较A寄存器
    //TM20CB: TM20比较B寄存器
    //注意：必须TM20CB < TM20CA
    //TM20BC: TM20计数器
    
    //--------------------------------------------------------------------------
    //引脚模式配置
    PCMD |= 0x07;   //PC0,PC1,PC2 is AD (0=IO 1=AD)
    //--------------------------------------------------------------------------
    //G20ICR：组20中断控制寄存器(AD1转换完中断、AD1触发B转换完)
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  LV2 |  LV1 |  LV0 |  -   |  -   |  IE1 |  IE0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | IR1  | IR1  |  -   |  -   | ID1  | ID0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //LV2~LV0 : 中断优先级设置, 0到7(0最高  7禁止)
    //IE1~IE0 : 中断使能标志
    //IR1~IR0 : 中断请求标志
    //ID1~ID0 : 中断检测标志     
    //--------------------------------------------------------------------------
    G20ICR = 0x2000;    //设置AD1的中断优先级为2 (低于PWM)
    
    //AN1CTR0: AD1转换控制寄存器0
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   | 5   |  4   |  3 |  2   |  1   |  0   |  
    // flag    |  -   |  -   | AN1SHC1~0  |     AN1CK2~0     |AN1OFF| 
    //at reset |  0   |  0   |  0  |  0   |  0 |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //AN1SHC1~0: AD1采样/保持周期    00=1   01=2   10=4   11=6  cycles
    //AN1CK2~0: AD1转换时钟   000=IOCLK   001=IOCLK/2   010=IOCLK/3  011=IOCLK/4 
    //                        100=IOCLK/8 101=IOCLK/16  110、111禁止设置
    //AN1OFF: AD1掉电模式选择    0=掉电模式    1=正常操作模式
    //--------------------------------------------------------------------------
    AN1CTR0 = 0x0022;  //AN1SHC1~0=10b,AN1CK2~0=001b,AN1OFF=0b
    
    //AN1CTR1A: AD1转换控制寄存器1A
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |     AN1ANCH2~0     |  -   |     AN1ACH2~0      |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   | 5    |  4   |  3   |  2   |  1    |  0   |  
    // flag    |  -   |  -   |  -   |  -   |  AN1AMD1~0  |AN1ATRG|AN1AEN| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0    |  0   |
    //--------------------------------------------------------------------------
    //在A触发时的设置 
    //AN1ANCH2~0: AD1在多通道转换模式下,转换结束的通道号   000=ADIN00 ... 111=ADIN07
    //AN1ACH2~0:  AD1在单通道转换模式下,转换的通道号   或
    //            AD1在多通道转换模式下,转换起始的通道号  000=ADIN00 ... 111=ADIN07
    //            转换过程中读,则为正在转化的通道号
    //AN1AMD1~0:  AD1转换模式选择  00=单次单通道转换  01=单次多通道转换
    //                             00=循环单通道转换  01=循环多通道转换
    //AN1ATRG:   A触发AD1转换启动使能选择  0=禁止  1=使能
    //AN1AEN:   AD1转换启动执行标志   0=未开始   1=启动或转换中
    //--------------------------------------------------------------------------
    AN1CTR1A = 0x6406;   //AN1ANCH2~0=110b,AN1ACH2~0=100b,AN1AMD1~0=01b,AN1ATRG=1,AN1AEN=0
    
    //AN1CTR1B:: AD1转换控制寄存器1B
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   | AN1BNCH1~0  |  -   |     AN1BCH2~0      |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   | 5    |  4   |  3   |  2   |  1    |  0   |  
    // flag    |  -   |  -   |  -   |  -   |  AN1BMD1~0  |AN1BTRG|AN1BEN| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0    |  0   |
    //--------------------------------------------------------------------------
    //在B触发时的设置
    //AN1BNCH1~0: AD1在多通道转换模式下,转换结束的通道号   00=ADIN00 ... 11=ADIN03
    //AN1BCH2~0:  AD1在单通道转换模式下,转换的通道号   或
    //            AD1在多通道转换模式下,转换起始的通道号
    //            在单通道转换模式下, 000=ADIN00 ... 111=ADIN07,转换结果存在AN1BUFB0中
    //            在多通道转换模式下, x00=ADIN00 ... x11=ADIN03,转换结果根据对应通道存在AN1BUFB3~0中
    //AN1BMD1~0:  AD1转换模式选择  00=单次单通道转换  01=单次多通道转换
    //                             00=循环单通道转换  01=循环多通道转换
    //AN1BTRG:   B触发AD1转换启动使能选择  0=禁止  1=使能
    //AN1BEN:   AD1转换启动执行标志   0=未开始   1=启动或转换中
    //--------------------------------------------------------------------------
    AN1CTR1B = 0x1006; //AN1BNCH1~0=010b,AN1BCH2~0=000b,AN1BMD1~0=01b,AN1BTRG=1,AN1BEN=0
    
    //AN1CUTA: AD1转换通道剔除寄存器A
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   | 5    |  4   |  3   |  2   |  1   |  0   |  
    // flag    |            AN1ACUT07        ~      AN1ACUT00          | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //AN1ACUT07~00: AD1在多通道转换模式下通道0x剔除转换选择   0=转换   1=剔除
    //--------------------------------------------------------------------------   
    AN1CUTA = 0x0020;   //AD1IN01剔除
    
    //AN1CUTB:: AD1转换通道剔除寄存器B
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   | 5    |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  AN1BCUT03 ~  AN1BCUT00   |  -   |  -   |  -   |  -   | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //AN1BCUT03~00: AD1在B触发多通道转换模式下通道0x剔除转换选择   0=转换   1=剔除
    //--------------------------------------------------------------------------
    AN1CUTB = 0x0000;  //未用
    
    //ADST0:: AD0启动转换触发源选择寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   | 5    |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | AD0BST1~0   |  -   |  -   |  AD0AST1~0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //AD0BST1~0: AD0的B触发源选择   00=PWM overflow        01=PWM underflow 
    //AD0AST1~0: AD0的A触发源选择  10=Timer 20 compare A   11=Timer 20 compare B 
    //--------------------------------------------------------------------------
    //ADST0=0x02;  //AD0BST1~0=00(未用),AD0AST1~0=10,AD0的A触发源为Timer 20 compare A 
    
    //ADST1:: AD1启动转换触发源选择寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   | 5    |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | AD1BST1~0   |  -   |  -   |  AD1AST1~0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //AD1BST1~0: AD1的B触发源选择   00=PWM overflow        01=PWM underflow 
    //AD1AST1~0: AD1的A触发源选择  10=Timer 20 compare A   11=Timer 20 compare B 
    //--------------------------------------------------------------------------
    ADST1 = 0x32;  //AD1BST1~0=11,AD1的B触发源为Timer 20 compare B, AD1AST1~0=10,AD1的A触发源为Timer 20 compare A
    
    //AN0CTREGA: 启动AD0转换的触发A次数寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   | 5    |  4   |  3   |  2   |  1   |  0   |  
    // flag    |       CNT3I~CNT3I         |         CNT3~CNT0         | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //CNT3I~CNT0I: AD0首次启动转换时的A触发源次数   1111=15   0000=0
    //CNT3~CNT0: AD0后续启动转换时的A触发源次数  1111=15   0000=0 
    //--------------------------------------------------------------------------
    //AN1CTREGA: 启动AD1转换的触发A次数寄存器 (定义与AN0CTREGA相似)	
    AN1CTREGA = 0x00;  //每次触发都启动AD0转换
    
    //AN1CTREGA: 启动AD1转换的触发B次数寄存器 (定义与AN0CTREGA相似)	  
    AN1CTREGB = 0x00;	//每次触发都启动AD0转换	
    
    //AN0BUF00~AN0BUF07: 对应存放ADIN00~ADIN07的AD0转换结果
    //AN0BUFB0：在单通道模式下,通过B触发,所选引脚的AD0转换结果
    //          在多通道模式下,通过B触发,ADIN04的AD0转换结果
    //AN0BUFB1 ~AN0BUFB3: 对应存放ADIN05~ADIN07在多通道模式下,通过B触发的AD0转换结果
    
    //AN1BUF00 ~AN1BUF07:: 对应存放ADIN00~ADIN07的AD1转换结果
    //AN1BUFB0: 在单通道模式下,通过B触发,所选引脚的AD1转换结果
    //          在多通道模式下,通过B触发,ADIN00的AD1转换结果
    //AN0BUFB1 ~AN0BUFB3: 对应存放ADIN01~ADIN03在多通道模式下,通过B触发的AD1转换结果
    //--------------------------------------------------------------------------
    AD_variable_init();       //调用与AD模块转换相关的变量初始化程序
    //--------------------------------------------------------------------------
    //开启AD1转换中断
    *(unsigned char *)&G20ICR = 0x03;   //清除AD1请求标志
    G20ICR |= 0x0100;    //置AD1中断使能标志    
    //--------------------------------------------------------------------------
    AN1CTR0 |=0x01;      //AD1进入运转模式
    //--------------------------------------------------------------------------
}
//------------------------------------------------------------------------------
void AD_variable_init(void)    //与AD转换相关的变量的初始化程序  //在AD初始化配置程序中调用
{
    uint32_t lul_tmp;
    //------------------------------------------------------
    //因每个载波周期累加一次Udc，所以每25ms对应的计数值如下计算
    //(25/1000)/(1/fc)=((1/40)/(1/carrier*100))=carrier*5/2
    lul_tmp = ram_para[num_pwm_carrier] * 5;
    lul_tmp >>= 1;
    gus_25ms_pwm_cnt = lul_tmp;
    //------------------------------------------------------
    //Idc相关变量初始化
    gus_Idc_zero_ad = ram_para[num_Iout_CT_zero_AD];   //得到Idc零点AD值
    
    gus_Idc_ad_up = gus_Idc_zero_ad;   //PWM载波上升时，Idc的AD初值
    gus_Idc_ad_dw = gus_Idc_zero_ad;   //PWM载波下降时，Idc的AD初值
    gus_Idc_ad = gus_Idc_zero_ad;
    
    gss_Idc_ad_max = gus_Idc_zero_ad;
    gus_Idc_ad_max_sum = 0;
    gus_Idc_ad_max_cnt = 0;
    
    gus_Iout_peak = 0;
    gus_Iout = 0;
    gus_Iout_for_disp = 0; 
    //------------------------------------------------------
    //Udc相关变量初始化
    gus_Udc_ad = 0;
    
    gus_Udc_ad_cnt = 0;            //Udc的AD值累计的计数器
    gus_Udc_ad_max_buf = 0;        //Udc的AD值的最大值缓冲寄存器
    gus_Udc_ad_min_buf = 1023;     //Udc的AD值的最小值缓冲寄存器
    gus_Udc_ad_max = 0;            //Udc的AD值的最大值寄存器
    gus_Udc_ad_min = 0;            //Udc的AD值的最小值寄存器
    
    gul_Udc_ad_sum_buf = 0;        //Udc的AD值累计器的缓冲器
    gul_Udc_ad_sum = 0;            //Udc的AD值累计器
    
    gus_Udc_adlim_cnt = 0;         //Udc的AD值极值累计的计数器     
    gul_Udc_ad_maxsum = 0;         //Udc的AD值的最大值累加器
    gul_Udc_ad_minsum = 0;         //Udc的AD值的最小值累加器
    gus_Udc_ad_max_arv = 0;        //Udc的AD值的平均最大值寄存器
    gus_Udc_ad_min_arv = 0;        //Udc的AD值的平均最小值寄存器
    
    gus_Udc = 0;                   //Udc的电压值(用于内部计算 单位0.1V)
    gus_Udc_for_disp = 0;          //用于显示的Udc电压值(用于显示 单位1V)
    gus_Udc_max = 0;               //Udc电压最大值(用于保护的判断 单位1V)
    gus_Udc_min = 0;               //Udc电压最小值(用于保护的判断 单位1V)     
    //------------------------------------------------------
    //Tm相关变量初始化
    gus_Tm_ad = 0;
    gul_Tm_ad_sum = 0;
    gus_Tm_ad_cnt = 0;
    gss_Tm = 0;     
    //------------------------------------------------------
}
//--------------------------------------------------------------------------
//AD1在B触发时，多通道转换，AN0BUFB0对应ADIN00(Idc),AN0BUFB1对应ADIN01(Vdc)
//AD1在A触发时，多通道转换，AN0BUF00对应ADIN00(Idc),AN0BUF03对应ADIN03(THM)
//--------------------------------------------------------------------------
void AD1_Complete_INT(void)   //AD1转换结束中断处理程序
{
    //------------------------------------------------------
    //Idc的相关处理
    gus_Idc_ad_up = AN1BUFB0;
    gus_Idc_ad_dw = AN1BUF00;
    //debug
    //gus_Idc_ad_up = 512;
    //gus_Idc_ad_dw = 512;
    //------------------------------------------------------
    //获得电流AD值
    if (bflg_allow_get_Idc == 1)        //如果可以采集电流AD
    {
    	  if (bflg_up_down_get_Idc == 0)       //如果是下降沿采集
        {
    	      gus_Idc_ad = gus_Idc_ad_dw;
        }
        else       //如果是上升沿采集
        {
        	  gus_Idc_ad = gus_Idc_ad_up;
        }
        
        prot_Iout_POC_deal(); //Iout瞬时保护处理程序
        
        get_Idc_ad();      //得到Idc的AD值的程序
    }
    //------------------------------------------------------
    //获得Tm、Udc的AD值
    gus_Tm_ad = AN1BUFB1;
    gus_Udc_ad = AN1BUF02;
    //debug
    //gus_Tm_ad = 1000;
    //gus_Udc_ad = 650;
    //------------------------------------------------------
    prot_Udc_POU_deal();      //Udc瞬时保护处理程序
    //------------------------------------------------------
}
//------------------------------------------------------------------------------
void get_Idc_ad(void)    //POC的保护、得到Idc的AD值的程序，在AD1中断中调用
{
    int16_t lss_Idc_ad;
        
    if (bflg_running == 1)   
    {
        lss_Idc_ad = gus_Idc_ad - gus_Idc_zero_ad;
        if (lss_Idc_ad < 0)
        {
        	  lss_Idc_ad = 0;
        }
        //------------------------------
        //得到Idc最大的AD值
        if (gss_Idc_ad_max < lss_Idc_ad)
        {
            gss_Idc_ad_max = lss_Idc_ad;
        }
    }
    else
    {
    	  gss_Idc_ad_max = 0;
    }
}
//------------------------------------------------------------------------------
void get_Udc_ad(void)    //得到计算Udc的相关AD值，PWM波形发生中断中调用
{
    //------------------------------------------------------
	  //每25ms计算一次Udc的AD平均值
    gul_Udc_ad_sum_buf += gus_Udc_ad;
    //每25ms计算一次Udc的AD最大值和最小值
    if (gus_Udc_ad_max_buf < gus_Udc_ad) gus_Udc_ad_max_buf = gus_Udc_ad;
    if (gus_Udc_ad_min_buf > gus_Udc_ad) gus_Udc_ad_min_buf = gus_Udc_ad;
    //------------------------------------------------------
    gus_Udc_ad_cnt++;
    if (gus_Udc_ad_cnt >= gus_25ms_pwm_cnt)   //25ms计数到，则进入
    {
        gul_Udc_ad_sum = gul_Udc_ad_sum_buf;      //将累加和缓冲器的值赋给累加和寄存器
        gul_Udc_ad_sum_buf = 0;  //清累加和寄存器的缓冲器
        gus_Udc_ad_cnt = 0;      //清计数器
         
        gus_Udc_ad_max = gus_Udc_ad_max_buf;    //得到当前Udc的AD最大值
        gus_Udc_ad_min = gus_Udc_ad_min_buf;    //得到当前Udc的AD最小值
        gus_Udc_ad_max_buf = 0;                 //buffer赋初值
        gus_Udc_ad_min_buf = 1023;
        //--------------------------------
        bflg_allow_calc_Udc = 1;    //每25ms，置要求计算Udc标志
        //--------------------------------
    }
    //------------------------------------------------------
    
}
//-------------------------------------------------------------------------------
void calc_Iout_delaytime(void)     //计算Iout延时程序，在1ms定时程序中调用
{
	  if (bflg_allow_calc_Iout == 0)      //如果未允许计算Iout
	  {
	  	  gss_calc_Iout_delaytimer++;
	  	  if (gss_calc_Iout_delaytimer >= 30)  //30ms(最小频率6.67Hz时，25ms可采得最大电流值)
	  	  {
	  	  	  gss_calc_Iout_delaytimer = 0;
	  	  	  bflg_allow_calc_Iout = 1;
	  	  }
	  }
}
//-------------------------------------------------------------------------------
void calc_Iout_deal(void)    //计算输出电流，在主循环中调用
{
    int32_t lsl_tmp;
   
    if (bflg_running == 1)
    {
        gus_Idc_ad_max_sum += gss_Idc_ad_max;
        gss_Idc_ad_max = 0;
        
        gus_Idc_ad_max_cnt++;
        if (gus_Idc_ad_max_cnt >= 32)
        {
        	  lsl_tmp = (gus_Idc_ad_max_sum >> 5);
        	  gus_Idc_ad_max_sum = 0;
        	  gus_Idc_ad_max_cnt = 0;
        	  
        	  lsl_tmp -= ram_para[num_Iout_ref_AD];
        	  lsl_tmp *= ram_para[num_Iout_gain];
        	  lsl_tmp >>= ram_para[num_Iout_gain_amp];
        	  lsl_tmp += ram_para[num_Iout_ref];
        	  
        	  //得到峰值
        	  if(lsl_tmp < 0) lsl_tmp = 0;
        	  
        	  gus_Iout_peak = lsl_tmp / 10;    //单位：0.1A
        	  
        	  //计算有效值
        	  lsl_tmp *= 181;
            lsl_tmp >>= 8;
            if(lsl_tmp < 0) lsl_tmp = 0;
            
            gus_Iout = (uint16_t)lsl_tmp;
            gus_Iout_for_disp = gus_Iout / 10;    //单位：0.1A
        }
    }
    else
    {
        gus_Idc_ad_max_sum = 0;
        gus_Idc_ad_max_cnt = 0;
        
        gus_Iout_peak = 0;
        gus_Iout = 0;
        gus_Iout_for_disp = 0;
    }
}
//---------------------------------------------------------------------------------
void calc_Udc_deal(void)     //计算直流电压,在主循环中调用
{
    int32_t lsl_tmp;
    //------------------------------------------------------
    lsl_tmp = gul_Udc_ad_sum / gus_25ms_pwm_cnt;  //得到Udc的AD平均值
    lsl_tmp -= ram_para[num_Udc_ref_AD];
    lsl_tmp *= 10;                                //乘以10将电压精度扩大到0.1V
    lsl_tmp *= ram_para[num_Udc_gain];
    lsl_tmp >>= ram_para[num_Udc_gain_amp];
    lsl_tmp += (ram_para[num_Udc_ref]*10);        //Udc电压精度为0.1V
    //------------------------------------------------------
    lsl_tmp += gus_Udc;
    lsl_tmp >>= 1;  //1/2根木滤波
    
    if (lsl_tmp < 1000) lsl_tmp = 1000; //防止溢出
    gus_Udc = lsl_tmp;                  //单位 0.1V
    //------------------------------------------------------
    //单位电压调制率＝[1/(直流电压/sqrt(2))],得到单位调制率的*(2^16)*4096倍,电压精度到0.1V
    //percent_pwr_unit = 4096*(2^16)/[(Udc_val)/sqrt(2)]
    //gul_percent_pwm_unit = 379625062 / gus_Udc;
    //------------------------------------------------------
    //得到用于保护和显示的当前电压平均值
    gus_Udc_for_disp = gus_Udc / 10;    //单位 1V
    //-----------------------------------------------------
    //计算Udc最大值和最小值的平均值
    gul_Udc_ad_maxsum += gus_Udc_ad_max;
    gul_Udc_ad_minsum += gus_Udc_ad_min;
    gus_Udc_adlim_cnt++;
    if(gus_Udc_adlim_cnt >= 64)
    {
        gus_Udc_adlim_cnt = 0;
        gus_Udc_ad_max_arv = gul_Udc_ad_maxsum >> 6;
        gus_Udc_ad_min_arv = gul_Udc_ad_minsum >> 6;
        gul_Udc_ad_maxsum=0;
        gul_Udc_ad_minsum=0;
        
        lsl_tmp = gus_Udc_ad_max_arv;
        lsl_tmp -= ram_para[num_Udc_ref_AD];
        lsl_tmp *= ram_para[num_Udc_gain];
        lsl_tmp >>= ram_para[num_Udc_gain_amp];
        lsl_tmp += ram_para[num_Udc_ref];
        if(lsl_tmp<0) lsl_tmp = 0; 
        gus_Udc_max = lsl_tmp;     //得到直流母线峰值电压,用于过压、失速保护 (单位 1V)
        
        lsl_tmp = gus_Udc_ad_min_arv;
        lsl_tmp -= ram_para[num_Udc_ref_AD];
        lsl_tmp *= ram_para[num_Udc_gain];
        lsl_tmp >>= ram_para[num_Udc_gain_amp];
        lsl_tmp += ram_para[num_Udc_ref];                        
        if(lsl_tmp<0) lsl_tmp = 0;
        gus_Udc_min = lsl_tmp;     //得到直流母线谷值电压,用于欠压保护 (单位 1V)
    }
}
//-------------------------------------------------------------------------------
void calc_Tm_deal(void)       //计算散热器温度的程序，每10ms调用一次
{
    uint16_t lus_ad_val;
    
    //------------------------------------------------------
    //Tm温度传感器故障判断
    if (ram_para[num_Tm_enbale] == 1)
    {
    	  if ((gus_Tm_ad < ram_para[num_THM_lowlim_AD]) || (gus_Tm_ad > ram_para[num_THM_uplim_AD])) //如果Tm的AD值不在正常范围内
        //if ((bflg_running == 1) && (gus_Tm_ad > ram_para[num_THM_uplim_AD]))    //如果运行阶段，且Tm的AD值一直大，表明温度一直很小
        {
    	      gss_Tm_fault_delaytimer++;
    	      if (gss_Tm_fault_delaytimer >= 500)  //5s
    	      {
    	  	      gss_Tm_fault_delaytimer = 0;
    	  	      if (bflg_Tm_fault == 0)
    	  	      {
    	  	  	      bflg_Tm_fault = 1;
    	  	  	      trip_stop_deal(Tm_FAULT_CODE);
    	  	      }
    	      }
        }
        else
        {
    	      gss_Tm_fault_delaytimer = 0;
    	      
    	      if (bflg_running == 0) //故障停机标志清除后，再清除故障标志
    	      {
    	          bflg_Tm_fault = 0;
            }
        }
    }
    //------------------------------------------------------
    gul_Tm_ad_sum += gus_Tm_ad;
    gus_Tm_ad_cnt++;
    if(gus_Tm_ad_cnt >= 64)
    {
        gus_Tm_ad_cnt = 0;
        
        lus_ad_val = (gul_Tm_ad_sum >> 6);   //求64次算术平均值
        gul_Tm_ad_sum = 0;
        
        gss_Tm  = acquire_Tm(lus_ad_val) + ram_para[num_THM_ref];//调用查表程序得到IPM模块温度值
    }
}
//-------------------------------------------------------------------------------
int16_t acquire_Tm(uint16_t ad_value)   //根据得到的a/d转换值查表得到相应的温度值
{
	  uint16_t lus_index = (TM_TAB_LENGTH / 2);     //从温度表的中间位置开始查表
	  
    if (ad_value > ad_to_Tm[lus_index])
    {
        do
        {
            lus_index++;
            if (lus_index >= TM_TAB_LENGTH)
            {
                lus_index = TM_TAB_LENGTH;
                break;
            }
            if (ad_value < ad_to_Tm[lus_index])
            {
                lus_index--;
                break;
            }
        }
        while (ad_value > ad_to_Tm[lus_index]);
    }
    else
    {
        do
        {
            lus_index--;
            if (lus_index <= 0)
            {
                lus_index = 0;
                break;
            }
            if (ad_value >= ad_to_Tm[lus_index])
            {
                break;
            }
        }
        while (ad_value < ad_to_Tm[lus_index]);
    }  
    
    return Tm_for_ad[lus_index];	 
}
//-------------------------------------------------------------------------------
#endif
