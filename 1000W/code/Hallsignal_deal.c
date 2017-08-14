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
void Hall_configure(void);         //霍尔信号口配置程序

void motor_var_init(void);         //电机相关变量程序化，在Hall初始化和通讯ID变更程序中调用

void Hall_input_int(void);         //Hall信号输入中断服务程序

void Hall_updata_deal(void);       //霍尔信号更新处理程序,在主循环中调用

void Hall_fault_delaytime(void);   //霍尔信号故障延时程序，在10ms定时程序中调用

void Hall_error_delaytime(void);   //霍尔信号错延时程序，加入10ms定时程序中

void Hall_direction_delaytime(void);      //霍尔信号方向错延时程序，加入10ms定时程序中

void Hall_off_delaytime(void);     //霍尔信号停止延时程序，在10ms定时程序中调用
//------------------------------------------------------------------------------
flag_type flg_Hall;
//------------------------------------------------------------------------------
uint16_t  PhaseValues_CCW[6]; //逆时针旋转，正弦波驱动时的区间对应的起始相位角
uint16_t  PhaseValues_CW[6];  //顺时针旋转，正弦波驱动时的区间对应的起始相位角
//------------------------------------------------------------------------------
const int8_t gsc_sector_table[] = {-1, 0, 2, 1, 4, 5, 3, -1};    //霍尔信号区间表

lword_type Hall_trigger_time;      //霍尔信号触发时刻寄存器

uint32_t  gul_Hall_trigger;        //霍尔信号触发器
uint32_t  gul_Hall_past_trigger;   //上次霍尔信号触发器

uint32_t  gul_Hall_delt_trigger;
uint32_t  gul_Hall_past_delt_trigger;
uint32_t  gul_Hall_delt_trigger_buffer; //霍尔信号时间差缓冲器

uint16_t  gus_Hall_value;          //霍尔信号触发值
uint16_t  gus_Hall_past_value;     //上次霍尔信号触发值
uint16_t  gss_Hall_value_buffer;   //霍尔信号触发值缓冲器

int8_t    gsc_Hall_sector;         //霍尔信号扇区值
int8_t    gsc_Hall_past_sector;    //上次霍尔信号扇区值

uint16_t  gus_same_direction_cnt;  //转向相同计数器
uint16_t  gus_dif_direction_cnt;   //转向不同计数器

uint16_t  gus_Hall_watchdog;       //霍尔信号看门狗
//----------------------------------------------------------
uint16_t  gus_same_HU_cnt;
uint16_t  gus_same_HV_cnt;
uint16_t  gus_same_HW_cnt;
uint16_t  gus_Hall_same_cnt;

uint16_t  gus_stall_rate;

//f=1/T=1/6*detl_t=1/(6*delt_cnt/20M)=(20M/6)/delt_cnt  (Hz)=[(20M/6)/delt_cnt]*100 (0.01Hz)
#define   FREQ_FACTOR    ((20000000 / 6) * 100)    //计算当前频率的计算因子 (精确到0.01Hz)

uint16_t  gus_Hall_freq;
uint16_t  gus_Hall_freq_mod;

uint16_t  gus_Hall_speed;

uint16_t  gus_sensor_startup_timer; //开机测速阶段计时器
//------------------------------------------------------------------------------
uint16_t  gus_avr_realfreq_cnt;
uint32_t  gul_avr_realfreq_sum;
uint16_t  gus_avr_realfreq;

uint16_t  gus_freq_array_len; //实际频率数组长度

uint16_t  gus_realfreq_array[50]; //用于计算频率平均值的数组
//------------------------------------------------------------------------------
uint16_t  gus_motor_overspeed;     //电机超速频率

int16_t   gss_Hall_fault_delaytimer;

uint16_t  gus_phase_adv_degree;
//------------------------------------------------------------------------------
uint8_t   guc_motor_ID;
//------------------------------------------------------------------------------
void Hall_configure(void)     //霍尔信号口配置
{
    //------------------------------------------------------
    //将8bit计数器timer4~7串接为32bit计数器，用于霍尔信号测速 
    //------------------------------------------------------
    //G5ICR：组5中断控制寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  LV2 |  LV1 |  LV0 |  -   |  -   |  IE1 |  IE0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | IR1  | IR1  |  -   |  -   | ID1  | ID0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    G5ICR = 0x0000;      //禁止timer5,timer4下溢中断
    G6ICR = 0x0000;      //禁止timer7,timer6下溢中断
    
    TM7MD &= 0x7F;       //禁止TM7BC计数
    TM6MD &= 0x7F;       //禁止TM6BC计数
    TM5MD &= 0x7F;       //禁止TM5BC计数
    TM4MD &= 0x7F;       //禁止TM4BC计数  
    
    //TM47PSC：预分频控制寄存器1
    //--------------------------------------------------------------------------
    //reg_bit  |   7    |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |TMPSCNE1|  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |   0    |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //TMPSCNE1:预分频操作使能选择  0=禁止  1=使能
    //--------------------------------------------------------------------------
    TM47PSC = 0x00;
    
    //TM47EXPSC：外部预分频控制寄存器1
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0    |  
    // flag    |TM7IN |TM6IN |TM5IN |TM4IN |  -   |  -   |  -   |EXPSCNE1| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0    |
    //--------------------------------------------------------------------------
    //TM7IN:T7计数时钟源选择  0=TM7IO pin input  1=IOCLK/128
    //TM6IN:T6计数时钟源选择  0=TM6IO pin input  1=IOCLK/128
    //TM5IN:T5计数时钟源选择  0=TM5IO pin input  1=IOCLK/128
    //TM4IN:T4计数时钟源选择  0=TM4IO pin input  1=IOCLK/128
    //EXPSCNE1:预分频(1/128)操作使能选择  0=禁止  1=使能
    //--------------------------------------------------------------------------
    TM47EXPSC = 0x00;   
    
    //TM4BR: T4基本寄存器 (TM7BR,TM6BR,TM5BR与TM4BR类似)
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM4BR7|TM4BR6|TM4BR5|TM4BR4|TM4BR3|TM4BR2|TM4BR1|TM4BR0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM4BR7~0: 用于在TM4BC下溢时,进行计数的初值。	TM4BC下溢后,从TM4BR加载到TM4BC中。
    //--------------------------------------------------------------------------
    TM4BR = 0xFF;
    TM5BR = 0xFF;
    TM6BR = 0xFF;
    TM7BR = 0xFF;
    
    //TM4MD: T4模式寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM4CNE|TM4LDE|  -   |  -   |  -   |TM4CK2|TM4CK1|TM4CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM4CNE: 计数器运转使能选择  0=不运转  1=运转
    //TM4LDE: 计数器初始化   0=正常运转   1=初始化
    //TM4CK2~0: 计数时钟源选择  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=禁止设置   
    //          100=禁止设置   101=T5下溢    110=T6下溢     111=TM4IO pin输入(上升沿),IOCLK/128
    //--------------------------------------------------------------------------
    TM4MD = 0x00;
     
    //TM5MD: T5模式寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM5CNE|TM5LDE|  -   |  -   |  -   |TM5CK2|TM5CK1|TM5CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM5CNE: 计数器运转使能选择  0=不运转  1=运转
    //TM5LDE: 计数器初始化   0=正常运转   1=初始化
    //TM5CK2~0: 计数时钟源选择  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=与T4级连   
    //          100=T4下溢   101=禁止设置    110=T6下溢     111=TM5IO pin输入(上升沿),IOCLK/128
    //--------------------------------------------------------------------------
    TM5MD = 0x03;
    
    //TM6MD: T6模式寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM6CNE|TM6LDE|  -   |  -   |  -   |TM6CK2|TM6CK1|TM6CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM6CNE: 计数器运转使能选择  0=不运转  1=运转
    //TM6LDE: 计数器初始化   0=正常运转   1=初始化
    //TM6CK2~0: 计数时钟源选择  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=与T5级连   
    //          100=T4下溢   101=T5下溢    110=禁止设置     111=TM6IO pin输入(上升沿),IOCLK/128
    //--------------------------------------------------------------------------
    TM6MD = 0x03;
    
    //TM7MD: T7模式寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM7CNE|TM7LDE|  -   |  -   |  -   |TM7CK2|TM7CK1|TM7CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM7CNE: 计数器运转使能选择  0=不运转  1=运转
    //TM7LDE: 计数器初始化   0=正常运转   1=初始化
    //TM7CK2~0: 计数时钟源选择  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=与T6级连   
    //             100=T4下溢   101=T5下溢    110=T6下溢     111=TM7IO pin输入(上升沿),IOCLK/128
    //--------------------------------------------------------------------------
    TM7MD = 0x03;   
    //----------------------------------
    TM4MD |= 0x40;  //初始化TM0BC
    TM5MD |= 0x40;  //初始化TM1BC
    TM6MD |= 0x40;  //初始化TM2BC
    TM7MD |= 0x40;  //初始化TM3BC
    
    TM4MD &= 0xBF;  //清初始化TM0BC标志
    TM5MD &= 0xBF;  //清初始化TM1BC标志
    TM6MD &= 0xBF;  //清初始化TM2BC标志
    TM7MD &= 0xBF;  //清初始化TM3BC标志   
    
    TM7MD |= 0x80;    //启动TM3BC计数
    TM6MD |= 0x80;    //启动TM3BC计数
    TM5MD |= 0x80;    //启动TM3BC计数
    TM4MD |= 0x80;    //启动TM3BC计数   
    //----------------------------------
    //禁止霍尔信号中断
    G29ICR = 0x0000;
    G30ICR = 0x0000;
    //--------------------------------------------------------------------------
    //NFCLK2: 采样时钟设置寄存器2
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | -    |  -  |  P41NFCK1~0 |  P40NFCK1~0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    NFCLK2 = 0x000F;     //P40,P41 选择 1/32 IOCLK
    
    //--------------------------------------------------------------------------
    //NFCLK3: 采样时钟设置寄存器3
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  P61NFCK1~0 |  P60NFCK1~0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | -    |   -  |  P51NFCK1~0 |  P50NFCK1~0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    NFCLK3 = 0x0003;     //P50 选择 1/32 IOCLK
    
    //--------------------------------------------------------------------------
    //NFCNT1: 滤波控制寄存器1
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
    IRQEDGESEL |= 0x0700;     //IRQ10,IRQ9,IRQ8上下沿触发使能  
    //--------------------------------------------------------------------------
    //EXTMD1:外部中断条件设定寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  IR11TG1~0  |  IR10TG1~0  |  IR09TG1~0  |  IR08TG1~0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //IRxTG1~0 : IRQx引脚触发条件设定, 00=上升沿触发 01=下降沿触发 10=H电平触发 11=L电平触发
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
    //清中断请求标志
    *(unsigned char *)&G29ICR = 0x03;  //清除外中断IRQ09、IRQ08请求标志
    *(unsigned char *)&G30ICR = 0x01;  //清除外中断IRQ10请求标志   
    //--------------------------------------------------------------------------
    //G29ICR：组29中断控制寄存器(控制外中断IRQ09、IRQ08)
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
    //IE2~IE0 : 中断使能标志     IE1=IRQ09   IE0=IRQ08
    //IR2~IR0 : 中断请求标志
    //ID2~ID0 : 中断检测标志     
    //--------------------------------------------------------------------------
    G29ICR = 0x1000;     //霍尔信号的中断优先级设为1 (仅低于ALM)
    //--------------------------------------------------
    //G30ICR：组30中断控制寄存器(控制外中断IRQ11、IRQ10)
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
    //IE1~IE0 : 中断使能标志  IE1=IRQ11   IE0=IRQ10
    //IR1~IR0 : 中断请求标志
    //ID1~ID0 : 中断检测标志     
    //--------------------------------------------------------------------------
    G30ICR = 0x1000;     //霍尔信号的中断优先级设为1 (仅低于ALM)
	  //--------------------------------------------------------------------------
	  guc_motor_ID = 0;    //初始化电机ID号为0
	  motor_var_init();    //电机相关变量程序化
}
//------------------------------------------------------------------------------
void motor_var_init(void)     //电机相关变量程序化，在Hall初始化和通讯ID变更程序中调用
{
	  uint32_t lul_tmp;
    
    //--------------------------------------------------------------------------
    //确定正弦波驱动逆时针旋转每个霍尔信号区间的起始相位角
    PhaseValues_CCW[0] = ram_para[num_MOTOR_CCW];//motor_para[guc_motor_ID][MOTOR_CCW];
    PhaseValues_CCW[1] = ((PhaseValues_CCW[0] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CCW[2] = ((PhaseValues_CCW[1] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CCW[3] = ((PhaseValues_CCW[2] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CCW[4] = ((PhaseValues_CCW[3] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CCW[5] = ((PhaseValues_CCW[4] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    //--------------------------------------------------------------------------
    //确定正弦波驱动顺时针旋转每个霍尔信号区间的起始相位角
    PhaseValues_CW[0]= ram_para[num_MOTOR_CW];//motor_para[guc_motor_ID][MOTOR_CW];
    PhaseValues_CW[5]= ((PhaseValues_CW[0] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CW[4]= ((PhaseValues_CW[5] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CW[3]= ((PhaseValues_CW[4] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CW[2]= ((PhaseValues_CW[3] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);
    PhaseValues_CW[1]= ((PhaseValues_CW[2] + SIN_TAB_LENGTH_4 / 6) % SIN_TAB_LENGTH_4);	 
	  //--------------------------------------------------------------------------
	  //频率平均数个数=电机极对数*电周期Hall数*圈数
	  gus_freq_array_len = ram_para[num_MOTOR_p] * 6 * 1;//motor_para[guc_motor_ID][MOTOR_p]
	  for (gus_avr_realfreq_cnt = 0; gus_avr_realfreq_cnt < 50; gus_avr_realfreq_cnt++)
	  {
	  	   gus_realfreq_array[gus_avr_realfreq_cnt] = 0;
	  }
	  gus_avr_realfreq_cnt = 0;
	  //--------------------------------------------------------------------------
	  //Hall故障检测信号相同次数=电机极对数*电周期Hall数*圈数
	  gus_Hall_same_cnt = ram_para[num_MOTOR_p] * 6 * 20;//10;//2;//debug堵转时间太短
	  //--------------------------------------------------------------------------
	  //得到电机超速频率
	  lul_tmp = ram_para[num_MOTOR_max_n];//motor_para[guc_motor_ID][MOTOR_max_n];
	  lul_tmp *= ram_para[num_motor_overspeed];
	  lul_tmp /= 1000;
	  gus_motor_overspeed = (uint16_t)lul_tmp;
	  //--------------------------------------------------------------------------
}
//------------------------------------------------------------------------------
void Hall_input_int(void)     //Hall信号输入中断服务程序
{
      //------------------------------------------------------
      //捕获当前中断触发的时刻
    TM4MD &= 0x7F;  //停止TM4BC~TM7BC 32bit计数
    Hall_trigger_time.uword.low = TM45BC;
    Hall_trigger_time.uword.high = TM67BC;
    TM4MD |= 0x80;  //启动TM4BC~TM7BC 32bit计数
    //------------------------------------------------------
    if (bflg_hall_int == 0)   //如果是首次进入霍尔中断
    {
    	  bflg_hall_int = 1;    //置首次进入霍尔中断标志
    	  
    	  gul_Hall_trigger = Hall_trigger_time.ulword;   //得到出发时刻
    	  //gul_Hall_past_trigger = gul_Hall_trigger;
    	  
    	  gus_Hall_value = HALL_PIN;                     //得到霍尔信号值
    	  gus_Hall_past_value = gus_Hall_value;
    	  
    	  gsc_Hall_sector = gsc_sector_table[gus_Hall_value]; //得到霍尔扇区值
    	  gsc_Hall_past_sector = gsc_Hall_sector;
    	  
    	  //theta.word.high = gul_theta.word.high + 600;   //向前走60°相位
        //theta.word.low  = 0;
        
        //gul_theta.word.high = theta.word.high;
    	  if (bflg_current_direction == 0)    //如果要求转向为逆时针
    	  {
    	      gul_theta.uword.high = PhaseValues_CCW[gsc_Hall_sector];
            gul_theta.uword.low  = 0;
                                
            //gul_theta.word.high = theta.word.high;
    	  }
        else  //如果实际转向为顺时针
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
    	  //因计数器为递减计数，所以此处使用上次计数值减本次计数值
    	  gul_Hall_delt_trigger_buffer = gul_Hall_trigger - Hall_trigger_time.ulword;
    	  //if (gul_Hall_delt_trigger_buffer > 40000)  //当计数值小于40000时，无效（即实测频率大于83.33Hz）
    	  //if (gul_Hall_delt_trigger_buffer > 35000)  //当计数值小于35000时，无效（即实测频率大于95.24Hz（1429rpm）） 
    	  //if (gul_Hall_delt_trigger_buffer > 27500)  //当计数值小于27500时，无效（即实测频率大于120Hz（1818rpm））     	  
    	  if (gul_Hall_delt_trigger_buffer > 27500)  //当计数值小于27500时，无效（即实测频率大于120Hz（1818rpm）1000W-4对极电机转速1800rpm=60*120/极对数4））     	      	  
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
    	  	  	  if ((bflg_hall_update == 0) && (bflg_no_direction == 0))   //如果无霍尔信号更新标志且无旋转方向标志
    	  	  	  {
    	  	  	  	  gul_Hall_past_delt_trigger = gul_Hall_delt_trigger;    //保存上一次的时间间隔
    	  	  	  	  gul_Hall_delt_trigger = gul_Hall_delt_trigger_buffer;  //记录本次的时间间隔
    	  	  	  	  
    	  	  	  	  bflg_hall_update = 1;
    	  	  	  }
    	  	  	  //--------------------------------------------------------------
    	  	  	  gus_Hall_past_value = gus_Hall_value;       //保存上次霍尔信号值
    	  	  	  gus_Hall_value = gss_Hall_value_buffer;     //记录本次霍尔信号值
    	  	  	  
    	  	  	  gsc_Hall_past_sector = gsc_Hall_sector;     //保存上次扇区值
    	  	  	  gsc_Hall_sector = gsc_sector_table[gus_Hall_value];   //记录本次扇区值
    	  	  	  //--------------------------------------------------------------
    	  	  	  bflg_past_actual_direction = bflg_actual_direction;   //保存上次实际转向标志
    	  	  	  //--------------------------------------------------------------
    	  	  	  //判断实际旋转方向
    	  	  	  if (gsc_Hall_sector == ((gsc_Hall_past_sector + 1) % 6))
    	  	  	  {
    	  	  	  	  bflg_actual_direction = 0;    //实际转向为顺时针
    	  	  	  	  bflg_no_direction = 0;        //清无法判断旋转方向标志

    	  	  	  }
    	  	  	  else if (gsc_Hall_sector == ((gsc_Hall_past_sector + 5) % 6))
    	  	  	  {
    	  	  	  	  bflg_actual_direction = 1;    //实际转向为逆时针
    	  	  	  	  bflg_no_direction = 0;        //清无法判断旋转方向标志
    	  	  	  }
    	  	  	  else
    	  	  	  {
    	  	  	  	  bflg_no_direction = 1;        //置无法判断旋转方向标志
    	  	  	  }
    	  	  	  //--------------------------------------------------------------
    	  	  	  if (bflg_running == 1)
    	  	  	  {
    	  	  	      //if (bflg_current_direction == bflg_actual_direction)   //如果实际转向与要求转向一致
    	  	  	      //{
    	  	  	  	      if (bflg_no_direction == 0)    //如果运行且可以判断转向
    	  	  	  	      {
										    	  if (bflg_current_direction == 0)    //如果要求转向为逆时针
										    	  {
										    	      gul_theta.uword.high = PhaseValues_CCW[gsc_Hall_sector];
										            gul_theta.uword.low  = 0;
										                                
										            //gul_theta.word.high = theta.word.high;
										    	  }
										        else  //如果实际转向为顺时针
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
    	  	  	      if ((bflg_current_direction != bflg_actual_direction)   //如果实际转向与要求转向不一致
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
    gus_Hall_watchdog = 0;    //清霍尔信号看门狗 
    //------------------------------------------------------
}
//------------------------------------------------------------------------------
void Hall_updata_deal(void)        //霍尔信号更新处理程序
{
	  uint16_t lus_index;
	  lword_type ltmp1,ltmp2;
	  
	  if (bflg_hall_update == 1)     //如果允许更新霍尔信号位置，则进入
	  {
        //----------------------------------------------------------------------
        //关于霍尔信号W故障的判断
        if ((gus_Hall_past_value & 0x0001) == (gus_Hall_value & 0x0001))
        {
        	  gus_same_HW_cnt++;     //霍尔信号W相同计数器增加
        }
        else
        {
        	  gus_same_HW_cnt = 0;   //清霍尔信号W相同计数器
        }
        //--------------------------------------------------
        //关于霍尔信号V故障的判断
        if ((gus_Hall_past_value & 0x0002) == (gus_Hall_value & 0x0002))
        {
        	  gus_same_HV_cnt++;     //霍尔信号V相同计数器增加
        }
        else
        {
        	  gus_same_HV_cnt = 0;   //清霍尔信号V相同计数器
        }
        //--------------------------------------------------
        //关于霍尔信号U故障的判断
        if ((gus_Hall_past_value & 0x0004) == (gus_Hall_value & 0x0004))
        {
        	  gus_same_HU_cnt++;     //霍尔信号U相同计数器增加
        }
        else
        {
        	  gus_same_HU_cnt = 0;   //清霍尔信号U相同计数器
        }
        //----------------------------------------------------------------------
        if (bflg_current_direction == bflg_actual_direction)     //实际转向与要求转向一致
        {
            if (bflg_running == 1)
            {
                //------------------------------------------
                //计算电机失速率
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
        //计算实测转速
        ltmp1.ulword = FREQ_FACTOR;
        ltmp1.ulword /= gul_Hall_delt_trigger;     //得到本次实测频率
        
        bflg_hall_update = 0;       //清允许更新霍尔信号位置标志
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
        //对Hallfreq进行1/4根木滤波
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
        gus_Hall_speed = ltmp1.ulword;//用于调试面板显示
        //----------------------------------------------
        //r = (f / 100) * 60 / P
        ltmp1.ulword = gus_avr_realfreq;//gus_Hall_freq;
        ltmp1.ulword *= 6;
        ltmp1.ulword /= 10;
        ltmp1.ulword /= ram_para[num_MOTOR_p];//motor_para[guc_motor_ID][MOTOR_p];
        gus_speed_real = ltmp1.ulword;//用于上位机显示        
        
        //gus_speed_real = gus_Hall_speed;
        //----------------------------------------------
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
        
        if (gus_phase_adv_degree >= ram_para[num_phase_max_degree])
        {
        	  gus_phase_adv_degree = ram_para[num_phase_max_degree];
        }
        //--------------------------------------------------
        /*if (bflg_sensor_startup == 1)
        {
            gus_sensor_startup_timer = 0;     //清开机测速阶段计时器
            //-----------------------------------------------
            if (gus_freq_real <= ram_para[num_min_freq])
            {
                //在开机测速阶段，已经计算出了转速，则清开机测速阶段标志
                bflg_sensor_startup = 0;     //置开机测速阶段标志
                sensored_start_init();       //带位置传感器控制时的开机初始化程序
            }                        	   
        }*/
        //--------------------------------------------------
	  }
}
//------------------------------------------------------------------------------
void Hall_fault_delaytime(void)    //霍尔信号故障延时程序，在10ms定时程序中调用
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

        if (bflg_running == 0) //故障停机标志清除后，再清除故障标志
        {
            bflg_Hall_fault = 0;   
        }	  	  
     }
}
//------------------------------------------------------------------------------
void Hall_error_delaytime(void)    //霍尔信号错延时程序，加入10ms定时程序中
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
	  	  
    	  if (bflg_running == 0) //故障停机标志清除后，再清除故障标志
    	  {
    	       bflg_Hall_error = 0;
        }	  	    
	  }
}
//------------------------------------------------------------------------------
void Hall_direction_delaytime(void)     //霍尔信号方向错延时程序，加入10ms定时程序中
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
	  	  
    	  if (bflg_running == 0) //故障停机标志清除后，再清除故障标志
    	  {
    	      bflg_prot_Hall_direction = 0;    
        }	  	    
	  }
}
//------------------------------------------------------------------------------
void Hall_off_delaytime(void)      //霍尔信号停止延时程序，在10ms定时程序中调用
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
	      
	      bflg_hall_int = 0;      //清霍尔信号中断标志
	      //--------------------------------------------------
	      //确定Hall区间
	      gus_Hall_value = HALL_PIN; //得到霍尔信号值
	      gsc_Hall_sector = gsc_sector_table[gus_Hall_value];
	      //--------------------------------------------------
	      //gus_freq_out = 0;
        
        //gus_Uout_real = 0;
        //gsl_FreqPI_integral = 0;
        
    	  if (bflg_current_direction == 0)    //如果要求转向为逆时针
    	  {
    	      gul_theta.uword.high = PhaseValues_CCW[gsc_Hall_sector];
            gul_theta.uword.low  = 0;
                                
            //gul_theta.word.high = theta.word.high;
    	  }
        else  //如果实际转向为顺时针
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
	  	  //得到积分初值
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
