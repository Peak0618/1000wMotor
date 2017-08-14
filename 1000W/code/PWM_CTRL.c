#ifndef _PWM_CTRL_C_
#define _PWM_CTRL_C_
//------------------------------------------------------------------------------
#include "srMN103SFJ9D.h"
#include "user_typedefine.h"
#include "comm_variable.h"
#include "user_asmfunc.h"

#include "EEPROM_PARA.h"
#include "PWM_CTRL.h"
#include "AD_Convert.h"
#include "SIN_TABLE.h"
#include "protect_deal.h"
#include "Hallsignal_deal.h"
#include "user_math.h"
//------------------------------------------------------------------------------
void PWM_configure(void);     //PWM模块配置程序

void freq_rate_init(void);    //初始化1ms频率增减量的程序

void open_pwm(void);          //开启PWM的程序，在开机程序中调用

void shut_pwm(void);          //关闭PWM的程序，在停机程序或保护停机程序中调用

void bootstrap_charger_delaytime(void);  //自举充电延时程序，在1ms定时程序中调用

void check_freq_updata(void);      //定时进行频率更新的程序(每1ms检查1次)

void Sensored_cale_Vout(void);     //带位置传感器控制计算输出电压的程序

void calc_percent_theta(void);     //用于计算当前调制率和相位角步长的程序

void calc_comp_val(void);          //计算下一次三相输出的比较计数值,在波形发生中断中调用

int16_t get_sin_value(uint16_t now_sin_index);    //查sin表的函数

uint16_t get_tai_comp_value(uint16_t now_index);  //台形波调制时计算PWM输出计数值的程序

uint16_t get_svpwm_comp_value(uint16_t now_index);//股形波调制时计算PWM输出计数值的程序

void PWM_Reload_INT(void);    //波形发生中断

void comp_val_order(void);    //对三相比较值进行排序

//------------------------------------------------------------------------------
//标志定义
flag_type flg_PWM;
//----------------------------------------------------------
//变量定义
//----------------------------------------------------------
//定义一个存放用于计算频率加减速变量的结构体
struct  
{
    uint32_t normal_accel;    //正常阶段的每1ms加速步长 (扩大了2^24)
    uint32_t normal_decel;    //正常阶段的每1ms减速步长 (扩大了2^24)
    
    lword_type sum_accel;     //加速步长尾数累计寄存器
    lword_type sum_decel;     //减速步长尾数累计寄存器
}freq_rate;

uint16_t  gus_stall_amp_accel;//加速过程的失速倍率寄存器
uint16_t  gus_stall_amp_decel;//减速过程的失速倍率寄存器

uint16_t  gus_Td;             //死区时间计数值寄存器

uint16_t  gus_half_pwm;       //pwm计数值的一半，即载波的幅值
uint16_t  gus_peak_pwm;       //pwm计数值，即载波的波峰到波谷值

uint16_t  gus_centi_half_pwm; //百分之一的half_pwm; (用于自举充电)

uint16_t  gus_val_a;
uint16_t  gus_val_b;
uint16_t  gus_val_c;

int16_t  gss_max_pwm;
int16_t  gss_min_pwm;

uint32_t  gul_percent_pwm_unit;    //单位电压(0.1V)调制率计数值寄存器(2^16倍)

uint16_t  gus_max_percent;    //最大允许输出调制率的4096倍(用于台形波控制)
uint16_t  gus_min_percent;    //最小允许输出调制率的4096倍(用于台形波控制)
uint16_t  gus_max_percent_gu; //最大允许输出调制率的4096倍(用于股形波控制)

uint16_t  gus_percent_k;      //实际调制率的4096倍

uint16_t  gus_percent_pwm;    //调制率对应的pwm计数值寄存器
uint16_t  gus_percent_pwm_buf;//调制率对应的pwm计数值寄存器的缓冲寄存器

uint16_t  gus_rate_per;       //最小频率单位(0.01Hz)对应的单位计算周期内的相位差
lword_type rate_theta;        //当前频率下的相位增量寄存器
lword_type rate_theta_buf;    //当前频率下的相位增量寄存器的缓冲寄存器

lword_type gul_theta;
lword_type theta;             //A相调制波的相位角寄存器
lword_type theta_buf;         //A相调制波的相位角缓冲寄存器 (用于加上相位超前角度后查表)

uint16_t  gus_max_comp_val;   //当前PWM输出比较值的最大值
uint16_t  gus_mid_comp_val;   //当前PWM输出比较值的中间值
uint16_t  gus_min_comp_val;   //当前PWM输出比较值的最小值

uint16_t  gus_val_abc_order;  //用于记录abc三相排序大小的寄存器

//实际真正赋给AD转换触发寄存器的值的暂存器
uint16_t  gus_TUP_ADtrigger;       //载波上升段AD触发时刻暂存器
uint16_t  gus_TDW_ADtrigger;       //载波下降段AD触发时刻暂存器   

uint16_t  gus_ADtrigger_repair_val;//AD触发时刻的补偿值寄存器

uint16_t  gus_AD_SH_val;      //AD采样保持时间的寄存器

uint16_t  gus_now_theta;      //当前控制周期对应的相位角
uint16_t  gus_next_theta;     //下个控制周期对应的相位角

int16_t   gss_bootstrap_charger_delaytimer;  //自举充电计数器
uint16_t  gus_freq_set;       //设定频率
uint16_t  gus_MAX_setfreq; //最大设定频率
//uint16_t  gus_freq_out_prev;  //上次输出频率
uint16_t  gus_freq_out;       //输出频率
uint16_t  gus_freq_real;      //真实输出频率 (实测频率)

uint16_t  gus_speed_out;      //输出转速
uint16_t  gus_speed_real;     //真实输出转速 (实测转速)  

uint16_t  gus_Uout_real;      //当前真实输出电压     (0.1V)
uint16_t  gus_Uout_for_disp;

uint16_t  gus_PIctrl_delaytimer;
//int16_t   gss_PI_DeltSpeed;
int32_t   gsl_FreqPI_integral;
int32_t   gsl_PI_initval;     //积分初值
//------------------------------------------------------------------------------
//指定ABC三相输出比较值按从大到小排序的指示标志宏
#define   ORDER_ABC      0
#define   ORDER_ACB      1
#define   ORDER_CAB      2
#define   ORDER_BAC      3
#define   ORDER_BCA      4
#define   ORDER_CBA      5
//------------------------------------------------------------------------------
void PWM_configure(void)      //PWM模块配置程序
{
    uint32_t lul_tmp;
    
    //------------------------------------------------------
    //以下计算是在PMWclock=MCLK时的计算(PWMMD0A.CLKSEL0=1)  
    //计算peak_pwm和half_pwm, pwm_carrier的精度为0.1khz 
    //peak_pwm为载波峰峰值, half_pwm为载波峰峰值的1/2
    //gus_peak_pwm = ([1 / (pwm_carrier * 1000 / 10)] / [1 / 40M]) / 2 - 1 
    //即gus_peak_pwm = (200000) / pwm_carrier - 1  
    gus_peak_pwm = 200000 / ram_para[num_pwm_carrier] - 1;
    gus_half_pwm = (gus_peak_pwm >> 1);
    gus_centi_half_pwm = (gus_half_pwm / 100) + 1;    //百分之一的half_pwm
    //------------------------------------------------------
    //计算最小导通时间和最大导通时间
    //最小导通时间=(Td+Tmin)/2
    //最大导通时间（最小关断时间）= gus_peak_pwm - 最小导通时间
    //因为Td和Tmin精度为0.1us 即x为x/[(10^6)*10] s
    //所以(((ram_para[num_dead_time]+ram_para[num_IPM_min_time])/[(10^6)*10])/2)*40M -1
    //即min_pwm=3*(ram_para[num_dead_time]+ram_para[num_IPM_min_time]) -1
    gss_min_pwm = 2 * (ram_para[num_dead_time] + ram_para[num_IPM_min_time]) - 1;  //最小导通时间计数值                      
    gss_max_pwm = gus_peak_pwm - gss_min_pwm;      //最大导通时间计数值(最小关断时间)
    //------------------------------------------------------
    //计算最大调制率(4096倍)和最小调制率(4096倍)
    //保证最小调制率不能小于3Td/peak，最大调制率不能大于(peak-3Td)/peak，
    //gus_max_percent=0x1000-[(ram_para[num_dead_time]+ram_para[num_IPM_min_time])*2*ram_para[num_pwm_carrier]*4096/100000] //最大调制率
    //即gus_max_percent=0x1000-2*(ram_para[num_dead_time]+ram_para[num_IPM_min_time])*ram_para[num_pwm_carrier]*128*32/3125*32;
    lul_tmp = ram_para[num_dead_time] + ram_para[num_IPM_min_time];
    lul_tmp *= ram_para[num_pwm_carrier];
    lul_tmp *= 128;
    lul_tmp /= 3125;
    gus_min_percent = lul_tmp * 2;
    if (ram_para[num_max_k_set] == 0)
    {
        gus_max_percent = 0x1000 - lul_tmp;
        gus_max_percent_gu = 0x1000-2*lul_tmp;
    }
    else if (ram_para[num_max_k_set] == 1)       //如果允许超调
    {
    	  gus_max_percent = 0x1000;
    	  gus_max_percent_gu = 0x1000;
    }
    //-------------------------------------------------------
    //计算单位调制率初值
    //单位电压调制率＝[1/(直流电压/sqrt(2))],得到单位调制率的*(2^16)*4096倍,电压精度到0.1V
    //percent_pwr_unit = 4096*(2^16)/[(Udc_val)/sqrt(2)] 
    //又因为上电时 Udc=Uin*sqrt(2),所以percent_pwr_unit的初值为 4096*(2^16)/Uin
    gul_percent_pwm_unit = 268435456 / ram_para[num_Uin_rate];
    //------------------------------------------------------
    //计算死区时间计数值gus_Td, Td精度为0.1us
    //Td * (10^-6) / 10 = (2 * gus_Td + 1) / 40M
    //gus_Td = 2 * Td - (1/2)
    //gus_Td = 2 * Td    //(1/2)忽略
    gus_Td =(2 * ram_para[num_dead_time]);
    //------------------------------------------------------
    //因开机时要进行自举充电
    //因为下臂与上臂极性相反，所以下臂占空比0开到50％时，对应占空比从100％减到50％   
    gus_val_a = gus_peak_pwm + 1;
    gus_val_b = gus_peak_pwm + 1;
    gus_val_c = gus_peak_pwm + 1;  //给各相计数值赋初值
    //------------------------------------------------------
    //初始化PWM输出比较值的排序寄存器   
    gus_max_comp_val = gus_half_pwm;    //当前PWM输出比较值的最大值寄存器赋初值
    gus_mid_comp_val = gus_half_pwm;    //当前PWM输出比较值的中间值寄存器赋初值
    gus_min_comp_val = gus_half_pwm;    //当前PWM输出比较值的最小值寄存器赋初值
    //------------------------------------------------------
    //计算rate_per的值{最小频率单位(0.01Hz)对应的单位计算周期内的相位差}
    lul_tmp = SIN_TAB_LENGTH_4;   //每0.1度取一个点，则一个周期内有SIN_TAB_LENGTH_4=3600个点
    lul_tmp <<= 16;      //gus_rate_per=（2^16）*(SIN_TAB_LENGTH_4)/(10000*f_carrier);  0.01Hz在一个载波周期内对应的度数(单位0.1度)
    lul_tmp /= 10000;
    lul_tmp /= ram_para[num_pwm_carrier];    //每个载波周期发生一次PWM中断
    gus_rate_per = lul_tmp;
    //------------------------------------------------------
    //初始化A相的相位角寄存器以及调制率寄存器、调制率计数值寄存器
    theta.ulword = 0;       //选择A相调制波的相位角为0
    gus_percent_k = 0;       //调制率寄存器清0
    gus_percent_pwm = 0;    //调制率计数值寄存器清0
    
    gus_now_theta = 0;
    gus_next_theta = 0;
    //------------------------------------------------------
    //选择TM20的时钟源为MCLK
    //计算AD触发时刻的补偿时间 (T/(1/40M)=(X/10)*(10^-6)*40*(10^6)=X*4
    lul_tmp = ram_para[num_ADrepair_time];
    lul_tmp *= 4;
    gus_ADtrigger_repair_val = lul_tmp;  
    //------------------------------------------------------
    //选择TM20的时钟源为MCLK
    //计算AD的采样保持时间 (T/(1/40M)=(X/10)*(10^-6)*40*(10^6)=X*4
    lul_tmp = ram_para[num_AD_SH_time];
    lul_tmp *=6;
    gus_AD_SH_val = lul_tmp;
    //------------------------------------------------------
    gus_TUP_ADtrigger = gus_half_pwm;
    gus_TDW_ADtrigger = gus_half_pwm;
    
    //PWM下溢中断使能时,触发TM20进行AD采样时刻计时
    TM20CB = gus_TUP_ADtrigger;
    TM20CA = (2*gus_peak_pwm - gus_TDW_ADtrigger);
    
    //--------------------------------------------------------------------------
    //G16ICR：组16中断控制寄存器(控制PWM0计数器上溢、下溢中断)
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
    G16ICR = 0x3000;    //设置PWM上溢、下溢的中断优先级为3 (仅低于ALM、Hall、AD)
    //--------------------------------------------------------------------------
    
    //PWMMD0A:PWM0模式控制寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12   |  11    |  10    |  9    |  8    | 
    // flag    |  -   |  -   |SFTEN0|CLKSEL0|TMSTAEN0|TMSTBEN0|SDSELA0|SDSELB0| 
    //at reset |  0   |  0   |  0   |  0    |  0     |  0     |  0    |  0    |
    //--------------------------------------------------------------------------
    //reg_bit  |  7    |  6    |  5    |  4    |  3   |  2   |  1   |  0    |  
    // flag    |PCRAEN0|PCRBEN0|INTAEN0|INTBEN0|DTEN0 |ORMD0 |TCEN0 |WAVEMD0| 
    //at reset |  0    |  0    |  0    |  0    |  0   |  0   |  0   |  0    |
    //--------------------------------------------------------------------------
    //SFTEN0: 输出实时变更功能选择   0=禁止    1=使能
    //CLKSEL0: 计数时钟选择      0=IOCLK   1=MCLK
    //TMSTAEN0: 通过PWM0计数器下溢激活TM20、AD0、AD1、AD2选择   0=禁止    1=使能
    //TMSTBEN0: 通过PWM0计数器上溢激活TM20、AD0、AD1、AD2选择   0=禁止    1=使能
    //SDSELA0:  OUTMD0缓冲器模式选择    0=单缓冲模式     1=双缓冲模式
    //SDSELB0:  PWMSEL0缓冲器模式选择   0=单缓冲模式     1=双缓冲模式
    //PCRAEN0:  双缓冲加载时刻选择(PWM0计数器下溢)   0=禁止    1=使能
    //PCRBEN0:  双缓冲加载时刻选择(PWM0计数器上溢)   0=禁止    1=使能
    //INTAEN0:  定时器中断时刻选择(PWM0计数器下溢)   0=禁止    1=使能
    //INTBEN0:  定时器中断时刻选择(PWM0计数器上溢)   0=禁止    1=使能
    //DTEN0:    死区时间插入选择   0=无死区时间      1=有死区时间
    //ORMD0:    死区时间插入逻辑   0=正逻辑(L有效)   1=负逻辑(H有效)
    //TCEN0:    PWM计数操作使能    0=禁止    1=使能
    //WAVEMD0:  PWM载波模式        0=三角波  1=锯齿波
    //--------------------------------------------------------------------------
    PWMMD0A = 0x0000;
    
    PWMMD0A |= 0x1000;   //选择PWM计数时钟为MCLK
    PWMMD0A |= 0x0800;   //通过PWM0计数器下溢激活TM20、AD0、AD1、AD2都使能 (用于AD采样)
    PWMMD0A |= 0x0100;   //PWMSEL0缓冲器模式选择双缓冲模式
    PWMMD0A |= 0x0080;   //下溢双缓冲使能
    PWMMD0A |= 0x0020;   //定时器下溢中断使能
    PWMMD0A &= ~0x0004;  //死区时间插入为正逻辑
    
    PWMMD0A &= 0xFFFC;   //PWM载波为三角波，禁止PWM0计数操作。
    
    if(gus_Td != 0)        //如果死区时间不为零，则设置为有死区时间插入
    {
        PWMMD0A |= 0x0008;   //有死区时间插入
    }
    else
    {
        PWMMD0A &= 0xFFF7;   //无死区时间插入   	  
    }
    //--------------------------------------------------------------------------
    //OUTMD0:PWM0输出极性控制寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   |  NW  |  W   |  NV  |  V   |  NU  |  U   | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //PXDTNx0 : 0=Negative phase  1=Positive phase
    //PXDTx0  : 0=Positive phase  1=Negative phase
    //--------------------------------------------------------------------------
    OUTMD0 = 0x003F;
    
    //PWMSEL0：PWM0输出控制寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   | PSEL | PSEL | PSEL | PSEL |
    // flag    |  -   |  -   |  -   |  -   | N02  |  02  | N01  |  01  |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   | 
    // flag    | PSEL | PSEL | OTLV | OTLV | OTLV | OTLV | OTLV | OTLV | 
    // flag    | N00  |  00  | N02  |  02  | N01  |  01  | N00  |  00  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //PSELN0x、PSEL0x :  0=PWM output       1=H/L level output
    //OTLVN0x、OTLV0x ： 0=L level output   1=H level output
    //--------------------------------------------------------------------------
    //PWMSEL0 = 0x0FFF;   //all pin for H  level output
    
    //PWMSET0: PWM0周期设定寄存器
    PWMSET0 = gus_peak_pwm;
    
    //TCMP0x: PWM0占空比设定寄存器
    TCMP0A = gus_val_a;  //PWM00占空比设定  W相
    TCMP0B = gus_val_b;  //PWM01占空比设定  V相
    TCMP0C = gus_val_c;  //PWM02占空比设定  U相
    
    //DTMSET0A: PWM0死区时间设定寄存器A
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |  -   |  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |                 DTST0A7~DTST0A0                       | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //DTST0A7~DTST0A0 ： (Dead_time*clock_cycle)/2 -1
    //--------------------------------------------------------------------------
    DTMSET0A = gus_Td;
    
    //DTMSET0A: PWM0死区时间设定寄存器B
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  -   |  -   |  -   |DT0BEN| 
    //at reset |  -   |  -   |  -   |  -   |  -   |  -   |  -   |  0   | 
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |                 DTST0B7~DTST0B0                       | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //DT0BEN :  0=Same dead time value is inserted at PWM0 output. 
    //          1=Another dead time values are inserted at PWM0 output
    //DTST0B7~DTST0B0 ： (Dead_time*clock_cycle)/2 -1
    //--------------------------------------------------------------------------
    DTMSET0B = (gus_Td | 0x0100);
    
    //PWMBC0: PWM0计数值读寄存器
    //PWMBC0  (only read)
    
    //BCSTR0: PWM0计数值状态寄存器
    //BCSTR0  (only read) bit0: PWM0STR 0=Down-count  1=Up-count (atreset 1)
    
    //--------------------------------------------------------------------------
    //G24ICR：组24中断控制寄存器(外中断IRQ3)
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  LV2 |  LV1 |  LV0 |  -   |  -   |  -   |  IE0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | -    | IR0  |  -   |  -   | -    | ID0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //LV2~LV0 : 中断优先级设置, 0到7(0最高  7禁止)
    //IE1~IE0 : 中断使能标志
    //IR1~IR0 : 中断请求标志
    //ID1~ID0 : 中断检测标志     
    //--------------------------------------------------------------------------
    G24ICR = 0x0000;       //外中断的优先级设置为0 (最高级)
    
    //PWMIRQCNT0: PWM0中断输出控制寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  IRQBSCNT03~IRQBSCNT00    |   IRQBCNT03~IRQBCNT00     |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   | 
    // flag    |  IRQASCNT03~IRQASCNT00    |   IRQACNT03~IRQACNT00     | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //IRQBSCNT03~IRQBSCNT00: 在PWM0计数器上溢几次后,产生首次上溢中断 
    //IRQBCNT03~IRQBCNT00:  在PWM0计数器上溢几次后,产生后续上溢中断
    //IRQASCNT03~IRQASCNT00: 在PWM0计数器下溢几次后,产生首次下溢中断
    //IRQACNT03~IRQACNT00:  在PWM0计数器下溢几次后,产生后续下溢中断
    //--------------------------------------------------------------------------
    PWMIRQCNT0 = 0x0000;      //每次都中断
    
    //PWMOFF0A: PWM0引脚保护控制寄存器A
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  PRTNW01~00 |  PRTW01~00  |  PRTNV01~00 |  PRTV01~00  |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   | 
    // flag    |  PRTNU01~00 |  PRTU01~00  |  -   | IRQSEL01~00 |OUTEN0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //PRTNW01~00,PRTW01~00: output pin protection function
    //                      00=PWM output (protection disabled)   01=Hi-Z output 
    //                      10=Inactive output                    11=Unused         
    //IRQSEL01~00: PWM0 pin protection External interrupt selection
    //                      00=IRQ00   01=IRQ01  10=IRQ02   11=IRQ03                  
    //OUTEN0:  PWM0pin output enable   0=Disabled   1=Enabled
    //--------------------------------------------------------------------------
    //发生保护时，上臂输出无效电平，下臂出无效电平，外中断选择为IRQ03,输出不使能 
    PWMOFF0A = 0x0000;    //未用
    
    //PWMOFF0A1: PWM0引脚保护控制寄存器A1
    //--------------------------------------------------------------------------
    //reg_bit  |  15                ~                      1    |  0   | 
    // flag    |  --------------------------------------------  |NMIEN0|
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //NMIEN0:  PWM0 pin output protection function by NMI enable
    //         0=Disabled   1=Enabled
    //--------------------------------------------------------------------------
    PWMOFF0A1 = 0x0001;
    
    //PWMOFF0B: PWM0引脚保护控制寄存器B   
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |   9    |    8    | 
    // flag    |   PRTBCNT03 ~PRTBCNT00    |  -   |  -   |PRTBBKA0|PRTBBKB0 |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |   0    |    0    |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4    |  3  |  2   |  1   |  0    | 
    // flag    |  -   | PRTBMD01~00 |IRQPHA0|  -  | IRQSELB01~00|PRTBEN0|
    //at reset |  0   |  0   |  0   |  0    |  0  |  0   |  0   |  0    |
    //--------------------------------------------------------------------------
    //PRTBCNT03 ~PRTBCNT00: 设置保护的外中断采样次数,当外中断次数达到设定值后保护
    //PRTBBKA0: 通过PWMBC0下溢解除保护控制选择,0=Disabled   1=Enabled
    //PRTBBKB0: 通过PWMBC0上溢解除保护控制选择,0=Disabled   1=Enabled
    //PRTBMD01~00: PWM0引脚保护B操作选择,
    //             00=PWM output(protection disabled),01=All pin is inactive output
    //             10=Only PWM00,01,02 is inactive output 11=The pin is automatically controlled by a state
    //IRQPHA0:  外中断电平选择 0=L   1=H
    //IRQSELB01~00: PWM0引脚保护B外中断选择 00=IRQ00   01=IRQ01  10=IRQ02   11=IRQ03 
    //PRTBEN0:  PWM0引脚保护B操作使能选择 0=Disabled   1=Enabled
    //--------------------------------------------------------------------------
    //设置为每次外中断都保护,仅保护所有PWM引脚,外中断电平为L,保护外中断为IRQ03,保护不使能
    PWMOFF0B = 0x0000;   //保护不使能
    
    //PWMOFF0B1: PWM0引脚保护控制寄存器B1
    //--------------------------------------------------------------------------
    //reg_bit  |  15                ~                      1    |  0    | 
    // flag    |  --------------------------------------------  |PRTBST0|
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0    |
    //--------------------------------------------------------------------------
    //PRTBST0:  State of the PWM0 pin protection B operation  0=Disabled   1=Enabled
    //--------------------------------------------------------------------------
    //PWMOFF0B1 =0x0000;
    //防止上电时误保护，放在自举前开启保护
    /*if (ram_para[num_prot_ALM_cnt] == 0) //如果参数设定计数次数为0
    {
   	    PWMOFF0B = 0xF026;
   	    
   	    PWMOFF0B1 &=~0x0001;      //清PRTBST0位，释放PWM0引脚硬件保护B功能
   	    PWMOFF0B |= 0x0001;
    }
    else
    {
   	    PWMOFF0B = 0x0000;      //计数次数不为0时，不用硬件保护
    }*/
    
    //--------------------------------------------------------------------------
    ///P0MD = 0x08;    //P03 is TM5IO pin/IRQ03 pin(0=IO 1=Special function)
    
    //IRQ03 32倍的噪声滤波，噪声滤波使能
    NFCLK0 |= 0x0080;//0x00C0;//
    NFCNT0 |= 0x0008;
    
    //IRQ03 下降沿触发中断
    EXTMD0 &= ~0x00C0;
    EXTMD0 |= 0x0040;
    
    //--------------------------------------------------------------------------
    //G24ICR：组16中断控制寄存器(外中断IRQ3)
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  LV2 |  LV1 |  LV0 |  -   |  -   |  -   |  IE0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   |  -   | -    | IR0  |  -   |  -   | -    | ID0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //LV2~LV0 : 中断优先级设置, 0到7(0最低  7最高)
    //IE1~IE0 : 中断使能标志
    //IR1~IR0 : 中断请求标志
    //ID1~ID0 : 中断检测标志     
    //--------------------------------------------------------------------------
    //开启IRQ03外中断，用于ALM保护
    *(unsigned char *) & G24ICR = 0x01; //清除IRQ03外中断请求标志
    //G24ICR |= 0x0100;       //置IRQ03外中断使能标志   //防止上电时误保护，放在自举前开启保护
    
    //--------------------------------------------------------------------------
    //PWMDCNT0A: PWM0输出时间控制寄存器A (控制PWM00 and NPWM00)
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |   9  |  8   | 
    // flag    |  -   |  -   |  -   |  -   |  -   |  -   |SDIRU0|SENU0 |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |   0  |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4    |  3  |  2   |  1   |  0    | 
    // flag    |                STIMU07 ~ STIMU00                       |
    //at reset |  0   |  0   |  0   |  0    |  0  |  0   |  0   |  0    |
    //--------------------------------------------------------------------------
    //SDIRU0:  PWM00 and NPWM00移动方向控制  0=向前移   1=向后移
    //SENU0：  PWM00 and NPWM00方向移动控制  0=禁止     1=使能
    //STIMU07 ~ STIMU00:  PWM00 and NPWM00输出移动数量
    //--------------------------------------------------------------------------
    PWMDCNT0A = 0x0000;
    
    //PWMDCNT0B: PWM0输出时间控制寄存器B (控制PWM01 and NPWM01,与PWMDCNT0A定义相似)
    PWMDCNT0B = 0x0000;
    
    //PWMDCNT0C: PWM0输出时间控制寄存器C (控制PWM02 and NPWM02,与PWMDCNT0A定义相似)
    PWMDCNT0C = 0x0000;
    
    //--------------------------------------------------------------------------
    *(unsigned char *) & G16ICR = 0x03; //清除PWM上溢、下溢请求标志
    G16ICR |= 0x0100;    //置PWM下溢使能标志   
    //------------------------------------------------------
    PWMSEL0 = 0x0FC0;    //PWM引脚由软件控制,上臂,下臂全无效
    PWMOFF0A |= 0x0001;  //PWM输出使能
    //------------------------------------------------------
    //设置P80~P85为PWM引脚
    P8MD |= 0x3F;  //P80~P85 Special function is PWM pin  (0=IO 1=PWM) 
    //------------------------------------------------------
    PWMMD0A |= 0x0002;   //PWM计数操作使能
    //------------------------------------------------------
}
//-----------------------------------------------------------------------------------
void freq_rate_init(void)  //初始化1ms频率增减量的程序 加减速速率设置程序，在控制初始化程序中调用
{
    lword_type ltmp;
    //------------------------------------------------------
    //因为加减速时间的定义为每50.00Hz需要多长时间，且设置范围为0.1~600.0s
    //所以在计算增量时扩大2^24
    //50.00/(0.1~600.0)*1000ms=5000/(1~6000)*100=50/(1~6000)即1ms增加多少个0.01Hz
    ltmp.ulword = 50;
    ltmp.ulword <<= 24;
    
    freq_rate.normal_accel = (ltmp.ulword / ram_para[num_accel_time]);
    freq_rate.normal_decel = (ltmp.ulword / ram_para[num_decel_time]);
    
    freq_rate.sum_accel.ulword = 0;
    freq_rate.sum_decel.ulword = 0;
    //------------------------------------------------------
}
//------------------------------------------------------------------------------
void open_pwm(void)   //开启PWM的程序，在开机程序中调用
{
    //------------------------------------------------------
    gus_max_comp_val = gus_half_pwm;    //当前PWM输出比较值的最大值寄存器赋初值
    gus_mid_comp_val = gus_half_pwm;    //当前PWM输出比较值的中间值寄存器赋初值
    gus_min_comp_val = gus_half_pwm;    //当前PWM输出比较值的最小值寄存器赋初值
    //------------------------------------------------------
    //占空比缓冲寄存器赋初值
    gus_val_a = gus_half_pwm;
    gus_val_b = gus_half_pwm;
    gus_val_c = gus_half_pwm;
    //------------------------------------------------------
    //初始化TCMP0x: PWM0占空比设定寄存器
    TCMP0A = gus_val_a;     //PWM00占空比设定  W相
    TCMP0B = gus_val_b;     //PWM01占空比设定  V相
    TCMP0C = gus_val_c;     //PWM02占空比设定  U相
    //-------------------------------------------------------
    //PWMSEL0：PWM0输出控制寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  -   |  -   |  -   | PSEL | PSEL | PSEL | PSEL |
    // flag    |  -   |  -   |  -   |  -   | N02  |  02  | N01  |  01  |
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   | 
    // flag    | PSEL | PSEL | OTLV | OTLV | OTLV | OTLV | OTLV | OTLV | 
    // flag    | N00  |  00  | N02  |  02  | N01  |  01  | N00  |  00  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //PSELN0x、PSEL0x :  0=PWM output       1=H/L level output
    //OTLVN0x、OTLV0x ： 0=L level output   1=H level output
    //------------------------------------------------------
    PWMSEL0 = 0x003F;    //PWM引脚由PWM功能控制
    //------------------------------------------------------
}
//------------------------------------------------------------------------------
void shut_pwm(void)      //关闭PWM的程序，在停机程序或保护停机程序中调用
{
    //------------------------------------------------------
    PWMSEL0 = 0x0FC0;    //PWM引脚由软件控制，全无效
    //------------------------------------------------------
    gus_val_a = gus_peak_pwm + 1;
    gus_val_b = gus_peak_pwm + 1;
    gus_val_c = gus_peak_pwm + 1;  //给各相计数值赋初值

    TCMP0A = gus_val_a;  //PWM00占空比设定  W相
    TCMP0B = gus_val_b;  //PWM01占空比设定  V相
    TCMP0C = gus_val_c;  //PWM02占空比设定  U相
    //------------------------------------------------------
    //初始化WM输出比较值的排序寄存器   
    gus_max_comp_val = gus_half_pwm;    //当前PWM输出比较值的最大值寄存器赋初值
    gus_mid_comp_val = gus_half_pwm;    //当前PWM输出比较值的中间值寄存器赋初值
    gus_min_comp_val = gus_half_pwm;    //当前PWM输出比较值的最小值寄存器赋初值

    rate_theta_buf.ulword = 0;
    rate_theta.ulword = 0;
    theta.ulword = 0; 
    
    gus_now_theta = 0;
    gus_next_theta = 0;
         
    gus_percent_k =0;
    
    gus_percent_pwm_buf = 0;
    gus_percent_pwm = 0;
    
    gus_freq_set = 0;
    //gus_freq_out_prev = 0;    
    gus_freq_out = 0;
    
    //bflg_theta_ctrl1 = 0;
    //bflg_theta_ctrl2 = 0;
    //gus_freq_real = 0;
    
    //gus_speed_real = 0;
    gus_Uout_for_disp = 0;
    gus_Uout_real = 0;
    gsl_FreqPI_integral = 0;
}
//------------------------------------------------------------------------------
void bootstrap_charger_delaytime(void)  //自举充电延时程序，在1ms定时程序中调用
{
	  if (bflg_bootstrap_charger_delaytime == 1)
	  {
	  	  gss_bootstrap_charger_delaytimer++;
	  	  
	  	  if (gss_bootstrap_charger_delaytimer > 700)
	  	  {
	  	  	  bflg_bootstrap_charger_delaytime = 0; //500计时到后，清自举电路充电标志
	  	  	  gss_bootstrap_charger_delaytimer = 0; //清计时器
	  	  }
	  	  else if (gss_bootstrap_charger_delaytimer > 400)
	  	  {
	  	  	  gus_val_a = gus_half_pwm;
	  	  	  gus_val_b = gus_half_pwm;
	  	  	  gus_val_c = gus_half_pwm;
	  	  	  
	  	  	  TCMP0A = gus_val_a;
	  	  	  TCMP0B = gus_val_b;
	  	  	  TCMP0C = gus_val_c;
	  	  	  
	  	  	  PWMSEL0 = 0x0540;
	  	  }
	  	  else if (gss_bootstrap_charger_delaytimer > 300)
	  	  {
	  	  	  gus_val_a = gus_half_pwm;
            gus_val_b = gus_half_pwm;
            gus_val_c -= gus_centi_half_pwm;
            
            TCMP0A = gus_val_a;
	  	  	  TCMP0B = gus_val_b;
	  	  	  TCMP0C = gus_val_c;
	  	  	  
	  	  	  PWMSEL0 = 0x0540; //100ms后W下臂也开始PWM输出
	  	  }
	  	  else if (gss_bootstrap_charger_delaytimer > 200)
	  	  {
	  	  	  gus_val_a = gus_half_pwm;
            gus_val_b -= gus_centi_half_pwm;
            gus_val_c = gus_peak_pwm + 1;
            
            TCMP0A = gus_val_a;
	  	  	  TCMP0B = gus_val_b;
	  	  	  TCMP0C = gus_val_c;
	  	  	  
	  	  	  PWMSEL0 = 0x0D40; //100ms后V下臂也开始PWM输出
	  	  }
	  	  else if (gss_bootstrap_charger_delaytimer > 100)
	  	  {
	  	  	  gus_val_a -= gus_centi_half_pwm;
            gus_val_b = gus_peak_pwm + 1;
            gus_val_c = gus_peak_pwm + 1;
            
            TCMP0A = gus_val_a;
	  	  	  TCMP0B = gus_val_b;
	  	  	  TCMP0C = gus_val_c;
	  	  	  
	  	  	  PWMSEL0 = 0x0F40; //U下臂先开始PWM输出
	  	  }
	  }
}
//------------------------------------------------------------------------------
void check_freq_updata(void)  //定时进行频率更新的程序(每1ms中断中检查1次)
{
    uint16_t lus_tmp;
    uint16_t lus_stall_amp_accel;
    
    if (bflg_running == 1)    //运行时，进入
    {
        //gus_freq_out_prev = gus_freq_out;    //保存上一次的输出频率
        //--------------------------------------------------
        if (bflg_current_direction == bflg_actual_direction)   //如果实际转向与要求转向一致
        {
		        if (gus_freq_set > gus_freq_out)     //加速时，进入
		        {
		            freq_rate.sum_decel.ulword = 0;  //加速时，清减速步长尾数累计寄存器
		            //bflg_ask_for_speeddown = 0;      //清除要求减速标志位
		            //bflg_ask_for_speedup = 1;        //置要求加速标志位
		            //----------------------------------------------
		            //得到当前加速时的失速赔率
		            lus_stall_amp_accel = gus_prot_Iout_stall_amp > gus_prot_Tm_stall_amp ? gus_prot_Iout_stall_amp : gus_prot_Tm_stall_amp;           
		            gus_stall_amp_accel = lus_stall_amp_accel > gus_prot_Udc_stall_amp ? lus_stall_amp_accel : gus_prot_Udc_stall_amp;           
		            //----------------------------------------------
		            freq_rate.sum_accel.ulword += (freq_rate.normal_accel >> gus_stall_amp_accel);
		            //----------------------------------------------
		            //hz_out的上限不得大于600.0Hz
		            lus_tmp = gus_freq_out;
		            lus_tmp += freq_rate.sum_accel.ubyte.HH;  //只取高字的高字节(即缩小2^24倍)
		            freq_rate.sum_accel.ubyte.HH = 0;          //清高字的高字节，保留其他字节，以便保留加法的尾数         
		            //----------------------------------------------
		            if(gus_freq_set < lus_tmp)
		            {   
		               lus_tmp = gus_freq_set;
		            }
		            //----------------------------------------------
		            //启动频率判断
		            if (lus_tmp < ram_para[num_begin_freq])
		            {
		            	  lus_tmp = ram_para[num_begin_freq];
		            }
		            //----------------------------------------------
		            gus_freq_out = lus_tmp;
		            //----------------------------------------------
		        }
		        else if (gus_freq_set < gus_freq_out)//减速时，进入
		        {
		            freq_rate.sum_accel.ulword = 0;  //减速时，清加速步长尾数累计寄存器
		            //bflg_ask_for_speedup = 0;        //清除要求加速标志位
		            //bflg_ask_for_speeddown = 1;      //置要求减速标志位
		            //----------------------------------------------
		            //得到当前减速时的失速配率
		            gus_stall_amp_decel = gus_prot_Udc_stall_amp;
		            //----------------------------------------------
		            freq_rate.sum_decel.ulword += (freq_rate.normal_decel >> gus_stall_amp_decel);
		            //----------------------------------------------
		            lus_tmp = gus_freq_out;
		            //----------------------------------------------
		            if (lus_tmp > freq_rate.sum_decel.ubyte.HH)
		            {
		                lus_tmp -= freq_rate.sum_decel.ubyte.HH;   //只取高字的高字节(即缩小2^24倍)
		            }
		            else
		            {
		                lus_tmp = 0;
		            }
		            //----------------------------------------------
		            freq_rate.sum_decel.ubyte.HH = 0; //清高字的高字节，保留其他字节，以便保留加法的尾数            
		            //----------------------------------------------
		            if(gus_freq_set > lus_tmp)
		            {
		                lus_tmp = gus_freq_set;
		            } 
		            //----------------------------------------------
		            gus_freq_out = lus_tmp;
		            //----------------------------------------------
		        }
		        else
		        {            
		            //bflg_ask_for_speedup = 0;        //清除要求加速标志位
		            //bflg_ask_for_speeddown = 0;      //清除要求减速标志位
		            freq_rate.sum_decel.ulword = 0;  //清减速步长尾数累计寄存器
		            freq_rate.sum_accel.ulword = 0;  //清加速步长尾数累计寄存器
		            //----------------------------------------------
		            gus_freq_out = gus_freq_set;
		            //----------------------------------------------
		            gus_stall_amp_accel = 0;
		            gus_stall_amp_decel = 0;
		            //----------------------------------------------
		        }
        }
        else
        {
        	  //gus_freq_out = 0;
        	  
        	      freq_rate.sum_accel.ulword = 0;  //减速时，清加速步长尾数累计寄存器
		            //bflg_ask_for_speedup = 0;        //清除要求加速标志位
		            //bflg_ask_for_speeddown = 1;      //置要求减速标志位
		            //----------------------------------------------
		            //得到当前减速时的失速配率
		            gus_stall_amp_decel = gus_prot_Udc_stall_amp;
		            //----------------------------------------------
		            freq_rate.sum_decel.ulword += (freq_rate.normal_decel >> (gus_stall_amp_decel << 1));
		            //----------------------------------------------
		            lus_tmp = gus_freq_out;
		            //----------------------------------------------
		            if (lus_tmp > freq_rate.sum_decel.ubyte.HH)
		            {
		                lus_tmp -= freq_rate.sum_decel.ubyte.HH;   //只取高字的高字节(即缩小2^24倍)
		            }
		            else
		            {
		                lus_tmp = 0;
		            }
		            //----------------------------------------------
		            freq_rate.sum_decel.ubyte.HH = 0;   //清高字的高字节，保留其他字节，以便保留加法的尾数            
		            //----------------------------------------------
		            /*if(gus_freq_set > lus_tmp)
		            {
		                lus_tmp = gus_freq_set;
		            }*/
		            //----------------------------------------------
		            gus_freq_out = lus_tmp;
		            //----------------------------------------------
        }
        //--------------------------------------------------
        //计算输出转速 r=(f/100)*60/P
        /*lus_tmp = gus_freq_out;
        lus_tmp *= 3;
        lus_tmp /= motor_para[guc_motor_ID][MOTOR_p];
        lus_tmp /= 5;
        gus_speed_out = lus_tmp;*/
        //--------------------------------------------------
        /*if (bflg_startup_time == 1)     //如果是启动阶段，则进入
        {
            if (gus_freq_out >= ram_para[num_base_freq])    //输出频率大于等于基准频率，则退出启动阶段
            {
                bflg_startup_time = 0;  //清启动阶段标志
            }
        }*/
    }
}
//------------------------------------------------------------------------------
//根据实际检测频率与设定频率的差值PI出实际的输出电压
//------------------------------------------------------------------------------
void Sensored_cale_Vout(void)      //带位置传感器控制计算输出电压的程序
{
	  int32_t lsl_tmp;
	  //------------------------------------------------------
	  gus_PIctrl_delaytimer++;
	  if (gus_PIctrl_delaytimer >= ram_para[num_PIctrl_delaytime]) //PI控制周期
	  {
	  	  gus_PIctrl_delaytimer = 0;
	  	  //--------------------------------------------------
	  	  //Ksp放大(2^15)*(2^8)倍,Ksi放大(2^15)*(2^8)倍
	  	  //--------------------------------------------------
	  	  if (bflg_current_direction == bflg_actual_direction)
	  	  {
	  	  	  lsl_tmp = gus_freq_out;
	  	      lsl_tmp -= gus_Hall_freq;//gus_avr_realfreq;
	  	  }
	  	  else
	  	  {
	  	  	  lsl_tmp = gus_Hall_freq;//gus_avr_realfreq;
	  	      lsl_tmp -= gus_freq_out;
	  	  }
	  	  //lsl_tmp = gus_freq_out;
	  	  //lsl_tmp -= gus_avr_realfreq;
	  	  lsl_tmp *= ram_para[num_Ksi];
	  	  gsl_FreqPI_integral += lsl_tmp;
	  	  if (gsl_FreqPI_integral < 0)
	  	  {
	  	  	  gsl_FreqPI_integral = 0;
	  	  }
	  	  else if (gsl_FreqPI_integral > 8388608)//32768 * 256)//debug
	  	  {
	  	  	  gsl_FreqPI_integral = 8388608;//32768 * 256;
	  	  }
	  	  //--------------------------------------------------
	  	  //lsl_tmp = gss_PI_DeltSpeed;
	  	  if (bflg_current_direction == bflg_actual_direction)
	  	  {
	  	  	  lsl_tmp = gus_freq_out;
	  	      lsl_tmp -= gus_Hall_freq;//gus_avr_realfreq;
	  	  }
	  	  else
	  	  {
	  	  	  lsl_tmp = gus_Hall_freq;//gus_avr_realfreq;
	  	      lsl_tmp -= gus_freq_out;
	  	  }
	  	  //lsl_tmp = gus_freq_out;
	  	  //lsl_tmp -= gus_avr_realfreq;
	  	  lsl_tmp *= ram_para[num_Ksp];
	  	  lsl_tmp += gsl_FreqPI_integral;    //积分环节叠加到比例环节上
	  	  //--------------------------------------------------
	  	  if (lsl_tmp < 0)
	  	  {
	  	  	  lsl_tmp = 0; //最小为零
	  	  }
	  	  else if (lsl_tmp > 8388608)//32768 * 256)
	  	  {
	  	  	  lsl_tmp = 8388608;//32768 * 256;  //最大不能超过最大的调制率
	  	  }
	  	  //--------------------------------------------------
	  	  //缩小32768倍后为实际调制率,再乘以最大输出线电压的有效值，得到要求输出的线电压
	  	  lsl_tmp >>= 8;
	  	  lsl_tmp *= ram_para[num_V_max];
	  	  lsl_tmp >>= 15;       //最后整个再缩小(2^15)倍
	  	  //--------------------------------------------------
	  	  gus_Uout_real = (uint16_t) lsl_tmp;   //得到要求输出的线电压，精度到0.1V，用于计算调制率
	  }
}
//-------------------------------------------------------------------------------
void calc_percent_theta(void)      //用于计算当前调制率和相位角步长的程序
{
    lword_type lul_tmp;
    int32_t lsl_tmp;
    //------------------------------------------------------
    lsl_tmp = gus_Udc_ad;
    lsl_tmp -= ram_para[num_Udc_ref_AD];
    lsl_tmp *= 10;                                //乘以10将电压精度扩大到0.1V
    lsl_tmp *= ram_para[num_Udc_gain];
    lsl_tmp >>= ram_para[num_Udc_gain_amp];
    lsl_tmp += (ram_para[num_Udc_ref]*10);        //Udc电压精度为0.1V
    
    if (lsl_tmp < 1000) lsl_tmp = 1000; //防止溢出
    
    gus_Udc_tmp = lsl_tmp;
    
    //单位电压调制率＝[1/(直流电压/sqrt(2))],得到单位调制率的*(2^16)*4096倍,电压精度到0.1V
    //percent_pwr_unit = 4096*(2^16)/[(Udc_val)/sqrt(2)]
    gul_percent_pwm_unit = 379625062 / gus_Udc_tmp;
    //------------------------------------------------------
    //计算当前调制率
    lul_tmp.ulword = gul_percent_pwm_unit;
    lul_tmp.ulword *= gus_Uout_real;
    //------------------------------------------------------
    if (ram_para[num_drive_mode] == 0)  //如果不允许台形波驱动模式，则进入
    {
        bflg_taiwave_ctrl = 0; 
    }       
    else  //如果允许台型波
    {
        //当输出频率大于切换频率，则开始进行台形波驱动输出
        if (bflg_taiwave_ctrl == 0)
        {
            //当达到切换频率后且调制率大于切换调制率，切换为台形波调制
            if (lul_tmp.uword.high > gus_max_percent_gu)
            {
                bflg_askfor_guwave_ctrl = 0;      //清请求进入股形波调制标志，保证互斥
                bflg_askfor_taiwave_ctrl = 1;     //置请求进入台形波驱动标志
            }
        }
        else
        {
            //或调制率小于台形波最小调制率，则切换回股形波调制
            if (lul_tmp.uword.high < (gus_max_percent_gu - 200))
            {
                bflg_askfor_taiwave_ctrl = 0;     //清请求进入台形波调制标志，保证互斥
                bflg_askfor_guwave_ctrl = 1;      //置请求进入股形波调制标志
            }
        }
    }
    //------------------------------------------------------
    if (((bflg_taiwave_ctrl == 1) && (bflg_askfor_guwave_ctrl == 0)) || (bflg_askfor_taiwave_ctrl == 1))
    {   //如果是切台型波
    	  //台形波调制时为4K*gus_peak_pwm
    	  if (lul_tmp.uword.high > gus_max_percent) 
        {
            lul_tmp.uword.high = gus_max_percent;
        }
        //--------------------------------------------------
        if (lul_tmp.uword.high < gus_min_percent)
        {
            lul_tmp.uword.high = gus_min_percent; //台形波调制时，调制率不能小于最小调制
        }
        //--------------------------------------------------
        gus_percent_k = lul_tmp.uword.high;       //实际调制率的4096倍
        //--------------------------------------------------
        lul_tmp.ulword = lul_tmp.uword.high;
        lul_tmp.ulword *= gus_peak_pwm;
        lul_tmp.ulword >>= 10;
        gus_percent_pwm_buf = lul_tmp.ulword;     //缩小1024倍，少缩小4倍，为了跟sin查表值相乘后，取高字
    }
    else
    {
    	  //股形波调制时，为4K*half
    	  if (lul_tmp.uword.high > gus_max_percent_gu) 
        {
            lul_tmp.uword.high = gus_max_percent_gu;
        }
        //--------------------------------------------------
        gus_percent_k = lul_tmp.uword.high;       //实际调制率的4096倍
        //--------------------------------------------------
        lul_tmp.ulword = lul_tmp.uword.high;
        lul_tmp.ulword *= gus_half_pwm;
        lul_tmp.ulword >>= 10;
        gus_percent_pwm_buf = lul_tmp.ulword;   //缩小1024倍，少缩小4倍，为了跟sin查表值相乘后，取高字
    }
    //------------------------------------------------------
    //计算实际输出电压显示值   Udc*percent_K/4096/(2^0.5)
    lul_tmp.ulword = gus_Udc_for_disp;
    lul_tmp.ulword *= gus_percent_k;
    lul_tmp.ulword *= 181;
    lul_tmp.ulword >>= 12;
    lul_tmp.ulword >>= 8;
    gus_Uout_for_disp = lul_tmp.ulword; 
    //------------------------------------------------------
    //计算rate_theta值;rate_per的计算已经在初始化阶段完成，这里只要做一步乘法就可以了
    //if (bflg_actual_direction != bflg_current_direction)
    //{
    //	  rate_theta_buf.ulword = 0;
    //}
    //else
    //{
    	  if (bflg_theta_ctrl2 == 0)///
    	  {
    	  	  rate_theta_buf.ulword = 0;
    	  }
    	  else
    	  {
    	  	  rate_theta_buf.ulword = gus_Hall_freq;
            rate_theta_buf.ulword *= gus_rate_per;
    	  }
    //}
    //rate_theta_buf.ulword = gus_Hall_freq;
    //rate_theta_buf.ulword *= gus_rate_per;
    //------------------------------------------------------
    bflg_pwm_allow_updata = 1;//置pwm允许更新标志
}
//------------------------------------------------------------------------------
void calc_comp_val(void)      //计算下一次三相输出的比较计数值，在波形发生中断中调用  
{
    //入口参数gus_percent_pwm、rate_theta
    //出口参数theta，gus_val_a,gus_val_b,gus_val_c
    
    uint16_t lus_next_theta;  //下一个载波周期的A相相位角
    lword_type lul_theta;

    //计算三相输出比较计数值
    //------------------------------------------------------
    theta.ulword += rate_theta.ulword;
    while (theta.uword.high >= SIN_TAB_LENGTH_4)
    {
        theta.uword.high -= SIN_TAB_LENGTH_4;
    }
    //------------------------------------------------------
    lul_theta.ulword = theta.ulword;
    lul_theta.uword.high += gus_phase_adv_degree;
    while (lul_theta.uword.high >= SIN_TAB_LENGTH_4)
    {
        lul_theta.uword.high -= SIN_TAB_LENGTH_4;
    }
    //------------------------------------------------------
    if (bflg_taiwave_ctrl == 1)    //如果是台形波调制，则进入
    {
        lus_next_theta = lul_theta.uword.high;
        gus_val_a = get_tai_comp_value(lus_next_theta);
        //--------------------------------------------------
        lus_next_theta = lul_theta.uword.high + SIN_TAB_LENGTH_8_3;
        gus_val_b = get_tai_comp_value(lus_next_theta);
        //--------------------------------------------------
        lus_next_theta = lul_theta.uword.high + SIN_TAB_LENGTH_4_3;
        gus_val_c = get_tai_comp_value(lus_next_theta);
    }
    else       //如果是股形波调制，则进入
    {
        lus_next_theta = lul_theta.uword.high;
        gus_val_a = get_svpwm_comp_value(lus_next_theta);
        //--------------------------------------------------
        lus_next_theta = lul_theta.uword.high + SIN_TAB_LENGTH_8_3;
        gus_val_b = get_svpwm_comp_value(lus_next_theta);
        //--------------------------------------------------
        lus_next_theta = lul_theta.uword.high + SIN_TAB_LENGTH_4_3;
        gus_val_c = get_svpwm_comp_value(lus_next_theta);  
    }
}
//------------------------------------------------------------------------------
int16_t get_sin_value(uint16_t now_sin_index)     //查sin表的函数
{
    short lsi_sin_val;
    
    while (now_sin_index >= SIN_TAB_LENGTH_4)
    {
        now_sin_index -= SIN_TAB_LENGTH_4;
    }
    //----------------------------------
    if (now_sin_index < SIN_TAB_LENGTH)
    {
        lsi_sin_val = sin_table[now_sin_index];
    }
    else if (now_sin_index < SIN_TAB_LENGTH_2)
    {
        now_sin_index = SIN_TAB_LENGTH_2 - now_sin_index;
        lsi_sin_val = sin_table[now_sin_index];
    }
    else if (now_sin_index < SIN_TAB_LENGTH_3)
    {  
        now_sin_index -= SIN_TAB_LENGTH_2;
        lsi_sin_val = -sin_table[now_sin_index];
    }
    else
    {
        now_sin_index = SIN_TAB_LENGTH_4 - now_sin_index;
        lsi_sin_val = -sin_table[now_sin_index];
    }
    
    return   lsi_sin_val; 
}
//------------------------------------------------------------------------------
uint16_t get_tai_comp_value(uint16_t now_index)   //台形波调制时计算PWM输出计数值的程序
{
    lword_type lsl_tai_val;
    
    while (now_index >= SIN_TAB_LENGTH_4)
    {
        now_index -= SIN_TAB_LENGTH_4;
    } 
    //----------------------------------
    if (now_index < SIN_TAB_LENGTH_2_3)
    {
        now_index += SIN_TAB_LENGTH_1_3;
        lsl_tai_val.lword = get_sin_value(now_index);
        lsl_tai_val.lword *= gus_percent_pwm;
        
        if (lsl_tai_val.word.high < gss_min_pwm)//debug
        {
        	  lsl_tai_val.word.high = gss_min_pwm;
        }
       	if (lsl_tai_val.word.high > gss_max_pwm)
       	{
       		  lsl_tai_val.word.high = gss_max_pwm;
       	}
    }
    else if (now_index < SIN_TAB_LENGTH_4_3)
    {
        lsl_tai_val.word.high = gus_peak_pwm + 1; 
    }
    else if (now_index < SIN_TAB_LENGTH_2)
    {
        now_index -= SIN_TAB_LENGTH_1_3;
        lsl_tai_val.lword = get_sin_value(now_index);
        lsl_tai_val.lword *= gus_percent_pwm;
        
        if (lsl_tai_val.word.high < gss_min_pwm)//debug
        {
        	  lsl_tai_val.word.high = gss_min_pwm;
        }
       	if (lsl_tai_val.word.high > gss_max_pwm)
       	{
       		  lsl_tai_val.word.high = gss_max_pwm;
       	}
    }  
    else if (now_index < SIN_TAB_LENGTH_8_3)
    {
        now_index += SIN_TAB_LENGTH_1_3;
        lsl_tai_val.lword = get_sin_value(now_index);
        lsl_tai_val.lword *= gus_percent_pwm;
        lsl_tai_val.word.high += gus_peak_pwm; 
        
        if (lsl_tai_val.word.high < gss_min_pwm)//debug
        {
        	  lsl_tai_val.word.high = gss_min_pwm;
        }
       	if (lsl_tai_val.word.high > gss_max_pwm)
       	{
       		  lsl_tai_val.word.high = gss_max_pwm;
       	}
    }
    else if (now_index < SIN_TAB_LENGTH_10_3)
    {
        lsl_tai_val.word.high = 0;
    }
    else if (now_index < SIN_TAB_LENGTH_4)
    {
        now_index -= SIN_TAB_LENGTH_1_3;
        lsl_tai_val.lword = get_sin_value(now_index);
        lsl_tai_val.lword *= gus_percent_pwm;
        lsl_tai_val.word.high += gus_peak_pwm;
        
        if (lsl_tai_val.word.high < gss_min_pwm)//debug
        {
        	  lsl_tai_val.word.high = gss_min_pwm;
        }
       	if (lsl_tai_val.word.high > gss_max_pwm)
       	{
       		  lsl_tai_val.word.high = gss_max_pwm;
       	}
    }
    
    return   lsl_tai_val.word.high;  
}
//------------------------------------------------------------------------------
uint16_t get_svpwm_comp_value(uint16_t now_index) //股形波调制时计算PWM输出计数值的程序
{
    lword_type lsl_svpwm_val;
    
    while (now_index >= SIN_TAB_LENGTH_4)
    {
       now_index -= SIN_TAB_LENGTH_4;
    } 
    //----------------------------------
    if (now_index < SIN_TAB_LENGTH_1_3)
    {
        lsl_svpwm_val.lword = gus_percent_pwm;  
        lsl_svpwm_val.lword *= 14189;      //(3^0.5)约为 14189/2^13
        lsl_svpwm_val.lword >>= 13;
        lsl_svpwm_val.lword *= get_sin_value(now_index);
        lsl_svpwm_val.word.high += gus_half_pwm;
    }
    else if (now_index < SIN_TAB_LENGTH)
    {
        now_index += SIN_TAB_LENGTH_1_3;
        lsl_svpwm_val.lword = get_sin_value(now_index);
        lsl_svpwm_val.lword *= gus_percent_pwm;
        lsl_svpwm_val.word.high += gus_half_pwm;
    }
    else if (now_index < SIN_TAB_LENGTH_5_3)
    {  
        now_index -= SIN_TAB_LENGTH_1_3;
        lsl_svpwm_val.lword = get_sin_value(now_index);
        lsl_svpwm_val.lword *= gus_percent_pwm;
        lsl_svpwm_val.word.high += gus_half_pwm;
    }
    else if (now_index < SIN_TAB_LENGTH_7_3)
    { 
        lsl_svpwm_val.lword = gus_percent_pwm;  
        lsl_svpwm_val.lword *= 14189;      //(3^0.5)约为 14189/2^13
        lsl_svpwm_val.lword >>= 13;     
        lsl_svpwm_val.lword *= get_sin_value(now_index);
        lsl_svpwm_val.word.high += gus_half_pwm;
    }
    else if (now_index < SIN_TAB_LENGTH_3)
    { 
        now_index += SIN_TAB_LENGTH_1_3;
        lsl_svpwm_val.lword = get_sin_value(now_index);
        lsl_svpwm_val.lword *= gus_percent_pwm;
        lsl_svpwm_val.word.high += gus_half_pwm;      
    }
    else if (now_index < SIN_TAB_LENGTH_11_3)
    { 
        now_index -= SIN_TAB_LENGTH_1_3;
        lsl_svpwm_val.lword = get_sin_value(now_index);
        lsl_svpwm_val.lword *= gus_percent_pwm;
        lsl_svpwm_val.word.high += gus_half_pwm;
    }
    else if (now_index < SIN_TAB_LENGTH_4)
    { 
        lsl_svpwm_val.lword = gus_percent_pwm;  
        lsl_svpwm_val.lword *= 14189;      //(3^0.5)约为 14189/2^13
        lsl_svpwm_val.lword >>= 13;
        lsl_svpwm_val.lword *= get_sin_value(now_index);
        lsl_svpwm_val.word.high += gus_half_pwm;
    }   
    
    if (lsl_svpwm_val.word.high < gss_min_pwm)
    {
    	  lsl_svpwm_val.word.high = gss_min_pwm;
    }
    if (lsl_svpwm_val.word.high > gss_max_pwm)
    {
    	  lsl_svpwm_val.word.high = gss_max_pwm;//debug 
    }
    
    return lsl_svpwm_val.word.high;               
}
//------------------------------------------------------------------------------
void PWM_Reload_INT(void)     //波形发生中断
{
    if (bflg_running == 1)
    {
        //--------------------------------------------------
        //加载下个周期的计数值
        if (bflg_current_direction == 0)//如果要求逆时针旋转，则进入
        {
            //占空比缓冲寄存器赋值
            //以U相为基准
            TCMP0C = gus_val_a;    //PWM00占空比设定  u相
            TCMP0B = gus_val_c;    //PWM01占空比设定  w相
            TCMP0A = gus_val_b;    //PWM02占空比设定  v相
        }
        else                            //如果要求顺时针旋转，则进入
        {
            //占空比缓冲寄存器赋值  
            //以U相为基准
            TCMP0C = gus_val_a;    //PWM02占空比设定  u相
            TCMP0B = gus_val_b;    //PWM01占空比设定  v相
            TCMP0A = gus_val_c;    //PWM00占空比设定  w相
        }
        //--------------------------------------------------
        //判断本周期能否取电流AD值
        if (bflg_taiwave_ctrl == 0)     //如果不是台型波
        {
        	  if (((gus_max_comp_val - gus_mid_comp_val) >= (gus_ADtrigger_repair_val + gus_AD_SH_val))
       	     || ((gus_mid_comp_val - gus_min_comp_val) >= (gus_ADtrigger_repair_val + gus_AD_SH_val)))
       	    {
       	    	  bflg_allow_get_Idc = 1;      //允许采集电流AD
       	    	  
       	    	  if ((gus_max_comp_val - gus_mid_comp_val) > (gus_mid_comp_val - gus_min_comp_val))
       	    	  {
       	    	  	  bflg_up_down_get_Idc = 0;     //下降沿采集
       	    	  }
       	    	  else
       	    	  {
       	    	  	  bflg_up_down_get_Idc = 1;     //上升沿采集
       	    	  }
       	    }
       	    else
       	    {
       	    	  bflg_allow_get_Idc = 0;      //不允许采集电流AD
       	    }
        }
        else
        {
        	  bflg_allow_get_Idc = 1;          //允许采集电流AD
        	  if (gus_max_comp_val == gus_peak_pwm + 1)
        	  {
        	  	  bflg_up_down_get_Idc = 0;    //下降沿采集
        	  }
        	  else if (gus_min_comp_val == 0)
        	  {
        	  	  bflg_up_down_get_Idc = 1;    //上升沿采集
        	  }
        	  else
        	  {
        	  	  bflg_allow_get_Idc = 0;      //不允许采集电流AD
        	  }
        }
        //--------------------------------------------------
        //对下周期三相计数值进行排序
        comp_val_order();
        //--------------------------------------------------
        //确定下周期上升计数和下降计数的AD0触发时刻         
        //PWM上升计数时，取mid和min之间的采样点
        //PWM下降计数时，取max和mid之间的采样点
        if (bflg_taiwave_ctrl == 0)     //如果不是台型波
        {
        	  gus_TUP_ADtrigger = (gus_min_comp_val + gus_mid_comp_val + gus_ADtrigger_repair_val - gus_AD_SH_val) >> 1;
            gus_TDW_ADtrigger = (gus_max_comp_val + gus_mid_comp_val + gus_AD_SH_val - gus_ADtrigger_repair_val) >> 1;
        }
        else        //如果是台型波
        {
        	  gus_TUP_ADtrigger = gus_ADtrigger_repair_val;
            gus_TDW_ADtrigger = gus_peak_pwm - gus_TUP_ADtrigger;
        }
        
        TM20CB = gus_TUP_ADtrigger;
        TM20CA = ((gus_peak_pwm << 1) - gus_TDW_ADtrigger);
        //--------------------------------------------------
        //更新调制率和相位增量
        if (bflg_pwm_allow_updata == 1) //如果允许PWM进行更新，则进入更新调制率和相位增量
        {
            bflg_pwm_allow_updata = 0;  //清允许PWM更新标志
            //----------------------------------------------
            //为了保证标志位和计算值的一致性 //只有在真正加载到对应的计算值且
            //开始计算计数值时，才置台形波驱动标志
            //----------------------------------------------
            if (bflg_askfor_taiwave_ctrl == 1)    //如果是请求进行台形波调制，则进入
            {
                bflg_taiwave_ctrl = 1;            //置台形波调制标志
                bflg_askfor_taiwave_ctrl = 0;     //清请求台形波调制标志
            }
            else if (bflg_askfor_guwave_ctrl == 1)//如果是请求进行股形波调制，则进入
            {
                bflg_taiwave_ctrl = 0;            //清台形波调制标志
                bflg_askfor_guwave_ctrl = 0;      //清请求股形波调制标志                      
            }
            //将缓冲器的值写入真正的寄存器中
            gus_percent_pwm = gus_percent_pwm_buf;
            rate_theta.ulword = rate_theta_buf.ulword;
        }
        //--------------------------------------------------
        //计算下下周期相位角、计数值
        calc_comp_val(); //计算下下周期三相输出的比较计数值,在波形发生中断中调用
        //--------------------------------------------------
    }
    else
    {
        bflg_allow_get_Idc = 1;         //允许采集电流AD
        
        gus_TUP_ADtrigger = gus_half_pwm;
        gus_TDW_ADtrigger = gus_peak_pwm;
        
        TM20CB = gus_TUP_ADtrigger;
        TM20CA = ((gus_peak_pwm << 1) - gus_TDW_ADtrigger);
    }
    //------------------------------------------------------
    get_Udc_ad();        //得到计算Udc的相关AD值
    //------------------------------------------------------
    prot_ALM_deal();      //ALM保护处理程序
}
//------------------------------------------------------------------------------
void comp_val_order(void)     //对三相比较值进行排序
{
    gus_now_theta = gus_next_theta;     //得到当前控制周期的相位角
    gus_next_theta = theta.uword.high;  //得到下个控制周期的相位角
    //------------------------------------------------------
    if (gus_val_a > gus_val_b)
    {
        if (gus_val_a > gus_val_c)
        {
            if (gus_val_b > gus_val_c)
            {
                gus_max_comp_val = gus_val_a;
                gus_mid_comp_val = gus_val_b;
                gus_min_comp_val = gus_val_c;
                
                gus_val_abc_order = ORDER_ABC;
            }
            else
            {
                gus_max_comp_val = gus_val_a;
                gus_mid_comp_val = gus_val_c;
                gus_min_comp_val = gus_val_b;   
                
                gus_val_abc_order = ORDER_ACB;                    
            }
        }
        else
        {
            gus_max_comp_val = gus_val_c;
            gus_mid_comp_val = gus_val_a;
            gus_min_comp_val = gus_val_b;
            
            gus_val_abc_order = ORDER_CAB;
        }
    }
    else
    {
        if (gus_val_b > gus_val_c)
        {
            if (gus_val_a > gus_val_c)
            {
                gus_max_comp_val = gus_val_b;
                gus_mid_comp_val = gus_val_a;
                gus_min_comp_val = gus_val_c;
                
                gus_val_abc_order = ORDER_BAC;
            }
            else
            {
                gus_max_comp_val = gus_val_b;
                gus_mid_comp_val = gus_val_c;
                gus_min_comp_val = gus_val_a;           
                
                gus_val_abc_order = ORDER_BCA;
            }
        }
        else
        {
            gus_max_comp_val = gus_val_c;
            gus_mid_comp_val = gus_val_b;
            gus_min_comp_val = gus_val_a;
            
            gus_val_abc_order = ORDER_CBA;
        }        
    }                  
}
//------------------------------------------------------------------------------
#endif            //end 
