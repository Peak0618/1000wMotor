//------------------------------------------------------------------------------
//包含文件
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
//函数声明
void main(void);              //主程序入口

void ports_configure(void);   //端口配置程序

void read_para_RomtoRam(void);//从rom中读取参数入ram

void control_init(void);      //在上电阶段，用于初始化一些控制量的程序

void TM23_configure(void);    //TMR23模块配置程序

void TM23_1MS_INT(void);      //1ms定时中断服务程序

void main_loop_deal(void);    //主循环处理程序

void timer_op(void);          //定时处理子程序，用于处理所有的定时延时程序  

void fan_ctrl_deal(void);     //压缩机控制程序

void freq_ctrl_deal(void);    //频率控制程序

uint16_t get_min_decfreq_deal(void);    //两个保护频率排序并得到最小不为0的保护频率的程序，在频率设定程序中调用

void check_powerup_delaytime(void);     //上电后的延时程序，在1ms定时程序中调用

void check_run_stop_deal(void);         //检查开关机处理的的程序

void run_key(void);                     //运行键处理

void stop_key(void);

//void check_auto_start(void);            //无论何种原因，只要要求启动即调用此程序

void check_auto_stop(void);             //正常停机时，调用的程序

//void check_free_stop(void); 

//void set_freq_op(void); 

void check_spry_ctrl_term(void);        //检查spry控制条件的程序，在100ms定时程序中调用

void check_spry_delaytime(void);        //主继电器吸合和断开的延时计时程序，在1ms定时程序中调用

void stop_space_delaytime(void);        //停机间隔延时程序，在1s定时程序中调用

void trip_lampblink_deal(void);         //故障灯闪烁程序,在10ms定时任务中调用

//------------------------------------------------------------------------------
flag_type flg_ctrl;
flag_type flg_delaytime;
flag_type flg_theta;
flag_type flg_SCI1;

int16_t   gss_timer_10ms;     //10ms计数寄存器
int16_t   gss_timer_100ms;    //100ms计数寄存器
int16_t   gss_timer_1s;       //1s计数寄存器
int16_t   gss_timer_1min;     //1min计数寄存器  


int16_t   gss_powerup_delaytimer;       //上电延时计时器
int16_t   gss_spry_delaytimer;//spry延时计时器
//----------------------------------------------------------
uint16_t  main_loop_cnt;
//----------------------------------------------------------
uint16_t  gus_trip_flash_count;    //故障灯闪烁次数寄存器
uint16_t  gus_trip_flash_timer;    //故障灯闪烁计时寄存器
//------------------------------------------------------------------------------  
//------------------------------------------------------------------------------
void main(void)
{
    //------------------------------------------------------
    disable_irq();            //关总中断
    set_imask(0);             //禁止响应任何优先级的中断
    //watchdog_disable();     //关看门狗
    watchdog_enable();        //开启看门狗
    //----------------------------------
    ports_configure();        //端口配置程序
    //----------------------------------
    delay_5us(20000);         //延时100ms
    
    if (read_para_eeptoram() != 0)        //从eeprom中读取参数入ram-peak读错
    {
        bflg_trip_stop = 1;               //置故障停机标志 
        bflg_eeprom_change = 1;           //置EEPROM变更标志
        bflg_eeprom_error = 1;            //置EEPROM出错标志
        guc_trip_code = EEPROM_ERR_CODE;  //读EEPROM错
        guc_lamp_trip_code = LAMP_EEPROM_ERR_CODE;

        read_para_RomtoRam();             //从rom中读取参数入ram
    }
    
    //read_para_RomtoRam();     //从rom中读取参数入ram
    //ID号重新确认
    if ((ram_para[num_App_ID] < APP_ID_MIN) || (ram_para[num_App_ID] > APP_ID_MAX))
    {
    	  ram_para[num_App_ID] = APP_ID_MIN;
    }	 
    //----------------------------------
    TM23_configure();         //TM16配置程序,用于200us定时
    //----------------------------------
    Hall_configure();         //霍尔信号配置程序
    //----------------------------------
    ADC_configure();          //AD转换配置程序
    //----------------------------------
    PWM_configure();          //PWM模块配置程序
    //----------------------------------
    SCI0_configure();         //配置SCI0串行通讯接口
   
    SCI1_configure();         //配置SCI1串行通讯接口
    //----------------------------------
    control_init();           //在上电阶段，用于初始化一些控制量的程序
    //----------------------------------
    //----------------------------------  
    //----------------------------------
    set_imask(7);             //允许响应所有优先级的中断
    enable_irq();             //开总中断
    //watchdog_enable();      //开启看门狗 
    //----------------------------------
    while(1)
    {
        main_loop_deal();
        clear_watchdog();
    }
}
//------------------------------------------------------------------------------
void ports_configure(void)   //端口配置程序
{
    //Configure P00~P03 (P00 is 主继电器RY, P01 is NC, P02 is EEPROM-WP, P03 is ALM)
    P0PLU = 0x00;   //P00~P03 Pull-up resistor not added
    P0OUT = 0x00;  //P00=1;
    P0ODC = 0x05;   //P00/P01/P03=0 P02=1;(0=Push-pull output 1=Nch open-drain output)  
    P0DIR = 0x05;   //P00=RY=input mode,P01,P02,P03 is input mode (0=input mode 1=output mode)
    //模式寄存器在具体功能中配置
    P0MD = 0x08;    //P00,P01,P02 is IO, P03 is TM5IO pin/IRQ03 pin(0=IO 1=Special function)
    
    //Configure P20~P25 (P20=TXD, P21=EEPROM-SCL, P22=RXD, P23=面板通讯TXD, P24=EEPROM-SDA, P25=面板通讯RXD)
    P2PLU = 0x20;  //not Pull-up
    P2OUT = 0x3F;  //P20~P25=1;
    P2ODC = 0x12;  //P21/P24=1;(0=Push-pull output 1=Nch open-drain output)
    P2DIR = 0x1B;  // (0=input mode 1=output mode)
    //模式寄存器在具体功能中配置
    P2MD = 0x2D;   //(0=IO 1=Special function)
    
    //Configure P40,P41(P40 is HW, P41 is HV)
    P4PLU = 0x00;   //P40,P41 not Pull-up
    P4OUT |= 0x00;  //P40,P41 input
    P4DIR = 0x00;   //P40,P41 is input mode (0=input mode 1=output mode)
    //模式寄存器在具体功能中配置
    P4MD1 = 0x00;   //P40,P41 is TMnIO pin/IRQ pin (0=IO 1=Special function)
    P4MD2 = 0x00;   //selected by P4MD1 register   
    
    //Configure P50,P51 (P50 is HU, P51 is LED)
    P5PLU = 0x00;   //P50,P51 not Pull-up
    P5OUT |= 0x02; //P51 output H
    P5DIR = 0x02;   //P50 is input mode, P51 is output mode (0=input mode 1=output mode)
    //模式寄存器在具体功能中配置
    P5MD1 = 0x00;   //P50 is TMnIO pin/IRQ pin ,P51 is IO (0=IO 1=Special function)
    P5MD2 = 0x00;   //selected by P5MD1 register
    
    //Configure P60,P61 (P60=地址拨码1,P61=地址拨码2)
    P6PLU = 0x00;   //P60,P61 Pull-up resistor not added 
    P6OUT |= 0x00;  //P60,P61 input
    P6DIR = 0x00;   //P60,P61 is input mode (0=input mode 1=output mode)
    //模式寄存器在具体功能中配置
    P6MD1 = 0x00;   //P60,P61 is IO (0=IO 1=Special function)
    P6MD2 = 0x00;   //selected by P6MD1 register
    
    //Configure P80~P85 (P80~P85 is PWM pin)
    P8PLU = 0x00;   //P80~P85 not Pull-up
    P8OUT &= ~0x3F; //P80~P85 out L (invalid level is L)
    P8DIR = 0x3F;   //P80~P85 is output mode  (0=input mode 1=output mode)
    //模式寄存器在具体功能中配置
    P8MD = 0x00;    //P80~P85 Special function is PWM pin  (0=IO 1=PWM)
    
    //Configure PC0~PC7 (PC0 is Idc, PC1 is THM, PC2 is Vdc, PC3~PC7 is NC)
    PCPLU = 0x00;   //PC0~PC7 not Pull-up
    PCOUT |= 0x00;  //PC0~PC7 input
    PCDIR = 0x00;   //PC0~PC7 is input mode  (0=input mode 1=output mode)
    //模式寄存器在具体功能中配置
    PCMD = 0x00;    //PC0~PC7 is IO (0=IO 1=AD)
    
    //Configure P20,P21,P22,P02,P03 Input voltage level
    SWCNT = 0x00;  //P20,P21,P22 Input voltage level is 0.8Vdd ,P02,P03 Input voltage level is 0.8Vdd 
}
//------------------------------------------------------------------------------
void read_para_RomtoRam(void)      //从rom中读取参数入ram
{
    uint16_t i;
    
    for (i = 0; i < (MAX_PARA_NUMBER + 1); i++)
    {
        ram_para[i] = EEPROM_InitData[i][VAL_ROW];   //从EEP读参数到RAM中
        clear_watchdog();     //清看门狗
    }
}
//------------------------------------------------------------------------------
void control_init(void)       //在上电阶段，用于初始化一些控制量的程序
{
    uint16_t lus_tmp = 0;    
    //------------------------------------------------------
    if (ram_para[num_cmd_source] == 1)  //如果是上位机控制，则进入
    {
        bflg_host_ctrl = 1;   //置上位机控制标志
    }
    else
    {
        bflg_host_ctrl = 0;   //清上位机控制标志，有LCD面板控制
        
        if (ram_para[num_direction] == 1)
        {
            bflg_current_direction = 1; //置顺时针旋转标志
        }
        else
        {
            bflg_current_direction = 0; //置逆时针旋转标志
        }        
    }
    //------------------------------------------------------
    guc_comm_address = COMM_ADDRESS;
    //guc_comm_address += 1;
    //------------------------------------------------------
    //上电欠压延时500ms
    bflg_powerup_delaytime = 1;    //上电后，置初始上电延时标志
    gss_powerup_delaytimer = 0;    //清初始上电延时计时器
    //------------------------------------------------------
    //停机间隔初始化
    bflg_stop_space_delaytime = 1;
    gss_stop_space_delaytimer = 5;//ram_para[num_stop_space_delaytime];
    //------------------------------------------------------
    Iout_prot_variable_init();     //与Iout保护有关的变量初始化程序
    Udc_prot_variable_init();      //母线电压保护有关变量初始化程序
    //------------------------------------------------------
    freq_rate_init();         //频率增减速初始化程序
    //------------------------------------------------------
    //得到积分项的积分项初值
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
    gus_MAX_setfreq = lus_tmp; //得到最大设定频率值     
}
//------------------------------------------------------------------------------
void TM23_configure(void)     //TMR23模块配置程序
{
	  //--------------------------------------------------------------------------
    //将8bit计数器timer2和timer3配置成16bit定时计时器，产生1ms中断
    //用于1ms定时的产生
    //------------------------------------------------------
    //G4ICR：组4中断控制寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  15  |  14  |  13  |  12  |  11  |  10  |  9   |  8   | 
    // flag    |  -   |  LV2 |  LV1 |  LV0 |  -   |  IE2 |  IE1 |  IE0 | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |  -   | IR2  | IR1  | IR1  |  -   | ID2  | ID1  | ID0  | 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //LV2~LV0 : 中断优先级设置, 0到7(0最高  7禁止)
    //IE2~IE0 : 中断使能标志  IE2=TM16 compare/capture B IE1=TM16 compare/capture A  IE0=TM16 overflow/underflow
    //IR2~IR0 : 中断请求标志
    //ID2~ID0 : 中断检测标志     
    //--------------------------------------------------------------------------
    G4ICR = 0x4000; //禁止TM3中断,并将中断优先级设为4(0最高  7禁止)
    
    TM3MD &= 0x7F;  //禁止TM3BC计数
    TM2MD &= 0x7F;  //禁止TM2BC计数
    
    //TM03PSC：预分频控制寄存器0
    //--------------------------------------------------------------------------
    //reg_bit  |   7    |  6   |  5   |  4   |  3   |  2   |  1   |  0   |  
    // flag    |TMPSCNE0|  -   |  -   |  -   |  -   |  -   |  -   |  -   | 
    //at reset |   0    |  0   |  0   |  0   |  0   |  0   |  0   |  0   |
    //--------------------------------------------------------------------------
    //TMPSCNE0:预分频操作使能选择  0=禁止  1=使能
    //--------------------------------------------------------------------------
    TM03PSC = 0x00;
    
    //TM03EXPSC：外部预分频控制寄存器0
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0    |  
    // flag    |TM3IN |TM2IN |TM1IN |TM0IN |  -   |  -   |  -   |EXPSCNE0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0    |
    //--------------------------------------------------------------------------
    //TM3IN:T3计数时钟源选择  0=TM3IO pin input  1=IOCLK/128
    //TM2IN:T2计数时钟源选择  0=TM2IO pin input  1=IOCLK/128
    //TM1IN:T1计数时钟源选择  0=TM1IO pin input  1=IOCLK/128
    //TM0IN:T0计数时钟源选择  0=TM0IO pin input  1=IOCLK/128
    //EXPSCNE0:预分频(1/128)操作使能选择  0=禁止  1=使能
    //--------------------------------------------------------------------------
    TM03EXPSC = 0x00;
    
    //TM3BR: T3基本寄存器 (TM2BR与TM3BR类似)
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM3BR7|TM3BR6|TM3BR5|TM3BR4|TM3BR3|TM3BR2|TM3BR1|TM3BR0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM3BR7~0: 用于在TM3BC下溢时,进行计数的初值。	TM3BC下溢后,从TM3BR加载到TM3BC中。
    //--------------------------------------------------------------------------
    TM3BR = 0xFF;
    TM2BR = 0xFF;
    
    //TM3MD: T3模式寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM3CNE|TM3LDE|  -   |  -   |  -   |TM3CK2|TM3CK1|TM3CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM3CNE: 计数器运转使能选择  0=不运转  1=运转
    //TM3LDE: 计数器初始化   0=正常运转   1=初始化
    //TM3CK2~0: 计数时钟源选择  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=与T2级连   
    //          100=T0下溢      101=T1下溢    110=T2下溢     111=TM3IO pin输入(上升沿),IOCLK/128
    //--------------------------------------------------------------------------  
    TM3MD = 0x03;
    
    //TM2MD: T2模式寄存器
    //--------------------------------------------------------------------------
    //reg_bit  |  7   |  6   |  5   |  4   |  3   |  2   |  1   |   0  |  
    // flag    |TM2CNE|TM2LDE|  -   |  -   |  -   |TM2CK2|TM2CK1|TM2CK0| 
    //at reset |  0   |  0   |  0   |  0   |  0   |  0   |  0   |   0  |
    //--------------------------------------------------------------------------
    //TM2CNE: 计数器运转使能选择  0=不运转  1=运转
    //TM2LDE: 计数器初始化   0=正常运转   1=初始化
    //TM2CK2~0: 计数时钟源选择  000=IOCLK      001=IOCLK/8   010=IOCLK/32   011=与T1级连   
    //          100=T0下溢      101=T1下溢    110=禁止设置   111=TM2IO pin输入(上升沿),IOCLK/128
    //--------------------------------------------------------------------------
    TM2MD = 0x00;
    
    //1000us / (1 / 20M) - 1 = 19999 = 0x4E1F
    TM3BR = 0x4E;
    TM2BR = 0x1F;
    
    TM3MD |= 0x40;       //初始化TM3BC
    TM2MD |= 0x40;       //初始化TM2BC
    
    TM3MD &= ~0x40;      //清初始化TM3BC标志
    TM2MD &= ~0x40;      //清初始化TM2BC标志
    
    G4ICR &= ~0x0020;    //清除外中断TM3中断请求标志
    G4ICR |= 0x0200;     //置TM3下溢中断使能标志   
    //------------------------------------------------------
    gss_timer_10ms = 10;      //初始化10ms计数寄存器
    gss_timer_100ms = 100;    //初始化10ms计数寄存器
    gss_timer_1s = 1000;      //初始化1s计数寄存器
    gss_timer_1min = 60;      //初始化1min计数寄存器  
    //------------------------------------------------------
    TM3MD |= 0x80;       //启动TM3BC计数
    TM2MD |= 0x80;       //启动TM2BC计数
}
//------------------------------------------------------------------------------
void TM23_1MS_INT(void)  //1ms定时中断服务程序
{     
    //------------------------------------------------------
	  bflg_1ms_reach = 1;  //置1ms到时标志位
    //------------------------------------------------------
    gss_timer_10ms--;
    if (gss_timer_10ms <= 0)  //10ms计时到
    {
        bflg_10ms_reach = 1;
        gss_timer_10ms = 10;
    }
    //------------------------------------------------------
    gss_timer_100ms--;
    if (gss_timer_100ms <= 0) //100ms计时到
    {
        bflg_100ms_reach = 1;
        gss_timer_100ms = 100;
    }
    //------------------------------------------------------
    gss_timer_1s--;
    if (gss_timer_1s <= 0)    //1s计时到
    {
        bflg_1s_reach = 1;
        gss_timer_1s = 1000;
    }
    //------------------------------------------------------
    if (bflg_running == 1)    //如果是运行模式
    {       
        //------------------------------
        if (bflg_askfor_calc_outpercent == 0)
        {
    	      check_freq_updata();   //定时进行频率更新的程序(每1ms检查1次)
    	      //----------------------------------------------
    	      bflg_askfor_calc_outpercent = 1; //置请求计算输出调制率标志
        }
    }
}
//------------------------------------------------------------------------------
void main_loop_deal(void)     //主循环处理程序
{
    timer_op();     //定时处理子程序
    //--------------------------------------------------------------------------
    Hall_updata_deal();  //霍尔信号更新处理程序     
    //--------------------------------------------------------------------------
    switch (main_loop_cnt)
    {
        case 0:     //AD转换、保护判断
            //----------------------------------------------
            if (bflg_allow_calc_Iout == 1)  
            {
	              calc_Iout_deal();  	    //计算Iout程序
                bflg_allow_calc_Iout = 0;
            }        	   
        	  //----------------------------------------------
            if (bflg_allow_calc_Udc == 1)
            {
   	            calc_Udc_deal();       //计算Udc程序
   	            bflg_allow_calc_Udc = 0;
            }       	   
        	  //----------------------------------------------
            if (bflg_allow_calc_Tm == 1)
            {
        	      calc_Tm_deal();         //计算Tm程序
        	      bflg_allow_calc_Tm = 0;
        	  }
        	  //----------------------------------------------
            if (bflg_powerup_delaytime == 0)//上电延时阶段，屏蔽保护故障检查
            {              	  
        	      prot_Udc_delaytime();//Udc电压保护延时程序
            }	  
            //----------------------------------------------
            //trip_code_deal();     //故障代码处理程序
            //----------------------------------------------
            //state_code_deal();    //状态代码处理程序
            //----------------------------------------------
            break;
             
        case 1:     //通讯数据处理、开关机判断
            //----------------------------------------------
            if (bflg_SCI0_allow_rx == 1)
            {
            	  bflg_SCI0_allow_rx = 0;
            	  SCI0_receive_int();     //SCI0通讯接收初始化程序
            }
            //----------------------------------------------
            if (bflg_SCI0_rx_ok == 1)
            {
            	  bflg_SCI0_rx_ok = 0;
            	  SCI0_rx_data_deal();    //通讯接收数据处理程序
            }
            //----------------------------------------------
            if (bflg_SCI0_allow_tx == 1)
            {
            	  bflg_SCI0_allow_tx = 0;
            	  SCI0_send_init();       //SCI0通讯发送初始化程序
            }
            //----------------------------------------------
            //sci0rx_data_deal();    //sci0接收数据处理程序
            //----------------------------------------------
            //get_host_setfreq();    //得到上位机设定频率的程序
            //----------------------------------------------
            trip_code_deal();     //故障代码处理程序
            //----------------------------------------------
            state_code_deal();    //状态代码处理程序            
            //----------------------------------------------
            fan_ctrl_deal();       //压机控制程序
            //----------------------------------------------
            freq_ctrl_deal();      //频率控制程序
            //----------------------------------------------
            SCI1rx_data_deal();     //SCI1接收数据处理程序,在主循环中调用             
            
            check_run_stop_deal();      //检查开关机处理的的程序 
            //----------------------------------------------

            //----------------------------------------------
            //----------------------------------------------
            break;
        case 2:
            //------------------------------------------------------------------
            if ((bflg_running == 1) && (bflg_askfor_calc_outpercent == 1))
            {
                bflg_askfor_calc_outpercent = 0;  //清要求计算输出调制率标志
                //--------------------------------------------------------------
                Sensored_cale_Vout();        //带位置传感器控制计算输出电压的程序
                //--------------------------------------------------------------
                calc_percent_theta();   //用于计算当前调制率和相位角步长的程序
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
void timer_op(void) //定时处理子程序，用于处理所有的定时延时程序
{
    if (bflg_1ms_reach == 1)
    {
        bflg_1ms_reach = 0;
        //---------------------------1ms定时任务--------------------------------
        check_powerup_delaytime();      //上电后的延时程序
        
        check_spry_delaytime();         //主继电器吸合和断开的延时计时程序
        
        bootstrap_charger_delaytime();  //自举充电延时程序
        
        //check_sensored_startup_deal();  //带位置传感器控制时的开机测速阶段的处理程序
        //----------------------------------------------------------------------
        SCI0_tx_delaytime();            //SCI0通讯发送延时程序
        
        SCI0_rx_delaytime();            //SCI0通讯接收延时程序
        
        SCI0_rx_end_delaytime();        //SCI0通讯接收后的延时程序
        
        SCI1rx_on_dealtime();           //延时接收计时程序
        
        SCI1rx_end_delaytimer();        //通讯接收后的延时程序
        
        SCI1tx_on_dealtime();           //延时发送计时程序
        
        //----------------------------------------------------------------------
        calc_Iout_delaytime();          //计算Iout延时程序
    }
    //--------------------------------------------------------------------------
    if (bflg_10ms_reach == 1)
    {
        bflg_10ms_reach = 0;
        
        bflg_allow_calc_Tm = 1;
        //---------------------------10ms定时任务-------------------------------
        if (bflg_powerup_delaytime == 0)     //上电延时阶段，屏蔽保护故障检查
        {
        	  Idc_zero_adjust();          //输出电流Idc零点校准程序
        }
        //----------------------------------------------------------------------
        Hall_fault_delaytime();    //霍尔信号故障延时程序
        
        Hall_error_delaytime();    //霍尔信号错延时程序
        
        Hall_direction_delaytime();     //霍尔信号方向错延时程序
        
        Hall_off_delaytime();      //霍尔信号停止延时程序
        //----------------------------------------------------------------------
        prot_ALM_delaytime();           //检测停机时ALM保护延时程序
        
        prot_motor_stall_delaytime();   //电机失速保护延时程序
        
        prot_motor_block_delaytime();   //电机堵转保护延时程序
        //----------------------------------------------------------------------
        //debug
        trip_lampblink_deal();     //故障灯闪烁程序
        //----------------------------------------------------------------------
    }
    //--------------------------------------------------------------------------
    if (bflg_100ms_reach == 1)
    {
        bflg_100ms_reach = 0;
        //-------------------------100ms定时任务--------------------------------
        if (bflg_powerup_delaytime == 0)     //上电延时阶段，屏蔽保护故障检查
        {       
            prot_Iout_delaytime();      //Iout电流保护延时程序
            
            //prot_Udc_delaytime();       //Udc电压保护延时程序
            
            prot_Tm_delaytime();        //Tm模块温度保护的条件
            
            prot_motor_overspeed_delaytime();//电机超速保护延时程序
            
            check_spry_ctrl_term();     //检查spry控制条件的程序
        }

    }
    //--------------------------------------------------------------------------
    if (bflg_1s_reach == 1)
    {
        bflg_1s_reach = 0;
        gss_timer_1min--;
        //---------------------------1s定时任务---------------------------------
        SCI0_check_delaytime();
        
        SCI0_fault_delaytime();    //SCI0通讯故障延时程序     
        //----------------------------------------------------------------------
        stop_space_delaytime();    //停机间隔延时程序
        
        SCI1comm_abend_dealtime();      //SCI1通讯异常计时程序，在1S定时程序中调用  
        SCI1rx_ctrl_abend_dealtime();   //SCI1接收控制信息异常处理程序,在1S定时程序中调用
        
        //----------------------------------------------------------------------
    }
    //-------------------------
    if (gss_timer_1min <= 0)
    {
        gss_timer_1min=60;
        //--------------------------1分钟任务-----------------------------------
        trip_lock_delaytime();     //故障锁定延时程序
    }
    //--------------------------------------------------------------------------
}
//------------------------------------------------------------------------------
void fan_ctrl_deal(void)     //压缩机控制程序
{
    if (bflg_running == 0)    //如果是停机状态
    {
        if ((bflg_powerup_delaytime == 0)&&(bflg_stop_space_delaytime == 0)
        && (bflg_trip_stop == 0)&&(bflg_trip_lock == 0)&& (SPRY_PIN_LEVEL == 1))    //如果停机延时到且无故障，且主继电器吸合
        {
            if (bflg_askfor_run == 1)   //如果请求运行，且设定频率不为0
            {
                if (bflg_bootstrap_charger == 0)  //如果未自举充电
                {
                    bflg_bootstrap_charger = 1;   //置自举充电标志
                    //----------------------------------------------------------
                    //硬件保护开启
                    if (ram_para[num_prot_ALM_cnt] == 0)    //如果参数设定计数次数为0
                    {
                        PWMOFF0B = 0x7026;//0xF026;//

                        PWMOFF0B1 &= ~0x0001;     //清PRTBST0位，释放PWM0引脚硬件保护B功能
                        PWMOFF0B |= 0x0001;
                    }
                    else
                    {
                        PWMOFF0B = 0x0000;        //计数次数不为0时，不用硬件保护
                    }
                    //----------------------------------------------------------
                    //开启IRQ03外中断，用于ALM保护
                    *(unsigned char *)&G24ICR = 0x01;  //清除IRQ03外中断请求标志
                    G24ICR |= 0x0100;        //置IRQ03外中断使能标志
                    //----------------------------------------------------------
                    //自举充电初始化
                    bflg_bootstrap_charger_delaytime = 1;
                    gss_bootstrap_charger_delaytimer = 0;
                    //----------------------------------------------------------
                    //霍尔信号相关处理

                    //----------------------------------------------------------
                    //开启霍尔信号中断
                    *(unsigned char *)&G29ICR = 0x03;  //清除外中断IRQ09、IRQ08请求标志
                    *(unsigned char *)&G30ICR = 0x01;  //清除外中断IRQ10请求标志

                    G29ICR |= 0x0300;        //置外中断IRQ09、IRQ08使能标志
                    G30ICR |= 0x0100;        //置外中断IRQ10使能标志
                    //----------------------------------------------------------
                }
                else if (bflg_bootstrap_charger_delaytime == 0)  //如果自举充电完成
                {
                    bflg_bootstrap_charger = 0;   //清自举充电标志
                    //--------------------------------------
                    bflg_fan_running = 1;   //置风机运行标志
                    bflg_running = 1;       //置运行标志
                    bflg_askfor_run = 0;

                    bflg_theta_ctrl1 = 0;
                    bflg_theta_ctrl2 = 0;
                    //----------------------------------------------------------
                    //确定起始角度
                    gus_Hall_value = HALL_PIN;    //得到霍尔信号值
                    gsc_Hall_sector = gsc_sector_table[gus_Hall_value];    //得到Hall区间

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
                    gul_theta.uword.high += 300;
                    theta.ulword = gul_theta.ulword;

                    //gus_Hall_watchdog = 0;//debug
                    //--------------------------------------
                    //得到积分初值
                    gsl_FreqPI_integral = gsl_PI_initval;
                    //----------------------------------------------------------
                    bflg_askfor_calc_outpercent = 1; //置请求计算输出调制率标志
                    //----------------------------------------------------------
                    open_pwm();             //开始PWM输出
                    //--------------------------------------	  	  	  	      
                    gus_freq_out = gus_freq_real;
                }
            }
        }
    }
    else  //如果是开机状态
    {
        if (bflg_hostComm_shutdown == 1)//上位机紧急停机命令
        {
            bflg_fan_running = 0;  //清风机运行标志

            bflg_running = 0;      //清除运行标志
            bflg_askfor_stop = 0;  //清请求停止标志

            shut_pwm();            //关闭PWM   	  	

            bflg_hostComm_shutdown = 0;
            bflg_stop_space_delaytime = 1;
            gss_stop_space_delaytimer = ram_para[num_stop_space_delaytime];		  	  	
        }
        else if (bflg_trip_stop == 1)   //如果是故障停机
        {
            bflg_fan_running = 0;  //清风机运行标志

            bflg_running = 0;      //清除运行标志
            bflg_askfor_stop = 0;  //清请求停止标志

            ///shut_pwm();            //关闭PWM(bflg_trip_stop置1函数中已经关闭PWM)

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
                    gss_stop_space_delaytimer = 2;//停机2s后重新启动	  	  	  	
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
                    gss_stop_space_delaytimer = 5;//停机2s后重新启动	  	  	  	
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
                    gss_stop_space_delaytimer = 2;//停机2s后重新启动	  	  	  	
                }		  	      	   
            }
            else
            {
                bflg_stop_space_delaytime = 1;
                gss_stop_space_delaytimer = ram_para[num_stop_space_delaytime];	  	      	
            } 
        }
        else if (bflg_askfor_stop == 1) //如果有请求停止标志
        {
            bflg_fan_running = 0;      //清风机运行标志

            if (gus_freq_real <= ram_para[num_shut_freq])   //启动频率 debug
            {
                bflg_running = 0;       //清除运行标志
                bflg_askfor_stop = 0;   //清请求停止标志

                shut_pwm();             //关闭PWM

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
void freq_ctrl_deal(void)     //频率控制程序
{
	  uint16_t lus_freq_set;
	  uint16_t lus_min_protfreq = 0;
	  
	  if (bflg_running == 1)    //如果是运行阶段
	  {
	  	  if (bflg_fan_running == 0)      //如果风机停止
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
            //两个保护频率排序并得到最小不为0的保护频率的程序
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
	  else  //如果是停机
	  {
	  	  gus_freq_set = 0;
	  }
}
//------------------------------------------------------------------------------
void check_run_stop_deal(void)     //检查开关机处理的的程序
{
    if (bflg_host_ctrl == 1)  //如果是上位机控制，则进入
    {
        //上位机要求运行且频率不为零且无紧急停机标志，则进行开机的处理
        if ((bflg_host_running == 1) && (gus_host_setfreq > 0))
        {
            run_key();        //运行键处理      
        }
        else if ((bflg_host_running == 0) || (gus_host_setfreq == 0))    //停机的处理
        {
            stop_key();       //停机键处理
        }
    }
    else       //如果是LCD面板控制，则进入
    {
        if (gus_lcd_setfreq > 0)   //开机的处理
        {
            run_key();   //运行键处理 
        }
        else
        {
            stop_key();  //停机键处理
        }    	  
    }   
}
//------------------------------------------------------------------------------
void run_key(void)       //运行键处理
{
    if (bflg_eeprom_change == 0)   //如果有eeprom变更标志，则退出
    {  
        if ((bflg_running == 0) && (bflg_askfor_run == 0)) //不是运行状态且未接收开机信号,则进入
        {  
            bflg_askfor_stop = 0;      //清除要求停机标志位
            bflg_askfor_run = 1;       //置要求运行标志
        } 
    } 
}
//------------------------------------------------------------------------------
void stop_key(void)      //停机键处理
{
    if (bflg_askfor_stop == 0)    //如果没有要求停机标志，则进入
    {         
        if (bflg_running == 1)
        {
            bflg_askfor_stop = 1; //置要求停机标志
        }
        else if (bflg_askfor_run == 1)
        {
            bflg_askfor_run = 0;  //如果不是运行状态，仅仅只有要求运行标志，则清除该标志
            //----------------------------------
            PWMSEL0 = 0x0FC0;      //PWM引脚由软件控制,全无效
            
            gus_val_a = gus_peak_pwm + 1;
            gus_val_b = gus_peak_pwm + 1;
            gus_val_c = gus_peak_pwm + 1;    //给各相计数值赋初值
            
            TCMP0A = gus_val_a;    //PWM00占空比设定  U相
            TCMP0B = gus_val_b;    //PWM01占空比设定  V相
            TCMP0C = gus_val_c;    //PWM02占空比设定  W相 
        }  
    } 
}
//------------------------------------------------------------------------------
uint16_t get_min_decfreq_deal(void)     //两个保护频率排序并得到最小不为0的保护频率的程序，在频率设定程序中调用
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
void check_powerup_delaytime(void)      //上电后的延时程序，在1ms定时程序中调用
{
    if (bflg_powerup_delaytime == 1)    //上电欠压延时计时处理
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
void check_spry_ctrl_term(void)    //检查spry控制条件的程序，在100ms定时程序中调用
{
    if ((bflg_prot_Udc_LU == 1) && (SPRY_PIN_LEVEL == 1))   //欠压且SPRY吸合状态,则进入
    {
        if (bflg_spry_off_delaytime == 0)
        {
            bflg_spry_off_delaytime = 1;     //置断开主继电器的延时标志
            bflg_spry_on_delaytime = 0;      //清吸合主继电器的延时标志
            gss_spry_delaytimer = 0;         //清主继电器延时计时器  
        }
    }
    else if ((bflg_prot_Udc_LU == 0) && (SPRY_PIN_LEVEL == 0))   //不欠压且SPRY断开状态,则进入
    {
        if (bflg_spry_on_delaytime == 0)
        {
            bflg_spry_off_delaytime = 0;     //清断开主继电器的延时标志
            bflg_spry_on_delaytime = 1;      //置吸合主继电器的延时标志
            gss_spry_delaytimer = 0;         //清主继电器延时计时器  
        }
    }
}
//------------------------------------------------------------------------------
void check_spry_delaytime(void)         //主继电器吸合和断开的延时计时程序，在1ms定时程序中调用
{
    if (bflg_spry_off_delaytime == 1)   //主继电器断开延时
    { 
        gss_spry_delaytimer++;
        if (gss_spry_delaytimer >= 300)
        {
            gss_spry_delaytimer = 0;
            bflg_spry_off_delaytime = 0;
            
            SPRY_PIN_OUTPUT_L;     //断开主继电器
        }
    }
    else if (bflg_spry_on_delaytime == 1)    //主继电器吸合延时
    {
        gss_spry_delaytimer++;
        if (gss_spry_delaytimer >= 20)
        {
            gss_spry_delaytimer = 0;
            bflg_spry_on_delaytime = 0;
            
            SPRY_PIN_OUTPUT_H;     //时间到，吸合主继电器
        }
    }
}
//------------------------------------------------------------------------------
void stop_space_delaytime(void)    //停机间隔延时程序，在s定时程序中调用
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
void trip_lampblink_deal(void)     //故障灯闪烁程序,在10ms定时任务中调用
{
    uint8_t luc_tmp;
    
    //区分海尔和通用故障灯闪烁次数
    if (guc_LED_trip_code != 0)
    {
    	  luc_tmp = guc_LED_trip_code;
    }
    else
    {
    	  luc_tmp = guc_lamp_trip_code;
    }
    
    if (bflg_trip_stop == 1)  //如果不是跳停，则转
    {
        if (gus_trip_flash_count > 0)   //如果闪烁次数已经减到零，则开始3秒间隔延时
        {
            gus_trip_flash_timer++;
            if (gus_trip_flash_timer >= 30)  //每0.30秒闪烁一次 //PEAK 原来30
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
            if (gus_trip_flash_timer >= 120) //1.5秒间隔延时 实际设定为1.5-0.3秒
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
        if (bflg_running == 1)     //运行时，等间隔闪烁
        {
             gus_trip_flash_timer++;
             if(gus_trip_flash_timer >= 10)     //每0.1秒闪烁一次
             {
                 gus_trip_flash_timer = 0;
                 NOT_TRIP_LAMP_PIN;
             }
        }
        else
        {
            SET_TRIP_LAMP_PIN;      //正常停机时情况下，故障灯（兼电源灯）常亮
            gus_trip_flash_timer = 0;
            gus_trip_flash_timer = 0;
        }
    }
}

//------------------------------------------------------------------------------
