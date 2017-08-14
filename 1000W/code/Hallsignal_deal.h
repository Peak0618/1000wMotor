#ifndef	_HALLSIGNAL_DEAL_H_
#define _HALLSIGNAL_DEAL_H_
//------------------------------------------------------------------------------
extern  void Hall_configure(void);   //霍尔信号配置程序

extern  void motor_var_init(void);     //电机相关变量程序化，在Hall初始化和通讯ID变更程序中调用

extern  void Hall_input_int(void);     //Hall信号输入中断服务程序

extern  void Hall_updata_deal(void);   //霍尔信号更新处理程序,在主循环中调用

extern  void Hall_fault_delaytime(void);     //霍尔信号故障延时程序，在10ms定时程序中调用

extern  void Hall_error_delaytime(void);     //霍尔信号错延时程序，加入10ms定时程序中

extern  void Hall_direction_delaytime(void);      //霍尔信号方向错延时程序，加入10ms定时程序中

//extern  void check_sensored_startup_deal(void);   //带位置传感器控制时的开机测速阶段的处理程序,在10ms定时程序中调用

//extern  void sensored_start_init(void);     //带位置传感器控制时的开机初始化程序

extern  void Hall_off_delaytime(void);       //霍尔信号停止延时程序，在10ms定时程序中调用

//---------------------------------------------------------------------
//霍尔信号引脚状态宏定义
#define   HALL_PIN  (((P5IN & 0x01) << 2) | (P4IN & 0x03))  //bit7~0:00000UVW
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
extern flag_type flg_Hall;
          #define   bflg_hall_int                 flg_Hall.bits.bit0
          #define   bflg_hall_update              flg_Hall.bits.bit1
          
          #define   bflg_no_direction             flg_Hall.bits.bit2
          #define   bflg_actual_direction         flg_Hall.bits.bit3
          #define   bflg_past_actual_direction    flg_Hall.bits.bit4
          
          #define   bflg_Hall_fault               flg_Hall.bits.bit5
          #define   bflg_Hall_error               flg_Hall.bits.bit6
          #define   bflg_prot_Hall_direction      flg_Hall.bits.bit7
          
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
extern uint16_t PhaseValues_CCW[6]; //逆时针旋转，正弦波驱动时的区间对应的起始相位角
extern uint16_t PhaseValues_CW[6];  //顺时针旋转，正弦波驱动时的区间对应的起始相位角
//------------------------------------------------------------------------------
extern const int8_t gsc_sector_table[];    //霍尔信号区间表

extern uint16_t gus_MotorStallRate;

extern uint16_t gus_Hall_freq;

extern uint16_t gus_Hall_speed;

extern uint16_t gus_sensor_startup_timer;   //开机测速阶段计时器

extern uint16_t gus_Hall_watchdog;   //霍尔信号看门狗

extern uint16_t gus_stall_rate;

extern uint32_t gul_DeltCapture;

extern uint16_t gus_Hall_value;         //当前霍尔信号值

extern int8_t   gsc_Hall_sector;        //霍尔信号扇区值

extern uint16_t gus_motor_overspeed;    //电机超速频率

extern uint16_t gus_avr_realfreq;

extern uint16_t gus_phase_adv_degree;

extern uint8_t guc_motor_ID;

//-------------------------------------------------------------------------------
#endif 						//end 
