#ifndef	_PWM_CTRL_H_
#define _PWM_CTRL_H_
//------------------------------------------------------------------------------
extern  void PWM_configure(void);  //PWM模块配置程序

extern  void freq_rate_init(void); //初始化1ms频率增减量的程序

extern  void open_pwm(void);       //开启PWM的程序，在开机程序中调用

extern  void shut_pwm(void);       //关闭PWM的程序，在停机程序或保护停机程序中调用

extern  void bootstrap_charger_delaytime(void);   //自举充电延时程序，在1ms定时程序中调用

extern  void check_freq_updata(void);   //定时进行频率更新的程序(每1ms检查1次)

extern  void Sensored_cale_Vout(void);  //带位置传感器控制计算输出电压的程序

extern  void calc_percent_theta(void);  //用于计算当前调制率和相位角步长的程序

extern  int16_t get_sin_value(uint16_t now_sin_index); //查sin表的函数

extern  void PWM_Reload_INT(void);      //波形发生中断服务程序,在Vector.c的对应中断服务程序中调用

//------------------------------------------------------------------------------
//标志定义
//------------------------------------------------------------------------------
extern flag_type flg_PWM;
          
          #define   bflg_askfor_calc_outpercent        flg_PWM.bits.bit0   //要求计算输出调制率的标志
          #define   bflg_startup_ready                 flg_PWM.bits.bit2   //启动准备阶段标志
          #define   bflg_sensor_startup                flg_PWM.bits.bit4   //启动测速阶段标志
          
          #define   bflg_startup_time                  flg_PWM.bits.bit6   //启动阶段标志
          
          #define   bflg_ask_for_speedup               flg_PWM.bits.bit7   //加速阶段标志
          #define   bflg_ask_for_speeddown             flg_PWM.bits.bit8   //减速阶段标志
          
          #define   bflg_askfor_taiwave_ctrl           flg_PWM.bits.bit9   //请求台形波调制标志
          #define   bflg_askfor_guwave_ctrl            flg_PWM.bits.bit10  //请求股形波调制标志
          #define   bflg_taiwave_ctrl                  flg_PWM.bits.bit11  //台形波调制标志
          
          #define   bflg_bootstrap_charger             flg_PWM.bits.bit12  //自举充电标志
          #define   bflg_pwm_allow_updata              flg_PWM.bits.bit13  //pwm允许更新标志
          #define   bflg_bootstrap_charger_delaytime   flg_PWM.bits.bit14  //自举充电延时标志
          
//------------------------------------------------------------------------------
extern uint16_t gus_max_comp_val;  //当前PWM输出比较值的最大值
extern uint16_t gus_mid_comp_val;  //当前PWM输出比较值的中间值
extern uint16_t gus_min_comp_val;  //当前PWM输出比较值的最小值

extern uint16_t gus_val_abc_order; //用于记录abc三相排序大小的寄存器

extern uint32_t gul_percent_pwm_unit;   //单位电压调制率寄存器

extern uint16_t gus_now_theta;     //当前控制周期对应的相位角

extern lword_type gul_theta;

extern lword_type theta;           //A相调制波的相位角寄存器

extern uint16_t gus_freq_set;      //设定频率
extern uint16_t gus_freq_out;      //输出频率(无HEVF补偿)
extern uint16_t gus_freq_real;     //真实输出频率 (有HEVF补偿)
extern uint16_t gus_speed_real;    //真实输出转速 (实测转速)    
extern uint16_t gus_MAX_setfreq;
extern int16_t  gss_bootstrap_charger_delaytimer; //自举充电计数器

extern uint16_t gus_Uout_VF;       //VF曲线设定的输出电压 (0.1V)

extern uint16_t gus_Uout_real;     //当前真实输出电压     (0.1V)

extern uint16_t gus_val_a;
extern uint16_t gus_val_b;
extern uint16_t gus_val_c;

extern uint16_t gus_peak_pwm;

extern uint16_t gus_Uout_for_disp;

extern uint16_t gus_hevf_percent;

extern int32_t gsl_FreqPI_integral;

extern int32_t gsl_PI_initval;
//------------------------------------------------------------------------------
#endif 						//end 
