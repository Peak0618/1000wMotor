#ifndef	_AD_CONVERT_H_
#define _AD_CONVERT_H_
//------------------------------------------------------------------------------
//函数外部声明
extern void ADC_configure(void);        //AD转换配置程序

extern void AD1_Complete_INT(void);     //AD1转换结束中断处理程序

extern void get_Udc_ad(void);           //得到计算Udc的相关AD值

extern void calc_Iout_delaytime(void);  //计算Iout延时程序，在1ms定时程序中调用

extern void calc_Iout_deal(void);       //每100ms计算一次, 计算三相peak值和Iout值及三相不平衡率,在主循环中调用

extern void calc_Iout_avr_deal(void);   //计算输出电流平均值

extern void calc_Udc_deal(void);        //计算直流电压,在主循环中调用

extern void calc_Tm_deal(void);         //计算IPM模块温度的程序，每10ms调用一次
//------------------------------------------------------------------------------
//外部变量定义
extern flag_type flg_ADC;
          #define   bflg_allow_get_Idc       flg_ADC.bits.bit0   //允许得到Idc标志
          #define   bflg_up_down_get_Idc     flg_ADC.bits.bit1   //上升沿下降沿得到Idc标志
          #define   bflg_allow_calc_Iout     flg_ADC.bits.bit2   //允许计算Iout标志
          #define   bflg_allow_calc_Tm       flg_ADC.bits.bit3   //允许计算Tm标志
          #define   bflg_allow_calc_Udc      flg_ADC.bits.bit4   //允许计算Udc标志
          
//------------------------------------------------------------------------------
extern uint16_t gus_Idc_ad;
extern uint16_t gus_Idc_ad_up;
extern uint16_t gus_Idc_ad_dw;

extern uint16_t gus_Iout_peak;

extern uint16_t gus_Iout;
extern uint16_t gus_Iout_for_disp;

extern uint16_t gus_Udc_ad;             //当前Udc采样的AD值
extern uint16_t gus_Udc_for_disp;       //用于显示的Udc电压值(用于显示 单位1V)
extern uint16_t  gus_Udc_tmp;

extern uint16_t gus_Udc_max;            //Udc电压最大值(用于保护的判断 单位1V)
extern uint16_t gus_Udc_min;            //Udc电压最小值(用于保护的判断 单位1V)

extern uint16_t gus_Tm_ad;
extern int16_t  gss_Tm;
//------------------------------------------------------------------------------
#endif 						//end 
