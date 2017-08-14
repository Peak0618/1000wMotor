#ifndef	_PROTECT_DEAL_H_
#define _PROTECT_DEAL_H_
//------------------------------------------------------------------------------
extern void Iout_prot_variable_init(void);   //与Iout保护有关的变量初始化程序，在初始化程序中调用

extern void Idc_zero_adjust(void);           //输出电流Idc零点校准程序,每100ms判断一次，在100ms定时程序中

extern void prot_ALM_delaytime(void);        //检测停机时ALM保护延时程序，加入10ms定时程序中

extern void ALM_prot_int(void);              //ALM保护中断，在IRQ03中断中调用

extern void prot_ALM_deal(void);             //ALM保护延时程序，在波形发生中断中调用

extern void prot_Iout_POC_deal(void);        //Iout瞬时保护处理程序，在AD1中断服务程序中调用

extern void prot_Iout_delaytime(void);       //Iout电流保护延时程序，在100ms定时程序中调用

extern void Udc_prot_variable_init(void);    //母线电压保护有关变量初始化程序,在初始化程序中调用

extern void prot_Udc_POU_deal(void);         //Udc瞬时保护处理程序，在AD1中断服务中调用

extern void prot_Udc_delaytime(void);        //Udc电压保护延时程序，在100ms定时程序中调用

extern void prot_Tm_delaytime(void);         //Tm模块温度保护的条件，在100ms定时程序中调用

extern void prot_motor_stall_delaytime(void);     //电机失速保护延时程序，在10ms定时程序中调用

extern void prot_motor_block_delaytime(void);     //电机堵转保护延时程序，在10ms定时程序中调用

extern void prot_motor_overspeed_delaytime(void); //电机超速保护延时程序，在100ms定时程序中调用

extern void trip_code_deal(void);            //故障代码处理程序，在主循环中调用

extern void state_code_deal(void);           //状态代码处理程序，在主循环中调用

extern void trip_stop_deal(uint8_t luc_trip_code);     //故障处理程序

extern void trip_lock_delaytime(void);       //故障锁定延时程序，在1min定时程序中调用
//------------------------------------------------------------------------------
extern flag_type flg_fault;
          
          #define   bflg_Idc_fault                flg_fault.bits.bit0
          #define   bflg_Tm_fault                 flg_fault.bits.bit1
          
          #define   bflg_trip_stop                flg_fault.bits.bit2      //故障保护停机标志
          #define   bflg_trip_clear               flg_fault.bits.bit3      //故障保护清除标志
          #define   bflg_trip_lock                flg_fault.bits.bit4      //故障保护锁定标志
          
          #define   bflg_trip_lock_delaytime      flg_fault.bits.bit5      //故障锁定延时标志
          
          #define   bflg_prot_motor_block         flg_fault.bits.bit6      //电机堵转保护标志
          #define   bflg_prot_motor_stall         flg_fault.bits.bit7      //电机堵转保护标志
          #define   bflg_prot_motor_overspeed     flg_fault.bits.bit8      //电机超速保护标志
          
          #define   bflg_motor_ID_error           flg_fault.bits.bit9      //电机ID号错标志
//----------------------------------------------------------
extern flag_type flg_prot;
          
          #define   bflg_prot_ALM                 flg_prot.bits.bit0
          #define   bflg_prot_ALM_delaycnt        flg_prot.bits.bit1
          
          #define   bflg_prot_Iout_POC            flg_prot.bits.bit2
          #define   bflg_prot_Iout_OC             flg_prot.bits.bit3
          
          #define   bflg_prot_Iout_OL_integ       flg_prot.bits.bit4
          #define   bflg_prot_Iout_OL             flg_prot.bits.bit5
          #define   bflg_prot_Iout_decfreq        flg_prot.bits.bit6
          #define   bflg_prot_Iout_stall          flg_prot.bits.bit7
          #define   bflg_prot_Iout_NC             flg_prot.bits.bit8
          
          
          #define   bflg_prot_Udc_POU             flg_prot.bits.bit9
          #define   bflg_prot_Udc_OU              flg_prot.bits.bit10
          #define   bflg_prot_Udc_LU              flg_prot.bits.bit11
          #define   bflg_prot_Udc_stall           flg_prot.bits.bit12
          
          #define   bflg_prot_Tm_OH               flg_prot.bits.bit13
          #define   bflg_prot_Tm_decfreq          flg_prot.bits.bit14
          #define   bflg_prot_Tm_stall            flg_prot.bits.bit15
//----------------------------------------------------------
extern uint16_t gus_Idc_zero_ad;

extern uint16_t gus_prot_Iout_POC_ad;
extern int16_t  gss_prot_Iout_POC_delaytimer;

extern uint16_t gus_prot_Iout_freq;

extern uint16_t gus_prot_Iout_stall_amp;
//--------------------------------------
extern uint16_t gus_prot_Udc_stall_amp;
//--------------------------------------
extern uint16_t gus_prot_Tm_freq;
extern uint16_t gus_prot_Tm_stall_amp;
//--------------------------------------
extern uint8_t guc_trip_code;
extern uint8_t guc_lamp_trip_code;
extern uint8_t guc_state_code;

extern uint16_t gus_trip_release_cnt;    //清故障释放计数器
extern int16_t   gss_stop_space_delaytimer;
extern uint8_t  guc_motor_block_cnt;
extern uint8_t  guc_Iout_fault_cnt;
extern uint8_t  guc_ALM_fault_cnt;
//------------------------------------------------------------------------------
//状态代码定义
#define   Normal_STATE_CODE        0    //正常状态

#define   OU_STALL_STATE_CODE      1    //过压失速

#define   OC_STALL_STATE_CODE      2    //输出过流失速
#define   OC_DECFREQ_STATE_CODE    3    //输出过流降频

#define   Tm_OH_DECFREQ_STATE_CODE 6    //散热器过热降频
#define   Tm_OH_STALL_STATE_CODE   8    //散热器过热失速
//------------------------------------------------------------------------------
//LCD面板故障代码的定义
#define   DCT_FAULT_CODE           30   //母线电流检测故障   lamp flash code 24
#define   Tm_FAULT_CODE            35   //IPM模块温度检测故障lamp flash code 26
#define   COMM_FAULT_CODE          40   //通讯故障           lamp flash code 1
#define   Hall_FAULT_CODE          73   //霍尔信号故障       lamp flash code 14
#define   Hall_direct_FAULT_CODE   75   //霍尔信号方向错     lamp flash code 16
#define   EEPROM_ERR_CODE          82   //EEPROM故障         lamp flash code 28
#define   MOTOR_ID_ERR_CODE        83   //电机ID号错故障     lamp flash code 29

//LCD面板保护代码的定义
#define   MOTOR_STALL_PROT_CODE      41 //电机失速保护       lamp flash code 11
#define   MOTOR_BLOCK_PROT_CODE      42 //电机堵转保护       lamp flash code 12
#define   MOTOR_OVERSPEEED_PROT_CODE 43 //电机过速保护       lamp flash code 13

#define   Udc_LU_PROT_CODE         52   //欠压保护           lamp flash code 5
#define   Udc_OU_PROT_CODE         53   //过压保护           lamp flash code 6
#define   Iout_OL_PROT_CODE        56   //输出过载保护       lamp flash code 2
#define   Iout_OC_PROT_CODE        57   //输出过流保护       lamp flash code 3
#define   ALM_PROT_CODE            58   //输出短路保护       lamp flash code 4
#define   Tm_OH_PROT_CODE          69   //IPM模块过热保护    lamp flash code 10

//#define   CLEAR_TRIP_CODE          80   //lamp flash code 0
#define   EEPROM_CHANGE_CODE       81   //lamp flash code 0

//------------------------------------------------------------------------------
//故障灯闪烁次数的定义
#define   LAMP_DCT_FAULT_CODE      24
#define   LAMP_Tm_FAULT_CODE       26
#define   LAMP_COMM_FAULT_CODE     1
#define   LAMP_Hall_FAULT_CODE     14
#define   LAMP_Hall_direct_FAULT_CODE   16
#define   LAMP_EEPROM_ERR_CODE     28
#define   LAMP_MOTOR_ID_ERR_CODE   29
//------------------------------------------
#define   LAMP_MOTOR_STALL_PROT_CODE    11
#define   LAMP_MOTOR_BLOCK_PROT_CODE    12
#define   LAMP_MOTOR_OVERSPEEED_PROT_CODE    13
#define   LAMP_Udc_LU_PROT_CODE    5
#define   LAMP_Udc_OU_PROT_CODE    6
#define   LAMP_Iout_OL_PROT_CODE   2
#define   LAMP_Iout_OC_PROT_CODE   3
#define   LAMP_ALM_PROT_CODE       4
#define   LAMP_Tm_OH_PROT_CODE     7

//#define   LAMP_CLEAR_TRIP_CODE     0
#define   LAMP_EEPROM_CHANGE_CODE  0
//------------------------------------------------------------------------------
#endif
