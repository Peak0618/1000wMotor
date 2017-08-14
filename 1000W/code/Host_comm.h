#ifndef	_HOST_COMM_H_
#define _HOST_COMM_H_
//------------------------------------------------------------------------------
extern void SCI0_configure(void);       //配置SCI0串行通讯接口

extern void SCI0comm_reset(void);

extern void SCI0_check_delaytime(void);

extern void SCI0_send_init(void);       //SCI0通讯发送初始化程序

extern void SCI0_receive_int(void);     //SCI0通讯接收初始化程序

extern void SCI0_rx_data_deal(void);    //通讯接收数据处理程序

extern void SCI0_tx_delaytime(void);    //SCI0通讯发送延时程序，在1ms定时程序中调用

extern void SCI0_rx_delaytime(void);    //SCI0通讯接收延时程序，在1ms定时程序中调用

extern void SCI0_rx_end_delaytime(void);//SCI0通讯接收后的延时程序，在1ms定时程序中调用

extern void SCI0_fault_delaytime(void); //SCI0通讯故障延时程序，在1s定时程序中调用

extern void SCI0_tx_int(void);          //SCI0发送中断程序

extern void SCI0_rx_int(void);          //SCI0接收中断程序
//------------------------------------------------------------------------------
extern flag_type flg_SCI0;
          #define   bflg_SCI0_tx_delaytime   flg_SCI0.bits.bit0       //SCI0通讯发送延时标志
          #define   bflg_SCI0_rx_delaytime   flg_SCI0.bits.bit1       //SCI0通讯接收延时标志
          #define   bflg_SCI0_allow_tx       flg_SCI0.bits.bit2       //SCI0通讯允许发送标志
          #define   bflg_SCI0_allow_rx       flg_SCI0.bits.bit3       //SCI0通讯允许接收标志
          #define   bflg_SCI0_tx_busy        flg_SCI0.bits.bit4       //SCI0通讯发送忙标志
          #define   bflg_SCI0_rx_busy        flg_SCI0.bits.bit5       //SCI0通讯接收忙标志
          #define   bflg_SCI0_rx_ok          flg_SCI0.bits.bit6       //SCI0通讯接收成功标志
          #define   bflg_SCI0_fault          flg_SCI0.bits.bit7       //SCI0通讯故障标志
          
          #define   bflg_SCI0_type_ok        flg_SCI0.bits.bit8       //SCI0通讯类型确定标志
          #define   bflg_SCI0_parity_type        flg_SCI0.bits.bit9       //
          
          
          
//------------------------------------------------------------------------------
extern uint16_t  gus_host_setfreq;
extern uint8_t   guc_comm_address;   //通讯地址

extern uint8_t   guc_SCI0_trip_code;
extern uint8_t   guc_LED_trip_code;
//------------------------------------------------------------------------------
#endif 						//end 
