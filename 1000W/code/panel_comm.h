#ifndef	_PANEL_COMM_H_
#define _PANEL_COMM_H_
//------------------------------------------------------------------------------
extern void SCI1_configure(void);      //配置SCI1串行通讯接口

//extern void SCI1comm_reset(void);      //SCI1通讯复位操作处理程序

extern void SCI1rx_on_dealtime(void);  //延时接收计时程序，在1ms定时程序中调用

extern void SCI1_rx_int(void);         //SCI1接收中断

//extern void SCI1_rx_err_int(void);    //SCI1接收错误中断

extern void SCI1rx_end_delaytimer(void);    //通讯接收后的延时程序，加入1ms定时程序中

extern void SCI1comm_abend_dealtime(void);  //串口通讯异常处理程序,在1S定时程序中调用

extern void SCI1rx_ctrl_abend_dealtime(void);   //串口通讯接收控制信息异常处理程序,在1S定时程序中调用

extern void SCI1rx_data_deal(void);     //SCI1接收数据处理程序

//extern void SCI1_handshake_info_deal(void);   //接收的握手信息处理程序

//extern void SCI1_wrcmd_info_deal(void);     //接收的写命令信息处理程序

//extern void SCI1_rdcmd_info_deal(void);     //接收的读命令信息处理程序

//extern void SCI1_ctrl_info_deal(void);     //接收的控制命令信息处理程序

extern void SCI1tx_on_dealtime(void);  //延时发送计时程序，在1ms定时程序中调用

extern void SCI1_tx_int(void);     //SCI1发送中断
//------------------------------------------------------------------------------
extern uint16_t gus_lcd_setfreq;	
//-----------------------------------------------------------

//------------------------------------------------------------------------------
#endif 						//end 
