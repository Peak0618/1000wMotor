#ifndef	_PANEL_COMM_H_
#define _PANEL_COMM_H_
//------------------------------------------------------------------------------
extern void SCI1_configure(void);      //����SCI1����ͨѶ�ӿ�

//extern void SCI1comm_reset(void);      //SCI1ͨѶ��λ�����������

extern void SCI1rx_on_dealtime(void);  //��ʱ���ռ�ʱ������1ms��ʱ�����е���

extern void SCI1_rx_int(void);         //SCI1�����ж�

//extern void SCI1_rx_err_int(void);    //SCI1���մ����ж�

extern void SCI1rx_end_delaytimer(void);    //ͨѶ���պ����ʱ���򣬼���1ms��ʱ������

extern void SCI1comm_abend_dealtime(void);  //����ͨѶ�쳣�������,��1S��ʱ�����е���

extern void SCI1rx_ctrl_abend_dealtime(void);   //����ͨѶ���տ�����Ϣ�쳣�������,��1S��ʱ�����е���

extern void SCI1rx_data_deal(void);     //SCI1�������ݴ������

//extern void SCI1_handshake_info_deal(void);   //���յ�������Ϣ�������

//extern void SCI1_wrcmd_info_deal(void);     //���յ�д������Ϣ�������

//extern void SCI1_rdcmd_info_deal(void);     //���յĶ�������Ϣ�������

//extern void SCI1_ctrl_info_deal(void);     //���յĿ���������Ϣ�������

extern void SCI1tx_on_dealtime(void);  //��ʱ���ͼ�ʱ������1ms��ʱ�����е���

extern void SCI1_tx_int(void);     //SCI1�����ж�
//------------------------------------------------------------------------------
extern uint16_t gus_lcd_setfreq;	
//-----------------------------------------------------------

//------------------------------------------------------------------------------
#endif 						//end 
