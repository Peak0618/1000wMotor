#ifndef	_HOST_COMM_H_
#define _HOST_COMM_H_
//------------------------------------------------------------------------------
extern void SCI0_configure(void);       //����SCI0����ͨѶ�ӿ�

extern void SCI0comm_reset(void);

extern void SCI0_check_delaytime(void);

extern void SCI0_send_init(void);       //SCI0ͨѶ���ͳ�ʼ������

extern void SCI0_receive_int(void);     //SCI0ͨѶ���ճ�ʼ������

extern void SCI0_rx_data_deal(void);    //ͨѶ�������ݴ������

extern void SCI0_tx_delaytime(void);    //SCI0ͨѶ������ʱ������1ms��ʱ�����е���

extern void SCI0_rx_delaytime(void);    //SCI0ͨѶ������ʱ������1ms��ʱ�����е���

extern void SCI0_rx_end_delaytime(void);//SCI0ͨѶ���պ����ʱ������1ms��ʱ�����е���

extern void SCI0_fault_delaytime(void); //SCI0ͨѶ������ʱ������1s��ʱ�����е���

extern void SCI0_tx_int(void);          //SCI0�����жϳ���

extern void SCI0_rx_int(void);          //SCI0�����жϳ���
//------------------------------------------------------------------------------
extern flag_type flg_SCI0;
          #define   bflg_SCI0_tx_delaytime   flg_SCI0.bits.bit0       //SCI0ͨѶ������ʱ��־
          #define   bflg_SCI0_rx_delaytime   flg_SCI0.bits.bit1       //SCI0ͨѶ������ʱ��־
          #define   bflg_SCI0_allow_tx       flg_SCI0.bits.bit2       //SCI0ͨѶ�����ͱ�־
          #define   bflg_SCI0_allow_rx       flg_SCI0.bits.bit3       //SCI0ͨѶ������ձ�־
          #define   bflg_SCI0_tx_busy        flg_SCI0.bits.bit4       //SCI0ͨѶ����æ��־
          #define   bflg_SCI0_rx_busy        flg_SCI0.bits.bit5       //SCI0ͨѶ����æ��־
          #define   bflg_SCI0_rx_ok          flg_SCI0.bits.bit6       //SCI0ͨѶ���ճɹ���־
          #define   bflg_SCI0_fault          flg_SCI0.bits.bit7       //SCI0ͨѶ���ϱ�־
          
          #define   bflg_SCI0_type_ok        flg_SCI0.bits.bit8       //SCI0ͨѶ����ȷ����־
          #define   bflg_SCI0_parity_type        flg_SCI0.bits.bit9       //
          
          
          
//------------------------------------------------------------------------------
extern uint16_t  gus_host_setfreq;
extern uint8_t   guc_comm_address;   //ͨѶ��ַ

extern uint8_t   guc_SCI0_trip_code;
extern uint8_t   guc_LED_trip_code;
//------------------------------------------------------------------------------
#endif 						//end 
