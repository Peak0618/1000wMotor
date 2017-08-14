#ifndef __COMM_VARIABLE_H__
#define __COMM_VARIABLE_H__
//------------------------------------------------------------------------------
#include "user_typedefine.h"      //���Ͷ����ļ�
//------------------------------------------------------------------------------
//�������ļ��ж���ĺ���
extern void disable_irq(void);     

extern void enable_irq(void);

extern void set_imask(int);
//------------------------------------------------------------------------------
//����ع���α����
#define watchdog_disable() {WPRTKEY = 0xD9; WDCTR &= 0x7F; WPRTKEY = 0xFF;}     //WDCNE=0;

//����Ź���α����
#define watchdog_enable() {WPRTKEY = 0xD9; WDCTR |= 0x40; WDCTR &= 0x60; WDCTR |= 0x01; WDCTR |= 0x80; WPRTKEY = 0xFF;}     

//�����߹���α����
#define clear_watchdog() {WPRTKEY = 0xD9; WDCTR |= 0x40; WPRTKEY = 0xFF;}       //WDRST=1;
//------------------------------------------------------------------------------
#define reset_dealprog() {RSTCTR = 0x00; RSTCTR = 0x01;}
//------------------------------------------------------------------------------
//���Ų�������
//#define   SPRY_PIN_LEVEL      ((P5IN & 0x02) >> 1)     //P51����SPRY
//#define   SPRY_PIN_OUTPUT_L   (P5OUT &= ~0x02)
//#define   SPRY_PIN_OUTPUT_H   (P5OUT |= 0x02)
#define   SPRY_PIN_LEVEL      ((P0IN & 0x01))     //P00����SPRY
#define   SPRY_PIN_OUTPUT_L   (P0OUT &= ~0x01)
#define   SPRY_PIN_OUTPUT_H   (P0OUT |= 0x01)

#define   ALM_PIN_LEVEL       ((P0IN & 0x08) >> 3)     //P03����ALM

#define   NOT_TRIP_LAMP_PIN   (P5OUT ^= 0x0002)
#define   CLR_TRIP_LAMP_PIN   (P5OUT &= 0xFFFD)
#define   SET_TRIP_LAMP_PIN   (P5OUT |= 0x0002)
//------------------------------------------------------------------------------
#define   COMM_ADDRESS        (P6IN & 0x03)//((~P6IN & 0x02) | (~P6IN & 0x01))   
//------------------------------------------------------------------------------
extern flag_type flg_ctrl;
          #define   bflg_askfor_run          flg_ctrl.bits.bit0  //�������б�־
          #define   bflg_askfor_stop         flg_ctrl.bits.bit1  //����ͣ����־
          #define   bflg_running             flg_ctrl.bits.bit2  //���б�־
          #define   bflg_host_running        flg_ctrl.bits.bit3  //��λ�����б�־
          #define   bflg_free_stop           flg_ctrl.bits.bit4  //����ͣ����־
          #define   bflg_host_ctrl           flg_ctrl.bits.bit5  //��λ�����Ʊ�־
          #define   bflg_current_direction   flg_ctrl.bits.bit6  //�����ת�����־
          #define   bflg_fan_running         flg_ctrl.bits.bit7  //������б�־
//------------------------------------------------------------------------------
extern flag_type flg_delaytime;
          #define   bflg_1ms_reach           flg_delaytime.bits.bit0  //1ms��ʱ����־
          #define   bflg_10ms_reach          flg_delaytime.bits.bit1  //10ms��ʱ����־
          #define   bflg_100ms_reach         flg_delaytime.bits.bit2  //100ms��ʱ����־
          #define   bflg_1s_reach            flg_delaytime.bits.bit3  //1s��ʱ����־
          
          #define   bflg_stop_space_delaytime     flg_delaytime.bits.bit4  //ͣ�������ʱ��־
          #define   bflg_powerup_delaytime        flg_delaytime.bits.bit5  //�ϵ���ʱ��־
          
          #define   bflg_spry_on_delaytime   flg_delaytime.bits.bit6  //SPRY��ʱOFF��־
          #define   bflg_spry_off_delaytime  flg_delaytime.bits.bit7  //SPRY��ʱOFF��־

extern flag_type flg_theta;
          #define   bflg_theta_ctrl1          flg_theta.bits.bit0  
          #define   bflg_theta_ctrl2          flg_theta.bits.bit1  
          #define   bflg_hostComm_shutdown    flg_theta.bits.bit2

	      #define  bflg_eeprom_error          flg_theta.bits.bit4
	      //#define  bflg_eeprom_clrtirp        flg_theta.bits.bit5
	      #define  bflg_eeprom_change         flg_theta.bits.bit6          


extern  flag_type flg_SCI1;
     //#define  bflg_outfan2_hall_delaytime       flg_SCI1.bits.bit0    //����2������ʱ��־
     //#define  bflg_outfan2_calc_speed           flg_SCI1.bits.bit1    //����2�����ź��˲���־
     //#define  bflg_outfan2_hall_fault           flg_SCI1.bits.bit2    //����2�����źŹ��ϱ�־
     //#define  bflg_outfan2_Ospeed_prot          flg_SCI1.bits.bit3    //����2���ٱ�����־
     //#define  bflg_outfan2_first_hall           flg_SCI1.bits.bit4    //����2�׸������źű�־
     
     #define  bflg_SCI1_allow_rx                flg_SCI1.bits.bit9
     #define  bflg_SCI1_rx_busy                 flg_SCI1.bits.bit10
     #define  bflg_SCI1_rx_ok                   flg_SCI1.bits.bit11
     #define  bflg_SCI1_allow_tx                flg_SCI1.bits.bit12
     #define  bflg_SCI1_tx_busy                 flg_SCI1.bits.bit13
     #define  bflg_SCI1comm_err_prot            flg_SCI1.bits.bit14          
//------------------------------------------------------------------------------
//����汾��
          #define   SOFT_VERSION        100// 
               
//------------------------------------------------------------------------------
#endif /* __VARIABLE_DEFINE_H__ */
