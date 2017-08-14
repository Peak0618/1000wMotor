#ifndef	_PROTECT_DEAL_H_
#define _PROTECT_DEAL_H_
//------------------------------------------------------------------------------
extern void Iout_prot_variable_init(void);   //��Iout�����йصı�����ʼ�������ڳ�ʼ�������е���

extern void Idc_zero_adjust(void);           //�������Idc���У׼����,ÿ100ms�ж�һ�Σ���100ms��ʱ������

extern void prot_ALM_delaytime(void);        //���ͣ��ʱALM������ʱ���򣬼���10ms��ʱ������

extern void ALM_prot_int(void);              //ALM�����жϣ���IRQ03�ж��е���

extern void prot_ALM_deal(void);             //ALM������ʱ�����ڲ��η����ж��е���

extern void prot_Iout_POC_deal(void);        //Iout˲ʱ�������������AD1�жϷ�������е���

extern void prot_Iout_delaytime(void);       //Iout����������ʱ������100ms��ʱ�����е���

extern void Udc_prot_variable_init(void);    //ĸ�ߵ�ѹ�����йر�����ʼ������,�ڳ�ʼ�������е���

extern void prot_Udc_POU_deal(void);         //Udc˲ʱ�������������AD1�жϷ����е���

extern void prot_Udc_delaytime(void);        //Udc��ѹ������ʱ������100ms��ʱ�����е���

extern void prot_Tm_delaytime(void);         //Tmģ���¶ȱ�������������100ms��ʱ�����е���

extern void prot_motor_stall_delaytime(void);     //���ʧ�ٱ�����ʱ������10ms��ʱ�����е���

extern void prot_motor_block_delaytime(void);     //�����ת������ʱ������10ms��ʱ�����е���

extern void prot_motor_overspeed_delaytime(void); //������ٱ�����ʱ������100ms��ʱ�����е���

extern void trip_code_deal(void);            //���ϴ��봦���������ѭ���е���

extern void state_code_deal(void);           //״̬���봦���������ѭ���е���

extern void trip_stop_deal(uint8_t luc_trip_code);     //���ϴ������

extern void trip_lock_delaytime(void);       //����������ʱ������1min��ʱ�����е���
//------------------------------------------------------------------------------
extern flag_type flg_fault;
          
          #define   bflg_Idc_fault                flg_fault.bits.bit0
          #define   bflg_Tm_fault                 flg_fault.bits.bit1
          
          #define   bflg_trip_stop                flg_fault.bits.bit2      //���ϱ���ͣ����־
          #define   bflg_trip_clear               flg_fault.bits.bit3      //���ϱ��������־
          #define   bflg_trip_lock                flg_fault.bits.bit4      //���ϱ���������־
          
          #define   bflg_trip_lock_delaytime      flg_fault.bits.bit5      //����������ʱ��־
          
          #define   bflg_prot_motor_block         flg_fault.bits.bit6      //�����ת������־
          #define   bflg_prot_motor_stall         flg_fault.bits.bit7      //�����ת������־
          #define   bflg_prot_motor_overspeed     flg_fault.bits.bit8      //������ٱ�����־
          
          #define   bflg_motor_ID_error           flg_fault.bits.bit9      //���ID�Ŵ��־
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

extern uint16_t gus_trip_release_cnt;    //������ͷż�����
extern int16_t   gss_stop_space_delaytimer;
extern uint8_t  guc_motor_block_cnt;
extern uint8_t  guc_Iout_fault_cnt;
extern uint8_t  guc_ALM_fault_cnt;
//------------------------------------------------------------------------------
//״̬���붨��
#define   Normal_STATE_CODE        0    //����״̬

#define   OU_STALL_STATE_CODE      1    //��ѹʧ��

#define   OC_STALL_STATE_CODE      2    //�������ʧ��
#define   OC_DECFREQ_STATE_CODE    3    //���������Ƶ

#define   Tm_OH_DECFREQ_STATE_CODE 6    //ɢ�������Ƚ�Ƶ
#define   Tm_OH_STALL_STATE_CODE   8    //ɢ��������ʧ��
//------------------------------------------------------------------------------
//LCD�����ϴ���Ķ���
#define   DCT_FAULT_CODE           30   //ĸ�ߵ���������   lamp flash code 24
#define   Tm_FAULT_CODE            35   //IPMģ���¶ȼ�����lamp flash code 26
#define   COMM_FAULT_CODE          40   //ͨѶ����           lamp flash code 1
#define   Hall_FAULT_CODE          73   //�����źŹ���       lamp flash code 14
#define   Hall_direct_FAULT_CODE   75   //�����źŷ����     lamp flash code 16
#define   EEPROM_ERR_CODE          82   //EEPROM����         lamp flash code 28
#define   MOTOR_ID_ERR_CODE        83   //���ID�Ŵ����     lamp flash code 29

//LCD��屣������Ķ���
#define   MOTOR_STALL_PROT_CODE      41 //���ʧ�ٱ���       lamp flash code 11
#define   MOTOR_BLOCK_PROT_CODE      42 //�����ת����       lamp flash code 12
#define   MOTOR_OVERSPEEED_PROT_CODE 43 //������ٱ���       lamp flash code 13

#define   Udc_LU_PROT_CODE         52   //Ƿѹ����           lamp flash code 5
#define   Udc_OU_PROT_CODE         53   //��ѹ����           lamp flash code 6
#define   Iout_OL_PROT_CODE        56   //������ر���       lamp flash code 2
#define   Iout_OC_PROT_CODE        57   //�����������       lamp flash code 3
#define   ALM_PROT_CODE            58   //�����·����       lamp flash code 4
#define   Tm_OH_PROT_CODE          69   //IPMģ����ȱ���    lamp flash code 10

//#define   CLEAR_TRIP_CODE          80   //lamp flash code 0
#define   EEPROM_CHANGE_CODE       81   //lamp flash code 0

//------------------------------------------------------------------------------
//���ϵ���˸�����Ķ���
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
