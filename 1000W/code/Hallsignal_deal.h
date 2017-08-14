#ifndef	_HALLSIGNAL_DEAL_H_
#define _HALLSIGNAL_DEAL_H_
//------------------------------------------------------------------------------
extern  void Hall_configure(void);   //�����ź����ó���

extern  void motor_var_init(void);     //�����ر������򻯣���Hall��ʼ����ͨѶID��������е���

extern  void Hall_input_int(void);     //Hall�ź������жϷ������

extern  void Hall_updata_deal(void);   //�����źŸ��´������,����ѭ���е���

extern  void Hall_fault_delaytime(void);     //�����źŹ�����ʱ������10ms��ʱ�����е���

extern  void Hall_error_delaytime(void);     //�����źŴ���ʱ���򣬼���10ms��ʱ������

extern  void Hall_direction_delaytime(void);      //�����źŷ������ʱ���򣬼���10ms��ʱ������

//extern  void check_sensored_startup_deal(void);   //��λ�ô���������ʱ�Ŀ������ٽ׶εĴ������,��10ms��ʱ�����е���

//extern  void sensored_start_init(void);     //��λ�ô���������ʱ�Ŀ�����ʼ������

extern  void Hall_off_delaytime(void);       //�����ź�ֹͣ��ʱ������10ms��ʱ�����е���

//---------------------------------------------------------------------
//�����ź�����״̬�궨��
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
extern uint16_t PhaseValues_CCW[6]; //��ʱ����ת�����Ҳ�����ʱ�������Ӧ����ʼ��λ��
extern uint16_t PhaseValues_CW[6];  //˳ʱ����ת�����Ҳ�����ʱ�������Ӧ����ʼ��λ��
//------------------------------------------------------------------------------
extern const int8_t gsc_sector_table[];    //�����ź������

extern uint16_t gus_MotorStallRate;

extern uint16_t gus_Hall_freq;

extern uint16_t gus_Hall_speed;

extern uint16_t gus_sensor_startup_timer;   //�������ٽ׶μ�ʱ��

extern uint16_t gus_Hall_watchdog;   //�����źſ��Ź�

extern uint16_t gus_stall_rate;

extern uint32_t gul_DeltCapture;

extern uint16_t gus_Hall_value;         //��ǰ�����ź�ֵ

extern int8_t   gsc_Hall_sector;        //�����ź�����ֵ

extern uint16_t gus_motor_overspeed;    //�������Ƶ��

extern uint16_t gus_avr_realfreq;

extern uint16_t gus_phase_adv_degree;

extern uint8_t guc_motor_ID;

//-------------------------------------------------------------------------------
#endif 						//end 
