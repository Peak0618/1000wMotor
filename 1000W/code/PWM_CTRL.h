#ifndef	_PWM_CTRL_H_
#define _PWM_CTRL_H_
//------------------------------------------------------------------------------
extern  void PWM_configure(void);  //PWMģ�����ó���

extern  void freq_rate_init(void); //��ʼ��1msƵ���������ĳ���

extern  void open_pwm(void);       //����PWM�ĳ����ڿ��������е���

extern  void shut_pwm(void);       //�ر�PWM�ĳ�����ͣ������򱣻�ͣ�������е���

extern  void bootstrap_charger_delaytime(void);   //�Ծٳ����ʱ������1ms��ʱ�����е���

extern  void check_freq_updata(void);   //��ʱ����Ƶ�ʸ��µĳ���(ÿ1ms���1��)

extern  void Sensored_cale_Vout(void);  //��λ�ô��������Ƽ��������ѹ�ĳ���

extern  void calc_percent_theta(void);  //���ڼ��㵱ǰ�����ʺ���λ�ǲ����ĳ���

extern  int16_t get_sin_value(uint16_t now_sin_index); //��sin��ĺ���

extern  void PWM_Reload_INT(void);      //���η����жϷ������,��Vector.c�Ķ�Ӧ�жϷ�������е���

//------------------------------------------------------------------------------
//��־����
//------------------------------------------------------------------------------
extern flag_type flg_PWM;
          
          #define   bflg_askfor_calc_outpercent        flg_PWM.bits.bit0   //Ҫ�������������ʵı�־
          #define   bflg_startup_ready                 flg_PWM.bits.bit2   //����׼���׶α�־
          #define   bflg_sensor_startup                flg_PWM.bits.bit4   //�������ٽ׶α�־
          
          #define   bflg_startup_time                  flg_PWM.bits.bit6   //�����׶α�־
          
          #define   bflg_ask_for_speedup               flg_PWM.bits.bit7   //���ٽ׶α�־
          #define   bflg_ask_for_speeddown             flg_PWM.bits.bit8   //���ٽ׶α�־
          
          #define   bflg_askfor_taiwave_ctrl           flg_PWM.bits.bit9   //����̨�β����Ʊ�־
          #define   bflg_askfor_guwave_ctrl            flg_PWM.bits.bit10  //������β����Ʊ�־
          #define   bflg_taiwave_ctrl                  flg_PWM.bits.bit11  //̨�β����Ʊ�־
          
          #define   bflg_bootstrap_charger             flg_PWM.bits.bit12  //�Ծٳ���־
          #define   bflg_pwm_allow_updata              flg_PWM.bits.bit13  //pwm������±�־
          #define   bflg_bootstrap_charger_delaytime   flg_PWM.bits.bit14  //�Ծٳ����ʱ��־
          
//------------------------------------------------------------------------------
extern uint16_t gus_max_comp_val;  //��ǰPWM����Ƚ�ֵ�����ֵ
extern uint16_t gus_mid_comp_val;  //��ǰPWM����Ƚ�ֵ���м�ֵ
extern uint16_t gus_min_comp_val;  //��ǰPWM����Ƚ�ֵ����Сֵ

extern uint16_t gus_val_abc_order; //���ڼ�¼abc���������С�ļĴ���

extern uint32_t gul_percent_pwm_unit;   //��λ��ѹ�����ʼĴ���

extern uint16_t gus_now_theta;     //��ǰ�������ڶ�Ӧ����λ��

extern lword_type gul_theta;

extern lword_type theta;           //A����Ʋ�����λ�ǼĴ���

extern uint16_t gus_freq_set;      //�趨Ƶ��
extern uint16_t gus_freq_out;      //���Ƶ��(��HEVF����)
extern uint16_t gus_freq_real;     //��ʵ���Ƶ�� (��HEVF����)
extern uint16_t gus_speed_real;    //��ʵ���ת�� (ʵ��ת��)    
extern uint16_t gus_MAX_setfreq;
extern int16_t  gss_bootstrap_charger_delaytimer; //�Ծٳ�������

extern uint16_t gus_Uout_VF;       //VF�����趨�������ѹ (0.1V)

extern uint16_t gus_Uout_real;     //��ǰ��ʵ�����ѹ     (0.1V)

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
