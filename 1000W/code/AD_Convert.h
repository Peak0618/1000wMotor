#ifndef	_AD_CONVERT_H_
#define _AD_CONVERT_H_
//------------------------------------------------------------------------------
//�����ⲿ����
extern void ADC_configure(void);        //ADת�����ó���

extern void AD1_Complete_INT(void);     //AD1ת�������жϴ������

extern void get_Udc_ad(void);           //�õ�����Udc�����ADֵ

extern void calc_Iout_delaytime(void);  //����Iout��ʱ������1ms��ʱ�����е���

extern void calc_Iout_deal(void);       //ÿ100ms����һ��, ��������peakֵ��Ioutֵ�����಻ƽ����,����ѭ���е���

extern void calc_Iout_avr_deal(void);   //�����������ƽ��ֵ

extern void calc_Udc_deal(void);        //����ֱ����ѹ,����ѭ���е���

extern void calc_Tm_deal(void);         //����IPMģ���¶ȵĳ���ÿ10ms����һ��
//------------------------------------------------------------------------------
//�ⲿ��������
extern flag_type flg_ADC;
          #define   bflg_allow_get_Idc       flg_ADC.bits.bit0   //����õ�Idc��־
          #define   bflg_up_down_get_Idc     flg_ADC.bits.bit1   //�������½��صõ�Idc��־
          #define   bflg_allow_calc_Iout     flg_ADC.bits.bit2   //�������Iout��־
          #define   bflg_allow_calc_Tm       flg_ADC.bits.bit3   //�������Tm��־
          #define   bflg_allow_calc_Udc      flg_ADC.bits.bit4   //�������Udc��־
          
//------------------------------------------------------------------------------
extern uint16_t gus_Idc_ad;
extern uint16_t gus_Idc_ad_up;
extern uint16_t gus_Idc_ad_dw;

extern uint16_t gus_Iout_peak;

extern uint16_t gus_Iout;
extern uint16_t gus_Iout_for_disp;

extern uint16_t gus_Udc_ad;             //��ǰUdc������ADֵ
extern uint16_t gus_Udc_for_disp;       //������ʾ��Udc��ѹֵ(������ʾ ��λ1V)
extern uint16_t  gus_Udc_tmp;

extern uint16_t gus_Udc_max;            //Udc��ѹ���ֵ(���ڱ������ж� ��λ1V)
extern uint16_t gus_Udc_min;            //Udc��ѹ��Сֵ(���ڱ������ж� ��λ1V)

extern uint16_t gus_Tm_ad;
extern int16_t  gss_Tm;
//------------------------------------------------------------------------------
#endif 						//end 
