#ifndef	_PROTECT_DEAL_C_
#define _PROTECT_DEAL_C_
//------------------------------------------------------------------------------
#include "srMN103SFJ9D.h"
#include "user_typedefine.h"
#include "comm_variable.h"

#include "EEPROM_PARA.h"
#include "EEPROM_RW.h"
#include "protect_deal.h"
#include "Hallsignal_deal.h"
#include "PWM_CTRL.h"
#include "AD_Convert.h"
#include "Host_comm.h"
//------------------------------------------------------------------------------
//��������
void Iout_prot_variable_init(void);//��Iout�����йصı�����ʼ�������ڳ�ʼ�������е���

void Idc_zero_adjust(void);        //�������Idc���У׼������10ms��ʱ������

void prot_ALM_delaytime(void);     //���ͣ��ʱALM������ʱ���򣬼���10ms��ʱ������

void ALM_prot_int(void);           //ALM�����жϣ���IRQ3�ж��е���

void prot_ALM_deal(void);          //ALM������������ڲ��η����ж��е���

void prot_Iout_POC_deal(void);     //Iout˲ʱ�������������AD1�жϷ�������е���

void prot_Iout_delaytime(void);    //Iout����������ʱ������100ms��ʱ�����е���

void Udc_prot_variable_init(void); //ĸ�ߵ�ѹ�����йر�����ʼ������,�ڳ�ʼ�������е���

void prot_Udc_POU_deal(void);      //Udc˲ʱ�������������AD1�жϷ����е���

void prot_Udc_delaytime(void);     //Udc��ѹ������ʱ������100ms��ʱ�����е���

void prot_Tm_delaytime(void);      //Tmģ���¶ȱ�������������100ms��ʱ�����е���

void prot_motor_stall_delaytime(void);  //���ʧ�ٱ�����ʱ������10ms��ʱ�����е���

void prot_motor_block_delaytime(void);  //�����ת������ʱ������10ms��ʱ�����е���

void prot_motor_overspeed_delaytime(void);   //������ٱ�����ʱ������100ms��ʱ�����е���

void trip_code_deal(void);         //���ϴ��봦���������ѭ���е���

void state_code_deal(void);        //״̬���봦���������ѭ���е���

void trip_stop_deal(uint8_t luc_trip_code);       //���ϴ������

void trip_lock_delaytime(void);    //����������ʱ������1min��ʱ�����е���
//------------------------------------------------------------------------------
//��־����
flag_type flg_fault;
flag_type flg_prot;
//----------------------------------------------------------
//��������
//----------------------------------------------------------
//Idc������ر���
uint16_t  gus_Idc_zero_ad_low;      //Idc������޼Ĵ���
uint16_t  gus_Idc_zero_ad_up;       //Idc������޼Ĵ���

uint16_t  gus_Idc_zero_cnt;         //Idc����ۼӴ����Ĵ���
uint32_t  gul_Idc_zero_ad_sum;      //Idc����ۼӺͼĴ���
uint16_t  gus_Idc_zero_ad;          //Idc���Ĵ���

uint16_t  gus_Idc_fault_cnt;        //Idc CT���ϳ���������
//----------------------------------------------------------
//ALM������ر���
uint16_t  gus_prot_ALM_delaytimer;
uint16_t  gus_quit_ALM_delaytimer;

uint16_t  gus_prot_ALM_cnt;
//----------------------------------------------------------
//�������������ر���
uint16_t  gus_prot_Iout_POC_ad;
int16_t   gss_prot_Iout_POC_delaycnt;

uint16_t  gus_prot_Iout_OC;
int16_t   gss_prot_Iout_OC_delaytimer;

uint16_t  gus_prot_Iout_OL;
int32_t   gsl_prot_Iout_OL_integ_buf;
int32_t   gsl_prot_Iout_OL_integ_max;

uint16_t  gus_prot_Iout_decfreq_enter;
uint16_t  gus_prot_Iout_decfreq_quit;
int16_t   gss_prot_Iout_decfreq_enter_delaytimer;
int16_t   gss_prot_Iout_decfreq_quit_delaytimer;  
uint16_t  gus_prot_Iout_freq;    

uint16_t  gus_prot_Iout_stall_enter;
uint16_t  gus_prot_Iout_stall_quit;
int16_t   gss_prot_Iout_stall_delaytimer;
uint16_t  gus_prot_Iout_stall_amp;

//----------------------------------------------------------
//ĸ�ߵ�ѹ������ر���
uint16_t  gus_prot_Udc_POU_ad;
uint16_t  gss_prot_Udc_POU_delaycnt;

uint16_t  gus_prot_Udc_stall_amp;
//----------------------------------------------------------
//ģ���¶ȱ�����ر���
uint16_t  gus_prot_Tm_OH_delaytimer;
uint16_t  gus_prot_Tm_decfreq_enter_delaytimer;
uint16_t  gus_prot_Tm_decfreq_quit_delaytimer;
uint16_t  gus_prot_Tm_freq;
uint16_t  gus_prot_Tm_stall_delaytimer;
uint16_t  gus_prot_Tm_stall_amp;
//----------------------------------------------------------
int16_t   gss_motor_stall_delaytimer;   //���ʧ����ʱ��ʱ��
int16_t   gss_motor_block_delaytimer;   //�����ת��ʱ��ʱ��
int16_t   gss_motor_overspeed_delaytimer;    //���������ʱ��ʱ��
//----------------------------------------------------------
int16_t   gss_trip_lock_delaytimer;   //����������ʱ��ʱ��
int16_t   gss_stop_space_delaytimer;    //ͣ�����ʱ��

uint8_t   guc_state_code;     //״̬����
uint8_t   guc_lamp_trip_code; //���ϵ���˸����
uint16_t gus_trip_release_cnt;     //������ͷż�����
//----------------------------------------------------------
uint8_t  guc_trip_code;      //���ϴ���
uint8_t  guc_trip_stop_cnt;       //����ͣ������
uint8_t  guc_motor_block_cnt;
uint8_t  guc_Iout_fault_cnt;
uint8_t  guc_ALM_fault_cnt;
//------------------------------------------------------------------------------
//Iout�������жϺʹ������
//------------------------------------------------------------------------------
void Iout_prot_variable_init(void)      //��Iout�����йصı�����ʼ�������ڳ�ʼ�������е���
{
    lword_type ltmp1;
    //------------------------------------------------------
    //��Idc������Ĳο�ֵ����ֵ
    gus_Idc_zero_ad = ram_para[num_Iout_CT_zero_AD];
    gus_Idc_zero_ad_low = ram_para[num_Iout_CT_zero_AD] - ram_para[num_Iout_CT_zero_margin];   //������ο�ֵ������
    gus_Idc_zero_ad_up  = ram_para[num_Iout_CT_zero_AD] + ram_para[num_Iout_CT_zero_margin];   //������ο�ֵ������  	
    //------------------------------------------------------
    gul_Idc_zero_ad_sum = 0;  //Idc����ۼӺͼĴ���
    gus_Idc_zero_cnt = 0;     //Idc����ۼӴ����Ĵ���
    gus_Idc_fault_cnt = 0;    //Idc CT���ϳ���������
    //------------------------------------------------------
    //������������ֵת��Ϊ��Ӧ��ADֵ
    ltmp1.ulword = ram_para[num_prot_Iout_POC];
    ltmp1.ulword -= ram_para[num_Iout_ref];
    ltmp1.ulword <<= ram_para[num_Iout_gain_amp];
    ltmp1.ulword /= ram_para[num_Iout_gain];
    ltmp1.ulword += ram_para[num_Iout_ref_AD];
    
    gus_prot_Iout_POC_ad = ltmp1.ulword;
    //------------------------------------------------------
    //�������������Ӧ�ĵ���ֵ
    ltmp1.ulword = ram_para[num_Iout_rate];
    ltmp1.ulword *= ram_para[num_prot_Iout_OC];
    ltmp1.ulword /= 1000;        
    
    gus_prot_Iout_OC = ltmp1.ulword;    
    //------------------------------------------------------
    //������ر�����Ӧ�ĵ���ֵ
    ltmp1.lword = ram_para[num_Iout_rate];
    ltmp1.lword *= ram_para[num_prot_Iout_OL];
    ltmp1.lword /= 1000;
    ltmp1.lword -= ram_para[num_Iout_rate];
    ltmp1.lword *= ram_para[num_prot_Iout_OL_delaytime];        
    
    gsl_prot_Iout_OL_integ_max = ltmp1.lword;
    //----------------------------------
    ltmp1.ulword = ram_para[num_Iout_rate];
    ltmp1.ulword *= ram_para[num_prot_Iout_OL_enter];
    ltmp1.ulword /= 1000;        
    
    gus_prot_Iout_OL = ltmp1.ulword;
    //------------------------------------------------------
    //���������Ƶ������Ӧ�ĵ���ֵ
    ltmp1.ulword = ram_para[num_Iout_rate];
    ltmp1.ulword *= ram_para[num_prot_Iout_decfreq_enter];
    ltmp1.ulword /= 1000;        
    
    gus_prot_Iout_decfreq_enter = ltmp1.ulword;
    //----------------------------------
    ltmp1.ulword = ram_para[num_Iout_rate];
    ltmp1.ulword *= ram_para[num_prot_Iout_decfreq_quit];
    ltmp1.ulword /= 1000;        
    
    gus_prot_Iout_decfreq_quit = ltmp1.ulword;        
    //------------------------------------------------------
    //�������ʧ�ٶ�Ӧ�ĵ���ֵ    
    ltmp1.ulword = ram_para[num_Iout_rate];
    ltmp1.ulword *= ram_para[num_prot_Iout_stall_enter];
    ltmp1.ulword /= 1000;        
    
    gus_prot_Iout_stall_enter = ltmp1.ulword;
    //----------------------------------
    ltmp1.ulword = ram_para[num_Iout_rate];
    ltmp1.ulword *= ram_para[num_prot_Iout_stall_quit];
    ltmp1.ulword /= 1000;        
    
    gus_prot_Iout_stall_quit = ltmp1.ulword; 
}
//------------------------------------------------------------------------------
void Idc_zero_adjust(void)    //�������Idc���У׼������10ms��ʱ������
{
    uint16_t lus_Idc_ad;
    
    if ((bflg_running == 0) && (gus_freq_out == 0) && (gus_freq_real == 0))     //ֻ����ͣ��״̬���Ž��������
    {
        lus_Idc_ad = (gus_Idc_ad_up + gus_Idc_ad_dw) >> 1;
        if ((lus_Idc_ad > gus_Idc_zero_ad_low) && (gus_Idc_ad < gus_Idc_zero_ad_up))
        {
            bflg_Idc_fault = 0;
            gus_Idc_fault_cnt = 0;
            
            gul_Idc_zero_ad_sum += gus_Idc_ad;
            gus_Idc_zero_cnt++;
            if (gus_Idc_zero_cnt >= 64)      //640ms ����һ�����
            {
                gus_Idc_zero_ad = (gul_Idc_zero_ad_sum >> 6);
                gus_Idc_zero_cnt = 0;
                gul_Idc_zero_ad_sum = 0;
            }
        }
        else
        {
            gus_Idc_fault_cnt++;
            if (gus_Idc_fault_cnt >= 100) 
            {
                gus_Idc_fault_cnt = 0;
                
                if (bflg_Idc_fault == 0)
                {
                    bflg_Idc_fault = 1;
                    trip_stop_deal(DCT_FAULT_CODE);
                }
            }
        }
    }
}
//------------------------------------------------------------------------------
void prot_ALM_delaytime(void)      //���ͣ��ʱALM������ʱ���򣬼���10ms��ʱ������
{
	  if (bflg_running == 0)
	  {
	  	  if (ALM_PIN_LEVEL == 0)
	  	  {
	  	  	  gus_prot_ALM_delaytimer++;
	  	  	  if (gus_prot_ALM_delaytimer >= 100)   //1s
	  	  	  {
	  	  	  	  gus_prot_ALM_delaytimer = 0;
	  	  	  	  
	  	  	  	  if (bflg_prot_ALM == 0)
	  	  	  	  {
	  	  	  	  	  bflg_prot_ALM = 1;  //��ALM������־
	  	  	  	  	  trip_stop_deal(ALM_PROT_CODE);
	  	  	  	  }
	  	  	  }
	  	  }
	  	  else
	  	  {
	  	  	  gus_prot_ALM_delaytimer = 0;
	  	  	  
	  	  	  gus_quit_ALM_delaytimer++;
	  	  	  if (gus_quit_ALM_delaytimer >= 100)   //1s
	  	  	  {
	  	  	  	  gus_quit_ALM_delaytimer = 0;
	  	  	  	  bflg_prot_ALM = 0;
	  	  	  	  
	  	  	  	  PWMOFF0B1 &=~0x0001;    //��PRTBST0λ���ͷ�PWM0����Ӳ������B����
	  	  	  }
	  	  }
	  }
	  else
	  {
	  	  gus_prot_ALM_delaytimer = 0;
	  }
}
//------------------------------------------------------------------------------
void ALM_prot_int(void)       //ALM�жϣ���IRQ03���ж��е���
{
	  if (ram_para[num_prot_ALM_cnt] == 0)     //��������趨��������Ϊ0����ֱ��
	  {
	  	  bflg_prot_ALM_delaycnt = 0;
	  	  gus_prot_ALM_cnt = 0;
	  	  
	  	  if (bflg_prot_ALM == 0)
	  	  {
            //----------------------------------------------
            PWMSEL0 = 0x0FC0;      //PWM�������������,ȫ��Ч
            
            G24ICR &= 0xFFEF;      //���IRQ03���ж������־
            //----------------------------------------------
            bflg_prot_ALM = 1;     //��ALM������־λ
            trip_stop_deal(ALM_PROT_CODE);
	  	  }
	  }
	  else
	  {
	  	  if (bflg_prot_ALM_delaycnt == 0)
	  	  {
	  	  	  bflg_prot_ALM_delaycnt = 1;     //��ALM��ʱ������־
	 	        gus_prot_ALM_cnt = 0;
	  	  }
	  }
}
//------------------------------------------------------------------------------
void prot_ALM_deal(void)      //ALM������������ڲ��η����ж��е���
{
    if (bflg_prot_ALM_delaycnt == 1)
    {
        if (ALM_PIN_LEVEL == 0)
      	{
      	 	  gus_prot_ALM_cnt++;
      	}
      	else
      	{
      	 	  bflg_prot_ALM_delaycnt = 0;
      	 	  gus_prot_ALM_cnt = 0;
      	}
        //--------------------------------------------------
      	if (gus_prot_ALM_cnt >= ram_para[num_prot_ALM_cnt])
      	{
      	 	  gus_prot_ALM_cnt = 0;
      	 	  bflg_prot_ALM_delaycnt = 0;
            
            if (bflg_prot_ALM == 0)
            {
            	  //------------------------------------------
                PWMSEL0 = 0x0FC0;       //PWM�������������,ȫ��Ч
                
                G24ICR &= 0xFFEF;       //���IRQ03���ж������־
                //------------------------------------------
                bflg_prot_ALM = 1;      //��Iout SC������־λ
                trip_stop_deal(ALM_PROT_CODE);
            }
      	}
    }
}
//------------------------------------------------------------------------------
void prot_Iout_POC_deal(void) //Iout˲ʱ�������������AD1�жϷ�������е���
{
	  int16_t lss_Iout_ad;
	  
	  if (bflg_running == 1)
	  {
	  	  lss_Iout_ad = gus_Idc_ad - gus_Idc_zero_ad;
	  	  if (lss_Iout_ad < 0)
	  	  {
	  	  	  lss_Iout_ad = 0;
	  	  }
	  	  //--------------------------------------------------
	  	  if (lss_Iout_ad > gus_prot_Iout_POC_ad)
        {
            gss_prot_Iout_POC_delaycnt++;
            if (gss_prot_Iout_POC_delaycnt >= ram_para[num_prot_Iout_POC_delaycnt])
            {
        	      gss_prot_Iout_POC_delaycnt = 0;
        	      
        	      if (bflg_prot_Iout_POC == 0)
                {
                    PWMSEL0 = 0x0FC0;   //PWM�������������,ȫ��Ч
                    bflg_prot_Iout_POC = 1;   //��Iout˲ʱ������־λ
                    trip_stop_deal(Iout_OC_PROT_CODE);
                }
            }
        }
        else
        {
            gss_prot_Iout_POC_delaycnt = 0;
            //bflg_prot_Iout_POC = 0;//debug
        }
	  }
	  else
	  {
	  	  gss_prot_Iout_POC_delaycnt = 0;
	  	  bflg_prot_Iout_POC = 0;
	  }
}
//------------------------------------------------------------------------------
void prot_Iout_delaytime(void)     //Iout����������ʱ������100ms��ʱ�����е���
{
	  int16_t lss_tmp;
	  
	  if (bflg_running == 1)
	  {
        //������������
        if (gus_Iout >= gus_prot_Iout_OC)
        {
      	    gss_prot_Iout_OC_delaytimer++;
      	    if (gss_prot_Iout_OC_delaytimer >= ram_para[num_prot_Iout_OC_delaytime])
      	    {
      	  	    gss_prot_Iout_OC_delaytimer = 0;
      	  	    
      	  	    if (bflg_prot_Iout_OC == 0)
      	  	    {
      	  	    	  bflg_prot_Iout_OC = 1;
      	  	    	  trip_stop_deal(Iout_OC_PROT_CODE);
      	  	    }
      	    }
        }
        else
        {
            gss_prot_Iout_OC_delaytimer = 0;
            ///bflg_prot_Iout_OC = 0;
            //------------------------------------------------------------------
            //���ر�������
            if (gus_Iout >= gus_prot_Iout_OL)
            {
          	    if (bflg_prot_Iout_OL_integ == 0)
          	    {
          	 	      bflg_prot_Iout_OL_integ = 1;    //�ù��ػ��ֱ�־
          	    }
            }
            else    //�������ֵС�ڿ�ʼ���ַ�ֵ�������
            {
          	    if ((bflg_prot_Iout_OL_integ == 1) && (gsl_prot_Iout_OL_integ_buf <= 0))  //�й��ػ��ֱ�־�һ���ֵΪ�㣬�����
          	    {
          	  	    bflg_prot_Iout_OL_integ = 0;  //����ػ��ֱ�־
          	  	    gsl_prot_Iout_OL_integ_buf = 0;
          	    }
            }
            //----------------------------------------------
	          if (bflg_prot_Iout_OL_integ == 1)     //����ǹ��ػ��ֱ�־�������
	          {
                gsl_prot_Iout_OL_integ_buf += (gus_Iout - ram_para[num_Iout_rate]);
                
                if (gsl_prot_Iout_OL_integ_buf >= gsl_prot_Iout_OL_integ_max)
                {
        	          gsl_prot_Iout_OL_integ_buf = 0;
        	          bflg_prot_Iout_OL_integ = 0;
        	          
        	          if (bflg_prot_Iout_OL == 0)
        	          {
        	          	  bflg_prot_Iout_OL = 1;    //������������ر���
        	          	  trip_stop_deal(Iout_OL_PROT_CODE);
        	          }
                }
                else if(gsl_prot_Iout_OL_integ_buf < 0)
                {
        	          gsl_prot_Iout_OL_integ_buf = 0;
                }
	          }          
            //------------------------------------------------------------------
            //������Ƶ����
            if (gus_Iout >= gus_prot_Iout_decfreq_enter)
            {
          	    gss_prot_Iout_decfreq_quit_delaytimer = 0;
          	    
          	    gss_prot_Iout_decfreq_enter_delaytimer++;
          	    if (gss_prot_Iout_decfreq_enter_delaytimer >= ram_para[num_prot_Iout_decfreq_enter_delaytime])
          	    {
          	  	    gss_prot_Iout_decfreq_enter_delaytimer = 0;
          	  	    bflg_prot_Iout_decfreq = 1;
          	  	    
				            lss_tmp = gus_freq_real - ram_para[num_prot_Iout_decfreq_deltfreq];
                    
                    if (lss_tmp < ram_para[num_min_freq])
                    {
                    	  lss_tmp = ram_para[num_min_freq];
                    }
                    
                    gus_prot_Iout_freq = lss_tmp;
          	    }
            }
            else if (gus_Iout >= gus_prot_Iout_decfreq_quit)
            {
          	    gss_prot_Iout_decfreq_enter_delaytimer = 0;
          	    gss_prot_Iout_decfreq_quit_delaytimer = 0;
          	    
          	    if (bflg_prot_Iout_decfreq == 1)  //����н�Ƶ��־
          	    {
          	    	  gus_prot_Iout_freq = gus_freq_real;
          	    }
            }
            else 
            {
          	    gss_prot_Iout_decfreq_enter_delaytimer = 0;
          	    
          	    gss_prot_Iout_decfreq_quit_delaytimer++;
          	    if (gss_prot_Iout_decfreq_quit_delaytimer >= ram_para[num_prot_Iout_decfreq_quit_delaytime])
          	    {
          	    	  gss_prot_Iout_decfreq_quit_delaytimer = 0;
          	    	  
          	    	  bflg_prot_Iout_decfreq = 0;
          	    	  gus_prot_Iout_freq = 0;
          	    }
            }
            //------------------------------------------------------------------
            //����ʧ���ж�
            if (bflg_prot_Iout_stall == 0)
            {
          	    if (gus_Iout >= gus_prot_Iout_stall_enter)
          	    {
          	        gss_prot_Iout_stall_delaytimer++;
          	        if (gss_prot_Iout_stall_delaytimer >= ram_para[num_prot_Iout_stall_enter_delaytime])
          	        {
          	      	    gss_prot_Iout_stall_delaytimer = 0;
          	      	    
          	      	    bflg_prot_Iout_stall = 1;
          	      	    gus_prot_Iout_stall_amp = ram_para[num_prot_Iout_stall_amp];
          	        }          	  	  
          	    }
          	    else
          	    {
          	    	  gss_prot_Iout_stall_delaytimer = 0;
          	    }
            }
            else
            {
          	    if (gus_Iout < gus_prot_Iout_stall_quit)
          	    {
          	  	    gss_prot_Iout_stall_delaytimer++;
          	  	    if (gss_prot_Iout_stall_delaytimer >= ram_para[num_prot_Iout_stall_quit_delaytime])
          	  	    {
          	  	 	      gss_prot_Iout_stall_delaytimer = 0;
          	  	 	      
          	  	 	      bflg_prot_Iout_stall = 0;
          	  	 	      gus_prot_Iout_stall_amp = 0;
          	  	    }
          	    }
          	    else
          	    {
          	    	  gss_prot_Iout_stall_delaytimer = 0;
          	    }
            }
            //------------------------------------------------------------------                 
        }
	  }
	  else
	  {
	 	    gss_prot_Iout_OC_delaytimer = 0;
        bflg_prot_Iout_OC = 0;
	 	    
	 	    bflg_prot_Iout_OL_integ = 0;
	 	    gsl_prot_Iout_OL_integ_buf = 0;
	 	    bflg_prot_Iout_OL = 0;
	 	    
	 	    gss_prot_Iout_decfreq_enter_delaytimer = 0;
	 	    gss_prot_Iout_decfreq_quit_delaytimer = 0;
	 	    bflg_prot_Iout_decfreq = 0;
	 	    gus_prot_Iout_freq = 0;
	 	    
	 	    gss_prot_Iout_stall_delaytimer = 0;
	 	    bflg_prot_Iout_stall = 0;
	 	    gus_prot_Iout_stall_amp = 0;
	  }
}
//------------------------------------------------------------------------------
//ĸ�ߵ�ѹUdc�������жϺʹ������
//------------------------------------------------------------------------------
void Udc_prot_variable_init(void)       //ĸ�ߵ�ѹ�����йر�����ʼ�������ڳ�ʼ�������е���
{
    lword_type ltmp1;
    //--------------------------------------------
    //��Udc������ֵת��Ϊ��Ӧ��ADֵ
    ltmp1.ulword = ram_para[num_prot_Udc_POU];
    ltmp1.ulword -= ram_para[num_Udc_ref];
    ltmp1.ulword <<= ram_para[num_Udc_gain_amp];
    ltmp1.ulword /= ram_para[num_Udc_gain];
    ltmp1.ulword += ram_para[num_Udc_ref_AD];
    
    gus_prot_Udc_POU_ad = ltmp1.ulword;
    //--------------------------------------------
}
//------------------------------------------------------------------------------
void prot_Udc_POU_deal(void)       //Udc˲ʱ�������������AD1�жϷ����е���
{
    if (gus_Udc_ad >= gus_prot_Udc_POU_ad)
    {
   	    gss_prot_Udc_POU_delaycnt++;
   	    if (gss_prot_Udc_POU_delaycnt >= ram_para[num_prot_Udc_POU_delaycnt])
   	    {
   	    	  gss_prot_Udc_POU_delaycnt = 0;
   	    	  
   	    	  if (bflg_prot_Udc_POU == 0)
   	    	  {
   	    	  	  PWMSEL0 = 0x0FC0;   //PWM�������������,ȫ��Ч
   	    	  	  
   	    	  	  bflg_prot_Udc_POU = 1;
   	    	  	  trip_stop_deal(Udc_OU_PROT_CODE);
   	    	  }
   	    }
    }
    else
    {
	      gss_prot_Udc_POU_delaycnt = 0;
	      //bflg_prot_Udc_POU = 0;//debug
	      
	      if (bflg_running == 0) //����ͣ����־�������������ϱ�־
    	  {
			  	  if (gus_Udc_max < ram_para[num_prot_Udc_OU_quit])   //376V
			  	  {
			  	  	  bflg_prot_Udc_POU = 0;
			  	  }    	  
        }	
    }
}
//------------------------------------------------------------------------------
void prot_Udc_delaytime(void)      //Udc��ѹ������ʱ������100ms��ʱ�����е���
{
	  //------------------------------------------------------
	  //��ѹ�����ж�
	  if (bflg_prot_Udc_OU == 0)     //�����ǰ�޹�ѹ����
	  {
	  	  if (gus_Udc_max > ram_para[num_prot_Udc_OU_enter])  //400V
	  	  {
	  	  	  bflg_prot_Udc_OU = 1;
	  	  	  trip_stop_deal(Udc_OU_PROT_CODE);
	  	  }
	  }
	  else  //����й�ѹ����
	  {
    	  if (bflg_running == 0) //����ͣ����־�������������ϱ�־
    	  {
			  	  if (gus_Udc_max < ram_para[num_prot_Udc_OU_quit])   //376V
			  	  {
			  	  	  bflg_prot_Udc_OU = 0;
			  	  }    	           
        }		  	  
	  }
	  //------------------------------------------------------
	  //Ƿѹ�����ж�
	  if (bflg_prot_Udc_LU == 0)     //�����Ƿѹ����
	  {
	  	  //if (gus_Udc_min < ram_para[num_prot_Udc_LU_enter])
	      if (gus_Udc_for_disp < ram_para[num_prot_Udc_LU_enter])  //194V
	      {
	      	  bflg_prot_Udc_LU = 1;
	  	  	  trip_stop_deal(Udc_LU_PROT_CODE);
	      }
	  }
	  else
	  {
	  	  //if (gus_Udc_min > ram_para[num_prot_Udc_LU_quit])
	  	  
    	  if (bflg_running == 0) //����ͣ����־�������������ϱ�־
    	  {
			      if (gus_Udc_for_disp > ram_para[num_prot_Udc_LU_quit])   //218V
			      {
			      	  bflg_prot_Udc_LU = 0;
			      }    	          
        }		  	  
	  }
	  //------------------------------------------------------
	  //��ѹʧ�ٱ����ж�
	  if (bflg_running == 1)
	  {
	  	  if (gus_Udc_max > ram_para[num_prot_Udc_stall_enter])    //383V
	  	  {
	  	  	  bflg_prot_Udc_stall = 1;
	  	  	  gus_prot_Udc_stall_amp = ram_para[num_prot_Udc_stall_amp];
	  	  }
	  	  else if (gus_Udc_max < ram_para[num_prot_Udc_stall_quit])     //373V
	  	  {
	  	  	  bflg_prot_Udc_stall = 0;
	  	  	  gus_prot_Udc_stall_amp = 0;
	  	  }
	  }
	  else
	  {
	  	  bflg_prot_Udc_stall = 0;
	  	  gus_prot_Udc_stall_amp = 0;
	  }
}
//-------------------------------------------------------------------------------
//ģ���¶ȱ����������
//-------------------------------------------------------------------------------
void prot_Tm_delaytime(void)       //Tmģ���¶ȱ�������������100ms��ʱ�����е���
{
	  int16_t lss_tmp;
	  
	  if (ram_para[num_Tm_enbale] == 1)   //���Tmģ���¶ȱ������������
	  {
        if (gss_Tm >= ram_para[num_prot_Tm_OH])
        {
      	    gus_prot_Tm_OH_delaytimer++;
      	    if (gus_prot_Tm_OH_delaytimer >= ram_para[num_prot_Tm_OH_delaytime])
      	    {
      	  	    gus_prot_Tm_OH_delaytimer = 0;
      	  	    
      	  	    if (bflg_prot_Tm_OH == 0)
      	  	    {
      	  	    	  bflg_prot_Tm_OH = 1;
      	  	    	  trip_stop_deal(Tm_OH_PROT_CODE);
      	  	    }
      	    }             	
        }
        else
        {
      	  	gus_prot_Tm_OH_delaytimer = 0;
      	  	//bflg_prot_Tm_OH = 0;
      	  	//----------------------------------------------
            if (bflg_running == 1)
            {
                //------------------------------------------
                //Tmģ���¶ȹ��Ƚ�Ƶ�ж�
                if (gss_Tm >= ram_para[num_prot_Tm_decfreq_enter])
                {
          	        gus_prot_Tm_decfreq_quit_delaytimer = 0;
          	        
          	        gus_prot_Tm_decfreq_enter_delaytimer++;
          	        if (gus_prot_Tm_decfreq_enter_delaytimer >= ram_para[num_prot_Tm_decfreq_delaytime])
          	        {
          	  	        gus_prot_Tm_decfreq_enter_delaytimer = 0;
          	  	        
          	  	        bflg_prot_Tm_decfreq = 1;
          	  	        
          	  	        lss_tmp = gus_freq_real - ram_para[num_prot_Tm_decfreq_deltfreq];
          	  	        if (lss_tmp < ram_para[num_min_freq])
                        {
                    	      lss_tmp = ram_para[num_min_freq];
                        }
                        
                        gus_prot_Tm_freq = lss_tmp;
          	        }
                }
                else if (gss_Tm >= ram_para[num_prot_Tm_decfreq_quit])
                {
          	        gus_prot_Tm_decfreq_quit_delaytimer = 0;
          	        gus_prot_Tm_decfreq_enter_delaytimer = 0;
          	        
          	        if (bflg_prot_Tm_decfreq == 1)
          	        {
          	        	  gus_prot_Tm_freq = gus_freq_real;
          	        }
                }
                else 
                {
          	        gus_prot_Tm_decfreq_enter_delaytimer = 0;
          	        
          	        gus_prot_Tm_decfreq_quit_delaytimer++;
          	        if (gus_prot_Tm_decfreq_quit_delaytimer >= ram_para[num_prot_Tm_decfreq_quit_delaytime])
          	        {
          	        	  gus_prot_Tm_decfreq_quit_delaytimer = 0;
          	        	  
          	        	  bflg_prot_Tm_decfreq = 0;
          	        	  gus_prot_Tm_freq = 0;
          	        }
                }
                //------------------------------------------
                //Tmģ���¶ȹ���ʧ���ж�
                if (bflg_prot_Tm_stall == 0)
                {
          	        if (gss_Tm >= ram_para[num_prot_Tm_stall_enter])
          	        {
          	            gus_prot_Tm_stall_delaytimer++;
          	            if (gus_prot_Tm_stall_delaytimer >= ram_para[num_prot_Tm_stall_enter_delaytime])
          	            {
          	      	        gus_prot_Tm_stall_delaytimer = 0;
          	      	        
          	      	        bflg_prot_Tm_stall = 1;
          	      	        gus_prot_Tm_stall_amp = ram_para[num_prot_Tm_stall_amp]; 
          	            }
          	        }
          	        else
          	        {
          	        	  gus_prot_Tm_stall_delaytimer = 0;
          	        }
                }
                else
                {
          	        if (gss_Tm < ram_para[num_prot_Tm_stall_quit])
          	        {
          	  	        gus_prot_Tm_stall_delaytimer++;
          	  	        if (gus_prot_Tm_stall_delaytimer >= ram_para[num_prot_Tm_stall_quit_delaytime])
          	  	        {
          	  	 	          gus_prot_Tm_stall_delaytimer = 0;
          	  	 	          
          	  	 	          bflg_prot_Tm_stall = 0;
          	  	 	          gus_prot_Tm_stall_amp = 0; 
          	  	        }
          	        }
          	        else
          	        {
          	        	  gus_prot_Tm_stall_delaytimer = 0;
          	        }
                }                      
            }
            else
            {
            	  bflg_prot_Tm_OH = 0;
            	  
            	  gus_prot_Tm_decfreq_enter_delaytimer = 0;
            	  gus_prot_Tm_decfreq_quit_delaytimer = 0;
            	  bflg_prot_Tm_decfreq = 0;
          	    gus_prot_Tm_freq = 0;
            	  
            	  gus_prot_Tm_stall_delaytimer = 0;
            	  bflg_prot_Tm_stall = 0;
            	  gus_prot_Tm_stall_amp = 0;
            }
        }	   	   
	  }
} 
//-------------------------------------------------------------------------------
void prot_motor_stall_delaytime(void)   //���ʧ�ٱ�����ʱ������10ms��ʱ�����е���
{
	  if (bflg_running == 1)    //����������
	  {
	  	  if (gus_stall_rate >= ram_para[num_motor_stall])
	  	  {
	  	  	  gss_motor_stall_delaytimer++;
	  	  	  if (gss_motor_stall_delaytimer >= ram_para[num_motor_stall_delaytime])
	  	  	  {
	  	  	  	  gss_motor_stall_delaytimer = 0;
	  	  	  	  
	  	  	  	  if (bflg_prot_motor_stall == 0)
	  	  	  	  {
	  	  	  	  	  bflg_prot_motor_stall = 1;
	  	  	  	  	  trip_stop_deal(MOTOR_STALL_PROT_CODE);
	  	  	  	  }
	  	  	  }
	  	  }
	  	  else
	  	  {
	  	  	  gss_motor_stall_delaytimer = 0;
	  	  	  ///bflg_prot_motor_stall = 0;
	  	  }
	  }
	  else
	  {
	  	  gss_motor_stall_delaytimer = 0;
	  	  bflg_prot_motor_stall = 0;
	  	  
	  	  gus_stall_rate = 0;
	  }
}
//-------------------------------------------------------------------------------
void prot_motor_block_delaytime(void)   //�����ת������ʱ������10ms��ʱ�����е���
{
	  //((bflg_actual_direction == bflg_current_direction) 
	  if  ((gus_freq_out >= 200) && (gus_freq_real <= 200))//ram_para[num_begin_freq]))
	  {
	  	  gss_motor_block_delaytimer++;
	  	  if (gss_motor_block_delaytimer >= 400)    //5s
	  	  {
	  	  	  gss_motor_block_delaytimer = 0;
	  	  	  
	  	  	  if (bflg_prot_motor_block == 0)
	  	  	  {
	  	  	  	  bflg_prot_motor_block = 1;
	  	  	  	  trip_stop_deal(MOTOR_BLOCK_PROT_CODE);
	  	  	  }
	  	  }
	  }
	  else
	  {
	  	  gss_motor_block_delaytimer = 0;
	  	  
    	  if (bflg_running == 0) //����ͣ����־�������������ϱ�־
    	  {
    	      bflg_prot_motor_block = 0;   
        }		  	    
	  }
}
//-------------------------------------------------------------------------------
void prot_motor_overspeed_delaytime(void)    //������ٱ�����ʱ������100ms��ʱ�����е���
{
	  if (gus_speed_real >= gus_motor_overspeed)    //���ʵ��ת�ٳ���
	  {
	  	  gss_motor_overspeed_delaytimer++;
	  	  if (gss_motor_overspeed_delaytimer >= 20)      //2s
	  	  {
	  	  	  gss_motor_overspeed_delaytimer = 0;
	  	  	  
	  	  	  if (bflg_prot_motor_overspeed == 0)
	  	  	  {
	  	  	  	  bflg_prot_motor_overspeed = 1;
	  	  	  	  trip_stop_deal(MOTOR_OVERSPEEED_PROT_CODE);
	  	  	  }
	  	  }
	  }
	  else
	  {
	  	  gss_motor_overspeed_delaytimer = 0;

    	  if (bflg_running == 0) //����ͣ����־�������������ϱ�־
    	  {
    	      bflg_prot_motor_overspeed = 0;  
        }	  	  
	  }
}
//-------------------------------------------------------------------------------
void trip_code_deal(void)     //���ϴ��봦���������ѭ���е���
{
    if(bflg_eeprom_change == 1)
    {
         if(bflg_eeprom_error==1)
         {
            guc_trip_code = EEPROM_ERR_CODE;
            guc_lamp_trip_code = LAMP_EEPROM_ERR_CODE;
         }
         /*else if(bflg_eeprom_clrtirp==1)
         {
            guc_trip_code = CLEAR_TRIP_CODE;     //�����ͣԭ�����ʾ����
            guc_lamp_trip_code = LAMP_CLEAR_TRIP_CODE;
         }*/
         else
         {
            guc_trip_code = EEPROM_CHANGE_CODE;
            guc_lamp_trip_code = LAMP_EEPROM_CHANGE_CODE;
         }     	      	
    }	  
	  else if (bflg_Idc_fault == 1)
    {
    	  guc_trip_code = DCT_FAULT_CODE;
        guc_lamp_trip_code = LAMP_DCT_FAULT_CODE;
    }
    else if (bflg_Tm_fault == 1)
    {
    	  guc_trip_code = Tm_FAULT_CODE;
        guc_lamp_trip_code = LAMP_Tm_FAULT_CODE;
    }
    else if (bflg_SCI0_fault == 1)
    {
    	  guc_trip_code = COMM_FAULT_CODE;
        guc_lamp_trip_code = LAMP_COMM_FAULT_CODE;
    }
    else if (bflg_prot_motor_stall == 1)
    {
    	  guc_trip_code = MOTOR_STALL_PROT_CODE;
        guc_lamp_trip_code = LAMP_MOTOR_STALL_PROT_CODE;
    }
    else if (bflg_prot_motor_block == 1)
    {
    	  guc_trip_code = MOTOR_BLOCK_PROT_CODE;
        guc_lamp_trip_code = LAMP_MOTOR_BLOCK_PROT_CODE;
    }
    else if (bflg_prot_motor_overspeed == 1)
    {
    	  guc_trip_code = MOTOR_OVERSPEEED_PROT_CODE;
        guc_lamp_trip_code = LAMP_MOTOR_OVERSPEEED_PROT_CODE;
    }
    else if (bflg_prot_Udc_LU == 1)
    {
    	  guc_trip_code = Udc_LU_PROT_CODE;
        guc_lamp_trip_code = LAMP_Udc_LU_PROT_CODE;
    }
    else if ((bflg_prot_Udc_POU == 1) || (bflg_prot_Udc_OU == 1))
    {
    	  guc_trip_code = Udc_OU_PROT_CODE;
        guc_lamp_trip_code = LAMP_Udc_OU_PROT_CODE;
    }
    else if (bflg_prot_Iout_OL == 1)
    {
    	  guc_trip_code = Iout_OL_PROT_CODE;
        guc_lamp_trip_code = LAMP_Iout_OL_PROT_CODE;
    }
    else if ((bflg_prot_Iout_POC == 1) || (bflg_prot_Iout_OC == 1))
    {
    	  guc_trip_code = Iout_OC_PROT_CODE;
        guc_lamp_trip_code = LAMP_Iout_OC_PROT_CODE;
    }
    else if (bflg_prot_ALM == 1)
    {
    	  guc_trip_code = ALM_PROT_CODE;
        guc_lamp_trip_code = LAMP_ALM_PROT_CODE;
    }
    else if (bflg_prot_Tm_OH == 1)
    {
    	  guc_trip_code = Tm_OH_PROT_CODE;
        guc_lamp_trip_code = LAMP_Tm_OH_PROT_CODE;
    }
    else if ((bflg_Hall_fault == 1) || (bflg_Hall_error == 1))
    {
    	  guc_trip_code = Hall_FAULT_CODE;
        guc_lamp_trip_code = LAMP_Hall_FAULT_CODE;
    }
    else if (bflg_prot_Hall_direction == 1)
    {
    	  guc_trip_code = Hall_direct_FAULT_CODE;
        guc_lamp_trip_code = LAMP_Hall_direct_FAULT_CODE;
    }
    else if (bflg_motor_ID_error == 1)
    {
    	  guc_trip_code = MOTOR_ID_ERR_CODE;
        guc_lamp_trip_code = LAMP_MOTOR_ID_ERR_CODE;
    }
    else if ((bflg_running == 0) && (bflg_stop_space_delaytime == 0) && (bflg_trip_lock == 0) && (bflg_trip_clear == 1))
    {
    	  guc_trip_code = 0;
    	  guc_lamp_trip_code = 0;
    	  bflg_trip_stop = 0;
    	  bflg_trip_clear = 0;
    }
    
}
//-------------------------------------------------------------------------------
void state_code_deal(void)    //״̬���봦���������ѭ���е���
{
	  if (bflg_prot_Iout_decfreq == 1)
	  {
	  	  guc_state_code = OC_DECFREQ_STATE_CODE;
	  }
	  else if (bflg_prot_Tm_decfreq == 1)
	  {
	  	  guc_state_code = Tm_OH_DECFREQ_STATE_CODE;
	  }
	  else if (bflg_prot_Udc_stall == 1)
	  {
	  	  guc_state_code = OU_STALL_STATE_CODE;
	  }
	  else if (bflg_prot_Iout_stall == 1)
	  {
	  	  guc_state_code = OC_STALL_STATE_CODE;
	  }
	  else if (bflg_prot_Tm_stall == 1)
	  {
	  	  guc_state_code = Tm_OH_STALL_STATE_CODE;
	  }
	  else
	  {
	  	  guc_state_code = Normal_STATE_CODE;
	  }
}
//-------------------------------------------------------------------------------
void trip_stop_deal(uint8_t luc_trip_code)   //���ϴ������
{
	  if (bflg_trip_stop == 0)
	  {
	  	  bflg_trip_stop = 1;
	  	  
	  	  shut_pwm();           //�ر�PWM
	  	  
	  	  bflg_trip_clear = 0;//debug 
	  	  gus_trip_release_cnt = 0;        //������ͷż�����
	  	  //------------------------------
	  	  record_trip_cause(luc_trip_code);    //��¼����ԭ��
	  	  
	  	  //luc_trip_code = 0;
	  	  //------------------------------
	  	  //��������������ʱ
	  	  if (bflg_trip_lock_delaytime == 0)
	  	  {
	  	  	  bflg_trip_lock_delaytime = 1;
	  	  	  gss_trip_lock_delaytimer = 0;
	  	  }
	  	  //------------------------------
	  	  //���ϴ����ۼӣ������������ж�
        if((bflg_eeprom_change == 0)&&(bflg_SCI0_fault==0))
        {	  //E2�������λ��ͨѶ���ϣ����ۼƹ��ϴ��� 	  
	  	      if ((bflg_prot_Iout_OL == 1)||(bflg_prot_Iout_POC == 1) || (bflg_prot_Iout_OC == 1))
	  	      {
	  	      	  if (guc_Iout_fault_cnt >= 5)//����ǵ����������ۼ�5������루����:�����ɹ���ֵΪ5�� 
	  	      	  {
	  	      	  	  guc_trip_stop_cnt++;
	  	      	  }
	  	      }
	  	      else if (bflg_prot_ALM == 1)
	  	      {
	  	      	  if (guc_ALM_fault_cnt >= 3) 
	  	      	  {
	  	      	  	  guc_trip_stop_cnt++;
	  	      	  }	  	      	
	  	      }
	  	      else if (bflg_prot_motor_block == 1)
	  	      {
	  	      	  if (guc_motor_block_cnt >= 5) 
	  	      	  {
	  	      	  	  guc_trip_stop_cnt++;
	  	      	  }	  	      	
	  	      }
	  	      else 
	  	      {
	  	      	  guc_trip_stop_cnt++;
	  	      }
	     	}  
	     	//--------------------------------------------------------
	  	  if (guc_trip_stop_cnt >= ram_para[num_trip_stop_cnt])//6
	  	  {
	  	  	  bflg_trip_lock = 1;
	  	  } 
	  }
}
//-------------------------------------------------------------------------------
/*void trip_stop_deal_op(void)    //���ϱ�����ͣ������
{
      bflg_running=0;       //��������б�־λ
      
      bflg_trip_stop = 1;     //�ù���ͣ����־
            
      bflg_askfor_stop=0;   //���Ҫ��ͣ����־λ
      bflg_startup_time=0;  //�������׶α�־
      
      bflg_startup_ready=0;  //������׼���׶α�־

      shut_pwm();            //�ر�PWM
      
      //record_trip_cause();    //��¼����ԭ��
      
    if (ram_para[num_space_delaytime] != 0)
    {
    	  bflg_space_delaytime = 1;
        gss_space_delaytimer = ram_para[num_space_delaytime];   
    }
      //--------------------------------------------
      bflg_askfor_release_tirp = 1;    //��Ҫ������ͷű�־λ
      gus_trip_release_cnt = 0;        //������ͷż�����
      //-------------------------------------------
      if(bflg_1Hr_trip_cnt == 0)    //���1Сʱ���ϼ�ʱ��־δ���������
      {
        bflg_1Hr_trip_cnt = 1;        //�ù���1Сʱ��ʱ��־
        gss_trip_1Hr_timer = 60;       //60min��1Сʱ
      }
      //---------------------------
      if((bflg_eeprom_change == 0)&&(bflg_sci0comm_err_prot==0))
      {
     	   gus_prot_stop_counter++;      //���ϱ���������(��ͨѶ���Ϻ�EEPROM�����)
      }
      //----------------------------- 
      if(gus_prot_stop_counter >= ram_para[num_prot_rst_cnt])   //���ϴ��������趨�����������
      {
         bflg_trip_lock = 1;  //����������־
      }
      //--------------------------
}
*/
//------------------------------------------------------------------------------
void trip_lock_delaytime(void)     //����������ʱ������1min��ʱ�����е���
{
    if (bflg_trip_lock_delaytime == 1)
    {
    	  gss_trip_lock_delaytimer++;
    	  if (gss_trip_lock_delaytimer >= ram_para[num_trip_lock_delaytime]) //60min
    	  {
    	  	  gss_trip_lock_delaytimer = 0;
    	  	  bflg_trip_lock_delaytime = 0;
    	  	  
    	  	  guc_trip_stop_cnt = 0;
    	  }
    }
}
//------------------------------------------------------------------------------
#endif
