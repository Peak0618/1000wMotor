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
//程序声明
void Iout_prot_variable_init(void);//与Iout保护有关的变量初始化程序，在初始化程序中调用

void Idc_zero_adjust(void);        //输出电流Idc零点校准程序，在10ms定时程序中

void prot_ALM_delaytime(void);     //检测停机时ALM保护延时程序，加入10ms定时程序中

void ALM_prot_int(void);           //ALM保护中断，在IRQ3中断中调用

void prot_ALM_deal(void);          //ALM保护处理程序，在波形发生中断中调用

void prot_Iout_POC_deal(void);     //Iout瞬时保护处理程序，在AD1中断服务程序中调用

void prot_Iout_delaytime(void);    //Iout电流保护延时程序，在100ms定时程序中调用

void Udc_prot_variable_init(void); //母线电压保护有关变量初始化程序,在初始化程序中调用

void prot_Udc_POU_deal(void);      //Udc瞬时保护处理程序，在AD1中断服务中调用

void prot_Udc_delaytime(void);     //Udc电压保护延时程序，在100ms定时程序中调用

void prot_Tm_delaytime(void);      //Tm模块温度保护的条件，在100ms定时程序中调用

void prot_motor_stall_delaytime(void);  //电机失速保护延时程序，在10ms定时程序中调用

void prot_motor_block_delaytime(void);  //电机堵转保护延时程序，在10ms定时程序中调用

void prot_motor_overspeed_delaytime(void);   //电机超速保护延时程序，在100ms定时程序中调用

void trip_code_deal(void);         //故障代码处理程序，在主循环中调用

void state_code_deal(void);        //状态代码处理程序，在主循环中调用

void trip_stop_deal(uint8_t luc_trip_code);       //故障处理程序

void trip_lock_delaytime(void);    //故障锁定延时程序，在1min定时程序中调用
//------------------------------------------------------------------------------
//标志定义
flag_type flg_fault;
flag_type flg_prot;
//----------------------------------------------------------
//变量定义
//----------------------------------------------------------
//Idc保护相关变量
uint16_t  gus_Idc_zero_ad_low;      //Idc零点上限寄存器
uint16_t  gus_Idc_zero_ad_up;       //Idc零点下限寄存器

uint16_t  gus_Idc_zero_cnt;         //Idc零点累加次数寄存器
uint32_t  gul_Idc_zero_ad_sum;      //Idc零点累加和寄存器
uint16_t  gus_Idc_zero_ad;          //Idc零点寄存器

uint16_t  gus_Idc_fault_cnt;        //Idc CT故障持续计数器
//----------------------------------------------------------
//ALM保护相关变量
uint16_t  gus_prot_ALM_delaytimer;
uint16_t  gus_quit_ALM_delaytimer;

uint16_t  gus_prot_ALM_cnt;
//----------------------------------------------------------
//输出电流保护相关变量
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
//母线电压保护相关变量
uint16_t  gus_prot_Udc_POU_ad;
uint16_t  gss_prot_Udc_POU_delaycnt;

uint16_t  gus_prot_Udc_stall_amp;
//----------------------------------------------------------
//模块温度保护相关变量
uint16_t  gus_prot_Tm_OH_delaytimer;
uint16_t  gus_prot_Tm_decfreq_enter_delaytimer;
uint16_t  gus_prot_Tm_decfreq_quit_delaytimer;
uint16_t  gus_prot_Tm_freq;
uint16_t  gus_prot_Tm_stall_delaytimer;
uint16_t  gus_prot_Tm_stall_amp;
//----------------------------------------------------------
int16_t   gss_motor_stall_delaytimer;   //电机失速延时计时器
int16_t   gss_motor_block_delaytimer;   //电机堵转延时计时器
int16_t   gss_motor_overspeed_delaytimer;    //电机超速延时计时器
//----------------------------------------------------------
int16_t   gss_trip_lock_delaytimer;   //故障锁定延时计时器
int16_t   gss_stop_space_delaytimer;    //停机间隔时间

uint8_t   guc_state_code;     //状态代码
uint8_t   guc_lamp_trip_code; //故障灯闪烁次数
uint16_t gus_trip_release_cnt;     //清故障释放计数器
//----------------------------------------------------------
uint8_t  guc_trip_code;      //故障代码
uint8_t  guc_trip_stop_cnt;       //故障停机次数
uint8_t  guc_motor_block_cnt;
uint8_t  guc_Iout_fault_cnt;
uint8_t  guc_ALM_fault_cnt;
//------------------------------------------------------------------------------
//Iout保护的判断和处理程序
//------------------------------------------------------------------------------
void Iout_prot_variable_init(void)      //与Iout保护有关的变量初始化程序，在初始化程序中调用
{
    lword_type ltmp1;
    //------------------------------------------------------
    //给Idc零电流的参考值赋初值
    gus_Idc_zero_ad = ram_para[num_Iout_CT_zero_AD];
    gus_Idc_zero_ad_low = ram_para[num_Iout_CT_zero_AD] - ram_para[num_Iout_CT_zero_margin];   //零电流参考值的下限
    gus_Idc_zero_ad_up  = ram_para[num_Iout_CT_zero_AD] + ram_para[num_Iout_CT_zero_margin];   //零电流参考值的上限  	
    //------------------------------------------------------
    gul_Idc_zero_ad_sum = 0;  //Idc零点累加和寄存器
    gus_Idc_zero_cnt = 0;     //Idc零点累加次数寄存器
    gus_Idc_fault_cnt = 0;    //Idc CT故障持续计数器
    //------------------------------------------------------
    //将电流保护峰值转换为对应的AD值
    ltmp1.ulword = ram_para[num_prot_Iout_POC];
    ltmp1.ulword -= ram_para[num_Iout_ref];
    ltmp1.ulword <<= ram_para[num_Iout_gain_amp];
    ltmp1.ulword /= ram_para[num_Iout_gain];
    ltmp1.ulword += ram_para[num_Iout_ref_AD];
    
    gus_prot_Iout_POC_ad = ltmp1.ulword;
    //------------------------------------------------------
    //计算过流保护对应的电流值
    ltmp1.ulword = ram_para[num_Iout_rate];
    ltmp1.ulword *= ram_para[num_prot_Iout_OC];
    ltmp1.ulword /= 1000;        
    
    gus_prot_Iout_OC = ltmp1.ulword;    
    //------------------------------------------------------
    //计算过载保护对应的电流值
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
    //计算过流降频保护对应的电流值
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
    //计算过流失速对应的电流值    
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
void Idc_zero_adjust(void)    //输出电流Idc零点校准程序，在10ms定时程序中
{
    uint16_t lus_Idc_ad;
    
    if ((bflg_running == 0) && (gus_freq_out == 0) && (gus_freq_real == 0))     //只有在停机状态，才进入检查零点
    {
        lus_Idc_ad = (gus_Idc_ad_up + gus_Idc_ad_dw) >> 1;
        if ((lus_Idc_ad > gus_Idc_zero_ad_low) && (gus_Idc_ad < gus_Idc_zero_ad_up))
        {
            bflg_Idc_fault = 0;
            gus_Idc_fault_cnt = 0;
            
            gul_Idc_zero_ad_sum += gus_Idc_ad;
            gus_Idc_zero_cnt++;
            if (gus_Idc_zero_cnt >= 64)      //640ms 更新一次零点
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
void prot_ALM_delaytime(void)      //检测停机时ALM保护延时程序，加入10ms定时程序中
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
	  	  	  	  	  bflg_prot_ALM = 1;  //置ALM保护标志
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
	  	  	  	  
	  	  	  	  PWMOFF0B1 &=~0x0001;    //清PRTBST0位，释放PWM0引脚硬件保护B功能
	  	  	  }
	  	  }
	  }
	  else
	  {
	  	  gus_prot_ALM_delaytimer = 0;
	  }
}
//------------------------------------------------------------------------------
void ALM_prot_int(void)       //ALM中断，在IRQ03外中断中调用
{
	  if (ram_para[num_prot_ALM_cnt] == 0)     //如果参数设定计数次数为0，则直接
	  {
	  	  bflg_prot_ALM_delaycnt = 0;
	  	  gus_prot_ALM_cnt = 0;
	  	  
	  	  if (bflg_prot_ALM == 0)
	  	  {
            //----------------------------------------------
            PWMSEL0 = 0x0FC0;      //PWM引脚由软件控制,全无效
            
            G24ICR &= 0xFFEF;      //清除IRQ03外中断请求标志
            //----------------------------------------------
            bflg_prot_ALM = 1;     //置ALM保护标志位
            trip_stop_deal(ALM_PROT_CODE);
	  	  }
	  }
	  else
	  {
	  	  if (bflg_prot_ALM_delaycnt == 0)
	  	  {
	  	  	  bflg_prot_ALM_delaycnt = 1;     //置ALM延时保护标志
	 	        gus_prot_ALM_cnt = 0;
	  	  }
	  }
}
//------------------------------------------------------------------------------
void prot_ALM_deal(void)      //ALM保护处理程序，在波形发生中断中调用
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
                PWMSEL0 = 0x0FC0;       //PWM引脚由软件控制,全无效
                
                G24ICR &= 0xFFEF;       //清除IRQ03外中断请求标志
                //------------------------------------------
                bflg_prot_ALM = 1;      //置Iout SC保护标志位
                trip_stop_deal(ALM_PROT_CODE);
            }
      	}
    }
}
//------------------------------------------------------------------------------
void prot_Iout_POC_deal(void) //Iout瞬时保护处理程序，在AD1中断服务程序中调用
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
                    PWMSEL0 = 0x0FC0;   //PWM引脚由软件控制,全无效
                    bflg_prot_Iout_POC = 1;   //置Iout瞬时保护标志位
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
void prot_Iout_delaytime(void)     //Iout电流保护延时程序，在100ms定时程序中调用
{
	  int16_t lss_tmp;
	  
	  if (bflg_running == 1)
	  {
        //过流保护处理
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
            //过载保护处理
            if (gus_Iout >= gus_prot_Iout_OL)
            {
          	    if (bflg_prot_Iout_OL_integ == 0)
          	    {
          	 	      bflg_prot_Iout_OL_integ = 1;    //置过载积分标志
          	    }
            }
            else    //如果电流值小于开始积分阀值，则进入
            {
          	    if ((bflg_prot_Iout_OL_integ == 1) && (gsl_prot_Iout_OL_integ_buf <= 0))  //有过载积分标志且积分值为零，则进入
          	    {
          	  	    bflg_prot_Iout_OL_integ = 0;  //清过载积分标志
          	  	    gsl_prot_Iout_OL_integ_buf = 0;
          	    }
            }
            //----------------------------------------------
	          if (bflg_prot_Iout_OL_integ == 1)     //如果是过载积分标志，则进入
	          {
                gsl_prot_Iout_OL_integ_buf += (gus_Iout - ram_para[num_Iout_rate]);
                
                if (gsl_prot_Iout_OL_integ_buf >= gsl_prot_Iout_OL_integ_max)
                {
        	          gsl_prot_Iout_OL_integ_buf = 0;
        	          bflg_prot_Iout_OL_integ = 0;
        	          
        	          if (bflg_prot_Iout_OL == 0)
        	          {
        	          	  bflg_prot_Iout_OL = 1;    //置输出电流过载保护
        	          	  trip_stop_deal(Iout_OL_PROT_CODE);
        	          }
                }
                else if(gsl_prot_Iout_OL_integ_buf < 0)
                {
        	          gsl_prot_Iout_OL_integ_buf = 0;
                }
	          }          
            //------------------------------------------------------------------
            //过流降频处理
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
          	    
          	    if (bflg_prot_Iout_decfreq == 1)  //如果有降频标志
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
            //过流失速判断
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
//母线电压Udc保护的判断和处理程序
//------------------------------------------------------------------------------
void Udc_prot_variable_init(void)       //母线电压保护有关变量初始化程序，在初始化程序中调用
{
    lword_type ltmp1;
    //--------------------------------------------
    //将Udc保护峰值转换为对应的AD值
    ltmp1.ulword = ram_para[num_prot_Udc_POU];
    ltmp1.ulword -= ram_para[num_Udc_ref];
    ltmp1.ulword <<= ram_para[num_Udc_gain_amp];
    ltmp1.ulword /= ram_para[num_Udc_gain];
    ltmp1.ulword += ram_para[num_Udc_ref_AD];
    
    gus_prot_Udc_POU_ad = ltmp1.ulword;
    //--------------------------------------------
}
//------------------------------------------------------------------------------
void prot_Udc_POU_deal(void)       //Udc瞬时保护处理程序，在AD1中断服务中调用
{
    if (gus_Udc_ad >= gus_prot_Udc_POU_ad)
    {
   	    gss_prot_Udc_POU_delaycnt++;
   	    if (gss_prot_Udc_POU_delaycnt >= ram_para[num_prot_Udc_POU_delaycnt])
   	    {
   	    	  gss_prot_Udc_POU_delaycnt = 0;
   	    	  
   	    	  if (bflg_prot_Udc_POU == 0)
   	    	  {
   	    	  	  PWMSEL0 = 0x0FC0;   //PWM引脚由软件控制,全无效
   	    	  	  
   	    	  	  bflg_prot_Udc_POU = 1;
   	    	  	  trip_stop_deal(Udc_OU_PROT_CODE);
   	    	  }
   	    }
    }
    else
    {
	      gss_prot_Udc_POU_delaycnt = 0;
	      //bflg_prot_Udc_POU = 0;//debug
	      
	      if (bflg_running == 0) //故障停机标志清除后，再清除故障标志
    	  {
			  	  if (gus_Udc_max < ram_para[num_prot_Udc_OU_quit])   //376V
			  	  {
			  	  	  bflg_prot_Udc_POU = 0;
			  	  }    	  
        }	
    }
}
//------------------------------------------------------------------------------
void prot_Udc_delaytime(void)      //Udc电压保护延时程序，在100ms定时程序中调用
{
	  //------------------------------------------------------
	  //过压保护判断
	  if (bflg_prot_Udc_OU == 0)     //如果当前无过压保护
	  {
	  	  if (gus_Udc_max > ram_para[num_prot_Udc_OU_enter])  //400V
	  	  {
	  	  	  bflg_prot_Udc_OU = 1;
	  	  	  trip_stop_deal(Udc_OU_PROT_CODE);
	  	  }
	  }
	  else  //如果有过压保护
	  {
    	  if (bflg_running == 0) //故障停机标志清除后，再清除故障标志
    	  {
			  	  if (gus_Udc_max < ram_para[num_prot_Udc_OU_quit])   //376V
			  	  {
			  	  	  bflg_prot_Udc_OU = 0;
			  	  }    	           
        }		  	  
	  }
	  //------------------------------------------------------
	  //欠压保护判断
	  if (bflg_prot_Udc_LU == 0)     //如果无欠压保护
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
	  	  
    	  if (bflg_running == 0) //故障停机标志清除后，再清除故障标志
    	  {
			      if (gus_Udc_for_disp > ram_para[num_prot_Udc_LU_quit])   //218V
			      {
			      	  bflg_prot_Udc_LU = 0;
			      }    	          
        }		  	  
	  }
	  //------------------------------------------------------
	  //过压失速保护判断
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
//模块温度保护处理程序
//-------------------------------------------------------------------------------
void prot_Tm_delaytime(void)       //Tm模块温度保护的条件，在100ms定时程序中调用
{
	  int16_t lss_tmp;
	  
	  if (ram_para[num_Tm_enbale] == 1)   //如果Tm模块温度保护允许，则进入
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
                //Tm模块温度过热降频判断
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
                //Tm模块温度过热失速判断
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
void prot_motor_stall_delaytime(void)   //电机失速保护延时程序，在10ms定时程序中调用
{
	  if (bflg_running == 1)    //如果电机运行
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
void prot_motor_block_delaytime(void)   //电机堵转保护延时程序，在10ms定时程序中调用
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
	  	  
    	  if (bflg_running == 0) //故障停机标志清除后，再清除故障标志
    	  {
    	      bflg_prot_motor_block = 0;   
        }		  	    
	  }
}
//-------------------------------------------------------------------------------
void prot_motor_overspeed_delaytime(void)    //电机超速保护延时程序，在100ms定时程序中调用
{
	  if (gus_speed_real >= gus_motor_overspeed)    //如果实际转速超速
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

    	  if (bflg_running == 0) //故障停机标志清除后，再清除故障标志
    	  {
    	      bflg_prot_motor_overspeed = 0;  
        }	  	  
	  }
}
//-------------------------------------------------------------------------------
void trip_code_deal(void)     //故障代码处理程序，在主循环中调用
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
            guc_trip_code = CLEAR_TRIP_CODE;     //清除跳停原因的显示代码
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
void state_code_deal(void)    //状态代码处理程序，在主循环中调用
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
void trip_stop_deal(uint8_t luc_trip_code)   //故障处理程序
{
	  if (bflg_trip_stop == 0)
	  {
	  	  bflg_trip_stop = 1;
	  	  
	  	  shut_pwm();           //关闭PWM
	  	  
	  	  bflg_trip_clear = 0;//debug 
	  	  gus_trip_release_cnt = 0;        //清故障释放计数器
	  	  //------------------------------
	  	  record_trip_cause(luc_trip_code);    //记录故障原因
	  	  
	  	  //luc_trip_code = 0;
	  	  //------------------------------
	  	  //开启故障锁定计时
	  	  if (bflg_trip_lock_delaytime == 0)
	  	  {
	  	  	  bflg_trip_lock_delaytime = 1;
	  	  	  gss_trip_lock_delaytimer = 0;
	  	  }
	  	  //------------------------------
	  	  //故障次数累加，及故障锁定判断
        if((bflg_eeprom_change == 0)&&(bflg_SCI0_fault==0))
        {	  //E2变更和上位机通讯故障，不累计故障次数 	  
	  	      if ((bflg_prot_Iout_OL == 1)||(bflg_prot_Iout_POC == 1) || (bflg_prot_Iout_OC == 1))
	  	      {
	  	      	  if (guc_Iout_fault_cnt >= 5)//如果是电流故障且累计5次则进入（处理:启动成功后赋值为5） 
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
/*void trip_stop_deal_op(void)    //故障保护的停机处理
{
      bflg_running=0;       //再清除运行标志位
      
      bflg_trip_stop = 1;     //置故障停机标志
            
      bflg_askfor_stop=0;   //清除要求停机标志位
      bflg_startup_time=0;  //清启动阶段标志
      
      bflg_startup_ready=0;  //清启动准备阶段标志

      shut_pwm();            //关闭PWM
      
      //record_trip_cause();    //记录故障原因
      
    if (ram_para[num_space_delaytime] != 0)
    {
    	  bflg_space_delaytime = 1;
        gss_space_delaytimer = ram_para[num_space_delaytime];   
    }
      //--------------------------------------------
      bflg_askfor_release_tirp = 1;    //置要求故障释放标志位
      gus_trip_release_cnt = 0;        //清故障释放计数器
      //-------------------------------------------
      if(bflg_1Hr_trip_cnt == 0)    //如果1小时故障计时标志未置起，则进入
      {
        bflg_1Hr_trip_cnt = 1;        //置故障1小时计时标志
        gss_trip_1Hr_timer = 60;       //60min＝1小时
      }
      //---------------------------
      if((bflg_eeprom_change == 0)&&(bflg_sci0comm_err_prot==0))
      {
     	   gus_prot_stop_counter++;      //故障保护次数加(除通讯故障和EEPROM变更外)
      }
      //----------------------------- 
      if(gus_prot_stop_counter >= ram_para[num_prot_rst_cnt])   //故障次数大于设定次数，则进入
      {
         bflg_trip_lock = 1;  //故障锁定标志
      }
      //--------------------------
}
*/
//------------------------------------------------------------------------------
void trip_lock_delaytime(void)     //故障锁定延时程序，在1min定时程序中调用
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
