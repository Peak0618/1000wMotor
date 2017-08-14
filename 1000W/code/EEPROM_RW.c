#ifndef _EEPROM_RW_C_
#define _EEPROM_RW_C_
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *    ��Ȩ˵����Copyright(C) 2007-2012 �ൺ�߿Ʊ�Ƶ�������޹�˾              *
// *    �ļ�����  EEPROM_RW.c                                                  * 
// *    ����:     �ְ���                                                       *
// *    �ļ��汾: 1.00                                                         *
// *    ��������: 2009��6��24��                                                *
// *    ������    ��EEPROM���в�����д�ĳ���                                   *
// *    �汾��Ϣ:                                                              *
// *        ����������ļ�:  EEPROM_RW.h                                       *
// *        ʹ�õĹ���:                                                        *
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *        ֧�ֵ�оƬ:                                                        *
// *    �����б�:                                                              *
// *            1.                                                             * 
// *    ��ʷ�޸ļ�¼:                                                          *
// *            1. ���ߣ�                                                      *
// *               ���ڣ�                                                      *
// *               �汾��                                                      *
// *               ������                                                      *
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
#include "user_typedefine.h"
#include "SWI2C_EEPROM_Drive.h" 
#include "EEPROM_PARA.h"
#include "comm_variable.h"
#include "protect_deal.h"
//------------------------------------------------------------------------------
uint8_t  writeword_to_eeprom(int16_t *value,uint16_t first_adds,uint16_t counter);   //����дEEPROM�ĳ���

uint8_t  readword_from_eeprom(int16_t *result,uint16_t first_adds,uint16_t counter);   //���ֶ�EEPROM�ĳ���

uint16_t read_para_eeptoram(void);    //��eeprom�ж�ȡ������ram

uint32_t get_eeppara_limit(uint16_t index);    //���ò��������޵ĳ���

//void para_convert_deal(void);  //����ת���������

uint16_t init_all_para(void);   //������ʼ������

void clear_trip_cause(void);   //�����ͣԭ��

void record_trip_cause(uint8_t luc_TripCode);
//------------------------------------------------------------------------------
uint8_t  writeword_to_eeprom(int16_t *value,uint16_t first_adds,uint16_t counter)   //����дEEPROM�ĳ���
{
   uint8_t  return_value;
   uint8_t  *lp_res;
   uint16_t byte_adds;
   uint16_t byte_cnt;

   lp_res = (uint8_t *)value;
   
   byte_adds = first_adds;
   byte_adds <<= 1;
   
   byte_cnt = counter;
   byte_cnt <<=1;

   //uint8_t write_to_eeprom(*value, byte_address, byte_counter,CS_A2A1A0);
	 //����ֵ��0�����������������  1����������쳣����  2���������������� 
   return_value = write_to_eeprom(lp_res,byte_adds,byte_cnt,0);
   
   //------------------------------------------------
   return return_value;	
}
//------------------------------------------------------------------------------
uint8_t  readword_from_eeprom(int16_t *result,uint16_t first_adds,uint16_t counter)   //���ֶ�EEPROM�ĳ���
{
    uint8_t  return_value;
    uint8_t  *lp_res;
    uint16_t byte_adds;
    uint16_t byte_cnt;

    lp_res = (uint8_t *)result;

    byte_adds = first_adds;
    byte_adds <<= 1;

    byte_cnt = counter;
    byte_cnt <<=1;

    //uint8_t read_from_eeprom(*result, byte_address, byte_counter,CS_A2A1A0);
    //����ֵ��0�����������������  1����������쳣����  2����������������
    return_value = read_from_eeprom(lp_res,byte_adds,byte_cnt,0);	
    //------------------------------------------------
    return return_value;		  
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  ��������:  uint8_t read_para_eeptoram(void)                              *
// *  ��������:  ��eeprom��ȡ������RAM�еĳ���                                 *
// *  ���뺯��:  //�����������õĺ����嵥                                      *
// *             read_from_eeprom();                                           *
// *             get_eeppara_limit();                                          *
// *  �ȳ�����:  //���ñ������ĺ����嵥                                        *
// *             δ֪                                                          *
// *  �������:   ��                                                           *
// *  �������:   ��                                                           *
// *  ȫ�ֱ�����  ��                                                           *
// *  ��������ֵ:        0�����������������                                        *
// *                1������������쳣����                                      *
// *                2��У���ִ�                                                *
// *                3��У��ʹ�                                                *
// *                4����������������                                      *
// *  ����˵��:    ��                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
uint16_t read_para_eeptoram(void)    //��eeprom�ж�ȡ������ram
{
	uint16_t i;
	lword_type tmp;
	
    //uint8_t read_from_eeprom(*result, byte_address, byte_counter,CS_A2A1A0);
	//����ֵ��0�����������������  1����������쳣����  2����������������
	//tmp.word.low = read_from_eeprom(ram_para,0,(MAX_PARA_NUMBER+2)*2,0);
	tmp.word.low = readword_from_eeprom(ram_para,0,MAX_PARA_NUMBER+2);  //20091214 jion
	//------------------------------------------------
	if(tmp.word.low!=0)
	{
		   tmp.word.low = 1;  //�ö�EEPROM���־
	}
	else
	{
		 	 if(ram_para[num_verify_word]!= VERIFY_WORD)
	     {
	      	   tmp.word.low = 2;   //��EEPROM�����ִ����־
	     }
	     else
	     {
	     	   	tmp.word.high = 0;
	     	   	/*for(i =0; i <MAX_PARA_NUMBER; i++)
	     	   	{
	     	   		  tmp.word.high += ram_para[i];
	     	   	}
	     	   	//--------------------------
	     	   	if(tmp.word.high != ram_para[num_check_sum])
	     	   	{
	     	   	    tmp.word.low = 3;   //��EEPROMУ��ʹ����־
	     	   	    	    
	     	   	}
	     	   	else*/
	     	   	//{
	               for(i =0;i<MAX_PARA_NUMBER;i++)     //���ÿ�������Ƿ��������޷�Χ֮��
	               {
	                    tmp.lword=get_eeppara_limit(i);
                      if((ram_para[i] > tmp.word.high)||(ram_para[i] < tmp.word.low))
                      {
                    	   tmp.word.low = 4;   //��EEPROM���������־
                         break;      	  
                      }
	               }
	     	   	//}
	     }
	}
	
	return tmp.uword.low;
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  ��������:  uint32_t get_eeppara_limit(uint16_t index)                    *
// *  ��������:  �õ�EEPROM�в��������޵ĳ���                                  *
// *  ���뺯��:  //�����������õĺ����嵥                                      *
// *  �ȳ�����:  //���ñ������ĺ����嵥                                        *
// *             δ֪                                                          *
// *  �������:   index   ����ָ������                                         *
// *  �������:   ��                                                           *
// *  ȫ�ֱ�����  ��                                                           *
// *  ��������ֵ:   //����32λ��� ��16λ�������  ��16λ�������              *
// *  ����˵��:    ��                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
uint32_t get_eeppara_limit(uint16_t index)    //���ò��������޵ĳ���
{
	lword_type tmp;
	
	switch(index)
	{
		 //case num_base_freq:
		 //	    tmp.word.high = ram_para[num_up_lim_freq];
		 //	    tmp.word.low = ram_para[num_hevf_on_freq];  
		 //	    break;
     //case num_max_freq:	
     	
		 	    //break;
		 //-----------------------------------------	         		 	
		 case num_min_freq:
		 	    tmp.word.high = ram_para[num_max_freq];
		 	    tmp.word.low = EEPROM_InitData[num_min_freq][MIN_ROW]; 
		 	    break;		 	    		 	
		 case num_max_freq:
		 	    tmp.word.high = EEPROM_InitData[num_max_freq][MAX_ROW];
		 	    tmp.word.low = ram_para[num_min_freq]; 
		 	    break;
		 //case num_begin_freq:
		 //	    tmp.word.high = ram_para[num_low_lim_freq];
		 //	    tmp.word.low = EEPROM_InitData[num_begin_freq][MIN_ROW];
		 //	    break;
		 case num_shut_freq:
		 	    tmp.word.high = ram_para[num_max_freq];
		 	    tmp.word.low = ram_para[num_min_freq];
		 	    break;		 
		 /*case num_VF_max_freq:
		 	    tmp.word.high = EEPROM_InitData[num_VF_max_freq][MAX_ROW];
		 	    tmp.word.low = (ram_para[num_VF_mid2_freq]+10);
		 	    break;
		 case num_VF_max_volt:
		 	    tmp.word.high = EEPROM_InitData[num_VF_max_volt][MAX_ROW];
		 	    tmp.word.low = ram_para[num_VF_mid2_volt];		 	
		 	    break;		 	    		 
		 case num_VF_mid2_freq:
		 	    tmp.word.high = (ram_para[num_VF_max_freq]-10);
		 	    tmp.word.low = (ram_para[num_VF_mid1_freq]+10); 	
		 	    break;
		 case num_VF_mid2_volt:
		 	    tmp.word.high = ram_para[num_VF_max_volt];
		 	    tmp.word.low = ram_para[num_VF_mid1_volt];  	
		 	    break;		 	    
     case num_VF_mid1_freq:
		 	    tmp.word.high = (ram_para[num_VF_mid2_freq]-10);
		 	    tmp.word.low = (ram_para[num_VF_mid0_freq]+10);
		 	    break;
		 case num_VF_mid1_volt:
		 	    tmp.word.high = ram_para[num_VF_mid2_volt];
		 	    tmp.word.low = ram_para[num_VF_mid0_volt];
		 	    break;			 	    
     case num_VF_mid0_freq:
		 	    tmp.word.high = (ram_para[num_VF_mid1_freq]-10);
		 	    tmp.word.low = (ram_para[num_VF_min_freq]+10);     	
		 	    break;
		 case num_VF_mid0_volt:
		 	    tmp.word.high = ram_para[num_VF_mid1_volt];
		 	    tmp.word.low = ram_para[num_VF_min_volt]; 
          break;				 	    
     case num_VF_min_freq:
		 	    tmp.word.high = (ram_para[num_VF_mid0_freq]-10);
		 	    tmp.word.low = EEPROM_InitData[num_VF_min_freq][MIN_ROW];   	
		 	    break;
		 case num_VF_min_volt:
		 	    tmp.word.high = ram_para[num_VF_mid0_volt];
		 	    tmp.word.low = EEPROM_InitData[num_VF_min_volt][MIN_ROW];
          break;
     case num_boost_amount_volt:
		 	    tmp.word.high = ram_para[num_VF_min_volt];
		 	    tmp.word.low = EEPROM_InitData[num_boost_amount_volt][MIN_ROW];
          break;*/     	
    //-----------------------------------------
    case  num_THM_lowlim_AD:
		 	    tmp.word.high = ram_para[num_THM_uplim_AD];
		 	    tmp.word.low = EEPROM_InitData[num_THM_lowlim_AD][MIN_ROW];
    	    break;
    case  num_THM_uplim_AD:
		 	    tmp.word.high = EEPROM_InitData[num_THM_uplim_AD][MAX_ROW];
		 	    tmp.word.low = ram_para[num_THM_lowlim_AD]; 
        	break;
    //-----------------------------------------
    /*case  num_THF_lowlim_AD:
		 	    tmp.word.high = ram_para[num_THF_uplim_AD];
		 	    tmp.word.low = EEPROM_InitData[num_THF_lowlim_AD][MIN_ROW]; 
    	    break;
    case  num_THF_uplim_AD:
		 	    tmp.word.high = EEPROM_InitData[num_THF_uplim_AD][MAX_ROW];
		 	    tmp.word.low = ram_para[num_THF_lowlim_AD];
        	break;*/
    //-----------------------------------------
    case  num_prot_Iout_decfreq_enter:
		 	    tmp.word.high = EEPROM_InitData[num_prot_Iout_decfreq_enter][MAX_ROW];
		 	    tmp.word.low = ram_para[num_prot_Iout_decfreq_quit];
    	    break;
    case  num_prot_Iout_decfreq_quit:
		 	    tmp.word.high = ram_para[num_prot_Iout_decfreq_enter];
		 	    tmp.word.low = EEPROM_InitData[num_prot_Iout_decfreq_quit][MIN_ROW];
          break;
    //-----------------------------------------    
    case  num_prot_Iout_stall_enter:
		 	    tmp.word.high = EEPROM_InitData[num_prot_Iout_stall_enter][MAX_ROW];
		 	    tmp.word.low = ram_para[num_prot_Iout_stall_quit];
    	    break;
    case  num_prot_Iout_stall_quit:
		 	    tmp.word.high = ram_para[num_prot_Iout_stall_enter];
		 	    tmp.word.low = EEPROM_InitData[num_prot_Iout_stall_quit][MIN_ROW];
          break;
    //-----------------------------------------     
    case  num_prot_Tm_decfreq_enter:
		 	    tmp.word.high = EEPROM_InitData[num_prot_Tm_decfreq_enter][MAX_ROW];
		 	    tmp.word.low = ram_para[num_prot_Tm_decfreq_quit];
    	    break;
    case  num_prot_Tm_decfreq_quit:
		 	    tmp.word.high = ram_para[num_prot_Tm_decfreq_enter];
		 	    tmp.word.low = EEPROM_InitData[num_prot_Tm_decfreq_quit][MIN_ROW];    	
          break;
    //-----------------------------------------
    case  num_prot_Tm_stall_enter:
		 	    tmp.word.high = EEPROM_InitData[num_prot_Tm_stall_enter][MAX_ROW];
		 	    tmp.word.low = ram_para[num_prot_Tm_stall_quit]; 
    	    break;
    case  num_prot_Tm_stall_quit:
		 	    tmp.word.high = ram_para[num_prot_Tm_stall_enter];
		 	    tmp.word.low = EEPROM_InitData[num_prot_Tm_stall_quit][MIN_ROW];
          break;
    //-----------------------------------------
    case  num_prot_Udc_OU_enter:
		 	    tmp.word.high = EEPROM_InitData[num_prot_Udc_OU_enter][MAX_ROW];
		 	    tmp.word.low = ram_para[num_prot_Udc_OU_quit];
          break;    	
    case  num_prot_Udc_OU_quit:
		 	    tmp.word.high = ram_para[num_prot_Udc_OU_enter];
		 	    tmp.word.low = EEPROM_InitData[num_prot_Udc_OU_quit][MIN_ROW];
          break;
   //-----------------------------------------
    case  num_prot_Udc_stall_enter:
		 	    tmp.word.high = EEPROM_InitData[num_prot_Udc_stall_enter][MAX_ROW];
		 	    tmp.word.low = ram_para[num_prot_Udc_stall_quit];
          break;
    case  num_prot_Udc_stall_quit:
		 	    tmp.word.high = ram_para[num_prot_Udc_stall_enter];
		 	    tmp.word.low = EEPROM_InitData[num_prot_Udc_stall_quit][MIN_ROW];
          break;
   //-----------------------------------------
    case  num_prot_Udc_LU_enter:
		 	    tmp.word.high = ram_para[num_prot_Udc_LU_quit];
		 	    tmp.word.low = EEPROM_InitData[num_prot_Udc_LU_enter][MIN_ROW];
          break;
    case  num_prot_Udc_LU_quit:
		 	    tmp.word.high = EEPROM_InitData[num_prot_Udc_LU_quit][MAX_ROW];
		 	    tmp.word.low = ram_para[num_prot_Udc_LU_enter];
          break;
   //-----------------------------------------
    /*case  num_Iin_decfreq_enter:
		 	    tmp.word.high = EEPROM_InitData[num_Iin_decfreq_enter][MAX_ROW];
		 	    tmp.word.low = ram_para[num_Iin_decfreq_quit];
    	    break;
    case  num_Iin_decfreq_quit:
		 	    tmp.word.high = ram_para[num_Iin_decfreq_enter];
		 	    tmp.word.low = EEPROM_InitData[num_Iin_decfreq_quit][MIN_ROW];
          break;
   //-----------------------------------------   
    case  num_THF_decfreq_enter:
		 	    tmp.word.high = EEPROM_InitData[num_THF_decfreq_enter][MAX_ROW];
		 	    tmp.word.low = ram_para[num_THF_decfreq_quit];
    	    break;
    case num_THF_decfreq_quit:
		 	    tmp.word.high = ram_para[num_THF_decfreq_enter];
		 	    tmp.word.low = EEPROM_InitData[num_THF_decfreq_quit][MIN_ROW];
    	    break;
   //-----------------------------------------   
     case num_hevf_on_freq:
		 	    tmp.word.high = ram_para[num_low_lim_freq];
		 	    tmp.word.low = ram_para[num_begin_freq];
    	    break;
   //-----------------------------------------
     case num_PFC_on_freq:
		 	    tmp.word.high = EEPROM_InitData[num_PFC_on_freq][MAX_ROW];
		 	    tmp.word.low = ram_para[num_PFC_off_freq];
    	    break;
     case num_PFC_off_freq:
		 	    tmp.word.high = ram_para[num_PFC_on_freq];
		 	    tmp.word.low = EEPROM_InitData[num_PFC_off_freq][MIN_ROW];
    	    break;
   //-----------------------------------------
     //case num_PFC_on_Pout:
		 	    tmp.word.high = EEPROM_InitData[num_PFC_on_Pout][MAX_ROW];
		 	    tmp.word.low = ram_para[num_PFC_off_Pout];
    	    break;
     case num_PFC_off_Pout:
		 	    tmp.word.high = ram_para[num_PFC_on_Pout];
		 	    tmp.word.low = EEPROM_InitData[num_PFC_off_Pout][MIN_ROW];
    	    break;
   //-----------------------------------------
     case num_pfc_min_time:
		 	    tmp.word.high = ram_para[num_pfc_max_time];
		 	    tmp.word.low = EEPROM_InitData[num_pfc_min_time][MIN_ROW];
    	    break;
     case num_pfc_max_time:
		 	    tmp.word.high = EEPROM_InitData[num_pfc_max_time][MAX_ROW];
		 	    tmp.word.low = ram_para[num_pfc_min_time];
    	    break;*/	    
	 //------------------------------------------
     default:
		 	    tmp.word.high = EEPROM_InitData[index][MAX_ROW];
		 	    tmp.word.low = EEPROM_InitData[index][MIN_ROW];
	}
	
	tmp.word.high = EEPROM_InitData[index][MAX_ROW];//debug 
	tmp.word.low = EEPROM_InitData[index][MIN_ROW];
		 	    	
	return tmp.lword;	
}
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  ��������:  void para_convert_deal(void)                                  *
// *  ��������:  ������RAM�е�EEPROM����ת��Ϊ���������ʹ�õĲ���             *
// *  ���뺯��:  //�����������õĺ����嵥                                      *
// *  �ȳ�����:  //���ñ������ĺ����嵥                                        *
// *             δ֪                                                          *
// *  �������:   ��                                                           *
// *  �������:   ��                                                           *
// *  ȫ�ֱ�����  ��                                                           *
// *  ��������ֵ:   ��                                                         *
// *  ����˵��:    ��                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
/*void para_convert_deal(void)  //����ת���������
{
      uint16_t i;
	    lword_type tmp;
      for(i=0;  i< MAX_PARA_NUMBER; i++)
      {
          tmp.lword =  ram_para[i];
          tmp.lword *= EEPROM_InitData[i].multiple;
          ram_para[i].uword = (uint16_t) tmp.lword;
      }
}*/
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  ��������:  uint16_t init_all_para(void)                                  *
// *  ��������:  �ó���Ĭ�ϲ�����ʼ��EEPROM�ĳ���                              *
// *  ���뺯��:  //�����������õĺ����嵥                                      *
// *             write_to_eeprom();                                            *
// *             read_from_eeprom();                                           *
// *  �ȳ�����:  //���ñ������ĺ����嵥                                        *
// *             δ֪                                                          *
// *  �������:   ��                                                           *
// *  �������:   ��                                                           *
// *  ȫ�ֱ�����  ��                                                           *
// *  ��������ֵ:   0�����������������  1������д�����쳣����  2��У���      *
// *  ����˵��:    ��                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
uint16_t init_all_para(void)   //������ʼ������
{
	lword_type tmp;         
	//word_type verifyword;
	//word_type checksum;	
  int16_t verifyword;
	//int16_t checksum;	
	
	//tmp.word.high=0;
	tmp.lword=0;
	for(tmp.word.low=0; tmp.word.low< MAX_PARA_NUMBER; tmp.word.low++)
  {
       ram_para[tmp.word.low] = EEPROM_InitData[tmp.word.low][VAL_ROW];
       tmp.word.high += EEPROM_InitData[tmp.word.low][VAL_ROW];
  }	
  //-------------------------------------------
  ram_para[num_verify_word]= VERIFY_WORD;
  //ram_para[num_check_sum] = tmp.word.high;
  ram_para[num_soft_ver]= SOFT_VERSION;//�����EE��ʼ��ʱ����������İ汾��д��EE.
  tmp.lword=0;
  //-------------------------------------------
  //uint8_t write_to_eeprom(*value, byte_address, byte_counter,CS_A2A1A0);
	//����ֵ��0�����������������  1����������쳣���� (д���ַ����)  
	//        2����������쳣���� (д�����ݳ���) 3: ��������������
  //tmp.word.low = write_to_eeprom(ram_para,0,(MAX_PARA_NUMBER+2)*2,0);
  tmp.word.low = writeword_to_eeprom(ram_para,0,MAX_PARA_NUMBER+2);   //����дEEPROM�ĳ���
	
	tmp.word.high = readword_from_eeprom(&verifyword,num_verify_word,1);  //20091214 jion
	    
  //tmp.word.low = readword_from_eeprom(&checksum,num_check_sum,1);       //20091214 jion

	if(tmp.word.low!=0)
	{
     tmp.word.low=1;     //д��EEPROM�쳣
	}
	else
	{
		  tmp.lword=0;
		  //tmp.word.high = read_from_eeprom(&(verifyword.word),num_verify_word*2,2,0);	
		  //tmp.word.low = read_from_eeprom(&(checksum.word),num_check_sum*2,2,0);	  
		  tmp.word.high = readword_from_eeprom(&verifyword,num_verify_word,1);  //20091214 jion
	    
	    //tmp.word.low = readword_from_eeprom(&checksum,num_check_sum,1);       //20091214 jion

		  if((tmp.word.high != 0)||(tmp.word.low != 0))
		  {
		  	  tmp.word.low=1;     //д��EEPROM�쳣
		  }
		  else
		  {
			    //if((ram_para[num_verify_word]!= verifyword)||(ram_para[num_check_sum]!= checksum))
			    if(ram_para[num_verify_word]!= verifyword)
			    {
				      tmp.word.low=2;     //д��EEPROM�쳣
			    }
		  }		 
	}
	
	return  tmp.word.low;
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void clear_trip_cause(void)   //�����ͣԭ��
{
    uint16_t  i;
    
    //lword_type tmp;
    //������ϼ�¼
    for(i=num_Trip_clear;i <=num_Trip_cause5;i++)
    {
        //ram_para[num_check_sum] -= ram_para[i];
        //ram_para[num_check_sum] += EEPROM_InitData[i][VAL_ROW];     
        ram_para[i] = EEPROM_InitData[i][VAL_ROW];
        //--------------------
        //tmp.ulword =  ram_para[i];
        //tmp.ulword *= EEPROM_InitData[i].multiple;
        //ram_para[i].uword = (uint16_t) tmp.ulword;  //������д��RAM��ʹ�ò�����         
        //--------------------
    }
    //----------------------------- 
    writeword_to_eeprom(&(ram_para[num_Trip_clear]),num_Trip_clear,6);   //����дEEPROM�ĳ���
    //-----------------------------
    //writeword_to_eeprom(&(ram_para[num_check_sum]),num_check_sum,1);   //дУ���
    //-----------------------------
}
//---------------------------------------------------------------------------
void record_trip_cause(uint8_t luc_TripCode)    //����ԭ���¼����
{
      uint16_t i;
      lword_type tmp;
      //--------------------      
      //ram_para[num_check_sum] -= ram_para[num_Trip_cause5];
      //ram_para[num_check_sum] += guc_trip_code;
      //-------------------- 
      for(i=0;i<4;i++)
      {
         ram_para[num_Trip_cause5-i]= ram_para[num_Trip_cause5-i-1];  //���ϴ������κ���һλ
      }
      ram_para[num_Trip_cause1]=luc_TripCode;//guc_trip_code;    //���µĹ��ϴ���д���һλ��
      //--------------------
      for(i=num_Trip_cause1;i<=num_Trip_cause5;i++)
      {
         tmp.ulword =  ram_para[i];
         //tmp.ulword *= EEPROM_InitData[i].multiple;
         ram_para[i] = (uint16_t) tmp.ulword;  //������д��RAM��ʹ�ò�����
      }
      //--------------------  
      writeword_to_eeprom(&(ram_para[num_Trip_cause1]),num_Trip_cause1,5);   //����дEEPROM�ĳ��� 
      //-----------------------------
      //writeword_to_eeprom(&(ram_para[num_check_sum]),num_check_sum,1);   //дУ���
      //-----------------------------
}
//------------------------------------------------------------------------------
#endif            //end 
