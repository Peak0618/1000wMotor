#ifndef _EEPROM_RW_C_
#define _EEPROM_RW_C_
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *    版权说明：Copyright(C) 2007-2012 青岛斑科变频技术有限公司              *
// *    文件名：  EEPROM_RW.c                                                  * 
// *    作者:     林爱军                                                       *
// *    文件版本: 1.00                                                         *
// *    生成日期: 2009年6月24日                                                *
// *    描述：    对EEPROM进行参数读写的程序                                   *
// *    版本信息:                                                              *
// *        必需的其他文件:  EEPROM_RW.h                                       *
// *        使用的工具:                                                        *
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *        支持的芯片:                                                        *
// *    函数列表:                                                              *
// *            1.                                                             * 
// *    历史修改记录:                                                          *
// *            1. 作者：                                                      *
// *               日期：                                                      *
// *               版本：                                                      *
// *               描述：                                                      *
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
uint8_t  writeword_to_eeprom(int16_t *value,uint16_t first_adds,uint16_t counter);   //按字写EEPROM的程序

uint8_t  readword_from_eeprom(int16_t *result,uint16_t first_adds,uint16_t counter);   //按字读EEPROM的程序

uint16_t read_para_eeptoram(void);    //从eeprom中读取参数入ram

uint32_t get_eeppara_limit(uint16_t index);    //设置参数上下限的程序

//void para_convert_deal(void);  //参数转换处理程序

uint16_t init_all_para(void);   //参数初始化程序

void clear_trip_cause(void);   //清除跳停原因

void record_trip_cause(uint8_t luc_TripCode);
//------------------------------------------------------------------------------
uint8_t  writeword_to_eeprom(int16_t *value,uint16_t first_adds,uint16_t counter)   //按字写EEPROM的程序
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
	 //返回值：0：代表操作正常结束  1：代表操作异常结束  2：输入参数错误，溢出 
   return_value = write_to_eeprom(lp_res,byte_adds,byte_cnt,0);
   
   //------------------------------------------------
   return return_value;	
}
//------------------------------------------------------------------------------
uint8_t  readword_from_eeprom(int16_t *result,uint16_t first_adds,uint16_t counter)   //按字读EEPROM的程序
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
    //返回值：0：代表操作正常结束  1：代表操作异常结束  2：输入参数错误，溢出
    return_value = read_from_eeprom(lp_res,byte_adds,byte_cnt,0);	
    //------------------------------------------------
    return return_value;		  
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// *****************************************************************************
// *                                                                           *
// *  函数名称:  uint8_t read_para_eeptoram(void)                              *
// *  功能描述:  从eeprom读取参数到RAM中的程序                                 *
// *  扇入函数:  //被本函数调用的函数清单                                      *
// *             read_from_eeprom();                                           *
// *             get_eeppara_limit();                                          *
// *  扇出函数:  //调用本函数的函数清单                                        *
// *             未知                                                          *
// *  输入参数:   无                                                           *
// *  输出参数:   无                                                           *
// *  全局变量：  无                                                           *
// *  函数返回值:        0：代表操作正常结束                                        *
// *                1：代表读操作异常结束                                      *
// *                2：校验字错                                                *
// *                3：校验和错                                                *
// *                4：输入参数错误，溢出                                      *
// *  其他说明:    无                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
uint16_t read_para_eeptoram(void)    //从eeprom中读取参数入ram
{
	uint16_t i;
	lword_type tmp;
	
    //uint8_t read_from_eeprom(*result, byte_address, byte_counter,CS_A2A1A0);
	//返回值：0：代表操作正常结束  1：代表操作异常结束  2：输入参数错误，溢出
	//tmp.word.low = read_from_eeprom(ram_para,0,(MAX_PARA_NUMBER+2)*2,0);
	tmp.word.low = readword_from_eeprom(ram_para,0,MAX_PARA_NUMBER+2);  //20091214 jion
	//------------------------------------------------
	if(tmp.word.low!=0)
	{
		   tmp.word.low = 1;  //置读EEPROM错标志
	}
	else
	{
		 	 if(ram_para[num_verify_word]!= VERIFY_WORD)
	     {
	      	   tmp.word.low = 2;   //置EEPROM检验字错误标志
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
	     	   	    tmp.word.low = 3;   //置EEPROM校验和错误标志
	     	   	    	    
	     	   	}
	     	   	else*/
	     	   	//{
	               for(i =0;i<MAX_PARA_NUMBER;i++)     //检查每个参数是否都在上下限范围之内
	               {
	                    tmp.lword=get_eeppara_limit(i);
                      if((ram_para[i] > tmp.word.high)||(ram_para[i] < tmp.word.low))
                      {
                    	   tmp.word.low = 4;   //置EEPROM参数溢出标志
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
// *  函数名称:  uint32_t get_eeppara_limit(uint16_t index)                    *
// *  功能描述:  得到EEPROM中参数上下限的程序                                  *
// *  扇入函数:  //被本函数调用的函数清单                                      *
// *  扇出函数:  //调用本函数的函数清单                                        *
// *             未知                                                          *
// *  输入参数:   index   参数指针索引                                         *
// *  输出参数:   无                                                           *
// *  全局变量：  无                                                           *
// *  函数返回值:   //返回32位结果 高16位存放上限  低16位存放下限              *
// *  其他说明:    无                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
uint32_t get_eeppara_limit(uint16_t index)    //设置参数上下限的程序
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
// *  函数名称:  void para_convert_deal(void)                                  *
// *  功能描述:  将读到RAM中的EEPROM参数转换为程序控制中使用的参数             *
// *  扇入函数:  //被本函数调用的函数清单                                      *
// *  扇出函数:  //调用本函数的函数清单                                        *
// *             未知                                                          *
// *  输入参数:   无                                                           *
// *  输出参数:   无                                                           *
// *  全局变量：  无                                                           *
// *  函数返回值:   无                                                         *
// *  其他说明:    无                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
/*void para_convert_deal(void)  //参数转换处理程序
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
// *  函数名称:  uint16_t init_all_para(void)                                  *
// *  功能描述:  用出厂默认参数初始化EEPROM的程序                              *
// *  扇入函数:  //被本函数调用的函数清单                                      *
// *             write_to_eeprom();                                            *
// *             read_from_eeprom();                                           *
// *  扇出函数:  //调用本函数的函数清单                                        *
// *             未知                                                          *
// *  输入参数:   无                                                           *
// *  输出参数:   无                                                           *
// *  全局变量：  无                                                           *
// *  函数返回值:   0：代表操作正常结束  1：代表写操作异常结束  2：校验错      *
// *  其他说明:    无                                                          *
// *                                                                           *
// *****************************************************************************
//------------------------------------------------------------------------------
uint16_t init_all_para(void)   //参数初始化程序
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
  ram_para[num_soft_ver]= SOFT_VERSION;//仅面板EE初始化时，将程序里的版本号写入EE.
  tmp.lword=0;
  //-------------------------------------------
  //uint8_t write_to_eeprom(*value, byte_address, byte_counter,CS_A2A1A0);
	//返回值：0：代表操作正常结束  1：代表操作异常结束 (写入地址出错)  
	//        2：代表操作异常结束 (写入数据出错) 3: 输入参数错误，溢出
  //tmp.word.low = write_to_eeprom(ram_para,0,(MAX_PARA_NUMBER+2)*2,0);
  tmp.word.low = writeword_to_eeprom(ram_para,0,MAX_PARA_NUMBER+2);   //按字写EEPROM的程序
	
	tmp.word.high = readword_from_eeprom(&verifyword,num_verify_word,1);  //20091214 jion
	    
  //tmp.word.low = readword_from_eeprom(&checksum,num_check_sum,1);       //20091214 jion

	if(tmp.word.low!=0)
	{
     tmp.word.low=1;     //写入EEPROM异常
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
		  	  tmp.word.low=1;     //写入EEPROM异常
		  }
		  else
		  {
			    //if((ram_para[num_verify_word]!= verifyword)||(ram_para[num_check_sum]!= checksum))
			    if(ram_para[num_verify_word]!= verifyword)
			    {
				      tmp.word.low=2;     //写入EEPROM异常
			    }
		  }		 
	}
	
	return  tmp.word.low;
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void clear_trip_cause(void)   //清除跳停原因
{
    uint16_t  i;
    
    //lword_type tmp;
    //清除故障记录
    for(i=num_Trip_clear;i <=num_Trip_cause5;i++)
    {
        //ram_para[num_check_sum] -= ram_para[i];
        //ram_para[num_check_sum] += EEPROM_InitData[i][VAL_ROW];     
        ram_para[i] = EEPROM_InitData[i][VAL_ROW];
        //--------------------
        //tmp.ulword =  ram_para[i];
        //tmp.ulword *= EEPROM_InitData[i].multiple;
        //ram_para[i].uword = (uint16_t) tmp.ulword;  //将参数写入RAM的使用参数区         
        //--------------------
    }
    //----------------------------- 
    writeword_to_eeprom(&(ram_para[num_Trip_clear]),num_Trip_clear,6);   //按字写EEPROM的程序
    //-----------------------------
    //writeword_to_eeprom(&(ram_para[num_check_sum]),num_check_sum,1);   //写校验和
    //-----------------------------
}
//---------------------------------------------------------------------------
void record_trip_cause(uint8_t luc_TripCode)    //故障原因记录程序
{
      uint16_t i;
      lword_type tmp;
      //--------------------      
      //ram_para[num_check_sum] -= ram_para[num_Trip_cause5];
      //ram_para[num_check_sum] += guc_trip_code;
      //-------------------- 
      for(i=0;i<4;i++)
      {
         ram_para[num_Trip_cause5-i]= ram_para[num_Trip_cause5-i-1];  //故障代码依次后移一位
      }
      ram_para[num_Trip_cause1]=luc_TripCode;//guc_trip_code;    //把新的故障代码写入第一位中
      //--------------------
      for(i=num_Trip_cause1;i<=num_Trip_cause5;i++)
      {
         tmp.ulword =  ram_para[i];
         //tmp.ulword *= EEPROM_InitData[i].multiple;
         ram_para[i] = (uint16_t) tmp.ulword;  //将参数写入RAM的使用参数区
      }
      //--------------------  
      writeword_to_eeprom(&(ram_para[num_Trip_cause1]),num_Trip_cause1,5);   //按字写EEPROM的程序 
      //-----------------------------
      //writeword_to_eeprom(&(ram_para[num_check_sum]),num_check_sum,1);   //写校验和
      //-----------------------------
}
//------------------------------------------------------------------------------
#endif            //end 
