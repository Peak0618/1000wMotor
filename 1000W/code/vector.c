//------------------------------------------------------------------------------
//包含文件
#include "srMN103SFJ9D.h"
#include "user_typedefine.h"
#include "comm_variable.h"
#include "EEPROM_PARA.h"

#include "PWM_CTRL.h"
#include "AD_Convert.h"
#include "Hallsignal_deal.h"
#include "protect_deal.h"
#include "Host_comm.h"
#include "panel_comm.h"

#include "SWI2C_EEPROM_Drive.h"
//-----------------------------------------------------------
//在主程序文件中声明和实现的中断服务程序,此处需要进行外部声明
extern  void TM23_1MS_INT(void);   //1ms定时中断服务程序
//--	#define macro definition		------------------
//--						valid only in this file

//--	RAM/structure declaration		------------------
//--						valid only in this file

//--	function prototype declaration	------------------
//--						valid only in this file
void  irq_nouse(void);          //  no use group interrupt
void	irq_group00( void );			//  group 00:
//void	irq_group01( void );			//  group 01:
void	irq_group02( void );			//  group 02:
void	irq_group03( void );			//  group 03:
void	irq_group04( void );			//  group 04:
void	irq_group05( void );			//  group 05:
void	irq_group06( void );			//  group 06:
void	irq_group07( void );			//  group 07:
//void	irq_group08( void );			//  group 08:
void	irq_group09( void );			//  group 09:
//void	irq_group10( void );			//  group 10:
void	irq_group11( void );			//  group 11:
//void	irq_group12( void );			//  group 12:
//void	irq_group13( void );			//  group 13:
//void	irq_group14( void );			//  group 14:
//void	irq_group15( void );			//  group 15:
void	irq_group16( void );			//  group 16:
//void	irq_group17( void );			//  group 17:
//void	irq_group18( void );			//  group 18:
void	irq_group19( void );			//  group 19:
void	irq_group20( void );			//  group 20:
//void	irq_group21( void );			//  group 21:
void	irq_group22( void );			//  group 22:
void	irq_group23( void );			//  group 23:
void	irq_group24( void );			//  group 24:
void	irq_group25( void );			//  group 25:
void	irq_group26( void );			//  group 26:
//void	irq_group27( void );			//  group 27:
//void	irq_group28( void );			//  group 28:
void	irq_group29( void );			//  group 29:
void	irq_group30( void );			//  group 30:

//--------------------------------------------------------------------------------------------------
/* declare functions for interrupt vector */
void (* const irq_vct_tbl[31])() = {
	irq_group00,						// group 00:	Watch-Dog Timer & system errer
	irq_nouse,					  	// group 01:	dummy
	irq_group02,						// group 02:	-
	irq_group03,						// group 03:	-
	irq_group04,						// group 04:	-
	irq_group05,						// group 05:	-
	irq_group06,						// group 06:	-
	irq_group07,						// group 07:	-
	irq_nouse,						  // group 08:	dummy
	irq_group09,						// group 09:	-
	irq_nouse,						  // group 10:	dummy
	irq_group11,						// group 11:	-
	irq_nouse,						  // group 12:	dummy
	irq_nouse,						  // group 13:	dummy
	irq_nouse,						  // group 14:	dummy
	irq_nouse,					  	// group 15:	dummy
	irq_group16,						// group 16:	-
	irq_nouse,					  	// group 17:	dummy 
	irq_nouse,              // group 18:	dummy
	irq_group19,						// group 19:	-
	irq_group20,						// group 20:	-
	irq_nouse,					  	// group 21:	dummy
	irq_group22,						// group 22:	-
	irq_group23,						// group 23:	-
	irq_group24,						// group 24:	-
	irq_group25,						// group 25:	-
	irq_group26,						// group 26:	-
	irq_nouse, 						  // group 27:	dummy
	irq_nouse,						  // group 28:	dummy
	irq_group29,						// group 29:	-
	irq_group30,						// group 30:	-
};


void vRESET( void )
{
	while(1){
		RSTCTR  = 0x00;					// 
		RSTCTR |= 0x01;					// micom reset
	}
}
/*-----------------------------------------------
    fail-safe function
-----------------------------------------------*/
void int_failsafe(void)
{
    vRESET();
}
/*-----------------------------------------------
    Watchdog timer overflow
-----------------------------------------------*/
void int_watchdog(void)
{
    RSTCTR = 0x00;                      /* Clear self-reset flag */
    RSTCTR = 0x01;                      /* Set self-reset flag */
}

/*-----------------------------------------------
    System error
-----------------------------------------------*/
void int_syserr(void)
{
	  vRESET();
}
//-------------------------------------------------

/*-----------------------------------------------
    no use group interrupt
-----------------------------------------------*/
void irq_nouse(void)
{
	 vRESET();
}

/*-----------------------------------------------
    Group 0 interrupt(Non-maskable)
-----------------------------------------------*/
void irq_group00(void)
{
    unsigned char factor;

    
    factor = *(unsigned char *)&NMICR;
    *(unsigned char *)&NMICR = (factor & 0x0f); /* clear interrupt request flag */
    
    if(factor & 0x01){                          /* fail-safe function */
    	 int_failsafe();
    }
    if(factor & 0x02){                          /* Watchdog timer overflow */
        int_watchdog();
    }
    if(factor & 0x04){                          /* System error */
        int_syserr();
    }
}

/*-----------------------------------------------
    Group 2 interrupt
-----------------------------------------------*/
void irq_group02(void)
{
    unsigned char factor;

    factor = *(unsigned char *)&G2ICR;
    *(unsigned char *)&G2ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                          /* External interrupt 0 */
        
    }
}

/*-----------------------------------------------
    Group 3 interrupt
-----------------------------------------------*/
void irq_group03(void)
{
    unsigned char factor;

    factor = *(unsigned char *)&G3ICR;
    *(unsigned char *)&G3ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                          /* Timer 0 underflow */
        
    }
    
    if(factor & 0x02){                          /* Timer 1 underflow */
    	
    }
}

/*-----------------------------------------------
    Group 4 interrupt
-----------------------------------------------*/
void irq_group04(void)
{
    unsigned char factor;

    factor = *(unsigned char *)&G4ICR;
    *(unsigned char *)&G4ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                          /* Timer 2 underflow */
        
    }

    if(factor & 0x02){                          /* Timer 3 underflow */
    	  
    	  TM23_1MS_INT();   //1ms定时中断服务程序
    	  
    }
}

/*-----------------------------------------------
    Group 5 interrupt
-----------------------------------------------*/
void irq_group05(void)
{
    unsigned char factor;

    factor = *(unsigned char *)&G5ICR;
    *(unsigned char *)&G5ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                          /* Timer 4 underflow */
        
    }
    
    if(factor & 0x02){                          /* Timer 5 underflow */
    	  
    	  
    }

}

/*-----------------------------------------------
    Group 6 interrupt
-----------------------------------------------*/
void irq_group06(void)
{
    unsigned char factor;

    factor = *(unsigned char *)&G6ICR;
    *(unsigned char *)&G6ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                          /* Timer 6 underflow */
        
    }

    if(factor & 0x02){                          /* Timer 7 underflow */
        
    }

}

/*-----------------------------------------------
    Group 7 interrupt
-----------------------------------------------*/
void irq_group07(void)
{
    unsigned char factor;

    factor = *(unsigned char *)&G7ICR;
    *(unsigned char *)&G7ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                          /* Timer 16 overflow/underflow */
        
    }

    if(factor & 0x02){                          /* Timer 16 compare/capture A  */
        
            
    }

    if(factor & 0x04){                          /* Timer 16 compare/capture B */
        
    }
}

/*-----------------------------------------------
    Group 9 interrupt
-----------------------------------------------*/
void irq_group09(void)
{
    unsigned char factor;

    factor = *(unsigned char *)&G9ICR;
    *(unsigned char *)&G9ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                          /* Timer 18 overflow/underflow */
        
    }

    if(factor & 0x02){                          /* Timer 18 compare/capture A  */
        
    	  
    }

    if(factor & 0x04){                          /* Timer 18 compare/capture B */
        
        
    }
}

/*-----------------------------------------------
    Group 11 interrupt
-----------------------------------------------*/
void irq_group11(void)
{
    unsigned char factor;

    factor = *(unsigned char *)&G11ICR;
    *(unsigned char *)&G11ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                          /* Timer 20 overflow/underflow */
        
    }

    if(factor & 0x02){                          /* Timer 20 compare/capture A  */
        
    }

    if(factor & 0x04){                          /* Timer 20 compare/capture B */
        
    }
}

/*-----------------------------------------------
    Group 16 interrupt
-----------------------------------------------*/
void irq_group16(void)
{
    unsigned char factor;
    factor = *(unsigned char *)&G16ICR;
    *(unsigned char *)&G16ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                           /* PWM0 underflow */

        PWM_Reload_INT();    //波形发生中断服务程序
    	  
    }

    if(factor & 0x02){                           /* PWM0 overflow */

    }
}

/*-----------------------------------------------
    Group 19 interrupt
-----------------------------------------------*/
void irq_group19(void)
{
    unsigned char factor;

    factor = *(unsigned char *)&G19ICR;
    *(unsigned char *)&G19ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                           /* AD0 conversion end  */
        
        
    }

    if(factor & 0x02){                           /* AD0 conversion end B */
        
    	  
    }
}

/*-----------------------------------------------
    Group 20 interrupt
-----------------------------------------------*/
void irq_group20(void)
{
    unsigned char factor;
    
    factor = *(unsigned char *)&G20ICR;
    *(unsigned char *)&G20ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                           /* AD1 conversion end  */
        
        AD1_Complete_INT();   //AD1转换结束中断处理程序
    	  
    }

    if(factor & 0x02){                           /* AD1 conversion end B */
        
        
    }
}
/*-----------------------------------------------
    Group 22 interrupt
-----------------------------------------------*/
void irq_group22(void)
{
    unsigned char factor;

    factor = *(unsigned char *)&G22ICR;
    *(unsigned char *)&G22ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                           /* External interrupt 1 */
        
    }
}

/*-----------------------------------------------
    Group 23 interrupt
-----------------------------------------------*/
void irq_group23(void)
{
    unsigned char factor;

    factor = *(unsigned char *)&G23ICR;
    *(unsigned char *)&G23ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                           /* External interrupt 2 */
    	
    }
}

/*-----------------------------------------------
    Group 24 interrupt
-----------------------------------------------*/
void irq_group24(void)
{
    unsigned char factor;

    factor = *(unsigned char *)&G24ICR;
    *(unsigned char *)&G24ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                           /* External interrupt 3 */
        
        ALM_prot_int();    //ALM保护中断，在IRQ03中断中调用
    }
}

/*-----------------------------------------------
    Group 25 interrupt
-----------------------------------------------*/
void irq_group25(void)
{
    unsigned char factor;

    factor = *(unsigned char *)&G25ICR;
    *(unsigned char *)&G25ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                           /* Serial 0 reception end  */
       
        SCI0_rx_int();        //SCI0接收中断程序
    	  
    }

    if(factor & 0x02){                           /* Serial 0 communication end/ transmission end  */
       
        SCI0_tx_int();        //SCI0发送中断程序
        
    }    
    
}

/*-----------------------------------------------
    Group 26 interrupt
-----------------------------------------------*/
void irq_group26(void)
{
    unsigned char factor;

    factor = *(unsigned char *)&G26ICR;
    *(unsigned char *)&G26ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                           /* Serial 1 reception end  */
        
        SCI1_rx_int();     //SCI1接收中断
        
    }

    if(factor & 0x02){                           /* Serial 1 communication end/ transmission end  */
        
        SCI1_tx_int();     //SCI1发送中断
        
    }  
}

/*-----------------------------------------------
    Group 29 interrupt
-----------------------------------------------*/
void irq_group29( void )				// disable to interrupt
{
    unsigned char factor;
    
    factor = *(unsigned char *)&G29ICR;
    *(unsigned char *)&G29ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                           /* External interrupt 8 */
        
        Hall_input_int();     //Hall信号输入中断服务程序
    }

    if(factor & 0x02){                           /* External interrupt 9 */
        
        Hall_input_int();     //Hall信号输入中断服务程序
    }    
}
/*-----------------------------------------------
    Group 30 interrupt
-----------------------------------------------*/
void irq_group30( void )				// disable to interrupt
{
    unsigned char factor;
     
    factor = *(unsigned char *)&G30ICR;
    *(unsigned char *)&G30ICR = (factor & 0x0f); /* clear interrupt request flag */

    if(factor & 0x01){                           /* External interrupt 10 */
        
        Hall_input_int();     //Hall信号输入中断服务程序
    }

    if(factor & 0x02){                           /* External interrupt 11 */
        
    }  

}
//------------------------------------------------------------------



