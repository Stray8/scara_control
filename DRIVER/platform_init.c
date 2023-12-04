/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : main.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "platform_init.h"
#include "Delay.h"
#include "Usart.h"
#include "out.h"


#define USART1_DR_Base  (USART1_BASE + 0x04)
#define USART3_DR_Base  (USART3_BASE + 0x04)


/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{  
    ErrorStatus HSEStartUpStatus;
    RCC_DeInit();                                /* RCC system reset(for debug purpose) */
    RCC_HSEConfig(RCC_HSE_ON);                   /* Enable HSE */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();  /* Wait till HSE is ready */
    if(HSEStartUpStatus == SUCCESS)
    {  
      FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);  /* Enable Prefetch Buffer */   
      FLASH_SetLatency(FLASH_Latency_2);                     /* Flash 2 wait state */     
      RCC_HCLKConfig(RCC_SYSCLK_Div1);                       /* HCLK = SYSCLK */     
      RCC_PCLK2Config(RCC_HCLK_Div1);                        /* PCLK2 = HCLK   72M*/   
      RCC_PCLK1Config(RCC_HCLK_Div2);                        /* PCLK1 = HCLK/2 36M*/   
      RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);   /* PLLCLK = 8MHz * 9 = 72 MHz */   
      RCC_PLLCmd(ENABLE);                                    /* Enable PLL */     
      while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)     /* Wait till PLL is ready */
      { 
      }    
      RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);             /* Select PLL as system clock source */    
      while(RCC_GetSYSCLKSource() != 0x08)                   /* Wait till PLL is used as system clock source:72M */
      {
      }
    }  
	/* Enable GPIOA, GPIOC,USART1 and AFIO clock */
  	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOG | RCC_APB2Periph_GPIOF | 
							              RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOD | 
							              RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | 
							              RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 |
							              RCC_APB2Periph_AFIO	 | RCC_APB2Periph_TIM1		 //72M
							              , ENABLE );

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 |
                            RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5 |		//36M
                            RCC_APB1Periph_TIM6 | RCC_APB1Periph_UART4
                            , ENABLE ); 
                                
   	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);//电源管理部分时钟开启              
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructureOut;
    GPIO_InitTypeDef GPIO_InitStructureIn;
   	GPIO_InitTypeDef GPIO_InitStructure;


	/***************Usart端口配置*****************/
	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
 	/* Configure USART1 Rx (PA.10) as input floating */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Configure USART3 Tx (PB.10) as alternate function push-pull */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
 	/* Configure USART3 Rx (PB.11) as input floating */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	/* Configure USART4 Tx (PC.10) as alternate function push-pull */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
    
 	/* Configure USART4 Rx (PC.11) as input floating */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
 	//PD0.PD1.-485DIR								
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD,&GPIO_InitStructure);

	/***************信号输入端口配置*****************/
  	GPIO_InitStructureIn.GPIO_Mode = GPIO_Mode_IN_FLOATING;				 		//配置为输入
	//PA口	 PA4
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_4;
  	GPIO_Init(GPIOA,&GPIO_InitStructureIn);	  
	
	//PB口	 PB0.PB1
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  	GPIO_Init(GPIOB,&GPIO_InitStructureIn);
	
	//PC口	 PC0.PC1.PC4.PC5.PC13
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_13;
  	GPIO_Init(GPIOC,&GPIO_InitStructureIn);

	//PE口	 PE4.PE5.PE6.PE7.PE8.PE9.PE10.PE11.PE12.PE15
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9	
	                               |GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_15;
  	GPIO_Init(GPIOE,&GPIO_InitStructureIn);

	//PF口	 PIN_ALL
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_All;
  	GPIO_Init(GPIOF,&GPIO_InitStructureIn);

	//PG口   PG0.PG1
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  	GPIO_Init(GPIOG,&GPIO_InitStructureIn);

	/***************信号输出端口配置*****************/
	GPIO_InitStructureOut.GPIO_Mode=GPIO_Mode_Out_PP;
  	GPIO_InitStructureOut.GPIO_Speed=GPIO_Speed_50MHz;
	//A口	PA8.PA15
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_15;
	GPIO_Init(GPIOA,&GPIO_InitStructureOut);
	GPIO_SetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_15);

	//B口	PB3.PB4.PB5.PB6.PB7								
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOB,&GPIO_InitStructureOut);
	GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	

	//C口	PC6.PC7.PC8.PC9.PC10.PC11.PC14.PC15
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOC,&GPIO_InitStructureOut);
	GPIO_SetBits(GPIOC,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);
	GPIO_ResetBits(GPIOC,GPIO_Pin_14|GPIO_Pin_15);
	
	//D口	PD0.PD1.PD2.PD3.PD4.PD5.PD6.PD7
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOD,&GPIO_InitStructureOut);
 	GPIO_SetBits(GPIOD,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);

	//E口	PE0.PE1
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_Init(GPIOE,&GPIO_InitStructureOut);  
	GPIO_SetBits(GPIOE,GPIO_Pin_0|GPIO_Pin_1);

	//G口	PG3.PG4.PG5.PG6.PG7.PG8.PG9.PG10.PG11.PG12.PG13.PG14.PG15
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOG,&GPIO_InitStructureOut); 
	GPIO_SetBits(GPIOG,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);


	/*----GPIOA_5 电机使能 输出端口配置----*/
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_5;
	GPIO_Init(GPIOA,&GPIO_InitStructureOut);   
	
//	/*----GPIOB_14输入端口配置，用于掉电保存----*/
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);


    /******************电机控制接口***************************/
    /*----符号SIGN 输出端口配置----*/
    GPIO_InitStructureOut.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructureOut.GPIO_Speed = GPIO_Speed_50MHz;
 
    GPIO_InitStructureOut.GPIO_Pin = GPIO_Pin_7 ;  //PA7:X-SIGN 
    GPIO_Init(GPIOA, &GPIO_InitStructureOut); 
   
	GPIO_InitStructureOut.GPIO_Pin = GPIO_Pin_1 ;  //PA1:Z-SIGN
    GPIO_Init(GPIOA, &GPIO_InitStructureOut);

    GPIO_InitStructureOut.GPIO_Pin = GPIO_Pin_9 ;  //PB9:O-SIGN
    GPIO_Init(GPIOB, &GPIO_InitStructureOut);
	
    GPIO_InitStructureOut.GPIO_Pin = GPIO_Pin_14 ; //PE14:L-SIGN
    GPIO_Init(GPIOE, &GPIO_InitStructureOut);	  

   /*---- 脉冲PULS 输出端口配置----*/
    GPIO_InitStructureOut.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_InitStructureOut.GPIO_Speed=GPIO_Speed_50MHz;

	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_6;     //PA6:X-PULS  TIM3-CH1
	GPIO_Init(GPIOA,&GPIO_InitStructureOut); 
	
	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_0;     //PA0:Z-PULS  TIM2-CH1
 	GPIO_Init(GPIOA,&GPIO_InitStructureOut);

	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_8;     //PB8:O-PULS  TIM4-CH3
 	GPIO_Init(GPIOB,&GPIO_InitStructureOut);


	GPIO_InitStructureOut.GPIO_Pin=GPIO_Pin_13;    //PE13:L-PULS TIM1-CH3
 	GPIO_Init(GPIOE,&GPIO_InitStructureOut);


   /*---- 报警信号 输入端口配置----*/
	GPIO_InitStructureIn.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3 ;  //PC2.PC3:AXIS-tickle
    GPIO_Init(GPIOC, &GPIO_InitStructureIn);

    GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3 ;  //PE2.PE3:AXIS-tickle
    GPIO_Init(GPIOE, &GPIO_InitStructureIn);
	
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_8 ;   //PD8 :急停报警
    GPIO_Init(GPIOD, &GPIO_InitStructureIn);

   /*---- 限位开关信号 输入端口配置----*/
   /*- X22:PG0 X23:PG1 X24:PE8 X25:PE9 -*/
   #ifdef HARDLIMITJUDGE_EXTI
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;  
		GPIO_Init(GPIOG, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;  
		GPIO_Init(GPIOE, &GPIO_InitStructure);
   #endif 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	

	//开机先一直亮红灯-DPF
	SetOutput(O_RUN_GREEN_LIGHT);
	SetOutput(O_WAIT_YELLOW_LIGHT);
	SetOutput(O_ALARM_BUZZER);	  			//蜂鸣器
	ResetOutput(O_ALARM_RED_LIGHT); 		//红灯
}

void Power_EXTI_Init()
{
	EXTI_InitTypeDef  EXTI_InitStructure; 
    EXTI_DeInit();
    EXTI_StructInit(&EXTI_InitStructure);  
    EXTI_InitStructure.EXTI_Line = EXTI_Line14; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 						//
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);

 }


void Limit_EXTI_Init()
{
	EXTI_InitTypeDef  EXTI_InitStructure; 
    EXTI_DeInit();
    EXTI_StructInit(&EXTI_InitStructure);  
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource7);
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource8);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource1);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource8);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource9);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource5);		
  /* 
   //PE7 X左极限
    EXTI_InitStructure.EXTI_Line = EXTI_Line7; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 						//
	
   //PE8 X右极限
    EXTI_InitStructure.EXTI_Line = EXTI_Line8; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 						//
*/	
  //PG0  O
    EXTI_InitStructure.EXTI_Line = EXTI_Line0; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 						//
  	
  //PG1 X
    EXTI_InitStructure.EXTI_Line = EXTI_Line1; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 						//
	
  //PE8 L
    EXTI_InitStructure.EXTI_Line = EXTI_Line8; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 						//
	
  //PE9 Z
    EXTI_InitStructure.EXTI_Line = EXTI_Line9; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 					//
	
  //PC5 Z轴防撞
    EXTI_InitStructure.EXTI_Line = EXTI_Line5; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 					//	
  
}


//void PVD_Init(void)
//{
//	EXTI_InitTypeDef  EXTI_InitStructure; 
//    EXTI_DeInit();
//    EXTI_StructInit(&EXTI_InitStructure);  
//    EXTI_InitStructure.EXTI_Line = EXTI_Line16; 			// PVD连接到中断线16上  
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//使用中断模式  
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//电压低于阀值时产生中断  
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//使能中断线  
//    EXTI_Init(&EXTI_InitStructure); 						//初始 
//    PWR_PVDLevelConfig(PWR_PVDLevel_2V8); 					//设定监控阀值  
//    PWR_PVDCmd(ENABLE); 									//使能PVD 
// }


void TimeInit(void)
{ 
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;	                       
  
 	/**************************************************************************
 	//     定时器6设置： 720分频，向上计数，计数值为1000->10ms	  
 	***************************************************************************/

	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_Prescaler = 719;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM6,ENABLE);		  
}



/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures the NVIC and Vector Table base address.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    #ifdef  VECT_TAB_RAM  
	    /* Set the Vector Table base location at 0x20000000 */ 
      	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
    #else  /* VECT_TAB_FLASH  */
	    /* Set the Vector Table base location at 0x08000000 */ 
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); 
    #endif

    /* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Enable the TIM1_IRQ Interrupt */	     //L
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;//响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
		
	  /* Enable the TIM2_IRQ Interrupt */    //Z
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	   
	  
	  /* Enable the TIM3_IRQ Interrupt */    //X
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	  /* Enable the TIM4_IRQ Interrupt */	 //O
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	  /* Enable the TIM6_IRQ Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);				
	  
	  /* Enable the USART1_IRQ Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the UART4_IRQ Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//	  /* Enable the PVD Interrupt */ //设置PVD中断
//
//	NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQChannel;	
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
//	NVIC_Init(&NVIC_InitStructure);

//	  /* Enable the DMA1_Channel4_IRQ Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQChannel;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	/* 配置端口PB14为外部中断，用于掉电保存 */ 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}



	
void USART1_Configuration(void)
{
/* USART1 configuration -----------------------------------------------

-------*/
  /* USART1 configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
        - USART Clock disabled
        - USART CPOL: Clock is active low
        - USART CPHA: Data is captured on the middle 
        - USART LastBit: The clock pulse of the last data bit is not 

output to 
                         the SCLK pin			   */
 
    USART_ClockInitTypeDef  USART_ClockInitStructure;
	USART_InitTypeDef USART_InitStructure;
//	DMA_InitTypeDef   DMA_InitStructure;


//	DMA_DeInit(DMA1_Channel4);    
//    DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;  
//    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)SendDataBuffer;  
//    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  
//    DMA_InitStructure.DMA_BufferSize = 0;//sizeof(g_SendBuffer);  
//    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
//    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
//    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
//    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
//    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  
//    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
//    DMA_Init(DMA1_Channel4, &DMA_InitStructure); 

	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART1, &USART_ClockInitStructure);

	USART_InitStructure.USART_BaudRate            = 115200;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
	/* Enable USART1 DMA Txrequest */  
//    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  
//    /* Enable DMA1 Channel4 */  
//    DMA_Cmd(DMA1_Channel4, ENABLE); 
}

void UART4_Configuration(void)
{
//    USART_ClockInitTypeDef  USART_ClockInitStructure;
	USART_InitTypeDef USART_InitStructure;

//	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
//	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
//	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
//	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
//	USART_ClockInit(UART4, &USART_ClockInitStructure);

	USART_InitStructure.USART_BaudRate            = 9600;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(UART4, &USART_InitStructure);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART4, ENABLE);
	
}






/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/

