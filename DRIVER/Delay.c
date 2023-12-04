/*************** (C) COPYRIGHT 2012 Kingrobot manipulator Team ****************
* File Name          : Delay.c
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 20/06/2012
* Description        : This file delimits all the delay functions.
******************************************************************************/
#include "Delay.h"

//static uint8_t  fac_us=0;							//us延时倍乘数			   
//static uint16_t fac_ms=0;							//ms延时倍乘数,在ucos下,代表每个节拍的ms数

/************************************************************************************************************
**	输入参数：无
**	输出参数：无
**	函数功能：延时函数初始化
*************************************************************************************************************/
void delay_init(u8 SYSCLK)
{
//	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8
//	
//	fac_us=SYSCLK/8;							//为系统时钟的1/8  
//	fac_ms=(u16)fac_us*1000;					//非OS下,代表每个ms需要的systick时钟数   
}

/************************************************************************************************************
**	输入参数：nus  延时时长，单位us
**	输出参数：无
**	函数功能：延时函数
*************************************************************************************************************/
void delay_us(u32 nus)
{
	u32 i = 0;
//	u32 temp;	    	 
//	SysTick->LOAD=nus*fac_us; 					//时间加载	  		 
//	SysTick->VAL=0x00;        					//清空计数器
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数	  
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
//	SysTick->VAL =0X00;      					 //清空计数器	 
//	Delay_us(nus);
	for(i=0; i< nus*30;i++)
	{}
}


void delay_xms(u16 nms)
{	 		  	  
//	u32 temp;		   
//	SysTick->LOAD=(u32)nms*fac_ms;			//时间加载(SysTick->LOAD为24bit)
//	SysTick->VAL =0x00;           			//清空计数器
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数 
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));	//等待时间到达   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
//	SysTick->VAL =0X00;     		  		//清空计数器	  	    
} 

/************************************************************************************************************
**	输入参数：nms  延时时长，单位ms
**	输出参数：无
**	函数功能：延时函数
*************************************************************************************************************/
void delay_ms(u32 nms)
{
	u32 j = 0;
//	u8 repeat=nms/540;						//这里用540,是考虑到某些客户可能超频使用,
//											//比如超频到248M的时候,delay_xms最大只能延时541ms左右了
//	u16 remain=nms%540;
//	while(repeat)
//	{
//		delay_xms(540);
//		repeat--;
//	}
//	if(remain)delay_xms(remain);
//	Delay_ms(nms);
	for(j=0; j< nms;j++)
	{
		delay_us(1000);
	}
}

/**************************************************************************************************
**  函数名：  TimeInit()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	 无
**  作者：      
**  开发日期：
***************************************************************************************************/
void TimeInit(void)
{ 
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;	                       
	NVIC_InitTypeDef NVIC_InitStructure;	                       
  
 	/**************************************************************************
 	//     定时器7设置： 720分频，向上计数，计数值为1000->10ms	  
 	***************************************************************************/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	
	/* Enable the TIM6_IRQ Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	  
	
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_Prescaler = 839;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM7,ENABLE);	
}

/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/
















