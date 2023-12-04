/*************** (C) COPYRIGHT 2012 Kingrobot manipulator Team ****************
* File Name          : Delay.c
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 20/06/2012
* Description        : This file delimits all the delay functions.
******************************************************************************/
#include "Delay.h"

//static uint8_t  fac_us=0;							//us��ʱ������			   
//static uint16_t fac_ms=0;							//ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��

/************************************************************************************************************
**	�����������
**	�����������
**	�������ܣ���ʱ������ʼ��
*************************************************************************************************************/
void delay_init(u8 SYSCLK)
{
//	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8
//	
//	fac_us=SYSCLK/8;							//Ϊϵͳʱ�ӵ�1/8  
//	fac_ms=(u16)fac_us*1000;					//��OS��,����ÿ��ms��Ҫ��systickʱ����   
}

/************************************************************************************************************
**	���������nus  ��ʱʱ������λus
**	�����������
**	�������ܣ���ʱ����
*************************************************************************************************************/
void delay_us(u32 nus)
{
	u32 i = 0;
//	u32 temp;	    	 
//	SysTick->LOAD=nus*fac_us; 					//ʱ�����	  		 
//	SysTick->VAL=0x00;        					//��ռ�����
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����	  
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
//	SysTick->VAL =0X00;      					 //��ռ�����	 
//	Delay_us(nus);
	for(i=0; i< nus*30;i++)
	{}
}


void delay_xms(u16 nms)
{	 		  	  
//	u32 temp;		   
//	SysTick->LOAD=(u32)nms*fac_ms;			//ʱ�����(SysTick->LOADΪ24bit)
//	SysTick->VAL =0x00;           			//��ռ�����
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ���� 
//	do
//	{
//		temp=SysTick->CTRL;
//	}while((temp&0x01)&&!(temp&(1<<16)));	//�ȴ�ʱ�䵽��   
//	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
//	SysTick->VAL =0X00;     		  		//��ռ�����	  	    
} 

/************************************************************************************************************
**	���������nms  ��ʱʱ������λms
**	�����������
**	�������ܣ���ʱ����
*************************************************************************************************************/
void delay_ms(u32 nms)
{
	u32 j = 0;
//	u8 repeat=nms/540;						//������540,�ǿ��ǵ�ĳЩ�ͻ����ܳ�Ƶʹ��,
//											//���糬Ƶ��248M��ʱ��,delay_xms���ֻ����ʱ541ms������
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
**  ��������  TimeInit()
**	���������
**	�����������
**	�������ܣ�
**	��ע��	 ��
**  ���ߣ�      
**  �������ڣ�
***************************************************************************************************/
void TimeInit(void)
{ 
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;	                       
	NVIC_InitTypeDef NVIC_InitStructure;	                       
  
 	/**************************************************************************
 	//     ��ʱ��7���ã� 720��Ƶ�����ϼ���������ֵΪ1000->10ms	  
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
















