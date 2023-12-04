#include "MotorInit.h"
#include "stmflash.h"

/**************************************************************************************************
**  ��������  MotorInit
**	�����������
**	�����������
**	�������ܣ������س�ʼ��
**	��ע��	  ��
**  ���ߣ�         
**  �������ڣ�
***************************************************************************************************/
void MotorInit(void)
{
//	MotorIOInit();
//	MotorNVICInit();
//	PulseInit();
}

/**************************************************************************************************
**  ��������  MotorIOInit
**	�����������
**	�����������
**	�������ܣ�������IO��ʼ��
**	��ע��	  ��
**  ���ߣ�         
**  �������ڣ��Ѹ�
***************************************************************************************************/
void MotorIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructureOut;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM10, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE); 

	/*----GPIOA_5 ���ʹ�� ����˿�����----*/
	GPIO_InitStructureOut.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructureOut.GPIO_Mode = GPIO_Mode_OUT;        //���ù���
	GPIO_InitStructureOut.GPIO_Speed = GPIO_Speed_100MHz;		//�ٶ�100MHz
	GPIO_InitStructureOut.GPIO_OType = GPIO_OType_PP;       //���츴�����
	GPIO_InitStructureOut.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructureOut);

	/******************������ƽӿ�***************************/

	/*----����SIGN ����˿�����----*/
	GPIO_InitStructureOut.GPIO_Pin = GPIO_Pin_6;	//PA6:X-SIGN
	GPIO_Init(GPIOA, &GPIO_InitStructureOut); 

	GPIO_InitStructureOut.GPIO_Pin = GPIO_Pin_13;	//PE13:Y-SIGN
	GPIO_Init(GPIOE, &GPIO_InitStructureOut);

	GPIO_InitStructureOut.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_9;	//PB7:Z-SIGN	PB9:O-SIGN
	GPIO_Init(GPIOB, &GPIO_InitStructureOut); 

	/*---- ����PULS ����˿�����----*/
	GPIO_InitStructureOut.GPIO_Mode = GPIO_Mode_AF;

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	GPIO_InitStructureOut.GPIO_Pin = GPIO_Pin_7; 	//PA7:X-PULS  TIM3-CH2
	GPIO_Init(GPIOA, &GPIO_InitStructureOut);

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);
	GPIO_InitStructureOut.GPIO_Pin = GPIO_Pin_14; //PE14:Y-PULS  TIM1-CH4
	GPIO_Init(GPIOE, &GPIO_InitStructureOut);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
	GPIO_InitStructureOut.GPIO_Pin = GPIO_Pin_6; 	//PB6:Z-PULS  TIM4-CH1
	GPIO_Init(GPIOB, &GPIO_InitStructureOut);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM10);
	GPIO_InitStructureOut.GPIO_Pin = GPIO_Pin_8; 	//PB8:O-PULS  TIM10-CH1
	GPIO_Init(GPIOB, &GPIO_InitStructureOut);
}

/**************************************************************************************************
**  ��������  MotorNVICInit
**	�����������
**	�����������
**	�������ܣ�������NVIC��ʼ��
**	��ע��	  ��
**  ���ߣ�         
**  �������ڣ��Ѹ�
***************************************************************************************************/
void MotorNVICInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable the TIM3_IRQ Interrupt */    									//X
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);		
	
	/* Enable the TIM1_UP_TIM10_IRQn Interrupt */	     				//Y
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;				
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
		
	/* Enable the TIM4_IRQn Interrupt */    									//Z
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	   
	
	/* Enable the TIM1_UP_TIM10_IRQn Interrupt */	 						//O
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**************************************************************************************************
**  ��������  PulseInit
**	�����������
**	�����������
**	�������ܣ���ʼ������
**	��ע��	  ��
**  ���ߣ�         
**  �������ڣ��Ѹ�
***************************************************************************************************/
void PulseInit(void)
{
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;	

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
	TIM_OCStructInit(&TIM_OCInitStructure); 

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;									//ʱ�ӷָ�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period = 5000 * 3 - 1;							//д��TIMx_ARR��ֵ��3M/15000=200hz
	TIM_TimeBaseStructure.TIM_Prescaler = 28 - 1;                	//84M / 28 = 3M,��ʱ��2~7,12~14��ʱ��Դ��APB1

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;             //�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 5000 * 3;                     //��װ�벶׽�ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //TIM����Ƚϼ��Ը�
	
	TIM_DeInit(TIM3);                                             //TIM3-CH2��X
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	
	TIM_DeInit(TIM4);                                             //TIM4-CH1,Z
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update,ENABLE);
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 56 - 1;                	//168M / 56 = 3M����ʱ��9~11��ʱ��Դ��APB2
	
	TIM_DeInit(TIM10);                                            //TIM10-CH1,O
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);
	TIM_ClearITPendingBit(TIM10,TIM_IT_Update);
	TIM_ITConfig(TIM10, TIM_IT_Update,ENABLE);
	TIM_OC1Init(TIM10, &TIM_OCInitStructure);

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;                  //ʱ�ӷָ�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	  //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period = 5000 * 3 - 1;       		  	//д��TIMx_ARR��ֵ
	TIM_TimeBaseStructure.TIM_Prescaler = 56 - 1;                	//168M / 56 = 3M����ʱ��1��8��ʱ��Դ��APB2
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;             //�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 5000 * 3;              		  	//��װ�벶׽�ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //TIM����Ƚϼ��Ը�

	TIM_DeInit(TIM1);                                           	//TIM1-CH4,Y
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_SelectOnePulseMode(TIM3, TIM_OPMode_Repetitive);
	TIM_SelectOnePulseMode(TIM1, TIM_OPMode_Repetitive);
	TIM_SelectOnePulseMode(TIM4, TIM_OPMode_Repetitive);
	TIM_SelectOnePulseMode(TIM10, TIM_OPMode_Repetitive);

	TIM_Cmd(TIM3, DISABLE);		//X
	TIM_Cmd(TIM1, DISABLE);		//Y
	TIM_Cmd(TIM4, DISABLE);		//Z
	TIM_Cmd(TIM10, DISABLE);	//O
}

/*�������ʼ��*/
void PVD_Init(void)
{ 
	NVIC_InitTypeDef NVIC_InitStruct;
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_PWR, ENABLE);
	PWR_PVDLevelConfig(PWR_CR_PLS_LEV7);//2.5V
	PWR_PVDCmd(ENABLE);
	
	NVIC_InitStruct.NVIC_IRQChannel = PVD_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);//�����ж�
	
	EXTI_ClearITPendingBit(EXTI_Line16);
	EXTI_InitStructure.EXTI_Line = EXTI_Line16;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}


void PVD_IRQHandler(void)
{
	if(PWR_GetFlagStatus(PWR_FLAG_PVDO)) //�����������ж�
	{
		if(STMFLASH_AllowOffPowerWrite == 1)
		{
			STMFLASH_WriteOffPowerData();
		}
		while(1);
	}

	EXTI_ClearITPendingBit(EXTI_Line16); //������if���档
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/




