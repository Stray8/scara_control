/*************** (C) COPYRIGHT 2012 Kingrobot manipulator Team ****************
* File Name          : in.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This file provides all the GPIO input functions.
******************************************************************************/
#include "in.h"
#include "out.h"
#include "Delay.h"
#include "Auto.h"
#include "StatusControl.h"
#include "Parameter.h"
#include "Auto_2.h"
#include "Error.h"

u8 Input_Detect_Status[6] = {0, 0, 0, 0, 0, 0};
/**************************************************************************************************
**  ��������  InputInit()
**	�����������
**	���������
**	�������ܣ�����ڳ�ʼ��
**	��ע��	 	
**  ���ߣ�    
**  �������ڣ��Ѹ�
***************************************************************************************************/
void InputInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructureIn;

//	EXTI_InitTypeDef EXTI_InitStructure;
// 	NVIC_InitTypeDef NVIC_InitStructure; 
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA, ENABLE);
	
	/***************�ź�����˿�����(�����ⲿ�ж�)*****************/
	GPIO_InitStructureIn.GPIO_Mode = GPIO_Mode_IN;					//��ͨ����ģʽ
	GPIO_InitStructureIn.GPIO_Speed = GPIO_Speed_100MHz;		//100M
	GPIO_InitStructureIn.GPIO_PuPd = GPIO_PuPd_UP;					//����	
	
	//PA��
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_6;
  GPIO_Init(GPIOA,&GPIO_InitStructureIn);
	
	//PB��
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_10|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_Init(GPIOB,&GPIO_InitStructureIn);

	//PD��
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
  GPIO_Init(GPIOD,&GPIO_InitStructureIn);

	//PE��
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_Init(GPIOE,&GPIO_InitStructureIn);
	
	/*---- ��������ź� ����˿�����----*/
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructureIn);

	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOE, &GPIO_InitStructureIn);
	
	/*---- ��ͣ�����ź� ����˿�����----*/
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOD, &GPIO_InitStructureIn);
	
//	/***************Z���ײ����*****************/
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);							//ʹ��SYSCFGʱ��
//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource13);			//PE13-X7-Z���ײ
//	
//	//PE13  Z���ײ
//	EXTI_InitStructure.EXTI_Line = EXTI_Line13;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;  
//	EXTI_Init(&EXTI_InitStructure); 
//	
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
}

/**************************************************************************************************
**  ��������  ReadEmergencyStop()
**	�����������
**	���������u8 (0/1)
**	�������ܣ���ȡ��ͣ�˿��źţ������Ƿ��¼�ͣ
**	��ע��	 	
**  ���ߣ�    
**  �������ڣ��Ѹ�
***************************************************************************************************/
u8 ReadEmergencyStop(void)
{
	u8 Result = 0;								//Ĭ�ϲ�����
	static u16 counter = 0;				//������������

	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_15) == 0)
	{//��ͣ�˿��ź�
		//delay_ms(10);
		counter++;
		if(counter > 2)
		{
			Result = 1;
		}
	}
	else
	{
		counter = 0;
	}
	
	return Result;
}

/**************************************************************************************************
**  ��������  ReadIOPort()
**	���������GPIOx�˿� PortNum�����Ϊ�˿ںű�־ PortPreviousStatus�ö˿�֮ǰ��״̬	
**	���������u8 (0/1)
**	�������ܣ���ȡĳ��������ź� ���� 1���ź� 0���ź�
**	��ע��	  Ӳ����·�ǵ���Ч��ͬʱ���˿ڶ�ȡ���͵�ƽΪ������ź�
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u8 ReadInputPort(GPIO_TypeDef* GPIOx, u16 PortNum, u8 PortPreviousStatus)
{
	if(PortPreviousStatus == 0x00)
	{
		if(GPIO_ReadInputDataBit(GPIOx, PortNum) == 0)
		{
			delay_us(200);
			if(GPIO_ReadInputDataBit(GPIOx, PortNum) == 0)
			{
				return 0;
			}
		}
		return 1;
	}
	else
	{
		if(GPIO_ReadInputDataBit(GPIOx, PortNum) != 0)
		{
			delay_us(200);
			if(GPIO_ReadInputDataBit(GPIOx, PortNum) != 0)
			{	
				return 1;
			}
		}
		return 0;
	}
}

/**************************************************************************************************
**  ��������  ReadInput()
**	���������u8 IO_Num:��Ҫ��ȡ�Ķ˿�
**	���������u8 (0/1)
**	�������ܣ���ȡ����˿��źţ����� 1���ź� 0���ź�
**	��ע��	  
**  ���ߣ�    
**  �������ڣ��Ѹ�
***************************************************************************************************/
u8 ReadInput(u8 IO_Num)
{
	u8 result = FALSE;
	switch (IO_Num)
	{
		case 0://X0
			result = ReadInputPort(X0_IN_PORT, X0_IN_PORT_NUM, X0_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X0_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{				
				X0_IN_PORT_LOW_LEVEL_SET;
			}
			break;

		case 1://X1
			result = ReadInputPort(X1_IN_PORT, X1_IN_PORT_NUM, X1_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X1_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X1_IN_PORT_LOW_LEVEL_SET;
			}
			break; 

		case 2://X2
			result = ReadInputPort(X2_IN_PORT, X2_IN_PORT_NUM, X2_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X2_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X2_IN_PORT_LOW_LEVEL_SET;
			}
			break; 

		case 3://X3
			result = ReadInputPort(X3_IN_PORT, X3_IN_PORT_NUM, X3_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X3_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X3_IN_PORT_LOW_LEVEL_SET;
			}
			break;

		case 4://X4
			result = ReadInputPort(X4_IN_PORT, X4_IN_PORT_NUM, X4_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X4_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X4_IN_PORT_LOW_LEVEL_SET;
			}
			break;

		case 5://X5
			result = ReadInputPort(X5_IN_PORT, X5_IN_PORT_NUM, X5_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X5_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X5_IN_PORT_LOW_LEVEL_SET;
			}
			break; 

		case 6://X6
			result = ReadInputPort(X6_IN_PORT, X6_IN_PORT_NUM, X6_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X6_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X6_IN_PORT_LOW_LEVEL_SET;     
			}
			break;

		case 7://X7
			result = ReadInputPort(X7_IN_PORT, X7_IN_PORT_NUM, X7_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X7_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X7_IN_PORT_LOW_LEVEL_SET;
			}
			break;


		case 8://X8
			result = ReadInputPort(X8_IN_PORT, X8_IN_PORT_NUM, X8_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X8_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X8_IN_PORT_LOW_LEVEL_SET;
			}
			break;

		case 9://X9
			result = ReadInputPort(X9_IN_PORT, X9_IN_PORT_NUM, X9_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X9_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X9_IN_PORT_LOW_LEVEL_SET;
			}
			break; 

		case 10://X10
			result = ReadInputPort(X10_IN_PORT, X10_IN_PORT_NUM, X10_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X10_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X10_IN_PORT_LOW_LEVEL_SET;
			}
			break; 

		case 11://X11
			result = ReadInputPort(X11_IN_PORT, X11_IN_PORT_NUM, X11_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X11_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X11_IN_PORT_LOW_LEVEL_SET;
			}
			break;

		case 12://X12
			result = ReadInputPort(X12_IN_PORT, X12_IN_PORT_NUM, X12_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X12_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X12_IN_PORT_LOW_LEVEL_SET;
			}
			break;

		case 13://X13
			result = ReadInputPort(X13_IN_PORT, X13_IN_PORT_NUM, X13_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X13_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X13_IN_PORT_LOW_LEVEL_SET;
			}
			break; 

		case 14://X14
			result = ReadInputPort(X14_IN_PORT, X14_IN_PORT_NUM, X14_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X14_IN_PORT_HIGH_LEVEL_RESET;			
			}
			else
			{
				X14_IN_PORT_LOW_LEVEL_SET;
			}
			break;

		case 15://X15
			result = ReadInputPort(X15_IN_PORT, X15_IN_PORT_NUM, X15_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X15_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X15_IN_PORT_LOW_LEVEL_SET;
			}
			break;

		case 16://X16
			result = ReadInputPort(X16_IN_PORT, X16_IN_PORT_NUM, X16_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X16_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X16_IN_PORT_LOW_LEVEL_SET;
			}
			break;

		case 17://X17
			result = ReadInputPort(X17_IN_PORT, X17_IN_PORT_NUM, X17_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X17_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X17_IN_PORT_LOW_LEVEL_SET;
			}
			break; 

		case 18://X18
			result = ReadInputPort(X18_IN_PORT, X18_IN_PORT_NUM, X18_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X18_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X18_IN_PORT_LOW_LEVEL_SET;
			}
			break;

		case 19://X19
			result = ReadInputPort(X19_IN_PORT, X19_IN_PORT_NUM, X19_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X19_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X19_IN_PORT_LOW_LEVEL_SET;
			}
			break;

		case 20://X20
			result = ReadInputPort(X20_IN_PORT, X20_IN_PORT_NUM, X20_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X20_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X20_IN_PORT_LOW_LEVEL_SET;
			}
			break;

		case 21://X21
			result = ReadInputPort(X21_IN_PORT, X21_IN_PORT_NUM, X21_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X21_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X21_IN_PORT_LOW_LEVEL_SET;
			}
			break; 

		case 22://X22
			result = ReadInputPort(X22_IN_PORT, X22_IN_PORT_NUM, X22_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X22_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X22_IN_PORT_LOW_LEVEL_SET;
			}
			break;

		case 23://X23
			result = ReadInputPort(X23_IN_PORT, X23_IN_PORT_NUM, X23_IN_PORT_LOW_LEVEL);
			if(result)
			{
				X23_IN_PORT_HIGH_LEVEL_RESET;
			}
			else
			{
				X23_IN_PORT_LOW_LEVEL_SET;
			}
			break;
			
		default:
			break;			
	}
	return (result);
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/


