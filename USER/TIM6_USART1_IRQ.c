/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : TIM6_USART1_IRQ.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f4xx.h"
#include "Usart.h"
#include "Manual.h"
#include "in.h"
#include "out.h"
#include "Auto.h"
#include "w25qxx.h"
#include "Error.h"
#include "Parameter.h"
#include "StatusControl.h"
#include "Auto_2.h"	
#include "ActionOperate.h"
#include "in.h"
#include "JDZ.h" 
#include "SpeedControl.h" 


//�Ѹ�
void TIM7_IRQHandler(void)
{//��ʱ����10msִ��һ��
	u8 j = 0;
//	u8 i = 0;
	
	if(Torque_T_count==65535)
		Torque_T_count = 0;
	else
		Torque_T_count++;
	
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) 
	{
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);		
		
		if(m_SystemTimeCounter < SYSTEM_TIME_MAX)  //ϵͳʱ��
		{
			m_SystemTimeCounter++;
		}
		else
		{
			m_SystemTimeCounter = 0;
		}
		
		/**-- �豸������ʱ���¼ --**/
		if(m_PowerOnTimeTotalCount < SYSTEM_TIME_MAX)
		{
			m_PowerOnTimeTotalCount++;
			m_PowerOnTimeTotal = m_PowerOnTimeTotalCount / 10;											//���ο���ʱ��
			m_PowerOnTimeCumulate = m_PrPowerOnTimeCumulate + m_PowerOnTimeTotal;    //�ܵ��ۼƿ���ʱ��=��ǰ�ۼ�ʱ��+���ο����ۼ�ʱ��
		}
		else
		{
			m_PowerOnTimeTotalCount = 0;
		}
		
		/**-- �ѻ�ͨ�Ŷ�ʱ --**/
		if(Communication_Time == 500)
		{//����5s
			if(OnLineCommunicated_Flag == FALSE)
			{//���û�н��ܵ�ͨ�Ź�
				OffLine_Flag = TRUE;//�ѻ���־λ��TRUE
			}
			Communication_Time++;
		}
		else if(Communication_Time<500 && Communication_Time>0)
		{
			Communication_Time++;
		}
		
		if(g_AutoStatue == AUTO_RUNNING)
		{
			Program_RunTime_Count++;
			m_ProRunTimeTotalCount++;
		}
		
		/**-- �ᱨ���źŶ�ʱ --**/
		if(Axsis_Error_Count > 0)
		{
			Axsis_Error_Count++;
			if(Axsis_Error_Count == 1500)
			{
				Axsis_Error_Permit_Flag = TRUE;
				Axsis_Error_Count = 0;
			}
		}
		
		/**-- ��������źŶ�ʱ --**/
		if(Input_Detect_Enable == DISABLE)
		{
			Input_Detect_Time++;
			if(Input_Detect_Time == 2)
			{//30ms���һ��
				Input_Detect_Time = 0;
				Input_Detect_Enable = ENABLE;
			}
		}
		
		/**-- ����ȫ�̼�⣬��ֹ����ʱ���޶��� --**/
		if(g_Auto_ActionRun_Timer > 0)
		{
			g_Auto_ActionRun_Timer++;
			if(g_Auto_ActionRun_Timer > 10000)		//����������һλС������100000��Ϊ10000������999.9ʱ�����޵ȴ�
			{
				g_Auto_ActionRun_Timer = 0;
			}
		}
		
		/**-- �ӳ�����ȫ�̼�⣬��ֹ����ʱ���޶��� --**/
		for(j=0; j<SAVEPROGRAMNUM_SUB; j++)
		{
			if(g_SubProgram_ActionRun_Timer[j] > 0)
			{
				g_SubProgram_ActionRun_Timer[j]++;
				if(g_SubProgram_ActionRun_Timer[j] > 6000)
				{
					g_SubProgram_ActionRun_Timer[j]=0;
				}
			}
		}
		
		/**-- �ŷ�����ʱ --**/
		if(Action_Done_Flag == TRUE)
		{
			Action_Delay_Time++;
			if(Action_Delay_Time == 40)
			{
				Action_Delay_Time = 0;
				Action_Done_Flag = FALSE;
				Action_Delay_Flag = TRUE;
			}
		}
		
		//����ÿ50ms��һ�ξ���ֵ����
		if(JDZ_Parameter.Switch==TRUE)
		{
			JDZ_ReadPosition_count++;
			if(JDZ_ReadPosition_count==5)
			{
				JDZ_ReadPosition_Flag = TRUE;
				JDZ_ReadPosition_count = 0;
			}
		}
		
    /**-- �����ź���ʱ1000ms --**/
//		if(Puls_Delay_Num > 0)
//		{
//			for(i=0; i<50; i++)
//			{
//				if(Puls_Delay_Enable[i])
//				{
//					Puls_Delay_Time[i]++;
//					if(Puls_Delay_Time[i] >= JXS_Parameter.PulseTime)
//					{//1000ms
//						SetOutput(i);
//						Puls_Delay_Enable[i] = DISABLE;
//						Puls_Delay_Time[i] = 0;
//						if(Puls_Delay_Num>0)
//						{
//							Puls_Delay_Num--;
//						}
//					}
//				}
//			}
//		}
		
		/**-- ȫ�Զ����м������� --**/
		if(g_AutoStatue == AUTO_RUNNING || g_Auto_Reset_Flag == TRUE)
		{
			if(g_ActionDelay_Step == 1)
			{
				g_ActionDelay_Timer ++;		//������ʱ��ʱ��
			}
			for(j=0; j<SAVEPROGRAMNUM_SUB; j++)
			{
				if(g_SubProgramDelay_Step[j] == 1)
				{
					g_SubProgramDelay_Timer[j]++;  //������ʱ��ʱ��
				}
			}
		}
		
		/**-- ��������ʱָ�����м������� --**/		  
		if(g_Key_Delay_Flag)
		{
			g_Key_Delay_Timer++;
		}
		
		/**-- �����������źű���ʱ����� --**/
		if(g_Auto_Valid_Flag == TRUE)
		{//��ʼ��ʱ
			g_Auto_Valid_Timer++;
			if(g_Auto_Valid_Timer > 100000)
			{
				g_Auto_Valid_Timer = 0;
			}
		}
		
		/**-- �ӳ������м������� --**/		  
		for(j=0; j<SAVEPROGRAMNUM_SUB; j++)
		{
			if(g_SubProgram_Key_Delay_Flag[j])
			{//��ʱָ��
				g_SubProgram_Key_Delay_Timer[j]++;
			}
			
			/**-- �����źű���ʱ����� --**/
			if(g_SubAuto_Valid_Flag[j] == TRUE)
			{//��ʼ��ʱ
				g_SubAuto_Valid_Timer[j]++;
				if(g_SubAuto_Valid_Timer[j] > 100000)
				{
					g_SubAuto_Valid_Timer[j] = 0;
				}
			}
		}
		
		/**-- �Զ�����ʱ����ͨ����ʱ�������� --**/
		if(Origin_Backed == TRUE)
		{
			g_USART_Delay_Timer++;	  //1000ms
			if(g_USART_Delay_Timer > 500)
			{
				g_USART_Delay_Timer = 0;
			}
		}
	}
}

/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearFlag(USART1, USART_IT_RXNE);
		ReceiveDataBuffer[ReceiveDataCounter] = USART_ReceiveData(USART1);

		if(NewOrder == FALSE)
		{
			if(StartReceiveDataFlag == FALSE)
			{
				if(ReceiveDataBuffer[ReceiveDataCounter] == USART_SEND_START)
				{//���յ���ʼ�ź�
					StartReceiveDataFlag = TRUE;
					USART1ErrorFlag = FALSE;
					ReceiveDataCounter = 0;
				}
				else 
				{
					ReceiveDataCounter = 0;
				}
			} 
			else
			{
				ReceiveDataCounter++;
				if(ReceiveDataCounter == ReceiveDataBuffer[0])
				{
					StartReceiveDataFlag = FALSE;		           									//�������
					if(ReceiveDataBuffer[ReceiveDataCounter-1] != USART_SEND_END)  
					{//���յ������һ�����ݲ��ǽ����źţ�������������ʧ��
						USART1ErrorFlag = TRUE;	               										//�����������ݳ���
						ReceiveDataCounter = 0;	
					}
					else  
					{
						USART1ErrorFlag = FALSE;                									//������������֡����
						ReceiveDataLen = ReceiveDataCounter;    									//��¼�������ݳ���
						ReceiveDataCounter = 0;
						OnLineCommunicated_Flag = TRUE;	   												//�ѻ�ͨ�Ÿ�-DPF
						NewOrder = TRUE;
					}
				}
				else if(ReceiveDataCounter >= USART_BUFFER_SIZE)
				{
					StartReceiveDataFlag = FALSE;																//�������
					ReceiveDataCounter = 0;	
				}
			}
		}
	}
	else	
	{//�������
		u16 temp = 0;
		USART1ErrorFlag = TRUE;	               														//�����������ݳ���
		ReceiveDataCounter = 0;
		temp = USART1->SR;
		temp = USART1->DR;
		(void)temp;
	}
}

//void PVD_IRQHandler(void)
//{
//  /* Clear the Key Button EXTI line pending bit */
//	EXTI_ClearITPendingBit(EXTI_Line16); 
//}


/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/

