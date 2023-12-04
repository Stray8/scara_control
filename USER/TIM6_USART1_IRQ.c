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


//已改
void TIM7_IRQHandler(void)
{//定时器，10ms执行一次
	u8 j = 0;
//	u8 i = 0;
	
	if(Torque_T_count==65535)
		Torque_T_count = 0;
	else
		Torque_T_count++;
	
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) 
	{
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);		
		
		if(m_SystemTimeCounter < SYSTEM_TIME_MAX)  //系统时钟
		{
			m_SystemTimeCounter++;
		}
		else
		{
			m_SystemTimeCounter = 0;
		}
		
		/**-- 设备开机总时间记录 --**/
		if(m_PowerOnTimeTotalCount < SYSTEM_TIME_MAX)
		{
			m_PowerOnTimeTotalCount++;
			m_PowerOnTimeTotal = m_PowerOnTimeTotalCount / 10;											//本次开机时间
			m_PowerOnTimeCumulate = m_PrPowerOnTimeCumulate + m_PowerOnTimeTotal;    //总的累计开机时间=此前累计时间+本次开机累计时间
		}
		else
		{
			m_PowerOnTimeTotalCount = 0;
		}
		
		/**-- 脱机通信定时 --**/
		if(Communication_Time == 500)
		{//超过5s
			if(OnLineCommunicated_Flag == FALSE)
			{//如果没有接受到通信过
				OffLine_Flag = TRUE;//脱机标志位置TRUE
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
		
		/**-- 轴报警信号定时 --**/
		if(Axsis_Error_Count > 0)
		{
			Axsis_Error_Count++;
			if(Axsis_Error_Count == 1500)
			{
				Axsis_Error_Permit_Flag = TRUE;
				Axsis_Error_Count = 0;
			}
		}
		
		/**-- 检测输入信号定时 --**/
		if(Input_Detect_Enable == DISABLE)
		{
			Input_Detect_Time++;
			if(Input_Detect_Time == 2)
			{//30ms检测一次
				Input_Detect_Time = 0;
				Input_Detect_Enable = ENABLE;
			}
		}
		
		/**-- 动作全程监测，防止过长时间无动作 --**/
		if(g_Auto_ActionRun_Timer > 0)
		{
			g_Auto_ActionRun_Timer++;
			if(g_Auto_ActionRun_Timer > 10000)		//输入检测增加一位小数，由100000改为10000，输入999.9时，无限等待
			{
				g_Auto_ActionRun_Timer = 0;
			}
		}
		
		/**-- 子程序动作全程监测，防止过长时间无动作 --**/
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
		
		/**-- 伺服器延时 --**/
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
		
		//开机每50ms读一次绝对值数据
		if(JDZ_Parameter.Switch==TRUE)
		{
			JDZ_ReadPosition_count++;
			if(JDZ_ReadPosition_count==5)
			{
				JDZ_ReadPosition_Flag = TRUE;
				JDZ_ReadPosition_count = 0;
			}
		}
		
    /**-- 脉冲信号延时1000ms --**/
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
		
		/**-- 全自动运行计数参数 --**/
		if(g_AutoStatue == AUTO_RUNNING || g_Auto_Reset_Flag == TRUE)
		{
			if(g_ActionDelay_Step == 1)
			{
				g_ActionDelay_Timer ++;		//动作延时计时器
			}
			for(j=0; j<SAVEPROGRAMNUM_SUB; j++)
			{
				if(g_SubProgramDelay_Step[j] == 1)
				{
					g_SubProgramDelay_Timer[j]++;  //动作延时计时器
				}
			}
		}
		
		/**-- 主程序延时指令运行计数参数 --**/		  
		if(g_Key_Delay_Flag)
		{
			g_Key_Delay_Timer++;
		}
		
		/**-- 主程序输入信号保持时间计数 --**/
		if(g_Auto_Valid_Flag == TRUE)
		{//开始计时
			g_Auto_Valid_Timer++;
			if(g_Auto_Valid_Timer > 100000)
			{
				g_Auto_Valid_Timer = 0;
			}
		}
		
		/**-- 子程序运行计数参数 --**/		  
		for(j=0; j<SAVEPROGRAMNUM_SUB; j++)
		{
			if(g_SubProgram_Key_Delay_Flag[j])
			{//延时指令
				g_SubProgram_Key_Delay_Timer[j]++;
			}
			
			/**-- 输入信号保持时间计数 --**/
			if(g_SubAuto_Valid_Flag[j] == TRUE)
			{//开始计时
				g_SubAuto_Valid_Timer[j]++;
				if(g_SubAuto_Valid_Timer[j] > 100000)
				{
					g_SubAuto_Valid_Timer[j] = 0;
				}
			}
		}
		
		/**-- 自动运行时串口通信延时计数参数 --**/
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
				{//接收到开始信号
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
					StartReceiveDataFlag = FALSE;		           									//接受完成
					if(ReceiveDataBuffer[ReceiveDataCounter-1] != USART_SEND_END)  
					{//接收到的最后一个数据不是结束信号，表明接收数据失败
						USART1ErrorFlag = TRUE;	               										//表明接收数据出错
						ReceiveDataCounter = 0;	
					}
					else  
					{
						USART1ErrorFlag = FALSE;                									//表明接收数据帧无误
						ReceiveDataLen = ReceiveDataCounter;    									//记录接收数据长度
						ReceiveDataCounter = 0;
						OnLineCommunicated_Flag = TRUE;	   												//脱机通信改-DPF
						NewOrder = TRUE;
					}
				}
				else if(ReceiveDataCounter >= USART_BUFFER_SIZE)
				{
					StartReceiveDataFlag = FALSE;																//接受完成
					ReceiveDataCounter = 0;	
				}
			}
		}
	}
	else	
	{//数据溢出
		u16 temp = 0;
		USART1ErrorFlag = TRUE;	               														//表明接收数据出错
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

