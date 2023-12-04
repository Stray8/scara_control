/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Auto.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f4xx.h"
#include "Usart.h" 
#include "Auto.h"
#include "StatusControl.h"
#include "SpeedControl.h"
#include "in.h"
#include "out.h"
#include "Manual.h"
#include "Parameter.h"
#include "Error.h"
#include "SignalWatch.h"
#include "Delay.h"
#include "w25qxx.h"
#include "Auto_2.h"
#include "ActionOperate.h"
#include "JDZ.h"
#include "BackToOrigin.h"
#include "CANopen.h"


/**-- �Զ�ģʽ�µĸ�ģʽ��־λ --**/
u32 Auto_Pulse_Count = 0;        						//�Զ�ģʽÿ�����������������
u8 Auto_Mode = 	LOOP_MODE;	     						//�Զ�ģʽѡ��:1-���Σ�3-ѭ��
u8 Loop_Mode_Enable = DISABLE;	 						//ѭ��ģʽʹ��
u8 Once_Mode_Enable = DISABLE;	 						//����ģʽʹ��
u8 Single_Mode_Enable = DISABLE; 						//����ģʽʹ��

u8  Auto_Reset_Step = 0;
u8  Auto_Reset_Step_Done = FALSE;

u32 Present_Position = MINROBOTPOSITION;	 	//��ǰλ��ֵ
u32 Action_Delay_Time = 0;          				//��е��ÿ�ζ������֮�����ʱ����Ϊ�ŷ��������Ǹ��涯��
//u8 	Puls_Delay_Time[50] = {0};		  				//�����ź���ʱ
//u8  Puls_Delay_Enable[50] = {DISABLE};  		//�����źű�ע
//u8  Puls_Delay_Num = 0;			        				//�����źű�ע
u8  Action_Done_Flag =	FALSE;		  				//�ŷ���������ɱ�־
u8  Action_Delay_Flag	=	FALSE;		  				//������ʱ���

u8  g_Robot_Has_Debuged_Flag = FALSE;	 			//��е��֮ǰ״̬�Ƿ������ɱ�̵���״̬
u8  g_Program_Is_Debuging = FALSE;					//��е���Ƿ��ڵ���ģʽ


/**************************************************************************************************
**  ��������  AutoReset()
**	�����������
**	�����������
**	�������ܣ��Զ��µı�־λ����
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void AutoReset()
{
	u16 i = 0;
	
	Back_Origin_Flag = FALSE;
	Origin_Backed = FALSE;
	Robot_Auto_Reset = FALSE;
	Once_Mode_Enable = DISABLE;
	Single_Mode_Enable = DISABLE;
	Loop_Mode_Enable = DISABLE;
	Jog_Move_Enable = DISABLE;
	Linked_Move_Enable = DISABLE;
	g_AutoStatue = AUTO_WAITE;
	Work_Status = WAIT_MODE;
	g_Auto_ActionError_Flag = FALSE;
	g_Auto_ActionTimeOut_Flag = FALSE;
	MD_PositionErr_Flag = FALSE;
	g_Auto_ActionRun_Timer = 0;
	g_Auto_ActionRun_Step = 0;
	g_Auto_PresentLine = 0;
	Action_Step_List_Num = 0;
	Action_Step_Run_Num = 0;
	Action_Step_Confirm_Num = 0;
	g_Auto_Valid_Timer = 0;
	g_Auto_Valid_Flag = FALSE;
	g_USART_Delay_Timer = 0;
	g_Program_Is_Debuging = FALSE;
	Auto_Reset_Step_Done = FALSE;
	Auto_Reset_Step = 0;
	g_Auto_Reset_Flag = FALSE;	
	g_Auto_Order_Pause = FALSE;
	g_Auto_Order_Stop = FALSE;
	g_ActionDelay_Step = 0;
	
	Axis_Machine_Origin_Flag = FALSE;
	Homing_Flag_Can = FALSE;
	
	for(i=0; i<SAVEPROGRAMNUM_SUB; i++)
	{
		g_SubProgram_ActionRun_Timer[i] = 0;
		g_SubProgram_ActionRun_Step[i] = 0;
		g_SubProgram_PresentLine[i] = 0;
		g_Read_SubProgram[i] = FALSE;
		g_SubProgramDelay_Step[i] = 0;

		g_SubProgram_Step_Run[i] = FALSE;
		g_SubProgram_Start[i] = FALSE;
		g_SubProgram_Finish[i] = FALSE;
		
		SubAction_Step_List_Num[i] = 0;
		SubAction_Step_Run_Num[i] = 0;
		SubAction_Step_Confirm_Num[i] = 0;
		g_SubAuto_Valid_Timer[i] = 0;
		g_SubAuto_Valid_Flag[i] = FALSE;
	}
	
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		Flag_Keep_Move[i] = 0;
		Increment_Finished[i] = FALSE;
		Program_Axis_Origin_Flag[i] = FALSE;
		SlowPointFlag[i] = 0;
	}
	
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		Axsis_MoveProNum[i] = 0;
		m_InterpAxisFlag[i] = 0;
	}
	m_InterpLenAxis = 0xff;
	
	g_Auto_ActionNcWait_Flag = 0;
	for(i=0; i<SAVEPROGRAMNUM_SUB; i++)
	{
		g_Auto_SubActionNcWait_Flag[i] = 0;
	}
}

/**************************************************************************************************
**  ��������  SafeAreaJudge()
**	�����������
**	�����������
**	�������ܣ��Զ�����ʱ��ȫ�����ж�
**	��ע��	  ��
**  ���ߣ�      
**  �������ڣ�
***************************************************************************************************/
void SafeAreaJudge(void)
{
 	u16 i = 0;
	
	if(Origin_Backed == TRUE)
	{//�������ܽ��밲ȫ����������
		for(i=0; i<SAVESAFEAREA; i++)
		{//ȷ����ǰX�����ĸ���ȫ����
			if(Robot_Safe_Area[i].SafeArea_Switch)
			{
				if((m_PulseTotalCounter[X_Axsis] >= Robot_Safe_Area[i].X_Left) && (m_PulseTotalCounter[X_Axsis] <= Robot_Safe_Area[i].X_Right))
				{//��ǰX�ᴦ�ڰ�ȫ����Χ��
					if(AxisMoveFlag[Z_Axsis] == 1 && Axsis_Move_Direction[Z_Axsis] == POSITIVE && \
							m_PulseTotalCounter[Z_Axsis] > Robot_Safe_Area[i].Z_Down)
					{//���Z�� ���ڰ�ȫ��Z����λ��	�� С�ڰ�ȫ��Z����λ�ã���ô��Ҫ���а�ȫ��������
						Robot_Error_Data[0] = Robot_Error_Data[0] | 0x02;
						CloseTotalMotorError();
						break;
					}
				}
			}
		}
	}
}

/**************************************************************************************************
**  ��������  SetSingle()
**	���������Reset_Output��Set_Output  ��λ����λIO�ڣ�60��ʾ��Ч
**	�����������
**	�������ܣ��ź����������λ��־λ
**	��ע��	  
**  ���ߣ�        
**  �������ڣ�
***************************************************************************************************/
void SetSingle(u8 Reset_Output,u8 Set_Output, u32 Detect_Flag)
{
	//�ź��������λ , ��λ , �����ź�
	if(Reset_Output == 60)
	{ //���ź�-��λĳIO
	}
	else
	{
		SetOutput(Reset_Output);
	}
	
	if(Set_Output ==60)
	{ //���ź�-��λĳIO
	}
	else
	{
		ResetOutput(Set_Output);
//		if(Detect_Flag == 15)
//		{
//			Puls_Delay_Num++;
//			Puls_Delay_Enable[Set_Output] = ENABLE;	   //��λ��־λ
//		}
	}
}

/**************************************************************************************************
**  ��������  Read_SaveProgram_IIC_Address()
**	�����������
**	�����������
**	�������ܣ�����ѡ�еĳ���Ż�ȡ�����ַ�ͳ�������
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void Read_SaveProgram_IIC_Address()
{
	u8 i = 0;

	SaveProgram_IIC_Address = 0;
	SaveProgram_IIC_Num = 0;
	
	if(g_Run_Program_Num > 0)
	{
		if(g_Run_Program_Num > SAVEPROGRAMNUM_MAIN)
		{
			SaveProgram_IIC_Address = Program_IIC_Address[g_Run_Program_Num-1].Address;
			SaveProgram_IIC_Num = Program_IIC_Address[g_Run_Program_Num-1].Num;
		}
		else
		{
			for(i=0; i<SAVEPROGRAMNUM_MAIN; i++)
			{//��ѯ������뵱ǰ�趨�Զ����б����ͬ����
				if(Program_IIC_Address[i].Code == g_Run_Program_Num)
				{
					SaveProgram_IIC_Address = Program_IIC_Address[i].Address;
					SaveProgram_IIC_Num = Program_IIC_Address[i].Num;
					break;
				}
			}
		}
	}
}

/**************************************************************************************************
**  ��������  AutoRun()
**	�����������
**	�����������
**	�������ܣ��Զ�ģʽ�µĸ��ֲ���
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void AutoRun()
{	
	switch(UsartReceiveData[1])
	{
		case P_ONCE_MODE:
			Auto_Mode = ONCE_MODE;
			Once_Mode_Enable = ENABLE;
			Loop_Mode_Enable = DISABLE;
			break;
		
		case P_CYCLE_MODE://ѭ��״̬
			Auto_Mode = LOOP_MODE;
			Loop_Mode_Enable = ENABLE;		//״̬��λ
			Once_Mode_Enable = DISABLE;
			break;	

		case P_SINGLE_MODE://����ģʽ
			if(UsartReceiveData[2] == 0)
			{//�ر�
				Single_Mode_Enable = DISABLE;
			}
			else
			{//����
				Single_Mode_Enable = ENABLE;
			}
			break;	

		case P_ACTION_RUN://����״̬
			if(Not_Get_Position() ==1 && JDZ_Parameter.Switch == 1)
			{
				return;
			}
			else
			{
				if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
				{
					g_Auto_Order_Start = TRUE;
					if(Auto_Mode == ONCE_MODE)
					{
						Once_Mode_Enable = ENABLE;
						Loop_Mode_Enable = DISABLE;
					}
					else if(Auto_Mode == LOOP_MODE)
					{
						Loop_Mode_Enable = ENABLE;
						Once_Mode_Enable = DISABLE;
					}
					g_Auto_Order_Pause = FALSE;
					g_Auto_Order_Stop = FALSE;
				}
			}
			
			break;

		case P_ACTION_PAUSE://��ͣ
			if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
			{
				g_Auto_Order_Pause = TRUE;
			}
			break;

		case P_ACTION_STOP://ֹͣ
			if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
			{
				g_Auto_Order_Stop = TRUE;
			}
			break;

		case P_ACTION_PROGRAM://��ȡҪ���еĳ���
			CurProgramRead(UsartReceiveData[2]);
			break;

		case P_ACTION_RESET:  //��е�ָ�λ			 
			Work_Status = AUTO_WORK_MODE;
			g_Auto_Reset_Flag = TRUE;
			Robot_Auto_Reset = FALSE;
			g_Auto_LOrigin_Flag = FALSE;
			break;

		case P_ACTION_DEBUG:  //��е�ֵ���
			g_Auto_Order_Pause = FALSE;
			g_Auto_Order_Stop = FALSE;			
			Work_Status = AUTO_WORK_MODE;		
			Single_Mode_Enable = ENABLE;
			g_Auto_PresentLine = UsartReceiveData[2];	  //��ǰ��������
			g_Auto_ActionRun_Step = 0;
			Robot_Auto_Reset = TRUE;
			g_Auto_Order_Start = TRUE;
			g_Robot_Has_Debuged_Flag = TRUE;
			g_Program_Is_Debuging = TRUE;
			break;

		case P_ACTION_LORIGIN:  //��е��L����			 
			g_Auto_LOrigin_Flag = TRUE;
			g_Auto_Reset_Flag = TRUE;
			Robot_Auto_Reset = FALSE;
			break;
		case P_ACTION_COMMUTE:	//��е���°๦��
			break;
		default:
			break;
	}
}

/**************************************************************************************************
**  ��������  AutoRunning()
**	�����������
**	�����������
**	�������ܣ��Զ�ģʽ�µĸ�ģʽ����
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
//void AutoRunning()
//{
//	switch(Auto_Mode)
//	{
//		case ONCE_MODE:		//����ģʽ	
//			if(Once_Mode_Enable == ENABLE)
//			{
//				AutoModeControl();
//			}              				 
//			break;

//		case SINGLE_MODE:	//����ģʽ
//			if(Single_Mode_Enable == ENABLE)
//			{
//				AutoModeControl();
//			}
//			break;

//		case LOOP_MODE:		//ѭ��ģʽ
//			if(Loop_Mode_Enable == ENABLE)
//			{
//				AutoModeControl();
//			}
//			break;

//		default:
//			break;
//	}
//}


/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
