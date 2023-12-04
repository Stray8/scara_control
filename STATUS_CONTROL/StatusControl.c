/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Auto.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f4xx.h"
#include "Usart.h" 
#include "StatusControl.h"
#include "SpeedControl.h"
#include "Auto.h"
#include "Manual.h"
#include "w25qxx.h"
#include "out.h"
#include "in.h"
#include "Error.h"
#include "Parameter.h"
#include "SignalWatch.h"
#include "Auto_2.h"
#include "BackToOrigin.h"
#include "ActionOperate.h"
#include "CANopen.h"

u8 Initialize_Finished = FALSE;	   		//��ʼ����ɱ�־λ
u8 Origin_Backed = FALSE;		   				//������ɱ�־λ
u8 Axsis_Origin_Backed[Axis_Num + Ext_Axis_Num] = {FALSE,FALSE,FALSE,FALSE};//�����������ɱ�־λ
u8 Back_Origin_Flag = FALSE;	   			//���������־λ

u8 Work_Status = AUTO_WORK_MODE;    	//��е�ֹ���ģʽ 
u8 Axis_Manul_BackToOrigin = FALSE;  	//�ֶ�����
u8 Axsis_Chosen = X_Axsis;		   			//�˶���ѡ��
u8 Axsis_Move_Direction[Axis_Num + Ext_Axis_Num] = {POSITIVE,POSITIVE,POSITIVE,POSITIVE,POSITIVE,POSITIVE};//�˶�����ѡ��			  
u32 Input_Detect_Time = 0;		   			//�����ⶨʱ����
u32 Communication_Time = 0;		   			//ͨ��ʱ�����-�ѻ���-DPF
u8 OffLine_Flag = FALSE;		   				//�ѻ���־λ-DPF
u8 OnLineCommunicated_Flag = FALSE;		//����ͨ�ű�־λ-DPF
u8 Input_Detect_Enable = ENABLE;   		//������ʹ�ܱ�־����һ��ʹ��
u8 Jog_Pause_Enable = DISABLE;       	//�綯������ͣ
u8 g_Current_SafeArea_Num = 0;	 	 		//��ȫ�����
u8 Input_Count17 = 0;			   					//����17���������㰴ť����
u8 Input_Count18 = 0;			   					//����18������������ť����
u8 Input_Count19 = 0;			   					//����19��������ͣ��ť����
u8 Input_Count20 = 0;			   					//����20������ֹͣ��ť����
u8 Servo_Stop_Done[Axis_Num + Ext_Axis_Num] = {FALSE, FALSE, FALSE, FALSE, FALSE, FALSE};		   //�ŷ���ֹͣ�����Ƿ�ִ�б�־
u8 g_Auto_Reset_Flag = FALSE;	   			//�Զ���λ�����־
u8 Robot_Auto_Reset = FALSE;					//�Զ���λ��ɱ�־
u8 g_Auto_LOrigin_Flag = FALSE;	   		//L�����
u32 g_USART_Delay_Timer = 0;		   		//������ʱ����-��ʽͨ��ʧ��
u8 gs_AutoStatue = 4;
u8 gs_Error_Status = NO_ERROR;
u8 g_MoveCmdRetrans = FALSE;

/**************************************************************************************************
**  ��������  WorkMode()
**	�����������
**	�����������
**	�������ܣ�����ģʽѡ��
**	��ע��	  ��
**  ���ߣ�         
**  �������ڣ� 
***************************************************************************************************/
void WorkMode()
{
	switch(UsartReceiveData[1])
	{
		case P_AUTO_MODE://�Զ�ģʽ
			Work_Status = AUTO_WORK_MODE;
			if(g_Robot_Has_Debuged_Flag)
			{
				g_AutoStatue = AUTO_WAITE;
				g_Auto_Order_Start = FALSE;
				g_Auto_Order_Pause = FALSE;
				g_Auto_Order_Stop = FALSE;
				Single_Mode_Enable = DISABLE;
				g_Auto_PresentLine = 0;	  					//��ǰ��������
				g_Auto_ActionRun_Step = 0;
				Robot_Auto_Reset = FALSE;		
				g_Robot_Has_Debuged_Flag = FALSE;	 
			}
			break;	

		case P_FREE_PROGRAM://���ɱ��
			Work_Status = FREE_PROGRAM_MODE;     
			break;	

		case P_IO_MODE://IO����
			Work_Status = IO_DEBUG_MODE;
			break;	

		case P_MANUL_MODE://�ֶ�ģʽ
			Work_Status = MANUAL_WORK_MODE;
			Axsis_Chosen = X_Axsis;										//�����ֶ�ģʽʱ��Ĭ��ѡ��X��
			Axsis_Move_Direction[X_Axsis] = POSITIVE;
			Linked_Mode_Enable = ENABLE;							//�����ֶ�ģʽʱ��Ĭ��Ϊ����ģʽ
			Jog_Mode_Enable = DISABLE;
			Jog_Pause_Enable=DISABLE;
			break;

		case WAIT_MODE://�ȴ�ģʽ
			Work_Status = WAIT_MODE;
			break;

		default:
			break;
	}
}

/**************************************************************************************************
**  ��������  u8 ManulSafeAreaDetec(void)
**	�����������
**	�����������
**	�������ܣ���е���˶���ȫ��������λ��⼰�ֶ��ƶ����봦��
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u8 ManulSafeAreaDetec(void)
{
	u16 i = 0;
	u8 safeAreaFlag = FALSE;
	
	if(Origin_Backed == TRUE)
	{
		for(i=0; i<SAVESAFEAREA; i++)
		{//ȷ����ǰX�����ĸ���ȫ����
			if(Robot_Safe_Area[i].SafeArea_Switch)
			{
				if(m_PulseTotalCounter[X_Axsis] >= Robot_Safe_Area[i].X_Left && m_PulseTotalCounter[X_Axsis] <= Robot_Safe_Area[i].X_Right)
				{
					if(Axsis_Chosen == X_Axsis && Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE && (m_PulseTotalCounter[Axsis_Chosen] <= Robot_Safe_Area[i].X_Left) && \
							(m_PulseTotalCounter[Axsis_Chosen] >= Robot_Safe_Area[i].Z_Up && m_PulseTotalCounter[Axsis_Chosen] <= Robot_Safe_Area[i].Z_Down))
					{//X�����ڵ�ǰ��ȫ����С�߽磬��������������
						safeAreaFlag = TRUE;
					}
					else if(Axsis_Chosen == X_Axsis && Axsis_Move_Direction[Axsis_Chosen] == POSITIVE && (m_PulseTotalCounter[Axsis_Chosen] >= Robot_Safe_Area[i].X_Right) && \
							(m_PulseTotalCounter[Axsis_Chosen] >= Robot_Safe_Area[i].Z_Up && m_PulseTotalCounter[Axsis_Chosen] <= Robot_Safe_Area[i].Z_Down))
					{//X�����ڵ�ǰ��ȫ�����߽磬��������������
						safeAreaFlag = TRUE;
					}
					else if(Axsis_Chosen == Z_Axsis && Axsis_Move_Direction[Axsis_Chosen] == POSITIVE && (m_PulseTotalCounter[Axsis_Chosen] >= Robot_Safe_Area[i].Z_Down))
					{//Z�����ڵ�ǰ��ȫ�����߽磬��������������Z���򸺷����������ƶ�
						safeAreaFlag = TRUE;
					}
					
					//����λ����
					if(Axsis_Move_Direction[Axsis_Chosen] == POSITIVE && m_PulseTotalCounter[Axsis_Chosen] >= Axsis_Maxlength[Axsis_Chosen])
					{
						safeAreaFlag = TRUE;
					}
					else if(Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE && m_PulseTotalCounter[Axsis_Chosen] <= Axsis_Minlength[Axsis_Chosen])
					{
						safeAreaFlag = TRUE;
					}
					
					if(safeAreaFlag == FALSE)
					{
						if(Axsis_Chosen == Z_Axsis)
						{//����Z��
							if(Linked_Mode_Enable == ENABLE && Axsis_Move_Direction[Axsis_Chosen] == POSITIVE)
							{//����ʱ���޸�Z�������ƶ�����
								if(Linked_Pulse > Robot_Safe_Area[i].Z_Down && Robot_Safe_Area[i].Z_Down > m_PulseTotalCounter[Axsis_Chosen])
								{
									Linked_Pulse = Robot_Safe_Area[i].Z_Down;
								}
							}
							else if(Jog_Mode_Enable == ENABLE && Axsis_Move_Direction[Axsis_Chosen] == POSITIVE)
							{//�綯ʱ���޸�Z�������ƶ�����
								if(Jog_Pulse_Count > Robot_Safe_Area[i].Z_Down && Robot_Safe_Area[i].Z_Down > m_PulseTotalCounter[Axsis_Chosen])
								{
									Jog_Pulse_Count = Robot_Safe_Area[i].Z_Down;
								}
							}
						}
						else if(Axsis_Chosen == X_Axsis && (m_PulseTotalCounter[Z_Axsis] > Robot_Safe_Area[i].Z_Up && \
									m_PulseTotalCounter[Z_Axsis] <= Robot_Safe_Area[i].Z_Down))
						{//����X��
							if(Linked_Mode_Enable == ENABLE && Axsis_Move_Direction[Axsis_Chosen] == POSITIVE)
							{//����ʱ���޸�X�������ƶ�����
								if(Linked_Pulse > Robot_Safe_Area[i].X_Right)
								{
									Linked_Pulse = Robot_Safe_Area[i].X_Right;
								}
							}
							else if(Linked_Mode_Enable == ENABLE && Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE)
							{//����ʱ���޸�X�Ḻ���ƶ�����
								if(Linked_Pulse < Robot_Safe_Area[i].X_Left)
								{
									Linked_Pulse = Robot_Safe_Area[i].X_Left;
								}
							}
							else if(Jog_Mode_Enable == ENABLE && Axsis_Move_Direction[Axsis_Chosen] == POSITIVE)
							{//�綯ʱ���޸�X�������ƶ�����
								if(Jog_Pulse_Count > Robot_Safe_Area[i].X_Right)
								{
									Jog_Pulse_Count = Robot_Safe_Area[i].X_Right;
								}
							}
							else if(Jog_Mode_Enable == ENABLE && Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE)
							{//�綯ʱ���޸�X�Ḻ���ƶ�����
								if(Jog_Pulse_Count <Robot_Safe_Area[i].X_Left)
								{
									Jog_Pulse_Count = Robot_Safe_Area[i].X_Left;
								}
							}
						}
					}
				}
			}
			
			if(safeAreaFlag == TRUE)
			{
				break;
			}
		}
	}
	
	return safeAreaFlag;
}


/**************************************************************************************************
**  ��������  CurProgramRead()
**	�����������
**	�����������
**	�������ܣ�ѡ�г����ȡ
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void CurProgramRead(u8 programNum)
{
	u16 j = 0;
	u16 k = 0;
	u8 Read_IIC_Auto_Program[16] = {0};
	
	if(programNum > 0 && programNum <= SAVEPROGRAMNUM)
	{
		g_Run_Program_Num = programNum;
		W25QXX_Write(&g_Run_Program_Num, 0x40E0, 1);
		g_Run_Program_Num_Pre = g_Run_Program_Num;
		Read_SaveProgram_IIC_Address();
		if(SaveProgram_IIC_Address > 0)
		{
			W25QXX_Read(Read_IIC_Auto_Program, SaveProgram_IIC_Address, 15);
			Free_Program_Operate.Flag  = Read_IIC_Auto_Program[0];
			Free_Program_Operate.Code  = Read_IIC_Auto_Program[1];
			Free_Program_Operate.Name  = (u32)(((u32)Read_IIC_Auto_Program[2])|((u32)Read_IIC_Auto_Program[3]<<8)|((u32)Read_IIC_Auto_Program[4]<<16)|((u32)Read_IIC_Auto_Program[5]<<24));
			Free_Program_Operate.Name2 = (u32)(((u32)Read_IIC_Auto_Program[6])|((u32)Read_IIC_Auto_Program[7]<<8)|((u32)Read_IIC_Auto_Program[8]<<16)|((u32)Read_IIC_Auto_Program[9]<<24));
			Free_Program_Operate.Name3 = (u32)(((u32)Read_IIC_Auto_Program[10])|((u32)Read_IIC_Auto_Program[11]<<8)|((u32)Read_IIC_Auto_Program[12]<<16)|((u32)Read_IIC_Auto_Program[13]<<24));				 
			Free_Program_Operate.Num   = Read_IIC_Auto_Program[14];
			for(j=0; j<Free_Program_Operate.Num; j++)
			{
				W25QXX_Read(Read_IIC_Auto_Program, SaveProgram_IIC_Address+0x0F+0x10*j, 16);
				Free_Program_Operate.Program[j].Flag   = Read_IIC_Auto_Program[0];
				Free_Program_Operate.Program[j].List   = Read_IIC_Auto_Program[1];
				Free_Program_Operate.Program[j].Order  = Read_IIC_Auto_Program[2];
				Free_Program_Operate.Program[j].Key    = Read_IIC_Auto_Program[3];
				Free_Program_Operate.Program[j].Value1 = (u32)(((u32)Read_IIC_Auto_Program[4])|((u32)Read_IIC_Auto_Program[5]<<8)|((u32)Read_IIC_Auto_Program[6]<<16)|((u32)Read_IIC_Auto_Program[7]<<24));
				Free_Program_Operate.Program[j].Value2 = (u32)(((u32)Read_IIC_Auto_Program[8])|((u32)Read_IIC_Auto_Program[9]<<8)|((u32)Read_IIC_Auto_Program[10]<<16)|((u32)Read_IIC_Auto_Program[11]<<24));	
				Free_Program_Operate.Program[j].Value3 = (u32)(((u32)Read_IIC_Auto_Program[12])|((u32)Read_IIC_Auto_Program[13]<<8)|((u32)Read_IIC_Auto_Program[14]<<16)|((u32)Read_IIC_Auto_Program[15]<<24)); 					
				Free_Program_Operate.Program[j].Value1 = Free_Program_Operate.Program[j].Value1 & 0x0fffffff;
				Free_Program_Operate.Program[j].Value2 = Free_Program_Operate.Program[j].Value2 & 0x0fffffff;
					
				if((Read_IIC_Auto_Program[3] == K_INCREMENT_RUNNING && Read_IIC_Auto_Program[15]>>4 == 0x09)\
					|| ((Read_IIC_Auto_Program[3] == K_IF || Read_IIC_Auto_Program[3] == K_ELSE ||Read_IIC_Auto_Program[3] == K_WHILE || Read_IIC_Auto_Program[3] == K_USER) && Read_IIC_Auto_Program[15]>>4 == 0x08))
				{
					Free_Program_Operate.Program[j].Value3 = Free_Program_Operate.Program[j].Value3 | 0xf0000000;	
				}
				else
				{
					Free_Program_Operate.Program[j].Value3 = Free_Program_Operate.Program[j].Value3 & 0x0fffffff;	
				}
			}
			for(k=j; k<LARGESTPROGRAMNUM; k++)
			{
				Free_Program_Operate.Program[k].Flag   = 0;
				Free_Program_Operate.Program[k].List   = 0;
				Free_Program_Operate.Program[k].Order  = 0;
				Free_Program_Operate.Program[k].Key    = 0;
				Free_Program_Operate.Program[k].Value1 = 0;
				Free_Program_Operate.Program[k].Value2 = 0;
				Free_Program_Operate.Program[k].Value3 = 0;
			}
		}
	}
}

/**************************************************************************************************
**  ��������  ActionControl()
**	�����������
**	�����������
**	�������ܣ���е�ֶ�������
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void ActionControl()
{
	u16 i = 0, j = 0;
	u8 Reset_Parameter[20] = {0};
	
	if(Error_Status != ERROR_EMERG_HAPPEN)
	{//������������ʱ������������ģʽ�µ����в���
		if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE && Origin_Backed == TRUE)
		{//�Զ�ģʽ
			if(g_Auto_Reset_Flag)
			{//ִ�и�λ�����˶���������㣩
				if(Temp_OUT_Switch_Parameter[O_RESETING_LIGHT] == 1)
				{
					ResetOutput(O_RESETING_LIGHT);
				}
				g_Run_Program_Num_Pre = g_Run_Program_Num;				
				for(i=0; i<SAVEPROGRAMNUM; i++)
				{//��ѯ������뵱ǰ�趨�Զ����б����ͬ����
					if(Program_IIC_Address[i].Code == SAVEPROGRAMNUM_MAIN && Program_IIC_Address[i].Flag == 1)
					{
						W25QXX_Read(Reset_Parameter, Program_IIC_Address[i].Address, 15);
						Free_Program_Operate.Flag  = Reset_Parameter[0];
						Free_Program_Operate.Code  = Reset_Parameter[1];
						Free_Program_Operate.Name  = (u32)(((u32)Reset_Parameter[2])|((u32)Reset_Parameter[3]<<8)|((u32)Reset_Parameter[4]<<16)|((u32)Reset_Parameter[5]<<24));
						Free_Program_Operate.Name2 = (u32)(((u32)Reset_Parameter[6])|((u32)Reset_Parameter[7]<<8)|((u32)Reset_Parameter[8]<<16)|((u32)Reset_Parameter[9]<<24));
						Free_Program_Operate.Name3 = (u32)(((u32)Reset_Parameter[10])|((u32)Reset_Parameter[11]<<8)|((u32)Reset_Parameter[12]<<16)|((u32)Reset_Parameter[13]<<24));
						Free_Program_Operate.Num   = Reset_Parameter[14];
						for(j=0; j<Free_Program_Operate.Num; j++)
						{
							W25QXX_Read(Reset_Parameter, Program_IIC_Address[i].Address+0x0F+0x10*j, 16);
							Free_Program_Operate.Program[j].Flag   = Reset_Parameter[0];
							Free_Program_Operate.Program[j].List   = Reset_Parameter[1];
							Free_Program_Operate.Program[j].Order  = Reset_Parameter[2];
							Free_Program_Operate.Program[j].Key    = Reset_Parameter[3];
							Free_Program_Operate.Program[j].Value1 = (u32)(((u32)Reset_Parameter[4])|((u32)Reset_Parameter[5]<<8)|((u32)Reset_Parameter[6]<<16)|((u32)Reset_Parameter[7]<<24));
							Free_Program_Operate.Program[j].Value2 = (u32)(((u32)Reset_Parameter[8])|((u32)Reset_Parameter[9]<<8)|((u32)Reset_Parameter[10]<<16)|((u32)Reset_Parameter[11]<<24));	
							Free_Program_Operate.Program[j].Value3 = (u32)(((u32)Reset_Parameter[12])|((u32)Reset_Parameter[13]<<8)|((u32)Reset_Parameter[14]<<16)|((u32)Reset_Parameter[15]<<24)); 					
							Free_Program_Operate.Program[j].Value1 = Free_Program_Operate.Program[j].Value1 & 0x0fffffff;
							Free_Program_Operate.Program[j].Value2 = Free_Program_Operate.Program[j].Value2 & 0x0fffffff;	
								
							if((Reset_Parameter[3] == K_INCREMENT_RUNNING && Reset_Parameter[15]>>4 == 0x09)\
								|| ((Reset_Parameter[3] == K_IF	 || Reset_Parameter[3] == K_ELSE || Reset_Parameter[3] == K_WHILE || Reset_Parameter[3] == K_USER) && Reset_Parameter[15]>>4 == 0x08))
							{
								Free_Program_Operate.Program[j].Value3 = Free_Program_Operate.Program[j].Value3 | 0xf0000000;	
							}
							else
							{
								Free_Program_Operate.Program[j].Value3 = Free_Program_Operate.Program[j].Value3 & 0x0fffffff;	
							}
						}
						break;
					}
				}
				ActionStepControl();
			}
			if(Robot_Auto_Reset)
			{//ִ�и�λ��ɺ����ִ����������
				AutoModeControl();
			}
		}
		else if(Work_Status == MANUAL_WORK_MODE && Back_Origin_Flag == FALSE)
		{//�ֶ�ģʽ
			if(Origin_Backed != TRUE)
			{//û������ֶ�����,ǿ����������
				for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
				{
					//m_PulseTotalCounter[i] = MINROBOTPOSITION + (Axsis_Maxlength[i] - Axsis_Minlength[i]) / 2;
					if(Axis_Manul_Speed_Temp[i] > 20)//û����ʱ����������ٶ�20%
					{
						Axis_Manul_Speed[i] = 20;
					}
					else
					{
						Axis_Manul_Speed[i] = Axis_Manul_Speed_Temp[i];
					}
				}
			}
			else if(Origin_Backed == TRUE)
			{//�ع�����Ҫ���ٶ�
				for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
				{
					Axis_Manul_Speed[i] = Axis_Manul_Speed_Temp[i];
				}
			}
			
			if(Jog_Mode_Enable == ENABLE)
			{//�綯ģʽ
				 ManualJogRunnig();								//�綯��������
				 if(Jog_Pause_Enable == ENABLE)
				 {
					Servo_Stop(Axsis_Chosen);
					Jog_Pause_Enable = DISABLE;			 
				 }
			}
			else if(Linked_Mode_Enable == ENABLE)
			{//����ģʽ
				if(Axis_Manul_BackToOrigin == TRUE && m_PulseTotalCounter[Axsis_Chosen] == MINROBOTPOSITION)
				{//�ֶ��������֮��
					Linked_Move_Enable = DISABLE;
					Axis_Manul_BackToOrigin = FALSE;
				}
				ManualLinkedRunning();						//������������
			}
		}
	}
}
/**************************************************************************************************
**  ��������  StatusControl()
**	�����������
**	�����������
**	�������ܣ�����״̬���
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void StatusControl()
{
  u8 i = 0;

	for(i=0; i<30; i++)
	{//��ȡ�����ƽ
		ReadInput(i);
	}
	HardLimitJudge();						//������ѯ��ʽʵ���ⲿ�ж�
	if(Input_Detect_Enable == ENABLE)
	{
		Input_Detect_Enable = DISABLE;
		
		/**-- �ѻ�ģʽ�£��ⲿ��ͣ��ֹͣ�����������㰴ť�Ĵ��� --**/
		if(OffLine_Flag == TRUE)
		{//�ѻ�״̬��
			if((Work_Status == AUTO_WORK_MODE) && (g_AutoStatue != AUTO_RUNNING))
			{//��е�ֹ���ģʽΪ�Զ�ģʽ���Զ�����״̬������������ʱ�������͸�λ����Ч
				if(JXS_Parameter.NcOrignin == 0)
				{//���㰴ťûѡ��ʱ�����������Զ���λ
					if((JXS_Parameter.NcStartin != 0) && (Input_Detect_Status[2] & 0x04))
					{//���� X18
						 Input_Count18++;
						 if(Input_Count18 == 5)
						 {//�����˲�
								Back_Origin_Flag = TRUE;//�Ȼ���
								if(Origin_Backed == TRUE)
								{//������ɣ�ֻ�踴λ
									g_Auto_Reset_Flag = TRUE;
									Back_Origin_Flag = FALSE;
								}
								
								if(Robot_Auto_Reset == TRUE)
								{//��λ��ɣ�ֱ������
									g_Auto_Order_Start = TRUE;
									g_Auto_Reset_Flag = FALSE;
								}
								
								if(Auto_Mode == ONCE_MODE)
								{//����ģʽ
									Once_Mode_Enable = ENABLE;
									Loop_Mode_Enable = DISABLE;
								}
								else if(Auto_Mode == LOOP_MODE)
								{//ѭ��ģʽ
									Loop_Mode_Enable = ENABLE;
									Once_Mode_Enable = DISABLE;
								}
								
								g_Auto_Order_Stop = FALSE;
								Input_Count18=0;
						 }
					}
					else
					{
						Input_Count18=0;
					}
					
					if((JXS_Parameter.NcStopin != 0) && (Input_Detect_Status[2] & 0x10))	 
					{//�����ͣ���ֹͣ X20
						 Input_Count20++;
						 if(Input_Count20 == 5)
						 {
								Robot_Auto_Reset=FALSE;
								g_AutoStatue = AUTO_WAITE;
								g_Auto_Order_Stop = TRUE;
								AutoPauseOperate();				//��ͣ������ز���
								AutoStopOperate();				//ֹͣ������ز���
								Input_Count20 = 0;
						 }
					}
					else
					{
						Input_Count20 = 0;
					}
				}
				else
				{//���㰴ťѡ��ʱ��ǿ��ȥ��λ����������
					if(Input_Detect_Status[2] & 0x02)	 
					{//��λ	X17
						 Input_Count17++;
						 if(Input_Count17 == 5)
						 {
								Back_Origin_Flag = TRUE;			//�Ȼ���
								if(Origin_Backed == TRUE)			//�������
								{
									g_Auto_Reset_Flag = TRUE;		//ȥ��λ
									Back_Origin_Flag = FALSE;		//�����־λ��0
								}
								Input_Count17=0;
						 }
					}
					else
					{
						Input_Count17=0;
					}
					
					if((JXS_Parameter.NcStartin != 0) && (Input_Detect_Status[2] & 0x04) && (Robot_Auto_Reset == TRUE))	 
					{//���� X18����λ���ʱ�ſ�����
						 Input_Count18++;
						 if(Input_Count18 == 5)
						 {
								g_Auto_Order_Start = TRUE;				//�������
								g_Auto_Reset_Flag = FALSE;				//��λ��־λ��0							
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
								g_Auto_Order_Stop = FALSE;				//��״̬Ϊ��0
								Input_Count18=0;
						 }
					}
					else
					{
						Input_Count18=0;
					}
					if((JXS_Parameter.NcStopin != 0) && (Input_Detect_Status[2] & 0x10))	  //�����ͣ���ֹͣ X20
					{
						 Input_Count20++;
						 if(Input_Count20 == 5)
						 {	
							Robot_Auto_Reset=FALSE;
							g_AutoStatue = AUTO_WAITE;
							g_Auto_Order_Stop = TRUE;
							AutoPauseOperate();				//��ͣ������ز���
							AutoStopOperate();				//ֹͣ������ز���
							Input_Count20=0;
						 }
					}
					else
					{
						Input_Count20 = 0;
					}
				}	
			}
			else if((Work_Status == AUTO_WORK_MODE) && (g_AutoStatue == AUTO_RUNNING))
			{//��е�ֹ���ģʽΪ�Զ�ģʽ���Զ�����״̬����������ʱ����ͣ��ֹͣ����Ч
				if((JXS_Parameter.NcPausein != 0) && (Input_Detect_Status[2] & 0x08))
				{//��ͣ	X19
					 Input_Count19++;
					 if(Input_Count19 == 5)
					 {				    
						g_Auto_Order_Pause = TRUE;
						AutoPauseOperate();						//��ͣ������ز���
						Input_Count19 = 0;
					 }
				}
				else
				{
					Input_Count19 = 0;
				}
				
				if((JXS_Parameter.NcStopin!=0)&&(Input_Detect_Status[2] & 0x10))	 
				{//ֹͣ X20
					 Input_Count20++;
					 if(Input_Count20 == 5)
					 {	
							Robot_Auto_Reset = FALSE;
							g_AutoStatue = AUTO_WAITE;
							g_Auto_Order_Stop = TRUE;
							AutoPauseOperate();					//��ͣ������ز���
							AutoStopOperate();					//ֹͣ������ز���
							Input_Count20 = 0;
					 }
				}
				else
				{
					Input_Count20 = 0;
				}
			}
		}
	}
	
	if(g_AutoStatue != gs_AutoStatue)
	{//��ɫ�Ƽ���������״̬����
		if(OffLine_Flag == TRUE)
		{//�ѻ�״̬�£�5s֮������
			gs_AutoStatue = g_AutoStatue;
			switch(g_AutoStatue)
			{
				case AUTO_RUNNING:
					SetOutput(O_WAIT_YELLOW_LIGHT);	   	//�Ƶ�
					SetOutput(O_ALARM_RED_LIGHT);	   		//���
					SetOutput(O_ALARM_BUZZER);	   			//������
					ResetOutput(O_RUN_GREEN_LIGHT);  		//�̵�
					break;

				case AUTO_PAUSE:
					if((Error_Status != ERROR_HAPPEN) && (g_Auto_ActionTimeOut_Flag !=TRUE))
					{
						SetOutput(O_RUN_GREEN_LIGHT);
						SetOutput(O_ALARM_RED_LIGHT);
						SetOutput(O_ALARM_BUZZER);	  		//������
						ResetOutput(O_WAIT_YELLOW_LIGHT); //�Ƶ�
					}
					else	//������ʱ����£������
					{
						gs_Error_Status=Error_Status;
						SetOutput(O_RUN_GREEN_LIGHT);
						SetOutput(O_WAIT_YELLOW_LIGHT);
						SetOutput(O_ALARM_BUZZER);	  		//������
						ResetOutput(O_ALARM_RED_LIGHT); 	//���
					}
					break;

				case AUTO_WAITE:
					SetOutput(O_RUN_GREEN_LIGHT);
					SetOutput(O_ALARM_RED_LIGHT);
					SetOutput(O_ALARM_BUZZER);	  			//������
					ResetOutput(O_WAIT_YELLOW_LIGHT); 	//�Ƶ�										   
					break;

				case AUTO_ERROR:
					SetOutput(O_RUN_GREEN_LIGHT);
					SetOutput(O_WAIT_YELLOW_LIGHT);					
					ResetOutput(O_ALARM_RED_LIGHT); 		//���
					ResetOutput(O_ALARM_BUZZER); 				//������
					break;

				default:
					break;
			}
		}
		else if(OffLine_Flag == FALSE && OnLineCommunicated_Flag == TRUE)
		{//�����ѻ�״̬���Ѿ�ͨ���ţ�����������
			gs_AutoStatue=g_AutoStatue;
			switch(g_AutoStatue)
			{
				case AUTO_RUNNING:
					SetOutput(O_WAIT_YELLOW_LIGHT);	   	//�Ƶ�
					SetOutput(O_ALARM_RED_LIGHT);	   		//���
					SetOutput(O_ALARM_BUZZER);	   			//������
					ResetOutput(O_RUN_GREEN_LIGHT);  		//�̵�
					break;

				case AUTO_PAUSE:
					if((Error_Status != ERROR_HAPPEN) && (g_Auto_ActionTimeOut_Flag !=TRUE))
					{
						SetOutput(O_RUN_GREEN_LIGHT);
						SetOutput(O_ALARM_RED_LIGHT);
						SetOutput(O_ALARM_BUZZER);	  		//������
						ResetOutput(O_WAIT_YELLOW_LIGHT); //�Ƶ�
					}
					else	//������ʱ����£������
					{
						gs_Error_Status=Error_Status;
						SetOutput(O_RUN_GREEN_LIGHT);
						SetOutput(O_WAIT_YELLOW_LIGHT);
						SetOutput(O_ALARM_BUZZER);	  		//������
						ResetOutput(O_ALARM_RED_LIGHT); 	//���
					}
					break;

				case AUTO_WAITE:
					SetOutput(O_RUN_GREEN_LIGHT);
					SetOutput(O_ALARM_RED_LIGHT);
					SetOutput(O_ALARM_BUZZER);	  			//������
					ResetOutput(O_WAIT_YELLOW_LIGHT); 	//�Ƶ�										   
					break;

				case AUTO_ERROR:
					SetOutput(O_RUN_GREEN_LIGHT);
					SetOutput(O_WAIT_YELLOW_LIGHT);					
					ResetOutput(O_ALARM_RED_LIGHT); 		//���
					ResetOutput(O_ALARM_BUZZER); 				//������
					break;

				default:
					break;
			}
		}
	}
	
	if(Error_Status != gs_Error_Status)
	{
		gs_Error_Status = Error_Status;
		if(Error_Status == ERROR_EMERG_HAPPEN)
		{		   
			SetOutput(O_RUN_GREEN_LIGHT);
			SetOutput(O_WAIT_YELLOW_LIGHT);					
			ResetOutput(O_ALARM_RED_LIGHT); 				//���
			ResetOutput(O_ALARM_BUZZER); 					//������	 
		}
		else 
		{	     	   
			SetOutput(O_RUN_GREEN_LIGHT);
			SetOutput(O_ALARM_RED_LIGHT);
			SetOutput(O_ALARM_BUZZER);							//������
			ResetOutput(O_WAIT_YELLOW_LIGHT); 			//�Ƶ�
			if(g_Auto_WorkFinished_Flag == TRUE || g_Auto_CJWorkFinished_Flag == TRUE)//�ӹ����״̬
			{
				ResetOutput(O_ALARM_BUZZER);					//������
			}
			else
			{
				SetOutput(O_ALARM_BUZZER);						//������
			}
		}
	}
	
	if(Temp_OUT_Switch_Parameter[O_ORIGINED_LIGHT] == 1)
	{
		if(Origin_Backed == TRUE)
		{
			ResetOutput(O_ORIGINED_LIGHT);
		}
		else
		{
			SetOutput(O_ORIGINED_LIGHT);
		}
	}
}

/**************************************************************************************************
**  ��������  Servo_Stop(u8)
**	�����������
**	�����������
**	�������ܣ��ŷ���ֹͣ���ƺ���
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void Servo_Stop(u8 Axis)
{
	ServoStopSet_PDO(Axis_To_ID(Axis));
	Pulse_StopRequest[Axis] = 1;
	if(ServoWorkModeRead(Axis_To_ID(Axis)) != SERVO_MODE_CYC_CSP && ServoWorkModeRead(Axis_To_ID(Axis)) != SERVO_MODE_CYC_CSV)
	{
		AxisMoveFlag[Axis] = 0;
	}
	Flag_Keep_Move[Axis] = FALSE;
}

/**************************************************************************************************
**  ��������  Robot_Reset()
**	�����������
**	�����������
**	�������ܣ�������λ��ɺ���ز�����λ����ʼֵ
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void Robot_Reset()
{
	u16 i = 0;
	
	//������ر�����λ
	g_Auto_ActionError_Flag = FALSE;			//���г�������־��λ
	g_Auto_ActionTimeOut_Flag = FALSE;		//���г�ʱ������־��λ
	MD_PositionErr_Flag = FALSE;					//���λ�ü�������־λ
	
	Cancle_Genaral_Warning = FALSE;				//ȡ������������־��λ
	Error_Status = NO_ERROR;							//����״̬��λ
	Robot_Error_Data[0] = 0;							//����������λ
	for(i=1; i<10; i++)
	{//��ͨ������λ
		Robot_Error_Data[i] = 0;
	}
	
	//������ر�����λ
	g_Auto_WorkFinished_Flag = FALSE;				//�������üӹ���ɱ�־��λ
	g_Auto_CJWorkFinished_Flag = FALSE;			//�����������ɱ�־��λ
	
	//���˶���ز�����λ
	if(JDZ_Parameter.Switch)		//����ֵģʽ
	{}
	else
	{
//		for(i=0; i<Axis_Num; i++)
//		{
//			m_PulseTotalCounter[i] = MINROBOTPOSITION + JXS_Parameter.OrignOffset[i] * Step_Coefficient[i] / 100;		//��λ��ɺ�ǰλ������Ϊ��ʱԭ��
//			
//			if(JXS_Parameter.OriginDir[i] == 0)
//			{//�������Ĭ���˶�����
//				Axsis_Move_Direction[i] = POSITIVE;
//			}
//			else
//			{
//				Axsis_Move_Direction[i] = NEGATIVE;
//			}
//		}
	}
	
	Axsis_Chosen = X_Axsis;									//����Ĭ��ѡ����ΪX��
	Work_Status = AUTO_WORK_MODE;						//��е�ֹ���ģʽΪ�Զ�ģʽ
		
	Action_Delay_Flag = FALSE;							//������ʱ���
	Action_Done_Flag = FALSE;								//�ŷ���������ɱ�־
	
	DIR485_L;	   														//485Ĭ�Ͻ���״̬
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
