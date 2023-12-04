/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : Auto_2.c
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 21/10/2015
* Description        : �Զ����п����ļ�
***************************************************************************************/
#include "stm32f4xx.h"
#include "Auto_2.h"
#include "ActionOperate.h"
#include "StatusControl.h"
#include "Auto.h"
#include "in.h"
#include "out.h"
#include "w25qxx.h"
#include "Error.h"
#include "SpeedControl.h"
#include "SignalWatch.h"
#include "stmflash.h"

/***************************��������������λ����ѯ�����**************************************/
u8  g_AutoStatue = AUTO_WAITE;		//�Զ�����״̬
u16 g_Auto_PresentLine = 0;				//�Զ������к�

u8  g_SubProgram_Step_Run[SAVEPROGRAMNUM_SUB] = {FALSE};    //�Ƿ������ӳ���������������λ
u8  g_SubProgram_Start[SAVEPROGRAMNUM_SUB] = {FALSE};				//�ӳ���ʼ
u8  g_SubProgram_Finish[SAVEPROGRAMNUM_SUB] = {FALSE};			//�ӳ������
u8  g_SubProgram_ContralEnd[SAVEPROGRAMNUM_SUB] = {FALSE};	//�����ӳ������
u8  g_Read_SubProgram[SAVEPROGRAMNUM_SUB] = {FALSE};				//�ӳ����ȡ���
u16 g_SubProgram_PresentLine[SAVEPROGRAMNUM_SUB] = {0};			//�ӳ����Զ������к�

u8  g_SubProgram_ActionRun_Step[SAVEPROGRAMNUM_SUB] = {0};				  //�ӳ���
u32 g_SubProgram_ActionRun_Timer[SAVEPROGRAMNUM_SUB] = {0};					//�ӳ�����ʱ������
u32 g_SubProgram_ActionTimeOut_Time[SAVEPROGRAMNUM_SUB] = {3000}; 	//10msΪ��λ,30sû��⵽ȷ���źţ�����Ϊ��ʱ

u8  g_Auto_Order_Start = FALSE;		//�Զ���������ָ��
u8  g_Auto_Order_Pause = FALSE;		//�Զ�������ָͣ��
u8  g_Auto_Order_Stop  = FALSE;		//�Զ�����ָֹͣ��

//���������ڲ�����
u32 g_Auto_ActionRun_Timer = 0;
u8  g_Auto_ActionRun_Step = 0;
u8	g_Auto_ActionNcWait_Flag = 0;														//�����������ⳬʱ����־λ
u8 g_Auto_SubActionNcWait_Flag[SAVEPROGRAMNUM_SUB] = {0};		//�ӳ��������ⳬʱ����־λ
u32	g_Auto_ActionTimeOut_Time = 3000; 											//10msΪ��λ		 30sû��⵽ȷ���źţ�����Ϊ��ʱ
u8 	g_Auto_ActionTimeOut_Flag = FALSE;											//������ⳬʱ
u8 	g_Auto_ActionError_Flag = FALSE;  											//�Զ����н��뵽����״̬AUTO_ERROR
u8 	g_Auto_WorkFinished_Flag = FALSE; 											//��ǰ�������������
u8 	g_Auto_CJWorkFinished_Flag = FALSE; 										//��ǰ�������������
u8  MD_PositionErr_Flag = FALSE;														//���λ����������쳣

u8 g_Auto_Valid_Flag = 0;//��Чʱ���ʱ��־λ
u32 g_Auto_Valid_Timer = 0;//��Чʱ���ʱ��������Ч�źſ�ʼ��ʱ����һ��������Ч�ͺ���λ��ʱ��������󱣳�ʱ��Ҳ��λ��ʱ

u8 g_SubAuto_Valid_Flag[SAVEPROGRAMNUM_SUB] = {0};//��Чʱ���ʱ��־λ
u32 g_SubAuto_Valid_Timer[SAVEPROGRAMNUM_SUB] = {0};//��Чʱ���ʱ��������Ч�źſ�ʼ��ʱ����һ��������Ч�ͺ���λ��ʱ��������󱣳�ʱ��Ҳ��λ��ʱ

//jump���ܲ���
u16 m_JumpStepRunNum = {0}; 																		//��������ת�����к�
u16 m_JumpSubStepRunNum[SAVEPROGRAMNUM_SUB] = {0};															//�ӳ�����ת�����к�

//while���ܲ���
u8  m_WhileNC = {0};																						//������while����Ƕ�ײ���������
u8  m_WhileRunFlag[WHILE_NEST_MAX] = {0};											//������while�������б�־
u16 m_WhileLineNum[WHILE_NEST_MAX] = {0};											//������while�����к�
//u16 m_WhileOverLineNum[WHILE_NEST_MAX] = {0};								//������while�����Ӧ����������к�
u16 m_WhileCycCounter[WHILE_NEST_MAX] = {0};										//������while����ѭ������
u8  m_WhileJudgeType[WHILE_NEST_MAX] = {0};										//������while������������
s32 m_WhileJudgePar1[WHILE_NEST_MAX] = {0};										//������while�������1
s32 m_WhileJudgePar2[WHILE_NEST_MAX] = {0};										//������while�������2
u8  m_WhileJudgeRes[WHILE_NEST_MAX] = {0};											//������while�����жϽ����0ʧ�ܣ�1�ɹ�

u8  m_WhileSubNC[SAVEPROGRAMNUM_SUB] = {0};																			//�ӳ���while����Ƕ�ײ���������
u8  m_WhileSubRunFlag[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};								//�ӳ���while�������б�־
u16 m_WhileSubLineNum[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};								//�ӳ���while�����к�
//u16 m_WhileOverSubLineNum[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};						//�ӳ���while�����Ӧ����������к�
u16 m_WhileSubCycCounter[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};							//�ӳ���while����ѭ������
u8  m_WhileSubJudgeType[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};							//�ӳ���while������������
s32 m_WhileSubJudgePar1[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};							//�ӳ���while�������1
s32 m_WhileSubJudgePar2[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};							//�ӳ���while�������2
u8  m_WhileSubJudgeRes[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX] = {0};								//�ӳ���while�����жϽ����0ʧ�ܣ�1�ɹ�

//if���ܲ���
u8  m_IfElseNC = {0};																					//������If-Else����Ƕ�ײ���������
u8  m_IfElseJudgeType[IF_ELSE_NEST_MAX] = {0};									//������If-Else������������
s32 m_IfElseJudgePar1[IF_ELSE_NEST_MAX] = {0};									//������If-Else�������1
s32 m_IfElseJudgePar2[IF_ELSE_NEST_MAX] = {0};									//������If-Else�������2
u8  m_IfElseJudgeRes[IF_ELSE_NEST_MAX] = {0};									//������If-Else�����жϽ����0ʧ�ܣ�1�ɹ�
u8  m_IfElseJudgeRunFlag[IF_ELSE_NEST_MAX] = {0};							//������If-Else�������ִ�б�־��0δִ�У�1��ִ��

u8  m_IfElseSubNC[SAVEPROGRAMNUM_SUB] = {0};																		//�ӳ���If-Else����Ƕ�ײ���������
u8  m_IfElseSubJudgeType[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX] = {0};						//�ӳ���If-Else������������
s32 m_IfElseSubJudgePar1[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX] = {0};						//�ӳ���If-Else�������1
s32 m_IfElseSubJudgePar2[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX] = {0};						//�ӳ���If-Else�������2
u8  m_IfElseSubJudgeRes[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX] = {0};						//�ӳ���If-Else�����жϽ����0ʧ�ܣ�1�ɹ�
u8  m_IfElseSubJudgeRunFlag[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX] = {0};				//�ӳ���If-Else�������ִ�б�־��0δִ�У�1��ִ��

//�����˶�
u8 Flag_Falling_Edge = FALSE;																		//�½��ر�־λ
u8 Flag_Rising_Edge = FALSE;																		//�����ر�־λ
u8 Flag_Falling_Edge_Sub = FALSE;																//�ӳ����½��ر�־λ
u8 Flag_Rising_Edge_Sub = FALSE;																//�ӳ��������ر�־λ
u8 Flag_Keep_Move[Axis_Num + Ext_Axis_Num] = {FALSE};						//�����˶�ָ���־λ

//������
u8 Detect_Falling_Edge = FALSE;																	//�½��ر�־λ
u8 Detect_Rising_Edge = FALSE;																	//�����ر�־λ
u8 Detect_Falling_Edge_Sub[SAVEPROGRAMNUM_SUB] = {FALSE};				//�ӳ����½��ر�־λ
u8 Detect_Rising_Edge_Sub[SAVEPROGRAMNUM_SUB] = {FALSE};				//�ӳ��������ر�־λ

u8 Program_Origin_Axis = X_Axsis;																//��е����ָ����
u8 Program_Axis_Origin_Flag[Axis_Num + Ext_Axis_Num] = {FALSE};	//��е����ָ���־λ
u8 Program_Axis_Origin_Speed[Axis_Num + Ext_Axis_Num] = {FALSE};//��е����ָ���ٶ�

u16 Action_Step_List_Num=0;					  		//��������ָ����
u16 Action_Step_Run_Num=0;					  		//�Ѿ����е�ָ������
u16 Action_Step_Confirm_Num=0;				  	//ȷ�ϵ�ָ������
u8  g_Start_ActionRun_Step = 0;				  	//�������ζ�������
u8  g_Reset_ActionRun_Step = 0;				  	//�������ζ�������

u16 SubAction_Step_List_Num[SAVEPROGRAMNUM_SUB] = {0};					//�ӳ��򵥴�����ָ����
u16 SubAction_Step_Run_Num[SAVEPROGRAMNUM_SUB] = {0};					  //�ӳ����Ѿ����е�ָ������
u16 SubAction_Step_Confirm_Num[SAVEPROGRAMNUM_SUB] = {0};				//�ӳ���ȷ�ϵ�ָ������

void SubProgramStepControl(u8 subProNum);
void ActionStepControl(void);
void ActionOverOperate(void);
void AutoPauseOperate(void);
void AutoStopOperate(void);
void StartActionControl(void);

/**************************************************************************************************
**  ��������  AutoModeControl()
**	�����������
**	�����������
**	�������ܣ��Զ�ģʽ���ƺ���
**	��ע��	  �����豸�Զ�����
**  ���ߣ�      
**  �������ڣ�
***************************************************************************************************/
void AutoModeControl(void)
{
	int i = 0;
	
	switch(g_AutoStatue)
	{
		case AUTO_WAITE://�ȴ�״̬�����������
			if(g_Auto_Order_Start == TRUE)
			{//���յ������Զ�����ָ��
				g_Auto_Order_Start = FALSE;		//��λ��־λ
				g_AutoStatue = AUTO_RUNNING;	//�����Զ�����״̬
				Program_RunTime = 0;
				for(i=0; i<USER_NUM; i++)
				{
					if(USER_Parameter.START_RESET[i] == TRUE)
					{
						USER_Parameter.CURR_Num[i] = USER_Parameter.INIT_Num[i];
					}
				}
				
				sMD_CurMDCode = 0;						//���㵱ǰ��������ŵ�����
			}
			break; 
		case AUTO_RUNNING://�Զ�������ز���
			if(g_Auto_Order_Pause == TRUE)		
			{//���������Զ�����ָ��
				g_Auto_Order_Pause = FALSE;		//��λ��־λ
				g_AutoStatue = AUTO_PAUSE;		//�ص��Զ�����״̬
				AutoPauseOperate();						//��ͣ������ز���		
				if(g_Program_Is_Debuging == TRUE)
				{
					g_AutoStatue = AUTO_RUNNING;
				}				
			}
			
			if(g_Auto_Order_Stop == TRUE)		
			{//�Զ�����ָֹͣ��
				g_Auto_Order_Stop = FALSE;		//��λ��־λ
				g_AutoStatue = AUTO_WAITE;		//�ص��Զ��ȴ�״̬
				AutoPauseOperate();						//��ͣ������ز���
				AutoStopOperate();						//ֹͣ������ز���
			}
			
			if(g_AutoStatue == AUTO_RUNNING)
			{//���г���
				if(g_Program_Is_Debuging == FALSE)
				{
					g_Write_FlashFlag = TRUE;
				}
				ActionStepControl();//���ж���������ƣ���Free_Program_Operate�ṹ����п��ƣ�

				for(i=0; i<SAVEPROGRAMNUM_SUB; i++)
				{
					if(g_SubProgram_Step_Run[i] == TRUE)
					{//�����ӳ���
						SubProgramStepControl(i);
					}
				}
			}
			break;
		case AUTO_PAUSE://��ͣת״̬����Ƿ�������ֹͣ
			if(g_Auto_Order_Start == TRUE)	//���������Զ�����ָ��
			{
				g_Auto_Order_Start = FALSE;		//��λ��־λ
				g_AutoStatue = AUTO_RUNNING;	//�ص��Զ�����״̬
			}
			if(g_Auto_Order_Stop == TRUE)		//�Զ�����ָֹͣ��
			{
				g_Auto_Order_Stop = FALSE;		//��λ��־λ
				g_AutoStatue = AUTO_WAITE;		//�ص��Զ��ȴ�״̬
				AutoStopOperate();						//ֹͣ������ز���
			}
			break;
		case AUTO_ERROR://�����⴦��
			g_Auto_ActionError_Flag=TRUE;
			g_Auto_Order_Stop = TRUE;
			if(g_Auto_Order_Stop == TRUE)		//�Զ�����ָֹͣ��
			{
				g_Auto_Order_Stop = FALSE;		//��λ��־λ
				g_AutoStatue = AUTO_WAITE;		//�ص��Զ��ȴ�״̬
				AutoPauseOperate();
				AutoStopOperate();						//ֹͣ������ز���
			}
			break;
		default:
			break;
	}		
}

/**************************************************************************************************
**  ��������  MainLogicComEndDeal()
**	�����������
**	�����������
**	�������ܣ��������߼�������ɺ�Ĵ���
**	��ע��	  ����ÿ�г��򵥶�����  
**  �������ڣ�
***************************************************************************************************/
void MainLogicComEndDeal(SaveProgram *Program_Operate, u8 ActionLine)
{
	u16 i = 0;
	u8 while_Counter = 0;							//ʵ��whileǶ��
	u8 if_else_Counter = 0;						//ʵ��if-elseǶ��
	
	switch(Program_Operate->Program[ActionLine].Key)
	{
		case K_JUMP://��ת����
						if(Program_Operate->Program[ActionLine].Value1 == V_JUMP_LABEL)
			{
				g_Auto_PresentLine = g_Auto_PresentLine+1;
				break;
			}
			if(ActionLine < m_JumpStepRunNum)
			{
				for(i=ActionLine+1; i<m_JumpStepRunNum; i++)
				{//������תʱ�����߼�����
					if(Program_Operate->Program[i].Key == K_WHILE)
					{
						m_WhileNC++;
						m_WhileLineNum[m_WhileNC - 1] = i;
						m_WhileRunFlag[m_WhileNC - 1] = 1;						//ѭ���������
					}
					else if(Program_Operate->Program[i].Key == K_CYCLEOVER)
					{
						m_WhileRunFlag[m_WhileNC - 1] = 0;						//ѭ���������
						m_WhileNC--;
					}
					
					if(Program_Operate->Program[i].Key == K_IF)
					{
						m_IfElseNC++;
						m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 1;
					}
					else if(Program_Operate->Program[i].Key == K_ELSE)
					{
						m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 1;
					}
					else if(Program_Operate->Program[i].Key == K_OVER)
					{
						m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 0;
						m_IfElseNC--;
					}
				}
				
				if(Program_Operate->Program[m_JumpStepRunNum].Key == K_ELSE)
				{//�����ת��else��Ҫ�����ִ�б�־
					m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 0;
				}
			}
			else if(m_JumpStepRunNum > 0 && ActionLine > m_JumpStepRunNum)
			{
				for(i=ActionLine-1; i>m_JumpStepRunNum; i--)
				{//������תʱ�����߼�����
					if(Program_Operate->Program[i].Key == K_WHILE)
					{
						m_WhileRunFlag[m_WhileNC - 1] = 0;						//ѭ���������
						m_WhileNC--;
					}
					else if(Program_Operate->Program[i].Key == K_CYCLEOVER)
					{
						m_WhileNC++;
						m_WhileRunFlag[m_WhileNC - 1] = 1;						//ѭ���������
					}
					
					if(Program_Operate->Program[i].Key == K_IF)
					{
						m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 0;
						m_IfElseNC--;
					}
					else if(Program_Operate->Program[i].Key == K_ELSE)
					{
						m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 0;
					}
					else if(Program_Operate->Program[i].Key == K_OVER)
					{
						m_IfElseNC++;
						m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 1;
					}
				}
				
				if(Program_Operate->Program[m_JumpStepRunNum].Key == K_ELSE)
				{//�����ת��else��Ҫ�����ִ�б�־
					m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 0;
				}
			}
			g_Auto_PresentLine = m_JumpStepRunNum;
			break;
		case K_WHILE://ѭ������
			if(m_WhileJudgeRes[m_WhileNC - 1] == 1)
			{//�жϳɹ�
				m_WhileJudgeRes[m_WhileNC - 1] = 0;
				g_Auto_PresentLine = g_Auto_PresentLine + 1;
			}
			else
			{
				m_WhileRunFlag[m_WhileNC - 1] = 0;						//ѭ���������
				for(i=ActionLine+1; i<Program_Operate->Num; i++)
				{//������ǰwhile��Ӧ��ѭ����������
					if(Program_Operate->Program[i].Key == K_WHILE)
					{
						while_Counter++;
					}
					else if(Program_Operate->Program[i].Key == K_CYCLEOVER)
					{
						if(while_Counter > 0)
						{
							while_Counter--;
						}
						else
						{
							m_WhileNC--;
							g_Auto_PresentLine = i + 1;
							break;
						}
					}
				}
				if(i == Program_Operate->Num)
				{//δ������ѭ������������ָ�����һ�г���
					g_Auto_PresentLine = Program_Operate->Num - 1;
					m_WhileNC--;
				}
			}
			break;
		case K_CYCLEOVER://ѭ����������
			g_Auto_PresentLine = m_WhileLineNum[m_WhileNC - 1];
			m_WhileNC--;
			break;
		case K_IF://IF�ж���������
		case K_ELSE://ELSE�ж���������
			if(m_IfElseJudgeRes[m_IfElseNC - 1] == 1 && m_IfElseJudgeRunFlag[m_IfElseNC - 1] == 0)
			{
				m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 1;
				g_Auto_PresentLine = g_Auto_PresentLine + 1;
			}
			else
			{
				m_IfElseJudgeRes[m_IfElseNC - 1] = 0;
				for(i=ActionLine+1; i<Program_Operate->Num; i++)
				{//������ǰIF-ELSE��Ӧ������
					if(Program_Operate->Program[i].Key == K_IF)
					{//���IF-ELSE���δִ�й����ȶ����µ�ELSE����ô��ִ�����е��ж�
						if_else_Counter++;
					}
					else if(if_else_Counter == 0 && Program_Operate->Program[i].Key == K_ELSE && m_IfElseJudgeRunFlag[m_IfElseNC - 1] == 0)
					{//���IF-ELSE���δִ�й����ȶ����µ�ELSE����ô��ִ�����е��ж�
						g_Auto_PresentLine = i;
						break;
					}
					else if(Program_Operate->Program[i].Key == K_OVER)
					{//����ȶ�ȡ���жϽ���������ôִ����һ��
						if(if_else_Counter > 0)
						{
							if_else_Counter--;
						}
						else
						{
							m_IfElseNC--;
							g_Auto_PresentLine = i + 1;
							break;
						}
					}
				}
				if(i == Program_Operate->Num)
				{//δ�������жϽ���������ָ�����һ�г���
					m_IfElseNC = 0;
					g_Auto_PresentLine = Program_Operate->Num - 1;
				}
			}
			break;
		case K_OVER://�ж��������
			m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 0;
			m_IfElseNC--;
			g_Auto_PresentLine = g_Auto_PresentLine + 1;
			break;
		case K_SPECIAL://����ָ���
			if(Program_Operate->Program[ActionLine].Value1 == V_SUSPEND)
			{
				g_Auto_Order_Pause = TRUE;						//��ͣ������ز���
			}
			else if(Program_Operate->Program[ActionLine].Value1 == V_STOP)
			{
				g_Auto_Order_Stop = TRUE;							//ֹͣ������ز���
			}
			g_Auto_PresentLine = g_Auto_PresentLine + 1;
			break;
		default://��������������кŵĲ���
			g_Auto_PresentLine = g_Auto_PresentLine + Action_Step_Run_Num;
			break;
	}
}

/**************************************************************************************************
**  ��������  SubLogicComEndDeal()
**	�����������
**	�����������
**	�������ܣ��ӳ����߼�������ɺ�Ĵ���
**	��ע��	  ����ÿ�г��򵥶�����  
**  �������ڣ�
***************************************************************************************************/
void SubLogicComEndDeal(u8 subProNum, u8 ActionLine)
{
		u16 i = 0;
	u8 while_Counter = 0;							//ʵ��whileǶ��
	u8 if_else_Counter = 0;						//ʵ��if-elseǶ��
	
	switch(SubProgram_Operate[subProNum].Program[ActionLine].Key)
	{
		case K_JUMP://��ת����
			if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_JUMP_LABEL)
			{
				g_SubProgram_PresentLine[subProNum] = g_SubProgram_PresentLine[subProNum]+1;
				break;
			}
			if(ActionLine < m_JumpSubStepRunNum[subProNum])
			{
				for(i=ActionLine+1; i<m_JumpSubStepRunNum[subProNum]; i++)
				{//������תʱ�����߼�����
					if(SubProgram_Operate[subProNum].Program[i].Key == K_WHILE)
					{
						m_WhileSubNC[subProNum]++;
						m_WhileSubLineNum[subProNum][m_WhileSubNC[subProNum] - 1] = i;
						m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] = 1;						//ѭ���������
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_CYCLEOVER)
					{
						m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] = 0;						//ѭ���������
						m_WhileSubNC[subProNum]--;
					}
					
					if(SubProgram_Operate[subProNum].Program[i].Key == K_IF)
					{
						m_IfElseSubNC[subProNum]++;
						m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_ELSE)
					{
						m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_OVER)
					{
						m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;
						m_IfElseSubNC[subProNum]--;
					}
				}
				
				if(SubProgram_Operate[subProNum].Program[m_JumpSubStepRunNum[subProNum]].Key == K_ELSE)
				{//�����ת��else��Ҫ�����ִ�б�־
					m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;
				}
			}
			else if(m_JumpSubStepRunNum[subProNum] > 0 && ActionLine > m_JumpSubStepRunNum[subProNum])
			{
				for(i=ActionLine-1; i>m_JumpSubStepRunNum[subProNum]; i--)
				{//������תʱ�����߼�����
					if(SubProgram_Operate[subProNum].Program[i].Key == K_WHILE)
					{
						m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] = 0;						//ѭ���������
						m_WhileSubNC[subProNum]--;
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_CYCLEOVER)
					{
						m_WhileSubNC[subProNum]++;
						m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] = 1;						//ѭ���������
					}
					
					if(SubProgram_Operate[subProNum].Program[i].Key == K_IF)
					{
						m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;
						m_IfElseSubNC[subProNum]--;
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_ELSE)
					{
						m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_OVER)
					{
						m_IfElseSubNC[subProNum]++;
						m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
				}
				
				if(SubProgram_Operate[subProNum].Program[m_JumpSubStepRunNum[subProNum]].Key == K_ELSE)
				{//�����ת��else��Ҫ�����ִ�б�־
					m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;
				}
			}
			g_SubProgram_PresentLine[subProNum] = m_JumpSubStepRunNum[subProNum];
			break;
		case K_WHILE://ѭ������
			if(m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] == 1)
			{
				m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 0;
				g_SubProgram_PresentLine[subProNum] = g_SubProgram_PresentLine[subProNum] + 1;
			}
			else
			{
				m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] = 0;						//ѭ���������
				for(i=ActionLine+1; i<SubProgram_Operate[subProNum].Num; i++)
				{//������ǰwhile��Ӧ��ѭ����������
					if(SubProgram_Operate[subProNum].Program[i].Key == K_WHILE)
					{
						while_Counter++;
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_CYCLEOVER)
					{
						if(while_Counter > 0)
						{
							while_Counter--;
						}
						else
						{
							m_WhileSubNC[subProNum]--;
							g_SubProgram_PresentLine[subProNum] = i + 1;
							break;
						}
					}
				}
				if(i == SubProgram_Operate[subProNum].Num)
				{//δ������ѭ������������ָ�����һ�г���
					g_SubProgram_PresentLine[subProNum] = SubProgram_Operate[subProNum].Num - 1;
					m_WhileSubNC[subProNum]--;
				}
			}
			break;
		case K_CYCLEOVER://ѭ����������
			g_SubProgram_PresentLine[subProNum] = m_WhileSubLineNum[subProNum][m_WhileSubNC[subProNum] - 1];
			m_WhileSubNC[subProNum]--;
			break;
		case K_IF://IF�ж���������
		case K_ELSE://ELSE�ж���������
			if(m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] == 1 && \
					m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] == 0)
			{
				m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
				g_SubProgram_PresentLine[subProNum] = g_SubProgram_PresentLine[subProNum] + 1;
			}
			else
			{
				m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;
				for(i=ActionLine+1; i<SubProgram_Operate[subProNum].Num; i++)
				{//������ǰIF-ELSE��Ӧ������
					if(SubProgram_Operate[subProNum].Program[i].Key == K_IF)
					{//���IF-ELSE���δִ�й����ȶ����µ�ELSE����ô��ִ�����е��ж�
						if_else_Counter++;
					}
					else if(if_else_Counter == 0 && SubProgram_Operate[subProNum].Program[i].Key == K_ELSE && \
										m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] == 0)
					{//���IF-ELSE���δִ�й����ȶ����µ�ELSE����ô��ִ�����е��ж�
						g_SubProgram_PresentLine[subProNum] = i;
						break;
					}
					else if(SubProgram_Operate[subProNum].Program[i].Key == K_OVER)
					{//����ȶ�ȡ���жϽ���������ôִ����һ��
						if(if_else_Counter > 0)
						{
							if_else_Counter--;
						}
						else
						{
							m_IfElseSubNC[subProNum]--;
							g_SubProgram_PresentLine[subProNum] = i + 1;
							break;
						}
					}
				}
				if(i == SubProgram_Operate[subProNum].Num)
				{//δ�������жϽ���������ָ�����һ�г���
					m_IfElseSubNC[subProNum] = 0;
					g_SubProgram_PresentLine[subProNum] = SubProgram_Operate[subProNum].Num - 1;
				}
			}
			break;
		case K_OVER://�ж��������
			m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;
			m_IfElseSubNC[subProNum]--;
			g_SubProgram_PresentLine[subProNum] = g_SubProgram_PresentLine[subProNum] + 1;
			break;
		case K_SPECIAL://����ָ���
			if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_SUSPEND)
			{
				g_Auto_Order_Pause = TRUE;						//��ͣ������ز���
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_STOP)
			{
				g_Auto_Order_Stop = TRUE;							//ֹͣ������ز���
			}
			g_SubProgram_PresentLine[subProNum] = g_SubProgram_PresentLine[subProNum] + 1;
			break;
		default://��������������кŵĲ���
			g_SubProgram_PresentLine[subProNum] = g_SubProgram_PresentLine[subProNum] + SubAction_Step_Run_Num[subProNum];
			break;
	}
}

/**************************************************************************************************
**  ��������  ActionStepControl()
**	�����������
**	�����������
**	�������ܣ����ж���������ƺ���
**	��ע��	  ����ÿ�г��򵥶�����  
**  �������ڣ�
***************************************************************************************************/
void ActionStepControl(void)
{
	u16 AutoPresentLine=0;//��ǰ��Ҫ�����к�
	u8 ActionSetResult = 0;//����ִ��״̬�Ĵ����
	u8 ioNum = 0;
	
	switch(g_Auto_ActionRun_Step)//����Stepȷ����Ӧ��ִ�л���
	{
		case 0:{		//�����������
			if(Single_Mode_Enable == ENABLE && g_Auto_Reset_Flag == FALSE)
			{
				Action_Step_List_Num = 1;//��������ָ��=1
			}
			else if(Action_Step_List_Num == 0)//�ǵ���ģʽ��û�й���������Ϣ��ʱ��
			{
				Action_Step_List_Num = AutoActionStepList(&Free_Program_Operate, g_Auto_PresentLine);	  //��ȡ�����е�����
				Action_Step_Run_Num = 0;											  																				//�����Ѿ�����������0
			}
			
			if(JXS_Parameter.SpeedLevel != Temp_JXS_Parameter_SpeedLevel)
			{//ʵ�ֻ�е���ٶȵȼ��˶��н����޸�
				JXS_Parameter.SpeedLevel = Temp_JXS_Parameter_SpeedLevel;
			}
			
			AutoPresentLine = g_Auto_PresentLine + Action_Step_Run_Num;												//��ǰ��Ҫ�����к�=�Զ����е��к�+����������
			ActionSetResult = AutoActionOutControl(&Free_Program_Operate, AutoPresentLine);		//��ȡִ�����
			
			g_Auto_ActionRun_Timer = 1;
			switch(ActionSetResult)//��Բ�ͬ������д���
			{
				case 0:		//����������ִ��
					 g_Auto_ActionRun_Step = 1;	//���붯��ȷ�ϻ���
					 Action_Step_List_Num=0;	//���������
					 Action_Step_Confirm_Num=0;	//
					 break;
				case 9:		//����ѭ����������������˲�����ش���������ת���������̽���
					 ActionOverOperate();
					 break;
				case 10:	//��ǰ�кų����쳣
					 g_AutoStatue = AUTO_ERROR;
					//�����Ӧ�쳣�����������뱨c����־λ
					 break;
				case 11:	//��Ҫָ�������쳣
					 g_AutoStatue = AUTO_ERROR;
					//�����Ӧ�쳣�����������뱨����־λ
					 break;
				case 12:	//����ָ�������쳣
					 g_AutoStatue = AUTO_ERROR;
					//�����Ӧ�쳣�����������뱨����־λ
					 break;
				case 13:	//���ָ�������쳣
					 g_AutoStatue = AUTO_ERROR;
					//�����Ӧ�쳣�����������뱨����־λ
					 break;
				case 14:	//IO����ָ�������쳣
					 g_AutoStatue = AUTO_ERROR;
					//�����Ӧ�쳣�����������뱨����־λ
					 break;
				case 15:
				   break;
				default:
					 break;
			}
		}break;
		case 1:{//����ȷ�ϻ���
			if(AutoActionOutConfirm(&Free_Program_Operate, g_Auto_PresentLine + Action_Step_Confirm_Num) == TRUE)//�ж϶���ȷ�����
			{//����ȷ�ϳɹ�
				Action_Step_Confirm_Num++;
				if(Action_Step_Confirm_Num == Action_Step_Run_Num)
				{//��������ִ����ɺ󣬽��붯��ȷ�ϻ���
					g_Auto_ActionRun_Step = 2;
					g_Auto_ActionRun_Timer = 0;
					Action_Step_Confirm_Num = 0;
					g_Auto_Valid_Timer = 0;
					g_Auto_Valid_Flag = FALSE;
				}
				
				if(g_Auto_ActionNcWait_Flag)//ȷ�ϳɹ���������볬ʱ����־
				{
					g_Auto_ActionNcWait_Flag = 0;
				}
			}
			else
			{//����ȷ�ϲ��ɹ�
				if(Free_Program_Operate.Program[g_Auto_PresentLine + Action_Step_Confirm_Num].Key == K_DELAY)//��ʱ����ɲ����г�ʱ���
				{}
				else if(Free_Program_Operate.Program[g_Auto_PresentLine + Action_Step_Confirm_Num].Key == K_SUBPROGRAM)//�ӳ�������ɲ����г�ʱ���
				{}
				else if(Free_Program_Operate.Program[g_Auto_PresentLine+Action_Step_Confirm_Num].Key == K_OUTDETECT)
				{
					ioNum = Free_Program_Operate.Program[g_Auto_PresentLine+Action_Step_Confirm_Num].Value1;
					if(g_Auto_ActionRun_Timer > Free_Program_Operate.Program[g_Auto_PresentLine+Action_Step_Confirm_Num].Value3)
					{//�ж������ⳬʱ
						Scan_TimeOut_OUTPUT(ioNum);	//����ת��Error.c����
						g_Auto_Order_Pause = TRUE;
						if(g_Program_Is_Debuging)
						{
							g_Auto_Order_Stop = TRUE;
						}
						
						g_Auto_ActionRun_Timer = 0;
					}
				}
				else if(g_Auto_ActionNcWait_Flag == 0)//�������ⳬʱ�ȴ�
				{
//					if(g_Auto_ActionRun_Timer > g_Auto_ActionTimeOut_Time)//��ǰ����ִ��ʱ��δ��ʱ����ֹ����ִ��ʱ�����
//					{
//						if(Free_Program_Operate.Program[g_Auto_PresentLine + Action_Step_Confirm_Num].Order == OR_IOORDER)//�ж���IOָ��
//						{//��ȡ���ڵ�IOָ�����Ӧ�ı��������ó������ձ�����Ϣ��								
//							Scan_TimeOut_IO(Free_Program_Operate.Program[g_Auto_PresentLine + Action_Step_Confirm_Num].Key);
//						}
//						else
//						{
//							g_Auto_ActionTimeOut_Flag = TRUE;
//						}
//						g_Auto_Order_Pause = TRUE;
//						g_Auto_ActionRun_Timer=0;
//						g_Auto_Valid_Timer = 0;
//						g_Auto_Valid_Flag = FALSE;
//					}
				}
				else
				{//�����ⳬʱ�ȴ�
					if(g_Auto_ActionRun_Timer > IO_Input_waittime[g_Auto_ActionNcWait_Flag-1])
					{//�ж������ⳬʱ
						if(Free_Program_Operate.Program[g_Auto_PresentLine+Action_Step_Confirm_Num].Order <= OR_AXISORDER)
						{
						}
						else
						{
							//���IO��ʱ��ʾ������Ϣ 20190524
							if(Free_Program_Operate.Program[g_Auto_PresentLine+Action_Step_Confirm_Num].Order == OR_IOORDER)//�ж���IOָ��
							{
								//��ȡ���ڵ�IOָ�����Ӧ�ı��������ó������ձ�����Ϣ��
								Scan_TimeOut_IO(Free_Program_Operate.Program[g_Auto_PresentLine+Action_Step_Confirm_Num].Key);	//����ת��Error.c����
							}
							else
							{
								g_Auto_ActionTimeOut_Flag=TRUE;
							}
							g_Auto_Order_Pause = TRUE;
							if(g_Program_Is_Debuging)
							{
								g_Auto_Order_Stop = TRUE;
							}
						}
						g_Auto_ActionRun_Timer=0;
						g_Auto_Valid_Timer = 0;
						g_Auto_Valid_Flag = FALSE;
					}
				}
			}
		}break;
		case 2:{    //������ʱ����
				if(Action_Step_Run_Num > 1 || AutoActionOutDelay(&Free_Program_Operate, g_Auto_PresentLine + (Action_Step_Run_Num-1)) == TRUE)
				{//ִֻ�в���������һ��������ʱ
					g_Auto_ActionRun_Step = 0;
					Action_Step_List_Num = 0;
					
					MainLogicComEndDeal(&Free_Program_Operate, g_Auto_PresentLine);//�����߼�����
					
					if(Single_Mode_Enable == ENABLE) 
					{//����ģʽ-������ͣ״̬
						g_Auto_Order_Pause = TRUE;	
					}
					
					if(g_Program_Is_Debuging)
					{
						g_Auto_Order_Stop = TRUE;
					}
				}
			}break;
			case 3:{		//��������
				g_Auto_ActionRun_Step = 0;
				Action_Step_List_Num = 0;
				if(Single_Mode_Enable == ENABLE) //����ģʽ-������ͣ״̬
				{
					g_Auto_Order_Pause = TRUE;	
				}
				if(g_Program_Is_Debuging)
				{
					g_Auto_Order_Stop = TRUE;
				}
				else
				{
					if(SC_Parameter.SC_Num == SC_Parameter.RW_Num && Program_Reset == FALSE)//��ǰ�ӹ��������
					{
						g_Auto_Order_Pause = TRUE;
						g_Auto_WorkFinished_Flag=TRUE;
					}
					else if(SC_Parameter.CJ_Num >= SC_Parameter.JG_Num && SC_Parameter.JG_Num != 0 && Program_Reset == FALSE)//��ǰ�ӹ�������� lin
					{
						g_Auto_Order_Pause = TRUE;
						g_Auto_CJWorkFinished_Flag = TRUE;
					}				
				}		
			}break;
			default:
				break;
	}
}

/**************************************************************************************************
**  ��������  SubProgramActionControl(u8 subProNum)
**	���������subProNum ִ�е��ӳ�����
**	�����������
**	�������ܣ��ӳ������п��ƺ������ӳ���֧�ֲ������ִ��
**	��ע��	  
**  ���ߣ�       
**  �������ڣ�
***************************************************************************************************/
void SubProgramActionControl(u8 subProNum)
{
	u8 SubProgramActionSetResult = 0;
	u16 AutoPresentLine = 0;
	u8 ioNum = 0;
	
	switch(g_SubProgram_ActionRun_Step[subProNum])
	{
		case 0:{		//�����������

			SubAction_Step_List_Num[subProNum] = AutoActionStepList(&SubProgram_Operate[subProNum], g_SubProgram_PresentLine[subProNum]);	  //��ȡ�����е�����

			if(SubAction_Step_List_Num[subProNum] == 1)//������
			{
				SubAction_Step_Run_Num[subProNum] = 0;
			}
			
			AutoPresentLine = g_SubProgram_PresentLine[subProNum] + SubAction_Step_Run_Num[subProNum];		//��ǰ��Ҫ�����к�=�Զ����е��к�+����������
			SubProgramActionSetResult = SubProgramActionOutControl(subProNum, AutoPresentLine);						//�˴������������������һ��
			g_SubProgram_ActionRun_Timer[subProNum] = 1;
			switch(SubProgramActionSetResult)
			{//case����������1~8��Ϊͬʱִ�е�������Ԥ��
				case 0:		//����������ִ��
					g_SubProgram_ActionRun_Step[subProNum] = 1;
					SubAction_Step_List_Num[subProNum] = 0;	//���������
					SubAction_Step_Confirm_Num[subProNum] = 0;
					break;
				case 9:		//����ѭ���������ӳ�����Ҫ���г��������Ĳ�����ش���
					SubAction_Step_List_Num[subProNum] = 0;					//�ӳ��򵥴�����ָ����	���ӳ�������0617
					SubAction_Step_Run_Num[subProNum] = 0;					  //�ӳ����Ѿ����е�ָ������
					SubAction_Step_Confirm_Num[subProNum] = 0;				//�ӳ���ȷ�ϵ�ָ������
					g_SubProgram_ActionRun_Step[subProNum] = 0;							
					break;
				case 10:	//��ǰ�кų����쳣
					g_AutoStatue = AUTO_ERROR;
					//�����Ӧ�쳣�����������뱨c����־λ
					break;
				case 11:	//��Ҫָ�������쳣
					g_AutoStatue = AUTO_ERROR;
					//�����Ӧ�쳣�����������뱨����־λ
					break;
				case 12:	//����ָ�������쳣
					g_AutoStatue = AUTO_ERROR;
					//�����Ӧ�쳣�����������뱨����־λ
					break;
				case 13:	//���ָ�������쳣
					g_AutoStatue = AUTO_ERROR;
					//�����Ӧ�쳣�����������뱨����־λ
					break;
				case 14:	//IO����ָ�������쳣
					g_AutoStatue = AUTO_ERROR;
					//�����Ӧ�쳣�����������뱨����־λ
					break;
				case 15:	//�ӳ���������
					g_SubProgram_ActionRun_Step[subProNum] = 3;
					break;
				default:
					break;
			}
		}break;
		case 1:{//����ȷ�ϻ���			
			if(SubProgramActionOutConfirm(subProNum, g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]) == TRUE)		
			{//�˴�����ȷ�Ϻ���Ҳ������һ��
				SubAction_Step_Confirm_Num[subProNum]++;
				if(SubAction_Step_Confirm_Num[subProNum] == SubAction_Step_Run_Num[subProNum])
				{
					g_SubProgram_ActionRun_Step[subProNum] = 2;
					g_SubProgram_ActionRun_Timer[subProNum] = 0;
					SubAction_Step_Confirm_Num[subProNum] = 0;
					g_SubAuto_Valid_Timer[subProNum] = 0;
					g_SubAuto_Valid_Flag[subProNum] = FALSE;
				}
				if(g_Auto_SubActionNcWait_Flag[subProNum])//ȷ�ϳɹ�����������ⳬʱ��־
				{
					g_Auto_SubActionNcWait_Flag[subProNum] = 0;
				}
			}
			else		//����ȷ�ϲ��ɹ�
			{
				if(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Key == K_DELAY)//��ʱ����ɲ����г�ʱ���
				{}
				else if(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Key == K_SUBPROGRAM)//�ӳ�������ɲ����г�ʱ���
				{}
				else if(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Key == K_OUTDETECT)
				{
					ioNum = SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Value1;
					if(g_SubProgram_ActionRun_Timer[subProNum] > SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Value3)
					{//�ж������ⳬʱ
						Scan_TimeOut_OUTPUT(ioNum);	//����ת��Error.c����
						g_Auto_Order_Pause = TRUE;
						if(g_Program_Is_Debuging)
						{
							g_Auto_Order_Stop = TRUE;
						}
						
						g_SubProgram_ActionRun_Timer[subProNum] = 0;
					}
				}
				else if(g_Auto_SubActionNcWait_Flag[subProNum] == 0)//�������ⳬʱ�ȴ�
				{
//					if(g_SubProgram_ActionRun_Timer[subProNum] > g_SubProgram_ActionTimeOut_Time[subProNum])	//����ִ�г�ʱ�ж�
//					{
//						if(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Order == OR_IOORDER)//�ж���IOָ��
//						{
//							//��ȡ��Ӧ��IOָ�����Ӧ�ı��������ó������ձ�����Ϣ��
//							Scan_TimeOut_IO(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Key);
//						}
//						else
//						{
//							g_Auto_ActionTimeOut_Flag = TRUE;
//						}
//						g_Auto_Order_Pause = TRUE;
//						g_SubProgram_ActionRun_Timer[subProNum] = 0;
//						g_SubAuto_Valid_Timer[subProNum] = 0;
//						g_SubAuto_Valid_Flag[subProNum] = FALSE;
//					}
				}
				else//�����ⳬʱ�ȴ�
				{//�ж������ⳬʱ�ȴ�
					if(g_SubProgram_ActionRun_Timer[subProNum] > IO_Input_waittime[g_Auto_SubActionNcWait_Flag[subProNum] - 1])
					{
						if(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Order <= OR_AXISORDER)
						{
						}
						else
						{
							//���IO��ʱ��ʾ������Ϣ 20190524
							if(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum]].Order == OR_IOORDER)//�ж���IOָ��
							{
								//��ȡ���ڵ�IOָ�����Ӧ�ı��������ó������ձ�����Ϣ��
								Scan_TimeOut_IO(SubProgram_Operate[subProNum].Program[g_SubProgram_PresentLine[subProNum] + SubAction_Step_Confirm_Num[subProNum]].Key);
							}
							else
							{
								g_Auto_ActionTimeOut_Flag = TRUE;
							}
							g_Auto_Order_Pause = TRUE;
							if(g_Program_Is_Debuging)
							{
								g_Auto_Order_Stop = TRUE;
							}
						}
//						g_Auto_ActionRun_Timer=0;
						g_SubProgram_ActionRun_Timer[subProNum] = 0;
						g_SubAuto_Valid_Timer[subProNum] = 0;
						g_SubAuto_Valid_Flag[subProNum] = FALSE;
					}
				}
			}
		}break;
		case 2:{		//������ʱ����
			if(SubAction_Step_Run_Num[subProNum] > 1 || SubProgramActionOutDelay(subProNum, g_SubProgram_PresentLine[subProNum] + SubAction_Step_Run_Num[subProNum] - 1) == TRUE)
			{//����в���ִ�еĳ��򣬾Ͳ�ִ�к������ʱ
				g_SubProgram_ActionRun_Step[subProNum] = 0;
				SubAction_Step_List_Num[subProNum] = 0;							//���������
				SubAction_Step_Confirm_Num[subProNum] = 0;
				
				SubLogicComEndDeal(subProNum, g_SubProgram_PresentLine[subProNum]);				//�߼��������
				
				SubAction_Step_Run_Num[subProNum] = 0;
			}
		}break;
		case 3:{		//��������
			g_SubProgram_ActionRun_Step[subProNum] = 0;
			g_SubProgram_PresentLine[subProNum] = 0;
			SubAction_Step_Confirm_Num[subProNum] = 0;
			SubAction_Step_Run_Num[subProNum] = 0;
			SubAction_Step_List_Num[subProNum] = 0;			
			if(g_SubProgram_ContralEnd[subProNum] == TRUE)
			{
				g_SubProgram_Start[subProNum] = FALSE;
				g_SubProgram_Finish[subProNum] = TRUE;
				g_SubProgram_ContralEnd[subProNum] = FALSE;
			}
		}break;
		default:
			break;
	}
}

/**************************************************************************************************
**  ��������  SubProgramStepControl(u8 subProNum)
**	���������subProNum �ӳ����� 
**	�����������
**	�������ܣ�ִ���ӳ���
**	��ע��	  
**  ���ߣ�       
**  �������ڣ�
***************************************************************************************************/
void SubProgramStepControl(u8 subProNum)
{
	if(g_SubProgram_Start[subProNum] == TRUE)
	{
		SubProgramActionControl(subProNum);			//��SubProgram_Operate�ṹ�����
	}
}

/**************************************************************************************************
**  ��������  ActionOverOperate()
**	�����������
**	�����������
**	�������ܣ�ѭ����������������
**	��ע��	  һ�����ڽ��������ݻָ�����
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void ActionOverOperate(void)
{
//	u8 Temp_data[30]={0};
	g_Auto_PresentLine = 0;
	Action_Step_List_Num=0;
	Action_Step_Run_Num=0;
	Action_Step_Confirm_Num=0;
	//дIIC,�ۼƲ����Լ���������
	if((g_Program_Is_Debuging == FALSE) && (Program_Reset == FALSE))
	{
		SC_Parameter.SC_Num++;
		SC_Parameter.LJ_Num++;
				
		if(SC_Parameter.JG_Num!=0)
		{
			SC_Parameter.CJ_Num++;	
		}
		if(SC_Parameter.LJ_Num>MINROBOTPOSITION)
		{
			SC_Parameter.LJ_Num=0;
		}
//		Temp_data[0] = SC_Parameter.RW_Num;
//		Temp_data[1] = SC_Parameter.RW_Num>>8;
//		Temp_data[2] = SC_Parameter.RW_Num>>16;
//		Temp_data[3] = SC_Parameter.RW_Num>>24;
//		Temp_data[4] = SC_Parameter.CJ_Num;
//		Temp_data[5] = SC_Parameter.CJ_Num>>8;
//		Temp_data[6] = SC_Parameter.CJ_Num>>16;
//		Temp_data[7] = SC_Parameter.CJ_Num>>24;
//		Temp_data[8] = SC_Parameter.JG_Num;
//		Temp_data[9] = SC_Parameter.JG_Num>>8;
//		Temp_data[10] = SC_Parameter.JG_Num>>16;
//		Temp_data[11] = SC_Parameter.JG_Num>>24;
//		Temp_data[12] = SC_Parameter.SC_Num;
//		Temp_data[13] = SC_Parameter.SC_Num>>8;
//		Temp_data[14] = SC_Parameter.SC_Num>>16;
//		Temp_data[15] = SC_Parameter.SC_Num>>24;
//		Temp_data[16] = SC_Parameter.LJ_Num;
//		Temp_data[17] = SC_Parameter.LJ_Num>>8;
//		Temp_data[18] = SC_Parameter.LJ_Num>>16;
//		Temp_data[19] = SC_Parameter.LJ_Num>>24;
//		Temp_data[20] = SC_Parameter.NG_Num;
//		Temp_data[21] = SC_Parameter.NG_Num>>8;
//		Temp_data[22] = SC_Parameter.NG_Num>>16;
//		Temp_data[23] = SC_Parameter.NG_Num>>24;
//		W25QXX_Write(Temp_data,P_SC_NUM_ADDRESS,24);
	}
	g_Auto_ActionRun_Step = 3;
}

/**************************************************************************************************
**  ��������  AutoPauseOperate()
**	�����������
**	�����������
**	�������ܣ��Զ���ͣ����
**	��ע��	  ��ͣ�������е�ȫ�Զ�������������ر�־λ
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void AutoPauseOperate(void)
{	
	u16 i = 0;
	
	if(AxisMoveFlag[X_Axsis] == ENABLE)		//������ڷ�������
	{
		Servo_Stop(X_Axsis);
	}
	if(AxisMoveFlag[Z_Axsis] == ENABLE)		//������ڷ�������
	{
		Servo_Stop(Z_Axsis);
	}
	if(AxisMoveFlag[L_Axsis] == ENABLE)		//������ڷ�������
	{
		Servo_Stop(L_Axsis);
	}
	if(AxisMoveFlag[O_Axsis] == ENABLE)		//������ڷ�������
	{
		Servo_Stop(O_Axsis);
	}
	if(AxisMoveFlag[U_Axsis] == ENABLE)		//������ڷ�������
	{
		Servo_Stop(U_Axsis);
	}
	if(AxisMoveFlag[V_Axsis] == ENABLE)		//������ڷ�������
	{
		Servo_Stop(V_Axsis);
	}
	
	for(i=0;i<OUTPUT_NUM;i++)
	{
		if(OutPut_Pause[i]==PAUSE_Select)
		{
			SetOutput(i);//ָʾ����
		}
	}	
	Action_Step_List_Num = 0;
	Action_Step_Run_Num = 0;
	Action_Step_Confirm_Num = 0;
	g_Auto_ActionRun_Step = 0;
	g_Auto_Valid_Timer = 0;
	g_Auto_Valid_Flag = FALSE;
	g_ActionDelay_Step = 0;
	for(i=0; i<SAVEPROGRAMNUM_SUB; i++)
	{
		g_SubProgram_ActionRun_Step[i] = 0;
		
		SubAction_Step_List_Num[i] = 0;					//�ӳ��򵥴�����ָ����	���ӳ�������0617
		SubAction_Step_Run_Num[i] = 0;					  //�ӳ����Ѿ����е�ָ������
		SubAction_Step_Confirm_Num[i] = 0;				//�ӳ���ȷ�ϵ�ָ������
		g_SubProgram_ActionRun_Step[i] = 0;
		g_SubAuto_Valid_Timer[i] = 0;
		g_SubAuto_Valid_Flag[i] = FALSE;
		g_SubProgramDelay_Step[i] = 0;
	}
	g_Reset_ActionRun_Step = 0;
	if(g_Write_FlashFlag == TRUE && Auto_Mode != SINGLE_MODE)
	{
		STMFLASH_WriteRunData();
	}
}

/**************************************************************************************************
**  ��������  AutoStopOperate()
**	�����������
**	�����������
**	�������ܣ�����ִ��ǰ���Ϸ����ж�
**	��ע��	  ֹͣȫ�Զ����ж���������������λ��־λ
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void AutoStopOperate(void)
{
	u16 i = 0;
//	u8 Temp_data[8]={0};
	
	for(i=0;i<OUTPUT_NUM;i++)
	{
		if(OutPut_Stop[i]==STOP_Select)
		{
			SetOutput(i);//ָʾ����
		}
	}

	if(g_Program_Is_Debuging)
	{
		g_Program_Is_Debuging = FALSE;
		g_AutoStatue = AUTO_WAITE;
	}
	else
	{
		
		if(g_Auto_PresentLine != 0 && g_Auto_PresentLine != Free_Program_Operate.Num-1)
		{
			SC_Parameter.NG_Num++;
		}
		g_AutoStatue = AUTO_WAITE;
		g_Auto_Order_Start = FALSE;
		g_Auto_Order_Pause = FALSE;
		g_Auto_Order_Stop = FALSE;
		g_Auto_PresentLine = 0;
		Single_Mode_Enable = DISABLE;
		Loop_Mode_Enable = DISABLE;
		Once_Mode_Enable = DISABLE;
		g_Auto_ActionTimeOut_Flag = FALSE;
//		Puls_Delay_Num = 0;
		Action_Step_List_Num = 0;
		Action_Step_Run_Num = 0;
		Action_Step_Confirm_Num = 0;
		g_Auto_Valid_Timer = 0;
		g_Auto_Valid_Flag = FALSE;
		for(i=0; i<SAVEPROGRAMNUM_SUB; i++)
		{
			g_Read_SubProgram[i] = FALSE;
			g_SubProgram_PresentLine[i] = 0;
			g_SubProgram_Step_Run[i] = FALSE;
			g_SubProgram_Start[i] = FALSE;
			g_SubProgram_Finish[i] = FALSE;
			
			SubAction_Step_List_Num[i] = 0;					//�ӳ��򵥴�����ָ����	���ӳ�������0617
			SubAction_Step_Run_Num[i] = 0;					  //�ӳ����Ѿ����е�ָ������
			SubAction_Step_Confirm_Num[i] = 0;				//�ӳ���ȷ�ϵ�ָ������	
			g_SubProgram_ActionRun_Step[i] = 0;
			g_SubAuto_Valid_Timer[i] = 0;
			g_SubAuto_Valid_Flag[i] = FALSE;
		}
		
		m_ProRunTimeTotalCount =  0;
		
		for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
		{
			Flag_Keep_Move[i] = 0;
			Increment_Finished[i] = FALSE;
			if(Program_Axis_Origin_Flag[i] == TRUE)
			{
				Origin_Backed = FALSE;
				Program_Axis_Origin_Flag[i] = FALSE;
			}
			SlowPointFlag[i] = 0;
		}
		
		g_ActionDelay_Step = 0;
		g_Auto_ActionNcWait_Flag = 0;
		for(i=0; i<SAVEPROGRAMNUM_SUB; i++)
		{
			g_Auto_SubActionNcWait_Flag[i] = 0;
			g_SubProgramDelay_Step[i] = 0;
		}
	}
	
	if(g_Write_FlashFlag == TRUE)
	{
  	STMFLASH_WriteRunData();
	}
}


/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/
