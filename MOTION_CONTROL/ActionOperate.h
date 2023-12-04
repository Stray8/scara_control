/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : AutoOperate.h
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 28/10/2015
* Description        : ����������������ͷ�ļ�
***************************************************************************************/
#ifndef _AutoOperate_h_
#define _AutoOperate_h_
#include "Parameter.h"

extern u8  g_Key_Delay_Flag ;
extern u32 g_Key_Delay_Timer;
extern u8  g_SubProgram_Key_Delay_Flag[SAVEPROGRAMNUM_SUB];
extern u32 g_SubProgram_Key_Delay_Timer[SAVEPROGRAMNUM_SUB];
extern u8  g_ActionDelay_Step;
extern u32 g_ActionDelay_Timer;
extern u8  g_SubProgramDelay_Step[SAVEPROGRAMNUM_SUB];
extern u32 g_SubProgramDelay_Timer[SAVEPROGRAMNUM_SUB];
extern s32 SlowPointIncrement[Axis_Num + Ext_Axis_Num];							//���ٶȵ�����λ��
extern s32 SlowPointSpeed[Axis_Num + Ext_Axis_Num];									//���ٶȵ��ٶ�
extern u8 SlowPointFlag[Axis_Num + Ext_Axis_Num];										//������ٶȵ��־
extern s32 AdvancePointInc[Axis_Num + Ext_Axis_Num];			   				//��ǰȷ�ϵ���
extern u8  AdvancePointFlag[Axis_Num + Ext_Axis_Num];			  				//��ǰȷ�ϵı�־
extern u8  WaitAxisMoveFlag[Axis_Num + Ext_Axis_Num];			  				//�ȴ�����ɶ�����־

extern u32 m_PulseOutputStartTime[OUTPUT_NUM];											//������������Ӧ�Ŀ�ʼʱ��
extern u32 m_PulseOutputEndTime[OUTPUT_NUM];												//������������Ӧ�Ľ���ʱ��
extern u8 m_PulseOutputSta[OUTPUT_NUM];															//������������Ӧ����λ��λ

extern u8 Axsis_MoveProNum[Axis_Num + Ext_Axis_Num];								//�������˶����̱߳��
extern s32 Axsis_MoveTarPos[Axis_Num + Ext_Axis_Num];								//�������˶���Ŀ��λ��
extern u32 Axsis_MoveTarSpeed[Axis_Num + Ext_Axis_Num];							//�������˶����ٶ�

extern u32 Program_RunTime ;																				//���򵥴�����ʱ��
extern u32 Program_RunTime_Count;																		//���򵥴�����ʱ�������
extern u32 m_ProRunTimeTotal;											    							//�����ܵ�����ʱ��
extern u32 m_ProRunTimeTotalCount;									  							//�����ܵ�����ʱ�������
extern u32 m_PreProRunTimeCumulate;								    							//��ǰ�����ۼ�����ʱ��
extern u32 m_PowerOnTimeTotal;											  							//�豸�ܵ��˿�����
extern u32 m_PowerOnTimeTotalCount;								    							//�豸�ܵĿ���ʱ�������
extern u32 m_PowerOnTimeCumulate;									   								//�豸�ܵ��ۼƿ���ʱ��
extern u32 m_PrPowerOnTimeCumulate;                   							//��ǰ�豸�ܵ��ۼƿ���ʱ��
extern u32 m_ProRunTimeCumulate;										  							//����˴��ۼ�����ʱ��

extern u8 Program_Reset;					  																//��λ������ɱ�־
extern u8 Increment_Finished[Axis_Num + Ext_Axis_Num];

extern u8 Key_Delay(u32);
extern u8 SubProgram_Key_Delay(u8, u32);
extern u16 AutoActionStepList(SaveProgram *Program_Operate, u16 ActionLine);
extern u8 AutoActionOutControl(SaveProgram *Program_Operate, u8 ActionLine);
extern u8 AutoActionOutConfirm(SaveProgram *Program_Operate, u8 ActionLine);
extern u8 SubProgramActionOutControl(u8 subProNum, u8 ActionLine);
extern u8 SubProgramActionOutConfirm(u8 subProNum,u8 ActionLine);
extern u8 AutoActionOutDelay(SaveProgram *Program_Operate,u8 ActionLine);
extern u8 ActionAllowJudge(SaveProgram *Program_Operate,u8 ActionLine);
extern u8 SubActionAllowJudge(u8 subProNum, u8 ActionLine);
extern u8 AXisMove(u8 Axsis, s32 Axsis_Position, u32 Axsis_Speed);
extern u16 Servo_MoveFinishSta(u8, s32);
//extern void KeepAXisMove(u8, u8, u32);
extern u8 SubProgramActionOutDelay(u8 subProNum,u8 ActionLine);

extern u8 Axis_To_ID(u8 Axis);
extern void ServoAccDecSet(u8 );
//extern void ServoAccDecSet_CSV(u8 Axsis);
extern void ServoAccDecSet_CST(u8 Axsis);
extern u8 AXisSncyMove(u8 procNum);

#endif

/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/

