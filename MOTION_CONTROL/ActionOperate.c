/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : AutoOperate.c
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 28/10/2015
* Description        : ����������������
***************************************************************************************/
#include "stm32f4xx.h"
#include "Auto_2.h"
#include "StatusControl.h"
#include "Auto.h"
#include "in.h"
#include "out.h"
#include "SpeedControl.h" 
#include "w25qxx.h"
#include "ActionOperate.h"
#include "Error.h"
#include "SignalWatch.h"
#include "JDZ.h"
#include "BackToOrigin.h"
#include "MD.h"
#include "CANopen.h"
#include "math.h" 
#include <stdio.h> 
#include <stdlib.h>

u8  g_Key_Delay_Flag = FALSE;
u32 g_Key_Delay_Timer = 0;
u8  g_ActionDelay_Step = 0;
u32	g_ActionDelay_Timer = 0;

u8  g_SubProgram_Key_Delay_Flag[SAVEPROGRAMNUM_SUB] = {FALSE};
u32 g_SubProgram_Key_Delay_Timer[SAVEPROGRAMNUM_SUB] = {0};
u8  g_SubProgramDelay_Step[SAVEPROGRAMNUM_SUB] = {0};
u32	g_SubProgramDelay_Timer[SAVEPROGRAMNUM_SUB] = {0};

u32 Program_RunTime = 0;									      //���򵥴�����ʱ��
u32 Program_RunTime_Count = 0;						      //���򵥴�����ʱ�������
u32 m_ProRunTimeTotal = 0;											//�����ܵ�����ʱ��
u32 m_ProRunTimeTotalCount = 0;									//�����ܵ�����ʱ�������
u32 m_ProRunTimeCumulate = 0;										//����˴��ۼ�����ʱ��
u32 m_PreProRunTimeCumulate = 0;								//��ǰ�����ۼ�����ʱ��
u32 m_PowerOnTimeTotal = 0;											//�豸�ܵĿ�������
u32 m_PowerOnTimeTotalCount = 0;								//�豸�ܵĿ���ʱ�������
u32 m_PowerOnTimeCumulate = 0;									//�豸�ܵ��ۼƿ���ʱ��
u32 m_PrPowerOnTimeCumulate = 0;                //��ǰ�豸�ܵ��ۼƿ���ʱ��

u8  Program_Reset = FALSE;					  		      //��λ������ɱ�־

s32 Increment_Target[Axis_Num + Ext_Axis_Num] = {0};					//�����˶�ʱ���ڼ�¼Ŀ��λ��
u8  Increment_Finished[Axis_Num + Ext_Axis_Num] = {FALSE};		//�����˶�ָ����ɱ�־����ֹ�����˶���ʱ������ͣ�����������������ԭ�趨ֵ

s32 SlowPointIncrement[Axis_Num + Ext_Axis_Num] = {0};				//���ٶȵ�����λ��
s32 SlowPointSpeed[Axis_Num + Ext_Axis_Num] = {0};						//���ٶȵ��ٶ�
u8  SlowPointFlag[Axis_Num + Ext_Axis_Num] = {FALSE};					//������ٶȵ��־

s32 AdvancePointInc[Axis_Num + Ext_Axis_Num] = {0};			   		//��ǰȷ�ϵ���
u8  AdvancePointFlag[Axis_Num + Ext_Axis_Num] = {FALSE};			//��ǰȷ�ϵı�־

u8  WaitAxisMoveFlag[Axis_Num + Ext_Axis_Num] = {FALSE};			//�ȴ�����ɶ�����־

u32 m_PulseOutputStartTime[OUTPUT_NUM] = {0};									//������������Ӧ�Ŀ�ʼʱ��
u32 m_PulseOutputEndTime[OUTPUT_NUM] = {0};										//������������Ӧ�Ľ���ʱ��
u8  m_PulseOutputSta[OUTPUT_NUM] = {0};												//������������Ӧ����λ��λ

u8  Axsis_MoveProNum[Axis_Num + Ext_Axis_Num] = {0};					//�������˶����̱߳��
s32 Axsis_MoveTarPos[Axis_Num + Ext_Axis_Num] = {0};					//�������˶���Ŀ��λ��
u32 Axsis_MoveTarSpeed[Axis_Num + Ext_Axis_Num] = {0};				//�������˶����ٶ�

u8 Axsis_InterAxisNum[Axis_Num + Ext_Axis_Num] = {0};					//�岹����
u8 Axsis_InterAxisCount = 0;																	//�岹�����
u8 Axsis_InterAxisSpeed = 0;																	//�岹�յ��ٶ�
u8 Axsis_InterAxisMaxSpeed = 0;																//�岹����ٶ�

/**************************************************************************************************
**  ��������Axis_To_ID
**	���������Axis ����
**	���������
**	�������ܣ�����תΪCANOPEN����ID
**	��ע��	
**  ���ߣ�         
**  �������ڣ������ 
***************************************************************************************************/
u8 Axis_To_ID(u8 Axis)
{
	u8 Axis_ID = 0;
	
	switch(Axis)
	{
		case X_Axsis:
			Axis_ID = SERVO_NODE_ID_01_X;
			break;
		case L_Axsis:
			Axis_ID = SERVO_NODE_ID_02_L;
			break;
		case Z_Axsis:
			Axis_ID = SERVO_NODE_ID_03_Z;
			break;
		case O_Axsis:
			Axis_ID = SERVO_NODE_ID_04_O;
			break;
		case U_Axsis:
			Axis_ID = SERVO_NODE_ID_05_U;
			break;
		case V_Axsis:
			Axis_ID = SERVO_NODE_ID_06_V;
			break;
		default:
			break;
	}
	
	return Axis_ID;
}

/**************************************************************************************************
**  ��������  ServoAccDecSet()
**	���������Axsis����
**	�����������
**	�������ܣ����ƶ�
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void ServoAccDecSet(u8 Axsis)
{
	u32 accTemp = 0;
	u32 decTemp = 0;
	
	if(Axsis < Axis_Num)
	{
		accTemp = JXS_Parameter.Accelerate.Time[Axsis];
		decTemp = accTemp;
	}
	else
	{
		accTemp = ExtendAix_Parameter[Axsis - Axis_Num].E_AccTime;
		decTemp = accTemp;
	}
	
	switch(JDZ_Parameter.Server)
	{
		case 0://�̴����
			accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
		  decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
			break;
		case 1://�㴨���
			accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
		  decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
			break;
		case 2://���ŵ��
			accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
		  decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
			break;
		case 3://�������
			accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
		  decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
			break;
		case 4://�Žݵ��
			accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
		  decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
			break;
		case 5://̨����
			accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
		  decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
			break;
		case 6://�������
			accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
		  decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
			break;
		default:
			accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
		  decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
			break;
	}
	
	ServoAccDecSet_PDO(Axis_To_ID(Axsis), accTemp, decTemp);
}

///**************************************************************************************************
//**  ��������  ServoAccDecSet_CSV()
//**	���������Axsis����
//**	�����������
//**	�������ܣ�������ͬ���ٶ�ģʽʱ�ļ��ٶ�����
//**	��ע��	  ��
//**  ���ߣ�    
//**  �������ڣ�
//***************************************************************************************************/
//void ServoAccDecSet_CSV(u8 Axsis)
//{
//	u32 accTemp = 0;
//	u32 decTemp = 0;
//	
//	switch(JDZ_Parameter.Server)
//	{
//		case 0://�̴����
//			accTemp = 30000 * Axsis_ParVelChange;
//		  decTemp = 30000 * Axsis_ParVelChange;
//			break;
//		case 1://�㴨���
//			accTemp = 30000 * Axsis_ParVelChange;
//		  decTemp = 30000 * Axsis_ParVelChange;
//			break;
//		case 2://���ŵ��
//			accTemp = 30000 * Axsis_ParVelChange;
//		  decTemp = 30000 * Axsis_ParVelChange;
//			break;
//		case 3://�������
//			accTemp = 30000 * Axsis_ParVelChange;
//		  decTemp = 30000 * Axsis_ParVelChange;
//			break;
//		case 4://�Žݵ��
//			accTemp = 30000 * Axsis_ParVelChange;
//		  decTemp = 30000 * Axsis_ParVelChange;
//			break;
//		case 5://̨����
//			accTemp = 30000 * Axsis_ParVelChange;
//		  decTemp = 30000 * Axsis_ParVelChange;
//			break;
//		case 6://�������
//			accTemp = 30000 * Axsis_ParVelChange;
//		  decTemp = 30000 * Axsis_ParVelChange;
//			break;
//		default:
//			accTemp = 30000 * Axsis_ParVelChange;
//		  decTemp = 30000 * Axsis_ParVelChange;
//			break;
//	}
//	
//	ServoAccDecSet_PDO(Axis_To_ID(Axsis), accTemp, decTemp);
//}

void ServoAccDecSet_CST(u8 Axsis)
{
	u32 accTemp = 0;
	u32 decTemp = 0;
	
//	if(Axsis < Axis_Num)
//	{
//		accTemp = JXS_Parameter.Accelerate.Time[Axsis];
//		decTemp = accTemp;
//	}
//	else
//	{
//		accTemp = ExtendAix_Parameter[Axsis - Axis_Num].E_AccTime;
//		decTemp = accTemp;
//	}
//	
	switch(JDZ_Parameter.Server)
	{
		case 0://�̴����
			accTemp = 3000 * Axsis_ParVelChange;
		  decTemp = 3000 * Axsis_ParVelChange;
			break;
		case 1://�㴨���
			accTemp = 3000 * Axsis_ParVelChange;
		  decTemp = 3000 * Axsis_ParVelChange;
			break;
		case 2://���ŵ��
			accTemp = 3000 * Axsis_ParVelChange;
		  decTemp = 3000 * Axsis_ParVelChange;
			break;
		case 3://�������
			accTemp = 3000 * Axsis_ParVelChange;
		  decTemp = 3000 * Axsis_ParVelChange;
			break;
		case 4://�Žݵ��
			accTemp = 3000 * Axsis_ParVelChange;
		  decTemp = 3000 * Axsis_ParVelChange;
			break;
		case 5://̨����
			accTemp = 3000 * Axsis_ParVelChange;
		  decTemp = 3000 * Axsis_ParVelChange;
			break;
		case 6://�������
			accTemp = 3000 * Axsis_ParVelChange;
		  decTemp = 3000 * Axsis_ParVelChange;
			break;
		default:
			accTemp = 3000 * Axsis_ParVelChange;
		  decTemp = 3000 * Axsis_ParVelChange;
			break;
	}
	
	ServoAccDecSet_PDO(Axis_To_ID(Axsis), accTemp, decTemp);
}


/**************************************************************************************************
**  ��������  Key_Delay()
**	���������Delay_Time������ĳ�ʱʱ��
**	�����������
**	�������ܣ��������ָ��ִ�г�ʱʱ��
**	��ע��	  ��
**  ���ߣ�      
**  �������ڣ�
***************************************************************************************************/
u8 Key_Delay(u32 Delay_Time)
{
	if(g_Key_Delay_Timer >= Delay_Time)
	{
		g_Key_Delay_Flag = FALSE;
		g_Key_Delay_Timer = 0;
		return 1;	
	}
	return 0;
}

/**************************************************************************************************
**  ��������  SubProgram_Key_Delay()
**	���������subProNum�ӳ����� Delay_Time��ʱʱ��
**	�����������
**	�������ܣ��ӳ����ָ��ִ�г�ʱ����
**	��ע��	  ��
**  ���ߣ�       
**  �������ڣ�
***************************************************************************************************/
u8 SubProgram_Key_Delay(u8 subProNum, u32 Delay_Time)
{
	if(g_SubProgram_Key_Delay_Timer[subProNum] >= Delay_Time)
	{
		g_SubProgram_Key_Delay_Flag[subProNum] = FALSE;
		g_SubProgram_Key_Delay_Timer[subProNum] = 0;
		return 1;	
	}
	return 0;
}
		
/**************************************************************************************************
**  ��������GetIO_Value
** ���������IONum IO���  effect��1Ϊ��Ч�źţ�0Ϊ��Ч�ź� proType��0������ ��Ϊ0�ӳ����+1
** ���������1��0
** �������ܣ���ȡ����ֵ
** ��ע��   ���ų���ʱ���͵�ƽ�����źţ�����1���ߵ�ƽ����0�����ų���ʱ���෴
**  ���ߣ�         
**  �������ڣ������
***************************************************************************************************/
u8 GetIO_Value(u8 ioNum, u8 effect, u8 proType)
{
	u8 result = 0;
	u8 index = ioNum / 8;
	u8 offset = ioNum % 8;
	if(effect==0)
	{//��Ч�ź�
		if(IO_Sign_On_Off[ioNum] == 0)
		{//����
			if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
			{
				result = 0;
			}
			else
			{
				result = 1;
			}
		}
		else	
		{//����
			if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
			{
				result = 1;
			}
			else
			{
				result = 0;
			}
		}		 
	}
	else if(effect==1)
	{//��Ч�ź�
		if(IO_Sign_On_Off[ioNum] == 0)
		{//����
			if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
			{
				result = 1;
			}
			else
			{
				result = 0;
			}
		}
		else	
		{//����
			if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
			{
				result = 0;
			}
			else
			{
				result = 1;
			}
		}
	}
	else if(effect==2)
	{//�½��أ�����
		if((proType == 0 && Detect_Falling_Edge == 1) \
				|| (proType > 0 && Detect_Falling_Edge_Sub[proType - 1] == 1))
		{//һ��ʼ��⵽�͵�ƽ
			if(((Input_Detect_Status[index] >> offset) & 0x01) == 0)
			{//��⵽�ߵ�ƽ
				if(proType == 0)
				{
					Detect_Falling_Edge = 3;
				}
				else
				{
					Detect_Falling_Edge_Sub[proType - 1] = 3;
				}
			}
		}
		if((proType == 0 && (Detect_Falling_Edge == 2 || Detect_Falling_Edge == 3)) \
				|| (proType > 0 && (Detect_Falling_Edge_Sub[proType - 1] == 2 || Detect_Falling_Edge_Sub[proType - 1] == 3)))
		{//һ��ʼ��⵽�ߵ�ƽ,�����ٴμ�⵽��
			if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
			{//��⵽�͵�ƽ,ֹͣ
				result = 1;
			}
		}
	}
	else if(effect==3)
	{//�����أ�����
		if((proType == 0 && Detect_Rising_Edge == 1) \
				|| (proType > 0 && Detect_Rising_Edge_Sub[proType - 1] == 1))
		{//һ��ʼ��⵽�ߵ�ƽ
			if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
			{//��⵽�͵�ƽ
				if(proType == 0)
				{
					Detect_Rising_Edge = 3;
				}
				else
				{
					Detect_Rising_Edge_Sub[proType - 1] = 3;
				}
			}
		}
		if((proType == 0 && (Detect_Rising_Edge == 2 || Detect_Rising_Edge == 3)) \
			|| (proType > 0 && (Detect_Rising_Edge_Sub[proType - 1] == 2 || Detect_Rising_Edge_Sub[proType - 1] == 3)))
		{//һ��ʼ��⵽�͵�ƽ,�����ٴμ�⵽��
			if(((Input_Detect_Status[index] >> offset) & 0x01) == 0)
			{//��⵽�ߵ�ƽ,ֹͣ
				result = 1;
			}
		}
	}
	return result;
}
	
/**************************************************************************************************
**  ��������  AXisMove()
**	���������Axsis����  Axsis_PositionĿ��λ�� Axsis_Speed���ٶ� speedMod�ٶȼ���ģʽ
**	�����������
**	�������ܣ����ƶ�
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u8 AXisMove(u8 Axsis, s32 Axsis_Position, u32 Axsis_Speed)
{
	u16 Result = 0;
	
	//����δ��ɵĶ���
	if(AxisMoveFlag[Axsis] == 1)
	{//��ǰ�ᶯ��δ���
		WaitAxisMoveFlag[Axsis] = 1;//ִ�е�������
		return 0;
	}
	else
	{
		WaitAxisMoveFlag[Axsis] = 0;
	}
	
	if(Axsis_Position > m_PulseTotalCounter[Axsis])
	{
		Axsis_Move_Direction[Axsis] = POSITIVE;
		if(SlowPointFlag[Axsis] == 1 && SlowPointIncrement[Axsis] > 0)
		{//���ٵ㴦��
			if(Axsis_Position < m_PulseTotalCounter[Axsis] + SlowPointIncrement[Axsis])
			{
				SlowPointFlag[Axsis] = 0;
				Axsis_Speed = SlowPointSpeed[Axsis];
			}
		}
		else
		{
			SlowPointFlag[Axsis] = 0;
		}
	}
	else if(Axsis_Position < m_PulseTotalCounter[Axsis])
	{
		Axsis_Move_Direction[Axsis] = NEGATIVE;
		if(SlowPointFlag[Axsis] == 1 && SlowPointIncrement[Axsis] > 0)
		{//���ٵ㴦��
			if(Axsis_Position + SlowPointIncrement[Axsis] > m_PulseTotalCounter[Axsis])
			{
				SlowPointFlag[Axsis] = 0;
				Axsis_Speed = SlowPointSpeed[Axsis];
			}
		}
		else
		{
			SlowPointFlag[Axsis] = 0;
		}
	}
	else
	{
		//��ֹĿ��λ�õ��ڵ�ǰλ��ʱ�����ٵ����ǰȷ��Ӱ����һ���ƶ�����
		SlowPointFlag[Axsis] = 0;
		AdvancePointFlag[Axsis] = 0;
		return Result;
	}
	CSP_Mode_AxisInit(Axis_To_ID(Axsis));
	
	AxisMoveFlag[Axsis] = 1;
//	m_LastAxsisSpeed[Axsis] = Axsis_Speed;
	Axsis_Speed = Axsis_Speed * JXS_Parameter.SpeedLevel * MAX_SPEED_CHANGE;
	SendPulse(Axsis, Axsis_Position, Axsis_Speed);
	
	return Result;
}

/**************************************************************************************************
**  ��������  Servo_MoveFinishSta(u8)
**	���������Axis ����   AxisTarPositionĿ��λ��
**	�����������
**	�������ܣ�����ȷ��
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u16 Servo_MoveFinishSta(u8 Axsis, s32 AxisTarPosition)
{
	u16 ret = 0;
	
	if(WaitAxisMoveFlag[Axsis] == 1)
	{//�ȴ�������ɹ����е����˶�
		if(AxisMoveFlag[Axsis] == DISABLE)
		{
			AXisMove(Axsis, AxisTarPosition, Axsis_MoveTarSpeed[Axsis]);
		}
		return 0;
	}
	else if(WaitAxisMoveFlag[Axsis] == 2)
	{//�ȴ�������ɹ����ж�������
		if(AxisMoveFlag[Axsis] == DISABLE)
		{
			AXisSncyMove(100);
		}
		return 0;
	}
	
	if(AxisMoveFlag[Axsis] == 0)
	{
		AdvancePointFlag[Axsis] = 0;
		ret = 1;
	}
	else if(AdvancePointFlag[Axsis] == 1)
	{//��ǰȷ��
		if(AxisTarPosition < m_PulseTotalCounter[Axsis] + abs(AdvancePointInc[Axsis]) && m_PulseTotalCounter[Axsis] < AxisTarPosition + abs(AdvancePointInc[Axsis]))
		{//��Ŀ������ǰȷ�Ϸ�Χ�ڣ��Ϳ���ȷ�ϳɹ�
			ret = 1;
			AdvancePointFlag[Axsis] = 0;			//�������ֵ(==2)��EtherCAT(==0)��һ��
		}
		else
		{
			ret = 0;
		}
	}

	return ret;
}

/**************************************************************************************************
**  ��������  AutoActionMove()
**	���������procNum ���еĽ��̺ż�ƫ��ֵ
**	���������
**	�������ܣ�ʵ�����˶�ͬʱֹͣ
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u8 AutoActionMove(u8 procNum)
{
	u16 i = 0;
	u32 axisMoveLen = 0;
	u32 axisMoveMaxLen = 0;
	u8  slowPointFlag = 0;
	u8  slowPointAxis = 0;
	s32 lenAxisDis = 0;
	u32 axisMoveDis[Axis_Num] = {0};
	
	m_InterpLenAxis = 0;
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		if(m_InterpAxisFlag[i] == procNum)
		{
			if(Axsis_MoveTarPos[i] > m_PulseTotalCounter[i])
			{
				axisMoveLen = Axsis_MoveTarPos[i] - m_PulseTotalCounter[i];
			}
			else
			{
				axisMoveLen = m_PulseTotalCounter[i] - Axsis_MoveTarPos[i];
			}
			axisMoveDis[i] = axisMoveLen;
			
			if(axisMoveMaxLen < axisMoveLen)
			{
				axisMoveMaxLen = axisMoveLen;
				m_InterpLenAxis = i;
			}
			
			AxisMoveFlag[i] = 1;
			CSP_Mode_AxisInit(Axis_To_ID(i));
		}
	}
	
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{//������ǰȷ�ϱ�־
		if(m_InterpAxisFlag[i] == procNum && AdvancePointFlag[i] == 1)
		{
			AdvancePointFlag[i] = 0;
			if(AdvancePointInc[i] > axisMoveMaxLen)
			{
				AdvancePointInc[m_InterpLenAxis] = axisMoveMaxLen;
			}
			else
			{
				AdvancePointInc[m_InterpLenAxis] = AdvancePointInc[i];
			}
			AdvancePointFlag[m_InterpLenAxis] = 1;
		}
	}
	if(AdvancePointFlag[m_InterpLenAxis] == 1)
	{
		for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
		{//������ǰȷ�ϱ�־
			if(i != m_InterpLenAxis && m_InterpAxisFlag[i] == procNum)
			{
				lenAxisDis = axisMoveDis[i] * AdvancePointInc[m_InterpLenAxis] / axisMoveMaxLen;
				AdvancePointInc[i] = lenAxisDis;
				AdvancePointFlag[i] = 1;
			}
		}
	}
		
	//������ٵ�
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{//Ѱ���Ƿ���ڼ��ٵ�
		if(m_InterpAxisFlag[i] == procNum && SlowPointFlag[i] == 1)
		{
			slowPointFlag = 1;
			slowPointAxis = i;
			break;
		}
	}
	if(slowPointFlag == 1)
	{
		SlowPointFlag[slowPointAxis] = 0;
		SlowPointFlag[m_InterpLenAxis] = 1;
		SlowPointIncrement[m_InterpLenAxis] = SlowPointIncrement[slowPointAxis];
		SlowPointSpeed[m_InterpLenAxis] = SlowPointSpeed[slowPointAxis];
		
		if(Axsis_MoveTarPos[m_InterpLenAxis] > m_PulseTotalCounter[m_InterpLenAxis])
		{
			if(SlowPointFlag[m_InterpLenAxis] == 1 && SlowPointIncrement[m_InterpLenAxis] > 0)
			{//���ٵ㴦��
				if(Axsis_MoveTarPos[m_InterpLenAxis] < m_PulseTotalCounter[m_InterpLenAxis] + SlowPointIncrement[m_InterpLenAxis])
				{
					SlowPointFlag[m_InterpLenAxis] = 0;
					Axsis_MoveTarSpeed[m_InterpLenAxis] = SlowPointSpeed[m_InterpLenAxis];
				}
			}
			else
			{
				SlowPointFlag[m_InterpLenAxis] = 0;
			}
		}
		else if(Axsis_MoveTarPos[m_InterpLenAxis] < m_PulseTotalCounter[m_InterpLenAxis])
		{
			if(SlowPointFlag[m_InterpLenAxis] == 1 && SlowPointIncrement[m_InterpLenAxis] > 0)
			{//���ٵ㴦��
				if(Axsis_MoveTarPos[m_InterpLenAxis] + SlowPointIncrement[m_InterpLenAxis] > m_PulseTotalCounter[m_InterpLenAxis])
				{
					SlowPointFlag[m_InterpLenAxis] = 0;
					Axsis_MoveTarSpeed[m_InterpLenAxis] = SlowPointSpeed[m_InterpLenAxis];
				}
			}
			else
			{
				SlowPointFlag[m_InterpLenAxis] = 0;
			}
		}
	}
	
	m_InterpCurveFlag = INTER_CURVE_ONE;
	InterpSendPulse();
	
	return 0;
}

/**************************************************************************************************
**  ��������  AXisSncyMove()
**	���������procNum ���еĽ��̺�
**	���������
**	�������ܣ�ʵ����ֱ�߲岹�˶�
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u8 AXisSncyMove(u8 procNum)
{
	u16 i = 0;
	u8 axisNum = 0;
	u32 axisMoveLen = 0;
	u32 axisMoveMaxLen = 0;
	u8  waitAxisMoveFlag = 0;
	u8 axisNum1 = 0;
	
	//��ֹĿ��λ�õ��ڵ�ǰλ��ʱ�����ٵ����ǰȷ��Ӱ����һ���ƶ�����
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		if(Axsis_MoveProNum[i] == procNum)
		{
			if(Axsis_MoveTarPos[i] == m_PulseTotalCounter[i])
			{
				//Axsis_MoveProNum[i] = 0;
			}
			else
			{
				axisNum++;
			}
			axisNum1++;
		}
	}
	if(axisNum1 > 0 && axisNum == 0)
	{
		for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
		{
			if(Axsis_MoveProNum[i] == procNum)
			{
				SlowPointFlag[i] = 0;
				AdvancePointFlag[i] = 0;
			}
		}
	}
		
	//����δ��ɶ�������
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		if(Axsis_MoveProNum[i] == procNum && AxisMoveFlag[i] == 1)
		{
			waitAxisMoveFlag = 1;
		}
		else if(Axsis_MoveProNum[i] == procNum)
		{
			WaitAxisMoveFlag[i] = 0;
		}
	}
	if(waitAxisMoveFlag == 1)
	{//���ᶯ��δ���
		for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
		{
			if(Axsis_MoveProNum[i] == procNum)
			{
				WaitAxisMoveFlag[i] = 2;			//2��ʾִ�ж�����������
			}
		}
		return 0;
	}
	
	axisNum = 0;
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		if(Axsis_MoveProNum[i] == procNum)
		{
			if(Axsis_MoveTarPos[i] == m_PulseTotalCounter[i])
			{
				Axsis_MoveProNum[i] = 0;
			}
			else
			{
				axisNum++;
			}
		}
	}
	
	if(axisNum == 1)
	{//�����˶�
		for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
		{
			if(Axsis_MoveProNum[i] == procNum)
			{
				AXisMove(i, Axsis_MoveTarPos[i], Axsis_MoveTarSpeed[i]);
				Axsis_MoveProNum[i] = 0;
				m_InterpAxisFlag[i] = 0;
				break;
			}
		}
	}
	else if(axisNum > 1)
	{//ֱ�߲岹��ֻ��һ��Ŀ���
		m_InterpLenAxis = 0;
		for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
		{//ȷ������
			if(Axsis_MoveProNum[i] == procNum)
			{
				if(Axsis_MoveTarPos[i] > m_PulseTotalCounter[i])
				{
					axisMoveLen = Axsis_MoveTarPos[i] - m_PulseTotalCounter[i];
				}
				else if(Axsis_MoveTarPos[i] < m_PulseTotalCounter[i])
				{
					axisMoveLen = m_PulseTotalCounter[i] - Axsis_MoveTarPos[i];
				}
				else
				{
					Axsis_MoveProNum[i] = 0;
					axisNum--;
					continue;
				}
				Axsis_MoveProNum[i] = 0;
				m_InterpAxisFlag[i] = procNum;
				
				if(axisMoveMaxLen < axisMoveLen)
				{
					axisMoveMaxLen = axisMoveLen;
					m_InterpLenAxis = i;
				}
			}
		}
		
		AutoActionMove(procNum);										//��ʼ�˶�
	}
	
	return 0;
}

/**************************************************************************************************
**  ��������  AutoActionStepList()
**	�����������Ҫ���е��к�
**	�����������������ָ�������
**	�������ܣ��жϹ�����ָ��������
**	��ע��	  
**  ���ߣ�       
**  ��������:
***************************************************************************************************/
u16 AutoActionStepList(SaveProgram *Program_Operate, u16 ActionLine)
{
	u16 result = 0;
	
	if(ActionLine < Program_Operate->Num)
	{
		while(Program_Operate->Program[ActionLine + result].List == Program_Operate->Program[ActionLine].List)
		{
			result++;
			
			if((ActionLine + result) >= Program_Operate->Num)
			{//�������
				break;
			}
			
			if(result == LISTNUM)	//�������е�ָ���������ܳ���LISTNUM��
			{
				break;
			}
		}
	}
	
	return result;	
}

/**************************************************************************************************
**  ��������  AutoActionOutControl()
**	�����������Ҫ���е��к�
**	��������������Ƿ�ɹ����
**	�������ܣ��������Զ�ģʽ���ƺ���
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u8 AutoActionOutControl(SaveProgram *Program_Operate, u8 ActionLine)
{
	u8 result = 0;
	u8 j = 0;
	u16 i = 0;
	u8 SubProgramNum = 0;
	u8 SubProgram_Sequence = 0;
	u8 IIC_Parameter[100] = {0};									//���ݶ�ȡʱʹ�õ��м����
	u32 pointTemp = 0;
	u8 axsisNum = 0;
	u8 keepMoveIoSta_Low[Axis_Num] = {0};					//������������Ӧ����ڵ�״̬-��
	u8 keepMoveIoSta_High[Axis_Num] = {0};				//������������Ӧ����ڵ�״̬-��
	s32 Increment_Distance = 0;
	u8 ret = 0;
	ST_MDPostion sPostion = {0};
	u8 ionum = 0;
	u8 index = 0;
	u8 offset = 0;
	
	result = Action_Step_List_Num;
	if(ActionLine < Program_Operate->Num)
	{//�жϴ�ִ�е��к��Ƿ���ȷ
		if(ActionAllowJudge(Program_Operate ,ActionLine) == 0)	
		{//����ִ��ǰ���ж�����ȫ�Ϸ����ж�
			switch(Program_Operate->Program[ActionLine].Order)
			{
			  case OR_BASICORDER://����ָ��
					switch(Program_Operate->Program[ActionLine].Key)
					{
						case K_PROGRAMSTART://����ʼ
							Program_RunTime_Count = 0;
						
							m_WhileNC = 0;	
							for(i=0; i<WHILE_NEST_MAX; i++)
							{
								m_WhileRunFlag[i] = 0;
							}
							
							m_IfElseNC = 0;	
							for(i=0; i<IF_ELSE_NEST_MAX; i++)
							{
								m_IfElseJudgeRunFlag[i] = 0;
							}
							break;
						case K_PROGRAMEND://�������
							result = 9;
							if(Program_Operate->Code == SAVEPROGRAMNUM_MAIN && Robot_Auto_Reset == FALSE)  //��λ����
							{	
								Program_Reset = TRUE;
								Robot_Auto_Reset = TRUE;
								g_Auto_Reset_Flag = FALSE;
								if(Temp_OUT_Switch_Parameter[O_RESETING_LIGHT] == 1)
								{
									SetOutput(O_RESETING_LIGHT);
								}
								CurProgramRead(g_Run_Program_Num_Pre);							//ѡ�г������Ϊԭ��ѡ�еĳ���								
							}
							else
							{
								Program_Reset = FALSE;
								Program_RunTime = Program_RunTime_Count / 10;
								m_ProRunTimeTotal = m_ProRunTimeTotalCount / 10;
								m_ProRunTimeCumulate = m_PreProRunTimeCumulate + m_ProRunTimeTotal;	
								if(Auto_Mode == ONCE_MODE) //����ģʽ-����ֹͣ״̬
								{
									g_Auto_Order_Stop = TRUE;	
								}
							}
							break;
						case K_DELAY://��ʱ
							g_Key_Delay_Timer =0;
							g_Key_Delay_Flag = TRUE;
							break;
						case K_SUBPROGRAM://�ӳ���  ��ʼ-����
							if(g_Run_Program_Num > SAVEPROGRAMNUM_MAIN && Program_Operate->Program[ActionLine].Value1 == g_Run_Program_Num - SAVEPROGRAMNUM_MAIN)
							{//֧���ӳ���������У���ǰִ�г���Ϊ�ӳ���ʱ�����ӳ���Ŀ�ʼ�ͽ�����Ϊ������Ŀ�ʼ����
								if(Program_Operate->Program[ActionLine].Value2 == V_PROGRAM_START)
								{//�ӳ���ʼ
//									Program_RunTime_Count = 0;
								}
								else 
								{//�ӳ������
									result = 9;
									Program_RunTime = Program_RunTime_Count / 10;
									m_ProRunTimeTotal = m_ProRunTimeTotalCount / 10;
								  m_ProRunTimeCumulate = m_PreProRunTimeCumulate + m_ProRunTimeTotal;	
								}
							}
							else if(g_Program_Is_Debuging == FALSE)
							{//����ʱ�����������ӳ�������
								SubProgramNum = Program_Operate->Program[ActionLine].Value1 - 1;//��ȡ�ӳ�����
								SubProgram_Sequence = SubProgramNum + SAVEPROGRAMNUM_MAIN;

								if(Program_Operate->Program[ActionLine].Value2 == V_PROGRAM_START && SubProgramNum < SAVEPROGRAMNUM_SUB)
								{//�ӳ���ʼ
									if(g_Read_SubProgram[SubProgramNum] == FALSE && g_SubProgram_Start[SubProgramNum] == FALSE)
									{//���ӳ���
										g_SubProgram_Step_Run[SubProgramNum] = TRUE;
										g_SubProgram_Start[SubProgramNum] = TRUE;
										g_SubProgram_ActionRun_Step[SubProgramNum] = 0;
										if(Program_Operate->Program[ActionLine].Value3 == V_ONCE)
										{
											g_SubProgram_ContralEnd[SubProgramNum] = TRUE;
										}
										else
										{
											g_SubProgram_ContralEnd[SubProgramNum] = FALSE;
										}
										
										W25QXX_Read(IIC_Parameter, Program_IIC_Address[SubProgram_Sequence].Address, 15);
										SubProgram_Operate[SubProgramNum].Flag  = IIC_Parameter[0];
										SubProgram_Operate[SubProgramNum].Code  = IIC_Parameter[1];
										SubProgram_Operate[SubProgramNum].Name  = (u32)( ((u32)IIC_Parameter[2])|((u32)IIC_Parameter[3]<<8)|((u32)IIC_Parameter[4]<<16)|((u32)IIC_Parameter[5]<<24) );
										SubProgram_Operate[SubProgramNum].Name2 = (u32)( ((u32)IIC_Parameter[6])|((u32)IIC_Parameter[7]<<8)|((u32)IIC_Parameter[8]<<16)|((u32)IIC_Parameter[9]<<24) );
										SubProgram_Operate[SubProgramNum].Name3 = (u32)( ((u32)IIC_Parameter[10])|((u32)IIC_Parameter[11]<<8)|((u32)IIC_Parameter[12]<<16)|((u32)IIC_Parameter[13]<<24) );
										SubProgram_Operate[SubProgramNum].Num   = IIC_Parameter[14];
										if(SubProgram_Operate[SubProgramNum].Flag==0)
										{
											result = 12;
										}
										else
										{
											for(j=0; j<SubProgram_Operate[SubProgramNum].Num; j++)
											{
												W25QXX_Read(IIC_Parameter, Program_IIC_Address[SubProgram_Sequence].Address+0x0F+0x10*j, 16);
												SubProgram_Operate[SubProgramNum].Program[j].Flag   = IIC_Parameter[0];
												SubProgram_Operate[SubProgramNum].Program[j].List   = IIC_Parameter[1];
												SubProgram_Operate[SubProgramNum].Program[j].Order  = IIC_Parameter[2];
												SubProgram_Operate[SubProgramNum].Program[j].Key    = IIC_Parameter[3];
												SubProgram_Operate[SubProgramNum].Program[j].Value1 = (u32)( ((u32)IIC_Parameter[4])|((u32)IIC_Parameter[5]<<8)|((u32)IIC_Parameter[6]<<16)|((u32)IIC_Parameter[7]<<24) );
												SubProgram_Operate[SubProgramNum].Program[j].Value2 = (u32)( ((u32)IIC_Parameter[8])|((u32)IIC_Parameter[9]<<8)|((u32)IIC_Parameter[10]<<16)|((u32)IIC_Parameter[11]<<24) );	
												SubProgram_Operate[SubProgramNum].Program[j].Value3 = (u32)( ((u32)IIC_Parameter[12])|((u32)IIC_Parameter[13]<<8)|((u32)IIC_Parameter[14]<<16)|((u32)IIC_Parameter[15]<<24) ); 					
												SubProgram_Operate[SubProgramNum].Program[j].Value1 = SubProgram_Operate[SubProgramNum].Program[j].Value1 & 0x0fffffff;
												SubProgram_Operate[SubProgramNum].Program[j].Value2 = SubProgram_Operate[SubProgramNum].Program[j].Value2 & 0x0fffffff;
						
												if((IIC_Parameter[3] == K_INCREMENT_RUNNING && IIC_Parameter[15]>>4 == 0x09)\
													|| ((IIC_Parameter[3] == K_IF || IIC_Parameter[3] == K_ELSE || IIC_Parameter[3] == K_WHILE || IIC_Parameter[3] == K_USER) && IIC_Parameter[15]>>4 == 0x08))
												{
													SubProgram_Operate[SubProgramNum].Program[j].Value3 = SubProgram_Operate[SubProgramNum].Program[j].Value3 | 0xf0000000;	
												}
												else
												{
													SubProgram_Operate[SubProgramNum].Program[j].Value3 = SubProgram_Operate[SubProgramNum].Program[j].Value3 & 0x0fffffff;
												}
											}
										}												 
										g_Read_SubProgram[SubProgramNum] = TRUE;
									}
								}
							}
							break;
						case K_JUMP://��תָ��
							if(Program_Operate->Program[ActionLine].Value1 == V_JUMP_TO)//��ת
							{
								for(i=0;i<Program_Operate->Num;i++)
								{
									if((Program_Operate->Program[i].Value1 == V_JUMP_LABEL)&&(Program_Operate->Program[ActionLine].Value3 == Program_Operate->Program[i].Value3))//��ת
									{
										m_JumpStepRunNum = i;
									}
								}
								if(m_JumpStepRunNum == Program_Operate->Num)
								{//����ת�кų�����Χ�������쳣
									result = 12;
								}
							}
							else if(Program_Operate->Program[ActionLine].Value1 == V_JUMP_LABEL)//��ǩ
							{							
								break;						
							}		
							break;
						case K_WHILE://Whileָ��
							m_WhileNC ++;
							if(m_WhileRunFlag[m_WhileNC - 1] == 0)
							{
								m_WhileRunFlag[m_WhileNC - 1] = 1;
								m_WhileLineNum[m_WhileNC - 1]= g_Auto_PresentLine;
								m_WhileCycCounter[m_WhileNC - 1] = 0;
								m_WhileJudgeType[m_WhileNC - 1] = Program_Operate->Program[ActionLine].Value1;
								
								if(Program_Operate->Program[ActionLine].Value1 == V_R_METHOD)
								{//R�ж�
									m_WhileJudgePar1[m_WhileNC - 1] = Program_Operate->Program[ActionLine].Value2;			//R�ж�ʱ��ʾ��ֵ�Ƚ�����
									m_WhileJudgePar2[m_WhileNC - 1] = Program_Operate->Program[ActionLine].Value3;			//R�ж�ʱ��ʾ�Ƚ�ֵ
								}
								else if(Program_Operate->Program[ActionLine].Value1 == V_I_METHOD)
								{//I�ж�
									m_WhileJudgePar1[m_WhileNC - 1] = Program_Operate->Program[ActionLine].Value2;			//I�ж�ʱ��ʾ�˿ں�
									m_WhileJudgePar2[m_WhileNC - 1] = Program_Operate->Program[ActionLine].Value3;			//I�ж�ʱ��ʾ�Ƚϳ�����ƽ��0��Ч��ƽ��1��Ч��ƽ
								}
								else if(V_USER1 <= Program_Operate->Program[ActionLine].Value1 && Program_Operate->Program[ActionLine].Value1 <= V_USER8)
								{//user1-user8
									if(V_ONLY_EQUAL <= Program_Operate->Program[ActionLine].Value2 && Program_Operate->Program[ActionLine].Value2 <= V_NOTONLY_EQUAL)
									{//�ж�����˿����ݺϷ���
										m_WhileJudgePar1[m_WhileNC - 1] = Program_Operate->Program[ActionLine].Value2;//�ж�
										m_WhileJudgePar2[m_WhileNC - 1] = Program_Operate->Program[ActionLine].Value3;//�û�����
									}
								}
								else
								{
									m_WhileRunFlag[m_WhileNC - 1] = 0;
									m_WhileNC = 0;
									result = 12;
								}
							}
							break;
						case K_CYCLEOVER://ѭ������
							break;
						case K_IF://ִ��IF���
						case K_ELSE://ִ��ELSE���
							if(Program_Operate->Program[ActionLine].Key == K_IF)
							{
								m_IfElseNC++;
								m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 0;																								//ÿ��ִ��IF����ʱҪ���Ƿ���ִ�б�־����
							}
							m_IfElseJudgeType[m_IfElseNC - 1] = Program_Operate->Program[ActionLine].Value1;
							m_IfElseJudgePar1[m_IfElseNC - 1] = Program_Operate->Program[ActionLine].Value2;
							m_IfElseJudgePar2[m_IfElseNC - 1] = Program_Operate->Program[ActionLine].Value3;
							break;
						case K_OVER://��ת����
							break;
						case K_SPECIAL:
							if(Program_Operate->Program[ActionLine].Value1 == V_SUSPEND) 		//��ͣ
							{}
							else if(Program_Operate->Program[ActionLine].Value1 == V_STOP)
							{}
							break;
						case K_PULSE_OUTPUT://�������
							if(Program_Operate->Program[ActionLine].Value1 < OUTPUT_NUM)
							{//�ж�����˿����ݺϷ���
								if(Program_Operate->Program[ActionLine].Value2 == V_SET)
								{
									SetSingle(60,Program_Operate->Program[ActionLine].Value1,0);							//��λ�˿�
									m_PulseOutputSta[Program_Operate->Program[ActionLine].Value1] = V_SET;
								}
								else
								{
									SetSingle(Program_Operate->Program[ActionLine].Value1,60,0);							//��λ�˿�
									m_PulseOutputSta[Program_Operate->Program[ActionLine].Value1] = V_RESET;
								}
								m_PulseOutputStartTime[Program_Operate->Program[ActionLine].Value1] = m_SystemTimeCounter;
								m_PulseOutputEndTime[Program_Operate->Program[ActionLine].Value1] = (m_SystemTimeCounter + Program_Operate->Program[ActionLine].Value3) % SYSTEM_TIME_MAX;
							}
							break;						
						case K_USER://�û�����
							if(Program_Operate->Program[ActionLine].Value2 == V_ADD)
							{// +
								USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] += Program_Operate->Program[ActionLine].Value3;
							}
							else if(Program_Operate->Program[ActionLine].Value2 == V_MINUS)
							{// -
								USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] -= Program_Operate->Program[ActionLine].Value3;
							}
							else if(Program_Operate->Program[ActionLine].Value2 == V_UEQUAL)
							{// =
								USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] = Program_Operate->Program[ActionLine].Value3;
							}
							else if(Program_Operate->Program[ActionLine].Value2 == V_MULTIP)
							{// *
								USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] *= Program_Operate->Program[ActionLine].Value3;
							}
							else if(Program_Operate->Program[ActionLine].Value2 == V_DIVIDE)
							{// /
								USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] /= Program_Operate->Program[ActionLine].Value3;
							}
							else if(Program_Operate->Program[ActionLine].Value2 == V_EXCESS)
							{// %
								USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] %= Program_Operate->Program[ActionLine].Value3;
							}
							break;
						case K_OUTDETECT://������
							break;
						default://����ָ�������쳣
							result = 12;	
							break;
					}
					break;
				case OR_AXISORDER://���ָ��
					switch(Program_Operate->Program[ActionLine].Key)
					{
						case K_MDPOSITION://���λ��
							ret = MD_ReadParData();
							if(ret == 1)
							{
								result = 13;
							}
							else
							{
								if(Program_Operate->Program[ActionLine].Value1 >= V_MD_AXSIS \
										&& Program_Operate->Program[ActionLine].Value1 < V_MD_AXSIS + Axis_Num)
								{//���λ��
									ret = MD_ReadCurPoint(&sPostion, 0);
									if(ret == 1)
									{
										result = 13;
										MD_PositionErr_Flag = TRUE;
										break;
									}
									axsisNum = Program_Operate->Program[ActionLine].Value1 - V_MD_AXSIS;
									pointTemp = sPostion.point[axsisNum];
								}
								else if(Program_Operate->Program[ActionLine].Value1 >= V_MD_AXSIS + Axis_Num \
										&& Program_Operate->Program[ActionLine].Value1 < V_MD_AXSIS + Axis_Num * 2)
								{//�ȴ���λ��
									ret = MD_ReadCurPoint(&sPostion, 1);
									if(ret == 1)
									{
										result = 13;
										MD_PositionErr_Flag = TRUE;
										break;
									}
									axsisNum = Program_Operate->Program[ActionLine].Value1 - V_MD_AXSIS - Axis_Num;
									pointTemp = sPostion.waitPoint[axsisNum];
								}
								
								if(axsisNum == O_Axsis && sMD_Parameter.revolveMode == 1)
								{//O��Ϊ����
									if(pointTemp == 0)
									{
										ResetOutput(sMD_Parameter.gasPort);
									}
									else
									{
										SetOutput(sMD_Parameter.gasPort);
									}
								}
								else
								{
									if(sCartesian_Para.axisInterpFlag[axsisNum] == 0)
									{//���᲻����岹
										AXisMove(axsisNum, pointTemp, Program_Operate->Program[ActionLine].Value2);
									}
									else
									{
										Axsis_MoveProNum[axsisNum] = 100;
										Axsis_MoveTarSpeed[axsisNum] = Program_Operate->Program[ActionLine].Value2;
									}
									Axsis_MoveTarPos[axsisNum] = pointTemp;
								}
							}
							break;
						case K_MDPOCOUNT://������
							ret = MD_ReadParData();
							if(ret == 1)
							{
								result = 13;
							}
							else
							{								
								ret = MD_StackCount(Program_Operate->Program[ActionLine].Value1, Program_Operate->Program[ActionLine].Value2, Program_Operate->Program[ActionLine].Value3);
								if(ret == 1)
								{
									result = 13;
								}
							}
							break;
						case K_XAXIS://��λ���ƶ�: λ�� , �ٶ�
						case K_ZAXIS:
						case K_LAXIS:
						case K_OAXIS:
							if(Program_Operate->Program[ActionLine].Key == K_XAXIS) 
							{//X��
								axsisNum = X_Axsis;
								pointTemp = Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Point_X + MINROBOTPOSITION;
							}
							else if(Program_Operate->Program[ActionLine].Key == K_ZAXIS) 
							{//Z��
								axsisNum = Z_Axsis;
								pointTemp = Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Point_Z + MINROBOTPOSITION;
							}
							else if(Program_Operate->Program[ActionLine].Key == K_LAXIS)
							{//Y��
								axsisNum = L_Axsis;
								pointTemp = Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Point_L + MINROBOTPOSITION;
							}
							else if(Program_Operate->Program[ActionLine].Key == K_OAXIS)
							{//O��
								axsisNum = O_Axsis;
								pointTemp = Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Point_O + MINROBOTPOSITION;
							}
							
							if(Program_Operate->Program[ActionLine].Value1 == 0 || Program_Operate->Program[ActionLine].Value1 > 75)
							{
								result = 13;
							}
							else if(Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Flag != 1)
							{
								result = 13;
							}
							else
							{
								if(sCartesian_Para.axisInterpFlag[axsisNum] == 0)
								{
									AXisMove(axsisNum, pointTemp, Program_Operate->Program[ActionLine].Value2);
								}
								else
								{
									Axsis_MoveProNum[axsisNum] = 100;
									Axsis_MoveTarPos[axsisNum] = pointTemp;
									Axsis_MoveTarSpeed[axsisNum] = Program_Operate->Program[ActionLine].Value2;
								}
							}
							break;
						case K_KEEP_MOVE://��������
						case K_NEGTIVE_SEARCH://��������
							keepMoveIoSta_Low[X_Axsis] = X_AXIS_MOVE_SIGN_LOW_LEVEL;
							keepMoveIoSta_Low[L_Axsis] = Y_AXIS_MOVE_SIGN_LOW_LEVEL;
							keepMoveIoSta_Low[Z_Axsis] = Z_AXIS_MOVE_SIGN_LOW_LEVEL;
							keepMoveIoSta_Low[O_Axsis] = O_AXIS_MOVE_SIGN_LOW_LEVEL;
							keepMoveIoSta_High[X_Axsis] = X_AXIS_MOVE_SIGN_HIGH_LEVEL;
							keepMoveIoSta_High[L_Axsis] = Y_AXIS_MOVE_SIGN_HIGH_LEVEL;
							keepMoveIoSta_High[Z_Axsis] = Z_AXIS_MOVE_SIGN_HIGH_LEVEL;
							keepMoveIoSta_High[O_Axsis] = O_AXIS_MOVE_SIGN_HIGH_LEVEL;
							
							axsisNum = Program_Operate->Program[ActionLine].Value1 - V_KEEP_MOVE_X;
							if(axsisNum < Axis_Num)
							{
								Flag_Keep_Move[axsisNum] = 2;
								if(Program_Operate->Program[ActionLine].Key == K_KEEP_MOVE)
								{
									Axsis_Move_Direction[axsisNum] = POSITIVE;
								}
								else if(Program_Operate->Program[ActionLine].Key == K_NEGTIVE_SEARCH)
								{
									Axsis_Move_Direction[axsisNum] = NEGATIVE;
								}
								
								if(Program_Operate->Program[ActionLine].Value3 == V_FALLING_EDGE)
								{//�½��أ�����
									if(keepMoveIoSta_Low[axsisNum] == 1)	
									{//һ��ʼ��⵽�͵�ƽ
										Flag_Falling_Edge = 1;
									}
									else if(keepMoveIoSta_High[axsisNum] == 1)
									{//һ��ʼ��⵽�ߵ�ƽ
										Flag_Falling_Edge = 2;
									}
									Flag_Keep_Move[axsisNum] = 1;
								}
								else if(Program_Operate->Program[ActionLine].Value3 == V_RISING_EDGE)
								{//�����أ�����
									if(keepMoveIoSta_Low[axsisNum] == 1)	
									{//һ��ʼ��⵽�͵�ƽ
										Flag_Rising_Edge = 2;
									}
									else if(keepMoveIoSta_High[axsisNum] == 1)
									{//һ��ʼ��⵽�ߵ�ƽ
										Flag_Rising_Edge = 1;
									}
									Flag_Keep_Move[axsisNum] = 1;
								}
								else if(Program_Operate->Program[ActionLine].Value3 == V_HIGH_LEVEL)
								{//���ߵ�ƽ
									if(keepMoveIoSta_Low[axsisNum] == 1)	
									{//һ��ʼ��⵽�͵�ƽ
										Flag_Keep_Move[axsisNum] = 1;
									}
								}
								else if(Program_Operate->Program[ActionLine].Value3 == V_LOW_LEVEL)
								{//���͵�ƽ
									if(keepMoveIoSta_High[axsisNum] == 1)	
									{//һ��ʼ��⵽�ߵ�ƽ
										Flag_Keep_Move[axsisNum] = 1;
									}
								}
								
								if(Flag_Keep_Move[axsisNum] == 1)
								{
									if((Program_Operate->Program[ActionLine].Value3 == V_LOW_LEVEL) && keepMoveIoSta_Low[axsisNum] == 1)
									{}
									else if((Program_Operate->Program[ActionLine].Value3 == V_HIGH_LEVEL) && keepMoveIoSta_High[axsisNum] == 1)
									{}
									else
									{
										if(Axsis_Move_Direction[axsisNum] == POSITIVE)
										{
											AXisMove(axsisNum, MAXROBOTPOSITION, Program_Operate->Program[ActionLine].Value2);
										}
										else
										{
											AXisMove(axsisNum, MINROBOTPOSITION, Program_Operate->Program[ActionLine].Value2);
										}
									}
								}
							}
							
							break;
						case K_INCREMENT_RUNNING:
							axsisNum = Program_Operate->Program[ActionLine].Value1;
							if(V_KEEP_MOVE_X <= axsisNum && axsisNum <= V_KEEP_MOVE_O)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_X;
							}
							else if(V_KEEP_MOVE_U <= axsisNum && axsisNum <= V_KEEP_MOVE_V)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_U + Axis_Num;
							}
							
							if(axsisNum < Axis_Num + Ext_Axis_Num) 
							{
								if(Increment_Finished[axsisNum] == FALSE)
								{//û��ִ�����ʱ��ͣ�����������ټ���
									Increment_Finished[axsisNum] = TRUE;
									Increment_Distance = Program_Operate->Program[ActionLine].Value3 * Step_Coefficient[axsisNum] / 100;
									Increment_Target[axsisNum] = m_PulseTotalCounter[axsisNum] + Increment_Distance;
								}
								if(Increment_Target[axsisNum] < MINROBOTPOSITION)
								{
									Increment_Target[axsisNum] = MINROBOTPOSITION;
								}
								
								if(sCartesian_Para.axisInterpFlag[axsisNum] == 0)
								{
									AXisMove(axsisNum, Increment_Target[axsisNum], Program_Operate->Program[ActionLine].Value2);
								}
								else
								{
									Axsis_MoveProNum[axsisNum] = 100;
									Axsis_MoveTarPos[axsisNum] = Increment_Target[axsisNum];
									Axsis_MoveTarSpeed[axsisNum] = Program_Operate->Program[ActionLine].Value2;
								}
							}
							break;
						case K_MACHINE_ORIGIN:
							break;
						case K_POSSET:
							axsisNum = Program_Operate->Program[ActionLine].Value1;
							if(V_KEEP_MOVE_X <= axsisNum && axsisNum <= V_KEEP_MOVE_O)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_X;
							}
							else if(V_KEEP_MOVE_U <= axsisNum && axsisNum <= V_KEEP_MOVE_V)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_U + Axis_Num;
							}
							m_PositionResetStep[axsisNum] = 0;
							break;
						case K_SLOWPOINT:
							axsisNum = Program_Operate->Program[ActionLine].Value1;
							if(V_KEEP_MOVE_X <= axsisNum && axsisNum <= V_KEEP_MOVE_O)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_X;
							}
							else if(V_KEEP_MOVE_U <= axsisNum && axsisNum <= V_KEEP_MOVE_V)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_U + Axis_Num;
							}
							SlowPointIncrement[axsisNum] = (Program_Operate->Program[ActionLine].Value3 & 0x0fffffff) * Step_Coefficient[axsisNum] / 100;			//���ٶȵ�����λ��
							SlowPointSpeed[axsisNum] = Program_Operate->Program[ActionLine].Value2;																							//���ٶȵ��ٶ�
							SlowPointFlag[axsisNum] = 1;																																												//������ٶȵ��־
							break;
						case K_INTER_START:
							break;
						case K_INTER_OVER:
							break;
						case K_ADVENCE:
							if(g_Program_Is_Debuging == FALSE && sCartesian_Para.MDCoordType != 2 && (Program_Operate->Program[ActionLine].Value3 & 0x0fffffff) > 10)
							{//SCARAģʽ�£���ȷ����Ч
								axsisNum = Program_Operate->Program[ActionLine].Value1 - V_KEEP_MOVE_X;
								AdvancePointInc[axsisNum] = (Program_Operate->Program[ActionLine].Value3 & 0x0fffffff) * Step_Coefficient[axsisNum] / 100;					//��ǰȷ����
								AdvancePointFlag[axsisNum] = 1;																																											//��ǰȷ������־
							}
							break;
						case K_INTER_LINE:
							break;
						case K_AXISMOVE://���ƶ�
							axsisNum = Program_Operate->Program[ActionLine].Value1;
							if(V_KEEP_MOVE_X <= axsisNum && axsisNum <= V_KEEP_MOVE_O)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_X;
							}
							else if(V_KEEP_MOVE_U <= axsisNum && axsisNum <= V_KEEP_MOVE_V)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_U + Axis_Num;
							}
							pointTemp = (Program_Operate->Program[ActionLine].Value3 & 0x0fffffff) * Step_Coefficient[axsisNum] / 100 + MINROBOTPOSITION;
							AXisMove(axsisNum, pointTemp, Program_Operate->Program[ActionLine].Value2);
							break;
						case K_ANGLE_ARC:
							break;
						default://���ָ�������쳣
							result = 13;	
							break;
					}
					break;
				case OR_IOORDER://IOָ��
					switch(Program_Operate->Program[ActionLine].Key)
					{//ȫ����Ϊֱ���������⣬һ�������Ӧ����ָ���λ����λ
						case K_IOINSTRUCT_OUTPUT1://Y0-1		
							SetSingle(60,O_IODEBUG_OUTPUT_0,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT2://Y0-0
							SetSingle(O_IODEBUG_OUTPUT_0,60,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT3://Y1-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_1,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT4://Y1-0
							SetSingle(O_IODEBUG_OUTPUT_1,60,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT5://Y2-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_2,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT6://Y2-0
							SetSingle(O_IODEBUG_OUTPUT_2,60,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT7://Y3-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_3,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT8://Y3-0
							SetSingle(O_IODEBUG_OUTPUT_3,60,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT9://Y4-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_4,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT10://Y4-0
							SetSingle(O_IODEBUG_OUTPUT_4,60,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT11://Y5-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_5,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT12://Y5-0
							SetSingle(O_IODEBUG_OUTPUT_5,60,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT13://Y6-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_6,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT14://Y6-0
							SetSingle(O_IODEBUG_OUTPUT_6,60,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT15://Y7-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_7,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT16://Y7-0
							SetSingle(O_IODEBUG_OUTPUT_7,60,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT17://Y8-1  
							SetSingle(60,O_IODEBUG_OUTPUT_8,Program_Operate->Program[ActionLine].Value1);
							break;							
						case K_IOINSTRUCT_OUTPUT18://Y8-0 
							SetSingle(O_IODEBUG_OUTPUT_8,60,Program_Operate->Program[ActionLine].Value1);
							break;							
						case K_IOINSTRUCT_OUTPUT19://Y9-1 
							SetSingle(60,O_IODEBUG_OUTPUT_9,Program_Operate->Program[ActionLine].Value1);
							break;							
						case K_IOINSTRUCT_OUTPUT20://Y9-0 
							SetSingle(O_IODEBUG_OUTPUT_9,60,Program_Operate->Program[ActionLine].Value1);
							break;							
						case K_IOINSTRUCT_OUTPUT21://Y10-1 
							SetSingle(60,O_IODEBUG_OUTPUT_10,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT22://Y10-0 
							SetSingle(O_IODEBUG_OUTPUT_10,60,Program_Operate->Program[ActionLine].Value1);
							break;							
						case K_IOINSTRUCT_OUTPUT23://Y11-1
							SetSingle(60,O_IODEBUG_OUTPUT_11,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT24://Y11-0 
							SetSingle(O_IODEBUG_OUTPUT_11,60,Program_Operate->Program[ActionLine].Value1);
							break;							
						case K_IOINSTRUCT_OUTPUT25://Y12-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_12,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT26://Y12-0
							SetSingle(O_IODEBUG_OUTPUT_12,60,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT27://Y13-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_13,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT28://Y13-0
							SetSingle(O_IODEBUG_OUTPUT_13,60,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT29://Y14-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_14,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT30://Y14-0
							SetSingle(O_IODEBUG_OUTPUT_14,60,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT31://Y15-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_15,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT32://Y15-0
							SetSingle(O_IODEBUG_OUTPUT_15,60,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT33://Y16-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_16,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT34://Y16-0
							SetSingle(O_IODEBUG_OUTPUT_16,60,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT35://Y17-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_17,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT36://Y17-0
							SetSingle(O_IODEBUG_OUTPUT_17,60,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT37://Y18-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_18,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT38://Y18-0
							SetSingle(O_IODEBUG_OUTPUT_18,60,Program_Operate->Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT39://Y19-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_19,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT40://Y19-0
							SetSingle(O_IODEBUG_OUTPUT_19,60,Program_Operate->Program[ActionLine].Value1);
							break;							
						case K_IOINSTRUCT_OUTPUT41://Y20-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_20,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT42://Y20-0
							SetSingle(O_IODEBUG_OUTPUT_20,60,Program_Operate->Program[ActionLine].Value1);
							break;							
						case K_IOINSTRUCT_OUTPUT43://Y21-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_21,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT44://Y21-0
							SetSingle(O_IODEBUG_OUTPUT_21,60,Program_Operate->Program[ActionLine].Value1);
							break;							
						case K_IOINSTRUCT_OUTPUT45://Y22-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_22,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT46://Y22-0
							SetSingle(O_IODEBUG_OUTPUT_22,60,Program_Operate->Program[ActionLine].Value1);
							break;							
						case K_IOINSTRUCT_OUTPUT47://Y23-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_23,Program_Operate->Program[ActionLine].Value1);							 
							break;						
						case K_IOINSTRUCT_OUTPUT48://Y23-0
							SetSingle(O_IODEBUG_OUTPUT_23,60,Program_Operate->Program[ActionLine].Value1);
							break;							
						case K_IOINSTRUCT_OUTPUT49://RY0-1
							SetSingle(60,O_IODEBUG_OUTPUT_28,Program_Operate->Program[ActionLine].Value1);	
							break;							
						case K_IOINSTRUCT_OUTPUT50://RY0-0
							SetSingle(O_IODEBUG_OUTPUT_28,60,Program_Operate->Program[ActionLine].Value1);	
							break;							
						case K_IOINSTRUCT_OUTPUT51://RY1-1
							SetSingle(60,O_IODEBUG_OUTPUT_29,Program_Operate->Program[ActionLine].Value1);		
							break;							
						case K_IOINSTRUCT_OUTPUT52://RY1-0
							SetSingle(O_IODEBUG_OUTPUT_29,60,Program_Operate->Program[ActionLine].Value1);		
							break;

						//�����źż��-22·
						case K_IOINSTRUCT_INPUT1://����1-X0
						case K_IOINSTRUCT_INPUT2://����2-X1
						case K_IOINSTRUCT_INPUT3://����3-X2
						case K_IOINSTRUCT_INPUT4://����4-X3
						case K_IOINSTRUCT_INPUT5://����5-X4
						case K_IOINSTRUCT_INPUT6://����6-X5
						case K_IOINSTRUCT_INPUT7://����7-X6
						case K_IOINSTRUCT_INPUT8://����8-X7
						case K_IOINSTRUCT_INPUT9://����9-X8
						case K_IOINSTRUCT_INPUT10://����10-X9
						case K_IOINSTRUCT_INPUT11://����11-X10
						case K_IOINSTRUCT_INPUT12://����12-X11
						case K_IOINSTRUCT_INPUT13://����13-X12
						case K_IOINSTRUCT_INPUT14://����14-X13
						case K_IOINSTRUCT_INPUT15://����15-X14
						case K_IOINSTRUCT_INPUT16://����16-X15
						case K_IOINSTRUCT_INPUT17://����17-X16
						case K_IOINSTRUCT_INPUT18://����18-X17
						case K_IOINSTRUCT_INPUT19://����19-X18
						case K_IOINSTRUCT_INPUT20://����20-X19
						case K_IOINSTRUCT_INPUT21://����21-X20
						case K_IOINSTRUCT_INPUT22://����22-X21
						case K_IOINSTRUCT_INPUT23://����23-X22
						case K_IOINSTRUCT_INPUT24:
						case K_IOINSTRUCT_INPUT25:
						case K_IOINSTRUCT_INPUT26:
						case K_IOINSTRUCT_INPUT27: 
						case K_IOINSTRUCT_INPUT28:
						case K_IOINSTRUCT_INPUT29:
						case K_IOINSTRUCT_INPUT30:
								ionum = Program_Operate->Program[ActionLine].Key-K_IOINSTRUCT_INPUT1;
								IO_Input_waittime[ionum] = Program_Operate->Program[ActionLine].Value3;
									
								index = ionum / 8;
								offset = ionum % 8;
									
								if(Program_Operate->Program[ActionLine].Value2 == V_FALLING_EDGE)
								{//�½��أ�����
									if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
									{//һ��ʼ��⵽�͵�ƽ
										Detect_Falling_Edge = 1;
									}
									else if(((Input_Detect_Status[index] >> offset) & 0x01) == 0)
									{//һ��ʼ��⵽�ߵ�ƽ
										Detect_Falling_Edge = 2;
									}
								}
								else if(Program_Operate->Program[ActionLine].Value2 == V_RISING_EDGE)
								{//�����أ�����
									if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
									{//һ��ʼ��⵽�͵�ƽ
										Detect_Rising_Edge = 2;
									}
									else if(((Input_Detect_Status[index] >> offset) & 0x01) == 0)	
									{//һ��ʼ��⵽�ߵ�ƽ
										Detect_Rising_Edge = 1;
									}
								}
							 break;
						default:
							 result = 14;	//IO����ָ�������쳣
							 break;
					}
					break;
				default:
					result = 11;	//��Ҫָ�������쳣
					break;
			}//switch(order)
			
			if(result < 9)
			{
				Action_Step_Run_Num++;
				if(Action_Step_List_Num >= Action_Step_Run_Num)
				{
					result = Action_Step_List_Num - Action_Step_Run_Num;//��������ָ����-������ָ��������=0�򵥴��������
					
					if(result == 0)
					{
						AXisSncyMove(100);
					}
				}
				else
				{
					result = 11;
				}
			}
		}//if(ActionAllowJudge(ActionLine))
		else
		{//��Ҫָ�������쳣
			result = 11; 
		}
	}//else->if(Flag)
	else
	{//��ǰ�кų����쳣
		result = 10;	
	}
	return result;		
}

/**************************************************************************************************
**  ��������  SubProgramActionOutControl()
**	�����������Ҫ���е��к�
**	���������
**	�������ܣ��ӳ����Զ�ģʽ���ƺ���
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u8 SubProgramActionOutControl(u8 subProNum, u8 ActionLine)
{
	u8 result = 0;
	u8 j = 0;
	u8 SubProgramNum = 0;
	u8 SubProgram_Sequence = 0;
	u8 IIC_Parameter[100] = {0};						//���ݶ�ȡʱʹ�õ��м����
	u32 pointTemp = 0;
	u8 axsisNum = 0;
	u8 keepMoveIoSta_Low[Axis_Num] = {0};					//������������Ӧ����ڵ�״̬-��
	u8 keepMoveIoSta_High[Axis_Num] = {0};					//������������Ӧ����ڵ�״̬-��
	s32 Increment_Distance = 0;
	u8 ret = 0;
	ST_MDPostion sPostion = {0};
	u8 ionum = 0;
	u8 index = 0;
	u8 offset = 0;

	if(ActionLine < SubProgram_Operate[subProNum].Num)
	{//ȷ�������к��Ƿ���������Χ��
		if(SubActionAllowJudge(subProNum, ActionLine) == 0)
		{//����ִ��ǰ���ж�����ȫ�Ϸ����ж�
			switch(SubProgram_Operate[subProNum].Program[ActionLine].Order)
			{
				case OR_BASICORDER://����ָ��
					switch(SubProgram_Operate[subProNum].Program[ActionLine].Key)
					{
						case K_PROGRAMSTART://����ʼ
							m_WhileSubNC[subProNum] = 0;	
							for(j=0; j<WHILE_NEST_MAX; j++)
							{
								m_WhileSubRunFlag[subProNum][j] = 0;
							}
							
							m_IfElseSubNC[subProNum] = 0;	
							for(j=0; j<IF_ELSE_NEST_MAX; j++)
							{
								m_IfElseSubJudgeRunFlag[subProNum][j] = 0;
							}
							break;
						case K_PROGRAMEND://�������
							result = 9;
							break;
						case K_DELAY://��ʱ
							g_SubProgram_Key_Delay_Timer[subProNum] = 0;
							g_SubProgram_Key_Delay_Flag[subProNum] = TRUE;
							break;
						case K_SUBPROGRAM://�ӳ���  ��ʼ-����
							if(g_Program_Is_Debuging == FALSE)
							{//����ʱ�����������ӳ�������
								SubProgramNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1;//��ȡ�ӳ�����
								SubProgram_Sequence = SubProgramNum + SAVEPROGRAMNUM_MAIN;

								if((SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_PROGRAM_START) && (SubProgramNum < SAVEPROGRAMNUM_SUB))
								{//�ӳ���ʼ
									if(g_Read_SubProgram[SubProgramNum] == FALSE  && g_SubProgram_Start[SubProgramNum] == FALSE)
									{//���ӳ���
										g_SubProgram_Step_Run[SubProgramNum] = TRUE;
										g_SubProgram_Start[SubProgramNum] = TRUE;
										g_SubProgram_ActionRun_Step[SubProgramNum] = 0;
										if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_ONCE)
										{
											g_SubProgram_ContralEnd[SubProgramNum] = TRUE;
										}
										else
										{
											g_SubProgram_ContralEnd[SubProgramNum] = FALSE;
										}
										
										W25QXX_Read(IIC_Parameter,Program_IIC_Address[SubProgram_Sequence].Address,15);
										SubProgram_Operate[SubProgramNum].Flag  = IIC_Parameter[0];
										SubProgram_Operate[SubProgramNum].Code  = IIC_Parameter[1];
										SubProgram_Operate[SubProgramNum].Name  = (u32)( ((u32)IIC_Parameter[2])|((u32)IIC_Parameter[3]<<8)|((u32)IIC_Parameter[4]<<16)|((u32)IIC_Parameter[5]<<24) );
										SubProgram_Operate[SubProgramNum].Name2 = (u32)( ((u32)IIC_Parameter[6])|((u32)IIC_Parameter[7]<<8)|((u32)IIC_Parameter[8]<<16)|((u32)IIC_Parameter[9]<<24) );
										SubProgram_Operate[SubProgramNum].Name3 = (u32)( ((u32)IIC_Parameter[10])|((u32)IIC_Parameter[11]<<8)|((u32)IIC_Parameter[12]<<16)|((u32)IIC_Parameter[13]<<24) );
										SubProgram_Operate[SubProgramNum].Num   = IIC_Parameter[14];
										if(SubProgram_Operate[SubProgramNum].Flag==0)
										{
											result = 12;
										}
										else
										{
											for(j=0;j<SubProgram_Operate[SubProgramNum].Num;j++)
											{
												W25QXX_Read(IIC_Parameter,Program_IIC_Address[SubProgram_Sequence].Address+0x0F+0x10*j,16);
												SubProgram_Operate[SubProgramNum].Program[j].Flag   = IIC_Parameter[0];
												SubProgram_Operate[SubProgramNum].Program[j].List   = IIC_Parameter[1];
												SubProgram_Operate[SubProgramNum].Program[j].Order  = IIC_Parameter[2];
												SubProgram_Operate[SubProgramNum].Program[j].Key    = IIC_Parameter[3];
												SubProgram_Operate[SubProgramNum].Program[j].Value1 = (u32)( ((u32)IIC_Parameter[4])|((u32)IIC_Parameter[5]<<8)|((u32)IIC_Parameter[6]<<16)|((u32)IIC_Parameter[7]<<24) );
												SubProgram_Operate[SubProgramNum].Program[j].Value2 = (u32)( ((u32)IIC_Parameter[8])|((u32)IIC_Parameter[9]<<8)|((u32)IIC_Parameter[10]<<16)|((u32)IIC_Parameter[11]<<24) );	
												SubProgram_Operate[SubProgramNum].Program[j].Value3 = (u32)( ((u32)IIC_Parameter[12])|((u32)IIC_Parameter[13]<<8)|((u32)IIC_Parameter[14]<<16)|((u32)IIC_Parameter[15]<<24) ); 					
												SubProgram_Operate[SubProgramNum].Program[j].Value1 = SubProgram_Operate[SubProgramNum].Program[j].Value1 & 0x0fffffff;
												SubProgram_Operate[SubProgramNum].Program[j].Value2 = SubProgram_Operate[SubProgramNum].Program[j].Value2 & 0x0fffffff;	
												if((IIC_Parameter[3] == K_INCREMENT_RUNNING && IIC_Parameter[15]>>4 == 0x09)\
													|| ((IIC_Parameter[3] == K_IF || IIC_Parameter[3] == K_ELSE || IIC_Parameter[3] == K_WHILE || IIC_Parameter[3] == K_USER) && IIC_Parameter[15]>>4 == 0x08))
												{
													SubProgram_Operate[SubProgramNum].Program[j].Value3 = SubProgram_Operate[SubProgramNum].Program[j].Value3 | 0xf0000000;	
												}
												else
												{
													SubProgram_Operate[SubProgramNum].Program[j].Value3 = SubProgram_Operate[SubProgramNum].Program[j].Value3 & 0x0fffffff;
												}
											}
										}												 
										g_Read_SubProgram[SubProgramNum] = TRUE;
									}
								}
								else
								{
									result = 15;
								}
							}
							break;
						case K_JUMP://��תָ��
							if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_JUMP_TO)//��ת
							{
								for(j=0;j<SubProgram_Operate[subProNum].Num;j++)
								{
									if((SubProgram_Operate[subProNum].Program[j].Value1 == V_JUMP_LABEL)&&(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == SubProgram_Operate[subProNum].Program[j].Value3))//��ת
									{
										m_JumpSubStepRunNum[subProNum] = j;
									}
								}
								if(m_JumpSubStepRunNum[subProNum] == SubProgram_Operate[subProNum].Num)
								{//����ת�кų�����Χ�������쳣
									result = 12;
								}
							}
							else if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_JUMP_LABEL)//��ǩ
							{							
								break;						
							}	
							break;
						case K_WHILE://Whileָ��
							m_WhileSubNC[subProNum]++;
							if(m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] == 0)
							{
								m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
								m_WhileSubLineNum[subProNum][m_WhileSubNC[subProNum] - 1]= g_SubProgram_PresentLine[subProNum];
								m_WhileSubCycCounter[subProNum][m_WhileSubNC[subProNum] - 1] = 0;
								m_WhileSubJudgeType[subProNum][m_WhileSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value1;
								
								if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_R_METHOD)
								{//R�ж�
									m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value2;//R�ж�ʱ��ʾ��ֵ�Ƚ�����
									m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value3;//R�ж�ʱ��ʾ�Ƚ�ֵ
								}
								else if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_I_METHOD)
								{//I�ж�
									m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value2;//I�ж�ʱ��ʾ�˿ں�
									m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value3;//I�ж�ʱ��ʾ�Ƚϳ�����ƽ��0��Ч��ƽ��1��Ч��ƽ
								}
								else if(V_USER1 <= SubProgram_Operate[subProNum].Program[ActionLine].Value1 && SubProgram_Operate[subProNum].Program[ActionLine].Value1 <= V_USER8)
								{//user1-user8
									if(V_ONLY_EQUAL <= SubProgram_Operate[subProNum].Program[ActionLine].Value2 && SubProgram_Operate[subProNum].Program[ActionLine].Value2 <= V_NOTONLY_EQUAL)
									{//�ж�����˿����ݺϷ���
										m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value2;	//�ж�
										m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value3;	//�û�����
									}
								}
								else
								{
									m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] = 0;
									m_WhileSubNC[SubProgramNum] = 0;
									result = 12;
								}
							}
							break;
						case K_CYCLEOVER://ѭ������
							break;
						case K_IF://ִ��IF���
						case K_ELSE://ִ��ELSE���
							if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_IF)
							{
								m_IfElseSubNC[subProNum]++;
								m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;//ÿ��ִ��IF����ʱҪ���Ƿ���ִ�б�־����
							}
							m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value1;
							m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value2;
							m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value3;
							break;
						case K_OVER://��ת����
							break;
						case K_SPECIAL:
							if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_SUSPEND) 		//��ͣ
							{}
							else if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_STOP)
							{}
							break;	
						case K_PULSE_OUTPUT://�������
							if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 < OUTPUT_NUM)
							{//�ж�����˿����ݺϷ���
								if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_SET)
								{
									SetSingle(60,SubProgram_Operate[subProNum].Program[ActionLine].Value1,0);							//��λ�˿�
									m_PulseOutputSta[SubProgram_Operate[subProNum].Program[ActionLine].Value1] = V_SET;
								}
								else
								{
									SetSingle(SubProgram_Operate[subProNum].Program[ActionLine].Value1,60,0);							//��λ�˿�
									m_PulseOutputSta[SubProgram_Operate[subProNum].Program[ActionLine].Value1] = V_RESET;
								}
								m_PulseOutputStartTime[SubProgram_Operate[subProNum].Program[ActionLine].Value1] = m_SystemTimeCounter;
								m_PulseOutputEndTime[SubProgram_Operate[subProNum].Program[ActionLine].Value1] = (m_SystemTimeCounter + SubProgram_Operate[subProNum].Program[ActionLine].Value3) % SYSTEM_TIME_MAX;
							}
							break;						
						case K_USER://�û�����
							if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_ADD)
							{// +
								USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1] += SubProgram_Operate[subProNum].Program[ActionLine].Value3;
							}
							else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_MINUS)
							{// -
								USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1] -= SubProgram_Operate[subProNum].Program[ActionLine].Value3;
							}
							else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_UEQUAL)
							{// =
								USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1] = SubProgram_Operate[subProNum].Program[ActionLine].Value3;
							}
							else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_MULTIP)
							{// *
								USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1] *= SubProgram_Operate[subProNum].Program[ActionLine].Value3;
							}
							else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_DIVIDE)
							{// /
								USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1] /= SubProgram_Operate[subProNum].Program[ActionLine].Value3;
							}
							else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_EXCESS)
							{// %
								USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1] %= SubProgram_Operate[subProNum].Program[ActionLine].Value3;
							}
							break;
						case K_OUTDETECT://������
							break;
						default://����ָ�������쳣
							result = 12;	
							break;
					}
					break;
				case OR_AXISORDER://���ָ��
					switch(SubProgram_Operate[subProNum].Program[ActionLine].Key)
					{
						case K_MDPOSITION://���λ��
							ret = MD_ReadParData();
							if(ret == 1)
							{
								result = 13;
							}
							else
							{
								if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 >= V_MD_AXSIS \
										&& SubProgram_Operate[subProNum].Program[ActionLine].Value1 < V_MD_AXSIS + Axis_Num)
								{//���λ��
									ret = MD_ReadCurPoint(&sPostion, 0);
									if(ret == 1)
									{
										result = 13;
										MD_PositionErr_Flag = TRUE;
										break;
									}
									axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1 - V_MD_AXSIS;
									pointTemp = sPostion.point[axsisNum];
								}
								else if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 >= V_MD_AXSIS + Axis_Num \
										&& SubProgram_Operate[subProNum].Program[ActionLine].Value1 < V_MD_AXSIS + Axis_Num * 2)
								{//�ȴ���λ��
									ret = MD_ReadCurPoint(&sPostion, 1);
									if(ret == 1)
									{
										result = 13;
										MD_PositionErr_Flag = TRUE;
										break;
									}
									axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1 - V_MD_AXSIS - Axis_Num;
									pointTemp = sPostion.waitPoint[axsisNum];
								}
								
								if(axsisNum == O_Axsis && sMD_Parameter.revolveMode == 1)
								{//O��Ϊ����
									if(pointTemp == 0)
									{
										ResetOutput(sMD_Parameter.gasPort);
									}
									else
									{
										SetOutput(sMD_Parameter.gasPort);
									}
								}
								else
								{
									AXisMove(axsisNum, pointTemp, SubProgram_Operate[subProNum].Program[ActionLine].Value2);
//								Axsis_MoveProNum[axsisNum] = 100;
								Axsis_MoveTarPos[axsisNum] = pointTemp;
//								Axsis_MoveTarSpeed[axsisNum] = SubProgram_Operate[subProNum].Program[ActionLine].Value2;
								}
							}
							break;
						case K_MDPOCOUNT://������
							ret = MD_ReadParData();
							if(ret == 1)
							{
								result = 13;
							}
							else
							{								
								ret = MD_StackCount(SubProgram_Operate[subProNum].Program[ActionLine].Value1, SubProgram_Operate[subProNum].Program[ActionLine].Value2, SubProgram_Operate[subProNum].Program[ActionLine].Value3);
								if(ret == 1)
								{
									result = 13;
								}
							}
							break;
						case K_XAXIS://��λ���ƶ�: λ�� , �ٶ�
						case K_ZAXIS:
						case K_LAXIS:
						case K_OAXIS:
							if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_XAXIS) 
							{//X��
								axsisNum = X_Axsis;
								pointTemp = Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Point_X + MINROBOTPOSITION;
							}
							else if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_ZAXIS) 
							{//Z��
								axsisNum = Z_Axsis;
								pointTemp = Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Point_Z + MINROBOTPOSITION;
							}
							else if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_LAXIS)
							{//Y��
								axsisNum = L_Axsis;
								pointTemp = Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Point_L + MINROBOTPOSITION;
							}
							else if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_OAXIS)
							{//O��
								axsisNum = O_Axsis;
								pointTemp = Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Point_O + MINROBOTPOSITION;
							}
							
							if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == 0 || SubProgram_Operate[subProNum].Program[ActionLine].Value1 > 75)
							{
								result = 13;
							}
							else if(Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Flag != 1)
							{
								result = 13;
							}
							else
							{
								AXisMove(axsisNum, pointTemp, SubProgram_Operate[subProNum].Program[ActionLine].Value2);
//								Axsis_MoveProNum[axsisNum] = subProNum + 1;
//								Axsis_MoveTarPos[axsisNum] = pointTemp;
//								Axsis_MoveTarSpeed[axsisNum] = SubProgram_Operate[subProNum].Program[ActionLine].Value2;
							}
							break;
						case K_KEEP_MOVE://��������
						case K_NEGTIVE_SEARCH://��������
							keepMoveIoSta_Low[X_Axsis] = X_AXIS_MOVE_SIGN_LOW_LEVEL;
							keepMoveIoSta_Low[L_Axsis] = Y_AXIS_MOVE_SIGN_LOW_LEVEL;
							keepMoveIoSta_Low[Z_Axsis] = Z_AXIS_MOVE_SIGN_LOW_LEVEL;
							keepMoveIoSta_Low[O_Axsis] = O_AXIS_MOVE_SIGN_LOW_LEVEL;
							keepMoveIoSta_High[X_Axsis] = X_AXIS_MOVE_SIGN_HIGH_LEVEL;
							keepMoveIoSta_High[L_Axsis] = Y_AXIS_MOVE_SIGN_HIGH_LEVEL;
							keepMoveIoSta_High[Z_Axsis] = Z_AXIS_MOVE_SIGN_HIGH_LEVEL;
							keepMoveIoSta_High[O_Axsis] = O_AXIS_MOVE_SIGN_HIGH_LEVEL;

							axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1 - V_KEEP_MOVE_X;
							if(axsisNum < Axis_Num)
							{
								Flag_Keep_Move[axsisNum] = 2;
								if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_KEEP_MOVE)
								{
									Axsis_Move_Direction[axsisNum] = POSITIVE;
								}
								else if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_NEGTIVE_SEARCH)
								{
									Axsis_Move_Direction[axsisNum] = NEGATIVE;
								}
								
								if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_FALLING_EDGE)
								{//�½��أ�����
									if(keepMoveIoSta_Low[axsisNum] == 1)
									{//һ��ʼ��⵽�͵�ƽ
										Flag_Falling_Edge_Sub = 1;
									}
									else if(keepMoveIoSta_High[axsisNum] == 1)
									{//һ��ʼ��⵽�ߵ�ƽ
										Flag_Falling_Edge_Sub = 2;
									}
									Flag_Keep_Move[axsisNum] = 1;
								}
								else if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_RISING_EDGE)
								{//�����أ�����
									if(keepMoveIoSta_Low[axsisNum] == 1)	
									{//һ��ʼ��⵽�͵�ƽ
										Flag_Rising_Edge_Sub = 2;
									}
									else if(keepMoveIoSta_High[axsisNum] == 1)
									{//һ��ʼ��⵽�ߵ�ƽ
										Flag_Rising_Edge_Sub = 1;
									}
									Flag_Keep_Move[axsisNum] = 1;
								}
								else if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_HIGH_LEVEL)
								{//���ߵ�ƽ
									if(keepMoveIoSta_Low[axsisNum] == 1)	
									{//һ��ʼ��⵽�͵�ƽ
										Flag_Keep_Move[axsisNum] = 1;
									}
								}
								else if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_LOW_LEVEL)
								{//���͵�ƽ
									if(keepMoveIoSta_High[axsisNum] == 1)	
									{//һ��ʼ��⵽�ߵ�ƽ
										Flag_Keep_Move[axsisNum] = 1;
									}
								}
								
								if(Flag_Keep_Move[axsisNum] == 1)
								{
									if((SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_LOW_LEVEL) && keepMoveIoSta_Low[axsisNum] == 1)
									{}
									else if((SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_HIGH_LEVEL) && keepMoveIoSta_High[axsisNum] == 1)
									{}
									else
									{
										if(Axsis_Move_Direction[axsisNum] == POSITIVE)
										{
											AXisMove(axsisNum, MAXROBOTPOSITION, SubProgram_Operate[subProNum].Program[ActionLine].Value2);
										}
										else
										{
											AXisMove(axsisNum, MINROBOTPOSITION, SubProgram_Operate[subProNum].Program[ActionLine].Value2);
										}
									}
								}
							}
							break;
						case K_INCREMENT_RUNNING:
							axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1;
							if(V_KEEP_MOVE_X <= axsisNum && axsisNum <= V_KEEP_MOVE_O)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_X;
							}
							else if(V_KEEP_MOVE_U <= axsisNum && axsisNum <= V_KEEP_MOVE_V)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_U + Axis_Num;
							}
							
							if(axsisNum < Axis_Num + Ext_Axis_Num)
							{
								if(Increment_Finished[axsisNum] == FALSE)
								{
									Increment_Finished[axsisNum] = TRUE;
									Increment_Distance = SubProgram_Operate[subProNum].Program[ActionLine].Value3 * Step_Coefficient[axsisNum] / 100;
									Increment_Target[axsisNum] = m_PulseTotalCounter[axsisNum] + Increment_Distance;
								}
								if(Increment_Target[axsisNum] < MINROBOTPOSITION)
								{
									Increment_Target[axsisNum] = MINROBOTPOSITION;
								}
								AXisMove(axsisNum, Increment_Target[axsisNum], SubProgram_Operate[subProNum].Program[ActionLine].Value2);
							}
							break;
						case K_MACHINE_ORIGIN:
							break;
						case K_POSSET:
							axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1;
							if(V_KEEP_MOVE_X <= axsisNum && axsisNum <= V_KEEP_MOVE_O)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_X;
							}
							else if(V_KEEP_MOVE_U <= axsisNum && axsisNum <= V_KEEP_MOVE_V)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_U + Axis_Num;
							}
							m_PositionResetStep[axsisNum] = 0;
							break;
						case K_SLOWPOINT:
							axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1;
							if(V_KEEP_MOVE_X <= axsisNum && axsisNum <= V_KEEP_MOVE_O)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_X;
							}
							else if(V_KEEP_MOVE_U <= axsisNum && axsisNum <= V_KEEP_MOVE_V)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_U + Axis_Num;
							}
							SlowPointIncrement[axsisNum] = (SubProgram_Operate[subProNum].Program[ActionLine].Value3 & 0x0fffffff) * Step_Coefficient[axsisNum] / 100;			//���ٶȵ�����λ��
							SlowPointSpeed[axsisNum] = SubProgram_Operate[subProNum].Program[ActionLine].Value2;																						//���ٶȵ��ٶ�
							SlowPointFlag[axsisNum] = 1;																																																		//������ٶȵ��־
							break;
						case K_INTER_START:
							break;
						case K_INTER_OVER:
							break;
						case K_ADVENCE:
							if(g_Program_Is_Debuging == FALSE && sCartesian_Para.MDCoordType != 2 && (SubProgram_Operate[subProNum].Program[ActionLine].Value3 & 0x0fffffff) > 10)
							{//SCARAģʽ�£���ȷ����Ч
								axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1 - V_KEEP_MOVE_X;
								AdvancePointInc[axsisNum] = (SubProgram_Operate[subProNum].Program[ActionLine].Value3 & 0x0fffffff) * Step_Coefficient[axsisNum] / 100;					//��ǰȷ����
								AdvancePointFlag[axsisNum] = 1;																																											//��ǰȷ������־
							}
							break;
						case K_INTER_LINE:
							break;
						case K_AXISMOVE:
							axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1;
							if(V_KEEP_MOVE_X <= axsisNum && axsisNum <= V_KEEP_MOVE_O)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_X;
							}
							else if(V_KEEP_MOVE_U <= axsisNum && axsisNum <= V_KEEP_MOVE_V)
							{
								axsisNum = axsisNum - V_KEEP_MOVE_U + Axis_Num;
							}
							pointTemp = (SubProgram_Operate[subProNum].Program[ActionLine].Value3 & 0x0fffffff) * Step_Coefficient[axsisNum] / 100 + MINROBOTPOSITION;
							AXisMove(axsisNum, pointTemp, SubProgram_Operate[subProNum].Program[ActionLine].Value2);
							break;
						case K_ANGLE_ARC:
							break;
						default:
							result = 13;	//���ָ�������쳣
							break;
					}
					break;
			  case OR_IOORDER://IOָ��
					switch(SubProgram_Operate[subProNum].Program[ActionLine].Key)
					{//ȫ����Ϊֱ���������⣬һ�������Ӧ����ָ���λ����λ
						case K_IOINSTRUCT_OUTPUT1://Y0-1		
							SetSingle(60,O_IODEBUG_OUTPUT_0,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT2://Y0-0
							SetSingle(O_IODEBUG_OUTPUT_0,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT3://Y1-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_1,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT4://Y1-0
							SetSingle(O_IODEBUG_OUTPUT_1,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT5://Y2-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_2,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT6://Y2-0
							SetSingle(O_IODEBUG_OUTPUT_2,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT7://Y3-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_3,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT8://Y3-0
							SetSingle(O_IODEBUG_OUTPUT_3,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT9://Y4-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_4,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT10://Y4-0
							SetSingle(O_IODEBUG_OUTPUT_4,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT11://Y5-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_5,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT12://Y5-0
							SetSingle(O_IODEBUG_OUTPUT_5,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT13://Y6-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_6,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT14://Y6-0
							SetSingle(O_IODEBUG_OUTPUT_6,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT15://Y7-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_7,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT16://Y7-0
							SetSingle(O_IODEBUG_OUTPUT_7,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT17://Y8-1  
							SetSingle(60,O_IODEBUG_OUTPUT_8,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;						
						case K_IOINSTRUCT_OUTPUT18://Y8-0 
							SetSingle(O_IODEBUG_OUTPUT_8,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;						
						case K_IOINSTRUCT_OUTPUT19://Y9-1 
							SetSingle(60,O_IODEBUG_OUTPUT_9,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;						
						case K_IOINSTRUCT_OUTPUT20://Y9-0 
							SetSingle(O_IODEBUG_OUTPUT_9,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;						
						case K_IOINSTRUCT_OUTPUT21://Y10-1 
							SetSingle(60,O_IODEBUG_OUTPUT_10,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT22://Y10-0 
							SetSingle(O_IODEBUG_OUTPUT_10,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;						
						case K_IOINSTRUCT_OUTPUT23://Y11-1
							SetSingle(60,O_IODEBUG_OUTPUT_11,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT24://Y11-0 
							SetSingle(O_IODEBUG_OUTPUT_11,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;						
						case K_IOINSTRUCT_OUTPUT25://Y12-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_12,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT26://Y12-0
							 SetSingle(O_IODEBUG_OUTPUT_12,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT27://Y13-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_13,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT28://Y13-0
							SetSingle(O_IODEBUG_OUTPUT_13,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT29://Y14-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_14,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT30://Y14-0
							SetSingle(O_IODEBUG_OUTPUT_14,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT31://Y15-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_15,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT32://Y15-0
							SetSingle(O_IODEBUG_OUTPUT_15,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT33://Y16-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_16,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT34://Y16-0
							SetSingle(O_IODEBUG_OUTPUT_16,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT35://Y17-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_17,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT36://Y17-0
							SetSingle(O_IODEBUG_OUTPUT_17,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT37://Y18-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_18,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT38://Y18-0
							SetSingle(O_IODEBUG_OUTPUT_18,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;
						case K_IOINSTRUCT_OUTPUT39://Y19-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_19,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT40://Y19-0
							SetSingle(O_IODEBUG_OUTPUT_19,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;						
						case K_IOINSTRUCT_OUTPUT41://Y20-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_20,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT42://Y20-0
							SetSingle(O_IODEBUG_OUTPUT_20,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;						
						case K_IOINSTRUCT_OUTPUT43://Y21-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_21,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT44://Y21-0
							SetSingle(O_IODEBUG_OUTPUT_21,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;						
						case K_IOINSTRUCT_OUTPUT45://Y22-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_22,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT46://Y22-0
							SetSingle(O_IODEBUG_OUTPUT_22,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;						
						case K_IOINSTRUCT_OUTPUT47://Y23-1								 
							SetSingle(60,O_IODEBUG_OUTPUT_23,SubProgram_Operate[subProNum].Program[ActionLine].Value1);							 
							break;					
						case K_IOINSTRUCT_OUTPUT48://Y23-0
							SetSingle(O_IODEBUG_OUTPUT_23,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);
							break;						
						case K_IOINSTRUCT_OUTPUT49://RY0-1
							SetSingle(60,O_IODEBUG_OUTPUT_28,SubProgram_Operate[subProNum].Program[ActionLine].Value1);	
							break;						
						case K_IOINSTRUCT_OUTPUT50://RY0-0
							SetSingle(O_IODEBUG_OUTPUT_28,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);	
							break;						
						case K_IOINSTRUCT_OUTPUT51://RY1-1
							SetSingle(60,O_IODEBUG_OUTPUT_29,SubProgram_Operate[subProNum].Program[ActionLine].Value1);		
							break;						
						case K_IOINSTRUCT_OUTPUT52://RY1-0
							SetSingle(O_IODEBUG_OUTPUT_29,60,SubProgram_Operate[subProNum].Program[ActionLine].Value1);		
							break;

						//�����źż��
						case K_IOINSTRUCT_INPUT1:	  	//����1-X0
						case K_IOINSTRUCT_INPUT2:	  	//����2-X1
						case K_IOINSTRUCT_INPUT3:	  	//����3-X2
						case K_IOINSTRUCT_INPUT4:	  	//����4-X3
						case K_IOINSTRUCT_INPUT5:	  	//����5-X4							 
						case K_IOINSTRUCT_INPUT6:	  	//����6-X5
						case K_IOINSTRUCT_INPUT7:	  	//����7-X6
						case K_IOINSTRUCT_INPUT8:	  	//����8-X7
						case K_IOINSTRUCT_INPUT9:	  	//����9-X8
						case K_IOINSTRUCT_INPUT10:	  //����10-X9
						case K_IOINSTRUCT_INPUT11:	  //����11-X10
						case K_IOINSTRUCT_INPUT12:	  //����12-X11
						case K_IOINSTRUCT_INPUT13:	  //����13-X12
						case K_IOINSTRUCT_INPUT14:	  //����14-X13
						case K_IOINSTRUCT_INPUT15:	  //����15-X14
						case K_IOINSTRUCT_INPUT16:	  //����16-X15
						case K_IOINSTRUCT_INPUT17:	  //����17-X16
						case K_IOINSTRUCT_INPUT18:	  //����18-X17
						case K_IOINSTRUCT_INPUT19:	  //����19-X18
						case K_IOINSTRUCT_INPUT20:	  //����20-X19
						case K_IOINSTRUCT_INPUT21:	  //����21-X20
						case K_IOINSTRUCT_INPUT22:	  //����22-X21
						case K_IOINSTRUCT_INPUT23:    //����23-X22
						case K_IOINSTRUCT_INPUT24:
						case K_IOINSTRUCT_INPUT25:
						case K_IOINSTRUCT_INPUT26:
						case K_IOINSTRUCT_INPUT27: 
						case K_IOINSTRUCT_INPUT28:
						case K_IOINSTRUCT_INPUT29:
						case K_IOINSTRUCT_INPUT30:
								ionum = SubProgram_Operate[subProNum].Program[ActionLine].Key-K_IOINSTRUCT_INPUT1;
								IO_Input_waittime[ionum] = SubProgram_Operate[subProNum].Program[ActionLine].Value3;
									
								index = ionum / 8;
								offset = ionum % 8;
									
								if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_FALLING_EDGE)
								{//�½��أ�����
									if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
									{//һ��ʼ��⵽�͵�ƽ
										Detect_Falling_Edge_Sub[subProNum] = 1;
									}
									else if(((Input_Detect_Status[index] >> offset) & 0x01) == 0)
									{//һ��ʼ��⵽�ߵ�ƽ
										Detect_Falling_Edge_Sub[subProNum] = 2;
									}
								}
								else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_RISING_EDGE)
								{//�����أ�����
									if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
									{//һ��ʼ��⵽�͵�ƽ
										Detect_Rising_Edge_Sub[subProNum] = 2;
									}
									else if(((Input_Detect_Status[index] >> offset) & 0x01) == 0)	
									{//һ��ʼ��⵽�ߵ�ƽ
										Detect_Rising_Edge_Sub[subProNum] = 1;
									}
								}
							break;
						default://IOָ�������쳣
							result = 14;	
							break;
					}
					break;
				default://��Ҫָ�������쳣
					result = 11;
					break;
			}//switch(order)
			
			if(result < 9)
			{
				SubAction_Step_Run_Num[subProNum]++;
				if(SubAction_Step_List_Num[subProNum] >= SubAction_Step_Run_Num[subProNum])
				{
					result = SubAction_Step_List_Num[subProNum] - SubAction_Step_Run_Num[subProNum];//��������ָ����-������ָ��������=0�򵥴��������
				}
				else
				{
					result = 11;
				}
			}
			
		}//if(ActionAllowJudge(SubProgram_Operate[subProNum], ActionLine) == 0)
		else
		{
			result = 11; //��Ҫָ�������쳣
		}
	}//if(ActionLine < SubProgram_Operate[subProNum].Num)
	else
	{
		result = 10;				//��ǰ�кų����쳣
	}
	return result;		
}


/**************************************************************************************************
**  ��������  AutoActionOutConfirm()
**	�����������Ҫ���е��к�
**	��������������Ƿ���ȷ��
**	�������ܣ��Զ�ģʽ���ƺ���
**	��ע��    �����豸�Զ�����
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u8 AutoActionOutConfirm(SaveProgram *Program_Operate, u8 ActionLine)
{
//	u8 Temp_data[8]={0};
	u8 result = 0;
	u8 SubProgramNum = 0;
	u32 pointTemp = 0;
	u8 axsisNum = 0;
	u8 keepMoveIoSta_Low[Axis_Num] = {0};					//������������Ӧ����ڵ�״̬-��
	u8 keepMoveIoSta_High[Axis_Num] = {0};					//������������Ӧ����ڵ�״̬-��
	u8 ionum = 0;
	u8 index = 0;
	u8 offset = 0;
	u8 signValue = 0;
	u8 axisNum = 0;

	switch(Program_Operate->Program[ActionLine].Key)
	{
		case K_PROGRAMSTART://����ʼ
			result = 1;
			break;
		case K_PROGRAMEND://�������
			 result = 1;
			break;
		case K_DELAY://��ʱ�Ƿ����
			 if(Key_Delay(Program_Operate->Program[ActionLine].Value2))
			 {
					result = 1;
			 }
			break;
		case K_SUBPROGRAM://�ӳ���  ��ʼ-����		     
			 if(g_Program_Is_Debuging == FALSE)
			 {
				 SubProgramNum = Program_Operate->Program[ActionLine].Value1 - 1;//��ȡ�ӳ�����
				 if(Program_Operate->Program[ActionLine].Value2 == V_PROGRAM_START)
				 {//�ӳ���ʼ
						 result = 1;
				 }
				 else
				 {//�ӳ������
					 if(g_SubProgram_Finish[SubProgramNum] == FALSE)
					 {
						 g_SubProgram_ContralEnd[SubProgramNum] = TRUE;
					 }
					 else
					 {
						 g_SubProgram_Finish[SubProgramNum] = FALSE;
						 g_SubProgram_Step_Run[SubProgramNum] = FALSE;
						 g_Read_SubProgram[SubProgramNum] = FALSE;
						 result = 1;
					 }
				 }
			 }
			 else
			 {
				 result = 1;
			 }
			break;
		case K_JUMP://��תָ��
			result = 1;
			break;
		case K_WHILE://WHILE����
			m_WhileJudgeRes[m_WhileNC - 1] = 0;									//Ĭ�����ó�����������
			if(m_WhileRunFlag[m_WhileNC - 1] == 1)
			{
				if(m_WhileJudgeType[m_WhileNC - 1] == V_R_METHOD)
				{//R�ж�
					if(m_WhileCycCounter[m_WhileNC - 1] < m_WhileJudgePar2[m_WhileNC - 1])
					{//�ж���������
						m_WhileJudgeRes[m_WhileNC - 1] = 1;
					}
				}
				else if(m_WhileJudgeType[m_WhileNC - 1] == V_I_METHOD)
				{//I�ж�
					if(m_WhileJudgePar1[m_WhileNC - 1] < INPUT_NUM)
					{
						if(m_WhileJudgePar2[m_WhileNC - 1] == V_LOW_LEVEL && ReadInput(m_WhileJudgePar1[m_WhileNC - 1]) == 0)
						{//�����Ч��ƽ
							m_WhileJudgeRes[m_WhileNC - 1] = 1;
						}
						else if(m_WhileJudgePar2[m_WhileNC - 1] == V_HIGH_LEVEL && ReadInput(m_WhileJudgePar1[m_WhileNC - 1]) == 1)
						{//�����Ч��ƽ
							m_WhileJudgeRes[m_WhileNC - 1] = 1;
						}
					}
				}
				else if(V_USER1<=m_WhileJudgeType[m_WhileNC - 1] && m_WhileJudgeType[m_WhileNC - 1]<=V_USER8)
				{//user1-user8
					if((USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] != 0) && (m_WhileJudgePar1[m_WhileNC - 1] == V_EQUAL)\
						&& (USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] % m_WhileJudgePar2[m_WhileNC - 1]) == 0)	
					{//user��ǰֵ = R�����ı���
						m_WhileJudgeRes[m_WhileNC - 1] = 1;
					}
					else if((m_WhileJudgePar1[m_WhileNC - 1] == V_ONLY_EQUAL) && (USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] == m_WhileJudgePar2[m_WhileNC - 1]))	
					{//user��ǰֵ = R����
						m_WhileJudgeRes[m_WhileNC - 1] = 1;
					}	
					else if((m_WhileJudgePar1[m_WhileNC - 1] == V_NOT_EQUAL) && (USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] != m_WhileJudgePar2[m_WhileNC - 1]))	
					{//user��ǰֵ != R����
						m_WhileJudgeRes[m_WhileNC - 1] = 1;
					}
					else if((m_WhileJudgePar1[m_WhileNC - 1] == V_NOTONLY_EQUAL) && (USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] % m_WhileJudgePar2[m_WhileNC - 1]) != 0)	
					{//user��ǰֵ != R�����ı���ʱ
						m_WhileJudgeRes[m_WhileNC - 1] = 1;
					}
				}
			}
			result = 1;
			break;
		case K_CYCLEOVER://ѭ������
			if(m_WhileRunFlag[m_WhileNC - 1] == 1)
			{
				if(m_WhileJudgeType[m_WhileNC - 1] == V_R_METHOD)
				{//R�ж�
					m_WhileCycCounter[m_WhileNC - 1]++;
				}
			}
			result = 1;
			break;
		case K_IF://IF�ж�����
		case K_ELSE://else�ж�����������ߴ���ʽ��ͬ
			m_IfElseJudgeRes[m_IfElseNC - 1] = 0;																//Ĭ���ж�����ʧ��
			if((V_USER1<=m_IfElseJudgeType[m_IfElseNC - 1])&&(m_IfElseJudgeType[m_IfElseNC - 1]<= V_USER8))
			{//�û������ж�
				if((USER_Parameter.CURR_Num[m_IfElseJudgeType[m_IfElseNC - 1]-V_USER1] != 0) && (m_IfElseJudgePar1[m_IfElseNC - 1] == V_EQUAL)\
						&& (USER_Parameter.CURR_Num[m_IfElseJudgeType[m_IfElseNC - 1]-V_USER1] % m_IfElseJudgePar2[m_IfElseNC - 1]) == 0)	
				{//user��ǰֵ = R�����ı���
					m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
				}
				else if((m_IfElseJudgePar1[m_IfElseNC - 1] == V_ONLY_EQUAL) && (USER_Parameter.CURR_Num[m_IfElseJudgeType[m_IfElseNC - 1]-V_USER1] == m_IfElseJudgePar2[m_IfElseNC - 1]))	
				{//user��ǰֵ = R����
					m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
				}
				else if((m_IfElseJudgePar1[m_IfElseNC - 1] == V_NOT_EQUAL) && (USER_Parameter.CURR_Num[m_IfElseJudgeType[m_IfElseNC - 1]-V_USER1] != m_IfElseJudgePar2[m_IfElseNC - 1]))	
				{//user��ǰֵ != R����
					m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
				}
				else if((m_IfElseJudgePar1[m_IfElseNC - 1] == V_NOTONLY_EQUAL) && (USER_Parameter.CURR_Num[m_IfElseJudgeType[m_IfElseNC - 1]-V_USER1] % m_IfElseJudgePar2[m_IfElseNC - 1]) != 0)	
				{//user��ǰֵ != R�����ı���ʱ
					m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
				}
			}
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_I_METHOD)
			{//I�ж�
				if(m_IfElseJudgePar1[m_IfElseNC - 1] < INPUT_NUM)
				{
					if(m_IfElseJudgePar2[m_IfElseNC - 1] == V_LOW_LEVEL && ReadInput(m_IfElseJudgePar1[m_IfElseNC - 1]) == 0)
					{//�����Ч��ƽ
						m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
					}
					else if(m_IfElseJudgePar2[m_IfElseNC - 1] == V_HIGH_LEVEL && ReadInput(m_IfElseJudgePar1[m_IfElseNC - 1]) == 1)
					{//�����Ч��ƽ
						m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
					}
				}
			}
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_O_METHOD)
			{//O�ж�
				if(m_IfElseJudgePar1[m_IfElseNC - 1] < OUTPUT_NUM)
				{
					ionum = m_IfElseJudgePar1[m_IfElseNC - 1];
					index = ionum / 8;
					offset = ionum % 8;
					if(m_IfElseJudgePar2[m_IfElseNC - 1] == V_RESET && ((Output_Status[index] >> offset) & 0x01) == 0x01)
					{//��⸴λ
						m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
					}
					else if(m_IfElseJudgePar2[m_IfElseNC - 1] == V_SET && ((Output_Status[index] >> offset) & 0x01) == 0)
					{//�����λ
						m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
					}
				}
			}
			
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_P_METHOD)
			{//P����
				if(m_IfElseJudgePar1[m_IfElseNC - 1] >= V_XAXISGREATER && \
					m_IfElseJudgePar1[m_IfElseNC - 1] <= V_OAXISGREATER)
				{//���ڵ���
					axisNum = m_IfElseJudgePar1[m_IfElseNC - 1] - V_XAXISGREATER;
					pointTemp = m_IfElseJudgePar2[m_IfElseNC - 1] * Step_Coefficient[axisNum] / 100 + MINROBOTPOSITION;
					if(m_PulseTotalCounter[axisNum] + JDZ_AllowError >= pointTemp)
					{//�ж���������
						m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
					}
				}
				else if(m_IfElseJudgePar1[m_IfElseNC - 1] >= V_XAXISEQUAL && \
					m_IfElseJudgePar1[m_IfElseNC - 1] <= V_OAXISEQUAL)
				{//����
					axisNum = m_IfElseJudgePar1[m_IfElseNC - 1] - V_XAXISEQUAL;
					pointTemp = m_IfElseJudgePar2[m_IfElseNC - 1] * Step_Coefficient[axisNum] / 100 + MINROBOTPOSITION;
					if(m_PulseTotalCounter[axisNum] + JDZ_AllowError >= pointTemp && m_PulseTotalCounter[axisNum] <= pointTemp + JDZ_AllowError)
					{//�ж���������
						m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
					}
				}
				else if(m_IfElseJudgePar1[m_IfElseNC - 1] >= V_XAXISLESS && \
					m_IfElseJudgePar1[m_IfElseNC - 1] <= V_OAXISLESS)
				{//С�ڵ���
					axisNum = m_IfElseJudgePar1[m_IfElseNC - 1] - V_XAXISLESS;
					pointTemp = m_IfElseJudgePar2[m_IfElseNC - 1] * Step_Coefficient[axisNum] / 100 + MINROBOTPOSITION;
					if(m_PulseTotalCounter[axisNum] <= pointTemp + JDZ_AllowError)
					{//�ж���������
						m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
					}
				}
			}
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_LAYER_NUM)
			{//�����ж�
				m_IfElseJudgeRes[m_IfElseNC - 1] = MD_LayerNumJudge(m_IfElseJudgePar1[m_IfElseNC - 1], m_IfElseJudgePar2[m_IfElseNC - 1]);
			}
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_LAYER_FULL)
			{//�����ж�
				m_IfElseJudgeRes[m_IfElseNC - 1] = MD_LayerFullJudge(m_IfElseJudgePar1[m_IfElseNC - 1], m_IfElseJudgePar2[m_IfElseNC - 1]);
			}
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_STACK_FULL)
			{//�����ж�
				m_IfElseJudgeRes[m_IfElseNC - 1] = MD_StackFullJudge(m_IfElseJudgePar1[m_IfElseNC - 1], m_IfElseJudgePar2[m_IfElseNC - 1]);
			}
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_MDGOOD)
			{//��Ʒ�ж�
				m_IfElseJudgeRes[m_IfElseNC - 1] = MD_GoodNumJudge(m_IfElseJudgePar1[m_IfElseNC - 1], m_IfElseJudgePar2[m_IfElseNC - 1]);
			}
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_RI_NULL)
			{//NULL��ת
				m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
			}
			result = 1;
			break;
		case K_OVER:
			result = 1;
			break;
		case K_SPECIAL://����ָ��							
			if(Program_Operate->Program[ActionLine].Value1 == V_SUSPEND)
			{
				result = 1;
			}
			else if(Program_Operate->Program[ActionLine].Value1 == V_STOP)
			{
				result = 1;
			}
			break;	
		case K_PULSE_OUTPUT:		       //�������
			result = 1;	
			break;
		case K_USER:		              //�û�����		
//			if(USER_Parameter.ELEC_RESET[Program_Operate->Program[ActionLine].Value1-V_USER1] == 0 \
//				&& USER_Parameter.START_RESET[Program_Operate->Program[ActionLine].Value1-V_USER1] == 0)
//			{
//				Temp_data[0] = USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1];
//				Temp_data[1] = USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1]>>8;
//				Temp_data[2] = USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1]>>16;
//				Temp_data[3] = USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1]>>24;
//				W25QXX_Write(Temp_data,P_USER_ADDRESS + 17 + 23*(Program_Operate->Program[ActionLine].Value1-V_USER1),4);
//			}
			result = 1;	
			break;
		case K_OUTDETECT://������
			if(Program_Operate->Program[ActionLine].Value1 < OUTPUT_NUM)
			{
				ionum = Program_Operate->Program[ActionLine].Value1;
//				if(ionum>=24)ionum += 4;//RY0-RY1
				index = ionum / 8;
				offset = ionum % 8;
				if(Program_Operate->Program[ActionLine].Value2 == V_RESET && ((Output_Status[index] >> offset) & 0x01) == 0x01)
				{//��λ
					result = 1;
				}
				else if(Program_Operate->Program[ActionLine].Value2 == V_SET && ((Output_Status[index] >> offset) & 0x01) == 0)
				{//��λ
					result = 1;
				}
			}
			break;
 		case K_MDPOSITION://ȷ����������Ƿ�λ
			if(Program_Operate->Program[ActionLine].Value1 >= V_MD_AXSIS \
					&& Program_Operate->Program[ActionLine].Value1 < V_MD_AXSIS + Axis_Num)
			{//���λ��
				axsisNum = Program_Operate->Program[ActionLine].Value1 - V_MD_AXSIS;
			}
			else if(Program_Operate->Program[ActionLine].Value1 >= V_MD_AXSIS + Axis_Num \
					&& Program_Operate->Program[ActionLine].Value1 < V_MD_AXSIS + Axis_Num * 2)
			{//�ȴ���λ��
				axsisNum = Program_Operate->Program[ActionLine].Value1 - V_MD_AXSIS - Axis_Num;
			}
			
			if(axsisNum == O_Axsis && sMD_Parameter.revolveMode == 1)
			{//O��Ϊ����
				result = 1;
			}
			else
			{
				if(Servo_MoveFinishSta(axsisNum, Axsis_MoveTarPos[axsisNum]))
				{
					result = Judge_JDZ_Error(axsisNum);
				}
			}
			break;
		case K_MDPOCOUNT://������
			result = 1;	
			break;
		case K_XAXIS://���Ƿ��ƶ���Ŀ��λ��
		case K_ZAXIS:
		case K_LAXIS:
		case K_OAXIS:
			if(Program_Operate->Program[ActionLine].Key == K_XAXIS) 
			{//X��
				axsisNum = X_Axsis;
				pointTemp = Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Point_X + MINROBOTPOSITION;
			}
			else if(Program_Operate->Program[ActionLine].Key == K_ZAXIS) 
			{//Z��
				axsisNum = Z_Axsis;
				pointTemp = Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Point_Z + MINROBOTPOSITION;
			}
			else if(Program_Operate->Program[ActionLine].Key == K_LAXIS)
			{//Y��
				axsisNum = L_Axsis;
				pointTemp = Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Point_L + MINROBOTPOSITION;
			}
			else if(Program_Operate->Program[ActionLine].Key == K_OAXIS)
			{//O��
				axsisNum = O_Axsis;
				pointTemp = Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Point_O + MINROBOTPOSITION;
			}
			
			if(Program_Operate->Program[ActionLine].Value1 == 0 || Program_Operate->Program[ActionLine].Value1 > 75)
			{
				result = 1;
			}
			else if(Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Flag != 1)
			{
				result = 1;
			}
			else
			{
				if(Servo_MoveFinishSta(axsisNum, pointTemp))
				{
					result = Judge_JDZ_Error(axsisNum);
				}
			}
			break;
		case K_KEEP_MOVE://��������
		case K_NEGTIVE_SEARCH://��������
			keepMoveIoSta_Low[X_Axsis] = X_AXIS_MOVE_SIGN_LOW_LEVEL;
			keepMoveIoSta_Low[L_Axsis] = Y_AXIS_MOVE_SIGN_LOW_LEVEL;
			keepMoveIoSta_Low[Z_Axsis] = Z_AXIS_MOVE_SIGN_LOW_LEVEL;
			keepMoveIoSta_Low[O_Axsis] = O_AXIS_MOVE_SIGN_LOW_LEVEL;
			keepMoveIoSta_High[X_Axsis] = X_AXIS_MOVE_SIGN_HIGH_LEVEL;
			keepMoveIoSta_High[L_Axsis] = Y_AXIS_MOVE_SIGN_HIGH_LEVEL;
			keepMoveIoSta_High[Z_Axsis] = Z_AXIS_MOVE_SIGN_HIGH_LEVEL;
			keepMoveIoSta_High[O_Axsis] = O_AXIS_MOVE_SIGN_HIGH_LEVEL;

			axsisNum = Program_Operate->Program[ActionLine].Value1 - V_KEEP_MOVE_X;
			if(axsisNum < Axis_Num && Flag_Keep_Move[axsisNum] == 1)
			{
				if(Program_Operate->Program[ActionLine].Value3 == V_LOW_LEVEL)
				{//�͵�ƽ
					if(keepMoveIoSta_Low[axsisNum])
					{//�յ��͵�ƽ�ź�
						Servo_Stop(axsisNum);
						Flag_Keep_Move[axsisNum] = 2;
					}
				}
				else if(Program_Operate->Program[ActionLine].Value3 == V_HIGH_LEVEL)
				{//�ߵ�ƽ
					if(keepMoveIoSta_High[axsisNum])	
					{//�յ��ߵ�ƽ�ź�
						Servo_Stop(axsisNum);
							Flag_Keep_Move[axsisNum] = 2;
					}
				}
				else if(Program_Operate->Program[ActionLine].Value3 == V_FALLING_EDGE)
				{//�½��أ�����
					if(Flag_Falling_Edge == 1)
					{//һ��ʼ��⵽�͵�ƽ
						if(keepMoveIoSta_High[axsisNum])
						{//��⵽�ߵ�ƽ
							Flag_Falling_Edge = 3;
						}
					}
					if(Flag_Falling_Edge == 2 || Flag_Falling_Edge == 3)
					{//һ��ʼ��⵽�ߵ�ƽ,�����ٴμ�⵽��
						if(keepMoveIoSta_Low[axsisNum])
						{//��⵽�͵�ƽ,ֹͣ
							Servo_Stop(axsisNum);
							Flag_Keep_Move[axsisNum] = 2;
						}
					}
				}
				else if(Program_Operate->Program[ActionLine].Value3 == V_RISING_EDGE)
				{//�����أ�����
					if(Flag_Rising_Edge == 1)
					{//һ��ʼ��⵽�ߵ�ƽ
						if(keepMoveIoSta_Low[axsisNum])
						{//��⵽�͵�ƽ
							Flag_Rising_Edge = 3;
						}
					}
					if(Flag_Rising_Edge == 2 || Flag_Rising_Edge == 3)
					{//һ��ʼ��⵽�͵�ƽ,�����ٴμ�⵽��
						if(keepMoveIoSta_High[axsisNum])
						{//��⵽�ߵ�ƽ,ֹͣ
							Servo_Stop(axsisNum);
							Flag_Keep_Move[axsisNum] = 2;
						}
					}
				}
			}
			
			if(Flag_Keep_Move[axsisNum] == 2)
			{
				result = 1;
				Flag_Keep_Move[axsisNum] = 0;
			}
			break;
		case K_INCREMENT_RUNNING://�Ƿ��ƶ���Ŀ��λ��
			axsisNum = Program_Operate->Program[ActionLine].Value1;
			if(V_KEEP_MOVE_X <= axsisNum && axsisNum <= V_KEEP_MOVE_O)
			{
				axsisNum = axsisNum - V_KEEP_MOVE_X;
			}
			else if(V_KEEP_MOVE_U <= axsisNum && axsisNum <= V_KEEP_MOVE_V)
			{
				axsisNum = axsisNum - V_KEEP_MOVE_U + Axis_Num;
			}
			
			if(axsisNum < Axis_Num + Ext_Axis_Num)
			{
				if(Servo_MoveFinishSta(axsisNum, Increment_Target[axsisNum]))
				{
					result = Judge_JDZ_Error(axsisNum);
					if(result ==1)
					{
						Increment_Finished[axsisNum] = FALSE;
					}
				}
			}
			break;
		case K_MACHINE_ORIGIN:
			result = 1;
			break;
		case K_POSSET:
			axsisNum = Program_Operate->Program[ActionLine].Value1;
			if(V_KEEP_MOVE_X <= axsisNum && axsisNum <= V_KEEP_MOVE_O)
			{
				axsisNum = axsisNum - V_KEEP_MOVE_X;
			}
			else if(V_KEEP_MOVE_U <= axsisNum && axsisNum <= V_KEEP_MOVE_V)
			{
				axsisNum = axsisNum - V_KEEP_MOVE_U + Axis_Num;
			}
			pointTemp = Program_Operate->Program[ActionLine].Value2 * Step_Coefficient[axsisNum] / 100 + MINROBOTPOSITION;
			result = Position_Reset(axsisNum, pointTemp);
			break;
		case K_SLOWPOINT:
			result = 1;
			break;
		case K_INTER_START:
			result = 1;	
			break;
		case K_INTER_OVER:
			result = 1;	
			break;
		case K_ADVENCE:
			result = 1;	
			break;
		case K_INTER_LINE:
			result = 1;	
			break;
		case K_AXISMOVE:
			axsisNum = Program_Operate->Program[ActionLine].Value1;
			if(V_KEEP_MOVE_X <= axsisNum && axsisNum <= V_KEEP_MOVE_O)
			{
				axsisNum = axsisNum - V_KEEP_MOVE_X;
			}
			else if(V_KEEP_MOVE_U <= axsisNum && axsisNum <= V_KEEP_MOVE_V)
			{
				axsisNum = axsisNum - V_KEEP_MOVE_U + Axis_Num;
			}
			pointTemp = (Program_Operate->Program[ActionLine].Value3 & 0x0fffffff) * Step_Coefficient[axsisNum] / 100 + MINROBOTPOSITION;
			if(Servo_MoveFinishSta(axsisNum, pointTemp))
			{
				result = Judge_JDZ_Error(axsisNum);
			}
			break;
		case K_ANGLE_ARC:
			result = 1;	
			break;
		
		//����ӿ�-�޼��
		case K_IOINSTRUCT_OUTPUT1:		 	//Y0-1
		case K_IOINSTRUCT_OUTPUT2:		 	//Y0-0
 		case K_IOINSTRUCT_OUTPUT3:		 	//Y1-1
		case K_IOINSTRUCT_OUTPUT4:		 	//Y1-0
		case K_IOINSTRUCT_OUTPUT5:		 	//Y2-1
		case K_IOINSTRUCT_OUTPUT6:		  //Y2-0
		case K_IOINSTRUCT_OUTPUT7:		  //Y3-1
		case K_IOINSTRUCT_OUTPUT8:		  //Y3-0
		case K_IOINSTRUCT_OUTPUT9:		 	//Y4-1
		case K_IOINSTRUCT_OUTPUT10:			//Y4-0
		case K_IOINSTRUCT_OUTPUT11:			//Y5-1
		case K_IOINSTRUCT_OUTPUT12:			//Y5-0
		case K_IOINSTRUCT_OUTPUT13:			//Y6-1
		case K_IOINSTRUCT_OUTPUT14:			//Y6-0
		case K_IOINSTRUCT_OUTPUT15:			//Y7-1
		case K_IOINSTRUCT_OUTPUT16:			//Y7-0
		case K_IOINSTRUCT_OUTPUT17:			//Y8-1
		case K_IOINSTRUCT_OUTPUT18:			//Y8-0
		case K_IOINSTRUCT_OUTPUT19:			//Y9-1
		case K_IOINSTRUCT_OUTPUT20:			//Y9-0
		case K_IOINSTRUCT_OUTPUT21:		 //Y10-1
		case K_IOINSTRUCT_OUTPUT22:		 //Y10-0
		case K_IOINSTRUCT_OUTPUT23:		 //Y11-1
		case K_IOINSTRUCT_OUTPUT24:		 //Y11-0
		case K_IOINSTRUCT_OUTPUT25:		 //Y12-1
		case K_IOINSTRUCT_OUTPUT26:		 //Y12-0
		case K_IOINSTRUCT_OUTPUT27:		 //Y13-1
		case K_IOINSTRUCT_OUTPUT28:		 //Y13-0
		case K_IOINSTRUCT_OUTPUT29:		 //Y14-1
		case K_IOINSTRUCT_OUTPUT30:		 //Y14-0
		case K_IOINSTRUCT_OUTPUT31:		 //Y15-1
		case K_IOINSTRUCT_OUTPUT32:		 //Y15-0
		case K_IOINSTRUCT_OUTPUT33:		 //Y16-1
		case K_IOINSTRUCT_OUTPUT34:		 //Y16-0
		case K_IOINSTRUCT_OUTPUT35:		 //Y17-1
		case K_IOINSTRUCT_OUTPUT36:		 //Y17-0
		case K_IOINSTRUCT_OUTPUT37:		 //Y18-1
		case K_IOINSTRUCT_OUTPUT38:		 //Y18-0
		case K_IOINSTRUCT_OUTPUT39:		 //Y19-1
		case K_IOINSTRUCT_OUTPUT40:		 //Y19-0
		case K_IOINSTRUCT_OUTPUT41:		 //Y20-1
		case K_IOINSTRUCT_OUTPUT42:		 //Y20-0
		case K_IOINSTRUCT_OUTPUT43:		 //Y21-1
		case K_IOINSTRUCT_OUTPUT44:		 //Y21-0
		case K_IOINSTRUCT_OUTPUT45:		 //Y22-1
		case K_IOINSTRUCT_OUTPUT46:		 //Y22-0
		case K_IOINSTRUCT_OUTPUT47:		 //Y23-1
		case K_IOINSTRUCT_OUTPUT48:		 //Y23-0
		case K_IOINSTRUCT_OUTPUT49:		 //RY0-1
		case K_IOINSTRUCT_OUTPUT50:		 //RY0-0
		case K_IOINSTRUCT_OUTPUT51:		 //RY1-1
		case K_IOINSTRUCT_OUTPUT52:		 //RY1-0
      result = 1;	
			break;

		//����ӿڼ��
		case K_IOINSTRUCT_INPUT1://X0
		case K_IOINSTRUCT_INPUT2://X1		
		case K_IOINSTRUCT_INPUT3://X2		
		case K_IOINSTRUCT_INPUT4://X3
		case K_IOINSTRUCT_INPUT5://X4	
		case K_IOINSTRUCT_INPUT6://X5
		case K_IOINSTRUCT_INPUT7://X6
		case K_IOINSTRUCT_INPUT8://X7
		case K_IOINSTRUCT_INPUT9://X8
		case K_IOINSTRUCT_INPUT10://X9
		case K_IOINSTRUCT_INPUT11://X10
		case K_IOINSTRUCT_INPUT12://X11
		case K_IOINSTRUCT_INPUT13://X12
		case K_IOINSTRUCT_INPUT14://X13
		case K_IOINSTRUCT_INPUT15://X14
		case K_IOINSTRUCT_INPUT16://X15
		case K_IOINSTRUCT_INPUT17://X16
		case K_IOINSTRUCT_INPUT18://X17
		case K_IOINSTRUCT_INPUT19://X18
		case K_IOINSTRUCT_INPUT20://X19
		case K_IOINSTRUCT_INPUT21://X20
		case K_IOINSTRUCT_INPUT22://X21
		case K_IOINSTRUCT_INPUT23://X22
		case K_IOINSTRUCT_INPUT24://X23
		case K_IOINSTRUCT_INPUT25://X24
		case K_IOINSTRUCT_INPUT26://X25
		case K_IOINSTRUCT_INPUT27://X26
		case K_IOINSTRUCT_INPUT28://X27
		case K_IOINSTRUCT_INPUT29://X28
		case K_IOINSTRUCT_INPUT30://X29	
			ionum = Program_Operate->Program[ActionLine].Key-K_IOINSTRUCT_INPUT1;
			
			g_Auto_ActionNcWait_Flag = ionum + 1;	
			if(Program_Operate->Program[ActionLine].Value2 == V_USEFUL)	
			{
				signValue = GetIO_Value(ionum, 1, 0);	 
			}
			else if(Program_Operate->Program[ActionLine].Value2 == V_USELESS)	
			{
				signValue = GetIO_Value(ionum, 0, 0);	 
			}
			else if(Program_Operate->Program[ActionLine].Value2 == V_FALLING_EDGE)
			{
				signValue = GetIO_Value(ionum, 2, 0);	 
			}
			else if(Program_Operate->Program[ActionLine].Value2 == V_RISING_EDGE)	
			{
				signValue = GetIO_Value(ionum, 3, 0);	 
			}
			
			if(Program_Operate->Program[ActionLine].Value2 == V_FALLING_EDGE)
			{//����½��� �뱣��ʱ���޹�
				if(signValue == 1)
				{//������Ч�ź�
					result = 1;
					Detect_Falling_Edge = 0;
				}
			}
			else if(Program_Operate->Program[ActionLine].Value2 == V_RISING_EDGE)
			{//��������� �뱣��ʱ���޹�
				if(signValue == 1)
				{//������Ч�ź�
					result = 1;
					Detect_Rising_Edge = 0;
				}
			}
			else
			{
				if(signValue == 1)
				{//������Ч�ź�
					if(IO_Input_keepMin[ionum] >= IO_Input_keepMax[ionum])
					{//��Чʱ�����ʼʱ�������ֹʱ�䣬��Ϊ��Чʱ��Ϊ0��һ��������Ч�ź�����ȷ�ϳɹ�
						result = 1;		
					}
					
					if(g_Auto_Valid_Timer > IO_Input_keepMax[ionum])
					{//��Ч�ź�ʱ�䳬����󱣳�ʱ��,�رռ�ʱ�������Ѽ�ʱ�����㣬��Ϊ�´γ�����Ч�ź�ʱ����0��ʼ��ʱ
						g_Auto_Valid_Timer = 0;
						g_Auto_Valid_Flag = FALSE;
					}
					
					if(g_Auto_Valid_Flag == FALSE)
					{//��ʼ����
						g_Auto_Valid_Timer = 0;
						g_Auto_Valid_Flag = TRUE;
					}
				}
				else
				{//����λ����������0��ʼ
					if(g_Auto_Valid_Timer > IO_Input_keepMin[ionum] && g_Auto_Valid_Timer < IO_Input_keepMax[ionum])
					{//��Чʱ�����������Сʱ�䣬С�����ʱ��
						result = 1;
					}
					g_Auto_Valid_Timer = 0;
					g_Auto_Valid_Flag = FALSE;
				}
			}
			break;		
		default://ָ�������쳣
			 result = 14;	
			 break;
	}
	
	return result;		
}


/**************************************************************************************************
**  ��������  SubProgramActionOutConfirm()
**	�����������Ҫ���е��к�
**	��������������Ƿ���ȷ��
**	�������ܣ��Զ�ģʽ���ƺ���-����ȷ�� 0δ���� 1��� ����ֵΪ�쳣
**	��ע��    
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u8 SubProgramActionOutConfirm(u8 subProNum, u8 ActionLine)
{
//	u8 Temp_data[8]={0};
	u8 result = 0;
	u8 SubProgramNum = 0;
	u32 pointTemp = 0;
	u8 axsisNum = 0;
	u8 keepMoveIoSta_Low[Axis_Num] = {0};					//������������Ӧ����ڵ�״̬-��
	u8 keepMoveIoSta_High[Axis_Num] = {0};					//������������Ӧ����ڵ�״̬-��
	u8 ionum = 0;
	u8 index = 0;
	u8 offset = 0;
	u8 signValue = 0;
	u8 axisNum = 0;
	
	switch(SubProgram_Operate[subProNum].Program[ActionLine].Key)
	{
		case K_PROGRAMSTART://������ʼ
			result = 1;
			break;
		case K_PROGRAMEND://���������
			result = 1;
			break;
		case K_DELAY://��ʱ�Ƿ����
			if(SubProgram_Key_Delay(subProNum, SubProgram_Operate[subProNum].Program[ActionLine].Value2))
			{
				result = 1;
			}
			break;
		case K_SUBPROGRAM://�ӳ���  ��ʼ-����		     
			if(g_Program_Is_Debuging == FALSE)
			{
				SubProgramNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1;//��ȡ�ӳ�����
				if(SubProgramNum == subProNum)
				{//�ӳ���ʼ
					result = 1;
				}
				else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_PROGRAM_START)
				{//�ӳ���ʼ
					result = 1;
				}
				else
				{//�ӳ������
					if(g_SubProgram_Finish[SubProgramNum] == FALSE)
					{
						g_SubProgram_ContralEnd[SubProgramNum] = TRUE;
					}
					else
					{
						g_SubProgram_Finish[SubProgramNum] = FALSE;
						g_SubProgram_Step_Run[SubProgramNum] = FALSE;
						g_Read_SubProgram[SubProgramNum] = FALSE;
						result = 1;
					}
				}
			}
			else
			{
				result = 1;
			}
			break;
		case K_JUMP://��תָ��
			result = 1;
			break;
		case K_WHILE:
			m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 0;									//Ĭ�����ó�����������
			if(m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] == 1)
			{
				if(m_WhileSubJudgeType[subProNum][m_WhileSubNC[subProNum] - 1] == V_R_METHOD)
				{//R�ж�
					if(m_WhileSubCycCounter[subProNum][m_WhileSubNC[subProNum] - 1] < m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1])
					{//�ж���������
						m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
					}
				}
				else if(m_WhileSubJudgeType[subProNum][m_WhileSubNC[subProNum] - 1] == V_I_METHOD)
				{//I�ж�
					if(m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1] < INPUT_NUM)
					{
						if(m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1] == V_LOW_LEVEL && \
								ReadInput(m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1]) == 0)
						{//�����Ч��ƽ
							m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
						}
						else if(m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1] == V_HIGH_LEVEL && \
											ReadInput(m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1]) == 1)
						{//�����Ч��ƽ
							m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
						}
					}
				}
				else if(V_USER1<=m_WhileSubJudgeType[subProNum][m_WhileSubNC[subProNum] - 1] && m_WhileSubJudgeType[subProNum][m_WhileSubNC[subProNum] - 1]<=V_USER8)
				{//user1-user8
					if((USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1] != 0) && (m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1]== V_EQUAL)\
					&& (USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1] % m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1]) == 0)	
					{//user��ǰֵ = R�����ı���
						m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
					}
					else if((m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1] == V_ONLY_EQUAL) && (USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1]  == m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1]))
					{//user��ǰֵ = R����
						m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
					}
					else if((m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1] == V_NOT_EQUAL) && (USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1]  != m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1]))	
					{//user��ǰֵ != R����
						m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
					}
					else if((m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1] == V_NOTONLY_EQUAL) && (USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1] % m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1]) != 0)	
					{//user��ǰֵ != R�����ı���ʱ
						m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
					}	
				}
			}
			result = 1;
			break;
		case K_CYCLEOVER://ѭ������
			if(m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] == 1)
			{
				if(m_WhileSubJudgeType[subProNum][m_WhileSubNC[subProNum] - 1] == V_R_METHOD)
				{//R�ж�
					m_WhileSubCycCounter[subProNum][m_WhileSubNC[subProNum] - 1]++;
				}
			}
			result = 1;
			break;
		case K_IF://IF�ж�����
		case K_ELSE://else�ж�����������ߴ���ʽ��ͬ
			m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;																//Ĭ���ж�����ʧ��
			if((V_USER1<=m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1])&&(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] <= V_USER8))
			{//�û������ж�
				if((USER_Parameter.CURR_Num[m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1]-V_USER1] != 0) && (m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] == V_EQUAL)\
						&& (USER_Parameter.CURR_Num[m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1]-V_USER1] % m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]) == 0)	
				{//user��ǰֵ = R�����ı���
					m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
				}
				else if((m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] == V_ONLY_EQUAL) && (USER_Parameter.CURR_Num[m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1]-V_USER1] == m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]))	
				{//user��ǰֵ = R����
					m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
				}
				else if((m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] == V_NOT_EQUAL) && (USER_Parameter.CURR_Num[m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1]-V_USER1] != m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]))	
				{//user��ǰֵ != R����
					m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
				}
				else if((m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] == V_NOTONLY_EQUAL) && (USER_Parameter.CURR_Num[m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1]-V_USER1] % m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]) != 0)	
				{//user��ǰֵ != R�����ı���ʱ
					m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
				}
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_I_METHOD)
			{//I��ת
				if(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] < INPUT_NUM)
				{
					if(m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] == V_LOW_LEVEL && \
							ReadInput(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1]) == 0)
					{//�����Ч��ƽ
						m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
					else if(m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] == V_HIGH_LEVEL && \
										ReadInput(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1]) == 1)
					{//�����Ч��ƽ
						m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
				}
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_O_METHOD)
			{//O�ж�
				if(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] < OUTPUT_NUM)
				{
					ionum = m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1];
					index = ionum / 8;
					offset = ionum % 8;
					if(m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] == V_RESET && ((Output_Status[index] >> offset) & 0x01) == 0x01)
					{//��⸴λ
						m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
					else if(m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] == V_SET && ((Output_Status[index] >> offset) & 0x01) == 0)
					{//�����λ
						m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
				}
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_P_METHOD)
			{//P�ж�
				if(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] >= V_XAXISGREATER && \
					m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] <= V_OAXISGREATER)
				{//���ڵ���
					axisNum = m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] - V_XAXISGREATER;		
					pointTemp = m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] * Step_Coefficient[axisNum] / 100 + MINROBOTPOSITION;
					if(m_PulseTotalCounter[axisNum] + JDZ_AllowError >= pointTemp)
					{//�ж���������
						m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
				}
				else if(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] >= V_XAXISEQUAL && \
					m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] <= V_OAXISEQUAL)
				{//����
					axisNum = m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] - V_XAXISEQUAL;
					pointTemp = m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] * Step_Coefficient[axisNum] / 100 + MINROBOTPOSITION;
					if(m_PulseTotalCounter[axisNum] + JDZ_AllowError >= pointTemp && m_PulseTotalCounter[axisNum] <= pointTemp + JDZ_AllowError)
					{//�ж���������
						m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
				}
				else if(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] >= V_XAXISLESS && \
					m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] <= V_OAXISLESS)
				{//С�ڵ���
					axisNum = m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] - V_XAXISLESS;
					pointTemp = m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] * Step_Coefficient[axisNum] / 100 + MINROBOTPOSITION;
					if(m_PulseTotalCounter[axisNum] <= pointTemp + JDZ_AllowError)
					{//�ж���������
						m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
				}
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_LAYER_NUM)
			{//�����ж�
				m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = MD_LayerNumJudge(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1], m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]);
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_LAYER_FULL)
			{//�����ж�
				m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = MD_LayerFullJudge(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1], m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]);
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_STACK_FULL)
			{//�����ж�
				m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = MD_StackFullJudge(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1], m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]);
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_MDGOOD)
			{//��Ʒ�ж�
				m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = MD_GoodNumJudge(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1], m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]);
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_RI_NULL)
			{//NULL��ת
				m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
			}
			result = 1;
			break;
		case K_OVER:
			result = 1;
			break;
		case K_SPECIAL://����ָ��								
			if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_SUSPEND)
			{
				result = 1;
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_STOP)
			{
				result = 1;
			}
			break;	
		case K_PULSE_OUTPUT:		       //�������
			result = 1;	
			break;
		case K_USER:		              //�û�����
//			if(USER_Parameter.ELEC_RESET[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1] == 0 \
//				&& USER_Parameter.START_RESET[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1] == 0)
//			{
//				Temp_data[0] = USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1];
//				Temp_data[1] = USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1]>>8;
//				Temp_data[2] = USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1]>>16;
//				Temp_data[3] = USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1]>>24;
//				W25QXX_Write(Temp_data,P_USER_ADDRESS + 17 + 23*(SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1),4);
//			}
			result = 1;	
			break;
		case K_OUTDETECT://������
			if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 < OUTPUT_NUM)
			{
				ionum = SubProgram_Operate[subProNum].Program[ActionLine].Value1;
//				if(ionum>=24)ionum += 4;//RY0-RY1
				index = ionum / 8;
				offset = ionum % 8;
				if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_RESET && ((Output_Status[index] >> offset) & 0x01) == 0x01)
				{//��λ
					result = 1;
				}
				else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_SET && ((Output_Status[index] >> offset) & 0x01) == 0)
				{//��λ
					result = 1;
				}
			}
		case K_MDPOSITION://ȷ����������Ƿ�λ
			if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 >= V_MD_AXSIS \
					&& SubProgram_Operate[subProNum].Program[ActionLine].Value1 < V_MD_AXSIS + Axis_Num)
			{//���λ��
				axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1 - V_MD_AXSIS;
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 >= V_MD_AXSIS + Axis_Num \
					&& SubProgram_Operate[subProNum].Program[ActionLine].Value1 < V_MD_AXSIS + Axis_Num * 2)
			{//�ȴ���λ��
				axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1 - V_MD_AXSIS - Axis_Num;
			}
			
			if(axsisNum == O_Axsis && sMD_Parameter.revolveMode == 1)
			{//O��Ϊ����
				result = 1;
			}
			else
			{
				if(Servo_MoveFinishSta(axsisNum, Axsis_MoveTarPos[axsisNum]))
				{
					result = Judge_JDZ_Error(axsisNum);
				}
			}
			break;
		case K_MDPOCOUNT://������
			result = 1;	
			break;
		case K_XAXIS://���Ƿ��ƶ���Ŀ��λ��
		case K_ZAXIS:
		case K_LAXIS:
		case K_OAXIS:
			if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_XAXIS) 
			{//X��
				axsisNum = X_Axsis;
				pointTemp = Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Point_X + MINROBOTPOSITION;
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_ZAXIS) 
			{//Z��
				axsisNum = Z_Axsis;
				pointTemp = Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Point_Z + MINROBOTPOSITION;
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_LAXIS)
			{//Y��
				axsisNum = L_Axsis;
				pointTemp = Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Point_L + MINROBOTPOSITION;
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_OAXIS)
			{//O��
				axsisNum = O_Axsis;
				pointTemp = Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Point_O + MINROBOTPOSITION;
			}
			
			if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == 0 || SubProgram_Operate[subProNum].Program[ActionLine].Value1 > 75)
			{
				result = 1;
			}
			else if(Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Flag != 1)
			{
				result = 1;
			}
			else
			{
				if(Servo_MoveFinishSta(axsisNum, pointTemp))
				{
					result = Judge_JDZ_Error(axsisNum);
				}
			}
			break;
		case K_KEEP_MOVE://��������
		case K_NEGTIVE_SEARCH://��������
			keepMoveIoSta_Low[X_Axsis] = X_AXIS_MOVE_SIGN_LOW_LEVEL;
			keepMoveIoSta_Low[L_Axsis] = Y_AXIS_MOVE_SIGN_LOW_LEVEL;
			keepMoveIoSta_Low[Z_Axsis] = Z_AXIS_MOVE_SIGN_LOW_LEVEL;
			keepMoveIoSta_Low[O_Axsis] = O_AXIS_MOVE_SIGN_LOW_LEVEL;
			keepMoveIoSta_High[X_Axsis] = X_AXIS_MOVE_SIGN_HIGH_LEVEL;
			keepMoveIoSta_High[L_Axsis] = Y_AXIS_MOVE_SIGN_HIGH_LEVEL;
			keepMoveIoSta_High[Z_Axsis] = Z_AXIS_MOVE_SIGN_HIGH_LEVEL;
			keepMoveIoSta_High[O_Axsis] = O_AXIS_MOVE_SIGN_HIGH_LEVEL;

			axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1 - V_KEEP_MOVE_X;
			if(axsisNum < Axis_Num && Flag_Keep_Move[axsisNum] == 1)
			{
				if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_LOW_LEVEL)
				{//�͵�ƽ
					if(keepMoveIoSta_Low[axsisNum])
					{//�յ��͵�ƽ�ź�
						Servo_Stop(axsisNum);
						Flag_Keep_Move[axsisNum] = 2;
					}
				}
				else if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_HIGH_LEVEL)
				{//�ߵ�ƽ
					if(keepMoveIoSta_High[axsisNum])	
					{//�յ��ߵ�ƽ�ź�
						Servo_Stop(axsisNum);
						Flag_Keep_Move[axsisNum] = 2;
					}
				}
				else if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_FALLING_EDGE)
				{//�½��أ�����
					if(Flag_Falling_Edge_Sub == 1)
					{//һ��ʼ��⵽�͵�ƽ
						if(keepMoveIoSta_High[axsisNum])
						{//��⵽�ߵ�ƽ
							Flag_Falling_Edge_Sub = 3;
						}
					}
					if(Flag_Falling_Edge_Sub == 2 || Flag_Falling_Edge_Sub == 3)
					{//һ��ʼ��⵽�ߵ�ƽ,�����ٴμ�⵽��
						if(keepMoveIoSta_Low[axsisNum])
						{//��⵽�͵�ƽ��ֹͣ
							Servo_Stop(axsisNum);
							Flag_Keep_Move[axsisNum] = 2;
						}
					}
				}
				else if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_RISING_EDGE)
				{//�����أ�����
					if(Flag_Rising_Edge_Sub == 1)
					{//һ��ʼ��⵽�ߵ�ƽ
						if(keepMoveIoSta_Low[axsisNum])
						{//��⵽�͵�ƽ
							Flag_Rising_Edge_Sub = 3;
						}
					}
					if(Flag_Rising_Edge_Sub == 2 || Flag_Rising_Edge_Sub == 3)
					{//һ��ʼ��⵽�͵�ƽ,�����ٴμ�⵽��
						if(keepMoveIoSta_High[axsisNum])
						{//��⵽�ߵ�ƽ��ֹͣ
							Servo_Stop(axsisNum);
							Flag_Keep_Move[axsisNum] = 2;
						}
					}
				}
			}
			
			if(Flag_Keep_Move[axsisNum] == 2)
			{
				result = 1;
				Flag_Keep_Move[axsisNum] = 0;
			}
			break;
		case K_INCREMENT_RUNNING://�Ƿ��ƶ���Ŀ��λ��
			axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1;
			if(V_KEEP_MOVE_X <= axsisNum && axsisNum <= V_KEEP_MOVE_O)
			{
				axsisNum = axsisNum - V_KEEP_MOVE_X;
			}
			else if(V_KEEP_MOVE_U <= axsisNum && axsisNum <= V_KEEP_MOVE_V)
			{
				axsisNum = axsisNum - V_KEEP_MOVE_U + Axis_Num;
			}
			
			if(axsisNum < Axis_Num + Ext_Axis_Num)
			{
				if(Servo_MoveFinishSta(axsisNum, Increment_Target[axsisNum]))
				{
					result = Judge_JDZ_Error(axsisNum);
					if(result ==1)
					{
						Increment_Finished[axsisNum] = FALSE;
					}
				}
			}
			break;
		case K_MACHINE_ORIGIN:
			result = 1;
			break;
		case K_POSSET:
			axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1;
			if(V_KEEP_MOVE_X <= axsisNum && axsisNum <= V_KEEP_MOVE_O)
			{
				axsisNum = axsisNum - V_KEEP_MOVE_X;
			}
			else if(V_KEEP_MOVE_U <= axsisNum && axsisNum <= V_KEEP_MOVE_V)
			{
				axsisNum = axsisNum - V_KEEP_MOVE_U + Axis_Num;
			}
			pointTemp = SubProgram_Operate[subProNum].Program[ActionLine].Value2 * Step_Coefficient[axsisNum] / 100 + MINROBOTPOSITION;
			result = Position_Reset(axsisNum, pointTemp);
			break;
		case K_SLOWPOINT:
			result = 1;
			break;
		case K_INTER_START:
			result = 1;	
			break;
		case K_INTER_OVER:
			result = 1;	
			break;
		case K_ADVENCE:
			result = 1;	
			break;
		case K_INTER_LINE:
			result = 1;	
			break;
		case K_AXISMOVE:
			axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1;
			if(V_KEEP_MOVE_X <= axsisNum && axsisNum <= V_KEEP_MOVE_O)
			{
				axsisNum = axsisNum - V_KEEP_MOVE_X;
			}
			else if(V_KEEP_MOVE_U <= axsisNum && axsisNum <= V_KEEP_MOVE_V)
			{
				axsisNum = axsisNum - V_KEEP_MOVE_U + Axis_Num;
			}
			pointTemp = (SubProgram_Operate[subProNum].Program[ActionLine].Value3 & 0x0fffffff) * Step_Coefficient[axsisNum] / 100 + MINROBOTPOSITION;
			if(Servo_MoveFinishSta(axsisNum, pointTemp))
			{
				result = Judge_JDZ_Error(axsisNum);
			}
			break;
		case K_ANGLE_ARC:
			result = 1;	
			break;
		
		//����ӿ�-�޼��
		case K_IOINSTRUCT_OUTPUT1:			//Y0-1
		case K_IOINSTRUCT_OUTPUT2:			//Y0-0
 		case K_IOINSTRUCT_OUTPUT3:			//Y1-1
		case K_IOINSTRUCT_OUTPUT4:			//Y1-0
		case K_IOINSTRUCT_OUTPUT5:			//Y2-1
		case K_IOINSTRUCT_OUTPUT6:		 	//Y2-0
		case K_IOINSTRUCT_OUTPUT7: 			//Y3-1
		case K_IOINSTRUCT_OUTPUT8:		 	//Y3-0
		case K_IOINSTRUCT_OUTPUT9:		 	//Y4-1
		case K_IOINSTRUCT_OUTPUT10:			//Y4-0
		case K_IOINSTRUCT_OUTPUT11:			//Y5-1
		case K_IOINSTRUCT_OUTPUT12:			//Y5-0
		case K_IOINSTRUCT_OUTPUT13:			//Y6-1
		case K_IOINSTRUCT_OUTPUT14:			//Y6-0
		case K_IOINSTRUCT_OUTPUT15:			//Y7-1
		case K_IOINSTRUCT_OUTPUT16:			//Y7-0
		case K_IOINSTRUCT_OUTPUT17:			//Y8-1
		case K_IOINSTRUCT_OUTPUT18:			//Y8-0
		case K_IOINSTRUCT_OUTPUT19:			//Y9-1
		case K_IOINSTRUCT_OUTPUT20:			//Y9-0
		case K_IOINSTRUCT_OUTPUT21:		 //Y10-1
		case K_IOINSTRUCT_OUTPUT22:		 //Y10-0
		case K_IOINSTRUCT_OUTPUT23:		 //Y11-1
		case K_IOINSTRUCT_OUTPUT24:		 //Y11-0
		case K_IOINSTRUCT_OUTPUT25:		 //Y12-1
		case K_IOINSTRUCT_OUTPUT26:		 //Y12-0
		case K_IOINSTRUCT_OUTPUT27:		 //Y13-1
		case K_IOINSTRUCT_OUTPUT28:		 //Y13-0
		case K_IOINSTRUCT_OUTPUT29:		 //Y14-1
		case K_IOINSTRUCT_OUTPUT30:		 //Y14-0
		case K_IOINSTRUCT_OUTPUT31:		 //Y15-1
		case K_IOINSTRUCT_OUTPUT32:		 //Y15-0
		case K_IOINSTRUCT_OUTPUT33:		 //Y16-1
		case K_IOINSTRUCT_OUTPUT34:		 //Y16-0
		case K_IOINSTRUCT_OUTPUT35:		 //Y17-1
		case K_IOINSTRUCT_OUTPUT36:		 //Y17-0
		case K_IOINSTRUCT_OUTPUT37:		 //Y18-1
		case K_IOINSTRUCT_OUTPUT38:		 //Y18-0
		case K_IOINSTRUCT_OUTPUT39:		 //Y19-1
		case K_IOINSTRUCT_OUTPUT40:		 //Y19-0
		case K_IOINSTRUCT_OUTPUT41:		 //Y20-1
		case K_IOINSTRUCT_OUTPUT42:		 //Y20-0
		case K_IOINSTRUCT_OUTPUT43:		 //Y21-1
		case K_IOINSTRUCT_OUTPUT44:		 //Y21-0
		case K_IOINSTRUCT_OUTPUT45:		 //Y22-1
		case K_IOINSTRUCT_OUTPUT46:		 //Y22-0
		case K_IOINSTRUCT_OUTPUT47:		 //Y23-1
		case K_IOINSTRUCT_OUTPUT48:		 //Y23-0
		case K_IOINSTRUCT_OUTPUT49:		 //RY0-1
		case K_IOINSTRUCT_OUTPUT50:		 //RY0-0
		case K_IOINSTRUCT_OUTPUT51:		 //RY1-1
		case K_IOINSTRUCT_OUTPUT52:		 //RY1-0
      result = 1;	
			break;

		//����ӿڼ��
		case K_IOINSTRUCT_INPUT1://X0
		case K_IOINSTRUCT_INPUT2://X1		
		case K_IOINSTRUCT_INPUT3://X2		
		case K_IOINSTRUCT_INPUT4://X3
		case K_IOINSTRUCT_INPUT5://X4	
		case K_IOINSTRUCT_INPUT6://X5
		case K_IOINSTRUCT_INPUT7://X6
		case K_IOINSTRUCT_INPUT8://X7
		case K_IOINSTRUCT_INPUT9://X8
		case K_IOINSTRUCT_INPUT10://X9
		case K_IOINSTRUCT_INPUT11://X10
		case K_IOINSTRUCT_INPUT12://X11
		case K_IOINSTRUCT_INPUT13://X12
		case K_IOINSTRUCT_INPUT14://X13
		case K_IOINSTRUCT_INPUT15://X14
		case K_IOINSTRUCT_INPUT16://X15
		case K_IOINSTRUCT_INPUT17://X16
		case K_IOINSTRUCT_INPUT18://X17
		case K_IOINSTRUCT_INPUT19://X18
		case K_IOINSTRUCT_INPUT20://X19
		case K_IOINSTRUCT_INPUT21://X20
		case K_IOINSTRUCT_INPUT22://X21
		case K_IOINSTRUCT_INPUT23://X22
		case K_IOINSTRUCT_INPUT24://X23
		case K_IOINSTRUCT_INPUT25://X24
		case K_IOINSTRUCT_INPUT26://X25
		case K_IOINSTRUCT_INPUT27://X26
		case K_IOINSTRUCT_INPUT28://X27
		case K_IOINSTRUCT_INPUT29://X28
		case K_IOINSTRUCT_INPUT30://X29	
			ionum = SubProgram_Operate[subProNum].Program[ActionLine].Key-K_IOINSTRUCT_INPUT1;
			
			g_Auto_SubActionNcWait_Flag[subProNum] = ionum + 1;	
			if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_USEFUL)	
			{
				signValue = GetIO_Value(ionum, 1, subProNum + 1);	 
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_USELESS)	
			{
				signValue = GetIO_Value(ionum, 0, subProNum + 1);	 
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_FALLING_EDGE)
			{
				signValue = GetIO_Value(ionum, 2, subProNum + 1);	 
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_RISING_EDGE)	
			{
				signValue = GetIO_Value(ionum, 3, subProNum + 1);	 
			}
			
			if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_FALLING_EDGE)
			{//����½��� �뱣��ʱ���޹�
				if(signValue == 1)
				{//������Ч�ź�
					result = 1;
					Detect_Falling_Edge_Sub[subProNum] = 0;
				}
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_RISING_EDGE)
			{//��������� �뱣��ʱ���޹�
				if(signValue == 1)
				{//������Ч�ź�
					result = 1;
					Detect_Rising_Edge_Sub[subProNum] = 0;
				}
			}
			else
			{
				if(signValue == 1)
				{//������Ч�ź�
					if(IO_Input_keepMin[ionum] >= IO_Input_keepMax[ionum])
					{//��Чʱ�����ʼʱ�������ֹʱ�䣬��Ϊ��Чʱ��Ϊ0��һ��������Ч�ź�����ȷ�ϳɹ�
						result = 1;		
					}
					
					if(g_SubAuto_Valid_Timer[subProNum] > IO_Input_keepMax[ionum])
					{//��Ч�ź�ʱ�䳬����󱣳�ʱ��,�رռ�ʱ�������Ѽ�ʱ�����㣬��Ϊ�´γ�����Ч�ź�ʱ����0��ʼ��ʱ
						g_SubAuto_Valid_Timer[subProNum] = 0;
						g_SubAuto_Valid_Flag[subProNum] = FALSE;
					}
					
					if(g_SubAuto_Valid_Flag[subProNum] == FALSE)
					{//��ʼ����
						g_SubAuto_Valid_Timer[subProNum] = 0;
						g_SubAuto_Valid_Flag[subProNum] = TRUE;
					}
				}
				else
				{//����λ����������0��ʼ
					if(g_SubAuto_Valid_Timer[subProNum] > IO_Input_keepMin[ionum] && g_SubAuto_Valid_Timer[subProNum] < IO_Input_keepMax[ionum])
					{//��Чʱ�����������Сʱ�䣬С�����ʱ��
						result = 1;
					}
					g_SubAuto_Valid_Timer[subProNum] = 0;
					g_SubAuto_Valid_Flag[subProNum] = FALSE;
				}
			}
			break;
		default://ָ�������쳣
			 result = 14;	
			 break;
	}
	return result;		
}
/**************************************************************************************************
**  ��������  AutoActionOutDelay()
**	�����������Ҫ���е��к�
**	�����������ʱ�Ƿ����
**	�������ܣ��������Զ����ж��������ʱ����
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u8 AutoActionOutDelay(SaveProgram *Program_Operate, u8 ActionLine)
{
	u8 result = FALSE;
	
	if(OR_IOORDER != Program_Operate->Program[ActionLine].Order)
	{//ֻ��IO�������Ҫ�������ʱ����
		result = TRUE;
	}
	else if(Program_Operate->Program[ActionLine].Key >= K_IOINSTRUCT_INPUT1 && Program_Operate->Program[ActionLine].Key <= K_IOINSTRUCT_INPUT30)
	{
		result = TRUE;
	}
	else if(Program_Operate->Program[ActionLine].Value3 == 0)
	{
		result = TRUE;
	}
	else
	{
		switch(g_ActionDelay_Step)
		{
			case 0:
				g_ActionDelay_Timer = 0;
				g_ActionDelay_Step = 1;
				break;
			case 1:
				if(g_ActionDelay_Timer >= Program_Operate->Program[ActionLine].Value3)
				{//��ʱʱ�䵽��
					g_ActionDelay_Step = 0;
					g_ActionDelay_Timer = 0;
					result = TRUE;
				}
				break;
			default:
				g_ActionDelay_Step = 0;
				break;
		}
	}
	return result;		
}

/**************************************************************************************************
**  ��������  AutoActionOutDelay()
**	�����������Ҫ���е��к�
**	�����������ʱ�Ƿ����
**	�������ܣ��ӳ����Զ����ж��������ʱ����
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u8 SubProgramActionOutDelay(u8 subProNum, u8 ActionLine)
{
	u8 result = FALSE;
	
	if(OR_IOORDER != SubProgram_Operate[subProNum].Program[ActionLine].Order)
	{//ֻ��IO�������Ҫ�������ʱ����
		result = TRUE;
	}
	else if(SubProgram_Operate[subProNum].Program[ActionLine].Key >= K_IOINSTRUCT_INPUT1 && SubProgram_Operate[subProNum].Program[ActionLine].Key <= K_IOINSTRUCT_INPUT30)
	{
		result = TRUE;
	}
	else if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == 0)
	{
		result = TRUE;
	}
	else
	{
		switch(g_SubProgramDelay_Step[subProNum])
		{
			case 0:
				g_SubProgramDelay_Timer[subProNum] = 0;
				g_SubProgramDelay_Step[subProNum] = 1;
				break;
			case 1:
				if(g_SubProgramDelay_Timer[subProNum] >= SubProgram_Operate[subProNum].Program[ActionLine].Value3)
				{//��ʱʱ�䵽
					g_SubProgramDelay_Step[subProNum] = 0;
					g_SubProgramDelay_Timer[subProNum] = 0;
					result = TRUE;
				}
				break;
			default:
				g_SubProgramDelay_Step[subProNum] = 0;
				break;
		}
	}
	return result;		
}

/**************************************************************************************************
**  ��������  ActionAllowJudge()
**	�����������
**	�����������
**	�������ܣ�����ִ��ǰ������ĺϷ����ж�
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
********************************** *****************************************************************/
u8 ActionAllowJudge(SaveProgram *Program_Operate, u8 ActionLine)
{
	u8 result = TRUE;
	
	switch(Program_Operate->Program[ActionLine].Order)
	{
		case OR_BASICORDER:
			if((K_PROGRAMSTART <= Program_Operate->Program[ActionLine].Key && Program_Operate->Program[ActionLine].Key <= K_OUTDETECT)\
				|| (K_PULSE_OUTPUT <= Program_Operate->Program[ActionLine].Key && Program_Operate->Program[ActionLine].Key <= K_USER))
			{
				result = FALSE;
			}
			else
			{//����ָ�������쳣
				result = 12;
			}
			break;
		case OR_AXISORDER:
			if((K_KEEP_MOVE <= Program_Operate->Program[ActionLine].Key && Program_Operate->Program[ActionLine].Key <= K_OAXIS)\
				|| Program_Operate->Program[ActionLine].Key == K_INCREMENT_RUNNING \
			  || (K_NEGTIVE_SEARCH <= Program_Operate->Program[ActionLine].Key && Program_Operate->Program[ActionLine].Key <= K_ANGLE_ARC))
			{
				result = FALSE;
			}
			else
			{//���ָ�������쳣
				result = 13;
			}
			break;
		case OR_IOORDER:
			if((K_IOINSTRUCT_OUTPUT1 <= Program_Operate->Program[ActionLine].Key) && (Program_Operate->Program[ActionLine].Key <= K_IOINSTRUCT_INPUT30))
			{
				result = FALSE;
			}
			else
			{//IO����ָ�������쳣
				result = 14;
			}
			break;
		default://��Ҫָ�������쳣
			result = 11;
			break;
	}
	return result;
}

/**************************************************************************************************
**  ��������  SubActionAllowJudge()
**	�����������
**	�����������
**	�������ܣ�����ִ��ǰ������ĺϷ����ж�
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
********************************** *****************************************************************/
u8 SubActionAllowJudge(u8 subProNum, u8 ActionLine)
{
	u8 result = TRUE;
	
	switch(SubProgram_Operate[subProNum].Program[ActionLine].Order)
	{
		case OR_BASICORDER:
			if((K_PROGRAMSTART <= SubProgram_Operate[subProNum].Program[ActionLine].Key && SubProgram_Operate[subProNum].Program[ActionLine].Key <= K_OUTDETECT)\
				|| (K_PULSE_OUTPUT <= SubProgram_Operate[subProNum].Program[ActionLine].Key && SubProgram_Operate[subProNum].Program[ActionLine].Key <= K_USER))
			{
				result = FALSE;
			}
			else
			{//����ָ�������쳣
				result = 12;
			}
			break;
		case OR_AXISORDER:
			if((K_KEEP_MOVE <= SubProgram_Operate[subProNum].Program[ActionLine].Key && SubProgram_Operate[subProNum].Program[ActionLine].Key <= K_OAXIS) \
				|| SubProgram_Operate[subProNum].Program[ActionLine].Key == K_INCREMENT_RUNNING \
			  || (K_NEGTIVE_SEARCH <= SubProgram_Operate[subProNum].Program[ActionLine].Key && SubProgram_Operate[subProNum].Program[ActionLine].Key <= K_ANGLE_ARC))
			{
				result = FALSE;
			}
			else
			{//���ָ�������쳣
				result = 13;
			}
			break;
		case OR_IOORDER: 
			if((K_IOINSTRUCT_OUTPUT1 <= SubProgram_Operate[subProNum].Program[ActionLine].Key) && (SubProgram_Operate[subProNum].Program[ActionLine].Key <= K_IOINSTRUCT_INPUT30))
			{
				result = FALSE;
			}
			else
			{//IO����ָ�������쳣
				result = 14;
			}
			break;
		default://��Ҫָ�������쳣
			result = 11;
			break;
	}
	return result;
}

/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/





