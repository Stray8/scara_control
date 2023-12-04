/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __speedcontrol_h_
#define __speedcontrol_h_

#include "StatusControl.h"

#define MAX_SPEED	       	    820000 																//����ٶ�820k HZ
#define MAX_SPEED_CHANGE	    (MAX_SPEED/100/100) 									//����ٶ�ת��ϵ�� MAX_SPEED/100/100

extern u8  AxisMoveFlag[Axis_Num + Ext_Axis_Num];
extern s32 m_PulseTotalCounter[Axis_Num + Ext_Axis_Num];

extern u8  Pulse_StopRequest[Axis_Num + Ext_Axis_Num];	    				//ֹͣ������Ҫ������ͣ��ֹͣ������
extern u8  m_InterpAxisMoveFlag[Axis_Num + Ext_Axis_Num];						//���ڱ�ʶ����Ĳ岹�˶�
extern u32 Pulse_MaxSpeed[Axis_Num + Ext_Axis_Num];									//����ٶ�Ƶ��
extern u8  m_InterpAxisFlag[Axis_Num + Ext_Axis_Num];								//��Ҫ�岹�����־
extern u8  m_InterpLenAxis;																					//�岹������

#define INTER_CURVE_NO												0          						//�岹�˶��Ĳ���
#define INTER_CURVE_ONE												1          						//
#define INTER_CURVE_TWO												2          						//

extern u8  m_InterpCurveFlag;																				//�����岹�˶����ȱ�־

extern void AxisMoveAccCal(u8 Axis);
extern u32 MoveMaxSpeed(u8 Axis, s32 axsiPosition, u32 maxCycleTime);
extern u8 SendPulse(u8 Axis, s32 axsiPosition, u32 maxSpeed);
extern void OneAxisSpeedInterpControl(void);
extern void KeepSendPulse(u8 Axis, u32 maxSpeed);
extern void SpeedInterpControl(void);
extern u8 InterpSendPulse(void);
extern void g_AxisActionNextPosRead(void);
extern void g_AxisActionNextPosRead(void);

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

