/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : main.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f4xx.h"
#include "SpeedControl.h" 
#include "StatusControl.h"
#include "Auto.h"
#include "Manual.h"
#include "Parameter.h"
#include "Error.h"
#include "Auto_2.h"
#include "out.h"
#include "in.h"
#include "ActionOperate.h"
#include "CANopen.h"
#include <stdio.h>
#include <stdlib.h>
#include "JDZ.h" 

s32 m_PulseTotalCounter[Axis_Num + Ext_Axis_Num] = {0};									//������
u8  AxisMoveFlag[Axis_Num + Ext_Axis_Num] = {0};
u32 Servo_Pulse_Count[Axis_Num + Ext_Axis_Num] = {0, 0, 0, 0, 0, 0};		//��Ҫ���͵��������

/*�Ӽ�����غ꼰����*/
#define SPEED_TIMER_BASE									1000000          							//�������ʱ��
#define SPEED_CONTROL_PER									2000          								//�岹���ڣ���λus
#define START_SPEED_FRE										1000         									//��ʼ�ٶ�

/*����������*/
#define BUF_INTERP_LINE_END								(0x7FFFFFC0)          				//�岹�߶ν���
#define BUF_INTERP_END										(0x7FFFFFC1)          				//�岹����
#define BUF_INTERP_BUF_NULL								(0x7FFFFFC2)          				//�岹������Ϊ��
/*�岹�˶������*/
#define LINE_RUN_FINISH       						(0x7FFFFFE0) 									//�߶��������
#define INTERP_RUN_FINISH       					(0x7FFFFFE1) 									//����˶����

u32 Pulse_MaxSpeed[Axis_Num + Ext_Axis_Num] = {0};																		//����ٶ�Ƶ��
u32 Pulse_MaxAcc[Axis_Num + Ext_Axis_Num] = {1000, 1000, 1000, 1000, 1000, 1000};  	  //�����ٶ�
u16 Pulse_AccDecMaxSpeedStep[Axis_Num + Ext_Axis_Num] = {0};	    										//�Ӽ�������ٶ�����λ��
u16 Pulse_PreSpeedStep[Axis_Num + Ext_Axis_Num] = {0};	    													//��ǰ�ٶ�����λ�ü���
u32 m_InterpAcc = 0;																																	//�����岹ʱ��ʵ�ʼ��ٶ�ֵ
u8  m_InterpAccFlag = 0;																															//�����岹ʱ�ļ��ٶȱ仯��־

/*���弰λ�û�������ز���*/
#define INTERP_AXIS_BUFFER_MAX 10																											//���������建������С
#define INTERP_AXIS_BUFFER_MIN 4																											//���������建������д��Сʣ��ռ䣬һ��Ҫ����5����ֹд����д����ȥ
s32 m_InterpAxisPulseBuf[Axis_Num + Ext_Axis_Num][INTERP_AXIS_BUFFER_MAX] = {{0}};		//�����建����
u16 m_InterpAxisPulseBufWrite[Axis_Num + Ext_Axis_Num] = {0};													//������дָ��
u16 m_InterpAxisPulseBufRead[Axis_Num + Ext_Axis_Num] = {0};													//��������ָ��
s32 m_InterAxisMoveLen[Axis_Num + Ext_Axis_Num] = {0};																//�Ѿ��˶��ĳ���
s32 m_StartPulseCounter[Axis_Num + Ext_Axis_Num] = {0};																//�������˶�����ʼλ��
s32 m_InterAxisMoveIncre[Axis_Num + Ext_Axis_Num] = {0};															//����ƶ�����
u8  m_InterpAxisFlag[Axis_Num + Ext_Axis_Num] = {0};																	//��Ҫ�岹�����־
u8  m_InterpAxisMoveFlag[Axis_Num + Ext_Axis_Num] = {0};															//���ڱ�ʶ����Ĳ岹�˶�
u8  Pulse_StopRequest[Axis_Num + Ext_Axis_Num] = {0};	    														//ֹͣ���� 1������ͣ 0��Ч
u8  m_InterpLenAxis = 0xff;																														//�岹������
u8  m_InterpCurveFlag = INTER_CURVE_NO;																								//���߲岹�˶����ȱ�־

/**************************************************************************************************
**  ��������  InterpAxisPreAcc()
**	�����������
**	�����������
**	�������ܣ�����ÿһ���ٶȽ׶ζ�Ӧ�ļ��ٶ�
**	��ע��	  			  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u32 InterpAxisPreAcc(u8 Axis, u16 speedStep)
{
	u32 preAcc = 0;
	
	preAcc = Pulse_MaxAcc[Axis];
	
	return preAcc;
}

/**************************************************************************************************
**  ��������  InterpAxisPulseNum()
**	�����������
**	�����������
**	�������ܣ�����ÿ��һ���ٶȽ׶ζ�Ӧ�ļ��پ������پ���
**	��ע��	  			  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u32 InterpAxisPulseNum(u8 Axis, u16 speedStep)
{
	u32 pulseNum = 0;
	
	pulseNum = Pulse_MaxAcc[Axis] * speedStep * (speedStep + 1) / 2 /500;							//SPEED_CONTROL_PER / SPEED_TIMER_BASE = 1/500
	if(pulseNum < speedStep)
	{
		pulseNum = 1 + speedStep;
	}
	else
	{
		pulseNum = 1 + pulseNum;
	}
	return pulseNum;
}

/**************************************************************************************************
**  ��������  SpeedPlanningReset()
**	�����������
**	�����������
**	�������ܣ���λ�˶���ر�������־λ
**	��ע��	  			  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void SpeedPlanningReset(u8 Axis)
{
	Pulse_StopRequest[Axis] = 0;
}

/**************************************************************************************************
**  ��������  ClosePulseReset()
**	�����������
**	�����������
**	�������ܣ����巢�ͽ�����λ��־λ
**	��ע��	  			  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void ClosePulseReset(u8 Axis)
{
//	Axsis_Move_Direction[Axis] = NONE;
//	Pulse_Counter[Axis] = 0;
}

/*
**���ܣ���ȡ�����建����ʣ�����ݸ���
**������l_AxisNum ����,0 ~ INTERP_AXIS_MAX_NUM-1
**���أ�ʣ�����ݸ���
*/
u16 g_InterpGetAxisPulseBufNum(u8 l_AxisNum)
{
	u16 size = 0;

	size = (m_InterpAxisPulseBufWrite[l_AxisNum] + INTERP_AXIS_BUFFER_MAX - m_InterpAxisPulseBufRead[l_AxisNum]) % INTERP_AXIS_BUFFER_MAX;
	
	return size;
}

/*
**���ܣ���ȡ�����建����ʣ���С
**������l_AxisNum ����,0 ~ INTERP_AXIS_MAX_NUM-1
**���أ�ʣ���С
*/
u16 g_InterpGetAxisPulseBufSize(u8 l_AxisNum)
{
	u16 size = 0;
	
	size = INTERP_AXIS_BUFFER_MAX - g_InterpGetAxisPulseBufNum(l_AxisNum);
	
	return size;
}

/*
**���ܣ������建����ָ���1
**������l_pBuf ������ָ��
**���أ�OK or ERR
*/
s16 g_InterpAxisPulseBufAdd(u16 *l_pBuf)
{
	*l_pBuf = ((*l_pBuf) + 1) % INTERP_AXIS_BUFFER_MAX;		//ָ��+1
	
	return 0;	//Ok
}

/*
**���ܣ������建����ָ���len
**������l_pBuf ������ָ��
**���أ�OK or ERR
*/
s16 g_InterpAxisPulseBufAddLen(u16 *l_pBuf, u16 l_Len)
{
	*l_pBuf = ((*l_pBuf) + l_Len) % INTERP_AXIS_BUFFER_MAX;		//ָ��+1
	
	return 0;	//Ok
}

/*
**���ܣ�д���建����
**������l_AxisNum �����
**���أ�OK or ERR
*/
s16 g_InterpAxisPulseBufWrite(void)
{	
	u16	i = 0;
	
	/*ֻҪ���Ỻ�������˾ͷ��أ�д���ݱ���ȷ���ռ���ڵ��� INTERP_AXIS_BUFFER_MIN*/
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		if(m_InterpAxisFlag[i] != 0 && g_InterpGetAxisPulseBufSize(i) < 1)//ֻҪ�пռ��������д
		{
			return 0;
		}
	}
	
	/*����д�뻺����*/
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		if(m_InterpAxisFlag[i] != 0)//ֻҪ�пռ��������д
		{
			if(m_InterAxisMoveIncre[i] == INTERP_RUN_FINISH || m_InterAxisMoveIncre[i] == LINE_RUN_FINISH)
			{
				m_InterpAxisPulseBuf[i][m_InterpAxisPulseBufWrite[i]] = m_InterAxisMoveIncre[i];//m_InterAxisMoveIncre[i];	//д������������
			}
			else if(Axsis_Move_Direction[i] == POSITIVE)
			{
				m_InterpAxisPulseBuf[i][m_InterpAxisPulseBufWrite[i]] = m_StartPulseCounter[i] + m_InterAxisMoveLen[i];//m_InterAxisMoveIncre[i];	//д������������
			}
			else
			{
				m_InterpAxisPulseBuf[i][m_InterpAxisPulseBufWrite[i]] = m_StartPulseCounter[i] - m_InterAxisMoveLen[i];//m_InterAxisMoveIncre[i];	//д������������
			}
			g_InterpAxisPulseBufAdd(&m_InterpAxisPulseBufWrite[i]);
		}
	}
	
	return 0;
}

/*
**���ܣ�д���建����
**������l_AxisNum �����
**���أ�OK or ERR
*/
s16 g_InterpOneAxisPulseBufWrite(u8 Axis)
{	
	if(g_InterpGetAxisPulseBufSize(Axis) < 1)//ֻҪ�пռ��������д
	{
		return 0;
	}
	
	/*����д�뻺����*/
	if(m_InterpAxisMoveFlag[Axis] != 0)//ֻҪ�пռ��������д
	{
		if(m_InterAxisMoveIncre[Axis] == INTERP_RUN_FINISH || m_InterAxisMoveIncre[Axis] == LINE_RUN_FINISH)
		{
			m_InterpAxisPulseBuf[Axis][m_InterpAxisPulseBufWrite[Axis]] = m_InterAxisMoveIncre[Axis];//m_InterAxisMoveIncre[i];	//д������������
		}
		else if(Axsis_Move_Direction[Axis] == POSITIVE)
		{
			m_InterpAxisPulseBuf[Axis][m_InterpAxisPulseBufWrite[Axis]] = m_StartPulseCounter[Axis] + m_InterAxisMoveLen[Axis];//m_InterAxisMoveIncre[i];	//д������������
		}
		else
		{
			m_InterpAxisPulseBuf[Axis][m_InterpAxisPulseBufWrite[Axis]] = m_StartPulseCounter[Axis] - m_InterAxisMoveLen[Axis];//m_InterAxisMoveIncre[i];	//д������������
		}
		g_InterpAxisPulseBufAdd(&m_InterpAxisPulseBufWrite[Axis]);
	}
	
	return 0;
}

/*
**���ܣ������建����
**������l_AxisNum ����
**������l_PluseNum ������
**������l_Fre ����Ƶ��
**���أ�
**ע�⣺�����ȡ�ķ���ֵΪ INTERP_RET_LOC_READ_EER ��˵����������ʧ��
*/
s32 g_InterpAxisPulseBufRead(u8 l_AxisNum, s32 *l_PluseNum, u32 *l_Fre)
{	
	if(g_InterpGetAxisPulseBufNum(l_AxisNum) == 0)	//��ѯ�����建����ʣ�����ݸ���
	{
		return 1;
	}
	
	*l_PluseNum = m_InterpAxisPulseBuf[l_AxisNum][m_InterpAxisPulseBufRead[l_AxisNum]];
	
	g_InterpAxisPulseBufAdd(&m_InterpAxisPulseBufRead[l_AxisNum]);	//���建����ָ��+1

	return 0;
}

/**************************************************************************************************
**  ��������  AxisMoveAccCal()
**	���������accTime ����ʱ�䣬��λms
**	�����������
**	�������ܣ����˶�������ٶȣ���ÿ���岹�������ӵ�Ƶ��
**	��ע��		ÿ�μ��ٶȱ仯ʱ����һ�Σ�����ʱ����һ��
**  ���ߣ�     
**  �������ڣ�
***************************************************************************************************/
void AxisMoveAccCal(u8 Axis)
{
	u32 speed = 0;
	u32 accValue = 200;
	
	if(Axis < Axis_Num)
	{
		accValue = JXS_Parameter.Accelerate.Time[Axis];
	}
	else
	{
		accValue = ExtendAix_Parameter[Axis - Axis_Num].E_AccTime;
	}
	
	if(m_InterpAccFlag == 1)
	{
		accValue = m_InterpAcc;
	}
	
	if(accValue < 100)
	{
		accValue = 100;
	}
	Pulse_MaxAcc[Axis] = MAX_SPEED / (accValue * 1000 / SPEED_CONTROL_PER);						//������ٶ�
		
	speed = START_SPEED_FRE;
	Pulse_AccDecMaxSpeedStep[Axis] = 1;
	while(speed + InterpAxisPreAcc(Axis, Pulse_AccDecMaxSpeedStep[Axis]) < Pulse_MaxSpeed[Axis])
	{//�������ٶ����ڲ���
		speed += InterpAxisPreAcc(Axis, Pulse_AccDecMaxSpeedStep[Axis]);
		Pulse_AccDecMaxSpeedStep[Axis]++;
	}
}

/**************************************************************************************************
**  ��������  SlowPointDeal()
**	���������accTime ����ʱ�䣬��λms
**	�����������
**	�������ܣ��޸�����ٶ�
**	��ע��		ÿ�μ��ٶȱ仯ʱ����һ�Σ�����ʱ����һ��
**  ���ߣ�     
**  �������ڣ�
***************************************************************************************************/
u16 SlowPointDeal(u8 Axis)
{
	u32 speed = 0;
	u32 maxSpeedStep = 0;
	u32 maxSpeed = 0;
	u32 speedChangeDis = 0;
	
	if(SlowPointFlag[Axis] == 1)
	{
		maxSpeed = SlowPointSpeed[Axis] * JXS_Parameter.SpeedLevel * MAX_SPEED_CHANGE + 1000;
		if(maxSpeed > MAX_SPEED)
		{
			maxSpeed = MAX_SPEED;
		}
		
		speed = START_SPEED_FRE;
		maxSpeedStep = 1;
		while(speed + InterpAxisPreAcc(Axis, maxSpeedStep) < maxSpeed)
		{//�������ٶ����ڲ���
			speed += InterpAxisPreAcc(Axis, maxSpeedStep);
			maxSpeedStep++;
		}
		
		if(maxSpeedStep > Pulse_AccDecMaxSpeedStep[Axis])
		{
			speedChangeDis = InterpAxisPulseNum(Axis, maxSpeedStep) - InterpAxisPulseNum(Axis, Pulse_PreSpeedStep[Axis]);
		}
		else if(maxSpeedStep < Pulse_AccDecMaxSpeedStep[Axis])
		{
			speedChangeDis = InterpAxisPulseNum(Axis, Pulse_PreSpeedStep[Axis]) - InterpAxisPulseNum(Axis, maxSpeedStep);
		}
		else
		{
			speedChangeDis = 0;
		}
		
		if(m_InterAxisMoveLen[Axis] + SlowPointIncrement[Axis] + speedChangeDis < Servo_Pulse_Count[Axis])
		{//��δ�������λ��
			return 0;
		}
		
		Pulse_AccDecMaxSpeedStep[Axis] = maxSpeedStep;
		SlowPointFlag[Axis] = 0;
	}
	
	return 0;
}

/**************************************************************************************************
**  ��������  SpeedControl()
**	�����������
**	�����������
**	�������ܣ�������ٶȿ���
**	��ע��	  			  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void SpeedControl(u8 Axis)
{
	u32 nextPulse = 0;
	
	while(1)
	{
		if(g_InterpGetAxisPulseBufSize(Axis) < INTERP_AXIS_BUFFER_MIN)
		{
			return;
		}
		
		SlowPointDeal(Axis);
		
		if(Pulse_StopRequest[Axis] == 1)
		{//�������
			if(Pulse_PreSpeedStep[Axis] >= 1)
			{
				m_InterAxisMoveIncre[Axis] = InterpAxisPulseNum(Axis, Pulse_PreSpeedStep[Axis]) - InterpAxisPulseNum(Axis, Pulse_PreSpeedStep[Axis] - 1);
				Pulse_PreSpeedStep[Axis] = Pulse_PreSpeedStep[Axis] - 1;
			}
			else
			{//��ͣ����
				m_InterAxisMoveIncre[Axis] = INTERP_RUN_FINISH;
				g_InterpOneAxisPulseBufWrite(Axis);
				m_InterpAxisMoveFlag[Axis] = 0;
				return;
			}
		}
		else
		{
			if(Pulse_PreSpeedStep[Axis] > 0)
			{
				m_InterAxisMoveIncre[Axis] = InterpAxisPulseNum(Axis, Pulse_PreSpeedStep[Axis]) - InterpAxisPulseNum(Axis, Pulse_PreSpeedStep[Axis] - 1);
			}
			else
			{
				m_InterAxisMoveIncre[Axis] = InterpAxisPulseNum(Axis, 1) - InterpAxisPulseNum(Axis, 0);
			}
			
			if(m_InterAxisMoveLen[Axis] >= Servo_Pulse_Count[Axis])
			{//��ͣ����
				m_InterAxisMoveIncre[Axis] = INTERP_RUN_FINISH;
				g_InterpOneAxisPulseBufWrite(Axis);
				m_InterpAxisMoveFlag[Axis] = 0;
				return;
			}
			else if(m_InterAxisMoveLen[Axis] + InterpAxisPulseNum(Axis, Pulse_PreSpeedStep[Axis]) + m_InterAxisMoveIncre[Axis] < Servo_Pulse_Count[Axis])
			{//ʣ�೤�Ȼ����Լ���
				if(Pulse_PreSpeedStep[Axis] > Pulse_AccDecMaxSpeedStep[Axis])
				{
					Pulse_PreSpeedStep[Axis]--;
				}
				else 
				{
					if(Pulse_PreSpeedStep[Axis] + 1 < Pulse_AccDecMaxSpeedStep[Axis])
					{//�����ǰ�ٶȻ�û��������ٶ�
						Pulse_PreSpeedStep[Axis] = Pulse_PreSpeedStep[Axis] + 1;
					}
					
					if(Pulse_PreSpeedStep[Axis] > 0)
					{
						nextPulse = InterpAxisPulseNum(Axis, Pulse_PreSpeedStep[Axis]) - InterpAxisPulseNum(Axis, Pulse_PreSpeedStep[Axis] - 1);
					}
					else
					{
						nextPulse = InterpAxisPulseNum(Axis, 1) - InterpAxisPulseNum(Axis, 0);
					}
					
					if(Pulse_PreSpeedStep[Axis] > 0 && m_InterAxisMoveLen[Axis] + InterpAxisPulseNum(Axis, Pulse_PreSpeedStep[Axis]) + m_InterAxisMoveIncre[Axis] + nextPulse >= Servo_Pulse_Count[Axis])
					{
						Pulse_PreSpeedStep[Axis] = Pulse_PreSpeedStep[Axis] - 1;
					}
				}
			}
			else if(Pulse_PreSpeedStep[Axis] > 0)
			{//ʣ�೤�ȱ���Ҫ��ʼ�������ٶȻ����Լ�С
				if(Pulse_PreSpeedStep[Axis] > 0 && m_InterAxisMoveLen[Axis] + InterpAxisPulseNum(Axis, Pulse_PreSpeedStep[Axis]) + m_InterAxisMoveIncre[Axis] >= Servo_Pulse_Count[Axis])
				{//ʣ�೤�ȱ���Ҫ��ʼ�������ٶȻ����Լ�С
					Pulse_PreSpeedStep[Axis] = Pulse_PreSpeedStep[Axis] - 1;
				}
			}
			else
			{//ʣ�೤�Ȱ���С�ٶ�����
			}
		}
		
		m_InterAxisMoveLen[Axis] = m_InterAxisMoveLen[Axis] + m_InterAxisMoveIncre[Axis];
		if(m_InterAxisMoveLen[Axis] > Servo_Pulse_Count[Axis])
		{
			m_InterAxisMoveLen[Axis] = Servo_Pulse_Count[Axis];
		}
		g_InterpOneAxisPulseBufWrite(Axis);
	}
}

/**************************************************************************************************
**  ��������  OneAxisSpeedInterpControl()
**	�����������
**	�����������
**	�������ܣ�����Ĳ岹ģʽ���ٶȿ���
**	��ע��	  			  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void OneAxisSpeedInterpControl(void)
{
	u16 i = 0;
	
	for(i = 0; i < Axis_Num + Ext_Axis_Num; i++){
		if(m_InterpAxisMoveFlag[i] != 0){//��Ҫ�˶�
			SpeedControl(i);
		}
	}
}

/**************************************************************************************************
**  ��������  SendPulse()
**	�����������
**	�����������
**	�������ܣ����ŷ�����������
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u8 SendPulse(u8 Axis, s32 axsiPosition, u32 maxSpeed)
{
	u16 i = 0;
	
	if(Axis >= Axis_Num + Ext_Axis_Num)
	{
		return 1;
	}
	
	Pulse_MaxSpeed[Axis] = maxSpeed + 1000;					//����ٶ�������1K
	if(Pulse_MaxSpeed[Axis] > MAX_SPEED)
	{
		Pulse_MaxSpeed[Axis] = MAX_SPEED;
	}
	
	/*�õ���Ҫ�˶����������ͷ���*/
	if(m_PulseTotalCounter[Axis] > axsiPosition)
	{
		Servo_Pulse_Count[Axis] = m_PulseTotalCounter[Axis] - axsiPosition;
		Axsis_Move_Direction[Axis] = NEGATIVE;
	}
	else if(m_PulseTotalCounter[Axis] < axsiPosition)
	{
		Servo_Pulse_Count[Axis] = axsiPosition - m_PulseTotalCounter[Axis];
		Axsis_Move_Direction[Axis] = POSITIVE;
	}
	else
	{
		return 1;
	}
	
	m_InterpAxisMoveFlag[Axis] = Axis + 1;		//��ʶ��ǰ��Ϊ�˶���
	Pulse_PreSpeedStep[Axis] = 0;
	m_InterAxisMoveLen[Axis] = 0;
	
	m_InterpAxisPulseBufWrite[Axis] = 0;
	m_InterpAxisPulseBufRead[Axis] = 0;

	m_StartPulseCounter[Axis] = m_PulseTotalCounter[Axis] - MINROBOTPOSITION;
	for(i=0; i<INTERP_AXIS_BUFFER_MAX; i++)
	{
		m_InterpAxisPulseBuf[Axis][i] = 0;
	}
	
	SpeedPlanningReset(Axis);									//��ʼ���˶���ز���
	AxisMoveAccCal(Axis);											//��ʼ�����ٶȺͼ��پ��룬����ʱ��ʱ���ã�����Ҫɾ��������������Ҫ�ٿ���������ٶȻ����ü��ٶȵĵط�����
	
	SpeedControl(Axis);
	
	return 0;
}

/**************************************************************************************************
**  ��������  InterpSendPulse()
**	�����������
**	�����������
**	�������ܣ��岹ģʽ���ŷ�����������
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u8 InterpSendPulse(void)
{
	u16 i = 0;
	u16 j = 0;
	
	if(m_InterpLenAxis >= Axis_Num + Ext_Axis_Num)
	{
		return 1;
	}
	
	Pulse_MaxSpeed[m_InterpLenAxis] = Axsis_MoveTarSpeed[m_InterpLenAxis] * JXS_Parameter.SpeedLevel * MAX_SPEED_CHANGE;
	if(Pulse_MaxSpeed[m_InterpLenAxis] < 1)
	{
		Pulse_MaxSpeed[m_InterpLenAxis] = 1;
	}
	else if(Pulse_MaxSpeed[m_InterpLenAxis] > MAX_SPEED)
	{
		Pulse_MaxSpeed[m_InterpLenAxis] = MAX_SPEED;
	}
	
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		if(m_InterpAxisFlag[i] != 0)
		{
			/*�õ���Ҫ�˶����������ͷ���*/
			if(Axsis_MoveTarPos[i] > m_PulseTotalCounter[i])
			{
				Servo_Pulse_Count[i] = Axsis_MoveTarPos[i] - m_PulseTotalCounter[i];
				Axsis_Move_Direction[i] = POSITIVE;
			}
			else
			{
				Servo_Pulse_Count[i] = m_PulseTotalCounter[i] - Axsis_MoveTarPos[i];
				Axsis_Move_Direction[i] = NEGATIVE;
			}
			
			m_InterAxisMoveLen[i] = 0;
			m_StartPulseCounter[i] = m_PulseTotalCounter[i] - MINROBOTPOSITION;
			
			if(m_InterpCurveFlag == INTER_CURVE_ONE)
			{//��ʾ���������߶�
				SpeedPlanningReset(i);						//��ʼ���˶���ز���
				/*������������ʼ��*/
				m_InterpAxisPulseBufWrite[i] = 0;
				m_InterpAxisPulseBufRead[i] = 0;
				
				for(j=0; j<INTERP_AXIS_BUFFER_MAX; j++)
				{
					m_InterpAxisPulseBuf[i][j] = 0;
				}
				
				Pulse_PreSpeedStep[i] = 0;
			}
		}
	}
	
	if(sCartesian_Para.length[0] > 0 && sCartesian_Para.length[1] > 0 && (m_InterpLenAxis == X_Axsis || m_InterpLenAxis == L_Axsis) \
			&& m_InterpAxisFlag[X_Axsis] != 0 && m_InterpAxisFlag[L_Axsis] != 0)
	{//����ʽ���ʱ����X-Y��ͬʱ�˶�ʱ����Ҫ���������ٶȣ���ֹ˦��
		if((sCartesian_Para.axisBackMinDir[X_Axsis] == sCartesian_Para.axisBackMinDir[L_Axsis] && Axsis_Move_Direction[X_Axsis] == Axsis_Move_Direction[L_Axsis]) \
				|| (sCartesian_Para.axisBackMinDir[X_Axsis] != sCartesian_Para.axisBackMinDir[L_Axsis] && Axsis_Move_Direction[X_Axsis] != Axsis_Move_Direction[L_Axsis]))
		{//ȷ��X-Y����˶�����˳ʱ�����ʱ��
			m_InterpAccFlag = 1;
			m_InterpAcc = JXS_Parameter.Accelerate.Time[m_InterpLenAxis] * ((float)(Servo_Pulse_Count[X_Axsis] + Servo_Pulse_Count[L_Axsis]) / Servo_Pulse_Count[m_InterpLenAxis]);
		}
	}
	
	AxisMoveAccCal(m_InterpLenAxis);							//��ʼ�����ٶȺͼ��پ��룬����ʱ��ʱ���ã�����Ҫɾ��������������Ҫ�ٿ���������ٶȻ����ü��ٶȵĵط�����
	m_InterpAccFlag = 0;
	
	/*�岹���㿪ʼ*/
	SpeedInterpControl();
	
	if(m_InterpCurveFlag == INTER_CURVE_ONE)
	{//��ʾ���������߶�
		m_InterpCurveFlag = INTER_CURVE_TWO;
	}
	
	return 0;
}

/**************************************************************************************************
**  ��������  SpeedInterpControl()
**	�����������
**	�����������
**	�������ܣ��岹ģʽ���ٶȿ���
**	��ע��	  			  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void  SpeedInterpControl(void)
{
	u16 i = 0;
	u32 perPulse = 0;
	float tempKey = 0.0f;
	s32 nextPulse = 0;

	if(m_InterpCurveFlag == INTER_CURVE_NO)
	{
		return;
	}
	
	while(1)
	{
		for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
		{/*ֻҪ���Ỻ�������˾ͷ���*/
			if(m_InterpAxisFlag[i] != 0)
			{
				if(g_InterpGetAxisPulseBufSize(i) < INTERP_AXIS_BUFFER_MIN)
				{
					return;
				}
			}
		}
	
		SlowPointDeal(m_InterpLenAxis);
		
		if(Pulse_StopRequest[m_InterpLenAxis] == 1)
		{//��ͣ���������
			if(Pulse_PreSpeedStep[m_InterpLenAxis] >= 1)
			{
				perPulse = InterpAxisPulseNum(m_InterpLenAxis, Pulse_PreSpeedStep[m_InterpLenAxis]) - InterpAxisPulseNum(m_InterpLenAxis, Pulse_PreSpeedStep[m_InterpLenAxis] - 1);
				Pulse_PreSpeedStep[m_InterpLenAxis]--;
			}
			else
			{
				perPulse = InterpAxisPulseNum(m_InterpLenAxis, Pulse_PreSpeedStep[m_InterpLenAxis]);
			}
		}
		else
		{
			if(Pulse_PreSpeedStep[m_InterpLenAxis] > 0)
			{
				perPulse = InterpAxisPulseNum(m_InterpLenAxis, Pulse_PreSpeedStep[m_InterpLenAxis]) - InterpAxisPulseNum(m_InterpLenAxis, Pulse_PreSpeedStep[m_InterpLenAxis] - 1);
			}
			else
			{
				perPulse = InterpAxisPulseNum(m_InterpLenAxis, 1) - InterpAxisPulseNum(m_InterpLenAxis, 0);
			}
			
			if(m_InterAxisMoveLen[m_InterpLenAxis] + InterpAxisPulseNum(m_InterpLenAxis, Pulse_PreSpeedStep[m_InterpLenAxis]) + perPulse < Servo_Pulse_Count[m_InterpLenAxis])
			{//ʣ�೤�Ȼ����Լ���
				if(Pulse_PreSpeedStep[m_InterpLenAxis] > Pulse_AccDecMaxSpeedStep[m_InterpLenAxis])
				{
					Pulse_PreSpeedStep[m_InterpLenAxis]--;
				}
				else
				{
					if(Pulse_PreSpeedStep[m_InterpLenAxis] + 1 < Pulse_AccDecMaxSpeedStep[m_InterpLenAxis])
					{//�����ǰ�ٶȻ�û��������ٶ�
						Pulse_PreSpeedStep[m_InterpLenAxis] = Pulse_PreSpeedStep[m_InterpLenAxis] + 1;
					}
					
					if(Pulse_PreSpeedStep[m_InterpLenAxis] > 0)
					{
						nextPulse = InterpAxisPulseNum(m_InterpLenAxis, Pulse_PreSpeedStep[m_InterpLenAxis]) - InterpAxisPulseNum(m_InterpLenAxis, Pulse_PreSpeedStep[m_InterpLenAxis] - 1);
					}
					else
					{
						nextPulse = InterpAxisPulseNum(m_InterpLenAxis, 1) - InterpAxisPulseNum(m_InterpLenAxis, 0);
					}
					
					if(Pulse_PreSpeedStep[m_InterpLenAxis] > 0 && m_InterAxisMoveLen[m_InterpLenAxis] + InterpAxisPulseNum(m_InterpLenAxis, Pulse_PreSpeedStep[m_InterpLenAxis]) + perPulse + nextPulse >= Servo_Pulse_Count[m_InterpLenAxis])
					{
						Pulse_PreSpeedStep[m_InterpLenAxis] = Pulse_PreSpeedStep[m_InterpLenAxis] - 1;
					}
				}
			}
			else if(Pulse_PreSpeedStep[m_InterpLenAxis] > 0)
			{//ʣ�೤�ȱ���Ҫ��ʼ�������ٶȻ����Լ�С
				if(Pulse_PreSpeedStep[m_InterpLenAxis] > 0 && m_InterAxisMoveLen[m_InterpLenAxis] + InterpAxisPulseNum(m_InterpLenAxis, Pulse_PreSpeedStep[m_InterpLenAxis]) + perPulse >= Servo_Pulse_Count[m_InterpLenAxis])
				{//ʣ�೤�ȱ���Ҫ��ʼ�������ٶȻ����Լ�С
					Pulse_PreSpeedStep[m_InterpLenAxis] = Pulse_PreSpeedStep[m_InterpLenAxis] - 1;
				}
			}
			else
			{//��ǰ�ٶ�Ϊ0���յ��ٶȲ�Ϊ0ʱ
			}
		}
		
		if(Servo_Pulse_Count[m_InterpLenAxis] < m_InterAxisMoveLen[m_InterpLenAxis] + perPulse)
		{
			perPulse = Servo_Pulse_Count[m_InterpLenAxis] - m_InterAxisMoveLen[m_InterpLenAxis];
			m_InterAxisMoveLen[m_InterpLenAxis] = Servo_Pulse_Count[m_InterpLenAxis];
		}
		else
		{
			m_InterAxisMoveLen[m_InterpLenAxis] = m_InterAxisMoveLen[m_InterpLenAxis] + perPulse;
		}
		m_InterAxisMoveIncre[m_InterpLenAxis] = perPulse;
		tempKey = (float)m_InterAxisMoveLen[m_InterpLenAxis] / Servo_Pulse_Count[m_InterpLenAxis];
		
		if(Servo_Pulse_Count[m_InterpLenAxis] <= m_InterAxisMoveLen[m_InterpLenAxis] \
				|| (Pulse_StopRequest[m_InterpLenAxis] == 1 && Pulse_PreSpeedStep[m_InterpLenAxis] == 0))
		{//���˶�����
			for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
			{
				if(i != m_InterpLenAxis && m_InterpAxisFlag[i] != 0)
				{
					if(Servo_Pulse_Count[m_InterpLenAxis] == m_InterAxisMoveLen[m_InterpLenAxis])
					{
						m_InterAxisMoveIncre[i] = Servo_Pulse_Count[i] - m_InterAxisMoveLen[i];
					}
					else
					{
						m_InterAxisMoveIncre[i] = Servo_Pulse_Count[i] * tempKey + 0.5f - m_InterAxisMoveLen[i];
						if(Servo_Pulse_Count[i] < m_InterAxisMoveLen[i] + m_InterAxisMoveIncre[i])
						{
							m_InterAxisMoveIncre[i] = Servo_Pulse_Count[i] - m_InterAxisMoveLen[i];
						}
					}
					m_InterAxisMoveLen[i] += m_InterAxisMoveIncre[i];
					Pulse_PreSpeedStep[i] = Pulse_PreSpeedStep[m_InterpLenAxis];
				}
			}
			g_InterpAxisPulseBufWrite();
			
			/*д���߶������������*/
			for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
			{
				if(m_InterpAxisFlag[i] != 0)
				{
					m_InterAxisMoveIncre[i] = INTERP_RUN_FINISH;
				}
			}
			g_InterpAxisPulseBufWrite();
			m_InterpCurveFlag = INTER_CURVE_NO;
			m_InterpLenAxis = 0xff;			//ÿ���߶��˶���ɣ����뽫�ñ�־λ����Ϊ0xff����ʾ��Ч
			return;
		}
		else
		{
			for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
			{
				if(i != m_InterpLenAxis && m_InterpAxisFlag[i] != 0)
				{//�������ÿ�����ڵ����������ٶ�
					m_InterAxisMoveIncre[i] = Servo_Pulse_Count[i] * tempKey + 0.5f - m_InterAxisMoveLen[i];
					if(Servo_Pulse_Count[i] <= m_InterAxisMoveLen[i] + m_InterAxisMoveIncre[i])// + 1)
					{
						m_InterAxisMoveIncre[i] = Servo_Pulse_Count[i] - m_InterAxisMoveLen[i];// - 1;
					}
					
					m_InterAxisMoveLen[i] += m_InterAxisMoveIncre[i];
					Pulse_PreSpeedStep[i] = Pulse_PreSpeedStep[m_InterpLenAxis];
				}
			}
			g_InterpAxisPulseBufWrite();
		}
	}
}

/**************************************************************************************************
**  ��������  AxisInterpIRQDeal()
**	�����������
**	�����������
**	�������ܣ��岹ģʽ���жϴ�����
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
s32 AxisInterpIRQDeal(u8 Axis)
{
	s32 ret = 0;
	u16 size = 0;
	s32 curPluseNum = 0;
	
READ_PULSE_BUF:
	size = (m_InterpAxisPulseBufWrite[Axis] + INTERP_AXIS_BUFFER_MAX - m_InterpAxisPulseBufRead[Axis]) % INTERP_AXIS_BUFFER_MAX;
	if(size == 0)
	{//��ѯ�����建����ʣ�����ݸ���
		ret = 1;
	}
	else
	{
		curPluseNum = m_InterpAxisPulseBuf[Axis][m_InterpAxisPulseBufRead[Axis]];
		m_InterpAxisPulseBufRead[Axis] = (m_InterpAxisPulseBufRead[Axis] + 1) % INTERP_AXIS_BUFFER_MAX;		//ָ��+1
	}
	
	if(ret == 0)
	{
		if(curPluseNum == LINE_RUN_FINISH)
		{
			goto READ_PULSE_BUF;
		}
		else if(curPluseNum == INTERP_RUN_FINISH)
		{
//			AxisMoveFlag[Axis] = DISABLE;
//			Pulse_StopRequest[Axis] = 0;
//			m_InterpAxisFlag[Axis] = 0;
			curPluseNum = BUF_INTERP_END;
		}
	}
	else
	{
		curPluseNum = BUF_INTERP_BUF_NULL;
	}
	
	return curPluseNum;
}

/**************************************************************************************************
**  ��������  g_AxisActionNextPosRead()
**	�����������
**	�����������
**	�������ܣ���岹λ�ö�ȡ����
**	��ע��	  �ú�����EtherCAT�ж������
**  ���ߣ�      
**  �������ڣ�
***************************************************************************************************/
void g_AxisActionNextPosRead(void)
{
	u16 i = 0;
	s32 tarPos = 0;
	
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		if(ServoWorkModeRead(Axis_To_ID(i)) == SERVO_MODE_CYC_CSP)
		{
			tarPos = AxisInterpIRQDeal(i);
			if(tarPos == BUF_INTERP_END)
			{
				AxisMoveFlag[i] = DISABLE;
				Pulse_StopRequest[i] = 0;
				m_InterpAxisFlag[i] = 0;
			}
			else if(tarPos == BUF_INTERP_BUF_NULL)
			{
				
			}
			else
			{
				ServoCSP_PDO(Axis_To_ID(i), (tarPos + JDZ_ZeroPosition[i]) * Axsis_ParPosChange);
			}
		}
	}
}


/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
