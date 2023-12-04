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

s32 m_PulseTotalCounter[Axis_Num + Ext_Axis_Num] = {0};									//脉冲数
u8  AxisMoveFlag[Axis_Num + Ext_Axis_Num] = {0};
u32 Servo_Pulse_Count[Axis_Num + Ext_Axis_Num] = {0, 0, 0, 0, 0, 0};		//需要发送的脉冲个数

/*加减速相关宏及变量*/
#define SPEED_TIMER_BASE									1000000          							//脉冲计算时基
#define SPEED_CONTROL_PER									2000          								//插补周期，单位us
#define START_SPEED_FRE										1000         									//起始速度

/*缓冲区命令*/
#define BUF_INTERP_LINE_END								(0x7FFFFFC0)          				//插补线段结束
#define BUF_INTERP_END										(0x7FFFFFC1)          				//插补结束
#define BUF_INTERP_BUF_NULL								(0x7FFFFFC2)          				//插补缓冲区为空
/*插补运动命令定义*/
#define LINE_RUN_FINISH       						(0x7FFFFFE0) 									//线段运行完成
#define INTERP_RUN_FINISH       					(0x7FFFFFE1) 									//多端运动完成

u32 Pulse_MaxSpeed[Axis_Num + Ext_Axis_Num] = {0};																		//最大速度频率
u32 Pulse_MaxAcc[Axis_Num + Ext_Axis_Num] = {1000, 1000, 1000, 1000, 1000, 1000};  	  //最大加速度
u16 Pulse_AccDecMaxSpeedStep[Axis_Num + Ext_Axis_Num] = {0};	    										//加减速最大速度所在位置
u16 Pulse_PreSpeedStep[Axis_Num + Ext_Axis_Num] = {0};	    													//当前速度所在位置计数
u32 m_InterpAcc = 0;																																	//联动插补时的实际加速度值
u8  m_InterpAccFlag = 0;																															//联动插补时的加速度变化标志

/*脉冲及位置缓冲区相关参数*/
#define INTERP_AXIS_BUFFER_MAX 10																											//联动轴脉冲缓冲区大小
#define INTERP_AXIS_BUFFER_MIN 4																											//联动轴脉冲缓冲区可写最小剩余空间，一定要大于5，防止写命令写不进去
s32 m_InterpAxisPulseBuf[Axis_Num + Ext_Axis_Num][INTERP_AXIS_BUFFER_MAX] = {{0}};		//轴脉冲缓冲区
u16 m_InterpAxisPulseBufWrite[Axis_Num + Ext_Axis_Num] = {0};													//缓冲区写指针
u16 m_InterpAxisPulseBufRead[Axis_Num + Ext_Axis_Num] = {0};													//缓冲区读指针
s32 m_InterAxisMoveLen[Axis_Num + Ext_Axis_Num] = {0};																//已经运动的长度
s32 m_StartPulseCounter[Axis_Num + Ext_Axis_Num] = {0};																//保存轴运动的起始位置
s32 m_InterAxisMoveIncre[Axis_Num + Ext_Axis_Num] = {0};															//存放移动增量
u8  m_InterpAxisFlag[Axis_Num + Ext_Axis_Num] = {0};																	//需要插补的轴标志
u8  m_InterpAxisMoveFlag[Axis_Num + Ext_Axis_Num] = {0};															//用于标识单轴的插补运动
u8  Pulse_StopRequest[Axis_Num + Ext_Axis_Num] = {0};	    														//停止请求 1请求暂停 0无效
u8  m_InterpLenAxis = 0xff;																														//插补长轴编号
u8  m_InterpCurveFlag = INTER_CURVE_NO;																								//曲线插补运动进度标志

/**************************************************************************************************
**  函数名：  InterpAxisPreAcc()
**	输入参数：无
**	输出参数：无
**	函数功能：计算每一个速度阶段对应的加速度
**	备注：	  			  
**  作者：    
**  开发日期：
***************************************************************************************************/
u32 InterpAxisPreAcc(u8 Axis, u16 speedStep)
{
	u32 preAcc = 0;
	
	preAcc = Pulse_MaxAcc[Axis];
	
	return preAcc;
}

/**************************************************************************************************
**  函数名：  InterpAxisPulseNum()
**	输入参数：无
**	输出参数：无
**	函数功能：计算每到一个速度阶段对应的加速距离或减速距离
**	备注：	  			  
**  作者：    
**  开发日期：
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
**  函数名：  SpeedPlanningReset()
**	输入参数：无
**	输出参数：无
**	函数功能：复位运动相关变量及标志位
**	备注：	  			  
**  作者：    
**  开发日期：
***************************************************************************************************/
void SpeedPlanningReset(u8 Axis)
{
	Pulse_StopRequest[Axis] = 0;
}

/**************************************************************************************************
**  函数名：  ClosePulseReset()
**	输入参数：无
**	输出参数：无
**	函数功能：脉冲发送结束后复位标志位
**	备注：	  			  
**  作者：    
**  开发日期：
***************************************************************************************************/
void ClosePulseReset(u8 Axis)
{
//	Axsis_Move_Direction[Axis] = NONE;
//	Pulse_Counter[Axis] = 0;
}

/*
**功能：获取轴脉冲缓冲区剩余数据个数
**参数：l_AxisNum 轴编号,0 ~ INTERP_AXIS_MAX_NUM-1
**返回：剩余数据个数
*/
u16 g_InterpGetAxisPulseBufNum(u8 l_AxisNum)
{
	u16 size = 0;

	size = (m_InterpAxisPulseBufWrite[l_AxisNum] + INTERP_AXIS_BUFFER_MAX - m_InterpAxisPulseBufRead[l_AxisNum]) % INTERP_AXIS_BUFFER_MAX;
	
	return size;
}

/*
**功能：获取轴脉冲缓冲区剩余大小
**参数：l_AxisNum 轴编号,0 ~ INTERP_AXIS_MAX_NUM-1
**返回：剩余大小
*/
u16 g_InterpGetAxisPulseBufSize(u8 l_AxisNum)
{
	u16 size = 0;
	
	size = INTERP_AXIS_BUFFER_MAX - g_InterpGetAxisPulseBufNum(l_AxisNum);
	
	return size;
}

/*
**功能：轴脉冲缓冲区指针加1
**参数：l_pBuf 缓冲区指针
**返回：OK or ERR
*/
s16 g_InterpAxisPulseBufAdd(u16 *l_pBuf)
{
	*l_pBuf = ((*l_pBuf) + 1) % INTERP_AXIS_BUFFER_MAX;		//指针+1
	
	return 0;	//Ok
}

/*
**功能：轴脉冲缓冲区指针加len
**参数：l_pBuf 缓冲区指针
**返回：OK or ERR
*/
s16 g_InterpAxisPulseBufAddLen(u16 *l_pBuf, u16 l_Len)
{
	*l_pBuf = ((*l_pBuf) + l_Len) % INTERP_AXIS_BUFFER_MAX;		//指针+1
	
	return 0;	//Ok
}

/*
**功能：写脉冲缓冲区
**参数：l_AxisNum 轴个数
**返回：OK or ERR
*/
s16 g_InterpAxisPulseBufWrite(void)
{	
	u16	i = 0;
	
	/*只要有轴缓冲区满了就返回，写数据必须确保空间大于等于 INTERP_AXIS_BUFFER_MIN*/
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		if(m_InterpAxisFlag[i] != 0 && g_InterpGetAxisPulseBufSize(i) < 1)//只要有空间就往里面写
		{
			return 0;
		}
	}
	
	/*脉冲写入缓冲区*/
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		if(m_InterpAxisFlag[i] != 0)//只要有空间就往里面写
		{
			if(m_InterAxisMoveIncre[i] == INTERP_RUN_FINISH || m_InterAxisMoveIncre[i] == LINE_RUN_FINISH)
			{
				m_InterpAxisPulseBuf[i][m_InterpAxisPulseBufWrite[i]] = m_InterAxisMoveIncre[i];//m_InterAxisMoveIncre[i];	//写入脉冲增量数
			}
			else if(Axsis_Move_Direction[i] == POSITIVE)
			{
				m_InterpAxisPulseBuf[i][m_InterpAxisPulseBufWrite[i]] = m_StartPulseCounter[i] + m_InterAxisMoveLen[i];//m_InterAxisMoveIncre[i];	//写入脉冲增量数
			}
			else
			{
				m_InterpAxisPulseBuf[i][m_InterpAxisPulseBufWrite[i]] = m_StartPulseCounter[i] - m_InterAxisMoveLen[i];//m_InterAxisMoveIncre[i];	//写入脉冲增量数
			}
			g_InterpAxisPulseBufAdd(&m_InterpAxisPulseBufWrite[i]);
		}
	}
	
	return 0;
}

/*
**功能：写脉冲缓冲区
**参数：l_AxisNum 轴个数
**返回：OK or ERR
*/
s16 g_InterpOneAxisPulseBufWrite(u8 Axis)
{	
	if(g_InterpGetAxisPulseBufSize(Axis) < 1)//只要有空间就往里面写
	{
		return 0;
	}
	
	/*脉冲写入缓冲区*/
	if(m_InterpAxisMoveFlag[Axis] != 0)//只要有空间就往里面写
	{
		if(m_InterAxisMoveIncre[Axis] == INTERP_RUN_FINISH || m_InterAxisMoveIncre[Axis] == LINE_RUN_FINISH)
		{
			m_InterpAxisPulseBuf[Axis][m_InterpAxisPulseBufWrite[Axis]] = m_InterAxisMoveIncre[Axis];//m_InterAxisMoveIncre[i];	//写入脉冲增量数
		}
		else if(Axsis_Move_Direction[Axis] == POSITIVE)
		{
			m_InterpAxisPulseBuf[Axis][m_InterpAxisPulseBufWrite[Axis]] = m_StartPulseCounter[Axis] + m_InterAxisMoveLen[Axis];//m_InterAxisMoveIncre[i];	//写入脉冲增量数
		}
		else
		{
			m_InterpAxisPulseBuf[Axis][m_InterpAxisPulseBufWrite[Axis]] = m_StartPulseCounter[Axis] - m_InterAxisMoveLen[Axis];//m_InterAxisMoveIncre[i];	//写入脉冲增量数
		}
		g_InterpAxisPulseBufAdd(&m_InterpAxisPulseBufWrite[Axis]);
	}
	
	return 0;
}

/*
**功能：读脉冲缓冲区
**参数：l_AxisNum 轴编号
**参数：l_PluseNum 脉冲数
**参数：l_Fre 脉冲频率
**返回：
**注意：如果读取的返回值为 INTERP_RET_LOC_READ_EER ，说明函数调用失败
*/
s32 g_InterpAxisPulseBufRead(u8 l_AxisNum, s32 *l_PluseNum, u32 *l_Fre)
{	
	if(g_InterpGetAxisPulseBufNum(l_AxisNum) == 0)	//查询轴脉冲缓冲区剩余数据个数
	{
		return 1;
	}
	
	*l_PluseNum = m_InterpAxisPulseBuf[l_AxisNum][m_InterpAxisPulseBufRead[l_AxisNum]];
	
	g_InterpAxisPulseBufAdd(&m_InterpAxisPulseBufRead[l_AxisNum]);	//脉冲缓冲区指针+1

	return 0;
}

/**************************************************************************************************
**  函数名：  AxisMoveAccCal()
**	输入参数：accTime 加速时间，单位ms
**	输出参数：无
**	函数功能：轴运动计算加速度，即每个插补周期增加的频率
**	备注：		每次加速度变化时调用一次，开机时调用一次
**  作者：     
**  开发日期：
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
	Pulse_MaxAcc[Axis] = MAX_SPEED / (accValue * 1000 / SPEED_CONTROL_PER);						//计算加速度
		
	speed = START_SPEED_FRE;
	Pulse_AccDecMaxSpeedStep[Axis] = 1;
	while(speed + InterpAxisPreAcc(Axis, Pulse_AccDecMaxSpeedStep[Axis]) < Pulse_MaxSpeed[Axis])
	{//求出最大速度所在步骤
		speed += InterpAxisPreAcc(Axis, Pulse_AccDecMaxSpeedStep[Axis]);
		Pulse_AccDecMaxSpeedStep[Axis]++;
	}
}

/**************************************************************************************************
**  函数名：  SlowPointDeal()
**	输入参数：accTime 加速时间，单位ms
**	输出参数：无
**	函数功能：修改最大速度
**	备注：		每次加速度变化时调用一次，开机时调用一次
**  作者：     
**  开发日期：
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
		{//求出最大速度所在步骤
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
		{//还未到达减速位置
			return 0;
		}
		
		Pulse_AccDecMaxSpeedStep[Axis] = maxSpeedStep;
		SlowPointFlag[Axis] = 0;
	}
	
	return 0;
}

/**************************************************************************************************
**  函数名：  SpeedControl()
**	输入参数：无
**	输出参数：无
**	函数功能：单轴的速度控制
**	备注：	  			  
**  作者：    
**  开发日期：
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
		{//请求减速
			if(Pulse_PreSpeedStep[Axis] >= 1)
			{
				m_InterAxisMoveIncre[Axis] = InterpAxisPulseNum(Axis, Pulse_PreSpeedStep[Axis]) - InterpAxisPulseNum(Axis, Pulse_PreSpeedStep[Axis] - 1);
				Pulse_PreSpeedStep[Axis] = Pulse_PreSpeedStep[Axis] - 1;
			}
			else
			{//暂停结束
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
			{//暂停结束
				m_InterAxisMoveIncre[Axis] = INTERP_RUN_FINISH;
				g_InterpOneAxisPulseBufWrite(Axis);
				m_InterpAxisMoveFlag[Axis] = 0;
				return;
			}
			else if(m_InterAxisMoveLen[Axis] + InterpAxisPulseNum(Axis, Pulse_PreSpeedStep[Axis]) + m_InterAxisMoveIncre[Axis] < Servo_Pulse_Count[Axis])
			{//剩余长度还可以加速
				if(Pulse_PreSpeedStep[Axis] > Pulse_AccDecMaxSpeedStep[Axis])
				{
					Pulse_PreSpeedStep[Axis]--;
				}
				else 
				{
					if(Pulse_PreSpeedStep[Axis] + 1 < Pulse_AccDecMaxSpeedStep[Axis])
					{//如果当前速度还没到达最大速度
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
			{//剩余长度必须要开始减速且速度还可以减小
				if(Pulse_PreSpeedStep[Axis] > 0 && m_InterAxisMoveLen[Axis] + InterpAxisPulseNum(Axis, Pulse_PreSpeedStep[Axis]) + m_InterAxisMoveIncre[Axis] >= Servo_Pulse_Count[Axis])
				{//剩余长度必须要开始减速且速度还可以减小
					Pulse_PreSpeedStep[Axis] = Pulse_PreSpeedStep[Axis] - 1;
				}
			}
			else
			{//剩余长度按最小速度运行
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
**  函数名：  OneAxisSpeedInterpControl()
**	输入参数：无
**	输出参数：无
**	函数功能：单轴的插补模式的速度控制
**	备注：	  			  
**  作者：    
**  开发日期：
***************************************************************************************************/
void OneAxisSpeedInterpControl(void)
{
	u16 i = 0;
	
	for(i = 0; i < Axis_Num + Ext_Axis_Num; i++){
		if(m_InterpAxisMoveFlag[i] != 0){//需要运动
			SpeedControl(i);
		}
	}
}

/**************************************************************************************************
**  函数名：  SendPulse()
**	输入参数：无
**	输出参数：无
**	函数功能：给伺服器发送脉冲
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
u8 SendPulse(u8 Axis, s32 axsiPosition, u32 maxSpeed)
{
	u16 i = 0;
	
	if(Axis >= Axis_Num + Ext_Axis_Num)
	{
		return 1;
	}
	
	Pulse_MaxSpeed[Axis] = maxSpeed + 1000;					//最大速度至少是1K
	if(Pulse_MaxSpeed[Axis] > MAX_SPEED)
	{
		Pulse_MaxSpeed[Axis] = MAX_SPEED;
	}
	
	/*得到需要运动的脉冲数和方向*/
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
	
	m_InterpAxisMoveFlag[Axis] = Axis + 1;		//标识当前轴为运动轴
	Pulse_PreSpeedStep[Axis] = 0;
	m_InterAxisMoveLen[Axis] = 0;
	
	m_InterpAxisPulseBufWrite[Axis] = 0;
	m_InterpAxisPulseBufRead[Axis] = 0;

	m_StartPulseCounter[Axis] = m_PulseTotalCounter[Axis] - MINROBOTPOSITION;
	for(i=0; i<INTERP_AXIS_BUFFER_MAX; i++)
	{
		m_InterpAxisPulseBuf[Axis][i] = 0;
	}
	
	SpeedPlanningReset(Axis);									//初始化运动相关参数
	AxisMoveAccCal(Axis);											//初始化加速度和加速距离，测试时临时调用，后续要删除，正常程序需要再开机读完加速度或设置加速度的地方调用
	
	SpeedControl(Axis);
	
	return 0;
}

/**************************************************************************************************
**  函数名：  InterpSendPulse()
**	输入参数：无
**	输出参数：无
**	函数功能：插补模式给伺服器发送脉冲
**	备注：	  
**  作者：    
**  开发日期：
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
			/*得到需要运动的脉冲数和方向*/
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
			{//表示曲线首条线段
				SpeedPlanningReset(i);						//初始化运动相关参数
				/*缓冲区参数初始化*/
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
	{//立柱式码垛时，当X-Y轴同时运动时，需要控制最大加速度，防止甩包
		if((sCartesian_Para.axisBackMinDir[X_Axsis] == sCartesian_Para.axisBackMinDir[L_Axsis] && Axsis_Move_Direction[X_Axsis] == Axsis_Move_Direction[L_Axsis]) \
				|| (sCartesian_Para.axisBackMinDir[X_Axsis] != sCartesian_Para.axisBackMinDir[L_Axsis] && Axsis_Move_Direction[X_Axsis] != Axsis_Move_Direction[L_Axsis]))
		{//确保X-Y轴的运动都是顺时针或逆时针
			m_InterpAccFlag = 1;
			m_InterpAcc = JXS_Parameter.Accelerate.Time[m_InterpLenAxis] * ((float)(Servo_Pulse_Count[X_Axsis] + Servo_Pulse_Count[L_Axsis]) / Servo_Pulse_Count[m_InterpLenAxis]);
		}
	}
	
	AxisMoveAccCal(m_InterpLenAxis);							//初始化加速度和加速距离，测试时临时调用，后续要删除，正常程序需要再开机读完加速度或设置加速度的地方调用
	m_InterpAccFlag = 0;
	
	/*插补计算开始*/
	SpeedInterpControl();
	
	if(m_InterpCurveFlag == INTER_CURVE_ONE)
	{//表示曲线首条线段
		m_InterpCurveFlag = INTER_CURVE_TWO;
	}
	
	return 0;
}

/**************************************************************************************************
**  函数名：  SpeedInterpControl()
**	输入参数：无
**	输出参数：无
**	函数功能：插补模式的速度控制
**	备注：	  			  
**  作者：    
**  开发日期：
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
		{/*只要有轴缓冲区满了就返回*/
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
		{//暂停，请求减速
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
			{//剩余长度还可以加速
				if(Pulse_PreSpeedStep[m_InterpLenAxis] > Pulse_AccDecMaxSpeedStep[m_InterpLenAxis])
				{
					Pulse_PreSpeedStep[m_InterpLenAxis]--;
				}
				else
				{
					if(Pulse_PreSpeedStep[m_InterpLenAxis] + 1 < Pulse_AccDecMaxSpeedStep[m_InterpLenAxis])
					{//如果当前速度还没到达最大速度
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
			{//剩余长度必须要开始减速且速度还可以减小
				if(Pulse_PreSpeedStep[m_InterpLenAxis] > 0 && m_InterAxisMoveLen[m_InterpLenAxis] + InterpAxisPulseNum(m_InterpLenAxis, Pulse_PreSpeedStep[m_InterpLenAxis]) + perPulse >= Servo_Pulse_Count[m_InterpLenAxis])
				{//剩余长度必须要开始减速且速度还可以减小
					Pulse_PreSpeedStep[m_InterpLenAxis] = Pulse_PreSpeedStep[m_InterpLenAxis] - 1;
				}
			}
			else
			{//当前速度为0，终点速度不为0时
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
		{//轴运动结束
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
			
			/*写入线段运行完成命令*/
			for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
			{
				if(m_InterpAxisFlag[i] != 0)
				{
					m_InterAxisMoveIncre[i] = INTERP_RUN_FINISH;
				}
			}
			g_InterpAxisPulseBufWrite();
			m_InterpCurveFlag = INTER_CURVE_NO;
			m_InterpLenAxis = 0xff;			//每段线段运动完成，必须将该标志位设置为0xff，表示无效
			return;
		}
		else
		{
			for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
			{
				if(i != m_InterpLenAxis && m_InterpAxisFlag[i] != 0)
				{//计算短轴每个周期的脉冲数和速度
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
**  函数名：  AxisInterpIRQDeal()
**	输入参数：无
**	输出参数：无
**	函数功能：插补模式的中断处理函数
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
s32 AxisInterpIRQDeal(u8 Axis)
{
	s32 ret = 0;
	u16 size = 0;
	s32 curPluseNum = 0;
	
READ_PULSE_BUF:
	size = (m_InterpAxisPulseBufWrite[Axis] + INTERP_AXIS_BUFFER_MAX - m_InterpAxisPulseBufRead[Axis]) % INTERP_AXIS_BUFFER_MAX;
	if(size == 0)
	{//查询轴脉冲缓冲区剩余数据个数
		ret = 1;
	}
	else
	{
		curPluseNum = m_InterpAxisPulseBuf[Axis][m_InterpAxisPulseBufRead[Axis]];
		m_InterpAxisPulseBufRead[Axis] = (m_InterpAxisPulseBufRead[Axis] + 1) % INTERP_AXIS_BUFFER_MAX;		//指针+1
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
**  函数名：  g_AxisActionNextPosRead()
**	输入参数：无
**	输出参数：无
**	函数功能：轴插补位置读取更新
**	备注：	  该函数在EtherCAT中断里调用
**  作者：      
**  开发日期：
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
