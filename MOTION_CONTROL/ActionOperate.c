/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : AutoOperate.c
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 28/10/2015
* Description        : 各个动作操作函数
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

u32 Program_RunTime = 0;									      //程序单次运行时间
u32 Program_RunTime_Count = 0;						      //程序单次运行时间计数器
u32 m_ProRunTimeTotal = 0;											//程序总的运行时间
u32 m_ProRunTimeTotalCount = 0;									//程序总的运行时间计数器
u32 m_ProRunTimeCumulate = 0;										//程序此次累计运行时间
u32 m_PreProRunTimeCumulate = 0;								//此前程序累计运行时间
u32 m_PowerOnTimeTotal = 0;											//设备总的开机机间
u32 m_PowerOnTimeTotalCount = 0;								//设备总的开机时间计数器
u32 m_PowerOnTimeCumulate = 0;									//设备总的累计开机时间
u32 m_PrPowerOnTimeCumulate = 0;                //此前设备总的累计开机时间

u8  Program_Reset = FALSE;					  		      //复位程序完成标志

s32 Increment_Target[Axis_Num + Ext_Axis_Num] = {0};					//增量运动时用于记录目标位置
u8  Increment_Finished[Axis_Num + Ext_Axis_Num] = {FALSE};		//增量运动指令完成标志，防止增量运动的时候点击暂停，再启动会继续增量原设定值

s32 SlowPointIncrement[Axis_Num + Ext_Axis_Num] = {0};				//减速度点增量位置
s32 SlowPointSpeed[Axis_Num + Ext_Axis_Num] = {0};						//减速度点速度
u8  SlowPointFlag[Axis_Num + Ext_Axis_Num] = {FALSE};					//允许减速度点标志

s32 AdvancePointInc[Axis_Num + Ext_Axis_Num] = {0};			   		//提前确认的量
u8  AdvancePointFlag[Axis_Num + Ext_Axis_Num] = {FALSE};			//提前确认的标志

u8  WaitAxisMoveFlag[Axis_Num + Ext_Axis_Num] = {FALSE};			//等待轴完成动作标志

u32 m_PulseOutputStartTime[OUTPUT_NUM] = {0};									//脉宽输出命令对应的开始时间
u32 m_PulseOutputEndTime[OUTPUT_NUM] = {0};										//脉宽输出命令对应的结束时间
u8  m_PulseOutputSta[OUTPUT_NUM] = {0};												//脉宽输出命令对应的置位或复位

u8  Axsis_MoveProNum[Axis_Num + Ext_Axis_Num] = {0};					//保存轴运动的线程编号
s32 Axsis_MoveTarPos[Axis_Num + Ext_Axis_Num] = {0};					//保存轴运动的目标位置
u32 Axsis_MoveTarSpeed[Axis_Num + Ext_Axis_Num] = {0};				//保存轴运动的速度

u8 Axsis_InterAxisNum[Axis_Num + Ext_Axis_Num] = {0};					//插补轴编号
u8 Axsis_InterAxisCount = 0;																	//插补轴个数
u8 Axsis_InterAxisSpeed = 0;																	//插补拐点速度
u8 Axsis_InterAxisMaxSpeed = 0;																//插补最大速度

/**************************************************************************************************
**  函数名：Axis_To_ID
**	输入参数：Axis 轴编号
**	输出参数：
**	函数功能：轴编号转为CANOPEN的轴ID
**	备注：	
**  作者：         
**  开发日期：六轴改 
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
**  函数名：  ServoAccDecSet()
**	输入参数：Axsis轴编号
**	输出参数：无
**	函数功能：轴移动
**	备注：	  无
**  作者：    
**  开发日期：
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
		case 0://禾川电机
			accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
		  decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
			break;
		case 1://汇川电机
			accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
		  decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
			break;
		case 2://迈信电机
			accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
		  decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
			break;
		case 3://雷赛电机
			accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
		  decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
			break;
		case 4://信捷电机
			accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
		  decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
			break;
		case 5://台邦电机
			accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
		  decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
			break;
		case 6://超川电机
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
//**  函数名：  ServoAccDecSet_CSV()
//**	输入参数：Axsis轴编号
//**	输出参数：无
//**	函数功能：轴周期同步速度模式时的加速度设置
//**	备注：	  无
//**  作者：    
//**  开发日期：
//***************************************************************************************************/
//void ServoAccDecSet_CSV(u8 Axsis)
//{
//	u32 accTemp = 0;
//	u32 decTemp = 0;
//	
//	switch(JDZ_Parameter.Server)
//	{
//		case 0://禾川电机
//			accTemp = 30000 * Axsis_ParVelChange;
//		  decTemp = 30000 * Axsis_ParVelChange;
//			break;
//		case 1://汇川电机
//			accTemp = 30000 * Axsis_ParVelChange;
//		  decTemp = 30000 * Axsis_ParVelChange;
//			break;
//		case 2://迈信电机
//			accTemp = 30000 * Axsis_ParVelChange;
//		  decTemp = 30000 * Axsis_ParVelChange;
//			break;
//		case 3://雷赛电机
//			accTemp = 30000 * Axsis_ParVelChange;
//		  decTemp = 30000 * Axsis_ParVelChange;
//			break;
//		case 4://信捷电机
//			accTemp = 30000 * Axsis_ParVelChange;
//		  decTemp = 30000 * Axsis_ParVelChange;
//			break;
//		case 5://台邦电机
//			accTemp = 30000 * Axsis_ParVelChange;
//		  decTemp = 30000 * Axsis_ParVelChange;
//			break;
//		case 6://超川电机
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
		case 0://禾川电机
			accTemp = 3000 * Axsis_ParVelChange;
		  decTemp = 3000 * Axsis_ParVelChange;
			break;
		case 1://汇川电机
			accTemp = 3000 * Axsis_ParVelChange;
		  decTemp = 3000 * Axsis_ParVelChange;
			break;
		case 2://迈信电机
			accTemp = 3000 * Axsis_ParVelChange;
		  decTemp = 3000 * Axsis_ParVelChange;
			break;
		case 3://雷赛电机
			accTemp = 3000 * Axsis_ParVelChange;
		  decTemp = 3000 * Axsis_ParVelChange;
			break;
		case 4://信捷电机
			accTemp = 3000 * Axsis_ParVelChange;
		  decTemp = 3000 * Axsis_ParVelChange;
			break;
		case 5://台邦电机
			accTemp = 3000 * Axsis_ParVelChange;
		  decTemp = 3000 * Axsis_ParVelChange;
			break;
		case 6://超川电机
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
**  函数名：  Key_Delay()
**	输入参数：Delay_Time主程序的超时时间
**	输出参数：无
**	函数功能：主程序的指令执行超时时间
**	备注：	  无
**  作者：      
**  开发日期：
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
**  函数名：  SubProgram_Key_Delay()
**	输入参数：subProNum子程序编号 Delay_Time超时时间
**	输出参数：无
**	函数功能：子程序的指令执行超时函数
**	备注：	  无
**  作者：       
**  开发日期：
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
**  函数名：GetIO_Value
** 输入参数：IONum IO编号  effect，1为有效信号，0为无效信号 proType，0主程序 不为0子程序号+1
** 输出参数：1或0
** 函数功能：读取引脚值
** 备注：   引脚常开时，低电平（有信号）返回1，高电平返回0；引脚常闭时，相反
**  作者：         
**  开发日期：六轴改
***************************************************************************************************/
u8 GetIO_Value(u8 ioNum, u8 effect, u8 proType)
{
	u8 result = 0;
	u8 index = ioNum / 8;
	u8 offset = ioNum % 8;
	if(effect==0)
	{//无效信号
		if(IO_Sign_On_Off[ioNum] == 0)
		{//常开
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
		{//常闭
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
	{//有效信号
		if(IO_Sign_On_Off[ioNum] == 0)
		{//常开
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
		{//常闭
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
	{//下降沿，检测低
		if((proType == 0 && Detect_Falling_Edge == 1) \
				|| (proType > 0 && Detect_Falling_Edge_Sub[proType - 1] == 1))
		{//一开始检测到低电平
			if(((Input_Detect_Status[index] >> offset) & 0x01) == 0)
			{//检测到高电平
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
		{//一开始检测到高电平,或者再次检测到高
			if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
			{//检测到低电平,停止
				result = 1;
			}
		}
	}
	else if(effect==3)
	{//上升沿，检测高
		if((proType == 0 && Detect_Rising_Edge == 1) \
				|| (proType > 0 && Detect_Rising_Edge_Sub[proType - 1] == 1))
		{//一开始检测到高电平
			if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
			{//检测到低电平
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
		{//一开始检测到低电平,或者再次检测到低
			if(((Input_Detect_Status[index] >> offset) & 0x01) == 0)
			{//检测到高电平,停止
				result = 1;
			}
		}
	}
	return result;
}
	
/**************************************************************************************************
**  函数名：  AXisMove()
**	输入参数：Axsis轴编号  Axsis_Position目标位置 Axsis_Speed轴速度 speedMod速度计算模式
**	输出参数：无
**	函数功能：轴移动
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
u8 AXisMove(u8 Axsis, s32 Axsis_Position, u32 Axsis_Speed)
{
	u16 Result = 0;
	
	//搜索未完成的动作
	if(AxisMoveFlag[Axsis] == 1)
	{//当前轴动作未完成
		WaitAxisMoveFlag[Axsis] = 1;//执行单轴命令
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
		{//减速点处理
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
		{//减速点处理
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
		//防止目标位置等于当前位置时，减速点和提前确认影响下一个移动命令
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
**  函数名：  Servo_MoveFinishSta(u8)
**	输入参数：Axis 轴编号   AxisTarPosition目标位置
**	输出参数：无
**	函数功能：动作确认
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
u16 Servo_MoveFinishSta(u8 Axsis, s32 AxisTarPosition)
{
	u16 ret = 0;
	
	if(WaitAxisMoveFlag[Axsis] == 1)
	{//等待动作完成过进行单轴运动
		if(AxisMoveFlag[Axsis] == DISABLE)
		{
			AXisMove(Axsis, AxisTarPosition, Axsis_MoveTarSpeed[Axsis]);
		}
		return 0;
	}
	else if(WaitAxisMoveFlag[Axsis] == 2)
	{//等待动作完成过进行多轴联动
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
	{//提前确认
		if(AxisTarPosition < m_PulseTotalCounter[Axsis] + abs(AdvancePointInc[Axsis]) && m_PulseTotalCounter[Axsis] < AxisTarPosition + abs(AdvancePointInc[Axsis]))
		{//在目标点的提前确认范围内，就可以确认成功
			ret = 1;
			AdvancePointFlag[Axsis] = 0;			//脉冲绝对值(==2)和EtherCAT(==0)不一样
		}
		else
		{
			ret = 0;
		}
	}

	return ret;
}

/**************************************************************************************************
**  函数名：  AutoActionMove()
**	输入参数：procNum 运行的进程号加偏移值
**	输出参数：
**	函数功能：实现轴运动同时停止
**	备注：	  
**  作者：    
**  开发日期：
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
	{//搜索提前确认标志
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
		{//搜索提前确认标志
			if(i != m_InterpLenAxis && m_InterpAxisFlag[i] == procNum)
			{
				lenAxisDis = axisMoveDis[i] * AdvancePointInc[m_InterpLenAxis] / axisMoveMaxLen;
				AdvancePointInc[i] = lenAxisDis;
				AdvancePointFlag[i] = 1;
			}
		}
	}
		
	//处理减速点
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{//寻找是否存在减速点
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
			{//减速点处理
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
			{//减速点处理
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
**  函数名：  AXisSncyMove()
**	输入参数：procNum 运行的进程号
**	输出参数：
**	函数功能：实现轴直线插补运动
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
u8 AXisSncyMove(u8 procNum)
{
	u16 i = 0;
	u8 axisNum = 0;
	u32 axisMoveLen = 0;
	u32 axisMoveMaxLen = 0;
	u8  waitAxisMoveFlag = 0;
	u8 axisNum1 = 0;
	
	//防止目标位置等于当前位置时，减速点和提前确认影响下一个移动命令
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
		
	//搜索未完成动作的轴
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
	{//有轴动作未完成
		for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
		{
			if(Axsis_MoveProNum[i] == procNum)
			{
				WaitAxisMoveFlag[i] = 2;			//2表示执行多轴联动命令
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
	{//单轴运动
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
	{//直线插补，只有一个目标点
		m_InterpLenAxis = 0;
		for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
		{//确定长轴
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
		
		AutoActionMove(procNum);										//开始运动
	}
	
	return 0;
}

/**************************************************************************************************
**  函数名：  AutoActionStepList()
**	输入参数：需要运行的行号
**	输出参数：关联运行指令的行数
**	函数功能：判断关联的指令总行数
**	备注：	  
**  作者：       
**  开发日期:
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
			{//搜索完成
				break;
			}
			
			if(result == LISTNUM)	//并行运行的指令行数不能超过LISTNUM行
			{
				break;
			}
		}
	}
	
	return result;	
}

/**************************************************************************************************
**  函数名：  AutoActionOutControl()
**	输入参数：需要运行的行号
**	输出参数：动作是否成功输出
**	函数功能：主程序自动模式控制函数
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
u8 AutoActionOutControl(SaveProgram *Program_Operate, u8 ActionLine)
{
	u8 result = 0;
	u8 j = 0;
	u16 i = 0;
	u8 SubProgramNum = 0;
	u8 SubProgram_Sequence = 0;
	u8 IIC_Parameter[100] = {0};									//数据读取时使用的中间变量
	u32 pointTemp = 0;
	u8 axsisNum = 0;
	u8 keepMoveIoSta_Low[Axis_Num] = {0};					//存放搜索命令对应输入口的状态-低
	u8 keepMoveIoSta_High[Axis_Num] = {0};				//存放搜索命令对应输入口的状态-高
	s32 Increment_Distance = 0;
	u8 ret = 0;
	ST_MDPostion sPostion = {0};
	u8 ionum = 0;
	u8 index = 0;
	u8 offset = 0;
	
	result = Action_Step_List_Num;
	if(ActionLine < Program_Operate->Num)
	{//判断待执行的行号是否正确
		if(ActionAllowJudge(Program_Operate ,ActionLine) == 0)	
		{//动作执行前进行动作安全合法性判断
			switch(Program_Operate->Program[ActionLine].Order)
			{
			  case OR_BASICORDER://基本指令
					switch(Program_Operate->Program[ActionLine].Key)
					{
						case K_PROGRAMSTART://程序开始
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
						case K_PROGRAMEND://程序结束
							result = 9;
							if(Program_Operate->Code == SAVEPROGRAMNUM_MAIN && Robot_Auto_Reset == FALSE)  //复位程序
							{	
								Program_Reset = TRUE;
								Robot_Auto_Reset = TRUE;
								g_Auto_Reset_Flag = FALSE;
								if(Temp_OUT_Switch_Parameter[O_RESETING_LIGHT] == 1)
								{
									SetOutput(O_RESETING_LIGHT);
								}
								CurProgramRead(g_Run_Program_Num_Pre);							//选中程序更换为原有选中的程序								
							}
							else
							{
								Program_Reset = FALSE;
								Program_RunTime = Program_RunTime_Count / 10;
								m_ProRunTimeTotal = m_ProRunTimeTotalCount / 10;
								m_ProRunTimeCumulate = m_PreProRunTimeCumulate + m_ProRunTimeTotal;	
								if(Auto_Mode == ONCE_MODE) //单次模式-进入停止状态
								{
									g_Auto_Order_Stop = TRUE;	
								}
							}
							break;
						case K_DELAY://延时
							g_Key_Delay_Timer =0;
							g_Key_Delay_Flag = TRUE;
							break;
						case K_SUBPROGRAM://子程序  开始-结束
							if(g_Run_Program_Num > SAVEPROGRAMNUM_MAIN && Program_Operate->Program[ActionLine].Value1 == g_Run_Program_Num - SAVEPROGRAMNUM_MAIN)
							{//支持子程序独立运行，当前执行程序为子程序时，把子程序的开始和结束作为主程序的开始结束
								if(Program_Operate->Program[ActionLine].Value2 == V_PROGRAM_START)
								{//子程序开始
//									Program_RunTime_Count = 0;
								}
								else 
								{//子程序结束
									result = 9;
									Program_RunTime = Program_RunTime_Count / 10;
									m_ProRunTimeTotal = m_ProRunTimeTotalCount / 10;
								  m_ProRunTimeCumulate = m_PreProRunTimeCumulate + m_ProRunTimeTotal;	
								}
							}
							else if(g_Program_Is_Debuging == FALSE)
							{//调试时不允许运行子程序命令
								SubProgramNum = Program_Operate->Program[ActionLine].Value1 - 1;//读取子程序编号
								SubProgram_Sequence = SubProgramNum + SAVEPROGRAMNUM_MAIN;

								if(Program_Operate->Program[ActionLine].Value2 == V_PROGRAM_START && SubProgramNum < SAVEPROGRAMNUM_SUB)
								{//子程序开始
									if(g_Read_SubProgram[SubProgramNum] == FALSE && g_SubProgram_Start[SubProgramNum] == FALSE)
									{//读子程序
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
						case K_JUMP://跳转指令
							if(Program_Operate->Program[ActionLine].Value1 == V_JUMP_TO)//跳转
							{
								for(i=0;i<Program_Operate->Num;i++)
								{
									if((Program_Operate->Program[i].Value1 == V_JUMP_LABEL)&&(Program_Operate->Program[ActionLine].Value3 == Program_Operate->Program[i].Value3))//跳转
									{
										m_JumpStepRunNum = i;
									}
								}
								if(m_JumpStepRunNum == Program_Operate->Num)
								{//待跳转行号超出范围，运行异常
									result = 12;
								}
							}
							else if(Program_Operate->Program[ActionLine].Value1 == V_JUMP_LABEL)//标签
							{							
								break;						
							}		
							break;
						case K_WHILE://While指令
							m_WhileNC ++;
							if(m_WhileRunFlag[m_WhileNC - 1] == 0)
							{
								m_WhileRunFlag[m_WhileNC - 1] = 1;
								m_WhileLineNum[m_WhileNC - 1]= g_Auto_PresentLine;
								m_WhileCycCounter[m_WhileNC - 1] = 0;
								m_WhileJudgeType[m_WhileNC - 1] = Program_Operate->Program[ActionLine].Value1;
								
								if(Program_Operate->Program[ActionLine].Value1 == V_R_METHOD)
								{//R判断
									m_WhileJudgePar1[m_WhileNC - 1] = Program_Operate->Program[ActionLine].Value2;			//R判断时表示数值比较类型
									m_WhileJudgePar2[m_WhileNC - 1] = Program_Operate->Program[ActionLine].Value3;			//R判断时表示比较值
								}
								else if(Program_Operate->Program[ActionLine].Value1 == V_I_METHOD)
								{//I判断
									m_WhileJudgePar1[m_WhileNC - 1] = Program_Operate->Program[ActionLine].Value2;			//I判断时表示端口号
									m_WhileJudgePar2[m_WhileNC - 1] = Program_Operate->Program[ActionLine].Value3;			//I判断时表示比较成立电平，0无效电平，1有效电平
								}
								else if(V_USER1 <= Program_Operate->Program[ActionLine].Value1 && Program_Operate->Program[ActionLine].Value1 <= V_USER8)
								{//user1-user8
									if(V_ONLY_EQUAL <= Program_Operate->Program[ActionLine].Value2 && Program_Operate->Program[ActionLine].Value2 <= V_NOTONLY_EQUAL)
									{//判断输入端口数据合法性
										m_WhileJudgePar1[m_WhileNC - 1] = Program_Operate->Program[ActionLine].Value2;//判断
										m_WhileJudgePar2[m_WhileNC - 1] = Program_Operate->Program[ActionLine].Value3;//用户变量
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
						case K_CYCLEOVER://循环结束
							break;
						case K_IF://执行IF语句
						case K_ELSE://执行ELSE语句
							if(Program_Operate->Program[ActionLine].Key == K_IF)
							{
								m_IfElseNC++;
								m_IfElseJudgeRunFlag[m_IfElseNC - 1] = 0;																								//每次执行IF命令时要把是否已执行标志清零
							}
							m_IfElseJudgeType[m_IfElseNC - 1] = Program_Operate->Program[ActionLine].Value1;
							m_IfElseJudgePar1[m_IfElseNC - 1] = Program_Operate->Program[ActionLine].Value2;
							m_IfElseJudgePar2[m_IfElseNC - 1] = Program_Operate->Program[ActionLine].Value3;
							break;
						case K_OVER://跳转结束
							break;
						case K_SPECIAL:
							if(Program_Operate->Program[ActionLine].Value1 == V_SUSPEND) 		//暂停
							{}
							else if(Program_Operate->Program[ActionLine].Value1 == V_STOP)
							{}
							break;
						case K_PULSE_OUTPUT://脉宽输出
							if(Program_Operate->Program[ActionLine].Value1 < OUTPUT_NUM)
							{//判断输入端口数据合法性
								if(Program_Operate->Program[ActionLine].Value2 == V_SET)
								{
									SetSingle(60,Program_Operate->Program[ActionLine].Value1,0);							//置位端口
									m_PulseOutputSta[Program_Operate->Program[ActionLine].Value1] = V_SET;
								}
								else
								{
									SetSingle(Program_Operate->Program[ActionLine].Value1,60,0);							//复位端口
									m_PulseOutputSta[Program_Operate->Program[ActionLine].Value1] = V_RESET;
								}
								m_PulseOutputStartTime[Program_Operate->Program[ActionLine].Value1] = m_SystemTimeCounter;
								m_PulseOutputEndTime[Program_Operate->Program[ActionLine].Value1] = (m_SystemTimeCounter + Program_Operate->Program[ActionLine].Value3) % SYSTEM_TIME_MAX;
							}
							break;						
						case K_USER://用户变量
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
						case K_OUTDETECT://输出检测
							break;
						default://基本指令类型异常
							result = 12;	
							break;
					}
					break;
				case OR_AXISORDER://轴控指令
					switch(Program_Operate->Program[ActionLine].Key)
					{
						case K_MDPOSITION://码垛位置
							ret = MD_ReadParData();
							if(ret == 1)
							{
								result = 13;
							}
							else
							{
								if(Program_Operate->Program[ActionLine].Value1 >= V_MD_AXSIS \
										&& Program_Operate->Program[ActionLine].Value1 < V_MD_AXSIS + Axis_Num)
								{//码垛位置
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
								{//等待点位置
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
								{//O轴为气缸
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
									{//该轴不参与插补
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
						case K_MDPOCOUNT://码垛计数
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
						case K_XAXIS://轴位置移动: 位置 , 速度
						case K_ZAXIS:
						case K_LAXIS:
						case K_OAXIS:
							if(Program_Operate->Program[ActionLine].Key == K_XAXIS) 
							{//X轴
								axsisNum = X_Axsis;
								pointTemp = Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Point_X + MINROBOTPOSITION;
							}
							else if(Program_Operate->Program[ActionLine].Key == K_ZAXIS) 
							{//Z轴
								axsisNum = Z_Axsis;
								pointTemp = Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Point_Z + MINROBOTPOSITION;
							}
							else if(Program_Operate->Program[ActionLine].Key == K_LAXIS)
							{//Y轴
								axsisNum = L_Axsis;
								pointTemp = Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Point_L + MINROBOTPOSITION;
							}
							else if(Program_Operate->Program[ActionLine].Key == K_OAXIS)
							{//O轴
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
						case K_KEEP_MOVE://正向搜素
						case K_NEGTIVE_SEARCH://反向搜索
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
								{//下降沿，检测低
									if(keepMoveIoSta_Low[axsisNum] == 1)	
									{//一开始检测到低电平
										Flag_Falling_Edge = 1;
									}
									else if(keepMoveIoSta_High[axsisNum] == 1)
									{//一开始检测到高电平
										Flag_Falling_Edge = 2;
									}
									Flag_Keep_Move[axsisNum] = 1;
								}
								else if(Program_Operate->Program[ActionLine].Value3 == V_RISING_EDGE)
								{//上升沿，检测高
									if(keepMoveIoSta_Low[axsisNum] == 1)	
									{//一开始检测到低电平
										Flag_Rising_Edge = 2;
									}
									else if(keepMoveIoSta_High[axsisNum] == 1)
									{//一开始检测到高电平
										Flag_Rising_Edge = 1;
									}
									Flag_Keep_Move[axsisNum] = 1;
								}
								else if(Program_Operate->Program[ActionLine].Value3 == V_HIGH_LEVEL)
								{//检测高电平
									if(keepMoveIoSta_Low[axsisNum] == 1)	
									{//一开始检测到低电平
										Flag_Keep_Move[axsisNum] = 1;
									}
								}
								else if(Program_Operate->Program[ActionLine].Value3 == V_LOW_LEVEL)
								{//检测低电平
									if(keepMoveIoSta_High[axsisNum] == 1)	
									{//一开始检测到高电平
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
								{//没有执行完成时暂停再启动，不再计算
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
							SlowPointIncrement[axsisNum] = (Program_Operate->Program[ActionLine].Value3 & 0x0fffffff) * Step_Coefficient[axsisNum] / 100;			//减速度点增量位置
							SlowPointSpeed[axsisNum] = Program_Operate->Program[ActionLine].Value2;																							//减速度点速度
							SlowPointFlag[axsisNum] = 1;																																												//允许减速度点标志
							break;
						case K_INTER_START:
							break;
						case K_INTER_OVER:
							break;
						case K_ADVENCE:
							if(g_Program_Is_Debuging == FALSE && sCartesian_Para.MDCoordType != 2 && (Program_Operate->Program[ActionLine].Value3 & 0x0fffffff) > 10)
							{//SCARA模式下，提确认无效
								axsisNum = Program_Operate->Program[ActionLine].Value1 - V_KEEP_MOVE_X;
								AdvancePointInc[axsisNum] = (Program_Operate->Program[ActionLine].Value3 & 0x0fffffff) * Step_Coefficient[axsisNum] / 100;					//提前确认量
								AdvancePointFlag[axsisNum] = 1;																																											//提前确认量标志
							}
							break;
						case K_INTER_LINE:
							break;
						case K_AXISMOVE://轴移动
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
						default://轴控指令类型异常
							result = 13;	
							break;
					}
					break;
				case OR_IOORDER://IO指令
					switch(Program_Operate->Program[ActionLine].Key)
					{//全部改为直接输出不检测，一个输出对应两条指令：复位，置位
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

						//输入信号检测-22路
						case K_IOINSTRUCT_INPUT1://输入1-X0
						case K_IOINSTRUCT_INPUT2://输入2-X1
						case K_IOINSTRUCT_INPUT3://输入3-X2
						case K_IOINSTRUCT_INPUT4://输入4-X3
						case K_IOINSTRUCT_INPUT5://输入5-X4
						case K_IOINSTRUCT_INPUT6://输入6-X5
						case K_IOINSTRUCT_INPUT7://输入7-X6
						case K_IOINSTRUCT_INPUT8://输入8-X7
						case K_IOINSTRUCT_INPUT9://输入9-X8
						case K_IOINSTRUCT_INPUT10://输入10-X9
						case K_IOINSTRUCT_INPUT11://输入11-X10
						case K_IOINSTRUCT_INPUT12://输入12-X11
						case K_IOINSTRUCT_INPUT13://输入13-X12
						case K_IOINSTRUCT_INPUT14://输入14-X13
						case K_IOINSTRUCT_INPUT15://输入15-X14
						case K_IOINSTRUCT_INPUT16://输入16-X15
						case K_IOINSTRUCT_INPUT17://输入17-X16
						case K_IOINSTRUCT_INPUT18://输入18-X17
						case K_IOINSTRUCT_INPUT19://输入19-X18
						case K_IOINSTRUCT_INPUT20://输入20-X19
						case K_IOINSTRUCT_INPUT21://输入21-X20
						case K_IOINSTRUCT_INPUT22://输入22-X21
						case K_IOINSTRUCT_INPUT23://输入23-X22
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
								{//下降沿，检测低
									if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
									{//一开始检测到低电平
										Detect_Falling_Edge = 1;
									}
									else if(((Input_Detect_Status[index] >> offset) & 0x01) == 0)
									{//一开始检测到高电平
										Detect_Falling_Edge = 2;
									}
								}
								else if(Program_Operate->Program[ActionLine].Value2 == V_RISING_EDGE)
								{//上升沿，检测高
									if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
									{//一开始检测到低电平
										Detect_Rising_Edge = 2;
									}
									else if(((Input_Detect_Status[index] >> offset) & 0x01) == 0)	
									{//一开始检测到高电平
										Detect_Rising_Edge = 1;
									}
								}
							 break;
						default:
							 result = 14;	//IO控制指令类型异常
							 break;
					}
					break;
				default:
					result = 11;	//主要指令类型异常
					break;
			}//switch(order)
			
			if(result < 9)
			{
				Action_Step_Run_Num++;
				if(Action_Step_List_Num >= Action_Step_Run_Num)
				{
					result = Action_Step_List_Num - Action_Step_Run_Num;//单次运行指令数-已运行指令数，若=0则单次运行完成
					
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
		{//主要指令类型异常
			result = 11; 
		}
	}//else->if(Flag)
	else
	{//当前行号长度异常
		result = 10;	
	}
	return result;		
}

/**************************************************************************************************
**  函数名：  SubProgramActionOutControl()
**	输入参数：需要运行的行号
**	输出参数：
**	函数功能：子程序自动模式控制函数
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
u8 SubProgramActionOutControl(u8 subProNum, u8 ActionLine)
{
	u8 result = 0;
	u8 j = 0;
	u8 SubProgramNum = 0;
	u8 SubProgram_Sequence = 0;
	u8 IIC_Parameter[100] = {0};						//数据读取时使用的中间变量
	u32 pointTemp = 0;
	u8 axsisNum = 0;
	u8 keepMoveIoSta_Low[Axis_Num] = {0};					//存放搜索命令对应输入口的状态-低
	u8 keepMoveIoSta_High[Axis_Num] = {0};					//存放搜索命令对应输入口的状态-高
	s32 Increment_Distance = 0;
	u8 ret = 0;
	ST_MDPostion sPostion = {0};
	u8 ionum = 0;
	u8 index = 0;
	u8 offset = 0;

	if(ActionLine < SubProgram_Operate[subProNum].Num)
	{//确定程序行号是否在正常范围内
		if(SubActionAllowJudge(subProNum, ActionLine) == 0)
		{//动作执行前进行动作安全合法性判断
			switch(SubProgram_Operate[subProNum].Program[ActionLine].Order)
			{
				case OR_BASICORDER://基本指令
					switch(SubProgram_Operate[subProNum].Program[ActionLine].Key)
					{
						case K_PROGRAMSTART://程序开始
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
						case K_PROGRAMEND://程序结束
							result = 9;
							break;
						case K_DELAY://延时
							g_SubProgram_Key_Delay_Timer[subProNum] = 0;
							g_SubProgram_Key_Delay_Flag[subProNum] = TRUE;
							break;
						case K_SUBPROGRAM://子程序  开始-结束
							if(g_Program_Is_Debuging == FALSE)
							{//调试时不允许运行子程序命令
								SubProgramNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1;//读取子程序编号
								SubProgram_Sequence = SubProgramNum + SAVEPROGRAMNUM_MAIN;

								if((SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_PROGRAM_START) && (SubProgramNum < SAVEPROGRAMNUM_SUB))
								{//子程序开始
									if(g_Read_SubProgram[SubProgramNum] == FALSE  && g_SubProgram_Start[SubProgramNum] == FALSE)
									{//读子程序
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
						case K_JUMP://跳转指令
							if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_JUMP_TO)//跳转
							{
								for(j=0;j<SubProgram_Operate[subProNum].Num;j++)
								{
									if((SubProgram_Operate[subProNum].Program[j].Value1 == V_JUMP_LABEL)&&(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == SubProgram_Operate[subProNum].Program[j].Value3))//跳转
									{
										m_JumpSubStepRunNum[subProNum] = j;
									}
								}
								if(m_JumpSubStepRunNum[subProNum] == SubProgram_Operate[subProNum].Num)
								{//待跳转行号超出范围，运行异常
									result = 12;
								}
							}
							else if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_JUMP_LABEL)//标签
							{							
								break;						
							}	
							break;
						case K_WHILE://While指令
							m_WhileSubNC[subProNum]++;
							if(m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] == 0)
							{
								m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
								m_WhileSubLineNum[subProNum][m_WhileSubNC[subProNum] - 1]= g_SubProgram_PresentLine[subProNum];
								m_WhileSubCycCounter[subProNum][m_WhileSubNC[subProNum] - 1] = 0;
								m_WhileSubJudgeType[subProNum][m_WhileSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value1;
								
								if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_R_METHOD)
								{//R判断
									m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value2;//R判断时表示数值比较类型
									m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value3;//R判断时表示比较值
								}
								else if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_I_METHOD)
								{//I判断
									m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value2;//I判断时表示端口号
									m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value3;//I判断时表示比较成立电平，0无效电平，1有效电平
								}
								else if(V_USER1 <= SubProgram_Operate[subProNum].Program[ActionLine].Value1 && SubProgram_Operate[subProNum].Program[ActionLine].Value1 <= V_USER8)
								{//user1-user8
									if(V_ONLY_EQUAL <= SubProgram_Operate[subProNum].Program[ActionLine].Value2 && SubProgram_Operate[subProNum].Program[ActionLine].Value2 <= V_NOTONLY_EQUAL)
									{//判断输入端口数据合法性
										m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value2;	//判断
										m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value3;	//用户变量
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
						case K_CYCLEOVER://循环结束
							break;
						case K_IF://执行IF语句
						case K_ELSE://执行ELSE语句
							if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_IF)
							{
								m_IfElseSubNC[subProNum]++;
								m_IfElseSubJudgeRunFlag[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;//每次执行IF命令时要把是否已执行标志清零
							}
							m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value1;
							m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value2;
							m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] = SubProgram_Operate[subProNum].Program[ActionLine].Value3;
							break;
						case K_OVER://跳转结束
							break;
						case K_SPECIAL:
							if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_SUSPEND) 		//暂停
							{}
							else if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_STOP)
							{}
							break;	
						case K_PULSE_OUTPUT://脉宽输出
							if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 < OUTPUT_NUM)
							{//判断输入端口数据合法性
								if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_SET)
								{
									SetSingle(60,SubProgram_Operate[subProNum].Program[ActionLine].Value1,0);							//置位端口
									m_PulseOutputSta[SubProgram_Operate[subProNum].Program[ActionLine].Value1] = V_SET;
								}
								else
								{
									SetSingle(SubProgram_Operate[subProNum].Program[ActionLine].Value1,60,0);							//复位端口
									m_PulseOutputSta[SubProgram_Operate[subProNum].Program[ActionLine].Value1] = V_RESET;
								}
								m_PulseOutputStartTime[SubProgram_Operate[subProNum].Program[ActionLine].Value1] = m_SystemTimeCounter;
								m_PulseOutputEndTime[SubProgram_Operate[subProNum].Program[ActionLine].Value1] = (m_SystemTimeCounter + SubProgram_Operate[subProNum].Program[ActionLine].Value3) % SYSTEM_TIME_MAX;
							}
							break;						
						case K_USER://用户变量
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
						case K_OUTDETECT://输出检测
							break;
						default://基本指令类型异常
							result = 12;	
							break;
					}
					break;
				case OR_AXISORDER://轴控指令
					switch(SubProgram_Operate[subProNum].Program[ActionLine].Key)
					{
						case K_MDPOSITION://码垛位置
							ret = MD_ReadParData();
							if(ret == 1)
							{
								result = 13;
							}
							else
							{
								if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 >= V_MD_AXSIS \
										&& SubProgram_Operate[subProNum].Program[ActionLine].Value1 < V_MD_AXSIS + Axis_Num)
								{//码垛位置
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
								{//等待点位置
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
								{//O轴为气缸
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
						case K_MDPOCOUNT://码垛计数
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
						case K_XAXIS://轴位置移动: 位置 , 速度
						case K_ZAXIS:
						case K_LAXIS:
						case K_OAXIS:
							if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_XAXIS) 
							{//X轴
								axsisNum = X_Axsis;
								pointTemp = Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Point_X + MINROBOTPOSITION;
							}
							else if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_ZAXIS) 
							{//Z轴
								axsisNum = Z_Axsis;
								pointTemp = Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Point_Z + MINROBOTPOSITION;
							}
							else if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_LAXIS)
							{//Y轴
								axsisNum = L_Axsis;
								pointTemp = Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Point_L + MINROBOTPOSITION;
							}
							else if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_OAXIS)
							{//O轴
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
						case K_KEEP_MOVE://正向搜素
						case K_NEGTIVE_SEARCH://反向搜索
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
								{//下降沿，检测低
									if(keepMoveIoSta_Low[axsisNum] == 1)
									{//一开始检测到低电平
										Flag_Falling_Edge_Sub = 1;
									}
									else if(keepMoveIoSta_High[axsisNum] == 1)
									{//一开始检测到高电平
										Flag_Falling_Edge_Sub = 2;
									}
									Flag_Keep_Move[axsisNum] = 1;
								}
								else if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_RISING_EDGE)
								{//上升沿，检测高
									if(keepMoveIoSta_Low[axsisNum] == 1)	
									{//一开始检测到低电平
										Flag_Rising_Edge_Sub = 2;
									}
									else if(keepMoveIoSta_High[axsisNum] == 1)
									{//一开始检测到高电平
										Flag_Rising_Edge_Sub = 1;
									}
									Flag_Keep_Move[axsisNum] = 1;
								}
								else if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_HIGH_LEVEL)
								{//检测高电平
									if(keepMoveIoSta_Low[axsisNum] == 1)	
									{//一开始检测到低电平
										Flag_Keep_Move[axsisNum] = 1;
									}
								}
								else if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_LOW_LEVEL)
								{//检测低电平
									if(keepMoveIoSta_High[axsisNum] == 1)	
									{//一开始检测到高电平
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
							SlowPointIncrement[axsisNum] = (SubProgram_Operate[subProNum].Program[ActionLine].Value3 & 0x0fffffff) * Step_Coefficient[axsisNum] / 100;			//减速度点增量位置
							SlowPointSpeed[axsisNum] = SubProgram_Operate[subProNum].Program[ActionLine].Value2;																						//减速度点速度
							SlowPointFlag[axsisNum] = 1;																																																		//允许减速度点标志
							break;
						case K_INTER_START:
							break;
						case K_INTER_OVER:
							break;
						case K_ADVENCE:
							if(g_Program_Is_Debuging == FALSE && sCartesian_Para.MDCoordType != 2 && (SubProgram_Operate[subProNum].Program[ActionLine].Value3 & 0x0fffffff) > 10)
							{//SCARA模式下，提确认无效
								axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1 - V_KEEP_MOVE_X;
								AdvancePointInc[axsisNum] = (SubProgram_Operate[subProNum].Program[ActionLine].Value3 & 0x0fffffff) * Step_Coefficient[axsisNum] / 100;					//提前确认量
								AdvancePointFlag[axsisNum] = 1;																																											//提前确认量标志
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
							result = 13;	//轴控指令类型异常
							break;
					}
					break;
			  case OR_IOORDER://IO指令
					switch(SubProgram_Operate[subProNum].Program[ActionLine].Key)
					{//全部改为直接输出不检测，一个输出对应两条指令：复位，置位
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

						//输入信号检测
						case K_IOINSTRUCT_INPUT1:	  	//输入1-X0
						case K_IOINSTRUCT_INPUT2:	  	//输入2-X1
						case K_IOINSTRUCT_INPUT3:	  	//输入3-X2
						case K_IOINSTRUCT_INPUT4:	  	//输入4-X3
						case K_IOINSTRUCT_INPUT5:	  	//输入5-X4							 
						case K_IOINSTRUCT_INPUT6:	  	//输入6-X5
						case K_IOINSTRUCT_INPUT7:	  	//输入7-X6
						case K_IOINSTRUCT_INPUT8:	  	//输入8-X7
						case K_IOINSTRUCT_INPUT9:	  	//输入9-X8
						case K_IOINSTRUCT_INPUT10:	  //输入10-X9
						case K_IOINSTRUCT_INPUT11:	  //输入11-X10
						case K_IOINSTRUCT_INPUT12:	  //输入12-X11
						case K_IOINSTRUCT_INPUT13:	  //输入13-X12
						case K_IOINSTRUCT_INPUT14:	  //输入14-X13
						case K_IOINSTRUCT_INPUT15:	  //输入15-X14
						case K_IOINSTRUCT_INPUT16:	  //输入16-X15
						case K_IOINSTRUCT_INPUT17:	  //输入17-X16
						case K_IOINSTRUCT_INPUT18:	  //输入18-X17
						case K_IOINSTRUCT_INPUT19:	  //输入19-X18
						case K_IOINSTRUCT_INPUT20:	  //输入20-X19
						case K_IOINSTRUCT_INPUT21:	  //输入21-X20
						case K_IOINSTRUCT_INPUT22:	  //输入22-X21
						case K_IOINSTRUCT_INPUT23:    //输入23-X22
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
								{//下降沿，检测低
									if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
									{//一开始检测到低电平
										Detect_Falling_Edge_Sub[subProNum] = 1;
									}
									else if(((Input_Detect_Status[index] >> offset) & 0x01) == 0)
									{//一开始检测到高电平
										Detect_Falling_Edge_Sub[subProNum] = 2;
									}
								}
								else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_RISING_EDGE)
								{//上升沿，检测高
									if(((Input_Detect_Status[index] >> offset) & 0x01) == 0x01)
									{//一开始检测到低电平
										Detect_Rising_Edge_Sub[subProNum] = 2;
									}
									else if(((Input_Detect_Status[index] >> offset) & 0x01) == 0)	
									{//一开始检测到高电平
										Detect_Rising_Edge_Sub[subProNum] = 1;
									}
								}
							break;
						default://IO指令类型异常
							result = 14;	
							break;
					}
					break;
				default://主要指令类型异常
					result = 11;
					break;
			}//switch(order)
			
			if(result < 9)
			{
				SubAction_Step_Run_Num[subProNum]++;
				if(SubAction_Step_List_Num[subProNum] >= SubAction_Step_Run_Num[subProNum])
				{
					result = SubAction_Step_List_Num[subProNum] - SubAction_Step_Run_Num[subProNum];//单次运行指令数-已运行指令数，若=0则单次运行完成
				}
				else
				{
					result = 11;
				}
			}
			
		}//if(ActionAllowJudge(SubProgram_Operate[subProNum], ActionLine) == 0)
		else
		{
			result = 11; //主要指令类型异常
		}
	}//if(ActionLine < SubProgram_Operate[subProNum].Num)
	else
	{
		result = 10;				//当前行号长度异常
	}
	return result;		
}


/**************************************************************************************************
**  函数名：  AutoActionOutConfirm()
**	输入参数：需要运行的行号
**	输出参数：动作是否已确认
**	函数功能：自动模式控制函数
**	备注：    控制设备自动运行
**  作者：    
**  开发日期：
***************************************************************************************************/
u8 AutoActionOutConfirm(SaveProgram *Program_Operate, u8 ActionLine)
{
//	u8 Temp_data[8]={0};
	u8 result = 0;
	u8 SubProgramNum = 0;
	u32 pointTemp = 0;
	u8 axsisNum = 0;
	u8 keepMoveIoSta_Low[Axis_Num] = {0};					//存放搜索命令对应输入口的状态-低
	u8 keepMoveIoSta_High[Axis_Num] = {0};					//存放搜索命令对应输入口的状态-高
	u8 ionum = 0;
	u8 index = 0;
	u8 offset = 0;
	u8 signValue = 0;
	u8 axisNum = 0;

	switch(Program_Operate->Program[ActionLine].Key)
	{
		case K_PROGRAMSTART://程序开始
			result = 1;
			break;
		case K_PROGRAMEND://程序结束
			 result = 1;
			break;
		case K_DELAY://延时是否完成
			 if(Key_Delay(Program_Operate->Program[ActionLine].Value2))
			 {
					result = 1;
			 }
			break;
		case K_SUBPROGRAM://子程序  开始-结束		     
			 if(g_Program_Is_Debuging == FALSE)
			 {
				 SubProgramNum = Program_Operate->Program[ActionLine].Value1 - 1;//读取子程序编号
				 if(Program_Operate->Program[ActionLine].Value2 == V_PROGRAM_START)
				 {//子程序开始
						 result = 1;
				 }
				 else
				 {//子程序结束
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
		case K_JUMP://跳转指令
			result = 1;
			break;
		case K_WHILE://WHILE命令
			m_WhileJudgeRes[m_WhileNC - 1] = 0;									//默认设置成条件不成立
			if(m_WhileRunFlag[m_WhileNC - 1] == 1)
			{
				if(m_WhileJudgeType[m_WhileNC - 1] == V_R_METHOD)
				{//R判断
					if(m_WhileCycCounter[m_WhileNC - 1] < m_WhileJudgePar2[m_WhileNC - 1])
					{//判断条件成立
						m_WhileJudgeRes[m_WhileNC - 1] = 1;
					}
				}
				else if(m_WhileJudgeType[m_WhileNC - 1] == V_I_METHOD)
				{//I判断
					if(m_WhileJudgePar1[m_WhileNC - 1] < INPUT_NUM)
					{
						if(m_WhileJudgePar2[m_WhileNC - 1] == V_LOW_LEVEL && ReadInput(m_WhileJudgePar1[m_WhileNC - 1]) == 0)
						{//检测有效电平
							m_WhileJudgeRes[m_WhileNC - 1] = 1;
						}
						else if(m_WhileJudgePar2[m_WhileNC - 1] == V_HIGH_LEVEL && ReadInput(m_WhileJudgePar1[m_WhileNC - 1]) == 1)
						{//检测无效电平
							m_WhileJudgeRes[m_WhileNC - 1] = 1;
						}
					}
				}
				else if(V_USER1<=m_WhileJudgeType[m_WhileNC - 1] && m_WhileJudgeType[m_WhileNC - 1]<=V_USER8)
				{//user1-user8
					if((USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] != 0) && (m_WhileJudgePar1[m_WhileNC - 1] == V_EQUAL)\
						&& (USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] % m_WhileJudgePar2[m_WhileNC - 1]) == 0)	
					{//user当前值 = R数量的倍数
						m_WhileJudgeRes[m_WhileNC - 1] = 1;
					}
					else if((m_WhileJudgePar1[m_WhileNC - 1] == V_ONLY_EQUAL) && (USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] == m_WhileJudgePar2[m_WhileNC - 1]))	
					{//user当前值 = R数量
						m_WhileJudgeRes[m_WhileNC - 1] = 1;
					}	
					else if((m_WhileJudgePar1[m_WhileNC - 1] == V_NOT_EQUAL) && (USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] != m_WhileJudgePar2[m_WhileNC - 1]))	
					{//user当前值 != R数量
						m_WhileJudgeRes[m_WhileNC - 1] = 1;
					}
					else if((m_WhileJudgePar1[m_WhileNC - 1] == V_NOTONLY_EQUAL) && (USER_Parameter.CURR_Num[Program_Operate->Program[ActionLine].Value1-V_USER1] % m_WhileJudgePar2[m_WhileNC - 1]) != 0)	
					{//user当前值 != R数量的倍数时
						m_WhileJudgeRes[m_WhileNC - 1] = 1;
					}
				}
			}
			result = 1;
			break;
		case K_CYCLEOVER://循环结束
			if(m_WhileRunFlag[m_WhileNC - 1] == 1)
			{
				if(m_WhileJudgeType[m_WhileNC - 1] == V_R_METHOD)
				{//R判断
					m_WhileCycCounter[m_WhileNC - 1]++;
				}
			}
			result = 1;
			break;
		case K_IF://IF判断条件
		case K_ELSE://else判断条件命令，两者处理方式相同
			m_IfElseJudgeRes[m_IfElseNC - 1] = 0;																//默认判断条件失败
			if((V_USER1<=m_IfElseJudgeType[m_IfElseNC - 1])&&(m_IfElseJudgeType[m_IfElseNC - 1]<= V_USER8))
			{//用户变量判断
				if((USER_Parameter.CURR_Num[m_IfElseJudgeType[m_IfElseNC - 1]-V_USER1] != 0) && (m_IfElseJudgePar1[m_IfElseNC - 1] == V_EQUAL)\
						&& (USER_Parameter.CURR_Num[m_IfElseJudgeType[m_IfElseNC - 1]-V_USER1] % m_IfElseJudgePar2[m_IfElseNC - 1]) == 0)	
				{//user当前值 = R数量的倍数
					m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
				}
				else if((m_IfElseJudgePar1[m_IfElseNC - 1] == V_ONLY_EQUAL) && (USER_Parameter.CURR_Num[m_IfElseJudgeType[m_IfElseNC - 1]-V_USER1] == m_IfElseJudgePar2[m_IfElseNC - 1]))	
				{//user当前值 = R数量
					m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
				}
				else if((m_IfElseJudgePar1[m_IfElseNC - 1] == V_NOT_EQUAL) && (USER_Parameter.CURR_Num[m_IfElseJudgeType[m_IfElseNC - 1]-V_USER1] != m_IfElseJudgePar2[m_IfElseNC - 1]))	
				{//user当前值 != R数量
					m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
				}
				else if((m_IfElseJudgePar1[m_IfElseNC - 1] == V_NOTONLY_EQUAL) && (USER_Parameter.CURR_Num[m_IfElseJudgeType[m_IfElseNC - 1]-V_USER1] % m_IfElseJudgePar2[m_IfElseNC - 1]) != 0)	
				{//user当前值 != R数量的倍数时
					m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
				}
			}
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_I_METHOD)
			{//I判断
				if(m_IfElseJudgePar1[m_IfElseNC - 1] < INPUT_NUM)
				{
					if(m_IfElseJudgePar2[m_IfElseNC - 1] == V_LOW_LEVEL && ReadInput(m_IfElseJudgePar1[m_IfElseNC - 1]) == 0)
					{//检测有效电平
						m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
					}
					else if(m_IfElseJudgePar2[m_IfElseNC - 1] == V_HIGH_LEVEL && ReadInput(m_IfElseJudgePar1[m_IfElseNC - 1]) == 1)
					{//检测无效电平
						m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
					}
				}
			}
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_O_METHOD)
			{//O判断
				if(m_IfElseJudgePar1[m_IfElseNC - 1] < OUTPUT_NUM)
				{
					ionum = m_IfElseJudgePar1[m_IfElseNC - 1];
					index = ionum / 8;
					offset = ionum % 8;
					if(m_IfElseJudgePar2[m_IfElseNC - 1] == V_RESET && ((Output_Status[index] >> offset) & 0x01) == 0x01)
					{//检测复位
						m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
					}
					else if(m_IfElseJudgePar2[m_IfElseNC - 1] == V_SET && ((Output_Status[index] >> offset) & 0x01) == 0)
					{//检测置位
						m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
					}
				}
			}
			
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_P_METHOD)
			{//P条件
				if(m_IfElseJudgePar1[m_IfElseNC - 1] >= V_XAXISGREATER && \
					m_IfElseJudgePar1[m_IfElseNC - 1] <= V_OAXISGREATER)
				{//大于等于
					axisNum = m_IfElseJudgePar1[m_IfElseNC - 1] - V_XAXISGREATER;
					pointTemp = m_IfElseJudgePar2[m_IfElseNC - 1] * Step_Coefficient[axisNum] / 100 + MINROBOTPOSITION;
					if(m_PulseTotalCounter[axisNum] + JDZ_AllowError >= pointTemp)
					{//判断条件成立
						m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
					}
				}
				else if(m_IfElseJudgePar1[m_IfElseNC - 1] >= V_XAXISEQUAL && \
					m_IfElseJudgePar1[m_IfElseNC - 1] <= V_OAXISEQUAL)
				{//等于
					axisNum = m_IfElseJudgePar1[m_IfElseNC - 1] - V_XAXISEQUAL;
					pointTemp = m_IfElseJudgePar2[m_IfElseNC - 1] * Step_Coefficient[axisNum] / 100 + MINROBOTPOSITION;
					if(m_PulseTotalCounter[axisNum] + JDZ_AllowError >= pointTemp && m_PulseTotalCounter[axisNum] <= pointTemp + JDZ_AllowError)
					{//判断条件成立
						m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
					}
				}
				else if(m_IfElseJudgePar1[m_IfElseNC - 1] >= V_XAXISLESS && \
					m_IfElseJudgePar1[m_IfElseNC - 1] <= V_OAXISLESS)
				{//小于等于
					axisNum = m_IfElseJudgePar1[m_IfElseNC - 1] - V_XAXISLESS;
					pointTemp = m_IfElseJudgePar2[m_IfElseNC - 1] * Step_Coefficient[axisNum] / 100 + MINROBOTPOSITION;
					if(m_PulseTotalCounter[axisNum] <= pointTemp + JDZ_AllowError)
					{//判断条件成立
						m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
					}
				}
			}
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_LAYER_NUM)
			{//层数判断
				m_IfElseJudgeRes[m_IfElseNC - 1] = MD_LayerNumJudge(m_IfElseJudgePar1[m_IfElseNC - 1], m_IfElseJudgePar2[m_IfElseNC - 1]);
			}
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_LAYER_FULL)
			{//层满判断
				m_IfElseJudgeRes[m_IfElseNC - 1] = MD_LayerFullJudge(m_IfElseJudgePar1[m_IfElseNC - 1], m_IfElseJudgePar2[m_IfElseNC - 1]);
			}
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_STACK_FULL)
			{//垛满判断
				m_IfElseJudgeRes[m_IfElseNC - 1] = MD_StackFullJudge(m_IfElseJudgePar1[m_IfElseNC - 1], m_IfElseJudgePar2[m_IfElseNC - 1]);
			}
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_MDGOOD)
			{//物品判断
				m_IfElseJudgeRes[m_IfElseNC - 1] = MD_GoodNumJudge(m_IfElseJudgePar1[m_IfElseNC - 1], m_IfElseJudgePar2[m_IfElseNC - 1]);
			}
			else if(m_IfElseJudgeType[m_IfElseNC - 1] == V_RI_NULL)
			{//NULL跳转
				m_IfElseJudgeRes[m_IfElseNC - 1] = 1;
			}
			result = 1;
			break;
		case K_OVER:
			result = 1;
			break;
		case K_SPECIAL://特殊指令							
			if(Program_Operate->Program[ActionLine].Value1 == V_SUSPEND)
			{
				result = 1;
			}
			else if(Program_Operate->Program[ActionLine].Value1 == V_STOP)
			{
				result = 1;
			}
			break;	
		case K_PULSE_OUTPUT:		       //脉宽输出
			result = 1;	
			break;
		case K_USER:		              //用户变量		
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
		case K_OUTDETECT://输出检测
			if(Program_Operate->Program[ActionLine].Value1 < OUTPUT_NUM)
			{
				ionum = Program_Operate->Program[ActionLine].Value1;
//				if(ionum>=24)ionum += 4;//RY0-RY1
				index = ionum / 8;
				offset = ionum % 8;
				if(Program_Operate->Program[ActionLine].Value2 == V_RESET && ((Output_Status[index] >> offset) & 0x01) == 0x01)
				{//复位
					result = 1;
				}
				else if(Program_Operate->Program[ActionLine].Value2 == V_SET && ((Output_Status[index] >> offset) & 0x01) == 0)
				{//置位
					result = 1;
				}
			}
			break;
 		case K_MDPOSITION://确认码垛命令是否到位
			if(Program_Operate->Program[ActionLine].Value1 >= V_MD_AXSIS \
					&& Program_Operate->Program[ActionLine].Value1 < V_MD_AXSIS + Axis_Num)
			{//码垛位置
				axsisNum = Program_Operate->Program[ActionLine].Value1 - V_MD_AXSIS;
			}
			else if(Program_Operate->Program[ActionLine].Value1 >= V_MD_AXSIS + Axis_Num \
					&& Program_Operate->Program[ActionLine].Value1 < V_MD_AXSIS + Axis_Num * 2)
			{//等待点位置
				axsisNum = Program_Operate->Program[ActionLine].Value1 - V_MD_AXSIS - Axis_Num;
			}
			
			if(axsisNum == O_Axsis && sMD_Parameter.revolveMode == 1)
			{//O轴为气缸
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
		case K_MDPOCOUNT://码垛计数
			result = 1;	
			break;
		case K_XAXIS://轴是否移动到目标位置
		case K_ZAXIS:
		case K_LAXIS:
		case K_OAXIS:
			if(Program_Operate->Program[ActionLine].Key == K_XAXIS) 
			{//X轴
				axsisNum = X_Axsis;
				pointTemp = Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Point_X + MINROBOTPOSITION;
			}
			else if(Program_Operate->Program[ActionLine].Key == K_ZAXIS) 
			{//Z轴
				axsisNum = Z_Axsis;
				pointTemp = Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Point_Z + MINROBOTPOSITION;
			}
			else if(Program_Operate->Program[ActionLine].Key == K_LAXIS)
			{//Y轴
				axsisNum = L_Axsis;
				pointTemp = Manul_Save_Point[Program_Operate->Program[ActionLine].Value1 - 1].Point_L + MINROBOTPOSITION;
			}
			else if(Program_Operate->Program[ActionLine].Key == K_OAXIS)
			{//O轴
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
		case K_KEEP_MOVE://正向搜素
		case K_NEGTIVE_SEARCH://反向搜索
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
				{//低电平
					if(keepMoveIoSta_Low[axsisNum])
					{//收到低电平信号
						Servo_Stop(axsisNum);
						Flag_Keep_Move[axsisNum] = 2;
					}
				}
				else if(Program_Operate->Program[ActionLine].Value3 == V_HIGH_LEVEL)
				{//高电平
					if(keepMoveIoSta_High[axsisNum])	
					{//收到高电平信号
						Servo_Stop(axsisNum);
							Flag_Keep_Move[axsisNum] = 2;
					}
				}
				else if(Program_Operate->Program[ActionLine].Value3 == V_FALLING_EDGE)
				{//下降沿，检测低
					if(Flag_Falling_Edge == 1)
					{//一开始检测到低电平
						if(keepMoveIoSta_High[axsisNum])
						{//检测到高电平
							Flag_Falling_Edge = 3;
						}
					}
					if(Flag_Falling_Edge == 2 || Flag_Falling_Edge == 3)
					{//一开始检测到高电平,或者再次检测到高
						if(keepMoveIoSta_Low[axsisNum])
						{//检测到低电平,停止
							Servo_Stop(axsisNum);
							Flag_Keep_Move[axsisNum] = 2;
						}
					}
				}
				else if(Program_Operate->Program[ActionLine].Value3 == V_RISING_EDGE)
				{//上升沿，检测高
					if(Flag_Rising_Edge == 1)
					{//一开始检测到高电平
						if(keepMoveIoSta_Low[axsisNum])
						{//检测到低电平
							Flag_Rising_Edge = 3;
						}
					}
					if(Flag_Rising_Edge == 2 || Flag_Rising_Edge == 3)
					{//一开始检测到低电平,或者再次检测到低
						if(keepMoveIoSta_High[axsisNum])
						{//检测到高电平,停止
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
		case K_INCREMENT_RUNNING://是否移动到目标位置
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
		
		//输出接口-无检测
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

		//输入接口检测
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
			{//检测下降沿 与保持时间无关
				if(signValue == 1)
				{//出现有效信号
					result = 1;
					Detect_Falling_Edge = 0;
				}
			}
			else if(Program_Operate->Program[ActionLine].Value2 == V_RISING_EDGE)
			{//检测上升沿 与保持时间无关
				if(signValue == 1)
				{//出现有效信号
					result = 1;
					Detect_Rising_Edge = 0;
				}
			}
			else
			{
				if(signValue == 1)
				{//出现有效信号
					if(IO_Input_keepMin[ionum] >= IO_Input_keepMax[ionum])
					{//有效时间的起始时间等于终止时间，认为有效时间为0，一旦出现有效信号则动作确认成功
						result = 1;		
					}
					
					if(g_Auto_Valid_Timer > IO_Input_keepMax[ionum])
					{//有效信号时间超过最大保持时间,关闭计时器，不把计时器清零，因为下次出现有效信号时，从0开始计时
						g_Auto_Valid_Timer = 0;
						g_Auto_Valid_Flag = FALSE;
					}
					
					if(g_Auto_Valid_Flag == FALSE)
					{//开始计数
						g_Auto_Valid_Timer = 0;
						g_Auto_Valid_Flag = TRUE;
					}
				}
				else
				{//否则复位计数器，从0开始
					if(g_Auto_Valid_Timer > IO_Input_keepMin[ionum] && g_Auto_Valid_Timer < IO_Input_keepMax[ionum])
					{//有效时间计数大于最小时间，小于最大时间
						result = 1;
					}
					g_Auto_Valid_Timer = 0;
					g_Auto_Valid_Flag = FALSE;
				}
			}
			break;		
		default://指令类型异常
			 result = 14;	
			 break;
	}
	
	return result;		
}


/**************************************************************************************************
**  函数名：  SubProgramActionOutConfirm()
**	输入参数：需要运行的行号
**	输出参数：动作是否已确认
**	函数功能：自动模式控制函数-动作确认 0未完整 1完成 其他值为异常
**	备注：    
**  作者：    
**  开发日期：
***************************************************************************************************/
u8 SubProgramActionOutConfirm(u8 subProNum, u8 ActionLine)
{
//	u8 Temp_data[8]={0};
	u8 result = 0;
	u8 SubProgramNum = 0;
	u32 pointTemp = 0;
	u8 axsisNum = 0;
	u8 keepMoveIoSta_Low[Axis_Num] = {0};					//存放搜索命令对应输入口的状态-低
	u8 keepMoveIoSta_High[Axis_Num] = {0};					//存放搜索命令对应输入口的状态-高
	u8 ionum = 0;
	u8 index = 0;
	u8 offset = 0;
	u8 signValue = 0;
	u8 axisNum = 0;
	
	switch(SubProgram_Operate[subProNum].Program[ActionLine].Key)
	{
		case K_PROGRAMSTART://主程序开始
			result = 1;
			break;
		case K_PROGRAMEND://主程序结束
			result = 1;
			break;
		case K_DELAY://延时是否完成
			if(SubProgram_Key_Delay(subProNum, SubProgram_Operate[subProNum].Program[ActionLine].Value2))
			{
				result = 1;
			}
			break;
		case K_SUBPROGRAM://子程序  开始-结束		     
			if(g_Program_Is_Debuging == FALSE)
			{
				SubProgramNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1;//读取子程序编号
				if(SubProgramNum == subProNum)
				{//子程序开始
					result = 1;
				}
				else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_PROGRAM_START)
				{//子程序开始
					result = 1;
				}
				else
				{//子程序结束
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
		case K_JUMP://跳转指令
			result = 1;
			break;
		case K_WHILE:
			m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 0;									//默认设置成条件不成立
			if(m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] == 1)
			{
				if(m_WhileSubJudgeType[subProNum][m_WhileSubNC[subProNum] - 1] == V_R_METHOD)
				{//R判断
					if(m_WhileSubCycCounter[subProNum][m_WhileSubNC[subProNum] - 1] < m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1])
					{//判断条件成立
						m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
					}
				}
				else if(m_WhileSubJudgeType[subProNum][m_WhileSubNC[subProNum] - 1] == V_I_METHOD)
				{//I判断
					if(m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1] < INPUT_NUM)
					{
						if(m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1] == V_LOW_LEVEL && \
								ReadInput(m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1]) == 0)
						{//检测有效电平
							m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
						}
						else if(m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1] == V_HIGH_LEVEL && \
											ReadInput(m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1]) == 1)
						{//检测无效电平
							m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
						}
					}
				}
				else if(V_USER1<=m_WhileSubJudgeType[subProNum][m_WhileSubNC[subProNum] - 1] && m_WhileSubJudgeType[subProNum][m_WhileSubNC[subProNum] - 1]<=V_USER8)
				{//user1-user8
					if((USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1] != 0) && (m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1]== V_EQUAL)\
					&& (USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1] % m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1]) == 0)	
					{//user当前值 = R数量的倍数
						m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
					}
					else if((m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1] == V_ONLY_EQUAL) && (USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1]  == m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1]))
					{//user当前值 = R数量
						m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
					}
					else if((m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1] == V_NOT_EQUAL) && (USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1]  != m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1]))	
					{//user当前值 != R数量
						m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
					}
					else if((m_WhileSubJudgePar1[subProNum][m_WhileSubNC[subProNum] - 1] == V_NOTONLY_EQUAL) && (USER_Parameter.CURR_Num[SubProgram_Operate[subProNum].Program[ActionLine].Value1-V_USER1] % m_WhileSubJudgePar2[subProNum][m_WhileSubNC[subProNum] - 1]) != 0)	
					{//user当前值 != R数量的倍数时
						m_WhileSubJudgeRes[subProNum][m_WhileSubNC[subProNum] - 1] = 1;
					}	
				}
			}
			result = 1;
			break;
		case K_CYCLEOVER://循环结束
			if(m_WhileSubRunFlag[subProNum][m_WhileSubNC[subProNum] - 1] == 1)
			{
				if(m_WhileSubJudgeType[subProNum][m_WhileSubNC[subProNum] - 1] == V_R_METHOD)
				{//R判断
					m_WhileSubCycCounter[subProNum][m_WhileSubNC[subProNum] - 1]++;
				}
			}
			result = 1;
			break;
		case K_IF://IF判断命令
		case K_ELSE://else判断条件命令，两者处理方式相同
			m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 0;																//默认判断条件失败
			if((V_USER1<=m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1])&&(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] <= V_USER8))
			{//用户变量判断
				if((USER_Parameter.CURR_Num[m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1]-V_USER1] != 0) && (m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] == V_EQUAL)\
						&& (USER_Parameter.CURR_Num[m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1]-V_USER1] % m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]) == 0)	
				{//user当前值 = R数量的倍数
					m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
				}
				else if((m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] == V_ONLY_EQUAL) && (USER_Parameter.CURR_Num[m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1]-V_USER1] == m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]))	
				{//user当前值 = R数量
					m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
				}
				else if((m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] == V_NOT_EQUAL) && (USER_Parameter.CURR_Num[m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1]-V_USER1] != m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]))	
				{//user当前值 != R数量
					m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
				}
				else if((m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] == V_NOTONLY_EQUAL) && (USER_Parameter.CURR_Num[m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1]-V_USER1] % m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]) != 0)	
				{//user当前值 != R数量的倍数时
					m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
				}
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_I_METHOD)
			{//I跳转
				if(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] < INPUT_NUM)
				{
					if(m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] == V_LOW_LEVEL && \
							ReadInput(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1]) == 0)
					{//检测有效电平
						m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
					else if(m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] == V_HIGH_LEVEL && \
										ReadInput(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1]) == 1)
					{//检测无效电平
						m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
				}
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_O_METHOD)
			{//O判断
				if(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] < OUTPUT_NUM)
				{
					ionum = m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1];
					index = ionum / 8;
					offset = ionum % 8;
					if(m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] == V_RESET && ((Output_Status[index] >> offset) & 0x01) == 0x01)
					{//检测复位
						m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
					else if(m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] == V_SET && ((Output_Status[index] >> offset) & 0x01) == 0)
					{//检测置位
						m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
				}
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_P_METHOD)
			{//P判断
				if(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] >= V_XAXISGREATER && \
					m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] <= V_OAXISGREATER)
				{//大于等于
					axisNum = m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] - V_XAXISGREATER;		
					pointTemp = m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] * Step_Coefficient[axisNum] / 100 + MINROBOTPOSITION;
					if(m_PulseTotalCounter[axisNum] + JDZ_AllowError >= pointTemp)
					{//判断条件成立
						m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
				}
				else if(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] >= V_XAXISEQUAL && \
					m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] <= V_OAXISEQUAL)
				{//等于
					axisNum = m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] - V_XAXISEQUAL;
					pointTemp = m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] * Step_Coefficient[axisNum] / 100 + MINROBOTPOSITION;
					if(m_PulseTotalCounter[axisNum] + JDZ_AllowError >= pointTemp && m_PulseTotalCounter[axisNum] <= pointTemp + JDZ_AllowError)
					{//判断条件成立
						m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
				}
				else if(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] >= V_XAXISLESS && \
					m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] <= V_OAXISLESS)
				{//小于等于
					axisNum = m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1] - V_XAXISLESS;
					pointTemp = m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1] * Step_Coefficient[axisNum] / 100 + MINROBOTPOSITION;
					if(m_PulseTotalCounter[axisNum] <= pointTemp + JDZ_AllowError)
					{//判断条件成立
						m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
					}
				}
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_LAYER_NUM)
			{//层数判断
				m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = MD_LayerNumJudge(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1], m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]);
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_LAYER_FULL)
			{//层满判断
				m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = MD_LayerFullJudge(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1], m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]);
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_STACK_FULL)
			{//垛满判断
				m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = MD_StackFullJudge(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1], m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]);
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_MDGOOD)
			{//物品判断
				m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = MD_GoodNumJudge(m_IfElseSubJudgePar1[subProNum][m_IfElseSubNC[subProNum] - 1], m_IfElseSubJudgePar2[subProNum][m_IfElseSubNC[subProNum] - 1]);
			}
			else if(m_IfElseSubJudgeType[subProNum][m_IfElseSubNC[subProNum] - 1] == V_RI_NULL)
			{//NULL跳转
				m_IfElseSubJudgeRes[subProNum][m_IfElseSubNC[subProNum] - 1] = 1;
			}
			result = 1;
			break;
		case K_OVER:
			result = 1;
			break;
		case K_SPECIAL://特殊指令								
			if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_SUSPEND)
			{
				result = 1;
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 == V_STOP)
			{
				result = 1;
			}
			break;	
		case K_PULSE_OUTPUT:		       //脉宽输出
			result = 1;	
			break;
		case K_USER:		              //用户变量
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
		case K_OUTDETECT://输出检测
			if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 < OUTPUT_NUM)
			{
				ionum = SubProgram_Operate[subProNum].Program[ActionLine].Value1;
//				if(ionum>=24)ionum += 4;//RY0-RY1
				index = ionum / 8;
				offset = ionum % 8;
				if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_RESET && ((Output_Status[index] >> offset) & 0x01) == 0x01)
				{//复位
					result = 1;
				}
				else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_SET && ((Output_Status[index] >> offset) & 0x01) == 0)
				{//置位
					result = 1;
				}
			}
		case K_MDPOSITION://确认码垛命令是否到位
			if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 >= V_MD_AXSIS \
					&& SubProgram_Operate[subProNum].Program[ActionLine].Value1 < V_MD_AXSIS + Axis_Num)
			{//码垛位置
				axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1 - V_MD_AXSIS;
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Value1 >= V_MD_AXSIS + Axis_Num \
					&& SubProgram_Operate[subProNum].Program[ActionLine].Value1 < V_MD_AXSIS + Axis_Num * 2)
			{//等待点位置
				axsisNum = SubProgram_Operate[subProNum].Program[ActionLine].Value1 - V_MD_AXSIS - Axis_Num;
			}
			
			if(axsisNum == O_Axsis && sMD_Parameter.revolveMode == 1)
			{//O轴为气缸
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
		case K_MDPOCOUNT://码垛计数
			result = 1;	
			break;
		case K_XAXIS://轴是否移动到目标位置
		case K_ZAXIS:
		case K_LAXIS:
		case K_OAXIS:
			if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_XAXIS) 
			{//X轴
				axsisNum = X_Axsis;
				pointTemp = Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Point_X + MINROBOTPOSITION;
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_ZAXIS) 
			{//Z轴
				axsisNum = Z_Axsis;
				pointTemp = Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Point_Z + MINROBOTPOSITION;
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_LAXIS)
			{//Y轴
				axsisNum = L_Axsis;
				pointTemp = Manul_Save_Point[SubProgram_Operate[subProNum].Program[ActionLine].Value1 - 1].Point_L + MINROBOTPOSITION;
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Key == K_OAXIS)
			{//O轴
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
		case K_KEEP_MOVE://正向搜素
		case K_NEGTIVE_SEARCH://反向搜索
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
				{//低电平
					if(keepMoveIoSta_Low[axsisNum])
					{//收到低电平信号
						Servo_Stop(axsisNum);
						Flag_Keep_Move[axsisNum] = 2;
					}
				}
				else if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_HIGH_LEVEL)
				{//高电平
					if(keepMoveIoSta_High[axsisNum])	
					{//收到高电平信号
						Servo_Stop(axsisNum);
						Flag_Keep_Move[axsisNum] = 2;
					}
				}
				else if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_FALLING_EDGE)
				{//下降沿，检测低
					if(Flag_Falling_Edge_Sub == 1)
					{//一开始检测到低电平
						if(keepMoveIoSta_High[axsisNum])
						{//检测到高电平
							Flag_Falling_Edge_Sub = 3;
						}
					}
					if(Flag_Falling_Edge_Sub == 2 || Flag_Falling_Edge_Sub == 3)
					{//一开始检测到高电平,或者再次检测到高
						if(keepMoveIoSta_Low[axsisNum])
						{//检测到低电平，停止
							Servo_Stop(axsisNum);
							Flag_Keep_Move[axsisNum] = 2;
						}
					}
				}
				else if(SubProgram_Operate[subProNum].Program[ActionLine].Value3 == V_RISING_EDGE)
				{//上升沿，检测高
					if(Flag_Rising_Edge_Sub == 1)
					{//一开始检测到高电平
						if(keepMoveIoSta_Low[axsisNum])
						{//检测到低电平
							Flag_Rising_Edge_Sub = 3;
						}
					}
					if(Flag_Rising_Edge_Sub == 2 || Flag_Rising_Edge_Sub == 3)
					{//一开始检测到低电平,或者再次检测到低
						if(keepMoveIoSta_High[axsisNum])
						{//检测到高电平，停止
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
		case K_INCREMENT_RUNNING://是否移动到目标位置
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
		
		//输出接口-无检测
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

		//输入接口检测
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
			{//检测下降沿 与保持时间无关
				if(signValue == 1)
				{//出现有效信号
					result = 1;
					Detect_Falling_Edge_Sub[subProNum] = 0;
				}
			}
			else if(SubProgram_Operate[subProNum].Program[ActionLine].Value2 == V_RISING_EDGE)
			{//检测上升沿 与保持时间无关
				if(signValue == 1)
				{//出现有效信号
					result = 1;
					Detect_Rising_Edge_Sub[subProNum] = 0;
				}
			}
			else
			{
				if(signValue == 1)
				{//出现有效信号
					if(IO_Input_keepMin[ionum] >= IO_Input_keepMax[ionum])
					{//有效时间的起始时间等于终止时间，认为有效时间为0，一旦出现有效信号则动作确认成功
						result = 1;		
					}
					
					if(g_SubAuto_Valid_Timer[subProNum] > IO_Input_keepMax[ionum])
					{//有效信号时间超过最大保持时间,关闭计时器，不把计时器清零，因为下次出现无效信号时，从0开始计时
						g_SubAuto_Valid_Timer[subProNum] = 0;
						g_SubAuto_Valid_Flag[subProNum] = FALSE;
					}
					
					if(g_SubAuto_Valid_Flag[subProNum] == FALSE)
					{//开始计数
						g_SubAuto_Valid_Timer[subProNum] = 0;
						g_SubAuto_Valid_Flag[subProNum] = TRUE;
					}
				}
				else
				{//否则复位计数器，从0开始
					if(g_SubAuto_Valid_Timer[subProNum] > IO_Input_keepMin[ionum] && g_SubAuto_Valid_Timer[subProNum] < IO_Input_keepMax[ionum])
					{//有效时间计数大于最小时间，小于最大时间
						result = 1;
					}
					g_SubAuto_Valid_Timer[subProNum] = 0;
					g_SubAuto_Valid_Flag[subProNum] = FALSE;
				}
			}
			break;
		default://指令类型异常
			 result = 14;	
			 break;
	}
	return result;		
}
/**************************************************************************************************
**  函数名：  AutoActionOutDelay()
**	输入参数：需要运行的行号
**	输出参数：延时是否完成
**	函数功能：主程序自动运行动作后的延时控制
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
u8 AutoActionOutDelay(SaveProgram *Program_Operate, u8 ActionLine)
{
	u8 result = FALSE;
	
	if(OR_IOORDER != Program_Operate->Program[ActionLine].Order)
	{//只有IO命令才需要加这个延时操作
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
				{//延时时间到达
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
**  函数名：  AutoActionOutDelay()
**	输入参数：需要运行的行号
**	输出参数：延时是否完成
**	函数功能：子程序自动运行动作后的延时控制
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
u8 SubProgramActionOutDelay(u8 subProNum, u8 ActionLine)
{
	u8 result = FALSE;
	
	if(OR_IOORDER != SubProgram_Operate[subProNum].Program[ActionLine].Order)
	{//只有IO命令才需要加这个延时操作
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
				{//延时时间到
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
**  函数名：  ActionAllowJudge()
**	输入参数：无
**	输出参数：无
**	函数功能：动作执行前，命令的合法性判断
**	备注：	  
**  作者：    
**  开发日期：
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
			{//基本指令类型异常
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
			{//轴控指令类型异常
				result = 13;
			}
			break;
		case OR_IOORDER:
			if((K_IOINSTRUCT_OUTPUT1 <= Program_Operate->Program[ActionLine].Key) && (Program_Operate->Program[ActionLine].Key <= K_IOINSTRUCT_INPUT30))
			{
				result = FALSE;
			}
			else
			{//IO控制指令类型异常
				result = 14;
			}
			break;
		default://主要指令类型异常
			result = 11;
			break;
	}
	return result;
}

/**************************************************************************************************
**  函数名：  SubActionAllowJudge()
**	输入参数：无
**	输出参数：无
**	函数功能：动作执行前，命令的合法性判断
**	备注：	  
**  作者：    
**  开发日期：
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
			{//基本指令类型异常
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
			{//轴控指令类型异常
				result = 13;
			}
			break;
		case OR_IOORDER: 
			if((K_IOINSTRUCT_OUTPUT1 <= SubProgram_Operate[subProNum].Program[ActionLine].Key) && (SubProgram_Operate[subProNum].Program[ActionLine].Key <= K_IOINSTRUCT_INPUT30))
			{
				result = FALSE;
			}
			else
			{//IO控制指令类型异常
				result = 14;
			}
			break;
		default://主要指令类型异常
			result = 11;
			break;
	}
	return result;
}

/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/





