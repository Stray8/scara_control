/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __manual_h_
#define __manual_h_

#include "StatusControl.h"

extern u8 Jog_Move_Enable ;			  		//寸动运行标志位
extern u8 Jog_Mode_Enable ;			  		//寸动模式使能标志位
extern u8 Linked_Mode_Enable;		  		//连动模式使能标志位
extern u8 Linked_Move_Enable;		  		//连动运行标志位

extern u32 Jog_Pulse_Count ;		  		//寸动模式的每个动作的脉冲数
extern u32 Jog_Pulse_Count_Init ;
extern u32 Linked_Pulse;
extern u8  Axis_Manul_Speed[Axis_Num + Ext_Axis_Num];					//轴手动速度
extern u8  Axis_Manul_Speed_Temp[Axis_Num + Ext_Axis_Num];		//在没有回原点时，用于保存原有轴手动速度
extern u16  Axis_Step_Distance[Axis_Num + Ext_Axis_Num];			//轴寸动距离1-100,默认50mm
extern u8  All_Axis_Point_Deleted_Flag;												//所有点已删除标志位

extern void IODebugOutput1(void);
extern void IODebugOutput2(void);
extern void IODebugOutput3(void);
extern void ManulDebug(void);		  							//手动调试
extern void ManualJogRunnig(void);	  					//手动操作点动模式
extern void ManualLinkedRunning(void);					//手动操作连动模式



#endif

/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/

