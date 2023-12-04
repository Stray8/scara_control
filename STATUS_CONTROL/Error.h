/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __error_h_
#define __error_h_
#include "stm32f4xx.h"

#define NO_ERROR									0x00
#define ERROR_HAPPEN							0x01
#define ERROR_EMERG_HAPPEN				0x02
#define ERROR_STATUS            	0x00  		//轴报警错误检测电平标志位  0x00低电平有效  0x01高电平有效

#define HARDLIMITJUDGE_EXTI         				//硬限位扫描方式-外部中断

#define SAFEVARIATION	100			//用于安全区、软限位的位置允许偏差

/*变量*/
extern u8 Error_Status;				  						//错误状态
extern u8 Robot_Error_Data[15];			  			//报警数据
extern u8 Cancle_Genaral_Warning;
extern u16 IO_Input_waittime[30];						//输入检测超时时间
extern u16 IO_Input_keepMin[30];						//输入保持时间-下限
extern u16 IO_Input_keepMax[30];						//输入保持时间-上限
extern u16 IO_Sign_On_Off[30];							//输入信号常开常闭标志，0常开，1常闭
extern u16 OutPut_BeforeOrigin[30];		   	 //回零前选择
extern u16 OutPut_AfterOrigin[30];			     //回零后选择
extern u16 OutPut_Common_Alarm[30];	 //普通报警
extern u16 OutPut_Emerge_Alarm[30];	 //急停报警
extern u16 OutPut_Pause[30];			     //暂停
extern u16 OutPut_Stop[30];			     //停止

extern u32 Axsis_Error_Count;								//伺服单独报警开始检测定时器，防止开机直接报警
extern u8 Axsis_Error_Permit_Flag;					//伺服单独报警开始检测允许标志

extern u8 g_Auto_ActionConflict_Flag;		//程序间重复操作

/*函数*/
extern void ErrorOperate(void);	      			//错误监测及操作
extern void Scan_TimeOut_IO(u8 i_num);	//报警IO识别
extern void Scan_TimeOut_OUTPUT(u8 i_num);	//报警IO识别
extern void CloseTotalMotorError(void);	//出错后关闭所有电机输出
extern void HardLimitJudge(void);							//硬限位检测

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/
