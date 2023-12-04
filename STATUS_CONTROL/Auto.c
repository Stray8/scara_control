/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Auto.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f4xx.h"
#include "Usart.h" 
#include "Auto.h"
#include "StatusControl.h"
#include "SpeedControl.h"
#include "in.h"
#include "out.h"
#include "Manual.h"
#include "Parameter.h"
#include "Error.h"
#include "SignalWatch.h"
#include "Delay.h"
#include "w25qxx.h"
#include "Auto_2.h"
#include "ActionOperate.h"
#include "JDZ.h"
#include "BackToOrigin.h"
#include "CANopen.h"


/**-- 自动模式下的各模式标志位 --**/
u32 Auto_Pulse_Count = 0;        						//自动模式每个动作发送脉冲个数
u8 Auto_Mode = 	LOOP_MODE;	     						//自动模式选择:1-单次，3-循环
u8 Loop_Mode_Enable = DISABLE;	 						//循环模式使能
u8 Once_Mode_Enable = DISABLE;	 						//单次模式使能
u8 Single_Mode_Enable = DISABLE; 						//单步模式使能

u8  Auto_Reset_Step = 0;
u8  Auto_Reset_Step_Done = FALSE;

u32 Present_Position = MINROBOTPOSITION;	 	//当前位置值
u32 Action_Delay_Time = 0;          				//机械手每次动作完成之后的延时，因为伺服器动作是跟随动作
//u8 	Puls_Delay_Time[50] = {0};		  				//脉冲信号延时
//u8  Puls_Delay_Enable[50] = {DISABLE};  		//脉冲信号标注
//u8  Puls_Delay_Num = 0;			        				//脉冲信号标注
u8  Action_Done_Flag =	FALSE;		  				//伺服器动作完成标志
u8  Action_Delay_Flag	=	FALSE;		  				//动作延时完成

u8  g_Robot_Has_Debuged_Flag = FALSE;	 			//机械手之前状态是否处于自由编程调试状态
u8  g_Program_Is_Debuging = FALSE;					//机械手是否处于调试模式


/**************************************************************************************************
**  函数名：  AutoReset()
**	输入参数：无
**	输出参数：无
**	函数功能：自动下的标志位清零
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void AutoReset()
{
	u16 i = 0;
	
	Back_Origin_Flag = FALSE;
	Origin_Backed = FALSE;
	Robot_Auto_Reset = FALSE;
	Once_Mode_Enable = DISABLE;
	Single_Mode_Enable = DISABLE;
	Loop_Mode_Enable = DISABLE;
	Jog_Move_Enable = DISABLE;
	Linked_Move_Enable = DISABLE;
	g_AutoStatue = AUTO_WAITE;
	Work_Status = WAIT_MODE;
	g_Auto_ActionError_Flag = FALSE;
	g_Auto_ActionTimeOut_Flag = FALSE;
	MD_PositionErr_Flag = FALSE;
	g_Auto_ActionRun_Timer = 0;
	g_Auto_ActionRun_Step = 0;
	g_Auto_PresentLine = 0;
	Action_Step_List_Num = 0;
	Action_Step_Run_Num = 0;
	Action_Step_Confirm_Num = 0;
	g_Auto_Valid_Timer = 0;
	g_Auto_Valid_Flag = FALSE;
	g_USART_Delay_Timer = 0;
	g_Program_Is_Debuging = FALSE;
	Auto_Reset_Step_Done = FALSE;
	Auto_Reset_Step = 0;
	g_Auto_Reset_Flag = FALSE;	
	g_Auto_Order_Pause = FALSE;
	g_Auto_Order_Stop = FALSE;
	g_ActionDelay_Step = 0;
	
	Axis_Machine_Origin_Flag = FALSE;
	Homing_Flag_Can = FALSE;
	
	for(i=0; i<SAVEPROGRAMNUM_SUB; i++)
	{
		g_SubProgram_ActionRun_Timer[i] = 0;
		g_SubProgram_ActionRun_Step[i] = 0;
		g_SubProgram_PresentLine[i] = 0;
		g_Read_SubProgram[i] = FALSE;
		g_SubProgramDelay_Step[i] = 0;

		g_SubProgram_Step_Run[i] = FALSE;
		g_SubProgram_Start[i] = FALSE;
		g_SubProgram_Finish[i] = FALSE;
		
		SubAction_Step_List_Num[i] = 0;
		SubAction_Step_Run_Num[i] = 0;
		SubAction_Step_Confirm_Num[i] = 0;
		g_SubAuto_Valid_Timer[i] = 0;
		g_SubAuto_Valid_Flag[i] = FALSE;
	}
	
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		Flag_Keep_Move[i] = 0;
		Increment_Finished[i] = FALSE;
		Program_Axis_Origin_Flag[i] = FALSE;
		SlowPointFlag[i] = 0;
	}
	
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		Axsis_MoveProNum[i] = 0;
		m_InterpAxisFlag[i] = 0;
	}
	m_InterpLenAxis = 0xff;
	
	g_Auto_ActionNcWait_Flag = 0;
	for(i=0; i<SAVEPROGRAMNUM_SUB; i++)
	{
		g_Auto_SubActionNcWait_Flag[i] = 0;
	}
}

/**************************************************************************************************
**  函数名：  SafeAreaJudge()
**	输入参数：无
**	输出参数：无
**	函数功能：自动运行时安全区域判断
**	备注：	  无
**  作者：      
**  开发日期：
***************************************************************************************************/
void SafeAreaJudge(void)
{
 	u16 i = 0;
	
	if(Origin_Backed == TRUE)
	{//回零后才能进入安全区保护功能
		for(i=0; i<SAVESAFEAREA; i++)
		{//确定当前X轴在哪个安全区内
			if(Robot_Safe_Area[i].SafeArea_Switch)
			{
				if((m_PulseTotalCounter[X_Axsis] >= Robot_Safe_Area[i].X_Left) && (m_PulseTotalCounter[X_Axsis] <= Robot_Safe_Area[i].X_Right))
				{//当前X轴处于安全区域范围内
					if(AxisMoveFlag[Z_Axsis] == 1 && Axsis_Move_Direction[Z_Axsis] == POSITIVE && \
							m_PulseTotalCounter[Z_Axsis] > Robot_Safe_Area[i].Z_Down)
					{//如果Z轴 大于安全区Z轴下位置	或 小于安全区Z轴上位置，那么需要进行安全区出错处理
						Robot_Error_Data[0] = Robot_Error_Data[0] | 0x02;
						CloseTotalMotorError();
						break;
					}
				}
			}
		}
	}
}

/**************************************************************************************************
**  函数名：  SetSingle()
**	输入参数：Reset_Output、Set_Output  复位、置位IO口，60表示无效
**	输出参数：无
**	函数功能：信号输出，并置位标志位
**	备注：	  
**  作者：        
**  开发日期：
***************************************************************************************************/
void SetSingle(u8 Reset_Output,u8 Set_Output, u32 Detect_Flag)
{
	//信号输出：复位 , 置位 , 脉冲信号
	if(Reset_Output == 60)
	{ //单信号-置位某IO
	}
	else
	{
		SetOutput(Reset_Output);
	}
	
	if(Set_Output ==60)
	{ //单信号-复位某IO
	}
	else
	{
		ResetOutput(Set_Output);
//		if(Detect_Flag == 15)
//		{
//			Puls_Delay_Num++;
//			Puls_Delay_Enable[Set_Output] = ENABLE;	   //置位标志位
//		}
	}
}

/**************************************************************************************************
**  函数名：  Read_SaveProgram_IIC_Address()
**	输入参数：无
**	输出参数：无
**	函数功能：根据选中的程序号获取程序地址和程序行数
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void Read_SaveProgram_IIC_Address()
{
	u8 i = 0;

	SaveProgram_IIC_Address = 0;
	SaveProgram_IIC_Num = 0;
	
	if(g_Run_Program_Num > 0)
	{
		if(g_Run_Program_Num > SAVEPROGRAMNUM_MAIN)
		{
			SaveProgram_IIC_Address = Program_IIC_Address[g_Run_Program_Num-1].Address;
			SaveProgram_IIC_Num = Program_IIC_Address[g_Run_Program_Num-1].Num;
		}
		else
		{
			for(i=0; i<SAVEPROGRAMNUM_MAIN; i++)
			{//查询到编号与当前设定自动运行编号相同数组
				if(Program_IIC_Address[i].Code == g_Run_Program_Num)
				{
					SaveProgram_IIC_Address = Program_IIC_Address[i].Address;
					SaveProgram_IIC_Num = Program_IIC_Address[i].Num;
					break;
				}
			}
		}
	}
}

/**************************************************************************************************
**  函数名：  AutoRun()
**	输入参数：无
**	输出参数：无
**	函数功能：自动模式下的各种操作
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void AutoRun()
{	
	switch(UsartReceiveData[1])
	{
		case P_ONCE_MODE:
			Auto_Mode = ONCE_MODE;
			Once_Mode_Enable = ENABLE;
			Loop_Mode_Enable = DISABLE;
			break;
		
		case P_CYCLE_MODE://循环状态
			Auto_Mode = LOOP_MODE;
			Loop_Mode_Enable = ENABLE;		//状态置位
			Once_Mode_Enable = DISABLE;
			break;	

		case P_SINGLE_MODE://单步模式
			if(UsartReceiveData[2] == 0)
			{//关闭
				Single_Mode_Enable = DISABLE;
			}
			else
			{//开启
				Single_Mode_Enable = ENABLE;
			}
			break;	

		case P_ACTION_RUN://启动状态
			if(Not_Get_Position() ==1 && JDZ_Parameter.Switch == 1)
			{
				return;
			}
			else
			{
				if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
				{
					g_Auto_Order_Start = TRUE;
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
					g_Auto_Order_Pause = FALSE;
					g_Auto_Order_Stop = FALSE;
				}
			}
			
			break;

		case P_ACTION_PAUSE://暂停
			if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
			{
				g_Auto_Order_Pause = TRUE;
			}
			break;

		case P_ACTION_STOP://停止
			if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
			{
				g_Auto_Order_Stop = TRUE;
			}
			break;

		case P_ACTION_PROGRAM://读取要运行的程序
			CurProgramRead(UsartReceiveData[2]);
			break;

		case P_ACTION_RESET:  //机械手复位			 
			Work_Status = AUTO_WORK_MODE;
			g_Auto_Reset_Flag = TRUE;
			Robot_Auto_Reset = FALSE;
			g_Auto_LOrigin_Flag = FALSE;
			break;

		case P_ACTION_DEBUG:  //机械手调试
			g_Auto_Order_Pause = FALSE;
			g_Auto_Order_Stop = FALSE;			
			Work_Status = AUTO_WORK_MODE;		
			Single_Mode_Enable = ENABLE;
			g_Auto_PresentLine = UsartReceiveData[2];	  //当前单步的行
			g_Auto_ActionRun_Step = 0;
			Robot_Auto_Reset = TRUE;
			g_Auto_Order_Start = TRUE;
			g_Robot_Has_Debuged_Flag = TRUE;
			g_Program_Is_Debuging = TRUE;
			break;

		case P_ACTION_LORIGIN:  //机械手L回零			 
			g_Auto_LOrigin_Flag = TRUE;
			g_Auto_Reset_Flag = TRUE;
			Robot_Auto_Reset = FALSE;
			break;
		case P_ACTION_COMMUTE:	//机械手下班功能
			break;
		default:
			break;
	}
}

/**************************************************************************************************
**  函数名：  AutoRunning()
**	输入参数：无
**	输出参数：无
**	函数功能：自动模式下的各模式运行
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
//void AutoRunning()
//{
//	switch(Auto_Mode)
//	{
//		case ONCE_MODE:		//单次模式	
//			if(Once_Mode_Enable == ENABLE)
//			{
//				AutoModeControl();
//			}              				 
//			break;

//		case SINGLE_MODE:	//单步模式
//			if(Single_Mode_Enable == ENABLE)
//			{
//				AutoModeControl();
//			}
//			break;

//		case LOOP_MODE:		//循环模式
//			if(Loop_Mode_Enable == ENABLE)
//			{
//				AutoModeControl();
//			}
//			break;

//		default:
//			break;
//	}
//}


/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
