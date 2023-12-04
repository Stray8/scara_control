/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Auto.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f4xx.h"
#include "Usart.h" 
#include "StatusControl.h"
#include "SpeedControl.h"
#include "Auto.h"
#include "Manual.h"
#include "w25qxx.h"
#include "out.h"
#include "in.h"
#include "Error.h"
#include "Parameter.h"
#include "SignalWatch.h"
#include "Auto_2.h"
#include "BackToOrigin.h"
#include "ActionOperate.h"
#include "CANopen.h"

u8 Initialize_Finished = FALSE;	   		//初始化完成标志位
u8 Origin_Backed = FALSE;		   				//回零完成标志位
u8 Axsis_Origin_Backed[Axis_Num + Ext_Axis_Num] = {FALSE,FALSE,FALSE,FALSE};//各个轴回零完成标志位
u8 Back_Origin_Flag = FALSE;	   			//回零请求标志位

u8 Work_Status = AUTO_WORK_MODE;    	//机械手工作模式 
u8 Axis_Manul_BackToOrigin = FALSE;  	//手动回零
u8 Axsis_Chosen = X_Axsis;		   			//运动轴选择
u8 Axsis_Move_Direction[Axis_Num + Ext_Axis_Num] = {POSITIVE,POSITIVE,POSITIVE,POSITIVE,POSITIVE,POSITIVE};//运动方向选择			  
u32 Input_Detect_Time = 0;		   			//输入检测定时参数
u32 Communication_Time = 0;		   			//通信时间计数-脱机改-DPF
u8 OffLine_Flag = FALSE;		   				//脱机标志位-DPF
u8 OnLineCommunicated_Flag = FALSE;		//联机通信标志位-DPF
u8 Input_Detect_Enable = ENABLE;   		//输入检测使能标志，第一次使能
u8 Jog_Pause_Enable = DISABLE;       	//寸动操作暂停
u8 g_Current_SafeArea_Num = 0;	 	 		//安全区编号
u8 Input_Count17 = 0;			   					//输入17计数，回零按钮计数
u8 Input_Count18 = 0;			   					//输入18计数，启动按钮计数
u8 Input_Count19 = 0;			   					//输入19计数，暂停按钮计数
u8 Input_Count20 = 0;			   					//输入20计数，停止按钮计数
u8 Servo_Stop_Done[Axis_Num + Ext_Axis_Num] = {FALSE, FALSE, FALSE, FALSE, FALSE, FALSE};		   //伺服器停止动作是否执行标志
u8 g_Auto_Reset_Flag = FALSE;	   			//自动复位请求标志
u8 Robot_Auto_Reset = FALSE;					//自动复位完成标志
u8 g_Auto_LOrigin_Flag = FALSE;	   		//L轴回零
u32 g_USART_Delay_Timer = 0;		   		//串口延时计数-方式通信失败
u8 gs_AutoStatue = 4;
u8 gs_Error_Status = NO_ERROR;
u8 g_MoveCmdRetrans = FALSE;

/**************************************************************************************************
**  函数名：  WorkMode()
**	输入参数：无
**	输出参数：无
**	函数功能：工作模式选择
**	备注：	  无
**  作者：         
**  开发日期： 
***************************************************************************************************/
void WorkMode()
{
	switch(UsartReceiveData[1])
	{
		case P_AUTO_MODE://自动模式
			Work_Status = AUTO_WORK_MODE;
			if(g_Robot_Has_Debuged_Flag)
			{
				g_AutoStatue = AUTO_WAITE;
				g_Auto_Order_Start = FALSE;
				g_Auto_Order_Pause = FALSE;
				g_Auto_Order_Stop = FALSE;
				Single_Mode_Enable = DISABLE;
				g_Auto_PresentLine = 0;	  					//当前单步的行
				g_Auto_ActionRun_Step = 0;
				Robot_Auto_Reset = FALSE;		
				g_Robot_Has_Debuged_Flag = FALSE;	 
			}
			break;	

		case P_FREE_PROGRAM://自由编程
			Work_Status = FREE_PROGRAM_MODE;     
			break;	

		case P_IO_MODE://IO调试
			Work_Status = IO_DEBUG_MODE;
			break;	

		case P_MANUL_MODE://手动模式
			Work_Status = MANUAL_WORK_MODE;
			Axsis_Chosen = X_Axsis;										//进入手动模式时，默认选中X轴
			Axsis_Move_Direction[X_Axsis] = POSITIVE;
			Linked_Mode_Enable = ENABLE;							//进入手动模式时，默认为连动模式
			Jog_Mode_Enable = DISABLE;
			Jog_Pause_Enable=DISABLE;
			break;

		case WAIT_MODE://等待模式
			Work_Status = WAIT_MODE;
			break;

		default:
			break;
	}
}

/**************************************************************************************************
**  函数名：  u8 ManulSafeAreaDetec(void)
**	输入参数：无
**	输出参数：无
**	函数功能：机械手运动安全区域、软限位检测及手动移动距离处理
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
u8 ManulSafeAreaDetec(void)
{
	u16 i = 0;
	u8 safeAreaFlag = FALSE;
	
	if(Origin_Backed == TRUE)
	{
		for(i=0; i<SAVESAFEAREA; i++)
		{//确定当前X轴在哪个安全区内
			if(Robot_Safe_Area[i].SafeArea_Switch)
			{
				if(m_PulseTotalCounter[X_Axsis] >= Robot_Safe_Area[i].X_Left && m_PulseTotalCounter[X_Axsis] <= Robot_Safe_Area[i].X_Right)
				{
					if(Axsis_Chosen == X_Axsis && Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE && (m_PulseTotalCounter[Axsis_Chosen] <= Robot_Safe_Area[i].X_Left) && \
							(m_PulseTotalCounter[Axsis_Chosen] >= Robot_Safe_Area[i].Z_Up && m_PulseTotalCounter[Axsis_Chosen] <= Robot_Safe_Area[i].Z_Down))
					{//X轴已在当前安全区最小边界，不能再往负方向
						safeAreaFlag = TRUE;
					}
					else if(Axsis_Chosen == X_Axsis && Axsis_Move_Direction[Axsis_Chosen] == POSITIVE && (m_PulseTotalCounter[Axsis_Chosen] >= Robot_Safe_Area[i].X_Right) && \
							(m_PulseTotalCounter[Axsis_Chosen] >= Robot_Safe_Area[i].Z_Up && m_PulseTotalCounter[Axsis_Chosen] <= Robot_Safe_Area[i].Z_Down))
					{//X轴已在当前安全区最大边界，不能再往正方向
						safeAreaFlag = TRUE;
					}
					else if(Axsis_Chosen == Z_Axsis && Axsis_Move_Direction[Axsis_Chosen] == POSITIVE && (m_PulseTotalCounter[Axsis_Chosen] >= Robot_Safe_Area[i].Z_Down))
					{//Z轴已在当前安全区最大边界，不能再往正方向，Z轴向负方向可以随便移动
						safeAreaFlag = TRUE;
					}
					
					//软限位处理
					if(Axsis_Move_Direction[Axsis_Chosen] == POSITIVE && m_PulseTotalCounter[Axsis_Chosen] >= Axsis_Maxlength[Axsis_Chosen])
					{
						safeAreaFlag = TRUE;
					}
					else if(Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE && m_PulseTotalCounter[Axsis_Chosen] <= Axsis_Minlength[Axsis_Chosen])
					{
						safeAreaFlag = TRUE;
					}
					
					if(safeAreaFlag == FALSE)
					{
						if(Axsis_Chosen == Z_Axsis)
						{//处理Z轴
							if(Linked_Mode_Enable == ENABLE && Axsis_Move_Direction[Axsis_Chosen] == POSITIVE)
							{//连动时，修改Z轴正向移动距离
								if(Linked_Pulse > Robot_Safe_Area[i].Z_Down && Robot_Safe_Area[i].Z_Down > m_PulseTotalCounter[Axsis_Chosen])
								{
									Linked_Pulse = Robot_Safe_Area[i].Z_Down;
								}
							}
							else if(Jog_Mode_Enable == ENABLE && Axsis_Move_Direction[Axsis_Chosen] == POSITIVE)
							{//寸动时，修改Z轴正向移动距离
								if(Jog_Pulse_Count > Robot_Safe_Area[i].Z_Down && Robot_Safe_Area[i].Z_Down > m_PulseTotalCounter[Axsis_Chosen])
								{
									Jog_Pulse_Count = Robot_Safe_Area[i].Z_Down;
								}
							}
						}
						else if(Axsis_Chosen == X_Axsis && (m_PulseTotalCounter[Z_Axsis] > Robot_Safe_Area[i].Z_Up && \
									m_PulseTotalCounter[Z_Axsis] <= Robot_Safe_Area[i].Z_Down))
						{//处理X轴
							if(Linked_Mode_Enable == ENABLE && Axsis_Move_Direction[Axsis_Chosen] == POSITIVE)
							{//连动时，修改X轴正向移动距离
								if(Linked_Pulse > Robot_Safe_Area[i].X_Right)
								{
									Linked_Pulse = Robot_Safe_Area[i].X_Right;
								}
							}
							else if(Linked_Mode_Enable == ENABLE && Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE)
							{//连动时，修改X轴负向移动距离
								if(Linked_Pulse < Robot_Safe_Area[i].X_Left)
								{
									Linked_Pulse = Robot_Safe_Area[i].X_Left;
								}
							}
							else if(Jog_Mode_Enable == ENABLE && Axsis_Move_Direction[Axsis_Chosen] == POSITIVE)
							{//寸动时，修改X轴正向移动距离
								if(Jog_Pulse_Count > Robot_Safe_Area[i].X_Right)
								{
									Jog_Pulse_Count = Robot_Safe_Area[i].X_Right;
								}
							}
							else if(Jog_Mode_Enable == ENABLE && Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE)
							{//寸动时，修改X轴负向移动距离
								if(Jog_Pulse_Count <Robot_Safe_Area[i].X_Left)
								{
									Jog_Pulse_Count = Robot_Safe_Area[i].X_Left;
								}
							}
						}
					}
				}
			}
			
			if(safeAreaFlag == TRUE)
			{
				break;
			}
		}
	}
	
	return safeAreaFlag;
}


/**************************************************************************************************
**  函数名：  CurProgramRead()
**	输入参数：无
**	输出参数：无
**	函数功能：选中程序读取
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void CurProgramRead(u8 programNum)
{
	u16 j = 0;
	u16 k = 0;
	u8 Read_IIC_Auto_Program[16] = {0};
	
	if(programNum > 0 && programNum <= SAVEPROGRAMNUM)
	{
		g_Run_Program_Num = programNum;
		W25QXX_Write(&g_Run_Program_Num, 0x40E0, 1);
		g_Run_Program_Num_Pre = g_Run_Program_Num;
		Read_SaveProgram_IIC_Address();
		if(SaveProgram_IIC_Address > 0)
		{
			W25QXX_Read(Read_IIC_Auto_Program, SaveProgram_IIC_Address, 15);
			Free_Program_Operate.Flag  = Read_IIC_Auto_Program[0];
			Free_Program_Operate.Code  = Read_IIC_Auto_Program[1];
			Free_Program_Operate.Name  = (u32)(((u32)Read_IIC_Auto_Program[2])|((u32)Read_IIC_Auto_Program[3]<<8)|((u32)Read_IIC_Auto_Program[4]<<16)|((u32)Read_IIC_Auto_Program[5]<<24));
			Free_Program_Operate.Name2 = (u32)(((u32)Read_IIC_Auto_Program[6])|((u32)Read_IIC_Auto_Program[7]<<8)|((u32)Read_IIC_Auto_Program[8]<<16)|((u32)Read_IIC_Auto_Program[9]<<24));
			Free_Program_Operate.Name3 = (u32)(((u32)Read_IIC_Auto_Program[10])|((u32)Read_IIC_Auto_Program[11]<<8)|((u32)Read_IIC_Auto_Program[12]<<16)|((u32)Read_IIC_Auto_Program[13]<<24));				 
			Free_Program_Operate.Num   = Read_IIC_Auto_Program[14];
			for(j=0; j<Free_Program_Operate.Num; j++)
			{
				W25QXX_Read(Read_IIC_Auto_Program, SaveProgram_IIC_Address+0x0F+0x10*j, 16);
				Free_Program_Operate.Program[j].Flag   = Read_IIC_Auto_Program[0];
				Free_Program_Operate.Program[j].List   = Read_IIC_Auto_Program[1];
				Free_Program_Operate.Program[j].Order  = Read_IIC_Auto_Program[2];
				Free_Program_Operate.Program[j].Key    = Read_IIC_Auto_Program[3];
				Free_Program_Operate.Program[j].Value1 = (u32)(((u32)Read_IIC_Auto_Program[4])|((u32)Read_IIC_Auto_Program[5]<<8)|((u32)Read_IIC_Auto_Program[6]<<16)|((u32)Read_IIC_Auto_Program[7]<<24));
				Free_Program_Operate.Program[j].Value2 = (u32)(((u32)Read_IIC_Auto_Program[8])|((u32)Read_IIC_Auto_Program[9]<<8)|((u32)Read_IIC_Auto_Program[10]<<16)|((u32)Read_IIC_Auto_Program[11]<<24));	
				Free_Program_Operate.Program[j].Value3 = (u32)(((u32)Read_IIC_Auto_Program[12])|((u32)Read_IIC_Auto_Program[13]<<8)|((u32)Read_IIC_Auto_Program[14]<<16)|((u32)Read_IIC_Auto_Program[15]<<24)); 					
				Free_Program_Operate.Program[j].Value1 = Free_Program_Operate.Program[j].Value1 & 0x0fffffff;
				Free_Program_Operate.Program[j].Value2 = Free_Program_Operate.Program[j].Value2 & 0x0fffffff;
					
				if((Read_IIC_Auto_Program[3] == K_INCREMENT_RUNNING && Read_IIC_Auto_Program[15]>>4 == 0x09)\
					|| ((Read_IIC_Auto_Program[3] == K_IF || Read_IIC_Auto_Program[3] == K_ELSE ||Read_IIC_Auto_Program[3] == K_WHILE || Read_IIC_Auto_Program[3] == K_USER) && Read_IIC_Auto_Program[15]>>4 == 0x08))
				{
					Free_Program_Operate.Program[j].Value3 = Free_Program_Operate.Program[j].Value3 | 0xf0000000;	
				}
				else
				{
					Free_Program_Operate.Program[j].Value3 = Free_Program_Operate.Program[j].Value3 & 0x0fffffff;	
				}
			}
			for(k=j; k<LARGESTPROGRAMNUM; k++)
			{
				Free_Program_Operate.Program[k].Flag   = 0;
				Free_Program_Operate.Program[k].List   = 0;
				Free_Program_Operate.Program[k].Order  = 0;
				Free_Program_Operate.Program[k].Key    = 0;
				Free_Program_Operate.Program[k].Value1 = 0;
				Free_Program_Operate.Program[k].Value2 = 0;
				Free_Program_Operate.Program[k].Value3 = 0;
			}
		}
	}
}

/**************************************************************************************************
**  函数名：  ActionControl()
**	输入参数：无
**	输出参数：无
**	函数功能：机械手动作控制
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void ActionControl()
{
	u16 i = 0, j = 0;
	u8 Reset_Parameter[20] = {0};
	
	if(Error_Status != ERROR_EMERG_HAPPEN)
	{//发生紧急报警时，不处理运行模式下的所有操作
		if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE && Origin_Backed == TRUE)
		{//自动模式
			if(g_Auto_Reset_Flag)
			{//执行复位请求（运动的坐标零点）
				if(Temp_OUT_Switch_Parameter[O_RESETING_LIGHT] == 1)
				{
					ResetOutput(O_RESETING_LIGHT);
				}
				g_Run_Program_Num_Pre = g_Run_Program_Num;				
				for(i=0; i<SAVEPROGRAMNUM; i++)
				{//查询到编号与当前设定自动运行编号相同数组
					if(Program_IIC_Address[i].Code == SAVEPROGRAMNUM_MAIN && Program_IIC_Address[i].Flag == 1)
					{
						W25QXX_Read(Reset_Parameter, Program_IIC_Address[i].Address, 15);
						Free_Program_Operate.Flag  = Reset_Parameter[0];
						Free_Program_Operate.Code  = Reset_Parameter[1];
						Free_Program_Operate.Name  = (u32)(((u32)Reset_Parameter[2])|((u32)Reset_Parameter[3]<<8)|((u32)Reset_Parameter[4]<<16)|((u32)Reset_Parameter[5]<<24));
						Free_Program_Operate.Name2 = (u32)(((u32)Reset_Parameter[6])|((u32)Reset_Parameter[7]<<8)|((u32)Reset_Parameter[8]<<16)|((u32)Reset_Parameter[9]<<24));
						Free_Program_Operate.Name3 = (u32)(((u32)Reset_Parameter[10])|((u32)Reset_Parameter[11]<<8)|((u32)Reset_Parameter[12]<<16)|((u32)Reset_Parameter[13]<<24));
						Free_Program_Operate.Num   = Reset_Parameter[14];
						for(j=0; j<Free_Program_Operate.Num; j++)
						{
							W25QXX_Read(Reset_Parameter, Program_IIC_Address[i].Address+0x0F+0x10*j, 16);
							Free_Program_Operate.Program[j].Flag   = Reset_Parameter[0];
							Free_Program_Operate.Program[j].List   = Reset_Parameter[1];
							Free_Program_Operate.Program[j].Order  = Reset_Parameter[2];
							Free_Program_Operate.Program[j].Key    = Reset_Parameter[3];
							Free_Program_Operate.Program[j].Value1 = (u32)(((u32)Reset_Parameter[4])|((u32)Reset_Parameter[5]<<8)|((u32)Reset_Parameter[6]<<16)|((u32)Reset_Parameter[7]<<24));
							Free_Program_Operate.Program[j].Value2 = (u32)(((u32)Reset_Parameter[8])|((u32)Reset_Parameter[9]<<8)|((u32)Reset_Parameter[10]<<16)|((u32)Reset_Parameter[11]<<24));	
							Free_Program_Operate.Program[j].Value3 = (u32)(((u32)Reset_Parameter[12])|((u32)Reset_Parameter[13]<<8)|((u32)Reset_Parameter[14]<<16)|((u32)Reset_Parameter[15]<<24)); 					
							Free_Program_Operate.Program[j].Value1 = Free_Program_Operate.Program[j].Value1 & 0x0fffffff;
							Free_Program_Operate.Program[j].Value2 = Free_Program_Operate.Program[j].Value2 & 0x0fffffff;	
								
							if((Reset_Parameter[3] == K_INCREMENT_RUNNING && Reset_Parameter[15]>>4 == 0x09)\
								|| ((Reset_Parameter[3] == K_IF	 || Reset_Parameter[3] == K_ELSE || Reset_Parameter[3] == K_WHILE || Reset_Parameter[3] == K_USER) && Reset_Parameter[15]>>4 == 0x08))
							{
								Free_Program_Operate.Program[j].Value3 = Free_Program_Operate.Program[j].Value3 | 0xf0000000;	
							}
							else
							{
								Free_Program_Operate.Program[j].Value3 = Free_Program_Operate.Program[j].Value3 & 0x0fffffff;	
							}
						}
						break;
					}
				}
				ActionStepControl();
			}
			if(Robot_Auto_Reset)
			{//执行复位完成后才能执行运行启动
				AutoModeControl();
			}
		}
		else if(Work_Status == MANUAL_WORK_MODE && Back_Origin_Flag == FALSE)
		{//手动模式
			if(Origin_Backed != TRUE)
			{//没回零进手动调试,强制设置坐标
				for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
				{
					//m_PulseTotalCounter[i] = MINROBOTPOSITION + (Axsis_Maxlength[i] - Axsis_Minlength[i]) / 2;
					if(Axis_Manul_Speed_Temp[i] > 20)//没回零时，限制最大速度20%
					{
						Axis_Manul_Speed[i] = 20;
					}
					else
					{
						Axis_Manul_Speed[i] = Axis_Manul_Speed_Temp[i];
					}
				}
			}
			else if(Origin_Backed == TRUE)
			{//回过零需要把速度
				for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
				{
					Axis_Manul_Speed[i] = Axis_Manul_Speed_Temp[i];
				}
			}
			
			if(Jog_Mode_Enable == ENABLE)
			{//寸动模式
				 ManualJogRunnig();								//寸动函数处理
				 if(Jog_Pause_Enable == ENABLE)
				 {
					Servo_Stop(Axsis_Chosen);
					Jog_Pause_Enable = DISABLE;			 
				 }
			}
			else if(Linked_Mode_Enable == ENABLE)
			{//连动模式
				if(Axis_Manul_BackToOrigin == TRUE && m_PulseTotalCounter[Axsis_Chosen] == MINROBOTPOSITION)
				{//手动回零完成之后
					Linked_Move_Enable = DISABLE;
					Axis_Manul_BackToOrigin = FALSE;
				}
				ManualLinkedRunning();						//连动函数处理
			}
		}
	}
}
/**************************************************************************************************
**  函数名：  StatusControl()
**	输入参数：无
**	输出参数：无
**	函数功能：输入状态检测
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
void StatusControl()
{
  u8 i = 0;

	for(i=0; i<30; i++)
	{//读取输入电平
		ReadInput(i);
	}
	HardLimitJudge();						//采用轮询方式实现外部中断
	if(Input_Detect_Enable == ENABLE)
	{
		Input_Detect_Enable = DISABLE;
		
		/**-- 脱机模式下，外部暂停、停止、启动、回零按钮的处理 --**/
		if(OffLine_Flag == TRUE)
		{//脱机状态下
			if((Work_Status == AUTO_WORK_MODE) && (g_AutoStatue != AUTO_RUNNING))
			{//机械手工作模式为自动模式且自动运行状态不是正在运行时，启动和复位才有效
				if(JXS_Parameter.NcOrignin == 0)
				{//回零按钮没选中时，按启动就自动复位
					if((JXS_Parameter.NcStartin != 0) && (Input_Detect_Status[2] & 0x04))
					{//启动 X18
						 Input_Count18++;
						 if(Input_Count18 == 5)
						 {//按键滤波
								Back_Origin_Flag = TRUE;//先回零
								if(Origin_Backed == TRUE)
								{//回零完成，只需复位
									g_Auto_Reset_Flag = TRUE;
									Back_Origin_Flag = FALSE;
								}
								
								if(Robot_Auto_Reset == TRUE)
								{//复位完成，直接启动
									g_Auto_Order_Start = TRUE;
									g_Auto_Reset_Flag = FALSE;
								}
								
								if(Auto_Mode == ONCE_MODE)
								{//单次模式
									Once_Mode_Enable = ENABLE;
									Loop_Mode_Enable = DISABLE;
								}
								else if(Auto_Mode == LOOP_MODE)
								{//循环模式
									Loop_Mode_Enable = ENABLE;
									Once_Mode_Enable = DISABLE;
								}
								
								g_Auto_Order_Stop = FALSE;
								Input_Count18=0;
						 }
					}
					else
					{
						Input_Count18=0;
					}
					
					if((JXS_Parameter.NcStopin != 0) && (Input_Detect_Status[2] & 0x10))	 
					{//点击暂停后点停止 X20
						 Input_Count20++;
						 if(Input_Count20 == 5)
						 {
								Robot_Auto_Reset=FALSE;
								g_AutoStatue = AUTO_WAITE;
								g_Auto_Order_Stop = TRUE;
								AutoPauseOperate();				//暂停运行相关操作
								AutoStopOperate();				//停止运行相关操作
								Input_Count20 = 0;
						 }
					}
					else
					{
						Input_Count20 = 0;
					}
				}
				else
				{//回零按钮选中时，强制去复位，才能启动
					if(Input_Detect_Status[2] & 0x02)	 
					{//复位	X17
						 Input_Count17++;
						 if(Input_Count17 == 5)
						 {
								Back_Origin_Flag = TRUE;			//先回零
								if(Origin_Backed == TRUE)			//回零完成
								{
									g_Auto_Reset_Flag = TRUE;		//去复位
									Back_Origin_Flag = FALSE;		//回零标志位置0
								}
								Input_Count17=0;
						 }
					}
					else
					{
						Input_Count17=0;
					}
					
					if((JXS_Parameter.NcStartin != 0) && (Input_Detect_Status[2] & 0x04) && (Robot_Auto_Reset == TRUE))	 
					{//启动 X18，复位完成时才可启动
						 Input_Count18++;
						 if(Input_Count18 == 5)
						 {
								g_Auto_Order_Start = TRUE;				//最后启动
								g_Auto_Reset_Flag = FALSE;				//复位标志位置0							
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
								g_Auto_Order_Stop = FALSE;				//将状态为置0
								Input_Count18=0;
						 }
					}
					else
					{
						Input_Count18=0;
					}
					if((JXS_Parameter.NcStopin != 0) && (Input_Detect_Status[2] & 0x10))	  //点击暂停后点停止 X20
					{
						 Input_Count20++;
						 if(Input_Count20 == 5)
						 {	
							Robot_Auto_Reset=FALSE;
							g_AutoStatue = AUTO_WAITE;
							g_Auto_Order_Stop = TRUE;
							AutoPauseOperate();				//暂停运行相关操作
							AutoStopOperate();				//停止运行相关操作
							Input_Count20=0;
						 }
					}
					else
					{
						Input_Count20 = 0;
					}
				}	
			}
			else if((Work_Status == AUTO_WORK_MODE) && (g_AutoStatue == AUTO_RUNNING))
			{//机械手工作模式为自动模式且自动运行状态是正在运行时，暂停和停止才有效
				if((JXS_Parameter.NcPausein != 0) && (Input_Detect_Status[2] & 0x08))
				{//暂停	X19
					 Input_Count19++;
					 if(Input_Count19 == 5)
					 {				    
						g_Auto_Order_Pause = TRUE;
						AutoPauseOperate();						//暂停运行相关操作
						Input_Count19 = 0;
					 }
				}
				else
				{
					Input_Count19 = 0;
				}
				
				if((JXS_Parameter.NcStopin!=0)&&(Input_Detect_Status[2] & 0x10))	 
				{//停止 X20
					 Input_Count20++;
					 if(Input_Count20 == 5)
					 {	
							Robot_Auto_Reset = FALSE;
							g_AutoStatue = AUTO_WAITE;
							g_Auto_Order_Stop = TRUE;
							AutoPauseOperate();					//暂停运行相关操作
							AutoStopOperate();					//停止运行相关操作
							Input_Count20 = 0;
					 }
				}
				else
				{
					Input_Count20 = 0;
				}
			}
		}
	}
	
	if(g_AutoStatue != gs_AutoStatue)
	{//三色灯及蜂鸣器的状态控制
		if(OffLine_Flag == TRUE)
		{//脱机状态下，5s之后亮灯
			gs_AutoStatue = g_AutoStatue;
			switch(g_AutoStatue)
			{
				case AUTO_RUNNING:
					SetOutput(O_WAIT_YELLOW_LIGHT);	   	//黄灯
					SetOutput(O_ALARM_RED_LIGHT);	   		//红灯
					SetOutput(O_ALARM_BUZZER);	   			//蜂鸣器
					ResetOutput(O_RUN_GREEN_LIGHT);  		//绿灯
					break;

				case AUTO_PAUSE:
					if((Error_Status != ERROR_HAPPEN) && (g_Auto_ActionTimeOut_Flag !=TRUE))
					{
						SetOutput(O_RUN_GREEN_LIGHT);
						SetOutput(O_ALARM_RED_LIGHT);
						SetOutput(O_ALARM_BUZZER);	  		//蜂鸣器
						ResetOutput(O_WAIT_YELLOW_LIGHT); //黄灯
					}
					else	//动作超时情况下，红灯亮
					{
						gs_Error_Status=Error_Status;
						SetOutput(O_RUN_GREEN_LIGHT);
						SetOutput(O_WAIT_YELLOW_LIGHT);
						SetOutput(O_ALARM_BUZZER);	  		//蜂鸣器
						ResetOutput(O_ALARM_RED_LIGHT); 	//红灯
					}
					break;

				case AUTO_WAITE:
					SetOutput(O_RUN_GREEN_LIGHT);
					SetOutput(O_ALARM_RED_LIGHT);
					SetOutput(O_ALARM_BUZZER);	  			//蜂鸣器
					ResetOutput(O_WAIT_YELLOW_LIGHT); 	//黄灯										   
					break;

				case AUTO_ERROR:
					SetOutput(O_RUN_GREEN_LIGHT);
					SetOutput(O_WAIT_YELLOW_LIGHT);					
					ResetOutput(O_ALARM_RED_LIGHT); 		//红灯
					ResetOutput(O_ALARM_BUZZER); 				//蜂鸣器
					break;

				default:
					break;
			}
		}
		else if(OffLine_Flag == FALSE && OnLineCommunicated_Flag == TRUE)
		{//不是脱机状态，已经通过信，则马上亮灯
			gs_AutoStatue=g_AutoStatue;
			switch(g_AutoStatue)
			{
				case AUTO_RUNNING:
					SetOutput(O_WAIT_YELLOW_LIGHT);	   	//黄灯
					SetOutput(O_ALARM_RED_LIGHT);	   		//红灯
					SetOutput(O_ALARM_BUZZER);	   			//蜂鸣器
					ResetOutput(O_RUN_GREEN_LIGHT);  		//绿灯
					break;

				case AUTO_PAUSE:
					if((Error_Status != ERROR_HAPPEN) && (g_Auto_ActionTimeOut_Flag !=TRUE))
					{
						SetOutput(O_RUN_GREEN_LIGHT);
						SetOutput(O_ALARM_RED_LIGHT);
						SetOutput(O_ALARM_BUZZER);	  		//蜂鸣器
						ResetOutput(O_WAIT_YELLOW_LIGHT); //黄灯
					}
					else	//动作超时情况下，红灯亮
					{
						gs_Error_Status=Error_Status;
						SetOutput(O_RUN_GREEN_LIGHT);
						SetOutput(O_WAIT_YELLOW_LIGHT);
						SetOutput(O_ALARM_BUZZER);	  		//蜂鸣器
						ResetOutput(O_ALARM_RED_LIGHT); 	//红灯
					}
					break;

				case AUTO_WAITE:
					SetOutput(O_RUN_GREEN_LIGHT);
					SetOutput(O_ALARM_RED_LIGHT);
					SetOutput(O_ALARM_BUZZER);	  			//蜂鸣器
					ResetOutput(O_WAIT_YELLOW_LIGHT); 	//黄灯										   
					break;

				case AUTO_ERROR:
					SetOutput(O_RUN_GREEN_LIGHT);
					SetOutput(O_WAIT_YELLOW_LIGHT);					
					ResetOutput(O_ALARM_RED_LIGHT); 		//红灯
					ResetOutput(O_ALARM_BUZZER); 				//蜂鸣器
					break;

				default:
					break;
			}
		}
	}
	
	if(Error_Status != gs_Error_Status)
	{
		gs_Error_Status = Error_Status;
		if(Error_Status == ERROR_EMERG_HAPPEN)
		{		   
			SetOutput(O_RUN_GREEN_LIGHT);
			SetOutput(O_WAIT_YELLOW_LIGHT);					
			ResetOutput(O_ALARM_RED_LIGHT); 				//红灯
			ResetOutput(O_ALARM_BUZZER); 					//蜂鸣器	 
		}
		else 
		{	     	   
			SetOutput(O_RUN_GREEN_LIGHT);
			SetOutput(O_ALARM_RED_LIGHT);
			SetOutput(O_ALARM_BUZZER);							//蜂鸣器
			ResetOutput(O_WAIT_YELLOW_LIGHT); 			//黄灯
			if(g_Auto_WorkFinished_Flag == TRUE || g_Auto_CJWorkFinished_Flag == TRUE)//加工完成状态
			{
				ResetOutput(O_ALARM_BUZZER);					//蜂鸣器
			}
			else
			{
				SetOutput(O_ALARM_BUZZER);						//蜂鸣器
			}
		}
	}
	
	if(Temp_OUT_Switch_Parameter[O_ORIGINED_LIGHT] == 1)
	{
		if(Origin_Backed == TRUE)
		{
			ResetOutput(O_ORIGINED_LIGHT);
		}
		else
		{
			SetOutput(O_ORIGINED_LIGHT);
		}
	}
}

/**************************************************************************************************
**  函数名：  Servo_Stop(u8)
**	输入参数：无
**	输出参数：无
**	函数功能：伺服器停止控制函数
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void Servo_Stop(u8 Axis)
{
	ServoStopSet_PDO(Axis_To_ID(Axis));
	Pulse_StopRequest[Axis] = 1;
	if(ServoWorkModeRead(Axis_To_ID(Axis)) != SERVO_MODE_CYC_CSP && ServoWorkModeRead(Axis_To_ID(Axis)) != SERVO_MODE_CYC_CSV)
	{
		AxisMoveFlag[Axis] = 0;
	}
	Flag_Keep_Move[Axis] = FALSE;
}

/**************************************************************************************************
**  函数名：  Robot_Reset()
**	输入参数：无
**	输出参数：无
**	函数功能：机器复位完成后，相关参数复位到初始值
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void Robot_Reset()
{
	u16 i = 0;
	
	//报警相关变量复位
	g_Auto_ActionError_Flag = FALSE;			//运行出错报警标志复位
	g_Auto_ActionTimeOut_Flag = FALSE;		//运行超时报警标志复位
	MD_PositionErr_Flag = FALSE;					//码垛位置计算错误标志位
	
	Cancle_Genaral_Warning = FALSE;				//取消报警操作标志复位
	Error_Status = NO_ERROR;							//报警状态复位
	Robot_Error_Data[0] = 0;							//紧急报警复位
	for(i=1; i<10; i++)
	{//普通报警复位
		Robot_Error_Data[i] = 0;
	}
	
	//生产相关变量复位
	g_Auto_WorkFinished_Flag = FALSE;				//产量设置加工完成标志复位
	g_Auto_CJWorkFinished_Flag = FALSE;			//抽检设置数完成标志复位
	
	//轴运动相关参数复位
	if(JDZ_Parameter.Switch)		//绝对值模式
	{}
	else
	{
//		for(i=0; i<Axis_Num; i++)
//		{
//			m_PulseTotalCounter[i] = MINROBOTPOSITION + JXS_Parameter.OrignOffset[i] * Step_Coefficient[i] / 100;		//复位完成后当前位置设置为临时原点
//			
//			if(JXS_Parameter.OriginDir[i] == 0)
//			{//设置轴的默认运动方向
//				Axsis_Move_Direction[i] = POSITIVE;
//			}
//			else
//			{
//				Axsis_Move_Direction[i] = NEGATIVE;
//			}
//		}
	}
	
	Axsis_Chosen = X_Axsis;									//设置默认选中轴为X轴
	Work_Status = AUTO_WORK_MODE;						//机械手工作模式为自动模式
		
	Action_Delay_Flag = FALSE;							//动作延时完成
	Action_Done_Flag = FALSE;								//伺服器动作完成标志
	
	DIR485_L;	   														//485默认接收状态
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
