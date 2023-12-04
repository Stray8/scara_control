/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f4xx.h"
#include "SignalWatch.h"
#include "StatusControl.h"
#include "Manual.h"
#include "Auto.h"
#include "Auto_2.h"
#include "w25qxx.h"
#include "BackToOrigin.h"
#include "JDZ.h"
#include "in.h"
#include "out.h"
#include "Error.h"
#include "Usart.h"
#include "Parameter.h"                  
#include "SpeedControl.h"
#include "ActionOperate.h"
#include "ExtendCom.h"

static u8 Watch_Usart_Data[50]={0};

/**************************************************************************************************
**  函数名：  WatchCommand()
**	输入参数：无
**	输出参数：无
**	函数功能：监视命令
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/						 
void WatchCommand()
{
	u8 i = 0;
	
	for(i=0; i<12; i++)
	{//缓冲区清零
		Watch_Usart_Data[i] = 0;
	}
	
	switch(UsartReceiveData[1])
	{
		case IO_DEBUG_INPUT1://读取输入状态，X0-X14 15位   
			Watch_Usart_Data[0] = Input_Detect_Status[0];
			Watch_Usart_Data[1] = Input_Detect_Status[1];
			Watch_Usart_Data[2] = Input_Detect_Status[2];
			Watch_Usart_Data[3] = Input_Detect_Status[3];
			Watch_Usart_Data[4] = Input_Detect_Status[4];
			USART1_SendData(5,0xE2,&Watch_Usart_Data[0]);
			break;
		case IO_DEBUG_OUTPUT1_LCD://读取输入状态，X0-X11
			Watch_Usart_Data[0] = Output_Status[0];
			Watch_Usart_Data[1] = Output_Status[1];
			Watch_Usart_Data[2] = Output_Status[2];
			Watch_Usart_Data[3] = Output_Status[3];
			Watch_Usart_Data[4] = Output_Status[4];
			USART1_SendData(5,0xE5,&Watch_Usart_Data[0]);
			break;
		case ROBOT_ORIGINED://查询回原点情况
			Watch_Usart_Data[0] = Origin_Backed;
			USART1_SendData(1,0xE8,&Watch_Usart_Data[0]);
			break;
		case ACTION_RESET_SCAN://返回复位状态值
			Watch_Usart_Data[0] = Robot_Auto_Reset;
			USART1_SendData(1,0xE9,&Watch_Usart_Data[0]);
			break;
		case ALARM_CLEAR://清除报警
			Cancle_Genaral_Warning=TRUE;			   
			USART1_SendData(1,0xEA,&Watch_Usart_Data[0]);
			break;
		case ROBOT_STATUS://读取工作状态及报警信息
			Watch_Usart_Data[0] = Work_Status;		   			//机械手状态
			Watch_Usart_Data[1] = g_AutoStatue;		   			//机械手运行状态
			Watch_Usart_Data[2] = AxisMoveFlag[X_Axsis]|AxisMoveFlag[Z_Axsis]|AxisMoveFlag[L_Axsis]|AxisMoveFlag[O_Axsis];		   //是否在发脉冲
			Watch_Usart_Data[3] = Origin_Backed;
			Watch_Usart_Data[4] = Robot_Error_Data[0];  	//报警信息
			Watch_Usart_Data[5] = Robot_Error_Data[1]; 
			Watch_Usart_Data[6] = Robot_Error_Data[2]; 
			Watch_Usart_Data[7] = Robot_Error_Data[3];
			Watch_Usart_Data[8] = Robot_Error_Data[4];
			Watch_Usart_Data[9] = Robot_Error_Data[5];
			Watch_Usart_Data[10] = Robot_Error_Data[6];
			Watch_Usart_Data[11] = Robot_Error_Data[7];
			Watch_Usart_Data[12] = Robot_Error_Data[8];
			Watch_Usart_Data[13] = Robot_Error_Data[9];
			Watch_Usart_Data[14] = Robot_Error_Data[10];
			Watch_Usart_Data[15] = Robot_Error_Data[11];
			Watch_Usart_Data[16] = Robot_Error_Data[12];
			Watch_Usart_Data[17] = Robot_Error_Data[13];
			Watch_Usart_Data[18] = Robot_Error_Data[14];
			Watch_Usart_Data[19] = Program_RunTime;
			Watch_Usart_Data[20] = Program_RunTime>>8;
			Watch_Usart_Data[21] = Program_RunTime>>16;
			Watch_Usart_Data[22] = Program_RunTime>>24;
			USART1_SendData(23,0xEB,&Watch_Usart_Data[0]);
			break;
		case AUTO_PARAMETER://读取当前产量等
			if(Action_Step_Run_Num >= 1)
			{
				Watch_Usart_Data[0] = g_Auto_PresentLine+Action_Step_Run_Num - 1;
			}
			else
			{
				Watch_Usart_Data[0] = g_Auto_PresentLine;
			}
			Watch_Usart_Data[1] = SC_Parameter.SC_Num;
			Watch_Usart_Data[2] = SC_Parameter.SC_Num>>8;
			Watch_Usart_Data[3] = SC_Parameter.SC_Num>>16;
			Watch_Usart_Data[4] = SC_Parameter.SC_Num>>24;  
			Watch_Usart_Data[5] = SC_Parameter.LJ_Num;
			Watch_Usart_Data[6] = SC_Parameter.LJ_Num>>8;
			Watch_Usart_Data[7] = SC_Parameter.LJ_Num>>16;
			Watch_Usart_Data[8] = SC_Parameter.LJ_Num>>24;
			Watch_Usart_Data[9] = SC_Parameter.NG_Num;
			Watch_Usart_Data[10] = SC_Parameter.NG_Num>>8;
			Watch_Usart_Data[11] = SC_Parameter.NG_Num>>16;
			Watch_Usart_Data[12] = SC_Parameter.NG_Num>>24;   
			Watch_Usart_Data[13] = SC_Parameter.CJ_Num;
			Watch_Usart_Data[14] = SC_Parameter.CJ_Num>>8;
			Watch_Usart_Data[15] = SC_Parameter.CJ_Num>>16;
			Watch_Usart_Data[16] = SC_Parameter.CJ_Num>>24;
			Watch_Usart_Data[17] = sMD_RunPara.curGood;
			Watch_Usart_Data[18] = sMD_RunPara.curLayer;
			Watch_Usart_Data[19] = sMD_RunPara.curNum;
			W25QXX_Read(&Watch_Usart_Data[20], P_MD_PARA_HEAD + (sMD_RunPara.curGood - 1) * P_MD_GOOD_LEN, 12);
			Watch_Usart_Data[32] = SC_Parameter.RW_Num;
			Watch_Usart_Data[33] = SC_Parameter.RW_Num>>8;
			Watch_Usart_Data[34] = SC_Parameter.RW_Num>>16;
			Watch_Usart_Data[35] = SC_Parameter.RW_Num>>24;
			USART1_SendData(36,0xEC,&Watch_Usart_Data[0]);
			break;
		case USER_CURNUM://用户变量
			for(i=0; i<USER_NUM; i++)
			{	
				Watch_Usart_Data[0 + i*4] = USER_Parameter.CURR_Num[i];
				Watch_Usart_Data[1 + i*4] = USER_Parameter.CURR_Num[i]>>8;
				Watch_Usart_Data[2 + i*4] = USER_Parameter.CURR_Num[i]>>16;
				Watch_Usart_Data[3 + i*4] = USER_Parameter.CURR_Num[i]>>24;
			}
			USART1_SendData(32,0xF8,&Watch_Usart_Data[0]);
			break;
		case X_AXSIS_POSITION://读取X轴坐标
			Watch_Usart_Data[0] =  m_PulseTotalCounter[X_Axsis];
			Watch_Usart_Data[1] = (m_PulseTotalCounter[X_Axsis])>>8;
			Watch_Usart_Data[2] = (m_PulseTotalCounter[X_Axsis])>>16;
			Watch_Usart_Data[3] = (m_PulseTotalCounter[X_Axsis])>>24;
			USART1_SendData(4,0xED,&Watch_Usart_Data[0]);
			break;
		case L_AXSIS_POSITION://读取L轴坐标
			Watch_Usart_Data[0] =  m_PulseTotalCounter[L_Axsis];
			Watch_Usart_Data[1] = (m_PulseTotalCounter[L_Axsis])>>8;
			Watch_Usart_Data[2] = (m_PulseTotalCounter[L_Axsis])>>16;
			Watch_Usart_Data[3] = (m_PulseTotalCounter[L_Axsis])>>24;
			USART1_SendData(4,0xEE,&Watch_Usart_Data[0]);
			break;
		case Z_AXSIS_POSITION://读取Z轴坐标
			Watch_Usart_Data[0] =  m_PulseTotalCounter[Z_Axsis];
			Watch_Usart_Data[1] = (m_PulseTotalCounter[Z_Axsis])>>8;
			Watch_Usart_Data[2] = (m_PulseTotalCounter[Z_Axsis])>>16;
			Watch_Usart_Data[3] = (m_PulseTotalCounter[Z_Axsis])>>24;
			USART1_SendData(4,0xEF,&Watch_Usart_Data[0]);
			break;
		case O_AXSIS_POSITION://读取O轴坐标
			Watch_Usart_Data[0] =  m_PulseTotalCounter[O_Axsis];
			Watch_Usart_Data[1] = (m_PulseTotalCounter[O_Axsis])>>8;
			Watch_Usart_Data[2] = (m_PulseTotalCounter[O_Axsis])>>16;
			Watch_Usart_Data[3] = (m_PulseTotalCounter[O_Axsis])>>24;
			USART1_SendData(4,0xF0,&Watch_Usart_Data[0]);
			break;
		case UV_AXSIS_POSITION://读取UV轴坐标
			Watch_Usart_Data[0] =  m_PulseTotalCounter[U_Axsis];
			Watch_Usart_Data[1] = (m_PulseTotalCounter[U_Axsis])>>8;
			Watch_Usart_Data[2] = (m_PulseTotalCounter[U_Axsis])>>16;
			Watch_Usart_Data[3] = (m_PulseTotalCounter[U_Axsis])>>24;
			Watch_Usart_Data[4] =  m_PulseTotalCounter[V_Axsis];
			Watch_Usart_Data[5] = (m_PulseTotalCounter[V_Axsis])>>8;
			Watch_Usart_Data[6] = (m_PulseTotalCounter[V_Axsis])>>16;
			Watch_Usart_Data[7] = (m_PulseTotalCounter[V_Axsis])>>24;
			USART1_SendData(8,0xFD,&Watch_Usart_Data[0]);
			break;
		case ROBOT_PRE_STATUS://读取机械手运行状态
			Watch_Usart_Data[0] = Work_Status;
			Watch_Usart_Data[1] = AxisMoveFlag[X_Axsis]|AxisMoveFlag[Z_Axsis]|AxisMoveFlag[L_Axsis]|AxisMoveFlag[O_Axsis];
			USART1_SendData(2,0xF1,&Watch_Usart_Data[0]);			   
			break;
		case ROBOT_DEBUG_STATUS://读取当前执行命令的行号
			Watch_Usart_Data[0] = g_Auto_PresentLine;
			USART1_SendData(2,0xF2,&Watch_Usart_Data[0]);			   
			break;
		case DELETE_POINT_STATUS://恢复出厂设置删除点反馈
			if(All_Axis_Point_Deleted_Flag == 1)
			{
				Watch_Usart_Data[0] = All_Axis_Point_Deleted_Flag;
				USART1_SendData(1,0xF3,&Watch_Usart_Data[0]);
			}
			break;
		case DELETE_PROGRAM_STATUS://恢复出厂设置删除程序反馈-DPF
			if(All_Program_Deleted_Flag == 1)
			{
				Watch_Usart_Data[0] = All_Program_Deleted_Flag;
				USART1_SendData(1,0xF4,&Watch_Usart_Data[0]);
			}
			break;
		case DELETE_MD_STATUS://恢复出厂设置删除码垛反馈
			if(All_MD_Deleted_Flag == 1)
			{
				Watch_Usart_Data[0] = All_MD_Deleted_Flag;
				USART1_SendData(1,0xF9,&Watch_Usart_Data[0]);
			}
			break;
		case ORIGIN_SETTING_STATUS:		//原点设置状态反馈
			Watch_Usart_Data[0] = JDZ_Parameter.OriginSetting[JDZ_Origin_Setting_Axis_Num - 1];
			USART1_SendData(1,0xF5,&Watch_Usart_Data[0]);
			break; 
		case ORIGIN_RESETTING_STATUS:		//原点重置状态反馈
			Watch_Usart_Data[0] = JDZ_Parameter.OriginSetting[JDZ_Origin_Resetting_Axis_Num - 1];
			USART1_SendData(1,0xF6,&Watch_Usart_Data[0]);
			break; 
		case MACHINE_ORIGIN_STATUS:		//机械回零状态反馈
			Watch_Usart_Data[0] = Axsis_Origin_Backed[JDZ_Machine_Ori_Axis_Num - 1];
			USART1_SendData(1,0xF7,&Watch_Usart_Data[0]);
			break;
		case EXTENDCOM_STATUS://机械手485通信状态变量
			Watch_Usart_Data[0] = ExtendProgramNum;
			Watch_Usart_Data[1] = ExtendProgramNum>>8;
			Watch_Usart_Data[2] = ExtendYieldChange;
			Watch_Usart_Data[3] = ExtendPosChangeNum;
			Watch_Usart_Data[4] = ExtendStateChange;
			Watch_Usart_Data[5] = ExtendCancleAlarm;
		  Watch_Usart_Data[6] = ExtendSerialNum;
			USART1_SendData(7,0xFA,&Watch_Usart_Data[0]);
			ExtendStateChange = 0;
			ExtendCancleAlarm = 0;
			break;
		
		case ONE_POINT_POSITON://一个点的位置信息
			if(ExtendPosChangeNum > 0)
			{
				Watch_Usart_Data[0] = Manul_Save_Point[ExtendPosChangeNum-1].Point_X;
				Watch_Usart_Data[1] = Manul_Save_Point[ExtendPosChangeNum-1].Point_X>>8;
				Watch_Usart_Data[2] = Manul_Save_Point[ExtendPosChangeNum-1].Point_X>>16;
				Watch_Usart_Data[3] = Manul_Save_Point[ExtendPosChangeNum-1].Point_X>>24;
				Watch_Usart_Data[4] = Manul_Save_Point[ExtendPosChangeNum-1].Point_L;
				Watch_Usart_Data[5] = Manul_Save_Point[ExtendPosChangeNum-1].Point_L>>8;
				Watch_Usart_Data[6] = Manul_Save_Point[ExtendPosChangeNum-1].Point_L>>16;
				Watch_Usart_Data[7] = Manul_Save_Point[ExtendPosChangeNum-1].Point_L>>24;
				Watch_Usart_Data[8] = Manul_Save_Point[ExtendPosChangeNum-1].Point_Z;
				Watch_Usart_Data[9] = Manul_Save_Point[ExtendPosChangeNum-1].Point_Z>>8;
				Watch_Usart_Data[10] = Manul_Save_Point[ExtendPosChangeNum-1].Point_Z>>16;
				Watch_Usart_Data[11] = Manul_Save_Point[ExtendPosChangeNum-1].Point_Z>>24;
				Watch_Usart_Data[12] = Manul_Save_Point[ExtendPosChangeNum-1].Point_O;
				Watch_Usart_Data[13] = Manul_Save_Point[ExtendPosChangeNum-1].Point_O>>8;
				Watch_Usart_Data[14] = Manul_Save_Point[ExtendPosChangeNum-1].Point_O>>16;
				Watch_Usart_Data[15] = Manul_Save_Point[ExtendPosChangeNum-1].Point_O>>24;
				USART1_SendData(16,0xFB,&Watch_Usart_Data[0]);
				ExtendPosChangeNum = 0;	
			}
			break;

		case EXTEND_SERIAL_NUM://序列号
			if(ExtendSerialNum > 0)
			{
				for(i=0;i<12;i++)
				{
					Watch_Usart_Data[i] = Internet_Parameter.Sequence[i];
				}
				USART1_SendData(12,0xFC,&Watch_Usart_Data[0]);
				ExtendSerialNum = 0;	
			}
			break;
		case MD_CURLAYER_CURNUM://当前物品的当前个数和当前层
			if(UsartReceiveData[2] > 0)
			{
				Watch_Usart_Data[0] = sMD_FlashCurLayer[UsartReceiveData[2] - 1];
				Watch_Usart_Data[1] = sMD_FlashCurNum[UsartReceiveData[2] - 1];
			}
			else
			{
				Watch_Usart_Data[0] = 1;
				Watch_Usart_Data[1] = 1;
			}
			USART1_SendData(2,0xFE,&Watch_Usart_Data[0]);
			break;
		default:
			break; 
	}
}
						            
/**************************************************************************************************
**  函数名：  WatchFunction()
**	输入参数：无
**	输出参数：无
**	函数功能：解码串口接收数据
**	备注：	  无
**  作者：         
**  开发日期： 
***************************************************************************************************/
void OrderDecoding()
{ 
	switch(UsartReceiveData[0])
	{
		case P_ROBOT_ENABLE_A_ORIGIN:		//机械手使能和回原点
			RobotEnableOrigin();
			break;
		case P_WORK_MODE:								//机械手工作模式
			WorkMode();	
			break;
		case P_AUTO_RUN:
			AutoRun();	
			break;
		case P_FREE_PROGRAM_SEND:
			FreeProgramSend();	
			break;
		case P_WATCH_COMMAND:						//监视命令
			WatchCommand();
			break;
		case P_READ_IIC:
			ReadIICData();	
			break; 
		case P_IO_DEBUG_OUTPUT1:
			IODebugOutput1();
			break;
		case P_IO_DEBUG_OUTPUT2:
			IODebugOutput2();
			break;
		case P_IO_DEBUG_OUTPUT3:
			IODebugOutput3();
			break;
		case P_MANUL_DEBUG:
			ManulDebug();
			break;
		case P_PARAMETER_ORDER:
			ParameterOrder();
			break;
		case P_SYSTEM_SET_SEND:
			Write_System_Set_IIC();
			break;
		case MDPARA_COPY_SEND:
			Write_MDPara_Copy_IIC();
			break;
		default: 
			break;
	}
	UsartAcknowledge(UsartReceiveData[0]);
}

/**************************************************************************************************
**  函数名：  WatchFunction()
**	输入参数：无
**	输出参数：无
**	函数功能：实现监视功能，并实时要求手控器变换显示的内容
**	备注：	  无
**  作者：         
**  开发日期： 
***************************************************************************************************/
void WatchFunction(void)
{
	u8 i = 0;
	
	for(i=0; i<OUTPUT_NUM; i++)
	{
		if(m_PulseOutputSta[i] > 0)
		{//处理脉宽命令的脉宽输出计时
			if(m_PulseOutputStartTime[i] == m_PulseOutputEndTime[i])
			{
				m_PulseOutputSta[i] = 0;
			}
			if((m_PulseOutputStartTime[i] < m_PulseOutputEndTime[i] && (m_SystemTimeCounter < m_PulseOutputStartTime[i] || m_SystemTimeCounter > m_PulseOutputEndTime[i])) || \
					(m_PulseOutputStartTime[i] > m_PulseOutputEndTime[i] && (m_SystemTimeCounter < m_PulseOutputStartTime[i] && m_SystemTimeCounter > m_PulseOutputEndTime[i])))
			{
				if(m_PulseOutputSta[i] == V_RESET)
				{
					SetSingle(60,i,0);															//置位端口
				}
				else
				{
					SetSingle(i,60,0);															//复位端口
				}
				
				m_PulseOutputSta[i] = 0;
				m_PulseOutputStartTime[i] = m_SystemTimeCounter;
				m_PulseOutputEndTime[i] = m_SystemTimeCounter;
			}
		}
	}
	
	if(USART1ErrorFlag	== TRUE)
	{
		ReceiveDataCounter = 0;
		NewOrder = FALSE;
		for(i=0; i<USART_BUFFER_SIZE; i++)
		{
			ReceiveDataBuffer[i] = 0;
		}
	}
	else
	{
		if(NewOrder == TRUE )
		{
			g_USART_Delay_Timer = 0;
			UsartDataDecode();				//串口接收数据处理
			OrderDecoding();					//命令解析
			NewOrder = FALSE;
			for(i=0; i<100; i++)				//清零处理后数据
			{
				UsartReceiveData[i] = 0;
			}		    
		}
	}
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
