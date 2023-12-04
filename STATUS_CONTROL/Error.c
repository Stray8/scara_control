/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f4xx.h"
#include "Usart.h" 
#include "Error.h"
#include "in.h" 
#include "out.h"
#include "w25qxx.h"
#include "Manual.h"
#include "Auto.h"
#include "Auto_2.h"
#include "SpeedControl.h"
#include "signalWatch.h"
#include "StatusControl.h"
#include "Parameter.h"
#include "BackToOrigin.h"
#include "JDZ.h"
#include "CANopen.h"
#include "ActionOperate.h"
#include "EtherCAT_App.h"
#include "stdlib.h"
#include "ExtendCom.h"

/*----- 全局变量，错误发生标记位 -----*/
u8 Error_Status = NO_ERROR;							//发生错误标志
u8 Cancle_Genaral_Warning = FALSE;  		//取消当前报警编号

/*- 报警数据，每一位保存一种类型报警，1表示报警，0表示无报警                -*/
/* Data[0] 急停   轴   机床故障 软限位 安全区 气压异常 润滑报警   预留 */
/* [7:0]    1      1      1      1       1       1        1         1 */
u8 Robot_Error_Data[15] = {0};					 

//[n]记录IO是否需要常开常闭功能
u16 IO_Input_waittime[30] = {0};			//输入检测超时时间
u16 IO_Input_keepMin[30] = {0};			//输入保持时间-下限
u16 IO_Input_keepMax[30] = {0};			//输入保持时间-上限
u16 IO_Sign_On_Off[30] = {0};					//输入信号常开常闭标志，0常开，1常闭

//[n]记录输出IO复位选择
u16 OutPut_BeforeOrigin[30] = {0};		   	 //回零前选择
u16 OutPut_AfterOrigin[30] = {0};			     //回零后选择
u16 OutPut_Common_Alarm[30] = {0};	 //普通报警
u16 OutPut_Emerge_Alarm[30] = {0};	 //急停报警
u16 OutPut_Pause[30] = {0};			     //暂停
u16 OutPut_Stop[30] = {0};			     //停止

//伺服单独报警开始检测定时器，防止开机直接报警
u32 Axsis_Error_Count = 1;						//伺服单独报警开始检测定时器，开机就开始计数
u8 Axsis_Error_Permit_Flag = FALSE;		//伺服单独报警开始检测允许标志
u8 g_Auto_ActionConflict_Flag = FALSE; 							//程序间动作重复

/**************************************************************************************************
**  函数名：  CloseTotalMotorError(u8 Axsis)
**	输入参数：Axsis 轴编号 
**	输出参数：无
**	函数功能：出错时，关闭所有电机的输出
**	备注：
**  作者：	Lin
**  开发日期： 
***************************************************************************************************/
void CloseTotalMotorError(void)
{
	u16 i = 0;
	
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		Servo_Stop(i);
	}
}

/**************************************************************************************************
**  函数名：  EmergencyStopJudge()
**	输入参数：无
**	输出参数：无
**	函数功能：急停判断
**	备注：	  急停按下，为高电平有效信号
**  作者：          
**  开发日期： 
***************************************************************************************************/
void EmergencyStopJudge(void)
{
	if(ReadEmergencyStop())
	{//检测到急停为按下状态
		Robot_Error_Data[0] = Robot_Error_Data[0] | 0x80;
		CloseTotalMotorError();
	}
	else if((Robot_Error_Data[0] & 0x80) && ExtendEmergencyStop == 0)
	{//急停键为释放状态
		Cancle_Genaral_Warning = TRUE;
	}
}

/**************************************************************************************************
**  函数名：  SoftLimitJudge()
**	输入参数：无
**	输出参数：无
**	函数功能：软极限检测
**	备注：	  无
**  作者：          
**  开发日期： 
***************************************************************************************************/
void SoftLimitJudge(void)
{	
	if(Origin_Backed == TRUE)
	{//回零成功后才会检测软限位、执行机械回零指令时不检测软限位
		if((AxisMoveFlag[X_Axsis] | AxisMoveFlag[Z_Axsis] | AxisMoveFlag[L_Axsis] | AxisMoveFlag[O_Axsis]) != 0)
		{
			if(Robot_SoftLimit[X_Axsis].Switch_Limit && ((m_PulseTotalCounter[X_Axsis] + JDZ_AllowError < Axsis_Minlength[X_Axsis] && Axsis_Move_Direction[X_Axsis] == NEGATIVE) ||
				(m_PulseTotalCounter[X_Axsis] > Axsis_Maxlength[X_Axsis] + JDZ_AllowError && Axsis_Move_Direction[X_Axsis] == POSITIVE)) && Program_Axis_Origin_Flag[X_Axsis] == FALSE)	//X轴
			{
				Robot_Error_Data[0] = Robot_Error_Data[0] | 0x10;
				CloseTotalMotorError();
			}
			else if(Robot_SoftLimit[L_Axsis].Switch_Limit && ((m_PulseTotalCounter[L_Axsis] + JDZ_AllowError < Axsis_Minlength[L_Axsis] && Axsis_Move_Direction[L_Axsis] == NEGATIVE) ||
				(m_PulseTotalCounter[L_Axsis] > Axsis_Maxlength[L_Axsis] + JDZ_AllowError && Axsis_Move_Direction[L_Axsis] == POSITIVE)) && Program_Axis_Origin_Flag[L_Axsis] == FALSE)	//L轴
			{
				Robot_Error_Data[0] = Robot_Error_Data[0] | 0x10;
				CloseTotalMotorError();
			}
			else if(Robot_SoftLimit[Z_Axsis].Switch_Limit && ((m_PulseTotalCounter[Z_Axsis] + JDZ_AllowError < Axsis_Minlength[Z_Axsis] && Axsis_Move_Direction[Z_Axsis] == NEGATIVE) ||
				(m_PulseTotalCounter[Z_Axsis] > Axsis_Maxlength[Z_Axsis] + JDZ_AllowError && Axsis_Move_Direction[Z_Axsis] == POSITIVE)) && Program_Axis_Origin_Flag[Z_Axsis] == FALSE)	//Z轴
			{
				Robot_Error_Data[0] = Robot_Error_Data[0] | 0x10;
				CloseTotalMotorError();
			}
			else if(Robot_SoftLimit[O_Axsis].Switch_Limit && ((m_PulseTotalCounter[O_Axsis] + JDZ_AllowError < Axsis_Minlength[O_Axsis] && Axsis_Move_Direction[O_Axsis] == NEGATIVE) ||
				(m_PulseTotalCounter[O_Axsis] > Axsis_Maxlength[O_Axsis] + JDZ_AllowError && Axsis_Move_Direction[O_Axsis] == POSITIVE)) && Program_Axis_Origin_Flag[O_Axsis] == FALSE)	//O轴
			{
				Robot_Error_Data[0] = Robot_Error_Data[0] | 0x10;
				CloseTotalMotorError();
			}
		}
		
		if(Robot_SoftDistance.MinDistance > MINROBOTPOSITION && Robot_SoftDistance.MaxDistance > Robot_SoftDistance.MinDistance)
		{//最小安全距离>0 且 最大距离大于最小距离时，才进行检测；最小距离设为0，则认为关闭状态
			if((Robot_SoftDistance.MaxDistance + 2 * MINROBOTPOSITION) < (Robot_SoftDistance.MinDistance + m_PulseTotalCounter[X_Axsis] + m_PulseTotalCounter[O_Axsis]) && \
				((AxisMoveFlag[X_Axsis] == TRUE && Axsis_Move_Direction[X_Axsis] == POSITIVE) || (AxisMoveFlag[O_Axsis] == TRUE && Axsis_Move_Direction[O_Axsis] == POSITIVE)))
			{//最大距离 < 最小安全距离+X轴位置+O轴位置，两个最小值用于补偿坐标计数器
				Robot_Error_Data[0] = Robot_Error_Data[0] | 0x20;
				CloseTotalMotorError();
			}
		}
		
		if((AxisMoveFlag[U_Axsis] | AxisMoveFlag[V_Axsis]) != 0)
		{
			if(ExtendAix_Parameter[U_Ext_Axsis].E_Origin_Set == 1 && ((m_PulseTotalCounter[U_Axsis] + JDZ_AllowError < Axsis_Minlength[U_Axsis] && Axsis_Move_Direction[U_Axsis] == NEGATIVE) ||
				(m_PulseTotalCounter[U_Axsis] > Axsis_Maxlength[U_Axsis] + JDZ_AllowError && Axsis_Move_Direction[U_Axsis] == POSITIVE)))	//U轴
			{
				Robot_Error_Data[0] = Robot_Error_Data[0] | 0x10;
				CloseTotalMotorError();
			}
			else if(ExtendAix_Parameter[V_Ext_Axsis].E_Origin_Set == 1 && ((m_PulseTotalCounter[V_Axsis] + JDZ_AllowError < Axsis_Minlength[V_Axsis] && Axsis_Move_Direction[V_Axsis] == NEGATIVE) ||
				(m_PulseTotalCounter[V_Axsis] > Axsis_Maxlength[V_Axsis] + JDZ_AllowError && Axsis_Move_Direction[V_Axsis] == POSITIVE)))	//V轴
			{
				Robot_Error_Data[0] = Robot_Error_Data[0] | 0x10;
				CloseTotalMotorError();
			}
		}
	}
}

/**************************************************************************************************
**  函数名：  XAxsisError()
**	输入参数：无
**	输出参数：无
**	函数功能：X轴错误
**	备注：	  无
**  作者：     
**  开发日期：
***************************************************************************************************/
void XAxsisError(u8 alarm)
{
	if(alarm == 1)					          //读取X轴错误信号
	{	
		Robot_Error_Data[6] = Robot_Error_Data[6] | 0x01;
		ServoDisable_PDO(SERVO_NODE_ID_01_X);
	}
}

/**************************************************************************************************
**  函数名：  ZAxsisError()
**	输入参数：无
**	输出参数：无
**	函数功能：Z轴错误
**	备注：	  无
**  作者：      
**  开发日期：
***************************************************************************************************/
void ZAxsisError(u8 alarm)
{
	if(alarm == 1)					          //读取Z轴错误信号
	{
		Robot_Error_Data[6] = Robot_Error_Data[6] | 0x02;
		ServoDisable_PDO(SERVO_NODE_ID_03_Z);
	}
}

/**************************************************************************************************
**  函数名：  LAxsisError()
**	输入参数：无
**	输出参数：无
**	函数功能：L轴错误
**	备注：	  无
**  作者：      
**  开发日期：
***************************************************************************************************/
void YAxsisError(u8 alarm)
{
	if(alarm == 1)					          //读取Y轴错误信号
	{	  
		Robot_Error_Data[6] = Robot_Error_Data[6] | 0x04;
		ServoDisable_PDO(SERVO_NODE_ID_02_L);
	}
}

/**************************************************************************************************
**  函数名：  OAxsisError()
**	输入参数：无
**	输出参数：无
**	函数功能：O轴错误
**	备注：	  无
**  作者：       
**  开发日期：
***************************************************************************************************/
void OAxsisError(u8 alarm)
{
	if(alarm == 1)					          //读取O轴错误信号
	{	  
		Robot_Error_Data[6] = Robot_Error_Data[6] | 0x08;
		ServoDisable_PDO(SERVO_NODE_ID_04_O);
	}
}

/**************************************************************************************************
**  函数名：  MotorAlarmProcess()
**	输入参数：无
**	输出参数：无
**	函数功能：电机报警处理
**	备注：	  无
**  作者：     
**  开发日期：
***************************************************************************************************/
void MotorAlarmProcess(void)
{
	u8 alarm[SERVO_NODE_NUM] = {0};
	
	ServoMoveAlarmSta(alarm);
	
	if(Axsis_Error_Permit_Flag == TRUE)
	{//5s之后再去检测轴报警
		if(JXS_Parameter.AlarmSwitch[X_Axsis] == 1)
		{
			XAxsisError(alarm[X_Axsis]);		   //X轴报警
		}
		else
		{
			Robot_Error_Data[6] = Robot_Error_Data[6] & 0xfe;
		}
		
		if(JXS_Parameter.AlarmSwitch[L_Axsis] == 1)
		{
			YAxsisError(alarm[L_Axsis]);		   //Y轴报警
		}
		else
		{
			Robot_Error_Data[6] = Robot_Error_Data[6] & 0xfb;
		}
		
		if(JXS_Parameter.AlarmSwitch[Z_Axsis] == 1)
		{
			ZAxsisError(alarm[Z_Axsis]);		   //Z轴报警
		}
		else
		{
			Robot_Error_Data[6] = Robot_Error_Data[6] & 0xfd;
		}
		
		if(JXS_Parameter.AlarmSwitch[O_Axsis] == 1)
		{
			OAxsisError(alarm[O_Axsis]);		   //O轴报警
		}
		else
		{
			Robot_Error_Data[6] = Robot_Error_Data[6] & 0xf7;
		}
	}
}

/**************************************************************************************************
**  函数名：  CurrentWorkFinished()
**	输入参数：无
**	输出参数：无
**	函数功能：加工完成
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void  CurrentWorkFinished(void)
{
	if(g_Auto_WorkFinished_Flag)
	{
		Robot_Error_Data[1] = Robot_Error_Data[1] | 0x20;
	}
	else
	{
		Robot_Error_Data[1] = Robot_Error_Data[1] & 0xdf;
	}
}

/**************************************************************************************************
**  函数名：  CurrentCJWorkFinished()
**	输入参数：无
**	输出参数：无
**	函数功能：抽检加工完成
**	备注：	 
**  作者：       
**  开发日期：2021/01/18
***************************************************************************************************/
void  CurrentCJWorkFinished(void)
{
	 if(g_Auto_CJWorkFinished_Flag)
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x01;
	 }
	 else
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xfe;
	 }
}

/**************************************************************************************************
**  函数名：  AutoActionError()
**	输入参数：无
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void  AutoActionError(void)
{
	 if(g_Auto_ActionError_Flag)
	 {
		 if(MD_PositionErr_Flag==FALSE)
		 {
			Robot_Error_Data[1] = Robot_Error_Data[1] | 0x10;
		 }
		 else if(MD_PositionErr_Flag==TRUE)
		 {
			 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x02;
		 }
	 }
	 else
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xed;
	 }

}
/**************************************************************************************************
**  函数名：  AutoActionTimeOut()
**	输入参数：无
**	输出参数：无
**	函数功能：动作超时报警
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void  AutoActionTimeOut(void)
{
	 if(g_Auto_ActionTimeOut_Flag)
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x04;
	 }
	 else
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xfb;
	 }
}

/**************************************************************************************************
**  函数名：  AutoActionConflict()
**	输入参数：无
**	输出参数：无
**	函数功能：程序间有重复动作
**	备注：	 
**  作者：       
**  开发日期：
***************************************************************************************************/
void AutoActionConflict(void)
{
	 if(g_Auto_ActionConflict_Flag)
	 {
	 	 Robot_Error_Data[7] = Robot_Error_Data[7] | 0x04;
	 }
	 else
	 {
	 	 Robot_Error_Data[7] = Robot_Error_Data[7] & 0xfb;
	 }
}

/**************************************************************************************************
**  函数名：  ErrorOperate()
**	输入参数：无
**	输出参数：无
**	函数功能：报警操作
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void ErrorOperate(void)
{
	u8 i=0;
//	u8 Temp_data[4]={0};
	
	EmergencyStopJudge(); 					//急停检测
	MotorAlarmProcess();						//电机报警检测
//#ifdef HARDLIMITJUDGE_INPUT
//	HardLimitJudge();	   	  			//硬限位报警
//#endif
	SoftLimitJudge();		  					//软限位检测	 
	SafeAreaJudge();	      				//安全区检测--紧急
	AutoActionConflict();					//程序间有重复动作
	
	if(Work_Status == AUTO_WORK_MODE)
	{//自动模式下进行处理
		CurrentWorkFinished();	   		//加工完成
		CurrentCJWorkFinished();			//抽检数量完成
		AutoActionError();	       		//自动运行进入AUTO_ERROR
		AutoActionTimeOut();	  			//动作超时
	}
	
	//取消当前报警
	if(Cancle_Genaral_Warning == TRUE)
	{
		CurProgramRead(g_Run_Program_Num_Pre);							//选中程序更换为原有选中的程序
		
		//紧急、急停报警不直接清零，其他报警直接清零
		for(i=0; i<15; i++)
		{
			Robot_Error_Data[i] = 0;
		}
		
		//清除报警标志
		g_Auto_ActionError_Flag = FALSE;
		g_Auto_ActionTimeOut_Flag = FALSE;
		Cancle_Genaral_Warning = FALSE;
		g_Auto_ActionConflict_Flag = FALSE;
		MD_PositionErr_Flag = FALSE;

		if(g_Auto_WorkFinished_Flag)	
		{//清除当前生产产量参数
			SC_Parameter.SC_Num=0;
			g_Auto_WorkFinished_Flag = FALSE;
		}
		else if(g_Auto_CJWorkFinished_Flag)	
		{//清除当前抽检产量参数
			SC_Parameter.CJ_Num=0;
			g_Auto_CJWorkFinished_Flag = FALSE;	
		}
	}
	 
	for(i=0; i<15; i++)
	{
		if(Robot_Error_Data[i] != 0)
		{
			if(Robot_Error_Data[0] != 0 || Robot_Error_Data[6] != 0 || Robot_Error_Data[8] != 0)
			{//紧急停止类报警
				Error_Status = ERROR_EMERG_HAPPEN;	//0x02紧急报警
			}
			else
			{//普通暂停停止类报警
				Error_Status = ERROR_HAPPEN;				//0x01
			}
			break;
		}
		else
		{
			Error_Status = NO_ERROR;
		}
	}
	
	if(Error_Status==NO_ERROR)
	{//未发生错误
		
	}
	else
	{//发生错误
		if(Error_Status == ERROR_EMERG_HAPPEN) 
		{//急停报警，直接停止机械手
			for(i=0;i<OUTPUT_NUM;i++)
			{
				if(OutPut_Emerge_Alarm[i]==Emerge_Alarm)
				{
					SetOutput(i);//指示灯灭
				}
			}
			Cancle_Get_Position_Flag();
			CloseTotalMotorError();
			if((g_Program_Is_Debuging == FALSE) && (g_AutoStatue == AUTO_RUNNING || g_AutoStatue == AUTO_PAUSE)\
				&& (g_Auto_PresentLine != 0 && g_Auto_PresentLine != Free_Program_Operate.Num-1))
			{
				SC_Parameter.NG_Num++;
			}
			
			if(m_InterpCurveFlag == INTER_CURVE_NO && m_InterpAxisMoveFlag[X_Axsis] == 0 && m_InterpAxisMoveFlag[L_Axsis] == 0 \
					&& m_InterpAxisMoveFlag[Z_Axsis] == 0 && m_InterpAxisMoveFlag[O_Axsis] == 0 && m_InterpAxisMoveFlag[U_Axsis] == 0 && m_InterpAxisMoveFlag[V_Axsis] == 0)
			{//插补运动执行完成后，可以写FLASH
				AutoReset();
			}
		}
		else
		{//其他报警，按照暂停或停止处理
			for(i=0;i<OUTPUT_NUM;i++)
			{
				if(OutPut_Common_Alarm[i]==Common_Alarm)
				{
					SetOutput(i);//指示灯灭
				}
			}
			if(g_AutoStatue == AUTO_RUNNING)	//自由编程-调试 状态也满足条件,将进入暂停状态
			{
				g_Auto_Order_Pause = TRUE;
			}
			Linked_Move_Enable = DISABLE;
			Jog_Move_Enable = DISABLE;
			Back_Origin_Flag = FALSE;
		}
	}
}

/**************************************************************************************************
**  函数名：  HardLimitJudge()
**	输入参数：无
**	输出参数：无
**	函数功能：硬限位检测
**	备注：	  无
**  作者：       
**  开发日期：
***************************************************************************************************/
void HardLimitJudge(void)
{
	static u8 hardLimitStaInit = 0;
	static u8 hardMinLimitSta[Axis_Num] = {0};
	static u8 hardMaxLimitSta[Axis_Num] = {0};
	
	if(hardLimitStaInit == 0)
	{
		hardMinLimitSta[X_Axsis] = X_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
		hardMinLimitSta[L_Axsis] = Y_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
		hardMinLimitSta[Z_Axsis] = Z_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
		hardMinLimitSta[O_Axsis] = O_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
		
		hardMaxLimitSta[X_Axsis] = X_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
		hardMaxLimitSta[L_Axsis] = Y_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
		hardMaxLimitSta[Z_Axsis] = Z_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
		hardMaxLimitSta[O_Axsis] = O_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
		
		hardLimitStaInit = 1;
	}
	
	//X轴最小硬限位-X16
	if(Temp_IO_Switch_Parameter[I_DETECT_X_MIN_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_X_MIN_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//常开
			if(X_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 1 && hardMinLimitSta[X_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x01;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_X_MIN_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//常闭
			if(X_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 0 && hardMinLimitSta[X_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x01;
				CloseTotalMotorError();
			}
		}
	}
	
	//Y轴最小硬限位-X17
	if(Temp_IO_Switch_Parameter[I_DETECT_Y_MIN_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_Y_MIN_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//常开
			if(Y_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 1 && hardMinLimitSta[L_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x02;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_Y_MIN_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//常闭
			if(Y_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 0 && hardMinLimitSta[L_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x02;
				CloseTotalMotorError();
			}
		}
	}
	
	//Z轴最小硬限位-X18
	if(Temp_IO_Switch_Parameter[I_DETECT_Z_MIN_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_Z_MIN_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//常开
			if(Z_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 1 && hardMinLimitSta[Z_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x04;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_Z_MIN_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//常闭
			if(Z_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 0 && hardMinLimitSta[Z_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x04;
				CloseTotalMotorError();
			}
		}
	}
	
	//O轴最小硬限位-X19
	if(Temp_IO_Switch_Parameter[I_DETECT_O_MIN_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_O_MIN_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//常开
			if(O_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 1 && hardMinLimitSta[O_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x08;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_O_MIN_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//常闭
			if(O_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 0 && hardMinLimitSta[O_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x08;
				CloseTotalMotorError();
			}
		}
	}
	
	
	//X轴最大硬限位-X16
	if(Temp_IO_Switch_Parameter[I_DETECT_X_MAX_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_X_MAX_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//常开
			if(X_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 1 && hardMaxLimitSta[X_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x10;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_X_MAX_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//常闭
			if(X_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 0 && hardMaxLimitSta[X_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x10;
				CloseTotalMotorError();
			}
		}
	}
		
	//Y轴最大硬限位-X17
	if(Temp_IO_Switch_Parameter[I_DETECT_Y_MAX_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_Y_MAX_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//常开
			if(Y_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 1 && hardMaxLimitSta[L_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x20;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_Y_MAX_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//常闭
			if(Y_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 0 && hardMaxLimitSta[L_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x20;
				CloseTotalMotorError();
			}
		}
	}
	
	//Z轴最大硬限位-X18
	if(Temp_IO_Switch_Parameter[I_DETECT_Z_MAX_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_Z_MAX_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//常开
			if(Z_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 1 && hardMaxLimitSta[Z_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x40;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_Z_MAX_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//常闭
			if(Z_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 0 && hardMaxLimitSta[Z_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x40;
				CloseTotalMotorError();
			}
		}
	}
	
	//O轴最大硬限位-X19
	if(Temp_IO_Switch_Parameter[I_DETECT_O_MAX_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_O_MAX_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//常开
			if(O_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 1 && hardMaxLimitSta[O_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x80;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_O_MAX_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//常闭
			if(O_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 0 && hardMaxLimitSta[O_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x80;
				CloseTotalMotorError();
			}
		}
	}

	
	hardMinLimitSta[X_Axsis] = X_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
	hardMinLimitSta[L_Axsis] = Y_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
	hardMinLimitSta[Z_Axsis] = Z_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
	hardMinLimitSta[O_Axsis] = O_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
	
	hardMaxLimitSta[X_Axsis] = X_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
	hardMaxLimitSta[L_Axsis] = Y_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
	hardMaxLimitSta[Z_Axsis] = Z_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
	hardMaxLimitSta[O_Axsis] = O_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
}

/**************************************************************************************************
**  函数名：  Scan_TimeOut_IO()
**	输入参数：i_num 输入口编号
**	输出参数：无
**	函数功能：输入超时报警设置
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void Scan_TimeOut_IO(u8 i_num)
{
	if((i_num >= K_IOINSTRUCT_INPUT1) && (i_num <= K_IOINSTRUCT_INPUT30))//单对双
	{
		i_num -= 0x48;								//X0对应0x44
		Robot_Error_Data[2 + i_num/8] |= 0x01<<(i_num % 8);
	}	
}

/**************************************************************************************************
**  函数名：  Scan_TimeOut_OUTPUT()
**	输入参数：i_num 输入口编号
**	输出参数：无
**	函数功能：输入超时报警设置
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void Scan_TimeOut_OUTPUT(u8 o_num)
{
	if(o_num <= OUTPUT_NUM)
	{
		o_num -= 0x14;
		Robot_Error_Data[5] |= 0x80;
	}	
}


/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/





