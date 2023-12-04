/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Auto.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f4xx.h"
#include "Usart.h" 
#include "Manual.h"
#include "out.h"
#include "StatusControl.h"
#include "SpeedControl.h"
#include "Parameter.h"
#include "SignalWatch.h"
#include "w25qxx.h"
#include "Error.h"
#include "JDZ.h"
#include "in.h"
#include "ActionOperate.h"
#include "CANopen.h"

u8 Jog_Mode_Enable = DISABLE;		  				//寸动模式使能标志位
u8 Linked_Mode_Enable = DISABLE;	      	//连动模式使能标志位
u8 Jog_Move_Enable = DISABLE;			  			//寸动动作使能位
u8 Linked_Move_Enable = DISABLE	;	  			//连动动作使能位

u32 Jog_Pulse_Count = 50;			 	 					//寸动模式的每个动作的脉冲数
u32 Jog_Pulse_Count_Init = 50;		  			//初始寸动距离
u32 Linked_Pulse = MINROBOTPOSITION;
u8  Axis_Manul_Speed[Axis_Num + Ext_Axis_Num] = {10, 10, 10, 10, 10, 10};					//轴手动速度
u8  Axis_Manul_Speed_Temp[Axis_Num + Ext_Axis_Num] = {10, 10, 10, 10, 10, 10};		//在没有回原点时，用于保存原有轴手动速度
u16  Axis_Step_Distance[Axis_Num + Ext_Axis_Num] = {50,50,50,50,50,50};						//轴寸动距离1-100,默认50mm
u8  All_Axis_Point_Deleted_Flag = 0;		 																					//所有点已删除标志位

u8  KeyStaUpDown = 0;		 									//手动时的按键状态

/**************************************************************************************************
**  函数名：  ManualAxisMaxHardLimitSta()
**	输入参数：无
**	输出参数：无
**	函数功能：轴最大硬限位信号状态
**	备注：	  
**  作者：         
**  开发日期：
***************************************************************************************************/
u8 ManualAxisMaxHardLimitSta(u8 Axis)
{
	u8 limitSta = 0;
	switch(Axis)
	{
		case X_Axsis:
			if(IO_Sign_On_Off[I_DETECT_X_MAX_LIMIT] == 0)
			{//常开
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_X_MAX_LIMIT] ? X_AXIS_HARD_MAX_LIMIT_LOW_LEVEL : 0;
			}
			else
			{//常闭
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_X_MAX_LIMIT] ? X_AXIS_HARD_MAX_LIMIT_HIGH_LEVEL : 0;
			}
			break;
		case L_Axsis:
			if(IO_Sign_On_Off[I_DETECT_Y_MAX_LIMIT] == 0)
			{//常开
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_Y_MAX_LIMIT] ? Y_AXIS_HARD_MAX_LIMIT_LOW_LEVEL : 0;
			}
			else
			{//常闭
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_Y_MAX_LIMIT] ? Y_AXIS_HARD_MAX_LIMIT_HIGH_LEVEL : 0;
			}
			break;
		case Z_Axsis:
			if(IO_Sign_On_Off[I_DETECT_Z_MAX_LIMIT] == 0)
			{//常开
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_Z_MAX_LIMIT] ? Z_AXIS_HARD_MAX_LIMIT_LOW_LEVEL : 0;
			}
			else
			{//常闭
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_Z_MAX_LIMIT] ? Z_AXIS_HARD_MAX_LIMIT_HIGH_LEVEL : 0;
			}
			break;
		case O_Axsis:
			if(IO_Sign_On_Off[I_DETECT_O_MAX_LIMIT] == 0)
			{//常开
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_O_MAX_LIMIT] ? O_AXIS_HARD_MAX_LIMIT_LOW_LEVEL : 0;
			}
			else
			{//常闭
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_O_MAX_LIMIT] ? O_AXIS_HARD_MAX_LIMIT_HIGH_LEVEL : 0;
			}
			break;
		default:
			break;
	}
	
	return limitSta;
}

/**************************************************************************************************
**  函数名：  ManualAxisMinHardLimitSta()
**	输入参数：无
**	输出参数：无
**	函数功能：轴最小硬限位信号状态
**	备注：	  
**  作者：         
**  开发日期：
***************************************************************************************************/
u8 ManualAxisMinHardLimitSta(u8 Axis)
{
	u8 limitSta = 0;
	switch(Axis)
	{
		case X_Axsis:
			if(IO_Sign_On_Off[I_DETECT_X_MIN_LIMIT] == 0)
			{//常开
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_X_MIN_LIMIT] ? X_AXIS_HARD_MIN_LIMIT_LOW_LEVEL : 0;
			}
			else
			{//常闭
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_X_MIN_LIMIT] ? X_AXIS_HARD_MIN_LIMIT_HIGH_LEVEL : 0;
			}
			break;
		case L_Axsis:
			if(IO_Sign_On_Off[I_DETECT_Y_MIN_LIMIT] == 0)
			{//常开
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_Y_MIN_LIMIT] ? Y_AXIS_HARD_MIN_LIMIT_LOW_LEVEL : 0;
			}
			else
			{//常闭
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_Y_MIN_LIMIT] ? Y_AXIS_HARD_MIN_LIMIT_HIGH_LEVEL : 0;
			}
			break;
		case Z_Axsis:
			if(IO_Sign_On_Off[I_DETECT_Z_MIN_LIMIT] == 0)
			{//常开
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_Z_MIN_LIMIT] ? Z_AXIS_HARD_MIN_LIMIT_LOW_LEVEL : 0;
			}
			else
			{//常闭
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_Z_MIN_LIMIT] ? Z_AXIS_HARD_MIN_LIMIT_HIGH_LEVEL : 0;
			}
			break;
		case O_Axsis:
			if(IO_Sign_On_Off[I_DETECT_O_MIN_LIMIT] == 0)
			{//常开
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_O_MIN_LIMIT] ? O_AXIS_HARD_MIN_LIMIT_LOW_LEVEL : 0;
			}
			else
			{//常闭
				limitSta = Temp_IO_Switch_Parameter[I_DETECT_O_MIN_LIMIT] ? O_AXIS_HARD_MIN_LIMIT_HIGH_LEVEL : 0;
			}
			break;
		default:
			break;
	}
	
	return limitSta;
}

/**************************************************************************************************
**  函数名：  ManualJogRunnig()
**	输入参数：无
**	输出参数：无
**	函数功能：手动模式->寸动模式
**	备注：	  方向键按下一次，轴运动一次
**  作者：    
**  开发日期：
***************************************************************************************************/
void ManualJogRunnig()
{
	if(Jog_Move_Enable == ENABLE)
	{
		if(Axsis_Chosen < Axis_Num + Ext_Axis_Num)
		{//根据选中轴进行寸动
			if(Axis_Step_Distance[Axsis_Chosen] == 0)
			{//若X寸动距离设置为0，那么默认距离设置为 Jog_Pulse_Count_Init 个脉冲
				Jog_Pulse_Count = Jog_Pulse_Count_Init;
			}
			else
			{
				Jog_Pulse_Count = Axis_Step_Distance[Axsis_Chosen] * Step_Coefficient[Axsis_Chosen] / 100;
			}
			
			if((m_PulseTotalCounter[Axsis_Chosen] <= MINROBOTPOSITION + Jog_Pulse_Count) && (Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE))
			{//防止寸动超过0点坐标
				Jog_Pulse_Count = MINROBOTPOSITION;
			}
			else if(Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE)
			{
				Jog_Pulse_Count = m_PulseTotalCounter[Axsis_Chosen] - Jog_Pulse_Count;
			}
			
			if((m_PulseTotalCounter[Axsis_Chosen] + Jog_Pulse_Count >= Axsis_Maxlength[Axsis_Chosen]) && (Axsis_Move_Direction[Axsis_Chosen] == POSITIVE))
			{//防止寸动超过最大软限位
				Jog_Pulse_Count = Axsis_Maxlength[Axsis_Chosen];
			}
			else if(Axsis_Move_Direction[Axsis_Chosen] == POSITIVE)
			{
				Jog_Pulse_Count = m_PulseTotalCounter[Axsis_Chosen] + Jog_Pulse_Count;
			}
			
			if(Axsis_Chosen < Axis_Num)
			{
				if(ManualAxisMaxHardLimitSta(Axsis_Chosen) && (Axsis_Move_Direction[Axsis_Chosen] == POSITIVE))
				{//防止硬限位报警后继续向远端移动
					Robot_Error_Data[0] = Robot_Error_Data[0] | 0x08;
					CloseTotalMotorError();
				}
				else if(ManualAxisMinHardLimitSta(Axsis_Chosen) && (Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE))
				{//防止硬限位报警后继续向远端移动
					Robot_Error_Data[0] = Robot_Error_Data[0] | 0x08;
					CloseTotalMotorError();
				}
			}
			
			if(Axis_Manul_Speed[Axsis_Chosen] > 20)
			{
				Axis_Manul_Speed[Axsis_Chosen] = 20;
			}
			AXisMove(Axsis_Chosen, Jog_Pulse_Count, Axis_Manul_Speed[Axsis_Chosen]);	
		}
		
		//寸动每次都要结束使能
		Jog_Move_Enable = DISABLE;
	}
	else
	{
		if(AxisMoveFlag[Axsis_Chosen] == 1)
		{
			Servo_MoveFinishSta(Axsis_Chosen, Jog_Pulse_Count);
		}
		else
		{
			Jog_Mode_Enable = DISABLE;
		}
	}
}

/**************************************************************************************************
**  函数名：  ManualLinkedRunning()
**	输入参数：无
**	输出参数：无
**	函数功能：自动模式->连动模式
**	备注：	  方向键按下，进入连动模式，再次按下停止运动
**  作者：    
**  开发日期：
***************************************************************************************************/
void ManualLinkedRunning()
{
	u16 i = 0;
	s32 tarPos = 0;
	
	if(Linked_Move_Enable == ENABLE)
	{
		if(KeyStaUpDown == 1 && (AxisMoveFlag[X_Axsis] | AxisMoveFlag[Z_Axsis] | AxisMoveFlag[L_Axsis] | AxisMoveFlag[O_Axsis] | AxisMoveFlag[U_Axsis] | AxisMoveFlag[V_Axsis]) == DISABLE)
		{//按键从抬起变为按下状态才能动作
			if(Origin_Backed == TRUE && Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE && (m_PulseTotalCounter[Axsis_Chosen] <= Axsis_Minlength[Axsis_Chosen]))
			{//当前选中轴已经在最小位置
				Linked_Move_Enable = DISABLE;
			}
			else if(Origin_Backed == TRUE && Axsis_Move_Direction[Axsis_Chosen] == POSITIVE && (m_PulseTotalCounter[Axsis_Chosen] >= Axsis_Maxlength[Axsis_Chosen]))
			{//当前选中轴已经在最大位置
				Linked_Move_Enable = DISABLE;
			}
			else
			{
				Linked_Pulse = MINROBOTPOSITION;
				if(Origin_Backed == TRUE)
				{
					if(Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE)
					{//防止连动超过0点坐标
						Linked_Pulse = MINROBOTPOSITION;
					}
					else if(Axsis_Move_Direction[Axsis_Chosen] == POSITIVE)
					{//防止连动超过最大软件限位
						Linked_Pulse = Axsis_Maxlength[Axsis_Chosen];
					}
					
					if(Axsis_Chosen < Axis_Num)
					{
						if(ManualAxisMaxHardLimitSta(Axsis_Chosen) && (Axsis_Move_Direction[Axsis_Chosen] == POSITIVE))
						{//防止硬限位报警后继续向远端移动
							Robot_Error_Data[0] = Robot_Error_Data[0] | 0x08;
							CloseTotalMotorError();
						}
						else if(ManualAxisMinHardLimitSta(Axsis_Chosen) && (Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE))
						{//防止硬限位报警后继续向远端移动
							Robot_Error_Data[0] = Robot_Error_Data[0] | 0x08;
							CloseTotalMotorError();
						}
					}
				}
				
				if(Axis_Manul_Speed[Axsis_Chosen] == 0)
				{
					Axis_Manul_Speed[Axsis_Chosen] = 1;
				}
				if(Origin_Backed != TRUE)
				{
					if(Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE)
					{
						tarPos = m_PulseTotalCounter[Axsis_Chosen] - MAXROBOTPOSITION;
					}
					else if(Axsis_Move_Direction[Axsis_Chosen] == POSITIVE)
					{
						tarPos = m_PulseTotalCounter[Axsis_Chosen] + MAXROBOTPOSITION;
					}
					AXisMove(Axsis_Chosen, tarPos, Axis_Manul_Speed[Axsis_Chosen]);
				}
				else
				{
					AXisMove(Axsis_Chosen, Linked_Pulse, Axis_Manul_Speed[Axsis_Chosen]);
				}
			}
			KeyStaUpDown = 0;
		}
	}
	else
	{//停止连动
		for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
		{
			Servo_Stop(i);
		}
		Linked_Mode_Enable = DISABLE;		
	}
}

/**************************************************************************************************
**  函数名：  IODebugOutput1()
**	输入参数：无
**	输出参数：无
**	函数功能：IO调试-输出调试1
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void IODebugOutput1()
{
	switch(UsartReceiveData[1])
	{
		case P_IODEBUG_OUTPUT1_1:					//Y0
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_0);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_0);
			}
			break;	

		case P_IODEBUG_OUTPUT1_2:		    	//Y1
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_1);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_1);
			}
			break;	

		case P_IODEBUG_OUTPUT1_3:	        //Y2
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_2);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_2);
			}
			break;	

		case P_IODEBUG_OUTPUT1_4:	        //Y3
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_3);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_3);
			}
			break;

		case P_IODEBUG_OUTPUT1_5:	    		//Y4
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_4);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_4);
			}
			break;

		case P_IODEBUG_OUTPUT1_6:					//Y5
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_5);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_5);
			}
			break;	

		case P_IODEBUG_OUTPUT1_7:					//Y6
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_6);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_6);
			}
			break;	

		case P_IODEBUG_OUTPUT1_8:					//Y7
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_7);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_7);
			}
			break;	

		case P_IODEBUG_OUTPUT1_9:		    	//Y8
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_8);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_8);
			}
			break;

		case P_IODEBUG_OUTPUT1_10:   	    //Y9
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_9);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_9);
			}
			break;

		case P_IODEBUG_OUTPUT1_11:	     	//Y10
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_10);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_10);
			}
			break;	

		case P_IODEBUG_OUTPUT1_12:		   //Y11
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_11);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_11);
			}
			break;	

		case P_IODEBUG_OUTPUT1_13:		   //Y12
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_12);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_12);
			}
			break;	

		case P_IODEBUG_OUTPUT1_14:		   //Y13
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_13);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_13);
			}
			break;

		case P_IODEBUG_OUTPUT1_15:	     	//Y14
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_14);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_14);
			}
			break;

		default:
			break;
	}
}

/**************************************************************************************************
**  函数名：  IODebugOutput2()
**	输入参数：无
**	输出参数：无
**	函数功能：IO调试-输出调试2
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void IODebugOutput2()
{
	switch(UsartReceiveData[1])
	{
		case P_IODEBUG_OUTPUT2_1:			//Y15
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_15);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_15);
			}
			break;	

		case P_IODEBUG_OUTPUT2_2:		    //Y16
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_16);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_16);
			}
			break;

		case P_IODEBUG_OUTPUT2_3:	        //Y17
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_17);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_17);
			}
			break;	

		case P_IODEBUG_OUTPUT2_4:	        //Y18
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_18);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_18);
			}
			break;

		case P_IODEBUG_OUTPUT2_5:	    	//Y19
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_19);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_19);
			}
			break;

		case P_IODEBUG_OUTPUT2_6:			//Y20
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_20);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_20);
			}
			break;	

		case P_IODEBUG_OUTPUT2_7:			//Y21
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_21);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_21);
			}
			break;	

		case P_IODEBUG_OUTPUT2_8:			//Y22
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_22);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_22);
			}
			break;	

		case P_IODEBUG_OUTPUT2_9:		    //Y23
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_23);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_23);
			}
			break;

		case P_IODEBUG_OUTPUT2_10:   	    //Y24
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_24);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_24);
			}
			break;

		case P_IODEBUG_OUTPUT2_11:	       //Y25
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_25);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_25);
			}
			break;	

		case P_IODEBUG_OUTPUT2_12:		   //Y26
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_26);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_26);
			}
			break;	

		case P_IODEBUG_OUTPUT2_13:		   //Y27
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_27);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_27);
			}
			break;	
			
		case P_IODEBUG_OUTPUT2_14:		   //Y28
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_28);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_28);
			}
			break;

		case P_IODEBUG_OUTPUT2_15:	     //Y29
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_29);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_29);
			}
			break;

		default:
			break;
	}

}

/**************************************************************************************************
**  函数名：  IODebugOutput3()
**	输入参数：无
**	输出参数：无
**	函数功能：IO调试
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void IODebugOutput3()
{
	switch(UsartReceiveData[1])
	{
		case P_IODEBUG_OUTPUT3_1:				//Y30
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_30);	   //置0
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_30);
			}
			break;	

		case P_IODEBUG_OUTPUT3_2:		    //Y31
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_31);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_31);
			}
			break;	

		case P_IODEBUG_OUTPUT3_3:	        //Y32
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_32);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_32);
			}
			break;	

		case P_IODEBUG_OUTPUT3_4:	        //Y33
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_33);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_33);
			}
			break;

		case P_IODEBUG_OUTPUT3_5:	    		//Y34
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_34);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_34);
			}
			break;

		case P_IODEBUG_OUTPUT3_6:					//Y35
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_35);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_35);
			}
			break;	

		case P_IODEBUG_OUTPUT3_7:					//Y36
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_36);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_36);
			}
			break;	

		case P_IODEBUG_OUTPUT3_8:					//Y37
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_37);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_37);
			}
			break;	

		case P_IODEBUG_OUTPUT3_9:		    	//Y38
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_38);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_38);
			}
			break;

		case P_IODEBUG_OUTPUT3_10:   	    //Y39
			if(UsartReceiveData[2])
			{
				ResetOutput(O_IODEBUG_OUTPUT_39);
			}
			else
			{
				SetOutput(O_IODEBUG_OUTPUT_39);
			}
			break;

		default:
		  break;
	}
}

/**************************************************************************************************
**  函数名：  ManulDebug()
**	输入参数：无
**	输出参数：无
**	函数功能：监视命令
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void ManulDebug()
{
	u16 i = 0;
	u16 j = 0;
	u8  pointCode = 0;
	u8  SavePointTemp[40]={0};
	
	switch(UsartReceiveData[1])
	{

		case P_AXIS_MANUL_SPEED:		//手动速度值
			for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
			{
				Axis_Manul_Speed[i] = UsartReceiveData[2];
				Axis_Manul_Speed_Temp[i] = UsartReceiveData[2];
			}
			break;
		case P_AXIS_STEP_MM:	    	//寸动距离
			for(i=0;i<Axis_Num + Ext_Axis_Num;i++)
			{
				Axis_Step_Distance[i] = (u16)((u16)UsartReceiveData[2])|((u16)UsartReceiveData[3]<<8);				 
			}
			break;
		case P_XAXIS_MOVE_LEFT:	   	//X轴-左移
		case P_YAXIS_MOVE_LEFT:	   	//Y轴-左移
		case P_ZAXIS_MOVE_LEFT:	   	//Z轴-左移
		case P_OAXIS_MOVE_LEFT:	   	//O轴-左移
		case P_UAXIS_MOVE_LEFT:	   	//U轴-左移
		case P_VAXIS_MOVE_LEFT:	   	//Z轴-左移
			if(JDZ_Parameter.Switch == 1 && Not_Get_Position() == 1)
			{
				return;
			}
			Axsis_Chosen = (UsartReceiveData[1] - P_XAXIS_MOVE_LEFT) / 2;
			Axsis_Move_Direction[Axsis_Chosen] = NEGATIVE;
			if(UsartReceiveData[2] == 0x01)	//寸动
			{
				if(UsartReceiveData[3])
				{
					if(AxisMoveFlag[Axsis_Chosen])
					{
					}
					else
					{
						Jog_Mode_Enable = ENABLE;
						Jog_Move_Enable = ENABLE;
						Linked_Mode_Enable = DISABLE;	
					}	
				}
				else
				{
					Jog_Mode_Enable = DISABLE;
					Jog_Move_Enable = DISABLE;
					Jog_Pause_Enable= DISABLE;					   
				}
			}
			else if(UsartReceiveData[2] == 0x02)	//连动
			{
				if(UsartReceiveData[3])
				{
					KeyStaUpDown = 1;
					Linked_Mode_Enable = ENABLE;
					Linked_Move_Enable = ENABLE;
					Jog_Mode_Enable = DISABLE;
					Jog_Move_Enable = DISABLE;
					Jog_Pause_Enable= DISABLE;		
				}
				else
				{
					KeyStaUpDown = 0;
					Linked_Move_Enable = DISABLE;
				}
			}
			break;
		case P_XAXIS_MOVE_RIGHT:   //X轴-右移
		case P_YAXIS_MOVE_RIGHT:   //Y轴-右移
		case P_ZAXIS_MOVE_RIGHT:   //Z轴-右移
		case P_OAXIS_MOVE_RIGHT:   //O轴-右移
		case P_UAXIS_MOVE_RIGHT:   //U轴-右移
		case P_VAXIS_MOVE_RIGHT:   //V轴-右移
			if(JDZ_Parameter.Switch == 1 && Not_Get_Position() == 1)
			{
				return;
			}
			Axsis_Chosen = (UsartReceiveData[1] - P_XAXIS_MOVE_RIGHT) / 2;
			Axsis_Move_Direction[Axsis_Chosen] = POSITIVE;
			if(UsartReceiveData[2] == 0x01)	//寸动
			{
				if(UsartReceiveData[3])
				{  			      
					if(AxisMoveFlag[Axsis_Chosen])
					{
					}
					else
					{
						Jog_Mode_Enable = ENABLE;
						Jog_Move_Enable = ENABLE;
						Linked_Mode_Enable = DISABLE;	
					}	
				}
				else
				{
					Jog_Mode_Enable = DISABLE;
					Jog_Move_Enable = DISABLE;
					Jog_Pause_Enable= DISABLE;					   
				}			 		 
			}
			else if(UsartReceiveData[2] == 0x02)	//连动
			{
				if(UsartReceiveData[3])
				{
					KeyStaUpDown = 1;
					Linked_Mode_Enable = ENABLE;
					Linked_Move_Enable = ENABLE;
					Jog_Mode_Enable = DISABLE;
					Jog_Move_Enable = DISABLE;
					Jog_Pause_Enable= DISABLE;		
				}
				else
				{
					KeyStaUpDown = 0;
					Linked_Move_Enable = DISABLE;
				}
			}
			break;
		case P_POINT_SAVE:   //点保存
			pointCode = UsartReceiveData[2] - 1; 						//下发的编号是1~40  减1后可以做数组下标		
			Manul_Save_Point[pointCode].Flag = UsartReceiveData[3];
			Manul_Save_Point[pointCode].Name = (u32)(((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24));
			Manul_Save_Point[pointCode].Name2 = (u32)(((u32)UsartReceiveData[8])|((u32)UsartReceiveData[9]<<8)|((u32)UsartReceiveData[10]<<16)|((u32)UsartReceiveData[11]<<24));
			Manul_Save_Point[pointCode].Name3 = (u32)(((u32)UsartReceiveData[12])|((u32)UsartReceiveData[13]<<8)|((u32)UsartReceiveData[14]<<16)|((u32)UsartReceiveData[15]<<24));		
			Manul_Save_Point[pointCode].Point_X = (u32)(((u32)UsartReceiveData[16])|((u32)UsartReceiveData[17]<<8)|((u32)UsartReceiveData[18]<<16)|((u32)UsartReceiveData[19]<<24));
			Manul_Save_Point[pointCode].Point_L = (u32)(((u32)UsartReceiveData[20])|((u32)UsartReceiveData[21]<<8)|((u32)UsartReceiveData[22]<<16)|((u32)UsartReceiveData[23]<<24));
			Manul_Save_Point[pointCode].Point_Z = (u32)(((u32)UsartReceiveData[24])|((u32)UsartReceiveData[25]<<8)|((u32)UsartReceiveData[26]<<16)|((u32)UsartReceiveData[27]<<24));
			Manul_Save_Point[pointCode].Point_O = (u32)(((u32)UsartReceiveData[28])|((u32)UsartReceiveData[29]<<8)|((u32)UsartReceiveData[30]<<16)|((u32)UsartReceiveData[31]<<24));	
		
			W25QXX_Write(&UsartReceiveData[2], P_POINT_SAVE_HEAD + pointCode * P_POINT_SAVE_LEN, 30);
			break;	
	  case P_DELETE_ONEPOINT:	//删除一个点		
			pointCode =  UsartReceiveData[2] - 1; //下发的编号是1~40  减1后可以做数组下标:当前要删除的点编号
			Manul_Save_Point[pointCode].Flag = 0;	  			
			for(j=0; j<40; j++)
			{
				SavePointTemp[j] = 0;
			}	
			W25QXX_Write(SavePointTemp, P_POINT_SAVE_HEAD + pointCode * P_POINT_SAVE_LEN, 30);	
			break;
		case P_MODIFY_ONEAXSIS:	//修改一个点的某个轴坐标
			pointCode =  UsartReceiveData[2]-1; //1~40  减1后可以做数组下标:当前要修改的点编号
			for(i=0; i<4; i++)
			{
				SavePointTemp[i] = UsartReceiveData[4+i];
			}
			if(UsartReceiveData[3] == X_Axsis)
			{
				W25QXX_Write(SavePointTemp, P_POINT_SAVE_HEAD + pointCode*P_POINT_SAVE_LEN+14, 4);	
				Manul_Save_Point[pointCode].Point_X = (u32)(((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24));    
			}
			else if(UsartReceiveData[3] == L_Axsis)
			{
				W25QXX_Write(SavePointTemp, P_POINT_SAVE_HEAD + pointCode*P_POINT_SAVE_LEN+18, 4);	
				Manul_Save_Point[pointCode].Point_L = (u32)(((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24));	    
			}
			else if(UsartReceiveData[3] == Z_Axsis)
			{
				W25QXX_Write(SavePointTemp, P_POINT_SAVE_HEAD + pointCode*P_POINT_SAVE_LEN+22, 4);	   
				Manul_Save_Point[pointCode].Point_Z = (u32)(((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24));	 
			}
			else if(UsartReceiveData[3] == O_Axsis)
			{
				W25QXX_Write(SavePointTemp, P_POINT_SAVE_HEAD + pointCode*P_POINT_SAVE_LEN+26, 4);	 
				Manul_Save_Point[pointCode].Point_O = (u32)(((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24));			    
			}    
			break;
		case P_RENAME_ONEPOINT:	//修改一个点的名称
			pointCode =  UsartReceiveData[2]-1; //1~40  减1后可以做数组下标:当前要修改的点编号
			for(i=0; i<12; i++)
			{
				SavePointTemp[i] = UsartReceiveData[3+i];
			}
			W25QXX_Write(SavePointTemp, P_POINT_SAVE_HEAD + pointCode*P_POINT_SAVE_LEN+2, 12);	      
			break;	
		case P_DELETE_ALLPOINT:		//恢复出厂设置删除点
			All_Axis_Point_Deleted_Flag = 0;
			for(j=0; j<40; j++)
			{
				SavePointTemp[j] = 0;
			}
			for(i=0; i<ManulSavePointMaxNum; i++)		
			{
				Manul_Save_Point[i].Flag = 0;	  
				W25QXX_Write(SavePointTemp, P_POINT_SAVE_HEAD + i * P_POINT_SAVE_LEN, 30);
			}
			All_Axis_Point_Deleted_Flag = 1;
			break;
		default:
			break;
	}
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
