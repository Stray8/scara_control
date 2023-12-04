#include "stm32f4xx.h"
#include "Usart.h" 
#include "BackToOrigin.h"
#include "StatusControl.h"
#include "SpeedControl.h"
#include "in.h"
#include "out.h"
#include "Auto.h"
#include "Auto_2.h"
#include "SignalWatch.h"
#include "Parameter.h"
#include "Error.h"
#include "JDZ.h"
#include "CANopen.h"
#include "ActionOperate.h"

u8  Reset_Step = 0;	        			//回原点步骤
u8  Robot_Enable = FALSE;
u8	Single_Reset_Step = 0;											//单轴复位步骤

u8 Axis_LimitFlag = FALSE;
u8 Axis_OriginFlag = FALSE;

u8 JDZ_Machine_Ori_Axis_Num = 0 ;								//机械轴回零轴编号：1-X 2-Z 3-Y 4-O
u8 JDZ_Origin_Resetting_Axis_Num =0;							//重置原点轴编号： 1-X 2-Z 3-Y 4-O
u8 JDZ_Origin_Setting_Axis_Num =0;								//原点设置轴编号： 1-X 2-Z 3-Y 4-O
u8 JDZ_SetOrigin_Flag = FALSE;									//JDZ设置原点标志位
u8 Axis_Machine_Origin_Flag = FALSE;									//机械回零
u8 Axis_BackORIGIN_FLAG[Axis_Num + Ext_Axis_Num] = {0};

/* 绝对值模式下，选中轴是否都设置过原点且已经获得过位置 */
void Axis_Set_Origin()
{
	Origin_Backed = FALSE;
	
	if(JDZ_Parameter.Switch==1)
	{
		switch(JXS_Parameter.Origin)			//回零方式
		{
			case FOM_X:
			case FOM_Z:
			case FOM_Y:
			case FOM_O:
			if((JXS_Parameter.Origin == FOM_X && JDZ_Parameter.OriginSetting[X_Axsis]==1 && First_Get_Position_Flag[X_Axsis] == 1) \
						|| (JXS_Parameter.Origin == FOM_Z && JDZ_Parameter.OriginSetting[Z_Axsis]==1 && First_Get_Position_Flag[Z_Axsis] == 1) \
						|| (JXS_Parameter.Origin == FOM_Y && JDZ_Parameter.OriginSetting[L_Axsis]==1 && First_Get_Position_Flag[L_Axsis] == 1) \
						|| (JXS_Parameter.Origin == FOM_O && JDZ_Parameter.OriginSetting[O_Axsis]==1 && First_Get_Position_Flag[O_Axsis] == 1))
				{
					Origin_Backed = TRUE;
				}
				break;
					
			case FOM_Y_X:
			case FOM_X_Y:
				if(JDZ_Parameter.OriginSetting[X_Axsis]==1 && JDZ_Parameter.OriginSetting[L_Axsis]==1 \
					&& First_Get_Position_Flag[X_Axsis] == 1 && First_Get_Position_Flag[L_Axsis] == 1)
				{
					Origin_Backed = TRUE;
				}
				break;

			case FOM_Z_X:
			case FOM_X_Z:
				if(JDZ_Parameter.OriginSetting[X_Axsis]==1 && JDZ_Parameter.OriginSetting[Z_Axsis]==1 \
					&& First_Get_Position_Flag[X_Axsis] == 1 && First_Get_Position_Flag[Z_Axsis] == 1)
				{
					Origin_Backed = TRUE;
				} 		 
				break;

			case FOM_O_X:
				if(JDZ_Parameter.OriginSetting[X_Axsis]==1 && JDZ_Parameter.OriginSetting[O_Axsis]==1 \
					&& First_Get_Position_Flag[X_Axsis] == 1 && First_Get_Position_Flag[O_Axsis] == 1)
				{
					Origin_Backed = TRUE;
				}     		 
				break;

			case FOM_Z_X_L:
			case FOM_Z_L_X:
				if(JDZ_Parameter.OriginSetting[X_Axsis]==1 && JDZ_Parameter.OriginSetting[Z_Axsis]==1 && JDZ_Parameter.OriginSetting[L_Axsis]==1 \
					&& First_Get_Position_Flag[X_Axsis] == 1 && First_Get_Position_Flag[Z_Axsis] == 1 && First_Get_Position_Flag[L_Axsis] == 1)
				{
					Origin_Backed = TRUE;
				} 		 
				break;

			case FOM_Z_L_X_O:
			case FOM_Z_X_L_O:
			case FOM_Z_O_X_L:
			case FOM_O_Z_X_L:
				if(JDZ_Parameter.OriginSetting[X_Axsis]==1 && JDZ_Parameter.OriginSetting[Z_Axsis]==1&& JDZ_Parameter.OriginSetting[L_Axsis]==1 && JDZ_Parameter.OriginSetting[O_Axsis]==1 \
					&& First_Get_Position_Flag[X_Axsis] == 1 && First_Get_Position_Flag[Z_Axsis] == 1 && First_Get_Position_Flag[L_Axsis] == 1 && First_Get_Position_Flag[O_Axsis] == 1)
				{
					Origin_Backed = TRUE;
				} 
				break;

			default:
				break;
		}
	}
	
}

/**************************************************************************************************
**  函数名：Axis_To_Reset
**	输入参数：Axis 轴编号
**	输出参数：
**	函数功能：轴复位 
**	备注：	
**  作者：         
**  开发日期：六轴改 
***************************************************************************************************/
void Axis_To_Reset(u8 Axis)
{
	u16 Result = 0;
	
	Result = ServoEnableSta();
	if(Result > 0)
	{
//		if(Robot_Error_Num == 0 || Robot_Error_Num > E_SERVO_DISABLE)
//		{
//			Robot_Error_Num = E_SERVO_DISABLE + Result - 1;
//		}
		Robot_Error_Data[10] |= (0x01 << Axis);//伺服未使能
		return;
	}
	
	switch(Single_Reset_Step)
	{
		case 0: //发送回零命令
			ServoHomingControl(Axis_To_ID(Axis));						//CANOpen回零函数
			Single_Reset_Step = 1;
			break;
			 
		case 1:	//原点信号消失时到达原点
			if(ServoHomingFinishSta(Axis_To_ID(Axis)))			 //判断回零是否完成
			{
				Single_Reset_Step = 0;
				Axsis_Origin_Backed[Axis] = TRUE;
				Axis_Machine_Origin_Flag = FALSE;
			}
			break;

		default:
			break;
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能： 
**	备注：	  无
**  作者：    
**  开发日期： 
***************************************************************************************************/
void RobotEnable()
{
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能： 
**	备注：	  无
**  作者：    
**  开发日期： 
***************************************************************************************************/
void RobotDisable()
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：回原点处理-每个CASE中的处理封装
**	备注：	  无
**  作者：       
**  开发日期：六轴改
***************************************************************************************************/
void Axis_BackToOrigin(u8 Axis)
{
	u32 offsetPoint = 0;
	
	if(Homing_Flag_Can == FALSE)
	{//搜索编号成功后开始回原点
//		Homing_Mode_AxisInit(Axis_To_ID(Axis), 0, (s32)((MOVE_Parameter[Axis].OriginOffset - MINROBOTPOSITION) * Axsis_ParPosChange));
		offsetPoint = JXS_Parameter.OrignOffset[Axis] * Step_Coefficient[Axis] / 100;// - MINROBOTPOSITION;
		Homing_Mode_AxisInit(Axis_To_ID(Axis), 0, (s32)(offsetPoint * Axsis_ParPosChange));
		Homing_Flag_Can = TRUE;
		delay_ms(100);
		Single_Reset_Step = 0;
	}
	Axis_To_Reset(Axis);
	
	if(Axsis_Origin_Backed[Axis] == TRUE)		//临时复位标志为0
	{
		Homing_Flag_Can = FALSE;
		
		PositionSetZero(Axis);
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：X轴回原点
**	备注：	  无
**  作者：    DPF     
**  开发日期： 
***************************************************************************************************/
void X_BackToOrigin()
{
	if(Axsis_Origin_Backed[X_Axsis] == TRUE && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		Axis_BackToOrigin(X_Axsis);
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：Z轴回原点
**	备注：	  无
**  作者：    DPF     
**  开发日期： 
***************************************************************************************************/
void Z_BackToOrigin()
{
	if(Axsis_Origin_Backed[Z_Axsis] == TRUE && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		Axis_BackToOrigin(Z_Axsis);
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：Y轴回原点
**	备注：	  无
**  作者：    DPF     
**  开发日期： 
***************************************************************************************************/
void Y_BackToOrigin()
{
	if(Axsis_Origin_Backed[L_Axsis] == TRUE && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		Axis_BackToOrigin(L_Axsis);
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：O轴回原点
**	备注：	  无
**  作者：    DPF     
**  开发日期： 
***************************************************************************************************/
void O_BackToOrigin()
{
	if(Axsis_Origin_Backed[O_Axsis] == TRUE && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		Axis_BackToOrigin(O_Axsis);
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：Y-X轴回原点
**	备注：	  无
**  作者：    DPF     
**  开发日期： 
***************************************************************************************************/
void Y_X_BackToOrigin()
{
	if((Axsis_Origin_Backed[L_Axsis] == TRUE) && (Axsis_Origin_Backed[X_Axsis] == TRUE) && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		if(Axsis_Origin_Backed[L_Axsis] == FALSE)
		{
			Axis_BackToOrigin(L_Axsis);
		}
		else
		{
			Axis_BackToOrigin(X_Axsis);
		}
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：X-Y轴回原点
**	备注：	  无
**  作者：    DPF     
**  开发日期： 
***************************************************************************************************/
void X_Y_BackToOrigin()
{
	if((Axsis_Origin_Backed[X_Axsis] == TRUE) && (Axsis_Origin_Backed[L_Axsis] == TRUE) && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		if(Axsis_Origin_Backed[X_Axsis] == FALSE)
		{
			Axis_BackToOrigin(X_Axsis);
		}
		else
		{
			Axis_BackToOrigin(L_Axsis);
		}
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：Z-X轴回原点
**	备注：	  无
**  作者：    
**  开发日期： 
***************************************************************************************************/
void Z_X_BackToOrigin()
{
	if((Axsis_Origin_Backed[Z_Axsis] == TRUE) && (Axsis_Origin_Backed[X_Axsis] == TRUE) && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		if(Axsis_Origin_Backed[Z_Axsis] == FALSE)
		{
			Axis_BackToOrigin(Z_Axsis);
		}
		else
		{
			Axis_BackToOrigin(X_Axsis);
		}
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：X-Z轴回原点
**	备注：	  无
**  作者：    
**  开发日期： 
***************************************************************************************************/
void X_Z_BackToOrigin()
{
	if((Axsis_Origin_Backed[X_Axsis] == TRUE) && (Axsis_Origin_Backed[Z_Axsis] == TRUE) && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		if(Axsis_Origin_Backed[X_Axsis] == FALSE)
		{
			Axis_BackToOrigin(X_Axsis);
		}
		else
		{
			Axis_BackToOrigin(Z_Axsis);
		}
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：O-X轴回原点
**	备注：	  无
**  作者：    
**  开发日期： 
***************************************************************************************************/
void O_X_BackToOrigin()
{
	if((Axsis_Origin_Backed[O_Axsis] == TRUE) && (Axsis_Origin_Backed[X_Axsis] == TRUE) && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		if(Axsis_Origin_Backed[O_Axsis] == FALSE)
		{
			Axis_BackToOrigin(O_Axsis);
		}
		else
		{
			Axis_BackToOrigin(X_Axsis);
		}
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：Z-X-L轴回原点
**	备注：	  无
**  作者：    
**  开发日期： 
***************************************************************************************************/
void Z_X_L_BackToOrigin()
{
	if((Axsis_Origin_Backed[Z_Axsis] == TRUE) && (Axsis_Origin_Backed[X_Axsis] == TRUE) && (Axsis_Origin_Backed[L_Axsis] == TRUE) && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		if(Axsis_Origin_Backed[Z_Axsis] == FALSE)
		{
			Axis_BackToOrigin(Z_Axsis);
		}
		else if(Axsis_Origin_Backed[X_Axsis] == FALSE)
		{
			Axis_BackToOrigin(X_Axsis);
		}
		else
		{
			Axis_BackToOrigin(L_Axsis);
		}
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：Z-L-X轴回原点
**	备注：	  无
**  作者：    
**  开发日期： 
***************************************************************************************************/
void Z_L_X_BackToOrigin()
{
	if((Axsis_Origin_Backed[Z_Axsis] == TRUE) && (Axsis_Origin_Backed[X_Axsis] == TRUE) && (Axsis_Origin_Backed[L_Axsis] == TRUE) && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		if(Axsis_Origin_Backed[Z_Axsis] == FALSE)
		{
			Axis_BackToOrigin(Z_Axsis);
		}
		else if(Axsis_Origin_Backed[L_Axsis] == FALSE)
		{
			Axis_BackToOrigin(L_Axsis);
		}
		else
		{
			Axis_BackToOrigin(X_Axsis);
		}
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：Z-L-X-O轴回原点
**	备注：	  无
**  作者：    
**  开发日期： 
***************************************************************************************************/
void Z_L_X_O_BackToOrigin()
{
	if((Axsis_Origin_Backed[Z_Axsis] == TRUE) && (Axsis_Origin_Backed[X_Axsis] == TRUE) \
			&& (Axsis_Origin_Backed[L_Axsis] == TRUE)&& (Axsis_Origin_Backed[O_Axsis] == TRUE) && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		if(Axsis_Origin_Backed[Z_Axsis] == FALSE)
		{
			Axis_BackToOrigin(Z_Axsis);
		}
		else if(Axsis_Origin_Backed[L_Axsis] == FALSE)
		{
			Axis_BackToOrigin(L_Axsis);
		}
		else if(Axsis_Origin_Backed[X_Axsis] == FALSE)
		{
			Axis_BackToOrigin(X_Axsis);
		}
		else
		{
			Axis_BackToOrigin(O_Axsis);
		}
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：Z-X-L-O轴回原点
**	备注：	  无
**  作者：    
**  开发日期： 
***************************************************************************************************/
void Z_X_L_O_BackToOrigin()
{
	if((Axsis_Origin_Backed[Z_Axsis] == TRUE) && (Axsis_Origin_Backed[X_Axsis] == TRUE) \
			&& (Axsis_Origin_Backed[L_Axsis] == TRUE)&& (Axsis_Origin_Backed[O_Axsis] == TRUE) && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		if(Axsis_Origin_Backed[Z_Axsis] == FALSE)
		{
			Axis_BackToOrigin(Z_Axsis);
		}
		else if(Axsis_Origin_Backed[X_Axsis] == FALSE)
		{
			Axis_BackToOrigin(X_Axsis);
		}
		else if(Axsis_Origin_Backed[L_Axsis] == FALSE)
		{
			Axis_BackToOrigin(L_Axsis);
		}
		else
		{
			Axis_BackToOrigin(O_Axsis);
		}
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：L-O-Z-X轴回原点
**	备注：	  无
**  作者：    
**  开发日期： 
***************************************************************************************************/
void L_O_Z_X_BackToOrigin()
{
	if((Axsis_Origin_Backed[Z_Axsis] == TRUE) && (Axsis_Origin_Backed[X_Axsis] == TRUE) \
			&& (Axsis_Origin_Backed[L_Axsis] == TRUE)&& (Axsis_Origin_Backed[O_Axsis] == TRUE) && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		if(Axsis_Origin_Backed[L_Axsis] == FALSE)
		{
			Axis_BackToOrigin(L_Axsis);
		}
		else if(Axsis_Origin_Backed[O_Axsis] == FALSE)
		{
			Axis_BackToOrigin(O_Axsis);
		}
		else if(Axsis_Origin_Backed[Z_Axsis] == FALSE)
		{
			Axis_BackToOrigin(Z_Axsis);
		}
		else
		{
			Axis_BackToOrigin(X_Axsis);
		}
	}
}


/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：Z_O_X_L轴回原点
**	备注：	  无
**  作者：    
**  开发日期： 
***************************************************************************************************/
void Z_O_X_L_BackToOrigin()
{
	if((Axsis_Origin_Backed[Z_Axsis] == TRUE) && (Axsis_Origin_Backed[O_Axsis] == TRUE) \
			&& (Axsis_Origin_Backed[X_Axsis] == TRUE)&& (Axsis_Origin_Backed[L_Axsis] == TRUE) && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		if(Axsis_Origin_Backed[Z_Axsis] == FALSE)
		{
			Axis_BackToOrigin(Z_Axsis);
		}
		else if(Axsis_Origin_Backed[O_Axsis] == FALSE)
		{
			Axis_BackToOrigin(O_Axsis);
		}
		else if(Axsis_Origin_Backed[X_Axsis] == FALSE)
		{
			Axis_BackToOrigin(X_Axsis);
		}
		else
		{
			Axis_BackToOrigin(L_Axsis);
		}
	}
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：O_Z_X_L轴回原点
**	备注：	  无
**  作者：    
**  开发日期： 
***************************************************************************************************/
void O_Z_X_L_BackToOrigin()
{
	if((Axsis_Origin_Backed[O_Axsis] == TRUE) && (Axsis_Origin_Backed[Z_Axsis] == TRUE) \
			&& (Axsis_Origin_Backed[X_Axsis] == TRUE)&& (Axsis_Origin_Backed[L_Axsis] == TRUE) && JDZ_Parameter.Switch == 0)
	{
		Origin_Backed = TRUE;
	}
	else
	{
		if(Axsis_Origin_Backed[O_Axsis] == FALSE)
		{
			Axis_BackToOrigin(O_Axsis);
		}
		else if(Axsis_Origin_Backed[Z_Axsis] == FALSE)
		{
			Axis_BackToOrigin(Z_Axsis);
		}
		else if(Axsis_Origin_Backed[X_Axsis] == FALSE)
		{
			Axis_BackToOrigin(X_Axsis);
		}
		else
		{
			Axis_BackToOrigin(L_Axsis);
		}
	}
}

/*
	绝对值--机械回零
*/
void Axis_Machine_Origin()
{
	u32 offsetPoint = 0;

	if(Axis_Machine_Origin_Flag)
	{
		if(Homing_Flag_Can == FALSE)
		{
//			Homing_Mode_AxisInit(Axis_To_ID(JDZ_Machine_Ori_Axis_Num), 0, (s32)((MOVE_Parameter[JDZ_Machine_Ori_Axis_Num].OriginOffset - MINROBOTPOSITION) * Axsis_ParPosChange));
			offsetPoint = JXS_Parameter.OrignOffset[JDZ_Machine_Ori_Axis_Num - 1] * Step_Coefficient[JDZ_Machine_Ori_Axis_Num - 1] / 100;// - MINROBOTPOSITION;
			Homing_Mode_AxisInit(Axis_To_ID(JDZ_Machine_Ori_Axis_Num - 1), 0, (s32)(offsetPoint * Axsis_ParPosChange));
			Homing_Flag_Can = TRUE;
			delay_ms(100);
			Single_Reset_Step = 0;
		}
		Axis_To_Reset(JDZ_Machine_Ori_Axis_Num - 1);
		
		if(Axis_Machine_Origin_Flag == FALSE)
		{
			PositionSetZero(JDZ_Machine_Ori_Axis_Num - 1);

			CSP_Mode_AxisInit(Axis_To_ID(JDZ_Machine_Ori_Axis_Num - 1));									//重新设置为位置模式
			
			Homing_Flag_Can = FALSE;
			JDZ_Parameter.OriginSetting[JDZ_Machine_Ori_Axis_Num - 1] = 0;
			JDZ_Save_Origin_Set();
		}
	}
}

///*
//	自由编程--机械回零
//*/
//void Program_Axis_Origin(u8 axis)
//{
//	if(Program_Axis_Origin_Flag[axis])
//	{
//		switch(axis)
//		{
//			case X_Axsis:
//				Axis_BackToOrigin(X_Axsis);
//				break;
//			case Z_Axsis:
//				Axis_BackToOrigin(Z_Axsis);
//				break;
//			case L_Axsis:
//				Axis_BackToOrigin(L_Axsis);
//				break;
//			case O_Axsis:
//				Axis_BackToOrigin(O_Axsis);
//				break;
//			default:
//				break;
//		}
//		
//		if(Program_Axis_Origin_Flag[axis] == FALSE)
//		{
//			m_PulseTotalCounter[axis] = MINROBOTPOSITION + JXS_Parameter.OrignOffset[axis];
//			Axsis_Origin_Speed[axis] = JXS_Parameter.AxisOriginSpeed[axis];
//		}
//	}
//}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能：回原点处理
**	备注：	  无
**  作者：    
**  开发日期： 
***************************************************************************************************/
void BackToOrigin()
{
	u16 i = 0;
	
	if(JDZ_Parameter.Switch == 1)
	{//JDZ轴机械回零
		Axis_Machine_Origin();
	}
	else if(Back_Origin_Flag == TRUE)
	{//有回零请求时执行回零动作(迈信不支持回零模式，系统必须是绝对值)
		for(i=0; i<OUTPUT_NUM; i++)
		{
			if(OutPut_BeforeOrigin[i] == Before_Origin)
			{
				SetOutput(i);//指示灯灭
			}
		}
		switch(JXS_Parameter.Origin)			//回零方式增加
		{
			case FOM_X:
				X_BackToOrigin();
				break;
			
			case FOM_Z: 
				Z_BackToOrigin();
				break;
			
			case FOM_Y:				 
				Y_BackToOrigin();
				break;
			
			case FOM_O:				 
				O_BackToOrigin();
				break;
					
			case FOM_Y_X:				 
				Y_X_BackToOrigin();
				break;
			
			case FOM_X_Y:
				X_Y_BackToOrigin();
				break;
			
			case FOM_Z_X:
				Z_X_BackToOrigin();	     		 
				break;

			case FOM_X_Z:
				X_Z_BackToOrigin();	     		 
				break;

			case FOM_O_X:
				O_X_BackToOrigin();	     		 
				break;

			case FOM_Z_X_L:
				Z_X_L_BackToOrigin();	     		 
				break;

			case FOM_Z_L_X:
				Z_L_X_BackToOrigin();	     		 
				break;

			case FOM_Z_L_X_O:
				Z_L_X_O_BackToOrigin();	     		 
				break;

			case FOM_L_O_Z_X:
				L_O_Z_X_BackToOrigin();	     		 
				break;

			case FOM_Z_X_L_O:
				Z_X_L_O_BackToOrigin();	     		 
				break;

			case FOM_Z_O_X_L:
				Z_O_X_L_BackToOrigin();	     		 
				break;

			case FOM_O_Z_X_L:
				O_Z_X_L_BackToOrigin();	     		 
				break;
			default:
				break;
		}
		
		if(Origin_Backed == TRUE)
		{
			CSP_Mode_Init();									//重新设置为位置模式
			
			for(i=0; i<OUTPUT_NUM; i++)
			{
				if(OutPut_AfterOrigin[i] == After_Origin)
				{
					SetOutput(i);//指示灯灭
				}
			}
			Back_Origin_Flag = FALSE;		//回零完成后清除回零命令标志
			Robot_Reset();
		}
	}
}

/**************************************************************************************************
**  函数名：  RobotEnableOrigin()
**	输入参数：无
**	输出参数：无
**	函数功能：原点相关通信命令处理
**	备注：	  无
**  作者：    
**  开发日期： 
***************************************************************************************************/
void RobotEnableOrigin()
{
	switch(UsartReceiveData[1])
	{
		case P_ROBOT_ORIGIN:
			if(Robot_Enable)  //机械手已使能-允许回原点
			{
				if(JDZ_Parameter.Switch ==1)
				{
					if(Origin_Backed)
					{
						Work_Status = AUTO_WORK_MODE;
						g_Auto_Reset_Flag = TRUE;
						Robot_Auto_Reset = FALSE;
					}
					
				}
				else
				{
					Back_Origin_Flag = TRUE; //机械手回原点标志位使能
					Axsis_Origin_Backed[X_Axsis] = FALSE;
					Axsis_Origin_Backed[Z_Axsis] = FALSE;
					Axsis_Origin_Backed[L_Axsis] = FALSE;
					Axsis_Origin_Backed[O_Axsis] = FALSE;

					Origin_Backed=FALSE;
				}
			}
			break;	
		case P_ORIGNSETTING:		//设置原点操作
			JDZ_Origin_Setting_Axis_Num = UsartReceiveData[2];
			JDZ_SetOrigin_Flag = TRUE;
			break;
		case P_ORIGNRESETTING:		//重置原点操作
			JDZ_Origin_Resetting_Axis_Num = UsartReceiveData[2];
			JDZ_Parameter.OriginSetting[JDZ_Origin_Resetting_Axis_Num - 1] = 0;
			JDZ_Save_Origin_Set();
			Origin_Backed = FALSE;
			break;
		case P_MACHINEORIGIN:		//机械回零操作
			JDZ_Machine_Ori_Axis_Num = UsartReceiveData[2];
			Axsis_Origin_Backed[JDZ_Machine_Ori_Axis_Num - 1] = 0;
			Origin_Backed = FALSE;
			Axis_Machine_Origin_Flag = TRUE;
			break;
		case P_ROBOT_ENABLE:
			if(UsartReceiveData[2])
			{
				RobotEnable();
				Robot_Enable = TRUE;
			}
			else
			{
				RobotDisable();
				Robot_Enable = FALSE;
				Origin_Backed = FALSE;			   //清除已回原点标志
			}
			break;
		default:
		  break;
	}
}
