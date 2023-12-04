#include "stm32f4xx.h"
#include "JDZ.h" 
#include "Usart.h" 
#include "Delay.h"
#include "BackToOrigin.h"
#include "StatusControl.h"
#include "w25qxx.h"
#include "SpeedControl.h" 
#include "Parameter.h" 
#include "Error.h"
#include "CANopen.h"
#include "ActionOperate.h"
#include "SignalWatch.h"
#include "EtherCAT_App.h"

u16 CRCresult = 0;													//CRC校验值
u8  CRCresult1 = 0;													//CRC校验值低八位
u8  CRCresult2 = 0;													//CRC校验值高八位
u8 JDZ_ReadPosition_count = 0;							//主循环读取编码器计数
u8 JDZ_ReadPosition_Flag = FALSE;						//主循环读取编码器值的标志位
u8  JDZ_ReceiveDataCounter = 0;		       		//绝对值接收数据计数

s32 JDZ_Encoder[2][Axis_Num + Ext_Axis_Num] = {0};									//【0/1】【0/1/2/3】0代表编码器圈数，1代表编码器单圈位置,后面代表4个轴
s32 JDZ_Encoder_Position[Axis_Num + Ext_Axis_Num] = {0};						//使用禾川电机直接获得位置

u8  JDZ_StartReceiveDataFlag = FALSE;								//绝对值开始接受数据标志
u8	JDZ_NewOrder = FALSE;														//接收到新的对值数据
u8  JDZ_AllowError = 100;														//从编码器读回来的脉冲值与定时器计数的脉冲值的误差（动作确认）

u8  IRQ_Count = 0;																	//接收到的数据帧不满足CRC校验，重新接收
u8  First_Get_Position_Flag[Axis_Num + Ext_Axis_Num] = {0};				//第一次开机或者报警后获取位置标志位
s32 JDZ_Position[Axis_Num + Ext_Axis_Num] = {0};										//从编码器读回来的脉冲值
u8  Send_Count = 0;

s32 JDZ_ZeroPosition[Axis_Num + Ext_Axis_Num] = {0};								//绝对值编码器0点位置
u8 m_PositionResetStep[Axis_Num + Ext_Axis_Num] = {0};
u32 m_PositionResetCounter[Axis_Num + Ext_Axis_Num] = {0};

/*
 *将标志位清零->将从驱动器读回来的脉冲重新赋值，报警与设置原点后会调用此函数
*/
void Cancle_Get_Position_Flag()
{
	u16 i = 0;
	
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		First_Get_Position_Flag[i] = 0;
	}
}

/*
 *第一次开机或者紧急停止后未读到当前位置,提示报警
*/
u8 Not_Get_Position()
{
	u8 result = 0;
	if(JDZ_Parameter.Switch == 1)
	{
		switch(JXS_Parameter.Origin)
		{
			case FOM_X:
				if(First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1)
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			
			case FOM_Z:
				if(First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1)
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			
			case FOM_Y:
				if(First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1)
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			
			case FOM_O:
				if(First_Get_Position_Flag[O_Axsis] == 0 && JDZ_Parameter.OriginSetting[O_Axsis] == 1)
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			
			case FOM_Y_X:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			
			case FOM_X_Y:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			
			case FOM_Z_X:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}		 
				break;

			case FOM_X_Z:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}    		 
				break;

			case FOM_O_X:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[O_Axsis] == 0 && JDZ_Parameter.OriginSetting[O_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}	 
				break;

			case FOM_Z_X_L:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}   		 
				break;

			case FOM_Z_L_X:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				} 
				break;

			case FOM_Z_L_X_O:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1) \
						|| (First_Get_Position_Flag[O_Axsis] == 0 && JDZ_Parameter.OriginSetting[O_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			case FOM_Z_X_L_O:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1) \
						|| (First_Get_Position_Flag[O_Axsis] == 0 && JDZ_Parameter.OriginSetting[O_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;

			case FOM_L_O_Z_X:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1) \
						|| (First_Get_Position_Flag[O_Axsis] == 0 && JDZ_Parameter.OriginSetting[O_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			case FOM_Z_O_X_L:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1) \
						|| (First_Get_Position_Flag[O_Axsis] == 0 && JDZ_Parameter.OriginSetting[O_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;

			case FOM_O_Z_X_L:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1) \
						|| (First_Get_Position_Flag[O_Axsis] == 0 && JDZ_Parameter.OriginSetting[O_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			default:
				break;
		}
		
	}
	return result;
}

/*
 * 绝对值模式下，反馈回来的脉冲和发出的脉冲比较，如果在误差范围内，返回1
*/
u8 Judge_JDZ_Error(u8 Axis)
{
	return 1;
}

/*
 * 保存原点是否设置
 */
void JDZ_Save_Origin_Set()
{
	u8 temp[4] = {0};
	
	temp[0] = JDZ_Parameter.OriginSetting[X_Axsis];	
	temp[1] = JDZ_Parameter.OriginSetting[L_Axsis];	
	temp[2] = JDZ_Parameter.OriginSetting[Z_Axsis];	
	temp[3] = JDZ_Parameter.OriginSetting[O_Axsis];
	
	W25QXX_Write(temp,0x1117,4);
}

/*
 * 原点位置开机读取
 */
void JDZ_OriginRead(void)
{
	u16 i = 0;
	u8 temp[32] = {0};
	
	W25QXX_Read(temp, P_SERVO_JDZ_ZERO_HEAD, P_SERVO_JDZ_ZERO_LEN);
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		JDZ_ZeroPosition[i] = 0;
		JDZ_ZeroPosition[i] |= (u32)(((u32)temp[0 + i * 4])|((u32)temp[1 + i * 4]<<8)|((u32)temp[2 + i * 4]<<16)|((u32)temp[3 + i * 4]<<24));
	}
}

/*
 * 原点设置
 */
void JDZ_Origin_Set(void)
{
	u16 i = 0;
	u32 offsetPoint = 0;
	u8 temp[32] = {0};
	
	if(JDZ_SetOrigin_Flag == TRUE)
	{//迈信伺服不支持直接设置零点位置，只能保持当前位置做为零点
		if(JDZ_Origin_Setting_Axis_Num - 1 < Axis_Num)
		{
			offsetPoint = JXS_Parameter.OrignOffset[JDZ_Origin_Setting_Axis_Num - 1] * Step_Coefficient[JDZ_Origin_Setting_Axis_Num - 1] / 100;
		}
		else
		{
			offsetPoint = ExtendAix_Parameter[JDZ_Origin_Setting_Axis_Num - Axis_Num - 1].E_OriginOffset * Step_Coefficient[JDZ_Origin_Setting_Axis_Num - 1] / 100;
		}
		JDZ_ZeroPosition[JDZ_Origin_Setting_Axis_Num - 1] = EtherCAT_GetCurrentPosition(JDZ_Origin_Setting_Axis_Num) / Axsis_ParPosChange - offsetPoint;
		
		for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
		{
			temp[0 + i * 4] = (u8)(JDZ_ZeroPosition[i] & 0x000000FF);	
			temp[1 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0x0000FF00)>>8);	
			temp[2 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0x00FF0000)>>16);	
			temp[3 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0xFF000000)>>24);
		}
		W25QXX_Write(temp, P_SERVO_JDZ_ZERO_HEAD, P_SERVO_JDZ_ZERO_LEN);
		
		CSP_Mode_AxisInit(JDZ_Origin_Setting_Axis_Num);									//重新设置为位置模式
		
		JDZ_Parameter.OriginSetting[JDZ_Origin_Setting_Axis_Num - 1] = 1;
		Robot_Reset();
		JDZ_SetOrigin_Flag = FALSE;
		JDZ_Save_Origin_Set();
		Cancle_Get_Position_Flag();
		
		delay_ms(100);									//确保位置同步成功后重新打开PID
	}
}

/*
 * 坐标复位函数
 */
u8 Position_Reset(u8 Axis, u32 Position)
{
	u16 i = 0;
	u32 offsetPoint = 0;
	u8 temp[32] = {0};

	if(m_PositionResetStep[Axis] == 0)
	{/*当前位置设置为待机点*/
		offsetPoint = Position - MINROBOTPOSITION;
		JDZ_ZeroPosition[Axis] = EtherCAT_GetCurrentPosition(Axis_To_ID(Axis)) / Axsis_ParPosChange - offsetPoint;
		
		if(m_InterpCurveFlag == INTER_CURVE_NO || m_InterpAxisMoveFlag[X_Axsis] > 0 || m_InterpAxisMoveFlag[L_Axsis] > 0 || m_InterpAxisMoveFlag[Z_Axsis] > 0 \
			|| m_InterpAxisMoveFlag[O_Axsis] > 0 || m_InterpAxisMoveFlag[U_Axsis] > 0 || m_InterpAxisMoveFlag[V_Axsis] > 0)
		{//正在执行插补运动，不能写flash
			JDZ_ZeroPos_ParSave = 1;
		}
		else
		{
			for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
			{
				temp[0 + i * 4] = (u8)(JDZ_ZeroPosition[i] & 0x000000FF);	
				temp[1 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0x0000FF00)>>8);	
				temp[2 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0x00FF0000)>>16);	
				temp[3 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0xFF000000)>>24);
			}
			W25QXX_Write(temp, P_SERVO_JDZ_ZERO_HEAD, P_SERVO_JDZ_ZERO_LEN);
		}
		
		m_PositionResetCounter[Axis] = 0;
		m_PositionResetStep[Axis] = 1;
	}
	
	if(m_PositionResetStep[Axis] == 1)
	{
		if(m_PositionResetCounter[Axis] > 20)
		{//零点复位超时后重新设零点
			return 1;
		}
		else
		{
			m_PositionResetCounter[Axis]++;
			delay_ms(1);
		}
	}
	
	return 0;
}

/*
 * 机械回零后坐标设为零点
 */
u8 PositionSetZero(u8 Axis)
{
	u16 i = 0;
	u8 temp[32] = {0};

	JDZ_ZeroPosition[Axis] = 0;
	
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		temp[0 + i * 4] = (u8)(JDZ_ZeroPosition[i] & 0x000000FF);
		temp[1 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0x0000FF00)>>8);
		temp[2 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0x00FF0000)>>16);
		temp[3 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0xFF000000)>>24);
	}
	W25QXX_Write(temp, P_SERVO_JDZ_ZERO_HEAD, P_SERVO_JDZ_ZERO_LEN);
	
	return 0;
}

/*
 * 485数据处理函数
 */
void JDZ_OrderDecoding()
{
	u16 i = 0;
	
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		if(PDO_FirstGetPosition[i] == 1)
		{
			m_PulseTotalCounter[i] = PDO_Cur_Position[i] + MINROBOTPOSITION;

			if(First_Get_Position_Flag[i] == 0 && JDZ_Parameter.OriginSetting[i] == 1)
			{
				First_Get_Position_Flag[i] = 1;
			}
			
			PDO_FirstGetPosition[i] = 0;
		}
		else
		{
			if(ServoGetCanBoffSta(i) == 0)
			{//未链接的伺服坐标设置为最小值
				m_PulseTotalCounter[i] = MINROBOTPOSITION;
			}
		}
	}
	
	if(JDZ_Parameter.Switch == 1)
	{//绝对值模式时需要设置
		if(Origin_Backed == FALSE)
		{
			Axis_Set_Origin();
		}
		
		JDZ_Origin_Set();
	}
}			




