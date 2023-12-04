#ifndef __485_JDZ_h_
#define __485_JDZ_h_

#include "StatusControl.h"
#define DIR485_JDZ_H  GPIO_SetBits(GPIOD,GPIO_Pin_1)   //**2019/08/14  绝对值伺服-ZCX
#define DIR485_JDZ_L  GPIO_ResetBits(GPIOD,GPIO_Pin_1) //**2019/08/14

extern u8 JDZ_ReceiveDataLen;			   //记录接收数据长度
extern u8 JDZ_NewOrder;		           //接收到新数据
extern u8 JDZ_StartReceiveDataFlag;
extern s32 JDZ_Encoder[2][Axis_Num + Ext_Axis_Num];
extern u8 JDZ_ReadPosition_count;
extern u8 JDZ_ReadPosition_Flag ;
extern s32 JDZ_Position[Axis_Num + Ext_Axis_Num];
extern u8  First_Get_Position_Flag[Axis_Num + Ext_Axis_Num];
extern u8  JDZ_AllowError;
extern s32 JDZ_ZeroPosition[Axis_Num + Ext_Axis_Num];								//绝对值编码器0点位置
extern u8 m_PositionResetStep[Axis_Num + Ext_Axis_Num];
extern u32 m_PositionResetCounter[Axis_Num + Ext_Axis_Num];

void JDZ_OrderDecoding(void);
void Read_Servo_acPosition(void);
void JDZ_Servo_PulseNum(u8 );
void JDZ_Origin_Set(void);
extern void JDZ_OriginRead(void);
extern void JDZ_Save_Origin_Set(void);
extern u8 Judge_JDZ_Error(u8 Axis);
extern u8 Not_Get_Position(void);
extern void Cancle_Get_Position_Flag(void);	
extern u8 Position_Reset(u8 Axis,u32 Position);
extern u8 PositionSetZero(u8 Axis);

#endif
