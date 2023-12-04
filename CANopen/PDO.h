#ifndef __PDO_H_
#define __PDO_H_

#include "CANopen.h"

extern u16 PDO_Actual_Status[SERVO_NODE_NUM];								//用于保存伺服反馈的状态
extern s32 PDO_Cur_Position[SERVO_NODE_NUM];								//用于保存伺服反馈的当前位置
extern u8 PDO_FirstGetPosition[SERVO_NODE_NUM];							//用于保存伺服首次位置获取成功
extern s16 PDO_Cur_Tprque[SERVO_NODE_NUM];									//用于保存伺服反馈的当前扭矩，单位0.1%
//extern u16 PDO_Cur_Controlword[SERVO_NODE_NUM];							//用于保存伺服反馈的当前控制子状态
extern u8 PDO_Cur_Mode[SERVO_NODE_NUM];											//用于保存伺服反馈的当前模式
extern s32 PDO_Cur_Speed[SERVO_NODE_NUM];										//用于保存伺服反馈的当前转速

extern u16 PDO_Actual_StatusUpdateTime[SERVO_NODE_NUM];			//用于标记状态标志计数器延时时间
extern u16 PDO_Actual_StatusUpdate[SERVO_NODE_NUM];					//用于标记状态标志计数器已更新

void PDO_ControlWordSet(u8 Node_ID, u16 controlword);																			//设置控制字
void PDO_ServoPositionSet(u8 Node_ID, u32 Target_Position, u32 Target_Velocity);          //发送位置模式的目标位置和速度
void PDO_ServoAccDecSet(u8 Node_ID, u32 Acc, u32 Dec);          													//发送位置模式的轮廓加速度和轮廓减速度
void Get_Status_Position(CanRxMsg RxMessage);            																	//CAN报文处理

#endif
