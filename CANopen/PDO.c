
#include "CANopen.h"
#include "Parameter.h"
#include "Error.h"
#include "EtherCAT_App.h"
#include "ethercatcoe.h"
#include "ethercattype.h"
#include "in.h"
#include "usart.h"

u16 PDO_Actual_Status[SERVO_NODE_NUM] = {0};								//用于保存伺服反馈的状态
s32 PDO_Cur_Position[SERVO_NODE_NUM] = {0};									//用于保存伺服反馈的当前位置
u8 PDO_FirstGetPosition[SERVO_NODE_NUM] = {0};							//用于保存伺服首次位置获取成功
s16 PDO_Cur_Tprque[SERVO_NODE_NUM] = {0};										//用于保存伺服反馈的当前扭矩，单位0.1%，额定扭矩的占比
//u16 PDO_Cur_Controlword[SERVO_NODE_NUM] = {0};							//用于保存伺服反馈的当前控制子状态
u8 PDO_Cur_Mode[SERVO_NODE_NUM] = {0};											//用于保存伺服反馈的当前模式
s32 PDO_Cur_Speed[SERVO_NODE_NUM] = {0};										//用于保存伺服反馈的当前转速

u16 PDO_Actual_StatusUpdateTime[SERVO_NODE_NUM] = {0};			//用于标记状态标志计数器延时时间
u16 PDO_Actual_StatusUpdate[SERVO_NODE_NUM] = {0};					//用于标记状态标志计数器已更新

/***************************************************************************
**  函数名：  PDO_ControlWordSet()
**	输入参数：无
**	输出参数：无
**	函数功能：控制字设置的PDO报文发送
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************/
void PDO_ControlWordSet(u8 Node_ID, u16 controlword)
{
//	u16 i = 0;
	
	PDO_TargetInf[ServoNodeID[Node_ID]].ControlWord = controlword;
	EtherCAT_SendPDOFinish();

//	while(i < 4)
//	{
//		i++;
//		
//		if(PDO_Cur_Controlword[Node_ID - 1] == controlword)
//		{
//			break;
//		}
//		delay_ms(CANOPEN_DEAL_PDO_CONTROL_DELAY);
//	}
	delay_ms(CANOPEN_DEAL_PDO_CONTROL_DELAY);
}

/***************************************************************************
**  函数名：  PDO_ServoPositionSet()
**	输入参数：无
**	输出参数：无
**	函数功能：发送PDO消息，设置伺服器位置、速度
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************/
void PDO_ServoPositionSet(u8 Node_ID, u32 Target_Position, u32 Target_Velocity)
{
	/*设置伺服器目标位置和速度值*/
//	PDO_TargetInf[ServoNodeID[Node_ID]].ContourVel = Target_Velocity;
//	PDO_TargetInf[ServoNodeID[Node_ID]].TargetPos = Target_Position;
//	
//	EtherCAT_SendPDOFinish();
//	delay_ms(CANOPEN_DEAL_PDO_DELAY);
}

/***************************************************************************
**  函数名：  PDO_ServoAccDecSet()
**	输入参数：无
**	输出参数：无
**	函数功能：发送PDO消息，设置伺服器加速度和剪速度
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************/
void PDO_ServoAccDecSet(u8 Node_ID, u32 Acc, u32 Dec)
{
	/*设置伺服器加速度和减速度*/
//	PDO_TargetInf[ServoNodeID[Node_ID]].Acc = Acc;
//	PDO_TargetInf[ServoNodeID[Node_ID]].Dec = Dec;
//	switch(JDZ_Parameter.Server)
//	{
//		case 0://禾川电机
//			PDO_TargetInf[ServoNodeID[Node_ID]].QukDec = Dec * 2;
//			break;
//		case 1://汇川电机
//			PDO_TargetInf[ServoNodeID[Node_ID]].QukDec = Dec * 2;
//			break;
//		case 2://迈信电机
//			PDO_TargetInf[ServoNodeID[Node_ID]].QukDec = Dec * 2;
//			break;
//		default:
//			PDO_TargetInf[ServoNodeID[Node_ID]].QukDec = Dec * 2;
//			break;
//	}
}

/***************************************************************************
**  函数名：  Get_Status_Position()
**	输入参数：无
**	输出参数：无
**	函数功能：接收伺服器上传的状态字和位置信息
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************/
void Get_Status_Position(CanRxMsg RxMessage)
{
	
	CAN_Recieve_Data.StdId = RxMessage.StdId;
	CAN_Recieve_Data.Data[0] = RxMessage.Data[0];
	CAN_Recieve_Data.Data[1] = RxMessage.Data[1];
	CAN_Recieve_Data.Data[2] = RxMessage.Data[2];
	CAN_Recieve_Data.Data[3] = RxMessage.Data[3];
	CAN_Recieve_Data.Data[4] = RxMessage.Data[4];
	CAN_Recieve_Data.Data[5] = RxMessage.Data[5];
	CAN_Recieve_Data.Data[6] = RxMessage.Data[6];
	CAN_Recieve_Data.Data[7] = RxMessage.Data[7];
	
	if(CAN_Recieve_Data.Data[0] == CAN_HEARTBEAT)
	{//接收读命令数据
//		CAN1_RecieveDataDecode();
		
	}
//	if(CAN_Recieve_Data.Data[0] == CAN_HEARTBEAT || CAN_Recieve_Data.Data[0] == CAN_HEART_SEND_CODE || CAN_Recieve_Data.Data[0] == CAN_HEART_WRITE_CODE)CANReceiveDataFinished = TRUE;
}























