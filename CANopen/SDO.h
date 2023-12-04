/***********************************
 *           SDO Protocol          *
 ***********************************/
#ifndef __SDO_H_
#define __SDO_H_

#include "CANopen.h"

/*SDO数据操作方向*/
#define INITIATE_SDO_DOWNLOAD   0x20             //SDO启动下载：写入Servo数据[4:7]:data-reponse:0x60 
#define INITIATE_SDO_UPLOAD     0x40             //SDO启动上传：读取Servo数据-reponse:0x40 [4:7]:data
#define ABORT_SDO_TRANSFER      0x80             //中止SDO通信   [4:7]:abort codes

/*SDO数据帧结构体*/
typedef struct   //SDO message
{
  u8  Node_ID;								//节点ID
  u16 index;									//索引
  u8  sub_index;							//子索引
  u8  len;      							//传输数据的长度，+4变为总得can报文长度
  u8  Data[8];								//数据
}SDO_Struct;

extern SDO_Struct SDO_Message;								//SDO信息帧


void SDO_Protocol(u8);               					//SDO通信协议
void SDO_Process(u8);              						//SDO消息处理函数
//void Set_Guard_Time(u8);											//设置保护节点时间
void Query_Servo_parameter(u8,u16,u8);				//查询伺服器参数
//void SDO_Process_PositionMode(u8);						//设置伺服器位置模式
//void SDO_Process_SpeedMode(u8 Node_ID);				//设置伺服器速度模式
void SDO_Process_HomingMode(u8, u8, s32);			//设置伺服器回原点模式
void SDO_Process_CycPosMode(u8 Node_ID);			//设置伺服器周期同步位置模式
//void SDO_Process_CycCSVMode(u8 Node_ID);			//设置伺服器周期同步速度模式
void SDO_Process_CycCSTMode(u8 Node_ID);			//设置伺服器周期同步转矩模式

#endif









