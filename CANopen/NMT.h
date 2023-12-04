
#ifndef __NMT_H_
#define __NMT_H_

#include "CANopen.h"

/*NMT命令*/
#define START_REMOTE_NODE           0x01								//启动远程节点命令
#define STOP_REMOTE_NODE            0x02								//停止远程节点命令
#define ENTER_PRE_OPERATIONAL_STATE 0x80								//进入预操作状态命令
#define RESET_NODE                  0x81								//复位节点命令
#define RESET_COMMUNICATION         0x82								//复位通信命令

/*NMT状态机定义*/
typedef enum {
	Initialisation = 0, 				//启动
	Stop = 4,										//停止
	Operate = 5, 								//运行
	Pre_Operate = 127 					//预操作
}NMT_Machine_state;

/***********NMT服务函数***************/
NMT_Machine_state Get_NMT_State(void);
void NMT_Boot_Up(u8);
void Start_Remote_Node(u8 Node_ID);
void Stop_Remote_Node(u8 Node_ID);
void Enter_Pre_Operational_State(u8 Node_ID);
void Reset_Node(u8 Node_ID);
void Reset_Communication(u8 Node_ID);

#endif







