#ifndef __CANOPEN_H_
#define __CANOPEN_H_

#include "stm32f4xx.h"

#define  CANOPEN_MODE 																					//CANOPEN模式下需要定义这个宏名称
#define  CANOPEN_DEAL_SDO_DELAY									10							//CANOPEN从机处理报文的延时，用于SDO延时
#define  CANOPEN_DEAL_PDO_DELAY									5								//CANOPEN从机处理报文的延时，用于PDO延时
#define  CANOPEN_DEAL_PDO_CONTROL_DELAY					10							//CANOPEN从机处理报文的延时，用于PDO控制字切换延时

#define CAN_HEARTBEAT         0x06   			//心跳包
#define CAN_HEART_SEND_CODE   0x04   			//数据发送的功能码
#define CAN_HEART_WRITE_CODE  0x05   			//数据写的功能码

/*伺服控制模式定义*/
#define  SERVO_MODE_INVALID					0										//无效模式
#define  SERVO_MODE_POSITION				1										//位置模式
#define  SERVO_MODE_BACKZERO1				2										//回零模式1，以负限位传感器为零点
#define  SERVO_MODE_BACKZERO2				3										//回零模式2，以当前位置为原点
#define  SERVO_MODE_SPEED						4										//速度模式
#define  SERVO_MODE_CYC_CSP					8										//周期同步位置模式
#define  SERVO_MODE_CYC_CSV					9										//周期同步速度模式
#define  SERVO_MODE_CYC_CST					10										//周期同步转矩模式

/*控制相关的阈值*/
#define  SERVO_ZERO_SPEED_VALUE			2										//判断为0rpm的最大转速


/*从节点个数及其NodeID定义*/
#define SERVO_NODE_NUM 			6														//从节点个数
#define SERVO_NODE_ID_NUM 	(SERVO_NODE_NUM + 1)				//NodeID个数，包括1个广播ID和其他从节点
#define SERVO_NODE_ID_00 			0													//NodeID必须从0开始编号，且连续，0为广播ID
#define SERVO_NODE_ID_01_X 		1													//X轴NodeID
#define SERVO_NODE_ID_02_L		2													//L轴NodeID
#define SERVO_NODE_ID_03_Z		3													//Z轴NodeID
#define SERVO_NODE_ID_04_O		4													//O轴NodeID
#define SERVO_NODE_ID_05_U 		5													//U轴NodeID
#define SERVO_NODE_ID_06_V 		6													//V轴NodeID
#define SERVO_NODE_ID_01_X2 	7													//X2轴NodeID
#define SERVO_NODE_ID_02_L2		8													//L2轴NodeID
#define SERVO_NODE_ID_03_Z2		9													//Z2轴NodeID
#define SERVO_NODE_ID_04_O2		10												//O2轴NodeID
#define SERVO_NODE_ID_05_U2 	11												//U2轴NodeID
#define SERVO_NODE_ID_06_V2 	12												//V2轴NodeID

#include "NMT.h"
#include "SDO.h"
#include "PDO.h"
#include "Delay.h"

/*伺服器各个地址定义*/
//禾川、汇川伺服地址定义
#define SERVO_WORK_MODE							(0x6060)							//工作模式地址，原点模式6、位置模式1等
#define SERVO_CONTROL_WORD					(0x6040)							//控制字地址
#define SERVO_TARGET_POSITION				(0x607A)							//目标位置地址
#define SERVO_ORIGIN_OFFSET					(0x607C)							//原点偏置地址
#define SERVO_MOVE_SPEED						(0x6081)							//轮廓速度地址
#define SERVO_MOVE_ACC							(0x6083)							//轮廓加速度地址
#define SERVO_MOVE_DEC							(0x6084)							//轮廓减速度地址
#define SERVO_STOP_DEC							(0x6085)							//快速停止减速度地址
#define SERVO_STOP_DEC_MODE					(0x605A)							//快速停止减速度模式地址
#define SERVO_PAUSE_DEC_MODE				(0x605D)							//轮廓暂停减速度模式地址
#define SERVO_STATE_WORD						(0x6041)							//状态字地址
#define SERVO_USER_POSITION					(0x6064)							//用户反馈位置地址
#define SERVO_BACK_HOME_MODE				(0x6098)							//回原点模式地址
#define SERVO_BACK_HOME_SP					(0x6099)							//回原点模式速度地址
#define SERVO_BACK_ACC							(0x609A)							//回原点加速度地址
#define SERVO_TARGET_SPEED					(0x60FF)							//目标速度地址
#define SERVO_ERROR_CODE						(0x603F)							//错误代码读取地址
#define SERVO_MOVE_DIR							(0x607E)							//运动方向极性地址
#define SERVO_INTERP_CYCLE					(0x60C2)							//插补周期及单位地址


/*伺服位置模式下，控制字第6位设置*/
#define SINGLE_SET_POINT       		((u8)0)								//等待当前位置指令执行完毕后再执行新命令
#define CHANGE_SET_IMMEDIATELY 		((u8)1)								//中止正在执行的指令，执行最新的位置指令
/*伺服控制字操作*/
#define  ENABLE_OPERATION   			(0x000F)							//使能伺服
#define  DISABLE_OPERATION  			(0x0007)							//等待打开伺服使能
#define  QUICK_STOP								(0x0002)							//快速停止
#define  DISABLE_VOLTAGE					(0x0000)							//伺服无故障

#define CANOPEN_SPEED_MAGNIF					20	   		//CANopen相关的速度倍率，即设置的速度需要乘以该倍率后才是伺服运行的转速
#define CANOPEN_ORIGIN_SPEED_MIN			25	   		//CANopen回零模式的最小转速
#define CANOPEN_MOVE_SPEED_MAX				5000	   	//CANopen最大转速
#define CANOPEN_LAST_MOVE_OVER_TIME		20	   			//运动超时时间，单位10ms

/*OD,对象字典结构体*/
typedef struct
{
  u16 index;									//索引
  u8  subindex;								//子索引
  u8  ctr;										//
  u16 length;									//数据长度
}DICT_OBJECT ;
/*PDO数据帧结构体*/
typedef struct
{
   u32 PDO_COB_ID;						//ID
   u8  len;										//数据长度
   u8  PDO_data[8];						//数据
}PDO_Struct;

extern u8  ServoNodeID[SERVO_NODE_ID_NUM] ;												//节点ID号
extern CanRxMsg CAN_Recieve_Data;																	//CAN接收的数据缓冲区
extern u16 Controlword[SERVO_NODE_NUM];      											//状态控制字
extern u8 Homing_Flag_Can;																				//伺服回零模式启动
extern u8 Servo_ConnectSta[SERVO_NODE_NUM];												//伺服连接状态，0未连接 1已连接

#define  SERVO_COMMUN_TIMEOUT					(20)												//伺服通信超时时间，200ms
extern u16 Servo_CommunTimeoutCounter[SERVO_NODE_NUM];						//伺服通信超时计数器
extern u8 Servo_ConnectError[SERVO_NODE_NUM];
extern u8 Servo_InitFinish;																				//伺服初始化完成标志

/*操作函数*/
void CANopen_Init(void);												//CANOpen初始化
void SDO_ControlWordSet(u8 Node_ID);						//控制字的SDO设置
void RPDO_Mapping(u8);													//RPDO映射
void TPDO_Mapping(u8);													//TPDO映射
//void PP_Mode_Init(void);   											//将所有伺服设置为位置模式
//void PP_Mode_AxisInit(u8 Node_ID);							//将单个伺服设置为位置模式
//void SP_Mode_Init(void);												//将所有伺服设置为速度模式
//void SP_Mode_AxisInit(u8 Node_ID);							//将单个伺服设置为速度模式
void CSP_Mode_Init(void);												//将所有伺服设置为周期同步位置模式
void CSP_Mode_AxisInit(u8 Node_ID);							//将单个伺服设置为周期同步位置模式
//void CSV_Mode_Init(void);												//将所有伺服设置为周期同步速度模式
//void CSV_Mode_AxisInit(u8 Node_ID);							//将单个伺服设置为周期同步速度模式
void CST_Mode_Init(void);												//将所有伺服设置为周期同步转矩模式
void CST_Mode_AxisInit(u8 Node_ID);							//将单个伺服设置为周期同步转矩模式
void Homing_Mode_AxisInit(u8,u8, s32);					//将单个伺服设置为回原点模式
void Fault_Reset(u8);														//故障复位
//void Set_Servo_PP_Mode(u8);          						//伺服器位置模式配置
//void Set_Servo_SP_Mode(u8);          						//伺服器速度模式配置
void Set_Servo_Homing_Mode(u8, u8, s32);	 			//伺服器Homing Mode回原点模式配置
void Set_Servo_CSP_Mode(u8 Node_ID);						//伺服器周期同步位置模式配置
//void Set_Servo_CSV_Mode(u8 Node_ID);						//伺服器周期同步速度模式配置
void Set_Servo_CST_Mode(u8 Node_ID);						//伺服器周期同步转矩模式配置
//void New_Set_Point_Reset(u8);										//复位更新位置指令
//void New_Set_Point_Set(u8);											//置位更新位置指令
//void ServoStart_PDO(u8 Node_ID, u32 Target_Position, u32 Target_Velocity);	//设置伺服器位置模式运动
//void ServoStartSP_PDO(u8 Node_ID, u32 Target_Velocity, u8 DirFlag);					//设置伺服器速度模式启动
void ServoCSP_PDO(u8 Node_ID, s32 intPosition);
//void ServoCSV_PDO(u8 Node_ID, s32 intVol);
void ServoCST_PDO(u8 Node_ID, s16 intTorque);			//
void ServoAccDecSet_PDO(u8 Node_ID, u32 Acc, u32 Dec);											//设置伺服器加速度和减速度
//void ServoEmergencyEnable_PDO(u8 Node_ID);			//设置控制字的使能指令
void ServoEmergencyStop_PDO(u8 Node_ID);				//设置控制字的快速停止指令
void ServoDisable_PDO(u8 Node_ID);							//设置控制字的失能指令
void ServoStopSet_PDO(u8 Node_ID);							//设置控制字的暂停位置指令
void ServoStopClear_PDO(u8 Node_ID);						//清除控制字的暂停位置指令
void Controlword_Set(u8 Node_ID, u8 Change_Set_Immediately);								//控制字的位置指令更新方式设置
void ServoHomingControl(u8 Node_ID);						//启动伺服回原点
u16 ServoHomingFinishSta(u8 Node_ID);						//读取回原点状态
u16 ServoMoveFinishSta(u8 Node_ID);							//读取移动状态
u16 ServoMoveSpeedZero(u8 Node_ID);							//读取速度模式下目标速度是否到达状态
u16 ServoMoveAlarmSta(u8 *alarm);								//伺服报警状态读取
u16 ServoPauseFinishSta(u8 Node_ID);						//读取暂停后速度是否为0
void ServoAlarmReset(void);											//复位伺服报警
void ServoWorkModeReset(u8 Node_ID);						//初始化伺服工作模式
u8 ServoGetCanBoffSta(u8 Node_Code);						//获取伺服总线是否BUSOFF
u16 ServoEnableSta(void);												//获取伺服总线是否有伺服失能
u16 ServoControlwordStaCheck(u8 Node_ID);				//检测控制字是否异常
void ServoStartStatusUpdateDelay(u8 Node_Code, u16 delayTime);
u16 ServoStatusUpdate(u8 Node_Code);
u8 ServoWorkModeRead(u8 Node_ID);
s32 ServoMoveCurSpeed(u8 Node_ID);

#endif




