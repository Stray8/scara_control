#include "CANopen.h"
#include "Parameter.h"
#include "Error.h"
#include "EtherCAT_App.h"
#include "ethercatcoe.h"
#include "ethercattype.h"
#include "can.h"
#include "ActionOperate.h"

CanRxMsg CAN_Recieve_Data;											//CAN接收的数据缓存区
u8  ServoNodeID[SERVO_NODE_ID_NUM] = {0};				//节点ID值存储变量
u16 Controlword[SERVO_NODE_NUM] = {							//伺服的状态控制字
				DISABLE_VOLTAGE,DISABLE_VOLTAGE,
				DISABLE_VOLTAGE,DISABLE_VOLTAGE};

u8  Homing_Flag_Can	=	FALSE;     								//伺服回零模式启动

u8 Servo_ModeFlag[SERVO_NODE_NUM] = {SERVO_MODE_INVALID};				//伺服模式，0无效 1位置 2回零1 3回零2 4速度
u8 Servo_ConnectSta[SERVO_NODE_NUM] = {0};											//伺服连接状态，0未连接 1已连接
u8 Servo_ConnectError[SERVO_NODE_NUM] = {0};											//伺服连接是否发生过报警，0未发生，1发生过

u16 Servo_CommunTimeoutCounter[SERVO_NODE_NUM] = {0};						//伺服通信超时计数器
u8 Servo_InitFinish = 0;																				//伺服初始化完成标志

/***************************************************************************
**  函数名：  CAN1_RX0_IRQHandler()
**	输入参数：无
**	输出参数：无
**	函数功能：CAN1的接收邮箱0的接收中断
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	
	Get_Status_Position(RxMessage);
}

/**************************************************************************************************
**  函数名：ID_To_Axis
**	输入参数：轴ID编号
**	输出参数：
**	函数功能：轴编号转为CANOPEN的轴ID
**	备注：	
**  作者：         
**  开发日期：
***************************************************************************************************/
u8 ID_To_Axis(u8 Node_ID)
{
	u8 Axis = 0;
	
	switch(Node_ID)
	{
		case SERVO_NODE_ID_01_X:
			Axis = X_Axsis;
			break;
		case SERVO_NODE_ID_02_L:
			Axis = L_Axsis;
			break;
		case SERVO_NODE_ID_03_Z:
			Axis = Z_Axsis;
			break;
		case SERVO_NODE_ID_04_O:
			Axis = O_Axsis;
			break;
		case SERVO_NODE_ID_05_U:
			Axis = U_Axsis;
			break;
		case SERVO_NODE_ID_06_V:
			Axis = V_Axsis;
			break;
		default:
			break;
	}
	
	return Axis;
}

///***************************************************************************
//**  函数名：  PP_Mode_Init()
//**	输入参数：无
//**	输出参数：无
//**	函数功能：所有伺服电机的位置模式初始化
//**	备注：
//**  作者：
//**  开发日期：
//***************************************************************************/
//void PP_Mode_Init()
//{
//	int i = 0;
//	
//	/*初始配置伺服器，进入operate状态*/
//	for(i=1; i<SERVO_NODE_ID_NUM; i++)
//	{
//		if(ServoNodeID[i] && ServoGetCanBoffSta(i - 1))
//		{
//			SDO_Process_PositionMode(i);				//初始化伺服器节点以及设置相关参数
//		}
//	}
//}

///***************************************************************************
//**  函数名：  PP_Mode_AxisInit()
//**	输入参数：无
//**	输出参数：无
//**	函数功能：单个伺服电机的位置模式初始化
//**	备注：	  
//**  作者：    
//**  开发日期：
//***************************************************************************/
//void PP_Mode_AxisInit(u8 Node_ID)
//{
//	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
//	{
//		return;
//	}
//	
//	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_POSITION)
//	{//如果已经是位置模式了，直接返回
//		return;
//	}
//	
//	/*初始配置伺服器，进入operate状态*/
//	SDO_Process_PositionMode(Node_ID);					//初始化伺服器节点以及设置相关参数
//}

///***************************************************************************
//**  函数名：  SP_Mode_Init()
//**	输入参数：无
//**	输出参数：无
//**	函数功能：所有伺服电机的速度模式初始化
//**	备注：	  
//**  作者：    
//**  开发日期：
//***************************************************************************/
//void SP_Mode_Init()
//{
//	int i = 0;
//			
//	/*初始配置伺服器，进入operate状态*/
//	for(i=1; i<SERVO_NODE_ID_NUM; i++)
//	{
//		if(ServoNodeID[i] && ServoGetCanBoffSta(i - 1))
//		{
//			SDO_Process_SpeedMode(i);					//初始化伺服器节点以及设置相关参数
//		}
//	}
//}

///***************************************************************************
//**  函数名：  SP_Mode_AxisInit()
//**	输入参数：无
//**	输出参数：无
//**	函数功能：单个伺服电机的速度模式初始化
//**	备注：	  
//**  作者：    
//**  开发日期：
//***************************************************************************/
//void SP_Mode_AxisInit(u8 Node_ID)
//{
//	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
//	{
//		return;
//	}
//	
//	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_SPEED)
//	{//如果已经是位置模式了，直接返回
//		return;
//	}
//	
//	/*初始配置伺服器，进入operate状态*/
//	SDO_Process_SpeedMode(Node_ID);							//初始化伺服器节点以及设置相关参数
//}

/***************************************************************************
**  函数名：  Homing_Mode_AxisInit()
**	输入参数：Node_ID 需要回零的轴对应的ID
**	输入参数：flag 0搜索原点信号 1当前位置为原点
**	输出参数：无
**	函数功能：单个伺服电机的回原点模式初始化
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
void Homing_Mode_AxisInit(u8 Node_ID, u8 flag, s32 oriOffset)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_BACKZERO1 && flag == 0)
	{//如果已经是回零模式1了，直接返回
		return;
	}
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_BACKZERO2 && flag == 1)
	{//如果已经是回零模式2了，直接返回
		return;
	}
		
	/*初始配置伺服器，进入回原点状态*/
	SDO_Process_HomingMode(Node_ID, flag, oriOffset);				 //初始化伺服器节点以及设置相关参数
}
/***************************************************************************
**  函数名：  CSP_Mode_Init()
**	输入参数：无
**	输出参数：无
**	函数功能：所有伺服电机的周期同步位置模式初始化
**	备注：
**  作者：
**  开发日期：
***************************************************************************/
void CSP_Mode_Init()
{
	int i = 0;
	
	/*初始配置伺服器，进入operate状态*/
	for(i=1; i<SERVO_NODE_ID_NUM; i++)
	{
		if(ServoNodeID[i] && ServoGetCanBoffSta(i - 1))
		{
			SDO_Process_CycPosMode(i);				//初始化伺服器节点以及设置相关参数
		}
	}
}

/***************************************************************************
**  函数名：  CSP_Mode_AxisInit()
**	输入参数：无
**	输出参数：无
**	函数功能：单个伺服电机的周期同步位置模式初始化
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
void CSP_Mode_AxisInit(u8 Node_ID)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_CYC_CSP)
	{//如果已经是周期同步位置模式了，直接返回
		return;
	}
	
	/*初始配置伺服器，进入operate状态*/
	SDO_Process_CycPosMode(Node_ID);					//初始化伺服器节点以及设置相关参数
}
///***************************************************************************
//**  函数名：  CSV_Mode_Init()
//**	输入参数：无
//**	输出参数：无
//**	函数功能：所有伺服电机的周期同步速度模式初始化
//**	备注：
//**  作者：
//**  开发日期：
//***************************************************************************/
//void CSV_Mode_Init()
//{
//	int i = 0;
//	
//	/*初始配置伺服器，进入operate状态*/
//	for(i=1; i<SERVO_NODE_ID_NUM; i++)
//	{
//		if(ServoNodeID[i] && ServoGetCanBoffSta(i - 1))
//		{
//			SDO_Process_CycCSVMode(i);				//初始化伺服器节点以及设置相关参数
//		}
//	}
//}

///***************************************************************************
//**  函数名：  CSV_Mode_AxisInit()
//**	输入参数：无
//**	输出参数：无
//**	函数功能：单个伺服电机的周期同步速度模式初始化
//**	备注：	  
//**  作者：    
//**  开发日期：
//***************************************************************************/
//void CSV_Mode_AxisInit(u8 Node_ID)
//{
//	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
//	{
//		return;
//	}
//	
//	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_CYC_CSV)
//	{//如果已经是周期同步位置模式了，直接返回
//		return;
//	}
//	
//	/*初始配置伺服器，进入operate状态*/
//	SDO_Process_CycCSVMode(Node_ID);					//初始化伺服器节点以及设置相关参数
//}

/***************************************************************************
**  函数名：  CST_Mode_Init()
**	输入参数：无
**	输出参数：无
**	函数功能：所有伺服电机的周期同步转矩模式初始化
**	备注：
**  作者：
**  开发日期：
***************************************************************************/
void CST_Mode_Init()
{
	int i = 0;
	
	/*初始配置伺服器，进入operate状态*/
	for(i=1; i<SERVO_NODE_ID_NUM; i++)
	{
		if(ServoNodeID[i] && ServoGetCanBoffSta(i - 1))
		{
			SDO_Process_CycCSTMode(i);				//初始化伺服器节点以及设置相关参数
		}
	}
}

/***************************************************************************
**  函数名：  CST_Mode_AxisInit()
**	输入参数：无
**	输出参数：无
**	函数功能：单个伺服电机的周期同步转矩模式初始化
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
void CST_Mode_AxisInit(u8 Node_ID)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_CYC_CST)
	{//如果已经是周期同步转矩模式了，直接返回
		return;
	}
	
	/*初始配置伺服器，进入operate状态*/
	SDO_Process_CycCSTMode(Node_ID);					//初始化伺服器节点以及设置相关参数
}

/***************************************************************************
**  函数名：  PDO_Mode_Change()
**	输入参数：Node_ID 需要回零的轴对应的ID
**	输入参数：Mode 控制模式
**	输出参数：无
**	函数功能：单个伺服电机的模式设置
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
u16 PDO_Mode_Change(u8 Node_ID, u8 Mode)
{
	u16 i = 0;
	
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return 1;
	}
	
	PDO_TargetInf[ServoNodeID[Node_ID]].TargetMode = Mode;					//位置模式
	EtherCAT_SendPDOFinish();
	
	while(i < 5)
	{
		i++;
		
		if(PDO_Cur_Mode[Node_ID - 1] == Mode)
		{
			return 0;
		}
		
		delay_ms(CANOPEN_DEAL_PDO_DELAY);
	}
	
	return 1;
}

///***************************************************************************
//**  函数名：  Set_Servo_PP_Mode()
//**	输入参数：无
//**	输出参数：无
//**	函数功能：配置伺服器位置模式，并设置位置，速度，加减速初始值
//**	备注：	  无
//**  作者：       
//**  开发日期：
//***************************************************************************/
//void Set_Servo_PP_Mode(u8 Node_ID)
//{
//	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_POSITION)
//	{//如果已经是位置模式了，直接返回
//		return;
//	}
//	
//	Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_POSITION;
//	ServoAccDecSet(ID_To_Axis(Node_ID));
//	
//	PDO_TargetInf[ServoNodeID[Node_ID]].TargetPos = 0;
//	EtherCAT_SendPDOFinish();
//	delay_ms(CANOPEN_DEAL_PDO_DELAY);
//	
//	PDO_Mode_Change(Node_ID, 0x01);														//位置模式
//	
//	if((Controlword[Node_ID - 1] & 0x000f) != 0x0002 && \
//			(Controlword[Node_ID - 1] & 0x000f) != 0x000F)
//	{//如果当前状态不为快速停止，那么需要先切换到0x0006，再切换到0x0007
//		Controlword[Node_ID - 1] = 0x06;
//		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//		
//		Controlword[Node_ID - 1] = 0x07;
//		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//	}
//	
//	Controlword[Node_ID - 1] = 0x000f | 0x0100;						//位置模式时，暂停标志置1，刚使能时必须处于暂停
//	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//}

///***************************************************************************
//**  函数名：  Set_Servo_SP_Mode()
//**	输入参数：无
//**	输出参数：无
//**	函数功能：配置伺服器速度模式，速度，加减速初始值
//**	备注：	  无
//**  作者：       
//**  开发日期：
//***************************************************************************/
//void Set_Servo_SP_Mode(u8 Node_ID)
//{
//	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_SPEED)
//	{//如果已经是速度模式了，直接返回
//		return;
//	}
	
//	Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_SPEED;
//	ServoAccDecSet(ID_To_Axis(Node_ID));
//	
//	PDO_TargetInf[ServoNodeID[Node_ID]].TarVelocity = 0;
//	EtherCAT_SendPDOFinish();
//	delay_ms(CANOPEN_DEAL_PDO_DELAY);
//	
//	PDO_Mode_Change(Node_ID, 0x03);																		//速度模式
//  
//	if((Controlword[Node_ID - 1] & 0x000f) != 0x0002 && \
//			(Controlword[Node_ID - 1] & 0x000f) != 0x000F)
//	{//如果当前状态不为快速停止，那么需要先切换到0x0006，再切换到0x0007
//		Controlword[Node_ID - 1] = 0x06;
//		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//		
//		Controlword[Node_ID - 1] = 0x07;
//		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//	}

//	Controlword[Node_ID - 1] = 0x010f;
//	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//}

/***************************************************************************
**  函数名：  Set_Servo_Homing_Mode()
**	输入参数：flag 0搜索原点信号 1当前位置为原点
**	输出参数：无
**	函数功能：配置伺服器回原点模式
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************/
void Set_Servo_Homing_Mode(u8 Node_ID, u8 flag, s32 oriOffset)
{
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_BACKZERO1 && flag == 0)
	{//如果已经是回零模式1了，直接返回
		return;
	}
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_BACKZERO2 && flag == 1)
	{//如果已经是回零模式2了，直接返回
		return;
	}
	
	PDO_Mode_Change(Node_ID, 0x06);																			//回零模式
	ServoAccDecSet(ID_To_Axis(Node_ID));
	
	//PDO_TargetInf[ServoNodeID[Node_ID]].OriginOffset = oriOffset;				//原点偏置
	
	/*回原点模式：35以当前位置为原点，27查找原点位置信号后退出为原点*/
	if(flag == 0)
	{
		PDO_TargetInf[ServoNodeID[Node_ID]].OriginMode = 27;
		Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_BACKZERO1;
	}
	else
	{
		PDO_TargetInf[ServoNodeID[Node_ID]].OriginMode = 35;
		Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_BACKZERO2;
	}
	EtherCAT_SendPDOFinish();
	delay_ms(CANOPEN_DEAL_PDO_DELAY);
	
	if((Controlword[Node_ID - 1] & 0x000f) != 0x0002 && \
			(Controlword[Node_ID - 1] & 0x000f) != 0x000F)
	{//如果当前状态不为快速停止，那么需要先切换到0x0006，再切换到0x0007
		Controlword[Node_ID - 1] = 0x06;
		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
		
		Controlword[Node_ID - 1] = 0x07;
		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
	}
	
	Controlword[Node_ID - 1] = 0x000f;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
}

/***************************************************************************
**  函数名：  Set_Servo_CSP_Mode()
**	输入参数：无
**	输出参数：无
**	函数功能：配置伺服器周期同步位置模式，并设置位置，速度，加减速初始值
**	备注：	  无
**  作者：       
**  开发日期：
***************************************************************************/
void Set_Servo_CSP_Mode(u8 Node_ID)
{
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_CYC_CSP)
	{//如果已经是位置模式了，直接返回
		return;
	}
	
	Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_CYC_CSP;
	ServoAccDecSet(ID_To_Axis(Node_ID));
	
	PDO_TargetInf[ServoNodeID[Node_ID]].TargetPos = EtherCAT_GetCurrentPosition(Node_ID);		//	确保当前位置与目标位置一直
	EtherCAT_SendPDOFinish();
	
	PDO_Mode_Change(Node_ID, SERVO_MODE_CYC_CSP);														//周期同步位置模式
	delay_ms(CANOPEN_DEAL_PDO_DELAY);
	
	if((Controlword[Node_ID - 1] & 0x000f) != 0x0002 && \
			(Controlword[Node_ID - 1] & 0x000f) != 0x000F)
	{//如果当前状态不为快速停止，那么需要先切换到0x0006，再切换到0x0007
		Controlword[Node_ID - 1] = 0x06;
		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
		
		Controlword[Node_ID - 1] = 0x07;
		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
	}
	
	Controlword[Node_ID - 1] = 0x000f;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
}
/***************************************************************************
**  函数名：  Set_Servo_CSV_Mode()
**	输入参数：无
**	输出参数：无
**	函数功能：配置伺服器周期同步速度模式，并设置位置，速度，加减速初始值
**	备注：	  无
**  作者：       
**  开发日期：
***************************************************************************/
//void Set_Servo_CSV_Mode(u8 Node_ID)
//{
//	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_CYC_CSV)
//	{//如果已经是位置模式了，直接返回
//		return;
//	}
	
//	Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_CYC_CSV;
//	ServoAccDecSet_CSV(ID_To_Axis(Node_ID));
//	
//	PDO_TargetInf[ServoNodeID[Node_ID]].TarVelocity = 0;
//	EtherCAT_SendPDOFinish();
//	
//	PDO_Mode_Change(Node_ID, SERVO_MODE_CYC_CSV);														//周期同步位置模式
//	delay_ms(CANOPEN_DEAL_PDO_DELAY);
//	
//	if((Controlword[Node_ID - 1] & 0x000f) != 0x0002 && \
//			(Controlword[Node_ID - 1] & 0x000f) != 0x000F)
//	{//如果当前状态不为快速停止，那么需要先切换到0x0006，再切换到0x0007
//		Controlword[Node_ID - 1] = 0x06;
//		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//		
//		Controlword[Node_ID - 1] = 0x07;
//		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//	}
//	
//	Controlword[Node_ID - 1] = 0x000f;
//	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//}

/***************************************************************************
**  函数名：  Set_Servo_CST_Mode()
**	输入参数：无
**	输出参数：无
**	函数功能：配置伺服器周期同步转矩模式，并设置位置，速度，加减速初始值
**	备注：	  无
**  作者：       
**  开发日期：
***************************************************************************/
void Set_Servo_CST_Mode(u8 Node_ID)
{
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_CYC_CST)
	{//如果已经是转矩模式了，直接返回
		return;
	}
	
	Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_CYC_CST;
	ServoAccDecSet_CST(ID_To_Axis(Node_ID));
	
	PDO_TargetInf[ServoNodeID[Node_ID]].TarTorque = 0;		//	确保当前位置与目标位置一直
	EtherCAT_SendPDOFinish();
	
	PDO_Mode_Change(Node_ID, SERVO_MODE_CYC_CST);														//周期同步位置模式
	delay_ms(CANOPEN_DEAL_PDO_DELAY);
	
	if((Controlword[Node_ID - 1] & 0x000f) != 0x0002 && \
			(Controlword[Node_ID - 1] & 0x000f) != 0x000F)
	{//如果当前状态不为快速停止，那么需要先切换到0x0006，再切换到0x0007
		Controlword[Node_ID - 1] = 0x06;
		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
		
		Controlword[Node_ID - 1] = 0x07;
		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
	}
	
	Controlword[Node_ID - 1] = 0x000f;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
}

/***************************************************************************
**  函数名：  Fault_Reset()
**	输入参数：无
**	输出参数：无
**	函数功能：复位故障
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
void Fault_Reset(u8 Node_ID)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	Controlword[Node_ID - 1] = Controlword[Node_ID - 1] | 0x0080;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
}

///***************************************************************************
//**  函数名：  New_Set_Point_Reset()
//**	输入参数：无
//**	输出参数：无
//**	函数功能：复位控制字的更新位置指令
//**	备注：	  无
//**  作者：    
//**  开发日期：
//***************************************************************************/
//void New_Set_Point_Reset(u8 Node_ID)
//{
//	if((Controlword[Node_ID - 1] & QUICK_STOP) == QUICK_STOP)
//	{
//		Controlword[Node_ID - 1] |= 0x000f;
//	}
//	
//	Controlword[Node_ID - 1] = Controlword[Node_ID - 1] & 0xffef;
//	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//	
//	while(PDO_Actual_Status[Node_ID - 1] & 0x1000);
//}

///***************************************************************************
//**  函数名：  New_Set_Point_Set()
//**	输入参数：无
//**	输出参数：无
//**	函数功能：置位控制字的更新位置指令
//**	备注：	  无
//**  作者：    
//**  开发日期：
//***************************************************************************/
//void New_Set_Point_Set(u8 Node_ID)
//{
//	Controlword[Node_ID - 1] = Controlword[Node_ID - 1] & 0xfeff;
//	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//	
//	Controlword[Node_ID - 1] = Controlword[Node_ID - 1] | 0x0030;
//	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//}

///***************************************************************************
//**  函数名：  ServoStart_PDO()
//**	输入参数：Target_Position 目标位置
//**	输出参数：Target_Velocity 速度
//**	函数功能：发送PDO消息，设置伺服器位置、速度
//**	备注：	  无
//**  作者：    
//**  开发日期：
//***************************************************************************/
//void ServoStart_PDO(u8 Node_ID, u32 Target_Position, u32 Target_Velocity)
//{
//	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
//	{
//		return;
//	}
//	
//	New_Set_Point_Reset(Node_ID);  							//复位控制字的位置指令更新
//	delay_ms(CANOPEN_DEAL_PDO_DELAY);
//	PDO_ServoPositionSet(Node_ID, Target_Position, Target_Velocity);//发送伺服移动的位置和速度
//	delay_ms(CANOPEN_DEAL_PDO_DELAY);
//	New_Set_Point_Set(Node_ID);    							//设置控制字的位置指令更新
//}

///***************************************************************************
//**  函数名：  ServoStartSP_PDO()
//**	输入参数：Target_Velocity 目标速度
//**	输入参数：DirFlag 方向 1正转 0反转
//**	输出参数：
//**	函数功能：发送PDO消息，设置伺服器速度且启动运行
//**	备注：	  无
//**  作者：    
//**  开发日期：
//***************************************************************************/
//void ServoStartSP_PDO(u8 Node_ID, u32 Target_Velocity, u8 DirFlag)
//{
//	s32 Vel = Target_Velocity;
//	
//	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
//	{
//		return;
//	}
		
//	if(DirFlag == 0)
//	{
//		Vel = -Vel;
//	}

//	PDO_TargetInf[ServoNodeID[Node_ID]].TarVelocity = Vel;
//	EtherCAT_SendPDOFinish();
//	delay_ms(CANOPEN_DEAL_PDO_DELAY);
//	
//	Controlword[Node_ID - 1] = 0x000f;
//	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//}

/***************************************************************************
**  函数名：  ServoCSP__PDO()
**	输入参数：Target_Position 目标位置
**	函数功能：发送PDO消息，设置伺服器周期同步位置模式的位置
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************/
void ServoCSP_PDO(u8 Node_ID, s32 intPosition)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	if((Controlword[Node_ID - 1] & QUICK_STOP) == QUICK_STOP)
	{
		Controlword[Node_ID - 1] |= 0x000f;
	}
	
	PDO_TargetInf[ServoNodeID[Node_ID]].TargetPos = intPosition;
	E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_EN;
}

///***************************************************************************
//**  函数名：  ServoCSP__PDO()
//**	输入参数：intVol 目标速度
//**	函数功能：发送PDO消息，设置伺服器周期同步位置模式的位置
//**	备注：	  无
//**  作者：    
//**  开发日期：
//***************************************************************************/
//void ServoCSV_PDO(u8 Node_ID, s32 intVol)
//{
//	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
//	{
//		return;
//	}
	
//	if((Controlword[Node_ID - 1] & QUICK_STOP) == QUICK_STOP)
//	{
//		Controlword[Node_ID - 1] |= 0x000f;
//		PDO_TargetInf[ServoNodeID[Node_ID]].ControlWord = Controlword[Node_ID - 1];
//	}
//	if((Controlword[Node_ID - 1] & 0x0100) == 0x0100)
//	{
//		Controlword[Node_ID - 1] &= 0xfeff;
//		PDO_TargetInf[ServoNodeID[Node_ID]].ControlWord = Controlword[Node_ID - 1];
//	}
//	
//	PDO_TargetInf[ServoNodeID[Node_ID]].TarVelocity = intVol;
//	E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_EN;
//}

/***************************************************************************
**  函数名：  ServoAccDecSet_PDO()
**	输入参数：无
**	输出参数：无
**	函数功能：发送PDO消息，设置伺服器的轮廓加速度、轮廓减速度、快速减速度
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************/
void ServoAccDecSet_PDO(u8 Node_ID, u32 Acc, u32 Dec)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	PDO_ServoAccDecSet(Node_ID, Acc, Dec);		//发送伺服移动的轮廓加速度和轮廓减速度
}


/***************************************************************************
**  函数名：  ServoCST__PDO()
**	输入参数：intTorque 目标转矩
**	函数功能：发送PDO消息，设置伺服器周期同步转矩模式的转矩
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************/
void ServoCST_PDO(u8 Node_ID, s16 intTorque)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	if((Controlword[Node_ID - 1] & QUICK_STOP) == QUICK_STOP)
	{
		Controlword[Node_ID - 1] |= 0x000f;
	}
	
	PDO_TargetInf[ServoNodeID[Node_ID]].TarTorque = intTorque;
	E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_EN;
}

/***************************************************************************
**  函数名：  ServoStopSet_PDO()
**	输入参数：无
**	输出参数：无
**	函数功能：置位控制字的暂停位置指令
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************/
void ServoStopSet_PDO(u8 Node_ID)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
//	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_SPEED)
//	{//如果已经是位置模式了，直接返回
//		PDO_TargetInf[ServoNodeID[Node_ID]].TarVelocity = 0;
//		EtherCAT_SendPDOFinish();
//		delay_ms(CANOPEN_DEAL_PDO_DELAY);
//	}
	
	if(Servo_ModeFlag[Node_ID - 1] != SERVO_MODE_CYC_CSP && Servo_ModeFlag[Node_ID - 1] != SERVO_MODE_CYC_CSV)
	{
		Controlword[Node_ID - 1] = Controlword[Node_ID - 1] | 0x0100;
		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
	}
}

/***************************************************************************
**  函数名：  ServoDisable_PDO()
**	输入参数：无
**	输出参数：无
**	函数功能：伺服失能
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************/
void ServoDisable_PDO(u8 Node_ID)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	Controlword[Node_ID - 1] = DISABLE_VOLTAGE;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
	
	ServoWorkModeReset(Node_ID);						//电机报警后需要将内部模式清除
}

/***************************************************************************
**  函数名：  ServoEmergencyStop_PDO()
**	输入参数：无
**	输出参数：无
**	函数功能：伺服快速停止
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************/
void ServoEmergencyStop_PDO(u8 Node_ID)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	Controlword[Node_ID - 1] = (Controlword[Node_ID - 1] & 0Xfff0) | QUICK_STOP;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
}

/***************************************************************************
**  函数名：  ServoHomingControl()
**	输入参数：无
**	输出参数：无
**	函数功能：启动伺服回原点
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
void ServoHomingControl(u8 Node_ID)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	Controlword[Node_ID - 1] = Controlword[Node_ID - 1] & 0xffef;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
	
	Controlword[Node_ID - 1] = Controlword[Node_ID - 1] & 0xfeff;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
	
	Controlword[Node_ID - 1] = Controlword[Node_ID - 1] | 0x0010;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
}

/***************************************************************************
**  函数名：  ServoHomingFinishSta()
**	输入参数：无
**	输出参数：无
**	函数功能：读取回零状态，bit12为1时完成回零
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
u16 ServoHomingFinishSta(u8 Node_ID)
{
	return ((PDO_Actual_Status[Node_ID - 1] >> 12) & 0x0001);
}

/***************************************************************************
**  函数名：  ServoMoveFinishSta()
**	输入参数：无
**	输出参数：无
**	函数功能：读取移动状态
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
u16 ServoMoveFinishSta(u8 Node_ID)
{
	return ((PDO_Actual_Status[Node_ID - 1] >> 10) & 0x0001);
}

/***************************************************************************
**  函数名：  ServoMoveSpeedZero()
**	输入参数：无
**	输出参数：无
**	函数功能：读取速度模式下目标速度是否到达状态
**	备注：	  返回值 1为零 0不为0
**  作者：    
**  开发日期：
***************************************************************************/
u16 ServoMoveSpeedZero(u8 Node_ID)
{
	return ((PDO_Actual_Status[Node_ID - 1] >> 10) & 0x0001);
}

/***************************************************************************
**  函数名：  ServoMoveAlarmSta()
**	输入参数：用于保存每个电机的报警情况
**	输出参数：无
**	函数功能：读取报警状态
**	返回：    只要有报警就返回1，否则返回0
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
u16 ServoMoveAlarmSta(u8 *alarm)
{
	u16 i = 0;
	u8 ret = 0;
	
	for(i=0; i<SERVO_NODE_NUM; i++)
	{
		alarm[i] = (u8)((PDO_Actual_Status[i] >> 3) & 0x0001);
		ret |= alarm[i];
	}
	
	return ret;
}

/***************************************************************************
**  函数名：  ServoPauseFinishSta()
**	输入参数：无
**	输出参数：无
**	函数功能：读取暂停后速度是否为0
**  函数返回：0未暂停 1暂停完成 2已暂停但在减速中
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
u16 ServoPauseFinishSta(u8 Node_ID)
{
	if(Controlword[Node_ID - 1] & 0x0100)
	{
		if((PDO_Actual_Status[Node_ID - 1] >> 10) & 0x0001)
		{
			return 1;
		}
		else
		{
			return 2;
		}
	}
	else
	{
		return 0;
	}
}

/***************************************************************************
**  函数名：  ServoAlarmReset()
**	输入参数：无
**	输出参数：无
**	函数功能：报警复位，复位控制位0->1时复位报警
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
void ServoAlarmReset(void)
{
	u16 i = 0;
	
	for(i=0; i<SERVO_NODE_NUM; i++)
	{
		if(((PDO_Actual_Status[i] >> 3) & 0x0001) && ServoGetCanBoffSta(i))
		{
			Controlword[i] = Controlword[i] | 0x0100;
			Controlword[i] = Controlword[i] & 0xff7f;
			PDO_ControlWordSet(i + 1, Controlword[i]);
			
			Controlword[i] = Controlword[i] | 0x0080;
			PDO_ControlWordSet(i + 1, Controlword[i]);
			
			Controlword[i] = Controlword[i] & 0xff7f;
			PDO_ControlWordSet(i + 1, Controlword[i]);
			
			Servo_ModeFlag[i] = SERVO_MODE_INVALID;
		}
	}
}

/***************************************************************************
**  函数名：  ServoWorkModeReset()
**	输入参数：无
**	输出参数：无
**	函数功能：初始化伺服工作模式
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
void ServoWorkModeReset(u8 Node_ID)
{
	Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_INVALID;
}

/***************************************************************************
**  函数名：  ServoGetCanBoffSta()
**	输入参数：无
**	输出参数：无
**	函数功能：获取伺服总线是否BUSOFF
**	备注：	  返回值 0总线未连接  1总线已连接
**  作者：    
**  开发日期：
***************************************************************************/
u8 ServoGetCanBoffSta(u8 Node_Code)
{	
	return Servo_ConnectSta[Node_Code];
}

/***************************************************************************
**  函数名：  ServoEnableSta()
**	输入参数：
**	输出参数：无
**	函数功能：用于保存每个电机的是否是能
**	返回：    所有伺服都是能返回0，否则返回相应ID
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
u16 ServoEnableSta(void)
{
	u16 i = 0;
	
	for(i=0; i<SERVO_NODE_NUM; i++)
	{
		if(ServoGetCanBoffSta(i) == 0)
		{
			continue;
		}
		
		if(((PDO_Actual_Status[i] >> 2) & 0x0001) == 0)
		{
			return (i + 1);
		}
	}
	
	return 0;
}

/***************************************************************************
**  函数名：  ServoControlwordStaChaeck()
**	输入参数：
**	输出参数：无
**	函数功能：检测控制状态，0正常 1异常
**	返回：    
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
u16 ServoControlwordStaCheck(u8 Node_ID)
{
//	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
//	{
//		return 0;
//	}
//	
//	if(PDO_Cur_Controlword[Node_ID - 1] != Controlword[Node_ID - 1])
//	{
//		return 1;
//	}
	
	return 0;
}

/***************************************************************************
**  函数名：  ServoStartStatusUpdateDelay()
**	输入参数：Node_Code 轴编号
**	输出参数：无
**	函数功能：启动状态更新延时
**	返回：    0表示已更新，不为零表示未更新
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
void ServoStartStatusUpdateDelay(u8 Node_Code, u16 delayTime)
{
	PDO_Actual_StatusUpdateTime[Node_Code] = delayTime;
	PDO_Actual_StatusUpdate[Node_Code] = 1;
}

/***************************************************************************
**  函数名：  ServoStatusUpdate()
**	输入参数：Node_Code 轴编号
**	输出参数：无
**	函数功能：读取轴状态更新标志
**	返回：    0表示已更新，不为零表示未更新
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
u16 ServoStatusUpdate(u8 Node_Code)
{
	return PDO_Actual_StatusUpdate[Node_Code];
}

/***************************************************************************
**  函数名：  ServoWorkModeRead()
**	输入参数：无
**	输出参数：无
**	函数功能：读取伺服工作模式
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
u8 ServoWorkModeRead(u8 Node_ID)
{
	return Servo_ModeFlag[Node_ID - 1];
}

/***************************************************************************
**  函数名：  ServoMoveCurSpeed()
**	输入参数：无
**	输出参数：无
**	函数功能：读取伺服当前速度
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************/
s32 ServoMoveCurSpeed(u8 Node_ID)
{
	return PDO_Cur_Speed[Node_ID - 1];
}





