/**
  ******************************************************************************
  * @file    EtherCAT_App.h
  * @author  
  * @version V1.1.0
  * @date    
  * @brief   
  */

#ifndef __ETHERCAT_APP_H
#define __ETHERCAT_APP_H

#include "stm32f4xx.h"
#include "CANopen.h"

/*从节点个数及其NodeID定义*/
#define E_ALLOW_UPDATA_SEND_EN			1												//允许数据更新发送
#define E_ALLOW_UPDATA_SEND_DIS			0												//不允许数据更新发送

//typedef __packed struct
//{//汇川最大10个对象，禾川最大20个对象
//	s32 TargetPos;							//目标位置
//	s32 ContourVel;							//轮廓速度
//	s32 TarVelocity;						//目标速度
//	s32 OriginOffset;						//原点偏置
//	u32 OriginDecPoVel;					//回零搜索减速度点速度
//	u32 OriginVel;							//回零搜索速度
//	u16 ControlWord;						//控制字
//	u8 OriginMode;							//回原模式
//	u8 TargetMode;							//控制模式

//	//uint8 relese;							//16位对其
//}PDO_Output;									//位置模式下的TPDO数据格式

typedef __packed struct
{//汇川最大10个对象，禾川最大20个对象，台达最大8个对象
	s32 TargetPos;							//目标位置
//	s32 ContourVel;							//轮廓速度
//	s32 TarVelocity;						//目标速度

//	s32 OriginOffset;						//原点偏置
//	u32 Acc;										//轮廓加速度
//	u32 Dec;										//轮廓减速度
//	u32 QukDec;									//快速停止减速度
	u16 ControlWord;						//控制字
	u8 OriginMode;							//回原模式
	u8 TargetMode;							//控制模式
	
	s16 TarTorque;							//目标转矩

	//uint8 relese;							//16位对其
}PDO_Output;									//位置模式下的TPDO数据格式

typedef __packed struct 
{
	u16 StatusWord;					//状态字
//	u16 ControlWord;				//控制字
	s32 CurrentPosition;		//用户位置反馈
	s32 CurrentVelocity;		//用户实际速度反馈
	s16 ActualTprque;				//实际扭矩，单位0.1%，额定扭矩的占比
	u8 CurrentMode;					//控制模式显示
//	u8 OriginMode;					//回原模式
	
	//u8 relese;						//16位对齐
}PDO_Input;								//位置模式下的RPDO数据格式

extern PDO_Output PDO_TargetInf[SERVO_NODE_ID_NUM];								//存放TPDO模式的TPDO数据
extern u8 E_AllowUpdataSend;																		//允许数据更新后发送

extern void TIM3_Config(void);
extern void EtherCATInit(char *ifname);
extern u8 EtherCAT_LinkSta(void);
extern u8 EtherCAT_LinkErrCheck(void);
extern u16 EtherCAT_SendPDOFinish(void);
extern s32 EtherCAT_GetCurrentPosition(u8 Node_ID);

#endif /* __EtherCAT_App_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
