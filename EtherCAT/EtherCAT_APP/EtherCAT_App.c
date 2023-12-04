/**
  ******************************************************************************
  * @file    EtherCAT.App.c
  * @author  
  * @version V1.1.0
  * @date    
  * @brief   实现EtherCAT应用功能
  */

#include "EtherCAT_App.h"
#include "Parameter.h"
#include "ActionOperate.h"
#include "Error.h"
#include "SpeedControl.h" 
#include "JDZ.h" 

#include <stdio.h>
#include "osal.h"

#include "LAN8742A.h"
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

extern __IO uint8_t EthLinkStatus;													//标志网络链接状态

#define SYNC0TIME 2000																			//过程数据发送周期: 2000 * 1us = 2ms

char PDO_IOmap[1024] = {0};																	//PDO过程数据缓冲区
u8 EtherCatConnectFlag = 0;																	//EtherCat连接状态标志，1成功，2正在连接，0失败
PDO_Output *PDO_OutPuts[SERVO_NODE_ID_NUM] = {NULL};				//指向从机模式的TPDO数据地址
PDO_Input *PDO_InPuts[SERVO_NODE_ID_NUM] = {NULL};					//指向从机模式的RPDO数据地址
PDO_Output PDO_TargetInf[SERVO_NODE_ID_NUM] = {0};					//存放TPDO模式的TPDO数据
u8 E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_DIS;							//允许TPDO数据更新标志
u8 EtherCatPHY_LinkSta = 1;																	//网络物理层连接状态标志

u8 E_LastDCtimeFlag = 0;
int64_t E_LastDCtime = 0;
int64_t E_CurDCtime = 0;
int64_t E_ErrDCtime = 0;

/**
  * @brief  通用定时器3中断初始化，用于定时发送和接受PDO周期帧数据
  * @retval 无
  * @note   定时器3的时钟源为84M
  */
void TIM3_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  							//使能TIM3时钟
	
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;										//定时器分频，us为单位
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; 		//向上计数模式
	TIM_TimeBaseInitStructure.TIM_Period = SYNC0TIME - 1;							//自动重装载值，us为单位
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 											//定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1; 					//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; 								//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);

	TIM_SelectOnePulseMode(TIM3, TIM_OPMode_Repetitive);									//采用单周期模式，便于纠正主站和从站之间的时钟误差

	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); 															//允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); 																								//使能定时器3
}

/*PDO数据帧数据结构列表初始化函数*/
int PDO_ServoSetup(u16 slave)
{
	int retval = 0;
	u16 u16val = 0;
	u8  u8val = 0;
	u32 u32val = 0;
	
	/*TPDO数据结构*/
	u8val = 0;
	retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	u16val = 0x1600;
	retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
	u8val = 1;
	retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

	u8val = 0;
	retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	u32val = 0x607A0020;	//TargetPos，目标位置
	retval += ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x60810020;	//ContourVel，轮廓速度
//	retval += ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x60FF0020;	//TarVelocity，目标速度
//	retval += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x607C0020;	//OriginOffset，原点偏置
//	retval += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x60830020;	//Acc，加速度
//	retval += ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x60840020;	//Dec，减速度
//	retval += ec_SDOwrite(slave, 0x1600, 0x06, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x60850020;	//QukDec，快速停止减速度
//	retval += ec_SDOwrite(slave, 0x1600, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60400010;	//ControlWord，控制字
	retval += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60980008;	//OriginMode，回原点模式
	retval += ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60600008;	//TargetMode，控制模式
	retval += ec_SDOwrite(slave, 0x1600, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60710010;	//TarTorque，目标转矩  
	retval += ec_SDOwrite(slave, 0x1600, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//		u32val = 0x00000008;
//    retval += ec_SDOwrite(slave, 0x1600, 0x0E, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u8val = 0x05;
	retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	
	/*RPDO数据结构*/
	u8val = 0;
	retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	u16val = 0x1a00;
	retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
	u8val = 1;
	retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	
	u8val = 0;
	retval += ec_SDOwrite(slave, 0x1a00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	u32val = 0x60410010;	//StatusWord，状态字
	retval += ec_SDOwrite(slave, 0x1a00, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x60400010;	//ControlWord，控制字
//	retval += ec_SDOwrite(slave, 0x1a00, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60640020;	//CurrentPosition，用户位置反馈
	retval += ec_SDOwrite(slave, 0x1a00, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x606C0020;	//CurrentVelocity，用户实际速度反馈
	retval += ec_SDOwrite(slave, 0x1a00, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60770010;	//ActualTprque，实际扭矩
	retval += ec_SDOwrite(slave, 0x1a00, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60610008;	//CurrentMode，当前模式
	retval += ec_SDOwrite(slave, 0x1a00, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x60980008;	//OriginMode，回零模式
//	retval += ec_SDOwrite(slave, 0x1a00, 0x07, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//		u32val = 0x00000008;
//    retval += ec_SDOwrite(slave, 0x1a00, 0x08, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u8val = 5;
	retval += ec_SDOwrite(slave, 0x1a00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	
	return 1;
}

/*PDO周期帧数据收发处理函数*/
void EtherCAT_RevSendDeal(void)
{
	u16 i = 0;
	u16 j = 0;
	static u8 startRevFlag = 0;																					//启动周期帧读取计数器标志
	static PDO_Output targetInf[SERVO_NODE_ID_NUM] = {0};								//缓存发送数据，确保每次的TPDO数据一致
	
	if(startRevFlag > 2)
	{//必须先进入周期帧发送，才能接收周期帧，暂定发送3次周期帧后开始接收
		ec_receive_processdata(EC_TIMEOUTRET);
		
		//用于周期性纠正主站和从站的时钟误差，非常重要，
		//所有EtherCAT系统必须带有这个功能，
		//否则会出现时钟不对齐问题，导致电机运行丢帧现象
		if(E_LastDCtimeFlag == 1)
		{
			E_CurDCtime = (ec_DCtime % 0x100000000);
			if(E_LastDCtime > 0x100000000 - 2 * SYNC0TIME * 1000 && E_CurDCtime < 2 * SYNC0TIME * 1000)
			{//计数器到最大值后从0开始循环但目标位置还在最大值
				E_LastDCtime = E_LastDCtime - 0x100000000;
			}
			else if(E_CurDCtime > 0x100000000 - 2 * SYNC0TIME * 1000 && E_LastDCtime < 2 * SYNC0TIME * 1000)
			{//计数器到最大值但目标位置已经从0开始循环
				E_CurDCtime = E_CurDCtime - 0x100000000;
			}
			E_ErrDCtime = SYNC0TIME * 1000 + E_LastDCtime - E_CurDCtime;
			TIM3->ARR = E_ErrDCtime / 1000 - 1;
			E_LastDCtime = (E_LastDCtime + SYNC0TIME * 1000) % 0x100000000;
			
//			if(JDZ_Parameter.Server == 0 || JDZ_Parameter.Server == 6)
//			{//禾川、超川同步时钟计数器为32位
//				E_CurDCtime = ec_DCtime;
//				if(E_CurDCtime < E_LastDCtime)
//				{
//					E_LastDCtime = E_CurDCtime;
//				}
//				E_ErrDCtime = SYNC0TIME * 1000 + E_LastDCtime - E_CurDCtime;
//				TIM3->ARR = E_ErrDCtime / 1000 - 1;
//				E_LastDCtime = ec_DCtime + SYNC0TIME * 1000;
//			}
//			else
//			{//汇川等同步时钟计数器为64位
//				E_CurDCtime = ec_DCtime;
//				E_ErrDCtime = SYNC0TIME * 1000 + E_LastDCtime - E_CurDCtime;
//				TIM3->ARR = E_ErrDCtime / 1000 - 1;
//				E_LastDCtime = E_LastDCtime + SYNC0TIME * 1000;
//			}
		}
		if(Servo_InitFinish == 1)
		{
			if(E_LastDCtimeFlag == 0)
			{
				E_LastDCtimeFlag = 1;
				E_LastDCtime = (ec_DCtime + SYNC0TIME * 1000) % 0x100000000;
//				E_LastDCtime = ec_DCtime + SYNC0TIME * 1000;
			}
		}
		
		for(i=1; i<=ec_slavecount; i++)
		{
			for(j=1; j<SERVO_NODE_ID_NUM; j++)
			{
				if(ServoNodeID[j] == i)
				{//根据轴ID读取相应的RPDO数据
					PDO_Actual_Status[j - 1] = PDO_InPuts[i]->StatusWord;
					PDO_Cur_Position[j - 1] = PDO_InPuts[i]->CurrentPosition / Axsis_ParPosChange - JDZ_ZeroPosition[j - 1];
//					PDO_Cur_Tprque[j - 1] = PDO_InPuts[i]->ActualTprque;				//当前扭矩反馈，可用于防撞
//					PDO_Cur_Controlword[j - 1] = PDO_InPuts[i]->ControlWord;
					PDO_Cur_Mode[j - 1] = PDO_InPuts[i]->CurrentMode;
					PDO_Cur_Speed[j - 1] = PDO_InPuts[i]->CurrentVelocity / Axsis_ParVelChange;
					PDO_FirstGetPosition[j - 1] = 1;
					
					if(PDO_Actual_StatusUpdate[j - 1] > 0)
					{
						PDO_Actual_StatusUpdate[j - 1]++;
						if(PDO_Actual_StatusUpdate[j - 1] > PDO_Actual_StatusUpdateTime[j - 1])
						{//状态更新延时 1*16ms=80ms ，确保状态字时最新是命令的响应
							PDO_Actual_StatusUpdate[j - 1] = 0;
						}
					}
				}
			}
		}
	}
	else
	{
		startRevFlag++;
	}
	
	g_AxisActionNextPosRead();
	
	if(E_AllowUpdataSend == E_ALLOW_UPDATA_SEND_EN)
	{//需要更新TPDO数据时，完成数据更新
		for(i=1; i<=ec_slavecount; i++)
		{
			targetInf[i].TargetPos = PDO_TargetInf[i].TargetPos;
//			targetInf[i].ContourVel = PDO_TargetInf[i].ContourVel;
//			targetInf[i].TarVelocity = PDO_TargetInf[i].TarVelocity;
//			targetInf[i].OriginOffset = PDO_TargetInf[i].OriginOffset;
//			targetInf[i].Acc = PDO_TargetInf[i].Acc;
//			targetInf[i].Dec = PDO_TargetInf[i].Dec;
//			targetInf[i].QukDec = PDO_TargetInf[i].QukDec;
			targetInf[i].ControlWord = PDO_TargetInf[i].ControlWord;
			targetInf[i].OriginMode = PDO_TargetInf[i].OriginMode;
			targetInf[i].TargetMode = PDO_TargetInf[i].TargetMode;
			targetInf[i].TarTorque = PDO_TargetInf[i].TarTorque;//
		}
		
		E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_DIS;
	}
	
	for(i=1; i<=ec_slavecount; i++)
	{
		PDO_OutPuts[i]->TargetPos = targetInf[i].TargetPos;
//		PDO_OutPuts[i]->ContourVel = targetInf[i].ContourVel;
//		PDO_OutPuts[i]->TarVelocity = targetInf[i].TarVelocity;
//		PDO_OutPuts[i]->OriginOffset = targetInf[i].OriginOffset;
//		PDO_OutPuts[i]->Acc = targetInf[i].Acc;
//		PDO_OutPuts[i]->Dec = targetInf[i].Dec;
//		PDO_OutPuts[i]->QukDec = targetInf[i].QukDec;
		PDO_OutPuts[i]->OriginMode = targetInf[i].OriginMode;
		PDO_OutPuts[i]->ControlWord = targetInf[i].ControlWord;
		PDO_OutPuts[i]->TargetMode = targetInf[i].TargetMode;
		PDO_OutPuts[i]->TarTorque = targetInf[i].TarTorque;
		
	}
	
	ec_send_processdata();
	ec_pdo_outframe();
}

void EtherCATInit(char *ifname)
{
	u16 i = 0;
	u16 j = 0;
//	s16 temp_s16 = 0;
	s32 temp_s32 = 0;
	u16 temp_u16 = 0;
//	s32 temp_u32 = 0;
	s8  temp_s08 = 0;
	int len = 0;
	u32 decTemp = 0;
	u32 decQueTemp = 0;
	u32 accTemp = 0;
	u32 oriDecSpTemp = 0;
	u32 oriFindSpTemp = 0;
//	s32 OriginOffsetTemp = 0;
//	u32 maxRpm = 0;
	
  EtherCatConnectFlag = 2;						//正在链接
	
	if(ec_init(ifname))									//初始化SOEM，socket绑定到ifname（网卡名称）
	{
		if(ec_config_init(TRUE) > 0)			//确定连接的从机个数，有设备连接再进行从机初始化
		{
			for(i = 1; i <= ec_slavecount; i++)
			{//为从机绑定PDO结构
				ec_slave[i].PO2SOconfig = &PDO_ServoSetup;
			}
			delay_ms(10);
			
			/*需要进行SDO设置的参数须在此处设置*/
			for(i = 1; i <= ec_slavecount; i++)
			{
				/*确定每个节点的ID对应的EtherCAT从站编号*/
				len = sizeof(temp_u16);
				switch(JDZ_Parameter.Server)
				{
					case 0://禾川电机，P09.18(2109-13H)设置EtherCAT通信地址，必须连续从1开始；从机ID设置P09.00(2109-01H)
						ec_SDOread(i, 0x2109, 0x01, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						break;
					case 1://汇川电机，H0E.21(200E-16H)设置EtherCAT通信地址，必须连续从1开始；从机ID设置H0E.00(200E-01H)
						ec_SDOread(i, 0x200E, 0x01, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						break;
					case 2://迈信电机，无法读取，必须按X-Y-Z-O的顺序接网线
						//ec_SDOread(i, 0x2300, 0x00, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						temp_u16 = i;
						break;
					case 3://雷赛电机，从机ID设置PA0.23(0x2023-00H)PA0.24=1
						ec_SDOread(i, 0x2023, 0x00, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						break;
					case 4://信捷电机，从机ID设置P7.00(2700-00H)
						ec_SDOread(i, 0x2700, 0x00, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						break;
					case 5://台邦电机，读取从机ID设置Pn010(2010-00H)
						ec_SDOread(i, 0x2010, 0x00, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						break;
					case 6://超川电机，读取从机ID设置PA82(2052-00H)
						ec_SDOread(i, 0x2052, 0x00, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						break;
					default://X默认，禾川电机，P09.18(2109-13H)设置EtherCAT通信地址，必须连续从1开始；从机ID设置P09.00(2109-01H)
						ec_SDOread(i, 0x2109, 0x01, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						break;
				}
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				if(temp_u16 > 0 && temp_u16 < SERVO_NODE_ID_NUM)
				{
					ServoNodeID[temp_u16] = i;						//记录各个轴对应的EtherCAT编号
					Servo_ConnectSta[temp_u16 - 1] = 1;		//记录每个轴的连接状态，1连接，2未连接
				}
				else
				{
					ServoNodeID[temp_u16] = 0;
					Servo_ConnectSta[temp_u16 - 1] = 0;
				}
			}
			
			/*需要进行SDO设置的参数须在此处设置*/
			for(i = 1; i <= ec_slavecount; i++)
			{
				/*设置快速停止减速度模式*/
//				temp_s16 = 6;
//				ec_SDOwrite(i, SERVO_STOP_DEC_MODE, 0x00, FALSE, sizeof(temp_s16), &temp_s16, EC_TIMEOUTRXM);
//				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				
				/*设置轮廓暂停减速度模式*/
//				temp_s16 = 2;
//				ec_SDOwrite(i, SERVO_PAUSE_DEC_MODE, 0x00, FALSE, sizeof(temp_s16), &temp_s16, EC_TIMEOUTRXM);
//				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				
				for(j = 0; j < Axis_Num + Ext_Axis_Num; j++)
				{
					if(ServoNodeID[j + 1] == i)
					{
						if(j < Axis_Num)
						{
							decQueTemp = JXS_Parameter.Accelerate.Time[j];//0824
							decTemp =	JXS_Parameter.Accelerate.Time[j];
							accTemp = JXS_Parameter.Accelerate.Time[j];
							
							oriDecSpTemp = JXS_Parameter.AxisOriginSpeed[j] * 50;
							oriFindSpTemp = CANOPEN_ORIGIN_SPEED_MIN;
						}
						else
						{//扩展轴没有机械回零，随便给个值
							decQueTemp = ExtendAix_Parameter[j].E_AccTime;//0824
							decTemp =	ExtendAix_Parameter[j].E_AccTime;
							accTemp = ExtendAix_Parameter[j].E_AccTime;
							
							oriDecSpTemp = JXS_Parameter.AxisOriginSpeed[0] * 50;
							oriFindSpTemp = CANOPEN_ORIGIN_SPEED_MIN;
						}
						break;
					}
				}
					decQueTemp = (2050 - decQueTemp) * 5 * Axsis_ParVelChange * 2;
					decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
					accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
				
					oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
					oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
//				switch(JDZ_Parameter.Server)
//				{
//					case 0://禾川电机
//						decQueTemp = (2050 - decQueTemp) * 5 * Axsis_ParVelChange * 2;
//						decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
//						accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
//					
//						oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
//						oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
////						OriginOffsetTemp = OriginOffsetTemp * Axsis_ParPosChange;
//						break;
//					case 1://汇川电机
//						decQueTemp = (2050 - decQueTemp) * 5 * Axsis_ParVelChange * 2;
//						decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
//						accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
//					
//						oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
//						oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
////						OriginOffsetTemp = OriginOffsetTemp * Axsis_ParPosChange;
//						break;
//					case 2://迈信电机
//						decQueTemp = (2050 - decQueTemp) * 5 * Axsis_ParVelChange * 2;
//						decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
//						accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
//					
//						oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
//						oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
////						OriginOffsetTemp = OriginOffsetTemp * Axsis_ParPosChange;
//						break;
//					case 3://雷赛电机
//						decQueTemp = (2050 - decQueTemp) * 5 * Axsis_ParVelChange * 2;
//						decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
//						accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
//					
//						oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
//						oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
////						OriginOffsetTemp = OriginOffsetTemp * Axsis_ParPosChange;
//						break;
//					case 4://信捷电机
//						decQueTemp = (2050 - decQueTemp) * 5 * Axsis_ParVelChange * 2;
//						decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
//						accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
//					
//						oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
//						oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
////						OriginOffsetTemp = OriginOffsetTemp * Axsis_ParPosChange;
//						break;
//					case 5://台邦电机
//						decQueTemp = (2050 - decQueTemp) * 5 * Axsis_ParVelChange * 2;
//						decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
//						accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
//					
//						oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
//						oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
////						OriginOffsetTemp = OriginOffsetTemp * Axsis_ParPosChange;
//						break;
//					default:
//						decQueTemp = (2001 - decTemp) * 5 * Axsis_ParVelChange * 2;
//						decTemp = (2001 - decTemp) * 5 * Axsis_ParVelChange;
//						accTemp = (2001 - accTemp) * 5 * Axsis_ParVelChange;
//					
//						oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
//						oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
////						OriginOffsetTemp = OriginOffsetTemp * Axsis_ParPosChange;
//						break;
//				}
				
//				/*设置快速停止时的减速度*/
//				ec_SDOwrite(i, SERVO_STOP_DEC, 0x00, FALSE, sizeof(decQueTemp), &decQueTemp, EC_TIMEOUTRXM);
//				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				
				/*设置回原点加速度*/
				ec_SDOwrite(i, SERVO_BACK_ACC, 0x00, FALSE, sizeof(accTemp), &accTemp, EC_TIMEOUTRXM);
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				
				/*设置回零搜索减速点速度*/
				ec_SDOwrite(i, SERVO_BACK_HOME_SP, 0x01, FALSE, sizeof(oriDecSpTemp), &oriDecSpTemp, EC_TIMEOUTRXM);
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				/*设置回零搜索原点速度*/
				ec_SDOwrite(i, SERVO_BACK_HOME_SP, 0x02, FALSE, sizeof(oriFindSpTemp), &oriFindSpTemp, EC_TIMEOUTRXM);
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
//				/*设置原点偏置*/
//				ec_SDOwrite(i, SERVO_ORIGIN_OFFSET, 0x00, FALSE, sizeof(OriginOffsetTemp), &OriginOffsetTemp, EC_TIMEOUTRXM);
//				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				
				/*读取当前位置*/
				len = sizeof(temp_s32);
				ec_SDOread(i, SERVO_USER_POSITION, 0x00, FALSE, &len, &temp_s32, EC_TIMEOUTRXM);
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				PDO_TargetInf[i].TargetPos = temp_s32;
//				/*设置目标位置*/
//				ec_SDOwrite(i, SERVO_TARGET_POSITION, 0x00, FALSE, sizeof(temp_s32), &temp_s32, EC_TIMEOUTRXM);
//				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				/*设置目标速度*/
				temp_s32 = 0;
				ec_SDOwrite(i, SERVO_TARGET_SPEED, 0x00, FALSE, sizeof(temp_s32), &temp_s32, EC_TIMEOUTRXM);
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				/*读取当前控制字*/
				len = sizeof(temp_u16);
				ec_SDOread(i, SERVO_CONTROL_WORD, 0x00, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				PDO_TargetInf[i].ControlWord = temp_u16;
				/*读取当前模式*/
				len = sizeof(temp_s08);
				ec_SDOread(i, SERVO_WORK_MODE, 0x00, FALSE, &len, &temp_s08, EC_TIMEOUTRXM);
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				PDO_TargetInf[i].TargetMode = temp_s08;
			}
			
			/*EtherCAT的DC时钟同步初始化*/
			ec_configdc();
			//ec_dcsync0(1, TRUE, SYNC0TIME, 250000);			//设置同步周期，以及帧周期误差
			for(i = 1; i <= ec_slavecount; i++)
			{
				ec_dcsync0(i, TRUE, SYNC0TIME, SYNC0TIME * 1000 / 8);			//设置同步周期，以及帧周期误差，帧误差一般为帧周期的八分之一
			}
			Delay_ms(100);
			
			/*初始化从机PDO结构*/
			ec_set_pdo_queue(ec_config_map(&PDO_IOmap),3);
			
			/*设置从机切换到安全操作模式*/
			ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
			ec_readstate();
			Delay_ms(2000);																//延时必须达到这个时间，否则电机使能有问题
			
			/*映射RPDO和TPDO数据结构地址*/
			for(i = 1; i <= ec_slavecount; i++)
			{
				PDO_OutPuts[i] = (PDO_Output *)ec_slave[i].outputs;
				PDO_InPuts[i]  = (PDO_Input *)ec_slave[i].inputs;
			}
			
			/*设置从机切换到操作模式*/
			ec_slave[0].state = EC_STATE_OPERATIONAL;
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET);
			ec_writestate(0);
			
			/*确定所有从机已进入操作模式*/
			j = 0;
			while(j < 5)
			{
				j++;
				for(i = 0; i <= ec_slavecount; i++)
				{
					if(ec_slave[i].state != EC_STATE_OPERATIONAL)
					{
						break;
					}
				}
				if(i > ec_slavecount)
				{
					break;
				}
				
				ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE * 4);
				ec_readstate();
			}
			
			if(j < 5 && ec_slave[1].state == EC_STATE_OPERATIONAL)
			{/*从机初始化成功后，进行主机内部相关变量初始化*/
				ServoNodeID[0] = 0;																	//对象0为主机站号，通信编号设为0
				EtherCatConnectFlag = 1;														//链接成功标志
				
				E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_EN;					//数据同步发送一帧，确保从机与主机的PDO数据保持一直
				while(E_AllowUpdataSend == E_ALLOW_UPDATA_SEND_EN);
			}
			else
			{
				EtherCatConnectFlag = 0;
			}
		}
		else
		{
			EtherCatConnectFlag = 0;
		}
	}
	else
	{
		EtherCatConnectFlag = 0;
	}
	
	if(EtherCatConnectFlag == 0)
	{//连接失败时，恢复初始状态且关闭EtherCAT的应用层连接
		ec_slave[0].state = EC_STATE_INIT;
		EtherCatPHY_LinkSta = 0;
		ec_writestate(0);
		ec_close();
		
		//伺服总线连接失败
//		if(Robot_Error_Num == 0 || Robot_Error_Num > E_SERVO_NOT_CON)
//		{
//			Robot_Error_Num = E_BUSCONNECTFAIL;
//		}
		Robot_Error_Data[9] |= 0x10;
	}
}

/**
  * @brief  EtherCAT_LinkSta
  * @param  无
  * @retval EtherCAT连接状态，1成功，2正在连接，0失败
  */
u8 EtherCAT_LinkSta(void)
{
	if(EtherCatPHY_LinkSta > 0 && GET_PHY_LINK_STATUS() == 0)
	{
		EtherCatPHY_LinkSta = 0;
		EtherCatConnectFlag = 0;
		
		//伺服总线连接失败
//		if(Robot_Error_Num == 0 || Robot_Error_Num > E_SERVO_NOT_CON)
//		{
//			Robot_Error_Num = E_BUSCONNECTFAIL;
//		}
		Robot_Error_Data[9] |= 0x10;
	}
	return EtherCatConnectFlag;
}

/**
  * @brief  EtherCAT_LinkErr
  * @param  无
  * @retval EtherCAT报警检测
  */
u8 EtherCAT_LinkErrCheck(void)
{
	EtherCAT_LinkSta();
	
	if(EtherCatPHY_LinkSta == 0 || EtherCatConnectFlag == 0)
	{
		//伺服总线连接失败
//		if(Robot_Error_Num == 0 || Robot_Error_Num > E_SERVO_NOT_CON)
//		{
//			Robot_Error_Num = E_BUSCONNECTFAIL;
//		}
		Robot_Error_Data[9] |= 0x10;
		return 1;
	}
	
	return 0;
}

/**
  * @brief  EtherCAT_SendPDOFinish
  * @param  无
  * @retval EtherCAT发送PDO数据完成
  */
u16 EtherCAT_SendPDOFinish(void)
{
	E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_EN;
	while(E_AllowUpdataSend == E_ALLOW_UPDATA_SEND_EN);
	
	return 0;
}

/*读取编码器当前位置*/
s32 EtherCAT_GetCurrentPosition(u8 Node_ID)
{
	return PDO_InPuts[ServoNodeID[Node_ID]]->CurrentPosition;
}

/**
  * @brief  定时器3中断服务函数
  * @param  无
  * @retval 无
  */
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
		/*读取Ethernet网络连接是否正常*/
		if(EtherCatPHY_LinkSta > 0)
		{
			if(EtherCatConnectFlag == 1)
			{//连接成功后，发送周期帧
				EtherCAT_RevSendDeal();
			}
			else if(EtherCatConnectFlag == 2)
			{//正在连接时发送周期帧
				ec_send_processdata();
				ec_pdo_outframe();
			}
			else
			{//网络连接失败后需要每次清除发送允许标志
				E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_DIS;
			}
		}
		else
		{//网络连接失败后需要每次清除发送允许标志
			E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_DIS;
		}
		TIM_Cmd(TIM3,ENABLE); //使能定时器3
	}
	
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
