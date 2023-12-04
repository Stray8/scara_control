/*********************** (C) COPYRIGHT 2021 ZH manipulator Team ************************
* File Name          : ExtendCom.c
* Author             : 
* Version            : V1.0.0
* Date               : 
* Description        : 处理通过485访问本设备的协议处理函数
***************************************************************************************/
#include "stm32f4xx.h"
#include "ExtendCom.h"
#include "Delay.h"
#include "StatusControl.h"
#include "SpeedControl.h"
#include "Auto_2.h"
#include "Error.h"
#include "w25qxx.h"
#include "BackToOrigin.h"
#include "Auto.h"
#include "in.h"
#include "out.h"
#include "SignalWatch.h"
#include "ActionOperate.h"
#include "Parameter.h"
#include "JDZ.h"
#include "Usart.h" 

/*485通信的数据缓冲区长度*/
#define EXTEND_BUF_LEN			50						//485通信的数据缓冲区长度

u8  ExtendRecDataFlag = FALSE;							//开始接收数据标志，0待接收数据，1开始接收数据，2接收数据完成

u8  ExtendSendDataBuf[EXTEND_BUF_LEN] = {0};			//发送数据缓存
u8  ExtendRecDataBuf[EXTEND_BUF_LEN] = {0};   			//接收数据缓存
u8  ExtendRecDataLen = 0;				           		//记录接收到的数据长度

/*须在应用程序中处理以下几个变量*/
u8  ExtendEquipID = 31;				           			//用于存放本设备ID，须在控制器上进行保持设置，默认ID = 31

u16 ExtendProgramNum = 0;								//通过通信选中的程序号，用于通知显示屏要切换的新程序编号,0表示程序不改变，范围1~14

u8  ExtendEmergencyStop = 0;				          	//通过通信急停设备标志，直接在控制器上进行处理

u8  ExtendYieldChange = 0;								//通过通信修改产量相关变量标志，用于通知显示屏来获取新的产量

u8  ExtendPosChangeNum = 0;								//通过通信修改坐标的编号，用于通知显示屏要获取的点编号,0表示没有点改变，范围1~40

u8  ExtendStateChange = 0;								//通过通信修改机械手状态，ExtendStateChange = 1复位、= 2回零、=3启动、=4停止、=5暂停

u8  ExtendCancleAlarm = 0;								//通过通信取消机械手报警

u8  ExtendSerialNum = 0;								  //通过通信烧录序列号

/*MODBUS标准的CRC校验**/
u16 CRC_chk(u8 * data,u8 length)
{
	int j = 0;
	u16 CRC_reg = 0xFFFF;
	
	while(length--)
	{
		CRC_reg ^= *data++;
		for(j=0; j<8; j++)
		{
			if(CRC_reg & 0x01)
			{
				CRC_reg = ( CRC_reg >> 1) ^ 0xA001;
			}
			else
			{
				CRC_reg = (CRC_reg >> 1);
			} 
		} 
	}
	return CRC_reg;
}

/*写命令的应答发送函数*/
void USART2_ModbusSendAnswer(u8 ID, u8 funCode, u16 addr, u16 dataLen)
{
	u8 i = 0;
	u16 CRC_16 = 0;
	
	ExtendSendDataBuf[0] = ID;  						 	//地址
	ExtendSendDataBuf[1] = funCode;		     	 			//Modbus指令
	ExtendSendDataBuf[2] = (u8)(addr >> 8); 		        //地址高字节
	ExtendSendDataBuf[3] = (u8)(addr); 		         		//地址低字节
	ExtendSendDataBuf[4] = (u8)(dataLen >> 8); 		        //长度高字节
	ExtendSendDataBuf[5] = (u8)(dataLen); 		         	//长度低字节
	
	CRC_16 = CRC_chk(ExtendSendDataBuf, 6);
	
	ExtendSendDataBuf[6] = (u8)CRC_16;						//CRC校验字低8位
	ExtendSendDataBuf[7] = (u8)(CRC_16 >> 8);				//CRC校验字高8位
	
	USART_ClearFlag(USART2,USART_FLAG_TC);
	DIROUT485_H;												//写方向
	for(i=0; i<8; i++)	//8
	{
		USART2->DR = ExtendSendDataBuf[i];
		while((USART2->SR & 0X40) == 0);	
	}
	DIROUT485_L;												//读方向
}

/*读命令的数据反馈发送函数*/
void USART2_ModbusSendData(u8 ID, u8 funCode, u8 *data, u16 len)
{	
	u8 i = 0;
	u16 CRC_16 = 0;
	
	ExtendSendDataBuf[0] = ID;  						 		//地址
	ExtendSendDataBuf[1] = funCode;		     	 				//Modbus指令
	ExtendSendDataBuf[2] = len; 		         				//字节个数
	for(i=0; i<len; i++)
	{
		ExtendSendDataBuf[3 + i] = data[i];
	}
	
	CRC_16 = CRC_chk(ExtendSendDataBuf, len + 3);
	
	ExtendSendDataBuf[len + 3] = (u8)CRC_16;					//CRC校验字低8位
	ExtendSendDataBuf[len + 4] = (u8)(CRC_16 >> 8);				//CRC校验字高8位
		
	USART_ClearFlag(USART2,USART_FLAG_TC);
	DIROUT485_H;                            						//写方向
	for(i=0; i<len+5; i++)	//5
	{
		USART2->DR = ExtendSendDataBuf[i];
		while((USART2->SR & 0X40) == 0);
	}
	DIROUT485_L;													//读方向
}

/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : 485扩展模块的串口中断函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART2_IRQHandler(void)
{
	u8 data = 0;
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{	
		USART_ClearFlag(USART2, USART_IT_RXNE);	
		
		data = USART_ReceiveData(USART2);
		if(ExtendRecDataFlag == EXTEND_WAIT_REC && (data == ExtendEquipID || data == EXTEND_RADIO_ID))
		{//本设备待接收数据且数据ID为本机或是广播
			ExtendRecDataBuf[0] = data;
			ExtendRecDataLen = 1;
			ExtendRecDataFlag = EXTEND_START_REC;
		}
		else if(ExtendRecDataFlag == EXTEND_START_REC && (ExtendRecDataBuf[0] == ExtendEquipID || ExtendRecDataBuf[0] == EXTEND_RADIO_ID))
		{//本设备已接收数据且数据ID为本机
			ExtendRecDataBuf[ExtendRecDataLen] = data;
			ExtendRecDataLen++;
			if(ExtendRecDataLen > 2)
			{
				if(ExtendRecDataBuf[1] == EXTEND_FUN_READ)
				{//接收读命令数据
					if(ExtendRecDataLen == 8)	//8
					{//接收完成
						ExtendRecDataFlag = EXTEND_END_REC;
					}
					else if(ExtendRecDataLen > 8)	//8
					{//接收出错，清除长度、标志、数据头
						ExtendRecDataLen = 0;
						ExtendRecDataFlag = EXTEND_WAIT_REC;
						ExtendRecDataBuf[0] = 0;
					}
				}
				else if(ExtendRecDataBuf[1] == EXTEND_FUN_WRITE)
				{//接收写命令数据
					if(ExtendRecDataLen == 9 && ExtendRecDataBuf[6] > 64)		//9
					{//接收到的数据长度超过缓冲区大小，报错
						ExtendRecDataLen = 0;
						ExtendRecDataFlag = EXTEND_WAIT_REC;
						ExtendRecDataBuf[0] = 0;
					}
					else if(ExtendRecDataLen == 9 + ExtendRecDataBuf[6])		//9
					{//接收完成确认
						ExtendRecDataFlag = EXTEND_END_REC;
					}
					else if(ExtendRecDataLen > 9 + ExtendRecDataBuf[6])			//9
					{//接收出错，清除长度、标志、数据头
						ExtendRecDataLen = 0;
						ExtendRecDataFlag = EXTEND_WAIT_REC;
						ExtendRecDataBuf[0] = 0;
					}
				}
				else
				{//接收出错，清除长度、标志、数据头
					ExtendRecDataLen = 0;
					ExtendRecDataFlag = EXTEND_WAIT_REC;
					ExtendRecDataBuf[0] = 0;
				}
			}
			else if(ExtendRecDataLen >= EXTEND_BUF_LEN)
			{//数据长度超过缓冲区大小，接收中断，清除标志和长度
				ExtendRecDataLen = 0;
				ExtendRecDataFlag = EXTEND_WAIT_REC;
				ExtendRecDataBuf[0] = 0;
			}
		}
	}
}

/*******************************************************************************
* Function Name  : ExtendGetSerialNum
* Description    : 得到序列号数据
* Input          : sendBuff 发送缓冲区
* Output         : None
* Return         : None
*******************************************************************************/
void ExtendGetSerialNum(u8 *sendBuff)
{
	u16 i = 0;
	for(i=0;i<12;i++)
	{
		sendBuff[i] = Internet_Parameter.Sequence[i];
	}
	
}

/*******************************************************************************
* Function Name  : ExtendGetSystemVer
* Description    : 得到系统版本号数据
* Input          : sendBuff 发送缓冲区
* Output         : None
* Return         : None
*******************************************************************************/
void ExtendGetSystemVer(u8 *sendBuff)
{
	sendBuff[0] = 'G';
	sendBuff[1] = '7';
	sendBuff[2] = '.';
	sendBuff[3] = '1';
	sendBuff[4] = '6';
	sendBuff[5] = '.';
	sendBuff[6] = '1';
	sendBuff[7] = '0';
	sendBuff[8] = ' ';
	sendBuff[9] = ' ';
	sendBuff[10] = ' ';
	sendBuff[11] = ' ';
}

/*******************************************************************************
* Function Name  : ExtendGetProductFac
* Description    : 得到厂家地址数据
* Input          : sendBuff 发送缓冲区
* Output         : None
* Return         : None
*******************************************************************************/
void ExtendGetProductFac(u8 *sendBuff)
{
	sendBuff[0] = 'H';
	sendBuff[1] = 'H';
	sendBuff[2] = 'M';
	sendBuff[3] = 'O';
	sendBuff[4] = 'C';
	sendBuff[5] = 'O';
	sendBuff[6] = 'N';
	sendBuff[7] = ' ';
	sendBuff[8] = ' ';
	sendBuff[9] = ' ';
	sendBuff[10] = ' ';
	sendBuff[11] = ' ';
}

/*******************************************************************************
* Function Name  : ExtendGetData_u32
* Description    : 得到u32位数据
* Input          : sendBuff 发送缓冲区
* Output         : None
* Return         : None
*******************************************************************************/
void ExtendGetData_u32(u8 *sendBuff, u32 data)
{
	sendBuff[0] = (u8)((data >> 8) & 0x000000ff);
	sendBuff[1] = (u8)(data & 0x000000ff);
	sendBuff[2] = (u8)((data >> 24) & 0x000000ff);
	sendBuff[3] = (u8)((data >> 16) & 0x000000ff);
}

/*******************************************************************************
* Function Name  : ExtendGetData_s32
* Description    : 得到s32位数据
* Input          : sendBuff 发送缓冲区
* Output         : None
* Return         : None
*******************************************************************************/
void ExtendGetData_s32(u8 *sendBuff, s32 data)
{
	sendBuff[0] = (u8)((data >> 8) & 0x000000ff);
	sendBuff[1] = (u8)(data & 0x000000ff);
	sendBuff[2] = (u8)((data >> 24) & 0x000000ff);
	sendBuff[3] = (u8)((data >> 16) & 0x000000ff);
}

/*******************************************************************************
* Function Name  : ExtendGetData_u16
* Description    : 得到u16位数据
* Input          : sendBuff 发送缓冲区
* Output         : None
* Return         : None
*******************************************************************************/
void ExtendGetData_u16(u8 *sendBuff, u16 data)
{
	sendBuff[0] = (u8)((data >> 8) & 0x000000ff);
	sendBuff[1] = (u8)(data & 0x000000ff);
}

/*******************************************************************************
* Function Name  : ExtendGetData_s16
* Description    : 得到s16位数据
* Input          : sendBuff 发送缓冲区
* Output         : None
* Return         : None
*******************************************************************************/
void ExtendGetData_s16(u8 *sendBuff, s16 data)
{
	sendBuff[0] = (u8)((data >> 8) & 0x000000ff);
	sendBuff[1] = (u8)(data & 0x000000ff);
}

/*******************************************************************************
* Function Name  : ExtendRecDataDeal
* Description    : 处理扩展模块485接收到的数据，在主循环中调用该函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ExtendRecDataDeal(void)
{
	u16 i = 0;
	u8 m = 0;
	u16 receiveCRC = 0;
	u16 calculCRC = 0;
	u8  funCode = 0;
	u16 receiveAddr = 0;
	u16 dataLen = 0;
	u8 sendDataBuf[EXTEND_BUF_LEN] = {0};
	u8 sendDataLen = 0;
//	u16 serialLen = 12;											//序列号长度
	u16 receiveAddr_Temp = 0;
	u16 pointNum = 0;
//	u8 writeDataBuf[EXTEND_BUF_LEN] = {0};
	u16 inportNum = 0;
	u16 outportNum = 0;
	u16 alarmNum = 0;

	if(ExtendRecDataFlag == EXTEND_END_REC)
	{
		receiveCRC = ((u16)ExtendRecDataBuf[ExtendRecDataLen - 1] << 8) + ExtendRecDataBuf[ExtendRecDataLen - 2];
		calculCRC = CRC_chk(ExtendRecDataBuf, ExtendRecDataLen - 2);
		if(receiveCRC == calculCRC)
		{//CRC校验数据
			funCode = ExtendRecDataBuf[1];
			receiveAddr = ((u16)ExtendRecDataBuf[2] << 8) + ExtendRecDataBuf[3];
			dataLen = ((u16)ExtendRecDataBuf[4] << 8) + ExtendRecDataBuf[5];
			sendDataLen = dataLen * 2;
			if(funCode == EXTEND_FUN_READ)
			{//读取命令处理部分
				/*转换地址方便处理*/
				receiveAddr_Temp = receiveAddr;
				if(EXT_ADDR_POINT_1_X <= receiveAddr && receiveAddr <= EXT_ADDR_POINT_1_X + 0x0040 * 32)
				{
					receiveAddr = EXT_ADDR_POINT_1_X;
				}
				else if(EXT_ADDR_OUT_CONTROL <= receiveAddr && receiveAddr <= EXT_ADDR_OUT_CONTROL + 0x00FF)
				{
					receiveAddr = EXT_ADDR_OUT_CONTROL;
				}
				else if(EXT_ADDR_INPUT_STA <= receiveAddr && receiveAddr <= EXT_ADDR_INPUT_STA + 0x00FF)
				{
					receiveAddr = EXT_ADDR_INPUT_STA;
				}
				switch(receiveAddr)
				{
					case EXT_ADDR_SERIAL_NUM://读取当前设备序列号
						ExtendGetSerialNum(sendDataBuf);
						break;
					case EXT_ADDR_SYSTEM_VER://读取当前设备系统版本号
						ExtendGetSystemVer(sendDataBuf);
						break;
					case EXT_ADDR_PRODUC_FAC://读取当前设备生产厂家
						ExtendGetProductFac(sendDataBuf);
						break;
					case EXT_ADDR_ONCE_T://读取当前产品单次加工时间
						ExtendGetData_u32(sendDataBuf, Program_RunTime);
						break;
					case EXT_ADDR_POWER_T://读取开机时间
						ExtendGetData_u32(sendDataBuf, m_PowerOnTimeTotal);
						break;
					case EXT_ADDR_RUN_T://读取运行时间
						ExtendGetData_u32(sendDataBuf, m_ProRunTimeTotal);
						break;
					case EXT_ADDR_TOTAL_POWER_T://读取总开机时间
						ExtendGetData_u32(sendDataBuf, m_PowerOnTimeCumulate);
						break;
					case EXT_ADDR_TOTAL_RUN_T://读取总运行时间
						ExtendGetData_u32(sendDataBuf, m_ProRunTimeCumulate);
						break;
					case EXT_ADDR_RESET_STA://读取当前设备是否复位完成
						sendDataBuf[0] = 0;
						if(Robot_Auto_Reset == TRUE)
						{
							sendDataBuf[1] = 1;
						}
						sendDataLen = 2;
						break;
					case EXT_ADDR_BACK_ORI_STA://读取当前设备是否回零完成
						sendDataBuf[0] = 0;
						if(Origin_Backed == TRUE)
						{
							sendDataBuf[1] = 1;
						}
						sendDataLen = 2;
						break;
					case EXT_ADDR_RUN_STA://读取当前运行状态
						if(g_Auto_Reset_Flag == TRUE)
						{//复位中
							ExtendGetData_u16(sendDataBuf, EXT_RESET);
						}
						else if(Back_Origin_Flag == TRUE)
						{//回零中
							ExtendGetData_u16(sendDataBuf, EXT_BACK);
						}
						else if(Error_Status != NO_ERROR)
						{//有报警，返回报警状态
							ExtendGetData_u16(sendDataBuf, EXT_ERR);
						}
						else if(g_AutoStatue == AUTO_RUNNING)
						{//运行中，返回运行状态
							ExtendGetData_u16(sendDataBuf, EXT_RUNNING);
						}
						else if(g_AutoStatue == AUTO_PAUSE)
						{//暂停中，返回暂停状态
							ExtendGetData_u16(sendDataBuf, EXT_PAUSE);
						}
						else if(g_AutoStatue == AUTO_WAITE && Robot_Auto_Reset == TRUE)
						{//待机中，且可以启动
							ExtendGetData_u16(sendDataBuf, EXT_OLINE_YES);
						}
						else
						{//待机中，但不可以启动
							ExtendGetData_u16(sendDataBuf, EXT_OLINE_NO);
						}
						break;
					case EXT_ADDR_ALARM_INFO://读取当前报警信息
						for(i=0; i<10; i++)
						{//报警信号查询
							if(Robot_Error_Data[i] == 0)
							{//无报警
								alarmNum = 0;
							}
							else
							{//有报警
								for(m=0; m<8; m++)
								{
									if(Robot_Error_Data[i] & (0x01 << m))
									{//按照高位-低位顺序查询当前报警
										alarmNum = i * 8 + m + 8;//+8为了与上位机报警号统一
										break;
									}
								}
								break;
							}
						}
						sendDataBuf[0] = alarmNum>>8;
						sendDataBuf[1] = alarmNum;
						sendDataLen = 2;
						break;
					case EXT_ADDR_INPUT_STA://读取当前输入状态
						inportNum = receiveAddr_Temp - EXT_ADDR_INPUT_STA;
						if(inportNum >= INPUT_NUM)
						{//输入端口号超本地输入端口数
							dataLen = 0;
						}
						else 
						{
							if(inportNum + dataLen >= INPUT_NUM)
							{//从起始地址开始，读取的输入端口个数超本地端口数
								dataLen = INPUT_NUM - inportNum;
							}
							for(i=0; i<dataLen; i++)
							{
								if(ReadInput(inportNum + i) == 0)
								{//有效信号
									sendDataBuf[dataLen*2 -1 - i*2] = 1;
								}
								else
								{//无效信号
									sendDataBuf[dataLen*2 -1 - i*2] = 0;
								}
							}
						}
						sendDataLen = dataLen * 2;	
						break;
					case EXT_ADDR_CUR_RUN_PRO://读取当前运行程序
						ExtendGetData_u32(sendDataBuf, Free_Program_Operate.Name);
						ExtendGetData_u32(sendDataBuf, Free_Program_Operate.Name2);
						ExtendGetData_u32(sendDataBuf, Free_Program_Operate.Name3);
						break;
					case EXT_ADDR_CUR_YIELD://读取当前产量
						ExtendGetData_u32(sendDataBuf, SC_Parameter.SC_Num);
						break;
					case EXT_ADDR_TAR_YIELD://读取目标产量
						ExtendGetData_u32(sendDataBuf, SC_Parameter.RW_Num);
						break;
					case EXT_ADDR_TOTAL_YIELD://读取总产量
						ExtendGetData_u32(sendDataBuf, SC_Parameter.LJ_Num);
						break;
					case EXT_ADDR_NG_YIELD://读取次品产量
						ExtendGetData_u32(sendDataBuf, SC_Parameter.NG_Num);
						break;
					case EXT_ADDR_OUT_CONTROL://读取当前输出状态
						break;
					case EXT_ADDR_CUR_POSITION_X://读取当前设备位置
						sendDataBuf[1] = (u8)((m_PulseTotalCounter[X_Axsis] - MINROBOTPOSITION));
						sendDataBuf[0] = (u8)((m_PulseTotalCounter[X_Axsis] - MINROBOTPOSITION) >> 8);
						sendDataBuf[3] = (u8)((m_PulseTotalCounter[X_Axsis] - MINROBOTPOSITION) >> 16);
						sendDataBuf[2] = (u8)((m_PulseTotalCounter[X_Axsis] - MINROBOTPOSITION) >> 24);
						sendDataLen = 4;
						if(dataLen == Axis_Num * 2)
						{
							sendDataBuf[5] = (u8)((m_PulseTotalCounter[L_Axsis] - MINROBOTPOSITION));
							sendDataBuf[4] = (u8)((m_PulseTotalCounter[L_Axsis] - MINROBOTPOSITION) >> 8);
							sendDataBuf[7] = (u8)((m_PulseTotalCounter[L_Axsis] - MINROBOTPOSITION) >> 16);
							sendDataBuf[6] = (u8)((m_PulseTotalCounter[L_Axsis] - MINROBOTPOSITION) >> 24);
							sendDataBuf[9] = (u8)((m_PulseTotalCounter[Z_Axsis] - MINROBOTPOSITION));
							sendDataBuf[8] = (u8)((m_PulseTotalCounter[Z_Axsis] - MINROBOTPOSITION) >> 8);
							sendDataBuf[11] = (u8)((m_PulseTotalCounter[Z_Axsis] - MINROBOTPOSITION) >> 16);
							sendDataBuf[10] = (u8)((m_PulseTotalCounter[Z_Axsis] - MINROBOTPOSITION) >> 24);
							sendDataBuf[13] = (u8)((m_PulseTotalCounter[O_Axsis] - MINROBOTPOSITION));
							sendDataBuf[12] = (u8)((m_PulseTotalCounter[O_Axsis] - MINROBOTPOSITION) >> 8);
							sendDataBuf[15] = (u8)((m_PulseTotalCounter[O_Axsis] - MINROBOTPOSITION) >> 16);
							sendDataBuf[14] = (u8)((m_PulseTotalCounter[O_Axsis] - MINROBOTPOSITION) >> 24);
							sendDataLen = Axis_Num * 4;
						}
						break;
					case EXT_ADDR_CUR_POSITION_Y:
						sendDataBuf[1] = (u8)((m_PulseTotalCounter[L_Axsis] - MINROBOTPOSITION));
						sendDataBuf[0] = (u8)((m_PulseTotalCounter[L_Axsis] - MINROBOTPOSITION) >> 8);
						sendDataBuf[3] = (u8)((m_PulseTotalCounter[L_Axsis] - MINROBOTPOSITION) >> 16);
						sendDataBuf[2] = (u8)((m_PulseTotalCounter[L_Axsis] - MINROBOTPOSITION) >> 24);
						sendDataLen = 4;
						break;
					case EXT_ADDR_CUR_POSITION_Z:
						sendDataBuf[1] = (u8)((m_PulseTotalCounter[Z_Axsis] - MINROBOTPOSITION));
						sendDataBuf[0] = (u8)((m_PulseTotalCounter[Z_Axsis] - MINROBOTPOSITION) >> 8);
						sendDataBuf[3] = (u8)((m_PulseTotalCounter[Z_Axsis] - MINROBOTPOSITION) >> 16);
						sendDataBuf[2] = (u8)((m_PulseTotalCounter[Z_Axsis] - MINROBOTPOSITION) >> 24);
						sendDataLen = 4;
						break;
					case EXT_ADDR_CUR_POSITION_O:
						sendDataBuf[1] = (u8)((m_PulseTotalCounter[O_Axsis] - MINROBOTPOSITION));
						sendDataBuf[0] = (u8)((m_PulseTotalCounter[O_Axsis] - MINROBOTPOSITION) >> 8);
						sendDataBuf[3] = (u8)((m_PulseTotalCounter[O_Axsis] - MINROBOTPOSITION) >> 16);
						sendDataBuf[2] = (u8)((m_PulseTotalCounter[O_Axsis] - MINROBOTPOSITION) >> 24);
						sendDataLen = 4;
						break;
					
					case EXT_ADDR_POINT_1_X://读取点库中的点坐标值
						pointNum = (receiveAddr_Temp - EXT_ADDR_POINT_1_X) / 0x0040;		//求出操作的是第几个点
						if(pointNum >= ManulSavePointMaxNum || (dataLen != 2 && dataLen != 8) || receiveAddr_Temp % 2 == 1)
						{//点编号超ManulSavePointMaxNum个、长度不为2或8，地址不是2的整数倍，说明发送的地址不对
							sendDataLen = 0;
						}
						else if(dataLen == Axis_Num * 2)
						{
							sendDataBuf[1] = (u8)(Manul_Save_Point[pointNum].Point_X);
							sendDataBuf[0] = (u8)(Manul_Save_Point[pointNum].Point_X >> 8);
							sendDataBuf[3] = (u8)(Manul_Save_Point[pointNum].Point_X >> 16);
							sendDataBuf[2] = (u8)(Manul_Save_Point[pointNum].Point_X >> 24);
							sendDataBuf[5] = (u8)(Manul_Save_Point[pointNum].Point_L);
							sendDataBuf[4] = (u8)(Manul_Save_Point[pointNum].Point_L >> 8);
							sendDataBuf[7] = (u8)(Manul_Save_Point[pointNum].Point_L >> 16);
							sendDataBuf[6] = (u8)(Manul_Save_Point[pointNum].Point_L >> 24);
							sendDataBuf[9] = (u8)(Manul_Save_Point[pointNum].Point_Z);
							sendDataBuf[8] = (u8)(Manul_Save_Point[pointNum].Point_Z >> 8);
							sendDataBuf[11] = (u8)(Manul_Save_Point[pointNum].Point_Z >> 16);
							sendDataBuf[10] = (u8)(Manul_Save_Point[pointNum].Point_Z >> 24);
							sendDataBuf[13] = (u8)(Manul_Save_Point[pointNum].Point_O);
							sendDataBuf[12] = (u8)(Manul_Save_Point[pointNum].Point_O >> 8);
							sendDataBuf[15] = (u8)(Manul_Save_Point[pointNum].Point_O >> 16);
							sendDataBuf[14] = (u8)(Manul_Save_Point[pointNum].Point_O >> 24);
							sendDataLen = Axis_Num * 4;
						}
						else if((receiveAddr_Temp - EXT_ADDR_POINT_1_X - pointNum * 0x0040) / 2 == X_Axsis)
						{
							sendDataBuf[1] = (u8)(Manul_Save_Point[pointNum].Point_X);
							sendDataBuf[0] = (u8)(Manul_Save_Point[pointNum].Point_X >> 8);
							sendDataBuf[3] = (u8)(Manul_Save_Point[pointNum].Point_X >> 16);
							sendDataBuf[2] = (u8)(Manul_Save_Point[pointNum].Point_X >> 24);
							sendDataLen = 4;
						}
						else if((receiveAddr_Temp - EXT_ADDR_POINT_1_X - pointNum * 0x0040) / 2 == L_Axsis)
						{
							sendDataBuf[1] = (u8)(Manul_Save_Point[pointNum].Point_L);
							sendDataBuf[0] = (u8)(Manul_Save_Point[pointNum].Point_L >> 8);
							sendDataBuf[3] = (u8)(Manul_Save_Point[pointNum].Point_L >> 16);
							sendDataBuf[2] = (u8)(Manul_Save_Point[pointNum].Point_L >> 24);
							sendDataLen = 4;
						}
						else if((receiveAddr_Temp - EXT_ADDR_POINT_1_X - pointNum * 0x0040) / 2 == Z_Axsis)
						{
							sendDataBuf[1] = (u8)(Manul_Save_Point[pointNum].Point_Z);
							sendDataBuf[0] = (u8)(Manul_Save_Point[pointNum].Point_Z >> 8);
							sendDataBuf[3] = (u8)(Manul_Save_Point[pointNum].Point_Z >> 16);
							sendDataBuf[2] = (u8)(Manul_Save_Point[pointNum].Point_Z >> 24);
							sendDataLen = 4;
						}
						else if((receiveAddr_Temp - EXT_ADDR_POINT_1_X - pointNum * 0x0040) / 2 == O_Axsis)
						{
							sendDataBuf[1] = (u8)(Manul_Save_Point[pointNum].Point_O);
							sendDataBuf[0] = (u8)(Manul_Save_Point[pointNum].Point_O >> 8);
							sendDataBuf[3] = (u8)(Manul_Save_Point[pointNum].Point_O >> 16);
							sendDataBuf[2] = (u8)(Manul_Save_Point[pointNum].Point_O >> 24);
							sendDataLen = 4;
						}
						break;
					
					default://未找到相关地址命令，应答0个字节数据
						sendDataLen = 0;
						break;
				}
				
				USART2_ModbusSendData(ExtendEquipID, EXTEND_FUN_READ, sendDataBuf, sendDataLen);
			}
			else if(funCode == EXTEND_FUN_WRITE)
			{//写入命令处理部分
				receiveAddr_Temp = receiveAddr;
				if(EXT_ADDR_POINT_1_X <= receiveAddr && receiveAddr <= EXT_ADDR_POINT_1_X + 0x0040 * 32)
				{
					receiveAddr = EXT_ADDR_POINT_1_X;
				}
				else if(EXT_ADDR_OUT_CONTROL <= receiveAddr && receiveAddr <= EXT_ADDR_OUT_CONTROL + 0x00FF)
				{
					receiveAddr = EXT_ADDR_OUT_CONTROL;
				}
				else if(EXT_ADDR_INPUT_STA <= receiveAddr && receiveAddr <= EXT_ADDR_INPUT_STA + 0x00FF)
				{
					receiveAddr = EXT_ADDR_INPUT_STA;
				}
				switch(receiveAddr)
				{
//					case EXT_ADDR_RUN_STOP://控制设备启停
//						
//						break;
//					case EXT_ADDR_OUT_CONTROL://控制输出
//						
//						break;
					case EXT_ADDR_CUR_YIELD://设置当前产量
						SC_Parameter.SC_Num = (u32)(((u32)ExtendRecDataBuf[10]) | ((u32)ExtendRecDataBuf[9]<<8) | ((u32)ExtendRecDataBuf[8]<<16) | ((u32)ExtendRecDataBuf[7]<<24));
//						writeDataBuf[0] = SC_Parameter.SC_Num;
//						writeDataBuf[1] = SC_Parameter.SC_Num>>8;
//						writeDataBuf[2] = SC_Parameter.SC_Num>>16;
//						writeDataBuf[3] = SC_Parameter.SC_Num>>24; 
//						W25QXX_Write(writeDataBuf, P_SC_NUM_ADDRESS + 0x0C, 4);
						ExtendYieldChange = 1;
						break;
					case EXT_ADDR_TAR_YIELD://设置目标产量
						SC_Parameter.RW_Num = (u32)(((u32)ExtendRecDataBuf[10]) | ((u32)ExtendRecDataBuf[9]<<8) | ((u32)ExtendRecDataBuf[8]<<16) | ((u32)ExtendRecDataBuf[7]<<24));
//						writeDataBuf[0] = SC_Parameter.RW_Num;
//						writeDataBuf[1] = SC_Parameter.RW_Num>>8;
//						writeDataBuf[2] = SC_Parameter.RW_Num>>16;
//						writeDataBuf[3] = SC_Parameter.RW_Num>>24; 
//						W25QXX_Write(writeDataBuf, P_SC_NUM_ADDRESS + 0x00, 4);
						ExtendYieldChange = 1;
						break;
					case EXT_ADDR_NG_YIELD://设置NG(不良品)产量
						SC_Parameter.NG_Num = (u32)(((u32)ExtendRecDataBuf[10]) | ((u32)ExtendRecDataBuf[9]<<8) | ((u32)ExtendRecDataBuf[8]<<16) | ((u32)ExtendRecDataBuf[7]<<24));
//						writeDataBuf[0] = SC_Parameter.NG_Num;
//						writeDataBuf[1] = SC_Parameter.NG_Num>>8;
//						writeDataBuf[2] = SC_Parameter.NG_Num>>16;
//						writeDataBuf[3] = SC_Parameter.NG_Num>>24; 
//						W25QXX_Write(writeDataBuf, P_SC_NUM_ADDRESS + 0x14, 4);
						ExtendYieldChange = 1;
						break;
					case EXT_ADDR_CHK_YIELD://设置抽检产量
						SC_Parameter.CJ_Num = (u32)(((u32)ExtendRecDataBuf[10]) | ((u32)ExtendRecDataBuf[9]<<8) | ((u32)ExtendRecDataBuf[8]<<16) | ((u32)ExtendRecDataBuf[7]<<24));
//						writeDataBuf[0] = SC_Parameter.CJ_Num;
//						writeDataBuf[1] = SC_Parameter.CJ_Num>>8;
//						writeDataBuf[2] = SC_Parameter.CJ_Num>>16;
//						writeDataBuf[3] = SC_Parameter.CJ_Num>>24; 
//						W25QXX_Write(writeDataBuf, P_SC_NUM_ADDRESS + 0x04, 4);
						ExtendYieldChange = 1;
						break;
					case EXT_ADDR_TOTAL_YIELD://设置总产量
						SC_Parameter.LJ_Num = (u32)(((u32)ExtendRecDataBuf[10]) | ((u32)ExtendRecDataBuf[9]<<8) | ((u32)ExtendRecDataBuf[8]<<16) | ((u32)ExtendRecDataBuf[7]<<24));
//						writeDataBuf[0] = SC_Parameter.LJ_Num;
//						writeDataBuf[1] = SC_Parameter.LJ_Num>>8;
//						writeDataBuf[2] = SC_Parameter.LJ_Num>>16;
//						writeDataBuf[3] = SC_Parameter.LJ_Num>>24;
//						W25QXX_Write(writeDataBuf, P_SC_NUM_ADDRESS + 0x10, 4);
						ExtendYieldChange = 1;
						break;
					case EXT_ADDR_CUR_RUN_PRO://程序选择
						ExtendProgramNum = (u16)(((u16)ExtendRecDataBuf[8]) | ((u16)ExtendRecDataBuf[7]<<8));
						if(ExtendProgramNum >= SAVEPROGRAMNUM_MAIN)
						{//程序只能选择1~14，否则表示不选择
							ExtendProgramNum = 0;
						}
						break;
					case EXT_ADDR_RESET://控制设备复位
						ExtendStateChange = EXTEND_RESET;
						break;
					case EXT_ADDR_BACK_ORI://控制设备回零
						ExtendStateChange = EXTEND_ORIGIN;
						break;
					case EXT_ADDR_RUN_AUTO://启动设备
						if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
						{
							ExtendStateChange = EXTEND_RUN;
						}
						break;
					case EXT_ADDR_STOP_RUN://停止设备
						ExtendStateChange = EXTEND_STOP;
						break;
					case EXT_ADDR_PAUSE_RUN://暂停设备
						ExtendStateChange = EXTEND_PAUSE;
						break;
					case EXT_ADDR_EM_STOP://急停设备
						ExtendEmergencyStop = 1;
						Robot_Error_Data[0] = Robot_Error_Data[0] | 0x80;
						CloseTotalMotorError();
						break;
					case EXT_ADDR_CANCEL_ALARM://取消设备报警
						ExtendEmergencyStop = 0;
						ExtendCancleAlarm = 1;
						break;
					case EXT_ADDR_POINT_1_X://写入点库中的坐标值
						pointNum = (receiveAddr_Temp - EXT_ADDR_POINT_1_X) / 0x0040;										//求出操作的是第几个点
						ExtendPosChangeNum = pointNum + 1;																	//通过通信修改坐标的编号，范围只能是1~40
						if(pointNum >= ManulSavePointMaxNum || (dataLen != 2 && dataLen != (Axis_Num * 2)) || (receiveAddr_Temp % 2) == 1)
						{//点编号超ManulSavePointMaxNum个、长度不为2或8，地址不是2的整数倍，说明发送的地址不对
							dataLen = 0;
							ExtendPosChangeNum = 0;
						}
						else if(dataLen == Axis_Num * 2)
						{
							Manul_Save_Point[pointNum].Point_X = (u32)(((u32)ExtendRecDataBuf[8])|((u32)ExtendRecDataBuf[7]<<8)|((u32)ExtendRecDataBuf[10]<<16)|((u32)ExtendRecDataBuf[9]<<24));
							Manul_Save_Point[pointNum].Point_L = (u32)(((u32)ExtendRecDataBuf[12])|((u32)ExtendRecDataBuf[11]<<8)|((u32)ExtendRecDataBuf[14]<<16)|((u32)ExtendRecDataBuf[13]<<24));
							Manul_Save_Point[pointNum].Point_Z = (u32)(((u32)ExtendRecDataBuf[16])|((u32)ExtendRecDataBuf[15]<<8)|((u32)ExtendRecDataBuf[18]<<16)|((u32)ExtendRecDataBuf[17]<<24));
							Manul_Save_Point[pointNum].Point_O = (u32)(((u32)ExtendRecDataBuf[20])|((u32)ExtendRecDataBuf[19]<<8)|((u32)ExtendRecDataBuf[22]<<16)|((u32)ExtendRecDataBuf[21]<<24));
//							writeDataBuf[0] = Manul_Save_Point[pointNum].Point_X;
//							writeDataBuf[1] = Manul_Save_Point[pointNum].Point_X>>8;
//							writeDataBuf[2] = Manul_Save_Point[pointNum].Point_X>>16;
//							writeDataBuf[3] = Manul_Save_Point[pointNum].Point_X>>24;
//							writeDataBuf[4] = Manul_Save_Point[pointNum].Point_L;
//							writeDataBuf[5] = Manul_Save_Point[pointNum].Point_L>>8;
//							writeDataBuf[6] = Manul_Save_Point[pointNum].Point_L>>16;
//							writeDataBuf[7] = Manul_Save_Point[pointNum].Point_L>>24;
//							writeDataBuf[8] = Manul_Save_Point[pointNum].Point_Z;
//							writeDataBuf[9] = Manul_Save_Point[pointNum].Point_Z>>8;
//							writeDataBuf[10] = Manul_Save_Point[pointNum].Point_Z>>16;
//							writeDataBuf[11] = Manul_Save_Point[pointNum].Point_Z>>24;
//							writeDataBuf[12] = Manul_Save_Point[pointNum].Point_O;
//							writeDataBuf[13] = Manul_Save_Point[pointNum].Point_O>>8;
//							writeDataBuf[14] = Manul_Save_Point[pointNum].Point_O>>16;
//							writeDataBuf[15] = Manul_Save_Point[pointNum].Point_O>>24;
//							W25QXX_Write(writeDataBuf, P_POINT_SAVE_HEAD + pointNum * P_POINT_SAVE_LEN + 0x0E, 16);
						}
						else if((receiveAddr_Temp - EXT_ADDR_POINT_1_X - pointNum * 0x0040) / 2 == X_Axsis)
						{
							Manul_Save_Point[pointNum].Point_X = (u32)(((u32)ExtendRecDataBuf[8])|((u32)ExtendRecDataBuf[7]<<8)|((u32)ExtendRecDataBuf[10]<<16)|((u32)ExtendRecDataBuf[9]<<24));														
//							writeDataBuf[0] = Manul_Save_Point[pointNum].Point_X;
//							writeDataBuf[1] = Manul_Save_Point[pointNum].Point_X>>8;
//							writeDataBuf[2] = Manul_Save_Point[pointNum].Point_X>>16;
//							writeDataBuf[3] = Manul_Save_Point[pointNum].Point_X>>24;
//							W25QXX_Write(writeDataBuf, P_POINT_SAVE_HEAD + pointNum*P_POINT_SAVE_LEN+0x0E, 4);
						}
						else if((receiveAddr_Temp - EXT_ADDR_POINT_1_X - pointNum * 0x0040) / 2 == L_Axsis)
						{
							Manul_Save_Point[pointNum].Point_L = (u32)(((u32)ExtendRecDataBuf[8])|((u32)ExtendRecDataBuf[7]<<8)|((u32)ExtendRecDataBuf[10]<<16)|((u32)ExtendRecDataBuf[9]<<24));
//							writeDataBuf[0] = Manul_Save_Point[pointNum].Point_L;
//							writeDataBuf[1] = Manul_Save_Point[pointNum].Point_L>>8;
//							writeDataBuf[2] = Manul_Save_Point[pointNum].Point_L>>16;
//							writeDataBuf[3] = Manul_Save_Point[pointNum].Point_L>>24;
//							W25QXX_Write(writeDataBuf, P_POINT_SAVE_HEAD + pointNum*P_POINT_SAVE_LEN+0x12, 4);
						}
						else if((receiveAddr_Temp - EXT_ADDR_POINT_1_X - pointNum * 0x0040) / 2 == Z_Axsis)
						{
							Manul_Save_Point[pointNum].Point_Z = (u32)(((u32)ExtendRecDataBuf[8])|((u32)ExtendRecDataBuf[7]<<8)|((u32)ExtendRecDataBuf[10]<<16)|((u32)ExtendRecDataBuf[9]<<24));
//							writeDataBuf[0] = Manul_Save_Point[pointNum].Point_Z;
//							writeDataBuf[1] = Manul_Save_Point[pointNum].Point_Z>>8;
//							writeDataBuf[2] = Manul_Save_Point[pointNum].Point_Z>>16;
//							writeDataBuf[3] = Manul_Save_Point[pointNum].Point_Z>>24;
//							W25QXX_Write(writeDataBuf, P_POINT_SAVE_HEAD + pointNum*P_POINT_SAVE_LEN+0x16, 4);
						}
						else if((receiveAddr_Temp - EXT_ADDR_POINT_1_X - pointNum * 0x0040) / 2 == O_Axsis)
						{
							Manul_Save_Point[pointNum].Point_O = (u32)(((u32)ExtendRecDataBuf[8])|((u32)ExtendRecDataBuf[7]<<8)|((u32)ExtendRecDataBuf[10]<<16)|((u32)ExtendRecDataBuf[9]<<24));
//							writeDataBuf[0] = Manul_Save_Point[pointNum].Point_O;
//							writeDataBuf[1] = Manul_Save_Point[pointNum].Point_O>>8;
//							writeDataBuf[2] = Manul_Save_Point[pointNum].Point_O>>16;
//							writeDataBuf[3] = Manul_Save_Point[pointNum].Point_O>>24;
//							W25QXX_Write(writeDataBuf, P_POINT_SAVE_HEAD + pointNum*P_POINT_SAVE_LEN+0x1A, 4);
						}
						break;
					case EXT_ADDR_OUT_CONTROL://设置设备输出口状态
						outportNum = receiveAddr_Temp - EXT_ADDR_OUT_CONTROL;
						if(outportNum >= OUTPUT_NUM)
						{//输出端口号超本地输出端口数
							dataLen = 0;
						}
						else
						{
							if(outportNum + dataLen >= OUTPUT_NUM)
							{//从起始地址开始，设置的输出端口个数超过本地输出端口个数
								dataLen = OUTPUT_NUM - outportNum;
							}
							
							for(i=0; i<dataLen; i++)
							{
								if(ExtendRecDataBuf[6 + dataLen*2 - i*2] == 1)
								{//输出有效信号
									ResetOutput(outportNum + i);
								}
								else
								{//输出无效信号
									SetOutput(outportNum + i);
								}
							}
						}
						break;
					
					case EXT_ADDR_WRITE_SERIAL_NUM://写序列号
						ExtendSerialNum = 1;
						for(i=0;i<12;i++)
						{
							Internet_Parameter.Sequence[i] = ExtendRecDataBuf[i + 7];
						}
						W25QXX_Write(&Internet_Parameter.Sequence[0],P_INTERNET_ADDRESS + 1,12);
						break;
						
					default://未找到相关地址命令，应答0个字节数据
						dataLen = 0;
						break;
				}
				USART2_ModbusSendAnswer(ExtendEquipID, EXTEND_FUN_WRITE, receiveAddr, dataLen);
			}
		}
		ExtendRecDataLen = 0;
		ExtendRecDataBuf[0] = 0;
		ExtendRecDataFlag = EXTEND_WAIT_REC;
	}
}



/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
