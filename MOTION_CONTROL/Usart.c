/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Usart.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "Usart.h" 
#include "Error.h"
#include "SignalWatch.h"
#include "StatusControl.h"
#include "Delay.h"

u8  StartReceiveDataFlag = FALSE;							//开始接受数据标志
u8  USART1ErrorFlag = FALSE;								//通信出错标志位
u8	NewOrder = FALSE;										//接收到新的数据

u8  SendDataBuffer[USART_BUFFER_SIZE] = {0};            			//发送数据缓存
u8  ReceiveDataBuffer[USART_BUFFER_SIZE] = {0};         			//接收数据缓存
u8  UsartReceiveData[USART_BUFFER_SIZE] = {0};		       			//串口接收到的数据
u8  ReceiveDataCounter = 0;		           			//接收数据计数
u8  ReceiveDataLen = 0;				           			//记录接收数据长度
u8  temp[7] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07};//应答命令数组

/**************************************************************************************************
**  函数名：  UsartInit()
**	输入参数：
**	输出参数：无
**	函数功能：串口初始化函数
**	备注：	 无
**  作者：      
**  开发日期：已改
***************************************************************************************************/	
void UsartInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
#if USART1_485_EN == 1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 			//使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);			//使能USART1时钟

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);		//GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);	//GPIOA10复用为USART1

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;							//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 						//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 							//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);										//初始化串口
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);							//使能接收中断
	USART_Cmd(USART1, ENABLE);																	//使能串口
	USART_ClearFlag(USART1, USART_FLAG_TC);    									//清发送完成标志

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;			//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;						//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);
#endif

#if USART2_485_EN == 1

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); 			//使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);			//使能USART2时钟
	
	//串口2引脚复用映射
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2); //GPIOG5复用为USART2
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2); //GPIOG6复用为USART2
	
	//USART2    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; 		//GPIOD5与GPIOD6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;							//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;				//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 						//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;							//上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure);
		
	//USART2 初始化设置
	USART_InitStructure.USART_BaudRate = 9600;										//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;													//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;															//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;																	//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;			//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;											//收发模式
	
	USART_Init(USART2, &USART_InitStructure);										//初始化串口
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);							//使能接收中断
	USART_Cmd(USART2, ENABLE);																	//使能串口
	USART_ClearFlag(USART2, USART_FLAG_TC);    									//清发送完成标志
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
#endif
}

/**************************************************************************************************
**  函数名：  USART1_SendData()
**	输入参数：无
**	输出参数：无
**	函数功能：按协议发送数据
**	备注：	  发完数据后才会退出
**  作者：    
**  开发日期：
***************************************************************************************************/
void USART1_SendData(u8 DataLen, u8 Order, u8 *Data)
{	
	u8 i = 3;
	u8 j = 0;
	u16 k = 0;
	
	SendDataBuffer[0] = USART_SEND_START;  		//发送数据开始标志	
	SendDataBuffer[1] = DataLen + 3;		     	//数据长度   
	SendDataBuffer[2] = Order; 		         		//发送命令
	if(DataLen>0)
	{
		for(i=3; i<DataLen+3; i++)
		{
			SendDataBuffer[i] = Data[i-3];     //发送数据
		}
	}
	SendDataBuffer[i] = USART_SEND_END;	   //发送数据结束标志
	i++;
	USART_ClearFlag(USART1,USART_FLAG_TC);
	DIR485_H;                            //写方向
	for(j=0; j<i; j++)
	{
		for(k=0; k<100; k++);
		
		USART1->DR = SendDataBuffer[j];
		while((USART1->SR&0X40) == 0);	
	}
	DIR485_L;                            //读方向
}


/**************************************************************************************************
**  函数名：  UsartAcknowledge()
**	输入参数：无
**	输出参数：无
**	函数功能：串口的应答处理
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void UsartAcknowledge(u8 Order)
{
	switch(Order)
	{
		case P_ROBOT_ENABLE_A_ORIGIN:	
			USART1_SendData(1,0xA0,&temp[0]);
			break; 
		case P_WORK_MODE:	
			USART1_SendData(1,0xB0,&temp[0]);
			break;            
		case P_AUTO_RUN:
			USART1_SendData(1,0xC0,&temp[0]);
			break;
		case P_FREE_PROGRAM_SEND:	
			USART1_SendData(1,0xD0,&temp[1]);
			break;
		case P_WATCH_COMMAND:		     
			break; 		   
		case P_READ_IIC:		     
			break; 
		case P_IO_DEBUG_OUTPUT1:	
			USART1_SendData(1,0xA1,&temp[2]);
			break;
		case P_IO_DEBUG_OUTPUT2:	
			USART1_SendData(1,0xB1,&temp[2]);
			break;
		case P_IO_DEBUG_OUTPUT3:	
			USART1_SendData(1,0xC1,&temp[2]);
			break;
		case P_MANUL_DEBUG:
			USART1_SendData(1,0xD1,&temp[5]);
			break;
		case P_PARAMETER_ORDER:
			USART1_SendData(1,0xE1,&temp[5]);
			break;
		case P_SYSTEM_SET_SEND:
			USART1_SendData(1,0xF1,&temp[6]);
			break;
		case MDPARA_COPY_SEND:
			USART1_SendData(1,0xA2,&temp[6]);
			break;
		default:
			break;
	}
}

/**************************************************************************************************
**  函数名：  UsartDataDecode()
**	输入参数：无
**	输出参数：无
**	函数功能：获取串口接收到的数据，并串口接收清空缓存
**	备注：	  
**  作者：       
**  开发日期：
***************************************************************************************************/
void UsartDataDecode()
{
  u8 i=0;
	for(i=0; i<ReceiveDataLen-1; i++)
	{
		UsartReceiveData[i] = ReceiveDataBuffer[i+1];
	}
	for(i=0; i<USART_BUFFER_SIZE; i++)
	{
	  ReceiveDataBuffer[i]=0;
	}
	ReceiveDataLen = 0;
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
