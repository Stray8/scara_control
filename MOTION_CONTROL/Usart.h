/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __usart_h_
#define __usart_h_

#include "stm32f4xx.h"

#define USART_SEND_START  0xf5        								 		//数据发送的结束信号
#define USART_SEND_END    0xfa	       										//数据发送的起始信号

#define	USART1_485_EN	1
#define	USART2_485_EN	1

#define	UART_485_EN	0

#define USART_BUFFER_SIZE	120	//串口通信缓冲区大小

#define DIR485_H  			GPIO_SetBits(GPIOC,GPIO_Pin_9)   	//示教器通信
#define DIR485_L  			GPIO_ResetBits(GPIOC,GPIO_Pin_9)
#define DIROUT485_H  		GPIO_SetBits(GPIOD,GPIO_Pin_4)   	//外部485通信
#define DIROUT485_L  		GPIO_ResetBits(GPIOD,GPIO_Pin_4)

extern u8  StartReceiveDataFlag;      	//开始接受数据标志
extern u8  USART1ErrorFlag;	           	//通信出错标志位
extern u8  SendDataBuffer[USART_BUFFER_SIZE];         	//发送数据缓存
extern u8  ReceiveDataBuffer[USART_BUFFER_SIZE];      	//接收数据缓存
extern u8  UsartReceiveData[USART_BUFFER_SIZE];	   		//串口接收数据
extern u8  ReceiveDataCounter;		   		//接收数据计数
extern u8  ReceiveDataLen;			   			//记录接收数据长度
extern u8  NewOrder;		           			//接收到新数据

extern void USART1_SendData(u8, u8, u8*);
extern void UsartAcknowledge(u8);
extern void UsartDataDecode(void);
void UsartInit(void);

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/
