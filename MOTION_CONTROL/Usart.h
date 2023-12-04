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

#define USART_SEND_START  0xf5        								 		//���ݷ��͵Ľ����ź�
#define USART_SEND_END    0xfa	       										//���ݷ��͵���ʼ�ź�

#define	USART1_485_EN	1
#define	USART2_485_EN	1

#define	UART_485_EN	0

#define USART_BUFFER_SIZE	120	//����ͨ�Ż�������С

#define DIR485_H  			GPIO_SetBits(GPIOC,GPIO_Pin_9)   	//ʾ����ͨ��
#define DIR485_L  			GPIO_ResetBits(GPIOC,GPIO_Pin_9)
#define DIROUT485_H  		GPIO_SetBits(GPIOD,GPIO_Pin_4)   	//�ⲿ485ͨ��
#define DIROUT485_L  		GPIO_ResetBits(GPIOD,GPIO_Pin_4)

extern u8  StartReceiveDataFlag;      	//��ʼ�������ݱ�־
extern u8  USART1ErrorFlag;	           	//ͨ�ų����־λ
extern u8  SendDataBuffer[USART_BUFFER_SIZE];         	//�������ݻ���
extern u8  ReceiveDataBuffer[USART_BUFFER_SIZE];      	//�������ݻ���
extern u8  UsartReceiveData[USART_BUFFER_SIZE];	   		//���ڽ�������
extern u8  ReceiveDataCounter;		   		//�������ݼ���
extern u8  ReceiveDataLen;			   			//��¼�������ݳ���
extern u8  NewOrder;		           			//���յ�������

extern void USART1_SendData(u8, u8, u8*);
extern void UsartAcknowledge(u8);
extern void UsartDataDecode(void);
void UsartInit(void);

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/
