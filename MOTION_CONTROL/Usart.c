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

u8  StartReceiveDataFlag = FALSE;							//��ʼ�������ݱ�־
u8  USART1ErrorFlag = FALSE;								//ͨ�ų����־λ
u8	NewOrder = FALSE;										//���յ��µ�����

u8  SendDataBuffer[USART_BUFFER_SIZE] = {0};            			//�������ݻ���
u8  ReceiveDataBuffer[USART_BUFFER_SIZE] = {0};         			//�������ݻ���
u8  UsartReceiveData[USART_BUFFER_SIZE] = {0};		       			//���ڽ��յ�������
u8  ReceiveDataCounter = 0;		           			//�������ݼ���
u8  ReceiveDataLen = 0;				           			//��¼�������ݳ���
u8  temp[7] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07};//Ӧ����������

/**************************************************************************************************
**  ��������  UsartInit()
**	���������
**	�����������
**	�������ܣ����ڳ�ʼ������
**	��ע��	 ��
**  ���ߣ�      
**  �������ڣ��Ѹ�
***************************************************************************************************/	
void UsartInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
#if USART1_485_EN == 1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 			//ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);			//ʹ��USART1ʱ��

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);		//GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);	//GPIOA10����ΪUSART1

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;							//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 						//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 							//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);										//��ʼ������
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);							//ʹ�ܽ����ж�
	USART_Cmd(USART1, ENABLE);																	//ʹ�ܴ���
	USART_ClearFlag(USART1, USART_FLAG_TC);    									//�巢����ɱ�־

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;			//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;						//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);
#endif

#if USART2_485_EN == 1

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); 			//ʹ��GPIOCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);			//ʹ��USART2ʱ��
	
	//����2���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2); //GPIOG5����ΪUSART2
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2); //GPIOG6����ΪUSART2
	
	//USART2    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; 		//GPIOD5��GPIOD6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;							//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;				//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 						//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;							//����
	GPIO_Init(GPIOD,&GPIO_InitStructure);
		
	//USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate = 9600;										//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;													//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;															//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;																	//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;			//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;											//�շ�ģʽ
	
	USART_Init(USART2, &USART_InitStructure);										//��ʼ������
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);							//ʹ�ܽ����ж�
	USART_Cmd(USART2, ENABLE);																	//ʹ�ܴ���
	USART_ClearFlag(USART2, USART_FLAG_TC);    									//�巢����ɱ�־
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
#endif
}

/**************************************************************************************************
**  ��������  USART1_SendData()
**	�����������
**	�����������
**	�������ܣ���Э�鷢������
**	��ע��	  �������ݺ�Ż��˳�
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void USART1_SendData(u8 DataLen, u8 Order, u8 *Data)
{	
	u8 i = 3;
	u8 j = 0;
	u16 k = 0;
	
	SendDataBuffer[0] = USART_SEND_START;  		//�������ݿ�ʼ��־	
	SendDataBuffer[1] = DataLen + 3;		     	//���ݳ���   
	SendDataBuffer[2] = Order; 		         		//��������
	if(DataLen>0)
	{
		for(i=3; i<DataLen+3; i++)
		{
			SendDataBuffer[i] = Data[i-3];     //��������
		}
	}
	SendDataBuffer[i] = USART_SEND_END;	   //�������ݽ�����־
	i++;
	USART_ClearFlag(USART1,USART_FLAG_TC);
	DIR485_H;                            //д����
	for(j=0; j<i; j++)
	{
		for(k=0; k<100; k++);
		
		USART1->DR = SendDataBuffer[j];
		while((USART1->SR&0X40) == 0);	
	}
	DIR485_L;                            //������
}


/**************************************************************************************************
**  ��������  UsartAcknowledge()
**	�����������
**	�����������
**	�������ܣ����ڵ�Ӧ����
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
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
**  ��������  UsartDataDecode()
**	�����������
**	�����������
**	�������ܣ���ȡ���ڽ��յ������ݣ������ڽ�����ջ���
**	��ע��	  
**  ���ߣ�       
**  �������ڣ�
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
