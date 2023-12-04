/*********************** (C) COPYRIGHT 2021 ZH manipulator Team ************************
* File Name          : ExtendCom.c
* Author             : 
* Version            : V1.0.0
* Date               : 
* Description        : ����ͨ��485���ʱ��豸��Э�鴦����
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

/*485ͨ�ŵ����ݻ���������*/
#define EXTEND_BUF_LEN			50						//485ͨ�ŵ����ݻ���������

u8  ExtendRecDataFlag = FALSE;							//��ʼ�������ݱ�־��0���������ݣ�1��ʼ�������ݣ�2�����������

u8  ExtendSendDataBuf[EXTEND_BUF_LEN] = {0};			//�������ݻ���
u8  ExtendRecDataBuf[EXTEND_BUF_LEN] = {0};   			//�������ݻ���
u8  ExtendRecDataLen = 0;				           		//��¼���յ������ݳ���

/*����Ӧ�ó����д������¼�������*/
u8  ExtendEquipID = 31;				           			//���ڴ�ű��豸ID�����ڿ������Ͻ��б������ã�Ĭ��ID = 31

u16 ExtendProgramNum = 0;								//ͨ��ͨ��ѡ�еĳ���ţ�����֪ͨ��ʾ��Ҫ�л����³�����,0��ʾ���򲻸ı䣬��Χ1~14

u8  ExtendEmergencyStop = 0;				          	//ͨ��ͨ�ż�ͣ�豸��־��ֱ���ڿ������Ͻ��д���

u8  ExtendYieldChange = 0;								//ͨ��ͨ���޸Ĳ�����ر�����־������֪ͨ��ʾ������ȡ�µĲ���

u8  ExtendPosChangeNum = 0;								//ͨ��ͨ���޸�����ı�ţ�����֪ͨ��ʾ��Ҫ��ȡ�ĵ���,0��ʾû�е�ı䣬��Χ1~40

u8  ExtendStateChange = 0;								//ͨ��ͨ���޸Ļ�е��״̬��ExtendStateChange = 1��λ��= 2���㡢=3������=4ֹͣ��=5��ͣ

u8  ExtendCancleAlarm = 0;								//ͨ��ͨ��ȡ����е�ֱ���

u8  ExtendSerialNum = 0;								  //ͨ��ͨ����¼���к�

/*MODBUS��׼��CRCУ��**/
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

/*д�����Ӧ���ͺ���*/
void USART2_ModbusSendAnswer(u8 ID, u8 funCode, u16 addr, u16 dataLen)
{
	u8 i = 0;
	u16 CRC_16 = 0;
	
	ExtendSendDataBuf[0] = ID;  						 	//��ַ
	ExtendSendDataBuf[1] = funCode;		     	 			//Modbusָ��
	ExtendSendDataBuf[2] = (u8)(addr >> 8); 		        //��ַ���ֽ�
	ExtendSendDataBuf[3] = (u8)(addr); 		         		//��ַ���ֽ�
	ExtendSendDataBuf[4] = (u8)(dataLen >> 8); 		        //���ȸ��ֽ�
	ExtendSendDataBuf[5] = (u8)(dataLen); 		         	//���ȵ��ֽ�
	
	CRC_16 = CRC_chk(ExtendSendDataBuf, 6);
	
	ExtendSendDataBuf[6] = (u8)CRC_16;						//CRCУ���ֵ�8λ
	ExtendSendDataBuf[7] = (u8)(CRC_16 >> 8);				//CRCУ���ָ�8λ
	
	USART_ClearFlag(USART2,USART_FLAG_TC);
	DIROUT485_H;												//д����
	for(i=0; i<8; i++)	//8
	{
		USART2->DR = ExtendSendDataBuf[i];
		while((USART2->SR & 0X40) == 0);	
	}
	DIROUT485_L;												//������
}

/*����������ݷ������ͺ���*/
void USART2_ModbusSendData(u8 ID, u8 funCode, u8 *data, u16 len)
{	
	u8 i = 0;
	u16 CRC_16 = 0;
	
	ExtendSendDataBuf[0] = ID;  						 		//��ַ
	ExtendSendDataBuf[1] = funCode;		     	 				//Modbusָ��
	ExtendSendDataBuf[2] = len; 		         				//�ֽڸ���
	for(i=0; i<len; i++)
	{
		ExtendSendDataBuf[3 + i] = data[i];
	}
	
	CRC_16 = CRC_chk(ExtendSendDataBuf, len + 3);
	
	ExtendSendDataBuf[len + 3] = (u8)CRC_16;					//CRCУ���ֵ�8λ
	ExtendSendDataBuf[len + 4] = (u8)(CRC_16 >> 8);				//CRCУ���ָ�8λ
		
	USART_ClearFlag(USART2,USART_FLAG_TC);
	DIROUT485_H;                            						//д����
	for(i=0; i<len+5; i++)	//5
	{
		USART2->DR = ExtendSendDataBuf[i];
		while((USART2->SR & 0X40) == 0);
	}
	DIROUT485_L;													//������
}

/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : 485��չģ��Ĵ����жϺ���
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
		{//���豸����������������IDΪ�������ǹ㲥
			ExtendRecDataBuf[0] = data;
			ExtendRecDataLen = 1;
			ExtendRecDataFlag = EXTEND_START_REC;
		}
		else if(ExtendRecDataFlag == EXTEND_START_REC && (ExtendRecDataBuf[0] == ExtendEquipID || ExtendRecDataBuf[0] == EXTEND_RADIO_ID))
		{//���豸�ѽ�������������IDΪ����
			ExtendRecDataBuf[ExtendRecDataLen] = data;
			ExtendRecDataLen++;
			if(ExtendRecDataLen > 2)
			{
				if(ExtendRecDataBuf[1] == EXTEND_FUN_READ)
				{//���ն���������
					if(ExtendRecDataLen == 8)	//8
					{//�������
						ExtendRecDataFlag = EXTEND_END_REC;
					}
					else if(ExtendRecDataLen > 8)	//8
					{//���ճ���������ȡ���־������ͷ
						ExtendRecDataLen = 0;
						ExtendRecDataFlag = EXTEND_WAIT_REC;
						ExtendRecDataBuf[0] = 0;
					}
				}
				else if(ExtendRecDataBuf[1] == EXTEND_FUN_WRITE)
				{//����д��������
					if(ExtendRecDataLen == 9 && ExtendRecDataBuf[6] > 64)		//9
					{//���յ������ݳ��ȳ�����������С������
						ExtendRecDataLen = 0;
						ExtendRecDataFlag = EXTEND_WAIT_REC;
						ExtendRecDataBuf[0] = 0;
					}
					else if(ExtendRecDataLen == 9 + ExtendRecDataBuf[6])		//9
					{//�������ȷ��
						ExtendRecDataFlag = EXTEND_END_REC;
					}
					else if(ExtendRecDataLen > 9 + ExtendRecDataBuf[6])			//9
					{//���ճ���������ȡ���־������ͷ
						ExtendRecDataLen = 0;
						ExtendRecDataFlag = EXTEND_WAIT_REC;
						ExtendRecDataBuf[0] = 0;
					}
				}
				else
				{//���ճ���������ȡ���־������ͷ
					ExtendRecDataLen = 0;
					ExtendRecDataFlag = EXTEND_WAIT_REC;
					ExtendRecDataBuf[0] = 0;
				}
			}
			else if(ExtendRecDataLen >= EXTEND_BUF_LEN)
			{//���ݳ��ȳ�����������С�������жϣ������־�ͳ���
				ExtendRecDataLen = 0;
				ExtendRecDataFlag = EXTEND_WAIT_REC;
				ExtendRecDataBuf[0] = 0;
			}
		}
	}
}

/*******************************************************************************
* Function Name  : ExtendGetSerialNum
* Description    : �õ����к�����
* Input          : sendBuff ���ͻ�����
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
* Description    : �õ�ϵͳ�汾������
* Input          : sendBuff ���ͻ�����
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
* Description    : �õ����ҵ�ַ����
* Input          : sendBuff ���ͻ�����
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
* Description    : �õ�u32λ����
* Input          : sendBuff ���ͻ�����
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
* Description    : �õ�s32λ����
* Input          : sendBuff ���ͻ�����
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
* Description    : �õ�u16λ����
* Input          : sendBuff ���ͻ�����
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
* Description    : �õ�s16λ����
* Input          : sendBuff ���ͻ�����
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
* Description    : ������չģ��485���յ������ݣ�����ѭ���е��øú���
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
//	u16 serialLen = 12;											//���кų���
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
		{//CRCУ������
			funCode = ExtendRecDataBuf[1];
			receiveAddr = ((u16)ExtendRecDataBuf[2] << 8) + ExtendRecDataBuf[3];
			dataLen = ((u16)ExtendRecDataBuf[4] << 8) + ExtendRecDataBuf[5];
			sendDataLen = dataLen * 2;
			if(funCode == EXTEND_FUN_READ)
			{//��ȡ�������
				/*ת����ַ���㴦��*/
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
					case EXT_ADDR_SERIAL_NUM://��ȡ��ǰ�豸���к�
						ExtendGetSerialNum(sendDataBuf);
						break;
					case EXT_ADDR_SYSTEM_VER://��ȡ��ǰ�豸ϵͳ�汾��
						ExtendGetSystemVer(sendDataBuf);
						break;
					case EXT_ADDR_PRODUC_FAC://��ȡ��ǰ�豸��������
						ExtendGetProductFac(sendDataBuf);
						break;
					case EXT_ADDR_ONCE_T://��ȡ��ǰ��Ʒ���μӹ�ʱ��
						ExtendGetData_u32(sendDataBuf, Program_RunTime);
						break;
					case EXT_ADDR_POWER_T://��ȡ����ʱ��
						ExtendGetData_u32(sendDataBuf, m_PowerOnTimeTotal);
						break;
					case EXT_ADDR_RUN_T://��ȡ����ʱ��
						ExtendGetData_u32(sendDataBuf, m_ProRunTimeTotal);
						break;
					case EXT_ADDR_TOTAL_POWER_T://��ȡ�ܿ���ʱ��
						ExtendGetData_u32(sendDataBuf, m_PowerOnTimeCumulate);
						break;
					case EXT_ADDR_TOTAL_RUN_T://��ȡ������ʱ��
						ExtendGetData_u32(sendDataBuf, m_ProRunTimeCumulate);
						break;
					case EXT_ADDR_RESET_STA://��ȡ��ǰ�豸�Ƿ�λ���
						sendDataBuf[0] = 0;
						if(Robot_Auto_Reset == TRUE)
						{
							sendDataBuf[1] = 1;
						}
						sendDataLen = 2;
						break;
					case EXT_ADDR_BACK_ORI_STA://��ȡ��ǰ�豸�Ƿ�������
						sendDataBuf[0] = 0;
						if(Origin_Backed == TRUE)
						{
							sendDataBuf[1] = 1;
						}
						sendDataLen = 2;
						break;
					case EXT_ADDR_RUN_STA://��ȡ��ǰ����״̬
						if(g_Auto_Reset_Flag == TRUE)
						{//��λ��
							ExtendGetData_u16(sendDataBuf, EXT_RESET);
						}
						else if(Back_Origin_Flag == TRUE)
						{//������
							ExtendGetData_u16(sendDataBuf, EXT_BACK);
						}
						else if(Error_Status != NO_ERROR)
						{//�б��������ر���״̬
							ExtendGetData_u16(sendDataBuf, EXT_ERR);
						}
						else if(g_AutoStatue == AUTO_RUNNING)
						{//�����У���������״̬
							ExtendGetData_u16(sendDataBuf, EXT_RUNNING);
						}
						else if(g_AutoStatue == AUTO_PAUSE)
						{//��ͣ�У�������ͣ״̬
							ExtendGetData_u16(sendDataBuf, EXT_PAUSE);
						}
						else if(g_AutoStatue == AUTO_WAITE && Robot_Auto_Reset == TRUE)
						{//�����У��ҿ�������
							ExtendGetData_u16(sendDataBuf, EXT_OLINE_YES);
						}
						else
						{//�����У�������������
							ExtendGetData_u16(sendDataBuf, EXT_OLINE_NO);
						}
						break;
					case EXT_ADDR_ALARM_INFO://��ȡ��ǰ������Ϣ
						for(i=0; i<10; i++)
						{//�����źŲ�ѯ
							if(Robot_Error_Data[i] == 0)
							{//�ޱ���
								alarmNum = 0;
							}
							else
							{//�б���
								for(m=0; m<8; m++)
								{
									if(Robot_Error_Data[i] & (0x01 << m))
									{//���ո�λ-��λ˳���ѯ��ǰ����
										alarmNum = i * 8 + m + 8;//+8Ϊ������λ��������ͳһ
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
					case EXT_ADDR_INPUT_STA://��ȡ��ǰ����״̬
						inportNum = receiveAddr_Temp - EXT_ADDR_INPUT_STA;
						if(inportNum >= INPUT_NUM)
						{//����˿ںų���������˿���
							dataLen = 0;
						}
						else 
						{
							if(inportNum + dataLen >= INPUT_NUM)
							{//����ʼ��ַ��ʼ����ȡ������˿ڸ��������ض˿���
								dataLen = INPUT_NUM - inportNum;
							}
							for(i=0; i<dataLen; i++)
							{
								if(ReadInput(inportNum + i) == 0)
								{//��Ч�ź�
									sendDataBuf[dataLen*2 -1 - i*2] = 1;
								}
								else
								{//��Ч�ź�
									sendDataBuf[dataLen*2 -1 - i*2] = 0;
								}
							}
						}
						sendDataLen = dataLen * 2;	
						break;
					case EXT_ADDR_CUR_RUN_PRO://��ȡ��ǰ���г���
						ExtendGetData_u32(sendDataBuf, Free_Program_Operate.Name);
						ExtendGetData_u32(sendDataBuf, Free_Program_Operate.Name2);
						ExtendGetData_u32(sendDataBuf, Free_Program_Operate.Name3);
						break;
					case EXT_ADDR_CUR_YIELD://��ȡ��ǰ����
						ExtendGetData_u32(sendDataBuf, SC_Parameter.SC_Num);
						break;
					case EXT_ADDR_TAR_YIELD://��ȡĿ�����
						ExtendGetData_u32(sendDataBuf, SC_Parameter.RW_Num);
						break;
					case EXT_ADDR_TOTAL_YIELD://��ȡ�ܲ���
						ExtendGetData_u32(sendDataBuf, SC_Parameter.LJ_Num);
						break;
					case EXT_ADDR_NG_YIELD://��ȡ��Ʒ����
						ExtendGetData_u32(sendDataBuf, SC_Parameter.NG_Num);
						break;
					case EXT_ADDR_OUT_CONTROL://��ȡ��ǰ���״̬
						break;
					case EXT_ADDR_CUR_POSITION_X://��ȡ��ǰ�豸λ��
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
					
					case EXT_ADDR_POINT_1_X://��ȡ����еĵ�����ֵ
						pointNum = (receiveAddr_Temp - EXT_ADDR_POINT_1_X) / 0x0040;		//����������ǵڼ�����
						if(pointNum >= ManulSavePointMaxNum || (dataLen != 2 && dataLen != 8) || receiveAddr_Temp % 2 == 1)
						{//���ų�ManulSavePointMaxNum�������Ȳ�Ϊ2��8����ַ����2����������˵�����͵ĵ�ַ����
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
					
					default://δ�ҵ���ص�ַ���Ӧ��0���ֽ�����
						sendDataLen = 0;
						break;
				}
				
				USART2_ModbusSendData(ExtendEquipID, EXTEND_FUN_READ, sendDataBuf, sendDataLen);
			}
			else if(funCode == EXTEND_FUN_WRITE)
			{//д���������
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
//					case EXT_ADDR_RUN_STOP://�����豸��ͣ
//						
//						break;
//					case EXT_ADDR_OUT_CONTROL://�������
//						
//						break;
					case EXT_ADDR_CUR_YIELD://���õ�ǰ����
						SC_Parameter.SC_Num = (u32)(((u32)ExtendRecDataBuf[10]) | ((u32)ExtendRecDataBuf[9]<<8) | ((u32)ExtendRecDataBuf[8]<<16) | ((u32)ExtendRecDataBuf[7]<<24));
//						writeDataBuf[0] = SC_Parameter.SC_Num;
//						writeDataBuf[1] = SC_Parameter.SC_Num>>8;
//						writeDataBuf[2] = SC_Parameter.SC_Num>>16;
//						writeDataBuf[3] = SC_Parameter.SC_Num>>24; 
//						W25QXX_Write(writeDataBuf, P_SC_NUM_ADDRESS + 0x0C, 4);
						ExtendYieldChange = 1;
						break;
					case EXT_ADDR_TAR_YIELD://����Ŀ�����
						SC_Parameter.RW_Num = (u32)(((u32)ExtendRecDataBuf[10]) | ((u32)ExtendRecDataBuf[9]<<8) | ((u32)ExtendRecDataBuf[8]<<16) | ((u32)ExtendRecDataBuf[7]<<24));
//						writeDataBuf[0] = SC_Parameter.RW_Num;
//						writeDataBuf[1] = SC_Parameter.RW_Num>>8;
//						writeDataBuf[2] = SC_Parameter.RW_Num>>16;
//						writeDataBuf[3] = SC_Parameter.RW_Num>>24; 
//						W25QXX_Write(writeDataBuf, P_SC_NUM_ADDRESS + 0x00, 4);
						ExtendYieldChange = 1;
						break;
					case EXT_ADDR_NG_YIELD://����NG(����Ʒ)����
						SC_Parameter.NG_Num = (u32)(((u32)ExtendRecDataBuf[10]) | ((u32)ExtendRecDataBuf[9]<<8) | ((u32)ExtendRecDataBuf[8]<<16) | ((u32)ExtendRecDataBuf[7]<<24));
//						writeDataBuf[0] = SC_Parameter.NG_Num;
//						writeDataBuf[1] = SC_Parameter.NG_Num>>8;
//						writeDataBuf[2] = SC_Parameter.NG_Num>>16;
//						writeDataBuf[3] = SC_Parameter.NG_Num>>24; 
//						W25QXX_Write(writeDataBuf, P_SC_NUM_ADDRESS + 0x14, 4);
						ExtendYieldChange = 1;
						break;
					case EXT_ADDR_CHK_YIELD://���ó�����
						SC_Parameter.CJ_Num = (u32)(((u32)ExtendRecDataBuf[10]) | ((u32)ExtendRecDataBuf[9]<<8) | ((u32)ExtendRecDataBuf[8]<<16) | ((u32)ExtendRecDataBuf[7]<<24));
//						writeDataBuf[0] = SC_Parameter.CJ_Num;
//						writeDataBuf[1] = SC_Parameter.CJ_Num>>8;
//						writeDataBuf[2] = SC_Parameter.CJ_Num>>16;
//						writeDataBuf[3] = SC_Parameter.CJ_Num>>24; 
//						W25QXX_Write(writeDataBuf, P_SC_NUM_ADDRESS + 0x04, 4);
						ExtendYieldChange = 1;
						break;
					case EXT_ADDR_TOTAL_YIELD://�����ܲ���
						SC_Parameter.LJ_Num = (u32)(((u32)ExtendRecDataBuf[10]) | ((u32)ExtendRecDataBuf[9]<<8) | ((u32)ExtendRecDataBuf[8]<<16) | ((u32)ExtendRecDataBuf[7]<<24));
//						writeDataBuf[0] = SC_Parameter.LJ_Num;
//						writeDataBuf[1] = SC_Parameter.LJ_Num>>8;
//						writeDataBuf[2] = SC_Parameter.LJ_Num>>16;
//						writeDataBuf[3] = SC_Parameter.LJ_Num>>24;
//						W25QXX_Write(writeDataBuf, P_SC_NUM_ADDRESS + 0x10, 4);
						ExtendYieldChange = 1;
						break;
					case EXT_ADDR_CUR_RUN_PRO://����ѡ��
						ExtendProgramNum = (u16)(((u16)ExtendRecDataBuf[8]) | ((u16)ExtendRecDataBuf[7]<<8));
						if(ExtendProgramNum >= SAVEPROGRAMNUM_MAIN)
						{//����ֻ��ѡ��1~14�������ʾ��ѡ��
							ExtendProgramNum = 0;
						}
						break;
					case EXT_ADDR_RESET://�����豸��λ
						ExtendStateChange = EXTEND_RESET;
						break;
					case EXT_ADDR_BACK_ORI://�����豸����
						ExtendStateChange = EXTEND_ORIGIN;
						break;
					case EXT_ADDR_RUN_AUTO://�����豸
						if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
						{
							ExtendStateChange = EXTEND_RUN;
						}
						break;
					case EXT_ADDR_STOP_RUN://ֹͣ�豸
						ExtendStateChange = EXTEND_STOP;
						break;
					case EXT_ADDR_PAUSE_RUN://��ͣ�豸
						ExtendStateChange = EXTEND_PAUSE;
						break;
					case EXT_ADDR_EM_STOP://��ͣ�豸
						ExtendEmergencyStop = 1;
						Robot_Error_Data[0] = Robot_Error_Data[0] | 0x80;
						CloseTotalMotorError();
						break;
					case EXT_ADDR_CANCEL_ALARM://ȡ���豸����
						ExtendEmergencyStop = 0;
						ExtendCancleAlarm = 1;
						break;
					case EXT_ADDR_POINT_1_X://д�����е�����ֵ
						pointNum = (receiveAddr_Temp - EXT_ADDR_POINT_1_X) / 0x0040;										//����������ǵڼ�����
						ExtendPosChangeNum = pointNum + 1;																	//ͨ��ͨ���޸�����ı�ţ���Χֻ����1~40
						if(pointNum >= ManulSavePointMaxNum || (dataLen != 2 && dataLen != (Axis_Num * 2)) || (receiveAddr_Temp % 2) == 1)
						{//���ų�ManulSavePointMaxNum�������Ȳ�Ϊ2��8����ַ����2����������˵�����͵ĵ�ַ����
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
					case EXT_ADDR_OUT_CONTROL://�����豸�����״̬
						outportNum = receiveAddr_Temp - EXT_ADDR_OUT_CONTROL;
						if(outportNum >= OUTPUT_NUM)
						{//����˿ںų���������˿���
							dataLen = 0;
						}
						else
						{
							if(outportNum + dataLen >= OUTPUT_NUM)
							{//����ʼ��ַ��ʼ�����õ�����˿ڸ���������������˿ڸ���
								dataLen = OUTPUT_NUM - outportNum;
							}
							
							for(i=0; i<dataLen; i++)
							{
								if(ExtendRecDataBuf[6 + dataLen*2 - i*2] == 1)
								{//�����Ч�ź�
									ResetOutput(outportNum + i);
								}
								else
								{//�����Ч�ź�
									SetOutput(outportNum + i);
								}
							}
						}
						break;
					
					case EXT_ADDR_WRITE_SERIAL_NUM://д���к�
						ExtendSerialNum = 1;
						for(i=0;i<12;i++)
						{
							Internet_Parameter.Sequence[i] = ExtendRecDataBuf[i + 7];
						}
						W25QXX_Write(&Internet_Parameter.Sequence[0],P_INTERNET_ADDRESS + 1,12);
						break;
						
					default://δ�ҵ���ص�ַ���Ӧ��0���ֽ�����
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
