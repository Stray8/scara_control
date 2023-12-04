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

/*�ӽڵ��������NodeID����*/
#define E_ALLOW_UPDATA_SEND_EN			1												//�������ݸ��·���
#define E_ALLOW_UPDATA_SEND_DIS			0												//���������ݸ��·���

//typedef __packed struct
//{//�㴨���10�����󣬺̴����20������
//	s32 TargetPos;							//Ŀ��λ��
//	s32 ContourVel;							//�����ٶ�
//	s32 TarVelocity;						//Ŀ���ٶ�
//	s32 OriginOffset;						//ԭ��ƫ��
//	u32 OriginDecPoVel;					//�����������ٶȵ��ٶ�
//	u32 OriginVel;							//���������ٶ�
//	u16 ControlWord;						//������
//	u8 OriginMode;							//��ԭģʽ
//	u8 TargetMode;							//����ģʽ

//	//uint8 relese;							//16λ����
//}PDO_Output;									//λ��ģʽ�µ�TPDO���ݸ�ʽ

typedef __packed struct
{//�㴨���10�����󣬺̴����20������̨�����8������
	s32 TargetPos;							//Ŀ��λ��
//	s32 ContourVel;							//�����ٶ�
//	s32 TarVelocity;						//Ŀ���ٶ�

//	s32 OriginOffset;						//ԭ��ƫ��
//	u32 Acc;										//�������ٶ�
//	u32 Dec;										//�������ٶ�
//	u32 QukDec;									//����ֹͣ���ٶ�
	u16 ControlWord;						//������
	u8 OriginMode;							//��ԭģʽ
	u8 TargetMode;							//����ģʽ
	
	s16 TarTorque;							//Ŀ��ת��

	//uint8 relese;							//16λ����
}PDO_Output;									//λ��ģʽ�µ�TPDO���ݸ�ʽ

typedef __packed struct 
{
	u16 StatusWord;					//״̬��
//	u16 ControlWord;				//������
	s32 CurrentPosition;		//�û�λ�÷���
	s32 CurrentVelocity;		//�û�ʵ���ٶȷ���
	s16 ActualTprque;				//ʵ��Ť�أ���λ0.1%���Ť�ص�ռ��
	u8 CurrentMode;					//����ģʽ��ʾ
//	u8 OriginMode;					//��ԭģʽ
	
	//u8 relese;						//16λ����
}PDO_Input;								//λ��ģʽ�µ�RPDO���ݸ�ʽ

extern PDO_Output PDO_TargetInf[SERVO_NODE_ID_NUM];								//���TPDOģʽ��TPDO����
extern u8 E_AllowUpdataSend;																		//�������ݸ��º���

extern void TIM3_Config(void);
extern void EtherCATInit(char *ifname);
extern u8 EtherCAT_LinkSta(void);
extern u8 EtherCAT_LinkErrCheck(void);
extern u16 EtherCAT_SendPDOFinish(void);
extern s32 EtherCAT_GetCurrentPosition(u8 Node_ID);

#endif /* __EtherCAT_App_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
