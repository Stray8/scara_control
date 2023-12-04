
#include "CANopen.h"
#include "Parameter.h"
#include "Error.h"
#include "EtherCAT_App.h"
#include "ethercatcoe.h"
#include "ethercattype.h"
#include "in.h"
#include "usart.h"

u16 PDO_Actual_Status[SERVO_NODE_NUM] = {0};								//���ڱ����ŷ�������״̬
s32 PDO_Cur_Position[SERVO_NODE_NUM] = {0};									//���ڱ����ŷ������ĵ�ǰλ��
u8 PDO_FirstGetPosition[SERVO_NODE_NUM] = {0};							//���ڱ����ŷ��״�λ�û�ȡ�ɹ�
s16 PDO_Cur_Tprque[SERVO_NODE_NUM] = {0};										//���ڱ����ŷ������ĵ�ǰŤ�أ���λ0.1%���Ť�ص�ռ��
//u16 PDO_Cur_Controlword[SERVO_NODE_NUM] = {0};							//���ڱ����ŷ������ĵ�ǰ������״̬
u8 PDO_Cur_Mode[SERVO_NODE_NUM] = {0};											//���ڱ����ŷ������ĵ�ǰģʽ
s32 PDO_Cur_Speed[SERVO_NODE_NUM] = {0};										//���ڱ����ŷ������ĵ�ǰת��

u16 PDO_Actual_StatusUpdateTime[SERVO_NODE_NUM] = {0};			//���ڱ��״̬��־��������ʱʱ��
u16 PDO_Actual_StatusUpdate[SERVO_NODE_NUM] = {0};					//���ڱ��״̬��־�������Ѹ���

/***************************************************************************
**  ��������  PDO_ControlWordSet()
**	�����������
**	�����������
**	�������ܣ����������õ�PDO���ķ���
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void PDO_ControlWordSet(u8 Node_ID, u16 controlword)
{
//	u16 i = 0;
	
	PDO_TargetInf[ServoNodeID[Node_ID]].ControlWord = controlword;
	EtherCAT_SendPDOFinish();

//	while(i < 4)
//	{
//		i++;
//		
//		if(PDO_Cur_Controlword[Node_ID - 1] == controlword)
//		{
//			break;
//		}
//		delay_ms(CANOPEN_DEAL_PDO_CONTROL_DELAY);
//	}
	delay_ms(CANOPEN_DEAL_PDO_CONTROL_DELAY);
}

/***************************************************************************
**  ��������  PDO_ServoPositionSet()
**	�����������
**	�����������
**	�������ܣ�����PDO��Ϣ�������ŷ���λ�á��ٶ�
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void PDO_ServoPositionSet(u8 Node_ID, u32 Target_Position, u32 Target_Velocity)
{
	/*�����ŷ���Ŀ��λ�ú��ٶ�ֵ*/
//	PDO_TargetInf[ServoNodeID[Node_ID]].ContourVel = Target_Velocity;
//	PDO_TargetInf[ServoNodeID[Node_ID]].TargetPos = Target_Position;
//	
//	EtherCAT_SendPDOFinish();
//	delay_ms(CANOPEN_DEAL_PDO_DELAY);
}

/***************************************************************************
**  ��������  PDO_ServoAccDecSet()
**	�����������
**	�����������
**	�������ܣ�����PDO��Ϣ�������ŷ������ٶȺͼ��ٶ�
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void PDO_ServoAccDecSet(u8 Node_ID, u32 Acc, u32 Dec)
{
	/*�����ŷ������ٶȺͼ��ٶ�*/
//	PDO_TargetInf[ServoNodeID[Node_ID]].Acc = Acc;
//	PDO_TargetInf[ServoNodeID[Node_ID]].Dec = Dec;
//	switch(JDZ_Parameter.Server)
//	{
//		case 0://�̴����
//			PDO_TargetInf[ServoNodeID[Node_ID]].QukDec = Dec * 2;
//			break;
//		case 1://�㴨���
//			PDO_TargetInf[ServoNodeID[Node_ID]].QukDec = Dec * 2;
//			break;
//		case 2://���ŵ��
//			PDO_TargetInf[ServoNodeID[Node_ID]].QukDec = Dec * 2;
//			break;
//		default:
//			PDO_TargetInf[ServoNodeID[Node_ID]].QukDec = Dec * 2;
//			break;
//	}
}

/***************************************************************************
**  ��������  Get_Status_Position()
**	�����������
**	�����������
**	�������ܣ������ŷ����ϴ���״̬�ֺ�λ����Ϣ
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void Get_Status_Position(CanRxMsg RxMessage)
{
	
	CAN_Recieve_Data.StdId = RxMessage.StdId;
	CAN_Recieve_Data.Data[0] = RxMessage.Data[0];
	CAN_Recieve_Data.Data[1] = RxMessage.Data[1];
	CAN_Recieve_Data.Data[2] = RxMessage.Data[2];
	CAN_Recieve_Data.Data[3] = RxMessage.Data[3];
	CAN_Recieve_Data.Data[4] = RxMessage.Data[4];
	CAN_Recieve_Data.Data[5] = RxMessage.Data[5];
	CAN_Recieve_Data.Data[6] = RxMessage.Data[6];
	CAN_Recieve_Data.Data[7] = RxMessage.Data[7];
	
	if(CAN_Recieve_Data.Data[0] == CAN_HEARTBEAT)
	{//���ն���������
//		CAN1_RecieveDataDecode();
		
	}
//	if(CAN_Recieve_Data.Data[0] == CAN_HEARTBEAT || CAN_Recieve_Data.Data[0] == CAN_HEART_SEND_CODE || CAN_Recieve_Data.Data[0] == CAN_HEART_WRITE_CODE)CANReceiveDataFinished = TRUE;
}























