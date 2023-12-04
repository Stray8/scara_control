#ifndef __PDO_H_
#define __PDO_H_

#include "CANopen.h"

extern u16 PDO_Actual_Status[SERVO_NODE_NUM];								//���ڱ����ŷ�������״̬
extern s32 PDO_Cur_Position[SERVO_NODE_NUM];								//���ڱ����ŷ������ĵ�ǰλ��
extern u8 PDO_FirstGetPosition[SERVO_NODE_NUM];							//���ڱ����ŷ��״�λ�û�ȡ�ɹ�
extern s16 PDO_Cur_Tprque[SERVO_NODE_NUM];									//���ڱ����ŷ������ĵ�ǰŤ�أ���λ0.1%
//extern u16 PDO_Cur_Controlword[SERVO_NODE_NUM];							//���ڱ����ŷ������ĵ�ǰ������״̬
extern u8 PDO_Cur_Mode[SERVO_NODE_NUM];											//���ڱ����ŷ������ĵ�ǰģʽ
extern s32 PDO_Cur_Speed[SERVO_NODE_NUM];										//���ڱ����ŷ������ĵ�ǰת��

extern u16 PDO_Actual_StatusUpdateTime[SERVO_NODE_NUM];			//���ڱ��״̬��־��������ʱʱ��
extern u16 PDO_Actual_StatusUpdate[SERVO_NODE_NUM];					//���ڱ��״̬��־�������Ѹ���

void PDO_ControlWordSet(u8 Node_ID, u16 controlword);																			//���ÿ�����
void PDO_ServoPositionSet(u8 Node_ID, u32 Target_Position, u32 Target_Velocity);          //����λ��ģʽ��Ŀ��λ�ú��ٶ�
void PDO_ServoAccDecSet(u8 Node_ID, u32 Acc, u32 Dec);          													//����λ��ģʽ���������ٶȺ��������ٶ�
void Get_Status_Position(CanRxMsg RxMessage);            																	//CAN���Ĵ���

#endif
