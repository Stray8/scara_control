
#ifndef __NMT_H_
#define __NMT_H_

#include "CANopen.h"

/*NMT����*/
#define START_REMOTE_NODE           0x01								//����Զ�̽ڵ�����
#define STOP_REMOTE_NODE            0x02								//ֹͣԶ�̽ڵ�����
#define ENTER_PRE_OPERATIONAL_STATE 0x80								//����Ԥ����״̬����
#define RESET_NODE                  0x81								//��λ�ڵ�����
#define RESET_COMMUNICATION         0x82								//��λͨ������

/*NMT״̬������*/
typedef enum {
	Initialisation = 0, 				//����
	Stop = 4,										//ֹͣ
	Operate = 5, 								//����
	Pre_Operate = 127 					//Ԥ����
}NMT_Machine_state;

/***********NMT������***************/
NMT_Machine_state Get_NMT_State(void);
void NMT_Boot_Up(u8);
void Start_Remote_Node(u8 Node_ID);
void Stop_Remote_Node(u8 Node_ID);
void Enter_Pre_Operational_State(u8 Node_ID);
void Reset_Node(u8 Node_ID);
void Reset_Communication(u8 Node_ID);

#endif







