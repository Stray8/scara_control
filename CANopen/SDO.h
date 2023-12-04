/***********************************
 *           SDO Protocol          *
 ***********************************/
#ifndef __SDO_H_
#define __SDO_H_

#include "CANopen.h"

/*SDO���ݲ�������*/
#define INITIATE_SDO_DOWNLOAD   0x20             //SDO�������أ�д��Servo����[4:7]:data-reponse:0x60 
#define INITIATE_SDO_UPLOAD     0x40             //SDO�����ϴ�����ȡServo����-reponse:0x40 [4:7]:data
#define ABORT_SDO_TRANSFER      0x80             //��ֹSDOͨ��   [4:7]:abort codes

/*SDO����֡�ṹ��*/
typedef struct   //SDO message
{
  u8  Node_ID;								//�ڵ�ID
  u16 index;									//����
  u8  sub_index;							//������
  u8  len;      							//�������ݵĳ��ȣ�+4��Ϊ�ܵ�can���ĳ���
  u8  Data[8];								//����
}SDO_Struct;

extern SDO_Struct SDO_Message;								//SDO��Ϣ֡


void SDO_Protocol(u8);               					//SDOͨ��Э��
void SDO_Process(u8);              						//SDO��Ϣ������
//void Set_Guard_Time(u8);											//���ñ����ڵ�ʱ��
void Query_Servo_parameter(u8,u16,u8);				//��ѯ�ŷ�������
//void SDO_Process_PositionMode(u8);						//�����ŷ���λ��ģʽ
//void SDO_Process_SpeedMode(u8 Node_ID);				//�����ŷ����ٶ�ģʽ
void SDO_Process_HomingMode(u8, u8, s32);			//�����ŷ�����ԭ��ģʽ
void SDO_Process_CycPosMode(u8 Node_ID);			//�����ŷ�������ͬ��λ��ģʽ
//void SDO_Process_CycCSVMode(u8 Node_ID);			//�����ŷ�������ͬ���ٶ�ģʽ
void SDO_Process_CycCSTMode(u8 Node_ID);			//�����ŷ�������ͬ��ת��ģʽ

#endif









