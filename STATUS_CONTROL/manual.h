/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __manual_h_
#define __manual_h_

#include "StatusControl.h"

extern u8 Jog_Move_Enable ;			  		//�綯���б�־λ
extern u8 Jog_Mode_Enable ;			  		//�綯ģʽʹ�ܱ�־λ
extern u8 Linked_Mode_Enable;		  		//����ģʽʹ�ܱ�־λ
extern u8 Linked_Move_Enable;		  		//�������б�־λ

extern u32 Jog_Pulse_Count ;		  		//�綯ģʽ��ÿ��������������
extern u32 Jog_Pulse_Count_Init ;
extern u32 Linked_Pulse;
extern u8  Axis_Manul_Speed[Axis_Num + Ext_Axis_Num];					//���ֶ��ٶ�
extern u8  Axis_Manul_Speed_Temp[Axis_Num + Ext_Axis_Num];		//��û�л�ԭ��ʱ�����ڱ���ԭ�����ֶ��ٶ�
extern u16  Axis_Step_Distance[Axis_Num + Ext_Axis_Num];			//��綯����1-100,Ĭ��50mm
extern u8  All_Axis_Point_Deleted_Flag;												//���е���ɾ����־λ

extern void IODebugOutput1(void);
extern void IODebugOutput2(void);
extern void IODebugOutput3(void);
extern void ManulDebug(void);		  							//�ֶ�����
extern void ManualJogRunnig(void);	  					//�ֶ������㶯ģʽ
extern void ManualLinkedRunning(void);					//�ֶ���������ģʽ



#endif

/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/

