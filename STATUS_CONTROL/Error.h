/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __error_h_
#define __error_h_
#include "stm32f4xx.h"

#define NO_ERROR									0x00
#define ERROR_HAPPEN							0x01
#define ERROR_EMERG_HAPPEN				0x02
#define ERROR_STATUS            	0x00  		//�ᱨ���������ƽ��־λ  0x00�͵�ƽ��Ч  0x01�ߵ�ƽ��Ч

#define HARDLIMITJUDGE_EXTI         				//Ӳ��λɨ�跽ʽ-�ⲿ�ж�

#define SAFEVARIATION	100			//���ڰ�ȫ��������λ��λ������ƫ��

/*����*/
extern u8 Error_Status;				  						//����״̬
extern u8 Robot_Error_Data[15];			  			//��������
extern u8 Cancle_Genaral_Warning;
extern u16 IO_Input_waittime[30];						//�����ⳬʱʱ��
extern u16 IO_Input_keepMin[30];						//���뱣��ʱ��-����
extern u16 IO_Input_keepMax[30];						//���뱣��ʱ��-����
extern u16 IO_Sign_On_Off[30];							//�����źų������ձ�־��0������1����
extern u16 OutPut_BeforeOrigin[30];		   	 //����ǰѡ��
extern u16 OutPut_AfterOrigin[30];			     //�����ѡ��
extern u16 OutPut_Common_Alarm[30];	 //��ͨ����
extern u16 OutPut_Emerge_Alarm[30];	 //��ͣ����
extern u16 OutPut_Pause[30];			     //��ͣ
extern u16 OutPut_Stop[30];			     //ֹͣ

extern u32 Axsis_Error_Count;								//�ŷ�����������ʼ��ⶨʱ������ֹ����ֱ�ӱ���
extern u8 Axsis_Error_Permit_Flag;					//�ŷ�����������ʼ��������־

extern u8 g_Auto_ActionConflict_Flag;		//������ظ�����

/*����*/
extern void ErrorOperate(void);	      			//�����⼰����
extern void Scan_TimeOut_IO(u8 i_num);	//����IOʶ��
extern void Scan_TimeOut_OUTPUT(u8 i_num);	//����IOʶ��
extern void CloseTotalMotorError(void);	//�����ر����е�����
extern void HardLimitJudge(void);							//Ӳ��λ���

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/
