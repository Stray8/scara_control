/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#ifndef __statuscontrol_h_
#define __statuscontrol_h_
#include "stm32f4xx.h"

//����ģʽ
#define WAIT_MODE			  			0x00     //�ȴ�ģʽ
#define AUTO_WORK_MODE        0x01	   //ѡ���Զ�����ģʽģʽ
#define MANUAL_WORK_MODE      0x02	   //ѡ���ֶ�����ģʽģʽ
#define FREE_PROGRAM_MODE	  	0x03     //ѡ�����ɱ��ģʽ
#define IO_DEBUG_MODE		  		0x04	   //ѡ��IO����ģʽ

//����ƶ�����
#define NEGATIVE              0x00	   //����
#define	POSITIVE              0x01	   //����
#define NONE                  0x02	   //

//�ᶨ��
#define Axis_Num							0x04	   //֧�ֵ������
#define X_Axsis				  			0x00	   //X��
#define L_Axsis				  			0x01	   //L��
#define Z_Axsis				  			0x02	   //Z��
#define O_Axsis				  			0x03	   //O��
#define U_Axsis				  			0x04	   //U��
#define V_Axsis				  			0x05	   //V��

#define Ext_Axis_Num					0x02	   //֧�ֵ���չ�����
#define U_Ext_Axsis				  	0x00	   //U��
#define V_Ext_Axsis				  	0x01	   //V��

//��չ��
#define EXTEN_AXIS_NUM	      0x02

//#define   NO_AXIS    0x00
//#define   X_AXIS     0x01
//#define   L_AXIS     0x02
//#define   Z_AXIS     0x03
//#define   O_AXIS     0x04

//��ԭ�㷽ʽ
#define FOM_X				0x00
#define FOM_Z				0x01
#define FOM_Y				0x02
#define FOM_O				0x03
#define FOM_Y_X     0x04
#define FOM_X_Y     0x05
#define FOM_Z_X     0x06
#define FOM_X_Z     0x07
#define FOM_O_X     0x08
#define FOM_Z_X_L   0x09
#define FOM_Z_L_X   0x0A
#define FOM_Z_L_X_O 0x0B
#define FOM_Z_X_L_O 0x0C
#define FOM_L_O_Z_X 0x0D
#define FOM_Z_O_X_L 0x0E
#define FOM_O_Z_X_L 0x0F

//���IO��λѡ��
#define Null				  			        0x00	   //δѡ��
#define Before_Origin				  			0x01	   //����ǰ
#define After_Origin				  			0x02	   //�����
#define Common_Alarm				  			0x03	   //��ͨ����
#define Emerge_Alarm				  			0x04	   //��ͣ����
#define PAUSE_Select				  		  0x05	   //��ͣ
#define STOP_Select				  		    0x06	   //ֹͣ

extern u8  Origin_Backed;			  							//��ԭ�����
extern u8  Axsis_Origin_Backed[Axis_Num + Ext_Axis_Num];		//�����������ɱ�־λ
extern u8  Back_Origin_Flag; 		 	 						//��ԭ�������־λ
extern u8  Initialize_Finished;		  					//��ʼ�����
extern u8  Work_Status ;				  						//����״̬
extern u32 Input_Detect_Time;
extern u32 Communication_Time;
extern u8  OffLine_Flag;
extern u8  OnLineCommunicated_Flag;
extern u8  Input_Detect_Enable;
extern u8  Jog_Pause_Enable;
extern u8  g_Current_SafeArea_Num;	  				//��ȫ�����
extern u8  Axis_Manul_BackToOrigin;    				//�ֶ�����
extern u8  Axsis_Chosen;				  						//�˶���ѡ��
extern u8  Axsis_Move_Direction[Axis_Num + Ext_Axis_Num] ;	//�˶��᷽��ѡ��
extern u8  Input_Count17;
extern u8  Input_Count18;
extern u8  Input_Count19;
extern u8  Input_Count20;
extern u8  Servo_Stop_Done[Axis_Num + Ext_Axis_Num];
extern u8  g_Auto_Reset_Flag;
extern u8  Robot_Auto_Reset;
extern u8  g_Auto_LOrigin_Flag;
extern u32 g_USART_Delay_Timer;
extern u8  g_MoveCmdRetrans;

extern void WorkMode(void);
extern void CurProgramRead(u8 programNum);
extern void ActionControl(void);	  			//��������
extern void StatusControl(void);	  			//����״̬���
extern void Robot_Reset(void);
extern void Servo_Stop(u8);
extern u8 ManulSafeAreaDetec(void);
extern void ClosePulseReset(u8 Axis);

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

