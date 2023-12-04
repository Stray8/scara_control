/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __auto_h_
#define __auto_h_

#define ONCE_MODE   0x01	  //����ģʽ
#define SINGLE_MODE 0x02    //����ģʽ
#define LOOP_MODE   0x03	  //ѭ��ģʽ
#define AUTO_START  0x04	  //����
#define AUTO_STOP   0x05	  //ֹͣ

#define RUN    0x00	   //����
#define PAUSE  0x01	   //��ͣ
#define STOP   0x02    //ֹͣ

extern u32 Auto_Pulse_Count;
extern u8 Auto_Mode;			  //�Զ�ģʽ�µ�ģʽѡ��
extern u8 Single_Mode_Enable;
extern u8 Once_Mode_Enable;
extern u8 Loop_Mode_Enable;

extern u32 Action_Delay_Time;         	//��е��ÿ�ζ������֮�����ʱ����Ϊ�ŷ��������Ǹ��涯��
//extern u8  Puls_Delay_Time[50];		 			//��������ź���ʱ
//extern u8  Puls_Delay_Enable[50];	     	//���������ʱʹ�ܱ�־λ
//extern u8  Puls_Delay_Num;
extern u8  Action_Done_Flag ;		 				//�ŷ���������ɱ�־
extern u8  Action_Delay_Flag;		 				//������ʱ���

extern u8 g_Robot_Has_Debuged_Flag;
extern u8 g_Program_Is_Debuging;

extern void Read_SaveProgram_IIC_Address(void);
extern void AutoRun(void);
//extern void AutoRunning(void);
extern void AutoReset(void);
extern void SafeAreaJudge(void);
extern void SetSingle(u8,u8,u32);

#endif


/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/


