/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : Auto_2.h
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 21/10/2015
* Description        : �Զ�����ͷ�ļ�
***************************************************************************************/
#ifndef _Auto_2_h_
#define _Auto_2_h_

#include "Parameter.h"
#include "StatusControl.h"

#define AUTO_WAITE				0
#define AUTO_RUNNING			1
#define AUTO_PAUSE				2
#define AUTO_ERROR				3

#define LISTNUM 					6						//���г����ܴ���6��

extern u8  g_AutoStatue;
extern u16 g_Auto_PresentLine;

//�ӳ�����ر���
extern u8  g_SubProgram_Step_Run[SAVEPROGRAMNUM_SUB];    				//�Ƿ������ӳ���������������λ
extern u8  g_SubProgram_Start[SAVEPROGRAMNUM_SUB];							//�ӳ���ʼ
extern u8  g_SubProgram_Finish[SAVEPROGRAMNUM_SUB];							//�ӳ������
extern u8  g_SubProgram_ContralEnd[SAVEPROGRAMNUM_SUB];					//�����ӳ������
extern u8  g_Read_SubProgram[SAVEPROGRAMNUM_SUB];								//�ӳ����ȡ���
extern u16 g_SubProgram_PresentLine[SAVEPROGRAMNUM_SUB];				//�ӳ��������к�
extern u32 g_SubProgram_ActionRun_Timer[SAVEPROGRAMNUM_SUB];
extern u8  g_SubProgram_ActionRun_Step[SAVEPROGRAMNUM_SUB];
extern u32 g_SubProgram_ActionTimeOut_Time[SAVEPROGRAMNUM_SUB];

extern u8  g_Auto_Order_Start;
extern u8  g_Auto_Order_Pause;
extern u8  g_Auto_Order_Stop;

extern u32 g_Auto_ActionRun_Timer;
extern u8  g_Auto_ActionRun_Step;
extern u8  g_Auto_ActionNcWait_Flag;
extern u8  g_Auto_SubActionNcWait_Flag[SAVEPROGRAMNUM_SUB];
extern u32 g_Auto_ActionTimeOut_Time;
extern u8  g_Auto_ActionTimeOut_Flag;
extern u8  g_Auto_ActionError_Flag;
extern u8  g_Auto_WorkFinished_Flag;
extern u8  g_Auto_CJWorkFinished_Flag;
extern u8  MD_PositionErr_Flag;
extern u8  g_Start_ActionRun_Step;
extern u8  g_Reset_ActionRun_Step;
extern u32 g_Auto_Valid_Timer;
extern u8  g_Auto_Valid_Flag;
extern u8 g_SubAuto_Valid_Flag[SAVEPROGRAMNUM_SUB];
extern u32 g_SubAuto_Valid_Timer[SAVEPROGRAMNUM_SUB];


extern u16 Action_Step_List_Num;
extern u16 Action_Step_Run_Num;
extern u16 Action_Step_Confirm_Num;
extern u16 SubAction_Step_List_Num[SAVEPROGRAMNUM_SUB];
extern u16 SubAction_Step_Run_Num[SAVEPROGRAMNUM_SUB];
extern u16 SubAction_Step_Confirm_Num[SAVEPROGRAMNUM_SUB];

//jump���ܲ���
extern u16 m_JumpStepRunNum;											//��������ת�����к�
extern u16 m_JumpSubStepRunNum[SAVEPROGRAMNUM_SUB];								//�ӳ�����ת�����к�

//while���ܲ���
#define WHILE_NEST_MAX							10														//whileǶ��������
extern u8  m_WhileNC;														//������while����Ƕ�ײ���������
extern u8  m_WhileRunFlag[WHILE_NEST_MAX];				//������while�������б�־
extern u16 m_WhileLineNum[WHILE_NEST_MAX];				//������while�����к�
//extern u16 m_WhileOverLineNum[WHILE_NEST_MAX];	//������while�����Ӧ����������к�
extern u16 m_WhileCycCounter[WHILE_NEST_MAX];		//������while����ѭ������
extern u8  m_WhileJudgeType[WHILE_NEST_MAX];			//������while������������
extern s32 m_WhileJudgePar1[WHILE_NEST_MAX];			//������while�������1
extern s32 m_WhileJudgePar2[WHILE_NEST_MAX];			//������while�������2
extern u8  m_WhileJudgeRes[WHILE_NEST_MAX];			//������while�����жϽ����0ʧ�ܣ�1�ɹ�

extern u8  m_WhileSubNC[SAVEPROGRAMNUM_SUB];																//�ӳ���while����Ƕ�ײ���������
extern u8  m_WhileSubRunFlag[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];						//�ӳ���while�������б�־
extern u16 m_WhileSubLineNum[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];						//�ӳ���while�����к�
//extern u16 m_WhileOverSubLineNum[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];			//�ӳ���while�����Ӧ����������к�
extern u16 m_WhileSubCycCounter[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];				//�ӳ���while����ѭ������
extern u8  m_WhileSubJudgeType[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];					//�ӳ���while������������
extern s32 m_WhileSubJudgePar1[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];					//�ӳ���while�������1
extern s32 m_WhileSubJudgePar2[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];					//�ӳ���while�������2
extern u8  m_WhileSubJudgeRes[SAVEPROGRAMNUM_SUB][WHILE_NEST_MAX];					//�ӳ���while�����жϽ����0ʧ�ܣ�1�ɹ�

//if-else���ܲ���
#define IF_ELSE_NEST_MAX							10																			//If-ElseǶ��������
extern u8  m_IfElseNC;																				//������If-Else����Ƕ�ײ���������
extern u8  m_IfElseJudgeType[IF_ELSE_NEST_MAX];							//������If-Else������������
extern s32 m_IfElseJudgePar1[IF_ELSE_NEST_MAX];							//������If-Else�������1
extern s32 m_IfElseJudgePar2[IF_ELSE_NEST_MAX];							//������If-Else�������2
extern u8  m_IfElseJudgeRes[IF_ELSE_NEST_MAX];								//������If-Else�����жϽ����0ʧ�ܣ�1�ɹ�
extern u8  m_IfElseJudgeRunFlag[IF_ELSE_NEST_MAX];						//������If-Else�������ִ�б�־��0δִ�У�1��ִ��

extern u8  m_IfElseSubNC[SAVEPROGRAMNUM_SUB];																	//�ӳ���If-Else����Ƕ�ײ���������
extern u8  m_IfElseSubJudgeType[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX];				//�ӳ���If-Else������������
extern s32 m_IfElseSubJudgePar1[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX];				//�ӳ���If-Else�������1
extern s32 m_IfElseSubJudgePar2[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX];				//�ӳ���If-Else�������2
extern u8  m_IfElseSubJudgeRes[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX];					//�ӳ���If-Else�����жϽ����0ʧ�ܣ�1�ɹ�
extern u8  m_IfElseSubJudgeRunFlag[SAVEPROGRAMNUM_SUB][IF_ELSE_NEST_MAX];			//�ӳ���If-Else�������ִ�б�־��0δִ�У�1��ִ��

extern u8 Flag_Falling_Edge;
extern u8 Flag_Rising_Edge;
extern u8 Flag_Falling_Edge_Sub;
extern u8 Flag_Rising_Edge_Sub;
extern u8 Flag_Keep_Move[Axis_Num + Ext_Axis_Num];

extern u8 Detect_Falling_Edge;
extern u8 Detect_Rising_Edge;
extern u8 Detect_Falling_Edge_Sub[SAVEPROGRAMNUM_SUB];
extern u8 Detect_Rising_Edge_Sub[SAVEPROGRAMNUM_SUB];

extern u8 Program_Origin_Axis;
extern u8 Program_Axis_Origin_Flag[Axis_Num + Ext_Axis_Num];
//void Program_Axis_Origin(u8);

void AutoModeControl(void);
void AutoPauseOperate(void);
void AutoStopOperate(void);
void ActionStepControl(void);


//��Ҫָ��--order--
#define   OR_BASICORDER      			0x01  //����ָ��
#define   OR_AXISORDER       			0x02  //���ָ��
#define   OR_IOORDER         			0x03  //IOָ��

//������---key------//
#define   K_PROGRAMSTART           0x01	 //������ʼ
#define   K_PROGRAMEND             0x02	 //���������
#define   K_DELAY                  0x03	 //��ʱ
#define   K_SUBPROGRAM             0x04  //�ӳ���
#define   K_JUMP                   0x05  //��תָ��
#define   K_WHILE                  0x06  //Whileָ��
#define   K_CYCLEOVER              0x07  //ѭ������
#define   K_IF                     0x08  //ִ��IF���
#define   K_ELSE                   0x09  //ִ��ELSE���
#define   K_OVER                   0x0A	 //ִ�н���IF-ELSE���
#define   K_SPECIAL                0x0B  //ִ������ָ��
#define   K_OUTDETECT            	 0x0C	 //������
#define   K_PULSE_OUTPUT           0x67	 //�������
#define   K_USER                   0x68	 //�û�����
#define   K_KEEP_MOVE			   			0x0D	 //��������
#define   K_MDPOSITION          	0x0E	 //���λ��
#define   K_MDPOCOUNT	          	0x0F	 //������
#define   K_XAXIS                 0x10	 //X���ƶ�
#define   K_LAXIS                 0x11	 //L���ƶ�
#define   K_ZAXIS                 0x12	 //Z���ƶ�
#define   K_OAXIS                 0x13	 //O���ƶ�
#define   K_INCREMENT_RUNNING     0x66	 //102�����˶�
#define   K_NEGTIVE_SEARCH        0x69	 //105��������
#define   K_MACHINE_ORIGIN        0x6A	 //106��е����
#define   K_POSSET				        0x6B	 //107λ������
#define		K_SLOWPOINT							0x6C	 //108���ٵ�
#define		K_INTER_START						0x6D		//109�岹��ʼ
#define		K_INTER_OVER						0x6E		//110�岹����
#define		K_ADVENCE								0x6F		//111��ǰȷ��
#define		K_INTER_LINE						0x70		//112ֱ�߲岹
#define		K_AXISMOVE							0x71		//113���ƶ�
#define		K_ANGLE_ARC							0x72		//114Բ�Ľ�Բ��

//�������-˫·���-4·
#define   K_IOINSTRUCT_OUTPUT1     0x14	//���ָ��1-��λY0
#define   K_IOINSTRUCT_OUTPUT2     0x15	//���ָ��2-��λY0
#define   K_IOINSTRUCT_OUTPUT3     0x16	//���ָ��3
#define   K_IOINSTRUCT_OUTPUT4     0x17	//���ָ��4
#define   K_IOINSTRUCT_OUTPUT5     0x18	//���ָ��5
#define   K_IOINSTRUCT_OUTPUT6     0x19	//���ָ��6
#define   K_IOINSTRUCT_OUTPUT7     0x1A	//���ָ��7
#define   K_IOINSTRUCT_OUTPUT8     0x1B	//���ָ��8

#define   K_IOINSTRUCT_OUTPUT9     0x1C	//���ָ��9 -��λY4
#define   K_IOINSTRUCT_OUTPUT10    0x1D	//���ָ��10-��λY4
#define   K_IOINSTRUCT_OUTPUT11    0x1E	//���ָ��11
#define   K_IOINSTRUCT_OUTPUT12    0x1F	//���ָ��12
#define   K_IOINSTRUCT_OUTPUT13    0x20	//���ָ��13
#define   K_IOINSTRUCT_OUTPUT14    0x21	//���ָ��14
#define   K_IOINSTRUCT_OUTPUT15    0x22	//���ָ��15
#define   K_IOINSTRUCT_OUTPUT16    0x23	//���ָ��16

#define   K_IOINSTRUCT_OUTPUT17    0x24	//���ָ��17
#define   K_IOINSTRUCT_OUTPUT18    0x25	//���ָ��18
#define   K_IOINSTRUCT_OUTPUT19    0x26	//���ָ��19
#define   K_IOINSTRUCT_OUTPUT20    0x27	//���ָ��20

#define   K_IOINSTRUCT_OUTPUT21    0x28	//���ָ��21
#define   K_IOINSTRUCT_OUTPUT22    0x29	//���ָ��22
#define   K_IOINSTRUCT_OUTPUT23    0x2A	//���ָ��23
#define   K_IOINSTRUCT_OUTPUT24    0x2B	//���ָ��24
#define   K_IOINSTRUCT_OUTPUT25    0x2C	//���ָ��25
#define   K_IOINSTRUCT_OUTPUT26    0x2D	//���ָ��26
#define   K_IOINSTRUCT_OUTPUT27    0x2E	//���ָ��27
#define   K_IOINSTRUCT_OUTPUT28    0x2F	//���ָ��28
#define   K_IOINSTRUCT_OUTPUT29    0x30	//���ָ��29
#define   K_IOINSTRUCT_OUTPUT30    0x31	//���ָ��30
#define   K_IOINSTRUCT_OUTPUT31    0x32	//���ָ��31
#define   K_IOINSTRUCT_OUTPUT32    0x33	//���ָ��32
#define   K_IOINSTRUCT_OUTPUT33    0x34	//���ָ��33
#define   K_IOINSTRUCT_OUTPUT34    0x35	//���ָ��34
#define   K_IOINSTRUCT_OUTPUT35    0x36	//���ָ��35
#define   K_IOINSTRUCT_OUTPUT36    0x37	//���ָ��36
#define   K_IOINSTRUCT_OUTPUT37    0x38	//���ָ��37
#define   K_IOINSTRUCT_OUTPUT38    0x39	//���ָ��38
#define   K_IOINSTRUCT_OUTPUT39    0x3A	//���ָ��39
#define   K_IOINSTRUCT_OUTPUT40    0x3B	//���ָ��40
#define   K_IOINSTRUCT_OUTPUT41    0x3C	//���ָ��41
#define   K_IOINSTRUCT_OUTPUT42    0x3D	//���ָ��42
#define   K_IOINSTRUCT_OUTPUT43    0x3E	//���ָ��43
#define   K_IOINSTRUCT_OUTPUT44    0x3F	//���ָ��44
#define   K_IOINSTRUCT_OUTPUT45    0x40	//���ָ��45
#define   K_IOINSTRUCT_OUTPUT46    0x41	//���ָ��46
#define   K_IOINSTRUCT_OUTPUT47    0x42	//���ָ��47
#define   K_IOINSTRUCT_OUTPUT48    0x43	//���ָ��48
#define   K_IOINSTRUCT_OUTPUT49    0x44	//���ָ��49
#define   K_IOINSTRUCT_OUTPUT50    0x45	//���ָ��50
#define   K_IOINSTRUCT_OUTPUT51    0x46	//���ָ��51
#define   K_IOINSTRUCT_OUTPUT52    0x47	//���ָ��52

//�����źż��ӿ�
#define   K_IOINSTRUCT_INPUT1      0x48	//����ָ��1
#define   K_IOINSTRUCT_INPUT2      0x49	//����ָ��2
#define   K_IOINSTRUCT_INPUT3      0x4A	//����ָ��3
#define   K_IOINSTRUCT_INPUT4      0x4B	//����ָ��4
#define   K_IOINSTRUCT_INPUT5      0x4C	//����ָ��5
#define   K_IOINSTRUCT_INPUT6      0x4D	//����ָ��6
#define   K_IOINSTRUCT_INPUT7      0x4E	//����ָ��7
#define   K_IOINSTRUCT_INPUT8      0x4F	//����ָ��8
#define   K_IOINSTRUCT_INPUT9      0x50	//����ָ��9
#define   K_IOINSTRUCT_INPUT10     0x51	//����ָ��10
#define   K_IOINSTRUCT_INPUT11     0x52	//����ָ��11
#define   K_IOINSTRUCT_INPUT12     0x53	//����ָ��12
#define   K_IOINSTRUCT_INPUT13     0x54	//����ָ��13
#define   K_IOINSTRUCT_INPUT14     0x55	//����ָ��14
#define   K_IOINSTRUCT_INPUT15     0x56	//����ָ��15
#define   K_IOINSTRUCT_INPUT16     0x57	//����ָ��16
#define   K_IOINSTRUCT_INPUT17     0x58	//����ָ��17
#define   K_IOINSTRUCT_INPUT18     0x59	//����ָ��18
#define   K_IOINSTRUCT_INPUT19     0x5A	//����ָ��19
#define   K_IOINSTRUCT_INPUT20     0x5B	//����ָ��20
#define   K_IOINSTRUCT_INPUT21     0x5C	//����ָ��21
#define   K_IOINSTRUCT_INPUT22     0x5D	//����ָ��22
#define   K_IOINSTRUCT_INPUT23     0x5E	//����ָ��23
#define   K_IOINSTRUCT_INPUT24     0x5F	//����ָ��24
#define   K_IOINSTRUCT_INPUT25     0x60	//����ָ��25
#define   K_IOINSTRUCT_INPUT26     0x61	//����ָ��26
#define   K_IOINSTRUCT_INPUT27     0x62	//����ָ��27
#define   K_IOINSTRUCT_INPUT28     0x63	//����ָ��28
#define   K_IOINSTRUCT_INPUT29     0x64	//����ָ��29
#define   K_IOINSTRUCT_INPUT30     0x65	//����ָ��30


//�����valueֵ����
#define V_Z_POSITION				0X04	//Z������
#define V_ONCE							0X08	//����
#define V_CYCLE							0X09	//ѭ��
#define V_O_METHOD					0X0A	//O��ʽ
#define V_X_POSITION				0X0B	//X������
#define V_Y_POSITION				0X0C	//Y������
#define V_PROGRAM_START			0x10	//����ʼ
#define V_PROGRAM_END				0x11	//�������
#define V_ONCE_RUN					0x12	//����ִ��
#define V_ONLY_EQUAL				0x14	//ֻ����
#define V_EQUAL							0x15	//���ڣ�������
#define V_NOT_EQUAL					0x16	//������
#define V_NOTONLY_EQUAL			0x17	//�����ڣ�������
#define V_R_METHOD					0x18	//R��ʽ
#define V_I_METHOD					0x19	//I��ʽ
#define V_R_LINE_NUM_IO			0x1A	//R�к�/IO��
#define V_R_NUM							0x1B	//R����/H&L
#define V_RI_NULL						0x1C	//NULL
#define V_MD_AXSIS					0X1D	//�����ѡ�� 29-36
#define V_SUBPROGRAM_SEQ		0x25	//�ӳ����
#define V_JUMP_TO						0x26	//��ת
#define V_HIGH_LEVEL       	0x27	//�ߵ�ƽ
#define V_LOW_LEVEL					0x28	//�͵�ƽ
#define V_RISING_EDGE				0x29	//������
#define V_FALLING_EDGE			0x2A	//�½���
#define V_KEEP_MOVE_X				0x2B	//X��
#define V_KEEP_MOVE_Y				0x2C	//Y��
#define V_KEEP_MOVE_Z				0x2D	//Z��
#define V_KEEP_MOVE_O				0x2E	//O��
#define V_SUSPEND						0X2F	//����ָ��-��ͣ
#define V_STOP							0X30	//����ָ��ֹͣ
#define V_COUNTER						0X31	//R��ʽ-������
#define V_JUMP_LABEL				0X32	//��ǩ
#define V_LABEL_NUM					0X33	//��ǩ��
#define V_DISTANCE    	    0X34	//52����λ��
#define V_USEFUL						0X35	//53��Ч
#define V_USELESS    	      0X36	//54��Ч
#define V_SET	   	  				0X38	//56��λ
#define V_RESET    	  			0X39	//57��λ
#define V_USER1    	  			0X3A	//58user1
#define V_USER2    	  			0X3B	//59user2
#define V_USER3    	  			0X3C	//60user3
#define V_USER4    	  			0X3D	//61user4
#define V_USER5    	  			0X3E	//62user5
#define V_USER6    	  			0X3F	//63user6
#define V_USER7    	  			0X40	//64user7
#define V_USER8    	  			0X41	//65user8
#define V_ADD    	    			0X42	//66 +
#define V_MINUS    	  			0X43	//67 -
#define V_UEQUAL						0X44	//68 =
#define V_MULTIP      			0X45	//69 *
#define V_DIVIDE    				0X46	//70 /
#define V_EXCESS    				0X47	//71 %
#define V_SLOWPOS						0X49	//74���ٵ�
#define V_MDGOOD						0X55	//85��Ʒ
#define V_NUMBER						0X56	//86����
#define V_LAYER_NUM					0X57	//87����
#define V_LAYER_FULL				0X58	//88����
#define V_STACK_FULL				0X59	//89����
#define V_MORE_THAN					0X5A	//90����
#define V_LESS_THAN					0X5B	//91С��
#define V_AUTO							0X5C	//92�Զ�
#define V_P_METHOD					0X5D	//93P��ʽ
#define V_AXISCHOOSE    		0X5E	//94��ѡ��
#define V_SETPOSITION    		0X5F	//95��������ֵ
#define V_XAXISGREATER    	0X60	//96X����ڵ��� 
#define V_YAXISGREATER    	0X61	//97Y����ڵ���
#define V_ZAXISGREATER    	0X62	//98Z����ڵ���
#define V_OAXISGREATER    	0X63	//99O����ڵ���
#define V_XAXISEQUAL       	0X64	//100X�����
#define V_YAXISEQUAL       	0X65	//101Y�����
#define V_ZAXISEQUAL       	0X66	//102Z�����
#define V_OAXISEQUAL       	0X67	//103O�����
#define V_XAXISLESS        	0X68	//104X��С�ڵ���
#define V_YAXISLESS        	0X69	//105Y��С�ڵ���
#define V_ZAXISLESS         0X6A	//106Z��С�ڵ���
#define V_OAXISLESS       	0X6B	//107O��С�ڵ���
#define V_FORMULATION      	0X6C	//108�䷽
#define V_KEEP_MOVE_U				0X6D	//109U��
#define V_KEEP_MOVE_V				0X6E	//110V��
#define V_ADVANCE						0X6F	//111��ǰȷ����

#endif

/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/

