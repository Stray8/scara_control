/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __parameter_h_
#define __parameter_h_
#include "stm32f4xx.h"
#include "StatusControl.h"

/******************���������***************************/
#define LARGESTPROGRAMNUM 				200						//������ָ����
#define SAVEPROGRAMNUM 						20						//�ɴ洢�ĳ�������
#define SAVEPROGRAMNUM_MAIN 			10	  				//�ɴ洢�����������
#define SAVEPROGRAMNUM_SUB 				10	  					//�ɴ洢���ӳ������

#define SAVEPROINT 								75						//ÿ��������ܱ���ĵ�
#define SAVESAFEAREA 							3							//�����õİ�ȫ������
#define SAVESOFTLIMIT 						4							//�����õ�����λ����

#define MINROBOTPOSITION   				1000000    		//���˶�����С����
#define MAXROBOTPOSITION   				9000000    		//���˶����������

#define MAXSAVEPROINT 						40						//ÿ��������ܱ���ĵ�
#define OUTPUT_ASSOCIATE_NUM    	2		//���������

#define	OUTPUT_NUM	18   //�����������
#define	INPUT_NUM		24

//��צ����
#define MDgrip_Num	6
//�û�����
#define USER_NUM	8


//���ɱ��
typedef struct 
{
	u8  Flag;  				//��־λ
	u8  List;  				//ָ����
	u8  Order; 				//ָ������
	u8  Key;   				//������
	u32 Value1;				//ָ�����
	u32 Value2;
	s32 Value3;
//	s32 XPoint;
//	s32 YPoint;
//	s32 ZPoint;
}FreeProgram;

//����洢
typedef struct 
{
	u8  Flag;	   			//�Ƿ��г���
	u8  Code;	   			//����ı�ţ��·��������ư���ʶ����
	u32 Name;	   			//��������
	u32 Name2;
	u32 Name3;
	u16 Num;	   			//�������Ĺ�ģ��С����������ʱ��ʾ�ж�
	FreeProgram Program[LARGESTPROGRAMNUM];
}SaveProgram;

//����洢
typedef struct 
{
	u8  Flag;	   			//�Ƿ��г���
	u8  Code;	   			//������
	u32 Name;	   			//��������
	u32 Name2;
	u32 Name3;
	u16 Num;	   			//�����ģ�����������
	u32 Address;   		//IIC�Ĵ洢��ַ
}SaveProgramIICAddress;

extern SaveProgramIICAddress Program_IIC_Address[SAVEPROGRAMNUM];	//�������г���Ļ�����Ϣ���洢��ַ
extern SaveProgram Free_Program_Operate;													//��ǰѡ�е�������
extern SaveProgram SubProgram_Operate[SAVEPROGRAMNUM_SUB];				//�ӳ���������

extern u8 Start_Recieve_Program_Flag;															//������տ�ʼ��־
extern u8 Current_Delete_Program;	        												//��ǰɾ������

typedef struct 
{
	u32 Left_Limit  ; //-
	u32 Right_Limit ; //+
	u8  Switch_Limit; //����
}SoftLimit;

typedef struct
{
	u32 MaxDistance;	//X���O��������
	u32 MinDistance;	//X���O����С��ȫ����
} SoftDistance;//��ȫ�г�

typedef struct 
{
	u32 X_Left ;
	u32 Z_Up ;
	u32 X_Right ;
	u32 Z_Down ;
	u8  SafeArea_Switch;
}SafeAreas;

typedef struct
{
	u8 carCoordSwitch;						//�ѿ������꿪�� 0�� 1��
	u8 MDCoordType;								//����������� 0������ 1�ѿ�������
	u32 pitchLength;							//˿���ݾ�
	u32 length[2];								//��е��1��2����
//	s32 startPoint[Axis_Num];			//�������ϻ�еĩ�����
//	s32 revolveAngle[Axis_Num];		//��ת�Ƕ�
//	s32 revolvePoint[Axis_Num];		//��ת�Ƕȶ�Ӧλ��
	u8 axisType[Axis_Num];				//������ 0ֱ���� 1��ת��
	u8 axisBackMinDir[Axis_Num];	//�����Сλ�õķ��� 0˳ʱ�� 1��ʱ��
	u8 axisInterpFlag[Axis_Num];	//��岹���� 0�� 1��
} ST_Cartesian; //�ѿ�������ϵ

//typedef struct
//{
//	float MC_P[Axis_Num];
//	float MC_I[Axis_Num];
//	float MC_D[Axis_Num];
//	float MC_C;													//����������̬����ֵ
//} ST_MotorControl_PID; //PID��������

//��洢
typedef struct 
{
	u8  Flag;			//�Ƿ��е�
	u32 Name;		  //�������  clj0831
	u32 Name2;
	u32 Name3;
	u32 Point_X;		//X��ֵ
	u32 Point_L;		//Y��ֵ	
	u32 Point_Z;		//Z��ֵ
	u32 Point_O;		//O��ֵ
}SavePoint;

////��洢����������洢
//typedef struct 
//{
//	u8  Flag;
//	u32 Point;			//X��ֵ
//}SavePointPar;

typedef struct
{
	u16 Time[Axis_Num];
}ACC;

typedef struct 
{
	u8  Axis;
	u8  Origin;											//��ԭ�㷽ʽ
	ACC Accelerate;
	u8  SpeedLevel;
	u32 AxisOriginSpeed[Axis_Num];	//��������ٶ�
	u8  OriginDir[Axis_Num];				//������㷽��0����1����
	u8  OriginPos[Axis_Num];				//����ԭ��λ�ã�0���ˣ�1����
//	u32 PulseTime;
	u8	NcPausein;									//�ⲿ��ͣ
	u8  NcStopin; 									//�ⲿֹͣ-DPF
	u8  NcStartin;									//�ⲿ����-DPF
	u8  NcOrignin;									//�ⲿ����-DPF
	u8  LCcirculation;							//�ϲ�ѭ����ʽ--������ʱ����
	u32 OrignOffset[Axis_Num];			//ԭ��ƫ��
	u8  OrignSignOnOff[Axis_Num];		//ԭ���źų�������
	u8  LimitSignOnOff[Axis_Num];		//��λ�źų�������
	u8  AlarmSignal;								//�����źŸߵ͵�ƽ
	u8  AlarmSwitch[Axis_Num];			//����������źſ���
	u8  MDgripSwitch;								//����צIO����
	u8  MDgripNum;									//����צIO���
	u8  MDgripPort[MDgrip_Num];			//����צIO�˿�
	u32 A_Circle_Pulse[Axis_Num];		//��Ȧ����
	u32 A_Circle_Distance[Axis_Num];//��Ȧ����
	u8  OutputAssociate[OUTPUT_ASSOCIATE_NUM];	//�����������Y0Y1,Y2Y3	
	u8  ZAxsisAvoidace;							//Z���ײ
	u32 ZAxsisLimit;								//Z������		
}JXSParameter;

typedef struct 
{
	u8  Switch;											//���ܿ���
	u8  Server;											//�ŷ�ѡ��
	u8  Resolu; 										//�ֱ���	
	u32 Circle_Pluse[Axis_Num];			//��Ȧ����
	u8  Motion_Dir[Axis_Num];				//�˶�����
  u8  OriginSetting[Axis_Num];		//����ֵԭ������
	u32 SERVO_STEP_LONG;						//������λ����Ӧ��ָ����	23λ-8388608 17λ-131072

}JDZParameter;	//����ֵ�����趨

typedef struct
{
	u32  Name;
	u32  Name1;
	u32  Name2;
} IONameParameter;

#define LOOP_MAX						10			//���ѭ������

#define MD_GOOD_NUM					60		//��Ʒ������15*4=60
#define MD_GOOD_PAGE_NUM		4
#define MD_GOOD_PAGE_PER		15

#define MD_POINT_NUM				64		//ÿ��ʾ�̵�������8*8=64
#define MD_POINT_PAGE_NUM		8
#define MD_POINT_PAGE_PER		8

#define PF_IONUM						4

typedef struct
{
	s32 point[Axis_Num];			//����
	s32 waitPoint[Axis_Num];	//�ȴ���
} ST_MDPostion; //���λ��

typedef struct
{
	u8 stackType;					//���ͣ�0��ʾ�̣�1������
	u8 property;					//���ԣ�0����⣬1�����
	u8 revolveMode;				//��ת��ʽ��0�������1������
	u8 gasPort;						//���׶˿�
	u8 topSwitch;					//���㹦��
	s32 goodOffset[Axis_Num];	//����ƫ��--�ѿ�������ϵ
	s32 goodheight;				//��Ʒ�߶�
	u8 stackLayer;				//�ѵ�����
	u8 loopLayer;					//ѭ�����������10�㣬���㹦�ܿ���ʱ��������һ��Ϊ����
	u8 layerNum[LOOP_MAX];//�������
	u8 horNum;						//�������
	u8 verNum;						//�������
//	ST_MDPostion sPostion[LOOP_MAX][MD_POINT_NUM];		//ȫ��ʾ�̵�
} ST_MDParameter; //������

typedef struct
{
	u8 mdMethed;					//��ⷽʽ��0��������1���ּ�
	u8 totalGood;					//����Ʒ��
	u8 startGood;					//��ʼ��Ʒ
	u8 curGood;						//��ǰ��Ʒ
	u8 curLayer;					//��ǰ����
	u8 curNum;						//��ǰ����
} ST_MDRunPara; //������в���

typedef struct
{
	u8	pfIOnum[PF_IONUM];					//�䷽�˿�
	u8	pfGood[PF_IONUM];						//�䷽��Ʒ
	u8	pfSwitch[PF_IONUM];					//�䷽����
} PFParameter; //�䷽����

typedef struct 
{
	u32 RW_Num ;					//����ƻ�
	u32 SC_Num ;					//����
	u32 CJ_Num ;					//���
	u32 JG_Num ;					//���
	u32 LJ_Num ;					//�ۼ�
	u32 NG_Num ;					//�������
}SCParameter;

typedef struct
{
	u32  USER_Name1[USER_NUM] ;
	u32  USER_Name2[USER_NUM] ;
	u32  USER_Name3[USER_NUM] ;
	s32  INIT_Num[USER_NUM] ;	//��ʼֵ
	s32  CURR_Num[USER_NUM] ;	//��ǰֵ
	u8   ELEC_RESET[USER_NUM] ;	 //������0
	u8   START_RESET[USER_NUM] ; //������0
}USERParameter; //�û�����

typedef struct
{
	u8   Switch;
	u8  Sequence[12];	
} InternetParameter;

typedef struct
{
	u8  E_OriginPosition;     //ԭ��λ��
	u32 E_OriginOffset;       //ԭ��ƫ��
	u32 E_Circle_Pulse;	      //��Ȧ����
	u32 E_Circle_Distance;	  //��Ȧ����
	u16 E_AccAcc;             //�Ӽ���
	u16 E_AccTime;            //���ٶ�ʱ��
	u32 E_MaxDistance;	      //����г�
	u8  E_Origin_Set; 			  //ԭ������
} ExtendAixParameter;

extern ExtendAixParameter ExtendAix_Parameter[EXTEN_AXIS_NUM];//��չ�����
extern InternetParameter Internet_Parameter;                  //����������
extern JXSParameter JXS_Parameter;						  							//���˶�����
extern SCParameter SC_Parameter ;						  								//��������
extern USERParameter USER_Parameter ;						  						//�û�����
extern SoftLimit Robot_SoftLimit[SAVESOFTLIMIT];		    			//������λ����
extern SafeAreas Robot_Safe_Area[SAVESAFEAREA];		      			//��ȫ�������
extern u8 All_Program_Deleted_Flag;														//�ָ������������г����Ѿ�ɾ����־λ
extern JDZParameter JDZ_Parameter;						  							//JDZ��ز���
extern ST_MDParameter sMD_Parameter; 												  //������
extern ST_MDRunPara sMD_RunPara;															//������в���
extern u8 sMD_CurMDCode;																			//��ǰ��Ʒ���
extern u8 sMD_GoodOffset_flag;																//��ǰ��Ʒ-����ƫ�Ʊ�־
extern u8 All_MD_Deleted_Flag;																//�ָ�����������������Ѿ�ɾ����־λ
extern ST_Cartesian sCartesian_Para;													//�ѿ�������ϵ����
//extern ST_MotorControl_PID sMC_PID_Para;											//�������PID����
extern PFParameter PF_Parameter;															//�䷽
extern u8 sMD_FlashCurLayer[MD_GOOD_NUM];  										//�������ĵ�ǰ��
extern u8 sMD_FlashCurNum[MD_GOOD_NUM];												//�������ĵ�ǰ��
extern u8  g_Write_FlashFlag;				  												//�����Ȳ���дflash��־λ

extern u8  g_Run_Program_Num ;																//��ǰ���еĳ�����,0��ʾ��ѡ�����
extern u8 g_Run_Program_Num_Pre;															//���ڱ��ֵ�ǰѡ�еĳ����
extern u32 SaveProgram_IIC_Address;														//������ĳ����ַ
extern u8  SaveProgram_IIC_Num ;															//������ĳ�������

extern u32 Axsis_Maxlength[Axis_Num + Ext_Axis_Num];													//���˶������λ��
extern u32 Axsis_Minlength[Axis_Num + Ext_Axis_Num];													//���˶�����Сλ��

extern u8  Temp_JXS_Parameter_SpeedLevel;											//�ٶȵȼ��ݴ����������ʵ�����˶�ʱ�޸��ٶȵȼ�

extern u8 Temp_IO_Switch_Parameter[INPUT_NUM];
extern u8 Temp_OUT_Switch_Parameter[OUTPUT_NUM];

extern SoftDistance  Robot_SoftDistance;
extern SoftDistance  Temp_SoftDistance;


#define ManulSavePointMaxNum											40					//������ĵ�����
extern SavePoint Manul_Save_Point[ManulSavePointMaxNum];
extern float Step_Coefficient[Axis_Num + EXTEN_AXIS_NUM];


#define SYSTEM_TIME_MAX 2000000000														//ϵͳʱ�Ӽ������ֵ����λ10ms��20000000s
extern u32 m_SystemTimeCounter;																//ϵͳʱ�Ӽ�����

extern u8 SpeedLevel_ParSave;																	//�ٶȵȼ��޸ı�־
extern u8 JDZ_ZeroPos_ParSave;																//����λ�����ñ�־

/*���ߵ���ٶȡ����ٶȡ�λ�õ�ת��ϵ��*/
extern float  Axsis_ParVelChange;														//�ٶȡ����ٶ�ת��ϵ��
extern float  Axsis_ParPosChange;														//λ��ת��ϵ��

extern u32 DistanceTransPulse(u8 axis,u32 distance);
extern void ParameterOrder(void);
extern void FreeProgramSend(void);
extern void ReadIICData(void);
extern void ReadIICSysParameter(void);
extern void Write_System_Set_IIC(void);
extern void Write_MDPara_Copy_IIC(void);
extern void AxisInterpParSave(void);

extern u16 Torque_T_count; //

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

