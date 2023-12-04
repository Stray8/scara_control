#ifndef __CANOPEN_H_
#define __CANOPEN_H_

#include "stm32f4xx.h"

#define  CANOPEN_MODE 																					//CANOPENģʽ����Ҫ�������������
#define  CANOPEN_DEAL_SDO_DELAY									10							//CANOPEN�ӻ������ĵ���ʱ������SDO��ʱ
#define  CANOPEN_DEAL_PDO_DELAY									5								//CANOPEN�ӻ������ĵ���ʱ������PDO��ʱ
#define  CANOPEN_DEAL_PDO_CONTROL_DELAY					10							//CANOPEN�ӻ������ĵ���ʱ������PDO�������л���ʱ

#define CAN_HEARTBEAT         0x06   			//������
#define CAN_HEART_SEND_CODE   0x04   			//���ݷ��͵Ĺ�����
#define CAN_HEART_WRITE_CODE  0x05   			//����д�Ĺ�����

/*�ŷ�����ģʽ����*/
#define  SERVO_MODE_INVALID					0										//��Чģʽ
#define  SERVO_MODE_POSITION				1										//λ��ģʽ
#define  SERVO_MODE_BACKZERO1				2										//����ģʽ1���Ը���λ������Ϊ���
#define  SERVO_MODE_BACKZERO2				3										//����ģʽ2���Ե�ǰλ��Ϊԭ��
#define  SERVO_MODE_SPEED						4										//�ٶ�ģʽ
#define  SERVO_MODE_CYC_CSP					8										//����ͬ��λ��ģʽ
#define  SERVO_MODE_CYC_CSV					9										//����ͬ���ٶ�ģʽ
#define  SERVO_MODE_CYC_CST					10										//����ͬ��ת��ģʽ

/*������ص���ֵ*/
#define  SERVO_ZERO_SPEED_VALUE			2										//�ж�Ϊ0rpm�����ת��


/*�ӽڵ��������NodeID����*/
#define SERVO_NODE_NUM 			6														//�ӽڵ����
#define SERVO_NODE_ID_NUM 	(SERVO_NODE_NUM + 1)				//NodeID����������1���㲥ID�������ӽڵ�
#define SERVO_NODE_ID_00 			0													//NodeID�����0��ʼ��ţ���������0Ϊ�㲥ID
#define SERVO_NODE_ID_01_X 		1													//X��NodeID
#define SERVO_NODE_ID_02_L		2													//L��NodeID
#define SERVO_NODE_ID_03_Z		3													//Z��NodeID
#define SERVO_NODE_ID_04_O		4													//O��NodeID
#define SERVO_NODE_ID_05_U 		5													//U��NodeID
#define SERVO_NODE_ID_06_V 		6													//V��NodeID
#define SERVO_NODE_ID_01_X2 	7													//X2��NodeID
#define SERVO_NODE_ID_02_L2		8													//L2��NodeID
#define SERVO_NODE_ID_03_Z2		9													//Z2��NodeID
#define SERVO_NODE_ID_04_O2		10												//O2��NodeID
#define SERVO_NODE_ID_05_U2 	11												//U2��NodeID
#define SERVO_NODE_ID_06_V2 	12												//V2��NodeID

#include "NMT.h"
#include "SDO.h"
#include "PDO.h"
#include "Delay.h"

/*�ŷ���������ַ����*/
//�̴����㴨�ŷ���ַ����
#define SERVO_WORK_MODE							(0x6060)							//����ģʽ��ַ��ԭ��ģʽ6��λ��ģʽ1��
#define SERVO_CONTROL_WORD					(0x6040)							//�����ֵ�ַ
#define SERVO_TARGET_POSITION				(0x607A)							//Ŀ��λ�õ�ַ
#define SERVO_ORIGIN_OFFSET					(0x607C)							//ԭ��ƫ�õ�ַ
#define SERVO_MOVE_SPEED						(0x6081)							//�����ٶȵ�ַ
#define SERVO_MOVE_ACC							(0x6083)							//�������ٶȵ�ַ
#define SERVO_MOVE_DEC							(0x6084)							//�������ٶȵ�ַ
#define SERVO_STOP_DEC							(0x6085)							//����ֹͣ���ٶȵ�ַ
#define SERVO_STOP_DEC_MODE					(0x605A)							//����ֹͣ���ٶ�ģʽ��ַ
#define SERVO_PAUSE_DEC_MODE				(0x605D)							//������ͣ���ٶ�ģʽ��ַ
#define SERVO_STATE_WORD						(0x6041)							//״̬�ֵ�ַ
#define SERVO_USER_POSITION					(0x6064)							//�û�����λ�õ�ַ
#define SERVO_BACK_HOME_MODE				(0x6098)							//��ԭ��ģʽ��ַ
#define SERVO_BACK_HOME_SP					(0x6099)							//��ԭ��ģʽ�ٶȵ�ַ
#define SERVO_BACK_ACC							(0x609A)							//��ԭ����ٶȵ�ַ
#define SERVO_TARGET_SPEED					(0x60FF)							//Ŀ���ٶȵ�ַ
#define SERVO_ERROR_CODE						(0x603F)							//��������ȡ��ַ
#define SERVO_MOVE_DIR							(0x607E)							//�˶������Ե�ַ
#define SERVO_INTERP_CYCLE					(0x60C2)							//�岹���ڼ���λ��ַ


/*�ŷ�λ��ģʽ�£������ֵ�6λ����*/
#define SINGLE_SET_POINT       		((u8)0)								//�ȴ���ǰλ��ָ��ִ����Ϻ���ִ��������
#define CHANGE_SET_IMMEDIATELY 		((u8)1)								//��ֹ����ִ�е�ָ�ִ�����µ�λ��ָ��
/*�ŷ������ֲ���*/
#define  ENABLE_OPERATION   			(0x000F)							//ʹ���ŷ�
#define  DISABLE_OPERATION  			(0x0007)							//�ȴ����ŷ�ʹ��
#define  QUICK_STOP								(0x0002)							//����ֹͣ
#define  DISABLE_VOLTAGE					(0x0000)							//�ŷ��޹���

#define CANOPEN_SPEED_MAGNIF					20	   		//CANopen��ص��ٶȱ��ʣ������õ��ٶ���Ҫ���Ըñ��ʺ�����ŷ����е�ת��
#define CANOPEN_ORIGIN_SPEED_MIN			25	   		//CANopen����ģʽ����Сת��
#define CANOPEN_MOVE_SPEED_MAX				5000	   	//CANopen���ת��
#define CANOPEN_LAST_MOVE_OVER_TIME		20	   			//�˶���ʱʱ�䣬��λ10ms

/*OD,�����ֵ�ṹ��*/
typedef struct
{
  u16 index;									//����
  u8  subindex;								//������
  u8  ctr;										//
  u16 length;									//���ݳ���
}DICT_OBJECT ;
/*PDO����֡�ṹ��*/
typedef struct
{
   u32 PDO_COB_ID;						//ID
   u8  len;										//���ݳ���
   u8  PDO_data[8];						//����
}PDO_Struct;

extern u8  ServoNodeID[SERVO_NODE_ID_NUM] ;												//�ڵ�ID��
extern CanRxMsg CAN_Recieve_Data;																	//CAN���յ����ݻ�����
extern u16 Controlword[SERVO_NODE_NUM];      											//״̬������
extern u8 Homing_Flag_Can;																				//�ŷ�����ģʽ����
extern u8 Servo_ConnectSta[SERVO_NODE_NUM];												//�ŷ�����״̬��0δ���� 1������

#define  SERVO_COMMUN_TIMEOUT					(20)												//�ŷ�ͨ�ų�ʱʱ�䣬200ms
extern u16 Servo_CommunTimeoutCounter[SERVO_NODE_NUM];						//�ŷ�ͨ�ų�ʱ������
extern u8 Servo_ConnectError[SERVO_NODE_NUM];
extern u8 Servo_InitFinish;																				//�ŷ���ʼ����ɱ�־

/*��������*/
void CANopen_Init(void);												//CANOpen��ʼ��
void SDO_ControlWordSet(u8 Node_ID);						//�����ֵ�SDO����
void RPDO_Mapping(u8);													//RPDOӳ��
void TPDO_Mapping(u8);													//TPDOӳ��
//void PP_Mode_Init(void);   											//�������ŷ�����Ϊλ��ģʽ
//void PP_Mode_AxisInit(u8 Node_ID);							//�������ŷ�����Ϊλ��ģʽ
//void SP_Mode_Init(void);												//�������ŷ�����Ϊ�ٶ�ģʽ
//void SP_Mode_AxisInit(u8 Node_ID);							//�������ŷ�����Ϊ�ٶ�ģʽ
void CSP_Mode_Init(void);												//�������ŷ�����Ϊ����ͬ��λ��ģʽ
void CSP_Mode_AxisInit(u8 Node_ID);							//�������ŷ�����Ϊ����ͬ��λ��ģʽ
//void CSV_Mode_Init(void);												//�������ŷ�����Ϊ����ͬ���ٶ�ģʽ
//void CSV_Mode_AxisInit(u8 Node_ID);							//�������ŷ�����Ϊ����ͬ���ٶ�ģʽ
void CST_Mode_Init(void);												//�������ŷ�����Ϊ����ͬ��ת��ģʽ
void CST_Mode_AxisInit(u8 Node_ID);							//�������ŷ�����Ϊ����ͬ��ת��ģʽ
void Homing_Mode_AxisInit(u8,u8, s32);					//�������ŷ�����Ϊ��ԭ��ģʽ
void Fault_Reset(u8);														//���ϸ�λ
//void Set_Servo_PP_Mode(u8);          						//�ŷ���λ��ģʽ����
//void Set_Servo_SP_Mode(u8);          						//�ŷ����ٶ�ģʽ����
void Set_Servo_Homing_Mode(u8, u8, s32);	 			//�ŷ���Homing Mode��ԭ��ģʽ����
void Set_Servo_CSP_Mode(u8 Node_ID);						//�ŷ�������ͬ��λ��ģʽ����
//void Set_Servo_CSV_Mode(u8 Node_ID);						//�ŷ�������ͬ���ٶ�ģʽ����
void Set_Servo_CST_Mode(u8 Node_ID);						//�ŷ�������ͬ��ת��ģʽ����
//void New_Set_Point_Reset(u8);										//��λ����λ��ָ��
//void New_Set_Point_Set(u8);											//��λ����λ��ָ��
//void ServoStart_PDO(u8 Node_ID, u32 Target_Position, u32 Target_Velocity);	//�����ŷ���λ��ģʽ�˶�
//void ServoStartSP_PDO(u8 Node_ID, u32 Target_Velocity, u8 DirFlag);					//�����ŷ����ٶ�ģʽ����
void ServoCSP_PDO(u8 Node_ID, s32 intPosition);
//void ServoCSV_PDO(u8 Node_ID, s32 intVol);
void ServoCST_PDO(u8 Node_ID, s16 intTorque);			//
void ServoAccDecSet_PDO(u8 Node_ID, u32 Acc, u32 Dec);											//�����ŷ������ٶȺͼ��ٶ�
//void ServoEmergencyEnable_PDO(u8 Node_ID);			//���ÿ����ֵ�ʹ��ָ��
void ServoEmergencyStop_PDO(u8 Node_ID);				//���ÿ����ֵĿ���ָֹͣ��
void ServoDisable_PDO(u8 Node_ID);							//���ÿ����ֵ�ʧ��ָ��
void ServoStopSet_PDO(u8 Node_ID);							//���ÿ����ֵ���ͣλ��ָ��
void ServoStopClear_PDO(u8 Node_ID);						//��������ֵ���ͣλ��ָ��
void Controlword_Set(u8 Node_ID, u8 Change_Set_Immediately);								//�����ֵ�λ��ָ����·�ʽ����
void ServoHomingControl(u8 Node_ID);						//�����ŷ���ԭ��
u16 ServoHomingFinishSta(u8 Node_ID);						//��ȡ��ԭ��״̬
u16 ServoMoveFinishSta(u8 Node_ID);							//��ȡ�ƶ�״̬
u16 ServoMoveSpeedZero(u8 Node_ID);							//��ȡ�ٶ�ģʽ��Ŀ���ٶ��Ƿ񵽴�״̬
u16 ServoMoveAlarmSta(u8 *alarm);								//�ŷ�����״̬��ȡ
u16 ServoPauseFinishSta(u8 Node_ID);						//��ȡ��ͣ���ٶ��Ƿ�Ϊ0
void ServoAlarmReset(void);											//��λ�ŷ�����
void ServoWorkModeReset(u8 Node_ID);						//��ʼ���ŷ�����ģʽ
u8 ServoGetCanBoffSta(u8 Node_Code);						//��ȡ�ŷ������Ƿ�BUSOFF
u16 ServoEnableSta(void);												//��ȡ�ŷ������Ƿ����ŷ�ʧ��
u16 ServoControlwordStaCheck(u8 Node_ID);				//���������Ƿ��쳣
void ServoStartStatusUpdateDelay(u8 Node_Code, u16 delayTime);
u16 ServoStatusUpdate(u8 Node_Code);
u8 ServoWorkModeRead(u8 Node_ID);
s32 ServoMoveCurSpeed(u8 Node_ID);

#endif




