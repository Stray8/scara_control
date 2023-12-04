#include "CANopen.h"
#include "Parameter.h"
#include "Error.h"
#include "EtherCAT_App.h"
#include "ethercatcoe.h"
#include "ethercattype.h"
#include "can.h"
#include "ActionOperate.h"

CanRxMsg CAN_Recieve_Data;											//CAN���յ����ݻ�����
u8  ServoNodeID[SERVO_NODE_ID_NUM] = {0};				//�ڵ�IDֵ�洢����
u16 Controlword[SERVO_NODE_NUM] = {							//�ŷ���״̬������
				DISABLE_VOLTAGE,DISABLE_VOLTAGE,
				DISABLE_VOLTAGE,DISABLE_VOLTAGE};

u8  Homing_Flag_Can	=	FALSE;     								//�ŷ�����ģʽ����

u8 Servo_ModeFlag[SERVO_NODE_NUM] = {SERVO_MODE_INVALID};				//�ŷ�ģʽ��0��Ч 1λ�� 2����1 3����2 4�ٶ�
u8 Servo_ConnectSta[SERVO_NODE_NUM] = {0};											//�ŷ�����״̬��0δ���� 1������
u8 Servo_ConnectError[SERVO_NODE_NUM] = {0};											//�ŷ������Ƿ�����������0δ������1������

u16 Servo_CommunTimeoutCounter[SERVO_NODE_NUM] = {0};						//�ŷ�ͨ�ų�ʱ������
u8 Servo_InitFinish = 0;																				//�ŷ���ʼ����ɱ�־

/***************************************************************************
**  ��������  CAN1_RX0_IRQHandler()
**	�����������
**	�����������
**	�������ܣ�CAN1�Ľ�������0�Ľ����ж�
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	
	Get_Status_Position(RxMessage);
}

/**************************************************************************************************
**  ��������ID_To_Axis
**	�����������ID���
**	���������
**	�������ܣ�����תΪCANOPEN����ID
**	��ע��	
**  ���ߣ�         
**  �������ڣ�
***************************************************************************************************/
u8 ID_To_Axis(u8 Node_ID)
{
	u8 Axis = 0;
	
	switch(Node_ID)
	{
		case SERVO_NODE_ID_01_X:
			Axis = X_Axsis;
			break;
		case SERVO_NODE_ID_02_L:
			Axis = L_Axsis;
			break;
		case SERVO_NODE_ID_03_Z:
			Axis = Z_Axsis;
			break;
		case SERVO_NODE_ID_04_O:
			Axis = O_Axsis;
			break;
		case SERVO_NODE_ID_05_U:
			Axis = U_Axsis;
			break;
		case SERVO_NODE_ID_06_V:
			Axis = V_Axsis;
			break;
		default:
			break;
	}
	
	return Axis;
}

///***************************************************************************
//**  ��������  PP_Mode_Init()
//**	�����������
//**	�����������
//**	�������ܣ������ŷ������λ��ģʽ��ʼ��
//**	��ע��
//**  ���ߣ�
//**  �������ڣ�
//***************************************************************************/
//void PP_Mode_Init()
//{
//	int i = 0;
//	
//	/*��ʼ�����ŷ���������operate״̬*/
//	for(i=1; i<SERVO_NODE_ID_NUM; i++)
//	{
//		if(ServoNodeID[i] && ServoGetCanBoffSta(i - 1))
//		{
//			SDO_Process_PositionMode(i);				//��ʼ���ŷ����ڵ��Լ�������ز���
//		}
//	}
//}

///***************************************************************************
//**  ��������  PP_Mode_AxisInit()
//**	�����������
//**	�����������
//**	�������ܣ������ŷ������λ��ģʽ��ʼ��
//**	��ע��	  
//**  ���ߣ�    
//**  �������ڣ�
//***************************************************************************/
//void PP_Mode_AxisInit(u8 Node_ID)
//{
//	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
//	{
//		return;
//	}
//	
//	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_POSITION)
//	{//����Ѿ���λ��ģʽ�ˣ�ֱ�ӷ���
//		return;
//	}
//	
//	/*��ʼ�����ŷ���������operate״̬*/
//	SDO_Process_PositionMode(Node_ID);					//��ʼ���ŷ����ڵ��Լ�������ز���
//}

///***************************************************************************
//**  ��������  SP_Mode_Init()
//**	�����������
//**	�����������
//**	�������ܣ������ŷ�������ٶ�ģʽ��ʼ��
//**	��ע��	  
//**  ���ߣ�    
//**  �������ڣ�
//***************************************************************************/
//void SP_Mode_Init()
//{
//	int i = 0;
//			
//	/*��ʼ�����ŷ���������operate״̬*/
//	for(i=1; i<SERVO_NODE_ID_NUM; i++)
//	{
//		if(ServoNodeID[i] && ServoGetCanBoffSta(i - 1))
//		{
//			SDO_Process_SpeedMode(i);					//��ʼ���ŷ����ڵ��Լ�������ز���
//		}
//	}
//}

///***************************************************************************
//**  ��������  SP_Mode_AxisInit()
//**	�����������
//**	�����������
//**	�������ܣ������ŷ�������ٶ�ģʽ��ʼ��
//**	��ע��	  
//**  ���ߣ�    
//**  �������ڣ�
//***************************************************************************/
//void SP_Mode_AxisInit(u8 Node_ID)
//{
//	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
//	{
//		return;
//	}
//	
//	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_SPEED)
//	{//����Ѿ���λ��ģʽ�ˣ�ֱ�ӷ���
//		return;
//	}
//	
//	/*��ʼ�����ŷ���������operate״̬*/
//	SDO_Process_SpeedMode(Node_ID);							//��ʼ���ŷ����ڵ��Լ�������ز���
//}

/***************************************************************************
**  ��������  Homing_Mode_AxisInit()
**	���������Node_ID ��Ҫ��������Ӧ��ID
**	���������flag 0����ԭ���ź� 1��ǰλ��Ϊԭ��
**	�����������
**	�������ܣ������ŷ�����Ļ�ԭ��ģʽ��ʼ��
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void Homing_Mode_AxisInit(u8 Node_ID, u8 flag, s32 oriOffset)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_BACKZERO1 && flag == 0)
	{//����Ѿ��ǻ���ģʽ1�ˣ�ֱ�ӷ���
		return;
	}
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_BACKZERO2 && flag == 1)
	{//����Ѿ��ǻ���ģʽ2�ˣ�ֱ�ӷ���
		return;
	}
		
	/*��ʼ�����ŷ����������ԭ��״̬*/
	SDO_Process_HomingMode(Node_ID, flag, oriOffset);				 //��ʼ���ŷ����ڵ��Լ�������ز���
}
/***************************************************************************
**  ��������  CSP_Mode_Init()
**	�����������
**	�����������
**	�������ܣ������ŷ����������ͬ��λ��ģʽ��ʼ��
**	��ע��
**  ���ߣ�
**  �������ڣ�
***************************************************************************/
void CSP_Mode_Init()
{
	int i = 0;
	
	/*��ʼ�����ŷ���������operate״̬*/
	for(i=1; i<SERVO_NODE_ID_NUM; i++)
	{
		if(ServoNodeID[i] && ServoGetCanBoffSta(i - 1))
		{
			SDO_Process_CycPosMode(i);				//��ʼ���ŷ����ڵ��Լ�������ز���
		}
	}
}

/***************************************************************************
**  ��������  CSP_Mode_AxisInit()
**	�����������
**	�����������
**	�������ܣ������ŷ����������ͬ��λ��ģʽ��ʼ��
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void CSP_Mode_AxisInit(u8 Node_ID)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_CYC_CSP)
	{//����Ѿ�������ͬ��λ��ģʽ�ˣ�ֱ�ӷ���
		return;
	}
	
	/*��ʼ�����ŷ���������operate״̬*/
	SDO_Process_CycPosMode(Node_ID);					//��ʼ���ŷ����ڵ��Լ�������ز���
}
///***************************************************************************
//**  ��������  CSV_Mode_Init()
//**	�����������
//**	�����������
//**	�������ܣ������ŷ����������ͬ���ٶ�ģʽ��ʼ��
//**	��ע��
//**  ���ߣ�
//**  �������ڣ�
//***************************************************************************/
//void CSV_Mode_Init()
//{
//	int i = 0;
//	
//	/*��ʼ�����ŷ���������operate״̬*/
//	for(i=1; i<SERVO_NODE_ID_NUM; i++)
//	{
//		if(ServoNodeID[i] && ServoGetCanBoffSta(i - 1))
//		{
//			SDO_Process_CycCSVMode(i);				//��ʼ���ŷ����ڵ��Լ�������ز���
//		}
//	}
//}

///***************************************************************************
//**  ��������  CSV_Mode_AxisInit()
//**	�����������
//**	�����������
//**	�������ܣ������ŷ����������ͬ���ٶ�ģʽ��ʼ��
//**	��ע��	  
//**  ���ߣ�    
//**  �������ڣ�
//***************************************************************************/
//void CSV_Mode_AxisInit(u8 Node_ID)
//{
//	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
//	{
//		return;
//	}
//	
//	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_CYC_CSV)
//	{//����Ѿ�������ͬ��λ��ģʽ�ˣ�ֱ�ӷ���
//		return;
//	}
//	
//	/*��ʼ�����ŷ���������operate״̬*/
//	SDO_Process_CycCSVMode(Node_ID);					//��ʼ���ŷ����ڵ��Լ�������ز���
//}

/***************************************************************************
**  ��������  CST_Mode_Init()
**	�����������
**	�����������
**	�������ܣ������ŷ����������ͬ��ת��ģʽ��ʼ��
**	��ע��
**  ���ߣ�
**  �������ڣ�
***************************************************************************/
void CST_Mode_Init()
{
	int i = 0;
	
	/*��ʼ�����ŷ���������operate״̬*/
	for(i=1; i<SERVO_NODE_ID_NUM; i++)
	{
		if(ServoNodeID[i] && ServoGetCanBoffSta(i - 1))
		{
			SDO_Process_CycCSTMode(i);				//��ʼ���ŷ����ڵ��Լ�������ز���
		}
	}
}

/***************************************************************************
**  ��������  CST_Mode_AxisInit()
**	�����������
**	�����������
**	�������ܣ������ŷ����������ͬ��ת��ģʽ��ʼ��
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void CST_Mode_AxisInit(u8 Node_ID)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_CYC_CST)
	{//����Ѿ�������ͬ��ת��ģʽ�ˣ�ֱ�ӷ���
		return;
	}
	
	/*��ʼ�����ŷ���������operate״̬*/
	SDO_Process_CycCSTMode(Node_ID);					//��ʼ���ŷ����ڵ��Լ�������ز���
}

/***************************************************************************
**  ��������  PDO_Mode_Change()
**	���������Node_ID ��Ҫ��������Ӧ��ID
**	���������Mode ����ģʽ
**	�����������
**	�������ܣ������ŷ������ģʽ����
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
u16 PDO_Mode_Change(u8 Node_ID, u8 Mode)
{
	u16 i = 0;
	
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return 1;
	}
	
	PDO_TargetInf[ServoNodeID[Node_ID]].TargetMode = Mode;					//λ��ģʽ
	EtherCAT_SendPDOFinish();
	
	while(i < 5)
	{
		i++;
		
		if(PDO_Cur_Mode[Node_ID - 1] == Mode)
		{
			return 0;
		}
		
		delay_ms(CANOPEN_DEAL_PDO_DELAY);
	}
	
	return 1;
}

///***************************************************************************
//**  ��������  Set_Servo_PP_Mode()
//**	�����������
//**	�����������
//**	�������ܣ������ŷ���λ��ģʽ��������λ�ã��ٶȣ��Ӽ��ٳ�ʼֵ
//**	��ע��	  ��
//**  ���ߣ�       
//**  �������ڣ�
//***************************************************************************/
//void Set_Servo_PP_Mode(u8 Node_ID)
//{
//	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_POSITION)
//	{//����Ѿ���λ��ģʽ�ˣ�ֱ�ӷ���
//		return;
//	}
//	
//	Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_POSITION;
//	ServoAccDecSet(ID_To_Axis(Node_ID));
//	
//	PDO_TargetInf[ServoNodeID[Node_ID]].TargetPos = 0;
//	EtherCAT_SendPDOFinish();
//	delay_ms(CANOPEN_DEAL_PDO_DELAY);
//	
//	PDO_Mode_Change(Node_ID, 0x01);														//λ��ģʽ
//	
//	if((Controlword[Node_ID - 1] & 0x000f) != 0x0002 && \
//			(Controlword[Node_ID - 1] & 0x000f) != 0x000F)
//	{//�����ǰ״̬��Ϊ����ֹͣ����ô��Ҫ���л���0x0006�����л���0x0007
//		Controlword[Node_ID - 1] = 0x06;
//		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//		
//		Controlword[Node_ID - 1] = 0x07;
//		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//	}
//	
//	Controlword[Node_ID - 1] = 0x000f | 0x0100;						//λ��ģʽʱ����ͣ��־��1����ʹ��ʱ���봦����ͣ
//	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//}

///***************************************************************************
//**  ��������  Set_Servo_SP_Mode()
//**	�����������
//**	�����������
//**	�������ܣ������ŷ����ٶ�ģʽ���ٶȣ��Ӽ��ٳ�ʼֵ
//**	��ע��	  ��
//**  ���ߣ�       
//**  �������ڣ�
//***************************************************************************/
//void Set_Servo_SP_Mode(u8 Node_ID)
//{
//	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_SPEED)
//	{//����Ѿ����ٶ�ģʽ�ˣ�ֱ�ӷ���
//		return;
//	}
	
//	Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_SPEED;
//	ServoAccDecSet(ID_To_Axis(Node_ID));
//	
//	PDO_TargetInf[ServoNodeID[Node_ID]].TarVelocity = 0;
//	EtherCAT_SendPDOFinish();
//	delay_ms(CANOPEN_DEAL_PDO_DELAY);
//	
//	PDO_Mode_Change(Node_ID, 0x03);																		//�ٶ�ģʽ
//  
//	if((Controlword[Node_ID - 1] & 0x000f) != 0x0002 && \
//			(Controlword[Node_ID - 1] & 0x000f) != 0x000F)
//	{//�����ǰ״̬��Ϊ����ֹͣ����ô��Ҫ���л���0x0006�����л���0x0007
//		Controlword[Node_ID - 1] = 0x06;
//		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//		
//		Controlword[Node_ID - 1] = 0x07;
//		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//	}

//	Controlword[Node_ID - 1] = 0x010f;
//	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//}

/***************************************************************************
**  ��������  Set_Servo_Homing_Mode()
**	���������flag 0����ԭ���ź� 1��ǰλ��Ϊԭ��
**	�����������
**	�������ܣ������ŷ�����ԭ��ģʽ
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void Set_Servo_Homing_Mode(u8 Node_ID, u8 flag, s32 oriOffset)
{
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_BACKZERO1 && flag == 0)
	{//����Ѿ��ǻ���ģʽ1�ˣ�ֱ�ӷ���
		return;
	}
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_BACKZERO2 && flag == 1)
	{//����Ѿ��ǻ���ģʽ2�ˣ�ֱ�ӷ���
		return;
	}
	
	PDO_Mode_Change(Node_ID, 0x06);																			//����ģʽ
	ServoAccDecSet(ID_To_Axis(Node_ID));
	
	//PDO_TargetInf[ServoNodeID[Node_ID]].OriginOffset = oriOffset;				//ԭ��ƫ��
	
	/*��ԭ��ģʽ��35�Ե�ǰλ��Ϊԭ�㣬27����ԭ��λ���źź��˳�Ϊԭ��*/
	if(flag == 0)
	{
		PDO_TargetInf[ServoNodeID[Node_ID]].OriginMode = 27;
		Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_BACKZERO1;
	}
	else
	{
		PDO_TargetInf[ServoNodeID[Node_ID]].OriginMode = 35;
		Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_BACKZERO2;
	}
	EtherCAT_SendPDOFinish();
	delay_ms(CANOPEN_DEAL_PDO_DELAY);
	
	if((Controlword[Node_ID - 1] & 0x000f) != 0x0002 && \
			(Controlword[Node_ID - 1] & 0x000f) != 0x000F)
	{//�����ǰ״̬��Ϊ����ֹͣ����ô��Ҫ���л���0x0006�����л���0x0007
		Controlword[Node_ID - 1] = 0x06;
		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
		
		Controlword[Node_ID - 1] = 0x07;
		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
	}
	
	Controlword[Node_ID - 1] = 0x000f;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
}

/***************************************************************************
**  ��������  Set_Servo_CSP_Mode()
**	�����������
**	�����������
**	�������ܣ������ŷ�������ͬ��λ��ģʽ��������λ�ã��ٶȣ��Ӽ��ٳ�ʼֵ
**	��ע��	  ��
**  ���ߣ�       
**  �������ڣ�
***************************************************************************/
void Set_Servo_CSP_Mode(u8 Node_ID)
{
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_CYC_CSP)
	{//����Ѿ���λ��ģʽ�ˣ�ֱ�ӷ���
		return;
	}
	
	Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_CYC_CSP;
	ServoAccDecSet(ID_To_Axis(Node_ID));
	
	PDO_TargetInf[ServoNodeID[Node_ID]].TargetPos = EtherCAT_GetCurrentPosition(Node_ID);		//	ȷ����ǰλ����Ŀ��λ��һֱ
	EtherCAT_SendPDOFinish();
	
	PDO_Mode_Change(Node_ID, SERVO_MODE_CYC_CSP);														//����ͬ��λ��ģʽ
	delay_ms(CANOPEN_DEAL_PDO_DELAY);
	
	if((Controlword[Node_ID - 1] & 0x000f) != 0x0002 && \
			(Controlword[Node_ID - 1] & 0x000f) != 0x000F)
	{//�����ǰ״̬��Ϊ����ֹͣ����ô��Ҫ���л���0x0006�����л���0x0007
		Controlword[Node_ID - 1] = 0x06;
		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
		
		Controlword[Node_ID - 1] = 0x07;
		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
	}
	
	Controlword[Node_ID - 1] = 0x000f;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
}
/***************************************************************************
**  ��������  Set_Servo_CSV_Mode()
**	�����������
**	�����������
**	�������ܣ������ŷ�������ͬ���ٶ�ģʽ��������λ�ã��ٶȣ��Ӽ��ٳ�ʼֵ
**	��ע��	  ��
**  ���ߣ�       
**  �������ڣ�
***************************************************************************/
//void Set_Servo_CSV_Mode(u8 Node_ID)
//{
//	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_CYC_CSV)
//	{//����Ѿ���λ��ģʽ�ˣ�ֱ�ӷ���
//		return;
//	}
	
//	Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_CYC_CSV;
//	ServoAccDecSet_CSV(ID_To_Axis(Node_ID));
//	
//	PDO_TargetInf[ServoNodeID[Node_ID]].TarVelocity = 0;
//	EtherCAT_SendPDOFinish();
//	
//	PDO_Mode_Change(Node_ID, SERVO_MODE_CYC_CSV);														//����ͬ��λ��ģʽ
//	delay_ms(CANOPEN_DEAL_PDO_DELAY);
//	
//	if((Controlword[Node_ID - 1] & 0x000f) != 0x0002 && \
//			(Controlword[Node_ID - 1] & 0x000f) != 0x000F)
//	{//�����ǰ״̬��Ϊ����ֹͣ����ô��Ҫ���л���0x0006�����л���0x0007
//		Controlword[Node_ID - 1] = 0x06;
//		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//		
//		Controlword[Node_ID - 1] = 0x07;
//		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//	}
//	
//	Controlword[Node_ID - 1] = 0x000f;
//	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//}

/***************************************************************************
**  ��������  Set_Servo_CST_Mode()
**	�����������
**	�����������
**	�������ܣ������ŷ�������ͬ��ת��ģʽ��������λ�ã��ٶȣ��Ӽ��ٳ�ʼֵ
**	��ע��	  ��
**  ���ߣ�       
**  �������ڣ�
***************************************************************************/
void Set_Servo_CST_Mode(u8 Node_ID)
{
	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_CYC_CST)
	{//����Ѿ���ת��ģʽ�ˣ�ֱ�ӷ���
		return;
	}
	
	Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_CYC_CST;
	ServoAccDecSet_CST(ID_To_Axis(Node_ID));
	
	PDO_TargetInf[ServoNodeID[Node_ID]].TarTorque = 0;		//	ȷ����ǰλ����Ŀ��λ��һֱ
	EtherCAT_SendPDOFinish();
	
	PDO_Mode_Change(Node_ID, SERVO_MODE_CYC_CST);														//����ͬ��λ��ģʽ
	delay_ms(CANOPEN_DEAL_PDO_DELAY);
	
	if((Controlword[Node_ID - 1] & 0x000f) != 0x0002 && \
			(Controlword[Node_ID - 1] & 0x000f) != 0x000F)
	{//�����ǰ״̬��Ϊ����ֹͣ����ô��Ҫ���л���0x0006�����л���0x0007
		Controlword[Node_ID - 1] = 0x06;
		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
		
		Controlword[Node_ID - 1] = 0x07;
		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
	}
	
	Controlword[Node_ID - 1] = 0x000f;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
}

/***************************************************************************
**  ��������  Fault_Reset()
**	�����������
**	�����������
**	�������ܣ���λ����
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void Fault_Reset(u8 Node_ID)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	Controlword[Node_ID - 1] = Controlword[Node_ID - 1] | 0x0080;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
}

///***************************************************************************
//**  ��������  New_Set_Point_Reset()
//**	�����������
//**	�����������
//**	�������ܣ���λ�����ֵĸ���λ��ָ��
//**	��ע��	  ��
//**  ���ߣ�    
//**  �������ڣ�
//***************************************************************************/
//void New_Set_Point_Reset(u8 Node_ID)
//{
//	if((Controlword[Node_ID - 1] & QUICK_STOP) == QUICK_STOP)
//	{
//		Controlword[Node_ID - 1] |= 0x000f;
//	}
//	
//	Controlword[Node_ID - 1] = Controlword[Node_ID - 1] & 0xffef;
//	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//	
//	while(PDO_Actual_Status[Node_ID - 1] & 0x1000);
//}

///***************************************************************************
//**  ��������  New_Set_Point_Set()
//**	�����������
//**	�����������
//**	�������ܣ���λ�����ֵĸ���λ��ָ��
//**	��ע��	  ��
//**  ���ߣ�    
//**  �������ڣ�
//***************************************************************************/
//void New_Set_Point_Set(u8 Node_ID)
//{
//	Controlword[Node_ID - 1] = Controlword[Node_ID - 1] & 0xfeff;
//	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//	
//	Controlword[Node_ID - 1] = Controlword[Node_ID - 1] | 0x0030;
//	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//}

///***************************************************************************
//**  ��������  ServoStart_PDO()
//**	���������Target_Position Ŀ��λ��
//**	���������Target_Velocity �ٶ�
//**	�������ܣ�����PDO��Ϣ�������ŷ���λ�á��ٶ�
//**	��ע��	  ��
//**  ���ߣ�    
//**  �������ڣ�
//***************************************************************************/
//void ServoStart_PDO(u8 Node_ID, u32 Target_Position, u32 Target_Velocity)
//{
//	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
//	{
//		return;
//	}
//	
//	New_Set_Point_Reset(Node_ID);  							//��λ�����ֵ�λ��ָ�����
//	delay_ms(CANOPEN_DEAL_PDO_DELAY);
//	PDO_ServoPositionSet(Node_ID, Target_Position, Target_Velocity);//�����ŷ��ƶ���λ�ú��ٶ�
//	delay_ms(CANOPEN_DEAL_PDO_DELAY);
//	New_Set_Point_Set(Node_ID);    							//���ÿ����ֵ�λ��ָ�����
//}

///***************************************************************************
//**  ��������  ServoStartSP_PDO()
//**	���������Target_Velocity Ŀ���ٶ�
//**	���������DirFlag ���� 1��ת 0��ת
//**	���������
//**	�������ܣ�����PDO��Ϣ�������ŷ����ٶ�����������
//**	��ע��	  ��
//**  ���ߣ�    
//**  �������ڣ�
//***************************************************************************/
//void ServoStartSP_PDO(u8 Node_ID, u32 Target_Velocity, u8 DirFlag)
//{
//	s32 Vel = Target_Velocity;
//	
//	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
//	{
//		return;
//	}
		
//	if(DirFlag == 0)
//	{
//		Vel = -Vel;
//	}

//	PDO_TargetInf[ServoNodeID[Node_ID]].TarVelocity = Vel;
//	EtherCAT_SendPDOFinish();
//	delay_ms(CANOPEN_DEAL_PDO_DELAY);
//	
//	Controlword[Node_ID - 1] = 0x000f;
//	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
//}

/***************************************************************************
**  ��������  ServoCSP__PDO()
**	���������Target_Position Ŀ��λ��
**	�������ܣ�����PDO��Ϣ�������ŷ�������ͬ��λ��ģʽ��λ��
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void ServoCSP_PDO(u8 Node_ID, s32 intPosition)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	if((Controlword[Node_ID - 1] & QUICK_STOP) == QUICK_STOP)
	{
		Controlword[Node_ID - 1] |= 0x000f;
	}
	
	PDO_TargetInf[ServoNodeID[Node_ID]].TargetPos = intPosition;
	E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_EN;
}

///***************************************************************************
//**  ��������  ServoCSP__PDO()
//**	���������intVol Ŀ���ٶ�
//**	�������ܣ�����PDO��Ϣ�������ŷ�������ͬ��λ��ģʽ��λ��
//**	��ע��	  ��
//**  ���ߣ�    
//**  �������ڣ�
//***************************************************************************/
//void ServoCSV_PDO(u8 Node_ID, s32 intVol)
//{
//	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
//	{
//		return;
//	}
	
//	if((Controlword[Node_ID - 1] & QUICK_STOP) == QUICK_STOP)
//	{
//		Controlword[Node_ID - 1] |= 0x000f;
//		PDO_TargetInf[ServoNodeID[Node_ID]].ControlWord = Controlword[Node_ID - 1];
//	}
//	if((Controlword[Node_ID - 1] & 0x0100) == 0x0100)
//	{
//		Controlword[Node_ID - 1] &= 0xfeff;
//		PDO_TargetInf[ServoNodeID[Node_ID]].ControlWord = Controlword[Node_ID - 1];
//	}
//	
//	PDO_TargetInf[ServoNodeID[Node_ID]].TarVelocity = intVol;
//	E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_EN;
//}

/***************************************************************************
**  ��������  ServoAccDecSet_PDO()
**	�����������
**	�����������
**	�������ܣ�����PDO��Ϣ�������ŷ������������ٶȡ��������ٶȡ����ټ��ٶ�
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void ServoAccDecSet_PDO(u8 Node_ID, u32 Acc, u32 Dec)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	PDO_ServoAccDecSet(Node_ID, Acc, Dec);		//�����ŷ��ƶ����������ٶȺ��������ٶ�
}


/***************************************************************************
**  ��������  ServoCST__PDO()
**	���������intTorque Ŀ��ת��
**	�������ܣ�����PDO��Ϣ�������ŷ�������ͬ��ת��ģʽ��ת��
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void ServoCST_PDO(u8 Node_ID, s16 intTorque)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	if((Controlword[Node_ID - 1] & QUICK_STOP) == QUICK_STOP)
	{
		Controlword[Node_ID - 1] |= 0x000f;
	}
	
	PDO_TargetInf[ServoNodeID[Node_ID]].TarTorque = intTorque;
	E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_EN;
}

/***************************************************************************
**  ��������  ServoStopSet_PDO()
**	�����������
**	�����������
**	�������ܣ���λ�����ֵ���ͣλ��ָ��
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void ServoStopSet_PDO(u8 Node_ID)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
//	if(Servo_ModeFlag[Node_ID - 1] == SERVO_MODE_SPEED)
//	{//����Ѿ���λ��ģʽ�ˣ�ֱ�ӷ���
//		PDO_TargetInf[ServoNodeID[Node_ID]].TarVelocity = 0;
//		EtherCAT_SendPDOFinish();
//		delay_ms(CANOPEN_DEAL_PDO_DELAY);
//	}
	
	if(Servo_ModeFlag[Node_ID - 1] != SERVO_MODE_CYC_CSP && Servo_ModeFlag[Node_ID - 1] != SERVO_MODE_CYC_CSV)
	{
		Controlword[Node_ID - 1] = Controlword[Node_ID - 1] | 0x0100;
		PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
	}
}

/***************************************************************************
**  ��������  ServoDisable_PDO()
**	�����������
**	�����������
**	�������ܣ��ŷ�ʧ��
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void ServoDisable_PDO(u8 Node_ID)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	Controlword[Node_ID - 1] = DISABLE_VOLTAGE;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
	
	ServoWorkModeReset(Node_ID);						//�����������Ҫ���ڲ�ģʽ���
}

/***************************************************************************
**  ��������  ServoEmergencyStop_PDO()
**	�����������
**	�����������
**	�������ܣ��ŷ�����ֹͣ
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void ServoEmergencyStop_PDO(u8 Node_ID)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	Controlword[Node_ID - 1] = (Controlword[Node_ID - 1] & 0Xfff0) | QUICK_STOP;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
}

/***************************************************************************
**  ��������  ServoHomingControl()
**	�����������
**	�����������
**	�������ܣ������ŷ���ԭ��
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void ServoHomingControl(u8 Node_ID)
{
	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
	{
		return;
	}
	
	Controlword[Node_ID - 1] = Controlword[Node_ID - 1] & 0xffef;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
	
	Controlword[Node_ID - 1] = Controlword[Node_ID - 1] & 0xfeff;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
	
	Controlword[Node_ID - 1] = Controlword[Node_ID - 1] | 0x0010;
	PDO_ControlWordSet(Node_ID, Controlword[Node_ID - 1]);
}

/***************************************************************************
**  ��������  ServoHomingFinishSta()
**	�����������
**	�����������
**	�������ܣ���ȡ����״̬��bit12Ϊ1ʱ��ɻ���
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
u16 ServoHomingFinishSta(u8 Node_ID)
{
	return ((PDO_Actual_Status[Node_ID - 1] >> 12) & 0x0001);
}

/***************************************************************************
**  ��������  ServoMoveFinishSta()
**	�����������
**	�����������
**	�������ܣ���ȡ�ƶ�״̬
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
u16 ServoMoveFinishSta(u8 Node_ID)
{
	return ((PDO_Actual_Status[Node_ID - 1] >> 10) & 0x0001);
}

/***************************************************************************
**  ��������  ServoMoveSpeedZero()
**	�����������
**	�����������
**	�������ܣ���ȡ�ٶ�ģʽ��Ŀ���ٶ��Ƿ񵽴�״̬
**	��ע��	  ����ֵ 1Ϊ�� 0��Ϊ0
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
u16 ServoMoveSpeedZero(u8 Node_ID)
{
	return ((PDO_Actual_Status[Node_ID - 1] >> 10) & 0x0001);
}

/***************************************************************************
**  ��������  ServoMoveAlarmSta()
**	������������ڱ���ÿ������ı������
**	�����������
**	�������ܣ���ȡ����״̬
**	���أ�    ֻҪ�б����ͷ���1�����򷵻�0
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
u16 ServoMoveAlarmSta(u8 *alarm)
{
	u16 i = 0;
	u8 ret = 0;
	
	for(i=0; i<SERVO_NODE_NUM; i++)
	{
		alarm[i] = (u8)((PDO_Actual_Status[i] >> 3) & 0x0001);
		ret |= alarm[i];
	}
	
	return ret;
}

/***************************************************************************
**  ��������  ServoPauseFinishSta()
**	�����������
**	�����������
**	�������ܣ���ȡ��ͣ���ٶ��Ƿ�Ϊ0
**  �������أ�0δ��ͣ 1��ͣ��� 2����ͣ���ڼ�����
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
u16 ServoPauseFinishSta(u8 Node_ID)
{
	if(Controlword[Node_ID - 1] & 0x0100)
	{
		if((PDO_Actual_Status[Node_ID - 1] >> 10) & 0x0001)
		{
			return 1;
		}
		else
		{
			return 2;
		}
	}
	else
	{
		return 0;
	}
}

/***************************************************************************
**  ��������  ServoAlarmReset()
**	�����������
**	�����������
**	�������ܣ�������λ����λ����λ0->1ʱ��λ����
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void ServoAlarmReset(void)
{
	u16 i = 0;
	
	for(i=0; i<SERVO_NODE_NUM; i++)
	{
		if(((PDO_Actual_Status[i] >> 3) & 0x0001) && ServoGetCanBoffSta(i))
		{
			Controlword[i] = Controlword[i] | 0x0100;
			Controlword[i] = Controlword[i] & 0xff7f;
			PDO_ControlWordSet(i + 1, Controlword[i]);
			
			Controlword[i] = Controlword[i] | 0x0080;
			PDO_ControlWordSet(i + 1, Controlword[i]);
			
			Controlword[i] = Controlword[i] & 0xff7f;
			PDO_ControlWordSet(i + 1, Controlword[i]);
			
			Servo_ModeFlag[i] = SERVO_MODE_INVALID;
		}
	}
}

/***************************************************************************
**  ��������  ServoWorkModeReset()
**	�����������
**	�����������
**	�������ܣ���ʼ���ŷ�����ģʽ
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void ServoWorkModeReset(u8 Node_ID)
{
	Servo_ModeFlag[Node_ID - 1] = SERVO_MODE_INVALID;
}

/***************************************************************************
**  ��������  ServoGetCanBoffSta()
**	�����������
**	�����������
**	�������ܣ���ȡ�ŷ������Ƿ�BUSOFF
**	��ע��	  ����ֵ 0����δ����  1����������
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
u8 ServoGetCanBoffSta(u8 Node_Code)
{	
	return Servo_ConnectSta[Node_Code];
}

/***************************************************************************
**  ��������  ServoEnableSta()
**	���������
**	�����������
**	�������ܣ����ڱ���ÿ��������Ƿ�����
**	���أ�    �����ŷ������ܷ���0�����򷵻���ӦID
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
u16 ServoEnableSta(void)
{
	u16 i = 0;
	
	for(i=0; i<SERVO_NODE_NUM; i++)
	{
		if(ServoGetCanBoffSta(i) == 0)
		{
			continue;
		}
		
		if(((PDO_Actual_Status[i] >> 2) & 0x0001) == 0)
		{
			return (i + 1);
		}
	}
	
	return 0;
}

/***************************************************************************
**  ��������  ServoControlwordStaChaeck()
**	���������
**	�����������
**	�������ܣ�������״̬��0���� 1�쳣
**	���أ�    
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
u16 ServoControlwordStaCheck(u8 Node_ID)
{
//	if(ServoGetCanBoffSta(Node_ID - 1) == 0)
//	{
//		return 0;
//	}
//	
//	if(PDO_Cur_Controlword[Node_ID - 1] != Controlword[Node_ID - 1])
//	{
//		return 1;
//	}
	
	return 0;
}

/***************************************************************************
**  ��������  ServoStartStatusUpdateDelay()
**	���������Node_Code ����
**	�����������
**	�������ܣ�����״̬������ʱ
**	���أ�    0��ʾ�Ѹ��£���Ϊ���ʾδ����
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
void ServoStartStatusUpdateDelay(u8 Node_Code, u16 delayTime)
{
	PDO_Actual_StatusUpdateTime[Node_Code] = delayTime;
	PDO_Actual_StatusUpdate[Node_Code] = 1;
}

/***************************************************************************
**  ��������  ServoStatusUpdate()
**	���������Node_Code ����
**	�����������
**	�������ܣ���ȡ��״̬���±�־
**	���أ�    0��ʾ�Ѹ��£���Ϊ���ʾδ����
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
u16 ServoStatusUpdate(u8 Node_Code)
{
	return PDO_Actual_StatusUpdate[Node_Code];
}

/***************************************************************************
**  ��������  ServoWorkModeRead()
**	�����������
**	�����������
**	�������ܣ���ȡ�ŷ�����ģʽ
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
u8 ServoWorkModeRead(u8 Node_ID)
{
	return Servo_ModeFlag[Node_ID - 1];
}

/***************************************************************************
**  ��������  ServoMoveCurSpeed()
**	�����������
**	�����������
**	�������ܣ���ȡ�ŷ���ǰ�ٶ�
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************/
s32 ServoMoveCurSpeed(u8 Node_ID)
{
	return PDO_Cur_Speed[Node_ID - 1];
}





