/**
  ******************************************************************************
  * @file    EtherCAT.App.c
  * @author  
  * @version V1.1.0
  * @date    
  * @brief   ʵ��EtherCATӦ�ù���
  */

#include "EtherCAT_App.h"
#include "Parameter.h"
#include "ActionOperate.h"
#include "Error.h"
#include "SpeedControl.h" 
#include "JDZ.h" 

#include <stdio.h>
#include "osal.h"

#include "LAN8742A.h"
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

extern __IO uint8_t EthLinkStatus;													//��־��������״̬

#define SYNC0TIME 2000																			//�������ݷ�������: 2000 * 1us = 2ms

char PDO_IOmap[1024] = {0};																	//PDO�������ݻ�����
u8 EtherCatConnectFlag = 0;																	//EtherCat����״̬��־��1�ɹ���2�������ӣ�0ʧ��
PDO_Output *PDO_OutPuts[SERVO_NODE_ID_NUM] = {NULL};				//ָ��ӻ�ģʽ��TPDO���ݵ�ַ
PDO_Input *PDO_InPuts[SERVO_NODE_ID_NUM] = {NULL};					//ָ��ӻ�ģʽ��RPDO���ݵ�ַ
PDO_Output PDO_TargetInf[SERVO_NODE_ID_NUM] = {0};					//���TPDOģʽ��TPDO����
u8 E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_DIS;							//����TPDO���ݸ��±�־
u8 EtherCatPHY_LinkSta = 1;																	//�������������״̬��־

u8 E_LastDCtimeFlag = 0;
int64_t E_LastDCtime = 0;
int64_t E_CurDCtime = 0;
int64_t E_ErrDCtime = 0;

/**
  * @brief  ͨ�ö�ʱ��3�жϳ�ʼ�������ڶ�ʱ���ͺͽ���PDO����֡����
  * @retval ��
  * @note   ��ʱ��3��ʱ��ԴΪ84M
  */
void TIM3_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  							//ʹ��TIM3ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;										//��ʱ����Ƶ��usΪ��λ
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; 		//���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_Period = SYNC0TIME - 1;							//�Զ���װ��ֵ��usΪ��λ
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 											//��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1; 					//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; 								//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);

	TIM_SelectOnePulseMode(TIM3, TIM_OPMode_Repetitive);									//���õ�����ģʽ�����ھ�����վ�ʹ�վ֮���ʱ�����

	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); 															//����ʱ��3�����ж�
	TIM_Cmd(TIM3,ENABLE); 																								//ʹ�ܶ�ʱ��3
}

/*PDO����֡���ݽṹ�б��ʼ������*/
int PDO_ServoSetup(u16 slave)
{
	int retval = 0;
	u16 u16val = 0;
	u8  u8val = 0;
	u32 u32val = 0;
	
	/*TPDO���ݽṹ*/
	u8val = 0;
	retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	u16val = 0x1600;
	retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
	u8val = 1;
	retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

	u8val = 0;
	retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	u32val = 0x607A0020;	//TargetPos��Ŀ��λ��
	retval += ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x60810020;	//ContourVel�������ٶ�
//	retval += ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x60FF0020;	//TarVelocity��Ŀ���ٶ�
//	retval += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x607C0020;	//OriginOffset��ԭ��ƫ��
//	retval += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x60830020;	//Acc�����ٶ�
//	retval += ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x60840020;	//Dec�����ٶ�
//	retval += ec_SDOwrite(slave, 0x1600, 0x06, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x60850020;	//QukDec������ֹͣ���ٶ�
//	retval += ec_SDOwrite(slave, 0x1600, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60400010;	//ControlWord��������
	retval += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60980008;	//OriginMode����ԭ��ģʽ
	retval += ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60600008;	//TargetMode������ģʽ
	retval += ec_SDOwrite(slave, 0x1600, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60710010;	//TarTorque��Ŀ��ת��  
	retval += ec_SDOwrite(slave, 0x1600, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//		u32val = 0x00000008;
//    retval += ec_SDOwrite(slave, 0x1600, 0x0E, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u8val = 0x05;
	retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	
	/*RPDO���ݽṹ*/
	u8val = 0;
	retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	u16val = 0x1a00;
	retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
	u8val = 1;
	retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	
	u8val = 0;
	retval += ec_SDOwrite(slave, 0x1a00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	u32val = 0x60410010;	//StatusWord��״̬��
	retval += ec_SDOwrite(slave, 0x1a00, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x60400010;	//ControlWord��������
//	retval += ec_SDOwrite(slave, 0x1a00, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60640020;	//CurrentPosition���û�λ�÷���
	retval += ec_SDOwrite(slave, 0x1a00, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x606C0020;	//CurrentVelocity���û�ʵ���ٶȷ���
	retval += ec_SDOwrite(slave, 0x1a00, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60770010;	//ActualTprque��ʵ��Ť��
	retval += ec_SDOwrite(slave, 0x1a00, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u32val = 0x60610008;	//CurrentMode����ǰģʽ
	retval += ec_SDOwrite(slave, 0x1a00, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//	u32val = 0x60980008;	//OriginMode������ģʽ
//	retval += ec_SDOwrite(slave, 0x1a00, 0x07, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
//		u32val = 0x00000008;
//    retval += ec_SDOwrite(slave, 0x1a00, 0x08, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
	u8val = 5;
	retval += ec_SDOwrite(slave, 0x1a00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
	
	return 1;
}

/*PDO����֡�����շ�������*/
void EtherCAT_RevSendDeal(void)
{
	u16 i = 0;
	u16 j = 0;
	static u8 startRevFlag = 0;																					//��������֡��ȡ��������־
	static PDO_Output targetInf[SERVO_NODE_ID_NUM] = {0};								//���淢�����ݣ�ȷ��ÿ�ε�TPDO����һ��
	
	if(startRevFlag > 2)
	{//�����Ƚ�������֡���ͣ����ܽ�������֡���ݶ�����3������֡��ʼ����
		ec_receive_processdata(EC_TIMEOUTRET);
		
		//���������Ծ�����վ�ʹ�վ��ʱ�����ǳ���Ҫ��
		//����EtherCATϵͳ�������������ܣ�
		//��������ʱ�Ӳ��������⣬���µ�����ж�֡����
		if(E_LastDCtimeFlag == 1)
		{
			E_CurDCtime = (ec_DCtime % 0x100000000);
			if(E_LastDCtime > 0x100000000 - 2 * SYNC0TIME * 1000 && E_CurDCtime < 2 * SYNC0TIME * 1000)
			{//�����������ֵ���0��ʼѭ����Ŀ��λ�û������ֵ
				E_LastDCtime = E_LastDCtime - 0x100000000;
			}
			else if(E_CurDCtime > 0x100000000 - 2 * SYNC0TIME * 1000 && E_LastDCtime < 2 * SYNC0TIME * 1000)
			{//�����������ֵ��Ŀ��λ���Ѿ���0��ʼѭ��
				E_CurDCtime = E_CurDCtime - 0x100000000;
			}
			E_ErrDCtime = SYNC0TIME * 1000 + E_LastDCtime - E_CurDCtime;
			TIM3->ARR = E_ErrDCtime / 1000 - 1;
			E_LastDCtime = (E_LastDCtime + SYNC0TIME * 1000) % 0x100000000;
			
//			if(JDZ_Parameter.Server == 0 || JDZ_Parameter.Server == 6)
//			{//�̴�������ͬ��ʱ�Ӽ�����Ϊ32λ
//				E_CurDCtime = ec_DCtime;
//				if(E_CurDCtime < E_LastDCtime)
//				{
//					E_LastDCtime = E_CurDCtime;
//				}
//				E_ErrDCtime = SYNC0TIME * 1000 + E_LastDCtime - E_CurDCtime;
//				TIM3->ARR = E_ErrDCtime / 1000 - 1;
//				E_LastDCtime = ec_DCtime + SYNC0TIME * 1000;
//			}
//			else
//			{//�㴨��ͬ��ʱ�Ӽ�����Ϊ64λ
//				E_CurDCtime = ec_DCtime;
//				E_ErrDCtime = SYNC0TIME * 1000 + E_LastDCtime - E_CurDCtime;
//				TIM3->ARR = E_ErrDCtime / 1000 - 1;
//				E_LastDCtime = E_LastDCtime + SYNC0TIME * 1000;
//			}
		}
		if(Servo_InitFinish == 1)
		{
			if(E_LastDCtimeFlag == 0)
			{
				E_LastDCtimeFlag = 1;
				E_LastDCtime = (ec_DCtime + SYNC0TIME * 1000) % 0x100000000;
//				E_LastDCtime = ec_DCtime + SYNC0TIME * 1000;
			}
		}
		
		for(i=1; i<=ec_slavecount; i++)
		{
			for(j=1; j<SERVO_NODE_ID_NUM; j++)
			{
				if(ServoNodeID[j] == i)
				{//������ID��ȡ��Ӧ��RPDO����
					PDO_Actual_Status[j - 1] = PDO_InPuts[i]->StatusWord;
					PDO_Cur_Position[j - 1] = PDO_InPuts[i]->CurrentPosition / Axsis_ParPosChange - JDZ_ZeroPosition[j - 1];
//					PDO_Cur_Tprque[j - 1] = PDO_InPuts[i]->ActualTprque;				//��ǰŤ�ط����������ڷ�ײ
//					PDO_Cur_Controlword[j - 1] = PDO_InPuts[i]->ControlWord;
					PDO_Cur_Mode[j - 1] = PDO_InPuts[i]->CurrentMode;
					PDO_Cur_Speed[j - 1] = PDO_InPuts[i]->CurrentVelocity / Axsis_ParVelChange;
					PDO_FirstGetPosition[j - 1] = 1;
					
					if(PDO_Actual_StatusUpdate[j - 1] > 0)
					{
						PDO_Actual_StatusUpdate[j - 1]++;
						if(PDO_Actual_StatusUpdate[j - 1] > PDO_Actual_StatusUpdateTime[j - 1])
						{//״̬������ʱ 1*16ms=80ms ��ȷ��״̬��ʱ�������������Ӧ
							PDO_Actual_StatusUpdate[j - 1] = 0;
						}
					}
				}
			}
		}
	}
	else
	{
		startRevFlag++;
	}
	
	g_AxisActionNextPosRead();
	
	if(E_AllowUpdataSend == E_ALLOW_UPDATA_SEND_EN)
	{//��Ҫ����TPDO����ʱ��������ݸ���
		for(i=1; i<=ec_slavecount; i++)
		{
			targetInf[i].TargetPos = PDO_TargetInf[i].TargetPos;
//			targetInf[i].ContourVel = PDO_TargetInf[i].ContourVel;
//			targetInf[i].TarVelocity = PDO_TargetInf[i].TarVelocity;
//			targetInf[i].OriginOffset = PDO_TargetInf[i].OriginOffset;
//			targetInf[i].Acc = PDO_TargetInf[i].Acc;
//			targetInf[i].Dec = PDO_TargetInf[i].Dec;
//			targetInf[i].QukDec = PDO_TargetInf[i].QukDec;
			targetInf[i].ControlWord = PDO_TargetInf[i].ControlWord;
			targetInf[i].OriginMode = PDO_TargetInf[i].OriginMode;
			targetInf[i].TargetMode = PDO_TargetInf[i].TargetMode;
			targetInf[i].TarTorque = PDO_TargetInf[i].TarTorque;//
		}
		
		E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_DIS;
	}
	
	for(i=1; i<=ec_slavecount; i++)
	{
		PDO_OutPuts[i]->TargetPos = targetInf[i].TargetPos;
//		PDO_OutPuts[i]->ContourVel = targetInf[i].ContourVel;
//		PDO_OutPuts[i]->TarVelocity = targetInf[i].TarVelocity;
//		PDO_OutPuts[i]->OriginOffset = targetInf[i].OriginOffset;
//		PDO_OutPuts[i]->Acc = targetInf[i].Acc;
//		PDO_OutPuts[i]->Dec = targetInf[i].Dec;
//		PDO_OutPuts[i]->QukDec = targetInf[i].QukDec;
		PDO_OutPuts[i]->OriginMode = targetInf[i].OriginMode;
		PDO_OutPuts[i]->ControlWord = targetInf[i].ControlWord;
		PDO_OutPuts[i]->TargetMode = targetInf[i].TargetMode;
		PDO_OutPuts[i]->TarTorque = targetInf[i].TarTorque;
		
	}
	
	ec_send_processdata();
	ec_pdo_outframe();
}

void EtherCATInit(char *ifname)
{
	u16 i = 0;
	u16 j = 0;
//	s16 temp_s16 = 0;
	s32 temp_s32 = 0;
	u16 temp_u16 = 0;
//	s32 temp_u32 = 0;
	s8  temp_s08 = 0;
	int len = 0;
	u32 decTemp = 0;
	u32 decQueTemp = 0;
	u32 accTemp = 0;
	u32 oriDecSpTemp = 0;
	u32 oriFindSpTemp = 0;
//	s32 OriginOffsetTemp = 0;
//	u32 maxRpm = 0;
	
  EtherCatConnectFlag = 2;						//��������
	
	if(ec_init(ifname))									//��ʼ��SOEM��socket�󶨵�ifname���������ƣ�
	{
		if(ec_config_init(TRUE) > 0)			//ȷ�����ӵĴӻ����������豸�����ٽ��дӻ���ʼ��
		{
			for(i = 1; i <= ec_slavecount; i++)
			{//Ϊ�ӻ���PDO�ṹ
				ec_slave[i].PO2SOconfig = &PDO_ServoSetup;
			}
			delay_ms(10);
			
			/*��Ҫ����SDO���õĲ������ڴ˴�����*/
			for(i = 1; i <= ec_slavecount; i++)
			{
				/*ȷ��ÿ���ڵ��ID��Ӧ��EtherCAT��վ���*/
				len = sizeof(temp_u16);
				switch(JDZ_Parameter.Server)
				{
					case 0://�̴������P09.18(2109-13H)����EtherCATͨ�ŵ�ַ������������1��ʼ���ӻ�ID����P09.00(2109-01H)
						ec_SDOread(i, 0x2109, 0x01, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						break;
					case 1://�㴨�����H0E.21(200E-16H)����EtherCATͨ�ŵ�ַ������������1��ʼ���ӻ�ID����H0E.00(200E-01H)
						ec_SDOread(i, 0x200E, 0x01, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						break;
					case 2://���ŵ�����޷���ȡ�����밴X-Y-Z-O��˳�������
						//ec_SDOread(i, 0x2300, 0x00, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						temp_u16 = i;
						break;
					case 3://����������ӻ�ID����PA0.23(0x2023-00H)PA0.24=1
						ec_SDOread(i, 0x2023, 0x00, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						break;
					case 4://�Žݵ�����ӻ�ID����P7.00(2700-00H)
						ec_SDOread(i, 0x2700, 0x00, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						break;
					case 5://̨��������ȡ�ӻ�ID����Pn010(2010-00H)
						ec_SDOread(i, 0x2010, 0x00, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						break;
					case 6://�����������ȡ�ӻ�ID����PA82(2052-00H)
						ec_SDOread(i, 0x2052, 0x00, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						break;
					default://XĬ�ϣ��̴������P09.18(2109-13H)����EtherCATͨ�ŵ�ַ������������1��ʼ���ӻ�ID����P09.00(2109-01H)
						ec_SDOread(i, 0x2109, 0x01, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
						break;
				}
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				if(temp_u16 > 0 && temp_u16 < SERVO_NODE_ID_NUM)
				{
					ServoNodeID[temp_u16] = i;						//��¼�������Ӧ��EtherCAT���
					Servo_ConnectSta[temp_u16 - 1] = 1;		//��¼ÿ���������״̬��1���ӣ�2δ����
				}
				else
				{
					ServoNodeID[temp_u16] = 0;
					Servo_ConnectSta[temp_u16 - 1] = 0;
				}
			}
			
			/*��Ҫ����SDO���õĲ������ڴ˴�����*/
			for(i = 1; i <= ec_slavecount; i++)
			{
				/*���ÿ���ֹͣ���ٶ�ģʽ*/
//				temp_s16 = 6;
//				ec_SDOwrite(i, SERVO_STOP_DEC_MODE, 0x00, FALSE, sizeof(temp_s16), &temp_s16, EC_TIMEOUTRXM);
//				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				
				/*����������ͣ���ٶ�ģʽ*/
//				temp_s16 = 2;
//				ec_SDOwrite(i, SERVO_PAUSE_DEC_MODE, 0x00, FALSE, sizeof(temp_s16), &temp_s16, EC_TIMEOUTRXM);
//				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				
				for(j = 0; j < Axis_Num + Ext_Axis_Num; j++)
				{
					if(ServoNodeID[j + 1] == i)
					{
						if(j < Axis_Num)
						{
							decQueTemp = JXS_Parameter.Accelerate.Time[j];//0824
							decTemp =	JXS_Parameter.Accelerate.Time[j];
							accTemp = JXS_Parameter.Accelerate.Time[j];
							
							oriDecSpTemp = JXS_Parameter.AxisOriginSpeed[j] * 50;
							oriFindSpTemp = CANOPEN_ORIGIN_SPEED_MIN;
						}
						else
						{//��չ��û�л�е���㣬������ֵ
							decQueTemp = ExtendAix_Parameter[j].E_AccTime;//0824
							decTemp =	ExtendAix_Parameter[j].E_AccTime;
							accTemp = ExtendAix_Parameter[j].E_AccTime;
							
							oriDecSpTemp = JXS_Parameter.AxisOriginSpeed[0] * 50;
							oriFindSpTemp = CANOPEN_ORIGIN_SPEED_MIN;
						}
						break;
					}
				}
					decQueTemp = (2050 - decQueTemp) * 5 * Axsis_ParVelChange * 2;
					decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
					accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
				
					oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
					oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
//				switch(JDZ_Parameter.Server)
//				{
//					case 0://�̴����
//						decQueTemp = (2050 - decQueTemp) * 5 * Axsis_ParVelChange * 2;
//						decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
//						accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
//					
//						oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
//						oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
////						OriginOffsetTemp = OriginOffsetTemp * Axsis_ParPosChange;
//						break;
//					case 1://�㴨���
//						decQueTemp = (2050 - decQueTemp) * 5 * Axsis_ParVelChange * 2;
//						decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
//						accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
//					
//						oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
//						oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
////						OriginOffsetTemp = OriginOffsetTemp * Axsis_ParPosChange;
//						break;
//					case 2://���ŵ��
//						decQueTemp = (2050 - decQueTemp) * 5 * Axsis_ParVelChange * 2;
//						decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
//						accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
//					
//						oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
//						oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
////						OriginOffsetTemp = OriginOffsetTemp * Axsis_ParPosChange;
//						break;
//					case 3://�������
//						decQueTemp = (2050 - decQueTemp) * 5 * Axsis_ParVelChange * 2;
//						decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
//						accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
//					
//						oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
//						oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
////						OriginOffsetTemp = OriginOffsetTemp * Axsis_ParPosChange;
//						break;
//					case 4://�Žݵ��
//						decQueTemp = (2050 - decQueTemp) * 5 * Axsis_ParVelChange * 2;
//						decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
//						accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
//					
//						oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
//						oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
////						OriginOffsetTemp = OriginOffsetTemp * Axsis_ParPosChange;
//						break;
//					case 5://̨����
//						decQueTemp = (2050 - decQueTemp) * 5 * Axsis_ParVelChange * 2;
//						decTemp = (2050 - decTemp) * 5 * Axsis_ParVelChange;
//						accTemp = (2050 - accTemp) * 5 * Axsis_ParVelChange;
//					
//						oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
//						oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
////						OriginOffsetTemp = OriginOffsetTemp * Axsis_ParPosChange;
//						break;
//					default:
//						decQueTemp = (2001 - decTemp) * 5 * Axsis_ParVelChange * 2;
//						decTemp = (2001 - decTemp) * 5 * Axsis_ParVelChange;
//						accTemp = (2001 - accTemp) * 5 * Axsis_ParVelChange;
//					
//						oriDecSpTemp = oriDecSpTemp * Axsis_ParVelChange;
//						oriFindSpTemp = oriFindSpTemp * Axsis_ParVelChange;
////						OriginOffsetTemp = OriginOffsetTemp * Axsis_ParPosChange;
//						break;
//				}
				
//				/*���ÿ���ֹͣʱ�ļ��ٶ�*/
//				ec_SDOwrite(i, SERVO_STOP_DEC, 0x00, FALSE, sizeof(decQueTemp), &decQueTemp, EC_TIMEOUTRXM);
//				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				
				/*���û�ԭ����ٶ�*/
				ec_SDOwrite(i, SERVO_BACK_ACC, 0x00, FALSE, sizeof(accTemp), &accTemp, EC_TIMEOUTRXM);
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				
				/*���û����������ٵ��ٶ�*/
				ec_SDOwrite(i, SERVO_BACK_HOME_SP, 0x01, FALSE, sizeof(oriDecSpTemp), &oriDecSpTemp, EC_TIMEOUTRXM);
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				/*���û�������ԭ���ٶ�*/
				ec_SDOwrite(i, SERVO_BACK_HOME_SP, 0x02, FALSE, sizeof(oriFindSpTemp), &oriFindSpTemp, EC_TIMEOUTRXM);
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
//				/*����ԭ��ƫ��*/
//				ec_SDOwrite(i, SERVO_ORIGIN_OFFSET, 0x00, FALSE, sizeof(OriginOffsetTemp), &OriginOffsetTemp, EC_TIMEOUTRXM);
//				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				
				/*��ȡ��ǰλ��*/
				len = sizeof(temp_s32);
				ec_SDOread(i, SERVO_USER_POSITION, 0x00, FALSE, &len, &temp_s32, EC_TIMEOUTRXM);
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				PDO_TargetInf[i].TargetPos = temp_s32;
//				/*����Ŀ��λ��*/
//				ec_SDOwrite(i, SERVO_TARGET_POSITION, 0x00, FALSE, sizeof(temp_s32), &temp_s32, EC_TIMEOUTRXM);
//				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				/*����Ŀ���ٶ�*/
				temp_s32 = 0;
				ec_SDOwrite(i, SERVO_TARGET_SPEED, 0x00, FALSE, sizeof(temp_s32), &temp_s32, EC_TIMEOUTRXM);
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				/*��ȡ��ǰ������*/
				len = sizeof(temp_u16);
				ec_SDOread(i, SERVO_CONTROL_WORD, 0x00, FALSE, &len, &temp_u16, EC_TIMEOUTRXM);
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				PDO_TargetInf[i].ControlWord = temp_u16;
				/*��ȡ��ǰģʽ*/
				len = sizeof(temp_s08);
				ec_SDOread(i, SERVO_WORK_MODE, 0x00, FALSE, &len, &temp_s08, EC_TIMEOUTRXM);
				delay_ms(CANOPEN_DEAL_SDO_DELAY);
				PDO_TargetInf[i].TargetMode = temp_s08;
			}
			
			/*EtherCAT��DCʱ��ͬ����ʼ��*/
			ec_configdc();
			//ec_dcsync0(1, TRUE, SYNC0TIME, 250000);			//����ͬ�����ڣ��Լ�֡�������
			for(i = 1; i <= ec_slavecount; i++)
			{
				ec_dcsync0(i, TRUE, SYNC0TIME, SYNC0TIME * 1000 / 8);			//����ͬ�����ڣ��Լ�֡������֡���һ��Ϊ֡���ڵİ˷�֮һ
			}
			Delay_ms(100);
			
			/*��ʼ���ӻ�PDO�ṹ*/
			ec_set_pdo_queue(ec_config_map(&PDO_IOmap),3);
			
			/*���ôӻ��л�����ȫ����ģʽ*/
			ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
			ec_readstate();
			Delay_ms(2000);																//��ʱ����ﵽ���ʱ�䣬������ʹ��������
			
			/*ӳ��RPDO��TPDO���ݽṹ��ַ*/
			for(i = 1; i <= ec_slavecount; i++)
			{
				PDO_OutPuts[i] = (PDO_Output *)ec_slave[i].outputs;
				PDO_InPuts[i]  = (PDO_Input *)ec_slave[i].inputs;
			}
			
			/*���ôӻ��л�������ģʽ*/
			ec_slave[0].state = EC_STATE_OPERATIONAL;
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET);
			ec_writestate(0);
			
			/*ȷ�����дӻ��ѽ������ģʽ*/
			j = 0;
			while(j < 5)
			{
				j++;
				for(i = 0; i <= ec_slavecount; i++)
				{
					if(ec_slave[i].state != EC_STATE_OPERATIONAL)
					{
						break;
					}
				}
				if(i > ec_slavecount)
				{
					break;
				}
				
				ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE * 4);
				ec_readstate();
			}
			
			if(j < 5 && ec_slave[1].state == EC_STATE_OPERATIONAL)
			{/*�ӻ���ʼ���ɹ��󣬽��������ڲ���ر�����ʼ��*/
				ServoNodeID[0] = 0;																	//����0Ϊ����վ�ţ�ͨ�ű����Ϊ0
				EtherCatConnectFlag = 1;														//���ӳɹ���־
				
				E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_EN;					//����ͬ������һ֡��ȷ���ӻ���������PDO���ݱ���һֱ
				while(E_AllowUpdataSend == E_ALLOW_UPDATA_SEND_EN);
			}
			else
			{
				EtherCatConnectFlag = 0;
			}
		}
		else
		{
			EtherCatConnectFlag = 0;
		}
	}
	else
	{
		EtherCatConnectFlag = 0;
	}
	
	if(EtherCatConnectFlag == 0)
	{//����ʧ��ʱ���ָ���ʼ״̬�ҹر�EtherCAT��Ӧ�ò�����
		ec_slave[0].state = EC_STATE_INIT;
		EtherCatPHY_LinkSta = 0;
		ec_writestate(0);
		ec_close();
		
		//�ŷ���������ʧ��
//		if(Robot_Error_Num == 0 || Robot_Error_Num > E_SERVO_NOT_CON)
//		{
//			Robot_Error_Num = E_BUSCONNECTFAIL;
//		}
		Robot_Error_Data[9] |= 0x10;
	}
}

/**
  * @brief  EtherCAT_LinkSta
  * @param  ��
  * @retval EtherCAT����״̬��1�ɹ���2�������ӣ�0ʧ��
  */
u8 EtherCAT_LinkSta(void)
{
	if(EtherCatPHY_LinkSta > 0 && GET_PHY_LINK_STATUS() == 0)
	{
		EtherCatPHY_LinkSta = 0;
		EtherCatConnectFlag = 0;
		
		//�ŷ���������ʧ��
//		if(Robot_Error_Num == 0 || Robot_Error_Num > E_SERVO_NOT_CON)
//		{
//			Robot_Error_Num = E_BUSCONNECTFAIL;
//		}
		Robot_Error_Data[9] |= 0x10;
	}
	return EtherCatConnectFlag;
}

/**
  * @brief  EtherCAT_LinkErr
  * @param  ��
  * @retval EtherCAT�������
  */
u8 EtherCAT_LinkErrCheck(void)
{
	EtherCAT_LinkSta();
	
	if(EtherCatPHY_LinkSta == 0 || EtherCatConnectFlag == 0)
	{
		//�ŷ���������ʧ��
//		if(Robot_Error_Num == 0 || Robot_Error_Num > E_SERVO_NOT_CON)
//		{
//			Robot_Error_Num = E_BUSCONNECTFAIL;
//		}
		Robot_Error_Data[9] |= 0x10;
		return 1;
	}
	
	return 0;
}

/**
  * @brief  EtherCAT_SendPDOFinish
  * @param  ��
  * @retval EtherCAT����PDO�������
  */
u16 EtherCAT_SendPDOFinish(void)
{
	E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_EN;
	while(E_AllowUpdataSend == E_ALLOW_UPDATA_SEND_EN);
	
	return 0;
}

/*��ȡ��������ǰλ��*/
s32 EtherCAT_GetCurrentPosition(u8 Node_ID)
{
	return PDO_InPuts[ServoNodeID[Node_ID]]->CurrentPosition;
}

/**
  * @brief  ��ʱ��3�жϷ�����
  * @param  ��
  * @retval ��
  */
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
		/*��ȡEthernet���������Ƿ�����*/
		if(EtherCatPHY_LinkSta > 0)
		{
			if(EtherCatConnectFlag == 1)
			{//���ӳɹ��󣬷�������֡
				EtherCAT_RevSendDeal();
			}
			else if(EtherCatConnectFlag == 2)
			{//��������ʱ��������֡
				ec_send_processdata();
				ec_pdo_outframe();
			}
			else
			{//��������ʧ�ܺ���Ҫÿ��������������־
				E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_DIS;
			}
		}
		else
		{//��������ʧ�ܺ���Ҫÿ��������������־
			E_AllowUpdataSend = E_ALLOW_UPDATA_SEND_DIS;
		}
		TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	}
	
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
