/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f4xx.h"
#include "Usart.h" 
#include "Error.h"
#include "in.h" 
#include "out.h"
#include "w25qxx.h"
#include "Manual.h"
#include "Auto.h"
#include "Auto_2.h"
#include "SpeedControl.h"
#include "signalWatch.h"
#include "StatusControl.h"
#include "Parameter.h"
#include "BackToOrigin.h"
#include "JDZ.h"
#include "CANopen.h"
#include "ActionOperate.h"
#include "EtherCAT_App.h"
#include "stdlib.h"
#include "ExtendCom.h"

/*----- ȫ�ֱ��������������λ -----*/
u8 Error_Status = NO_ERROR;							//���������־
u8 Cancle_Genaral_Warning = FALSE;  		//ȡ����ǰ�������

/*- �������ݣ�ÿһλ����һ�����ͱ�����1��ʾ������0��ʾ�ޱ���                -*/
/* Data[0] ��ͣ   ��   �������� ����λ ��ȫ�� ��ѹ�쳣 �󻬱���   Ԥ�� */
/* [7:0]    1      1      1      1       1       1        1         1 */
u8 Robot_Error_Data[15] = {0};					 

//[n]��¼IO�Ƿ���Ҫ�������չ���
u16 IO_Input_waittime[30] = {0};			//�����ⳬʱʱ��
u16 IO_Input_keepMin[30] = {0};			//���뱣��ʱ��-����
u16 IO_Input_keepMax[30] = {0};			//���뱣��ʱ��-����
u16 IO_Sign_On_Off[30] = {0};					//�����źų������ձ�־��0������1����

//[n]��¼���IO��λѡ��
u16 OutPut_BeforeOrigin[30] = {0};		   	 //����ǰѡ��
u16 OutPut_AfterOrigin[30] = {0};			     //�����ѡ��
u16 OutPut_Common_Alarm[30] = {0};	 //��ͨ����
u16 OutPut_Emerge_Alarm[30] = {0};	 //��ͣ����
u16 OutPut_Pause[30] = {0};			     //��ͣ
u16 OutPut_Stop[30] = {0};			     //ֹͣ

//�ŷ�����������ʼ��ⶨʱ������ֹ����ֱ�ӱ���
u32 Axsis_Error_Count = 1;						//�ŷ�����������ʼ��ⶨʱ���������Ϳ�ʼ����
u8 Axsis_Error_Permit_Flag = FALSE;		//�ŷ�����������ʼ��������־
u8 g_Auto_ActionConflict_Flag = FALSE; 							//����䶯���ظ�

/**************************************************************************************************
**  ��������  CloseTotalMotorError(u8 Axsis)
**	���������Axsis ���� 
**	�����������
**	�������ܣ�����ʱ���ر����е�������
**	��ע��
**  ���ߣ�	Lin
**  �������ڣ� 
***************************************************************************************************/
void CloseTotalMotorError(void)
{
	u16 i = 0;
	
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		Servo_Stop(i);
	}
}

/**************************************************************************************************
**  ��������  EmergencyStopJudge()
**	�����������
**	�����������
**	�������ܣ���ͣ�ж�
**	��ע��	  ��ͣ���£�Ϊ�ߵ�ƽ��Ч�ź�
**  ���ߣ�          
**  �������ڣ� 
***************************************************************************************************/
void EmergencyStopJudge(void)
{
	if(ReadEmergencyStop())
	{//��⵽��ͣΪ����״̬
		Robot_Error_Data[0] = Robot_Error_Data[0] | 0x80;
		CloseTotalMotorError();
	}
	else if((Robot_Error_Data[0] & 0x80) && ExtendEmergencyStop == 0)
	{//��ͣ��Ϊ�ͷ�״̬
		Cancle_Genaral_Warning = TRUE;
	}
}

/**************************************************************************************************
**  ��������  SoftLimitJudge()
**	�����������
**	�����������
**	�������ܣ����޼��
**	��ע��	  ��
**  ���ߣ�          
**  �������ڣ� 
***************************************************************************************************/
void SoftLimitJudge(void)
{	
	if(Origin_Backed == TRUE)
	{//����ɹ���Ż�������λ��ִ�л�е����ָ��ʱ���������λ
		if((AxisMoveFlag[X_Axsis] | AxisMoveFlag[Z_Axsis] | AxisMoveFlag[L_Axsis] | AxisMoveFlag[O_Axsis]) != 0)
		{
			if(Robot_SoftLimit[X_Axsis].Switch_Limit && ((m_PulseTotalCounter[X_Axsis] + JDZ_AllowError < Axsis_Minlength[X_Axsis] && Axsis_Move_Direction[X_Axsis] == NEGATIVE) ||
				(m_PulseTotalCounter[X_Axsis] > Axsis_Maxlength[X_Axsis] + JDZ_AllowError && Axsis_Move_Direction[X_Axsis] == POSITIVE)) && Program_Axis_Origin_Flag[X_Axsis] == FALSE)	//X��
			{
				Robot_Error_Data[0] = Robot_Error_Data[0] | 0x10;
				CloseTotalMotorError();
			}
			else if(Robot_SoftLimit[L_Axsis].Switch_Limit && ((m_PulseTotalCounter[L_Axsis] + JDZ_AllowError < Axsis_Minlength[L_Axsis] && Axsis_Move_Direction[L_Axsis] == NEGATIVE) ||
				(m_PulseTotalCounter[L_Axsis] > Axsis_Maxlength[L_Axsis] + JDZ_AllowError && Axsis_Move_Direction[L_Axsis] == POSITIVE)) && Program_Axis_Origin_Flag[L_Axsis] == FALSE)	//L��
			{
				Robot_Error_Data[0] = Robot_Error_Data[0] | 0x10;
				CloseTotalMotorError();
			}
			else if(Robot_SoftLimit[Z_Axsis].Switch_Limit && ((m_PulseTotalCounter[Z_Axsis] + JDZ_AllowError < Axsis_Minlength[Z_Axsis] && Axsis_Move_Direction[Z_Axsis] == NEGATIVE) ||
				(m_PulseTotalCounter[Z_Axsis] > Axsis_Maxlength[Z_Axsis] + JDZ_AllowError && Axsis_Move_Direction[Z_Axsis] == POSITIVE)) && Program_Axis_Origin_Flag[Z_Axsis] == FALSE)	//Z��
			{
				Robot_Error_Data[0] = Robot_Error_Data[0] | 0x10;
				CloseTotalMotorError();
			}
			else if(Robot_SoftLimit[O_Axsis].Switch_Limit && ((m_PulseTotalCounter[O_Axsis] + JDZ_AllowError < Axsis_Minlength[O_Axsis] && Axsis_Move_Direction[O_Axsis] == NEGATIVE) ||
				(m_PulseTotalCounter[O_Axsis] > Axsis_Maxlength[O_Axsis] + JDZ_AllowError && Axsis_Move_Direction[O_Axsis] == POSITIVE)) && Program_Axis_Origin_Flag[O_Axsis] == FALSE)	//O��
			{
				Robot_Error_Data[0] = Robot_Error_Data[0] | 0x10;
				CloseTotalMotorError();
			}
		}
		
		if(Robot_SoftDistance.MinDistance > MINROBOTPOSITION && Robot_SoftDistance.MaxDistance > Robot_SoftDistance.MinDistance)
		{//��С��ȫ����>0 �� �����������С����ʱ���Ž��м�⣻��С������Ϊ0������Ϊ�ر�״̬
			if((Robot_SoftDistance.MaxDistance + 2 * MINROBOTPOSITION) < (Robot_SoftDistance.MinDistance + m_PulseTotalCounter[X_Axsis] + m_PulseTotalCounter[O_Axsis]) && \
				((AxisMoveFlag[X_Axsis] == TRUE && Axsis_Move_Direction[X_Axsis] == POSITIVE) || (AxisMoveFlag[O_Axsis] == TRUE && Axsis_Move_Direction[O_Axsis] == POSITIVE)))
			{//������ < ��С��ȫ����+X��λ��+O��λ�ã�������Сֵ���ڲ������������
				Robot_Error_Data[0] = Robot_Error_Data[0] | 0x20;
				CloseTotalMotorError();
			}
		}
		
		if((AxisMoveFlag[U_Axsis] | AxisMoveFlag[V_Axsis]) != 0)
		{
			if(ExtendAix_Parameter[U_Ext_Axsis].E_Origin_Set == 1 && ((m_PulseTotalCounter[U_Axsis] + JDZ_AllowError < Axsis_Minlength[U_Axsis] && Axsis_Move_Direction[U_Axsis] == NEGATIVE) ||
				(m_PulseTotalCounter[U_Axsis] > Axsis_Maxlength[U_Axsis] + JDZ_AllowError && Axsis_Move_Direction[U_Axsis] == POSITIVE)))	//U��
			{
				Robot_Error_Data[0] = Robot_Error_Data[0] | 0x10;
				CloseTotalMotorError();
			}
			else if(ExtendAix_Parameter[V_Ext_Axsis].E_Origin_Set == 1 && ((m_PulseTotalCounter[V_Axsis] + JDZ_AllowError < Axsis_Minlength[V_Axsis] && Axsis_Move_Direction[V_Axsis] == NEGATIVE) ||
				(m_PulseTotalCounter[V_Axsis] > Axsis_Maxlength[V_Axsis] + JDZ_AllowError && Axsis_Move_Direction[V_Axsis] == POSITIVE)))	//V��
			{
				Robot_Error_Data[0] = Robot_Error_Data[0] | 0x10;
				CloseTotalMotorError();
			}
		}
	}
}

/**************************************************************************************************
**  ��������  XAxsisError()
**	�����������
**	�����������
**	�������ܣ�X�����
**	��ע��	  ��
**  ���ߣ�     
**  �������ڣ�
***************************************************************************************************/
void XAxsisError(u8 alarm)
{
	if(alarm == 1)					          //��ȡX������ź�
	{	
		Robot_Error_Data[6] = Robot_Error_Data[6] | 0x01;
		ServoDisable_PDO(SERVO_NODE_ID_01_X);
	}
}

/**************************************************************************************************
**  ��������  ZAxsisError()
**	�����������
**	�����������
**	�������ܣ�Z�����
**	��ע��	  ��
**  ���ߣ�      
**  �������ڣ�
***************************************************************************************************/
void ZAxsisError(u8 alarm)
{
	if(alarm == 1)					          //��ȡZ������ź�
	{
		Robot_Error_Data[6] = Robot_Error_Data[6] | 0x02;
		ServoDisable_PDO(SERVO_NODE_ID_03_Z);
	}
}

/**************************************************************************************************
**  ��������  LAxsisError()
**	�����������
**	�����������
**	�������ܣ�L�����
**	��ע��	  ��
**  ���ߣ�      
**  �������ڣ�
***************************************************************************************************/
void YAxsisError(u8 alarm)
{
	if(alarm == 1)					          //��ȡY������ź�
	{	  
		Robot_Error_Data[6] = Robot_Error_Data[6] | 0x04;
		ServoDisable_PDO(SERVO_NODE_ID_02_L);
	}
}

/**************************************************************************************************
**  ��������  OAxsisError()
**	�����������
**	�����������
**	�������ܣ�O�����
**	��ע��	  ��
**  ���ߣ�       
**  �������ڣ�
***************************************************************************************************/
void OAxsisError(u8 alarm)
{
	if(alarm == 1)					          //��ȡO������ź�
	{	  
		Robot_Error_Data[6] = Robot_Error_Data[6] | 0x08;
		ServoDisable_PDO(SERVO_NODE_ID_04_O);
	}
}

/**************************************************************************************************
**  ��������  MotorAlarmProcess()
**	�����������
**	�����������
**	�������ܣ������������
**	��ע��	  ��
**  ���ߣ�     
**  �������ڣ�
***************************************************************************************************/
void MotorAlarmProcess(void)
{
	u8 alarm[SERVO_NODE_NUM] = {0};
	
	ServoMoveAlarmSta(alarm);
	
	if(Axsis_Error_Permit_Flag == TRUE)
	{//5s֮����ȥ����ᱨ��
		if(JXS_Parameter.AlarmSwitch[X_Axsis] == 1)
		{
			XAxsisError(alarm[X_Axsis]);		   //X�ᱨ��
		}
		else
		{
			Robot_Error_Data[6] = Robot_Error_Data[6] & 0xfe;
		}
		
		if(JXS_Parameter.AlarmSwitch[L_Axsis] == 1)
		{
			YAxsisError(alarm[L_Axsis]);		   //Y�ᱨ��
		}
		else
		{
			Robot_Error_Data[6] = Robot_Error_Data[6] & 0xfb;
		}
		
		if(JXS_Parameter.AlarmSwitch[Z_Axsis] == 1)
		{
			ZAxsisError(alarm[Z_Axsis]);		   //Z�ᱨ��
		}
		else
		{
			Robot_Error_Data[6] = Robot_Error_Data[6] & 0xfd;
		}
		
		if(JXS_Parameter.AlarmSwitch[O_Axsis] == 1)
		{
			OAxsisError(alarm[O_Axsis]);		   //O�ᱨ��
		}
		else
		{
			Robot_Error_Data[6] = Robot_Error_Data[6] & 0xf7;
		}
	}
}

/**************************************************************************************************
**  ��������  CurrentWorkFinished()
**	�����������
**	�����������
**	�������ܣ��ӹ����
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void  CurrentWorkFinished(void)
{
	if(g_Auto_WorkFinished_Flag)
	{
		Robot_Error_Data[1] = Robot_Error_Data[1] | 0x20;
	}
	else
	{
		Robot_Error_Data[1] = Robot_Error_Data[1] & 0xdf;
	}
}

/**************************************************************************************************
**  ��������  CurrentCJWorkFinished()
**	�����������
**	�����������
**	�������ܣ����ӹ����
**	��ע��	 
**  ���ߣ�       
**  �������ڣ�2021/01/18
***************************************************************************************************/
void  CurrentCJWorkFinished(void)
{
	 if(g_Auto_CJWorkFinished_Flag)
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x01;
	 }
	 else
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xfe;
	 }
}

/**************************************************************************************************
**  ��������  AutoActionError()
**	�����������
**	�����������
**	�������ܣ�
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void  AutoActionError(void)
{
	 if(g_Auto_ActionError_Flag)
	 {
		 if(MD_PositionErr_Flag==FALSE)
		 {
			Robot_Error_Data[1] = Robot_Error_Data[1] | 0x10;
		 }
		 else if(MD_PositionErr_Flag==TRUE)
		 {
			 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x02;
		 }
	 }
	 else
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xed;
	 }

}
/**************************************************************************************************
**  ��������  AutoActionTimeOut()
**	�����������
**	�����������
**	�������ܣ�������ʱ����
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void  AutoActionTimeOut(void)
{
	 if(g_Auto_ActionTimeOut_Flag)
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x04;
	 }
	 else
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xfb;
	 }
}

/**************************************************************************************************
**  ��������  AutoActionConflict()
**	�����������
**	�����������
**	�������ܣ���������ظ�����
**	��ע��	 
**  ���ߣ�       
**  �������ڣ�
***************************************************************************************************/
void AutoActionConflict(void)
{
	 if(g_Auto_ActionConflict_Flag)
	 {
	 	 Robot_Error_Data[7] = Robot_Error_Data[7] | 0x04;
	 }
	 else
	 {
	 	 Robot_Error_Data[7] = Robot_Error_Data[7] & 0xfb;
	 }
}

/**************************************************************************************************
**  ��������  ErrorOperate()
**	�����������
**	�����������
**	�������ܣ���������
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void ErrorOperate(void)
{
	u8 i=0;
//	u8 Temp_data[4]={0};
	
	EmergencyStopJudge(); 					//��ͣ���
	MotorAlarmProcess();						//����������
//#ifdef HARDLIMITJUDGE_INPUT
//	HardLimitJudge();	   	  			//Ӳ��λ����
//#endif
	SoftLimitJudge();		  					//����λ���	 
	SafeAreaJudge();	      				//��ȫ�����--����
	AutoActionConflict();					//��������ظ�����
	
	if(Work_Status == AUTO_WORK_MODE)
	{//�Զ�ģʽ�½��д���
		CurrentWorkFinished();	   		//�ӹ����
		CurrentCJWorkFinished();			//����������
		AutoActionError();	       		//�Զ����н���AUTO_ERROR
		AutoActionTimeOut();	  			//������ʱ
	}
	
	//ȡ����ǰ����
	if(Cancle_Genaral_Warning == TRUE)
	{
		CurProgramRead(g_Run_Program_Num_Pre);							//ѡ�г������Ϊԭ��ѡ�еĳ���
		
		//��������ͣ������ֱ�����㣬��������ֱ������
		for(i=0; i<15; i++)
		{
			Robot_Error_Data[i] = 0;
		}
		
		//���������־
		g_Auto_ActionError_Flag = FALSE;
		g_Auto_ActionTimeOut_Flag = FALSE;
		Cancle_Genaral_Warning = FALSE;
		g_Auto_ActionConflict_Flag = FALSE;
		MD_PositionErr_Flag = FALSE;

		if(g_Auto_WorkFinished_Flag)	
		{//�����ǰ������������
			SC_Parameter.SC_Num=0;
			g_Auto_WorkFinished_Flag = FALSE;
		}
		else if(g_Auto_CJWorkFinished_Flag)	
		{//�����ǰ����������
			SC_Parameter.CJ_Num=0;
			g_Auto_CJWorkFinished_Flag = FALSE;	
		}
	}
	 
	for(i=0; i<15; i++)
	{
		if(Robot_Error_Data[i] != 0)
		{
			if(Robot_Error_Data[0] != 0 || Robot_Error_Data[6] != 0 || Robot_Error_Data[8] != 0)
			{//����ֹͣ�౨��
				Error_Status = ERROR_EMERG_HAPPEN;	//0x02��������
			}
			else
			{//��ͨ��ֹͣͣ�౨��
				Error_Status = ERROR_HAPPEN;				//0x01
			}
			break;
		}
		else
		{
			Error_Status = NO_ERROR;
		}
	}
	
	if(Error_Status==NO_ERROR)
	{//δ��������
		
	}
	else
	{//��������
		if(Error_Status == ERROR_EMERG_HAPPEN) 
		{//��ͣ������ֱ��ֹͣ��е��
			for(i=0;i<OUTPUT_NUM;i++)
			{
				if(OutPut_Emerge_Alarm[i]==Emerge_Alarm)
				{
					SetOutput(i);//ָʾ����
				}
			}
			Cancle_Get_Position_Flag();
			CloseTotalMotorError();
			if((g_Program_Is_Debuging == FALSE) && (g_AutoStatue == AUTO_RUNNING || g_AutoStatue == AUTO_PAUSE)\
				&& (g_Auto_PresentLine != 0 && g_Auto_PresentLine != Free_Program_Operate.Num-1))
			{
				SC_Parameter.NG_Num++;
			}
			
			if(m_InterpCurveFlag == INTER_CURVE_NO && m_InterpAxisMoveFlag[X_Axsis] == 0 && m_InterpAxisMoveFlag[L_Axsis] == 0 \
					&& m_InterpAxisMoveFlag[Z_Axsis] == 0 && m_InterpAxisMoveFlag[O_Axsis] == 0 && m_InterpAxisMoveFlag[U_Axsis] == 0 && m_InterpAxisMoveFlag[V_Axsis] == 0)
			{//�岹�˶�ִ����ɺ󣬿���дFLASH
				AutoReset();
			}
		}
		else
		{//����������������ͣ��ֹͣ����
			for(i=0;i<OUTPUT_NUM;i++)
			{
				if(OutPut_Common_Alarm[i]==Common_Alarm)
				{
					SetOutput(i);//ָʾ����
				}
			}
			if(g_AutoStatue == AUTO_RUNNING)	//���ɱ��-���� ״̬Ҳ��������,��������ͣ״̬
			{
				g_Auto_Order_Pause = TRUE;
			}
			Linked_Move_Enable = DISABLE;
			Jog_Move_Enable = DISABLE;
			Back_Origin_Flag = FALSE;
		}
	}
}

/**************************************************************************************************
**  ��������  HardLimitJudge()
**	�����������
**	�����������
**	�������ܣ�Ӳ��λ���
**	��ע��	  ��
**  ���ߣ�       
**  �������ڣ�
***************************************************************************************************/
void HardLimitJudge(void)
{
	static u8 hardLimitStaInit = 0;
	static u8 hardMinLimitSta[Axis_Num] = {0};
	static u8 hardMaxLimitSta[Axis_Num] = {0};
	
	if(hardLimitStaInit == 0)
	{
		hardMinLimitSta[X_Axsis] = X_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
		hardMinLimitSta[L_Axsis] = Y_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
		hardMinLimitSta[Z_Axsis] = Z_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
		hardMinLimitSta[O_Axsis] = O_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
		
		hardMaxLimitSta[X_Axsis] = X_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
		hardMaxLimitSta[L_Axsis] = Y_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
		hardMaxLimitSta[Z_Axsis] = Z_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
		hardMaxLimitSta[O_Axsis] = O_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
		
		hardLimitStaInit = 1;
	}
	
	//X����СӲ��λ-X16
	if(Temp_IO_Switch_Parameter[I_DETECT_X_MIN_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_X_MIN_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//����
			if(X_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 1 && hardMinLimitSta[X_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x01;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_X_MIN_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//����
			if(X_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 0 && hardMinLimitSta[X_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x01;
				CloseTotalMotorError();
			}
		}
	}
	
	//Y����СӲ��λ-X17
	if(Temp_IO_Switch_Parameter[I_DETECT_Y_MIN_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_Y_MIN_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//����
			if(Y_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 1 && hardMinLimitSta[L_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x02;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_Y_MIN_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//����
			if(Y_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 0 && hardMinLimitSta[L_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x02;
				CloseTotalMotorError();
			}
		}
	}
	
	//Z����СӲ��λ-X18
	if(Temp_IO_Switch_Parameter[I_DETECT_Z_MIN_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_Z_MIN_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//����
			if(Z_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 1 && hardMinLimitSta[Z_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x04;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_Z_MIN_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//����
			if(Z_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 0 && hardMinLimitSta[Z_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x04;
				CloseTotalMotorError();
			}
		}
	}
	
	//O����СӲ��λ-X19
	if(Temp_IO_Switch_Parameter[I_DETECT_O_MIN_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_O_MIN_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//����
			if(O_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 1 && hardMinLimitSta[O_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x08;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_O_MIN_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//����
			if(O_AXIS_HARD_MIN_LIMIT_LOW_LEVEL == 0 && hardMinLimitSta[O_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x08;
				CloseTotalMotorError();
			}
		}
	}
	
	
	//X�����Ӳ��λ-X16
	if(Temp_IO_Switch_Parameter[I_DETECT_X_MAX_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_X_MAX_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//����
			if(X_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 1 && hardMaxLimitSta[X_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x10;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_X_MAX_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//����
			if(X_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 0 && hardMaxLimitSta[X_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x10;
				CloseTotalMotorError();
			}
		}
	}
		
	//Y�����Ӳ��λ-X17
	if(Temp_IO_Switch_Parameter[I_DETECT_Y_MAX_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_Y_MAX_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//����
			if(Y_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 1 && hardMaxLimitSta[L_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x20;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_Y_MAX_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//����
			if(Y_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 0 && hardMaxLimitSta[L_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x20;
				CloseTotalMotorError();
			}
		}
	}
	
	//Z�����Ӳ��λ-X18
	if(Temp_IO_Switch_Parameter[I_DETECT_Z_MAX_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_Z_MAX_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//����
			if(Z_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 1 && hardMaxLimitSta[Z_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x40;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_Z_MAX_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//����
			if(Z_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 0 && hardMaxLimitSta[Z_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x40;
				CloseTotalMotorError();
			}
		}
	}
	
	//O�����Ӳ��λ-X19
	if(Temp_IO_Switch_Parameter[I_DETECT_O_MAX_LIMIT])
	{
		if(IO_Sign_On_Off[I_DETECT_O_MAX_LIMIT] == 0 && Back_Origin_Flag == FALSE)	
		{//����
			if(O_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 1 && hardMaxLimitSta[O_Axsis] == 0)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x80;
				CloseTotalMotorError();
			}
		}
		else if(IO_Sign_On_Off[I_DETECT_O_MAX_LIMIT] == 1 && Back_Origin_Flag == FALSE)	
		{//����
			if(O_AXIS_HARD_MAX_LIMIT_LOW_LEVEL == 0 && hardMaxLimitSta[O_Axsis] == 1)
			{
				Robot_Error_Data[8] = Robot_Error_Data[8] | 0x80;
				CloseTotalMotorError();
			}
		}
	}

	
	hardMinLimitSta[X_Axsis] = X_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
	hardMinLimitSta[L_Axsis] = Y_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
	hardMinLimitSta[Z_Axsis] = Z_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
	hardMinLimitSta[O_Axsis] = O_AXIS_HARD_MIN_LIMIT_LOW_LEVEL;
	
	hardMaxLimitSta[X_Axsis] = X_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
	hardMaxLimitSta[L_Axsis] = Y_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
	hardMaxLimitSta[Z_Axsis] = Z_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
	hardMaxLimitSta[O_Axsis] = O_AXIS_HARD_MAX_LIMIT_LOW_LEVEL;
}

/**************************************************************************************************
**  ��������  Scan_TimeOut_IO()
**	���������i_num ����ڱ��
**	�����������
**	�������ܣ����볬ʱ��������
**	��ע��	  ��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void Scan_TimeOut_IO(u8 i_num)
{
	if((i_num >= K_IOINSTRUCT_INPUT1) && (i_num <= K_IOINSTRUCT_INPUT30))//����˫
	{
		i_num -= 0x48;								//X0��Ӧ0x44
		Robot_Error_Data[2 + i_num/8] |= 0x01<<(i_num % 8);
	}	
}

/**************************************************************************************************
**  ��������  Scan_TimeOut_OUTPUT()
**	���������i_num ����ڱ��
**	�����������
**	�������ܣ����볬ʱ��������
**	��ע��	  ��
**  ���ߣ�    ����     
**  �������ڣ�2013/12/20 
***************************************************************************************************/
void Scan_TimeOut_OUTPUT(u8 o_num)
{
	if(o_num <= OUTPUT_NUM)
	{
		o_num -= 0x14;
		Robot_Error_Data[5] |= 0x80;
	}	
}


/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/





