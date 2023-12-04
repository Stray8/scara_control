#include "stm32f4xx.h"
#include "JDZ.h" 
#include "Usart.h" 
#include "Delay.h"
#include "BackToOrigin.h"
#include "StatusControl.h"
#include "w25qxx.h"
#include "SpeedControl.h" 
#include "Parameter.h" 
#include "Error.h"
#include "CANopen.h"
#include "ActionOperate.h"
#include "SignalWatch.h"
#include "EtherCAT_App.h"

u16 CRCresult = 0;													//CRCУ��ֵ
u8  CRCresult1 = 0;													//CRCУ��ֵ�Ͱ�λ
u8  CRCresult2 = 0;													//CRCУ��ֵ�߰�λ
u8 JDZ_ReadPosition_count = 0;							//��ѭ����ȡ����������
u8 JDZ_ReadPosition_Flag = FALSE;						//��ѭ����ȡ������ֵ�ı�־λ
u8  JDZ_ReceiveDataCounter = 0;		       		//����ֵ�������ݼ���

s32 JDZ_Encoder[2][Axis_Num + Ext_Axis_Num] = {0};									//��0/1����0/1/2/3��0���������Ȧ����1�����������Ȧλ��,�������4����
s32 JDZ_Encoder_Position[Axis_Num + Ext_Axis_Num] = {0};						//ʹ�ú̴����ֱ�ӻ��λ��

u8  JDZ_StartReceiveDataFlag = FALSE;								//����ֵ��ʼ�������ݱ�־
u8	JDZ_NewOrder = FALSE;														//���յ��µĶ�ֵ����
u8  JDZ_AllowError = 100;														//�ӱ�����������������ֵ�붨ʱ������������ֵ��������ȷ�ϣ�

u8  IRQ_Count = 0;																	//���յ�������֡������CRCУ�飬���½���
u8  First_Get_Position_Flag[Axis_Num + Ext_Axis_Num] = {0};				//��һ�ο������߱������ȡλ�ñ�־λ
s32 JDZ_Position[Axis_Num + Ext_Axis_Num] = {0};										//�ӱ�����������������ֵ
u8  Send_Count = 0;

s32 JDZ_ZeroPosition[Axis_Num + Ext_Axis_Num] = {0};								//����ֵ������0��λ��
u8 m_PositionResetStep[Axis_Num + Ext_Axis_Num] = {0};
u32 m_PositionResetCounter[Axis_Num + Ext_Axis_Num] = {0};

/*
 *����־λ����->�������������������������¸�ֵ������������ԭ������ô˺���
*/
void Cancle_Get_Position_Flag()
{
	u16 i = 0;
	
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		First_Get_Position_Flag[i] = 0;
	}
}

/*
 *��һ�ο������߽���ֹͣ��δ������ǰλ��,��ʾ����
*/
u8 Not_Get_Position()
{
	u8 result = 0;
	if(JDZ_Parameter.Switch == 1)
	{
		switch(JXS_Parameter.Origin)
		{
			case FOM_X:
				if(First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1)
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			
			case FOM_Z:
				if(First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1)
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			
			case FOM_Y:
				if(First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1)
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			
			case FOM_O:
				if(First_Get_Position_Flag[O_Axsis] == 0 && JDZ_Parameter.OriginSetting[O_Axsis] == 1)
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			
			case FOM_Y_X:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			
			case FOM_X_Y:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			
			case FOM_Z_X:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}		 
				break;

			case FOM_X_Z:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}    		 
				break;

			case FOM_O_X:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[O_Axsis] == 0 && JDZ_Parameter.OriginSetting[O_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}	 
				break;

			case FOM_Z_X_L:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}   		 
				break;

			case FOM_Z_L_X:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				} 
				break;

			case FOM_Z_L_X_O:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1) \
						|| (First_Get_Position_Flag[O_Axsis] == 0 && JDZ_Parameter.OriginSetting[O_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			case FOM_Z_X_L_O:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1) \
						|| (First_Get_Position_Flag[O_Axsis] == 0 && JDZ_Parameter.OriginSetting[O_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;

			case FOM_L_O_Z_X:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1) \
						|| (First_Get_Position_Flag[O_Axsis] == 0 && JDZ_Parameter.OriginSetting[O_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			case FOM_Z_O_X_L:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1) \
						|| (First_Get_Position_Flag[O_Axsis] == 0 && JDZ_Parameter.OriginSetting[O_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;

			case FOM_O_Z_X_L:
				if((First_Get_Position_Flag[X_Axsis] == 0 && JDZ_Parameter.OriginSetting[X_Axsis] == 1) \
						|| (First_Get_Position_Flag[Z_Axsis] == 0 && JDZ_Parameter.OriginSetting[Z_Axsis] == 1) \
						|| (First_Get_Position_Flag[L_Axsis] == 0 && JDZ_Parameter.OriginSetting[L_Axsis] == 1) \
						|| (First_Get_Position_Flag[O_Axsis] == 0 && JDZ_Parameter.OriginSetting[O_Axsis] == 1))
				{
					result = 1;
					Robot_Error_Data[7] |= 0x02;
				}
				break;
			default:
				break;
		}
		
	}
	return result;
}

/*
 * ����ֵģʽ�£���������������ͷ���������Ƚϣ��������Χ�ڣ�����1
*/
u8 Judge_JDZ_Error(u8 Axis)
{
	return 1;
}

/*
 * ����ԭ���Ƿ�����
 */
void JDZ_Save_Origin_Set()
{
	u8 temp[4] = {0};
	
	temp[0] = JDZ_Parameter.OriginSetting[X_Axsis];	
	temp[1] = JDZ_Parameter.OriginSetting[L_Axsis];	
	temp[2] = JDZ_Parameter.OriginSetting[Z_Axsis];	
	temp[3] = JDZ_Parameter.OriginSetting[O_Axsis];
	
	W25QXX_Write(temp,0x1117,4);
}

/*
 * ԭ��λ�ÿ�����ȡ
 */
void JDZ_OriginRead(void)
{
	u16 i = 0;
	u8 temp[32] = {0};
	
	W25QXX_Read(temp, P_SERVO_JDZ_ZERO_HEAD, P_SERVO_JDZ_ZERO_LEN);
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		JDZ_ZeroPosition[i] = 0;
		JDZ_ZeroPosition[i] |= (u32)(((u32)temp[0 + i * 4])|((u32)temp[1 + i * 4]<<8)|((u32)temp[2 + i * 4]<<16)|((u32)temp[3 + i * 4]<<24));
	}
}

/*
 * ԭ������
 */
void JDZ_Origin_Set(void)
{
	u16 i = 0;
	u32 offsetPoint = 0;
	u8 temp[32] = {0};
	
	if(JDZ_SetOrigin_Flag == TRUE)
	{//�����ŷ���֧��ֱ���������λ�ã�ֻ�ܱ��ֵ�ǰλ����Ϊ���
		if(JDZ_Origin_Setting_Axis_Num - 1 < Axis_Num)
		{
			offsetPoint = JXS_Parameter.OrignOffset[JDZ_Origin_Setting_Axis_Num - 1] * Step_Coefficient[JDZ_Origin_Setting_Axis_Num - 1] / 100;
		}
		else
		{
			offsetPoint = ExtendAix_Parameter[JDZ_Origin_Setting_Axis_Num - Axis_Num - 1].E_OriginOffset * Step_Coefficient[JDZ_Origin_Setting_Axis_Num - 1] / 100;
		}
		JDZ_ZeroPosition[JDZ_Origin_Setting_Axis_Num - 1] = EtherCAT_GetCurrentPosition(JDZ_Origin_Setting_Axis_Num) / Axsis_ParPosChange - offsetPoint;
		
		for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
		{
			temp[0 + i * 4] = (u8)(JDZ_ZeroPosition[i] & 0x000000FF);	
			temp[1 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0x0000FF00)>>8);	
			temp[2 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0x00FF0000)>>16);	
			temp[3 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0xFF000000)>>24);
		}
		W25QXX_Write(temp, P_SERVO_JDZ_ZERO_HEAD, P_SERVO_JDZ_ZERO_LEN);
		
		CSP_Mode_AxisInit(JDZ_Origin_Setting_Axis_Num);									//��������Ϊλ��ģʽ
		
		JDZ_Parameter.OriginSetting[JDZ_Origin_Setting_Axis_Num - 1] = 1;
		Robot_Reset();
		JDZ_SetOrigin_Flag = FALSE;
		JDZ_Save_Origin_Set();
		Cancle_Get_Position_Flag();
		
		delay_ms(100);									//ȷ��λ��ͬ���ɹ������´�PID
	}
}

/*
 * ���긴λ����
 */
u8 Position_Reset(u8 Axis, u32 Position)
{
	u16 i = 0;
	u32 offsetPoint = 0;
	u8 temp[32] = {0};

	if(m_PositionResetStep[Axis] == 0)
	{/*��ǰλ������Ϊ������*/
		offsetPoint = Position - MINROBOTPOSITION;
		JDZ_ZeroPosition[Axis] = EtherCAT_GetCurrentPosition(Axis_To_ID(Axis)) / Axsis_ParPosChange - offsetPoint;
		
		if(m_InterpCurveFlag == INTER_CURVE_NO || m_InterpAxisMoveFlag[X_Axsis] > 0 || m_InterpAxisMoveFlag[L_Axsis] > 0 || m_InterpAxisMoveFlag[Z_Axsis] > 0 \
			|| m_InterpAxisMoveFlag[O_Axsis] > 0 || m_InterpAxisMoveFlag[U_Axsis] > 0 || m_InterpAxisMoveFlag[V_Axsis] > 0)
		{//����ִ�в岹�˶�������дflash
			JDZ_ZeroPos_ParSave = 1;
		}
		else
		{
			for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
			{
				temp[0 + i * 4] = (u8)(JDZ_ZeroPosition[i] & 0x000000FF);	
				temp[1 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0x0000FF00)>>8);	
				temp[2 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0x00FF0000)>>16);	
				temp[3 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0xFF000000)>>24);
			}
			W25QXX_Write(temp, P_SERVO_JDZ_ZERO_HEAD, P_SERVO_JDZ_ZERO_LEN);
		}
		
		m_PositionResetCounter[Axis] = 0;
		m_PositionResetStep[Axis] = 1;
	}
	
	if(m_PositionResetStep[Axis] == 1)
	{
		if(m_PositionResetCounter[Axis] > 20)
		{//��㸴λ��ʱ�����������
			return 1;
		}
		else
		{
			m_PositionResetCounter[Axis]++;
			delay_ms(1);
		}
	}
	
	return 0;
}

/*
 * ��е�����������Ϊ���
 */
u8 PositionSetZero(u8 Axis)
{
	u16 i = 0;
	u8 temp[32] = {0};

	JDZ_ZeroPosition[Axis] = 0;
	
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		temp[0 + i * 4] = (u8)(JDZ_ZeroPosition[i] & 0x000000FF);
		temp[1 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0x0000FF00)>>8);
		temp[2 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0x00FF0000)>>16);
		temp[3 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0xFF000000)>>24);
	}
	W25QXX_Write(temp, P_SERVO_JDZ_ZERO_HEAD, P_SERVO_JDZ_ZERO_LEN);
	
	return 0;
}

/*
 * 485���ݴ�����
 */
void JDZ_OrderDecoding()
{
	u16 i = 0;
	
	for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
	{
		if(PDO_FirstGetPosition[i] == 1)
		{
			m_PulseTotalCounter[i] = PDO_Cur_Position[i] + MINROBOTPOSITION;

			if(First_Get_Position_Flag[i] == 0 && JDZ_Parameter.OriginSetting[i] == 1)
			{
				First_Get_Position_Flag[i] = 1;
			}
			
			PDO_FirstGetPosition[i] = 0;
		}
		else
		{
			if(ServoGetCanBoffSta(i) == 0)
			{//δ���ӵ��ŷ���������Ϊ��Сֵ
				m_PulseTotalCounter[i] = MINROBOTPOSITION;
			}
		}
	}
	
	if(JDZ_Parameter.Switch == 1)
	{//����ֵģʽʱ��Ҫ����
		if(Origin_Backed == FALSE)
		{
			Axis_Set_Origin();
		}
		
		JDZ_Origin_Set();
	}
}			




