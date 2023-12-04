/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Parameter.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/25/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f4xx.h"
#include "Parameter.h"
#include "Usart.h"
#include "Manual.h"
#include "w25qxx.h"
#include "SpeedControl.h"
#include "StatusControl.h"
#include "Auto.h"
#include "Auto_2.h"
#include "SignalWatch.h"
#include "BackToOrigin.h" 
#include "Error.h"
#include "ActionOperate.h"
#include "JDZ.h" 
#include "MD.h" 
#include "stmflash.h"

u32 Axsis_Maxlength[Axis_Num + Ext_Axis_Num] = {MAXROBOTPOSITION,MAXROBOTPOSITION,MAXROBOTPOSITION,MAXROBOTPOSITION,MAXROBOTPOSITION,MAXROBOTPOSITION};//轴运动的最大位置
u32 Axsis_Minlength[Axis_Num + Ext_Axis_Num] = {MINROBOTPOSITION,MINROBOTPOSITION,MINROBOTPOSITION,MINROBOTPOSITION,MINROBOTPOSITION,MINROBOTPOSITION};//轴运动的最小位置

SoftLimit Robot_SoftLimit[SAVESOFTLIMIT]={0};		                	//XZ软限位参数
SafeAreas Robot_Safe_Area[SAVESAFEAREA]={0};	        						//安全区域参数
JXSParameter JXS_Parameter = {0,0,0,0,0,100,5};		    						//机械手参数
SCParameter SC_Parameter = {0};					        									//生产参数
USERParameter USER_Parameter = {0};		               	            //用户变量

IONameParameter Input_Name[INPUT_NUM] = {0};											//输入口名称保存
IONameParameter Output_Name[OUTPUT_NUM] = {0};
u8 Temp_IO_Switch_Parameter[INPUT_NUM] = {0};											//当前IO口功能标志，0，普通io口；1，限位io

u8 Temp_OUT_Switch_Parameter[OUTPUT_NUM] = {0};										//当前IO口功能标志，0，普通io口；1，状态io

JDZParameter JDZ_Parameter = {0,0,0,{10000},{0}};									//绝对值参数设定

u16 Temp_Num = 0;		                                							//程序编号的临时规模计数

SoftDistance  Robot_SoftDistance= {0};
SoftDistance  Temp_SoftDistance= {0};

SaveProgramIICAddress Program_IIC_Address[SAVEPROGRAMNUM] = {	  	//程序IIC存储的地址，0xE00一个程序
	{0,1,0,0,0,0,0x14100}, {0,2,0,0,0,0,0x14F00}, {0,3,0,0,0,0,0x15D00}, {0,4,0,0,0,0,0x16B00},
	{0,5,0,0,0,0,0x17900}, {0,6,0,0,0,0,0x18700}, {0,7,0,0,0,0,0x19500}, {0,8,0,0,0,0,0x1A300},
	{0,9,0,0,0,0,0x1B100}, {0,10,0,0,0,0,0x1BF00},{0,11,0,0,0,0,0x1CD00},{0,12,0,0,0,0,0x1DB00},
	{0,13,0,0,0,0,0x1E900},{0,14,0,0,0,0,0x1F700},{0,15,0,0,0,0,0x20500},{0,16,0,0,0,0,0x21300},
	{0,17,0,0,0,0,0x22100},{0,18,0,0,0,0,0x22F00},{0,19,0,0,0,0,0x23D00},{0,20,0,0,0,0,0x24B00}};

u32 SaveProgram_IIC_Address = 0;                      						//记录当前程序要保存的地址
u8  SaveProgram_IIC_Num = 0;                          						//记录当前程序要保存的数量

SaveProgram Free_Program_Operate = {0};														//当前选中的主程序
SaveProgram SubProgram_Operate[SAVEPROGRAMNUM_SUB] = {{0}};				//当前选中的子程序

u8 Start_Recieve_Program_Flag = FALSE;                  					//开始接收程序标志位
u8 Program_From_UDisk_Flag = FALSE;                     					//程序来自U盘标志
u8 Current_Delete_Program = 0;	                        					//当前删除程序
u8 g_Run_Program_Num = 0;			                    								//当前运行的程序编号,0表示无选择程序，-1可做数组下标
u8 g_Run_Program_Num_Pre = 0;																			//用于保持当前选中的程序号
u8 All_Program_Deleted_Flag = 0;																	//恢复出厂设置所有程序已经删除标志位

u8  Search_Time[12] = {0};                                 				//记录搜索次数
u16 m_Copy_Time = 0;
u8  Send_Time = 0;                                        				//单个程序发送次数
u32 Temp_Search_Time = 0;																					//存储点的下发次数计数
u8  Temp_JXS_Parameter_SpeedLevel = 0;														//速度等级暂存变量，用于实现在运动时修改速度等级

SavePoint Manul_Save_Point[ManulSavePointMaxNum]={0};	  														//保存点的数组，Flag、Name、P-X，其中Flag表示是否有点保存
float Step_Coefficient[Axis_Num + EXTEN_AXIS_NUM]={0};

u32 m_SystemTimeCounter = 0;																			//系统时钟计数器
u16 Torque_T_count = 0;  //速度为0时力矩改变周期计数

/*插补运动时，需要写FLASH的参数用到的标志*/
u8 SpeedLevel_ParSave = 0;								//速度等级修改标志
u8 JDZ_ZeroPos_ParSave = 0;								//绝对位置设置标志

/*码垛相关参数*/
IONameParameter sMD_Name = {0};						//码垛名称-用于恢复出厂设置
ST_MDParameter 	sMD_Parameter = {0};   		//码垛参数
ST_MDRunPara 		sMD_RunPara = {0};				//码垛运行参数
u8 sMD_CurMDCode = 0;											//当前物品编号
u8 sMD_GoodOffset_flag = 0;								//当前物品-物料偏移标志
u8 All_MD_Deleted_Flag = 0;								//恢复出厂设置所有码垛已经删除标志位
u8 sMD_FlashCurLayer[MD_GOOD_NUM] = {0};  //各个码垛的当前层
u8 sMD_FlashCurNum[MD_GOOD_NUM] = {0};		//各个码垛的当前个

u8 g_Write_FlashFlag = 0;				  				//产量等参数写flash标志位

//笛卡尔坐标系相关参数
ST_Cartesian sCartesian_Para = {0};

//电机控制PID相关参数
//ST_MotorControl_PID sMC_PID_Para = {0};

//配方参数
PFParameter PF_Parameter = {0};																			

/*总线电机速度、加速度、位置的转换系数*/
float  Axsis_ParVelChange = 1.0f;																		//速度、加速度转换系数
float  Axsis_ParPosChange = 1.0f;																		//位置转换系数

InternetParameter Internet_Parameter = {0};													//物联网功能设置

ExtendAixParameter ExtendAix_Parameter[EXTEN_AXIS_NUM] = {0};				//扩展轴参数

/**************************************************************************************************
**  函数名：  AxsisEncoderBit()
**	输入参数：
**	输出参数：
**	函数功能：返回不同编码器位数对应的值
**	备注：	  无
**  作者：         
**  开发日期：
***************************************************************************************************/
u32 AxsisEncoderBit(void)
{
	switch(JDZ_Parameter.Resolu)
	{
		case 0://17位编码器
			return 131072;
		case 1://18位编码器
			return 262144;
		case 2://21位编码器
			return 2097152;
		case 3://23位编码器
			return 8388608;
		default://默认17位编码器
			return 131072;
	}
}

/**************************************************************************************************
**  函数名：  AxsisMoveCoefChange()
**	输入参数：
**	输出参数：
**	函数功能：计算速度、加速度、距离的参数
**	备注：	  无
**  作者：         
**  开发日期：
***************************************************************************************************/
void AxsisMoveCoefChange(void)
{
	switch(JDZ_Parameter.Server)
	{
		case 0://禾川电机
			Axsis_ParVelChange = (float)(AxsisEncoderBit() / 60);
			Axsis_ParPosChange = ((float)(AxsisEncoderBit()) / 10000);
			break;
		case 1://汇川电机
			Axsis_ParVelChange = (float)(AxsisEncoderBit() / 60);
			Axsis_ParPosChange = ((float)(AxsisEncoderBit()) / 10000);
			break;
		case 2://迈信电机
			Axsis_ParVelChange = (float)(AxsisEncoderBit() / 60);
			Axsis_ParPosChange = ((float)(AxsisEncoderBit()) / 10000);
			break;
		case 3://雷赛电机
			Axsis_ParVelChange = (float)(10000 / 60);
			Axsis_ParPosChange = 1.0;//((float)(AxsisEncoderBit()) / 10000);
			break;
		case 4://信捷电机
			Axsis_ParVelChange = (float)(AxsisEncoderBit() / 60);
			Axsis_ParPosChange = ((float)(AxsisEncoderBit()) / 10000);
			break;
		case 5://台邦电机
			Axsis_ParVelChange = 10.0f;																		//速度单位0.1rpm
			Axsis_ParPosChange = ((float)(AxsisEncoderBit()) / 10000);		//齿轮比设置
			break;
		case 6://超川电机
			Axsis_ParVelChange = 10.0f;																		//速度单位0.1rpm
			Axsis_ParPosChange = ((float)(AxsisEncoderBit()) / 10000);		//齿轮比设置
			break;
		default:
			Axsis_ParVelChange = (float)(AxsisEncoderBit() / 60);
			Axsis_ParPosChange = ((float)(AxsisEncoderBit()) / 10000);
			break;
	}
}

/**************************************************************************************************
**  函数名：  DistanceTransPulse()
**	输入参数：轴编号、距离
**	输出参数：距离对应的脉冲数
**	函数功能：距离转化为脉冲
**	备注：	  无
**  作者：         
**  开发日期：
***************************************************************************************************/
u32 DistanceTransPulse(u8 Axis,u32 Distance)
{
	return (u32)(Distance * ((double)JXS_Parameter.A_Circle_Pulse[Axis] / JXS_Parameter.A_Circle_Distance[Axis])+MINROBOTPOSITION);
}

/**************************************************************************************************
**  函数名：  DistanceToPulse()
**	输入参数：无
**	输出参数：无
**	函数功能：距离转化为脉冲
**	备注：	  改变齿轮比之后应用
**  作者：        
**  开发日期：
***************************************************************************************************/
void DistanceToPulse()
{
	u8 i=0;
	u8 IIC_Parameter[30] = {0};						//数据读取时使用的中间变量
	
	//读取软限位参数
	for(i = 0; i < SAVESOFTLIMIT + 1; i++)
	{
		if(i < 4)
		{
			W25QXX_Read(IIC_Parameter, 0x2000+i*0x09, 9);
			Robot_SoftLimit[i].Left_Limit  = (u32)(((u32)IIC_Parameter[0])|((u32)IIC_Parameter[1]<<8)|((u32)IIC_Parameter[2]<<16)|((u32)IIC_Parameter[3]<<24));
			Robot_SoftLimit[i].Right_Limit = (u32)(((u32)IIC_Parameter[4])|((u32)IIC_Parameter[5]<<8)|((u32)IIC_Parameter[6]<<16)|((u32)IIC_Parameter[7]<<24));
			Robot_SoftLimit[i].Switch_Limit = IIC_Parameter[8];
			
			if(Robot_SoftLimit[i].Switch_Limit)         //软限位开
			{
				Axsis_Minlength[i] = (Robot_SoftLimit[i].Left_Limit * Step_Coefficient[i] / 100)+MINROBOTPOSITION;
				Axsis_Maxlength[i] = (Robot_SoftLimit[i].Right_Limit * Step_Coefficient[i] / 100)+MINROBOTPOSITION;
			}
			else
			{
				Axsis_Minlength[i] = MINROBOTPOSITION;
				Axsis_Maxlength[i] = MAXROBOTPOSITION;
			}
		}
		else if(i == 4)
		{
			W25QXX_Read(IIC_Parameter,0x2030,8);		
			Robot_SoftDistance.MaxDistance = (u32)(((u32)IIC_Parameter[0])|((u32)IIC_Parameter[1]<<8)|((u32)IIC_Parameter[2]<<16)|((u32)IIC_Parameter[3]<<24));
			Robot_SoftDistance.MinDistance = (u32)(((u32)IIC_Parameter[4])|((u32)IIC_Parameter[5]<<8)|((u32)IIC_Parameter[6]<<16)|((u32)IIC_Parameter[7]<<24));
			
			Robot_SoftDistance.MaxDistance = (Robot_SoftDistance.MaxDistance * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
			Robot_SoftDistance.MinDistance = (Robot_SoftDistance.MinDistance * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
		}
	}
	
	//读取安全区
	for(i = 0; i<SAVESAFEAREA; i++)                        
	{
		W25QXX_Read(IIC_Parameter,0x2100+i*0x20,17);
		Robot_Safe_Area[i].X_Left = (u32)(((u32)IIC_Parameter[0])|((u32)IIC_Parameter[1]<<8)|((u32)IIC_Parameter[2]<<16)|((u32)IIC_Parameter[3]<<24));
		Robot_Safe_Area[i].X_Left = (Robot_Safe_Area[i].X_Left * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;		
													  
		Robot_Safe_Area[i].Z_Up = (u32)(((u32)IIC_Parameter[4])|((u32)IIC_Parameter[5]<<8)|((u32)IIC_Parameter[6]<<16)|((u32)IIC_Parameter[7]<<24));
		Robot_Safe_Area[i].Z_Up = (Robot_Safe_Area[i].Z_Up * Step_Coefficient[Z_Axsis] / 100) + MINROBOTPOSITION;	
		
		Robot_Safe_Area[i].X_Right = (u32)(((u32)IIC_Parameter[8])|((u32)IIC_Parameter[9]<<8)|((u32)IIC_Parameter[10]<<16)|((u32)IIC_Parameter[11]<<24));
		Robot_Safe_Area[i].X_Right = (Robot_Safe_Area[i].X_Right * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
		
		Robot_Safe_Area[i].Z_Down = (u32)(((u32)IIC_Parameter[12])|((u32)IIC_Parameter[13]<<8)|((u32)IIC_Parameter[14]<<16)|((u32)IIC_Parameter[15]<<24));
		Robot_Safe_Area[i].Z_Down = (Robot_Safe_Area[i].Z_Down * Step_Coefficient[Z_Axsis] / 100) + MINROBOTPOSITION;
		
		Robot_Safe_Area[i].SafeArea_Switch = IIC_Parameter[16];
	}
	
	W25QXX_Read(IIC_Parameter,0x1230,5);  //Z轴防撞
	JXS_Parameter.ZAxsisAvoidace = IIC_Parameter[0];
	JXS_Parameter.ZAxsisLimit = (u32)(((u32)IIC_Parameter[1])|((u32)IIC_Parameter[2]<<8)|((u32)IIC_Parameter[3]<<16)|((u32)IIC_Parameter[4]<<24));
	JXS_Parameter.ZAxsisLimit = (JXS_Parameter.ZAxsisLimit * Step_Coefficient[Z_Axsis] / 100) + MINROBOTPOSITION;
}

/**************************************************************************************************
**  函数名：  AxisInterpParSave()
**	输入参数：无
**	输出参数：无
**	函数功能：插补运动时，需要写FLASH的参数必须放到这个函数中保存
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void AxisInterpParSave(void)
{
	u16 i = 0;
	u8 temp[20] = {0};
	
	if(m_InterpCurveFlag == INTER_CURVE_NO && m_InterpAxisMoveFlag[X_Axsis] == 0 && m_InterpAxisMoveFlag[L_Axsis] == 0 \
			&& m_InterpAxisMoveFlag[Z_Axsis] == 0 && m_InterpAxisMoveFlag[O_Axsis] == 0 && m_InterpAxisMoveFlag[U_Axsis] == 0 && m_InterpAxisMoveFlag[V_Axsis] == 0)
	{//插补运动执行完成后，可以写FLASH
		if(SpeedLevel_ParSave == 1)
		{//插补运动中的速度修改保存
			W25QXX_Write(&Temp_JXS_Parameter_SpeedLevel, 0x10B0 + 2, 1);
			
			SpeedLevel_ParSave = 0;
		}
		
		if(JDZ_ZeroPos_ParSave == 1)
		{//插补运动中的绝对位置修改
			for(i=0; i<Axis_Num + Ext_Axis_Num; i++)
			{
				temp[0 + i * 4] = (u8)(JDZ_ZeroPosition[i] & 0x000000FF);	
				temp[1 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0x0000FF00)>>8);	
				temp[2 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0x00FF0000)>>16);	
				temp[3 + i * 4] = (u8)((JDZ_ZeroPosition[i] & 0xFF000000)>>24);
			}
			W25QXX_Write(temp, P_SERVO_JDZ_ZERO_HEAD, P_SERVO_JDZ_ZERO_LEN);
			
			JDZ_ZeroPos_ParSave = 0;
		}
	}
}

/**************************************************************************************************
**  函数名：  ParameterOrder()
**	输入参数：无
**	输出参数：无
**	函数功能：参数命令：软限位、安全区、功能设定参数
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void ParameterOrder()
{
	u8 Code = 0;
	u8 Layer = 0;
	u8 Pcode = 0;
	u16 i = 0;
	u16 j = 0;
	u32 address1 = 0;
	u32 address2 = 0;
	u8 USER_Code = 0;
	u8 Selected_IO = 0;			//选中的IO编号
	u8 Detect_OUTPUT = 0;			//选中的输出IO编号
	u8 Selected_InPut_Num = 0;	//选中的输入IO编号
	u8 Selected_OutPut_Num = 0;	//选中的输出IO编号
	u8 data_temp[64] = {0};
//	u32 temp = 0;
	u16 axisnum = 0;
	
	switch(UsartReceiveData[1])
	{
		case P_PARAMETER_SOFT_LIMIT://最大软件限位行程设定 0x2000
			if(UsartReceiveData[2] >= 0x01 && UsartReceiveData[2] <= 0x04)
			{//X轴软限位
				Robot_SoftLimit[UsartReceiveData[2] - 1].Left_Limit  = (u32)(((u32)UsartReceiveData[3])|((u32)UsartReceiveData[4]<<8)|((u32)UsartReceiveData[5]<<16)|((u32)UsartReceiveData[6]<<24));
				Robot_SoftLimit[UsartReceiveData[2] - 1].Right_Limit = (u32)(((u32)UsartReceiveData[7])|((u32)UsartReceiveData[8]<<8)|((u32)UsartReceiveData[9]<<16)|((u32)UsartReceiveData[10]<<24));
				Robot_SoftLimit[UsartReceiveData[2] - 1].Switch_Limit = UsartReceiveData[11];
				W25QXX_Write(&UsartReceiveData[3],0x2000 + (UsartReceiveData[2] - 1) * 9,9);
				if(Robot_SoftLimit[UsartReceiveData[2] - 1].Switch_Limit)         
				{//X轴软限位开
					Axsis_Minlength[UsartReceiveData[2] - 1] = (Robot_SoftLimit[UsartReceiveData[2] - 1].Left_Limit * Step_Coefficient[UsartReceiveData[2] - 1] / 100) + MINROBOTPOSITION;
					Axsis_Maxlength[UsartReceiveData[2] - 1] = (Robot_SoftLimit[UsartReceiveData[2] - 1].Right_Limit * Step_Coefficient[UsartReceiveData[2] - 1] / 100) + MINROBOTPOSITION;
				}
				else
				{
					Axsis_Minlength[UsartReceiveData[2] - 1] = MINROBOTPOSITION;
					Axsis_Maxlength[UsartReceiveData[2] - 1] = MAXROBOTPOSITION;
				}
			}
//			if(UsartReceiveData[2] == 0x01)
//			{//X轴软限位
//				Robot_SoftLimit[X_Axsis].Left_Limit  = (u32)(((u32)UsartReceiveData[3])|((u32)UsartReceiveData[4]<<8)|((u32)UsartReceiveData[5]<<16)|((u32)UsartReceiveData[6]<<24));
//				Robot_SoftLimit[X_Axsis].Right_Limit = (u32)(((u32)UsartReceiveData[7])|((u32)UsartReceiveData[8]<<8)|((u32)UsartReceiveData[9]<<16)|((u32)UsartReceiveData[10]<<24));
//				Robot_SoftLimit[X_Axsis].Switch_Limit = UsartReceiveData[11];
//				W25QXX_Write(&UsartReceiveData[3],0x2000,9);			   
//				if(Robot_SoftLimit[X_Axsis].Switch_Limit)         
//				{//X轴软限位开
//					Axsis_Minlength[X_Axsis] = (Robot_SoftLimit[X_Axsis].Left_Limit * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
//					Axsis_Maxlength[X_Axsis] = (Robot_SoftLimit[X_Axsis].Right_Limit * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
//				}
//				else
//				{
//					Axsis_Minlength[X_Axsis] = MINROBOTPOSITION;
//					Axsis_Maxlength[X_Axsis] = MAXROBOTPOSITION;
//				}
//			}
//			else if(UsartReceiveData[2] == 0x02)            	
//			{//L轴软限位
//				Robot_SoftLimit[L_Axsis].Left_Limit  = (u32)(((u32)UsartReceiveData[3])|((u32)UsartReceiveData[4]<<8)|((u32)UsartReceiveData[5]<<16)|((u32)UsartReceiveData[6]<<24));
//				Robot_SoftLimit[L_Axsis].Right_Limit = (u32)(((u32)UsartReceiveData[7])|((u32)UsartReceiveData[8]<<8)|((u32)UsartReceiveData[9]<<16)|((u32)UsartReceiveData[10]<<24));
//				Robot_SoftLimit[L_Axsis].Switch_Limit= UsartReceiveData[11];
//				W25QXX_Write(&UsartReceiveData[3],0x2009,9);			   
//				if(Robot_SoftLimit[L_Axsis].Switch_Limit)         
//				{//L轴软限位开
//					Axsis_Minlength[L_Axsis] = (Robot_SoftLimit[L_Axsis].Left_Limit * Step_Coefficient[L_Axsis] / 100) + MINROBOTPOSITION;
//					Axsis_Maxlength[L_Axsis] = (Robot_SoftLimit[L_Axsis].Right_Limit * Step_Coefficient[L_Axsis] / 100) + MINROBOTPOSITION;
//				}
//				else
//				{
//					Axsis_Minlength[L_Axsis] = MINROBOTPOSITION;
//					Axsis_Maxlength[L_Axsis] = MAXROBOTPOSITION;
//				}
//			}
//			else if(UsartReceiveData[2] == 0x03)
//			{//Z软限位
//				Robot_SoftLimit[Z_Axsis].Left_Limit  = (u32)(((u32)UsartReceiveData[3])|((u32)UsartReceiveData[4]<<8)|((u32)UsartReceiveData[5]<<16)|((u32)UsartReceiveData[6]<<24));
//				Robot_SoftLimit[Z_Axsis].Right_Limit = (u32)(((u32)UsartReceiveData[7])|((u32)UsartReceiveData[8]<<8)|((u32)UsartReceiveData[9]<<16)|((u32)UsartReceiveData[10]<<24));
//				Robot_SoftLimit[Z_Axsis].Switch_Limit = UsartReceiveData[11];
//				W25QXX_Write(&UsartReceiveData[3],0x2012,9);			   
//				if(Robot_SoftLimit[Z_Axsis].Switch_Limit)
//				{//Z轴软限位开
//					Axsis_Minlength[Z_Axsis] = (Robot_SoftLimit[Z_Axsis].Left_Limit * Step_Coefficient[Z_Axsis] / 100) + MINROBOTPOSITION;
//					Axsis_Maxlength[Z_Axsis] = (Robot_SoftLimit[Z_Axsis].Right_Limit * Step_Coefficient[Z_Axsis] / 100) + MINROBOTPOSITION;
//				}
//				else
//				{
//					Axsis_Minlength[Z_Axsis] = MINROBOTPOSITION;
//					Axsis_Maxlength[Z_Axsis] = MAXROBOTPOSITION;
//				}
//			}
//			else if(UsartReceiveData[2] == 0x04)
//			{//O
//				Robot_SoftLimit[O_Axsis].Left_Limit  = (u32)(((u32)UsartReceiveData[3])|((u32)UsartReceiveData[4]<<8)|((u32)UsartReceiveData[5]<<16)|((u32)UsartReceiveData[6]<<24));
//				Robot_SoftLimit[O_Axsis].Right_Limit = (u32)(((u32)UsartReceiveData[7])|((u32)UsartReceiveData[8]<<8)|((u32)UsartReceiveData[9]<<16)|((u32)UsartReceiveData[10]<<24));
//				Robot_SoftLimit[O_Axsis].Switch_Limit= UsartReceiveData[11];
//				W25QXX_Write(&UsartReceiveData[3],0x201B,9);			   
//				if(Robot_SoftLimit[O_Axsis].Switch_Limit)
//				{//O轴软限位开
//					Axsis_Minlength[O_Axsis] = (Robot_SoftLimit[O_Axsis].Left_Limit * Step_Coefficient[O_Axsis] / 100) + MINROBOTPOSITION;
//					Axsis_Maxlength[O_Axsis] = (Robot_SoftLimit[O_Axsis].Right_Limit * Step_Coefficient[O_Axsis] / 100) + MINROBOTPOSITION;
//				}
//				else
//				{
//					Axsis_Minlength[O_Axsis] = MINROBOTPOSITION;
//					Axsis_Maxlength[O_Axsis] = MAXROBOTPOSITION;
//				}
//			}
			else if(UsartReceiveData[2] == 0x05)
			{//X-O轴安全行程
				Robot_SoftDistance.MaxDistance = (u32)(((u32)UsartReceiveData[3])|((u32)UsartReceiveData[4]<<8)|((u32)UsartReceiveData[5]<<16)|((u32)UsartReceiveData[6]<<24));
				Robot_SoftDistance.MinDistance = (u32)(((u32)UsartReceiveData[7])|((u32)UsartReceiveData[8]<<8)|((u32)UsartReceiveData[9]<<16)|((u32)UsartReceiveData[10]<<24));
				W25QXX_Write(&UsartReceiveData[3],0x2030,8);
				Robot_SoftDistance.MaxDistance = (Robot_SoftDistance.MaxDistance * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
				Robot_SoftDistance.MinDistance = (Robot_SoftDistance.MinDistance * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
			}
			break;

		case P_PARAMETER_SAFE_AREA://安全区域参数 0x2100
			address1 = (UsartReceiveData[2]&0xf0)>>4;
			address2 = UsartReceiveData[2]&0x0f;
			Code = UsartReceiveData[3] - 1;
			if(address1 == 0x01)	                         
			{//左上
				if(address2 == 0x01)                         
				{//X轴安全区信息
					Robot_Safe_Area[Code].X_Left = (u32)(((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24));
					Robot_Safe_Area[Code].X_Left = (Robot_Safe_Area[Code].X_Left * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
					W25QXX_Write(&UsartReceiveData[4],0x2100+0x20*Code,4);
				}
				else			                                   
				{//Z轴安全区信息
					Robot_Safe_Area[Code].Z_Up = (u32)(((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24));
					Robot_Safe_Area[Code].Z_Up = (Robot_Safe_Area[Code].Z_Up * Step_Coefficient[Z_Axsis] / 100) + MINROBOTPOSITION;
					W25QXX_Write(&UsartReceiveData[4],0x2100+0x20*Code+0x04,4);
				}
			}
			else if(address1 == 0x02)
			{//右下
				if(address2 == 0x01)
				{//X轴安全区信息
					Robot_Safe_Area[Code].X_Right = (u32)(((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24));
					Robot_Safe_Area[Code].X_Right = (Robot_Safe_Area[Code].X_Right * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
					W25QXX_Write(&UsartReceiveData[4],0x2100+0x20*Code+0x08,4);
				}
				else
				{//Z安全区信息
					Robot_Safe_Area[Code].Z_Down = (u32)(((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24));
					Robot_Safe_Area[Code].Z_Down = (Robot_Safe_Area[Code].Z_Down * Step_Coefficient[Z_Axsis] / 100) + MINROBOTPOSITION;
					W25QXX_Write(&UsartReceiveData[4],0x2100+0x20*Code+0x0C,4);
				}
			}
			else	                                         
			{//安全区 启用-关闭
				Robot_Safe_Area[Code].SafeArea_Switch=UsartReceiveData[4];
				W25QXX_Write(&UsartReceiveData[4],0x2100+0x20*Code+0x10,1);
			}
			break;

		case P_PARAMETER_FUCTION_SET://功能设定参数 0x1000
			switch(UsartReceiveData[2])
			{
				case 0x01://机械手参数
					JXS_Parameter.Axis 	 = UsartReceiveData[3];
					JXS_Parameter.Origin = UsartReceiveData[4];
					if(g_AutoStatue == AUTO_RUNNING)
					{
						Temp_JXS_Parameter_SpeedLevel = UsartReceiveData[5];
					}
					else
					{
						JXS_Parameter.SpeedLevel = UsartReceiveData[5];
						Temp_JXS_Parameter_SpeedLevel = JXS_Parameter.SpeedLevel;
					}
					
					JXS_Parameter.AlarmSignal = UsartReceiveData[6];		//报警信号：低电平报警还是高电平报警
					JXS_Parameter.NcOrignin = UsartReceiveData[7];			//外部回零端口,X17
					JXS_Parameter.NcStartin = UsartReceiveData[8];			//外部启动端口,X18
					JXS_Parameter.NcPausein = UsartReceiveData[9];			//外部暂停端口,X19
					JXS_Parameter.NcStopin  = UsartReceiveData[10];			//外部停止端口,X20

					JXS_Parameter.LCcirculation = UsartReceiveData[11];
					
					for(i=0; i<Axis_Num; i++)
					{
						JXS_Parameter.AlarmSwitch[i]=UsartReceiveData[12 + i];
						JXS_Parameter.Accelerate.Time[i]=(u16)(((u16)UsartReceiveData[16 + i * 2])|((u16)UsartReceiveData[17 + i * 2]<<8));
					}
//					JXS_Parameter.AlarmSwitch[X_Axsis]=UsartReceiveData[12];
//					JXS_Parameter.AlarmSwitch[L_Axsis]=UsartReceiveData[13];
//					JXS_Parameter.AlarmSwitch[Z_Axsis]=UsartReceiveData[14];
//					JXS_Parameter.AlarmSwitch[O_Axsis]=UsartReceiveData[15];

//					JXS_Parameter.Accelerate.Time[X_Axsis]=(u16)(((u16)UsartReceiveData[16])|((u16)UsartReceiveData[17]<<8));
//					JXS_Parameter.Accelerate.Time[L_Axsis]=(u16)(((u16)UsartReceiveData[18])|((u16)UsartReceiveData[19]<<8));
//					JXS_Parameter.Accelerate.Time[Z_Axsis]=(u16)(((u16)UsartReceiveData[20])|((u16)UsartReceiveData[21]<<8));
//					JXS_Parameter.Accelerate.Time[O_Axsis]=(u16)(((u16)UsartReceiveData[22])|((u16)UsartReceiveData[23]<<8));
					
					JXS_Parameter.MDgripSwitch = UsartReceiveData[24];
					
					for(i=0; i<MDgrip_Num; i++)
					{
						JXS_Parameter.MDgripPort[i] = UsartReceiveData[25 + i];
					}
//					JXS_Parameter.MDgripPort[0] = UsartReceiveData[25];
//					JXS_Parameter.MDgripPort[1] = UsartReceiveData[26];
//					JXS_Parameter.MDgripPort[2] = UsartReceiveData[27];
//					JXS_Parameter.MDgripPort[3] = UsartReceiveData[28];
//					JXS_Parameter.MDgripPort[4] = UsartReceiveData[29];
//					JXS_Parameter.MDgripPort[5] = UsartReceiveData[30];
					JXS_Parameter.OutputAssociate[0] = UsartReceiveData[31];
					JXS_Parameter.OutputAssociate[1] = UsartReceiveData[32];
					JXS_Parameter.ZAxsisAvoidace = UsartReceiveData[33];
					JXS_Parameter.ZAxsisLimit = (u32)(((u32)UsartReceiveData[34])|((u32)UsartReceiveData[35]<<8)|((u32)UsartReceiveData[36]<<16)|((u32)UsartReceiveData[37]<<24));
//					for(i=0; i<Axis_Num; i++)
//					{
//						ServoAccDecSet(i);
//					}
					if(m_InterpLenAxis < Axis_Num || m_InterpAxisMoveFlag[X_Axsis] > 0 || m_InterpAxisMoveFlag[L_Axsis] > 0 \
							|| m_InterpAxisMoveFlag[Z_Axsis] > 0 || m_InterpAxisMoveFlag[O_Axsis] || m_InterpAxisMoveFlag[U_Axsis] || m_InterpAxisMoveFlag[V_Axsis] > 0)
					{//正在执行插补运动，不能写flash
						SpeedLevel_ParSave = 1;
					}
					else
					{
						W25QXX_Write(&UsartReceiveData[3],0x10B0,30);
						W25QXX_Write(&UsartReceiveData[33],0x1230,5);
					}
					JXS_Parameter.ZAxsisLimit = (JXS_Parameter.ZAxsisLimit * Step_Coefficient[Z_Axsis] / 100) + MINROBOTPOSITION;	//因为各个轴最小为MINROBOTPOSITION，所以防撞下限也加上
					break;

				case 0x02:
					sCartesian_Para.length[0] = (u32)(((u32)UsartReceiveData[3])|((u32)UsartReceiveData[4]<<8)|((u32)UsartReceiveData[5]<<16)|((u32)UsartReceiveData[6]<<24));
					sCartesian_Para.length[1] = (u32)(((u32)UsartReceiveData[7])|((u32)UsartReceiveData[8]<<8)|((u32)UsartReceiveData[9]<<16)|((u32)UsartReceiveData[10]<<24));
					for(i=0; i<Axis_Num; i++)
					{
//						sCartesian_Para.startPoint[i] = (s32)(((u32)UsartReceiveData[11+i*12])|((u32)UsartReceiveData[12+i*12]<<8)|((u32)UsartReceiveData[13+i*12]<<16)|((u32)UsartReceiveData[14+i*12]<<24));
//						sCartesian_Para.revolveAngle[i] = (s32)(((u32)UsartReceiveData[15+i*12])|((u32)UsartReceiveData[16+i*12]<<8)|((u32)UsartReceiveData[17+i*12]<<16)|((u32)UsartReceiveData[18+i*12]<<24));
//						sCartesian_Para.revolvePoint[i] = (s32)(((u32)UsartReceiveData[19+i*12])|((u32)UsartReceiveData[20+i*12]<<8)|((u32)UsartReceiveData[21+i*12]<<16)|((u32)UsartReceiveData[22+i*12]<<24));
						
						sCartesian_Para.axisType[i] = UsartReceiveData[61 + i];
						
						sCartesian_Para.axisBackMinDir[i] = UsartReceiveData[69 + i];
						sCartesian_Para.axisInterpFlag[i] = UsartReceiveData[73 + i];
					}
					sCartesian_Para.carCoordSwitch = UsartReceiveData[59];
					sCartesian_Para.MDCoordType = UsartReceiveData[60];
//					for(i=0; i<Axis_Num; i++)
//					{
//						sCartesian_Para.axisType[i] = UsartReceiveData[61 + i];
//					}
					sCartesian_Para.pitchLength = (u32)(((u32)UsartReceiveData[65])|((u32)UsartReceiveData[66]<<8)|((u32)UsartReceiveData[67]<<16)|((u32)UsartReceiveData[68]<<24));
//					for(i=0; i<Axis_Num; i++)
//					{
//						sCartesian_Para.axisBackMinDir[i] = UsartReceiveData[69 + i];
//						sCartesian_Para.axisInterpFlag[i] = UsartReceiveData[73 + i];
//					}
					W25QXX_Write(&UsartReceiveData[3],P_CARTESIAN_PARA_HEAD,P_CARTESIAN_PARA_LEN);
					break;

				case 0x03:
//					for(i=0; i<Axis_Num; i++)
//					{
//						temp = 300 + (u32)(((u32)UsartReceiveData[3 + i * 4])|((u32)UsartReceiveData[4 + i * 4]<<8)|((u32)UsartReceiveData[5 + i * 4]<<16)|((u32)UsartReceiveData[6 + i * 4]<<24));
//						if(temp > 2300) temp = 2300;
//						sMC_PID_Para.MC_P[i] = temp / 10000.0f;
//						temp = (u32)(((u32)UsartReceiveData[19 + i * 4])|((u32)UsartReceiveData[20 + i * 4]<<8)|((u32)UsartReceiveData[21 + i * 4]<<16)|((u32)UsartReceiveData[22 + i * 4]<<24));
//						temp = 0;
//						sMC_PID_Para.MC_I[i] = temp / 10000.0f;
//						temp = 100 + (u32)(((u32)UsartReceiveData[35 + i * 4])|((u32)UsartReceiveData[36 + i * 4]<<8)|((u32)UsartReceiveData[37 + i * 4]<<16)|((u32)UsartReceiveData[38 + i * 4]<<24));
//						if(temp > 1100) temp = 1100;
//						sMC_PID_Para.MC_D[i] = temp / 10000.0f;
//					}
//					W25QXX_Write(&UsartReceiveData[3], P_MOTORCONTROL_PARA_HEAD, P_MOTORCONTROL_PARA_LEN);
					break;

				case 0x04:
					break;

				case 0x05:
					break;

				case 0x06:	//生产参数
					SC_Parameter.RW_Num = (u32)(((u32)UsartReceiveData[3])|((u32)UsartReceiveData[4]<<8)|((u32)UsartReceiveData[5]<<16)|((u32)UsartReceiveData[6]<<24));
					SC_Parameter.CJ_Num = (u32)(((u32)UsartReceiveData[7])|((u32)UsartReceiveData[8]<<8)|((u32)UsartReceiveData[9]<<16)|((u32)UsartReceiveData[10]<<24));
					SC_Parameter.JG_Num = (u32)(((u32)UsartReceiveData[11])|((u32)UsartReceiveData[12]<<8)|((u32)UsartReceiveData[13]<<16)|((u32)UsartReceiveData[14]<<24));
					SC_Parameter.SC_Num = (u32)(((u32)UsartReceiveData[15])|((u32)UsartReceiveData[16]<<8)|((u32)UsartReceiveData[17]<<16)|((u32)UsartReceiveData[18]<<24));
					SC_Parameter.LJ_Num = (u32)(((u32)UsartReceiveData[19])|((u32)UsartReceiveData[20]<<8)|((u32)UsartReceiveData[21]<<16)|((u32)UsartReceiveData[22]<<24));
					SC_Parameter.NG_Num = (u32)(((u32)UsartReceiveData[23])|((u32)UsartReceiveData[24]<<8)|((u32)UsartReceiveData[25]<<16)|((u32)UsartReceiveData[26]<<24));
					sMD_RunPara.mdMethed = UsartReceiveData[27];
					sMD_RunPara.totalGood = UsartReceiveData[28];
					sMD_RunPara.startGood = UsartReceiveData[29];
					sMD_RunPara.curGood = UsartReceiveData[30];
					sMD_RunPara.curLayer = UsartReceiveData[31];
					sMD_RunPara.curNum = UsartReceiveData[32];
					if(sMD_RunPara.mdMethed == 1)
					{//分拣模式
						sMD_FlashCurLayer[sMD_RunPara.curGood - 1] = UsartReceiveData[31];
						sMD_FlashCurNum[sMD_RunPara.curGood - 1] = UsartReceiveData[32];
					}
					W25QXX_Write(&UsartReceiveData[27],P_SC_NUM_ADDRESS,3);
					STMFLASH_WriteRunData();
					break;

				case 0x07:      //原点参数
					for(i=0; i<Axis_Num; i++)
					{
						JXS_Parameter.OriginDir[i] = UsartReceiveData[i+3];							//轴回零方向
						JXS_Parameter.AxisOriginSpeed[i] = UsartReceiveData[i+7];				//轴回零速度
//						Axsis_Origin_Speed[i] = JXS_Parameter.AxisOriginSpeed[i];				//初始化原点回零速度
						JXS_Parameter.OriginPos[i] = UsartReceiveData[i+11];						//轴原点位置
						//轴回零偏移
						JXS_Parameter.OrignOffset[i] = (u32)(((u32)UsartReceiveData[i*4+15])|((u32)UsartReceiveData[i*4+16]<<8)|((u32)UsartReceiveData[i*4+17]<<16)|((u32)UsartReceiveData[i*4+18]<<24));
						JXS_Parameter.A_Circle_Pulse[i] = (u32)(((u32)UsartReceiveData[i*4+31])|((u32)UsartReceiveData[i*4+32]<<8)|((u32)UsartReceiveData[i*4+33]<<16)|((u32)UsartReceiveData[i*4+34]<<24));	//单圈位置
						JXS_Parameter.A_Circle_Distance[i] = (u32)(((u32)UsartReceiveData[i*4+47])|((u32)UsartReceiveData[i*4+48]<<8)|((u32)UsartReceiveData[i*4+49]<<16)|((u32)UsartReceiveData[i*4+50]<<24));//单圈脉冲
						
						Step_Coefficient[i] = (float)JXS_Parameter.A_Circle_Pulse[i]/((float)JXS_Parameter.A_Circle_Distance[i]/1000);
//						JXS_Parameter.OrignOffset[i] = JXS_Parameter.OrignOffset[i] * Step_Coefficient[i] / 100;
					}			
					DistanceToPulse();
					W25QXX_Write(&UsartReceiveData[3],0x10D0,28);	//连续存不下，所以分开存
					W25QXX_Write(&UsartReceiveData[31],0x1210,32);
					break;
					
				case 0x08:		//绝对值参数
					JDZ_Parameter.Switch = UsartReceiveData[3];	//功能开关
					JDZ_Parameter.Server = UsartReceiveData[4];	//伺服选择
					JDZ_Parameter.Resolu = UsartReceiveData[5];	//编码器分辨率
				
					for(i=0; i<Axis_Num; i++)
					{
						JDZ_Parameter.Circle_Pluse[i] = (u32)(((u32)UsartReceiveData[6 + i * 4])|((u32)UsartReceiveData[7 + i * 4]<<8)|((u32)UsartReceiveData[8 + i * 4]<<16)|((u32)UsartReceiveData[9 + i * 4]<<24));
						JDZ_Parameter.Motion_Dir[i]	=  UsartReceiveData[22 + i];	
						JDZ_Parameter.OriginSetting[i]=  UsartReceiveData[26 + i];	
					}

//					JDZ_Parameter.Circle_Pluse[X_Axsis] = (u32)(((u32)UsartReceiveData[6])|((u32)UsartReceiveData[7]<<8)|((u32)UsartReceiveData[8]<<16)|((u32)UsartReceiveData[9]<<24));
//					JDZ_Parameter.Circle_Pluse[L_Axsis] = (u32)(((u32)UsartReceiveData[10])|((u32)UsartReceiveData[11]<<8)|((u32)UsartReceiveData[12]<<16)|((u32)UsartReceiveData[13]<<24));
//					JDZ_Parameter.Circle_Pluse[Z_Axsis] = (u32)(((u32)UsartReceiveData[14])|((u32)UsartReceiveData[15]<<8)|((u32)UsartReceiveData[16]<<16)|((u32)UsartReceiveData[17]<<24));
//					JDZ_Parameter.Circle_Pluse[O_Axsis] = (u32)(((u32)UsartReceiveData[18])|((u32)UsartReceiveData[19]<<8)|((u32)UsartReceiveData[20]<<16)|((u32)UsartReceiveData[21]<<24));
//					JDZ_Parameter.Motion_Dir[X_Axsis]	=  UsartReceiveData[22];	
//					JDZ_Parameter.Motion_Dir[L_Axsis]	=  UsartReceiveData[23];	
//					JDZ_Parameter.Motion_Dir[Z_Axsis]	=  UsartReceiveData[24];	
//					JDZ_Parameter.Motion_Dir[O_Axsis]	=  UsartReceiveData[25];
//					JDZ_Parameter.OriginSetting[X_Axsis]=  UsartReceiveData[26];	
//					JDZ_Parameter.OriginSetting[L_Axsis]=  UsartReceiveData[27];	
//					JDZ_Parameter.OriginSetting[Z_Axsis]=  UsartReceiveData[28];	
//					JDZ_Parameter.OriginSetting[O_Axsis]=  UsartReceiveData[29];
					AxsisMoveCoefChange();
					W25QXX_Write(&UsartReceiveData[3],0x1100,27);
					break;
					
				case 0x09:		//用户变量	
					USER_Code = UsartReceiveData[3]-1;
					USER_Parameter.USER_Name1[USER_Code] = (u32)(((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24));
					USER_Parameter.USER_Name2[USER_Code] = (u32)(((u32)UsartReceiveData[8])|((u32)UsartReceiveData[9]<<8)|((u32)UsartReceiveData[10]<<16)|((u32)UsartReceiveData[11]<<24));
					USER_Parameter.USER_Name3[USER_Code] = (u32)(((u32)UsartReceiveData[12])|((u32)UsartReceiveData[13]<<8)|((u32)UsartReceiveData[14]<<16)|((u32)UsartReceiveData[15]<<24));
					USER_Parameter.INIT_Num[USER_Code] = (s32)(((u32)UsartReceiveData[16])|((u32)UsartReceiveData[17]<<8)|((u32)UsartReceiveData[18]<<16)|((u32)UsartReceiveData[19]<<24));
					USER_Parameter.CURR_Num[USER_Code] = (s32)(((u32)UsartReceiveData[20])|((u32)UsartReceiveData[21]<<8)|((u32)UsartReceiveData[22]<<16)|((u32)UsartReceiveData[23]<<24));
					USER_Parameter.ELEC_RESET[USER_Code]  = UsartReceiveData[24];
					USER_Parameter.START_RESET[USER_Code] = UsartReceiveData[25];
					W25QXX_Write(&UsartReceiveData[3],P_USER_ADDRESS + 23*USER_Code,23);
				  STMFLASH_WriteRunData();
					break;
				
				case 0x0A:	//配方参数
					for(i=0; i<PF_IONUM; i++)
					{
						PF_Parameter.pfIOnum[i]= UsartReceiveData[3 + i];//3-6
						PF_Parameter.pfGood[i]= UsartReceiveData[7 + i];//7-10
						PF_Parameter.pfSwitch[i]= UsartReceiveData[11 + i];//11-14
					}
					W25QXX_Write(&UsartReceiveData[3], P_FORMULATION_PARA_HEAD, P_FORMULATION_PARA_LEN);
					break;
				case 0x0B:   //物联网
					Internet_Parameter.Switch = UsartReceiveData[3];	//功能开关
					W25QXX_Write(&UsartReceiveData[3],P_INTERNET_ADDRESS,1);
					break;
				case 0x0C: //扩展轴参数
					axisnum = UsartReceiveData[3];
					ExtendAix_Parameter[axisnum].E_OriginPosition = UsartReceiveData[4];      //原点位置
					ExtendAix_Parameter[axisnum].E_OriginOffset = (u32)(((u32)UsartReceiveData[5])|((u32)UsartReceiveData[6]<<8)|((u32)UsartReceiveData[7]<<16)|((u32)UsartReceiveData[8]<<24));         //原点偏移
					ExtendAix_Parameter[axisnum].E_Circle_Pulse = (u32)(((u32)UsartReceiveData[9])|((u32)UsartReceiveData[10]<<8)|((u32)UsartReceiveData[11]<<16)|((u32)UsartReceiveData[12]<<24));      //单圈脉冲
					ExtendAix_Parameter[axisnum].E_Circle_Distance = (u32)(((u32)UsartReceiveData[13])|((u32)UsartReceiveData[14]<<8)|((u32)UsartReceiveData[15]<<16)|((u32)UsartReceiveData[16]<<24));  //单圈距离
					ExtendAix_Parameter[axisnum].E_AccAcc = (u16)( ((u16)UsartReceiveData[17])|((u16)UsartReceiveData[18]<<8) );            //加加速 
					ExtendAix_Parameter[axisnum].E_AccTime = (u16)( ((u16)UsartReceiveData[19])|((u16)UsartReceiveData[20]<<8) );           //加速时间
					ExtendAix_Parameter[axisnum].E_MaxDistance = (u32)(((u32)UsartReceiveData[21])|((u32)UsartReceiveData[22]<<8)|((u32)UsartReceiveData[23]<<16)|((u32)UsartReceiveData[24]<<24));       //最大行程
							
					if(UsartReceiveData[25] == 1 && ExtendAix_Parameter[axisnum].E_Origin_Set == 0)
					{//设置原点
						JDZ_Origin_Setting_Axis_Num = axisnum + Axis_Num + 1;
						JDZ_SetOrigin_Flag = TRUE;
					}
					ExtendAix_Parameter[axisnum].E_Origin_Set = UsartReceiveData[25];        //原点设置

					W25QXX_Write(&UsartReceiveData[4],P_EXTENDAIX_ADDRESS + axisnum * P_EXTENDAIX_PARA_SIZE, P_EXTENDAIX_PARA_SIZE);
					
				  Step_Coefficient[axisnum + Axis_Num] = ExtendAix_Parameter[axisnum].E_Circle_Pulse / (ExtendAix_Parameter[axisnum].E_Circle_Distance / 1000.0f);
					Axsis_Minlength[axisnum + Axis_Num] = MINROBOTPOSITION;
					Axsis_Maxlength[axisnum + Axis_Num] = (ExtendAix_Parameter[axisnum].E_MaxDistance * Step_Coefficient[axisnum + Axis_Num] / 100) + MINROBOTPOSITION;
					break;
			}
			break;
		case P_PARAMETER_IO_DETECT_SET://IO参数
			Selected_IO = UsartReceiveData[2];//被选中的是第几个IO口
			IO_Input_keepMin[Selected_IO] = (u16)(((u16)UsartReceiveData[3])|((u16)UsartReceiveData[4]<<8));
			IO_Input_keepMax[Selected_IO] = (u16)(((u16)UsartReceiveData[5])|((u16)UsartReceiveData[6]<<8));
			IO_Sign_On_Off[Selected_IO] = UsartReceiveData[7];
			Input_Name[Selected_IO].Name = (u32)(((u32)UsartReceiveData[8])|((u32)UsartReceiveData[9]<<8)|((u32)UsartReceiveData[10]<<16)|((u32)UsartReceiveData[11]<<24));
			Input_Name[Selected_IO].Name1 = (u32)(((u32)UsartReceiveData[12])|((u32)UsartReceiveData[13]<<8)|((u32)UsartReceiveData[14]<<16)|((u32)UsartReceiveData[15]<<24));
			Input_Name[Selected_IO].Name2 = (u32)(((u32)UsartReceiveData[16])|((u32)UsartReceiveData[17]<<8)|((u32)UsartReceiveData[18]<<16)|((u32)UsartReceiveData[19]<<24));
			
			IO_Input_keepMin[Selected_IO] = IO_Input_keepMin[Selected_IO]*10;
			IO_Input_keepMax[Selected_IO] = IO_Input_keepMax[Selected_IO]*10;
			//四路限位信号常开常闭
			if(Selected_IO >= 22 && Selected_IO <= 25)
			{
				JXS_Parameter.LimitSignOnOff[Selected_IO - 22] = IO_Sign_On_Off[Selected_IO];
			}
			//四路原点信号常开常闭
			if(Selected_IO >= 26 && Selected_IO <= 29)
			{
				JXS_Parameter.OrignSignOnOff[Selected_IO - 26] = IO_Sign_On_Off[Selected_IO];
			}
			W25QXX_Write(&UsartReceiveData[3],0x1120+5*Selected_IO,5);
			W25QXX_Write(&UsartReceiveData[8],0x1300+12*Selected_IO,12);
			break;
		case P_PARAMETER_OUTPUT_SET://输出IO参数
			Detect_OUTPUT = UsartReceiveData[2];//被选中的是第几个IO口
			OutPut_BeforeOrigin[Detect_OUTPUT] = UsartReceiveData[3];
			OutPut_AfterOrigin[Detect_OUTPUT] = UsartReceiveData[4];
			OutPut_Common_Alarm[Detect_OUTPUT] = UsartReceiveData[5];
			OutPut_Emerge_Alarm[Detect_OUTPUT] = UsartReceiveData[6];
			OutPut_Pause[Detect_OUTPUT] = UsartReceiveData[7];
			OutPut_Stop[Detect_OUTPUT] = UsartReceiveData[8];
			Output_Name[Detect_OUTPUT].Name = (u32)(((u32)UsartReceiveData[9])|((u32)UsartReceiveData[10]<<8)|((u32)UsartReceiveData[11]<<16)|((u32)UsartReceiveData[12]<<24));
			Output_Name[Detect_OUTPUT].Name1 = (u32)(((u32)UsartReceiveData[13])|((u32)UsartReceiveData[14]<<8)|((u32)UsartReceiveData[15]<<16)|((u32)UsartReceiveData[16]<<24));
			Output_Name[Detect_OUTPUT].Name2 = (u32)(((u32)UsartReceiveData[17])|((u32)UsartReceiveData[18]<<8)|((u32)UsartReceiveData[19]<<16)|((u32)UsartReceiveData[20]<<24));
			W25QXX_Write(&UsartReceiveData[3],0x1240+6*Detect_OUTPUT,6);
			W25QXX_Write(&UsartReceiveData[9],0x1468+12*Detect_OUTPUT,12);
			break;
		case P_PARAMETER_IO_INSWITCH_SET://特殊输入口处理
			{
				Selected_InPut_Num = UsartReceiveData[2];//被选中的是第几个输出IO口
				Temp_IO_Switch_Parameter[Selected_InPut_Num] = UsartReceiveData[3];
				W25QXX_Write(&UsartReceiveData[3],0x11C0+1*Selected_InPut_Num,1);
			}
			break;
		case P_PARAMETER_OUT_INSWITCH_SET://特殊输出口处理
			{
				Selected_OutPut_Num = UsartReceiveData[2];//被选中的是第几个输出IO口
				Temp_OUT_Switch_Parameter[Selected_OutPut_Num] = UsartReceiveData[3];
				Output_Name[Selected_OutPut_Num].Name = (u32)(((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24));
				Output_Name[Selected_OutPut_Num].Name1 = (u32)(((u32)UsartReceiveData[8])|((u32)UsartReceiveData[9]<<8)|((u32)UsartReceiveData[10]<<16)|((u32)UsartReceiveData[11]<<24));
				Output_Name[Selected_OutPut_Num].Name2 = (u32)(((u32)UsartReceiveData[12])|((u32)UsartReceiveData[13]<<8)|((u32)UsartReceiveData[14]<<16)|((u32)UsartReceiveData[15]<<24));
				W25QXX_Write(&UsartReceiveData[3],0x11E0+1*Selected_OutPut_Num,1);
				W25QXX_Write(&UsartReceiveData[4],0x1468+12*Selected_OutPut_Num,12);
			}
			break;
			
		case P_PARAMETER_MD_PARA:
			Code = UsartReceiveData[2] - 1;
			address1 = P_MD_PARA_HEAD + Code*P_MD_GOOD_LEN;
			W25QXX_Write(&UsartReceiveData[3], address1, P_MD_PARA_LEN);
			break;
		case P_PARAMETER_MD_POINT:
			Code = UsartReceiveData[2] - 1;
			Layer = UsartReceiveData[3] - 1;
			Pcode = UsartReceiveData[4] - 1;
			address1 = P_MD_POINT_HEAD + Code*P_MD_GOOD_LEN+(Layer*MD_POINT_NUM + Pcode)*P_MD_POINT_LEN;
			W25QXX_Write(&UsartReceiveData[5], address1, P_MD_POINT_LEN);
			break;
		case P_PARAMETER_MD_DELETE://码垛参数恢复出厂设置
			All_MD_Deleted_Flag = 0;
			//点恢复-先全部擦除为0
			for(i=0; i<P_MD_CLEAR_NUM; i++)
			{//每次写4K字节的数据0
				W25QXX_Writ0_4K(P_MD_PARA_HEAD + i*4096);
			}
			
			//参数恢复
			sMD_Name.Name1 = 0;
			sMD_Name.Name2 = 0;
			sMD_Parameter.stackType = 0;
			sMD_Parameter.property = 0;
			sMD_Parameter.revolveMode = 0;
			sMD_Parameter.gasPort = 0;
			sMD_Parameter.topSwitch = 0;
			sMD_Parameter.goodheight = 0;
			sMD_Parameter.stackLayer = 1;
			sMD_Parameter.loopLayer = 1;
			for(i=0; i<LOOP_MAX; i++)
			{
				sMD_Parameter.layerNum[i] = 1;
			}
			for(i=0; i<Axis_Num; i++)
			{
				sMD_Parameter.goodOffset[i] = 0;
			}
			sMD_Parameter.horNum = 1;
			sMD_Parameter.verNum = 1;
			
			data_temp[4] = sMD_Name.Name1;
			data_temp[5] = sMD_Name.Name1>>8;
			data_temp[6] = sMD_Name.Name1>>16;
			data_temp[7] = sMD_Name.Name1>>24;
			data_temp[8] = sMD_Name.Name2;
			data_temp[9] = sMD_Name.Name2>>8;
			data_temp[10] = sMD_Name.Name2>>16;
			data_temp[11] = sMD_Name.Name2>>24;
			data_temp[12] = sMD_Parameter.stackType;
			data_temp[13] = sMD_Parameter.property;
			data_temp[14] = sMD_Parameter.revolveMode;
			data_temp[15] = sMD_Parameter.gasPort;
			data_temp[16] = sMD_Parameter.topSwitch;
			data_temp[17] = sMD_Parameter.goodheight;
			data_temp[18] = sMD_Parameter.goodheight>>8;
			data_temp[19] = sMD_Parameter.goodheight>>16;
			data_temp[20] = sMD_Parameter.goodheight>>24;
			data_temp[21] = sMD_Parameter.stackLayer;
			data_temp[22] = sMD_Parameter.loopLayer;
			for(i=0; i<LOOP_MAX; i++)
			{
				data_temp[23 + i] = sMD_Parameter.layerNum[i];
			}
			for(i=0; i<Axis_Num; i++)
			{
				data_temp[23 + LOOP_MAX + i * 4] = sMD_Parameter.goodOffset[i];
				data_temp[24 + LOOP_MAX + i * 4] = sMD_Parameter.goodOffset[i]>>8;
				data_temp[25 + LOOP_MAX + i * 4] = sMD_Parameter.goodOffset[i]>>16;
				data_temp[26 + LOOP_MAX + i * 4] = sMD_Parameter.goodOffset[i]>>24;
			}
			data_temp[39 + LOOP_MAX] = sMD_Parameter.horNum;
			data_temp[40 + LOOP_MAX] = sMD_Parameter.verNum;
			
			for(i=0; i<MD_GOOD_NUM; i++)
			{//名称不同
				sMD_Name.Name = 0x4D443030 + ((u32)(i+1)/10 << 8) + (i+1)%10;
				
				data_temp[0] = sMD_Name.Name;
				data_temp[1] = sMD_Name.Name>>8;
				data_temp[2] = sMD_Name.Name>>16;
				data_temp[3] = sMD_Name.Name>>24;
				//写参数
				W25QXX_Write(data_temp, P_MD_PARA_HEAD + i*P_MD_GOOD_LEN, P_MD_PARA_LEN);
			}
			
			for(j=0; j< (MD_GOOD_NUM * 2 / 60 + 1); j++)
			{//恢复分拣模式的当前层和个数的值
				for(i=0; i<60; i++)
				{//每次恢复60个字节
					data_temp[i] = 1;
				}
				W25QXX_Write(data_temp, P_MD_SORT_ADDRESS + j * 60, 60);
			}
			
			for(i=0; i<MD_GOOD_NUM; i++)
			{//当前值恢复
				sMD_FlashCurLayer[i] = 1;
				sMD_FlashCurNum[i] = 1;
			}
			
			sMD_RunPara.mdMethed = 1;
			sMD_RunPara.startGood = 1;
			sMD_RunPara.totalGood = 1;
			sMD_RunPara.curGood = 1;
			sMD_RunPara.curLayer = 1;
			sMD_RunPara.curNum = 1;
			
			All_MD_Deleted_Flag = 1;
			break;

		default:
			break;
	}
}

/*************************************************************************
**  函数名：  Parameter_StringChang_Min()
**	输入参数：
**	输出参数：无
**	函数功能：12字节的名字字符串赋值-小端
**	备注：	  无
**  作者：    
**  开发日期：
**************************************************************************/
void Parameter_StringChang_Min(u8 *string, u16 startIdx, u32 name1, u32 name2, u32 name3)
{
	string[startIdx] = (u8)(name1);
	string[startIdx + 1] = (u8)(name1>>8);
	string[startIdx + 2] = (u8)(name1>>16);
	string[startIdx + 3] = (u8)(name1>>24);
	string[startIdx + 4] = (u8)(name2);
	string[startIdx + 5] = (u8)(name2>>8);
	string[startIdx + 6] = (u8)(name2>>16);
	string[startIdx + 7] = (u8)(name2>>24);
	string[startIdx + 8] = (u8)(name3);
	string[startIdx + 9] = (u8)(name3>>8);
	string[startIdx + 10] = (u8)(name3>>16);
	string[startIdx + 11] = (u8)(name3>>24);
}

/**************************************************************************************************
**  函数名：  FreeProgramSend()
**	输入参数：无
**	输出参数：无
**	函数功能：自由编程内容发送
**	备注：	  无
**  作者：    
**  开发日期：
***************************************************************************************************/
void FreeProgramSend()
{
	u16 i = 0,j = 0;
	u8 adrees_temp[20] = {0};

	switch(UsartReceiveData[1])
	{
		case PROGRAM_FROM_USB_START://本段程序下发来自U盘 开始
			Program_From_UDisk_Flag = TRUE;
			//将地址数组清零
			for(i=0; i<SAVEPROGRAMNUM; i++)
			{
				Program_IIC_Address[i].Flag = 0;
				Program_IIC_Address[i].Name = 0;
				Program_IIC_Address[i].Name2 = 0;
				Program_IIC_Address[i].Name3 = 0;
				Program_IIC_Address[i].Num  = 0;
			}
			break;	

		case PROGRAM_FROM_USB_END://本段程序下发来自U盘 结束
			Program_From_UDisk_Flag = FALSE;
			g_Run_Program_Num = 0;
			W25QXX_Write(&g_Run_Program_Num,0x40E0,1);
			g_Run_Program_Num_Pre = g_Run_Program_Num;		
			//将地址数组保存
			for(i=0; i<SAVEPROGRAMNUM; i++)
			{
				adrees_temp[0] = Program_IIC_Address[i].Flag;
				adrees_temp[1] = Program_IIC_Address[i].Code;
				Parameter_StringChang_Min(adrees_temp, 2, Program_IIC_Address[i].Name, Program_IIC_Address[i].Name2, Program_IIC_Address[i].Name3);
//				adrees_temp[2] = Program_IIC_Address[i].Name;
//				adrees_temp[3] = Program_IIC_Address[i].Name>>8;
//				adrees_temp[4] = Program_IIC_Address[i].Name>>16;
//				adrees_temp[5] = Program_IIC_Address[i].Name>>24;
//				adrees_temp[6] = Program_IIC_Address[i].Name2;
//				adrees_temp[7] = Program_IIC_Address[i].Name2>>8;
//				adrees_temp[8] = Program_IIC_Address[i].Name2>>16;
//				adrees_temp[9] = Program_IIC_Address[i].Name2>>24;
//				adrees_temp[10] = Program_IIC_Address[i].Name3;
//				adrees_temp[11] = Program_IIC_Address[i].Name3>>8;
//				adrees_temp[12] = Program_IIC_Address[i].Name3>>16;
//				adrees_temp[13] = Program_IIC_Address[i].Name3>>24;
				adrees_temp[14] = Program_IIC_Address[i].Num;			 
				W25QXX_Write(adrees_temp,Program_IIC_Address[i].Address,15);
			}
			break;	

		case P_PROGRAM_START://开始接收本次程序
			Start_Recieve_Program_Flag = TRUE;
			Temp_Num = 0;
			SaveProgram_IIC_Address = 0;
			Free_Program_Operate.Flag = 0;
			Free_Program_Operate.Code = 0;   
			Free_Program_Operate.Name = 0;
			Free_Program_Operate.Name2 = 0;
			Free_Program_Operate.Name3 = 0;
			Free_Program_Operate.Num  = 0;
			break;	

		case PROGRAM_INFO://本次程序信息
			if(Start_Recieve_Program_Flag)
			{
				if(UsartReceiveData[3] <= SAVEPROGRAMNUM)
				{
					Free_Program_Operate.Flag  = UsartReceiveData[2];
					Free_Program_Operate.Code  = UsartReceiveData[3];   //第一个程序Code=1
					Free_Program_Operate.Name  = (u32)(((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24));
					Free_Program_Operate.Name2 = (u32)(((u32)UsartReceiveData[8])|((u32)UsartReceiveData[9]<<8)|((u32)UsartReceiveData[10]<<16)|((u32)UsartReceiveData[11]<<24));
					Free_Program_Operate.Name3 = (u32)(((u32)UsartReceiveData[12])|((u32)UsartReceiveData[13]<<8)|((u32)UsartReceiveData[14]<<16)|((u32)UsartReceiveData[15]<<24));					
					Free_Program_Operate.Num   = UsartReceiveData[16];

					SaveProgram_IIC_Address = Program_IIC_Address[Free_Program_Operate.Code - 1].Address;
					
					Program_IIC_Address[Free_Program_Operate.Code - 1].Flag  = Free_Program_Operate.Flag;
					Program_IIC_Address[Free_Program_Operate.Code - 1].Code  = Free_Program_Operate.Code;
					Program_IIC_Address[Free_Program_Operate.Code - 1].Name  = Free_Program_Operate.Name;
					Program_IIC_Address[Free_Program_Operate.Code - 1].Name2 = Free_Program_Operate.Name2;
					Program_IIC_Address[Free_Program_Operate.Code - 1].Name3 = Free_Program_Operate.Name3;
					Program_IIC_Address[Free_Program_Operate.Code - 1].Num   = Free_Program_Operate.Num;					
					W25QXX_Write(&UsartReceiveData[2],SaveProgram_IIC_Address,15);
				}
			}
			break;	

		case PROGRAM_CONT://本次程序内容
			if(Start_Recieve_Program_Flag)
			{
				Temp_Num = UsartReceiveData[18];
				Free_Program_Operate.Program[Temp_Num].Flag   = UsartReceiveData[2];
				Free_Program_Operate.Program[Temp_Num].List   = UsartReceiveData[3];
				Free_Program_Operate.Program[Temp_Num].Order  = UsartReceiveData[4];
				Free_Program_Operate.Program[Temp_Num].Key    = UsartReceiveData[5];
				Free_Program_Operate.Program[Temp_Num].Value1 = (u32)(((u32)UsartReceiveData[6])|((u32)UsartReceiveData[7]<<8)|((u32)UsartReceiveData[8]<<16)|((u32)UsartReceiveData[9]<<24));
				Free_Program_Operate.Program[Temp_Num].Value2 = (u32)(((u32)UsartReceiveData[10])|((u32)UsartReceiveData[11]<<8)|((u32)UsartReceiveData[12]<<16)|((u32)UsartReceiveData[13]<<24));	
				Free_Program_Operate.Program[Temp_Num].Value3 = (u32)(((u32)UsartReceiveData[14])|((u32)UsartReceiveData[15]<<8)|((u32)UsartReceiveData[16]<<16)|((u32)UsartReceiveData[17]<<24)); 					
				Free_Program_Operate.Program[Temp_Num].Value1 = Free_Program_Operate.Program[Temp_Num].Value1 & 0x0fffffff;
				Free_Program_Operate.Program[Temp_Num].Value2 = Free_Program_Operate.Program[Temp_Num].Value2 & 0x0fffffff;	
				if((UsartReceiveData[5] == K_INCREMENT_RUNNING && UsartReceiveData[17]>>4 == 0x09)\
					|| ((UsartReceiveData[5] == K_IF || UsartReceiveData[5] == K_ELSE || UsartReceiveData[5] == K_WHILE || UsartReceiveData[5] == K_USER) && UsartReceiveData[17]>>4 == 0x08))
				{//负数
					Free_Program_Operate.Program[Temp_Num].Value3 = Free_Program_Operate.Program[Temp_Num].Value3 | 0xf0000000;	
				}
				else
				{
					Free_Program_Operate.Program[Temp_Num].Value3 = Free_Program_Operate.Program[Temp_Num].Value3 & 0x0fffffff;	
				}
				W25QXX_Write(&UsartReceiveData[2],SaveProgram_IIC_Address+0x0F+Temp_Num*0x10,16);	 //每次写入一条语句，写的条数由Num决定
				//Temp_Num++;
			}
			break;	

		case P_PROGRAM_END://结束本次程序接收
			Start_Recieve_Program_Flag = FALSE;
			Temp_Num = 0;
			SaveProgram_IIC_Address = 0;
			m_WhileNC = 0;	
			for(i=0; i<WHILE_NEST_MAX; i++)
			{
				m_WhileRunFlag[i] = 0;
			}
			g_Run_Program_Num = Free_Program_Operate.Code;
			W25QXX_Write(&g_Run_Program_Num,0x40E0,1);
			g_Run_Program_Num_Pre = g_Run_Program_Num;		
			break;	

		case PROGRAM_DELETE:		 //删除程序		 
			Current_Delete_Program = UsartReceiveData[2];
			
			if((Current_Delete_Program > SAVEPROGRAMNUM_MAIN) && (Current_Delete_Program <= SAVEPROGRAMNUM))
			{
				Program_IIC_Address[Current_Delete_Program-1].Flag  = 0;
				Program_IIC_Address[Current_Delete_Program-1].Name  = 0;
				Program_IIC_Address[Current_Delete_Program-1].Name2 = 0;
				Program_IIC_Address[Current_Delete_Program-1].Name3 = 0;
				Program_IIC_Address[Current_Delete_Program-1].Num   = 0;				 

				//判断当前运行程序编号
				if(Current_Delete_Program == Free_Program_Operate.Code)
				{
					Free_Program_Operate.Flag  = 0;
					Free_Program_Operate.Name  = 0;
					Free_Program_Operate.Name2 = 0;
					Free_Program_Operate.Name3 = 0;
					Free_Program_Operate.Num   = 0;
					g_Run_Program_Num = 0;
					g_Run_Program_Num_Pre = g_Run_Program_Num;					
				}
			}
			else if(Current_Delete_Program < SAVEPROGRAMNUM_MAIN)
			{	
				for(i=0; i<SAVEPROGRAMNUM_MAIN; i++)
				{
					if(Program_IIC_Address[i].Code == Current_Delete_Program)
					{//查询到当前需要删除的程序编号，删除标志位，并清空IIC
						Program_IIC_Address[i].Flag  = 0;
						Program_IIC_Address[i].Name  = 0;
						Program_IIC_Address[i].Name2 = 0;
						Program_IIC_Address[i].Name3 = 0;
						Program_IIC_Address[i].Num   = 0;
						break;
					}
				}
				
				if(Current_Delete_Program == Free_Program_Operate.Code)
				{//判断当前运行程序编号
					Free_Program_Operate.Flag  = 0;
					Free_Program_Operate.Name  = 0;
					Free_Program_Operate.Name2 = 0;
					Free_Program_Operate.Name3 = 0;
					Free_Program_Operate.Num   = 0;
					g_Run_Program_Num = 0;
					g_Run_Program_Num_Pre = g_Run_Program_Num;					
				}
			}
			else if(Current_Delete_Program == SAVEPROGRAMNUM_MAIN)
			{	
				Program_IIC_Address[SAVEPROGRAMNUM_MAIN-1].Flag  = 1;
				Program_IIC_Address[SAVEPROGRAMNUM_MAIN-1].Name  = 0xB8B4CEBB;
				Program_IIC_Address[SAVEPROGRAMNUM_MAIN-1].Name2 = 0xB3CCD0F2;
				Program_IIC_Address[SAVEPROGRAMNUM_MAIN-1].Name3 = 0;
				Program_IIC_Address[SAVEPROGRAMNUM_MAIN-1].Num   = 2;
				
				if(Current_Delete_Program == Free_Program_Operate.Code)
				{//判断当前运行程序编号
					Free_Program_Operate.Flag  = 0;
					Free_Program_Operate.Name  = 0;
					Free_Program_Operate.Name2 = 0;
					Free_Program_Operate.Name3 = 0;
					Free_Program_Operate.Num   = 0;
					g_Run_Program_Num = 0;
					g_Run_Program_Num_Pre = g_Run_Program_Num;
				}
				
				for(i=0; i<Program_IIC_Address[SAVEPROGRAMNUM_MAIN-1].Num; i++)
				{
					adrees_temp[0] = 1;
					adrees_temp[1] = i + 1;
					adrees_temp[2] = OR_BASICORDER;
					adrees_temp[3] = K_PROGRAMSTART + i;
					for(j=4; j<16; j++)
					{
						adrees_temp[j] = 0;
					}
					W25QXX_Write(adrees_temp,Program_IIC_Address[SAVEPROGRAMNUM_MAIN-1].Address+0x0F+i*0x10,16);	 //每次写入一条语句，写的条数由Num决定
				}
			}
				
			adrees_temp[0] = Program_IIC_Address[Current_Delete_Program-1].Flag;
			adrees_temp[1] = Program_IIC_Address[Current_Delete_Program-1].Code;
			Parameter_StringChang_Min(adrees_temp, 2, Program_IIC_Address[Current_Delete_Program-1].Name, Program_IIC_Address[Current_Delete_Program-1].Name2, Program_IIC_Address[Current_Delete_Program-1].Name3);
//			adrees_temp[2] = Program_IIC_Address[Current_Delete_Program-1].Name;
//			adrees_temp[3] = Program_IIC_Address[Current_Delete_Program-1].Name>>8;
//			adrees_temp[4] = Program_IIC_Address[Current_Delete_Program-1].Name>>16;
//			adrees_temp[5] = Program_IIC_Address[Current_Delete_Program-1].Name>>24;
//			adrees_temp[6] = Program_IIC_Address[Current_Delete_Program-1].Name2;
//			adrees_temp[7] = Program_IIC_Address[Current_Delete_Program-1].Name2>>8;
//			adrees_temp[8] = Program_IIC_Address[Current_Delete_Program-1].Name2>>16;
//			adrees_temp[9] = Program_IIC_Address[Current_Delete_Program-1].Name2>>24;
//			adrees_temp[10] = Program_IIC_Address[Current_Delete_Program-1].Name3;
//			adrees_temp[11] = Program_IIC_Address[Current_Delete_Program-1].Name3>>8;
//			adrees_temp[12] = Program_IIC_Address[Current_Delete_Program-1].Name3>>16;
//			adrees_temp[13] = Program_IIC_Address[Current_Delete_Program-1].Name3>>24;
			adrees_temp[14] = Program_IIC_Address[Current_Delete_Program-1].Num;		 
			W25QXX_Write(adrees_temp,Program_IIC_Address[Current_Delete_Program-1].Address,15);	
			
			Current_Delete_Program = 0;
			W25QXX_Write(&g_Run_Program_Num,0x40E0,1);
			break;
			
		case P_PROGRAM_DELETE:  //程序恢复出厂设置
			All_Program_Deleted_Flag = 0;
			for(i=0; i<SAVEPROGRAMNUM; i++)
			{
				Program_IIC_Address[i].Flag  = 0;
				Program_IIC_Address[i].Code  = i + 1;
				Program_IIC_Address[i].Name  = 0;
				Program_IIC_Address[i].Name2 = 0;
				Program_IIC_Address[i].Name3 = 0;
				Program_IIC_Address[i].Num   = 0;
				
				adrees_temp[0] = Program_IIC_Address[i].Flag;
				adrees_temp[1] = Program_IIC_Address[i].Code;
				Parameter_StringChang_Min(adrees_temp, 2, Program_IIC_Address[i].Name, Program_IIC_Address[i].Name2, Program_IIC_Address[i].Name3);
//				adrees_temp[2] = Program_IIC_Address[i].Name;
//				adrees_temp[3] = Program_IIC_Address[i].Name>>8;
//				adrees_temp[4] = Program_IIC_Address[i].Name>>16;
//				adrees_temp[5] = Program_IIC_Address[i].Name>>24;
//				adrees_temp[6] = Program_IIC_Address[i].Name2;
//				adrees_temp[7] = Program_IIC_Address[i].Name2>>8;
//				adrees_temp[8] = Program_IIC_Address[i].Name2>>16;
//				adrees_temp[9] = Program_IIC_Address[i].Name2>>24;
//				adrees_temp[10] = Program_IIC_Address[i].Name3;
//				adrees_temp[11] = Program_IIC_Address[i].Name3>>8;
//				adrees_temp[12] = Program_IIC_Address[i].Name3>>16;
//				adrees_temp[13] = Program_IIC_Address[i].Name3>>24;
				adrees_temp[14] = Program_IIC_Address[i].Num;		
				W25QXX_Write(adrees_temp,Program_IIC_Address[i].Address,15);
			}
			g_Run_Program_Num = 0;
			g_Run_Program_Num_Pre = g_Run_Program_Num;			
			W25QXX_Write(&g_Run_Program_Num,0x40E0,1);
			All_Program_Deleted_Flag = 1;
			break;

		default:
			break;
	}
}

/**************************************************************************************************
**  函数名：  ReadIIC_Setting()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC机械手设置参数。
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
void ReadIIC_Setting()
{
	u16 i = 0;
	u8 data_temp[100] = {0};
	
	Search_Time[0] = UsartReceiveData[2];
	if(Search_Time[0] == 10 + USER_NUM)
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[0] + 1;
		USART1_SendData(2,M_READ_IIC_1,data_temp);
		Search_Time[0] = 0;
	}
	else
	{
		data_temp[0] = 0x11;
		data_temp[1] = Search_Time[0] + 1;	
		if(Search_Time[0] == 0)
		{//机械手参数
			W25QXX_Read(&data_temp[2],0x10B0,30);
			W25QXX_Read(&data_temp[32],0x1230,5);
			USART1_SendData(37,M_READ_IIC_1,data_temp);	
		}
		else if(Search_Time[0] == 1)			 
		{//生产参数
//			W25QXX_Read(&data_temp[2],P_SC_NUM_ADDRESS,30);
			data_temp[2] = SC_Parameter.RW_Num;
			data_temp[3] = SC_Parameter.RW_Num>>8;
			data_temp[4]= SC_Parameter.RW_Num>>16;
			data_temp[5]= SC_Parameter.RW_Num>>24;
			data_temp[6]= SC_Parameter.CJ_Num;
			data_temp[7]= SC_Parameter.CJ_Num>>8;
			data_temp[8]= SC_Parameter.CJ_Num>>16;
			data_temp[9]= SC_Parameter.CJ_Num>>24;
			data_temp[10]= SC_Parameter.JG_Num;
			data_temp[11]= SC_Parameter.JG_Num>>8;
			data_temp[12] = SC_Parameter.JG_Num>>16;
			data_temp[13] = SC_Parameter.JG_Num>>24;
			data_temp[14] = SC_Parameter.SC_Num;
			data_temp[15] = SC_Parameter.SC_Num>>8;
			data_temp[16] = SC_Parameter.SC_Num>>16;
			data_temp[17] = SC_Parameter.SC_Num>>24;
			data_temp[18] = SC_Parameter.LJ_Num;
			data_temp[19] = SC_Parameter.LJ_Num>>8;
			data_temp[20] = SC_Parameter.LJ_Num>>16;
			data_temp[21] = SC_Parameter.LJ_Num>>24;
			data_temp[22] = SC_Parameter.NG_Num;
			data_temp[23] = SC_Parameter.NG_Num>>8;
			data_temp[24] = SC_Parameter.NG_Num>>16;
			data_temp[25] = SC_Parameter.NG_Num>>24;
			
			W25QXX_Read(&data_temp[26],P_SC_NUM_ADDRESS,3);
			data_temp[29] = sMD_RunPara.curGood;
			data_temp[30] = sMD_RunPara.curLayer;
			data_temp[31] = sMD_RunPara.curNum;
			if(sMD_RunPara.mdMethed == 1)
			{//分拣模式
//				W25QXX_Write(&data_temp[30], P_MD_SORT_ADDRESS	+ 2 * (sMD_RunPara.curGood - 1), 2);
				data_temp[30] = sMD_FlashCurLayer[sMD_RunPara.curGood - 1];
				data_temp[31] = sMD_FlashCurNum[sMD_RunPara.curGood - 1];
			}
			
			W25QXX_Read(&data_temp[32], P_MD_PARA_HEAD + (sMD_RunPara.startGood-1)*P_MD_GOOD_LEN, 12);
			W25QXX_Read(&data_temp[44], P_MD_PARA_HEAD + (sMD_RunPara.curGood-1)*P_MD_GOOD_LEN, 12);
			USART1_SendData(56,M_READ_IIC_1,data_temp);	
		}
		else if(Search_Time[0] == 2)
		{//轴回原点参数设置
			W25QXX_Read(&data_temp[2],0x10D0,28);
			W25QXX_Read(&data_temp[30],0x1210,32);
			USART1_SendData(62,M_READ_IIC_1,data_temp);
		}
		else if(Search_Time[0] == 3)
		{//绝对值参数
			W25QXX_Read(&data_temp[2],0x1100,27);
			USART1_SendData(29,M_READ_IIC_1,data_temp);
		}	
		else if(Search_Time[0] < 4 + USER_NUM)
		{//用户变量
			i = Search_Time[0] - 4;
			W25QXX_Read(&data_temp[2],P_USER_ADDRESS + 23*(Search_Time[0]-4),23);
			USER_Parameter.USER_Name1[i] = (u32)(((u32)data_temp[3])|((u32)data_temp[4]<<8)|((u32)data_temp[5]<<16)|((u32)data_temp[6]<<24));
			USER_Parameter.USER_Name2[i] = (u32)(((u32)data_temp[7])|((u32)data_temp[8]<<8)|((u32)data_temp[9]<<16)|((u32)data_temp[10]<<24));
			USER_Parameter.USER_Name3[i] = (u32)(((u32)data_temp[11])|((u32)data_temp[12]<<8)|((u32)data_temp[13]<<16)|((u32)data_temp[14]<<24));
			USER_Parameter.INIT_Num[i] = (s32)(((u32)data_temp[15])|((u32)data_temp[16]<<8)|((u32)data_temp[17]<<16)|((u32)data_temp[18]<<24));
//				USER_Parameter.CURR_Num[i] = (s32)(((u32)data_temp[19])|((u32)data_temp[20]<<8)|((u32)data_temp[21]<<16)|((u32)data_temp[22]<<24));
			USER_Parameter.ELEC_RESET[i]  = data_temp[23];
			USER_Parameter.START_RESET[i] = data_temp[24];
			if(USER_Parameter.ELEC_RESET[i] == TRUE)
			{
				USER_Parameter.CURR_Num[i] = USER_Parameter.INIT_Num[i];
			}
			data_temp[19] = USER_Parameter.CURR_Num[i];
			data_temp[20] = USER_Parameter.CURR_Num[i]>>8;
			data_temp[21] = USER_Parameter.CURR_Num[i]>>16;
			data_temp[22] = USER_Parameter.CURR_Num[i]>>24;
			USART1_SendData(25,M_READ_IIC_1,data_temp);
		}
		else if(Search_Time[0] < 5 + USER_NUM)
		{//笛卡尔坐标系
			W25QXX_Read(&data_temp[2], P_CARTESIAN_PARA_HEAD, P_CARTESIAN_PARA_LEN);
			USART1_SendData(2+P_CARTESIAN_PARA_LEN,M_READ_IIC_1,data_temp);
		}
		else if(Search_Time[0] < 6 + USER_NUM)
		{//配方
			W25QXX_Read(&data_temp[2], P_FORMULATION_PARA_HEAD, P_FORMULATION_PARA_LEN);
			USART1_SendData(2+P_FORMULATION_PARA_LEN,M_READ_IIC_1,data_temp);
		}
		else if(Search_Time[0] < 7 + USER_NUM)
		{//电机控制PID参数
			W25QXX_Read(&data_temp[2], P_MOTORCONTROL_PARA_HEAD, P_MOTORCONTROL_PARA_LEN);
			USART1_SendData(2+P_MOTORCONTROL_PARA_LEN,M_READ_IIC_1,data_temp);
		}
		else if(Search_Time[0] < 8+USER_NUM)
		{//物联网
			W25QXX_Read(&data_temp[2],P_INTERNET_ADDRESS,13);
			USART1_SendData(15,M_READ_IIC_1,data_temp);
		}
		else if(Search_Time[0] < 10+USER_NUM)
		{//扩展轴
			W25QXX_Read(&data_temp[2],P_EXTENDAIX_ADDRESS + (Search_Time[0] - (8+USER_NUM))*P_EXTENDAIX_PARA_SIZE, P_EXTENDAIX_PARA_SIZE);
			USART1_SendData(P_EXTENDAIX_PARA_SIZE + 2,M_READ_IIC_1,data_temp);
		}
//		Search_Time[0]++;
	}
}

/**************************************************************************************************
**  函数名：  ReadIIC_Point()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC存储点数据。
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
void ReadIIC_Point()
{
	u8 data_temp[40] = {0};

	Search_Time[1] = UsartReceiveData[2];
	if(Search_Time[1] == MAXSAVEPROINT)
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[1] + 1;
		USART1_SendData(2,M_READ_IIC_2,data_temp);
		Search_Time[1] = 0;
	}
	else
	{
		data_temp[0] = 0x11;			
		if(Search_Time[1] < MAXSAVEPROINT)		 
		{
			W25QXX_Read(&data_temp[2],P_POINT_SAVE_HEAD + Search_Time[1]*P_POINT_SAVE_LEN + 1,29);		//+1:不读取编号
			data_temp[31] = 0;
			data_temp[1] = Search_Time[1] + 1;
			USART1_SendData(32,M_READ_IIC_2,data_temp);
		}
//		Search_Time[1]++;
	}
}

/**************************************************************************************************
**  函数名：  ReadIIC_Program_Inf()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC程序数据。
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
void ReadIIC_Program_Inf()
{
	u8 data_temp[20] = {0};

	Search_Time[2] = UsartReceiveData[2];
	if(Search_Time[2] == (SAVEPROGRAMNUM + 1))
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[2] + 1;
		USART1_SendData(2,M_READ_IIC_3,data_temp);
		Search_Time[2] = 0;		
	}
	else
	{
		data_temp[0] = 0x11;
		data_temp[1] = Search_Time[2] + 1;
		if(Search_Time[2] == 0)
		{
			data_temp[2] = g_Run_Program_Num;
			USART1_SendData(3,M_READ_IIC_3,data_temp);
		}
		else if(Search_Time[2]<=SAVEPROGRAMNUM)
		{
			data_temp[2] =  Program_IIC_Address[Search_Time[2]-1].Flag;
			data_temp[3] =  Program_IIC_Address[Search_Time[2]-1].Code;
			Parameter_StringChang_Min(data_temp, 4, Program_IIC_Address[Search_Time[2]-1].Name, Program_IIC_Address[Search_Time[2]-1].Name2, Program_IIC_Address[Search_Time[2]-1].Name3);
//			data_temp[4] =  Program_IIC_Address[Search_Time[2]-1].Name;
//			data_temp[5] =  Program_IIC_Address[Search_Time[2]-1].Name>>8;
//			data_temp[6] =  Program_IIC_Address[Search_Time[2]-1].Name>>16;
//			data_temp[7] =  Program_IIC_Address[Search_Time[2]-1].Name>>24;
//			data_temp[8] =  Program_IIC_Address[Search_Time[2]-1].Name2;
//			data_temp[9] =  Program_IIC_Address[Search_Time[2]-1].Name2>>8;
//			data_temp[10] = Program_IIC_Address[Search_Time[2]-1].Name2>>16;
//			data_temp[11] = Program_IIC_Address[Search_Time[2]-1].Name2>>24;
//			data_temp[12] = Program_IIC_Address[Search_Time[2]-1].Name3;
//			data_temp[13] = Program_IIC_Address[Search_Time[2]-1].Name3>>8;
//			data_temp[14] = Program_IIC_Address[Search_Time[2]-1].Name3>>16;
//			data_temp[15] = Program_IIC_Address[Search_Time[2]-1].Name3>>24;
			data_temp[16] = Program_IIC_Address[Search_Time[2]-1].Num;
			data_temp[17] = Program_IIC_Address[Search_Time[2]-1].Address;
			data_temp[18]= Program_IIC_Address[Search_Time[2]-1].Address>>8;
			USART1_SendData(19,M_READ_IIC_3,data_temp);		
		}
//		Search_Time[2]++;
	}
}
/**************************************************************************************************
**  函数名：  ReadIIC_Program()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC程序数据。
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
void ReadIIC_Program()
{
	u8 data_temp[20] = {0};

	Search_Time[3] = UsartReceiveData[3];
	if(Search_Time[3]==0)
	{//本条程序第一次查询发送：程序信息
		if(UsartReceiveData[2]<=0x02)
		{//开机同步-打开程序
			Read_SaveProgram_IIC_Address();
		}
		else if(UsartReceiveData[2]>0x02 && UsartReceiveData[2] < 0x03+SAVEPROGRAMNUM)
		{
			SaveProgram_IIC_Address = Program_IIC_Address[UsartReceiveData[2]-3].Address;
			SaveProgram_IIC_Num = Program_IIC_Address[UsartReceiveData[2]-3].Num ;
		}
		if(SaveProgram_IIC_Address > 0 && SaveProgram_IIC_Num > 0)
		{//程序信息
			data_temp[0]=0x11;
			data_temp[1]=Search_Time[3];
			W25QXX_Read(&data_temp[2],SaveProgram_IIC_Address,15);  	//读取每个程序的前7位程序信息						
			USART1_SendData(17,M_READ_IIC_4,data_temp);	   						//程序编码-发送次数-七位程序信息
		}
		else
		{//空程序
			data_temp[0] = 0xAA;
			data_temp[1] = Search_Time[3]+1;//空程序时
			USART1_SendData(2,M_READ_IIC_4,data_temp);
			Search_Time[3] = 0;
		}
	}
	else if(Search_Time[3] == SaveProgram_IIC_Num + 1)
	{//发送完成
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[3];
		USART1_SendData(2,M_READ_IIC_4,data_temp);
		Search_Time[3] = 0;
	}
	else if(Search_Time[3] < SaveProgram_IIC_Num + 1)
	{//程序内容
		if(SaveProgram_IIC_Address > 0 && SaveProgram_IIC_Num > 0)
		{
			data_temp[0]=0x11;
			data_temp[1]=Search_Time[3];
			W25QXX_Read(&data_temp[2],SaveProgram_IIC_Address+0x0F+(Search_Time[3]-1)*0x10,16);
			USART1_SendData(18,M_READ_IIC_4,data_temp);	   						//程序编码-发送次数-三位程序信息
		}
	}
}

/**************************************************************************************************
**  函数名：  ReadIIC_SoftLimit()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC存储软限位参数。
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
void ReadIIC_SoftLimit()
{
	u16 i = 0;
	u8 data_temp[12] = {0};

	Search_Time[4] = UsartReceiveData[2];
  if(Search_Time[4] == SAVESOFTLIMIT +1)//增加X-O轴安全设定参数，所以+1
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[4] + 1;
		USART1_SendData(2,M_READ_IIC_5,data_temp);
		Search_Time[4] = 0;
		
		for(i=0; i<Axis_Num; i++)
		{
			if(Robot_SoftLimit[i].Switch_Limit)
			{//轴软限位开
				 Axsis_Minlength[i] = (Robot_SoftLimit[i].Left_Limit * Step_Coefficient[i] / 100) + MINROBOTPOSITION;
				 Axsis_Maxlength[i] = (Robot_SoftLimit[i].Right_Limit * Step_Coefficient[i] / 100) + MINROBOTPOSITION;
			}
			else
			{
				 Axsis_Minlength[i] = MINROBOTPOSITION;
				 Axsis_Maxlength[i] = MAXROBOTPOSITION;
			}
		}
		
//		if(Robot_SoftLimit[X_Axsis].Switch_Limit)
//		{//X轴软限位开
//		 	 Axsis_Minlength[X_Axsis] = (Robot_SoftLimit[X_Axsis].Left_Limit * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
//			 Axsis_Maxlength[X_Axsis] = (Robot_SoftLimit[X_Axsis].Right_Limit * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
//		}
//		else
//		{
//		 	 Axsis_Minlength[X_Axsis] = MINROBOTPOSITION;
//			 Axsis_Maxlength[X_Axsis] = MAXROBOTPOSITION;
//		}
//		if(Robot_SoftLimit[L_Axsis].Switch_Limit)
//		{//L轴软限位开
//		 	 Axsis_Minlength[L_Axsis] = (Robot_SoftLimit[L_Axsis].Left_Limit * Step_Coefficient[L_Axsis] / 100) + MINROBOTPOSITION;
//			 Axsis_Maxlength[L_Axsis] = (Robot_SoftLimit[L_Axsis].Right_Limit * Step_Coefficient[L_Axsis] / 100) + MINROBOTPOSITION;
//		}
//		else
//		{
//		 	 Axsis_Minlength[L_Axsis] = MINROBOTPOSITION;
//			 Axsis_Maxlength[L_Axsis] = MAXROBOTPOSITION;
//		}
//		if(Robot_SoftLimit[Z_Axsis].Switch_Limit)
//		{//Z轴软限位开
//		 	 Axsis_Minlength[Z_Axsis] = (Robot_SoftLimit[Z_Axsis].Left_Limit * Step_Coefficient[Z_Axsis] / 100) + MINROBOTPOSITION;
//			 Axsis_Maxlength[Z_Axsis] = (Robot_SoftLimit[Z_Axsis].Right_Limit * Step_Coefficient[Z_Axsis] / 100) + MINROBOTPOSITION;
//		}
//		else
//		{
//		 	 Axsis_Minlength[Z_Axsis] = MINROBOTPOSITION;
//			 Axsis_Maxlength[Z_Axsis] = MAXROBOTPOSITION;
//		}
//		if(Robot_SoftLimit[O_Axsis].Switch_Limit)
//		{//O轴软限位开
//		 	 Axsis_Minlength[O_Axsis] = (Robot_SoftLimit[O_Axsis].Left_Limit * Step_Coefficient[O_Axsis] / 100) + MINROBOTPOSITION;
//			 Axsis_Maxlength[O_Axsis] = (Robot_SoftLimit[O_Axsis].Right_Limit * Step_Coefficient[O_Axsis] / 100) + MINROBOTPOSITION;
//		}
//		else
//		{
//		 	 Axsis_Minlength[O_Axsis] = MINROBOTPOSITION;
//			 Axsis_Maxlength[O_Axsis] = MAXROBOTPOSITION;
//		}
		Robot_SoftDistance.MaxDistance = (Robot_SoftDistance.MaxDistance * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
		Robot_SoftDistance.MinDistance = (Robot_SoftDistance.MinDistance * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
	}
	else
	{
		if(Search_Time[4] < 4)//前4帧发送软限位参数
		{
			data_temp[0] = 0x11;
			data_temp[1] = Search_Time[4]+1;
			W25QXX_Read(&data_temp[2],0x2000+Search_Time[4]*0x09,9);	
			
			USART1_SendData(11,M_READ_IIC_5,data_temp);
			
//			Robot_SoftLimit[Search_Time[4]].Left_Limit  = (u32)(((u32)data_temp[2])|((u32)data_temp[3]<<8)|((u32)data_temp[4]<<16)|((u32)data_temp[5]<<24));
//			Robot_SoftLimit[Search_Time[4]].Right_Limit = (u32)(((u32)data_temp[6])|((u32)data_temp[7]<<8)|((u32)data_temp[8]<<16)|((u32)data_temp[9]<<24));
//			Robot_SoftLimit[Search_Time[4]].Switch_Limit= data_temp[10];				
		}
		else if(Search_Time[4] == 4)//第5帧发送安全距离参数
		{
			data_temp[0] = 0x11;
			data_temp[1] = Search_Time[4]+1;
			W25QXX_Read(&data_temp[2],0x2030,8);
			
//			Robot_SoftDistance.MaxDistance = (u32)(((u32)data_temp[2])|((u32)data_temp[3]<<8)|((u32)data_temp[4]<<16)|((u32)data_temp[5]<<24));
//			Robot_SoftDistance.MinDistance = (u32)(((u32)data_temp[6])|((u32)data_temp[7]<<8)|((u32)data_temp[8]<<16)|((u32)data_temp[9]<<24));
			USART1_SendData(10,M_READ_IIC_5,data_temp);			
		}		
//		Search_Time[4]++;
	}

}

/**************************************************************************************************
**  函数名：  ReadIIC_SafeArea()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC存储安全区参数。
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
void ReadIIC_SafeArea()
{
	u8 data_temp[20]={0};

	Search_Time[5] = UsartReceiveData[2];
  if(Search_Time[5] == SAVESAFEAREA)
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[5] + 1;
		USART1_SendData(2,M_READ_IIC_6,data_temp);
		Search_Time[5] = 0;

		GPIO_SetBits(GPIOA,GPIO_Pin_5);		  //机械手伺服使能
		Robot_Enable = TRUE;
	}
	else
	{
		data_temp[0] = 0x11;
		data_temp[1] = Search_Time[5] + 1;
		W25QXX_Read(&data_temp[2],0x2100+Search_Time[5]*0x20,17);	
		USART1_SendData(19,M_READ_IIC_6,data_temp);
//	  Robot_Safe_Area[Search_Time[5]].X_Left  = (u32)(((u32)data_temp[2])|((u32)data_temp[3]<<8)|((u32)data_temp[4]<<16)|((u32)data_temp[5]<<24));
//		Robot_Safe_Area[Search_Time[5]].X_Left  = (Robot_Safe_Area[Search_Time[5]].X_Left * Step_Coefficient[X_Axsis] / 100)+MINROBOTPOSITION;
//		Robot_Safe_Area[Search_Time[5]].Z_Up    = (u32)(((u32)data_temp[6])|((u32)data_temp[7]<<8)|((u32)data_temp[8]<<16)|((u32)data_temp[9]<<24));
//		Robot_Safe_Area[Search_Time[5]].Z_Up    = (Robot_Safe_Area[Search_Time[5]].Z_Up * Step_Coefficient[Z_Axsis] / 100)+MINROBOTPOSITION;
//		Robot_Safe_Area[Search_Time[5]].X_Right = (u32)(((u32)data_temp[10])|((u32)data_temp[11]<<8)|((u32)data_temp[12]<<16)|((u32)data_temp[13]<<24));
//		Robot_Safe_Area[Search_Time[5]].X_Right = (Robot_Safe_Area[Search_Time[5]].X_Right * Step_Coefficient[X_Axsis] / 100)+MINROBOTPOSITION;
//		Robot_Safe_Area[Search_Time[5]].Z_Down  = (u32)(((u32)data_temp[14])|((u32)data_temp[15]<<8)|((u32)data_temp[16]<<16)|((u32)data_temp[17]<<24));
//		Robot_Safe_Area[Search_Time[5]].Z_Down  = (Robot_Safe_Area[Search_Time[5]].Z_Down * Step_Coefficient[Z_Axsis] / 100)+MINROBOTPOSITION;
//		Robot_Safe_Area[Search_Time[5]].SafeArea_Switch = data_temp[18];
//		Search_Time[5]++;
	}
}

/**************************************************************************************************
**  函数名：  ReadIIC_IOSet()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC机械手IO常开常闭设置参数。
**	备注：	  
**  作者：    Lin
**  开发日期：2018/05/14
***************************************************************************************************/
void ReadIIC_IOSet(void)
{
	u8 i = 0;
	u8 data_temp[50] = {0};
	
	Search_Time[6] = UsartReceiveData[2];
	if(Search_Time[6] == 14)
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[6] + 1;
		USART1_SendData(2,M_READ_IIC_7,data_temp);
		Search_Time[6] = 0;
		Not_Get_Position();
	}
	else
	{
		data_temp[0] = 0x11;
		data_temp[1] = Search_Time[6] + 1;
		
		if(Search_Time[6] < 3)
		{
			i = Search_Time[6];
			W25QXX_Read(&data_temp[2],0x1120 + 50*i,50);
			USART1_SendData(52,M_READ_IIC_7,data_temp);
		}
		else if(Search_Time[6] < 13)
		{
			i = Search_Time[6]-3;
			W25QXX_Read(&data_temp[2],0x1300 + 36*i,36);
			USART1_SendData(38,M_READ_IIC_7,data_temp);
		}
		else if(Search_Time[6] == 13)
		{
			W25QXX_Read(&data_temp[2],0x11C0,30);
			USART1_SendData(32,M_READ_IIC_7,data_temp);
		}
//		Search_Time[6]++;
	}
}

/**************************************************************************************************
**  函数名：  ReadIIC_MD_Point()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取码垛点
**	备注：	  
**  作者：        
**  开发日期：
***************************************************************************************************/
void ReadIIC_MD_Point()
{
	u8 code = 0;
	u8 layer = 0;
	u8 pcode = 0;
	u8 data_temp[70] = {0};
	
	Search_Time[7] = UsartReceiveData[2];
	code = UsartReceiveData[3] - 1;
	layer = UsartReceiveData[4] - 1;
	pcode = UsartReceiveData[5] - 1;
	
	if(Search_Time[7] == MD_POINT_PAGE_PER)
	{ 
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[7];
		USART1_SendData(2,M_READ_IIC_8,data_temp);
		Search_Time[7] = 0;
	}
	else
	{
		data_temp[0] = 0x11;
		data_temp[1] = Search_Time[7];
		W25QXX_Read(&data_temp[2], P_MD_POINT_HEAD + code*P_MD_GOOD_LEN+(layer*MD_POINT_NUM +pcode)*P_MD_POINT_LEN, P_MD_POINT_LEN);
		USART1_SendData(2+P_MD_POINT_LEN,M_READ_IIC_8,data_temp);
		
	}
}

/**************************************************************************************************
**  函数名：  ReadIIC_MD_Para()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取码垛参数。
**	备注：	  
**  作者：       
**  开发日期：
***************************************************************************************************/
void ReadIIC_MD_Para()
{
	u8 code = 0;
	u8 data_temp[70] = {0};
	
	Search_Time[8] = UsartReceiveData[2];
	code = UsartReceiveData[3] - 1;
	if(Search_Time[8] == 1)
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[8];
		USART1_SendData(2,M_READ_IIC_9,data_temp);
		Search_Time[8] = 0;
	}
	else
	{
		data_temp[0] = 0x11;
		data_temp[1] = Search_Time[8];
		W25QXX_Read(&data_temp[2], P_MD_PARA_HEAD + code*P_MD_GOOD_LEN, P_MD_PARA_LEN);
		USART1_SendData(2+P_MD_PARA_LEN,M_READ_IIC_9,data_temp);
		
	}
}

/**************************************************************************************************
**  函数名：  ReadIIC_MD_Name()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取码垛名字
**	备注：	  
**  作者：     
**  开发日期：
***************************************************************************************************/
void ReadIIC_MD_Name()
{
	u8 code = 0;
	u8 data_temp[64] = {0};

	Search_Time[9] = UsartReceiveData[2];
	code = UsartReceiveData[3] - 1;
  if(Search_Time[9] == MD_GOOD_PAGE_PER)
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[9];
		USART1_SendData(2,M_READ_IIC_10,data_temp);
		Search_Time[9] = 0;
	}
	else
	{
		data_temp[0] = 0x11;
		data_temp[1] = Search_Time[9];
		W25QXX_Read(&data_temp[2], P_MD_PARA_HEAD + code*P_MD_GOOD_LEN, 12);
		USART1_SendData(14,M_READ_IIC_10,data_temp);
	}
}

/**************************************************************************************************
**  函数名：  ReadIIC_OUTReset()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC机械手输出IO复位选择。
**	备注：	  
**  作者：    LLL
**  开发日期：20210525
***************************************************************************************************/
void ReadIIC_OUTReset(void)
{
	u8 i = 0;
	u8 data_temp[50] = {0};
	
	Search_Time[10] = UsartReceiveData[2];
	if(Search_Time[10] == 17)
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[10] + 1;
		USART1_SendData(2,M_READ_IIC_11,data_temp);
		Search_Time[10] = 0;
//		Not_Get_Position();  //什么作用
	}
	else
	{
		data_temp[0] = 0x11;
		data_temp[1] = Search_Time[10] + 1;
		
		if(Search_Time[10] < 6)
		{
			i = Search_Time[10];
			W25QXX_Read(&data_temp[2],0x1240 + 30*i,30);
//			OutPut_BeforeOrigin[0 + 5*i] = data_temp[2];
//			OutPut_AfterOrigin[0 + 5*i]  = data_temp[3];
//			OutPut_Common_Alarm[0 + 5*i] = data_temp[4];
//			OutPut_Emerge_Alarm[0 + 5*i] = data_temp[5];
//			OutPut_Pause[0 + 5*i] = data_temp[6];
//			OutPut_Stop[0 + 5*i] = data_temp[7];
//			OutPut_BeforeOrigin[1 + 5*i] = data_temp[8];
//			OutPut_AfterOrigin[1 + 5*i]  = data_temp[9];
//			OutPut_Common_Alarm[1 + 5*i] = data_temp[10];
//			OutPut_Emerge_Alarm[1 + 5*i] = data_temp[11];
//			OutPut_Pause[1 + 5*i] = data_temp[12];
//			OutPut_Stop[1 + 5*i] = data_temp[13];
//			OutPut_BeforeOrigin[2 + 5*i] = data_temp[14];
//			OutPut_AfterOrigin[2 + 5*i]  = data_temp[15];
//			OutPut_Common_Alarm[2 + 5*i] = data_temp[16];
//			OutPut_Emerge_Alarm[2 + 5*i] = data_temp[17];
//			OutPut_Pause[2 + 5*i] = data_temp[18];
//			OutPut_Stop[2 + 5*i] = data_temp[19];
//			OutPut_BeforeOrigin[3 + 5*i] = data_temp[20];
//			OutPut_AfterOrigin[3 + 5*i]  = data_temp[21];
//			OutPut_Common_Alarm[3 + 5*i] = data_temp[22];
//			OutPut_Emerge_Alarm[3 + 5*i] = data_temp[23];
//			OutPut_Pause[3 + 5*i] = data_temp[24];
//			OutPut_Stop[3 + 5*i] = data_temp[25];
//			OutPut_BeforeOrigin[4 + 5*i] = data_temp[26];
//			OutPut_AfterOrigin[4 + 5*i]  = data_temp[27];
//			OutPut_Common_Alarm[4 + 5*i] = data_temp[28];
//			OutPut_Emerge_Alarm[4 + 5*i] = data_temp[29];
//			OutPut_Pause[4 + 5*i] = data_temp[30];
//			OutPut_Stop[4 + 5*i] = data_temp[31];
			USART1_SendData(32,M_READ_IIC_11,data_temp);
		}
		else if(Search_Time[10] < 16)
		{
			i = Search_Time[10]-6;
			W25QXX_Read(&data_temp[2],0x1468 + 36*i,36);
//			Output_Name[0 + 3*i].Name = (u32)(((u32)data_temp[2]) |((u32)data_temp[3]<<8) |((u32)data_temp[4]<<16) |((u32)data_temp[5]<<24));
//			Output_Name[0 + 3*i].Name1 = (u32)(((u32)data_temp[6]) |((u32)data_temp[7]<<8) |((u32)data_temp[8]<<16) |((u32)data_temp[9]<<24));
//			Output_Name[0 + 3*i].Name2 = (u32)(((u32)data_temp[10]) |((u32)data_temp[11]<<8) |((u32)data_temp[12]<<16) |((u32)data_temp[13]<<24));
//			Output_Name[1 + 3*i].Name = (u32)(((u32)data_temp[14]) |((u32)data_temp[15]<<8) |((u32)data_temp[16]<<16) |((u32)data_temp[17]<<24));
//			Output_Name[1 + 3*i].Name1 = (u32)(((u32)data_temp[18]) |((u32)data_temp[19]<<8) |((u32)data_temp[20]<<16) |((u32)data_temp[21]<<24));
//			Output_Name[1 + 3*i].Name2 = (u32)(((u32)data_temp[22]) |((u32)data_temp[23]<<8) |((u32)data_temp[24]<<16) |((u32)data_temp[25]<<24));
//			Output_Name[2 + 3*i].Name = (u32)(((u32)data_temp[26]) |((u32)data_temp[27]<<8) |((u32)data_temp[28]<<16) |((u32)data_temp[29]<<24));
//			Output_Name[2 + 3*i].Name1 = (u32)(((u32)data_temp[30]) |((u32)data_temp[31]<<8) |((u32)data_temp[32]<<16) |((u32)data_temp[33]<<24));
//			Output_Name[2 + 3*i].Name2 = (u32)(((u32)data_temp[34]) |((u32)data_temp[35]<<8) |((u32)data_temp[36]<<16) |((u32)data_temp[37]<<24));
			USART1_SendData(38,M_READ_IIC_11,data_temp);
		}
		else if(Search_Time[10] == 16)
		{
			W25QXX_Read(&data_temp[2],0x11E0,30);
//			for(i = 0; i < OUTPUT_NUM; i++)
//			{
//				Temp_OUT_Switch_Parameter[i] = data_temp[i+2];
//			}
			USART1_SendData(32,M_READ_IIC_11,data_temp);
		}
	}
}

/**************************************************************************************************
**  函数名：  ReadIIC_ParameterCopy()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC机械手拷贝参数。
**	备注：	  
**  作者：    Lin
**  开发日期：2018/05/14
***************************************************************************************************/
void ReadIIC_ParameterCopy(void)
{
	u8 data_temp[60] = {0};
	
	Search_Time[8] = UsartReceiveData[2];
	if(Search_Time[8] == 131)
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[8] + 1;
		USART1_SendData(2,M_READ_IIC_12,data_temp);
		Search_Time[8] = 0;
	}
	else
	{
		data_temp[0] = 0x11;
		data_temp[1] = Search_Time[8] + 1;
		
		if(Search_Time[8] < 27)
		{
			W25QXX_Read(&data_temp[2],0x1090+50*Search_Time[8],50);   //0x1090-0x15D6  加上输入输出名字
			USART1_SendData(52,M_READ_IIC_12,data_temp);
		}
		else if(Search_Time[8] < 35)
		{
			W25QXX_Read(&data_temp[2],0x2000+50*(Search_Time[8]-27),50);   //0x2000-0x218F
			USART1_SendData(52,M_READ_IIC_12,data_temp);
		}
		else if(Search_Time[8] < 62)
		{
			W25QXX_Read(&data_temp[2],0x5000+50*(Search_Time[8]-35),50);   //0x5000-0x5545
			USART1_SendData(52,M_READ_IIC_12,data_temp);
		}
		else if(Search_Time[8] < 126)
		{
			W25QXX_Read(&data_temp[2],0x7330+50*(Search_Time[8]-62),50);  //0x7330-0x7FB0
			USART1_SendData(52,M_READ_IIC_12,data_temp);
		}
		else if(Search_Time[8] < 131)
		{
			W25QXX_Read(&data_temp[2],P_USER_ADDRESS+50*(Search_Time[8]-126),50);  //0xFFF070-0xFFF16A  用户变量 
			USART1_SendData(52,M_READ_IIC_12,data_temp);
		}
//		Search_Time[8]++;
	}
}

/**************************************************************************************************
**  函数名：  ReadIIC_MDParaCopy()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取码垛拷贝参数。
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
void ReadIIC_MDParaCopy(void)
{
	u8 data_temp[60] = {0};
	u8 code = 0;
	
	m_Copy_Time = (u16)(((u16)UsartReceiveData[2])|((u16)UsartReceiveData[3]<<8));
	code = UsartReceiveData[4] - 1;
	if(m_Copy_Time == (P_MD_GOOD_LEN/50+1))
	{
		data_temp[0] = 0xAA;
		data_temp[1] = m_Copy_Time + 1;
		data_temp[2] = (m_Copy_Time + 1)>>8;
		USART1_SendData(3,M_READ_IIC_13,data_temp);
		m_Copy_Time = 0;
	}
	else
	{
		data_temp[0] = 0x11;
		data_temp[1] = m_Copy_Time + 1;
		data_temp[2] = (m_Copy_Time + 1)>>8;
		
		W25QXX_Read(&data_temp[3],P_MD_PARA_HEAD+P_MD_GOOD_LEN*code+50*m_Copy_Time,50);
		USART1_SendData(53,M_READ_IIC_13,data_temp);
	}

}

/**************************************************************************************************
**  函数名：  ReadIICData()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC数据，开机数据从IIC读取之后，接收到串口命令则将数据发送给手控器
**	备注：	  分为六种类型数据读取
**  作者：    
**  开发日期：
***************************************************************************************************/
void ReadIICData()
{
	//修改IIC读取协议,第一字节为协议标识位,当前数据查询结束为0xAA ,否则为0x11
	switch(UsartReceiveData[1])
	{
		case 0x01:				//读取设置参数
			ReadIIC_Setting();
			break;

		case 0x02:				//读取保存点
			ReadIIC_Point();
			break;

		case 0x03:				//读取保存程序
			Temp_Search_Time = 0;//清除点数据的读取
			ReadIIC_Program_Inf();		 
			break;

		case 0x04:				//读取保存程序
			ReadIIC_Program();		 
			break;

		case 0x05:				//读取软限位
			ReadIIC_SoftLimit();
			break;

		case 0x06:				//读取安全区
			ReadIIC_SafeArea();
			break;

		case 0x07:				//读取IO数据
			ReadIIC_IOSet();
			break;

		case 0x08:				//读取码垛点
			ReadIIC_MD_Point();		 
			break;

		case 0x09:				//读取码垛参数
			ReadIIC_MD_Para();		 
			break;

		case 0x0A:				//读取码垛名称
			ReadIIC_MD_Name();		 
			break;
		
		case 0x0B:				//读取输出IO选择
			ReadIIC_OUTReset();
			break;
		
		case 0x0C:				//参数拷贝
			ReadIIC_ParameterCopy();
			break;
		
		case 0x0D:				//码垛拷贝
			ReadIIC_MDParaCopy();
			break;

		default:
			break;
	}
}

/**************************************************************************************************
**  函数名：  ReadIICSysParameter()
**	输入参数：无
**	输出参数：无
**	函数功能：开机读取IIC中的参数
**	备注：	  
**  作者：    
**  开发日期：
***************************************************************************************************/
void ReadIICSysParameter()
{
	u8 i=0;
	u8 j=0;
	u8 IIC_Parameter[120] = {0};						//数据读取时使用的中间变量
//	u32 temp = 0;

	for(i=0;i<SAVEPROGRAMNUM;i++)
	{//读取所有程序的基本信息
		W25QXX_Read(IIC_Parameter,Program_IIC_Address[i].Address,15);
		Program_IIC_Address[i].Flag = IIC_Parameter[0];
		if(Program_IIC_Address[i].Flag == 1)
		{
			Program_IIC_Address[i].Code  = i + 1;
			Program_IIC_Address[i].Name  = (u32)(((u32)IIC_Parameter[2])|((u32)IIC_Parameter[3]<<8)|((u32)IIC_Parameter[4]<<16)|((u32)IIC_Parameter[5])<<24);
			Program_IIC_Address[i].Name2 = (u32)(((u32)IIC_Parameter[6])|((u32)IIC_Parameter[7]<<8)|((u32)IIC_Parameter[8]<<16)|((u32)IIC_Parameter[9])<<24);
			Program_IIC_Address[i].Name3 = (u32)(((u32)IIC_Parameter[10])|((u32)IIC_Parameter[11]<<8)|((u32)IIC_Parameter[12]<<16)|((u32)IIC_Parameter[13])<<24);
			Program_IIC_Address[i].Num   = IIC_Parameter[14];
		}
		else
		{
			Program_IIC_Address[i].Flag  = 0;
			Program_IIC_Address[i].Code  = i + 1;
			Program_IIC_Address[i].Name  = 0;
			Program_IIC_Address[i].Name2 = 0;
			Program_IIC_Address[i].Name3 = 0;
			Program_IIC_Address[i].Num   = 0;
		}
	}
	
	for(i=0;i<16;i++)
	{
		IIC_Parameter[i] = 0;
	}
	
	//读取运行程序
	W25QXX_Read(IIC_Parameter, 0x40E0, 1);	//读取选中的程序编号
	g_Run_Program_Num = IIC_Parameter[0];
	if(g_Run_Program_Num > SAVEPROGRAMNUM)
	{
		g_Run_Program_Num = 0;
	}
	
	if(g_Run_Program_Num != 0)	 
	{//有程序，读取对应的程序基本信息及程序内容
		for(i=0; i<SAVEPROGRAMNUM; i++)
		{//查询到编号与当前设定自动运行编号相同数组
			if(Program_IIC_Address[i].Code == g_Run_Program_Num \
					&& Program_IIC_Address[i].Flag == 1)
			{
				W25QXX_Read(IIC_Parameter, Program_IIC_Address[i].Address, 15);
				Free_Program_Operate.Flag  = IIC_Parameter[0];
				Free_Program_Operate.Code  = IIC_Parameter[1];
				Free_Program_Operate.Name  = (u32)(((u32)IIC_Parameter[2])|((u32)IIC_Parameter[3]<<8)|((u32)IIC_Parameter[4]<<16)|((u32)IIC_Parameter[5]<<24));
				Free_Program_Operate.Name2 = (u32)(((u32)IIC_Parameter[6])|((u32)IIC_Parameter[7]<<8)|((u32)IIC_Parameter[8]<<16)|((u32)IIC_Parameter[9]<<24));
				Free_Program_Operate.Name3 = (u32)(((u32)IIC_Parameter[10])|((u32)IIC_Parameter[11]<<8)|((u32)IIC_Parameter[12]<<16)|((u32)IIC_Parameter[13]<<24));
				Free_Program_Operate.Num   = IIC_Parameter[14];
				for(j=0; j<Free_Program_Operate.Num; j++)
				{
					W25QXX_Read(IIC_Parameter, Program_IIC_Address[i].Address+0x0F+0x10*j, 16);
					Free_Program_Operate.Program[j].Flag   = IIC_Parameter[0];
					Free_Program_Operate.Program[j].List   = IIC_Parameter[1];
					Free_Program_Operate.Program[j].Order  = IIC_Parameter[2];
					Free_Program_Operate.Program[j].Key    = IIC_Parameter[3];
					Free_Program_Operate.Program[j].Value1 = (u32)(((u32)IIC_Parameter[4])|((u32)IIC_Parameter[5]<<8)|((u32)IIC_Parameter[6]<<16)|((u32)IIC_Parameter[7]<<24));
					Free_Program_Operate.Program[j].Value2 = (u32)(((u32)IIC_Parameter[8])|((u32)IIC_Parameter[9]<<8)|((u32)IIC_Parameter[10]<<16)|((u32)IIC_Parameter[11]<<24));	
					Free_Program_Operate.Program[j].Value3 = (u32)(((u32)IIC_Parameter[12])|((u32)IIC_Parameter[13]<<8)|((u32)IIC_Parameter[14]<<16)|((u32)IIC_Parameter[15]<<24)); 					
					Free_Program_Operate.Program[j].Value1 = Free_Program_Operate.Program[j].Value1 & 0x0fffffff;
					Free_Program_Operate.Program[j].Value2 = Free_Program_Operate.Program[j].Value2 & 0x0fffffff;	
						
					if((IIC_Parameter[3] == K_INCREMENT_RUNNING && IIC_Parameter[15]>>4 == 0x09)\
						|| ((IIC_Parameter[3] == K_IF || IIC_Parameter[3] == K_ELSE ||IIC_Parameter[3] == K_WHILE || IIC_Parameter[3] == K_USER) && IIC_Parameter[15]>>4 == 0x08))
					{
						Free_Program_Operate.Program[j].Value3 = Free_Program_Operate.Program[j].Value3 | 0xf0000000;	
					}
					else
					{
						Free_Program_Operate.Program[j].Value3 = Free_Program_Operate.Program[j].Value3 & 0x0fffffff;	
					}
				}
				break;
			}
			else if(Program_IIC_Address[i].Code == g_Run_Program_Num)
			{
				g_Run_Program_Num = 0;
				W25QXX_Write(&g_Run_Program_Num,0x40E0,1);
			}
		}
	}
	
	g_Run_Program_Num_Pre = g_Run_Program_Num;	

//	if(JXS_Parameter.PulseTime == 0)
//	{
//		JXS_Parameter.PulseTime = 50;
//	}

	//读取功能设定参数
	W25QXX_Read(IIC_Parameter, 0x10B0, 30);
	W25QXX_Read(&IIC_Parameter[30],0x1230,5);	
	JXS_Parameter.Axis       = IIC_Parameter[0];
	JXS_Parameter.Origin     = IIC_Parameter[1];
	JXS_Parameter.SpeedLevel = IIC_Parameter[2];
	Temp_JXS_Parameter_SpeedLevel = JXS_Parameter.SpeedLevel;

	JXS_Parameter.AlarmSignal = IIC_Parameter[3];
	JXS_Parameter.NcOrignin = IIC_Parameter[4];									//外部回零端口,X17
	JXS_Parameter.NcStartin = IIC_Parameter[5];									//外部启动端口,X18
	JXS_Parameter.NcPausein = IIC_Parameter[6];									//外部暂停端口,X19
	JXS_Parameter.NcStopin  = IIC_Parameter[7];									//外部停零端口,X20

	JXS_Parameter.LCcirculation = IIC_Parameter[8];

	JXS_Parameter.AlarmSwitch[X_Axsis] =IIC_Parameter[9]; 
	JXS_Parameter.AlarmSwitch[L_Axsis] =IIC_Parameter[10]; 
	JXS_Parameter.AlarmSwitch[Z_Axsis] =IIC_Parameter[11]; 
	JXS_Parameter.AlarmSwitch[O_Axsis] =IIC_Parameter[12]; 
	JXS_Parameter.Accelerate.Time[X_Axsis] =(u16)(((u16)IIC_Parameter[13])|((u16)IIC_Parameter[14]<<8)); 
	JXS_Parameter.Accelerate.Time[L_Axsis] =(u16)(((u16)IIC_Parameter[15])|((u16)IIC_Parameter[16]<<8)); 
	JXS_Parameter.Accelerate.Time[Z_Axsis] =(u16)(((u16)IIC_Parameter[17])|((u16)IIC_Parameter[18]<<8)); 
	JXS_Parameter.Accelerate.Time[O_Axsis] =(u16)(((u16)IIC_Parameter[19])|((u16)IIC_Parameter[20]<<8));
	JXS_Parameter.MDgripSwitch = IIC_Parameter[21];
	JXS_Parameter.MDgripPort[0] = IIC_Parameter[22];
	JXS_Parameter.MDgripPort[1] = IIC_Parameter[23];
	JXS_Parameter.MDgripPort[2] = IIC_Parameter[24];
	JXS_Parameter.MDgripPort[3] = IIC_Parameter[25];
	JXS_Parameter.MDgripPort[4] = IIC_Parameter[26];
	JXS_Parameter.MDgripPort[5] = IIC_Parameter[27];
	JXS_Parameter.OutputAssociate[0] = IIC_Parameter[28];
	JXS_Parameter.OutputAssociate[1] = IIC_Parameter[29];
	JXS_Parameter.ZAxsisAvoidace = IIC_Parameter[30];
	JXS_Parameter.ZAxsisLimit = (u32)(((u32)IIC_Parameter[31])|((u32)IIC_Parameter[32]<<8)|((u32)IIC_Parameter[33]<<16)|((u32)IIC_Parameter[34]<<24));		
	JXS_Parameter.ZAxsisLimit = (JXS_Parameter.ZAxsisLimit * Step_Coefficient[Z_Axsis] / 100) + MINROBOTPOSITION;
//	for(i=0; i<Axis_Num; i++)
//	{
//		ServoAccDecSet(i);
//	}

	//读取扩展轴参数
	for(i=0; i<EXTEN_AXIS_NUM; i++)
	{
		W25QXX_Read(IIC_Parameter, P_EXTENDAIX_ADDRESS + i*P_EXTENDAIX_PARA_SIZE, P_EXTENDAIX_PARA_SIZE);
		ExtendAix_Parameter[i].E_OriginPosition = IIC_Parameter[0];      //原点位置
		ExtendAix_Parameter[i].E_OriginOffset = (u32)(((u32)IIC_Parameter[1])|((u32)IIC_Parameter[2]<<8)|((u32)IIC_Parameter[3]<<16)|((u32)IIC_Parameter[4]<<24));         //原点偏移
		ExtendAix_Parameter[i].E_Circle_Pulse = (u32)(((u32)IIC_Parameter[5])|((u32)IIC_Parameter[6]<<8)|((u32)IIC_Parameter[7]<<16)|((u32)IIC_Parameter[8]<<24));      //单圈脉冲
		ExtendAix_Parameter[i].E_Circle_Distance = (u32)(((u32)IIC_Parameter[9])|((u32)IIC_Parameter[10]<<8)|((u32)IIC_Parameter[11]<<16)|((u32)IIC_Parameter[12]<<24));  //单圈距离
		ExtendAix_Parameter[i].E_AccAcc = (u16)(((u16)IIC_Parameter[13])|((u16)IIC_Parameter[14]<<8));            //加加速 
		ExtendAix_Parameter[i].E_AccTime = (u16)(((u16)IIC_Parameter[15])|((u16)IIC_Parameter[16]<<8));           //加速时间
		ExtendAix_Parameter[i].E_MaxDistance = (u32)(((u32)IIC_Parameter[17])|((u32)IIC_Parameter[18]<<8)|((u32)IIC_Parameter[19]<<16)|((u32)IIC_Parameter[20]<<24));       //最大行程
		ExtendAix_Parameter[i].E_Origin_Set = IIC_Parameter[21];        //原点设置
		
		Step_Coefficient[i + Axis_Num] = ExtendAix_Parameter[i].E_Circle_Pulse / (ExtendAix_Parameter[i].E_Circle_Distance / 1000.0f);
		Axsis_Minlength[i + Axis_Num] = MINROBOTPOSITION;
		Axsis_Maxlength[i + Axis_Num] = (ExtendAix_Parameter[i].E_MaxDistance * Step_Coefficient[i + Axis_Num] / 100) + MINROBOTPOSITION;
	}
	
	//读取笛卡尔坐标系设置参数
	W25QXX_Read(IIC_Parameter, P_CARTESIAN_PARA_HEAD, P_CARTESIAN_PARA_LEN);
	sCartesian_Para.length[0] = (u32)(((u32)IIC_Parameter[0])|((u32)IIC_Parameter[1]<<8)|((u32)IIC_Parameter[2]<<16)|((u32)IIC_Parameter[3]<<24));
	sCartesian_Para.length[1] = (u32)(((u32)IIC_Parameter[4])|((u32)IIC_Parameter[5]<<8)|((u32)IIC_Parameter[6]<<16)|((u32)IIC_Parameter[7]<<24));
//	for(i=0; i<Axis_Num; i++)
//	{
//		sCartesian_Para.startPoint[i] = (s32)(((u32)IIC_Parameter[8+i*12])|((u32)IIC_Parameter[9+i*12]<<8)|((u32)IIC_Parameter[10+i*12]<<16)|((u32)IIC_Parameter[11+i*12]<<24));
//		sCartesian_Para.revolveAngle[i] = (s32)(((u32)IIC_Parameter[12+i*12])|((u32)IIC_Parameter[13+i*12]<<8)|((u32)IIC_Parameter[14+i*12]<<16)|((u32)IIC_Parameter[15+i*12]<<24));
//		sCartesian_Para.revolvePoint[i] = (s32)(((u32)IIC_Parameter[16+i*12])|((u32)IIC_Parameter[17+i*12]<<8)|((u32)IIC_Parameter[18+i*12]<<16)|((u32)IIC_Parameter[19+i*12]<<24));
//	}
	sCartesian_Para.carCoordSwitch = IIC_Parameter[56];
	sCartesian_Para.MDCoordType = IIC_Parameter[57];
	for(i=0; i<Axis_Num; i++)
	{
		sCartesian_Para.axisType[i] = IIC_Parameter[58 + i];
	}
	sCartesian_Para.pitchLength = (u32)(((u32)IIC_Parameter[62])|((u32)IIC_Parameter[63]<<8)|((u32)IIC_Parameter[64]<<16)|((u32)IIC_Parameter[65]<<24));
	for(i=0; i<Axis_Num; i++)
	{
		sCartesian_Para.axisBackMinDir[i] = IIC_Parameter[66 + i];
		sCartesian_Para.axisInterpFlag[i] = IIC_Parameter[70 + i];
	}
	
	//读取电机控制PID参数
//	W25QXX_Read(IIC_Parameter, P_MOTORCONTROL_PARA_HEAD, P_MOTORCONTROL_PARA_LEN);
//	for(i=0; i<Axis_Num; i++)
//	{
//		temp = 300 + (u32)(((u32)IIC_Parameter[0 + i * 4])|((u32)IIC_Parameter[1 + i * 4]<<8)|((u32)IIC_Parameter[2 + i * 4]<<16)|((u32)IIC_Parameter[3 + i * 4]<<24));
//		if(temp > 2300) temp = 2300;
//		sMC_PID_Para.MC_P[i] = temp / 10000.0f;
//		temp = (u32)(((u32)IIC_Parameter[16 + i * 4])|((u32)IIC_Parameter[17 + i * 4]<<8)|((u32)IIC_Parameter[18 + i * 4]<<16)|((u32)IIC_Parameter[19 + i * 4]<<24));
//		temp = 0;
//		sMC_PID_Para.MC_I[i] = temp / 10000.0f;
//		temp = 100 + (u32)(((u32)IIC_Parameter[32 + i * 4])|((u32)IIC_Parameter[33 + i * 4]<<8)|((u32)IIC_Parameter[34 + i * 4]<<16)|((u32)IIC_Parameter[35 + i * 4]<<24));
//		if(temp > 1100) temp = 1100;
//		sMC_PID_Para.MC_D[i] = temp / 10000.0f;
//	}
	
	//读取配方设置参数
	W25QXX_Read(IIC_Parameter, P_FORMULATION_PARA_HEAD, P_FORMULATION_PARA_LEN);
	for(i=0; i<PF_IONUM; i++)
	{
		PF_Parameter.pfIOnum[i]= IIC_Parameter[0 + i];//0-3
		PF_Parameter.pfGood[i]= IIC_Parameter[4 + i];//4-7
		PF_Parameter.pfSwitch[i]= IIC_Parameter[7 + i];//8-11
	}
	
	//物联网
	W25QXX_Read(IIC_Parameter, P_INTERNET_ADDRESS, 13);
	Internet_Parameter.Switch = IIC_Parameter[0];
	for(i=0;i<12;i++)
	{
		Internet_Parameter.Sequence[i] = IIC_Parameter[i+1];
	}
	
	//读取原定信号相关参数
	W25QXX_Read(IIC_Parameter, 0x10D0, 28);
	W25QXX_Read(&IIC_Parameter[28],0x1210,32);
	for(i=0; i<Axis_Num; i++)
	{
		JXS_Parameter.OriginDir[i] = IIC_Parameter[i];						//轴回零方向
		JXS_Parameter.AxisOriginSpeed[i] = IIC_Parameter[i+4];		//轴回零速度
//		Axsis_Origin_Speed[i] = JXS_Parameter.AxisOriginSpeed[i];	//初始化原点回零速度
		JXS_Parameter.OriginPos[i] = IIC_Parameter[i+8];					//轴原点位置
		
		//轴回零偏移
		JXS_Parameter.OrignOffset[i] = (u32)(((u32)IIC_Parameter[i*4+12])|((u32)IIC_Parameter[i*4+13]<<8)|((u32)IIC_Parameter[i*4+14]<<16)|((u32)IIC_Parameter[i*4+15]<<24));
		JXS_Parameter.A_Circle_Pulse[i] = (u32)(((u32)IIC_Parameter[i*4+28])|((u32)IIC_Parameter[i*4+29]<<8)|((u32)IIC_Parameter[i*4+30]<<16)|((u32)IIC_Parameter[i*4+31]<<24));	//单圈位置
		JXS_Parameter.A_Circle_Distance[i] = (u32)(((u32)IIC_Parameter[i*4+44])|((u32)IIC_Parameter[i*4+45]<<8)|((u32)IIC_Parameter[i*4+46]<<16)|((u32)IIC_Parameter[i*4+47]<<24));//单圈脉冲
		
		Step_Coefficient[i] = (float)JXS_Parameter.A_Circle_Pulse[i]/((float)JXS_Parameter.A_Circle_Distance[i]/1000);
//		JXS_Parameter.OrignOffset[i] = JXS_Parameter.OrignOffset[i] * Step_Coefficient[i] / 100;
	}
	
	//读取IO检测时间参数
	for(i=0; i<INPUT_NUM; i++)
	{
		W25QXX_Read(IIC_Parameter,0x1120 + 5*i,5);
		IO_Input_keepMin[i]  = (u16)(((u16)IIC_Parameter[0])|((u16)IIC_Parameter[1]<<8));
		IO_Input_keepMax[i]  = (u16)(((u16)IIC_Parameter[2])|((u16)IIC_Parameter[3]<<8));
		IO_Sign_On_Off[i] 	  = IIC_Parameter[4];
		IO_Input_keepMin[i] = IO_Input_keepMin[i]*10;
		IO_Input_keepMax[i] = IO_Input_keepMax[i]*10;
	}
	//读取IO名字
	for(i=0; i<INPUT_NUM; i++)
	{
		W25QXX_Read(IIC_Parameter,0x1300 + 12*i,12);
		Input_Name[i].Name = (u32)(((u32)IIC_Parameter[0]) |((u32)IIC_Parameter[1]<<8) |((u32)IIC_Parameter[2]<<16) |((u32)IIC_Parameter[3]<<24));
		Input_Name[i].Name1 = (u32)(((u32)IIC_Parameter[4]) |((u32)IIC_Parameter[5]<<8) |((u32)IIC_Parameter[6]<<16) |((u32)IIC_Parameter[7]<<24));
		Input_Name[i].Name2 = (u32)(((u32)IIC_Parameter[8]) |((u32)IIC_Parameter[9]<<8) |((u32)IIC_Parameter[10]<<16) |((u32)IIC_Parameter[11]<<24));
	}
	//读取特殊IO
	W25QXX_Read(IIC_Parameter,0x11C0,30);
	for(i = 0; i < INPUT_NUM; i++)
	{
		Temp_IO_Switch_Parameter[i] = IIC_Parameter[i];
	}
	
	for(i=0; i<Axis_Num; i++)
	{
		JXS_Parameter.LimitSignOnOff[i] = IO_Sign_On_Off[22 + i];
		JXS_Parameter.OrignSignOnOff[i] = IO_Sign_On_Off[26 + i];
	}
//	JXS_Parameter.LimitSignOnOff[X_Axsis] = IO_Sign_On_Off[22];
//	JXS_Parameter.LimitSignOnOff[L_Axsis] = IO_Sign_On_Off[23];
//	JXS_Parameter.LimitSignOnOff[Z_Axsis] = IO_Sign_On_Off[24];
//	JXS_Parameter.LimitSignOnOff[O_Axsis] = IO_Sign_On_Off[25];
//	JXS_Parameter.OrignSignOnOff[X_Axsis] = IO_Sign_On_Off[26];
//	JXS_Parameter.OrignSignOnOff[L_Axsis] = IO_Sign_On_Off[27];
//	JXS_Parameter.OrignSignOnOff[Z_Axsis] = IO_Sign_On_Off[28];
//	JXS_Parameter.OrignSignOnOff[O_Axsis] = IO_Sign_On_Off[29];
	
	//读取输出IO复位选择参数
	for(i=0; i<OUTPUT_NUM; i++)
	{
		W25QXX_Read(IIC_Parameter,0x1240 + 6*i,6);
		OutPut_BeforeOrigin[i] = IIC_Parameter[0];
		OutPut_AfterOrigin[i] = IIC_Parameter[1];
		OutPut_Common_Alarm[i] = IIC_Parameter[2];
		OutPut_Emerge_Alarm[i] = IIC_Parameter[3];
		OutPut_Pause[i] = IIC_Parameter[4];
		OutPut_Stop[i] = IIC_Parameter[5];
	}
	//读取IO名字
	for(i=0; i<OUTPUT_NUM; i++)
	{
		W25QXX_Read(IIC_Parameter,0x1468 + 12*i,12);
		Output_Name[i].Name = (u32)(((u32)IIC_Parameter[0]) |((u32)IIC_Parameter[1]<<8) |((u32)IIC_Parameter[2]<<16) |((u32)IIC_Parameter[3]<<24));
		Output_Name[i].Name1 = (u32)(((u32)IIC_Parameter[4]) |((u32)IIC_Parameter[5]<<8) |((u32)IIC_Parameter[6]<<16) |((u32)IIC_Parameter[7]<<24));
		Output_Name[i].Name2 = (u32)(((u32)IIC_Parameter[8]) |((u32)IIC_Parameter[9]<<8) |((u32)IIC_Parameter[10]<<16) |((u32)IIC_Parameter[11]<<24));
	}
	//读取特殊IO
	W25QXX_Read(IIC_Parameter,0x11E0,30);
	for(i = 0; i < OUTPUT_NUM; i++)
	{
		Temp_OUT_Switch_Parameter[i] = IIC_Parameter[i];
	}

	//读取生产参数
	W25QXX_Read(IIC_Parameter,P_SC_NUM_ADDRESS,3);
//	W25QXX_Read(IIC_Parameter,P_SC_NUM_ADDRESS,30);
//	SC_Parameter.RW_Num = (u32)(((u32)IIC_Parameter[0]) |((u32)IIC_Parameter[1]<<8) |((u32)IIC_Parameter[2]<<16) |((u32)IIC_Parameter[3]<<24));
//	SC_Parameter.CJ_Num = (u32)(((u32)IIC_Parameter[4]) |((u32)IIC_Parameter[5]<<8) |((u32)IIC_Parameter[6]<<16) |((u32)IIC_Parameter[7]<<24));
//	SC_Parameter.JG_Num = (u32)(((u32)IIC_Parameter[8]) |((u32)IIC_Parameter[9]<<8) |((u32)IIC_Parameter[10]<<16)|((u32)IIC_Parameter[11]<<24));
//	SC_Parameter.SC_Num = (u32)(((u32)IIC_Parameter[12])|((u32)IIC_Parameter[13]<<8)|((u32)IIC_Parameter[14]<<16)|((u32)IIC_Parameter[15]<<24));
//	SC_Parameter.LJ_Num = (u32)(((u32)IIC_Parameter[16])|((u32)IIC_Parameter[17]<<8)|((u32)IIC_Parameter[18]<<16)|((u32)IIC_Parameter[19]<<24));
//	SC_Parameter.NG_Num = (u32)(((u32)IIC_Parameter[20])|((u32)IIC_Parameter[21]<<8)|((u32)IIC_Parameter[22]<<16)|((u32)IIC_Parameter[23]<<24));
	sMD_RunPara.mdMethed = IIC_Parameter[0];
	sMD_RunPara.totalGood = IIC_Parameter[1];
	sMD_RunPara.startGood = IIC_Parameter[2];
//	sMD_RunPara.curGood = IIC_Parameter[27];
//	sMD_RunPara.curLayer = IIC_Parameter[28];
//	sMD_RunPara.curNum = IIC_Parameter[29];
	if(sMD_RunPara.curGood < 1 || sMD_RunPara.curGood > MD_GOOD_NUM)
	{
		sMD_RunPara.curGood = 1;
	}
	if(sMD_RunPara.mdMethed == 1)
	{//分拣模式
//		W25QXX_Read(IIC_Parameter, P_MD_SORT_ADDRESS	+ 2 * (sMD_RunPara.curGood - 1), 2);
//		sMD_RunPara.curLayer = IIC_Parameter[0];
//		sMD_RunPara.curNum = IIC_Parameter[1];
		
		sMD_RunPara.curLayer = sMD_FlashCurLayer[sMD_RunPara.curGood - 1];
		sMD_RunPara.curNum = sMD_FlashCurNum[sMD_RunPara.curGood - 1];
	}
	
	//读取绝对值参数
	W25QXX_Read(&IIC_Parameter[2],0x1100,27);
	JDZ_Parameter.Switch = IIC_Parameter[2];	//功能开关
	JDZ_Parameter.Server = IIC_Parameter[3];	//伺服选择
	JDZ_Parameter.Resolu = IIC_Parameter[4];	//编码器分辨率
	
	for(i=0; i<Axis_Num; i++)
	{
		JDZ_Parameter.Circle_Pluse[i] = (u32)(((u32)IIC_Parameter[5 + i * 4])|((u32)IIC_Parameter[6 + i * 4]<<8)|((u32)IIC_Parameter[7 + i * 4]<<16)|((u32)IIC_Parameter[8 + i * 4]<<24));
		JDZ_Parameter.Motion_Dir[i]	=  IIC_Parameter[21 + i];	
		JDZ_Parameter.OriginSetting[i]=  IIC_Parameter[25 + i];	
	}
//	JDZ_Parameter.Circle_Pluse[X_Axsis] = (u32)(((u32)IIC_Parameter[5])|((u32)IIC_Parameter[6]<<8)|((u32)IIC_Parameter[7]<<16)|((u32)IIC_Parameter[8]<<24));
//	JDZ_Parameter.Circle_Pluse[L_Axsis] = (u32)(((u32)IIC_Parameter[9])|((u32)IIC_Parameter[10]<<8)|((u32)IIC_Parameter[11]<<16)|((u32)IIC_Parameter[12]<<24));
//	JDZ_Parameter.Circle_Pluse[Z_Axsis] = (u32)(((u32)IIC_Parameter[13])|((u32)IIC_Parameter[14]<<8)|((u32)IIC_Parameter[15]<<16)|((u32)IIC_Parameter[16]<<24));
//	JDZ_Parameter.Circle_Pluse[O_Axsis] = (u32)(((u32)IIC_Parameter[17])|((u32)IIC_Parameter[18]<<8)|((u32)IIC_Parameter[19]<<16)|((u32)IIC_Parameter[20]<<24));
//	JDZ_Parameter.Motion_Dir[X_Axsis]	=  IIC_Parameter[21];	
//	JDZ_Parameter.Motion_Dir[L_Axsis]	=  IIC_Parameter[22];	
//	JDZ_Parameter.Motion_Dir[Z_Axsis]	=  IIC_Parameter[23];	
//	JDZ_Parameter.Motion_Dir[O_Axsis]	=  IIC_Parameter[24];
//	JDZ_Parameter.OriginSetting[X_Axsis]=  IIC_Parameter[25];	
//	JDZ_Parameter.OriginSetting[L_Axsis]=  IIC_Parameter[26];	
//	JDZ_Parameter.OriginSetting[Z_Axsis]=  IIC_Parameter[27];	
//	JDZ_Parameter.OriginSetting[O_Axsis]=  IIC_Parameter[28];
	AxsisMoveCoefChange();
	
	//读取软限位参数
	for(i = 0; i < SAVESOFTLIMIT + 1; i++)//增加的1为X-O轴安全距离参数
	{
		if(i < 4)
		{
			W25QXX_Read(IIC_Parameter, 0x2000+i*0x09, 9);
			Robot_SoftLimit[i].Left_Limit  = (u32)(((u32)IIC_Parameter[0])|((u32)IIC_Parameter[1]<<8)|((u32)IIC_Parameter[2]<<16)|((u32)IIC_Parameter[3]<<24));
			Robot_SoftLimit[i].Right_Limit = (u32)(((u32)IIC_Parameter[4])|((u32)IIC_Parameter[5]<<8)|((u32)IIC_Parameter[6]<<16)|((u32)IIC_Parameter[7]<<24));
			Robot_SoftLimit[i].Switch_Limit = IIC_Parameter[8];
			if(Robot_SoftLimit[i].Switch_Limit)
			{
				Axsis_Minlength[i] = (Robot_SoftLimit[i].Left_Limit * Step_Coefficient[i] / 100)+MINROBOTPOSITION;
				Axsis_Maxlength[i] = (Robot_SoftLimit[i].Right_Limit * Step_Coefficient[i] / 100)+MINROBOTPOSITION;
			}
			else
			{
				Axsis_Minlength[i] = MINROBOTPOSITION;
				Axsis_Maxlength[i] = MAXROBOTPOSITION;
			}
		}
		else if(i == 4)
		{
			W25QXX_Read(IIC_Parameter,0x2030,8);		
			Robot_SoftDistance.MaxDistance = (u32)(((u32)IIC_Parameter[0])|((u32)IIC_Parameter[1]<<8)|((u32)IIC_Parameter[2]<<16)|((u32)IIC_Parameter[3]<<24));
			Robot_SoftDistance.MinDistance = (u32)(((u32)IIC_Parameter[4])|((u32)IIC_Parameter[5]<<8)|((u32)IIC_Parameter[6]<<16)|((u32)IIC_Parameter[7]<<24));
			
			Robot_SoftDistance.MaxDistance = (Robot_SoftDistance.MaxDistance * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
			Robot_SoftDistance.MinDistance = (Robot_SoftDistance.MinDistance * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
		}
	}
	
	//读取安全区
	for(i = 0; i<SAVESAFEAREA; i++)                        
	{
		W25QXX_Read(IIC_Parameter,0x2100+i*0x20,17);
		Robot_Safe_Area[i].X_Left = (u32)(((u32)IIC_Parameter[0])|((u32)IIC_Parameter[1]<<8)|((u32)IIC_Parameter[2]<<16)|((u32)IIC_Parameter[3]<<24));
		Robot_Safe_Area[i].X_Left = (Robot_Safe_Area[i].X_Left * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;		
													  
		Robot_Safe_Area[i].Z_Up = (u32)(((u32)IIC_Parameter[4])|((u32)IIC_Parameter[5]<<8)|((u32)IIC_Parameter[6]<<16)|((u32)IIC_Parameter[7]<<24));
		Robot_Safe_Area[i].Z_Up = (Robot_Safe_Area[i].Z_Up * Step_Coefficient[Z_Axsis] / 100) + MINROBOTPOSITION;	
		
		Robot_Safe_Area[i].X_Right = (u32)(((u32)IIC_Parameter[8])|((u32)IIC_Parameter[9]<<8)|((u32)IIC_Parameter[10]<<16)|((u32)IIC_Parameter[11]<<24));
		Robot_Safe_Area[i].X_Right = (Robot_Safe_Area[i].X_Right * Step_Coefficient[X_Axsis] / 100) + MINROBOTPOSITION;
		
		Robot_Safe_Area[i].Z_Down = (u32)(((u32)IIC_Parameter[12])|((u32)IIC_Parameter[13]<<8)|((u32)IIC_Parameter[14]<<16)|((u32)IIC_Parameter[15]<<24));
		Robot_Safe_Area[i].Z_Down = (Robot_Safe_Area[i].Z_Down * Step_Coefficient[Z_Axsis] / 100) + MINROBOTPOSITION;
		
		Robot_Safe_Area[i].SafeArea_Switch = IIC_Parameter[16];
	}
	
	//读取用户变量
	for(i=0;i<USER_NUM;i++)
	{
		W25QXX_Read(IIC_Parameter,P_USER_ADDRESS+23*i,23);
		USER_Parameter.USER_Name1[i] = (u32)(((u32)IIC_Parameter[1])|((u32)IIC_Parameter[2]<<8)|((u32)IIC_Parameter[3]<<16)|((u32)IIC_Parameter[4]<<24));
		USER_Parameter.USER_Name2[i] = (u32)(((u32)IIC_Parameter[5])|((u32)IIC_Parameter[6]<<8)|((u32)IIC_Parameter[7]<<16)|((u32)IIC_Parameter[8]<<24));
		USER_Parameter.USER_Name3[i] = (u32)(((u32)IIC_Parameter[9])|((u32)IIC_Parameter[10]<<8)|((u32)IIC_Parameter[11]<<16)|((u32)IIC_Parameter[12]<<24));
		USER_Parameter.INIT_Num[i] = (s32)(((u32)IIC_Parameter[13])|((u32)IIC_Parameter[14]<<8)|((u32)IIC_Parameter[15]<<16)|((u32)IIC_Parameter[16]<<24));
//		USER_Parameter.CURR_Num[i] = (s32)(((u32)IIC_Parameter[17])|((u32)IIC_Parameter[18]<<8)|((u32)IIC_Parameter[19]<<16)|((u32)IIC_Parameter[20]<<24));
		USER_Parameter.ELEC_RESET[i]  = IIC_Parameter[21];
		USER_Parameter.START_RESET[i] = IIC_Parameter[22];
		if(USER_Parameter.ELEC_RESET[i] == TRUE)
		{
			USER_Parameter.CURR_Num[i] = USER_Parameter.INIT_Num[i];
		}
	}
	
	//点读取		
	for(i=0; i<40; i++)
	{				
		W25QXX_Read(&IIC_Parameter[3], P_POINT_SAVE_HEAD + i * P_POINT_SAVE_LEN + 1, 29);		//+1:不读取编号
		Manul_Save_Point[i].Flag = IIC_Parameter[3];
		Manul_Save_Point[i].Name = (u32)(((u32)IIC_Parameter[4])|((u32)IIC_Parameter[5]<<8)|((u32)IIC_Parameter[6]<<16)|((u32)IIC_Parameter[7]<<24));
		Manul_Save_Point[i].Name2 = (u32)(((u32)IIC_Parameter[8])|((u32)IIC_Parameter[9]<<8)|((u32)IIC_Parameter[10]<<16)|((u32)IIC_Parameter[11]<<24));
		Manul_Save_Point[i].Name3 = (u32)(((u32)IIC_Parameter[12])|((u32)IIC_Parameter[13]<<8)|((u32)IIC_Parameter[14]<<16)|((u32)IIC_Parameter[15]<<24));		
		Manul_Save_Point[i].Point_X = (u32)(((u32)IIC_Parameter[16])|((u32)IIC_Parameter[17]<<8)|((u32)IIC_Parameter[18]<<16)|((u32)IIC_Parameter[19]<<24));
		Manul_Save_Point[i].Point_L = (u32)(((u32)IIC_Parameter[20])|((u32)IIC_Parameter[21]<<8)|((u32)IIC_Parameter[22]<<16)|((u32)IIC_Parameter[23]<<24));
		Manul_Save_Point[i].Point_Z = (u32)(((u32)IIC_Parameter[24])|((u32)IIC_Parameter[25]<<8)|((u32)IIC_Parameter[26]<<16)|((u32)IIC_Parameter[27]<<24));
		Manul_Save_Point[i].Point_O = (u32)(((u32)IIC_Parameter[28])|((u32)IIC_Parameter[29]<<8)|((u32)IIC_Parameter[30]<<16)|((u32)IIC_Parameter[31]<<24));	
	}
	
	for(i=0; i<SAVEPROGRAMNUM_SUB; i++)
	{//初始化子程序语句执行超时时间
		g_SubProgram_ActionTimeOut_Time[i] = 3000;
	}
	
	JDZ_OriginRead();
}


/*************************************************************************
**  函数名：  Write_System_Set_IIC()
**	输入参数：无
**	输出参数：无
**	函数功能：向指定地址写16个字节数据
**	备注：	  无
**  作者：       
**  开发日期：
**************************************************************************/
void Write_System_Set_IIC(void)
{
	u32 address = 0;

	if(UsartReceiveData[1]<27)
	{
		address = 0x1090+50*UsartReceiveData[1];			//获取地址
	}
	else if(UsartReceiveData[1]<35)
	{
		address = 0x2000+50*(UsartReceiveData[1]-27);			//获取地址
	}
	else if(UsartReceiveData[1]<62)
	{
		address = 0x5000+50*(UsartReceiveData[1]-35);			//获取地址
	}
	else if(UsartReceiveData[1]<126)
	{
		address = 0x7330+50*(UsartReceiveData[1]-62);		//获取地址
	}
	else
	{
		address = P_USER_ADDRESS+50*(UsartReceiveData[1]-126);			//获取地址
	}
	
	W25QXX_Write(&UsartReceiveData[2], address, 50);
}

/*************************************************************************
**  函数名：  Write_MDPara_Copy_IIC()
**	输入参数：无
**	输出参数：无
**	函数功能：向指定地址写16个字节数据
**	备注：	  无
**  作者：       
**  开发日期：
**************************************************************************/
void Write_MDPara_Copy_IIC(void)
{
	u8 code = 0;
	u8 lenth = 0;
	
	m_Copy_Time = (u16)(((u16)UsartReceiveData[1])|((u16)UsartReceiveData[2]<<8));
	code = UsartReceiveData[3] - 1;
	
	if(m_Copy_Time < P_MD_GOOD_LEN/50)
	{
		lenth = 50;
	}
	else if(m_Copy_Time == P_MD_GOOD_LEN/50)
	{
		lenth = P_MD_GOOD_LEN%50;
	}
	W25QXX_Write(&UsartReceiveData[4],P_MD_PARA_HEAD+P_MD_GOOD_LEN*code+50*m_Copy_Time,lenth);

}


/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/

