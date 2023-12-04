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

//工作模式
#define WAIT_MODE			  			0x00     //等待模式
#define AUTO_WORK_MODE        0x01	   //选择自动工作模式模式
#define MANUAL_WORK_MODE      0x02	   //选择手动工作模式模式
#define FREE_PROGRAM_MODE	  	0x03     //选择自由编程模式
#define IO_DEBUG_MODE		  		0x04	   //选择IO调试模式

//电机移动方向
#define NEGATIVE              0x00	   //反向
#define	POSITIVE              0x01	   //正向
#define NONE                  0x02	   //

//轴定义
#define Axis_Num							0x04	   //支持的轴个数
#define X_Axsis				  			0x00	   //X轴
#define L_Axsis				  			0x01	   //L轴
#define Z_Axsis				  			0x02	   //Z轴
#define O_Axsis				  			0x03	   //O轴
#define U_Axsis				  			0x04	   //U轴
#define V_Axsis				  			0x05	   //V轴

#define Ext_Axis_Num					0x02	   //支持的扩展轴个数
#define U_Ext_Axsis				  	0x00	   //U轴
#define V_Ext_Axsis				  	0x01	   //V轴

//扩展轴
#define EXTEN_AXIS_NUM	      0x02

//#define   NO_AXIS    0x00
//#define   X_AXIS     0x01
//#define   L_AXIS     0x02
//#define   Z_AXIS     0x03
//#define   O_AXIS     0x04

//回原点方式
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

//输出IO复位选择
#define Null				  			        0x00	   //未选择
#define Before_Origin				  			0x01	   //回零前
#define After_Origin				  			0x02	   //回零后
#define Common_Alarm				  			0x03	   //普通报警
#define Emerge_Alarm				  			0x04	   //急停报警
#define PAUSE_Select				  		  0x05	   //暂停
#define STOP_Select				  		    0x06	   //停止

extern u8  Origin_Backed;			  							//回原点完成
extern u8  Axsis_Origin_Backed[Axis_Num + Ext_Axis_Num];		//各个轴回零完成标志位
extern u8  Back_Origin_Flag; 		 	 						//回原点命令标志位
extern u8  Initialize_Finished;		  					//初始化完成
extern u8  Work_Status ;				  						//工作状态
extern u32 Input_Detect_Time;
extern u32 Communication_Time;
extern u8  OffLine_Flag;
extern u8  OnLineCommunicated_Flag;
extern u8  Input_Detect_Enable;
extern u8  Jog_Pause_Enable;
extern u8  g_Current_SafeArea_Num;	  				//安全区编号
extern u8  Axis_Manul_BackToOrigin;    				//手动回零
extern u8  Axsis_Chosen;				  						//运动轴选择
extern u8  Axsis_Move_Direction[Axis_Num + Ext_Axis_Num] ;	//运动轴方向选择
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
extern void ActionControl(void);	  			//动作控制
extern void StatusControl(void);	  			//输入状态检测
extern void Robot_Reset(void);
extern void Servo_Stop(u8);
extern u8 ManulSafeAreaDetec(void);
extern void ClosePulseReset(u8 Axis);

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

