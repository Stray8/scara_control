/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __auto_h_
#define __auto_h_

#define ONCE_MODE   0x01	  //单次模式
#define SINGLE_MODE 0x02    //单步模式
#define LOOP_MODE   0x03	  //循环模式
#define AUTO_START  0x04	  //启动
#define AUTO_STOP   0x05	  //停止

#define RUN    0x00	   //运行
#define PAUSE  0x01	   //暂停
#define STOP   0x02    //停止

extern u32 Auto_Pulse_Count;
extern u8 Auto_Mode;			  //自动模式下的模式选择
extern u8 Single_Mode_Enable;
extern u8 Once_Mode_Enable;
extern u8 Loop_Mode_Enable;

extern u32 Action_Delay_Time;         	//机械手每次动作完成之后的延时，因为伺服器动作是跟随动作
//extern u8  Puls_Delay_Time[50];		 			//检测输入信号延时
//extern u8  Puls_Delay_Enable[50];	     	//检测输入延时使能标志位
//extern u8  Puls_Delay_Num;
extern u8  Action_Done_Flag ;		 				//伺服器动作完成标志
extern u8  Action_Delay_Flag;		 				//动作延时完成

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


