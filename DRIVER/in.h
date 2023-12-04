//******************************************************//
//*****************  输入信号检测  *********************//
//**   输入信号全部JHR210的输入，后面标注为JHR105     **//
//**   的输入信号为小型号机床使用的输入               **//
//******************************************************//
#ifndef __in_h_
#define __in_h_

#include "stm32f4xx.h"
/*输入口的端口及端口号*/
#define X0_IN_PORT									(GPIOA)						//X0引脚为GPIOA
#define X0_IN_PORT_NUM							(GPIO_Pin_3)			//X0引脚诤盼狿A3
#define X1_IN_PORT									(GPIOA)						//X1引脚为GPIOA
#define X1_IN_PORT_NUM							(GPIO_Pin_4)			//X1引脚号为PA4
#define X2_IN_PORT									(GPIOE)						//X2引脚为GPIOE
#define X2_IN_PORT_NUM							(GPIO_Pin_8)			//X2引脚号为PE8
#define X3_IN_PORT									(GPIOE)						//X3引脚为GPIOE
#define X3_IN_PORT_NUM							(GPIO_Pin_9)			//X3引脚号为PE9
#define X4_IN_PORT									(GPIOE)						//X4引脚为GPIOE
#define X4_IN_PORT_NUM							(GPIO_Pin_10)			//X4引脚号为PE10
#define X5_IN_PORT									(GPIOE)						//X5引脚为GPIOE
#define X5_IN_PORT_NUM							(GPIO_Pin_11)			//X5引脚号为PE11
#define X6_IN_PORT									(GPIOE)						//X6引脚为GPIOE
#define X6_IN_PORT_NUM							(GPIO_Pin_12)			//X6引脚号为PE12
#define X7_IN_PORT									(GPIOE)						//X7引脚为GPIOE
#define X7_IN_PORT_NUM							(GPIO_Pin_13)			//X7引脚号为PE13
#define X8_IN_PORT									(GPIOE)						//X8引脚为GPIOE
#define X8_IN_PORT_NUM							(GPIO_Pin_14)			//X8引脚号为PE14
#define X9_IN_PORT									(GPIOE)						//X9引脚为GPIOE
#define X9_IN_PORT_NUM							(GPIO_Pin_15)			//X9引脚号为PE15
#define X10_IN_PORT									(GPIOB)						//X10引脚为GPIOB
#define X10_IN_PORT_NUM							(GPIO_Pin_10)			//X10引脚号为PB10
#define X11_IN_PORT									(GPIOB)						//X11引脚为GPIOB
#define X11_IN_PORT_NUM							(GPIO_Pin_14)			//X11引脚号为PB14
#define X12_IN_PORT									(GPIOA)						//X12引脚为GPIOA
#define X12_IN_PORT_NUM							(GPIO_Pin_6)			//X12引脚号为PA6
#define X13_IN_PORT									(GPIOB)						//X13引脚为GPIOB
#define X13_IN_PORT_NUM							(GPIO_Pin_0)			//X13引脚号为PB0
#define X14_IN_PORT									(GPIOB)						//X14引脚为GPIOB
#define X14_IN_PORT_NUM							(GPIO_Pin_1)			//X14引脚号为PB1
#define X15_IN_PORT									(GPIOE)						//X15引脚为GPIOE
#define X15_IN_PORT_NUM							(GPIO_Pin_7)			//X15引脚号为PE7
#define X16_IN_PORT									(GPIOB)						//X16引脚为GPIOB
#define X16_IN_PORT_NUM							(GPIO_Pin_15)			//X16引脚号为PB15
#define X17_IN_PORT									(GPIOD)						//X17引脚为GPIOD
#define X17_IN_PORT_NUM							(GPIO_Pin_8)			//X17引脚号为PD8
#define X18_IN_PORT									(GPIOD)						//X18引脚为GPIOD
#define X18_IN_PORT_NUM							(GPIO_Pin_9)			//X18引脚号为PD9
#define X19_IN_PORT									(GPIOD)						//X19引脚为GPIOD
#define X19_IN_PORT_NUM							(GPIO_Pin_10)			//X19引脚号为PD10
#define X20_IN_PORT									(GPIOD)						//X20引脚为GPIOD
#define X20_IN_PORT_NUM							(GPIO_Pin_11)			//X20引脚号为PD11
#define X21_IN_PORT									(GPIOD)						//X21引脚为GPIOD
#define X21_IN_PORT_NUM							(GPIO_Pin_12)			//X21引脚号为PD12
#define X22_IN_PORT									(GPIOD)						//X22引脚为GPIOD
#define X22_IN_PORT_NUM							(GPIO_Pin_13)			//X22引脚号为PD13
#define X23_IN_PORT									(GPIOD)						//X23引脚为GPIOD
#define X23_IN_PORT_NUM							(GPIO_Pin_14)			//X23引脚号为PD14

/*普通输入检测口的电平设置*/
#define X0_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[0] = (Input_Detect_Status[0]|0x01))	//X0检测信号低电平置位
#define X1_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[0] = (Input_Detect_Status[0]|0x02))	//X1检测信号低电平置位
#define X2_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[0] = (Input_Detect_Status[0]|0x04))	//X2检测信号低电平置位
#define X3_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[0] = (Input_Detect_Status[0]|0x08))	//X3检测信号低电平置位
#define X4_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[0] = (Input_Detect_Status[0]|0x10))	//X4检测信号低电平置位
#define X5_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[0] = (Input_Detect_Status[0]|0x20))	//X5检测信号低电平置位
#define X6_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[0] = (Input_Detect_Status[0]|0x40))	//X6检测信号低电平置位
#define X7_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[0] = (Input_Detect_Status[0]|0x80))	//X7检测信号低电平置位
#define X8_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[1] = (Input_Detect_Status[1]|0x01))	//X8检测信号低电平置位
#define X9_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[1] = (Input_Detect_Status[1]|0x02))	//X9检测信号低电平置位
#define X10_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[1] = (Input_Detect_Status[1]|0x04))	//X10检测信号低电平置位
#define X11_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[1] = (Input_Detect_Status[1]|0x08))	//X11检测信号低电平置位
#define X12_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[1] = (Input_Detect_Status[1]|0x10))	//X12检测信号低电平置位
#define X13_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[1] = (Input_Detect_Status[1]|0x20))	//X13检测信号低电平置位
#define X14_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[1] = (Input_Detect_Status[1]|0x40))	//X14检测信号低电平置位
#define X15_IN_PORT_LOW_LEVEL_SET	  	(Input_Detect_Status[1] = (Input_Detect_Status[1]|0x80))	//X15检测信号低电平置位
#define X16_IN_PORT_LOW_LEVEL_SET   	(Input_Detect_Status[2] = (Input_Detect_Status[2]|0x01))	//X16检测信号低电平置位
#define X17_IN_PORT_LOW_LEVEL_SET		  (Input_Detect_Status[2] = (Input_Detect_Status[2]|0x02))	//X17检测信号低电平置位
#define X18_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[2] = (Input_Detect_Status[2]|0x04))	//X18检测信号低电平置位
#define X19_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[2] = (Input_Detect_Status[2]|0x08))	//X19检测信号低电平置位
#define X20_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[2] = (Input_Detect_Status[2]|0x10))	//X20检测信号低电平置位
#define X21_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[2] = (Input_Detect_Status[2]|0x20))	//X21检测信号低电平置位
#define X22_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[2] = (Input_Detect_Status[2]|0x40))	//X22检测信号低电平置位
#define X23_IN_PORT_LOW_LEVEL_SET			(Input_Detect_Status[2] = (Input_Detect_Status[2]|0x80))	//X23检测信号低电平置位

#define X0_IN_PORT_HIGH_LEVEL_RESET			(Input_Detect_Status[0] = (Input_Detect_Status[0]&0xfe))	//X0检测信号高电平复位
#define X1_IN_PORT_HIGH_LEVEL_RESET			(Input_Detect_Status[0] = (Input_Detect_Status[0]&0xfd))	//X1检测信号高电平复位
#define X2_IN_PORT_HIGH_LEVEL_RESET			(Input_Detect_Status[0] = (Input_Detect_Status[0]&0xfb))	//X2检测信号高电平复位
#define X3_IN_PORT_HIGH_LEVEL_RESET			(Input_Detect_Status[0] = (Input_Detect_Status[0]&0xf7))	//X3检测信号高电平复位
#define X4_IN_PORT_HIGH_LEVEL_RESET			(Input_Detect_Status[0] = (Input_Detect_Status[0]&0xef))	//X4检测信号高电平复位
#define X5_IN_PORT_HIGH_LEVEL_RESET			(Input_Detect_Status[0] = (Input_Detect_Status[0]&0xdf))	//X5检测信号高电平复位
#define X6_IN_PORT_HIGH_LEVEL_RESET			(Input_Detect_Status[0] = (Input_Detect_Status[0]&0xbf))	//X6检测信号高电平复位
#define X7_IN_PORT_HIGH_LEVEL_RESET			(Input_Detect_Status[0] = (Input_Detect_Status[0]&0x7f))	//X7检测信号高电平复位
#define X8_IN_PORT_HIGH_LEVEL_RESET			(Input_Detect_Status[1] = (Input_Detect_Status[1]&0xfe))	//X8检测信号高电平复位
#define X9_IN_PORT_HIGH_LEVEL_RESET			(Input_Detect_Status[1] = (Input_Detect_Status[1]&0xfd))	//X9检测信号高电平复位
#define X10_IN_PORT_HIGH_LEVEL_RESET		(Input_Detect_Status[1] = (Input_Detect_Status[1]&0xfb))	//X10检测信号高电平复位
#define X11_IN_PORT_HIGH_LEVEL_RESET		(Input_Detect_Status[1] = (Input_Detect_Status[1]&0xf7))	//X11检测信号高电平复位
#define X12_IN_PORT_HIGH_LEVEL_RESET		(Input_Detect_Status[1] = (Input_Detect_Status[1]&0xef))	//X12检测信号高电平复位
#define X13_IN_PORT_HIGH_LEVEL_RESET		(Input_Detect_Status[1] = (Input_Detect_Status[1]&0xdf))	//X13检测信号高电平复位
#define X14_IN_PORT_HIGH_LEVEL_RESET		(Input_Detect_Status[1] = (Input_Detect_Status[1]&0xbf))	//X14检测信号高电平复位
#define X15_IN_PORT_HIGH_LEVEL_RESET  	(Input_Detect_Status[1] = (Input_Detect_Status[1]&0x7f))	//X15检测信号高电平复位
#define X16_IN_PORT_HIGH_LEVEL_RESET 		(Input_Detect_Status[2] = (Input_Detect_Status[2]&0xfe))	//X16检测信号高电平复位
#define X17_IN_PORT_HIGH_LEVEL_RESET 		(Input_Detect_Status[2] = (Input_Detect_Status[2]&0xfd))	//X17检测信号高电平复位
#define X18_IN_PORT_HIGH_LEVEL_RESET		(Input_Detect_Status[2] = (Input_Detect_Status[2]&0xfb))	//X18检测信号高电平复位
#define X19_IN_PORT_HIGH_LEVEL_RESET		(Input_Detect_Status[2] = (Input_Detect_Status[2]&0xf7))	//X19检测信号高电平复位
#define X20_IN_PORT_HIGH_LEVEL_RESET		(Input_Detect_Status[2] = (Input_Detect_Status[2]&0xef))	//X20检测信号高电平复位
#define X21_IN_PORT_HIGH_LEVEL_RESET		(Input_Detect_Status[2] = (Input_Detect_Status[2]&0xdf))	//X21检测信号高电平复位
#define X22_IN_PORT_HIGH_LEVEL_RESET		(Input_Detect_Status[2] = (Input_Detect_Status[2]&0xbf))	//X22检测信号高电平复位
#define X23_IN_PORT_HIGH_LEVEL_RESET		(Input_Detect_Status[2] = (Input_Detect_Status[2]&0x7f))	//X23检测信号高电平复位


/*普通输入检测口的电平读取*/
#define X0_IN_PORT_LOW_LEVEL			(Input_Detect_Status[0]&0x01)	//X0检测信号低电平检测
#define X1_IN_PORT_LOW_LEVEL			(Input_Detect_Status[0]&0x02)	//X1检测信号低电平检测
#define X2_IN_PORT_LOW_LEVEL			(Input_Detect_Status[0]&0x04)	//X2检测信号低电平检测
#define X3_IN_PORT_LOW_LEVEL			(Input_Detect_Status[0]&0x08)	//X3检测信号低电平检测
#define X4_IN_PORT_LOW_LEVEL			(Input_Detect_Status[0]&0x10)	//X4检测信号低电平检测
#define X5_IN_PORT_LOW_LEVEL			(Input_Detect_Status[0]&0x20)	//X5检测信号低电平检测
#define X6_IN_PORT_LOW_LEVEL			(Input_Detect_Status[0]&0x40)	//X6检测信号低电平检测
#define X7_IN_PORT_LOW_LEVEL			(Input_Detect_Status[0]&0x80)	//X7检测信号低电平检测
#define X8_IN_PORT_LOW_LEVEL			(Input_Detect_Status[1]&0x01)	//X8检测信号低电平检测
#define X9_IN_PORT_LOW_LEVEL			(Input_Detect_Status[1]&0x02)	//X9检测信号低电平检测
#define X10_IN_PORT_LOW_LEVEL			(Input_Detect_Status[1]&0x04)	//X10检测信号低电平检测
#define X11_IN_PORT_LOW_LEVEL			(Input_Detect_Status[1]&0x08)	//X11检测信号低电平检测
#define X12_IN_PORT_LOW_LEVEL			(Input_Detect_Status[1]&0x10)	//X12检测信号低电平检测
#define X13_IN_PORT_LOW_LEVEL			(Input_Detect_Status[1]&0x20)	//X13检测信号低电平检测
#define X14_IN_PORT_LOW_LEVEL			(Input_Detect_Status[1]&0x40)	//X14检测信号低电平检测
#define X15_IN_PORT_LOW_LEVEL	  	(Input_Detect_Status[1]&0x80)	//X15检测信号低电平检测
#define X16_IN_PORT_LOW_LEVEL   	(Input_Detect_Status[2]&0x01)	//X16检测信号低电平检测
#define X17_IN_PORT_LOW_LEVEL		  (Input_Detect_Status[2]&0x02)	//X17检测信号低电平检测
#define X18_IN_PORT_LOW_LEVEL			(Input_Detect_Status[2]&0x04)	//X18检测信号低电平检测
#define X19_IN_PORT_LOW_LEVEL			(Input_Detect_Status[2]&0x08)	//X19检测信号低电平检测
#define X20_IN_PORT_LOW_LEVEL			(Input_Detect_Status[2]&0x10)	//X20检测信号低电平检测
#define X21_IN_PORT_LOW_LEVEL			(Input_Detect_Status[2]&0x20)	//X21检测信号低电平检测
#define X22_IN_PORT_LOW_LEVEL			(Input_Detect_Status[2]&0x40)	//X22检测信号低电平检测
#define X23_IN_PORT_LOW_LEVEL			(Input_Detect_Status[2]&0x80)	//X23检测信号低电平检测

#define X0_IN_PORT_HIGH_LEVEL			(Input_Detect_Status[0]|0xfe)	//X0检测信号高电平检测
#define X1_IN_PORT_HIGH_LEVEL			(Input_Detect_Status[0]|0xfd)	//X1检测信号高电平检测
#define X2_IN_PORT_HIGH_LEVEL			(Input_Detect_Status[0]|0xfb)	//X2检测信号高电平检测
#define X3_IN_PORT_HIGH_LEVEL			(Input_Detect_Status[0]|0xf7)	//X3检测信号高电平检测
#define X4_IN_PORT_HIGH_LEVEL			(Input_Detect_Status[0]|0xef)	//X4检测信号高电平检测
#define X5_IN_PORT_HIGH_LEVEL			(Input_Detect_Status[0]|0xdf)	//X5检测信号高电平检测
#define X6_IN_PORT_HIGH_LEVEL			(Input_Detect_Status[0]|0xbf)	//X6检测信号高电平检测
#define X7_IN_PORT_HIGH_LEVEL			(Input_Detect_Status[0]|0x7f)	//X7检测信号高电平检测
#define X8_IN_PORT_HIGH_LEVEL			(Input_Detect_Status[1]|0xfe)	//X8检测信号高电平检测
#define X9_IN_PORT_HIGH_LEVEL			(Input_Detect_Status[1]|0xfd)	//X9检测信号高电平检测
#define X10_IN_PORT_HIGH_LEVEL		(Input_Detect_Status[1]|0xfb)	//X10检测信号高电平检测
#define X11_IN_PORT_HIGH_LEVEL		(Input_Detect_Status[1]|0xf7)	//X11检测信号高电平检测
#define X12_IN_PORT_HIGH_LEVEL		(Input_Detect_Status[1]|0xef)	//X12检测信号高电平检测
#define X13_IN_PORT_HIGH_LEVEL		(Input_Detect_Status[1]|0xdf)	//X13检测信号高电平检测
#define X14_IN_PORT_HIGH_LEVEL		(Input_Detect_Status[1]|0xbf)	//X14检测信号高电平检测
#define X15_IN_PORT_HIGH_LEVEL  	(Input_Detect_Status[1]|0x7f)	//X15检测信号高电平检测
#define X16_IN_PORT_HIGH_LEVEL 		(Input_Detect_Status[2]|0xfe)	//X16检测信号高电平检测
#define X17_IN_PORT_HIGH_LEVEL 		(Input_Detect_Status[2]|0xfd)	//X17检测信号高电平检测
#define X18_IN_PORT_HIGH_LEVEL		(Input_Detect_Status[2]|0xfb)	//X18检测信号高电平检测
#define X19_IN_PORT_HIGH_LEVEL		(Input_Detect_Status[2]|0xf7)	//X19检测信号高电平检测
#define X20_IN_PORT_HIGH_LEVEL		(Input_Detect_Status[2]|0xef)	//X20检测信号高电平检测
#define X21_IN_PORT_HIGH_LEVEL		(Input_Detect_Status[2]|0xdf)	//X21检测信号高电平检测
#define X22_IN_PORT_HIGH_LEVEL		(Input_Detect_Status[2]|0xbf)	//X22检测信号高电平检测
#define X23_IN_PORT_HIGH_LEVEL		(Input_Detect_Status[2]|0x7f)	//X23检测信号高电平检测

//#define X_AXIS_MOVE_SIGN_LOW_LEVEL					(X1_IN_PORT_LOW_LEVEL == 0x02)	//X1检测信号低电平,X轴搜索
//#define Y_AXIS_MOVE_SIGN_LOW_LEVEL					(X2_IN_PORT_LOW_LEVEL == 0x04)	//X2检测信号低电平,Y轴搜索
//#define Z_AXIS_MOVE_SIGN_LOW_LEVEL					(X3_IN_PORT_LOW_LEVEL == 0x08)	//X3检测信号低电平,Z轴搜索
//#define O_AXIS_MOVE_SIGN_LOW_LEVEL					(X4_IN_PORT_LOW_LEVEL == 0x10)	//X4检测信号低电平,O轴搜索
//#define U_AXIS_MOVE_SIGN_LOW_LEVEL					(X5_IN_PORT_LOW_LEVEL == 0x20)	//X5检测信号低电平,U轴搜索
//#define V_AXIS_MOVE_SIGN_LOW_LEVEL					(X6_IN_PORT_LOW_LEVEL == 0x40)	//X6检测信号低电平,V轴搜索

//#define X_AXIS_MOVE_SIGN_HIGH_LEVEL					(X1_IN_PORT_HIGH_LEVEL == 0xfd)	//X1检测信号高电平,X轴搜索
//#define Y_AXIS_MOVE_SIGN_HIGH_LEVEL					(X2_IN_PORT_HIGH_LEVEL == 0xfb)	//X2检测信号高电平,Y轴搜索
//#define Z_AXIS_MOVE_SIGN_HIGH_LEVEL					(X3_IN_PORT_HIGH_LEVEL == 0xf7)	//X3检测信号高电平,Z轴搜索
//#define O_AXIS_MOVE_SIGN_HIGH_LEVEL					(X4_IN_PORT_HIGH_LEVEL == 0xef)	//X4检测信号高电平,O轴搜索
//#define U_AXIS_MOVE_SIGN_HIGH_LEVEL					(X5_IN_PORT_HIGH_LEVEL == 0xdf)	//X5检测信号高电平,U轴搜索
//#define V_AXIS_MOVE_SIGN_HIGH_LEVEL					(X6_IN_PORT_HIGH_LEVEL == 0xbf)	//X6检测信号高电平,V轴搜索


/*普通输入的复用功能定义*/
#define Z_AXIS_BUMP_SIGN_LOW_LEVEL					(X7_IN_PORT_LOW_LEVEL == 0x80)	//X7检测信号低电平,Z轴防撞
#define Z_AXIS_BUMP_SIGN_HIGH_LEVEL					(X7_IN_PORT_HIGH_LEVEL == 0x7f)	//X7检测信号高电平,Z轴防撞

#define EXTERNAL_ORIGIN_SIGN_LOW_LEVEL			(X8_IN_PORT_LOW_LEVEL == 0x01)	//X8检测信号高电平,外部回零
#define EXTERNAL_START_SIGN_LOW_LEVEL				(X9_IN_PORT_LOW_LEVEL == 0x02)	//X9检测信号高电平,外部启动
#define EXTERNAL_PAUSE_SIGN_LOW_LEVEL				(X10_IN_PORT_LOW_LEVEL == 0x04)	//X10检测信号高电平,外部暂停
#define EXTERNAL_STOP_SIGN_LOW_LEVEL				(X11_IN_PORT_LOW_LEVEL == 0x08)	//X11检测信号高电平,外部停止

#define EXTERNAL_ORIGIN_SIGN_HIGH_LEVEL			(X8_IN_PORT_HIGH_LEVEL == 0xfe)	//X8检测信号高电平,外部回零
#define EXTERNAL_START_SIGN_HIGH_LEVEL			(X9_IN_PORT_HIGH_LEVEL == 0xfd)	//X9检测信号高电平,外部启动
#define EXTERNAL_PAUSE_SIGN_HIGH_LEVEL			(X10_IN_PORT_HIGH_LEVEL == 0xfb)//X10检测信号高电平,外部暂停
#define EXTERNAL_STOP_SIGN_HIGH_LEVEL				(X11_IN_PORT_HIGH_LEVEL == 0xf7)//X11检测信号高电平,外部停止

#define X_AXIS_MOVE_SIGN_LOW_LEVEL					(X12_IN_PORT_LOW_LEVEL == 0x10)	//X12检测信号低电平,X轴搜索
#define Y_AXIS_MOVE_SIGN_LOW_LEVEL					(X13_IN_PORT_LOW_LEVEL == 0x20)	//X13检测信号低电平,Y轴搜索
#define Z_AXIS_MOVE_SIGN_LOW_LEVEL					(X14_IN_PORT_LOW_LEVEL == 0x40)	//X14检测信号低电平,Z轴搜索
#define O_AXIS_MOVE_SIGN_LOW_LEVEL	  			(X15_IN_PORT_LOW_LEVEL == 0x80)	//X15检测信号低电平,O轴搜索

#define X_AXIS_MOVE_SIGN_HIGH_LEVEL					(X12_IN_PORT_HIGH_LEVEL == 0xef)	//X12检测信号高电平,X轴搜索
#define Y_AXIS_MOVE_SIGN_HIGH_LEVEL					(X13_IN_PORT_HIGH_LEVEL == 0xdf)	//X13检测信号高电平,Y轴搜索
#define Z_AXIS_MOVE_SIGN_HIGH_LEVEL					(X14_IN_PORT_HIGH_LEVEL == 0xbf)	//X14检测信号高电平,Z轴搜索
#define O_AXIS_MOVE_SIGN_HIGH_LEVEL  				(X15_IN_PORT_HIGH_LEVEL == 0x7f)	//X15检测信号高电平,O轴搜索

#define X_AXIS_HARD_MIN_LIMIT_LOW_LEVEL   	(X16_IN_PORT_LOW_LEVEL == 0x01)	//X16检测信号低电平,X轴最小硬限位
#define Y_AXIS_HARD_MIN_LIMIT_LOW_LEVEL		  (X17_IN_PORT_LOW_LEVEL == 0x02)	//X17检测信号低电平,Y轴最小硬限位
#define Z_AXIS_HARD_MIN_LIMIT_LOW_LEVEL			(X18_IN_PORT_LOW_LEVEL == 0x04)	//X18检测信号低电平,Z轴最大硬限位
#define O_AXIS_HARD_MIN_LIMIT_LOW_LEVEL			(X19_IN_PORT_LOW_LEVEL == 0x08)	//X19检测信号低电平,O轴最大硬限位

#define X_AXIS_HARD_MIN_LIMIT_HIGH_LEVEL 		(X16_IN_PORT_HIGH_LEVEL == 0xfe)	//X16检测信号高电平,X轴最小硬限位
#define Y_AXIS_HARD_MIN_LIMIT_HIGH_LEVEL 		(X17_IN_PORT_HIGH_LEVEL == 0xfd)	//X17检测信号高电平,Y轴最小硬限位
#define Z_AXIS_HARD_MIN_LIMIT_HIGH_LEVEL		(X18_IN_PORT_HIGH_LEVEL == 0xfb)	//X18检测信号高电平,Z轴最大硬限位
#define O_AXIS_HARD_MIN_LIMIT_HIGH_LEVEL		(X19_IN_PORT_HIGH_LEVEL == 0xf7)	//X19检测信号高电平,O轴最大硬限位

#define X_AXIS_HARD_MAX_LIMIT_LOW_LEVEL			(X20_IN_PORT_LOW_LEVEL == 0x10)	//X20检测信号低电平,X轴最大硬限位
#define Y_AXIS_HARD_MAX_LIMIT_LOW_LEVEL			(X21_IN_PORT_LOW_LEVEL == 0x20)	//X21检测信号低电平,Y轴最大硬限位
#define Z_AXIS_HARD_MAX_LIMIT_LOW_LEVEL			(X22_IN_PORT_LOW_LEVEL == 0x40)	//X22检测信号低电平,Z轴最大硬限位
#define O_AXIS_HARD_MAX_LIMIT_LOW_LEVEL			(X23_IN_PORT_LOW_LEVEL == 0x80)	//X23检测信号低电平,O轴最大硬限位

#define X_AXIS_HARD_MAX_LIMIT_HIGH_LEVEL		(X20_IN_PORT_HIGH_LEVEL == 0xef)	//X20检测信号高电平,X轴最大硬限位
#define Y_AXIS_HARD_MAX_LIMIT_HIGH_LEVEL		(X21_IN_PORT_HIGH_LEVEL == 0xdf)	//X21检测信号高电平,Y轴最大硬限位
#define Z_AXIS_HARD_MAX_LIMIT_HIGH_LEVEL		(X22_IN_PORT_HIGH_LEVEL == 0xbf)	//X22检测信号高电平,Z轴最大硬限位
#define O_AXIS_HARD_MAX_LIMIT_HIGH_LEVEL		(X23_IN_PORT_HIGH_LEVEL == 0x7f)	//X23检测信号高电平,O轴最大硬限位

/*急停信号状态，不滤波*/
#define EMERGENCY_STOP_STATUS								(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_15))	//急停信号状态

/*硬限位端口编号定义*/
#define  I_DETECT_X_MIN_LIMIT		16					//X轴最小限位信号的端口号X16
#define  I_DETECT_Y_MIN_LIMIT		17					//Y轴最小限位信号的端口号X17
#define  I_DETECT_Z_MIN_LIMIT		18					//Z轴最小限位信号的端口号X18
#define  I_DETECT_O_MIN_LIMIT		19					//O轴最小限位信号的端口号X19

#define  I_DETECT_X_MAX_LIMIT		20					//X轴最大限位信号的端口号X20
#define  I_DETECT_Y_MAX_LIMIT		21					//Y轴最大限位信号的端口号X21
#define  I_DETECT_Z_MAX_LIMIT		22					//Z轴最大限位信号的端口号X22
#define  I_DETECT_O_MAX_LIMIT		23					//O轴最大限位信号的端口号X23

extern u8 Input_Detect_Status[6];					//输入状态保存

extern u8 ReadInput(u8 IO_Num);
extern u8 Axis_ServoAlarm(u8 Axsis);
extern u8 ReadEmergencyStop(void);
extern u8 ReadIOPort(GPIO_TypeDef* GPIOx, u16 PortNum, u8 PortPreviousStatus);

void InputInit(void);		//输入口初始化

#endif
/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/
