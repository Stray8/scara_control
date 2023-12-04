/********************************* START OF FILE ***************************************
* File Name          : ExtendCom.c
* Author             : 
* Version            : 
* Date               : 
* Description        : This is the ....
***************************************************************************************/

#ifndef __ExtendCom_h_
#define __ExtendCom_h_

#include "stm32f4xx.h"

/*须在应用程序中处理以下几个变量*/
extern u16 ExtendProgramNum;							//通过通信选中的程序号
extern u8  ExtendEmergencyStop;				    //通过急停设备标志
extern u8  ExtendYieldChange;							//通过通信修改产量相关变量标志
extern u8  ExtendPosChange;								//通过通信修改坐标相关变量标志
extern u8  ExtendPosChangeNum;						//通过通信修改坐标的编号
extern u8  ExtendStateChange;							//通过通信修改坐标的编号
extern u8  ExtendCancleAlarm;
extern u8  ExtendSerialNum;								//通过通信烧录序列号

#define DIR_485_H  {GPIO_SetBits(GPIOD, GPIO_Pin_1);delay_ms(5);}
#define DIR_485_L  {GPIO_ResetBits(GPIOD, GPIO_Pin_1);}

/*广播发送时的ID*/
#define EXTEND_RADIO_ID									0x00        //485扩展功能的广播ID

/*扩展模块的接收数据标志*/
#define EXTEND_WAIT_REC									0x00        //待接收
#define EXTEND_START_REC								0x01        //开始接收
#define EXTEND_END_REC									0x02        //完成接收

/*扩展模块功能码定义*/
#define EXTEND_FUN_READ									0x03        //读命令-输入寄存器读取
#define EXTEND_FUN_WRITE								0x10        //写命令-多寄存器写入

/*485访问地址*/
//系统类
#define EXT_ADDR_SERIAL_NUM							0x0000      //序列号地址，可读不可写
#define EXT_ADDR_SYSTEM_VER							0x0006      //系统版本号地址，可读不可写
#define EXT_ADDR_PRODUC_FAC							0x000C      //生产厂家地址，可读不可写
#define EXT_ADDR_WRITE_SERIAL_NUM			  0x0012      //控制设备序列号地址，不可读可写

//时间类
#define EXT_ADDR_ONCE_T									0x0100      //当前产品单次加工时间地址，可读不可写
#define EXT_ADDR_POWER_T								0x0102      //开机时间地址，可读不可写
#define EXT_ADDR_RUN_T									0x0104      //运行时间地址，可读不可写
#define EXT_ADDR_TOTAL_POWER_T					0x0106      //总开机时间地址，可读不可写
#define EXT_ADDR_TOTAL_RUN_T						0x0108      //总运行时间地址，可读不可写

//状态类
#define EXT_ADDR_RUN_STA								0x0200      //当前运行状态地址，可读不可写
#define EXT_ADDR_RESET_STA							0x0201      //当前复位状态地址，可读不可写
#define EXT_ADDR_BACK_ORI_STA						0x0202      //当前回零状态地址，可读不可写
#define EXT_ADDR_ALARM_INFO							0x0203      //当前设备报警地址，可读不可写

//运行参数
#define EXT_ADDR_CUR_RUN_PRO						0x0300      //当前运行程序地址，可读不可写
#define EXT_ADDR_CUR_YIELD							0x0306      //当前产量地址，可读不可写
#define EXT_ADDR_TAR_YIELD							0x0308      //目标产量地址，可读不可写
#define EXT_ADDR_TOTAL_YIELD						0x030A      //总产量地址，可读可不写
#define EXT_ADDR_NG_YIELD								0x030C      //次品产量地址，可读不可写
#define EXT_ADDR_CHK_YIELD							0x030E      //抽检产量地址，可读可写

//控制类
//#define EXT_ADDR_RUN_STOP								0x0600      //控制设备启停地址，不可读可写
#define EXT_ADDR_INPUT_STA							0x0400      //当前设备输入地址，可读不可写 0x0400~0x04FF,预留256路
#define EXT_ADDR_OUT_CONTROL						0x0500      //控制设备输出地址，不可读可写 0x0500~0x05FF,预留256路

/*读设备当前位置地址*/
#define EXT_ADDR_CUR_POSITION_X					0x1000      //当前位置X地址，可读不可写
#define EXT_ADDR_CUR_POSITION_Y					0x1002      //当前位置Y地址，可读不可写
#define EXT_ADDR_CUR_POSITION_Z					0x1004      //当前位置Z地址，可读不可写
#define EXT_ADDR_CUR_POSITION_O					0x1006      //当前位置O地址，可读不可写

//控制设备状态地址
#define EXT_ADDR_BACK_ORI								0x1100      //设备回零地址，不可读可写
#define EXT_ADDR_RESET									0x1101      //设备复位地址，不可读可写
#define EXT_ADDR_RUN_AUTO								0x1102      //设备启动运行地址，不可读可写
#define EXT_ADDR_STOP_RUN								0x1103      //设备停止运行地址，不可读可写
#define EXT_ADDR_PAUSE_RUN							0x1104      //设备暂停运行地址，不可读可写
#define EXT_ADDR_EM_STOP								0x1105      //设备急停地址，不可读可写
#define EXT_ADDR_CANCEL_ALARM						0x1106      //取消设备报警，不可读可写

/*点坐标信息访问地址，范围0x2000~0x29FF，每个点占有0x40个地址*/
#define EXT_ADDR_POINT_1_X							0x2000      //坐标1的X地址，可读可写
#define EXT_ADDR_POINT_1_Y							0x2002      //坐标1的Y地址，可读可写
#define EXT_ADDR_POINT_1_Z							0x2004      //坐标1的Z地址，可读可写
#define EXT_ADDR_POINT_1_O							0x2006      //坐标1的O地址，可读可写
#define EXT_ADDR_POINT_2_X							0x2040      //坐标2的X地址，可读可写
#define EXT_ADDR_POINT_2_Y							0x2042      //坐标2的Y地址，可读可写
#define EXT_ADDR_POINT_2_Z							0x2044      //坐标2的Z地址，可读可写
#define EXT_ADDR_POINT_2_O							0x2046      //坐标2的O地址，可读可写

/**-------外部485通信读取机械手状态--------------**/
#define	EXT_OLINE_NO						        0x0000	//待机，不能启动
#define	EXT_OLINE_YES						        0x0001	//待机，可以启动
#define	EXT_RUNNING							        0x0002	//运行中
#define	EXT_PAUSE								        0x0003	//暂停中
#define	EXT_ERR									        0x0004	//故障
#define	EXT_RESET								        0x0005	//复位中
#define	EXT_BACK								        0x0006	//回零中

/**-------外部485通信控制机械手状态--------------**/
#define	EXTEND_RESET			              0x01	//机械手复位
#define	EXTEND_ORIGIN			              0x02	//回零
#define	EXTEND_RUN				              0x03	//启动
#define	EXTEND_STOP				              0x04	//停止
#define	EXTEND_PAUSE			              0x05	//暂停


/*******************************************************************************
* Function Name  : ExtendRecDataDeal
* Description    : 处理扩展模块485接收到的数据，在主循环中调用该函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
extern void ExtendRecDataDeal(void);

#endif

/************************************END OF FILE**************************************/
