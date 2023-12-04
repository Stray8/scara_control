/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : main.c
***************************************************************************************/
#include "stm32f4xx.h"
#include "MotorInit.h"
#include "Delay.h"
#include "w25qxx.h" 
#include "Usart.h"
#include "Parameter.h"
#include "SpeedControl.h"
#include "SignalWatch.h"
#include "Error.h" 
#include "BackToOrigin.h"
#include "in.h"
#include "out.h"
#include "JDZ.h" 
#include "SpeedControl.h"
#include "can.h"
#include "CANopen.h"
#include "ActionOperate.h"
#include "SpeedControl.h"
#include "platform_init.h"
#include "EtherCAT_App.h"
#include "bsp_SysTick.h"
#include "netconf.h"
#include "LAN8742A.h"
#include "osal.h"
#include "stmflash.h"
#include "ExtendCom.h"
#include "DragTeach.h"

int main(void){
#ifdef	BootloaderMode
	SCB->VTOR = FLASH_BASE | 0X10000;
	__enable_irq();
#endif
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	SysTick_Init();					//初始化系统滴答定时器
	TIM3_Config();					//1ms定时器，用于实现EtherCAT总线周期帧发送接收
	
	InputInit(); 
	OutputInit();
	TimeInit();
	UsartInit();
	PVD_Init();
	W25QXX_Init();					//W25QXX初始化
	W25QXX_Check();					//第一次自动清除FLASH
	delay_ms(100);
	STMFLASH_OnPowerDataDeal();		//读取掉电保持的数据
	ReadIICSysParameter();

	/*EtherCAT相关初始化*/
	delay_ms(1000);
	delay_ms(1000);
	ETH_BSP_Config();
    ethernetif_init();
	osal_usleep(5000000);
    EtherCATInit("eth0");
	delay_ms(100);									//EtherCAT初始化成功后必须加这个延时，否则伺服应答会有问题


    //////////////////////////////////////////
    //位置模式
	// CSP_Mode_Init();
    //力矩模式
	CST_Mode_Init();					//配置为转矩模式
    //////////////////////////////////////////
    

	Servo_InitFinish = 1;						//EtherCAT初始化完成标志
	
	delay_ms(100);
	Robot_Reset();									//复位相关参数
	Communication_Time = 1;					//通信计时开启
	
	DIROUT485_L;                      //初始化485为接收状态
  
	delay_ms(1000);									//确保伺服参数同步
	/***********************************/
    /**         正常运行状态          **/
    /***********************************/
	while(1) {
        if(Internet_Parameter.Switch == 1){//优先使用物联网功能
			ExtendRecDataDeal();		    //485物联网数据处理
		}
		JDZ_OrderDecoding();		//485数据处理
		BackToOrigin();	 			//回原点处理
		ErrorOperate();	 			//报警检测
		WatchFunction(); 			//与手控器进行通信及相关参数，状态的监视
		StatusControl(); 			//输入信号检测和状态控制
		ActionControl(); 			//动作控制，手动/自动模式下的动作控制
		
		OneAxisSpeedInterpControl();
		SpeedInterpControl();	//插补计算
		AxisInterpParSave();	//插补运动中的参数变化时，统一放在这个函数里保存
        //////////////////////////////////////////
        //力矩补偿
		TorqueProcess_T();
        //////////////////////////////////////////
	}
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
