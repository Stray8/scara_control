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
	
	SysTick_Init();					//��ʼ��ϵͳ�δ�ʱ��
	TIM3_Config();					//1ms��ʱ��������ʵ��EtherCAT��������֡���ͽ���
	
	InputInit(); 
	OutputInit();
	TimeInit();
	UsartInit();
	PVD_Init();
	W25QXX_Init();					//W25QXX��ʼ��
	W25QXX_Check();					//��һ���Զ����FLASH
	delay_ms(100);
	STMFLASH_OnPowerDataDeal();		//��ȡ���籣�ֵ�����
	ReadIICSysParameter();

	/*EtherCAT��س�ʼ��*/
	delay_ms(1000);
	delay_ms(1000);
	ETH_BSP_Config();
    ethernetif_init();
	osal_usleep(5000000);
    EtherCATInit("eth0");
	delay_ms(100);									//EtherCAT��ʼ���ɹ������������ʱ�������ŷ�Ӧ���������


    //////////////////////////////////////////
    //λ��ģʽ
	// CSP_Mode_Init();
    //����ģʽ
	CST_Mode_Init();					//����Ϊת��ģʽ
    //////////////////////////////////////////
    

	Servo_InitFinish = 1;						//EtherCAT��ʼ����ɱ�־
	
	delay_ms(100);
	Robot_Reset();									//��λ��ز���
	Communication_Time = 1;					//ͨ�ż�ʱ����
	
	DIROUT485_L;                      //��ʼ��485Ϊ����״̬
  
	delay_ms(1000);									//ȷ���ŷ�����ͬ��
	/***********************************/
    /**         ��������״̬          **/
    /***********************************/
	while(1) {
        if(Internet_Parameter.Switch == 1){//����ʹ������������
			ExtendRecDataDeal();		    //485���������ݴ���
		}
		JDZ_OrderDecoding();		//485���ݴ���
		BackToOrigin();	 			//��ԭ�㴦��
		ErrorOperate();	 			//�������
		WatchFunction(); 			//���ֿ�������ͨ�ż���ز�����״̬�ļ���
		StatusControl(); 			//�����źż���״̬����
		ActionControl(); 			//�������ƣ��ֶ�/�Զ�ģʽ�µĶ�������
		
		OneAxisSpeedInterpControl();
		SpeedInterpControl();	//�岹����
		AxisInterpParSave();	//�岹�˶��еĲ����仯ʱ��ͳһ������������ﱣ��
        //////////////////////////////////////////
        //���ز���
		TorqueProcess_T();
        //////////////////////////////////////////
	}
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
