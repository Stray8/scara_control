/*************** (C) COPYRIGHT 2019 Kingrobot manipulator Team ****************
* File Name          : w25qxx.c
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 06/08/2019
* Description        : This file is complete the SPI settings.
******************************************************************************/

//#include "stm32f10x_lib.h"		
#include "w25qxx.h"
#include "Delay.h"
#include "out.h"
#include "SignalWatch.h"
#include "Parameter.h"

u16 W25QXX_TYPE = W25Q128;	//Ĭ����W25Q128

//4KbytesΪһ��Sector
//16������Ϊ1��Block
//W25Q128
//����Ϊ16M�ֽ�,����128��Block,4096��Sector 
													 
//��ʼ��SPI FLASH��IO��
void W25QXX_Init(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_SPI3);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 							//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 								//����
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	W25QXX_CS=1;			//SPI FLASH��ѡ��
	SPI3_Init();		   	//��ʼ��SPI
}   

//��ȡW25QXX��״̬�Ĵ���
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����)
//Ĭ��:0x00
u8 W25QXX_ReadSR(void)   
{  
	u8 byte=0;   
	W25QXX_CS=0;                            //ʹ������   
	SPI3_ReadWriteByte(W25X_ReadStatusReg); //���Ͷ�ȡ״̬�Ĵ�������
	byte=SPI3_ReadWriteByte(0Xff);          //��ȡһ���ֽ�  
	W25QXX_CS=1;                            //ȡ��Ƭѡ     
	return byte;   
} 
//дW25QXX״̬�Ĵ���
//ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д!!!
void W25QXX_Write_SR(u8 sr)   
{   
	W25QXX_CS=0;                            //ʹ������   
	SPI3_ReadWriteByte(W25X_WriteStatusReg);//����дȡ״̬�Ĵ�������    
	SPI3_ReadWriteByte(sr);               	//д��һ���ֽ�  
	W25QXX_CS=1;                            //ȡ��Ƭѡ     	      
}   
//W25QXXдʹ��	
//��WEL��λ   
void W25QXX_Write_Enable(void)   
{
	W25QXX_CS=0;                          	//ʹ������   
  SPI3_ReadWriteByte(W25X_WriteEnable); 	//����дʹ��  
	W25QXX_CS=1;                           	//ȡ��Ƭѡ     	      
} 
//W25QXXд��ֹ	
//��WEL����  
void W25QXX_Write_Disable(void)   
{  
	W25QXX_CS=0;                            //ʹ������   
  SPI3_ReadWriteByte(W25X_WriteDisable);  //����д��ָֹ��    
	W25QXX_CS=1;                            //ȡ��Ƭѡ     	      
} 		
//��ȡоƬID
//����ֵ����:				   
//0XEF13,��ʾоƬ�ͺ�ΪW25Q80  
//0XEF14,��ʾоƬ�ͺ�ΪW25Q16    
//0XEF15,��ʾоƬ�ͺ�ΪW25Q32  
//0XEF16,��ʾоƬ�ͺ�ΪW25Q64 
//0XEF17,��ʾоƬ�ͺ�ΪW25Q128 	  
u16 W25QXX_ReadID(void)
{
	u16 Temp = 0;	  
	W25QXX_CS=0;				    
	SPI3_ReadWriteByte(0x90);//���Ͷ�ȡID����	    
	SPI3_ReadWriteByte(0x00); 	    
	SPI3_ReadWriteByte(0x00); 	    
	SPI3_ReadWriteByte(0x00); 	 			   
	Temp|=SPI3_ReadWriteByte(0xFF)<<8;  
	Temp|=SPI3_ReadWriteByte(0xFF);	 
	W25QXX_CS=1;				    
	return Temp;
}   		    

//SPI��һҳ(0~65535)��д������256���ֽڵ�����
//��ָ����ַ��ʼд�����256�ֽڵ�����
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!	 
void W25QXX_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
 	u16 i;  
	W25QXX_Write_Enable();                  	//SET WEL 
	W25QXX_CS=0;                            	//ʹ������   
	SPI3_ReadWriteByte(W25X_PageProgram);      	//����дҳ����   
	SPI3_ReadWriteByte((u8)((WriteAddr)>>16)); 	//����24bit��ַ    
	SPI3_ReadWriteByte((u8)((WriteAddr)>>8));   
	SPI3_ReadWriteByte((u8)WriteAddr);   
	for(i=0;i<NumByteToWrite;i++)SPI3_ReadWriteByte(pBuffer[i]);//ѭ��д��  
	W25QXX_CS=1;                            	//ȡ��Ƭѡ 
	W25QXX_Wait_Busy();					   		//�ȴ�д�����
} 
//�޼���дSPI FLASH 
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ���� 
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void W25QXX_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)   
{ 			 		 
	u16 pageremain;	   
	pageremain=256-WriteAddr%256; //��ҳʣ����ֽ���		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//������256���ֽ�
	while(1)
	{	   
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//д�������
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //��ȥ�Ѿ�д���˵��ֽ���
			if(NumByteToWrite>256)pageremain=256; //һ�ο���д��256���ֽ�
			else pageremain=NumByteToWrite; 	  //����256���ֽ���
		}
	};	    
} 
//дSPI FLASH  
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)						
//NumByteToWrite:Ҫд����ֽ���(���65535)   
u8 W25QXX_BUFFER[4096];
void W25QXX_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{ 
	u32 secpos = 0;
	u16 secoff = 0;
	u16 secremain = 0;
 	u16 i = 0;    
	u8 *W25QXX_BUF = W25QXX_BUFFER;
	
 	secpos = WriteAddr / 4096;//������ַ  
	secoff = WriteAddr % 4096;//�������ڵ�ƫ��
	secremain = 4096 - secoff;//����ʣ��ռ��С

 	if(NumByteToWrite <= secremain)
	{
		secremain = NumByteToWrite;//������4096���ֽ�
	}
	
	while(1) 
	{	
		W25QXX_CS=0;                            	//ʹ������   
		SPI3_ReadWriteByte(W25X_ReadData);         	//���Ͷ�ȡ����   
		SPI3_ReadWriteByte((u8)((secpos*4096)>>16));  	//����24bit��ַ    
		SPI3_ReadWriteByte((u8)((secpos*4096)>>8));   
		SPI3_ReadWriteByte((u8)(secpos*4096));   
		for(i=0;i<4096;i++)
		{ 
				W25QXX_BUF[i]=SPI3_ReadWriteByte(0XFF);   	//ѭ������  
		}
		W25QXX_CS=1;
		
		W25QXX_Erase_Sector(secpos);		//�����������
		for(i=0; i<secremain; i++)	   	//����
		{
			W25QXX_BUF[i + secoff] = pBuffer[i];
		}
		
		if(secpos*4096 == SAVE_PROTECT_START_ADDRESS)
		{//��������д��У���ֽ���ֵ
			W25QXX_BUF[4096 - 1] = SAVE_PROTECT_VALUE;
		}
		
		W25QXX_Write_NoCheck(W25QXX_BUF, secpos*4096, 4096);//д����������
		
		if(secpos*4096 == SAVE_PROTECT_START_ADDRESS)
		{
			W25QXX_Erase_Sector(SAVE_PROTECT_BACKUP_ADDRESS/4096);										//������������ı�������
			W25QXX_Write_NoCheck(W25QXX_BUF, SAVE_PROTECT_BACKUP_ADDRESS, 4096);			//д�뱣������ı�������
		}
		
		if(NumByteToWrite == secremain)
		{
			break;//д�������
		}
		else//д��δ����
		{
			secpos++;//������ַ��1
			secoff = 0;//ƫ��λ��Ϊ0 	 

		  pBuffer += secremain;  				//ָ��ƫ��
			WriteAddr += secremain;				//д��ַƫ��	   
		  NumByteToWrite -= secremain;			//�ֽ����ݼ�
			if(NumByteToWrite > 4096)
				secremain = 4096;//��һ����������д����
			else 
				secremain = NumByteToWrite;		//��һ����������д����
		}	 
	}
}

//��ȡSPI FLASH  
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
void W25QXX_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)   
{ 
 	u16 i = 0;
	u8 protectValue = 0;
	
	W25QXX_CS=0;                            	//ʹ������   
	SPI3_ReadWriteByte(W25X_ReadData);         	//���Ͷ�ȡ����   
	SPI3_ReadWriteByte((u8)((ReadAddr)>>16));  	//����24bit��ַ    
	SPI3_ReadWriteByte((u8)((ReadAddr)>>8));   
	SPI3_ReadWriteByte((u8)ReadAddr);   
	for(i=0;i<NumByteToRead;i++)
	{ 
			pBuffer[i]=SPI3_ReadWriteByte(0XFF);   	//ѭ������  
	}
	W25QXX_CS=1;
#if 1	
	if(ReadAddr >= SAVE_PROTECT_START_ADDRESS && ReadAddr <= SAVE_PROTECT_END_ADDRESS)
	{//���ݺͻָ�����������������ݺͱ�����������
		
		/*��ȡ���������У���֣�����У�鼰�ָ�*/
		W25QXX_CS = 0;                            									//ʹ������   
		SPI3_ReadWriteByte(W25X_ReadData);         									//���Ͷ�ȡ����   
		SPI3_ReadWriteByte((u8)((SAVE_PROTECT_END_ADDRESS)>>16));  	//����24bit��ַ    
		SPI3_ReadWriteByte((u8)((SAVE_PROTECT_END_ADDRESS)>>8));   
		SPI3_ReadWriteByte((u8)SAVE_PROTECT_END_ADDRESS);   
		protectValue = SPI3_ReadWriteByte(0XFF);											//��ȡ�����ֽ�
		W25QXX_CS = 1;
		
		if(protectValue != SAVE_PROTECT_VALUE)
		{//�ָ�������������
			W25QXX_CS = 0;                            												//ʹ������   
			SPI3_ReadWriteByte(W25X_ReadData);         												//���Ͷ�ȡ����   
			SPI3_ReadWriteByte((u8)((SAVE_PROTECT_BACKUP_ADDRESS)>>16));  		//����24bit��ַ    
			SPI3_ReadWriteByte((u8)((SAVE_PROTECT_BACKUP_ADDRESS)>>8));   
			SPI3_ReadWriteByte((u8)SAVE_PROTECT_BACKUP_ADDRESS);   
			for(i=0; i<SAVE_PROTECT_SIZE; i++)
			{ 
					W25QXX_BUFFER[i] = SPI3_ReadWriteByte(0XFF);   								//ѭ������  
			}
			W25QXX_CS = 1;
			
			if(W25QXX_BUFFER[4096 - 1] == SAVE_PROTECT_VALUE)
			{
				W25QXX_Erase_Sector(SAVE_PROTECT_START_ADDRESS/4096);										//����������������
				W25QXX_Write_NoCheck(W25QXX_BUFFER, SAVE_PROTECT_START_ADDRESS, 4096);	//д�뱣����������
			}
		}
		else
		{//�ָ���������ı�����������
			W25QXX_CS = 0;                            									//ʹ������   
			SPI3_ReadWriteByte(W25X_ReadData);         									//���Ͷ�ȡ����   
			SPI3_ReadWriteByte((u8)((SAVE_PROTECT_BACKUP_END_ADDRESS)>>16));  	//����24bit��ַ    
			SPI3_ReadWriteByte((u8)((SAVE_PROTECT_BACKUP_END_ADDRESS)>>8));   
			SPI3_ReadWriteByte((u8)SAVE_PROTECT_BACKUP_END_ADDRESS);   
			protectValue = SPI3_ReadWriteByte(0XFF);											//��ȡ�����ֽ�
			W25QXX_CS = 1;
			
			if(protectValue != SAVE_PROTECT_VALUE)
			{
				W25QXX_CS = 0;                            												//ʹ������   
				SPI3_ReadWriteByte(W25X_ReadData);         												//���Ͷ�ȡ����   
				SPI3_ReadWriteByte((u8)((SAVE_PROTECT_START_ADDRESS)>>16));  		//����24bit��ַ    
				SPI3_ReadWriteByte((u8)((SAVE_PROTECT_START_ADDRESS)>>8));   
				SPI3_ReadWriteByte((u8)SAVE_PROTECT_START_ADDRESS);   
				for(i=0; i<SAVE_PROTECT_SIZE; i++)
				{ 
						W25QXX_BUFFER[i] = SPI3_ReadWriteByte(0XFF);   								//ѭ������  
				}
				W25QXX_CS = 1;
				
				if(W25QXX_BUFFER[4096 - 1] == SAVE_PROTECT_VALUE)
				{
					W25QXX_Erase_Sector(SAVE_PROTECT_BACKUP_ADDRESS/4096);										//����������������
					W25QXX_Write_NoCheck(W25QXX_BUFFER, SAVE_PROTECT_BACKUP_ADDRESS, 4096);		//д�뱣����������
				}
			}
		}
	}
#endif
}

//��������оƬ		  
//�ȴ�ʱ�䳬��...
void W25QXX_Erase_Chip(void)   
{                                   
	W25QXX_Write_Enable();                 	 	//SET WEL 
	W25QXX_Wait_Busy();   
	W25QXX_CS = 0;                           	//ʹ������   
	SPI3_ReadWriteByte(W25X_ChipErase);        	//����Ƭ��������  
	W25QXX_CS = 1;                            	//ȡ��Ƭѡ     	      
	W25QXX_Wait_Busy();   				   		//�ȴ�оƬ��������
}   
//����һ������
//Dst_Addr:������ַ ����ʵ����������
//����һ��ɽ��������ʱ��:150ms
void W25QXX_Erase_Sector(u32 Dst_Addr)   
{  
	//����falsh�������,������   
// 	printf("fe:%x\r\n",Dst_Addr);	  			//��ɾ�����ų���
	Dst_Addr *= 4096;
	W25QXX_Write_Enable();                  	//SET WEL 	 
	W25QXX_Wait_Busy();   
	W25QXX_CS = 0;                            	//ʹ������   
	SPI3_ReadWriteByte(W25X_SectorErase);      	//������������ָ�� 
	SPI3_ReadWriteByte((u8)((Dst_Addr)>>16));  	//����24bit��ַ    
	SPI3_ReadWriteByte((u8)((Dst_Addr)>>8));   
	SPI3_ReadWriteByte((u8)Dst_Addr);  
	W25QXX_CS = 1;                            	//ȡ��Ƭѡ     	      
	W25QXX_Wait_Busy();   				   		//�ȴ��������
}

//�ȴ�����
void W25QXX_Wait_Busy(void)
{   
	while((W25QXX_ReadSR()&0x01)==0x01);  		// �ȴ�BUSYλ���
}

//�������ģʽ
void W25QXX_PowerDown(void)   
{ 
	W25QXX_CS = 0;                           	 	//ʹ������   
	SPI3_ReadWriteByte(W25X_PowerDown);        //���͵�������  
	W25QXX_CS = 1;                            	//ȡ��Ƭѡ     	      
	delay_us(3);	//�ȴ�TPD  
}

//����
void W25QXX_WAKEUP(void)   
{  
	W25QXX_CS = 0;                            	//ʹ������   
	SPI3_ReadWriteByte(W25X_ReleasePowerDown);	//  send W25X_PowerDown command 0xAB    
	W25QXX_CS = 1;                            	//ȡ��Ƭѡ     	      
	delay_us(3);                           	//�ȴ�TRES1
}   

/**************************************************************************************************
**  ��������  W25QXX_Check()
**	�����������
**	�������������� 1�����ʧ�ܣ�0�����ɹ�
**	�������ܣ����оƬ�Ƿ�����
**	��ע��	  ��оƬ���һ����ַ�����־�֣�����У��
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
u8 W25QXX_Check(void)
{
	u8 temp = 0;
	u8 tempone = SAVE_PROTECT_VALUE;
	u32 i = 0;
	
	//����ÿ�ο�����дW25QXX
	W25QXX_Read(&temp, SAVE_PROTECT_END_ADDRESS, 1);
	if(temp == SAVE_PROTECT_VALUE)
	{
		return 0;
	}
	else
	{//�ų���һ�γ�ʼ�������
		//�ڶ���ȷ�ϣ���һ�ζ�ʱ����б�������Ĳ����ָ�
		W25QXX_Read(&temp, SAVE_PROTECT_END_ADDRESS, 1);
		if(temp == SAVE_PROTECT_VALUE)
		{
			return 0;
		}
		else
		{
			W25QXX_Erase_Chip();
			W25QXX_Write(&tempone, SAVE_PROTECT_END_ADDRESS, 1);
			W25QXX_Read(&temp, SAVE_PROTECT_END_ADDRESS, 1);
			if(temp == SAVE_PROTECT_VALUE)
			{
				for(i=0; i<OUTPUT_NUM - 4; i++)
				{
					SetOutput(i);
				}
				return 0;
			}
		}
	}
	return 1;											  
}

/**************************************************************************************************
**  ��������  W25QXX_Writ0_4K()
**	�����������
**	���������
**	�������ܣ���ַWriteAddr��ʼд4K�ֽ�����0
**	��ע��	  
**  ���ߣ�    
**  �������ڣ�
***************************************************************************************************/
void W25QXX_Writ0_4K(u32 WriteAddr)
{	
	u16 i = 0;
	u32 secpos = 0;
	
	for(i=0; i<4096; i++)
	{ 
			W25QXX_BUFFER[i] = 0;
	}
	
 	secpos = WriteAddr / 4096;//������ַ
	
	W25QXX_Erase_Sector(secpos);		//�����������
	
	if(secpos*4096 == SAVE_PROTECT_START_ADDRESS)
	{//��������д��У���ֽ���ֵ
		W25QXX_BUFFER[4096 - 1] = SAVE_PROTECT_VALUE;
	}
	
	W25QXX_Write_NoCheck(W25QXX_BUFFER, secpos*4096, 4096);//д����������
	
	if(secpos*4096 == SAVE_PROTECT_START_ADDRESS)
	{
		W25QXX_Erase_Sector(SAVE_PROTECT_BACKUP_ADDRESS/4096);										//������������ı�������
		W25QXX_Write_NoCheck(W25QXX_BUFFER, SAVE_PROTECT_BACKUP_ADDRESS, 4096);			//д�뱣������ı�������
	}
}

