#include "stmflash.h"
#include "Parameter.h"
#include "ActionOperate.h"

#define STMFLASH_PROTECT_DATA_ADDR1			ADDR_FLASH_SECTOR_10			//�ڲ�FLASH����ʼ��ַ1���õ�ַ�Ƕϵ籣������ݵ�ַ
#define STMFLASH_PROTECT_DATA_ADDR2			ADDR_FLASH_SECTOR_11			//�ڲ�FLASH����ʼ��ַ2���õ�ַ�����б�������ݵ�ַ
#define STMFLASH_PROTECT_DATA_NUM				64												//�ڲ�FLASH�����ݸ���
#define STMFLASH_PROTECT_CHACK					0xA55A9DD9								//�ڲ�FLASH��У��ֵ
#define STMFLASH_RUN_DATA_CYC						500												//�ڲ�FLASH���������ݹ���洢,64*500*4=128K
u16 STMFLASH_RunDataCounter = 0;																	//��¼������ı��ݴ���
u8 STMFLASH_AllowOffPowerWrite = 0;																//����ϵ�д��־

//��ȡָ����ַ�İ���(16λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  
//��ȡĳ����ַ���ڵ�flash����
//addr:flash��ַ
//����ֵ:0~11,��addr���ڵ�����
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ر�ע��:��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
//         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
//         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
//         û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
//�ú�����OTP����Ҳ��Ч!��������дOTP��!
//OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)
//pBuffer:����ָ��
//NumToWrite:��(32λ)��(����Ҫд���32λ���ݵĸ���.) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//�Ƿ���ַ
	FLASH_Unlock();									//���� 
  FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
 		
	addrx=WriteAddr;				//д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*4;	//д��Ľ�����ַ
	if(addrx<0X1FFF0000)			//ֻ�����洢��,����Ҫִ�в�������!!
	{
		while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V֮��!!
				if(status!=FLASH_COMPLETE)break;	//����������
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//д����
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//д������
			{ 
				break;	//д���쳣
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH_Lock();//����
}

void STMFLASH_Write_NoErase(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
	u32 endaddr = 0;
	
  if(WriteAddr < STM32_FLASH_BASE || WriteAddr % 4)
	{//�Ƿ���ַ
		return;
	}
	
	FLASH_Unlock();													//���� 
  FLASH_DataCacheCmd(DISABLE);						//FLASH�����ڼ�,�����ֹ���ݻ���
 		
	endaddr = WriteAddr + NumToWrite * 4;		//д��Ľ�����ַ

	while(WriteAddr < endaddr)//д����
	{
		if(FLASH_ProgramWord(WriteAddr,*pBuffer) != FLASH_COMPLETE)//д������
		{//д���쳣
			break;
		}
		WriteAddr += 4;
		pBuffer++;
	}

  FLASH_DataCacheCmd(ENABLE);				//FLASH��������,�������ݻ���
	FLASH_Lock();											//����
} 

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:��(4λ)��
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=4;//ƫ��4���ֽ�.	
	}
}

void STMFLASH_Erase(u32 WriteAddr,u32 NumToWrite)
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//�Ƿ���ַ
	FLASH_Unlock();									//���� 
  FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
 		
	addrx=WriteAddr;				//д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*4;	//д��Ľ�����ַ
	if(addrx<0X1FFF0000)			//ֻ�����洢��,����Ҫִ�в�������!!
	{
		while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V֮��!!
				if(status!=FLASH_COMPLETE)break;	//����������
			}else addrx+=4;
		}
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH_Lock();//����
}

/*���籣�����ݶ�ȡ*/
u16 STMFLASH_ReadOffPowerData(void)
{
	u16 i = 0;
	u16 ret = 0;
	u32 InFlash_Data[STMFLASH_PROTECT_DATA_NUM] = {0};
	
	//������
	STMFLASH_Read(STMFLASH_PROTECT_DATA_ADDR1, InFlash_Data, STMFLASH_PROTECT_DATA_NUM);
	
	if(InFlash_Data[STMFLASH_PROTECT_DATA_NUM - 1] == STMFLASH_PROTECT_CHACK)
	{//�ϵ�ʱ��������д��ɹ�
		/*��Ҫ�����������ڴ˶�ȡ*/
		SC_Parameter.RW_Num = InFlash_Data[0];
		SC_Parameter.SC_Num = InFlash_Data[1];
		SC_Parameter.CJ_Num = InFlash_Data[2];
		SC_Parameter.JG_Num = InFlash_Data[3];
		SC_Parameter.LJ_Num = InFlash_Data[4];
		SC_Parameter.NG_Num = InFlash_Data[5];

		for(i=0; i<8; i++)
		{
			USER_Parameter.CURR_Num[i] = InFlash_Data[6 + i];
		}
		
		sMD_RunPara.curGood = InFlash_Data[14];
		sMD_RunPara.curLayer = InFlash_Data[14]>>8;
		sMD_RunPara.curNum = InFlash_Data[14]>>16;
		
		for(i=0; i<MD_GOOD_NUM / 2; i++)
		{//30��
			sMD_FlashCurLayer[i * 2] = InFlash_Data[15 + i];
			sMD_FlashCurNum[i * 2] = InFlash_Data[15 + i]>>8;
			sMD_FlashCurLayer[i * 2 + 1] = InFlash_Data[15 + i]>>16;
			sMD_FlashCurNum[i * 2 + 1] = InFlash_Data[15 + i]>>24;
		}
		
		m_ProRunTimeCumulate = InFlash_Data[45];
		m_PowerOnTimeCumulate = InFlash_Data[46];
		m_PreProRunTimeCumulate = m_ProRunTimeCumulate;
		m_PrPowerOnTimeCumulate = m_PowerOnTimeCumulate;
		
		/*�ϴζϵ籣��ɹ������������ݽ��п�������*/
		FLASH_EraseSector(STMFLASH_GetFlashSector(STMFLASH_PROTECT_DATA_ADDR2),VoltageRange_3);//������
		STMFLASH_RunDataCounter = 0;
		STMFLASH_WriteRunData();
	}
	else
	{
		ret = 1;
	}
	
	STMFLASH_Erase(STMFLASH_PROTECT_DATA_ADDR1, STMFLASH_PROTECT_DATA_NUM);	//����������
	
	if(ret == 0)
	{//������ɹ�����ô������ϵ籣��
		STMFLASH_AllowOffPowerWrite = 1;
	}
	
	return ret;
}

/*���籣������*/
void STMFLASH_WriteOffPowerData(void)
{
	u16 i = 0;
	u32 InFlash_Data[STMFLASH_PROTECT_DATA_NUM]= {0};
	
	/*����д��InFlash_Data���ٴ���FLASH*/
	InFlash_Data[STMFLASH_PROTECT_DATA_NUM - 1] = STMFLASH_PROTECT_CHACK;
	
	InFlash_Data[0] = SC_Parameter.RW_Num;
	InFlash_Data[1] = SC_Parameter.SC_Num;
	InFlash_Data[2] = SC_Parameter.CJ_Num;
	InFlash_Data[3] = SC_Parameter.JG_Num;
	InFlash_Data[4] = SC_Parameter.LJ_Num;
	InFlash_Data[5] = SC_Parameter.NG_Num;
	
	for(i=0; i<8; i++)
	{
		InFlash_Data[6 + i] = USER_Parameter.CURR_Num[i];
	}
		
	InFlash_Data[14] = (u32)sMD_RunPara.curGood | (u32)sMD_RunPara.curLayer<<8
													| (u32)sMD_RunPara.curNum<<16;
	
	for(i=0; i<MD_GOOD_NUM / 2; i++)
	{//30��
		InFlash_Data[15 + i] = (u32)sMD_FlashCurLayer[i * 2] | (u32)sMD_FlashCurNum[i * 2]<<8
												| (u32)sMD_FlashCurLayer[i * 2 + 1]<<16 | (u32)sMD_FlashCurNum[i * 2 + 1]<<24;
	}
	
	InFlash_Data[45] = m_ProRunTimeCumulate;//�ۼ�����ʱ���¼
	InFlash_Data[46] = m_PowerOnTimeCumulate;//�ۼƿ���ʱ���¼
	
	STMFLASH_Write_NoErase(STMFLASH_PROTECT_DATA_ADDR1, InFlash_Data, STMFLASH_PROTECT_DATA_NUM);
}

/*�������ݶ�ȡ*/
void STMFLASH_ReadRunData(void)
{
	u16 i = 0, k = 0;
	u32 InFlash_Data[STMFLASH_PROTECT_DATA_NUM] = {0};
	
	for(k=0; k<STMFLASH_RUN_DATA_CYC; k++)
	{//������
		STMFLASH_Read(STMFLASH_PROTECT_DATA_ADDR2 + k * STMFLASH_PROTECT_DATA_NUM * 4, InFlash_Data, STMFLASH_PROTECT_DATA_NUM);
		
		if(InFlash_Data[STMFLASH_PROTECT_DATA_NUM - 1] == STMFLASH_PROTECT_CHACK)
		{//�ϵ�ʱ��������д��ɹ�
			/*��Ҫ�����������ڴ˶�ȡ*/
			SC_Parameter.RW_Num = InFlash_Data[0];
			SC_Parameter.SC_Num = InFlash_Data[1];
			SC_Parameter.CJ_Num = InFlash_Data[2];
			SC_Parameter.JG_Num = InFlash_Data[3];
			SC_Parameter.LJ_Num = InFlash_Data[4];
			SC_Parameter.NG_Num = InFlash_Data[5];

			for(i=0; i<8; i++)
			{
				USER_Parameter.CURR_Num[i] = InFlash_Data[6 + i];
			}
			
			sMD_RunPara.curGood = InFlash_Data[14];
			sMD_RunPara.curLayer = InFlash_Data[14]>>8;
			sMD_RunPara.curNum = InFlash_Data[14]>>16;
			
			for(i=0; i<MD_GOOD_NUM / 2; i++)
			{//30��
				sMD_FlashCurLayer[i * 2] = InFlash_Data[15 + i];
				sMD_FlashCurNum[i * 2] = InFlash_Data[15 + i]>>8;
				sMD_FlashCurLayer[i * 2 + 1] = InFlash_Data[15 + i]>>16;
				sMD_FlashCurNum[i * 2 + 1] = InFlash_Data[15 + i]>>24;
			}
			
			m_ProRunTimeCumulate = InFlash_Data[45];
			m_PowerOnTimeCumulate = InFlash_Data[46];
			m_PreProRunTimeCumulate = m_ProRunTimeCumulate;
			m_PrPowerOnTimeCumulate = m_PowerOnTimeCumulate;
			
			STMFLASH_RunDataCounter = k + 1;
			if(STMFLASH_RunDataCounter == STMFLASH_RUN_DATA_CYC)
			{//���������ݻ�������Ѵ�������
				FLASH_EraseSector(STMFLASH_GetFlashSector(STMFLASH_PROTECT_DATA_ADDR2),VoltageRange_3);//������
				STMFLASH_RunDataCounter = 0;
				STMFLASH_WriteRunData();
			}
		}
		else
		{
			break;
		}
	}
	
	if(k > 0)
	{//������ɹ�����ô������ϵ籣��
		STMFLASH_AllowOffPowerWrite = 1;
	}
}

/*���б�������*/
void STMFLASH_WriteRunData(void)
{
	u16 i = 0;
	u32 InFlash_Data[STMFLASH_PROTECT_DATA_NUM]= {0};
	
	g_Write_FlashFlag = FALSE;
	
	/*����д��InFlash_Data���ٴ���FLASH*/
	InFlash_Data[STMFLASH_PROTECT_DATA_NUM - 1] = STMFLASH_PROTECT_CHACK;
	
	InFlash_Data[0] = SC_Parameter.RW_Num;
	InFlash_Data[1] = SC_Parameter.SC_Num;
	InFlash_Data[2] = SC_Parameter.CJ_Num;
	InFlash_Data[3] = SC_Parameter.JG_Num;
	InFlash_Data[4] = SC_Parameter.LJ_Num;
	InFlash_Data[5] = SC_Parameter.NG_Num;
	
	for(i=0; i<8; i++)
	{
		InFlash_Data[6 + i] = USER_Parameter.CURR_Num[i];
	}
	
	InFlash_Data[14] = (u32)sMD_RunPara.curGood | (u32)sMD_RunPara.curLayer<<8
												| (u32)sMD_RunPara.curNum<<16;
	
	for(i=0; i<MD_GOOD_NUM / 2; i++)
	{//30��
		InFlash_Data[15 + i] = (u32)sMD_FlashCurLayer[i * 2] | (u32)sMD_FlashCurNum[i * 2]<<8
												| (u32)sMD_FlashCurLayer[i * 2 + 1]<<16 | (u32)sMD_FlashCurNum[i * 2 + 1]<<24;
	}
	
	InFlash_Data[45] = m_ProRunTimeCumulate;//�ۼ�����ʱ���¼
	InFlash_Data[46] = m_PowerOnTimeCumulate;//�ۼƿ���ʱ���¼

	/*������д�뱸����*/
	STMFLASH_Write(STMFLASH_PROTECT_DATA_ADDR2 + STMFLASH_RunDataCounter * STMFLASH_PROTECT_DATA_NUM * 4, InFlash_Data, STMFLASH_PROTECT_DATA_NUM);
	STMFLASH_RunDataCounter++;
	if(STMFLASH_RunDataCounter == STMFLASH_RUN_DATA_CYC)
	{//���������ݻ�������Ѵ�������
		FLASH_EraseSector(STMFLASH_GetFlashSector(STMFLASH_PROTECT_DATA_ADDR2),VoltageRange_3);//������
		STMFLASH_RunDataCounter = 0;
		STMFLASH_WriteRunData();
	}
}

/*�����������ݶ�ȡ*/
void STMFLASH_OnPowerDataDeal(void)
{
	u16 ret = 0;
	
	ret = STMFLASH_ReadOffPowerData();			//������ȡ�ϵ籣����������
	if(ret == 1)
	{//�ϴζϵ籣��ʧ�ܣ�������������
		STMFLASH_ReadRunData();
	}
}






