#include "stmflash.h"
#include "Parameter.h"
#include "ActionOperate.h"

#define STMFLASH_PROTECT_DATA_ADDR1			ADDR_FLASH_SECTOR_10			//内部FLASH的起始地址1，该地址是断电保存的数据地址
#define STMFLASH_PROTECT_DATA_ADDR2			ADDR_FLASH_SECTOR_11			//内部FLASH的起始地址2，该地址是运行保存的数据地址
#define STMFLASH_PROTECT_DATA_NUM				64												//内部FLASH的数据个数
#define STMFLASH_PROTECT_CHACK					0xA55A9DD9								//内部FLASH的校验值
#define STMFLASH_RUN_DATA_CYC						500												//内部FLASH的运行数据滚码存储,64*500*4=128K
u16 STMFLASH_RunDataCounter = 0;																	//记录开机后的备份次数
u8 STMFLASH_AllowOffPowerWrite = 0;																//允许断电写标志

//读取指定地址的半字(16位数据) 
//faddr:读地址 
//返回值:对应数据.
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  
//获取某个地址所在的flash扇区
//addr:flash地址
//返回值:0~11,即addr所在的扇区
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
//从指定地址开始写入指定长度的数据
//特别注意:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
//         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
//         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
//         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
//该函数对OTP区域也有效!可以用来写OTP区!
//OTP区域地址范围:0X1FFF7800~0X1FFF7A0F
//WriteAddr:起始地址(此地址必须为4的倍数!!)
//pBuffer:数据指针
//NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
	FLASH_Unlock();									//解锁 
  FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
	addrx=WriteAddr;				//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
	if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)break;	//发生错误了
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//写数据
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//写入数据
			{ 
				break;	//写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
}

void STMFLASH_Write_NoErase(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
	u32 endaddr = 0;
	
  if(WriteAddr < STM32_FLASH_BASE || WriteAddr % 4)
	{//非法地址
		return;
	}
	
	FLASH_Unlock();													//解锁 
  FLASH_DataCacheCmd(DISABLE);						//FLASH擦除期间,必须禁止数据缓存
 		
	endaddr = WriteAddr + NumToWrite * 4;		//写入的结束地址

	while(WriteAddr < endaddr)//写数据
	{
		if(FLASH_ProgramWord(WriteAddr,*pBuffer) != FLASH_COMPLETE)//写入数据
		{//写入异常
			break;
		}
		WriteAddr += 4;
		pBuffer++;
	}

  FLASH_DataCacheCmd(ENABLE);				//FLASH擦除结束,开启数据缓存
	FLASH_Lock();											//上锁
} 

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToRead:字(4位)数
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
	}
}

void STMFLASH_Erase(u32 WriteAddr,u32 NumToWrite)
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
	FLASH_Unlock();									//解锁 
  FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
	addrx=WriteAddr;				//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
	if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)break;	//发生错误了
			}else addrx+=4;
		}
	}
  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
}

/*掉电保存数据读取*/
u16 STMFLASH_ReadOffPowerData(void)
{
	u16 i = 0;
	u16 ret = 0;
	u32 InFlash_Data[STMFLASH_PROTECT_DATA_NUM] = {0};
	
	//读数据
	STMFLASH_Read(STMFLASH_PROTECT_DATA_ADDR1, InFlash_Data, STMFLASH_PROTECT_DATA_NUM);
	
	if(InFlash_Data[STMFLASH_PROTECT_DATA_NUM - 1] == STMFLASH_PROTECT_CHACK)
	{//断电时保护数据写入成功
		/*需要保护的数据在此读取*/
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
		{//30个
			sMD_FlashCurLayer[i * 2] = InFlash_Data[15 + i];
			sMD_FlashCurNum[i * 2] = InFlash_Data[15 + i]>>8;
			sMD_FlashCurLayer[i * 2 + 1] = InFlash_Data[15 + i]>>16;
			sMD_FlashCurNum[i * 2 + 1] = InFlash_Data[15 + i]>>24;
		}
		
		m_ProRunTimeCumulate = InFlash_Data[45];
		m_PowerOnTimeCumulate = InFlash_Data[46];
		m_PreProRunTimeCumulate = m_ProRunTimeCumulate;
		m_PrPowerOnTimeCumulate = m_PowerOnTimeCumulate;
		
		/*上次断电保存成功，备份区数据进行开机备份*/
		FLASH_EraseSector(STMFLASH_GetFlashSector(STMFLASH_PROTECT_DATA_ADDR2),VoltageRange_3);//擦除块
		STMFLASH_RunDataCounter = 0;
		STMFLASH_WriteRunData();
	}
	else
	{
		ret = 1;
	}
	
	STMFLASH_Erase(STMFLASH_PROTECT_DATA_ADDR1, STMFLASH_PROTECT_DATA_NUM);	//擦除该扇区
	
	if(ret == 0)
	{//如果读成功，那么就允许断电保存
		STMFLASH_AllowOffPowerWrite = 1;
	}
	
	return ret;
}

/*掉电保存数据*/
void STMFLASH_WriteOffPowerData(void)
{
	u16 i = 0;
	u32 InFlash_Data[STMFLASH_PROTECT_DATA_NUM]= {0};
	
	/*数据写入InFlash_Data后，再存入FLASH*/
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
	{//30个
		InFlash_Data[15 + i] = (u32)sMD_FlashCurLayer[i * 2] | (u32)sMD_FlashCurNum[i * 2]<<8
												| (u32)sMD_FlashCurLayer[i * 2 + 1]<<16 | (u32)sMD_FlashCurNum[i * 2 + 1]<<24;
	}
	
	InFlash_Data[45] = m_ProRunTimeCumulate;//累计运行时间记录
	InFlash_Data[46] = m_PowerOnTimeCumulate;//累计开机时间记录
	
	STMFLASH_Write_NoErase(STMFLASH_PROTECT_DATA_ADDR1, InFlash_Data, STMFLASH_PROTECT_DATA_NUM);
}

/*运行数据读取*/
void STMFLASH_ReadRunData(void)
{
	u16 i = 0, k = 0;
	u32 InFlash_Data[STMFLASH_PROTECT_DATA_NUM] = {0};
	
	for(k=0; k<STMFLASH_RUN_DATA_CYC; k++)
	{//读数据
		STMFLASH_Read(STMFLASH_PROTECT_DATA_ADDR2 + k * STMFLASH_PROTECT_DATA_NUM * 4, InFlash_Data, STMFLASH_PROTECT_DATA_NUM);
		
		if(InFlash_Data[STMFLASH_PROTECT_DATA_NUM - 1] == STMFLASH_PROTECT_CHACK)
		{//断电时保护数据写入成功
			/*需要保护的数据在此读取*/
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
			{//30个
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
			{//备份区数据缓存次数已达最大次数
				FLASH_EraseSector(STMFLASH_GetFlashSector(STMFLASH_PROTECT_DATA_ADDR2),VoltageRange_3);//擦除块
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
	{//如果读成功，那么就允许断电保存
		STMFLASH_AllowOffPowerWrite = 1;
	}
}

/*运行保存数据*/
void STMFLASH_WriteRunData(void)
{
	u16 i = 0;
	u32 InFlash_Data[STMFLASH_PROTECT_DATA_NUM]= {0};
	
	g_Write_FlashFlag = FALSE;
	
	/*数据写入InFlash_Data后，再存入FLASH*/
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
	{//30个
		InFlash_Data[15 + i] = (u32)sMD_FlashCurLayer[i * 2] | (u32)sMD_FlashCurNum[i * 2]<<8
												| (u32)sMD_FlashCurLayer[i * 2 + 1]<<16 | (u32)sMD_FlashCurNum[i * 2 + 1]<<24;
	}
	
	InFlash_Data[45] = m_ProRunTimeCumulate;//累计运行时间记录
	InFlash_Data[46] = m_PowerOnTimeCumulate;//累计开机时间记录

	/*将数据写入备份区*/
	STMFLASH_Write(STMFLASH_PROTECT_DATA_ADDR2 + STMFLASH_RunDataCounter * STMFLASH_PROTECT_DATA_NUM * 4, InFlash_Data, STMFLASH_PROTECT_DATA_NUM);
	STMFLASH_RunDataCounter++;
	if(STMFLASH_RunDataCounter == STMFLASH_RUN_DATA_CYC)
	{//备份区数据缓存次数已达最大次数
		FLASH_EraseSector(STMFLASH_GetFlashSector(STMFLASH_PROTECT_DATA_ADDR2),VoltageRange_3);//擦除块
		STMFLASH_RunDataCounter = 0;
		STMFLASH_WriteRunData();
	}
}

/*开机参数数据读取*/
void STMFLASH_OnPowerDataDeal(void)
{
	u16 ret = 0;
	
	ret = STMFLASH_ReadOffPowerData();			//开机读取断电保存区的数据
	if(ret == 1)
	{//上次断电保存失败，读备份区数据
		STMFLASH_ReadRunData();
	}
}






