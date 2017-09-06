#include "Testing.h"

//void NRF_Test(uint8_t cmd)
//{
//	uint8_t state;
//	switch (cmd)
//	{
//		case  1:
//		printf("%d\r\n",HAL_GPIO_ReadPin(NRF_INT_GPIO_Port,NRF_INT_Pin));
//		break;
//		
//		case  2:
//		__HAL_GPIO_EXTI_GENERATE_SWIT(NRF_INT_Pin) ;
//		break;
//		case	3:			
//			printf("0x%x\r\n",NRF24L01_Read_Reg(STATUS) );break;
//		
//		case  4:NRF24L01_TX_Mode();	break;
//		case 	5:NRF24L01_RX_Mode();	break;
//		case  6:
//				state=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
//				NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,state); //���TX_DS��MAX_RT�жϱ�־
//				NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
//			break;
//		
//		
//		default:break;
//		}			
//		
//				
//				
//}
void unlockflash(unsigned int passwd)
{
	
	#ifdef FLASHPROTECT
	static uint8 i=5;
		#ifndef MAXRDPLEVEL
		if(i>0)
		{
			if(passwd==26172617)
			{
				HAL_FLASH_Unlock();
				HAL_FLASH_OB_Unlock();
				FLASH_OB_RDP_LevelConfig((uint8_t)0xAA);
				HAL_FLASH_OB_Lock();
				HAL_FLASH_Lock();
				printf("Chip will unlock and flash will be erased after reset. \r\n");
			}
			else
			{
				i--;
				printf("Error password! %d times left!\r\n",i);
			}
		}
		else
		{
			printf("CHIP LOCKED!!!");
		}
		#else
		printf("The chip has been protected forever!");
		#endif
	#else
	printf("Chip will explode in 3 secs!!!");
	#endif
	
}	
