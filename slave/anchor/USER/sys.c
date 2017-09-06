#include "sys.h"
void System_GetClocks(void)
{
  RCC_ClocksTypeDef rcc_clocks;
   
  RCC_GetClocksFreq(&rcc_clocks);
   
  printf("SYSCLK    = %dMHz\r\n", rcc_clocks.SYSCLK_Frequency / 1000000);
  printf("HCLK(AHB) = %dMHz\r\n", rcc_clocks.HCLK_Frequency / 1000000);
  printf("HCLK(AHB) = %dMHz\r\n", rcc_clocks.HCLK_Frequency / 1000000);
  printf("PCLK(APB) = %dMHz\r\n", rcc_clocks.PCLK_Frequency / 1000000);
}



//void GetIDs(void)
//{
//	uint32 lotID ;
//	uint32 partID;
//	uint32 devID;
//	lotID = dwt_getlotid();
//	partID = dwt_getpartid();
//	devID = dwt_readdevid();
//	
//}

void getSYSstatus(uint8 index)
{
	uint8 headbuff[1]={0x0f};
	uint8 headlength=1;
	uint8 bodylength=5;
	uint8 bodybuff[5];
	readfromspi(headlength,headbuff,bodylength,bodybuff);
	printf("%d\r\n",bodybuff[index]);
	
}
void getIDs(uint8 index)
{
	uint8 headbuff[1]={0x00};
	uint8 headlength=1;
	uint8 bodylength=4;
	uint8 bodybuff[4];
	readfromspi(headlength,headbuff,bodylength,bodybuff);
	printf("%d\r\n",bodybuff[index]);
}	

void read_test(unsigned char add)
{
	uint32 id;
	id=dwt_readdevid();
	printf("%lx\r\n",id);
	
}
	
