#include "usmart.h"
#include "usmart_str.h" 

////////////////////////////�û�������///////////////////////////////////////////////
//������Ҫ�������õ��ĺ�����������ͷ�ļ�(�û��Լ����) 
#include "delay.h"	
#include "usart_v1.h"		
#include "sys.h"

												 
extern void test_fun(void(*ledset)(uint8_t),uint8_t sta);
extern void * test1(void);
//�������б��ʼ��(�û��Լ����)
//�û�ֱ������������Ҫִ�еĺ�����������Ҵ�
struct _m_usmart_nametab usmart_nametab[]=
{
#if USMART_USE_WRFUNS==0 	//���ʹ���˶�д����
	(void*)read_addr,"uint32_t read_addr(uint32_t addr)",
	(void*)write_addr,"void write_addr(uint32_t addr,uint32_t val)",	 
#endif
	(void*)Delay_ms,"void delay_ms(uint16_t nms)",
//	(void*)Delay_us,"void delay_us(uint32_t nus)",
	(void*)System_GetClocks,"void System_GetClocks(void)",
	(void*)getSYSstatus,"void getSYSstatus(uint8 index)",
	(void*)getIDs,"void getIDs(uint8 index)",
	(void*)read_test,"void read_test(unsigned char add)",
	(void*)unlockflash,"void unlockflash(unsigned int passwd)",

											 
};						  
///////////////////////////////////END///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//�������ƹ�������ʼ��
//�õ������ܿغ���������
//�õ�����������
struct _m_usmart_dev usmart_dev=
{
	usmart_nametab,
	usmart_init,
	usmart_cmd_rec,
	usmart_exe,
	usmart_scan,
	sizeof(usmart_nametab)/sizeof(struct _m_usmart_nametab),//��������
	0,	  	//��������
	0,	 	//����ID
	1,		//������ʾ����,0,10����;1,16����
	0,		//��������.bitx:,0,����;1,�ַ���	    
	0,	  	//ÿ�������ĳ����ݴ��,��ҪMAX_PARM��0��ʼ��
	0,		//�����Ĳ���,��ҪPARM_LEN��0��ʼ��
};   



















