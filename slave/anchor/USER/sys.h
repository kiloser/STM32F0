
#ifndef __SYS_H
#define __SYS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "spi_v1.h"
#include "usart_v1.h"
#include "delay.h"
#include "LED.h"
#include "usmart.h"
#include "EXTI_v1.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "string.h"
#include "deca_callback.h"
#include "rf24l01.h"
#include "node_sync.h"
#include "stdio.h"
#define FLASHPROTECT
typedef struct 
{
	uint16_t RC :1;
	uint16_t RD :1;
	uint16_t count :14;//16383 bytes
	
}usart_bitfield;

void System_GetClocks(void);
void getSYSstatus(uint8 index);
void getIDs(uint8 index);
void read_test(unsigned char add);
void unlockflash(unsigned int passwd);

void function_scan(void);
void set_fre(void);
void read_fre(void);
void set_gain(void);
void read_gain(void);
void read_power(void);
extern usart_bitfield USART_STA;
extern uint8_t usart_rx_buff[64];
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
