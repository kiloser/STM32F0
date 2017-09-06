#ifndef __USART_V1_H
#define __USART_V1_H

#include "stm32f0xx_usart.h"

#define USART_REC_LEN 128
//#define TXBUFFERSIZE   128
//#define RXBUFFERSIZE   128
/* pravite */
//extern uint8_t TxBuffer[TXBUFFERSIZE];
//extern uint8_t RxBuffer[RXBUFFERSIZE];
//extern uint8_t NbrOfDataToTransfer;
//extern uint8_t NbrOfDataToRead;
//extern __IO uint8_t TxCount; 
//extern __IO uint16_t RxCount;
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern uint8_t  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern uint16_t USART_RX_STA;         		//����״̬���
extern uint8_t usart_rx_buff[64];
extern uint8_t Res_in;
void USART_Config(void);
void USART1_IRQHandler(void);
#endif
