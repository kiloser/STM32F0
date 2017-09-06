#ifndef __MAIN_H
#define __MAIN_H
#include "dw1000.h"
#include "deca_callback.h"
#include "delay.h"
#include "usmart.h"
#include "USART.h"
#define RNG_DELAY_MS 5000
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436
#define XTAL_FREQ_HZ 38400000
//#define SLEEP_TIME_MS 500

#define ID_LEN 2
#define ALL_MSG_COMMON_LEN 5
#define ALL_MSG_SN_IDX 5
#define RESP_MSG_ANCHOR_ID_IDX 8
#define TAG_ID_IDX 6
#define FINAL_MSG_POLL_TX_TS_IDX 8
#define FINAL_MSG_RESP_RX_TS_IDX 12
#define FINAL_MSG_FINAL_TX_TS_IDX 8
#define FINAL_MSG_TS_LEN 4
#define RX_BUF_LEN 60
#define TIME_OUT_UUS 10000

#define XTAL_FREQ_HZ 38400000
#define SLEEP_TIME_MS 1000

#define DUMMY_BUFFER_LEN 500
//#define FLASHPROTECT

typedef struct 
{
	uint16_t RC :1;
	uint16_t RD :1;
	uint16_t count :14;//16383 bytes
	
}usart_bitfield;

extern uint8_t usart_rx_buff[64];
extern usart_bitfield USART_STA;
extern uint32 localtime;
extern uint16 startCnt;
extern uint16 cnta;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart1;
extern char newdata;
extern uint8 startFlag;
extern uint8 flag10s;
#endif
