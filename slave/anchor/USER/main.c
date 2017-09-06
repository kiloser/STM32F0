
#include "sys.h"


static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* Use non-standard SFD (Boolean) */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};
/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16495
#define RX_ANT_DLY 16495


#define SEND_ORDER 3 //max 3
/* Frames used in the ranging process. See NOTE 2 below. */
static uint8 rx_poll_msg[] = {0x26, 0x17, 0x26, 0x16, 0x33, 0, 0, 0, 0, 0};
static uint8 tx_resp_msg[] = {0x26, 0x17, 0x26, 0x16, 0x44, 0, 0, 0, 0x38,SEND_ORDER,0, 0};//anchor ID here
static uint8 rx_final_msg[] = {0x26, 0x17, 0x26, 0x16, 0x55,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 rx_tell_msg[] = {0x26, 0x17, 0x26, 0x16, 0x66,0, 0, 0, 0, 0, 0, 0, 0,0};
static uint8 report_msg[] = {0x26, 0x17, 0x26, 0x16,0x77,0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0};
static uint8 send_pc[]={0xFB, 0xFB, 0x0F, 0, 0, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 set_fre_package[]={0xFB,0xFB,0x03,0,0,0x03,0,0};
static uint8 read_fre_package[]={0xFB,0xFB,0x06,0,0,0x04,0,0,0,0,0};
static uint8 set_gain_package[]={0xFB,0xFB,0x03,0,0,0x05,0,0};
static uint8 read_gain_package[]={0xFB,0xFB,0x04,0,0,0x06,0,0,0};
static uint8 read_power_package[]={0xFB,0xFB,0x04,0,0,0x07,0,0,0};
//static uint8 send_pc_end[]={0xFB,0xFB,0x03,0x00,0x00,0x00,0x00,0x00};
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ID_LEN 2
#define ALL_MSG_COMMON_LEN 5
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 5
#define TAG_ID_IDX 6
#define RESP_MSG_ANCHOR_ID_IDX 8
#define FINAL_MSG_POLL_TX_TS_IDX 8
#define FINAL_MSG_RESP_RX_TS_IDX 12
#define FINAL_MSG_FINAL_TX_TS_IDX 8
#define FINAL_MSG_TS_LEN 4
#define REPORT_MSG_ID_IDX 8
#define RX_BUF_LEN 60	
#define get_bit(var,n)(var&(1<<n))
#define set_bit(var,n)(var|=(1<<n))

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */

static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

//60ms没有收到数据就进行下次接收
#define FINAL_RX_TIMEOUT_UUS 60000

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;
static uint64 poll_tx_ts;
static uint64 final_tx_ts;
static uint64 resp_rx_ts;



static uint8 interrupt_triggered=0;
static uint8 received=0;
static uint8 RF_timeout=0;

static uint8 received_numble=0;
static double distance_recv[4]={0,0,0,0};
static uint8 id_buff[2][ID_LEN]={0,0,0,0};
static uint8 tag_id[2] ={0x00,0x00};
static uint8 wearing=0;
static uint8 anchor[5]={0,0,0,0,0};
static uint8 should_send=0;
uint8_t usart_rx_buff[64];//串口buff
//uint8_t usart_rx_buff[64];

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof;
static double distance;

/* String used to display measured distance on LCD screen (16 characters maximum). */
char dist_str[16] = {0};

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static uint64 final_msg_get_ts(const uint8 *ts_field);
void send_to_PC(void);
void send_to_PC_none(void);
void send_to_PC_end(void);

void reset_DW1000(void);
usart_bitfield USART_STA={
	0,
	0,
	0
};//串口接受狀態

int main(void)
{
	SystemInit();
	LED_Init();
	delay_init();
  USART_Config();
//	function_scan();
	usmart_dev.init(48);
	
#ifdef	FLASHPROTECT
	FLASH_Unlock();
	FLASH_OB_Unlock();
		#ifdef MAXRDPLEVEL
		FLASH_OB_RDPConfig(OB_RDP_Level_2);
		#else
		if(FLASH_OB_RDPConfig(OB_RDP_Level_1)==FLASH_COMPLETE)
		{
		printf("set right\r\n");
		}
		else
		{
		printf("set fault\r\n");
		}
	
		#endif
	FLASH_OB_Lock();
	FLASH_Lock();
#endif
	printf("delayinit_OK,usartinit_OK\r\n");
//	usmart_dev.init(48);
//	printf("usamrtinit_OK\r\n");
	spi_init();
	printf("spiinit_OK\r\n");
//	SPI_RF_Init();
//	printf("2401init_OK\r\n");
	reset_DW1000();
	printf("dwreset_OK\r\n");
//	init_variables();
//  if(NRF24L01_Check())
//	{
//			printf("RFID not exist!\r\n");
//        while (1)
//        { };
//	}		
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
   {
        printf("INIT FAILED\r\n");
        while (1)
        { };
    }
	 
	
//		//port_set_deca_isr(dwt_isr);
//		dw1000IRQ_init();
//		printf("dwIRQinit_OK\r\n");
//------------------------		
//		stat = decamutexon() ;
//	//	set_spi_high();
	dw1000IRQ_init();
	dwt_configure(&config);
	printf("dwconfig_OK\r\n");
	
	
	dwt_setleds(DWT_LEDS_ENABLE);

//		decamutexoff(stat) ;
//--------------------------------
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);
	
		
	dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO |DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);	
	
	while (1)
		{
				/* Clear reception timeout to start next ranging process. */
				dwt_rxreset();
				dwt_setrxtimeout(0);
				/* Activate reception immediately. */
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
			  
				/* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
				while (!interrupt_triggered)
				{ };
				interrupt_triggered=0;
			/* Increment frame sequence number after transmission of the response message (modulo 256). */
				
				
				if (received)
				{
						uint32 frame_len;
						received=0;
						/* Clear good RX frame event in the DW1000 status register. */
					//	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
						/* A frame has been received, read it into the local buffer. */
						frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
						if (frame_len <= RX_BUFFER_LEN)
						{
								dwt_readrxdata(rx_buffer, frame_len, 0);
						}

						/* Check that the frame is a poll sent by "DS TWR initiator" example.
						 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
						if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
						{
								memcpy(tag_id,&rx_buffer[TAG_ID_IDX],ID_LEN);
								should_send=1;
								wearing=rx_buffer[ALL_MSG_SN_IDX];
								/* Retrieve poll reception timestamp. */
								poll_rx_ts = get_rx_timestamp_u64();
								/* Write and send the response message. See NOTE 9 below.*/
								Delay_ms((SEND_ORDER-1)*5);
								dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
								dwt_writetxfctrl(sizeof(tx_resp_msg), 0,0);
								dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
								dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
								RF_timeout=0;
								/* We assume that the transmission is achieved correctly, now poll for reception of expected "final" frame or error/timeout.
								 * See NOTE 7 below. */							
								while(RF_timeout==0)
								{
										while (!interrupt_triggered)
										{ };
										interrupt_triggered=0;									
										if (received)
										{
												received=0;
												dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_TXFRS);
												/* A frame has been received, read it into the local buffer. */
												frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
												if (frame_len <= RX_BUF_LEN)
												{
														dwt_readrxdata(rx_buffer, frame_len, 0);
												}

												/* Check that the frame is a final message sent by "DS TWR initiator" example.
												 * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
												if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
												{
														
														double Ra=0, Rb=0, Da=0, Db=0;
														double tof_dtu=0;
														uint8 j;
														uint8 n;
														/* Retrieve response transmission and final reception timestamps. */
														resp_tx_ts = get_tx_timestamp_u64();
														final_rx_ts = get_rx_timestamp_u64();
														/* Get timestamps embedded in the final message. */
														poll_tx_ts=0;
														resp_rx_ts=0;
														for(j=0;j<3;j++)
														{
															if(memcmp(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX+j*(FINAL_MSG_TS_LEN+ID_LEN)],
																&tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX],ID_LEN)==0)
															{																											
																poll_tx_ts=final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX]);
																resp_rx_ts=final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX+j*(FINAL_MSG_TS_LEN+ID_LEN)+ID_LEN]);
																break;
															}
														}
														dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
														dwt_rxenable(0);
														while (!interrupt_triggered)
														{ };
														interrupt_triggered=0;

														if (received)
														{
															received=0;
															/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
														//	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

															/* A frame has been received, read it into the local buffer. */
															frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
															if (frame_len <= RX_BUF_LEN)
															{
																	dwt_readrxdata(rx_buffer, frame_len, 0);
															}

															/* Check that the frame is a final message sent by "DS TWR initiator" example.
															 * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
															if (memcmp(rx_buffer, rx_tell_msg, ALL_MSG_COMMON_LEN) == 0)
															{
																final_tx_ts=0;
																final_tx_ts=final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX]);
																memcpy(tag_id,&rx_buffer[TAG_ID_IDX],ID_LEN);
																wearing=rx_buffer[ALL_MSG_SN_IDX];
																/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped*/

																Ra = (double)(resp_rx_ts - poll_tx_ts);
																Rb = (double)(final_rx_ts- resp_tx_ts);
																Da = (double)(final_tx_ts- resp_rx_ts);
																Db = (double)(resp_tx_ts - poll_rx_ts);
																tof_dtu = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);

																tof = tof_dtu * DWT_TIME_UNITS;
																distance = tof * SPEED_OF_LIGHT;
																if(distance<0)
																	distance=-distance;
																/* Display */
																
																while(SEND_ORDER==1)
																{
																	
																	dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
																	dwt_rxenable(0);
																	while (!interrupt_triggered)
																	{ };
																	interrupt_triggered=0;
																	if (received)
																	{
																		received=0;
																		/* A frame has been received, read it into the local buffer. */
																		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
																		if (frame_len <= RX_BUF_LEN)
																		{
																				dwt_readrxdata(rx_buffer, frame_len, 0);
																		}
																		
																		if (memcmp(rx_buffer, report_msg, ALL_MSG_COMMON_LEN) == 0)
																		{																			
																			memcpy(&distance_recv[received_numble],&rx_buffer[REPORT_MSG_ID_IDX+ID_LEN],sizeof(double));
																			memcpy(&id_buff[received_numble][0],&rx_buffer[REPORT_MSG_ID_IDX],ID_LEN);
																			//printf("DIST from %x%x is:%3.2f m\r\n",rx_buffer[8],rx_buffer[9],distance_recv[received_numble]);
																			received_numble++;						
																		}																	
																	}																	
																	if(RF_timeout>0||received_numble>=2)
																			break;													
																}
																
																if(SEND_ORDER!=1)	
																{
																	memcpy(&report_msg[RESP_MSG_ANCHOR_ID_IDX],&tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX],ID_LEN);
																	memcpy(&report_msg[RESP_MSG_ANCHOR_ID_IDX+ID_LEN],&distance,sizeof(double));
																	Delay_ms((3-SEND_ORDER)*5+1);
																	dwt_writetxdata(sizeof(report_msg), report_msg, 0);
																	dwt_writetxfctrl(sizeof(report_msg), 0,0);
																	dwt_starttx(DWT_START_TX_IMMEDIATE);
																	while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
																	{ };
																}else
																{
															//		printf("[%d:%d:%3.2f:%d]\r\n",tx_resp_msg[9],tag_id[1],distance,wearing);
																	if(received_numble>=2)
																		if(distance<150&&distance_recv[0]<150&&distance_recv[1]<150)
																		{
																			send_to_PC();
																			should_send=0;
																		}
														//			for(n=0;n<received_numble;n++)
														//			{
														//				printf("[%d:%d:%3.2f:%d]\r\n",id_buff[n][1],tag_id[1],distance_recv[n],wearing);
														//			}
																	received_numble=0;
																}
															}																														
														}
													break;
												}
												else
												{
													if(RF_timeout>0)
													{break;}
													dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
													dwt_rxenable(0);
												}
										}else
										{
												if(RF_timeout>0)
												{break;}													
												dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
												dwt_rxenable(0);
										}
								}
									
						}
				}
				if(should_send)
				{
					should_send=0;
					send_to_PC_none();
				}
				
		}
	
}


void EXTI4_15_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line12))
	{
		EXTI_ClearITPendingBit(EXTI_Line12);
		interrupt_triggered=1;
		status_reg = dwt_read32bitreg(SYS_STATUS_ID);
		if (status_reg & SYS_STATUS_RXFCG)
		{
			received=1;
		}
		else if(status_reg & SYS_STATUS_RXRFTO)
			RF_timeout++;
		dwt_write32bitreg(SYS_STATUS_ID, status_reg);
	}
}

void send_to_PC(void)
{
	uint64 buff_dis;
	uint8_t i;
	uint8_t len=16;
	uint8_t crc=0;
	uint8_t *pointerP;
	send_pc[3]=tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX];
	send_pc[4]=tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX+1];
	send_pc[6]=tag_id[0];
	send_pc[7]=tag_id[1];
	if(get_bit(wearing,0))
		set_bit(send_pc[8],0);
	send_pc[8]|=0x0E;
//	send_pc[8]=状态字；
	buff_dis=(uint64)(distance*1000);
	send_pc[10]=(uint8)(buff_dis&0xff);
	send_pc[9]=(uint8)((buff_dis>>8)&0xff);
	buff_dis=(uint64)(distance_recv[0]*1000);
	send_pc[12]=(uint8)(buff_dis&0xff);
	send_pc[11]=(uint8)((buff_dis>>8)&0xff);
	buff_dis=(uint64)(distance_recv[1]*1000);
	send_pc[14]=(uint8)(buff_dis&0xff);
	send_pc[13]=(uint8)((buff_dis>>8)&0xff);
	buff_dis=(uint64)(distance_recv[2]*1000);
	send_pc[16]=(uint8)(buff_dis&0xff);
	send_pc[15]=(uint8)((buff_dis>>8)&0xff);
	buff_dis=(uint64)(distance_recv[3]*1000);
	send_pc[18]=(uint8)(buff_dis&0xff);
	send_pc[17]=(uint8)((buff_dis>>8)&0xff);
	pointerP=&send_pc[2];
	while(len--)
	{
		for(i=0x80;i!=0;i>>=1)
		{
			if((crc&0x40)!=0) {crc<<=1;crc^=9;}
			else crc<<=1;
			if((*pointerP&i)!=0) crc^=9;		 
		}
		pointerP++;
	}
	crc=(uint8_t)(crc&0x7f);
	crc=(uint8_t)(crc<<1);
	crc=(uint8_t)(crc|0x01);
	send_pc[19]=crc;
	for(i=0;i<20;i++)
	{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 
   USART_SendData(USART1,(uint8_t)send_pc[i]);
	}
}

void set_fre(void)
{
	uint8_t i;
	uint8_t len=3;
	uint8_t crc=0;
	uint8_t *pointerP;
	set_fre_package[3]=tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX];
	set_fre_package[4]=tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX+1];
	set_fre_package[6]=0x00;                //返回的定位频率值
  pointerP=&set_fre_package[2];
	while(len--)
	{
		for(i=0x80;i!=0;i>>=1)
		{
			if((crc&0x40)!=0) {crc<<=1;crc^=9;}
			else crc<<=1;
			if((*pointerP&i)!=0) crc^=9;		 
		}
		pointerP++;
	}
	crc=(uint8_t)(crc&0x7f);
	crc=(uint8_t)(crc<<1);
	crc=(uint8_t)(crc|0x01);
	set_fre_package[7]=crc;
	for(i=0;i<8;i++)
	{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
   USART_SendData(USART1,(uint8_t)set_fre_package[i]);
	}
}

void read_fre(void)
{
	uint8_t i;
	uint8_t len=6;
	uint8_t crc=0;
	uint8_t *pointerP;
	read_fre_package[3]=tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX];
	read_fre_package[4]=tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX+1];
	read_fre_package[6]=0x00;	              //读取的定位频率值
	read_fre_package[7]=0x00;
	read_fre_package[8]=0x00;
	read_fre_package[9]=0x00;
  pointerP=&read_fre_package[2];
	while(len--)
	{
		for(i=0x80;i!=0;i>>=1)
		{
			if((crc&0x40)!=0) {crc<<=1;crc^=9;}
			else crc<<=1;
			if((*pointerP&i)!=0) crc^=9;		 
		}
		pointerP++;
	}
	crc=(uint8_t)(crc&0x7f);
	crc=(uint8_t)(crc<<1);
	crc=(uint8_t)(crc|0x01);
	read_fre_package[10]=crc;
	for(i=0;i<11;i++)
	{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
   USART_SendData(USART1,(uint8_t)read_fre_package[i]);
	}
}

void set_gain(void)
{
	uint8_t i;
	uint8_t len=3;
	uint8_t crc=0;
	uint8_t *pointerP;
	set_gain_package[3]=tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX];
	set_gain_package[4]=tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX+1];
	set_gain_package[6]=0x00;                //返回值，设置成功返回1
  pointerP=&set_gain_package[2];
	while(len--)
	{
		for(i=0x80;i!=0;i>>=1)
		{
			if((crc&0x40)!=0) {crc<<=1;crc^=9;}
			else crc<<=1;
			if((*pointerP&i)!=0) crc^=9;		 
		}
		pointerP++;
	}
	crc=(uint8_t)(crc&0x7f);
	crc=(uint8_t)(crc<<1);
	crc=(uint8_t)(crc|0x01);
	set_gain_package[7]=crc;
	for(i=0;i<8;i++)
	{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
   USART_SendData(USART1,(uint8_t)set_gain_package[i]);
	}
}

void read_gain(void)
{
	uint8_t i;
	uint8_t len=4;
	uint8_t crc=0;
	uint8_t *pointerP;
	read_gain_package[3]=tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX];
	read_gain_package[4]=tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX+1];
	read_gain_package[6]=0x00;                //返回的增益值
	read_gain_package[7]=0x00;
  pointerP=&read_gain_package[2];
	while(len--)
	{
		for(i=0x80;i!=0;i>>=1)
		{
			if((crc&0x40)!=0) {crc<<=1;crc^=9;}
			else crc<<=1;
			if((*pointerP&i)!=0) crc^=9;		 
		}
		pointerP++;
	}
	crc=(uint8_t)(crc&0x7f);
	crc=(uint8_t)(crc<<1);
	crc=(uint8_t)(crc|0x01);
	read_gain_package[8]=crc;
	for(i=0;i<9;i++)
	{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
   USART_SendData(USART1,(uint8_t)read_gain_package[i]);
	}
}
void read_power(void)
{
	uint8_t i;
	uint8_t len=4;
	uint8_t crc=0;
	uint8_t *pointerP;
	read_power_package[3]=tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX];
	read_power_package[4]=tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX+1];
	read_power_package[6]=0x00;                //返回的增益值
	read_power_package[7]=0x00;
  pointerP=&read_power_package[2];
	while(len--)
	{
		for(i=0x80;i!=0;i>>=1)
		{
			if((crc&0x40)!=0) {crc<<=1;crc^=9;}
			else crc<<=1;
			if((*pointerP&i)!=0) crc^=9;		 
		}
		pointerP++;
	}
	crc=(uint8_t)(crc&0x7f);
	crc=(uint8_t)(crc<<1);
	crc=(uint8_t)(crc|0x01);
	read_power_package[8]=crc;
	for(i=0;i<9;i++)
	{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
   USART_SendData(USART1,(uint8_t)read_power_package[i]);
	}
}

void send_to_PC_none(void)
{
	uint64 buff_dis;
	uint8_t i;
	uint8_t len=17;
	uint8_t crc=0;
	uint8_t *pointerP;
	send_pc[3]=tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX];
	send_pc[4]=tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX+1];
	send_pc[6]=tag_id[0];
	send_pc[7]=tag_id[1];
	if(get_bit(wearing,0))
		set_bit(send_pc[8],0);
	send_pc[9]=0;
	send_pc[10]=0;
	send_pc[11]=0;
	send_pc[12]=0;
	send_pc[13]=0;
	send_pc[14]=0;
	send_pc[15]=0;
	send_pc[16]=0;
	send_pc[17]=0;
	send_pc[18]=0;
	pointerP=&send_pc[2];
	while(len--)
	{
		for(i=0x80;i!=0;i>>=1)
		{
			if((crc&0x40)!=0) {crc<<=1;crc^=9;}
			else crc<<=1;
			if((*pointerP&i)!=0) crc^=9;		 
		}
		pointerP++;
	}
	crc=(uint8_t)(crc&0x7f);
	crc=(uint8_t)(crc<<1);
	crc=(uint8_t)(crc|0x01);
	send_pc[19]=crc;
	for(i=0;i<20;i++)
	{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 
   USART_SendData(USART1,(uint8_t)send_pc[i]);
	}
}

void reset_DW1000(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable GPIO used for DW1000 reset
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//drive the RSTn pin low
	GPIO_ResetBits(GPIOC, GPIO_Pin_1);
	Delay_ms(1);

	//put the pin back to tri-state ... as input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

  Delay_ms(5);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static uint64 final_msg_get_ts(const uint8 *ts_field)
{
    uint64 buff=0;
		int i;
    for (i=FINAL_MSG_TS_LEN-1; i >=0 ; i--)
    {
        buff <<=8;
				buff |= ts_field[i];
    }
		return buff;
}

void unlockflash(unsigned int passwd)
{
	
	#ifdef FLASHPROTECT
	static uint8 i=5;
		#ifndef MAXRDPLEVEL
		if(i>0)
		{
			if(passwd==26172617)
			{
				FLASH_Unlock();
				FLASH_OB_Unlock();
				printf("Chip will unlock and flash will be erased after reset. \r\n");
				if(FLASH_OB_RDPConfig(OB_RDP_Level_0)==FLASH_COMPLETE)
				{
				printf("set right\r\n");
				}
				else
				{
				printf("set fault\r\n");
				}
				FLASH_OB_Lock();
				FLASH_Lock();
				
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
