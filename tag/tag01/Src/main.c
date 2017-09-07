
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
/* USER CODE BEGIN Includes */
#include "main.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
//#define NEW
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


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
/* USER CODE END PV */

void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);

static uint8 tag_id[2] ={0x00,0x04};
static uint16 tag_delay=450;
static uint8 tx_poll_msg[] = {0x26, 0x17, 0x26, 0x16, 0x33, 0, 0, 0, 0, 0};
static uint8 rx_resp_msg[] = {0x26, 0x17, 0x26, 0x16, 0x44, 0, 0, 0, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x26, 0x17, 0x26, 0x16, 0x55, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,
0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 tx_tell_msg[] = {0x26, 0x17, 0x26, 0x16, 0x66, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8 rx_buffer[RX_BUF_LEN];
static uint32 status_reg = 0;	
	
static uint8 final_ok =0;
static uint8 interrupt_triggered =0;
static uint8 received =0;		
static uint8 RF_timeout=0;
static uint8 wearing=0;
uint32 localtime=0;//本地时间 在TIM14中断中每1ms增加1
uint32 acTime=0;
uint16 startCnt=0;
uint16 cnta=0;
uint8_t usart_rx_buff[64];//串口buff
uint32 startTime=1;

uint8 startFlag=0;
uint8 flag10s=0;
uint8 cntx=0;

typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

/*online timestamp*/
static uint64 online_tx_ts;
static uint32 tag_order=1;

uint32 start_frame_len;

static uint8 dummy_buffer[DUMMY_BUFFER_LEN];
/* Private function prototypes -----------------------------------------------*/


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void EXTI2_3_IRQHandler_Config(void);
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
void dw_setARER(int);

usart_bitfield USART_STA={
	0,
	0,
	0
};//串口接受狀態
char newdata=0;

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

int main(void)
{    

  /* USER CODE BEGIN 1 */
	uint16 lp_osc_freq, sleep_cnt;
  /* USER CODE END 1 */
	uint8 leftstatus=0;
	uint8 rightstatus=0;
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
	MX_TIM14_Init();
	MX_TIM16_Init();
	
#ifdef	FLASHPROTECT
	HAL_FLASH_Unlock();
	HAL_FLASH_OB_Unlock();
		#ifdef MAXRDPLEVEL
		FLASH_OB_RDP_LevelConfig(OB_RDP_Level_2);
		#else
		FLASH_OB_RDP_LevelConfig((uint8_t)0xBB);
		#endif
	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();
#endif

//  HAL_TIM_Base_Start_IT(&htim14);
//	HAL_TIM_Base_Start_IT(&htim16);
	delay_init();
	usmart_init(48);//in this case, parameter is useless
	HAL_UART_Receive_IT(&huart1, usart_rx_buff, 64);
	
	EXTI->PR = 0x7bffff;//clear pending bits
	EXTI2_3_IRQHandler_Config();
	memcpy(&tx_poll_msg[TAG_ID_IDX],tag_id,ID_LEN);
	memcpy(&rx_resp_msg[TAG_ID_IDX],tag_id,ID_LEN);
	memcpy(&tx_final_msg[TAG_ID_IDX],tag_id,ID_LEN);
	memcpy(&tx_tell_msg[TAG_ID_IDX],tag_id,ID_LEN);
	reset_DW1000();
	
	
	/**************************************************/
	/*change*/
	dwt_spicswakeup(dummy_buffer, DUMMY_BUFFER_LEN);
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
			{
					printf("INIT FAILED\r\n");
					while (1)
					{ };
			}
			
	dwt_configure(&config);
	/* Clear reception timeout to start next ranging process. */
				dw_setARER(1);			
				dwt_rxreset();
				dwt_setrxantennadelay(RX_ANT_DLY);
				dwt_settxantennadelay(TX_ANT_DLY);
				dwt_setleds(DWT_LEDS_ENABLE);
				dwt_setrxtimeout(0);
				dwt_setinterrupt( DWT_INT_RFTO | DWT_INT_RFCG, 1);
				/* Activate reception immediately. */
lab:		dwt_forcetrxoff();
				dwt_setleds(DWT_LEDS_ENABLE);
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
			
			
			
  
			while(1)
			{
						
						printf("enter waiting :\t %ld\r\n",localtime);
						while(!interrupt_triggered);
						interrupt_triggered=0;
						received=0;
						/* A frame has been received, read it into the local buffer. */
						start_frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
						if (start_frame_len <= RX_BUF_LEN)
						{
							dwt_readrxdata(rx_buffer, start_frame_len, 0);
						}
				
							if(rx_buffer[0]==0x26&rx_buffer[1]==0x17&rx_buffer[4]==0x88)
							{
								acTime=0;
								acTime=rx_buffer[5];
								acTime+=(uint32)(rx_buffer[6]<<8);
								acTime+=(uint32)(rx_buffer[7]<<16);
								acTime+=(uint32)(rx_buffer[8]<<24);
							  
								HAL_Delay(1000-acTime%1000);
								HAL_Delay(100*tag_id[1]);
								
							  HAL_TIM_Base_Start_IT(&htim14);
								HAL_TIM_Base_Start_IT(&htim16);
								
								
								cntx=0;
								break;
						
							}
							else
							{
								/* Activate reception immediately. */
								dwt_rxenable(DWT_START_RX_IMMEDIATE);
							}
						
			}
			
			
			
			
	dwt_configuresleep(DWT_PRESRV_SLEEP | DWT_CONFIG, DWT_WAKE_CS |DWT_WAKE_WK| DWT_SLP_EN);
	/**************************************************/		
	
	while (1)
	{
		while(dwt_spicswakeup(dummy_buffer, DUMMY_BUFFER_LEN)!=DWT_SUCCESS);
		wearing=0;
		#ifdef NEW
		leftstatus=HAL_GPIO_ReadPin(GPIOA,SENL_INT_Pin);
		rightstatus=HAL_GPIO_ReadPin(GPIOB,SENR_INT_Pin);
		#else
		leftstatus=HAL_GPIO_ReadPin(GPIOA,SENL_P2_Pin)||HAL_GPIO_ReadPin(GPIOA,SENL_P1_Pin);
		rightstatus=HAL_GPIO_ReadPin(GPIOA,SENR_P2_Pin)||HAL_GPIO_ReadPin(GPIOA,SENR_P1_Pin);
		#endif

		if(leftstatus==1&&rightstatus==0)
		{wearing=1;printf("status=%d\r\n",wearing);}
		else if(leftstatus==0&&rightstatus==1)
		{wearing=2;printf("status=%d\r\n",wearing);}
		else if(leftstatus==1&&rightstatus==1)
		{wearing=3;printf("status=%d\r\n",wearing);}
		///07-25 change by liujing
		if(wearing==3)
		{wearing=1;}
		else
		{
			wearing=0;
		}
		
			if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
			{
					printf("INIT FAILED\r\n");
					while (1)
					{ };
			}
			lp_osc_freq = (XTAL_FREQ_HZ / 2) / dwt_calibratesleepcnt();
			sleep_cnt = ((SLEEP_TIME_MS * lp_osc_freq) / 1000) >> 12;
			dwt_configuresleepcnt(sleep_cnt);
			dwt_configure(&config);
				
			dwt_setleds(DWT_LEDS_ENABLE);
			/* Apply default antenna delay value. See NOTE 1 below. */
			dwt_setrxantennadelay(RX_ANT_DLY);
			dwt_settxantennadelay(TX_ANT_DLY);
			dwt_setinterrupt( DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT|DWT_INT_RFCG, 1);
			dwt_configuresleep(DWT_PRESRV_SLEEP | DWT_CONFIG, DWT_WAKE_SLPCNT | DWT_SLP_EN);
		
		
		/* USER CODE BEGIN 2 */	
			HAL_Delay(5);
			/* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
			dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
			dwt_writetxfctrl(sizeof(tx_poll_msg), 0,0);
			dwt_setrxtimeout(TIME_OUT_UUS);
			/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
			 * set by dwt_setrxaftertxdelay() has elapsed. */
			dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);
			RF_timeout=0;
			/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
			while (!interrupt_triggered)
			{ };
		//	interrupt_triggered=0;
		//	status_reg = dwt_read32bitreg(SYS_STATUS_ID);
			/* Increment frame sequence number after transmission of the poll message (modulo 256). */
			memset(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX],0,sizeof(tx_final_msg)-FINAL_MSG_RESP_RX_TS_IDX);
				
			
				while(RF_timeout==0)
				{
					while (!interrupt_triggered)
					{ };
					interrupt_triggered=0;
					if (received)
					{
							uint32 frame_len;
							
							received=0;
							/* Clear good TX frame sent in the DW1000 status register. */
							dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_TXFRS);
							
							/* A frame has been received, read it into the local buffer. */
							frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
							if (frame_len <= RX_BUF_LEN)
							{
									dwt_readrxdata(rx_buffer, frame_len, 0);
							}

							/* Check that the frame is the expected response from the companion "DS TWR responder" example.
							 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
							if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
							{
									final_ok++;
									resp_rx_ts = get_rx_timestamp_u64();
									poll_tx_ts = get_tx_timestamp_u64();
									final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
									memcpy(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX+(final_ok-1)*(FINAL_MSG_TS_LEN
									+ID_LEN)],&rx_buffer[RESP_MSG_ANCHOR_ID_IDX],ID_LEN);
									final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX+(final_ok-1)*(FINAL_MSG_TS_LEN
									+ID_LEN)+ID_LEN], resp_rx_ts);
							}
						}
					if(final_ok>=3||RF_timeout>0)
						break;
					dwt_setrxtimeout(TIME_OUT_UUS);
					dwt_rxenable(0);
				}
				if(final_ok)
				{
						final_ok=0;
						for(int j=0;j<10000;j++);
						/* Write and send final message */
						dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
						dwt_writetxfctrl(sizeof(tx_final_msg),0,0);
						dwt_starttx(DWT_START_TX_IMMEDIATE);
						while (!(status_reg=dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{ };

						/* Clear TXFRS event. */
						dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
						for(int j=0;j<20000;j++);

						/*------------------------- send tell message ------------------------------*/	
						final_tx_ts = get_tx_timestamp_u64();
						final_msg_set_ts(&tx_tell_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);	
							
						dwt_writetxdata(sizeof(tx_tell_msg), tx_tell_msg, 0);
						dwt_writetxfctrl(sizeof(tx_tell_msg),0,0);
						dwt_starttx(DWT_START_TX_IMMEDIATE);
						while (!(status_reg=dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{ };

						/* Clear TXFRS event. */
						dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
				}
				cntx++;
				
				
				
				printf("time :\t %ld\r\n",localtime);
				if(cntx==9)
				{		
					while(!flag10s);
					flag10s=0;

					dwt_setrxtimeout(0);
					goto lab;
				}
				else
				{
					dwt_entersleep();
				}
				
				while(!cnta);
				cnta=0;
					
			//	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_SLP2INIT);
	}
	

}


/* USER CODE BEGIN 1 */
void EXTI2_3_IRQHandler(void)
{ 
	uint32_t status;
	status = dwt_read32bitreg(SYS_STATUS_ID);// Read status register low 32bits
	interrupt_triggered=1;	
	if(status&SYS_STATUS_RXFCG)
	{
		received=1;
	}
	else if(status & SYS_STATUS_RXRFTO)
			RF_timeout++;
	dwt_write32bitreg(SYS_STATUS_ID, status);
//    } while (HAL_GPIO_ReadPin(DW_IRQ_GPIO_Port,DW_IRQ_Pin) != RESET);
    /* Clear EXTI Line  Pending Bit */
	__HAL_GPIO_EXTI_CLEAR_IT(DW_IRQ_Pin);
		
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
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}




/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_TIM14_Init(void)
{

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 4799;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 10000;//1ms 中断一次
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 47990;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9900;//9.9s中断一次
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : MPU_INT_Pin */
  GPIO_InitStruct.Pin = MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU_AD0_Pin */
  GPIO_InitStruct.Pin = MPU_AD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MPU_AD0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SENR_P2_Pin SENR_P1_Pin SENL_P2_Pin SENL_P1_Pin */
  GPIO_InitStruct.Pin = SENR_P2_Pin|SENR_P1_Pin|SENL_P2_Pin|SENL_P1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DW_IRQ_Pin */
  GPIO_InitStruct.Pin = DW_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DW_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_CE_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NRF_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_INT_Pin */
  GPIO_InitStruct.Pin = NRF_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(NRF_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DWWAKE_Pin */
  GPIO_InitStruct.Pin = DWWAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DWWAKE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DWRESET_Pin */
  GPIO_InitStruct.Pin = DWRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DWRESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_NSS_Pin I2C1_SCL_Pin I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_Pin|I2C1_SCL_Pin|I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SENL_INT_Pin */
  GPIO_InitStruct.Pin = SENL_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SENL_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENR_INT_Pin */
  GPIO_InitStruct.Pin = SENR_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SENR_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MPU_AD0_GPIO_Port, MPU_AD0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DWWAKE_Pin|I2C1_SCL_Pin|I2C1_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);

}

/* USER CODE BEGIN 4 */
static void EXTI2_3_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;


  __HAL_RCC_GPIOA_CLK_ENABLE();


  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Pin = DW_IRQ_Pin;
  HAL_GPIO_Init(DW_IRQ_GPIO_Port, &GPIO_InitStructure);

  /* Enable and set EXTI line 4_15 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}


//使能自動重啓接收器
void dw_setARER(int enable)
{
	uint32 syscfg;
	syscfg=dwt_read32bitreg(SYS_CFG_ID);
	if(enable)
	{
		syscfg |= SYS_CFG_RXAUTR;
	}
	else
	{
		syscfg &=~(SYS_CFG_RXAUTR);
	}
	dwt_write32bitreg(SYS_CFG_ID,syscfg);
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
