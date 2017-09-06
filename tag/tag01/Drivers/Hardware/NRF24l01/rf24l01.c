//////////////////////////////////////////////
//  * Ӳ������ ----------------------------
//  *         | PA3-CE          :24L01-CE |
//  *         | PB1-IRQ  :      24L01-IRQ |
//  *         | PA4-CS  :        24L01-CS  |
//  *         | PA5-SPI1-SCK  : 24L01-CLK |
//  *         | PA6-SPI1-MISO : 24L01-DO  |
//  *         | PA7-SPI1-MOSI : 24L01-DIO |
//  *          ----------------------------
//  * ��汾  ��ST3.0.0
//  *
////////////////////////////////////////////

#include  "rf24l01.h"

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x68,0x86,0x66,0x88,0x28}; //���͵�ַ
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x68,0x86,0x66,0x88,0x28}; //���͵�ַ

/**************************************
* ������   : SPI_RF_Init
* ����     : RF24L01��ʼ��
* ������� : ��
* ����ֵ   : ��
**************************************/
void SPI_RF_Init(void)
{
	 
  GPIO_InitTypeDef  GPIO_InitStruct;
  SPI_InitTypeDef   SPI_InitStruct;
	EXTI_InitTypeDef	EXTI_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure; 
  /*!< SD_SPI_CS_GPIO, SD_SPI_MOSI_GPIO, SD_SPI_MISO_GPIO, SD_SPI_DETECT_GPIO 
       and SD_SPI_SCK_GPIO Periph clock enable */
// 	 RCC_AHBPeriphClockCmd(FLASH_CS_PIN_SCK|FLASH_SCK_PIN_SCK|FLASH_MISO_PIN_SCK | FLASH_MOSI_PIN_SCK, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA| RCC_AHBPeriph_GPIOB, ENABLE);
  /*!< SD_SPI Periph clock enable */
  RCC_APB2PeriphClockCmd(RF_SPI1, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
  /*!< Configure RF_SPI pins: SCK */
  GPIO_InitStruct.GPIO_Pin = RF_SCK_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP; 
  GPIO_Init(RF_SCK_PORT, &GPIO_InitStruct);
	
  /*!< Configure RF_SPI pins: MISO */
  GPIO_InitStruct.GPIO_Pin = RF_MISO_PIN;
  GPIO_Init(RF_MISO_PORT, &GPIO_InitStruct);

  /*!< Configure RF_SPI pins: MOSI */
  GPIO_InitStruct.GPIO_Pin =RF_MOSI_PIN;
  GPIO_Init(RF_MOSI_PORT, &GPIO_InitStruct);
  
  /* Connect PXx to RF_SPI_SCK */
  GPIO_PinAFConfig(RF_SCK_PORT, RF_SCK_SOURCE, RF_SCK_AF);

  /* Connect PXx to RF_SPI_MISO */
  GPIO_PinAFConfig(RF_MISO_PORT, RF_MISO_SOURCE, RF_MISO_AF); 

  /* Connect PXx to RF_SPI_MOSI */
  GPIO_PinAFConfig(RF_MOSI_PORT, RF_MOSI_SOURCE, RF_MOSI_AF);
	
	
	 /*!< Configure SD_SPI_CS_PIN pin: SD Card CS pin */
  GPIO_InitStruct.GPIO_Pin =RF_CS_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
  GPIO_Init(RF_CS_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin =RF_CE_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
  GPIO_Init(RF_CE_PORT, &GPIO_InitStruct);
	// define IRQ pin	
		
	GPIO_InitStruct.GPIO_Pin =RF_IQR_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_Init(RF_IQR_PORT , &GPIO_InitStruct);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);
	
// 	SPI_RF_CE_LOW();
  	SPI_RF_CS_HIGH() ;
  /*!< SD_SPI Config */
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
//   SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
//   SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
	  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;		 				//ʱ�Ӽ��ԣ�����ʱΪ��
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;		
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStruct.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStruct);
  SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
  SPI_Cmd(SPI1, ENABLE); /*!< SD_SPI enable */
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	/* Enable and set EXTI11 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**************************************
* ������   : NRF24L01_Write_Reg
* ����     : ��?4L01�ļĴ���дֵ��һ���ֽڣ�
* ������� : uint8_t reg,uint8_t value
*            reg    Ҫд�ļĴ�����ַ  
*            value  ���Ĵ���д��ֵ
* ����ֵ   : һ���ֽڵ�״ֵ̬ 
**************************************/
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;

	SPI_RF_CS_LOW() ;	 //CSN=0;   
  status = SPI_RF_SendByte(reg);//���ͼĴ�����ַ,����ȡ״ֵ̬
	SPI_RF_SendByte(value);
	SPI_RF_CS_HIGH();   //CSN=1;
	return status;
}

/*************************************************/
/* �������ܣ���24L01�ļĴ���ֵ ��һ���ֽڣ�      */
/* ��ڲ�����reg  Ҫ���ļĴ�����ַ               */
/* ���ڲ�����value �����Ĵ�����ֵ                */
/*************************************************/
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
 	uint8_t value;

	SPI_RF_CS_LOW() ; //CSN=0;   
  SPI_RF_SendByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	value = SPI_RF_SendByte(NOP);
	SPI_RF_CS_HIGH();  //CSN=1;
	return value;
}

/*********************************************/
/* �������ܣ���24L01�ļĴ���ֵ������ֽڣ�   */
/* ��ڲ�����reg   �Ĵ�����ַ                */
/*           *pBuf �����Ĵ���ֵ�Ĵ������    */
/*           len   �����ֽڳ���              */
/* ���ڲ�����status ״ֵ̬                   */
/*********************************************/
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,u8_ctr;
	SPI_RF_CS_LOW() ;//CSN=0       
  status=SPI_RF_SendByte(reg);//���ͼĴ�����ַ,����ȡ״ֵ̬   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
	pBuf[u8_ctr]=SPI_RF_SendByte(0XFF);//��������
	SPI_RF_CS_HIGH(); //CSN=1
  return status;        //���ض�����״ֵ̬
}

/**********************************************/
/* �������ܣ���24L01�ļĴ���дֵ������ֽڣ�  */
/* ��ڲ�����reg  Ҫд�ļĴ�����ַ            */
/*           *pBuf ֵ�Ĵ������               */
/*           len   �����ֽڳ���               */
/**********************************************/
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,u8_ctr;
	SPI_RF_CS_LOW() ;	    
  status = SPI_RF_SendByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  for(u8_ctr=0; u8_ctr<len; u8_ctr++)
	SPI_RF_SendByte(*pBuf++); //д������
	SPI_RF_CS_HIGH(); 
  return status;          //���ض�����״ֵ̬
}

/********************************************/
/* �������ܣ����24L01�Ƿ����              */
/* ����ֵ��  0  ����                        */
/*           1  ������                      */
/********************************************/	
uint8_t NRF24L01_Check(void)
{
	uint8_t check_in_buf[5]={0x11,0x22,0x33,0x44,0x55};
	uint8_t check_out_buf[5]={0x00};

	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR, check_in_buf, 5);

	NRF24L01_Read_Buf(READ_REG+TX_ADDR, check_out_buf, 5);

	if((check_out_buf[0] == 0x11)&&\
	   (check_out_buf[1] == 0x22)&&\
	   (check_out_buf[2] == 0x33)&&\
	   (check_out_buf[3] == 0x44)&&\
	   (check_out_buf[4] == 0x55))return 0;
	else return 1;
}

/*********************************************/
/* �������ܣ�����24L01Ϊ����ģʽ             */
/*********************************************/
void NRF24L01_RX_Mode(void)
{

	SPI_RF_CE_LOW() ;	//CE���ͣ�ʹ��24L01����
	
	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0, (uint8_t*)RX_ADDRESS, RX_ADR_WIDTH);//дRX���յ�ַ
	  
  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x00);    //����ͨ��0�Զ�Ӧ��    
  	NRF24L01_Write_Reg(WRITE_REG+EN_RXADDR,0x01);//ͨ��0��������  	 
  	NRF24L01_Write_Reg(WRITE_REG+RF_CH,40);	     //����RF����ͨ��Ƶ�� 		  
  	NRF24L01_Write_Reg(WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��
	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x0f);//����TX�������,0db����,2Mbps,���������濪�� 	     
  	NRF24L01_Write_Reg(WRITE_REG+CONFIG, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ
	NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
	SPI_RF_CE_HIGH();	//CE�øߣ�ʹ�ܽ���
}

/*********************************************/
/* �������ܣ�����24L01Ϊ����ģʽ             */
/*********************************************/
void NRF24L01_TX_Mode(void)
{
		SPI_RF_CE_LOW() ;	//CE���ͣ�ʹ��24L01����	    
  	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  
  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x00);     //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
  	NRF24L01_Write_Reg(WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	NRF24L01_Write_Reg(WRITE_REG+RF_CH,40);       //����RFͨ��Ϊ40
  	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	  SPI_RF_CE_HIGH();	//CE�øߣ�ʹ�ܷ���
}

/*********************************************/
/* �������ܣ�24L01��������                   */
/* ��ڲ�����rxbuf ������������              */
/* ����ֵ�� 0   �ɹ��յ�����                 */
/*          1   û���յ�����                 */
/*********************************************/
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;

	state=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(WRITE_REG+STATUS,state); //���TX_DS��MAX_RT�жϱ�־
	if(state&RX_OK)//���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
		return 0; 
	}	   
	return 1;//û�յ��κ�����
}

/**********************************************/
/* �������ܣ�����24L01Ϊ����ģʽ              */
/* ��ڲ�����txbuf  ������������              */
/* ����ֵ�� 0x10    �ﵽ����ط�����������ʧ��*/
/*          0x20    �ɹ��������              */
/*          0xff    ����ʧ��                  */
/**********************************************/
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t state;
   
	SPI_RF_CE_LOW() ;	//CE���ͣ�ʹ��24L01����
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 	SPI_RF_CE_HIGH();	//CE�øߣ�ʹ�ܷ���
	while (SPI_RF_IRQ()!=0);//�ȴ�������� (note: this can be cahnged)
	state=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(WRITE_REG+STATUS,state); //���TX_DS��MAX_RT�жϱ�־
	NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
	if(state&MAX_TX)//�ﵽ����ط�����
	{
		
		return MAX_TX; 
	}
	if(state&TX_OK)//�������
	{
		return TX_OK;
	}
	return 0xff;//����ʧ��
}					    

/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
uint8_t SPI_RF_SendByte(uint8_t byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI1 peripheral */
  SPI_SendData8(SPI1, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_ReceiveData8(SPI1);
	
/*	uint16_t	 retry=0;	  
	while((SPI1->SR&1<<1)==0)	
	{ 
	retry++; 
	if(retry>=0XFFFE)return 0; 	
	}	   
	SPI1->DR=byte;	 	  	 
	retry=0; 
	while((SPI1->SR&1<<0)==0) 	  
	{ 
	retry++; 
	if(retry>=0XFFFE)return 0;	
	}	  	     
	return SPI1->DR ;   
	*/
}

void EXTI4_15_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	{	
		if(NRF24L01_RxPacket(Rx_Buffer)==0)	 //������յ�����
		{

		}
    EXTI_ClearITPendingBit(EXTI_Line11);
	}	
}

