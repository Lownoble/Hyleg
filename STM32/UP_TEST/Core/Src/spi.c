/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  else printf("\nSPI2 start success!!!\n");
  /* USER CODE END SPI2_Init 2 */

}
/* SPI3 init function */
void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */
  else printf("\nSPI3 start success!!!\n");
  /* USER CODE END SPI3_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI2 interrupt Init */
    HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* SPI3 clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

    /* SPI2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI2_IRQn);
  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN SPI3_MspDeInit 1 */
	SPI_CS(NRF2401,1);
	SPI_CS(MPU9250,1);
	SPI_CS(CS_FLASH,1);
	SPI_CS(MS5611,1);
	SPI_CS(CS_LIS,1);
  /* USER CODE END SPI3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

uint8_t Spi_RW(uint8_t Txdata)
{
//	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
//	SPI_I2S_SendData(SPI3, dat);
//	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);
//	return SPI_I2S_ReceiveData(SPI3);
	uint8_t Rxdata;
	if(HAL_SPI_TransmitReceive(&hspi3,&Txdata,&Rxdata,1,1000) == HAL_OK)
	{
		return Rxdata;
	}
	else
		printf("\nSPI receive failed!!!\n");
	return 9;
}


void SPI_Receive(uint8_t *pData, uint16_t Size)
{
    for(uint16_t i=0; i<Size; i++)
    {
        pData[i] = Spi_RW(0);
    }
}

void SPI_CS(uint8_t sel,uint8_t set)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,GPIO_PIN_SET);
	HAL_Delay(1);
switch(sel)
{
	case MPU9250:
		if(set)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);
		break;
	case NRF2401:
		if(set)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,GPIO_PIN_RESET);
		break;
	case MS5611:
		if(set)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
		break;
	case CS_FLASH:
		if(set)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);
		break;
	case CS_LIS:
		if(set)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_RESET);
		break;
}
}


//-----------------------------SPI通信--------------------------------

static __IO uint32_t  TimeOut = SPIT_LONG_TIMEOUT;
int spi_master_connect_pi=0;
int spi_master_loss_pi=0;
int spi_comm_mess_type=0;
unsigned char  spi_tx_buf[SPI_BUF_SIZE]={0};
unsigned char  spi_rx_buf[SPI_BUF_SIZE]={0};
int spi_tx_cnt=0;
int spi_rx_cnt=0;
int spi_rx_cnt_all=0;
char spi_flag_pi[2]={0,0};
float spi_dt[10]={0};


uint8_t SPI_Send_Byte(uint16_t data)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&data, &rx_data, 1, TimeOut);
    return rx_data;
}

static void setDataInt_spi(int i)
{
	spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
	spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}

static void setDataFloat_spi(float f)
{
	int i = *(int *)&f;
	spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
	spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}


static float floatFromData_spi(unsigned char *data,int* anal_cnt)
{
	int i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));

	*anal_cnt +=4;
	return *(float *)&i;
}

static float floatFromData_spi_int(unsigned char *data, int *anal_cnt,float size)
{
	float temp=0;
	temp=(float)((int16_t)(*(data + *anal_cnt + 0)<<8)|*(data + *anal_cnt + 1))/size;
	*anal_cnt += 2;
	return temp;
}

static char charFromData_spi(unsigned char *data,int* anal_cnt)
{
	int temp=*anal_cnt ;
	*anal_cnt +=1;
	return *(data+temp);
}

static int intFromData_spi(unsigned char *data,int* anal_cnt)
{
	int i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	*anal_cnt +=4;
	return i;
}

//for slave
void slave_send(char sel)
{
	int i;
	char sum_t=0;
	spi_tx_cnt=0;

	spi_tx_buf[spi_tx_cnt++]=0xFF;
	spi_tx_buf[spi_tx_cnt++]=0xFB;
	spi_tx_buf[spi_tx_cnt++]=sel;
	spi_tx_buf[spi_tx_cnt++]=0;

	setDataFloat_spi(1.23);
	setDataFloat_spi(-1.23);

	spi_tx_buf[3] =(spi_tx_cnt)-4;
	for( i=0;i<spi_tx_cnt;i++)
		sum_t += spi_tx_buf[i];
	spi_tx_buf[spi_tx_cnt++] = sum_t;

}

float test_spi_rx[2]={0};
int sum_spi_err=0;
int temp_sel[4][128]={0};
void slave_rx(uint8_t *data_buf,uint8_t num)//---------------------------从控制器接收
{

	uint8_t sum = 0;
	uint8_t i;
	int anal_cnt=4;

	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1))){
		sum_spi_err++;
		i=0;
		return;}		//判断sum
	if(!(*(data_buf)==0xFE && *(data_buf+1)==0xFC))		return;		//判断帧头


	else if (*(data_buf+2)==99)//
	{

		test_spi_rx[0]=floatFromData_spi(spi_rx_buf,&anal_cnt);
		test_spi_rx[1]=floatFromData_spi(spi_rx_buf,&anal_cnt);
	}

}

uint8_t spi_tx[TX_SIZE] = {	0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
							0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C,
							0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12,
							0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
							0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E,
							0x1F, 0x20};
uint32_t tx_idx = 0;
uint8_t spi_rx[RX_SIZE];
uint32_t rx_idx = 0;







void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)//中断回调函数
{
	if(hspi->Instance == SPI2)
	{
		if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE) != RESET)
		{
			HAL_SPI_Transmit_IT(hspi, spi_tx, 1);
		}

		// 从SPI读取数据
		if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE) != RESET)
		{
			HAL_SPI_Receive_IT(hspi, spi_rx, 1);
		// 收完32个字节后将收到的内容通过串口发�?�出�?
			DmaPrintf("%x",spi_rx);
		}
	}
}

/* USER CODE END 1 */