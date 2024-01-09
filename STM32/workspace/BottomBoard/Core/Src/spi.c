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
extern uint32_t sys_time;
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
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
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
  else DmaPrintf("SPI2 start success!!!\n");

//    HAL_SPI_Receive_IT(&hspi2, spi_rx_temp,sizeof(spi_rx_temp));
  	HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
  	HAL_NVIC_EnableIRQ(SPI2_IRQn);
  	__HAL_SPI_ENABLE(&hspi2);             //使能SPI
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
  else printf("SPI3 start success!!!\n");
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
    HAL_NVIC_SetPriority(SPI2_IRQn, 6, 0);
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

  /* USER CODE END SPI3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
//-----------------------------SPI通信--------------------------------
int SPI2_RX_FLAG = 0;
static __IO uint32_t  TimeOut = SPIT_LONG_TIMEOUT;
int spi_master_connect_pi=0;
int spi_master_loss_pi=0;
int spi_comm_mess_type=0;
unsigned char  spi_tx_buf[SPI_BUF_SIZE]={0};
unsigned char  spi_rx_buf[SPI_BUF_SIZE]={0};
uint8_t spi_tx_temp[1] = {0};
uint8_t spi_rx_temp[1] = {0};
int spi_tx_cnt=0;
int spi_rx_cnt=0;
int spi_rx_cnt_all=0;
int spi_flag_pi[2]={0,0};
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

//将四字节数据转换为浮点数
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

//将二字节数据转换为浮点数
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
//---------------------------发�??---------------------------
void slave_tx(int sel)
{
	char sum_t=0;
	spi_tx_cnt=0;

	spi_tx_buf[spi_tx_cnt++]=0xFF;
	spi_tx_buf[spi_tx_cnt++]=0xFB;
	spi_tx_buf[spi_tx_cnt++]=sel;
	spi_tx_buf[spi_tx_cnt++]=0;

	switch(sel)
	{
	case 0://发�?�电�?????????????????0 ID,角度，�?��?�，电流
		setDataInt_spi(motor[0].ID);
		setDataFloat_spi(motor[0].position);
		setDataFloat_spi(motor[0].velocity);
		setDataFloat_spi(motor[0].current);
		break;
	case 1:
	case 2:
	case 3:
	case 4:
		setDataInt_spi(motorState[sel].ID);
		setDataFloat_spi(motorState[sel].q);
		setDataFloat_spi(motorState[sel].dq);
		setDataFloat_spi(motorState[sel].ddq);
		setDataFloat_spi(motorState[sel].current);
		setDataFloat_spi(motorState[sel].none);
		break;
	case 5:

		break;
	case 10:
		for(int i=1; i<5; i++){
			setDataInt_spi(motorState[i].ID);
			setDataFloat_spi(motorState[i].q);
			setDataFloat_spi(motorState[i].dq);
			setDataFloat_spi(motorState[i].ddq);
			setDataFloat_spi(motorState[i].current);
			setDataFloat_spi(motorState[i].none);
		}
		break;

	case 98://FF FB 62 08 0F 01 00 00 9C 00 00 00 10 00 00 00
		setDataInt_spi(271);
		setDataInt_spi(156);
		break;

	default:
		for(int id=0;id<4;id++)
		{
			setDataFloat_spi(0);
			setDataFloat_spi(0);
			setDataFloat_spi(0);
			setDataFloat_spi(0);
		}
		break;
	}

	spi_tx_buf[3] =(spi_tx_cnt)-4;
	for(int i=0;i<spi_tx_cnt;i++)
		sum_t += spi_tx_buf[i];
	spi_tx_buf[spi_tx_cnt++] = sum_t;
	spi_tx_buf[spi_tx_cnt++] = 0;
	spi_tx_buf[spi_tx_cnt++] = 0;
	spi_tx_buf[spi_tx_cnt++] = 0;

	//spi_rx_cnt_all++;
}

//---------------------------从控制器接收---------------------------
void slave_rx(uint8_t *data_buf, uint8_t num)
{
	uint8_t ID;
	uint8_t sum = 0;
	uint8_t i;
	uint8_t sel = *(data_buf + 2);
	int anal_cnt=4;

	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))//判断sum
	{
		return;
	}
	if(!(*(data_buf)==0xFA && *(data_buf+1)==0xFF))		return;		//判断帧头

	switch (sel){
	case 0:
		motorCmd[0].ID = intFromData_spi(data_buf, &anal_cnt);
		motorCmd[0].q = floatFromData_spi(data_buf, &anal_cnt);
		motorCmd[0].dq = floatFromData_spi(data_buf, &anal_cnt);
		motorCmd[0].tau = floatFromData_spi(data_buf, &anal_cnt);
		break;
	case 1:
	case 2:
	case 3:
	case 4:
		ID = intFromData_spi(data_buf, &anal_cnt);
		motorCmd[ID].ID = ID;
		motorCmd[ID].q = floatFromData_spi(data_buf, &anal_cnt);
		motorCmd[ID].dq = floatFromData_spi(data_buf, &anal_cnt);
		motorCmd[ID].tau = floatFromData_spi(data_buf, &anal_cnt);
		motorCmd[ID].Kp = floatFromData_spi(data_buf, &anal_cnt);
		motorCmd[ID].Kd = floatFromData_spi(data_buf, &anal_cnt);
		break;
	case 10:
		for(int i=1;i<5;i++){
			ID = intFromData_spi(data_buf, &anal_cnt);
			motorCmd[ID].ID = ID;
			motorCmd[ID].q = floatFromData_spi(data_buf, &anal_cnt);
			motorCmd[ID].dq = floatFromData_spi(data_buf, &anal_cnt);
			motorCmd[ID].tau = floatFromData_spi(data_buf, &anal_cnt);
			motorCmd[ID].Kp = floatFromData_spi(data_buf, &anal_cnt);
			motorCmd[ID].Kd = floatFromData_spi(data_buf, &anal_cnt);
		}
		break;
	}
}

void spi_recieve(){
	static char state=0;
	static int spi_rx_cnt=0;
	static uint8_t _data_len2 = 0,_data_cnt2 = 0;
	uint8_t data;

	for(int i=0;i<spi_tx_cnt;i++){
		data = spi_rx_buf[i];

		if(state==0&&data==0xFA)
		{
			state=1;
			spi_rx[0]=data;
		}
		else if(state==1&&data==0xFF)
		{
			state=2;
			spi_rx[1]=data;
		}
		else if(state==2&&data>=0&&data<0XF1)
		{
			state=3;
			spi_rx[2]=data;
		}
		else if(state==3&&data<SPI_BUF_SIZE)
		{
			state = 4;
			spi_rx[3]=data;
			_data_len2 = data;
			_data_cnt2 = 0;
		}
		else if(state==4&&_data_len2>0)
		{
			_data_len2--;
			spi_rx[4+_data_cnt2++]=data;
			if(_data_len2==0)
				state= 5;
		}
		else if(state==5)
		{
			state = 0;
			spi_rx[4+_data_cnt2]=data;
			spi_rx_cnt=5+_data_cnt2;
			slave_rx(spi_rx,spi_rx_cnt);
			spi_rx_cnt_all++;
		}
		else  state = 0;
	}
}

uint8_t spi_tx[TX_SIZE] = {	0xFD, 0xFB, 0x01, 0x04, 0x05, 0x06,
							0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C,
							0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12,
							0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
							0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E,
							0x1F, 0x20};
uint8_t spi_rx[RX_SIZE] = {0};

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)//中断回调函数
{
	if(hspi->Instance == SPI2)
	{

		char tm_buf[100];
		int tmp_num = 0;


		static int spi_sel = 0;
		spi_sel++;
		slave_tx(spi_sel);
		if(spi_sel==4) spi_sel=0;

		SPI2_RX_FLAG = 1;

		for(int i=0;i<spi_tx_cnt;i++){
			tmp_num = 0;
			tmp_num  = sprintf(tm_buf,"%02x ",spi_rx_buf[i]);
			spi_rx_buf[i] = 0;
			HAL_UART_Transmit(&huart1, tm_buf, tmp_num, 0xff);
		}

		tmp_num  = sprintf(tm_buf,"%.4f \n",sys_time/2000.0);
		HAL_UART_Transmit(&huart1, tm_buf, tmp_num, 0xff);

//		HAL_SPI_Transmit(&hspi2, spi_tx_buf,  spi_tx_cnt,10);
//		HAL_SPI_Receive_IT(&hspi2,spi_rx_buf, spi_tx_cnt);

		HAL_SPI_TransmitReceive_IT(&hspi2, spi_tx_buf, spi_rx_buf, spi_tx_cnt);
	}
}

void SPI_Task(int enable){
	if(enable){
		HAL_StatusTypeDef status;
		static int running = 0;
		int spi_sel = 10;
		slave_tx(spi_sel);
		status = HAL_SPI_TransmitReceive(&hspi2, spi_tx_buf, spi_rx_buf, spi_tx_cnt,0XFF);
		if (status == HAL_OK) {
			spi_recieve();
			motor_setCmd();
			running = 1;
		}
		if(status != HAL_OK && !running){
			state_passive();
		}
	}
}

/* USER CODE END 1 */
