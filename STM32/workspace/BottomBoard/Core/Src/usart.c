/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
uint8_t USART1_RX_BUF[200] = {0};
uint8_t USART1_RX_FLAG = 0;
uint32_t USART1_RX_CNT = 0;
uint8_t USART1_RX_TEMP[1]={0};
uint8_t USART1_TX_BUF[1200] = {0};
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
//int fputc(int ch, FILE *f)
//{
//    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
//    return ch;
//}
//
//int fgetc(FILE *f)
//{
//    uint8_t ch = 0;
//    HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
//    return ch;
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //中断回调函数
{
	if(huart == &huart1)
	{
		USART1_RX_BUF[USART1_RX_CNT] = USART1_RX_TEMP[0];
		USART1_RX_CNT++;
		if( 0x0a == USART1_RX_TEMP[0]){
			USART1_RX_FLAG = 1;
			//DmaPrintf("11111");
		}
		if(USART1_RX_BUF[0] =='R'){
			HAL_NVIC_SystemReset();
		}
		HAL_UART_Receive_IT(&huart1, USART1_RX_TEMP, sizeof(USART1_RX_TEMP));
	}

}

void UART_MyTask(int data)
{
	if(data){
		//motorState
//		HAL_Delay(1);
//		DmaPrintf("%d,%.2f,%.2f,%.2f,",motorState[0].ID,motorState[0].q,motorState[0].dq,motorState[0].current);
//		HAL_Delay(1);
//		DmaPrintf("%d,%.2f,%.2f,%.2f,",motorState[1].ID,motorState[1].q,motorState[1].dq,motorState[1].current);
//		HAL_Delay(1);
//		DmaPrintf("%d,%.2f,%.2f,%.2f,",motorState[2].ID,motorState[2].q,motorState[2].dq,motorState[2].current);
//		HAL_Delay(1);
//		DmaPrintf("%d,%.2f,%.2f,%.2f,",motorState[3].ID,motorState[3].q,motorState[3].dq,motorState[3].current);
//		HAL_Delay(1);
//		DmaPrintf("%d,%.2f,%.2f,%.2f,",motorState[4].ID,motorState[4].q,motorState[4].dq,motorState[4].current);
//		CAN_FLAG = 0;

		//motorCmd
//		HAL_Delay(1);
//		DmaPrintf("%d,%.2f,%.2f,%.2f,",motorCmd[1].ID,motorCmd[1].q,motorCmd[1].dq,motorCmd[1].tau);
//		HAL_Delay(1);
//		DmaPrintf("%d,%.2f,%.2f,%.2f,",motorCmd[2].ID,motorCmd[2].q,motorCmd[2].dq,motorCmd[2].tau);
//		HAL_Delay(1);
//		DmaPrintf("%d,%.2f,%.2f,%.2f,",motorCmd[3].ID,motorCmd[3].q,motorCmd[3].dq,motorCmd[3].tau);
//		HAL_Delay(1);
//		DmaPrintf("%d,%.2f,%.2f,%.2f,",motorCmd[4].ID,motorCmd[4].q,motorCmd[4].dq,motorCmd[4].tau);

		//footPosition
		float* Position = FK(motorCmd[1].q,motorCmd[2].q);
		float Pd_lx = Position[0];	float Pd_lz = Position[1];
		Position = FK(motorState[1].q,motorState[2].q);
		float Pf_lx = Position[0];	float Pf_lz = Position[1];
		Position = FK(motorCmd[3].q,motorCmd[4].q);
		float Pd_rx = Position[0];	float Pd_rz = Position[1];
		Position = FK(motorState[3].q,motorState[4].q);
		float Pf_rx = Position[0];	float Pf_rz = Position[1];

		//footVelocity
//		float Velocity[2];
//		calVelocity(motorCmd[1].q,motorCmd[2].q,motorCmd[1].dq,motorCmd[2].dq,Velocity);
//		float Vd_lx = Velocity[0];	float Vd_lz = Velocity[1];
//		calVelocity(motorState[1].q,motorState[2].q,motorState[1].dq,motorState[2].dq,Velocity);
//		float Vf_lx = Velocity[0];	float Vf_lz = Velocity[1];
//		calVelocity(motorCmd[3].q,motorCmd[4].q,motorCmd[3].dq,motorCmd[4].dq,Velocity);
//		float Vd_rx = Velocity[0];	float Vd_rz = Velocity[1];
//		calVelocity(motorState[3].q,motorState[4].q,motorState[3].dq,motorState[4].dq,Velocity);
//		float Vf_rx = Velocity[0];	float Vf_rz = Velocity[1];

		DmaPrintf("%d,%.3f,%.2f,%.2f,%d,%.3f,%.2f,%.2f,%d,%.3f,%.2f,%.2f,%d,%.3f,%.2f,%.2f,%d,%.3f,%.2f,%.2f,%d,%.4f,%.2f,%.2f,%d,%.4f,%.2f,%.2f,%d,%.4f,%.2f,%.2f,%d,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
				motorState[0].ID,motorState[0].q,motorState[0].dq,motorState[0].current,\
				motorState[1].ID,motorState[1].q,motorState[1].dq,motorState[1].current,\
				motorState[2].ID,motorState[2].q,motorState[2].dq,motorState[2].current,\
				motorState[3].ID,motorState[3].q,motorState[3].dq,motorState[3].current,\
				motorState[4].ID,motorState[4].q,motorState[4].dq,motorState[4].current,\
				motorCmd[1].ID,motorCmd[1].q,motorCmd[1].dq,motorCmd[1].tau,\
				motorCmd[2].ID,motorCmd[2].q,motorCmd[2].dq,motorCmd[2].tau,\
				motorCmd[3].ID,motorCmd[3].q,motorCmd[3].dq,motorCmd[3].tau,\
				motorCmd[4].ID,motorCmd[4].q,motorCmd[4].dq,motorCmd[4].tau,motorCmd[4].Kp,motorCmd[4].Kd,\
				Pd_lx,Pd_lz,Pf_lx,Pf_lz,Pd_rx,Pd_rz,Pf_rx,Pf_rz);
	}

	static int spi_FLAG = 0;
	if(spi_FLAG){
		HAL_Delay(1);
		DmaPrintf("RX:  ");
		HAL_Delay(1);
		for(int i=0;i<spi_tx_cnt;i++){
			DmaPrintf("%02x  ",spi_rx_buf[i]);
			HAL_Delay(1);
		}
		DmaPrintf("\n");
	}

	if(USART1_RX_FLAG){
		if(USART1_RX_BUF[0]=='E'){
			motor_enable(0);motor_enable(1);motor_enable(2);motor_enable(3);motor_enable(4);
		}
		if(USART1_RX_BUF[0]=='D'){
			motor_disable(0);motor_disable(1);motor_disable(2);motor_disable(3);motor_disable(4);
		}
		if(USART1_RX_BUF[0]=='Z'){
			motor_setzero(1);motor_setzero(2);motor_setzero(3);motor_setzero(4);
		}
		if(USART1_RX_BUF[0]=='H'){
			DmaPrintf("hello world!\n");
		}
		if(USART1_RX_BUF[0]=='P'){
			state_passive();
		}
		if(USART1_RX_BUF[0]=='S'){
			spi_FLAG = !spi_FLAG;
		}
		if(USART1_RX_BUF[0]=='+'){
			motor[0].t_ff += 1;
			motor_settau(0, motor[0].t_ff);
			//DmaPrintf("%f\n",motor[0].t_ff);
		}
		if(USART1_RX_BUF[0]=='-'){
			motor[0].t_ff -= 1;
			motor_settau(0, motor[0].t_ff);
			//DmaPrintf("%f\n",motor[0].t_ff);
		}

		for( int i = 0; i<USART1_RX_CNT; i ++){
			USART1_RX_BUF[i] = 0;
		}
		USART1_RX_CNT = 0;
		USART1_RX_FLAG = 0;

	}

}


/* USER CODE END 1 */
