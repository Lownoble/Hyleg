/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef 	CAN1_TX;
CAN_RxHeaderTypeDef 	CAN1_RX;
CAN_TxHeaderTypeDef 	CAN2_TX;
CAN_RxHeaderTypeDef 	CAN2_RX;

uint8_t CAN1_Rx_data[6];
uint8_t CAN1_Tx_data[8];
uint8_t CAN2_Rx_data[6];
uint8_t CAN2_Tx_data[8];

uint8_t CAN_FLAG = 0;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
	  HAL_StatusTypeDef 		HAL_Status;
	  CAN_FilterTypeDef 		CAN1_Filter; //CAN1滤波
	  uint32_t StdId = 0x01;
	  uint32_t ExtId = 0x00;
	  uint32_t DLC = 8;
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  //Tx_Init
  CAN1_TX.StdId  			 			 = 	StdId;   		 //11�????????????     标准标识�????????????
  CAN1_TX.ExtId    					  	 = 	ExtId;   		 //29�????????????     扩展标识�????????????
  CAN1_TX.IDE      						 = 	CAN_ID_STD;  	//1�????????????        0:标准�???????????? 1:拓展�????????????
  CAN1_TX.RTR      			  			 = 	CAN_RTR_DATA; 	 //1�????????????   0:数据�???????????? 1:远程�????????????
  CAN1_TX.DLC      				  		 = 	DLC;     	 	//4�????????????   发�?�的数据的长�????????????
  CAN1_TX.TransmitGlobalTime    		 =  ENABLE;

  //Filter_Init
  CAN1_Filter.FilterIdHigh               = 	0;
  CAN1_Filter.FilterIdLow                = 	0;
  CAN1_Filter.FilterMaskIdHigh        	 =  0;
  CAN1_Filter.FilterMaskIdLow            =  0;
  CAN1_Filter.FilterFIFOAssignment		 = 	CAN_FILTER_FIFO0;
  CAN1_Filter.FilterBank                 =	0;
  CAN1_Filter.FilterMode                 =	CAN_FILTERMODE_IDMASK;
  CAN1_Filter.FilterScale                = 	CAN_FILTERSCALE_32BIT;
  CAN1_Filter.FilterActivation         	 =	ENABLE;
  CAN1_Filter.SlaveStartFilterBank 		 =	0;

  HAL_CAN_ConfigFilter(&hcan1,&CAN1_Filter);
  HAL_Status = HAL_CAN_Start(&hcan1);
  if(HAL_Status != HAL_OK){
	  DmaPrintf("\n\rCAN1 start Failed!\n\r");
  }
  else	DmaPrintf("CAN1 Start Success!!\n");
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */
	HAL_StatusTypeDef 		HAL_Status;
	CAN_FilterTypeDef 		CAN2_Filter; //CAN2滤波
	uint32_t StdId = 0x03;
	uint32_t ExtId = 0x00;
	uint32_t DLC = 8;
  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 2;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  //Tx_Init
    CAN2_TX.StdId  			 			 = 	StdId;   		 //11�????????????     标准标识�????????????
    CAN2_TX.ExtId    					  	 = 	ExtId;   		 //29�????????????     扩展标识�????????????
    CAN2_TX.IDE      						 = 	CAN_ID_STD;  	//1�????????????        0:标准�???????????? 1:拓展�????????????
    CAN2_TX.RTR      			  			 = 	CAN_RTR_DATA; 	 //1�????????????   0:数据�???????????? 1:远程�????????????
    CAN2_TX.DLC      				  		 = 	DLC;     	 	//4�????????????   发�?�的数据的长�????????????
    CAN2_TX.TransmitGlobalTime    		 =  ENABLE;

    //Filter_Init
    CAN2_Filter.FilterIdHigh               = 	0;
    CAN2_Filter.FilterIdLow                = 	0;
    CAN2_Filter.FilterMaskIdHigh        	 =  0;
    CAN2_Filter.FilterMaskIdLow            =  0;
    CAN2_Filter.FilterFIFOAssignment		 = 	CAN_FILTER_FIFO0;
    CAN2_Filter.FilterBank                 =	0;
    CAN2_Filter.FilterMode                 =	CAN_FILTERMODE_IDMASK;
    CAN2_Filter.FilterScale                = 	CAN_FILTERSCALE_32BIT;
    CAN2_Filter.FilterActivation         	 =	ENABLE;
    CAN2_Filter.SlaveStartFilterBank 		 =	0;

    HAL_CAN_ConfigFilter(&hcan2,&CAN2_Filter);
    HAL_Status = HAL_CAN_Start(&hcan2);
    if(HAL_Status != HAL_OK){
  	  DmaPrintf("\n\rCAN2 start Failed!\n\r");
    }
    else	DmaPrintf("CAN2 Start Success!!\n");
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
//CAN接收函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == &hcan1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_RX, CAN1_Rx_data);
		CAN_FLAG = 1;
//		printf("\nCAN_RX:");
//		for(int i=0; i<6; i++){
//			printf(" %x ", CAN1_Rx_data[i]);
//		}
		motor_setdata(CAN1_Rx_data);
		switch (CAN1_RX.StdId)
		{
//			case 1: motor1 = unpack_RX(CAN1_Rx_data); break;
//			case 2: motor2 = unpack_RX(CAN1_Rx_data); break;
			default:
			{
				break;
			}
		}

		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	}

	else if(hcan == &hcan2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN2_RX, CAN2_Rx_data);
		CAN_FLAG = 1;
		if(CAN2_RX.StdId == 3)	DmaPrintf("3!!!!!!\n");
		motor_setdata(CAN2_Rx_data);
		switch (CAN2_RX.StdId)
		{

			default:
			{
				break;
			}
		}
		__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
}

//CAN发�?�函�??
int CAN_TxMeg( uint8_t ID, uint8_t *pData, uint16_t Len)
{
	HAL_StatusTypeDef 	HAL_RetVal;
	uint32_t	Tx_MailBox;
	switch(ID)
	{
		case 0 :
		case 1 :
		case 2 :
			CAN1_TX.StdId = ID;
			CAN1_TX.DLC = Len;
			HAL_RetVal = HAL_CAN_AddTxMessage(&hcan1, &CAN1_TX, pData, &Tx_MailBox);
			break;

		case 3 :
			CAN2_TX.StdId = ID;
			CAN2_TX.DLC = Len;
			HAL_RetVal = HAL_CAN_AddTxMessage(&hcan2, &CAN2_TX, pData, &Tx_MailBox);
			break;
		case 4 :
			CAN2_TX.StdId = 2*ID;
			CAN2_TX.DLC = Len;
			HAL_RetVal = HAL_CAN_AddTxMessage(&hcan2, &CAN2_TX, pData, &Tx_MailBox);
			break;

	}
	if(HAL_OK != HAL_RetVal){
		printf("\n\rCAN TxMeg Failed!!\n\r");
		return 0;
	}
	else{
//		printf("\nSend Tx Message Success!!");
		return 1;
	}
}
/* USER CODE END 1 */
