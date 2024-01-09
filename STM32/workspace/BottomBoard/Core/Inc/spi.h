/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi2;

extern SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN Private defines */
#define TX_SIZE 256
#define RX_SIZE 256
#define SPI_BUF_SIZE 254  //偶数
#define SPIT_FLAG_TIMEOUT              ((uint32_t)0x1000)
#define SPIT_LONG_TIMEOUT              ((uint32_t)(10 * SPIT_FLAG_TIMEOUT))
/* USER CODE END Private defines */

void MX_SPI2_Init(void);
void MX_SPI3_Init(void);

/* USER CODE BEGIN Prototypes */

extern int SPI2_RX_FLAG;

extern uint8_t spi_tx_temp[1];
extern uint8_t spi_rx_temp[1];
extern int spi_rx_cnt_all;
extern int spi_tx_cnt;
extern int spi_rx_cnt;
extern unsigned char  spi_rx_buf[SPI_BUF_SIZE];
extern unsigned char  spi_tx_buf[SPI_BUF_SIZE];
extern uint8_t spi_tx[TX_SIZE];
extern uint8_t spi_rx[RX_SIZE];
extern int spi_sel;
void slave_tx(int sel);
void slave_rx(uint8_t *data_buf,uint8_t num);
void SPI_Communicate();
void spi_recieve();
void SPI_Task(int enable);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
