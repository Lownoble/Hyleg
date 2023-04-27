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
#define SPI_PI_IS_MASTER 0//SPI主发�?????

void SPI3_Init(void);			 //初始化SPI1�?????
void SPI2_Init(void);			 //初始化SPI1�?????
uint8_t Spi_RW(uint8_t TxData);//SPI1总线读写�?????个字�?????

#define MPU9250 0
#define CS_ACCGRO 0
#define NRF2401 1
#define MS5611  2
#define CS_FLASH 3
#define CS_LIS 4
void SPI_CS(uint8_t sel,uint8_t set);
void SPI_Receive(uint8_t *pData, uint16_t Size);
void SPI_SetSpeed(uint8_t SPI_BaudRatePrescaler);
#define SPI_CE_H()   GPIO_SetBits(GPIOC, GPIO_Pin_2)
#define SPI_CE_L()   GPIO_ResetBits(GPIOC, GPIO_Pin_2)

#define SPI_CSN_H()  SPI_CS(NRF2401,1)//GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define SPI_CSN_L()  SPI_CS(NRF2401,0)//GPIO_ResetBits(GPIOB, GPIO_Pin_12)
void master_send(void);
extern float test_spi_rx[2];
extern int spi_master_loss_pi;
extern int spi_master_connect_pi;
#define SPI_BUF_SIZE 254  //偶数
#define SPIT_FLAG_TIMEOUT              ((uint32_t)0x1000)
#define SPIT_LONG_TIMEOUT              ((uint32_t)(10 * SPIT_FLAG_TIMEOUT))
extern unsigned char  spi_tx_buf[SPI_BUF_SIZE];
extern unsigned char  spi_rx_buf[SPI_BUF_SIZE];
extern int spi_tx_cnt;
extern int spi_rx_cnt;
extern int spi_rx_cnt_all;
void slave_rx(uint8_t *data_buf,uint8_t num);
extern char spi_flag_pi[2];
extern float spi_dt[10];
void slave_send(char sel);


#define TX_SIZE 32
#define RX_SIZE 32

/* USER CODE END Private defines */

void MX_SPI2_Init(void);
void MX_SPI3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

