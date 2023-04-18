/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId LEDHandle;
osThreadId CANHandle;
osThreadId UARTHandle;
osThreadId GAITHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void LED_Task(void const * argument);
void CAN_Task(void const * argument);
void UART_Task(void const * argument);
void GAIT_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LED */
  osThreadDef(LED, LED_Task, osPriorityIdle, 0, 128);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

  /* definition and creation of CAN */
  osThreadDef(CAN, CAN_Task, osPriorityIdle, 0, 128);
  CANHandle = osThreadCreate(osThread(CAN), NULL);

  /* definition and creation of UART */
  osThreadDef(UART, UART_Task, osPriorityIdle, 0, 256);
  UARTHandle = osThreadCreate(osThread(UART), NULL);

  /* definition and creation of GAIT */
  osThreadDef(GAIT, GAIT_Task, osPriorityIdle, 0, 256);
  GAITHandle = osThreadCreate(osThread(GAIT), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LED_Task */
/**
* @brief Function implementing the LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_Task */
void LED_Task(void const * argument)
{
  /* USER CODE BEGIN LED_Task */

	int led = 1;
	int flag = 0;
  /* Infinite loop */
  for(;;)
  {
	  if(led){
		flag=!flag;
		LEDRGB_RED(flag);
		LEDRGB_BLUE(!flag);
		Cylinder(flag);
	  }
	  osDelay(5000);
  }
  /* USER CODE END LED_Task */
}

/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the CAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
void CAN_Task(void const * argument)
{
  /* USER CODE BEGIN CAN_Task */

  /* Infinite loop */
  for(;;)
  {
	  motor_control();
	  osDelay(1);
  }
  /* USER CODE END CAN_Task */
}

/* USER CODE BEGIN Header_UART_Task */
/**
* @brief Function implementing the UART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_Task */
void UART_Task(void const * argument)
{
  /* USER CODE BEGIN UART_Task */
  /* Infinite loop */
  for(;;)
  {
	  if(CAN_FLAG){
//		  HAL_Delay(100);
//		  DmaPrintf("Motor:%d position:%.2f velocity:%.2f current:%.2f\n",motor[1].ID,motor[1].position,motor[1].velocity,motor[1].current);
//		  HAL_Delay(100);
//		  DmaPrintf("Motor:%d position:%.2f velocity:%.2f current:%.2f\n",motor[2].ID,motor[2].position,motor[2].velocity,motor[2].current);
//		  HAL_Delay(100);
//		  DmaPrintf("Motor:%d position:%.2f velocity:%.2f current:%.2f\n",motor[3].ID,motor[3].position,motor[3].velocity,motor[3].current);
//		  HAL_Delay(100);
//		  DmaPrintf("Motor:%d position:%.2f velocity:%.2f current:%.2f\n\n\n",motor[4].ID,motor[4].position,motor[4].velocity,motor[4].current);
//		  HAL_Delay(2);
//		  DmaPrintf("%d %.2f %.2f %.2f\n",motor[1].ID,motor[1].position,motor[1].velocity,motor[1].current);
//		  HAL_Delay(2);
//		  DmaPrintf("%d %.2f %.2f %.2f\n",motor[1].ID,motor[1].position,motor[1].velocity,motor[1].current);
//		  HAL_Delay(2);
//		  DmaPrintf("%d %.2f %.2f %.2f\n",motor[1].ID,motor[1].position,motor[1].velocity,motor[1].current);
//		  HAL_Delay(2);
//		  DmaPrintf("%d %.2f %.2f %.2f\n",motor[1].ID,motor[1].position,motor[1].velocity,motor[1].current);
		  CAN_FLAG = 0;
	  }

		if( USART1_RX_FLAG){
			HAL_Delay(100);
			if(USART1_RX_BUF[0]=='E'){
				motor_enable_all();
			}
			if(USART1_RX_BUF[0]=='D'){
				motor_disable(1);motor_disable(2);motor_disable(3);motor_disable(4);
			}
			if(USART1_RX_BUF[0]=='Z'){
				motor_setzero(1);motor_setzero(2);motor_setzero(3);motor_setzero(4);
			}
			if(USART1_RX_BUF[0]=='P'){
				int position = (USART1_RX_BUF[1] - '0')*10 + (USART1_RX_BUF[2] - '0');
				DmaPrintf("position:%d",position);
				pack_TX(2, (float)position/180*3.14, 0, 5, 1, 0);
			}
			if(USART1_RX_BUF[0]=='T'){
				int count = (USART1_RX_BUF[1] - '0')*10 + (USART1_RX_BUF[2] - '0');
				pack_TX(1, stand_trajectory[count][0], 0, 5, 1, 0);
				pack_TX(2, stand_trajectory[count][0], 0, 5, 1, 0);
			}

			for( int i = 0; i<USART1_RX_CNT; i ++){
				USART1_RX_BUF[i] = 0;
			}
			USART1_RX_CNT = 0;
			USART1_RX_FLAG = 0;
		}

	  osDelay(2);
  }
  /* USER CODE END UART_Task */
}

/* USER CODE BEGIN Header_GAIT_Task */
/**
* @brief Function implementing the GAIT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GAIT_Task */
void GAIT_Task(void const * argument)
{
  /* USER CODE BEGIN GAIT_Task */
  /* Infinite loop */
  for(;;)
  {
//	  int aaa = motor_setdes(motor[1], test_trajectory[test_cnt]);
//	  if(aaa == 1)		DmaPrintf("position limits\n");
//	  if(aaa == 2)		DmaPrintf("velocity limits\n");
//	  HAL_Delay(10);
//	  test_cnt ++;
//	  DmaPrintf("p_des:%f test_trajectory:%f\n",motor[1].p_des,test_trajectory[test_cnt]);
//
//	  if(test_cnt == 10){
//	  		test_cnt = 0;
//	  }

	  if(Leg_left.stand_flag){
		  motor[1].limit_flag = motor_setdes(motor[1], stand_trajectory[Leg_left.count][0]);
		  motor[2].limit_flag = motor_setdes(motor[2], stand_trajectory[Leg_left.count][1]);
		  Leg_left.count ++;
		  if(Leg_left.count == 100){
			  Leg_left.count = 0;
			  Leg_left.stand_flag = 0;
			  Leg_left.swing_flag = 1;
		  }
	  }
	  if(Leg_left.swing_flag){
		  motor[1].limit_flag = motor_setdes(motor[1], swing_trajectory[Leg_left.count][0]);
		  motor[2].limit_flag = motor_setdes(motor[2], swing_trajectory[Leg_left.count][1]);
		  Leg_left.count ++;
		  if(Leg_left.count == 100){
			  Leg_left.count = 0;
			  Leg_left.stand_flag = 1;
			  Leg_left.swing_flag = 0;
		  }
	  }
	  if(motor[1].limit_flag == 1 || motor[2].limit_flag == 1 )		DmaPrintf("position limits\n");
	  if(motor[1].limit_flag == 2 || motor[2].limit_flag == 2 )		DmaPrintf("velocity limits\n");

	  //pack_TX(motor[2].ID, motor[2].p_des, motor[2].v_des, motor[2].kp, motor[2].kd, motor[2].t_ff);
      osDelay(10);
  }
  /* USER CODE END GAIT_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
