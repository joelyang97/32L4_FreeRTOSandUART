/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
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
osMessageQueueId_t osQueueUart1;
osMessageQueueId_t osQueueUart2;
osMessageQueueId_t osQueueUart4;

osThreadId_t uart1TaskHandle;
osThreadId_t uart2TaskHandle;
osThreadId_t uart4TaskHandle;
/* USER CODE END Variables */
osThreadId_t defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void uart1RxTask(void *argument);
void uart2RxTask(void *argument);
void uart4RxTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */
	osKernelInitialize();

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
	osQueueUart1 = osMessageQueueNew(UARTQUEUENUM, sizeof(UARTMessageQueue_TypeDef), NULL);
	osQueueUart2 = osMessageQueueNew(UARTQUEUENUM, sizeof(UARTMessageQueue_TypeDef), NULL);
	osQueueUart4 = osMessageQueueNew(UARTQUEUENUM, sizeof(UARTMessageQueue_TypeDef), NULL);
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	//const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
	//		.priority = (osPriority_t) osPriorityNormal, .stack_size = 128 };
	//defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
	//		&defaultTask_attributes);

	const osThreadAttr_t uart1RxTask_attributes = { .name = "uart1RxTask",
			.priority = (osPriority_t) osPriorityNormal, .stack_size = 1024 };
	uart1TaskHandle = osThreadNew(uart1RxTask, NULL, &uart1RxTask_attributes);
	const osThreadAttr_t uart2RxTask_attributes = { .name = "uart2RxTask",
			.priority = (osPriority_t) osPriorityNormal, .stack_size = 1024 };
	uart2TaskHandle = osThreadNew(uart2RxTask, NULL, &uart2RxTask_attributes);
	const osThreadAttr_t uart4RxTask_attributes = { .name = "uart4RxTask",
			.priority = (osPriority_t) osPriorityNormal, .stack_size = 1024 };
	uart4TaskHandle = osThreadNew(uart4RxTask, NULL, &uart4RxTask_attributes);
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
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void uart1RxTask(void *argument) {
	UARTMessageQueue_TypeDef msg;
	osStatus_t status;
	while (1) {
		status = osMessageQueueGet(osQueueUart1, &msg, NULL, 0);

		if (status == osOK) {
			HAL_UART_Transmit(&huart1, msg.uartRxTemp, strlen(msg.uartRxTemp),
					1000);
			memset(msg.uartRxTemp, 0x00, strlen(msg.uartRxTemp));
		}
		osThreadYield();
	}
}

void uart2RxTask(void *argument) {
	UARTMessageQueue_TypeDef msg;
	osStatus_t status;
	while (1) {
		status = osMessageQueueGet(osQueueUart2, &msg, NULL, 0);

		if (status == osOK) {
			HAL_UART_Transmit(&huart2, msg.uartRxTemp, strlen(msg.uartRxTemp),
					1000);
			memset(msg.uartRxTemp, 0x00, strlen(msg.uartRxTemp));
		}
		osThreadYield();
	}
}

void uart4RxTask(void *argument) {
	UARTMessageQueue_TypeDef msg;
	osStatus_t status;
	while (1) {
		status = osMessageQueueGet(osQueueUart4, &msg, NULL, 0);

		if (status == osOK) {
			HAL_UART_Transmit(&huart4, msg.uartRxTemp, strlen(msg.uartRxTemp),
					1000);
			memset(msg.uartRxTemp, 0x00, strlen(msg.uartRxTemp));
		}
		osThreadYield();
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
