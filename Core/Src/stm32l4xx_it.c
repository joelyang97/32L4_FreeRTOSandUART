/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32l4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim8;

/* USER CODE BEGIN EV */
extern uint8_t uart1RxBuffer[UARTRXBUFFERSIZE];
extern uint8_t uart2RxBuffer[UARTRXBUFFERSIZE];
extern uint8_t uart4RxBuffer[UARTRXBUFFERSIZE];

extern osMessageQueueId_t osQueueUart1;
extern osMessageQueueId_t osQueueUart2;
extern osMessageQueueId_t osQueueUart4;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */

	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void) {
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles DMA1 channel5 global interrupt.
 */
void DMA1_Channel5_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

	/* USER CODE END DMA1_Channel5_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart1_rx);
	/* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

	/* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel6 global interrupt.
 */
void DMA1_Channel6_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

	/* USER CODE END DMA1_Channel6_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart2_rx);
	/* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

	/* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void) {
	/* USER CODE BEGIN USART1_IRQn 0 */
	uint8_t temp;
	UARTMessageQueue_TypeDef msg;
	uint8_t str[7] = "USART1";
	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) {
		//空闲中断
		//temp = huart4.Instance->ISR;
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		temp = huart1.Instance->RDR;
		temp = temp;
		HAL_UART_DMAStop(&huart1);

		if (strlen(uart1RxBuffer) != 0) {
			memset(msg.uartRxTemp, 0x00, sizeof(msg.uartRxTemp));
			memcpy(msg.uartRxTemp, uart1RxBuffer, strlen(uart1RxBuffer));
			memcpy(msg.uartRxName, str, strlen(str));
			memset(uart1RxBuffer, 0x00, strlen(uart1RxBuffer));
			osMessageQueuePut(osQueueUart1, &msg, 0, 0);
		}

		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart1, uart1RxBuffer, UARTRXBUFFERSIZE);
	}
	/* USER CODE END USART1_IRQn 0 */
	HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART1_IRQn 1 */

	/* USER CODE END USART1_IRQn 1 */
}

/**
 * @brief This function handles USART2 global interrupt.
 */
void USART2_IRQHandler(void) {
	/* USER CODE BEGIN USART2_IRQn 0 */
	uint8_t temp;
	UARTMessageQueue_TypeDef msg;
	uint8_t str[7] = "USART2";
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET) {
		//空闲中断
		//temp = huart4.Instance->ISR;
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		temp = huart2.Instance->RDR;
		temp = temp;
		HAL_UART_DMAStop(&huart2);

		if (strlen(uart2RxBuffer) != 0) {
			memset(msg.uartRxTemp, 0x00, sizeof(msg.uartRxTemp));
			memcpy(msg.uartRxTemp, uart2RxBuffer, strlen(uart2RxBuffer));
			memcpy(msg.uartRxName, str, strlen(str));
			memset(uart2RxBuffer, 0x00, strlen(uart2RxBuffer));
			osMessageQueuePut(osQueueUart2, &msg, 0, 0);
		}

		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart2, uart2RxBuffer, UARTRXBUFFERSIZE);
	}
	/* USER CODE END USART2_IRQn 0 */
	HAL_UART_IRQHandler(&huart2);
	/* USER CODE BEGIN USART2_IRQn 1 */

	/* USER CODE END USART2_IRQn 1 */
}

/**
 * @brief This function handles TIM8 update interrupt.
 */
void TIM8_UP_IRQHandler(void) {
	/* USER CODE BEGIN TIM8_UP_IRQn 0 */

	/* USER CODE END TIM8_UP_IRQn 0 */
	HAL_TIM_IRQHandler(&htim8);
	/* USER CODE BEGIN TIM8_UP_IRQn 1 */

	/* USER CODE END TIM8_UP_IRQn 1 */
}

/**
 * @brief This function handles UART4 global interrupt.
 */
void UART4_IRQHandler(void) {
	/* USER CODE BEGIN UART4_IRQn 0 */
	uint8_t temp;
	UARTMessageQueue_TypeDef msg;
	uint8_t str[7] = "UART4";
	if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE) != RESET) {
		//空闲中断
		//temp = huart4.Instance->ISR;
		__HAL_UART_CLEAR_IDLEFLAG(&huart4);
		temp = huart4.Instance->RDR;
		temp = temp;
		HAL_UART_DMAStop(&huart4);

		if (strlen(uart4RxBuffer) != 0) {
			memset(msg.uartRxTemp, 0x00, sizeof(msg.uartRxTemp));
			memcpy(msg.uartRxTemp, uart4RxBuffer, strlen(uart4RxBuffer));
			memcpy(msg.uartRxName, str, strlen(str));
			memset(uart4RxBuffer, 0x00, strlen(uart4RxBuffer));
			osMessageQueuePut(osQueueUart4, &msg, 0, 0);
		}

		__HAL_UART_CLEAR_IDLEFLAG(&huart4);
		__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart4, uart4RxBuffer, UARTRXBUFFERSIZE);
	}
	/* USER CODE END UART4_IRQn 0 */
	HAL_UART_IRQHandler(&huart4);
	/* USER CODE BEGIN UART4_IRQn 1 */

	/* USER CODE END UART4_IRQn 1 */
}

/**
 * @brief This function handles DMA2 channel5 global interrupt.
 */
void DMA2_Channel5_IRQHandler(void) {
	/* USER CODE BEGIN DMA2_Channel5_IRQn 0 */

	/* USER CODE END DMA2_Channel5_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_uart4_rx);
	/* USER CODE BEGIN DMA2_Channel5_IRQn 1 */

	/* USER CODE END DMA2_Channel5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
