/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
uint8_t uart1RxBuffer[UARTRXBUFFERSIZE] = { 0 };
uint8_t uart2RxBuffer[UARTRXBUFFERSIZE] = { 0 };
uint8_t uart4RxBuffer[UARTRXBUFFERSIZE] = { 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void uartEnableIDLE(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_UART4_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	uartEnableIDLE();
	/* USER CODE END 2 */

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_UART4;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
/* 开启IDLE空闲中断和DMA接收
 *
 */
void uartEnableIDLE(void) {
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
	HAL_UART_Receive_DMA(&huart1, uart1RxBuffer, UARTRXBUFFERSIZE);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
	HAL_UART_Receive_DMA(&huart2, uart2RxBuffer, UARTRXBUFFERSIZE);
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
	HAL_UART_Receive_DMA(&huart4, uart4RxBuffer, UARTRXBUFFERSIZE);
}

/**
 * @brief  UART error callback.
 * @param  huart UART handle.
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);
	uint8_t temp;
	/* NOTE : This function should not be modified, when the callback is needed,
	 the HAL_UART_ErrorCallback can be implemented in the user file.
	 */
	if (huart->Instance == USART1) {
		if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET) {
			temp = huart1.Instance->RDR;
			temp = temp;
			__HAL_UART_CLEAR_OREFLAG(&huart1);
			HAL_UART_Receive_DMA(&huart1, uart1RxBuffer, UARTRXBUFFERSIZE);
		}
	} else if (huart->Instance == USART2) {
		if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET) {
			temp = huart2.Instance->RDR;
			temp = temp;
			__HAL_UART_CLEAR_OREFLAG(&huart2);
			HAL_UART_Receive_DMA(&huart2, uart2RxBuffer, UARTRXBUFFERSIZE);
		}
	} else if (huart->Instance == UART4) {
		if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_ORE) != RESET) {
			temp = huart4.Instance->RDR;
			temp = temp;
			__HAL_UART_CLEAR_OREFLAG(&huart4);
			HAL_UART_Receive_DMA(&huart4, uart4RxBuffer, UARTRXBUFFERSIZE);
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM8 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM8) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
