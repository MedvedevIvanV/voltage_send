/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "signal.h" // Файл с конфигурацией ПЛИС (массив uint8_t fpga_config[])
#include "stm32f4xx_hal.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief Загрузка конфигурации в ПЛИС
  */
void FPGA_LoadConfig(void) {
	 GPIO_InitTypeDef GPIO_InitStruct = {0};

	    // 1. Настройка пинов (оставляем как было)
	    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	    GPIO_InitStruct.Pin = GPIO_PIN_15;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	    GPIO_InitStruct.Pin = GPIO_PIN_8;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    // 2. Установка начальных состояний
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);   // TH_CS = 1
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   // CE = 1
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // CLK = 0
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // DATA = 0
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // nCONFIG = 0
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // CSO = 1

	    // 3. ВАЖНО: Увеличиваем задержку для стабилизации питания при холодном старте
	    HAL_Delay(100); // Возвращаем 100 мс для надежности

	    // 4. Сброс ПЛИС с достаточной паузой
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // nCONFIG = 0
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);  // CE = 0
	    HAL_Delay(5); // Увеличиваем до 5 мс (вместо 1 мс)

	    // 5. Запуск конфигурации
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   // nCONFIG = 1

	    // 6. Ожидание готовности ПЛИС (увеличиваем)
	    HAL_Delay(3); // Увеличиваем до 3 мс

	    // 7. ОПТИМИЗИРОВАННАЯ передача данных (оставляем быструю)
	    const uint8_t *config_data = fpga_config;
	    uint32_t config_size = sizeof(fpga_config);

	    GPIO_TypeDef* data_port = GPIOC;
	    GPIO_TypeDef* clk_port = GPIOC;
	    uint16_t data_pin = GPIO_PIN_11;
	    uint16_t clk_pin = GPIO_PIN_10;

	    for (uint32_t i = 0; i < config_size; i++) {
	        uint8_t byte = config_data[i];

	        // Развернутый цикл для скорости (оставляем как было)
	        // Бит 0
	        if (byte & 0x01) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;

	        // Бит 1
	        if (byte & 0x02) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;

	        // Бит 2
	        if (byte & 0x04) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;

	        // Бит 3
	        if (byte & 0x08) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;

	        // Бит 4
	        if (byte & 0x10) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;

	        // Бит 5
	        if (byte & 0x20) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;

	        // Бит 6
	        if (byte & 0x40) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;

	        // Бит 7
	        if (byte & 0x80) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;
	    }

	    // 8. Завершающие импульсы (увеличиваем количество для надежности)
	    for (int i = 0; i < 12; i++) { // Увеличиваем до 12
	        clk_port->BSRR = clk_pin;
	        __NOP(); __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;
	        __NOP(); __NOP();
	    }

	    // 9. Активация ПЛИС с достаточной паузой
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   // CE = 1
	    HAL_Delay(3); // Увеличиваем до 3 мс

	    // 10. Финальные настройки
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);  // TH_CS = 0
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);  // CSO = 0

	    // 11. Финальная пауза для стабилизации
	    HAL_Delay(5); // Увеличиваем до 5 мс
}

void FPGA_SendConfig(uint8_t *config_data, uint32_t size) {
    FPGA_LoadConfig();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	 /* USER CODE BEGIN 1 */

	  /* USER CODE END 1 */

	  /* MCU Configuration--------------------------------------------------------*/
	  HAL_Init();
	  SystemClock_Config();

	  /* USER CODE BEGIN SysInit */
	  // Принудительно включаем тактирование всех используемых портов
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  /* USER CODE END SysInit */

	  /* Initialize all configured peripherals */
	  MX_GPIO_Init();

	  /* USER CODE BEGIN 2 */
	  // Даем время на стабилизацию питания после включения
	  // Умеренное уменьшение задержки
	      HAL_Delay(150);

	      // Загружаем конфигурацию ПЛИС
	      FPGA_LoadConfig();

	      // Уменьшаем дополнительную паузу
	      HAL_Delay(10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
