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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Определяем адрес во Flash памяти для хранения напряжения (последняя страница Flash)
#define VOLTAGE_STORAGE_ADDR 0x080E0000 // Адрес Sector 11 (для STM32F405VG)
#define FLASH_SECTOR FLASH_SECTOR_11
#define CS_LOW    GPIOC->BSRR = GPIO_BSRR_BR_8
#define CS_HIGH   GPIOC->BSRR = GPIO_BSRR_BS_8
// Переменная напряжения, размещенная во Flash памяти (только для чтения)
__attribute__((section(".rodata"))) const float default_voltage = 3.3f; // Значение по умолчанию
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t usb_led_command = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef Write_Voltage_To_Flash(float voltage) {
    HAL_StatusTypeDef status;
    uint32_t voltageData = *(uint32_t*)&voltage;

    // Разблокировка Flash
    HAL_FLASH_Unlock();

    // Очистка ошибок
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                          FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    // Стирание сектора
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = FLASH_SECTOR;
    EraseInitStruct.NbSectors = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; // Для работы при 2.7-3.6V

    uint32_t SectorError;
    status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    // Запись данных
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, VOLTAGE_STORAGE_ADDR, voltageData);
    FLASH_WaitForLastOperation(100);

    HAL_FLASH_Lock();
    return status;
}

// Функция для чтения float из Flash
float Read_Voltage_From_Flash() {
    uint32_t voltageData = *(__IO uint32_t*)(VOLTAGE_STORAGE_ADDR);

    // Если Flash пуста (все 0xFF), возвращаем значение по умолчанию
    if (voltageData == 0xFFFFFFFF) {
        return default_voltage;
    }
    return *(float*)&voltageData;
}



void Spi3_Init(void) {
    // Включаем тактирование порта C и SPI3
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

    // Настройка PC8 (CS) как выход
    GPIOC->MODER |= GPIO_MODER_MODER8_0;  // Output mode
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT_8;   // Push-pull
    GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED8; // High speed

    // Настройка PC10 (SCK) и PC11 (MISO) как альтернативные функции
    GPIOC->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1; // Alternate function mode
    GPIOC->AFR[1] |= (6 << (4*(10-8))) | (6 << (4*(11-8))); // AF6 for SPI3

    // Настройка SPI3
    SPI3->CR1 = SPI_CR1_SSM |          // Software slave management
                SPI_CR1_SSI |          // Internal slave select
                SPI_CR1_MSTR |         // Master mode
                SPI_CR1_BR_2 |         // Baud rate control: fPCLK/256
                SPI_CR1_CPHA |         // CPHA = 1
                SPI_CR1_DFF;           // 16-bit data format

    SPI3->CR1 |= SPI_CR1_SPE;          // Enable SPI3
}

uint16_t Spi3_Read_Data(void) {
    uint16_t data = 0;

    // Активируем Chip Select
    CS_LOW;

    // Ждем, пока Tx буфер опустеет
    while(!(SPI3->SR & SPI_SR_TXE));

    // Отправляем "пустые" данные для генерации тактовых импульсов
    SPI3->DR = 0x0000;

    // Ждем, пока данные будут получены
    while(!(SPI3->SR & SPI_SR_RXNE));

    // Считываем полученные данные
    data = SPI3->DR;

    // Деактивируем Chip Select
    CS_HIGH;

    return data;
}

float Read_Temperature(void) {
    uint16_t raw_data = Spi3_Read_Data();

    // Проверка на разрыв термопары (бит D2)
    if(raw_data & 0x04) {
        return -1.0f; // Ошибка - разрыв термопары
    }

    // Извлекаем 12-битное значение температуры (биты D15-D3)
    raw_data >>= 3;

    // Преобразуем в градусы Цельсия (0.25°C на LSB)
    return raw_data * 0.25f;
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
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  // Включаем транзистор
  Spi3_Init();

  float temperature = Read_Temperature();

          if(temperature < 0) {
              printf("Error: Thermocouple open!\n");
          } else {
              printf("Temperature: %.2f C\n", temperature);
          }

          // Задержка ~1 секунда
          HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
//    if (usb_rx_flag) {
//
//	  float current_voltage = Read_Voltage_From_Flash();
//
//	  char init_msg[32];
//	  sprintf(init_msg, "Initial voltage: %.4fV\r\n", current_voltage);
//	  CDC_Transmit_FS((uint8_t*)init_msg, strlen(init_msg));
//
//      usb_rx_flag = 0;
//      uint8_t usb_led_command = 'm';
//
//      // 1. Отправляем команду дежурному МК
//      if (HAL_UART_Transmit(&huart1, &usb_led_command, 1, 100) == HAL_OK) {
//          uint8_t uart_rx_buffer[64] = {0};
//          uint16_t index = 0;
//          float voltage = 0.0f;
//
//          // 2. Принимаем полное значение напряжения
//          while (1) {
//              if (HAL_UART_Receive(&huart1, &uart_rx_buffer[index], 1, 500) == HAL_OK) {
//                  // Проверяем конец строки
//                  if (uart_rx_buffer[index] == '\n' || index >= sizeof(uart_rx_buffer) - 1) {
//                      uart_rx_buffer[index] = '\0'; // Добавляем терминатор
//
//                      // 3. Преобразуем строку в float
//                      voltage = atof((char*)uart_rx_buffer);
//
//                      // 4. Записываем ВЕСЬ результат во Flash
//                      if (Write_Voltage_To_Flash(voltage) == HAL_OK) {
//                          // 5. Только после успешной записи читаем и отправляем
//                          float stored_voltage = Read_Voltage_From_Flash();
//
//                          char voltage_msg[32];
//                          sprintf(voltage_msg, "FLASH: %.4fV\r\n", stored_voltage);
//                          CDC_Transmit_FS((uint8_t*)voltage_msg, strlen(voltage_msg));
//                      } else {
//                          CDC_Transmit_FS((uint8_t*)"Flash write error\r\n", 19);
//                      }
//                      break;
//                  }
//                  index++;
//              } else {
//                  CDC_Transmit_FS((uint8_t*)"UART timeout\r\n", 14);
//                  break;
//              }
//          }
//      }
//    }
    HAL_Delay(5000);
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|TH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 TH_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|TH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
