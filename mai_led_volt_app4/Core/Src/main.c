/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  SPI_STATE_IDLE,
  SPI_STATE_CS_LOW,
  SPI_STATE_COMPLETE
} SPI_GPIO_State;

typedef struct {
  uint16_t data[1000];        // Хранение 1000 12-битных значений
  uint16_t current_word;      // Текущее принимаемое слово (12 бит)
  uint8_t bit_counter;        // Счетчик битов (0-11)
  uint16_t word_counter;      // Счетчик слов (0-999)
  uint8_t last_sclk_state;    // Последнее состояние SCLK
  uint8_t last_cs_state;      // Последнее состояние CS
} SPI_Receiver;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_EXPECTED_WORDS 1000  // Ожидаем 1000 12-битных слов
#define SPI_DATA_BITS 12         // Работаем с 12 битами
#define SPI_MODE 0               // CPOL=0, CPHA=0
#define VALUES_PER_LINE 10       // Количество значений в строке вывода

// Настройки FLASH-памяти
#define FLASH_TARGET_SECTOR FLASH_SECTOR_7  // Используем 7-й сектор (проверьте для вашего MCU)
#define FLASH_TARGET_ADDR 0x08060000        // Адрес начала сектора (проверьте для вашего MCU)
#define FLASH_DATA_SIZE (NUM_EXPECTED_WORDS * sizeof(uint16_t)) // Размер данных для сохранения
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
SPI_Receiver spi_receiver;
volatile SPI_GPIO_State spi_state = SPI_STATE_IDLE;
char usb_msg[128];
bool data_saved_to_flash = false; // Флаг сохранения данных в FLASH
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void ProcessSPI_GPIO(void);
void PrintSPIData(void);
void ResetSPIReceiver(void);
bool SaveToFlash(uint16_t* data, uint32_t size);
bool ReadFromFlash(uint16_t* data, uint32_t size);
void EraseFlashSector(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief Сброс состояния SPI приемника
  */
void ResetSPIReceiver(void) {
  memset(&spi_receiver, 0, sizeof(spi_receiver));
  spi_receiver.last_sclk_state = (SPI_MODE == 0 || SPI_MODE == 2) ? 0 : 1;
  spi_state = SPI_STATE_IDLE;
  data_saved_to_flash = false;
}

/**
  * @brief Вывод данных SPI через USB
  */
void PrintSPIData(void) {
  uint16_t flash_data[NUM_EXPECTED_WORDS];

  // Сначала пытаемся прочитать данные из FLASH
  if (ReadFromFlash(flash_data, FLASH_DATA_SIZE)) {
    for (int i = 0; i < NUM_EXPECTED_WORDS; i += VALUES_PER_LINE) {
      snprintf(usb_msg, sizeof(usb_msg), "FLASH Data: ");
      int end = (i + VALUES_PER_LINE) < NUM_EXPECTED_WORDS ? (i + VALUES_PER_LINE) : NUM_EXPECTED_WORDS;

      for (int j = i; j < end; j++) {
        char word_str[8];
        snprintf(word_str, sizeof(word_str), "%04X ", flash_data[j] & 0xFFF); // Маска для 12 бит
        strncat(usb_msg, word_str, sizeof(usb_msg) - strlen(usb_msg) - 1);
      }
      strncat(usb_msg, "\r\n", sizeof(usb_msg) - strlen(usb_msg) - 1);
      CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
      HAL_Delay(10); // Небольшая задержка для USB
    }
  } else {
    snprintf(usb_msg, sizeof(usb_msg), "Failed to read data from FLASH!\r\n");
    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
  }
}

/**
  * @brief Обработка SPI через GPIO (битбэнг)
  */
void ProcessSPI_GPIO(void) {
    uint8_t current_cs = HAL_GPIO_ReadPin(CS_GPIO_Port, CS_Pin);
    uint8_t current_sclk = HAL_GPIO_ReadPin(SCLK_GPIO_Port, SCLK_Pin);
    uint8_t current_miso = HAL_GPIO_ReadPin(MISO_GPIO_Port, MISO_Pin);

    // SPI Mode 0 (CPOL=0, CPHA=0) - sampling on rising edge
    if (spi_state == SPI_STATE_CS_LOW && current_cs == 0) {
        if (current_sclk && !spi_receiver.last_sclk_state) { // Rising edge
            spi_receiver.current_word <<= 1;
            spi_receiver.current_word |= current_miso;
            spi_receiver.bit_counter++;

            if (spi_receiver.bit_counter >= SPI_DATA_BITS) {
                spi_receiver.data[spi_receiver.word_counter++] = spi_receiver.current_word & 0xFFF; // Сохраняем только 12 бит
                spi_receiver.current_word = 0;
                spi_receiver.bit_counter = 0;

                if (spi_receiver.word_counter >= NUM_EXPECTED_WORDS) {
                    spi_state = SPI_STATE_COMPLETE;
                }
            }
        }
    }
    else if (current_cs) {
        spi_state = SPI_STATE_IDLE;
    }
    else if (!current_cs && spi_state == SPI_STATE_IDLE) {
        spi_state = SPI_STATE_CS_LOW;
        spi_receiver.bit_counter = 0;
        spi_receiver.current_word = 0;
        spi_receiver.word_counter = 0;
    }

    spi_receiver.last_sclk_state = current_sclk;
    spi_receiver.last_cs_state = current_cs;
}

/**
  * @brief Стирание сектора FLASH
  */
void EraseFlashSector(void) {
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0;

    HAL_FLASH_Unlock(); // Разблокируем FLASH для записи

    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Sector = FLASH_TARGET_SECTOR;
    erase_init.NbSectors = 1;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK) {
        // Ошибка стирания
        snprintf(usb_msg, sizeof(usb_msg), "FLASH erase failed!\r\n");
        CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
    }

    HAL_FLASH_Lock(); // Блокируем FLASH после записи
}

/**
  * @brief Сохранение данных во FLASH
  * @param data Указатель на данные
  * @param size Размер данных в байтах
  * @return true если успешно, false если ошибка
  */
bool SaveToFlash(uint16_t* data, uint32_t size) {
    uint32_t address = FLASH_TARGET_ADDR;
    uint32_t num_words = size / sizeof(uint16_t);

    // Сначала стираем сектор
    EraseFlashSector();

    HAL_FLASH_Unlock(); // Разблокируем FLASH для записи

    // Записываем данные по полусловам (16 бит)
    for (uint32_t i = 0; i < num_words; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, data[i]) != HAL_OK) {
            HAL_FLASH_Lock(); // Блокируем FLASH в случае ошибки
            return false;
        }
        address += sizeof(uint16_t);
    }

    HAL_FLASH_Lock(); // Блокируем FLASH после записи
    return true;
}

/**
  * @brief Чтение данных из FLASH
  * @param data Указатель на буфер для данных
  * @param size Размер данных в байтах
  * @return true если успешно, false если ошибка
  */
bool ReadFromFlash(uint16_t* data, uint32_t size) {
    uint32_t address = FLASH_TARGET_ADDR;
    uint32_t num_words = size / sizeof(uint16_t);

    // Просто копируем данные из FLASH в RAM
    for (uint32_t i = 0; i < num_words; i++) {
        data[i] = *(__IO uint16_t*)address;
        address += sizeof(uint16_t);
    }

    return true;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_DAC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  ResetSPIReceiver();
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED off

  // Выводим сообщение о готовности
  snprintf(usb_msg, sizeof(usb_msg), "System initialized. Waiting for SPI data...\r\n");
  CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    ProcessSPI_GPIO();

    if (spi_state == SPI_STATE_COMPLETE && !data_saved_to_flash) {
      // Сохраняем данные в FLASH перед выводом
      if (SaveToFlash(spi_receiver.data, FLASH_DATA_SIZE)) {
        data_saved_to_flash = true;
        snprintf(usb_msg, sizeof(usb_msg), "Data saved to FLASH successfully!\r\n");
        CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
      } else {
        snprintf(usb_msg, sizeof(usb_msg), "Failed to save data to FLASH!\r\n");
        CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
      }

      // Выводим данные из FLASH
      PrintSPIData();
      ResetSPIReceiver();
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Toggle LED on complete
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

// Остальной код (SystemClock_Config, MX_GPIO_Init, MX_USART1_UART_Init, MX_DAC_Init, MX_TIM3_Init, Error_Handler)
// остается без изменений, как в вашем исходном коде
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{
  DAC_ChannelConfTypeDef sConfig = {0};

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 167;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFFFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 UART Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|TH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : CS_Pin MISO_Pin SCLK_Pin */
  GPIO_InitStruct.Pin = CS_Pin|MISO_Pin|SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIGGER_Pin */
  GPIO_InitStruct.Pin = TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIGGER_GPIO_Port, &GPIO_InitStruct);
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
