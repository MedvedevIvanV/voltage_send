/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for STM32F405VGT6 to FPGA communication via FSMC
  ******************************************************************************
  * @attention
  *
  * This code configures FSMC interface to communicate with FPGA (10CL006YE144I7G)
  * without external memory. It reads ADC data from FPGA and sends it via USB CDC.
  *
  * Connections:
  * - FSMC_D[15:0] to FPGA data bus
  * - FSMC_NOE to FPGA_OE (read enable)
  * - FSMC_NWE to FPGA_WE (write enable, not used in this case)
  * - FSMC_NE1 to FPGA chip select
  * - PD6 to FPGA START_FGPA (start signal)
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
#include "usbd_cdc_if.h"
#include "pin_69.h" // Файл с конфигурацией ПЛИС (массив uint8_t fpga_config[])
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DATA_VALUES_COUNT 10000       // Количество значений в ПЛИС (соответствует коду ПЛИС)
#define VALUES_PER_LINE 10            // Количество значений в строке вывода

typedef struct {
    uint16_t data[DATA_VALUES_COUNT]; // Буфер для хранения данных от ПЛИС (12-битные значения)
    bool data_ready;                  // Флаг готовности данных
    uint8_t data_count;               // Количество считанных данных
} FPGA_Data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FPGA_BASE_ADDRESS 0x60000000  // Базовый адрес Bank1 (NE1)
#define START_PULSE_DURATION_NS 200   // Длительность стартового импульса в наносекундах
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
FPGA_Data fpga_data;                 // Структура для хранения данных ПЛИС
char usb_msg[128];                   // Буфер для USB сообщений
volatile uint16_t *fpga_reg;         // Указатель на регистр ПЛИС

// Добавляем объявления переменных из usbd_cdc_if.c
extern volatile uint8_t usb_rx_buffer[64];
extern volatile uint8_t new_data_received;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
static void MX_FSMC_Init(void);
/* USER CODE BEGIN PFP */
void ReadFPGAData(void);
void PrintDataToUSB(void);
void SendUSBDebugMessage(const char *message);
void GenerateStartPulse(void);
void ProcessUSBCommand(uint8_t cmd);
void FPGA_SendConfig(uint8_t *config_data, uint32_t size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief Чтение данных из ПЛИС через FSMC интерфейс
  * @note Читает 10000 значений по 12 бит из ПЛИС (каждое значение в младших 12 битах 16-битного слова)
  * @note ПЛИС автоматически переключает индекс данных при каждом чтении
  */
void ReadFPGAData(void) {
    fpga_data.data_count = 0;
    fpga_data.data_ready = false;

    __disable_irq(); // Отключаем прерывания для атомарного чтения

    for (int i = 0; i < DATA_VALUES_COUNT; i++) {
        // Читаем значение - ПЛИС автоматически переключает индекс при каждом чтении
        uint16_t value = fpga_reg[0];
        fpga_data.data[i] = value & 0x0FFF; // Извлекаем 12-битное значение
        fpga_data.data_count++;

        // Небольшая задержка между чтениями для стабильности
        for(volatile int j = 0; j < 10; j++);
    }

    __enable_irq(); // Включаем прерывания обратно
    fpga_data.data_ready = true;
}

/**
  * @brief Вывод данных через USB CDC
  * @note Форматирует данные в виде таблицы 10x10 значений
  */
void PrintDataToUSB(void) {
    if (!fpga_data.data_ready) return;

    // Формируем заголовок
    snprintf(usb_msg, sizeof(usb_msg), "FPGA Data [0-%d]:\r\n", DATA_VALUES_COUNT-1);
    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
    HAL_Delay(10);

    // Формируем строки с данными
    char data_line[64] = "";
    for (int i = 0; i < DATA_VALUES_COUNT; i++) {
        char val_str[8];
        snprintf(val_str, sizeof(val_str), "%4d ", fpga_data.data[i]);
        strncat(data_line, val_str, sizeof(data_line) - strlen(data_line) - 1);

        // Если строка заполнена или это последнее значение
        if ((i+1) % VALUES_PER_LINE == 0 || i == DATA_VALUES_COUNT-1) {
            strncat(data_line, "\r\n", sizeof(data_line) - strlen(data_line) - 1);
            CDC_Transmit_FS((uint8_t*)data_line, strlen(data_line));
            HAL_Delay(10);
            data_line[0] = '\0'; // Очищаем строку
        }
    }
}

/**
  * @brief Отправка отладочного сообщения через USB
  * @param message Текст сообщения
  */
void SendUSBDebugMessage(const char *message) {
    snprintf(usb_msg, sizeof(usb_msg), "[%lu] %s\r\n", HAL_GetTick(), message);
    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
    HAL_Delay(10); // Задержка для стабильной работы USB
}

/**
  * @brief Генерация стартового импульса для ПЛИС
  * @note Импульс длительностью 200 нс на пине PD6
  */
void GenerateStartPulse(void) {
    // Устанавливаем высокий уровень на PD6
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);

    // Задержка для формирования импульса 200 нс
    // При тактовой частоте 168 МГц (5.95 нс на цикл) нужно ~34 цикла
    for(volatile int i = 0; i < 34; i++);

    // Устанавливаем низкий уровень на PD6
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);

    SendUSBDebugMessage("Start pulse generated");
}

/**
  * @brief Обработка команд от USB
  * @param cmd Полученная команда
  */
void ProcessUSBCommand(uint8_t cmd) {
    switch(cmd) {
        case '1': // Стартовая команда
            GenerateStartPulse();
            ReadFPGAData();

            if (fpga_data.data_ready) {
                SendUSBDebugMessage("Data received from FPGA:");
                PrintDataToUSB();
                fpga_data.data_ready = false;
            }
            break;

        default:
            // Неизвестная команда
            SendUSBDebugMessage("Unknown command received");
            break;
    }
}



void FPGA_SendConfig(uint8_t *config_data, uint32_t size) {
    // 1. Инициализация пинов
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Настройка DATA (PC11) как выход
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 2. Последовательность сброса ПЛИС
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);   // TH_CS = 1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // cso = 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // nCONFIG = 0
    HAL_Delay(100); // Длительный сброс (100 мс)

    // 3. Запуск конфигурации
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);  // CE = 0
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   // nCONFIG = 1
    HAL_Delay(10); // Ожидание готовности ПЛИС

    // 4. Отправка данных конфигурации
    for (uint32_t i = 0; i < size; i++) {
        uint8_t byte = config_data[i];
        for (int bit = 0; bit < 8; bit++) {
            // Установка бита данных (LSB first)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, (byte & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            byte >>= 1;

            // Тактовый импульс (минимум 50 нс)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
            __NOP(); __NOP(); __NOP(); __NOP(); // Короткая задержка (~20 нс при 168 MHz)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
            __NOP(); __NOP(); // Пауза между битами
        }
    }

    // 5. Завершение конфигурации
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);    // CE = 1
    HAL_Delay(1);

    // 6. Дополнительные тактовые импульсы
    for (int i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
        __NOP(); __NOP();
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
        __NOP(); __NOP();
    }

    // 7. Возврат в исходное состояние
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);  // TH_CS = 0
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);  // cso = 0
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // DATA = 0
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
  //MX_USART1_UART_Init();
  //MX_USB_DEVICE_Init();
  //MX_DAC_Init();
  //MX_TIM3_Init();
  //MX_FSMC_Init();
  /* USER CODE BEGIN 2 */

    // Получаем данные конфигурации из pin_69.h
    uint8_t *config_data = fpga_config; // Используем массив из pin_69.h
    uint32_t config_size = sizeof(fpga_config); // Размер автоматически вычисляется

    // Вызов функции загрузки конфигурации
    FPGA_SendConfig(config_data, config_size);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
//    	char debug_msg[100];
//
//    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
//    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
//    	snprintf(debug_msg, sizeof(debug_msg), "STATE: PC8=%d, PA15=%d, PB8=%d, PC9=%d, PC12=%d, PC10=%d\r\n",
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8),
//    	         HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15),
//    	         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10));
//    	CDC_Transmit_FS((uint8_t*)debug_msg, strlen(debug_msg));
//    	HAL_Delay(10);
//
//    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
//    	snprintf(debug_msg, sizeof(debug_msg), "STATE: PC8=%d, PA15=%d, PB8=%d, PC9=%d, PC12=%d, PC10=%d\r\n",
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8),
//    	         HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15),
//    	         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10));
//    	CDC_Transmit_FS((uint8_t*)debug_msg, strlen(debug_msg));
//    	HAL_Delay(10);
//
//    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
//    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
//    	snprintf(debug_msg, sizeof(debug_msg), "STATE: PC8=%d, PA15=%d, PB8=%d, PC9=%d, PC12=%d, PC10=%d\r\n",
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8),
//    	         HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15),
//    	         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10));
//    	CDC_Transmit_FS((uint8_t*)debug_msg, strlen(debug_msg));
//    	HAL_Delay(100);
//
//    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
//    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
//    	snprintf(debug_msg, sizeof(debug_msg), "STATE: PC8=%d, PA15=%d, PB8=%d, PC9=%d, PC12=%d, PC10=%d\r\n",
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8),
//    	         HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15),
//    	         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10));
//    	CDC_Transmit_FS((uint8_t*)debug_msg, strlen(debug_msg));
//    	HAL_Delay(1);
//
//    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
//    	snprintf(debug_msg, sizeof(debug_msg), "STATE: PC8=%d, PA15=%d, PB8=%d, PC9=%d, PC12=%d, PC10=%d\r\n",
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8),
//    	         HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15),
//    	         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10));
//    	CDC_Transmit_FS((uint8_t*)debug_msg, strlen(debug_msg));
//    	HAL_Delay(1);
//
//    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
//    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
//    	snprintf(debug_msg, sizeof(debug_msg), "STATE: PC8=%d, PA15=%d, PB8=%d, PC9=%d, PC12=%d, PC10=%d\r\n",
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8),
//    	         HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15),
//    	         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12),
//    	         HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10));
//    	CDC_Transmit_FS((uint8_t*)debug_msg, strlen(debug_msg));
//    	HAL_Delay(1000);
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

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
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 167;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_ENABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_PSRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_DISABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
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
