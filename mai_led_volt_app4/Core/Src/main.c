/* USER CODE BEGIN Header */
/**
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
#include "arm_math.h"
#include <stdlib.h>
#include "a-scan.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
char usb_msg[128];                   // Буфер для USB сообщений

// Добавляем объявления переменных из usbd_cdc_if.c
extern volatile uint8_t usb_rx_buffer[64];
extern volatile uint8_t new_data_received;


#define VALUES_PER_LINE 10
#define DATA_VALUES_COUNT 4600

extern const float measurement_data[];

float32_t normalized_data[DATA_VALUES_COUNT];
float32_t autocorrelation_result[DATA_VALUES_COUNT];
float32_t mean, std_dev;

arm_status status;

// Добавленные переменные
uint32_t start_index = 100;          // Начальный индекс для среза массива
float wave_speed = 3200.0;           // Скорость волны в м/с
float frequency = 12.5e-9;           // Частота в секундах (12.5 нс)
float one_point_mm;                  // Разрешение одного отсчета в мм
uint32_t max_index;                  // Индекс максимального значения АКФ

// Добавляем новые переменные
#define THRESHOLD 2080.0f
#define CYCLES_NUMBER 10
#define FLASH_DATA_ADDRESS 0x08080000  // Адрес во FLASH памяти для сохранения данных

float32_t flash_stored_data[DATA_VALUES_COUNT];
float32_t working_data[DATA_VALUES_COUNT];  // Рабочий массив для модификаций
uint8_t cycles_completed = 0;
bool data_stored_in_flash = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
static void MX_FSMC_Init(void);
/* USER CODE BEGIN PFP */
void SendUSBDebugMessage(const char *message);
void GenerateStartPulse(void);
void ProcessUSBCommand(uint8_t cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
            break;

        default:
            // Неизвестная команда
            SendUSBDebugMessage("Unknown command received");
            break;
    }
}

void NormalizeData(void)
{
    // Используем normalized_data как входной массив вместо measurement_data
    arm_mean_f32(normalized_data, DATA_VALUES_COUNT, &mean);

    float32_t subtracted_mean[DATA_VALUES_COUNT];
    arm_offset_f32(normalized_data, -mean, subtracted_mean, DATA_VALUES_COUNT);

    arm_std_f32(subtracted_mean, DATA_VALUES_COUNT, &std_dev);

    if (std_dev != 0.0f) {
        arm_scale_f32(subtracted_mean, 1.0f/std_dev, normalized_data, DATA_VALUES_COUNT);
    } else {
        arm_copy_f32(subtracted_mean, normalized_data, DATA_VALUES_COUNT);
    }
}

void CalculateAutocorrelation(void)
{
    for (uint32_t lag = 0; lag < DATA_VALUES_COUNT; lag++) {
        double sum = 0.0; // Накопление в double для точности
        uint32_t count = DATA_VALUES_COUNT - lag;

        for (uint32_t i = 0; i < count; i++) {
            // Накопление суммы в double для большей точности
            sum += (double)normalized_data[i] * (double)normalized_data[i + lag];
        }

        // Вычисление среднего значения и взятие модуля
        double result = sum / count;

        // Сохранение результата по модулю в float32_t
        autocorrelation_result[lag] = (float32_t)fabs(result);
    }
}

/**
  * @brief Поиск индекса максимального значения автокорреляционной функции
  * @retval Индекс максимального значения в диапазоне [start_index:N-start_index]
  */
uint32_t FindMaxAutocorrelationIndex(void)
{
    float32_t max_value = 0.0f;
    uint32_t max_idx = start_index;

    // Поиск максимума в диапазоне [start_index:DATA_VALUES_COUNT-start_index]
    for (uint32_t i = start_index; i < DATA_VALUES_COUNT - start_index; i++) {
        if (autocorrelation_result[i] > max_value) {
            max_value = autocorrelation_result[i];
            max_idx = i;
        }
    }

    return max_idx;
}

/**
  * @brief Расчет толщины и отправка результата по USB
  */
void CalculateAndSendThickness(void)
{
    // Расчет разрешения одного отсчета в мм
    one_point_mm = 1.0f / (wave_speed * 1000.0f * frequency);

    // Поиск индекса максимального значения АКФ
    max_index = FindMaxAutocorrelationIndex();

    // Расчет толщины
    float thickness = max_index / (2.0f * one_point_mm);

    // Отправка результата по USB
    snprintf(usb_msg, sizeof(usb_msg), "Thickness: %.3f\r\n", thickness);
    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
    HAL_Delay(10);
}

void PrintMeasurementDataToUSB(void)
{
    // Формируем заголовок
    snprintf(usb_msg, sizeof(usb_msg), "Autocorrelation Result [0-%d]:\r\n", DATA_VALUES_COUNT-1);
    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
    HAL_Delay(10);

    // Формируем строки с данными
    char data_line[128] = "";

    for (int i = 0; i < DATA_VALUES_COUNT; i++) {
        char val_str[16];

        // Форматируем float значение с двумя знаками после запятой
        snprintf(val_str, sizeof(val_str), "%7.2f ", autocorrelation_result[i]);
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
  * @brief Проверка данных на превышение threshold
  * @param data Массив данных для проверки
  * @retval true если все значения в пределах threshold, false если есть превышения
  */
bool CheckThreshold(const float32_t* data) {
    for (uint32_t i = 0; i < DATA_VALUES_COUNT; i++) {
        if (fabsf(data[i]) > THRESHOLD) {
            return false;
        }
    }
    return true;
}

/**
  * @brief Сохранение данных во FLASH память
  * @param data Массив данных для сохранения
  */
void SaveToFlash(const float32_t* data) {
    // Разблокируем FLASH память
    HAL_FLASH_Unlock();

    // Очищаем страницу FLASH
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = FLASH_SECTOR_11;  // Выберите подходящий сектор
    EraseInitStruct.NbSectors = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

    // Записываем данные во FLASH
    uint32_t* flash_ptr = (uint32_t*)FLASH_DATA_ADDRESS;
    uint32_t* data_ptr = (uint32_t*)data;

    for (uint32_t i = 0; i < DATA_VALUES_COUNT; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)flash_ptr, *data_ptr);
        flash_ptr++;
        data_ptr++;
    }

    // Блокируем FLASH память
    HAL_FLASH_Lock();

    data_stored_in_flash = true;
}

/**
  * @brief Чтение данных из FLASH памяти
  * @param data Буфер для сохранения прочитанных данных
  */
void ReadFromFlash(float32_t* data) {
    uint32_t* flash_ptr = (uint32_t*)FLASH_DATA_ADDRESS;
    uint32_t* data_ptr = (uint32_t*)data;

    for (uint32_t i = 0; i < DATA_VALUES_COUNT; i++) {
        *data_ptr = *flash_ptr;
        flash_ptr++;
        data_ptr++;
    }
}

/**
  * @brief Усреднение текущих данных с данными из FLASH
  * @param current_data Текущий массив данных
  */
void AverageWithFlashData(float32_t* current_data) {
    float32_t flash_data[DATA_VALUES_COUNT];
    ReadFromFlash(flash_data);

    for (uint32_t i = 0; i < DATA_VALUES_COUNT; i++) {
        flash_data[i] = (flash_data[i] + current_data[i]) / 2.0f;
    }

    SaveToFlash(flash_data);
}

/**
  * @brief Копирование исходных данных в рабочий массив и добавление шума
  * @param dest Массив назначения
  * @param src Исходный массив
  */
void CopyAndAddNoise(float32_t* dest, const float32_t* src) {
    // Копируем исходные данные
    arm_copy_f32((float32_t*)src, dest, DATA_VALUES_COUNT);

    // Добавляем случайный шум ±50% от значения
    for (uint32_t i = 0; i < DATA_VALUES_COUNT; i++) {
        float noise = (rand() % 10000 - 5000) / 10000.0f;  // ±50%
        dest[i] = dest[i] * (1.0f + noise);
    }
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
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_FSMC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);

  // Инициализация добавленных переменных
  one_point_mm = 1.0f / (wave_speed * 1000.0f * frequency);

  // Инициализация генератора случайных чисел
  srand(HAL_GetTick());

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
      // Проверяем наличие команды от USB
      if (new_data_received) {
          cycles_completed = 0;
          data_stored_in_flash = false;

          // Основной цикл обработки 10 циклов
          while (cycles_completed < CYCLES_NUMBER) {
              // Копируем исходные данные и добавляем шум
              CopyAndAddNoise(working_data, measurement_data);

              // ПРОВЕРКА НА ПОРОГ: если хотя бы одна точка превысила - переходим к следующему циклу
              if (!CheckThreshold(working_data)) {
                  SendUSBDebugMessage("Not written to FLASH: array exceeded threshold");
                  cycles_completed++;
                  continue; // Немедленно переходим к следующему циклу
              }

              // Если все точки прошли проверку - запись/усреднение во FLASH
              if (!data_stored_in_flash) {
                  // Первое сохранение во FLASH
                  SaveToFlash(working_data);
                  SendUSBDebugMessage("First written to FLASH");
                  data_stored_in_flash = true;
              } else {
                  // Усреднение с данными из FLASH
                  AverageWithFlashData(working_data);
                  SendUSBDebugMessage("Successfully written to FLASH with averaging");
              }

              cycles_completed++;
          }

          // После завершения всех 10 циклов - обработка финальных данных
          if (data_stored_in_flash) {
              ReadFromFlash(normalized_data);
              NormalizeData();
              CalculateAutocorrelation();
              CalculateAndSendThickness();
              PrintMeasurementDataToUSB();
          }

          new_data_received = 0; // Сбрасываем флаг
      }

      HAL_Delay(100);
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
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
