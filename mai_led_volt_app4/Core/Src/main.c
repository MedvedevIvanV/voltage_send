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
  * Also includes DAC voltage control functionality with Flash storage.
  *
  * Connections:
  * - FSMC_D[15:0] to FPGA data bus
  * - FSMC_NOE to FPGA_OE (read enable)
  * - FSMC_NWE to FPGA_WE (write enable, not used in this case)
  * - FSMC_NE1 to FPGA chip select
  * - PD6 to FPGA START_FPGA signal
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
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FPGA_FIFO_SIZE 100       // Размер FIFO в ПЛИС (должен соответствовать коду ПЛИС)
#define VALUES_PER_LINE 10       // Количество значений в строке вывода

typedef struct {
    uint16_t data[FPGA_FIFO_SIZE]; // Буфер для хранения данных от ПЛИС
    bool data_ready;               // Флаг готовности данных
    uint8_t data_count;            // Количество считанных данных
} FPGA_Data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FPGA_BASE_ADDRESS 0x60000000  // Базовый адрес Bank1 (NE1)
#define START_PULSE_DURATION_MS 1     // Длительность импульса START в мс
#define MEASUREMENT_INTERVAL_MS 5000  // Интервал между измерениями в мс

// Адрес во Flash-памяти для хранения напряжения
#define VOLTAGE_STORAGE_ADDR 0x080E0000
// Сектор Flash-памяти для хранения напряжения
#define FLASH_SECTOR FLASH_SECTOR_11
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
uint32_t last_measurement_time = 0;  // Время последнего измерения
volatile uint16_t *fpga_reg;         // Указатель на регистр ПЛИС

// DAC control variables
float dac_voltage = 0.0f;            // Текущее напряжение на DAC
uint32_t dac_last_update = 0;        // Время последнего обновления напряжения
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
static void MX_FSMC_Init(void);
/* USER CODE BEGIN PFP */
void GenerateStartPulse(void);
void ReadFPGAData(void);
void PrintDataToUSB(void);
void SendUSBDebugMessage(const char *message);

// DAC control functions
HAL_StatusTypeDef Write_Voltage_To_Flash(float voltage);
float Read_Voltage_From_Flash(void);
void Set_DAC_Voltage(float voltage);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief Генерация импульса START для запуска измерения в ПЛИС
  * @note Импульс длительностью 1 мс на выводе PD6 (START_FPGA)
  */
void GenerateStartPulse(void) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(START_PULSE_DURATION_MS);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
}

/**
  * @brief Чтение данных из ПЛИС через FSMC интерфейс
  * @note Читает всю FIFO из ПЛИС (100 значений по 12 бит)
  */
void ReadFPGAData(void) {
    fpga_data.data_count = 0;
    uint16_t test_value = fpga_reg[0]; // Читаем первое значение для теста

    snprintf(usb_msg, sizeof(usb_msg), "First read value: 0x%04X", test_value);
    SendUSBDebugMessage(usb_msg);

    for (int i = 0; i < FPGA_FIFO_SIZE; i++) {
        uint16_t value = fpga_reg[i];

        snprintf(usb_msg, sizeof(usb_msg), "Read[%d]: 0x%04X", i, value);
        SendUSBDebugMessage(usb_msg);

        if ((value & 0x0FFF) != 0x0FFF) {
            fpga_data.data[i] = value & 0x0FFF;
            fpga_data.data_count++;
        } else {
            break;
        }
    }
    fpga_data.data_ready = (fpga_data.data_count > 0);
}

/**
  * @brief Вывод данных через USB CDC
  */
void PrintDataToUSB(void) {
    if (!fpga_data.data_ready) return;

    for (int i = 0; i < fpga_data.data_count; i += VALUES_PER_LINE) {
        int start_idx = i;
        int end_idx = (i + VALUES_PER_LINE - 1) < fpga_data.data_count ?
                      (i + VALUES_PER_LINE - 1) : (fpga_data.data_count - 1);

        // Формируем строку с данными
        snprintf(usb_msg, sizeof(usb_msg), "Data [%03d-%03d]: ", start_idx, end_idx);

        // Добавляем значения в строку
        for (int j = i; j <= end_idx; j++) {
            char val_str[8];
            snprintf(val_str, sizeof(val_str), "%4d ", fpga_data.data[j]);
            strncat(usb_msg, val_str, sizeof(usb_msg) - strlen(usb_msg) - 1);
        }

        strncat(usb_msg, "\r\n", sizeof(usb_msg) - strlen(usb_msg) - 1);
        CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
        HAL_Delay(10); // Задержка для стабильной работы USB
    }
}

/**
  * @brief Отправка отладочного сообщения через USB
  * @param message Текст сообщения
  */
void SendUSBDebugMessage(const char *message) {
    snprintf(usb_msg, sizeof(usb_msg), "[%lu] %s\r\n", HAL_GetTick(), message);
    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
}

/**
  * @brief Записывает значение напряжения во Flash-память
  * @param voltage: Значение напряжения для сохранения
  * @retval Статус операции (HAL_OK в случае успеха)
  */
HAL_StatusTypeDef Write_Voltage_To_Flash(float voltage) {
    HAL_StatusTypeDef status;
    // Преобразуем float в uint32_t для записи
    uint32_t voltageData = *(uint32_t*)&voltage;

    // Разблокируем Flash для записи
    HAL_FLASH_Unlock();
    // Очищаем флаги ошибок
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                          FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    // Настраиваем структуру для стирания сектора
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = FLASH_SECTOR;
    EraseInitStruct.NbSectors = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t SectorError;
    // Стираем сектор перед записью
    status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    // Записываем значение напряжения
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, VOLTAGE_STORAGE_ADDR, voltageData);
    // Ожидаем завершения операции
    FLASH_WaitForLastOperation(100);
    // Блокируем Flash
    HAL_FLASH_Lock();
    return status;
}

/**
  * @brief Читает значение напряжения из Flash-памяти
  * @retval Прочитанное значение напряжения (3.3V по умолчанию, если данные невалидны)
  */
float Read_Voltage_From_Flash() {
    // Читаем данные как 32-битное целое
    uint32_t voltageData = *(__IO uint32_t*)(VOLTAGE_STORAGE_ADDR);
    // Проверяем, не является ли значение "пустым" (0xFFFFFFFF)
    if (voltageData == 0xFFFFFFFF) {
        return 3.3f;  // Возвращаем значение по умолчанию
    }
    // Преобразуем обратно в float и возвращаем
    return *(float*)&voltageData;
}

/**
  * @brief Устанавливает напряжение на DAC
  * @param voltage: Значение напряжения для установки (от 0.0 до 3.3)
  */
void Set_DAC_Voltage(float voltage) {
    // Ограничиваем значение напряжения в допустимом диапазоне
    if (voltage < 0) voltage = 0;
    if (voltage > 3.3f) voltage = 3.3f;

    // Преобразуем напряжение в 12-битное значение DAC
    uint32_t dac_value = (voltage / 3.3f) * 4095;

    // Устанавливаем значение на DAC
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
    // Включаем DAC
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

    // Сохраняем текущее значение напряжения
    dac_voltage = voltage;

    // Сохраняем значение в Flash (по желанию)
    // Write_Voltage_To_Flash(voltage);
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

    // Инициализация указателя на регистр ПЛИС
    fpga_reg = (volatile uint16_t *)FPGA_BASE_ADDRESS;

    // Инициализация структуры данных
    memset(&fpga_data, 0, sizeof(fpga_data));

    // Инициализация DAC с начальным напряжением 0V
    Set_DAC_Voltage(0.0f);

    // Сообщение о готовности системы
    SendUSBDebugMessage("System initialized. Ready to communicate with FPGA...");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
        uint32_t current_time = HAL_GetTick();

        // Циклическое изменение напряжения DAC каждые 3 секунды
        if (current_time - dac_last_update > 3000) {
            dac_last_update = current_time;

            if (dac_voltage < 0.1f) {          // Если ~0V
                Set_DAC_Voltage(0.300f);       // Устанавливаем точно 0.3V
            }
            else if (dac_voltage < 0.6f) {     // Если ~0.3V
                Set_DAC_Voltage(1.000f);       // Устанавливаем точно 1.0V
            }
            else {                             // Если ~1.0V
                Set_DAC_Voltage(0.000f);       // Сбрасываем в 0V
            }
        }

        // Проверяем, прошло ли достаточно времени с последнего измерения
        if ((current_time - last_measurement_time) >= MEASUREMENT_INTERVAL_MS) {
            SendUSBDebugMessage("Starting new measurement cycle...");

            // Генерируем стартовый импульс для ПЛИС
            GenerateStartPulse();
            SendUSBDebugMessage("START pulse sent to FPGA");

            // Ждем пока ПЛИС заполнит FIFO (задержка должна соответствовать коду ПЛИС)
            HAL_Delay(100);

            // Читаем данные из ПЛИС
            ReadFPGAData();

            // Выводим результаты
            if (fpga_data.data_ready) {
                SendUSBDebugMessage("Data received from FPGA:");
                PrintDataToUSB();
                fpga_data.data_ready = false;
            } else {
                SendUSBDebugMessage("No data received from FPGA!");
            }

            last_measurement_time = current_time;
        }

        HAL_Delay(100); // Небольшая задержка для снижения нагрузки на CPU
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
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
