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
#include "adc.h"
#include "usart.h"
#include "rtc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t magic;         // Magic number for data validation
    uint32_t timestamp;     // Seconds since 2000-01-01 00:00:00
    uint32_t period_sec;    // Update period in seconds
} FlashCalendarData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Flash memory defines
#define FLASH_USER_START_ADDR   (FLASH_BASE + FLASH_SIZE - FLASH_PAGE_SIZE)
#define FLASH_USER_END_ADDR     (FLASH_BASE + FLASH_SIZE)
#define FLASH_DATA_MAGIC        0xABCD1234

// RTC backup register for initialization check
#define RTC_INIT_FLAG          RTC_BKP_DR0
#define RTC_INIT_VALUE         0x32F2

// LED defines
#define LED_PIN                GPIO_PIN_0
#define LED_PORT               GPIOB
#define LED_BLINK_COUNT        3
#define LED_BLINK_DELAY_MS     200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
FlashCalendarData calendar_data;
uint32_t last_update_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void save_to_flash(FlashCalendarData data);
static FlashCalendarData read_from_flash(void);
static void blink_led(void);
static uint32_t convert_to_timestamp(RTC_DateTypeDef date, RTC_TimeTypeDef time);
static void convert_from_timestamp(uint32_t timestamp, RTC_DateTypeDef* date, RTC_TimeTypeDef* time);
static bool is_rtc_initialized(void);
static void initialize_rtc(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Converts RTC date and time to timestamp (seconds since 2000-01-01)
  */
static uint32_t convert_to_timestamp(RTC_DateTypeDef date, RTC_TimeTypeDef time)
{
    return (date.Year * 365 * 24 * 3600) +
           ((date.Month-1) * 30 * 24 * 3600) +
           ((date.Date-1) * 24 * 3600) +
           (time.Hours * 3600) +
           (time.Minutes * 60) +
           time.Seconds;
}

/**
  * @brief  Converts timestamp to RTC date and time
  */
static void convert_from_timestamp(uint32_t timestamp, RTC_DateTypeDef* date, RTC_TimeTypeDef* time)
{
    time->Seconds = timestamp % 60;
    timestamp /= 60;
    time->Minutes = timestamp % 60;
    timestamp /= 60;
    time->Hours = timestamp % 24;
    timestamp /= 24;

    date->Date = (timestamp % 30) + 1;
    timestamp /= 30;
    date->Month = (timestamp % 12) + 1;
    timestamp /= 12;
    date->Year = timestamp;
    date->WeekDay = 1; // Monday
}

/**
  * @brief  Saves calendar data to flash memory
  */
static void save_to_flash(FlashCalendarData data)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Page = (FLASH_USER_START_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE,
        .NbPages = 1
    };

    uint32_t page_error;
    HAL_FLASHEx_Erase(&erase_init, &page_error);

    uint64_t *data_ptr = (uint64_t*)&data;
    uint32_t address = FLASH_USER_START_ADDR;
    uint32_t words_to_write = (sizeof(FlashCalendarData) + 7) / 8; // Round up to 8-byte words

    for(uint32_t i = 0; i < words_to_write; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, *data_ptr);
        address += 8;
        data_ptr++;
    }

    HAL_FLASH_Lock();
    blink_led(); // Мигаем после записи
}

/**
  * @brief  Reads calendar data from flash memory
  */
static FlashCalendarData read_from_flash(void)
{
    FlashCalendarData* data = (FlashCalendarData*)FLASH_USER_START_ADDR;
    if(data->magic != FLASH_DATA_MAGIC) {
        FlashCalendarData default_data = {
            .magic = FLASH_DATA_MAGIC,
            .timestamp = 0,
            .period_sec = 5
        };
        return default_data;
    }
    return *data;
}

/**
  * @brief  Blinks LED several times
  */
static void blink_led(void)
{
    for(int i = 0; i < LED_BLINK_COUNT; i++) {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        HAL_Delay(LED_BLINK_DELAY_MS);
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        HAL_Delay(LED_BLINK_DELAY_MS);
    }
}

/**
  * @brief  Checks if RTC is initialized
  */
static bool is_rtc_initialized(void)
{
    return (HAL_RTCEx_BKUPRead(&hrtc, RTC_INIT_FLAG) == RTC_INIT_VALUE);
}

/**
  * @brief  Initializes RTC with default values
  */
static void initialize_rtc(void)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours = 0;
    sTime.Minutes = 0;
    sTime.Seconds = 0;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    sDate.Year = 0;
    sDate.Month = 1;
    sDate.Date = 1;
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_INIT_FLAG, RTC_INIT_VALUE);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_LPUART1_UART_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

  if(!is_rtc_initialized()) {
      initialize_rtc();
  }

  calendar_data = read_from_flash();

  if(calendar_data.timestamp == 0) {
      RTC_TimeTypeDef sTime = {0};
      RTC_DateTypeDef sDate = {0};

      HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
      HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

      calendar_data.timestamp = convert_to_timestamp(sDate, sTime);
      calendar_data.period_sec = 5;
      calendar_data.magic = FLASH_DATA_MAGIC;

      save_to_flash(calendar_data);
  } else {
      RTC_TimeTypeDef sTime = {0};
      RTC_DateTypeDef sDate = {0};

      convert_from_timestamp(calendar_data.timestamp, &sDate, &sTime);
      HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
      HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  }

  last_update_tick = HAL_GetTick();

  char uart_buffer[64];
  /* USER CODE END 2 */

  while (1)
  {
	    if((HAL_GetTick() - last_update_tick) >= (calendar_data.period_sec * 1000)) {
	        last_update_tick = HAL_GetTick();

	        // Получаем текущие дату и время
	        RTC_DateTypeDef date;
	        RTC_TimeTypeDef time;
	        HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	        HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

	        // Обновляем данные в памяти
	        calendar_data.timestamp = convert_to_timestamp(date, time);
	        save_to_flash(calendar_data);  // Это вызовет мигание светодиода

	        // Формируем строку для отправки
	        char uart_msg[64];
	        snprintf(uart_msg, sizeof(uart_msg),
	               "DATE:%04d-%02d-%02d;TIME:%02d:%02d:%02d;PERIOD:%lu\r\n",
	               date.Year + 2000, date.Month, date.Date,
	               time.Hours, time.Minutes, time.Seconds,
	               calendar_data.period_sec);
	        // Отправляем по UART
	        HAL_UART_Transmit(&hlpuart1, (uint8_t*)uart_msg, strlen(uart_msg), 100);
	    }
  }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK) {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif /* USE_FULL_ASSERT */
