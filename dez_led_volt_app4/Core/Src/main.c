/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for Duty MCU with accurate date handling
  ******************************************************************************
  * @attention
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
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
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint32_t magic;
    uint32_t timestamp;  // Unix timestamp (seconds since 1970-01-01)
    uint32_t period_sec;
    uint8_t data_valid;
} FlashCalendarData;

/* Private defines -----------------------------------------------------------*/
#define FLASH_USER_START_ADDR   (FLASH_BASE + FLASH_SIZE - FLASH_PAGE_SIZE)
#define FLASH_DATA_MAGIC        0xABCD1234
#define DATA_VALID_FLAG         0xAA

#define RTC_INIT_FLAG          RTC_BKP_DR0
#define RTC_INIT_VALUE         0x32F2

#define LED_PIN                GPIO_PIN_0
#define LED_PORT               GPIOB
#define LED_BLINK_COUNT        3
#define LED_BLINK_DELAY_MS     200

#define UART_RX_BUF_SIZE       128
#define UART_TIMEOUT_MS        100

/* Private variables ---------------------------------------------------------*/
FlashCalendarData calendar_data;
uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
uint8_t uart_rx_pos = 0;
volatile uint8_t uart_cmd_ready = 0;
uint32_t uart_last_rx_time = 0;
uint32_t period_counter = 0;
uint32_t last_rtc_seconds = 0;
bool data_received_from_main = false;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static uint32_t convert_to_timestamp(RTC_DateTypeDef date, RTC_TimeTypeDef time);
static void convert_from_timestamp(uint32_t timestamp, RTC_DateTypeDef* date, RTC_TimeTypeDef* time);
static bool is_data_valid(FlashCalendarData* data);
static void save_to_flash(FlashCalendarData data);
static FlashCalendarData read_from_flash(void);
static void blink_led(void);
static bool is_rtc_initialized(void);
static void initialize_rtc(void);
static void process_uart_command(uint8_t* data, uint8_t len);
static void send_current_datetime(void);
static void send_datetime_with_voltage_and_temp(void);
static void update_calendar(void);
static uint8_t days_in_month(uint8_t month, uint8_t year);
static float read_voltage(void);
static float read_temperature(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Get days in month (considering leap years)
  */
static uint8_t days_in_month(uint8_t month, uint8_t year) {
    const uint8_t days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if(month == 2 && ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)) {
        return 29;
    }
    return days[month-1];
}

/**
  * @brief Read voltage from ADC
  */
static float read_voltage(void)
{
    uint32_t adcValue = 0;
    float voltage = 0.0f;
    ADC_ChannelConfTypeDef sConfig = {0};

    // Конфигурация канала ADC
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        return 0.0f;
    }

    // Запуск преобразования ADC
    if (HAL_ADC_Start(&hadc1) == HAL_OK)
    {
        // Ожидание завершения преобразования
        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        {
            adcValue = HAL_ADC_GetValue(&hadc1);
            voltage = (float)adcValue * 3.3f / 4095.0f;
        }
        HAL_ADC_Stop(&hadc1);
    }

    return voltage;
}

/**
  * @brief Read temperature from internal sensor
  */
static float read_temperature(void)
{
    uint32_t adcValue = 0;
    float temperature = 0.0f;
    ADC_ChannelConfTypeDef sConfig = {0};

    // Конфигурация канала температурного датчика
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        return 0.0f;
    }

    // Калибровка ADC
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return 0.0f;
    }

    // Запуск преобразования ADC
    if (HAL_ADC_Start(&hadc1) == HAL_OK)
    {
        // Ожидание завершения преобразования
        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        {
            adcValue = HAL_ADC_GetValue(&hadc1);

            // Калибровочные значения для STM32L5
            uint32_t raw_ts_cal1 = *((uint16_t*)0x0BFA05A8); // Калибровка при 30°C
            uint32_t raw_ts_cal2 = *((uint16_t*)0x0BFA05CA); // Калибровка при 130°C

            // Расчет температуры по калибровочным значениям
            temperature = 30.0f + ((float)adcValue - (float)raw_ts_cal1) *
                         (100.0f / ((float)raw_ts_cal2 - (float)raw_ts_cal1));
        }
        HAL_ADC_Stop(&hadc1);
    }

    return temperature;
}

/**
  * @brief Convert RTC date/time to Unix timestamp (accurate)
  */
static uint32_t convert_to_timestamp(RTC_DateTypeDef date, RTC_TimeTypeDef time)
{
    struct tm tm_time = {
        .tm_sec = time.Seconds,
        .tm_min = time.Minutes,
        .tm_hour = time.Hours,
        .tm_mday = date.Date,
        .tm_mon = date.Month - 1,
        .tm_year = date.Year + 100,  // STM32 RTC year is offset from 2000 (2000=0)
        .tm_isdst = -1
    };

    return mktime(&tm_time);
}

/**
  * @brief Convert Unix timestamp to RTC date/time
  */
static void convert_from_timestamp(uint32_t timestamp, RTC_DateTypeDef* date, RTC_TimeTypeDef* time)
{
    struct tm *tm_time;
    time_t t = timestamp;
    tm_time = localtime(&t);

    time->Seconds = tm_time->tm_sec;
    time->Minutes = tm_time->tm_min;
    time->Hours = tm_time->tm_hour;

    date->Date = tm_time->tm_mday;
    date->Month = tm_time->tm_mon + 1;
    date->Year = tm_time->tm_year - 100;  // Convert back to STM32 RTC format
    date->WeekDay = tm_time->tm_wday;
}

static bool is_data_valid(FlashCalendarData* data)
{
    return (data->magic == FLASH_DATA_MAGIC &&
            data->data_valid == DATA_VALID_FLAG &&
            data->period_sec > 0);
}

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

    uint32_t address = FLASH_USER_START_ADDR;
    uint64_t data64 = ((uint64_t)data.magic << 32) | data.magic;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data64);
    address += 8;

    data64 = ((uint64_t)data.timestamp << 32) | data.timestamp;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data64);
    address += 8;

    data64 = ((uint64_t)data.period_sec << 32) | data.period_sec;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data64);
    address += 8;

    data64 = ((uint64_t)data.data_valid << 32) | data.data_valid;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data64);

    HAL_FLASH_Lock();
    calendar_data = data;
    data_received_from_main = true;
    blink_led();
}

static FlashCalendarData read_from_flash(void)
{
    FlashCalendarData* data = (FlashCalendarData*)FLASH_USER_START_ADDR;
    if(is_data_valid(data)) {
        return *data;
    }

    FlashCalendarData default_data = {
        .magic = FLASH_DATA_MAGIC,
        .timestamp = 0,
        .period_sec = 0,
        .data_valid = 0
    };
    return default_data;
}

static void blink_led(void)
{
    for(int i = 0; i < LED_BLINK_COUNT; i++) {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        HAL_Delay(LED_BLINK_DELAY_MS);
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        HAL_Delay(LED_BLINK_DELAY_MS);
    }
}

static bool is_rtc_initialized(void)
{
    return (HAL_RTCEx_BKUPRead(&hrtc, RTC_INIT_FLAG) == RTC_INIT_VALUE);
}

static void initialize_rtc(void)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours = 0;
    sTime.Minutes = 0;
    sTime.Seconds = 0;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    sDate.Year = 0;  // 2000
    sDate.Month = 1;
    sDate.Date = 1;
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_INIT_FLAG, RTC_INIT_VALUE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == LPUART1) {
        uart_last_rx_time = HAL_GetTick();

        if(uart_rx_buf[uart_rx_pos] == '\n' || uart_rx_pos >= sizeof(uart_rx_buf)-1) {
            uart_cmd_ready = 1;
            uart_rx_buf[uart_rx_pos] = '\0';
            HAL_UART_AbortReceive_IT(&hlpuart1);
        } else {
            uart_rx_pos++;
            HAL_UART_Receive_IT(&hlpuart1, &uart_rx_buf[uart_rx_pos], 1);
        }
    }
}

static void process_uart_command(uint8_t* data, uint8_t len)
{
    char* date_ptr = strstr((char*)data, "DATE:");
    char* time_ptr = strstr((char*)data, ";TIME:");
    char* period_ptr = strstr((char*)data, ";PERIOD:");

    if(date_ptr && time_ptr && period_ptr) {
        RTC_DateTypeDef date;
        RTC_TimeTypeDef time;
        int year, month, day, hour, min, sec;
        uint32_t period;

        if(sscanf(date_ptr, "DATE:%d-%d-%d", &year, &month, &day) != 3) return;
        if(sscanf(time_ptr, ";TIME:%d:%d:%d", &hour, &min, &sec) != 3) return;
        if(sscanf(period_ptr, ";PERIOD:%lu", &period) != 1) return;

        date.Year = year - 2000;
        date.Month = month;
        date.Date = day;
        time.Hours = hour;
        time.Minutes = min;
        time.Seconds = sec;

        calendar_data.timestamp = convert_to_timestamp(date, time);
        calendar_data.period_sec = period;
        calendar_data.magic = FLASH_DATA_MAGIC;
        calendar_data.data_valid = DATA_VALID_FLAG;

        save_to_flash(calendar_data);

        HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
        HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);

        // Сразу отправляем подтверждение с текущими данными, напряжением и температурой
        send_datetime_with_voltage_and_temp();

        // Сбрасываем счетчик периода
        period_counter = 0;
    }

    uart_rx_pos = 0;
    memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
    HAL_UART_Receive_IT(&hlpuart1, &uart_rx_buf[uart_rx_pos], 1);
}

static void send_current_datetime(void)
{
    RTC_DateTypeDef date;
    RTC_TimeTypeDef time;
    char uart_msg[128];

    HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

    // Форматируем сообщение в том же формате, что ожидает основной МК
    snprintf(uart_msg, sizeof(uart_msg),
           "DATE:%04d-%02d-%02d;TIME:%02d:%02d:%02d;PERIOD:%lu\r\n",
           date.Year + 2000, date.Month, date.Date,
           time.Hours, time.Minutes, time.Seconds,
           calendar_data.period_sec);

    HAL_UART_Transmit(&hlpuart1, (uint8_t*)uart_msg, strlen(uart_msg), 100);

    // Мигаем LED для индикации отправки
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

static void send_datetime_with_voltage_and_temp(void)
{
    RTC_DateTypeDef date;
    RTC_TimeTypeDef time;
    float voltage = read_voltage(); // Измеряем напряжение
    float temperature = read_temperature(); // Измеряем температуру
    char uart_msg[128]; // Увеличили буфер для добавления температуры

    HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

    // Форматируем сообщение с добавлением напряжения и температуры
    snprintf(uart_msg, sizeof(uart_msg),
           "DATE:%04d-%02d-%02d;TIME:%02d:%02d:%02d;PERIOD:%lu;VOLTAGE:%.4f;TEMP:%.2f\r\n",
           date.Year + 2000, date.Month, date.Date,
           time.Hours, time.Minutes, time.Seconds,
           calendar_data.period_sec, voltage, temperature);

    HAL_UART_Transmit(&hlpuart1, (uint8_t*)uart_msg, strlen(uart_msg), 100);

    // Мигаем LED для индикации отправки
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

static void update_calendar(void)
{
    RTC_TimeTypeDef current_time;
    HAL_RTC_GetTime(&hrtc, &current_time, RTC_FORMAT_BIN);

    if(current_time.Seconds != last_rtc_seconds) {
        last_rtc_seconds = current_time.Seconds;

        if(data_received_from_main && calendar_data.period_sec > 0) {
            period_counter++;

            if(period_counter >= calendar_data.period_sec) {
                period_counter = 0;

                // Обновляем timestamp на величину периода
                calendar_data.timestamp += calendar_data.period_sec;

                // Обновляем RTC
                RTC_DateTypeDef date;
                RTC_TimeTypeDef time;
                convert_from_timestamp(calendar_data.timestamp, &date, &time);
                HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
                HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);

                // Сохраняем обновленные данные во FLASH
                save_to_flash(calendar_data);

                // ОТПРАВЛЯЕМ обновленные данные основному МК с напряжением и температурой
                send_datetime_with_voltage_and_temp();
            }
        }
    }
}

/* Main function -------------------------------------------------------------*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_LPUART1_UART_Init();
    MX_RTC_Init();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

    // Включаем транзистор для измерения напряжения
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    if(!is_rtc_initialized()) {
        initialize_rtc();
    }

    calendar_data = read_from_flash();
    data_received_from_main = is_data_valid(&calendar_data);

    if(data_received_from_main) {
        RTC_TimeTypeDef sTime = {0};
        RTC_DateTypeDef sDate = {0};

        convert_from_timestamp(calendar_data.timestamp, &sDate, &sTime);
        HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

        // Отправляем текущие данные при старте (если есть валидные данные)
        send_datetime_with_voltage_and_temp(); // Используем функцию с измерением напряжения и температуры
    }

    uart_last_rx_time = HAL_GetTick();
    last_rtc_seconds = 0;
    period_counter = 0;

    HAL_UART_Receive_IT(&hlpuart1, &uart_rx_buf[uart_rx_pos], 1);

    while (1)
    {
        if(uart_cmd_ready) {
            uart_cmd_ready = 0;
            process_uart_command(uart_rx_buf, uart_rx_pos);
        }

        if(uart_rx_pos > 0 && (HAL_GetTick() - uart_last_rx_time) > UART_TIMEOUT_MS) {
            uart_rx_pos = 0;
            memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
            HAL_UART_Receive_IT(&hlpuart1, &uart_rx_buf[uart_rx_pos], 1);
        }

        if(data_received_from_main) {
            update_calendar();
        }

        HAL_Delay(10);
    }
}
/* System Clock Configuration */
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
    while (1) {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(100);
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    while (1) {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        HAL_Delay(1000);
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        HAL_Delay(1000);
    }
}
#endif /* USE_FULL_ASSERT */
