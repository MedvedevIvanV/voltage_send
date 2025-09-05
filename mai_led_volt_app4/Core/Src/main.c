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
#define USB_RX_BUFFER_SIZE 300
#define FINAL_DATA_SIZE 10000
#define PARAMS_FLASH_ADDRESS 0x08080000  // Адрес во Flash памяти для хранения параметров

#define UART_RX_TIMEOUT_MS 100
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
char usb_msg[1024];                   // Буфер для USB сообщений

// Добавляем объявления переменных из usbd_cdc_if.c
extern volatile uint8_t usb_rx_buffer[USB_RX_BUFFER_SIZE];
extern volatile uint8_t new_data_received;
extern volatile uint16_t usb_rx_index;

#define VALUES_PER_LINE 10
#define DATA_VALUES_COUNT 4600

extern const float measurement_data[];

float32_t normalized_data[DATA_VALUES_COUNT];
float32_t autocorrelation_result[DATA_VALUES_COUNT];
float32_t mean, std_dev;

arm_status status;
float frequency = 12.5e-9;           // Частота в секундах (12.5 нс)

float one_point_mm;                  // Разрешение одного отсчета в мм
uint32_t max_index;                  // Индекс максимального значения АКФ

// Структура для хранения параметров в энергонезависимой памяти
typedef struct {
    uint32_t start_index;
    float wave_speed;
    uint32_t first_left_strobe;
    uint32_t first_right_strobe;
    uint32_t second_left_strobe;
    uint32_t second_right_strobe;
    float threshold;
    float threshold_zero_crossing;
    uint32_t probe_length;
    uint32_t method;
    uint32_t end_index;
    uint32_t cycle_number;
    char sensor_number[5];           // 4 символа + нулевой терминатор
    float gain;
    uint32_t crc; // Контрольная сумма для проверки целостности данных
} Parameters_t;

Parameters_t params; // Глобальная структура параметров
bool parameters_initialized = false;
bool calculate_thickness_requested = false;

// Переменные для временного хранения start_date и period (НЕ сохраняются во Flash)
char start_date[20] = {0}; // Формат: "2024-01-15 14:30:25"
uint32_t period = 0;

// Буферы для обработки данных
float32_t temp_data[FINAL_DATA_SIZE]; // Временный буфер для обработки
float32_t final_data[FINAL_DATA_SIZE]; // Финальный буфер для усредненных данных
uint32_t successful_cycles = 0; // Счетчик успешных циклов

// Добавляем переменные для UART передачи
#define UART_TX_BUF_SIZE 128
char uart_tx_buf[UART_TX_BUF_SIZE];



#define UART_RX_BUF_SIZE 128
char uart_rx_buf[UART_RX_BUF_SIZE];
uint8_t uart_rx_pos = 0;
volatile uint8_t uart_cmd_ready = 0;
uint32_t uart_last_rx_time = 0;


volatile uint8_t uart_rx_data[UART_RX_BUF_SIZE];
volatile uint8_t uart_rx_len = 0;
volatile uint8_t uart_message_received = 0;


float thermocouple_temperature = 0.0f;
bool thermocouple_error = false;

float thickness_value = 0.0f;  // Глобальная переменная для хранения толщины
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
void CalculateZeroCrossingThickness(const float32_t* data);
void CalculateStrobeThickness(const float32_t* data);
void ParseParameters(const char* params_str);
void SendParametersResponse(void);
void InitializeParameters(void);
void SaveParametersToFlash(void);
void LoadParametersFromFlash(void);
uint32_t CalculateCRC32(const uint8_t *data, size_t length);
void AddRandomNoiseAndExtend(const float32_t* src, float32_t* dest, uint32_t dest_size);
bool ProcessCycle(uint32_t cycle_num);
void NormalizeData(void);
void CalculateAutocorrelation(void);
uint32_t FindMaxAutocorrelationIndex(void);
void CalculateAndSendACFThickness(void);
void PrintMeasurementDataToUSB(void);
bool CheckThreshold(const float32_t* data, uint32_t size);
void ProcessDataByMethod(void);
void SendDateTimeToBackupMCU(void);
// НОВЫЙ КОД ДЛЯ ПРИЕМА ОТ ДЕЖУРНОГО МК - ДОБАВЛЯЕМ СЮДА
void ProcessUARTCommand(uint8_t* data, uint8_t len);

uint16_t Read_Thermocouple_Temperature(void);
float Get_Thermocouple_Temperature(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief Расчет CRC32 для проверки целостности данных
  */
uint32_t CalculateCRC32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return ~crc;
}

/**
  * @brief Загрузка параметров из Flash памяти
  */
void LoadParametersFromFlash(void) {
    Parameters_t* flash_params = (Parameters_t*)PARAMS_FLASH_ADDRESS;

    // Проверяем контрольную сумму
    uint32_t calculated_crc = CalculateCRC32((uint8_t*)flash_params, sizeof(Parameters_t) - sizeof(uint32_t));

    if (flash_params->crc == calculated_crc) {
        // Данные корректны, загружаем их
        params = *flash_params;
        parameters_initialized = true;
        SendUSBDebugMessage("Parameters loaded from Flash memory");
    } else {
        // Данные повреждены, используем значения по умолчанию
        InitializeParameters();
        SendUSBDebugMessage("Flash data corrupted, using default parameters");
    }
}

/**
  * @brief Сохранение параметров в Flash память
  */
void SaveParametersToFlash(void) {
    HAL_FLASH_Unlock();

    // Рассчитываем контрольную сумму
    params.crc = CalculateCRC32((uint8_t*)&params, sizeof(Parameters_t) - sizeof(uint32_t));

    // Стираем страницу Flash
    FLASH_Erase_Sector(FLASH_SECTOR_11, VOLTAGE_RANGE_3);

    // Записываем данные
    uint32_t* source = (uint32_t*)&params;
    uint32_t* destination = (uint32_t*)PARAMS_FLASH_ADDRESS;
    uint32_t words = sizeof(Parameters_t) / sizeof(uint32_t);

    for (uint32_t i = 0; i < words; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)destination, *source);
        source++;
        destination++;
    }

    HAL_FLASH_Lock();
    SendUSBDebugMessage("Parameters saved to Flash memory");
}

/**
  * @brief Инициализация параметров значениями по умолчанию
  */
void InitializeParameters(void) {
    params.start_index = 100;
    params.wave_speed = 3200.0f;
    params.first_left_strobe = 20;
    params.first_right_strobe = 120;
    params.second_left_strobe = 140;
    params.second_right_strobe = 240;
    params.threshold = 2080.0f;
    params.threshold_zero_crossing = 600.0f;
    params.probe_length = 400;
    params.method = 1;
    params.end_index = 400;
    params.cycle_number = 10;
    strncpy(params.sensor_number, "0001", sizeof(params.sensor_number));
    params.gain = 1.0f;

    parameters_initialized = true;
    SaveParametersToFlash(); // Сохраняем значения по умолчанию
    SendUSBDebugMessage("Parameters initialized with defaults and saved to Flash");
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
  * @brief Парсинг параметров из строки
  * @param params_str Строка с параметрами (после "SETPARAMS=")
  */
void ParseParameters(const char* params_str) {
    char buffer[USB_RX_BUFFER_SIZE];
    strncpy(buffer, params_str, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    // Загружаем текущие параметры (если еще не инициализированы)
    if (!parameters_initialized) {
        LoadParametersFromFlash();
    }

    char* token = strtok(buffer, "|");

    while (token != NULL) {
        char* equals_sign = strchr(token, '=');
        if (equals_sign != NULL) {
            *equals_sign = '\0'; // Разделяем на имя и значение
            char* param_name = token;
            char* param_value = equals_sign + 1;

            // Парсим параметры
            if (strcmp(param_name, "wave_speed") == 0) {
                params.wave_speed = atof(param_value);
            } else if (strcmp(param_name, "threshold") == 0) {
                params.threshold = atof(param_value);
            } else if (strcmp(param_name, "threshold_zero_crossing") == 0) {
                params.threshold_zero_crossing = atof(param_value);
            } else if (strcmp(param_name, "start_index") == 0) {
                params.start_index = atoi(param_value);
            } else if (strcmp(param_name, "probe_length") == 0) {
                params.probe_length = atoi(param_value);
            } else if (strcmp(param_name, "strobe_left1") == 0) {
                params.first_left_strobe = atoi(param_value);
            } else if (strcmp(param_name, "strobe_right1") == 0) {
                params.first_right_strobe = atoi(param_value);
            } else if (strcmp(param_name, "strobe_left2") == 0) {
                params.second_left_strobe = atoi(param_value);
            } else if (strcmp(param_name, "strobe_right2") == 0) {
                params.second_right_strobe = atoi(param_value);
            } else if (strcmp(param_name, "method") == 0) {
                params.method = atoi(param_value);
            } else if (strcmp(param_name, "end_index") == 0) {
                params.end_index = atoi(param_value);
            } else if (strcmp(param_name, "cycle_number") == 0) {
                params.cycle_number = atoi(param_value);
            } else if (strcmp(param_name, "sensor_number") == 0) {
                strncpy(params.sensor_number, param_value, sizeof(params.sensor_number) - 1);
                params.sensor_number[sizeof(params.sensor_number) - 1] = '\0';
            } else if (strcmp(param_name, "gain") == 0) {
                params.gain = atof(param_value);
            } else if (strcmp(param_name, "start_date") == 0) {
                // Сохраняем start_date во временную переменную (НЕ во Flash)
                strncpy(start_date, param_value, sizeof(start_date) - 1);
                start_date[sizeof(start_date) - 1] = '\0';
                SendUSBDebugMessage("Start date parsed (not saved to Flash)");
            } else if (strcmp(param_name, "period") == 0) {
                // Сохраняем period во временную переменную (НЕ во Flash)
                period = atoi(param_value);
                snprintf(usb_msg, sizeof(usb_msg), "Period parsed: %lu (not saved to Flash)", period);
                SendUSBDebugMessage(usb_msg);
            }
        }
        token = strtok(NULL, "|");
    }

    // Сохраняем обновленные параметры в Flash (без start_date и period)
    SaveParametersToFlash();

    // ОТПРАВЛЯЕМ ДАННЫЕ НА ДЕЖУРНЫЙ МК ПО UART
    SendDateTimeToBackupMCU();

    // Устанавливаем флаг для запуска расчета
    calculate_thickness_requested = true;
    SendUSBDebugMessage("Parameters parsed and saved successfully - calculation requested");
}

/**
  * @brief Отправка текущих параметров обратно в приложение
  */
void SendParametersResponse(void) {
    if (!parameters_initialized) {
        SendUSBDebugMessage("Parameters not initialized yet");
        return;
    }

    snprintf(usb_msg, sizeof(usb_msg),
        "wave_speed=%.1f|threshold=%.1f|threshold_zero_crossing=%.1f|"
        "start_index=%lu|probe_length=%lu|strobe_left1=%lu|strobe_right1=%lu|"
        "strobe_left2=%lu|strobe_right2=%lu|method=%lu|end_index=%lu|cycle_number=%lu|"
        "sensor_number=%s|gain=%.1f|start_date=%s|period=%lu\r\n",
        params.wave_speed, params.threshold, params.threshold_zero_crossing,
        params.start_index, params.probe_length, params.first_left_strobe, params.first_right_strobe,
        params.second_left_strobe, params.second_right_strobe, params.method, params.end_index, params.cycle_number,
        params.sensor_number, params.gain, start_date, period);

    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
    HAL_Delay(10);
}

/**
  * @brief Генерация стартового импульса для ПЛИС
  * @note Импульс длительностью 200 нс на пине PD6
  */
void GenerateStartPulse(void) {
    // Устанавливаем высокий уровень на PD6
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);

    // Задержка для формирования импульса 200 нс
    for(volatile int i = 0; i < 34; i++);

    // Устанавливаем низкий уровень на PD6
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);

    // Загружаем параметры из памяти и запускаем расчет
    if (!parameters_initialized) {
        LoadParametersFromFlash();
    }
    calculate_thickness_requested = true;

    SendUSBDebugMessage("Start pulse generated and calculation requested");
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

void NormalizeData(void) {
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

void CalculateAutocorrelation(void) {
    for (uint32_t lag = 0; lag < DATA_VALUES_COUNT; lag++) {
        double sum = 0.0;
        uint32_t count = DATA_VALUES_COUNT - lag;

        for (uint32_t i = 0; i < count; i++) {
            sum += (double)normalized_data[i] * (double)normalized_data[i + lag];
        }

        autocorrelation_result[lag] = (float32_t)fabs(sum / count);
    }
}

uint32_t FindMaxAutocorrelationIndex(void) {
    if (!parameters_initialized) {
        SendUSBDebugMessage("Parameters not initialized for ACF");
        return 0;
    }

    float32_t max_value = 0.0f;
    uint32_t max_idx = params.start_index;

    // Используем end_index для ограничения диапазона поиска
    uint32_t search_end = (params.end_index < DATA_VALUES_COUNT) ? params.end_index : DATA_VALUES_COUNT - 1;

    for (uint32_t i = params.start_index; i < search_end; i++) {
        if (autocorrelation_result[i] > max_value) {
            max_value = autocorrelation_result[i];
            max_idx = i;
        }
    }

    return max_idx;
}

/**
  * @brief Расчет толщины методом перехода через ноль
  */
void CalculateZeroCrossingThickness(const float32_t* data) {
    if (!parameters_initialized) {
        SendUSBDebugMessage("Parameters not initialized for zero crossing");
        return;
    }

    // Вычисляем one_point_mm для методов 1 и 2
    one_point_mm = 1.0f / (params.wave_speed * 1000.0f * frequency);

    uint32_t first_above_threshold_index = 0;
    uint32_t zero_crossing_index = 0;
    bool found_threshold = false;

    for (uint32_t i = 0; i < FINAL_DATA_SIZE; i++) {
        if (data[i] >= params.threshold_zero_crossing) {
            first_above_threshold_index = i;
            found_threshold = true;
            break;
        }
    }

    if (!found_threshold) {
        SendUSBDebugMessage("Zero crossing: threshold not found");
        thickness_value = 0.0f; // Сбрасываем толщину при ошибке
        return;
    }

    bool sign_positive = (data[first_above_threshold_index] >= 0);
    for (uint32_t i = first_above_threshold_index + 1; i < FINAL_DATA_SIZE; i++) {
        bool current_sign_positive = (data[i] >= 0);
        if (current_sign_positive != sign_positive) {
            zero_crossing_index = i;
            break;
        }
    }

    if (zero_crossing_index == 0) {
        SendUSBDebugMessage("Zero crossing: zero crossing not found");
        thickness_value = 0.0f; // Сбрасываем толщину при ошибке
        return;
    }

    uint32_t final_index = zero_crossing_index + params.probe_length;
    thickness_value = final_index / (2.0f * one_point_mm); // Сохраняем толщину

    snprintf(usb_msg, sizeof(usb_msg), "Zero_crossing:%.3f\r\n", thickness_value);
    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
    HAL_Delay(10);
}



/**
  * @brief Расчет толщины методом по стробам
  */
void CalculateStrobeThickness(const float32_t* data) {
    if (!parameters_initialized) {
        SendUSBDebugMessage("Parameters not initialized for strobe method");
        return;
    }

    // Вычисляем one_point_mm для методов 1 и 2
    one_point_mm = 1.0f / (params.wave_speed * 1000.0f * frequency);

    float32_t max_value_first = -FLT_MAX;
    uint32_t max_index_first = params.first_left_strobe;
    float32_t max_value_second = -FLT_MAX;
    uint32_t max_index_second = params.second_left_strobe;

    for (uint32_t i = params.first_left_strobe; i <= params.first_right_strobe; i++) {
        if (i < FINAL_DATA_SIZE && data[i] > max_value_first) {
            max_value_first = data[i];
            max_index_first = i;
        }
    }

    for (uint32_t i = params.second_left_strobe; i <= params.second_right_strobe; i++) {
        if (i < FINAL_DATA_SIZE && data[i] > max_value_second) {
            max_value_second = data[i];
            max_index_second = i;
        }
    }

    if (max_value_first == -FLT_MAX || max_value_second == -FLT_MAX) {
        SendUSBDebugMessage("Strobe method: max values not found");
        thickness_value = 0.0f; // Сбрасываем толщину при ошибке
        return;
    }

    uint32_t index_difference = max_index_second - max_index_first;
    thickness_value = index_difference / (2.0f * one_point_mm); // Сохраняем толщину

    snprintf(usb_msg, sizeof(usb_msg), "Strobe:%.3f\r\n", thickness_value);
    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
    HAL_Delay(10);
}
/**
  * @brief Расчет толщины автокорреляционным методом и отправка результата по USB
  */
void CalculateAndSendACFThickness(void) {
    if (!parameters_initialized) {
        SendUSBDebugMessage("Parameters not initialized for ACF thickness calculation");
        return;
    }

    one_point_mm = 1.0f / (params.wave_speed * 1000.0f * frequency);
    max_index = FindMaxAutocorrelationIndex();
    thickness_value = max_index / (2.0f * one_point_mm);  // Сохраняем толщину в глобальную переменную

    snprintf(usb_msg, sizeof(usb_msg), "ACF:%.3f\r\n", thickness_value);
    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
    HAL_Delay(10);
}

void PrintMeasurementDataToUSB(void) {
    snprintf(usb_msg, sizeof(usb_msg), "Autocorrelation Result [0-%d]:\r\n", DATA_VALUES_COUNT-1);
    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
    HAL_Delay(10);

    char data_line[128] = "";
    for (int i = 0; i < DATA_VALUES_COUNT; i++) {
        char val_str[16];
        snprintf(val_str, sizeof(val_str), "%7.2f ", autocorrelation_result[i]);
        strncat(data_line, val_str, sizeof(data_line) - strlen(data_line) - 1);

        if ((i+1) % VALUES_PER_LINE == 0 || i == DATA_VALUES_COUNT-1) {
            strncat(data_line, "\r\n", sizeof(data_line) - strlen(data_line) - 1);
            CDC_Transmit_FS((uint8_t*)data_line, strlen(data_line));
            HAL_Delay(10);
            data_line[0] = '\0';
        }
    }
}

/**
  * @brief Проверка данных на превышение threshold
  * @return true если данные НЕ превышают порог, false если превышают
  */
bool CheckThreshold(const float32_t* data, uint32_t size) {
    if (!parameters_initialized) {
        SendUSBDebugMessage("Threshold parameter not initialized");
        return false; // Не пропускаем обработку если параметры не инициализированы
    }

    for (uint32_t i = 0; i < size; i++) {
        if (fabsf(data[i]) > params.threshold) {
            return false; // Обнаружено превышение порога
        }
    }
    return true; // Все данные ниже порога
}

/**
  * @brief Добавление случайного шума и расширение массива до 10000 точек
  */
void AddRandomNoiseAndExtend(const float32_t* src, float32_t* dest, uint32_t dest_size) {
    // Копируем исходные данные с небольшим случайным шумом
    for (uint32_t i = 0; i < DATA_VALUES_COUNT; i++) {
        float noise = (rand() % 100 - 50) / 100.0f; // Случайный шум ±0.5
        dest[i] = src[i] + noise;
    }

    // Дополняем массив до 10000 точек случайными колебаниям
    for (uint32_t i = DATA_VALUES_COUNT; i < dest_size; i++) {
        dest[i] = (rand() % 2000 - 1000) / 10.0f; // Случайные значения в диапазоне ±100
    }
}

/**
  * @brief Обработка одного цикла
  * @param cycle_num Номер текущего цикла
  * @return true если цикл прошел проверку порога, false если нет
  */
bool ProcessCycle(uint32_t cycle_num) {
    // Добавляем шум и расширяем данные
    AddRandomNoiseAndExtend(measurement_data, temp_data, FINAL_DATA_SIZE);

    // Применяем коэффициент усиления
    if (params.gain != 1.0f) {
        arm_scale_f32(temp_data, params.gain, temp_data, FINAL_DATA_SIZE);
    }

    // Проверяем порог - функция возвращает true если НЕ превысило порог
    bool below_threshold = CheckThreshold(temp_data, FINAL_DATA_SIZE);

    // Отправляем статус цикла по USB (английский текст)
    if (below_threshold) {
        snprintf(usb_msg, sizeof(usb_msg), "Cycle:%lu - threshold NOT exceeded, data added to averaged array\r\n", cycle_num);

        if (successful_cycles == 0) {
            // Первый успешный цикл - просто копируем данные
            arm_copy_f32(temp_data, final_data, FINAL_DATA_SIZE);
        } else {
            // Усредняем с предыдущими успешными данными
            for (uint32_t i = 0; i < FINAL_DATA_SIZE; i++) {
                final_data[i] = (final_data[i] * successful_cycles + temp_data[i]) / (successful_cycles + 1);
            }
        }
        successful_cycles++;
    } else {
        snprintf(usb_msg, sizeof(usb_msg), "Cycle:%lu - THRESHOLD EXCEEDED! Array skipped\r\n", cycle_num);
    }

    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
    HAL_Delay(10);

    return below_threshold;
}

/**
  * @brief Обработка данных в зависимости от выбранного метода
  */
void ProcessDataByMethod(void) {
    if (!parameters_initialized) {
        SendUSBDebugMessage("Parameters not initialized");
        return;
    }

    // Добавляем информацию о сенсоре в отладочное сообщение
    snprintf(usb_msg, sizeof(usb_msg), "Processing data for sensor: %s, gain: %.1f", params.sensor_number, params.gain);
    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
    HAL_Delay(10);

    // Сбрасываем счетчик успешных циклов
    successful_cycles = 0;

    // Выполняем указанное количество циклов
    for (uint32_t cycle = 1; cycle <= params.cycle_number; cycle++) {
        ProcessCycle(cycle);
        HAL_Delay(10); // Небольшая задержка между циклами
    }

    if (successful_cycles == 0) {
        SendUSBDebugMessage("Ни один цикл не прошел проверку порога");
        return;
    }

    SendUSBDebugMessage("Усреднение завершено, начинаем расчет толщины");

    // Используем усредненные данные для расчета толщины
    switch (params.method) {
        case 0: // Автокорреляционный метод
            // Копируем и нормализуем данные для АКФ
            arm_copy_f32(final_data, normalized_data, DATA_VALUES_COUNT);
            NormalizeData();
            CalculateAutocorrelation();
            CalculateAndSendACFThickness();
            PrintMeasurementDataToUSB();
            break;

        case 1: // Только метод перехода через ноль
            CalculateZeroCrossingThickness(final_data);
            break;

        case 2: // Только метод по стробам
            CalculateStrobeThickness(final_data);
            break;

        default:
            SendUSBDebugMessage("Unknown method specified");
            break;
    }
}




/**
  * @brief Отправка даты/времени и периода на дежурный МК по UART
  */
void SendDateTimeToBackupMCU(void) {
    // Проверяем что данные не пустые
    if (strlen(start_date) > 0 && period > 0) {
        // Формируем сообщение в формате: "DATE:YYYY-MM-DD;TIME:HH:MM:SS;PERIOD:XXXXX"
        snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                 "DATE:%.10s;TIME:%.8s;PERIOD:%lu\r\n",
                 start_date, start_date + 11, period);

        // Отправляем по UART
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx_buf, strlen(uart_tx_buf), 100);

        // Отладочное сообщение по USB
        snprintf(usb_msg, sizeof(usb_msg), "Sent to backup MCU: %s", uart_tx_buf);
        SendUSBDebugMessage(usb_msg);
    } else {
        SendUSBDebugMessage("No date/time data to send to backup MCU");
    }
}





/**
  * @brief Обработка команды от дежурного МК через UART
  * @param data Данные для обработки
  * @param len Длина данных
  */
/**
  * @brief Обработка команды от дежурного МК через UART
  * @param data Данные для обработки
  * @param len Длина данных
  */
void ProcessUARTCommand(uint8_t* data, uint8_t len) {
    char* date_ptr = strstr((char*)data, "DATE:");
    char* time_ptr = strstr((char*)data, ";TIME:");
    char* period_ptr = strstr((char*)data, ";PERIOD:");
    char* voltage_ptr = strstr((char*)data, ";VOLTAGE:");
    char* temp_ptr = strstr((char*)data, ";TEMP:");

    // Новый формат с напряжением и температурой
    if(date_ptr && time_ptr && period_ptr && voltage_ptr && temp_ptr) {
        int year, month, day, hour, min, sec;
        uint32_t received_period;
        float received_voltage, received_temp;

        // Парсим дату, время, период, напряжение и температуру
        if(sscanf(date_ptr, "DATE:%d-%d-%d", &year, &month, &day) == 3 &&
           sscanf(time_ptr, ";TIME:%d:%d:%d", &hour, &min, &sec) == 3 &&
           sscanf(period_ptr, ";PERIOD:%lu", &received_period) == 1 &&
           sscanf(voltage_ptr, ";VOLTAGE:%f", &received_voltage) == 1 &&
           sscanf(temp_ptr, ";TEMP:%f", &received_temp) == 1) {

            // Формируем строку даты в формате "YYYY-MM-DD HH:MM:SS"
            snprintf(start_date, sizeof(start_date), "%04d-%02d-%02d %02d:%02d:%02d",
                    year, month, day, hour, min, sec);
            period = received_period;

            // ИЗМЕРЯЕМ ТЕМПЕРАТУРУ ТЕРМОПАРЫ
            thermocouple_temperature = Get_Thermocouple_Temperature();

            // ВЫЧИСЛЯЕМ ТОЛЩИНУ (если еще не вычислена)
            if (thickness_value == 0.0f && parameters_initialized) {
                // Запускаем расчет толщины
                calculate_thickness_requested = true;

                // Ждем завершения расчета
                uint32_t start_time = HAL_GetTick();
                while (calculate_thickness_requested && (HAL_GetTick() - start_time) < 5000) {
                    // Обрабатываем данные
                    if (calculate_thickness_requested) {
                        calculate_thickness_requested = false;
                        ProcessDataByMethod();
                    }
                    HAL_Delay(10);
                }
            }

            // ОТПРАВЛЯЕМ РАСШИРЕННЫЕ ДАННЫЕ ПО USB
            if(thermocouple_error) {
                snprintf(usb_msg, sizeof(usb_msg),
                        "%s|%lu|%.4f|%.2f|ERROR|%.3f|%.1f|%.1f|%.1f|%lu|%lu|%lu|%lu|%lu|%lu|%lu|%lu|%s|%.1f|%s|%lu\r\n",
                        start_date, period, received_voltage, received_temp, thickness_value,
                        params.wave_speed, params.threshold, params.threshold_zero_crossing,
                        params.start_index, params.probe_length, params.first_left_strobe,
                        params.first_right_strobe, params.second_left_strobe, params.second_right_strobe,
                        params.method, params.end_index, params.cycle_number, params.sensor_number,
                        params.gain, start_date, period);
            } else {
                snprintf(usb_msg, sizeof(usb_msg),
                        "%s|%lu|%.4f|%.2f|%.2f|%.3f|%.1f|%.1f|%.1f|%lu|%lu|%lu|%lu|%lu|%lu|%lu|%lu|%s|%.1f|%s|%lu\r\n",
                        start_date, period, received_voltage, received_temp, thermocouple_temperature,
                        thickness_value, params.wave_speed, params.threshold, params.threshold_zero_crossing,
                        params.start_index, params.probe_length, params.first_left_strobe,
                        params.first_right_strobe, params.second_left_strobe, params.second_right_strobe,
                        params.method, params.end_index, params.cycle_number, params.sensor_number,
                        params.gain, start_date, period);
            }

            CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
            HAL_Delay(10);

            SendUSBDebugMessage("Extended data sent via USB");
        }
    }
    // Формат с напряжением (без температуры)
    else if(date_ptr && time_ptr && period_ptr && voltage_ptr) {
        int year, month, day, hour, min, sec;
        uint32_t received_period;
        float received_voltage;

        // Парсим дату, время, период и напряжение
        if(sscanf(date_ptr, "DATE:%d-%d-%d", &year, &month, &day) == 3 &&
           sscanf(time_ptr, ";TIME:%d:%d:%d", &hour, &min, &sec) == 3 &&
           sscanf(period_ptr, ";PERIOD:%lu", &received_period) == 1 &&
           sscanf(voltage_ptr, ";VOLTAGE:%f", &received_voltage) == 1) {

            // Формируем строку даты в формате "YYYY-MM-DD HH:MM:SS"
            snprintf(start_date, sizeof(start_date), "%04d-%02d-%02d %02d:%02d:%02d",
                    year, month, day, hour, min, sec);
            period = received_period;

            // ИЗМЕРЯЕМ ТЕМПЕРАТУРУ ТЕРМОПАРЫ
            thermocouple_temperature = Get_Thermocouple_Temperature();

            // ВЫЧИСЛЯЕМ ТОЛЩИНУ (если еще не вычислена)
            if (thickness_value == 0.0f && parameters_initialized) {
                // Запускаем расчет толщины
                calculate_thickness_requested = true;

                // Ждем завершения расчета
                uint32_t start_time = HAL_GetTick();
                while (calculate_thickness_requested && (HAL_GetTick() - start_time) < 5000) {
                    // Обрабатываем данные
                    if (calculate_thickness_requested) {
                        calculate_thickness_requested = false;
                        ProcessDataByMethod();
                    }
                    HAL_Delay(10);
                }
            }

            // ОТПРАВЛЯЕМ РАСШИРЕННЫЕ ДАННЫЕ ПО USB
            if(thermocouple_error) {
                snprintf(usb_msg, sizeof(usb_msg),
                        "%s|%lu|%.4f|ERROR|%.3f|%.1f|%.1f|%.1f|%lu|%lu|%lu|%lu|%lu|%lu|%lu|%lu|%s|%.1f|%s|%lu\r\n",
                        start_date, period, received_voltage, thickness_value,
                        params.wave_speed, params.threshold, params.threshold_zero_crossing,
                        params.start_index, params.probe_length, params.first_left_strobe,
                        params.first_right_strobe, params.second_left_strobe, params.second_right_strobe,
                        params.method, params.end_index, params.cycle_number, params.sensor_number,
                        params.gain, start_date, period);
            } else {
                snprintf(usb_msg, sizeof(usb_msg),
                        "%s|%lu|%.4f|%.2f|%.3f|%.1f|%.1f|%.1f|%lu|%lu|%lu|%lu|%lu|%lu|%lu|%lu|%s|%.1f|%s|%lu\r\n",
                        start_date, period, received_voltage, thermocouple_temperature, thickness_value,
                        params.wave_speed, params.threshold, params.threshold_zero_crossing,
                        params.start_index, params.probe_length, params.first_left_strobe,
                        params.first_right_strobe, params.second_left_strobe, params.second_right_strobe,
                        params.method, params.end_index, params.cycle_number, params.sensor_number,
                        params.gain, start_date, period);
            }

            CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
            HAL_Delay(10);

            SendUSBDebugMessage("Extended data sent via USB");
        }
    }
    // Старый формат (только дата, время и период)
    else if(date_ptr && time_ptr && period_ptr) {
        int year, month, day, hour, min, sec;
        uint32_t received_period;

        // Парсим дату и время
        if(sscanf(date_ptr, "DATE:%d-%d-%d", &year, &month, &day) == 3 &&
           sscanf(time_ptr, ";TIME:%d:%d:%d", &hour, &min, &sec) == 3 &&
           sscanf(period_ptr, ";PERIOD:%lu", &received_period) == 1) {

            // Формируем строку даты в формате "YYYY-MM-DD HH:MM:SS"
            snprintf(start_date, sizeof(start_date), "%04d-%02d-%02d %02d:%02d:%02d",
                    year, month, day, hour, min, sec);
            period = received_period;

            // ИЗМЕРЯЕМ ТЕМПЕРАТУРУ ТЕРМОПАРЫ
            thermocouple_temperature = Get_Thermocouple_Temperature();

            // ВЫЧИСЛЯЕМ ТОЛЩИНУ (если еще не вычислена)
            if (thickness_value == 0.0f && parameters_initialized) {
                // Запускаем расчет толщины
                calculate_thickness_requested = true;

                // Ждем завершения расчета
                uint32_t start_time = HAL_GetTick();
                while (calculate_thickness_requested && (HAL_GetTick() - start_time) < 5000) {
                    // Обрабатываем данные
                    if (calculate_thickness_requested) {
                        calculate_thickness_requested = false;
                        ProcessDataByMethod();
                    }
                    HAL_Delay(10);
                }
            }

            // ОТПРАВЛЯЕМ РАСШИРЕННЫЕ ДАННЫЕ ПО USB
            if(thermocouple_error) {
                snprintf(usb_msg, sizeof(usb_msg),
                        "%s|%lu|ERROR|%.3f|%.1f|%.1f|%.1f|%lu|%lu|%lu|%lu|%lu|%lu|%lu|%lu|%s|%.1f|%s|%lu\r\n",
                        start_date, period, thickness_value,
                        params.wave_speed, params.threshold, params.threshold_zero_crossing,
                        params.start_index, params.probe_length, params.first_left_strobe,
                        params.first_right_strobe, params.second_left_strobe, params.second_right_strobe,
                        params.method, params.end_index, params.cycle_number, params.sensor_number,
                        params.gain, start_date, period);
            } else {
                snprintf(usb_msg, sizeof(usb_msg),
                        "%s|%lu|%.2f|%.3f|%.1f|%.1f|%.1f|%lu|%lu|%lu|%lu|%lu|%lu|%lu|%lu|%s|%.1f|%s|%lu\r\n",
                        start_date, period, thermocouple_temperature, thickness_value,
                        params.wave_speed, params.threshold, params.threshold_zero_crossing,
                        params.start_index, params.probe_length, params.first_left_strobe,
                        params.first_right_strobe, params.second_left_strobe, params.second_right_strobe,
                        params.method, params.end_index, params.cycle_number, params.sensor_number,
                        params.gain, start_date, period);
            }

            CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
            HAL_Delay(10);

            SendUSBDebugMessage("Extended data sent via USB");
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        uart_last_rx_time = HAL_GetTick();

        if(uart_rx_buf[uart_rx_pos] == '\n' || uart_rx_pos >= UART_RX_BUF_SIZE-1) {
            // Копируем данные в буфер для обработки
            memcpy((void*)uart_rx_data, uart_rx_buf, uart_rx_pos);
            uart_rx_len = uart_rx_pos;
            uart_message_received = 1;

            uart_rx_pos = 0;
            memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
        } else {
            uart_rx_pos++;
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart_rx_buf[uart_rx_pos], 1);
    }
}



uint16_t Read_Thermocouple_Temperature(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint16_t raw_data = 0;

    // Настройка PC11 (DATA) как входа
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Активация чипа (активный низкий уровень на PC8 - термопара CS)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

    // Задержка для стабилизации (минимум 100ns по даташиту)
    for(volatile int i = 0; i < 10; i++);

    // Чтение 16 бит данных
    for(uint8_t i = 0; i < 16; i++) {
        // Генерация тактового импульса (PC10 - DCLK)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
        for(volatile int j = 0; j < 5; j++); // Короткая задержка

        // Чтение бита данных (MSB first)
        if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11)) {
            raw_data |= (1 << (15 - i));
        }

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
        for(volatile int j = 0; j < 5; j++); // Короткая задержка
    }

    // Деактивация чипа
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

    return raw_data;
}

/**
  * @brief Получение температуры термопары в градусах Цельсия
  */
float Get_Thermocouple_Temperature(void) {
    uint16_t raw_data = Read_Thermocouple_Temperature();

    // Проверка на разомкнутую цепь (бит D2)
    if(raw_data & 0x04) {
        thermocouple_error = true;
        return -999.0f; // Код ошибки
    }

    thermocouple_error = false;

    // Извлечение 12-битного значения температуры (биты D14-D3)
    raw_data >>= 3; // Убираем 3 младших бита (D2-D0)
    raw_data &= 0x0FFF; // Оставляем только 12 бит температуры

    // Конверсия в градусы (каждый LSB = 0.25°C)
    return (float)raw_data * 0.25f;
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
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_FSMC_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  srand(HAL_GetTick());

  HAL_UART_Receive_IT(&huart1, (uint8_t*)uart_rx_buf, 1);


  // Загружаем параметры из энергонезависимой памяти при старте
  HAL_Delay(1000);
  LoadParametersFromFlash();
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  // Обработка USB команд
	      if (new_data_received) {
	          if (strncmp((char*)usb_rx_buffer, "SETPARAMS=", 10) == 0) {
	              ParseParameters((char*)usb_rx_buffer + 10);
	              SendParametersResponse();
	          }
	          else if (strncmp((char*)usb_rx_buffer, "1", 1) == 0) {
	              ProcessUSBCommand('1');
	          }
	          memset((void*)usb_rx_buffer, 0, USB_RX_BUFFER_SIZE);
	          usb_rx_index = 0;
	          new_data_received = 0;
	      }

	      // Обработка UART от дежурного МК
	      if(uart_message_received) {
	          uart_message_received = 0;
	          ProcessUARTCommand((uint8_t*)uart_rx_data, uart_rx_len);
	      }

	      // Таймаут UART приема
	      if(uart_rx_pos > 0 && (HAL_GetTick() - uart_last_rx_time) > UART_RX_TIMEOUT_MS) {
	          uart_rx_pos = 0;
	          memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
	          HAL_UART_Receive_IT(&huart1, (uint8_t*)uart_rx_buf, 1);
	      }

	      // Проверяем, нужно ли выполнить расчет толщин
	      if (calculate_thickness_requested && parameters_initialized) {
	          calculate_thickness_requested = false;
	          ProcessDataByMethod();
	          SendUSBDebugMessage("Thickness calculation completed");
	      }

	      HAL_Delay(10);
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
