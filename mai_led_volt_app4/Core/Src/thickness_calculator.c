#include "thickness_calculator.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"
#include "kiss_fft.h"
#include <math.h>
// Определения
#define PARAMS_FLASH_ADDRESS 0x08080000
#define PARAMS_SET_SIZE (sizeof(Parameters_t))
#define TOTAL_FLASH_SIZE (PARAMS_SET_SIZE * NUM_PARAM_SETS)

// Глобальные переменные
Parameters_t params[NUM_PARAM_SETS];
TempParams_t temp_params;
bool parameters_initialized[NUM_PARAM_SETS] = {false};
bool calculate_thickness_requested = false;
float thickness_value = 0.0f;
float frequency_ns = 12.5e-9f;

// Буферы данных
// Новое объявление
float32_t* normalized_data = NULL;
float32_t* autocorrelation_result = NULL;
uint32_t processed_data_size = 0;

float32_t* new_final_data = NULL;
// float32_t new_final_data[DATA_SIZE] = {0};

// Внешние переменные
extern float averaged_fpga_data[DATA_SIZE];
extern bool averaging_complete;

// Вспомогательная функция для отправки сообщений
void SendUSBDebugMessage(const char *message);

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
    uint8_t* flash_address = (uint8_t*)PARAMS_FLASH_ADDRESS;

    for (uint8_t i = 0; i < NUM_PARAM_SETS; i++) {
        Parameters_t* flash_params = (Parameters_t*)(flash_address + (i * PARAMS_SET_SIZE));
        uint32_t calculated_crc = CalculateCRC32((uint8_t*)flash_params, sizeof(Parameters_t) - sizeof(uint32_t));

        if (flash_params->crc == calculated_crc) {
            params[i] = *flash_params;
            parameters_initialized[i] = true;
        } else {
            // Инициализируем конкретный набор по умолчанию
            InitializeParamSet(i);
        }
    }
}

/**
  * @brief Сохранение параметров в Flash память
  */
void SaveParametersToFlash(void) {
    HAL_FLASH_Unlock();

    // Рассчитываем CRC для всех наборов перед сохранением
    for (uint8_t i = 0; i < NUM_PARAM_SETS; i++) {
        params[i].crc = CalculateCRC32((uint8_t*)&params[i], sizeof(Parameters_t) - sizeof(uint32_t));
    }

    // Стираем сектор Flash
    FLASH_Erase_Sector(FLASH_SECTOR_11, VOLTAGE_RANGE_3);

    // Сохраняем все наборы параметров
    uint8_t* destination = (uint8_t*)PARAMS_FLASH_ADDRESS;

    for (uint8_t i = 0; i < NUM_PARAM_SETS; i++) {
        uint32_t* source = (uint32_t*)&params[i];
        uint32_t words = sizeof(Parameters_t) / sizeof(uint32_t);

        for (uint32_t j = 0; j < words; j++) {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)destination, *source);
            source++;
            destination += sizeof(uint32_t);
        }
    }

    HAL_FLASH_Lock();
}

/**
  * @brief Инициализация конкретного набора параметров значениями по умолчанию
  */
void InitializeParamSet(uint8_t set_number) {
    if (set_number >= NUM_PARAM_SETS) return;

    params[set_number].start_index = 100;
    params[set_number].wave_speed = 520.0f;
    params[set_number].first_left_strobe = 20;
    params[set_number].first_right_strobe = 120;
    params[set_number].second_left_strobe = 140;
    params[set_number].second_right_strobe = 240;
    params[set_number].threshold = 9999.0f;
    params[set_number].threshold_zero_crossing = 600.0f;
    params[set_number].probe_length = 400;
    params[set_number].method = set_number; // Разные методы по умолчанию для разных наборов
    params[set_number].end_index = 400;
    params[set_number].cycle_number = 10;
    strncpy(params[set_number].sensor_number, "0001", sizeof(params[set_number].sensor_number));
    params[set_number].gain = 1.0f;
    params[set_number].points_count = 5000; // Значение по умолчанию для points_count
    parameters_initialized[set_number] = true;
}

/**
  * @brief Инициализация всех параметров
  */
void InitializeParameters(void) {
    for (uint8_t i = 0; i < NUM_PARAM_SETS; i++) {
        InitializeParamSet(i);
    }
    SaveParametersToFlash();
}

/**
  * @brief Копирование данных
  */
void CopyDataWithoutModification(const float32_t* src, float32_t* dest, uint32_t dest_size) {
    uint32_t copy_size = (DATA_SIZE < dest_size) ? DATA_SIZE : dest_size;
    arm_copy_f32(src, dest, copy_size);

    for (uint32_t i = copy_size; i < dest_size; i++) {
        dest[i] = 0.0f;
    }
}

/**
  * @brief Нормализация данных
  */
void NormalizeData(uint8_t param_set) {
    if (!parameters_initialized[param_set] || normalized_data == NULL || processed_data_size == 0) return;

    float32_t mean, std_dev;
    float32_t* subtracted_mean = (float32_t*)malloc(processed_data_size * sizeof(float32_t));

    if (subtracted_mean == NULL) return;

    arm_mean_f32(normalized_data, processed_data_size, &mean);
    arm_offset_f32(normalized_data, -mean, subtracted_mean, processed_data_size);
    arm_std_f32(subtracted_mean, processed_data_size, &std_dev);

    if (std_dev != 0.0f) {
        arm_scale_f32(subtracted_mean, 1.0f/std_dev, normalized_data, processed_data_size);
    } else {
        arm_copy_f32(subtracted_mean, normalized_data, processed_data_size);
    }

    free(subtracted_mean);
}

/**
  * @brief Расчет автокорреляционной функции
  */
void CalculateAutocorrelation(uint8_t param_set) {
    if (!parameters_initialized[param_set] || normalized_data == NULL ||
        autocorrelation_result == NULL || processed_data_size == 0) return;

    for (uint32_t lag = 0; lag < processed_data_size; lag++) {
        double sum = 0.0;
        uint32_t count = processed_data_size - lag;

        for (uint32_t i = 0; i < count; i++) {
            sum += (double)normalized_data[i] * (double)normalized_data[i + lag];
        }

        autocorrelation_result[lag] = (float32_t)fabs(sum / count);
    }
}

/**
  * @brief Поиск индекса максимальной автокорреляции
  */
uint32_t FindMaxAutocorrelationIndex(uint8_t param_set) {
    if (!parameters_initialized[param_set] || autocorrelation_result == NULL) {
        return 0;
    }

    float32_t max_value = 0.0f;
    uint32_t max_idx = params[param_set].start_index;

    // Проверяем границы
    uint32_t search_start = params[param_set].start_index;
    uint32_t search_end = DATA_SIZE - params[param_set].end_index - params[param_set].probe_length;

    if (search_end > processed_data_size) {
        search_end = processed_data_size;
    }

    if (search_start >= search_end) {
        return search_start;
    }

    for (uint32_t i = search_start; i < search_end; i++) {
        if (autocorrelation_result[i] > max_value) {
            max_value = autocorrelation_result[i];
            max_idx = i;
        }
    }

    return max_idx;
}

/**
  * @brief Очистка динамически выделенной памяти
  */
void CleanupDynamicBuffers(void) {
    if (normalized_data != NULL) {
        free(normalized_data);
        normalized_data = NULL;
    }
    if (autocorrelation_result != NULL) {
        free(autocorrelation_result);
        autocorrelation_result = NULL;
    }
    processed_data_size = 0;
}

/**
  * @brief Расчет толщины методом перехода через ноль
  */
void CalculateZeroCrossingThickness(const float32_t* data, uint8_t param_set) {
    if (!parameters_initialized[param_set]) {
        return;
    }

    float one_point_mm = 1.0f / (params[param_set].wave_speed * 1000.0f * frequency_ns);
    uint32_t first_above_threshold_index = 0;
    uint32_t zero_crossing_index = 0;
    bool found_threshold = false;

    for (uint32_t i = params[param_set].probe_length; i < DATA_SIZE; i++) {
        if (data[i] >= params[param_set].threshold_zero_crossing) {
            first_above_threshold_index = i;
            found_threshold = true;
            break;
        }
    }

    if (!found_threshold) {
        thickness_value = 0.0f;
        return;
    }

    bool sign_positive = (data[first_above_threshold_index] >= 0);
    for (uint32_t i = first_above_threshold_index + 1; i < DATA_SIZE; i++) {
        bool current_sign_positive = (data[i] >= 0);
        if (current_sign_positive != sign_positive) {
            zero_crossing_index = i;
            break;
        }
    }

    if (zero_crossing_index == 0) {
        thickness_value = 0.0f;
        return;
    }

    uint32_t final_index = zero_crossing_index;
    thickness_value = final_index / (2.0f * one_point_mm);
}

/**
  * @brief Расчет толщины методом по стробам
  */
void CalculateStrobeThickness(const float32_t* data, uint8_t param_set) {
    if (!parameters_initialized[param_set]) return;

    // Проверка параметров
    if (params[param_set].wave_speed <= 0 || frequency_ns <= 0) {
        thickness_value = 0.0f;
        return;
    }

    float one_point_mm = 1.0f / (params[param_set].wave_speed * 1000.0f * frequency_ns);
    if (one_point_mm <= 0) {
        thickness_value = 0.0f;
        return;
    }

    // Проверка границ стробов
    uint32_t left1 = params[param_set].first_left_strobe;
    uint32_t right1 = params[param_set].first_right_strobe;
    uint32_t left2 = params[param_set].second_left_strobe;
    uint32_t right2 = params[param_set].second_right_strobe;

    if (left1 > right1 || left2 > right2 ||
        right1 >= DATA_SIZE || right2 >= DATA_SIZE) {
        thickness_value = 0.0f;
        return;
    }

    // Поиск максимумов с защитой от пустых данных
    float32_t max1 = data[left1];
    uint32_t idx1 = left1;
    for (uint32_t i = left1 + 1; i <= right1; i++) {
        if (data[i] > max1) {
            max1 = data[i];
            idx1 = i;
        }
    }

    float32_t max2 = data[left2];
    uint32_t idx2 = left2;
    for (uint32_t i = left2 + 1; i <= right2; i++) {
        if (data[i] > max2) {
            max2 = data[i];
            idx2 = i;
        }
    }

    // Проверка разницы индексов
    if (idx2 <= idx1) {
        thickness_value = 0.0f;
        return;
    }

    uint32_t diff = idx2 - idx1;
    thickness_value = diff / (2.0f * one_point_mm);
}
void PrintAutocorrelationToUSB(int param_index) {
    if (autocorrelation_result == NULL || processed_data_size == 0) return;

    char usb_msg[128];

    // Заголовок с информацией о корреляции
    snprintf(usb_msg, sizeof(usb_msg), "Averaged ACF Data for params[%d] [%lu cycles, 0-%d]:\r\n",
             param_index, processed_data_size);
    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
    HAL_Delay(10);

    // Данные корреляционной функции
    char data_line[256] = "";
    for (uint32_t i = 0; i < processed_data_size; i++) {
        char val_str[16];
        // АБСОЛЮТНЫЕ значения уже сохранены в autocorrelation_result
        snprintf(val_str, sizeof(val_str), "%8.4f ", autocorrelation_result[i]);
        strncat(data_line, val_str, sizeof(data_line) - strlen(data_line) - 1);

        // Перенос строки каждые 8 значений
        if ((i+1) % 8 == 0 || i == processed_data_size-1) {
            strncat(data_line, "\r\n", sizeof(data_line) - strlen(data_line) - 1);
            CDC_Transmit_FS((uint8_t*)data_line, strlen(data_line));
            HAL_Delay(10);
            data_line[0] = '\0';
        }
    }

    // Пустая строка в конце
    CDC_Transmit_FS((uint8_t*)"\r\n", 2);
}
/**
  * @brief Расчет толщины автокорреляционным методом
  */
void CalculateAndSendACFThickness(uint8_t param_set) {
    if (!parameters_initialized[param_set]) {
        return;
    }

    float one_point_mm = 1.0f / (params[param_set].wave_speed * 1000.0f * frequency_ns);
    uint32_t max_index = FindMaxAutocorrelationIndex(param_set);
    thickness_value = max_index / (2.0f * one_point_mm);
}

void CalculateAutocorrelation2(uint8_t param_set) {

	}
void ProcessDataByMethod(uint8_t param_set) {
    // Очистка перед началом новой обработки
    CleanupDynamicBuffers();
    if (param_set >= NUM_PARAM_SETS || !parameters_initialized[param_set]) {
        return;
    }

    if (!averaging_complete) {
        thickness_value = 0.0f;
        return;
    }

    // Освобождаем предыдущие буферы, если они были выделены
    if (normalized_data != NULL) {
        free(normalized_data);
        normalized_data = NULL;
    }
    if (autocorrelation_result != NULL) {
        free(autocorrelation_result);
        autocorrelation_result = NULL;
    }

    // Рассчитываем размер данных для обработки
    processed_data_size = params[param_set].end_index;
    if (processed_data_size <= 0 || processed_data_size > DATA_SIZE) {
        thickness_value = 0.0f;
        return;
    }

    // Выделяем память для буферов
    normalized_data = (float32_t*)malloc(processed_data_size * sizeof(float32_t));
    autocorrelation_result = (float32_t*)malloc((processed_data_size) * sizeof(float32_t));

    if (normalized_data == NULL || autocorrelation_result == NULL) {
        // Освобождаем в случае ошибки
        if (normalized_data != NULL) free(normalized_data);
        if (autocorrelation_result != NULL) free(autocorrelation_result);
        thickness_value = 0.0f;
        return;
    }

    // Инициализируем буферы нулями
    memset(normalized_data, 0, processed_data_size * sizeof(float32_t));
    memset(autocorrelation_result, 0, processed_data_size * sizeof(float32_t));
    memset(new_final_data, 0, params[param_set].points_count * sizeof(float32_t));

    // Копируем данные из усредненного массива FPGA
    uint32_t copy_size = (DATA_SIZE < DATA_SIZE) ? DATA_SIZE : DATA_SIZE;
    for (uint32_t i = 0; i < copy_size; i++) {
        new_final_data[i] = averaged_fpga_data[i];
    }

    for (uint32_t i = copy_size; i < DATA_SIZE; i++) {
        new_final_data[i] = 0.0f;
    }

    // Обрабатываем данные в зависимости от выбранного метода
    switch (params[param_set].method) {
        case 0:
            // Автокорреляционный метод
            // Копируем только нужную часть данных
            for (uint32_t i = 0; i < processed_data_size; i++) {
                normalized_data[i] = new_final_data[i + params[param_set].probe_length];
            }
            NormalizeData(param_set);
            CalculateAutocorrelation(param_set);
          //  PrintAutocorrelationToUSB(param_set);
            CalculateAndSendACFThickness(param_set);

            break;

        case 1:
            // Метод перехода через ноль
            CalculateZeroCrossingThickness(new_final_data, param_set);
            break;

        case 2:
            // Метод стробов
            CalculateStrobeThickness(new_final_data, param_set);
            break;

        default:
            break;
    }
}
/**
  * @brief Функция для получения активного набора параметров (по умолчанию первый)
  */
uint8_t GetActiveParamSet(void) {
    return 0; // Всегда используем первый набор параметров как указано в задании
}

