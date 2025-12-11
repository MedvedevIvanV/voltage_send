#include "thickness_calculator.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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
float32_t normalized_data[DATA_VALUES_COUNT] = {0};
float32_t autocorrelation_result[DATA_VALUES_COUNT] = {0};
float32_t final_data[FINAL_DATA_SIZE] = {0};

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
  * @brief Копирование данных без добавления шума и расширения
  */
void CopyDataWithoutModification(const float32_t* src, float32_t* dest, uint32_t dest_size) {
    uint32_t copy_size = (DATA_VALUES_COUNT < dest_size) ? DATA_VALUES_COUNT : dest_size;
    arm_copy_f32(src, dest, copy_size);

    for (uint32_t i = copy_size; i < dest_size; i++) {
        dest[i] = 0.0f;
    }
}

/**
  * @brief Нормализация данных
  */
void NormalizeData(uint8_t param_set) {
    if (!parameters_initialized[param_set]) return;

    float32_t mean, std_dev;
    float32_t subtracted_mean[DATA_VALUES_COUNT];

    arm_mean_f32(normalized_data, DATA_VALUES_COUNT, &mean);
    arm_offset_f32(normalized_data, -mean, subtracted_mean, DATA_VALUES_COUNT);
    arm_std_f32(subtracted_mean, DATA_VALUES_COUNT, &std_dev);

    if (std_dev != 0.0f) {
        arm_scale_f32(subtracted_mean, 1.0f/std_dev, normalized_data, DATA_VALUES_COUNT);
    } else {
        arm_copy_f32(subtracted_mean, normalized_data, DATA_VALUES_COUNT);
    }
}

/**
  * @brief Расчет автокорреляционной функции
  */
void CalculateAutocorrelation(uint8_t param_set) {
    if (!parameters_initialized[param_set]) return;

    for (uint32_t lag = 0; lag < DATA_VALUES_COUNT; lag++) {
        double sum = 0.0;
        uint32_t count = DATA_VALUES_COUNT - lag;

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
    if (!parameters_initialized[param_set]) {
        return 0;
    }

    float32_t max_value = 0.0f;
    uint32_t max_idx = params[param_set].start_index;
    uint32_t search_end = (params[param_set].end_index < DATA_VALUES_COUNT) ? params[param_set].end_index : DATA_VALUES_COUNT - 1;

    for (uint32_t i = params[param_set].start_index; i < search_end; i++) {
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
void CalculateZeroCrossingThickness(const float32_t* data, uint8_t param_set) {
    if (!parameters_initialized[param_set]) {
        return;
    }

    float one_point_mm = 1.0f / (params[param_set].wave_speed * 1000.0f * frequency_ns);
    uint32_t first_above_threshold_index = 0;
    uint32_t zero_crossing_index = 0;
    bool found_threshold = false;

    for (uint32_t i = 0; i < FINAL_DATA_SIZE; i++) {
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
    for (uint32_t i = first_above_threshold_index + 1; i < FINAL_DATA_SIZE; i++) {
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

    uint32_t final_index = zero_crossing_index + params[param_set].probe_length;
    thickness_value = final_index / (2.0f * one_point_mm);
}

/**
  * @brief Расчет толщины методом по стробам
  */
void CalculateStrobeThickness(const float32_t* data, uint8_t param_set) {
    if (!parameters_initialized[param_set]) {
        return;
    }

    float one_point_mm = 1.0f / (params[param_set].wave_speed * 1000.0f * frequency_ns);
    float32_t max_value_first = -FLT_MAX;
    uint32_t max_index_first = params[param_set].first_left_strobe;
    float32_t max_value_second = -FLT_MAX;
    uint32_t max_index_second = params[param_set].second_left_strobe;

    for (uint32_t i = params[param_set].first_left_strobe; i <= params[param_set].first_right_strobe; i++) {
        if (i < FINAL_DATA_SIZE && data[i] > max_value_first) {
            max_value_first = data[i];
            max_index_first = i;
        }
    }

    for (uint32_t i = params[param_set].second_left_strobe; i <= params[param_set].second_right_strobe; i++) {
        if (i < FINAL_DATA_SIZE && data[i] > max_value_second) {
            max_value_second = data[i];
            max_index_second = i;
        }
    }

    if (max_value_first == -FLT_MAX || max_value_second == -FLT_MAX) {
        thickness_value = 0.0f;
        return;
    }

    uint32_t index_difference = max_index_second - max_index_first;
    thickness_value = index_difference / (2.0f * one_point_mm);
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

/**
  * @brief Обработка данных в зависимости от выбранного метода для конкретного набора параметров
  */
void ProcessDataByMethod(uint8_t param_set) {
    if (param_set >= NUM_PARAM_SETS || !parameters_initialized[param_set]) {
        return;
    }

    if (!averaging_complete) {
        thickness_value = 0.0f;
        return;
    }

    // Копируем данные из усредненного массива FPGA
    uint32_t copy_size = (DATA_SIZE < FINAL_DATA_SIZE) ? DATA_SIZE : FINAL_DATA_SIZE;
    for (uint32_t i = 0; i < copy_size; i++) {
        final_data[i] = averaged_fpga_data[i];
    }

    for (uint32_t i = copy_size; i < FINAL_DATA_SIZE; i++) {
        final_data[i] = 0.0f;
    }

    // Обрабатываем данные в зависимости от выбранного метода
    switch (params[param_set].method) {
        case 0:
            // Автокорреляционный метод
            arm_copy_f32(final_data, normalized_data, DATA_VALUES_COUNT);
            NormalizeData(param_set);
            CalculateAutocorrelation(param_set);
            CalculateAndSendACFThickness(param_set);
            break;

        case 1:
            // Метод перехода через ноль
            CalculateZeroCrossingThickness(final_data, param_set);
            break;

        case 2:
            // Метод стробов
            CalculateStrobeThickness(final_data, param_set);
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
