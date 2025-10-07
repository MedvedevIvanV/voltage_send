#include "thickness_calculator.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Определения
#define PARAMS_FLASH_ADDRESS 0x08080000

// Глобальные переменные
Parameters_t params;
bool parameters_initialized = false;
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
    Parameters_t* flash_params = (Parameters_t*)PARAMS_FLASH_ADDRESS;
    uint32_t calculated_crc = CalculateCRC32((uint8_t*)flash_params, sizeof(Parameters_t) - sizeof(uint32_t));

    if (flash_params->crc == calculated_crc) {
        params = *flash_params;
        parameters_initialized = true;
        SendUSBDebugMessage("Parameters loaded from Flash memory");
    } else {
        InitializeParameters();
        SendUSBDebugMessage("Flash data corrupted, using default parameters");
    }
}

/**
  * @brief Сохранение параметров в Flash память
  */
void SaveParametersToFlash(void) {
    HAL_FLASH_Unlock();
    params.crc = CalculateCRC32((uint8_t*)&params, sizeof(Parameters_t) - sizeof(uint32_t));

    FLASH_Erase_Sector(FLASH_SECTOR_11, VOLTAGE_RANGE_3);

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
    SaveParametersToFlash();
    SendUSBDebugMessage("Parameters initialized with defaults and saved to Flash");
}

/**
  * @brief Копирование данных без добавления шума и расширения
  */
void CopyDataWithoutModification(const float32_t* src, float32_t* dest, uint32_t dest_size) {
    // Копируем только существующие данные, не увеличивая длину массива
    uint32_t copy_size = (DATA_VALUES_COUNT < dest_size) ? DATA_VALUES_COUNT : dest_size;
    arm_copy_f32(src, dest, copy_size);

    // Остальные элементы заполняем нулями (если dest_size > DATA_VALUES_COUNT)
    for (uint32_t i = copy_size; i < dest_size; i++) {
        dest[i] = 0.0f;
    }
}

/**
  * @brief Нормализация данных
  */
void NormalizeData(void) {
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

/**
  * @brief Поиск индекса максимальной автокорреляции
  */
uint32_t FindMaxAutocorrelationIndex(void) {
    if (!parameters_initialized) {
        SendUSBDebugMessage("Parameters not initialized for ACF");
        return 0;
    }

    float32_t max_value = 0.0f;
    uint32_t max_idx = params.start_index;
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

    float one_point_mm = 1.0f / (params.wave_speed * 1000.0f * frequency_ns);
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
        SendUSBDebugMessage("Zero crossing: zero crossing not found");
        thickness_value = 0.0f;
        return;
    }

    uint32_t final_index = zero_crossing_index + params.probe_length;
    thickness_value = final_index / (2.0f * one_point_mm);
}

/**
  * @brief Расчет толщины методом по стробам
  */
void CalculateStrobeThickness(const float32_t* data) {
    if (!parameters_initialized) {
        SendUSBDebugMessage("Parameters not initialized for strobe method");
        return;
    }

    float one_point_mm = 1.0f / (params.wave_speed * 1000.0f * frequency_ns);
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
        thickness_value = 0.0f;
        return;
    }

    uint32_t index_difference = max_index_second - max_index_first;
    thickness_value = index_difference / (2.0f * one_point_mm);
}

/**
  * @brief Расчет толщины автокорреляционным методом
  */
void CalculateAndSendACFThickness(void) {
    if (!parameters_initialized) {
        SendUSBDebugMessage("Parameters not initialized for ACF thickness calculation");
        return;
    }

    float one_point_mm = 1.0f / (params.wave_speed * 1000.0f * frequency_ns);
    uint32_t max_index = FindMaxAutocorrelationIndex();
    thickness_value = max_index / (2.0f * one_point_mm);
}

/**
  * @brief Обработка данных в зависимости от выбранного метода
  */
void ProcessDataByMethod(void) {
    if (!parameters_initialized) {
        SendUSBDebugMessage("Parameters not initialized");
        return;
    }

    // ПРОВЕРЯЕМ, ЧТО УСРЕДНЕНИЕ ЗАВЕРШЕНО И ИСПОЛЬЗУЕМ УСРЕДНЕННЫЙ МАССИВ FPGA
    if (!averaging_complete) {
        SendUSBDebugMessage("Averaging not complete, cannot calculate thickness");
        thickness_value = 0.0f;
        return;
    }

    // Копируем данные из усредненного массива FPGA
    uint32_t copy_size = (DATA_SIZE < FINAL_DATA_SIZE) ? DATA_SIZE : FINAL_DATA_SIZE;
    for (uint32_t i = 0; i < copy_size; i++) {
        final_data[i] = averaged_fpga_data[i];
    }

    // Заполняем остаток нулями если необходимо
    for (uint32_t i = copy_size; i < FINAL_DATA_SIZE; i++) {
        final_data[i] = 0.0f;
    }

    // Обрабатываем данные в зависимости от выбранного метода
    switch (params.method) {
        case 0:
            // Автокорреляционный метод
            arm_copy_f32(final_data, normalized_data, DATA_VALUES_COUNT);
            NormalizeData();
            CalculateAutocorrelation();
            CalculateAndSendACFThickness();
            break;

        case 1:
            // Метод перехода через ноль
            CalculateZeroCrossingThickness(final_data);
            break;

        case 2:
            // Метод стробов
            CalculateStrobeThickness(final_data);
            break;

        default:
            SendUSBDebugMessage("Unknown method specified");
            break;
    }
}
