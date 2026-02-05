#ifndef THICKNESS_CALCULATOR_H
#define THICKNESS_CALCULATOR_H

#include "main.h"
#include "arm_math.h"
#include <stdbool.h>
#include <string.h>

// Определения размеров
#define DATA_SIZE 5100
#define PLIS_DATA_SIZE 10000
#define NUM_PARAM_SETS 4

// Структура для хранения параметров (одного набора)
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
    char sensor_number[5];
    float gain;
    uint32_t points_count;
    uint32_t crc;
} Parameters_t;

// Структура для временных параметров (не сохраняются в FLASH)
typedef struct {
    char start_date[20];  // "2024-01-15 14:30:00"
    uint32_t period;      // в секундах
} TempParams_t;

// Внешние переменные
extern Parameters_t params[NUM_PARAM_SETS];  // 4 набора параметров
extern TempParams_t temp_params;             // временные параметры
extern bool parameters_initialized[NUM_PARAM_SETS];
extern bool calculate_thickness_requested;
extern float thickness_value;
extern float frequency_ns;
extern float averaged_fpga_data[DATA_SIZE];
extern bool averaging_complete;

// Буферы данных
//extern float32_t normalized_data[DATA_VALUES_COUNT];
//extern float32_t autocorrelation_result[DATA_VALUES_COUNT];
//extern float32_t temp_data[FINAL_DATA_SIZE];
//extern float32_t final_data[FINAL_DATA_SIZE];
extern uint32_t successful_cycles;

// Функции для работы с параметрами
void InitializeParameters(void);
void SaveParametersToFlash(void);
void LoadParametersFromFlash(void);
uint32_t CalculateCRC32(const uint8_t *data, size_t length);

// Функции парсинга входящих данных
bool ParseGeneralParams(const char* data);
bool ParseSetParams(const char* data, uint8_t set_number);
bool ParseIncomingData(const char* incoming_string);

// Функции расчета толщины
void CalculateZeroCrossingThickness(const float32_t* data, uint8_t param_set);
void CalculateStrobeThickness(const float32_t* data, uint8_t param_set);
void CalculateAndSendACFThickness(uint8_t param_set);
void ProcessDataByMethod(uint8_t param_set);
bool ProcessCycle(uint32_t cycle_num, uint8_t param_set);
bool CheckThreshold(const float32_t* data, uint32_t size, uint8_t param_set);

// Вспомогательные функции
void AddRandomNoiseAndExtend(const float32_t* src, float32_t* dest, uint32_t dest_size, uint8_t param_set);
void NormalizeData(uint8_t param_set);
void CalculateAutocorrelation(uint8_t param_set);
uint32_t FindMaxAutocorrelationIndex(uint8_t param_set);

// Функции выбора активного набора параметров
uint8_t GetActiveParamSet(void);
void SetActiveParamSet(uint8_t set_number);

#endif // THICKNESS_CALCULATOR_H
