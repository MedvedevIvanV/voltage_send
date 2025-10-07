#ifndef THICKNESS_CALCULATOR_H
#define THICKNESS_CALCULATOR_H

#include "main.h"
#include "arm_math.h"
#include <stdbool.h>

// Определения размеров
#define FINAL_DATA_SIZE 5000
#define DATA_VALUES_COUNT 4600
#define DATA_SIZE 5000


// Структура для хранения параметров
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
    uint32_t crc;
} Parameters_t;

// Внешние переменные
extern Parameters_t params;
extern bool parameters_initialized;
extern bool calculate_thickness_requested;
extern float thickness_value;
extern float frequency_ns;
extern float averaged_fpga_data[DATA_SIZE];
extern bool averaging_complete;
// Буферы данных
extern float32_t normalized_data[DATA_VALUES_COUNT];
extern float32_t autocorrelation_result[DATA_VALUES_COUNT];
extern float32_t temp_data[FINAL_DATA_SIZE];
extern float32_t final_data[FINAL_DATA_SIZE];
extern uint32_t successful_cycles;

// Функции для работы с параметрами
void InitializeParameters(void);
void SaveParametersToFlash(void);
void LoadParametersFromFlash(void);
uint32_t CalculateCRC32(const uint8_t *data, size_t length);

// Функции расчета толщины
void CalculateZeroCrossingThickness(const float32_t* data);
void CalculateStrobeThickness(const float32_t* data);
void CalculateAndSendACFThickness(void);
void ProcessDataByMethod(void);
bool ProcessCycle(uint32_t cycle_num);
bool CheckThreshold(const float32_t* data, uint32_t size);

// Вспомогательные функции
void AddRandomNoiseAndExtend(const float32_t* src, float32_t* dest, uint32_t dest_size);
void NormalizeData(void);
void CalculateAutocorrelation(void);
uint32_t FindMaxAutocorrelationIndex(void);

#endif // THICKNESS_CALCULATOR_H
