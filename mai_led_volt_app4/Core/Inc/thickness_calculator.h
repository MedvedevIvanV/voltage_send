#ifndef THICKNESS_CALCULATIONS_H
#define THICKNESS_CALCULATIONS_H

#include "main.h"
#include "arm_math.h"
#include <stdbool.h>

// Структура параметров (должна быть такой же как в main.c)
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

// Внешние переменные, которые будут определены в main.c
extern Parameters_t params;
extern bool parameters_initialized;
extern float frequency;
extern const float measurement_data[];
extern float32_t normalized_data[];
extern float32_t autocorrelation_result[];
extern float32_t final_data[];
extern uint32_t successful_cycles;

// Объявления функций
void CalculateZeroCrossingThickness(const float32_t* data);
void CalculateStrobeThickness(const float32_t* data);
void CalculateAndSendACFThickness(void);
void NormalizeData(void);
void CalculateAutocorrelation(void);
uint32_t FindMaxAutocorrelationIndex(void);
bool CheckThreshold(const float32_t* data, uint32_t size);
void AddRandomNoiseAndExtend(const float32_t* src, float32_t* dest, uint32_t dest_size);
bool ProcessCycle(uint32_t cycle_num);
void ProcessDataByMethod(void);

#endif // THICKNESS_CALCULATIONS_H
