#ifndef U_UTILS_H
#define U_UTILS_H

#include <stdint.h>

typedef struct {
    float ChipTemperature;  // Температура чипа в °C
    float PA10_Voltage;     // Напряжение на PA10 (после делителя)
    float Vdd_Voltage;      // Напряжение питания (VBAT)
} TU1AdcValues;

void GetAdcValues(TU1AdcValues *const OutVal);

#endif // U_UTILS_H
