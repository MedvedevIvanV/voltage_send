#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include "main.h"
#include <stdbool.h>

// Внешние переменные
extern float thermocouple_temperature;
extern bool thermocouple_error;

// Функции для работы с термопарой
uint16_t Read_Thermocouple_Temperature(void);
float Get_Thermocouple_Temperature(void);

#endif // TEMPERATURE_SENSOR_H
