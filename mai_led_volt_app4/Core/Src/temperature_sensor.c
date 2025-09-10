#include "temperature_sensor.h"

// Глобальные переменные
float thermocouple_temperature = 0.0f;
bool thermocouple_error = false;

/**
  * @brief Чтение сырых данных с термопары
  */
uint16_t Read_Thermocouple_Temperature(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint16_t raw_data = 0;

    // Настройка PC11 (DATA) как входа
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Активация чипа
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

    for(volatile int i = 0; i < 10; i++);

    // Чтение 16 бит данных
    for(uint8_t i = 0; i < 16; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
        for(volatile int j = 0; j < 5; j++);

        if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11)) {
            raw_data |= (1 << (15 - i));
        }

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
        for(volatile int j = 0; j < 5; j++);
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

    // Проверка на разомкнутую цепь
    if(raw_data & 0x04) {
        thermocouple_error = true;
        return -999.0f;
    }

    thermocouple_error = false;

    // Извлечение 12-битного значения температуры
    raw_data >>= 3;
    raw_data &= 0x0FFF;

    // Конверсия в градусы
    return (float)raw_data * 0.25f;
}
