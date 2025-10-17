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
#include "sx126x.h"
#include "sx126x_hal.h"
#include "thickness_calculator.h"
#include "temperature_sensor.h"
#include "adc_plis.h" // Файл с конфигурацией ПЛИС (массив uint8_t fpga_config[])
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define START_PULSE_DURATION_NS 200   // Длительность стартового импульса в наносекундах
#define USB_RX_BUFFER_SIZE 300
#define FINAL_DATA_SIZE 5000
#define UART_RX_TIMEOUT_MS 100

// LoRa module pins
#define sx1262_busy_pin GPIO_PIN_4
#define sx1262_busy_port GPIOC
#define sx1262_dio1_pin GPIO_PIN_2
#define sx1262_dio1_port GPIOE
#define sx1262_dio2_pin GPIO_PIN_1
#define sx1262_dio2_port GPIOE
#define sx1262_cs_pin GPIO_PIN_12
#define sx1262_cs_port GPIOB
#define sx1262_txen_pin GPIO_PIN_6
#define sx1262_txen_port GPIOC
#define sx1262_reset_pin GPIO_PIN_7
#define sx1262_reset_port GPIOC
#define USB_CONNECTED() HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)


// Добавить новые определения
#define ACK_STRING           "ACK\r\n"
#define COMPLETE_STRING      "COMPLETE\r\n"

#define DATA_SIZE 5000       // Количество значений в ПЛИС
#define VALUES_PER_LINE 10            // Количество значений в строке вывода
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
char usb_msg[1024];                   // Буфер для USB сообщений

// Добавляем объявления переменных из usbd_cdc_if.c
extern volatile uint8_t usb_rx_buffer[USB_RX_BUFFER_SIZE];
extern volatile uint8_t new_data_received;
extern volatile uint16_t usb_rx_index;

// Временные переменные (НЕ сохраняются во Flash)
char start_date[20] = {0}; // Формат: "2024-01-15 14:30:25"
uint32_t period = 0;


// UART переменные
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

// LoRa variables
sx126x_context radio;
sx126x_mod_params_lora_t lora_params;
sx126x_pkt_params_lora_t pkt_params;
sx126x_pa_cfg_params_t pa_params;
int8_t pa_power = 10;
uint32_t frequency = 868000000U;
bool lora_initialized = false;
uint32_t lora_last_send_time = 0;  // Добавьте эту переменную
// Переменные для данных от дежурного МК
float received_voltage = 0.0f;
float received_temp = 0.0f;


typedef struct {
    uint16_t data[DATA_SIZE]; // Буфер для хранения данных от ПЛИС
    bool data_ready;                  // Флаг готовности данных
    uint8_t data_count;               // Количество считанных данных
} FPGA_Data;

FPGA_Data fpga_data;                 // Структура для хранения данных ПЛИС
volatile uint16_t *fpga_reg;         // Указатель на регистр ПЛИС
#define FPGA_BASE_ADDRESS 0x60000000  // Базовый адрес Bank1 (NE1)


uint16_t temp_fpga_buffer[DATA_SIZE];     // Временный буфер для чтения
float averaged_fpga_data[DATA_SIZE];   // Итоговый усредненный массив
bool averaging_complete = false;          // Флаг завершения усреднения
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
static void MX_FSMC_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void SendUSBDebugMessage(const char *message);
void GenerateStartPulse(void);
void ProcessUSBCommand(uint8_t cmd);
void ParseParameters(const char* params_str);
void SendParametersResponse(void);
void SendDateTimeToBackupMCU(void);
void ProcessUARTCommand(uint8_t* data, uint8_t len);
bool InitializeLoRa(void);
void SendTestDataViaLoRa(void);
// Добавить прототип
void SendUARTResponse(const char* response);
void ReadFPGAData(void);
void PrintDataToUSB(void);

// ДОБАВИТЬ ЭТУ ФУНКЦИЮ
bool CheckAndLoadFPGAConfig(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief Отправка отладочного сообщения через USB
  * @param message Текст сообщения
  */
void SendUSBDebugMessage(const char *message) {
    if (USB_CONNECTED()) {
        snprintf(usb_msg, sizeof(usb_msg), "[%lu] %s\r\n", HAL_GetTick(), message);
        CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
        HAL_Delay(10); // Задержка для стабильной работы USB
    }
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

//    // Загружаем параметры из памяти и запускаем расчет
//    if (!parameters_initialized) {
//        LoadParametersFromFlash();
//    }
//    calculate_thickness_requested = true;
//
//    SendUSBDebugMessage("Start pulse generated and calculation requested");
}

/**
  * @brief Обработка команд от USB - не использовать!
  * @param cmd Полученная команда
  */
void ProcessUSBCommand(uint8_t cmd) {
    switch(cmd) {
        case '1':
            // Команда 1
               SendUSBDebugMessage("Unknown command received 1");
            break;

        default:
            // Неизвестная команда
            SendUSBDebugMessage("Unknown command received");
            break;
    }
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
void ProcessUARTCommand(uint8_t* data, uint8_t len) {

    HAL_Delay(150);

    // Загружаем конфигурацию ПЛИС
    FPGA_LoadConfig();

    // Уменьшаем дополнительную паузу
    HAL_Delay(10);
	ReadFPGAData(); // Теперь эта функция делает все: START + многократное чтение + усреднение

	    if (fpga_data.data_ready) {

	        // СРАЗУ ВЫЧИСЛЯЕМ ТОЛЩИНУ ПО УСРЕДНЕННОМУ МАССИВУ
	        if (parameters_initialized && averaging_complete) {
	            calculate_thickness_requested = true;

	            uint32_t start_time = HAL_GetTick();
	            while (calculate_thickness_requested && (HAL_GetTick() - start_time) < 5000) {
	                if (calculate_thickness_requested) {
	                    calculate_thickness_requested = false;
	                    ProcessDataByMethod(); // Теперь использует averaged_fpga_data
	                }
	                HAL_Delay(10);
	            }
	        }
	        SendUSBDebugMessage("Averaged data received from FPGA:");
	        PrintDataToUSB();
	        fpga_data.data_ready = false;
	    }
    // Поиск всех параметров в данных
    char* date_ptr = strstr((char*)data, "DATE:");
    char* time_ptr = strstr((char*)data, ";TIME:");
    char* period_ptr = strstr((char*)data, ";PERIOD:");
    char* voltage_ptr = strstr((char*)data, ";VOLTAGE:");
    char* temp_ptr = strstr((char*)data, ";TEMP:");

    // Инициализация значений по умолчанию
    int year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;
    uint32_t received_period = 0;
    received_voltage = 0.0f;
    received_temp = 0.0f;

    // Парсинг доступных параметров
    if(date_ptr) sscanf(date_ptr, "DATE:%d-%d-%d", &year, &month, &day);
    if(time_ptr) sscanf(time_ptr, ";TIME:%d:%d:%d", &hour, &min, &sec);
    if(period_ptr) sscanf(period_ptr, ";PERIOD:%lu", &received_period);
    if(voltage_ptr) sscanf(voltage_ptr, ";VOLTAGE:%f", &received_voltage);
    if(temp_ptr) sscanf(temp_ptr, ";TEMP:%f", &received_temp);

    // Формируем строку даты
    snprintf(start_date, sizeof(start_date), "%04d-%02d-%02d %02d:%02d:%02d",
            year, month, day, hour, min, sec);
    period = received_period;

    // ИЗМЕРЯЕМ ТЕМПЕРАТУРУ ТЕРМОПАРЫ
    thermocouple_temperature = Get_Thermocouple_Temperature();

    // ВЫЧИСЛЯЕМ ТОЛЩИНУ (если еще не вычислена)
    if (thickness_value == 0.0f && parameters_initialized) {
        calculate_thickness_requested = true;

        uint32_t start_time = HAL_GetTick();
        while (calculate_thickness_requested && (HAL_GetTick() - start_time) < 5000) {
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

    SendMeasurementDataViaLoRa();
    // После завершения всех операций отправляем COMPLETE
    SendUARTResponse(COMPLETE_STRING);

}
/**
  * @brief Отправка данных измерений через LoRa
  */
void SendMeasurementDataViaLoRa(void) {
    // Проверяем инициализацию LoRa
    if (!lora_initialized) {
        if (!InitializeLoRa()) {
            SendUSBDebugMessage("LoRa initialization failed for measurement data send");
            return;
        }
    }

    // Формируем данные для отправки
    uint8_t lora_data[128] = {0};
    uint8_t data_index = 0;

    // Добавляем period (4 байта) - временная переменная
    memcpy(&lora_data[data_index], &period, sizeof(uint32_t));
    data_index += sizeof(uint32_t);

    // Добавляем received_voltage (4 байта) - из ProcessUARTCommand
    float voltage = received_voltage;
    memcpy(&lora_data[data_index], &voltage, sizeof(float));
    data_index += sizeof(float);

    // Добавляем received_temp (4 байта) - из ProcessUARTCommand
    float temp = received_temp;
    memcpy(&lora_data[data_index], &temp, sizeof(float));
    data_index += sizeof(float);

    // Добавляем thermocouple_temperature (4 байта) - глобальная переменная
    memcpy(&lora_data[data_index], &thermocouple_temperature, sizeof(float));
    data_index += sizeof(float);

    // Добавляем thickness_value (4 байта) - расчетная переменная
    memcpy(&lora_data[data_index], &thickness_value, sizeof(float));
    data_index += sizeof(float);

    // Добавляем wave_speed (4 байта) - из параметров
    float wave_speed = params.wave_speed;
    memcpy(&lora_data[data_index], &wave_speed, sizeof(float));
    data_index += sizeof(float);

    // Добавляем threshold (4 байта) - из параметров
    float threshold = params.threshold;
    memcpy(&lora_data[data_index], &threshold, sizeof(float));
    data_index += sizeof(float);

    // Добавляем threshold_zero_crossing (4 байта) - из параметров
    float threshold_zero = params.threshold_zero_crossing;
    memcpy(&lora_data[data_index], &threshold_zero, sizeof(float));
    data_index += sizeof(float);

    // Добавляем start_index (4 байта) - из параметров
    uint32_t start_idx = params.start_index;
    memcpy(&lora_data[data_index], &start_idx, sizeof(uint32_t));
    data_index += sizeof(uint32_t);

    // Добавляем probe_length (4 байта) - из параметров
    uint32_t probe_len = params.probe_length;
    memcpy(&lora_data[data_index], &probe_len, sizeof(uint32_t));
    data_index += sizeof(uint32_t);

    // Добавляем first_left_strobe (4 байта) - из параметров
    uint32_t strobe_l1 = params.first_left_strobe;
    memcpy(&lora_data[data_index], &strobe_l1, sizeof(uint32_t));
    data_index += sizeof(uint32_t);

    // Добавляем first_right_strobe (4 байта) - из параметров
    uint32_t strobe_r1 = params.first_right_strobe;
    memcpy(&lora_data[data_index], &strobe_r1, sizeof(uint32_t));
    data_index += sizeof(uint32_t);

    // Добавляем second_left_strobe (4 байта) - из параметров
    uint32_t strobe_l2 = params.second_left_strobe;
    memcpy(&lora_data[data_index], &strobe_l2, sizeof(uint32_t));
    data_index += sizeof(uint32_t);

    // Добавляем second_right_strobe (4 байта) - из параметров
    uint32_t strobe_r2 = params.second_right_strobe;
    memcpy(&lora_data[data_index], &strobe_r2, sizeof(uint32_t));
    data_index += sizeof(uint32_t);

    // Добавляем method (4 байта) - из параметров
    uint32_t method = params.method;
    memcpy(&lora_data[data_index], &method, sizeof(uint32_t));
    data_index += sizeof(uint32_t);

    // Добавляем end_index (4 байта) - из параметров
    uint32_t end_idx = params.end_index;
    memcpy(&lora_data[data_index], &end_idx, sizeof(uint32_t));
    data_index += sizeof(uint32_t);

    // Добавляем cycle_number (4 байта) - из параметров
    uint32_t cycle_num = params.cycle_number;
    memcpy(&lora_data[data_index], &cycle_num, sizeof(uint32_t));
    data_index += sizeof(uint32_t);

    // Добавляем sensor_number (максимум 16 байт) - из параметров
    uint8_t sensor_len = strlen(params.sensor_number);
    if (sensor_len > 15) sensor_len = 15;
    lora_data[data_index++] = sensor_len;
    memcpy(&lora_data[data_index], params.sensor_number, sensor_len);
    data_index += sensor_len;

    // Добавляем gain (4 байта) - из параметров
    float gain = params.gain;
    memcpy(&lora_data[data_index], &gain, sizeof(float));
    data_index += sizeof(float);

    // Общая длина данных
    uint8_t total_length = data_index;

    // Настройка параметров передачи LoRa
    sx126x_set_tx_params(&radio, pa_power, SX126X_RAMP_200_US);

    // Ожидаем, пока модуль освободится
    while (HAL_GPIO_ReadPin(sx1262_busy_port, sx1262_busy_pin) == GPIO_PIN_SET) {
        HAL_Delay(1);
    }

    // Записываем данные в буфер модуля LoRa
    sx126x_status_t status = sx126x_write_buffer(&radio, 0, lora_data, total_length);
    if (status != SX126X_STATUS_OK) {
        SendUSBDebugMessage("LoRa write buffer failed for measurement data");
        return;
    }

    // Обновляем параметры пакета с актуальной длиной
    pkt_params.pld_len_in_bytes = total_length;
    status = sx126x_set_lora_pkt_params(&radio, &pkt_params);
    if (status != SX126X_STATUS_OK) {
        SendUSBDebugMessage("LoRa set packet params failed");
        return;
    }

    // Запускаем передачу
    status = sx126x_set_tx(&radio, SX126X_MAX_TIMEOUT_IN_MS);
    if (status != SX126X_STATUS_OK) {
        SendUSBDebugMessage("LoRa transmission failed for measurement data");
    } else {
        snprintf(usb_msg, sizeof(usb_msg), "LoRa measurement data sent (%d bytes)", total_length);
        SendUSBDebugMessage(usb_msg);
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1) {
        uart_last_rx_time = HAL_GetTick();

        if(uart_rx_buf[uart_rx_pos] == '\n' || uart_rx_pos >= UART_RX_BUF_SIZE-1) {
            // Копируем данные в буфер для обработки
            memcpy((void*)uart_rx_data, uart_rx_buf, uart_rx_pos);
            uart_rx_len = uart_rx_pos;
            uart_message_received = 1;

            uart_rx_pos = 0;
            memset(uart_rx_buf, 0, sizeof(uart_rx_buf));

            // Немедленно запускаем прием следующего байта
            HAL_UART_Receive_IT(&huart1, (uint8_t*)uart_rx_buf, 1);
        } else {
            uart_rx_pos++;
            HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart_rx_buf[uart_rx_pos], 1);
        }
    }
}

/**
  * @brief Инициализация модуля LoRa
  */
bool InitializeLoRa(void) {
    // Initialize radio context
    radio.cs_port = sx1262_cs_port;
    radio.cs_pin = sx1262_cs_pin;
    radio.busy_port = sx1262_busy_port;
    radio.busy_pin = sx1262_busy_pin;
    radio.reset_port = sx1262_reset_port;
    radio.reset_pin = sx1262_reset_pin;
    radio.hspi = &hspi2;

    // Initialize LoRa parameters
    lora_params.sf = SX126X_LORA_SF12;
    lora_params.bw = SX126X_LORA_BW_125;
    lora_params.cr = SX126X_LORA_CR_4_7;
    lora_params.ldro = 0;

    pkt_params.preamble_len_in_symb = 12;
    pkt_params.header_type = SX126X_LORA_PKT_EXPLICIT;
    pkt_params.pld_len_in_bytes = 128;
    pkt_params.crc_is_on = false;
    pkt_params.invert_iq_is_on = false;

    pa_params.pa_duty_cycle = 0x04;
    pa_params.hp_max = 0x07;
    pa_params.device_sel = 0x00;
    pa_params.pa_lut = 0x01;

    // Initialize LoRa module
    sx126x_status_t status = sx126x_hal_reset(&radio);
    if (status != SX126X_STATUS_OK) {
        SendUSBDebugMessage("LoRa reset failed");
        return false;
    }
    HAL_Delay(250);

    status = sx126x_set_standby(&radio, SX126X_STANDBY_CFG_RC);
    HAL_Delay(250);

    status = sx126x_hal_wakeup(&radio);
    HAL_Delay(250);

    status = sx126x_set_reg_mode(&radio, SX126X_REG_MODE_LDO);
    HAL_Delay(250);

    status = sx126x_cal(&radio, SX126X_CAL_ALL);
    HAL_Delay(250);

    status = sx126x_set_standby(&radio, SX126X_STANDBY_CFG_RC);
    HAL_Delay(250);

    status = sx126x_set_reg_mode(&radio, SX126X_REG_MODE_LDO);
    HAL_Delay(250);

    status = sx126x_set_pkt_type(&radio, SX126X_PKT_TYPE_LORA);
    HAL_Delay(250);

    status = sx126x_set_lora_mod_params(&radio, &lora_params);
    HAL_Delay(250);

    status = sx126x_set_lora_pkt_params(&radio, &pkt_params);
    HAL_Delay(250);

    sx126x_set_dio3_as_tcxo_ctrl(&radio, SX126X_TCXO_CTRL_2_4V, 5000);
    HAL_Delay(250);

    status = sx126x_set_lora_sync_word(&radio, 0x12);
    HAL_Delay(250);

    status = sx126x_set_rf_freq(&radio, frequency);
    HAL_Delay(250);

    status = sx126x_set_pa_cfg(&radio, &pa_params);
    HAL_Delay(250);

    status = sx126x_set_dio_irq_params(&radio,
            SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE,
            SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE, SX126X_IRQ_NONE,
            SX126X_IRQ_NONE);
    HAL_Delay(100);

    lora_initialized = true;
    SendUSBDebugMessage("LoRa module initialized successfully");
    return true;
}

/**
  * @brief Отправка данных через LoRa
  */
void SendTestDataViaLoRa(void) {
    if (!lora_initialized) {
        if (!InitializeLoRa()) {
            SendUSBDebugMessage("LoRa not initialized, cannot send data");
            return;
        }
    }

    // Создаем тестовые данные - 5 чисел (например: 1,2,3,4,5)
    const uint8_t test_data[] = {1, 2, 3, 4, 5};

    sx126x_set_tx_params(&radio, pa_power, SX126X_RAMP_200_US);

    // Wait while module is busy
    while (HAL_GPIO_ReadPin(sx1262_busy_port, sx1262_busy_pin) == GPIO_PIN_SET);

    // Write data to buffer
    sx126x_status_t status = sx126x_write_buffer(&radio, 0, test_data, sizeof(test_data));
    if (status != SX126X_STATUS_OK) {
        SendUSBDebugMessage("LoRa write buffer failed");
        return;
    }

    // Start transmission
    status = sx126x_set_tx(&radio, SX126X_MAX_TIMEOUT_IN_MS);
    if (status != SX126X_STATUS_OK) {
        SendUSBDebugMessage("LoRa transmission failed");
    } else {
        SendUSBDebugMessage("LoRa test data sent successfully");
    }
}



/**
  * @brief Отправка ответа по UART
  */
void SendUARTResponse(const char* response)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), 100);
    HAL_Delay(10);
}




/**
  * @brief Вывод усредненных данных через USB CDC
  */
void PrintDataToUSB(void) {
    if (!fpga_data.data_ready) return;

    // Формируем заголовок с информацией об усреднении
    snprintf(usb_msg, sizeof(usb_msg), "Averaged FPGA Data [%lu cycles, 0-%d]:\r\n",
             params.cycle_number, DATA_SIZE-1);
    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
    HAL_Delay(10);

    // Формируем строки с данными
    char data_line[128] = "";
    for (int i = 0; i < DATA_SIZE; i++) {
        char val_str[12];
        snprintf(val_str, sizeof(val_str), "%6.1f ", averaged_fpga_data[i]); // Форматирование для float
        strncat(data_line, val_str, sizeof(data_line) - strlen(data_line) - 1);

        // Если строка заполнена или это последнее значение
        if ((i+1) % VALUES_PER_LINE == 0 || i == DATA_SIZE-1) {
            strncat(data_line, "\r\n", sizeof(data_line) - strlen(data_line) - 1);
            CDC_Transmit_FS((uint8_t*)data_line, strlen(data_line));
            HAL_Delay(10);
            data_line[0] = '\0'; // Очищаем строку
        }
    }
}



/**
  * @brief Проверка и загрузка конфигурации ПЛИС при включении питания
  * @return true если конфигурация успешно загружена, false в случае ошибки
  */
bool CheckAndLoadFPGAConfig(void) {
    SendUSBDebugMessage("Checking FPGA configuration...");

    // Получаем данные конфигурации из adc_plis.h
    uint8_t *config_data = fpga_config;
    uint32_t config_size = sizeof(fpga_config);

    // Загружаем конфигурацию в ПЛИС
    FPGA_SendConfig(config_data, config_size);

    // Даем время ПЛИС на инициализацию
    HAL_Delay(100);


    SendUSBDebugMessage("FPGA configuration loaded successfully");
    return true;
}

/**
  * @brief Отправка конфигурации в ПЛИС
  * @param config_data Указатель на данные конфигурации
  * @param size Размер данных конфигурации
  */
void FPGA_LoadConfig(void) {
	 GPIO_InitTypeDef GPIO_InitStruct = {0};

	    // 1. Настройка пинов (оставляем как было)
	    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	    GPIO_InitStruct.Pin = GPIO_PIN_15;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	    GPIO_InitStruct.Pin = GPIO_PIN_8;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    // 2. Установка начальных состояний
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);   // TH_CS = 1
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   // CE = 1
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // CLK = 0
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // DATA = 0
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // nCONFIG = 0
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // CSO = 1

	    // 3. ВАЖНО: Увеличиваем задержку для стабилизации питания при холодном старте
	    HAL_Delay(100); // Возвращаем 100 мс для надежности

	    // 4. Сброс ПЛИС с достаточной паузой
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // nCONFIG = 0
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);  // CE = 0
	    HAL_Delay(5); // Увеличиваем до 5 мс (вместо 1 мс)

	    // 5. Запуск конфигурации
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   // nCONFIG = 1

	    // 6. Ожидание готовности ПЛИС (увеличиваем)
	    HAL_Delay(3); // Увеличиваем до 3 мс

	    // 7. ОПТИМИЗИРОВАННАЯ передача данных (оставляем быструю)
	    const uint8_t *config_data = fpga_config;
	    uint32_t config_size = sizeof(fpga_config);

	    GPIO_TypeDef* data_port = GPIOC;
	    GPIO_TypeDef* clk_port = GPIOC;
	    uint16_t data_pin = GPIO_PIN_11;
	    uint16_t clk_pin = GPIO_PIN_10;

	    for (uint32_t i = 0; i < config_size; i++) {
	        uint8_t byte = config_data[i];

	        // Развернутый цикл для скорости (оставляем как было)
	        // Бит 0
	        if (byte & 0x01) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;

	        // Бит 1
	        if (byte & 0x02) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;

	        // Бит 2
	        if (byte & 0x04) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;

	        // Бит 3
	        if (byte & 0x08) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;

	        // Бит 4
	        if (byte & 0x10) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;

	        // Бит 5
	        if (byte & 0x20) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;

	        // Бит 6
	        if (byte & 0x40) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;

	        // Бит 7
	        if (byte & 0x80) data_port->BSRR = data_pin;
	        else data_port->BSRR = (uint32_t)data_pin << 16;
	        __NOP(); __NOP();
	        clk_port->BSRR = clk_pin;
	        __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;
	    }

	    // 8. Завершающие импульсы (увеличиваем количество для надежности)
	    for (int i = 0; i < 12; i++) { // Увеличиваем до 12
	        clk_port->BSRR = clk_pin;
	        __NOP(); __NOP();
	        clk_port->BSRR = (uint32_t)clk_pin << 16;
	        __NOP(); __NOP();
	    }

	    // 9. Активация ПЛИС с достаточной паузой
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   // CE = 1
	    HAL_Delay(3); // Увеличиваем до 3 мс

	    // 10. Финальные настройки
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);  // TH_CS = 0
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);  // CSO = 0

	    // 11. Финальная пауза для стабилизации
	    HAL_Delay(5); // Увеличиваем до 5 мс
}

void FPGA_SendConfig(uint8_t *config_data, uint32_t size) {
    FPGA_LoadConfig();
}





/**
  * @brief Многократное чтение и усреднение данных из ПЛИС с проверкой порога
  */
void ReadFPGAData(void) {
    // Инициализация итогового массива нулями
    memset(averaged_fpga_data, 0, sizeof(averaged_fpga_data));
    averaging_complete = false;

    // Получаем количество циклов из параметров
    uint32_t cycles = params.cycle_number;
    float threshold = params.threshold; // Получаем порог из параметров

    if (cycles == 0) {
        cycles = 1; // Минимум один цикл
    }

    snprintf(usb_msg, sizeof(usb_msg), "Starting %lu averaging cycles with threshold: %.1f", cycles, threshold);
    SendUSBDebugMessage(usb_msg);

    uint32_t valid_cycles = 0; // Счетчик валидных циклов (без превышения порога)

    for (uint32_t cycle = 0; cycle < cycles; cycle++) {
        // Генерируем START импульс для нового измерения
        GenerateStartPulse();

        // Ждем некоторое время для стабилизации ПЛИС
        HAL_Delay(1);

        bool threshold_exceeded = false; // Флаг превышения порога

        // Читаем данные во временный буфер с проверкой порога
        __disable_irq(); // Отключаем прерывания для атомарного чтения

        for (int i = 0; i < DATA_SIZE; i++) {
            // Читаем значение - ПЛИС автоматически переключает индекс при каждом чтении
            uint16_t value = fpga_reg[0];
            uint16_t raw_value = value & 0x0FFF - 2048; // Извлекаем 12-битное значение

            // Проверяем порог (по модулю)
            if (abs((int16_t)raw_value) > threshold) { // 2048 - среднее значение для 12-битного АЦП
                threshold_exceeded = true;
                __enable_irq(); // Включаем прерывания перед выходом
                break; // Немедленно выходим из цикла чтения
            }

            temp_fpga_buffer[i] = raw_value;

            // Небольшая задержка между чтениями для стабильности
            for(volatile int j = 0; j < 10; j++);
        }

        __enable_irq(); // Включаем прерывания обратно

        // Если порог превышен, пропускаем этот цикл
        if (threshold_exceeded) {
            snprintf(usb_msg, sizeof(usb_msg), "Cycle %lu skipped - threshold exceeded", cycle + 1);
            SendUSBDebugMessage(usb_msg);
            continue; // Переходим к следующей итерации цикла
        }

        // Усредняем данные только если цикл валидный
        valid_cycles++;
        for (int i = 0; i < DATA_SIZE; i++) {
            // Первый валидный цикл - просто копируем, последующие - усредняем
            if (valid_cycles == 1) {
                averaged_fpga_data[i] = (float)temp_fpga_buffer[i];
            } else {
                // Усреднение: (предыдущее * циклы + новое) / (циклы + 1)
                averaged_fpga_data[i] = (averaged_fpga_data[i] * (valid_cycles - 1) + (float)temp_fpga_buffer[i]) / valid_cycles;
            }
        }

        // Опционально: отправляем прогресс по USB
        if ((cycle + 1) % 10 == 0 || cycle == cycles - 1) {
            snprintf(usb_msg, sizeof(usb_msg), "Averaging progress: %lu/%lu cycles, valid: %lu",
                     cycle + 1, cycles, valid_cycles);
            SendUSBDebugMessage(usb_msg);
        }

        // Небольшая пауза между циклами
        HAL_Delay(10);
    }

    // Копируем усредненные данные в основную структуру только если есть валидные циклы
    if (valid_cycles > 0) {
        for (int i = 0; i < DATA_SIZE; i++) {
            fpga_data.data[i] = (uint16_t)averaged_fpga_data[i]; // Приводим к uint16_t для обратной совместимости
        }
        fpga_data.data_count = DATA_SIZE;
        fpga_data.data_ready = true;
        averaging_complete = true;

        snprintf(usb_msg, sizeof(usb_msg), "Averaging completed: %lu valid cycles out of %lu", valid_cycles, cycles);
        SendUSBDebugMessage(usb_msg);
    } else {
        fpga_data.data_ready = false;
        averaging_complete = false;
        SendUSBDebugMessage("Averaging failed: no valid cycles (all exceeded threshold)");
    }
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // Принудительно включаем тактирование всех используемых портов
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_FSMC_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  // Умеренное уменьшение задержки
  HAL_Delay(150);


  	      // Загружаем конфигурацию ПЛИС
  	      FPGA_LoadConfig();

  	      // Уменьшаем дополнительную паузу
  	      HAL_Delay(100);
	  HAL_TIM_Base_Start(&htim3);
	  srand(HAL_GetTick());

	  HAL_UART_Receive_IT(&huart1, (uint8_t*)uart_rx_buf, 1);

	  // Инициализация указателя на регистр ПЛИС
	  fpga_reg = (volatile uint16_t *)FPGA_BASE_ADDRESS;

	  // Инициализация структуры данных ПЛИС
	  memset(&fpga_data, 0, sizeof(fpga_data));
	  HAL_Delay(1000);
	  // Загружаем параметры из энергонезависимой памяти при старте
	  HAL_Delay(1000);
	  LoadParametersFromFlash();
	  HAL_Delay(1000);


	  InitializeLoRa();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  while (1) {


	      if (new_data_received) {

	          uart_message_received = 0;

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

	      // Проверяем, нужно ли выполнить расчет толщины
	      if (calculate_thickness_requested && parameters_initialized) {
	          calculate_thickness_requested = false;
	          ProcessDataByMethod();
	          SendUSBDebugMessage("Thickness calculation completed");
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9
                           PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11;
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
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_PSRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_DISABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 2;
  Timing.AddressHoldTime = 1;
  Timing.DataSetupTime =  5;
  Timing.BusTurnAroundDuration = 1;
  Timing.CLKDivision = 2;
  Timing.DataLatency = 2;
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
