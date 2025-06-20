/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    char date[20];    // Формат: "YYYY-MM-DD HH:MM:SS"
    uint32_t period;  // Период в секундах
} DateTimeData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VOLTAGE_STORAGE_ADDR 0x080E0000
#define FLASH_SECTOR FLASH_SECTOR_11
#define CS_LOW    GPIOC->BSRR = GPIO_BSRR_BR_8
#define CS_HIGH   GPIOC->BSRR = GPIO_BSRR_BS_8
#define UART_BUF_SIZE 128
#define LED_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_13
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
SPI_HandleTypeDef hspi3;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t usb_led_command = 0;
extern volatile uint8_t usb_rx_buffer[64];
extern volatile uint8_t new_data_received;

DateTimeData app_data = {0};
DateTimeData backup_data = {0};
uint8_t backup_data_valid = 0;

uint8_t uart_buf[UART_BUF_SIZE];
uint16_t uart_pos = 0;
volatile uint8_t uart_msg_ready = 0;
uint8_t uart_byte;

float dac_voltage = 0.0f;
uint32_t dac_last_update = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_DAC_Init(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
HAL_StatusTypeDef Write_Voltage_To_Flash(float voltage);
float Read_Voltage_From_Flash(void);
void Spi3_Init(void);
uint16_t Spi3_Read_Data(void);
float Read_Temperature(void);
void Process_USB_Command(volatile uint8_t* data);
void Process_UART_Data(uint8_t* data);
void Send_To_Backup_MK(DateTimeData* data);
void Set_DAC_Voltage(float voltage);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef Write_Voltage_To_Flash(float voltage) {
    HAL_StatusTypeDef status;
    uint32_t voltageData = *(uint32_t*)&voltage;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                          FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = FLASH_SECTOR;
    EraseInitStruct.NbSectors = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t SectorError;
    status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, VOLTAGE_STORAGE_ADDR, voltageData);
    FLASH_WaitForLastOperation(100);
    HAL_FLASH_Lock();
    return status;
}

float Read_Voltage_From_Flash() {
    uint32_t voltageData = *(__IO uint32_t*)(VOLTAGE_STORAGE_ADDR);
    if (voltageData == 0xFFFFFFFF) {
        return 3.3f;
    }
    return *(float*)&voltageData;
}

void Spi3_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

    GPIOC->MODER |= GPIO_MODER_MODER8_0;
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT_8;
    GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED8;

    GPIOC->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
    GPIOC->AFR[1] |= (6 << (4*(10-8))) | (6 << (4*(11-8)));

    SPI3->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR |
               SPI_CR1_BR_2 | SPI_CR1_CPHA | SPI_CR1_DFF;
    SPI3->CR1 |= SPI_CR1_SPE;
}

uint16_t Spi3_Read_Data(void) {
    uint16_t data = 0;
    CS_LOW;
    while(!(SPI3->SR & SPI_SR_TXE));
    SPI3->DR = 0x0000;
    while(!(SPI3->SR & SPI_SR_RXNE));
    data = SPI3->DR;
    CS_HIGH;
    return data;
}

float Read_Temperature(void) {
    uint16_t raw_data = Spi3_Read_Data();
    if(raw_data & 0x04) {
        return -1.0f;
    }
    raw_data >>= 3;
    return raw_data * 0.25f;
}

void Set_DAC_Voltage(float voltage) {
    if (voltage < 0) voltage = 0;
    if (voltage > 1) voltage = 1;

    uint32_t dac_value = (voltage / 3.3f) * 4095;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    dac_voltage = voltage;
}

void Process_USB_Command(volatile uint8_t* data) {
    char response[64];
    static uint32_t led_off_time = 0;

    if (strncmp((char*)data, "SETDATA=", 8) == 0) {
        char* comma_pos = strchr((char*)data + 8, ',');
        char* newline_pos = strchr((char*)data, '\n');

        if (!comma_pos || !newline_pos || comma_pos > newline_pos) {
            CDC_Transmit_FS((uint8_t*)"ERR: Bad format\n", 16);
            return;
        }

        size_t date_len = comma_pos - ((char*)data + 8);
        strncpy(app_data.date, (char*)data + 8, date_len);
        app_data.date[date_len] = '\0';
        app_data.period = atoi(comma_pos + 1);

        snprintf(response, sizeof(response),
                "ACK: Date=%s, Period=%lus\n",
                app_data.date, app_data.period);
        CDC_Transmit_FS((uint8_t*)response, strlen(response));
        Send_To_Backup_MK(&app_data);
        return;
    }
    else if (data[0] == '1') {
        float voltage_pa0 = 0.0f;
        float voltage_pa1 = 0.0f;

        snprintf(response, sizeof(response),
                "PA0=%.2fV,PA1=%.2fV\n",
                voltage_pa0, voltage_pa1);
        CDC_Transmit_FS((uint8_t*)response, strlen(response));
    }
    else if (data[0] == '2') {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        led_off_time = HAL_GetTick() + 5000;
        CDC_Transmit_FS((uint8_t*)"LED: ON 5s\n", 11);
    }
    else {
        CDC_Transmit_FS((uint8_t*)"ERR: Unknown cmd\n", 17);
    }

    if (led_off_time && HAL_GetTick() > led_off_time) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        led_off_time = 0;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        if(uart_byte == '\n' || uart_byte == '\r') {
            if(uart_pos > 0) {
                uart_buf[uart_pos] = '\0';
                uart_msg_ready = 1;
            }
            uart_pos = 0;
        }
        else if(uart_byte >= 32 && uart_byte <= 126) {
            if(uart_pos < UART_BUF_SIZE-1) {
                uart_buf[uart_pos++] = uart_byte;
            } else {
                uart_pos = 0;
            }
        }
        HAL_UART_Receive_IT(&huart1, &uart_byte, 1);
    }
}

void Process_UART_Data(uint8_t* data) {
    if(strlen((char*)data) < 20) {
        CDC_Transmit_FS((uint8_t*)"ERR: Message too short\r\n", 24);
        return;
    }

    char* date_ptr = strstr((char*)data, "DATE:");
    char* time_ptr = date_ptr ? strstr(date_ptr, ";TIME:") : NULL;
    char* period_ptr = time_ptr ? strstr(time_ptr, ";PERIOD:") : NULL;

    if(!date_ptr || !time_ptr || !period_ptr) {
        CDC_Transmit_FS((uint8_t*)"ERR: Invalid message format\r\n", 28);
        return;
    }

    int year, month, day, hour, min, sec;
    unsigned long period;

    if(sscanf(date_ptr, "DATE:%d-%d-%d", &year, &month, &day) != 3 ||
       sscanf(time_ptr, ";TIME:%d:%d:%d", &hour, &min, &sec) != 3 ||
       sscanf(period_ptr, ";PERIOD:%lu", &period) != 1) {
        CDC_Transmit_FS((uint8_t*)"ERR: Parsing failed\r\n", 20);
        return;
    }

    char usb_msg[128];
    snprintf(usb_msg, sizeof(usb_msg),
           "Date=%04d-%02d-%02d Time=%02d:%02d:%02d Period=%lus\r\n",
           year, month, day, hour, min, sec, period);

    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));
}

void Send_To_Backup_MK(DateTimeData* data) {
    char uart_msg[64];
    snprintf(uart_msg, sizeof(uart_msg),
           "DATE:%s;TIME:%s;PERIOD:%lu\r\n",
           data->date, data->date + 11, data->period);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_msg, strlen(uart_msg), 100);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_SPI3_Init();
  MX_DAC_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  Spi3_Init();

  float temperature = Read_Temperature();
  if(temperature < 0) {
      printf("Error: Thermocouple open!\n");
  } else {
      printf("Temperature: %.2f C\n", temperature);
  }

  HAL_Delay(1000);
  HAL_UART_Receive_IT(&huart1, &uart_byte, 1);
  Set_DAC_Voltage(0.0f);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
      if(new_data_received) {
          new_data_received = 0;
          Process_USB_Command(usb_rx_buffer);
          memset((void*)usb_rx_buffer, 0, sizeof(usb_rx_buffer));
      }

      if(uart_msg_ready) {
          uart_msg_ready = 0;
          Process_UART_Data(uart_buf);
          memset(uart_buf, 0, sizeof(uart_buf));
          uart_pos = 0;
      }

      // Точное циклическое изменение напряжения каждые 3 секунды
              if (HAL_GetTick() - dac_last_update > 3000) {
                  dac_last_update = HAL_GetTick();

                  if (dac_voltage < 0.1f) {          // Если ~0V
                      Set_DAC_Voltage(0.300f);       // Устанавливаем точно 0.5V
                  }
                  else if (dac_voltage < 0.6f) {     // Если ~0.5V
                      Set_DAC_Voltage(1.000f);       // Устанавливаем точно 1.0V
                  }
                  else {                             // Если ~1.0V
                      Set_DAC_Voltage(0.000f);       // Сбрасываем в 0V
                  }
              }

              HAL_Delay(1);

  }
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  DAC_ChannelConfTypeDef sConfig = {0};

  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|TH_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_13|TH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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
