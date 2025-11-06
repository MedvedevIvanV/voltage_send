#include "u_utils.h"
#include "main.h"
#include <stdio.h>

void GetAdcValues(TU1AdcValues *const OutVal)
{


	// Настраиваем АЦП
	// Включаем питание АЦП
  // Включаем питание датчика температуры, напряжения батареи и опорного напряжения
  MODIFY_REG(ADC12_COMMON->CCR, ADC_CCR_PRESC_Msk | ADC_CCR_CKMODE_Msk, ADC_CCR_TSEN | ADC_CCR_VBATEN | ADC_CCR_VREFEN | ADC_CCR_CKMODE_1);
  // Задержка для стабилизации датчика температуры
  volatile int i;
  for(i = 0; i < 1000; i++); // Короткая задержка
  MODIFY_REG(ADC12_COMMON->CCR, 0, ADC_CCR_TSEN | ADC_CCR_VBATEN | ADC_CCR_VREFEN);

   MODIFY_REG(ADC1->CR,   0,  ADC_CR_ADEN);
   while(!(ADC1->ISR & ADC_ISR_ADRDY));

   MODIFY_REG(ADC1->CR,   ADC_CR_ADCALDIF,  0);
   MODIFY_REG(ADC1->CR,   0,  ADC_CR_ADCAL);
   while(!(ADC1->CR & ADC_CR_ADCAL));

   HAL_Delay(10);


   // Устанавливаем максимальное время выборки-хранения для этих каналов и 5 входа АЙП
   MODIFY_REG(ADC1->SMPR1, 0, ADC_SMPR1_SMP0 | ADC_SMPR1_SMP5);
   MODIFY_REG(ADC1->SMPR2, 0, ADC_SMPR2_SMP17 | ADC_SMPR2_SMP18);


   // Будем преобразовывать По одному входу за измерение
   MODIFY_REG(ADC1->SQR1, ADC_SQR1_L, (0 << ADC_SQR1_L_Pos));

   // Получаем напряжение питания c 5 входа АЦП (вывод PA0 - 10 нога)
   MODIFY_REG(ADC1->SQR1, ADC_SQR1_SQ1,  (5 << ADC_SQR1_SQ1_Pos));
   MODIFY_REG(ADC1->CR,   0, ADC_CR_ADSTART);
   while(!(ADC1->ISR & ADC_ISR_EOS));
   unsigned int PA10_DATA = ADC1->DR;
   WRITE_REG(ADC1->ISR, ADC_ISR_EOS);

   // Получаем напряжение опорного источника 0 вход АЦП
   MODIFY_REG(ADC1->SQR1, ADC_SQR1_SQ1,  (0 << ADC_SQR1_SQ1_Pos));
   MODIFY_REG(ADC1->CR,   0,     ADC_CR_ADSTART);
   while(!(ADC1->ISR & ADC_ISR_EOS));
   unsigned int VREFINT_DATA = ADC1->DR;
   WRITE_REG(ADC1->ISR, ADC_ISR_EOS);

   // Получаем 1/3 напряжение питания МК 18 вход АЦП
   MODIFY_REG(ADC1->SQR1, ADC_SQR1_SQ1,  (18 << ADC_SQR1_SQ1_Pos));
   MODIFY_REG(ADC1->CR,   0,     ADC_CR_ADSTART);
   while(!(ADC1->ISR & ADC_ISR_EOS));
   unsigned int VBAT_DATA = ADC1->DR;
   WRITE_REG(ADC1->ISR, ADC_ISR_EOS);

   // Получаем напряжение датчика температуры 17 вход АЦП
   MODIFY_REG(ADC1->SQR1, ADC_SQR1_SQ1,  (17 << ADC_SQR1_SQ1_Pos));
   MODIFY_REG(ADC1->CR,   0,     ADC_CR_ADSTART);
   while(!(ADC1->ISR & ADC_ISR_EOS));
   unsigned int TEMPERETURE_DATA = ADC1->DR;
   WRITE_REG(ADC1->ISR, ADC_ISR_EOS);

   MODIFY_REG(ADC1->CR, ADC_CR_ADEN,  0);

   // VREF parameters Table 15. Internal voltage reference calibration values page 62 of stm32l552cc.pdf
   uint16_t TS_CAL1 = *((unsigned short*) 0x0BFA05A8UL);
   uint16_t TS_CAL2 = *((unsigned short*) 0x0BFA05CAUL);
   const float  TS_1  = 30.0f;
   const float  TS_2  = 130.0f;
   uint16_t VREFINT_CALIB = *((unsigned short*) 0x0BFA05AAUL);

   // Корректируем записанные производителем коэффициенты в соответствии с текущим напряжением питания
   // Информация получена из статьи
   // https://programel.ru/%D0%BF%D0%BE%D0%BB%D1%83%D1%87%D0%B0%D0%B5%D0%BC-%D1%82%D0%B5%D0%BC%D0%BF%D0%B5%D1%80%D0%B0%D1%82%D1%83%D1%80%D1%83-%D0%BA%D0%BE%D0%BD%D1%82%D1%80%D0%BE%D0%BB%D0%BB%D0%B5%D1%80%D0%B0-stm32l4-%D0%BF/
   // https://programel.ru/получаем-температуру -контроллера-stm32l4-п/
   const float v_calib = (float)VREFINT_DATA / (float)VREFINT_CALIB;
   float TS_CALIB1_NEW = TS_CAL1 * v_calib;
   float TS_CALIB2_NEW = TS_CAL2 * v_calib;

   OutVal->ChipTemperature = (((float)TEMPERETURE_DATA - TS_CALIB1_NEW) * (TS_2 - TS_1) / (TS_CALIB2_NEW - TS_CALIB1_NEW)) + TS_1;

   const float Vdda_Val = (3.0f * (float)VREFINT_CALIB)/(float)VREFINT_DATA;
   const float CodeToVoltage = Vdda_Val / 4096;

   // Переключаемый делитель напряжения
   // Вычисляем переходную характеристику в зависимости от состояния переключателя
   const float R10 = 36.5, R8 = 20.0, R11 = 10.0;
   const float RLow = (HAL_GPIO_ReadPin(Bat23_GPIO_Port, Bat23_Pin)) ? ((R11 * R8) / (R11 + R8)): R11;
   const float PA10TransferK = (R10 + RLow) / RLow;

   OutVal->PA10_Voltage = PA10_DATA * CodeToVoltage * PA10TransferK;
   OutVal->Vdd_Voltage = VBAT_DATA * CodeToVoltage * 3;
}
