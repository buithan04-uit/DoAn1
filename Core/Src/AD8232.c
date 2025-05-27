/*
 * AD8232.c
 *
 *  Created on: May 13, 2025
 *      Author: PC
 */

#include "AD8232.h"

// Extern từ main.c hoặc stm32f4xx_it.c
extern ADC_HandleTypeDef AD8232_ADC_HANDLE;

void AD8232_Init(void)
{
    // Đã khởi tạo ADC trong CubeMX rồi, nếu cần init lại:
    HAL_ADC_Start(&AD8232_ADC_HANDLE);
}

uint16_t AD8232_Read(void)
{
    uint16_t adc_value = 0;

    HAL_ADC_Start(&AD8232_ADC_HANDLE); // Bắt đầu chuyển đổi
    if (HAL_ADC_PollForConversion(&AD8232_ADC_HANDLE, 10) == HAL_OK)
    {
        adc_value = HAL_ADC_GetValue(&AD8232_ADC_HANDLE); // Lấy giá trị
    }
    HAL_ADC_Stop(&AD8232_ADC_HANDLE);
    return adc_value;
}

