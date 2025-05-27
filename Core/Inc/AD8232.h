#ifndef __AD8232_H
#define __AD8232_H

#include "stm32f4xx_hal.h"

// Chọn ADC handle, tùy theo bạn bật ADC1/ADC2/ADC3
#define AD8232_ADC_HANDLE hadc1

// Gọi hàm khởi tạo ADC từ main.c
void AD8232_Init(void);

// Đọc tín hiệu analog từ AD8232 (0 ~ 4095)
uint16_t AD8232_Read(void);

#endif
