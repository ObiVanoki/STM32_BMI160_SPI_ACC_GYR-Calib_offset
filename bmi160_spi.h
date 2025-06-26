#ifndef BMI160_SPI_H
#define BMI160_SPI_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} SensorData;

bool BMI160_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
void BMI160_Calibrate(void);
bool BMI160_ReadRawData(SensorData *acc, SensorData *gyro);
bool BMI160_ReadScaledData(float *acc_g, float *gyro_dps);

#endif
  
