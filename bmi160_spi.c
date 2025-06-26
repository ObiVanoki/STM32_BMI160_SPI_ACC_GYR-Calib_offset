#include "bmi160_spi.h"
#include <string.h>
#include <stdio.h>
#include "main.h"

#define BMI160_CHIP_ID         0x00
#define BMI160_CMD_REG_ADDR    0x7E
#define BMI160_SOFT_RESET_CMD  0xB6

#define BMI160_ACC_CONF_ADDR   0x40
#define BMI160_ACC_RANGE_ADDR  0x41
#define BMI160_GYR_CONF_ADDR   0x42
#define BMI160_GYR_RANGE_ADDR  0x43

#define BMI160_DATA_START      0x0C

#define ACC_RANGE_16G          0x03
#define GYR_RANGE_2000DPS      0x00

#define ACC_SCALE_16G          (2048.0f)     // LSB/g
#define GYR_SCALE_2000DPS      (16.4f)       // LSB/dps

#define CALIBRATION_SAMPLES    1000

extern UART_HandleTypeDef huart2;

static SPI_HandleTypeDef *bmi_hspi;
static GPIO_TypeDef *bmi_cs_port;
static uint16_t bmi_cs_pin;

static SensorData acc_offset = {0};
static SensorData gyro_offset = {0};

static void cs_low(void)  { HAL_GPIO_WritePin(bmi_cs_port, bmi_cs_pin, GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(bmi_cs_port, bmi_cs_pin, GPIO_PIN_SET); }

static uint8_t BMI160_ReadRegister(uint8_t reg) {
    uint8_t tx = reg | 0x80;
    uint8_t rx = 0;
    cs_low();
    HAL_SPI_Transmit(bmi_hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(bmi_hspi, &rx, 1, HAL_MAX_DELAY);
    cs_high();
    return rx;
}

static void BMI160_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { reg & 0x7F, value };
    cs_low();
    HAL_SPI_Transmit(bmi_hspi, tx, 2, HAL_MAX_DELAY);
    cs_high();
    HAL_Delay(1);
}

bool BMI160_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    bmi_hspi = hspi;
    bmi_cs_port = cs_port;
    bmi_cs_pin = cs_pin;

    HAL_Delay(100);
    BMI160_WriteRegister(BMI160_CMD_REG_ADDR, BMI160_SOFT_RESET_CMD);
    HAL_Delay(50);

    uint8_t id = BMI160_ReadRegister(BMI160_CHIP_ID);
    if (id != 0xD1 && id != 0x90) {
        char msg[64];
        sprintf(msg, "BMI160: Invalid CHIP_ID: 0x%02X\r\n", id);
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        return false;
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)"BMI160: CHIP_ID OK\r\n", 21, HAL_MAX_DELAY);

    BMI160_WriteRegister(BMI160_ACC_CONF_ADDR, 0x28);         // ODR=1600Hz, BW=OSR4
    BMI160_WriteRegister(BMI160_ACC_RANGE_ADDR, ACC_RANGE_16G);

    BMI160_WriteRegister(BMI160_GYR_CONF_ADDR, 0x28);         // ODR=3200Hz, BW=OSR4
    BMI160_WriteRegister(BMI160_GYR_RANGE_ADDR, GYR_RANGE_2000DPS);

    BMI160_WriteRegister(BMI160_CMD_REG_ADDR, 0x11);          // ACC normal mode
    HAL_Delay(10);
    BMI160_WriteRegister(BMI160_CMD_REG_ADDR, 0x15);          // GYR normal mode
    HAL_Delay(80);

    return true;
}

bool BMI160_ReadRawData(SensorData *acc, SensorData *gyro) {
    uint8_t reg = BMI160_DATA_START;
    uint8_t buf[12];

    reg |= 0x80;
    cs_low();
    HAL_SPI_Transmit(bmi_hspi, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(bmi_hspi, buf, 12, HAL_MAX_DELAY);
    cs_high();

    acc->x  = (int16_t)((buf[1] << 8) | buf[0]);
    acc->y  = (int16_t)((buf[3] << 8) | buf[2]);
    acc->z  = (int16_t)((buf[5] << 8) | buf[4]);

    gyro->x = (int16_t)((buf[7] << 8) | buf[6]);
    gyro->y = (int16_t)((buf[9] << 8) | buf[8]);
    gyro->z = (int16_t)((buf[11] << 8) | buf[10]);

    return true;
}

bool BMI160_ReadScaledData(float *acc_g, float *gyro_dps) {
    SensorData raw_acc, raw_gyro;

    if (!BMI160_ReadRawData(&raw_acc, &raw_gyro)) return false;

    // Відняти офсети
    raw_acc.x  -= acc_offset.x;
    raw_acc.y  -= acc_offset.y;
    raw_acc.z  -= acc_offset.z;

    raw_gyro.x -= gyro_offset.x;
    raw_gyro.y -= gyro_offset.y;
    raw_gyro.z -= gyro_offset.z;

    // Масштабування
    acc_g[0] = raw_acc.x / ACC_SCALE_16G;
    acc_g[1] = raw_acc.y / ACC_SCALE_16G;
    acc_g[2] = raw_acc.z / ACC_SCALE_16G;

    gyro_dps[0] = raw_gyro.x / GYR_SCALE_2000DPS;
    gyro_dps[1] = raw_gyro.y / GYR_SCALE_2000DPS;
    gyro_dps[2] = raw_gyro.z / GYR_SCALE_2000DPS;

    return true;
}

void BMI160_Calibrate(void) {
    SensorData acc_sum = {0}, gyro_sum = {0};
    SensorData acc, gyro;

    HAL_UART_Transmit(&huart2, (uint8_t *)"Calibrating BMI160...\r\n", 24, HAL_MAX_DELAY);

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        BMI160_ReadRawData(&acc, &gyro);

        acc_sum.x  += acc.x;
        acc_sum.y  += acc.y;
        acc_sum.z  += acc.z;

        gyro_sum.x += gyro.x;
        gyro_sum.y += gyro.y;
        gyro_sum.z += gyro.z;

        HAL_Delay(2);
    }

    acc_offset.x = acc_sum.x / CALIBRATION_SAMPLES;
    acc_offset.y = acc_sum.y / CALIBRATION_SAMPLES;
    acc_offset.z = (acc_sum.z / CALIBRATION_SAMPLES) - ACC_SCALE_16G; // Враховуємо 1g вниз

    gyro_offset.x = gyro_sum.x / CALIBRATION_SAMPLES;
    gyro_offset.y = gyro_sum.y / CALIBRATION_SAMPLES;
    gyro_offset.z = gyro_sum.z / CALIBRATION_SAMPLES;

    HAL_UART_Transmit(&huart2, (uint8_t *)"Calibration done.\r\n", 19, HAL_MAX_DELAY);
}
