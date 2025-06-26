How used

This command is used for calibration. 
It is important that the sensor lies flat - do not move it.
if (BMI160_Init(&hspi1, GPIOA, GPIO_PIN_2)) {
    BMI160_Calibrate();  
}


float acc[3], gyro[3];
BMI160_ReadScaledData(acc, gyro);
// acc[] - g, gyro[] - gd/s
