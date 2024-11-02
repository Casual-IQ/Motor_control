#include "imu_task.h"
#include "BMI088Middleware.h"
#include "BMI088driver.h"

fp32 gyro[3], accel[3], temp;

void EKF(void const * argument){
    while(BMI088_init())
    {
        ;
    }
    while (1)
    {
        BMI088_read(gyro, accel, &temp);
        osDelay(10);
    }
    
}
