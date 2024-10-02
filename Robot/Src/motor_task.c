#include "motor_task.h"

/**
 * @brief Motor task
 * @param argument: NULL
 * @retval None
 */
void motor_task(void const * argument){
    while (1){
        set_GM6020_motor_voltage(&hcan1, 1000);
        osDelay(1);
    }
}