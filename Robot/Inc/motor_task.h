#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "struct_typedef.h"
#include "bsp_can.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "pid.h"

#define PI 3.14159265358979323846f

/**
 * @brief Motor task
 * @param argument: NULL
 * @retval None
 */
extern void motor_task(void const * argument);
double msp(double x, double in_min, double in_max, double out_min, double out_max);

#endif // MOTOR_TASK_H
