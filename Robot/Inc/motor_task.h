#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "struct_typedef.h"
#include "bsp_can.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/**
 * @brief Motor task
 * @param argument: NULL
 * @retval None
 */
extern void motor_task(void const * argument);

#endif // MOTOR_TASK_H
