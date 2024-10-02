#ifndef LED_TASK_H
#define LED_TASK_H

#include "gpio.h"
#include "struct_typedef.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/**
 * @brief LED task
 * @param argument: NULL
 * @retval None
 */
extern void led_task(void const * argument);

#endif // LED_TASK_H
