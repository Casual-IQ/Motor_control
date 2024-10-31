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

typedef enum{
    Gimbal_No_Force = 0,
    Recenter_Yaw,
    Recenter_Pitch,
    Normal,
    // Auto_Aim_Debug,
    // No_Yaw,
    // No_Pitch,
    // Turn_round,
    // Fly,
} Gimbal_Mode;

void motor_init(void);
void mode_set(void);
void gimbal_control(void);
extern void motor_task(void const * argument);
double msp(double x, double in_min, double in_max, double out_min, double out_max);

#endif // MOTOR_TASK_H
