#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"
#include "stm32f4xx.h"
#include "can.h"

typedef struct
{
    uint16_t can_id;
    int16_t  set_voltage;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  temp;
} moto_info_t;

void CAN_Filter_Init(void);
void set_GM6020_motor_voltage(CAN_HandleTypeDef* hcan, int16_t v1, int16_t v2);

#endif
