#include "led_task.h"

/**
 * @brief LED task
 * @param argument: NULL
 * @retval None
 */
void led_task(void const * argument){
    while (1){
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET); 
        osDelay(500); 
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET); 
        osDelay(500);
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET); 
        osDelay(500); 
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET); 
        osDelay(500); 
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET); 
        osDelay(500); 
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET); 
        osDelay(500); 
    }
}
