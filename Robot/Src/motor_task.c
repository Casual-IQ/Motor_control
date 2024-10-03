#include "motor_task.h"

int16_t text_speed = 0;
int16_t target_yaw_speed;
float target_yaw_angle = 0;
float now_yaw_angle;
extern moto_info_t motor_info;
extern pid_struct_t motor_speed_pid;
extern pid_struct_t motor_angle_pid;
/**
 * @brief Motor task
 * @param argument: NULL
 * @retval None
 */

double msp(double x, double in_min, double in_max, double out_min, double out_max)//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi）
{
	return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

void motor_task(void const * argument){
    gimbal_PID_init();//初始化PID参数
    while (1){
        now_yaw_angle=msp(motor_info.rotor_angle,0,8191,-PI,PI);//计算当前的编码器角度值，运用msp函数将编码器的值映射为弧度制
 
		pid_calc(&motor_angle_pid, target_yaw_angle, now_yaw_angle);//角度环
		pid_calc(&motor_speed_pid, motor_angle_pid.output, motor_info.rotor_speed);//速度环
		
		set_GM6020_motor_voltage(&hcan1,motor_speed_pid.output);//can发送函数，发送经过PID计算的电压值
        osDelay(40);
    }
}
