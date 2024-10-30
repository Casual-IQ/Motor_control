#include "motor_task.h"

int16_t text_speed = 0;
int16_t target_yaw_speed;
float target_yaw_angle1 = 0, target_yaw_angle2 = 0;
float now_yaw_angle1, now_yaw_angle2;
extern moto_info_t motor_info1;
extern moto_info_t motor_info2;
extern pid_struct_t motor1_speed_pid;
extern pid_struct_t motor1_angle_pid;
extern pid_struct_t motor2_speed_pid;
extern pid_struct_t motor2_angle_pid;

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
    gimbal_PID_init(); // 初始化 PID 参数

    while (1){
        // 计算当前的编码器角度值，运用 msp 函数将编码器的值映射为弧度制
        now_yaw_angle1 = msp(motor_info1.rotor_angle, 0, 8191, -PI, PI);
        now_yaw_angle2 = msp(motor_info2.rotor_angle, 0, 8191, -PI, PI);

        // 电机 1 的角度环 PID 计算
        pid_calc(&motor1_angle_pid, target_yaw_angle1, now_yaw_angle1);
        // 电机 1 的速度环 PID 计算
        pid_calc(&motor1_speed_pid, motor1_angle_pid.output, motor_info1.rotor_speed);

        // 电机 2 的角度环 PID 计算
        pid_calc(&motor2_angle_pid, target_yaw_angle2, now_yaw_angle2);
        // 电机 2 的速度环 PID 计算
        pid_calc(&motor2_speed_pid, motor2_angle_pid.output, motor_info2.rotor_speed);

        // 设置两个电机的电压
        set_GM6020_motor_voltage(&hcan1, motor1_speed_pid.output, motor2_speed_pid.output);

        osDelay(40);
    }
}
