#include "motor_task.h"
#include "remote_control.h"

float target_yaw_angle = 0, target_pitch_angle = 2.6;
float now_yaw_angle, now_pitch_angle;
extern moto_info_t motor_info1;
extern moto_info_t motor_info2;
extern pid_struct_t motor1_speed_pid;
extern pid_struct_t motor1_angle_pid;
extern pid_struct_t motor2_speed_pid;
extern pid_struct_t motor2_angle_pid;

const RC_ctrl_t *rc_ctrl_motor;

float yaw_sensitivity = 0.00001f;
float pitch_sensitivity = 0.00001f; 
float yaw_min_angle = -PI;
float yaw_max_angle = PI;
float pitch_min_angle = -PI;
float pitch_max_angle = PI;

/**
 * @brief Motor task
 * @param argument: NULL
 * @retval None
 */
double msp(double x, double in_min, double in_max, double out_min, double out_max) // 映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi）
{
	return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

void motor_init(void){
    rc_ctrl_motor = get_remote_control_point();
    gimbal_PID_init(); // 初始化 PID 参数
}

void motor_task(void const * argument){
    motor_init();
    while (1){
        // 计算当前的编码器角度值，运用 msp 函数将编码器的值映射为弧度制
        now_yaw_angle = msp(motor_info1.rotor_angle, 0, 8191, -PI, PI);
        now_pitch_angle = msp(motor_info2.rotor_angle, 0, 8191, -PI, PI);

         // 根据遥控器数据设定目标偏航角和俯仰角
				
        target_yaw_angle += -(float)rc_ctrl_motor->rc.ch[0] * yaw_sensitivity;
        target_pitch_angle += -(float)rc_ctrl_motor->rc.ch[1] * pitch_sensitivity;

        // 对目标偏航角进行限幅
        if (target_yaw_angle < yaw_min_angle)
            target_yaw_angle = yaw_min_angle;
        if (target_yaw_angle > yaw_max_angle)
            target_yaw_angle = yaw_max_angle;

        // 对目标俯仰角进行限幅
        if (target_pitch_angle < pitch_min_angle)
            target_pitch_angle = pitch_min_angle;
        if (target_pitch_angle > pitch_max_angle)
            target_pitch_angle = pitch_max_angle;

        // 电机 1 的角度环 PID 计算
        pid_calc(&motor1_angle_pid, target_yaw_angle, now_yaw_angle);
        // 电机 1 的速度环 PID 计算
        pid_calc(&motor1_speed_pid, motor1_angle_pid.output, motor_info1.rotor_speed);

        // 电机 2 的角度环 PID 计算
        pid_calc(&motor2_angle_pid, target_pitch_angle, now_pitch_angle);
        // 电机 2 的速度环 PID 计算
        pid_calc(&motor2_speed_pid, motor2_angle_pid.output, motor_info2.rotor_speed);

        // 设置两个电机的电压
        set_GM6020_motor_voltage(&hcan1, motor1_speed_pid.output, motor2_speed_pid.output);

        osDelay(10);
    }
}
