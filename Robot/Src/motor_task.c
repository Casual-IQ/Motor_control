#include "motor_task.h"
#include "remote_control.h"
#include <stdlib.h> 

extern moto_info_t motor1_info;
extern pid_struct_t motor1_speed_pid;
extern pid_struct_t motor1_angle_pid;
extern moto_info_t motor2_info;
extern pid_struct_t motor2_speed_pid;
extern pid_struct_t motor2_angle_pid;

const float yaw_sensitivity = 0.0001f;
const float pitch_sensitivity = 0.0001f; 
const float yaw_min_angle = -PI;
const float yaw_max_angle = PI;
const float pitch_min_angle = 1.95f;
const float pitch_max_angle = 2.90f;
const float recenter_threshold = 0.01f;
const float yaw_init_angle = 0.0f;
const float pitch_init_angle = 2.6f;

const RC_ctrl_t *rc_ctrl_motor;

float target_yaw_angle, target_pitch_angle;
float now_yaw_angle, now_pitch_angle;
float yaw_err;
Gimbal_Mode gimbal_mode;

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
    target_yaw_angle = yaw_init_angle;
    target_pitch_angle = pitch_init_angle;

    gimbal_mode = Gimbal_No_Force;
    rc_ctrl_motor = get_remote_control_point();
    gimbal_PID_init(); // 初始化 PID 参数
}

void mode_set(void){
    if (switch_is_down(rc_ctrl_motor->rc.s[0]))
    {
        gimbal_mode = Gimbal_No_Force;
    }
    switch (gimbal_mode){
        case Gimbal_No_Force:
            if (switch_is_mid(rc_ctrl_motor->rc.s[0]))
            {
                gimbal_mode = Recenter_Yaw;
            }
            break;
        case Recenter_Yaw:
            if (abs(now_yaw_angle-target_yaw_angle) < recenter_threshold){
                gimbal_mode = Recenter_Pitch;
            }
            break;
        case Recenter_Pitch:
            if (abs(now_pitch_angle-target_pitch_angle) < recenter_threshold){
                gimbal_mode = Normal;
            }
            break;
        case Normal:
            break;
        default:
            break;
    }
}

void gimble_control(void){
    switch(gimbal_mode){
        case(Gimbal_No_Force):
            motor1_speed_pid.output = 0;
            motor2_speed_pid.output = 0;
            break;
        case(Recenter_Yaw):
            target_yaw_angle = yaw_init_angle;
            yaw_err = target_yaw_angle - now_yaw_angle;
            if (yaw_err > PI)
                yaw_err -= 2 * PI;
            if (yaw_err < -PI)
                yaw_err += 2 * PI;
            pid_calc(&motor1_angle_pid, yaw_err, 0.0);
            pid_calc(&motor1_speed_pid, motor1_angle_pid.output, motor1_info.rotor_speed);
            break;
        case(Recenter_Pitch):
            target_pitch_angle = pitch_init_angle;
            pid_calc(&motor2_angle_pid, target_pitch_angle, now_pitch_angle);
            pid_calc(&motor2_speed_pid, motor2_angle_pid.output, motor2_info.rotor_speed);
            break;
        case(Normal):
            // 根据遥控器数据设定yaw和pitch的目标角度
            target_yaw_angle += -(float)rc_ctrl_motor->rc.ch[0] * yaw_sensitivity;
            target_pitch_angle += -(float)rc_ctrl_motor->rc.ch[1] * pitch_sensitivity;

            // 对yaw进行连续性处理
            if (target_yaw_angle < -PI)
                target_yaw_angle += 2 * PI;
            if (target_yaw_angle > PI)
                target_yaw_angle -= 2 * PI;

            // yaw优弧劣弧处理
            yaw_err = target_yaw_angle - now_yaw_angle;
            if (yaw_err > PI)
                yaw_err -= 2 * PI;
            if (yaw_err < -PI)
                yaw_err += 2 * PI;
            
            // 对pitch限幅
            if (target_pitch_angle < pitch_min_angle)
                target_pitch_angle = pitch_min_angle;
            if (target_pitch_angle > pitch_max_angle)
                target_pitch_angle = pitch_max_angle;

            pid_calc(&motor1_angle_pid, yaw_err, 0.0);
            pid_calc(&motor1_speed_pid, motor1_angle_pid.output, motor1_info.rotor_speed);
            pid_calc(&motor2_angle_pid, target_pitch_angle, now_pitch_angle);
            pid_calc(&motor2_speed_pid, motor2_angle_pid.output, motor2_info.rotor_speed);

    }
}

void motor_task(void const * argument){
    motor_init();
    while (1){
        //msp 函数将编码器的值映射为弧度制
        now_yaw_angle = msp(motor1_info.rotor_angle, 0, 8191, -PI, PI);
        now_pitch_angle = msp(motor2_info.rotor_angle, 0, 8191, -PI, PI);
        mode_set();
        gimble_control();
        set_GM6020_motor_voltage(&hcan1, motor1_speed_pid.output, motor2_speed_pid.output);

        osDelay(10);
    }
}
