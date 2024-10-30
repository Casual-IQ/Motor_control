#include "pid.h"

pid_struct_t motor1_speed_pid;
pid_struct_t motor1_angle_pid;
pid_struct_t motor2_speed_pid;
pid_struct_t motor2_angle_pid;
 
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)//PID初始化函数
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}
 
 
float pid_calc(pid_struct_t *pid, float ref, float fdb)//PID运算函数
{
  pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->ref - pid->fdb;
  
  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}
 
void gimbal_PID_init() // 角度环和速度环的 PID 初始化
{
    // 电机 1 的 PID 初始化
    pid_init(&motor1_speed_pid, 25, 0, 0.003, 30000, 30000); // P=25, I=0, D=0.003
    pid_init(&motor1_angle_pid, 450, 0.01, 0.002, 320, 320); // P=450, I=0.01, D=0.002

    // 电机 2 的 PID 初始化
    pid_init(&motor2_speed_pid, 25, 0, 0.003, 30000, 30000); // P=25, I=0, D=0.003
    pid_init(&motor2_angle_pid, 450, 0.01, 0.002, 320, 320); // P=450, I=0.01, D=0.002

    // 激进参数
    // pid_init(&motor1_speed_pid, 30, 0, 0.03, 30000, 30000); // P=30, I=0, D=0.03
    // pid_init(&motor1_angle_pid, 350, 0.01, 0.08, 320, 320); // P=350, I=0.01, D=0.08
    // pid_init(&motor2_speed_pid, 30, 0, 0.03, 30000, 30000); // P=30, I=0, D=0.03
    // pid_init(&motor2_angle_pid, 350, 0.01, 0.08, 320, 320); // P=350, I=0.01, D=0.08
}
