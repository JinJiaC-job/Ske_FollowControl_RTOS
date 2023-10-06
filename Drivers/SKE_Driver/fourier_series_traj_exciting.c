#include "main.h"
#include "math.h"
#include "stdlib.h"
#include "lkmoto.h"
#include "ids830can.h"
#include "fourier_series_traj_exciting.h"
#include "tim.h"

//电机控制时间节点：
unsigned int motor_control_k = 0;
//motor control interval time：每过motor control interval time秒输出一次控制命令
float control_interval_time = 0.05;
// sampling period
float traj_Ts = 0.1;
// trajectory fundamental frequency = 1/T; T = run time of skeleton = 20s.
float traj_f = 0.05;
// trajectory fundamental frequency in radian
float traj_wf;
// number of sampling points
uint8_t traj_n;
// order of trajectory generation 
uint8_t traj_order = 5;
// number of revolute joints
uint8_t dof = 6;
// 6个关节角度信息
float q[7] = {0};
float q_last[7] = {0, 0, 0, 0, 0, 0, 0};
float q_next[7] = {0};
// fourier series params
float traj_param[] = {0,
-8.1665,
-0.91963,
-2.6538,
14.5642,
8.8495,
12.9341,
10.4759,
-6.7139,
-8.505,
-8.0311,
118.8705,
0.1125,
0.028044,
0.10917,
0.072059,
0.030441,
0.026517,
-0.35127,
-0.13509,
0.099166,
0.057732,
0.16134,
0.13232,
-0.12859,
0.17561,
0.021126,
0.049659,
-0.04276,
0.051084,
-0.29927,
-0.40868,
0.28234,
-0.47946,
0.10572,
0.016394,
-0.12149,
0.24361,
0.19525,
0.089167,
-0.10414,
-0.050933,
-0.075338,
-0.11348,
-1.1491,
-0.052254,
0.27284,
-0.25721,
0.25012,
-0.1642,
-0.068208,
0.063287,
0.13132,
0.41038,
-0.21875,
2.7302,
-0.039817,
0.15864,
0.0642,
-0.18079,
0.33285,
-0.31627,
-0.092946,
0.057923,
-0.26429,
0.18401,
0.044888};


void fourier_series_traj(float time)
{
	uint8_t order_prod_2, m;
  order_prod_2 = traj_order * 2;

	traj_wf = traj_f * 2.0 * pi;
	
	for(int i=1; i<=dof; i++)
	{
		m = (order_prod_2 + 1) * (i - 1); 
		q[i] = traj_param[m + order_prod_2 + 1];//q0
		for(int j=1; j<=traj_order; j++)
		{
			// alpha(a)=traj_param(m+2*(j-1)+1), beta(b)=traj_param(m+2*(j-1)+2)
			q[i] = q[i] + ((traj_param[m + 2*(j-1) + 1] / (traj_wf * j)) * sin(traj_wf * j * time) - (traj_param[m + 2*(j-1) + 2] / (traj_wf * j)) * cos(traj_wf * j * time));
//			if(motor_control_k<200)
//			{
//				q_next[i] = q[i] + ((traj_param[m + 2*(j-1) + 1] / (traj_wf * j)) * sin(traj_wf * j * (time+control_interval_time)) - (traj_param[m + 2*(j-1) + 2] / (traj_wf * j)) * cos(traj_wf * j * (time+control_interval_time)));
//			}
//			else
//				q_next[i] = q[i];
		}
	}
}

void traj_exciting_init(void)
{
	traj_n = 1.0 / control_interval_time / traj_f;
}

void run_fourier_series_traj(void)
{
	float motor_speed = 0;

	fourier_series_traj(motor_control_k*control_interval_time);//电机控制信号点
	for(int i=1; i<=6; i++)
	{
		if(i == 1)
		{
			q[i] = q[i]-95.35;//直线电缸单位mm
//			q_next[i] = q_next[i]*1000;
			motor_speed = fabs((q[i]-q_last[i])/control_interval_time);//fabs:float类型的绝对值函数
			LinearActuator_startRun_maxspeed_position(i, q[i], motor_speed);
			q_last[i] = q[i];
//			printf("linear=%.3f, linearSpeed=%.3f\r\n",q[1], motor_speed);
		}
		else if(i == 4)
		{
			q[i] = q[i]/pi*180 + 90;
//			q_next[i] = q_next[i]/pi*180;
			motor_speed = fabs((q[i]-q_last[i])/control_interval_time);//fabs:float类型的绝对值函数
			if(motor_speed<1)//速度不能太小
				motor_speed=1;
			angle_close_loop_with_speed(i, -q[i], motor_speed);
			q_last[i] = q[i];
//			printf("MOTORSPEED4=%.3f, MOTORpos4=%.3f\r\n", motor_speed, q[4]);
		}
		else if(i == 5)
		{
			q[i] = q[i]/pi*180 - 90;
//			q_next[i] = q_next[i]/pi*180;
			motor_speed = fabs((q[i]-q_last[i])/control_interval_time);//fabs:float类型的绝对值函数
			if(motor_speed<1)
				motor_speed=1;
			angle_close_loop_with_speed(i, q[i], motor_speed);
			q_last[i] = q[i];
		}
		else if(i == 6)
		{
			if((fabs(q[i]-q_last[i]))>=0.05)
			{
				q[i] = q[i]/pi*180;
//				q_next[i] = q_next[i]/pi*180;
				motor_speed = fabs((q[i]-q_last[i])/control_interval_time);//fabs:float类型的绝对值函数
				if(motor_speed<1)
				  motor_speed=1;
				angle_close_loop_with_speed(i, q[i], motor_speed);
				q_last[i] = q[i];
			}
		}
		else
		{
			q[i] = q[i]/pi*180;
//			q_next[i] = q_next[i]/pi*180;
			motor_speed = fabs((q[i]-q_last[i])/control_interval_time);//fabs:float类型的绝对值函数
			if(motor_speed<1)
				motor_speed=1;
			angle_close_loop_with_speed(i, q[i], motor_speed);
			q_last[i] = q[i];
//			if(i==6)
//				printf("MOTORSPEED6=%.3f, MOTORpos=%.3f\r\n", motor_speed, q[6]);
		}
	}
//	printf("k=%d", motor_control_k);
	if(++motor_control_k > 400)
		HAL_TIM_Base_Stop_IT(&htim2);
}




