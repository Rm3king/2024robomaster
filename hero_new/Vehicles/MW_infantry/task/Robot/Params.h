#ifdef TEMPLATE
#ifndef PARAMS_H
#define PARAMS_H

#include "Motor_RM.h"
#include "PID_Controller.h"

//任务周期
typedef struct _Control_Tasks_Interval_t
{
  float ammo_task_interval;
  float chassis_task_interval;
  float gimbal_task_interval;
  float led_task_interval;
  float referee_system_task_interval;
  float main_control_task_interval;
  float can1_send_0x1ff_task_interval;
  float can2_send_0x1ff_task_interval;
  float can2_send_0x200_task_interval;
  float can1_send_0x200_task_interval;
	float can2_send_0x401_task_interval;

} Control_Tasks_Interval_t;

//初始化参数结构体，包含了电机参数，PID参数和任务时间周期参数
typedef struct _Init_Params_t
{
  Motor_RM_Params_t gimbal_yaw,
                    gimbal_pitch,
                    chassis_motor_1,
                    chassis_motor_2,
                    chassis_motor_3,
                    chassis_motor_4,
                    ammo_booster_motor_1,
                    ammo_booster_motor_2,
                    trigger_motor;

  PID_Params_t gimbal_yaw_ang,
               gimbal_yaw_ang_vel,
               gimbal_pitch_ang,
               gimbal_pitch_ang_vel,
               chassis_motor_1_ang_vel,
               chassis_motor_2_ang_vel,
               chassis_motor_3_ang_vel,
               chassis_motor_4_ang_vel,
               ammo_booster_motor_1_ang_vel,
               ammo_booster_motor_2_ang_vel,
               trigger_motor_ang,
               trigger_motor_ang_vel,
							 chassis_follow_ang;
               
  Control_Tasks_Interval_t control_tasks_interval;

	CAN_Rx_Data_Pack_t CAN_SUPERCAP_RX,CAN_MINIPC_RX;
	
	CAN_Tx_Data_Link_t CAN_SUPERCAP_TX,CAN_MINIPC_TX;
} Init_Params_t;

class Params
{
public:
  Params() {}
  void initMotorsParams();
  Init_Params_t motor_params;

};




#endif
#endif