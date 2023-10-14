#include "Params.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/***********************************************************************
** 函 数 名： Params::initMotorsParams()
** 函数说明： 初始化各个电机参数，PID参数和任务时间周期参数
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Params::initMotorsParams()
{
	/************* 超级电容 MINIpc CAN包 *************/
	
	motor_params.CAN_SUPERCAP_RX = { .std_id = 0x301, .dlc = 8};//can2
	motor_params.CAN_MINIPC_RX = {.std_id = 0x500, .dlc = 8};//can1
	motor_params.CAN_MINIPC_TX = {};
	motor_params.CAN_SUPERCAP_TX = {.std_id = 0x401, .dlc = 8 };//can2
	
	
  /************** RM 电机 CAN总线地址 **************/

  motor_params.chassis_motor_1 = {.can_tx_id = 0x200, .can_rx_id = 0x201, .can_tx_data_start_pos = 0, .canx = 2};
  motor_params.chassis_motor_2 = {.can_tx_id = 0x200, .can_rx_id = 0x202, .can_tx_data_start_pos = 2, .canx = 2};
  motor_params.chassis_motor_3 = {.can_tx_id = 0x200, .can_rx_id = 0x203, .can_tx_data_start_pos = 4, .canx = 2};
  motor_params.chassis_motor_4 = {.can_tx_id = 0x200, .can_rx_id = 0x204, .can_tx_data_start_pos = 6, .canx = 2};
  motor_params.trigger_motor = {.can_tx_id = 0x1FF, .can_rx_id = 0x205, .can_tx_data_start_pos = 0, .canx = 2};
  motor_params.gimbal_yaw = {.can_tx_id = 0x1FF, .can_rx_id = 0x206, .can_tx_data_start_pos = 2, .canx = 2};

  motor_params.gimbal_pitch = {.can_tx_id = 0x1FF, .can_rx_id = 0x205, .can_tx_data_start_pos = 0, .canx = 1};
  motor_params.ammo_booster_motor_1 = {.can_tx_id = 0x1FF, .can_rx_id = 0x206, .can_tx_data_start_pos = 2, .canx = 1};
  motor_params.ammo_booster_motor_2 = {.can_tx_id = 0x1FF, .can_rx_id = 0x207, .can_tx_data_start_pos = 4, .canx = 1};

  /************** RM 电机 CAN总线地址 **************/



  /**************  电机参数  **************/

  // 摩擦轮1电机参数
  motor_params.ammo_booster_motor_1.reduction_ratio = 1.0f;
  motor_params.ammo_booster_motor_1.output_radius = 0.029f;
  motor_params.ammo_booster_motor_1.direction = MOTOR_CW;
  motor_params.ammo_booster_motor_1.max_value_ecd = 8192;
  motor_params.ammo_booster_motor_1.offset_ecd = 0;

  // 摩擦轮2电机参数
  motor_params.ammo_booster_motor_2.reduction_ratio = 1.0f;
  motor_params.ammo_booster_motor_2.output_radius = 0.029f;
  motor_params.ammo_booster_motor_2.direction = MOTOR_CCW;
  motor_params.ammo_booster_motor_2.max_value_ecd = 8192;
  motor_params.ammo_booster_motor_2.offset_ecd = 0;

  // 拨弹电机参数
  motor_params.trigger_motor.reduction_ratio = 3591.0f/187.0f;
  motor_params.trigger_motor.output_radius = 1.0f;
  motor_params.trigger_motor.direction = MOTOR_CCW;
  motor_params.trigger_motor.max_value_ecd = 8192;
  motor_params.trigger_motor.offset_ecd = 0;

  // Yaw轴云台电机参数
  motor_params.gimbal_yaw.reduction_ratio = 1.0f;
  motor_params.gimbal_yaw.output_radius = 1.0f;
  motor_params.gimbal_yaw.direction = MOTOR_CCW;
  motor_params.gimbal_yaw.max_value_ecd = 8192;
  motor_params.gimbal_yaw.offset_ecd =2432;//2248 ;//1826;

  // Pitch轴云台电机参数
  motor_params.gimbal_pitch.reduction_ratio = 1.0f;
  motor_params.gimbal_pitch.output_radius = 1.0f;
  motor_params.gimbal_pitch.direction = MOTOR_CW;
  motor_params.gimbal_pitch.max_value_ecd = 8192;
  motor_params.gimbal_pitch.offset_ecd = 263;//-2276;//3035;

  // 四个底盘电机电机参数
  motor_params.chassis_motor_1.reduction_ratio = 19.0f;
  motor_params.chassis_motor_1.output_radius = .075f;
  motor_params.chassis_motor_1.direction = MOTOR_CW;
  motor_params.chassis_motor_1.max_value_ecd = 8192;
  motor_params.chassis_motor_1.offset_ecd = 0;

  motor_params.chassis_motor_2.reduction_ratio = 19.0f;
  motor_params.chassis_motor_2.output_radius = .075f;
  motor_params.chassis_motor_2.direction = MOTOR_CW;
  motor_params.chassis_motor_2.max_value_ecd = 8192;
  motor_params.chassis_motor_2.offset_ecd = 0;

  motor_params.chassis_motor_3.reduction_ratio = 19.0f;
  motor_params.chassis_motor_3.output_radius = .075f;
  motor_params.chassis_motor_3.direction = MOTOR_CW;
  motor_params.chassis_motor_3.max_value_ecd = 8192;
  motor_params.chassis_motor_3.offset_ecd = 0;

  motor_params.chassis_motor_4.reduction_ratio = 19.0f;
  motor_params.chassis_motor_4.output_radius = .075f;
  motor_params.chassis_motor_4.direction = MOTOR_CW;
  motor_params.chassis_motor_4.max_value_ecd = 8192;
  motor_params.chassis_motor_4.offset_ecd = 0;

  /**************  电机参数  **************/



  /************** 电机PID参数 **************/

  // 摩擦轮1电机角速度环PID参数
  motor_params.ammo_booster_motor_1_ang_vel.type_selection = PID_DELTA;
  motor_params.ammo_booster_motor_1_ang_vel.kp = 5000;//5000;
  motor_params.ammo_booster_motor_1_ang_vel.ki = 100000;//50000;
  motor_params.ammo_booster_motor_1_ang_vel.kd_fb = 0;
  motor_params.ammo_booster_motor_1_ang_vel.kd_ex = 0;
  motor_params.ammo_booster_motor_1_ang_vel.k_ff = 0;
  motor_params.ammo_booster_motor_1_ang_vel.max_out_value = 16384;
  motor_params.ammo_booster_motor_1_ang_vel.min_out_value = -16384;
  motor_params.ammo_booster_motor_1_ang_vel.limit_output = true;
  motor_params.ammo_booster_motor_1_ang_vel.max_integral = 0.1;
  motor_params.ammo_booster_motor_1_ang_vel.min_integral = -0.1;
  motor_params.ammo_booster_motor_1_ang_vel.limit_integral = true;
  motor_params.ammo_booster_motor_1_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.ammo_booster_motor_1_ang_vel.kd_able_error_range = 0;
  motor_params.ammo_booster_motor_1_ang_vel.ki_able_error_range = 0;

  // 摩擦轮2电机角速度环PID参数
  motor_params.ammo_booster_motor_2_ang_vel.type_selection = PID_DELTA;
  motor_params.ammo_booster_motor_2_ang_vel.kp = 5000;//5000;
  motor_params.ammo_booster_motor_2_ang_vel.ki = 100000;//50000;
  motor_params.ammo_booster_motor_2_ang_vel.kd_fb = 0;
  motor_params.ammo_booster_motor_2_ang_vel.kd_ex = 0;
  motor_params.ammo_booster_motor_2_ang_vel.k_ff = 0;
  motor_params.ammo_booster_motor_2_ang_vel.max_out_value = 16384;
  motor_params.ammo_booster_motor_2_ang_vel.min_out_value = -16384;
  motor_params.ammo_booster_motor_2_ang_vel.limit_output = true;
  motor_params.ammo_booster_motor_2_ang_vel.max_integral = 0.1;
  motor_params.ammo_booster_motor_2_ang_vel.min_integral = -0.1;
  motor_params.ammo_booster_motor_2_ang_vel.limit_integral = true;
  motor_params.ammo_booster_motor_2_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.ammo_booster_motor_2_ang_vel.kd_able_error_range = 0;
  motor_params.ammo_booster_motor_2_ang_vel.ki_able_error_range = 0;

  // 拨弹盘角度环PID参数
  motor_params.trigger_motor_ang.type_selection = PID_ABSOLUTE;
  motor_params.trigger_motor_ang.kp = 15;//3.5;
  motor_params.trigger_motor_ang.ki = 0.01;//0.5;
  motor_params.trigger_motor_ang.kd_fb = 0.01;
  motor_params.trigger_motor_ang.kd_ex = 0;
  motor_params.trigger_motor_ang.k_ff = 0;
  motor_params.trigger_motor_ang.max_out_value = 16384;
  motor_params.trigger_motor_ang.min_out_value = -16384;
  motor_params.trigger_motor_ang.limit_output = true;
  motor_params.trigger_motor_ang.max_integral = 1;
  motor_params.trigger_motor_ang.min_integral = -1;
  motor_params.trigger_motor_ang.limit_integral = true;
  motor_params.trigger_motor_ang.add = &cmt::simple_adder_instance_f;
  motor_params.trigger_motor_ang.kd_able_error_range = 50.0f;
  motor_params.trigger_motor_ang.ki_able_error_range = 50.0f;

  // 拨弹盘角速度环PID参数
  motor_params.trigger_motor_ang_vel.type_selection = PID_DELTA;
  motor_params.trigger_motor_ang_vel.kp = 1200;
  motor_params.trigger_motor_ang_vel.ki = 22000;
  motor_params.trigger_motor_ang_vel.kd_fb = 0;
  motor_params.trigger_motor_ang_vel.kd_ex = 0;
  motor_params.trigger_motor_ang_vel.k_ff = 0;
  motor_params.trigger_motor_ang_vel.max_out_value = 16384;
  motor_params.trigger_motor_ang_vel.min_out_value = -16384;
  motor_params.trigger_motor_ang_vel.limit_output = true;
  motor_params.trigger_motor_ang_vel.max_integral = 0.1;
  motor_params.trigger_motor_ang_vel.min_integral = -0.1;
  motor_params.trigger_motor_ang_vel.limit_integral = true;
  motor_params.trigger_motor_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.trigger_motor_ang_vel.kd_able_error_range = 0;
  motor_params.trigger_motor_ang_vel.ki_able_error_range = 0;

  // 云台Yaw轴电机角度环PID参数
  motor_params.gimbal_yaw_ang.type_selection = PID_ABSOLUTE;
  motor_params.gimbal_yaw_ang.kp = 20;
  motor_params.gimbal_yaw_ang.ki = 0;
  motor_params.gimbal_yaw_ang.kd_fb = 0;
  motor_params.gimbal_yaw_ang.kd_ex = 0.0;
  motor_params.gimbal_yaw_ang.k_ff = 0;
  motor_params.gimbal_yaw_ang.max_out_value = 30;
  motor_params.gimbal_yaw_ang.min_out_value = -30;
  motor_params.gimbal_yaw_ang.limit_output = true;
  motor_params.gimbal_yaw_ang.max_integral = 0.1;
  motor_params.gimbal_yaw_ang.min_integral = -0.1;
  motor_params.gimbal_yaw_ang.limit_integral = true;
  motor_params.gimbal_yaw_ang.add = &cmt::simple_adder_instance_f;
  motor_params.gimbal_yaw_ang.kd_able_error_range = 0;
  motor_params.gimbal_yaw_ang.ki_able_error_range = 0;

  // 云台Yaw轴电机角速度环PID参数
  motor_params.gimbal_yaw_ang_vel.type_selection = PID_DELTA;
  motor_params.gimbal_yaw_ang_vel.kp = 20000;
  motor_params.gimbal_yaw_ang_vel.ki = 250000;
  motor_params.gimbal_yaw_ang_vel.kd_fb = 0;
  motor_params.gimbal_yaw_ang_vel.kd_ex = 0;
  motor_params.gimbal_yaw_ang_vel.k_ff = 0;
  motor_params.gimbal_yaw_ang_vel.max_out_value = 30000;
  motor_params.gimbal_yaw_ang_vel.min_out_value = -30000;
  motor_params.gimbal_yaw_ang_vel.limit_output = true;
  motor_params.gimbal_yaw_ang_vel.max_integral = 0.1;
  motor_params.gimbal_yaw_ang_vel.min_integral = -0.1;
  motor_params.gimbal_yaw_ang_vel.limit_integral = true;
  motor_params.gimbal_yaw_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.gimbal_yaw_ang_vel.kd_able_error_range = 0;
  motor_params.gimbal_yaw_ang_vel.ki_able_error_range = 0;

  // 云台Pitch轴电机角度环PID参数
  motor_params.gimbal_pitch_ang.type_selection = PID_ABSOLUTE;
  motor_params.gimbal_pitch_ang.kp = 18;
  motor_params.gimbal_pitch_ang.ki = 1;
  motor_params.gimbal_pitch_ang.kd_fb = 0;
  motor_params.gimbal_pitch_ang.kd_ex = 0;
  motor_params.gimbal_pitch_ang.k_ff = 0;
  motor_params.gimbal_pitch_ang.max_out_value = 30;
  motor_params.gimbal_pitch_ang.min_out_value = -30;
  motor_params.gimbal_pitch_ang.limit_output = true;
  motor_params.gimbal_pitch_ang.max_integral = 0.1;
  motor_params.gimbal_pitch_ang.min_integral = -0.1;
  motor_params.gimbal_pitch_ang.limit_integral = true;
  motor_params.gimbal_pitch_ang.add = &cmt::simple_adder_instance_f;
  motor_params.gimbal_pitch_ang.kd_able_error_range = 0;
  motor_params.gimbal_pitch_ang.ki_able_error_range = 0;

  // 云台Pitch轴电机角速度环PID参数
  motor_params.gimbal_pitch_ang_vel.type_selection = PID_DELTA;
  motor_params.gimbal_pitch_ang_vel.kp = 12000;
  motor_params.gimbal_pitch_ang_vel.ki = 200000;
  motor_params.gimbal_pitch_ang_vel.kd_fb = 0;
  motor_params.gimbal_pitch_ang_vel.kd_ex = 0;
  motor_params.gimbal_pitch_ang_vel.k_ff = 0;
  motor_params.gimbal_pitch_ang_vel.max_out_value = 30000;
  motor_params.gimbal_pitch_ang_vel.min_out_value = -30000;
  motor_params.gimbal_pitch_ang_vel.limit_output = true;
  motor_params.gimbal_pitch_ang_vel.max_integral = 0.1;
  motor_params.gimbal_pitch_ang_vel.min_integral = -0.1;
  motor_params.gimbal_pitch_ang_vel.limit_integral = true;
  motor_params.gimbal_pitch_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.gimbal_pitch_ang_vel.kd_able_error_range = 0;
  motor_params.gimbal_pitch_ang_vel.ki_able_error_range = 0;

  // 底盘四个电机PID参数PID参数
  motor_params.chassis_motor_1_ang_vel.type_selection = PID_DELTA;
  motor_params.chassis_motor_1_ang_vel.kp = 10000;
  motor_params.chassis_motor_1_ang_vel.ki = 200000;
  motor_params.chassis_motor_1_ang_vel.kd_fb = 0;
  motor_params.chassis_motor_1_ang_vel.kd_ex = 0;
  motor_params.chassis_motor_1_ang_vel.k_ff = 0;
  motor_params.chassis_motor_1_ang_vel.max_out_value = 10000;
  motor_params.chassis_motor_1_ang_vel.min_out_value = -10000;
  motor_params.chassis_motor_1_ang_vel.limit_output = true;
  motor_params.chassis_motor_1_ang_vel.max_integral = 0;
  motor_params.chassis_motor_1_ang_vel.min_integral = -0;
  motor_params.chassis_motor_1_ang_vel.limit_integral = true;
  motor_params.chassis_motor_1_ang_vel.add = &cmt::simple_adder_instance_f;
  motor_params.chassis_motor_1_ang_vel.kd_able_error_range = 0;
  motor_params.chassis_motor_1_ang_vel.ki_able_error_range = 0;

  motor_params.chassis_motor_2_ang_vel = motor_params.chassis_motor_1_ang_vel;
  motor_params.chassis_motor_3_ang_vel = motor_params.chassis_motor_1_ang_vel;
  motor_params.chassis_motor_4_ang_vel = motor_params.chassis_motor_1_ang_vel;

	//底盘跟随PID参数
	motor_params.chassis_follow_ang.type_selection = PID_ABSOLUTE;
  motor_params.chassis_follow_ang.kp = 0.004;
  motor_params.chassis_follow_ang.ki = 0.004;
  motor_params.chassis_follow_ang.kd_fb = 0.0001;
  motor_params.chassis_follow_ang.kd_ex = 0.0001;
  motor_params.chassis_follow_ang.k_ff = 0;
  motor_params.chassis_follow_ang.max_out_value = 4;
  motor_params.chassis_follow_ang.min_out_value = -4;
  motor_params.chassis_follow_ang.limit_output = true;
  motor_params.chassis_follow_ang.max_integral = 0.1;
  motor_params.chassis_follow_ang.min_integral = -0.1;
  motor_params.chassis_follow_ang.limit_integral = true;
  motor_params.chassis_follow_ang.add = &cmt::simple_adder_instance_f;
  motor_params.chassis_follow_ang.kd_able_error_range = 0;
  motor_params.chassis_follow_ang.ki_able_error_range = 0;
  /************** 电机PID参数 **************/


  /************** 任务频率 **************/

  // Yaw轴电机传感器数据获取任务与PID运算任务
  motor_params.gimbal_yaw.interval = 1e6f / 800.0f;
  motor_params.gimbal_yaw_ang.interval = 1e6f / 500.0f;
  motor_params.gimbal_yaw_ang_vel.interval = 1e6f / 800.0f;

  // Pitch轴电机传感器数据获取任务与PID运算任务
  motor_params.gimbal_pitch.interval = 1e6f / 800.0f;
  motor_params.gimbal_pitch_ang.interval = 1e6f / 500.0f;
  motor_params.gimbal_pitch_ang_vel.interval = 1e6f / 800.0f;

  // 四个底盘电机传感器数据获取任务与PID运算任务
  motor_params.chassis_motor_1.interval = 1e6f / 400.0f;
  motor_params.chassis_motor_1_ang_vel.interval = 1e6f / 400.0f;

  motor_params.chassis_motor_2.interval = 1e6f / 400.0f;
  motor_params.chassis_motor_2_ang_vel.interval = 1e6f / 400.0f;

  motor_params.chassis_motor_3.interval = 1e6f / 400.0f;
  motor_params.chassis_motor_3_ang_vel.interval = 1e6f / 400.0f;

  motor_params.chassis_motor_4.interval = 1e6f / 400.0f;
  motor_params.chassis_motor_4_ang_vel.interval = 1e6f / 400.0f;
	
	//底盘跟随云台PID运算任务
	motor_params.chassis_follow_ang.interval = 1e6f / 400.0f;

  // 两个摩擦轮电机传感器数据获取任务与PID运算任务
  motor_params.ammo_booster_motor_1.interval = 1e6f / 1000.0f;
  motor_params.ammo_booster_motor_1_ang_vel.interval = 1e6f / 400.0f;

  motor_params.ammo_booster_motor_2.interval = 1e6f / 1000.0f;
  motor_params.ammo_booster_motor_2_ang_vel.interval = 1e6f / 400.0f;

  // 拨弹电机传感器数据获取任务与PID运算任务
  motor_params.trigger_motor_ang.interval = 1e6f / 400.0f;
  motor_params.trigger_motor_ang_vel.interval = 1e6f / 400.0f;
  motor_params.trigger_motor.interval = 1e6f / 1000.0f;

  // 发弹任务
  motor_params.control_tasks_interval.ammo_task_interval = 1e6f / 400.0f;

  // 底盘任务
  motor_params.control_tasks_interval.chassis_task_interval = 1e6f / 400.0f;

  // 云台任务
  motor_params.control_tasks_interval.gimbal_task_interval = 1e6f / 800.0f;

  // LED 任务
  motor_params.control_tasks_interval.led_task_interval = 1e6f / 200.0f;

  // 裁判系统数据处理任务
  motor_params.control_tasks_interval.referee_system_task_interval = 1e6f / 100.0f;

  // CAN1 0x1FF地址发送任务
  motor_params.control_tasks_interval.can1_send_0x1ff_task_interval = 1e6f / 400.0f;

  // CAN2 0x1FF地址发送任务
  motor_params.control_tasks_interval.can2_send_0x1ff_task_interval = 1e6f / 400.0f;
  
  // CAN1 0x200地址发送任务
  motor_params.control_tasks_interval.can1_send_0x200_task_interval = 1e6f / 400.0f;

  // CAN2 0x200地址发送任务
  motor_params.control_tasks_interval.can2_send_0x200_task_interval = 1e6f / 400.0f;
	
  // CAN2 0x401地址发送任务
  motor_params.control_tasks_interval.can2_send_0x401_task_interval = 1e6f / 400.0f;
  /************** 任务频率 **************/


}

