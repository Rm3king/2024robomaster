/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   AmmoRM_ControlTask.cpp
** 文件说明：   云台控制任务
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     林子涵     	     2022-07-26
***************************************************************************/
#include "/Ammo/AmmoControlTask.h"
#include "/Robot/Robot.h"
#include "UARTDriver.h"
#include "/Referee/RefereeSystemTask.h"

#define REGIONS_NUM 6.0f//拨弹孔位

/***********************************************************************
** 函 数 名： AmmoRM_ControlTask构造函数
** 函数说明： 指定robot引用，CAN设备引用，指定yaw和pitch轴电机CAN总线id
**            和CAN总线数据包发送起始下标，指定任务更新时间间隔，同时
**            给yaw和pitch轴电机实例化Motor_RM_PIDControlTask任务
**---------------------------------------------------------------------
** 输入参数： robot引用、CAN设备引用、yaw和pitch轴电机CAN总线id、
**            CAN总线数据包发送起始下标、任务更新时间间隔
** 返回参数： 无
***********************************************************************/
AmmoRM_ControlTask::AmmoRM_ControlTask(
  Robot &robot0,
  Motor_RM_Params_t *booster_1, PID_Params_t *booster_1_ang, PID_Params_t *booster_1_ang_vel,
  Motor_RM_Params_t *booster_2, PID_Params_t *booster_2_ang, PID_Params_t *booster_2_ang_vel,
  Motor_RM_Params_t *trigger, PID_Params_t *trigger_ang, PID_Params_t *trigger_ang_vel,
  timeus_t interval_tick_us0
) : Task_Base(robot0)
{
  this->interval_tick_us = interval_tick_us0;
  

  if(booster_1 != NULL)
  {
    if(booster_1->canx == 1)
    {
      this->motor_booster_1_pid_task = new Motor_RM_PIDControlTask(robot0,//显式实例化
          robot0.can1_device,
          booster_1->can_rx_id,
          booster_1->can_tx_id,
          booster_1->can_tx_data_start_pos,
          RoboMaster_3508,
          booster_1->interval);
    }
    else if(booster_1->canx == 2)
    {
      this->motor_booster_1_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          booster_1->can_rx_id,
          booster_1->can_tx_id,
          booster_1->can_tx_data_start_pos,
          RoboMaster_3508,
          booster_1->interval);
    }
    this->motor_booster_1_pid_task->motor_backend_p->setParams(*booster_1);
  }
  
  if(booster_2 != NULL)
  {
    if(booster_2->canx == 1)
    {
      this->motor_booster_2_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          booster_2->can_rx_id,
          booster_2->can_tx_id,
          booster_2->can_tx_data_start_pos,
          RoboMaster_3508,
          booster_2->interval);
    }
    else if(booster_2->canx == 2)
    {
      this->motor_booster_2_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          booster_2->can_rx_id,
          booster_2->can_tx_id,
          booster_2->can_tx_data_start_pos,
          RoboMaster_3508,
          booster_2->interval);
    }
    this->motor_booster_2_pid_task->motor_backend_p->setParams(*booster_2);
  }
  
  if(trigger != NULL)
  {
    if(trigger->canx == 1)
    {
      this->motor_trigger_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          trigger->can_rx_id,
          trigger->can_tx_id,
          trigger->can_tx_data_start_pos,
          RoboMaster_3508,
          trigger->interval);
    }
    else if(trigger->canx == 2)
    {
      this->motor_trigger_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          trigger->can_rx_id,
          trigger->can_tx_id,
          trigger->can_tx_data_start_pos,
          RoboMaster_3508,
          trigger->interval);
    }
    this->motor_trigger_pid_task->motor_backend_p->setParams(*trigger);
  }
  
  if(booster_2_ang_vel != NULL)
  {
    this->motor_booster_2_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*booster_2_ang_vel);
    this->motor_booster_2_pid_task->angular_velocity_control_task_p->setInterval((*booster_2_ang_vel).interval);
  }

//  if(booster_2_ang_vel != NULL)
//  {
//    this->motor_booster_2_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*booster_2_ang_vel);
//    this->motor_booster_2_pid_task->angular_velocity_control_task_p->setInterval((*booster_2_ang_vel).interval);
//  }
  
  
  if(booster_1_ang_vel != NULL)
  {
    this->motor_booster_1_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*booster_1_ang_vel);
    this->motor_booster_1_pid_task->angular_velocity_control_task_p->setInterval((*booster_1_ang_vel).interval);
  }

//  if(booster_1_ang_vel != NULL)
//  {
//    this->motor_booster_1_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*booster_1_ang_vel);
//    this->motor_booster_1_pid_task->angular_velocity_control_task_p->setInterval((*booster_1_ang_vel).interval);
//  }  
  
  
  if(trigger_ang != NULL)
  {
    this->motor_trigger_pid_task->angle_control_task_p->setPIDControllerParams(*trigger_ang);
    this->motor_trigger_pid_task->angle_control_task_p->setInterval((*trigger_ang).interval);
  }

  if(trigger_ang_vel != NULL)
  {
    this->motor_trigger_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*trigger_ang_vel);
    this->motor_trigger_pid_task->angular_velocity_control_task_p->setInterval((*trigger_ang_vel).interval);
  }

}

/***********************************************************************
** 函 数 名： AmmoRM_ControlTask::init()
** 函数说明： 注册云台各个电机Motor_RM_PIDControlTask任务
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void AmmoRM_ControlTask::init(void)
{
  inited = true;
	if(motor_booster_2_pid_task != NULL)
  robot.scheduler.registerTask(motor_booster_2_pid_task);
	if(motor_booster_1_pid_task != NULL)	
  robot.scheduler.registerTask(motor_booster_1_pid_task);
	if(motor_trigger_pid_task != NULL)		
  robot.scheduler.registerTask(motor_trigger_pid_task);
	
	if(motor_booster_2_pid_task->angular_velocity_control_task_p != NULL)
		 robot.scheduler.registerTask(motor_booster_2_pid_task->angular_velocity_control_task_p);
	if(motor_booster_1_pid_task->angular_velocity_control_task_p != NULL)
		 robot.scheduler.registerTask(motor_booster_1_pid_task->angular_velocity_control_task_p);
	if(motor_trigger_pid_task->angle_control_task_p != NULL)
		 robot.scheduler.registerTask(motor_trigger_pid_task->angle_control_task_p);
	if(motor_trigger_pid_task->angular_velocity_control_task_p != NULL)
		 robot.scheduler.registerTask(motor_trigger_pid_task->angular_velocity_control_task_p);
	booster_mode = BOOSTER_OFF;
  exp_spd = shoot_speed_16;
  shoot_freq = 0;
	moving_flag = 0;
	upper_clk=200;
	HAL_Delay(10);
	//exp_tri_angle=motor_trigger_pid_task->motor_backend_p->getCommonMeasurement().output.angle;	`
}
	 
/***********************************************************************
** 函 数 名： AmmoRM_ControlTask::update(timeus_t dT_us)
** 函数说明： 通过云台电机编码器和IMU计算云台Yaw轴机构、车底盘
**            等部分的空间姿态，更新
云台各个电机控制任务，并发送
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void AmmoRM_ControlTask::update(timeus_t dT_us)
{
  float dT_s = dT_us / 1e6f;
  
  // 右边摩擦轮
	//设置反馈
	if(exp_spd_flag == 1)
	{
		if(exp_spd >15)
		{
			exp_spd = shoot_speed_10;
		}
		else if(exp_spd <15)
		{
			exp_spd = shoot_speed_16;
		}
		clear_exp_spd_flag();
	}
  motor_booster_2_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(
    motor_booster_2_pid_task->motor_backend_p->getCommonMeasurement().linear.velocity
  );

  motor_booster_2_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
    -exp_spd
  );

  // 左边摩擦轮
  motor_booster_1_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(
    motor_booster_1_pid_task->motor_backend_p->getCommonMeasurement().linear.velocity
  );

  motor_booster_1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
    -exp_spd
  );
  // 拨弹盘，获取角速度和角度反馈值
  motor_trigger_pid_task->angle_control_task_p->setPIDControllerFeedback(
     motor_trigger_pid_task->motor_backend_p->getCommonMeasurement().output.angle
  );
	motor_trigger_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(
		 motor_trigger_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);
  if(robot.referee_system_task_p->getRobotRefereeStatus_t().game_robot_status.robot_level == 1)
	{
		if(((robot.referee_system_task_p->getRobotRefereeStatus_t().buff.power_rune_buff)&0x02)==0)
		{
			upper_clk = 2000;//5s
		}
		else
		{
			upper_clk = 400;
		}
	}
	else
	{
		upper_clk = 200;
	}

  //发射机构失能
  if(booster_mode == BOOSTER_OFF || robot.getRefereeSystemTaskPointer()->getRobotRefereeStatus_t().game_robot_status.remain_HP == 0)
  {
		trigger_speed = 0;
			exp_tri_angle = motor_trigger_pid_task->motor_backend_p->getCommonMeasurement().output.angle;
		//trigger_angle = motor_trigger_pid_task->motor_backend_p->getCommonMeasurement().output.angle;
		motor_trigger_pid_task->motor_backend_p->setMotorInput(0);
		motor_trigger_pid_task->angular_velocity_control_task_p->clearPID();
		  motor_booster_2_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
      0
    );
    motor_booster_1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
      0 );
		} 
//	//预置键
//	else if(booster_mode == BOOSTER_YUZHI || robot.getRefereeSystemTaskPointer()->getRobotRefereeStatus_t().game_robot_status.remain_HP == 0)
//	{
//		motor_trigger_pid_task->motor_backend_p->setMotorInput(0);
//		motor_trigger_pid_task->angular_velocity_control_task_p->clearPID();
//		  motor_booster_2_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
//      0
//    );
//    motor_booster_1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
//      0 );
//	}
	//单发模式 
  else if(booster_mode == TRIGGER_AUTO && robot.getRefereeSystemTaskPointer()->getRobotRefereeStatus_t().game_robot_status.remain_HP > 0)
  {
		//退弹逻辑
      if(robot.ammo_task_p->block_flag&& block_one_flag==0&&double_press_flag&&robot.referee_system_task_p->getRobotRefereeStatus_t().game_robot_status.mains_power_shooter_output){
				exp_tri_angle-=2*PI *(1.0f/REGIONS_NUM);
				block_one_flag = 1;
				double_press_flag = 0;
      }
		//发弹逻辑
		else if(shoot_flag && one_flag==0&&double_press_flag&&robot.referee_system_task_p->getRobotRefereeStatus_t().game_robot_status.mains_power_shooter_output)
		{
			exp_tri_angle+=2*PI *(1.0f/REGIONS_NUM);
			one_flag = 1;	
			double_press_flag = 0;
			LIMIT_TRI_SPEED=15.0f;
		}
		if(double_press_flag == 0)
		{
			double_press_clk++;
			if(double_press_clk >= upper_clk)
			{
				double_press_flag = 1;
				double_press_clk = 0;
				}
		}
		
		motor_trigger_pid_task->angle_control_task_p->setPIDControllerExpect(exp_tri_angle);
	  trigger_speed = motor_trigger_pid_task->angle_control_task_p->getOutput();
  }
//	if(fabs(motor_trigger_pid_task->angle_control_task_p->geterror())<0.005)
//	{
//		trigger_speed = 0;
//		//motor_trigger_pid_task->angle_control_task_p->clear_one_trigger();
//	}
		if(trigger_speed>LIMIT_TRI_SPEED) trigger_speed = LIMIT_TRI_SPEED; //??拨弹盘的角速度
		motor_trigger_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
		trigger_speed);
	  motor_trigger_pid_task->motor_backend_p->setMotorInput(
    motor_trigger_pid_task->angular_velocity_control_task_p->getOutput()
  );
	//向电机发送期望值
  motor_booster_2_pid_task->motor_backend_p->setMotorInput(
  motor_booster_2_pid_task->angular_velocity_control_task_p->getOutput()
  );
  motor_booster_1_pid_task->motor_backend_p->setMotorInput(
    motor_booster_1_pid_task->angular_velocity_control_task_p->getOutput()
 );
	

	}
	
/***********************************************************************
** 函 数 名： AmmoRM_ControlTask::uninit()
** 函数说明： 任务反初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void AmmoRM_ControlTask::uninit(void) {}
