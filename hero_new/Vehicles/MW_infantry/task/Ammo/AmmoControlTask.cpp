/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   AmmoRM_ControlTask.cpp
** �ļ�˵����   ��̨��������
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���Ӻ�     	     2022-07-26
***************************************************************************/
#include "/Ammo/AmmoControlTask.h"
#include "/Robot/Robot.h"
#include "UARTDriver.h"
#include "/Referee/RefereeSystemTask.h"

#define REGIONS_NUM 6.0f//������λ

/***********************************************************************
** �� �� ���� AmmoRM_ControlTask���캯��
** ����˵���� ָ��robot���ã�CAN�豸���ã�ָ��yaw��pitch����CAN����id
**            ��CAN�������ݰ�������ʼ�±ָ꣬���������ʱ������ͬʱ
**            ��yaw��pitch����ʵ����Motor_RM_PIDControlTask����
**---------------------------------------------------------------------
** ��������� robot���á�CAN�豸���á�yaw��pitch����CAN����id��
**            CAN�������ݰ�������ʼ�±ꡢ�������ʱ����
** ���ز����� ��
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
      this->motor_booster_1_pid_task = new Motor_RM_PIDControlTask(robot0,//��ʽʵ����
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
** �� �� ���� AmmoRM_ControlTask::init()
** ����˵���� ע����̨�������Motor_RM_PIDControlTask����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
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
** �� �� ���� AmmoRM_ControlTask::update(timeus_t dT_us)
** ����˵���� ͨ����̨�����������IMU������̨Yaw�������������
**            �Ȳ��ֵĿռ���̬������
��̨��������������񣬲�����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void AmmoRM_ControlTask::update(timeus_t dT_us)
{
  float dT_s = dT_us / 1e6f;
  
  // �ұ�Ħ����
	//���÷���
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

  // ���Ħ����
  motor_booster_1_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(
    motor_booster_1_pid_task->motor_backend_p->getCommonMeasurement().linear.velocity
  );

  motor_booster_1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
    -exp_spd
  );
  // �����̣���ȡ���ٶȺͽǶȷ���ֵ
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

  //�������ʧ��
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
//	//Ԥ�ü�
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
	//����ģʽ 
  else if(booster_mode == TRIGGER_AUTO && robot.getRefereeSystemTaskPointer()->getRobotRefereeStatus_t().game_robot_status.remain_HP > 0)
  {
		//�˵��߼�
      if(robot.ammo_task_p->block_flag&& block_one_flag==0&&double_press_flag&&robot.referee_system_task_p->getRobotRefereeStatus_t().game_robot_status.mains_power_shooter_output){
				exp_tri_angle-=2*PI *(1.0f/REGIONS_NUM);
				block_one_flag = 1;
				double_press_flag = 0;
      }
		//�����߼�
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
		if(trigger_speed>LIMIT_TRI_SPEED) trigger_speed = LIMIT_TRI_SPEED; //??�����̵Ľ��ٶ�
		motor_trigger_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
		trigger_speed);
	  motor_trigger_pid_task->motor_backend_p->setMotorInput(
    motor_trigger_pid_task->angular_velocity_control_task_p->getOutput()
  );
	//������������ֵ
  motor_booster_2_pid_task->motor_backend_p->setMotorInput(
  motor_booster_2_pid_task->angular_velocity_control_task_p->getOutput()
  );
  motor_booster_1_pid_task->motor_backend_p->setMotorInput(
    motor_booster_1_pid_task->angular_velocity_control_task_p->getOutput()
 );
	

	}
	
/***********************************************************************
** �� �� ���� AmmoRM_ControlTask::uninit()
** ����˵���� ���񷴳�ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void AmmoRM_ControlTask::uninit(void) {}
