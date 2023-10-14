#include "/Control/MainControlTask.h"
#include "/Robot/Robot.h"
#include "UARTDriver.h"

extern Robot robot;

void MainControlTask::init(void)
{
  inited = true;
}

void MainControlTask::update(timeus_t dT_us)
{

}

void MainControlTask::uninit(void)
{

}

void MainControlTask::setChassisVelTarget(float vel)
{

}

void MainControlTask::setChassisYawTarget(float yaw)
{

}

/***********************************************************************
** �� �� ���� MainControlTask::setCMD
** ����˵���� ���ü�ͣ����
**---------------------------------------------------------------------
** ��������� Emergency_Stop_Command msg�ṹ��
** ���ز����� ��
***********************************************************************/
void MainControlTask::setCMD(Robot_CMD::Emergency_Stop_Command msg)
{
	if(msg.locked)
	{
		robot.gimbal_task_p->gimbal_mode = GIMBAL_OFF;
		robot.ammo_task_p->booster_mode = BOOSTER_OFF;
	}
	else
	{
		robot.gimbal_task_p->gimbal_mode = GIMBAL_RC_MODE;
	}
}

/***********************************************************************
** �� �� ���� MainControlTask::setCMD
** ����˵���� ������������
**---------------------------------------------------------------------
** ��������� Auto_Aim_Command msg�ṹ��
** ���ز����� ��
***********************************************************************/
void MainControlTask::setCMD(Robot_CMD::Auto_Aim_Command msg)
{
	if(robot.gimbal_task_p->gimbal_mode != GIMBAL_OFF)
	{
		if(msg.enable)
		{
			robot.gimbal_task_p->gimbal_mode = GIMBAL_AUTO_AIMING_MODE;
		}
		else
		{
			robot.gimbal_task_p->gimbal_mode = GIMBAL_RC_MODE;
		}
	}
}

/***********************************************************************
** �� �� ���� MainControlTask::setCMD
** ����˵���� 
**---------------------------------------------------------------------
** ���������  msg�ṹ��
** ���ز����� ��
***********************************************************************/
void MainControlTask::setCMD(Robot_CMD::Gimbal_Vec_Command msg)
{

}

/***********************************************************************
** �� �� ���� MainControlTask::setCMD
** ����˵���� 
**---------------------------------------------------------------------
** ��������� Gimbal_Rotate_Command msg�ṹ��
** ���ز����� ��
***********************************************************************/
void MainControlTask::setCMD(Robot_CMD::Gimbal_Rotate_Command msg)
{
	robot.gimbal_task_p->gimbal_pitch_vel = msg.pitch_vel;
	robot.gimbal_task_p->gimbal_yaw_vel = msg.yaw_vel;
}
