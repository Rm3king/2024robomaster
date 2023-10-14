#ifdef TEMPLATE

#ifndef __MAIN_CONTROL_TASK_H__
#define __MAIN_CONTROL_TASK_H__

#include "Task.h"
#include "CommonMath.h"
#include "/Remote/RemoteControlTask.h"

// ���ٶ�����
#define ACCEL_LIMIT 0.5f 
#define YAW_ANGULAR_VEL_LIMIT 

namespace Robot_CMD
{
//	enum Command 
//	{
//			EMERGENCY_STOP_CMD,
//			GIMBAL_VEC_CMD,
//			GIMBAL_SHOOT_CMD,
//			CHASSIS_MOVE_CMD,
//			CHASSIS_MODE_CMD
//	};
	struct Emergency_Stop_Command//��ͣ����
	{
		// lockedΪtrueʱ����ͣ������lockedΪfalseʱ����ͣ���
		bool locked;
	};
	struct Gimbal_Vec_Command//��̨��������
	{
		float vec[3];
	};
	struct Gimbal_Rotate_Command//��̨��ת����
	{
		float pitch_vel;
		float yaw_vel;
	};
	struct Auto_Aim_Command//��������
	{
		bool enable;
	};
};

class MainControlTask : public Task_Base
{
public:
  friend class Robot;
  friend class RemoteControlTask;
  MainControlTask(Robot &robot0, timeus_t interval_tick_us0 = 1e6f / 60.0f) : Task_Base(robot0) // 60Hz
  {
    this->interval_tick_us = interval_tick_us0;
  }

  virtual void init(void);
  virtual void update(timeus_t dT_us);
  virtual void uninit(void);

  void setChassisYawTarget(float yaw);
  void setChassisVelTarget(float vel);
  void setChassisYawVelTarget(float yaw_vel);
  
  void setGimbalYawVelTarget(float yaw_vel);
  void setGimbalPitchVelTarget(float pitch_vel);
	
	void setCMD(Robot_CMD::Emergency_Stop_Command msg);
	void setCMD(Robot_CMD::Gimbal_Vec_Command msg);
	void setCMD(Robot_CMD::Gimbal_Rotate_Command msg);
	void setCMD(Robot_CMD::Auto_Aim_Command msg);

protected:

};

#endif

#endif
