#ifdef TEMPLATE

#ifndef GIMBAL_CONTROL_TASK_H
#define GIMBAL_CONTROL_TASK_H

#include "Motor.h"
#include "Motor_RM_Tasks.h"
#include "Matrix3.h"
#include "Filters.h"
//#define k1  0.0027
#define g 9.8011 
#define m 0.043
#define k3 0.0027
#define threshold 0.00001
#define ex_number 25
typedef enum
{
  GIMBAL_OFF = 0, // 下电模式
  GIMBAL_RC_MODE = 1, // 遥控器模式
  GIMBAL_AUTO_AIMING_MODE = 2, //自瞄模式
    GIMABAL_HANGING_MODE = 3,   //吊射模式
	GIMBAL_HANGING_LOCKALL_MODE=4,//锁死yaw和pitch
} Gimbal_Control_Mode;

class GimbalControlTask : public Task_Base
{
public:
  friend class Robot;
  friend class RemoteControlTask;
  friend class MainControlTask;
  friend class RefereeSystemTask;

  GimbalControlTask(
    Robot &robot0,
    Motor_RM_Params_t *yaw, PID_Params_t *yaw_ang, PID_Params_t *yaw_ang_vel,
    Motor_RM_Params_t *pitch, PID_Params_t *pitch_ang, PID_Params_t *pitch_ang_vel,
    timeus_t interval_tick_us0 = 0
  );

  virtual ~GimbalControlTask(void) {}

  virtual void init(void);

  virtual void update(timeus_t dT_us);

  virtual void uninit(void);

  void gimbal_soft_limit();


  Motor_RM_PIDControlTask* getYawPIDControlTaskPointer(void)
  {
    return motor_yaw_pid_task;
  }
  Motor_RM_PIDControlTask* getPitchPIDControlTaskPointer(void)
  {
    return motor_pitch_pid_task;
  }

  void setExpectVec(Vector3f vec)
  {
    expect_vec = vec;
  }

  Vector3f getExpectVec(void)
  {
    return expect_vec;
  }

  Vector3f getYawAxis(void)
  {
    return coord_chassis.k;
  }

  Vector3f getPitchAxis(void)
  {
    return coord_imu.j;
  }

  void setControlMode(Gimbal_Control_Mode mode)
  {
    gimbal_mode = mode;
  }

  Gimbal_Control_Mode getControlMode()
  {
    return gimbal_mode;
  }

	float getdelta_yaw()
	{
		return delta_yaw;
	}
	
	float getgimbal_yaw_vel()
	{
		return gimbal_yaw_vel;
	}
	
	void set_GIMBAL_AUTO_AIMING_MODE_flag()
	{
		GIMBAL_AUTO_AIMING_MODE_flag = 1;
	}
	uint8_t get_GIMBAL_AUTO_AIMING_MODE_flag()
	{
		return GIMBAL_AUTO_AIMING_MODE_flag;
	}
	void clear_GIMBAL_AUTO_AIMING_MODE_flag()
	{
		GIMBAL_AUTO_AIMING_MODE_flag = 0; 
	}
	
	void set_GIMBAL_RC_MODE_flag()
	{
		GIMBAL_RC_MODE_flag = 1;
	}
	uint8_t get_GIMBAL_RC_MODE_flag()
	{
		return GIMBAL_RC_MODE_flag;
	}
	void clear_GIMBAL_RC_MODE_flag()
	{
		GIMBAL_RC_MODE_flag = 0; 
	}
		float get_goal_distance()
	{
		return distance;
	}

protected:

  Motor_RM_PIDControlTask *motor_yaw_pid_task, *motor_pitch_pid_task;
  Matrix3f coord_yaw, coord_chassis;
  Matrix3f coord_imu;  //imu坐标系
  Vector3f expect_vec; // 云台炮管固连的机构期望指向
  Vector3f exp_cross, imu_cross; // 期望向量×转轴， IMU的x轴×转轴
  Matrix3f gyro_vec;

	uint8_t GIMBAL_AUTO_AIMING_MODE_flag;
	uint8_t GIMBAL_RC_MODE_flag;

  float delta_yaw, delta_pitch;

  Gimbal_Control_Mode gimbal_mode;

  // 云台（遥控器控制转速）
  float gimbal_yaw_vel;
  float gimbal_pitch_vel;

  float gimbal_limit_ang_max = (22.f / 180 * PI + PI / 2.f);
  float gimbal_limit_ang_min = (-30.f / 180 * PI + PI / 2.f);

  float gimbal_limit_ang_max_cos;
  float gimbal_limit_ang_min_cos;
  float gimbal_ammo_auto_z;
  LowPassFilter<float> lp_pitch_out;
	float k1 =  0.0027;
	float k2 = 1.1;
	float distance= 0.0f;
	float pitch_vision;
};
#endif

#endif