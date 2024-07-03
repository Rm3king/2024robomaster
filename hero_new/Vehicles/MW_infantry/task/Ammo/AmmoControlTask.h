#ifdef TEMPLATE

#ifndef AMMO_CONTROL_TASK_H
#define AMMO_CONTROL_TASK_H

#include "Motor.h"
#include "Motor_RM_Tasks.h"
#include "Matrix3.h"

typedef enum
{
  BOOSTER_OFF = 0, //关闭摩擦轮
  BOOSTER_ON = 1, //开启摩擦轮
  TRIGGER_AUTO = 2,//自动拨弹
	BOOSTER_YUZHI = 3//预置
} Ammo_Control_Mode;



class AmmoRM_ControlTask : public Task_Base
{
public:
  friend class Robot;
  friend class RemoteControlTask;
  friend class MainControlTask;

  AmmoRM_ControlTask(
    Robot &robot0,
    Motor_RM_Params_t *booster_1, PID_Params_t *booster_1_ang, PID_Params_t *booster_1_ang_vel,
    Motor_RM_Params_t *booster_2, PID_Params_t *booster_2_ang, PID_Params_t *booster_2_ang_vel,
    Motor_RM_Params_t *trigger, PID_Params_t *trigger_ang, PID_Params_t *trigger_ang_vel,
    timeus_t interval_tick_us0 = 0
  );

  virtual ~AmmoRM_ControlTask(void) {}

  virtual void init(void);

  virtual void update(timeus_t dT_us);

  virtual void uninit(void);

  Motor_RM_PIDControlTask* getBoosterLeftPIDControlTaskPointer(void)
  {
    return motor_booster_2_pid_task;
  }
  Motor_RM_PIDControlTask* getBoosterRightPIDControlTaskPointer(void)
  {
    return motor_booster_1_pid_task;
  }
  Motor_RM_PIDControlTask* getTriggerPIDControlTaskPointer(void)
  {
    return motor_trigger_pid_task;
  }
	float getShoot_Rreq()
	{
		return shoot_freq;
	}
	float getShoot_Flag()
	{
		return shoot_flag;
	}
	float getExp_Spd()
	{
		return exp_spd;
	}
	float getTrigger_angle()
	{
		return trigger_angle;
	}
	Ammo_Control_Mode get_booster_mode()
	{
			return booster_mode;
	}
	
	void set_Booster_off_flag()
	{
		booster_off_flag = 1;
	}
	uint8_t get_Booster_off_flag()
	{
		return booster_off_flag;
	}
	void clear_Booster_off_flag()
	{
		booster_off_flag = 0;
	}
	
	void set_Booster_on_flag()
	{
		booster_on_flag = 1;
	}
	void clear_Booster_on_flag()
	{
		booster_on_flag = 0;
	}
	uint8_t get_Booster_on_flag()
	{
		return booster_on_flag;
	}	
	void set_exp_spd_flag()
	{
		exp_spd_flag =1;
	}		
	void clear_exp_spd_flag()
	{
		exp_spd_flag = 0;
	}
protected:

  Motor_RM_PIDControlTask *motor_booster_2_pid_task, *motor_booster_1_pid_task,  *motor_trigger_pid_task;
  Ammo_Control_Mode booster_mode;
	uint8_t booster_off_flag;
	uint8_t booster_on_flag;
  float exp_spd;
  float shoot_freq;
	int8_t shoot_flag;
   int8_t shoot_b_flag;
	int8_t one_flag;
	int8_t block_one_flag;
	int8_t block_flag;
  int8_t moving_flag;
  int16_t double_press_clk;
  int8_t double_press_flag = 1;
  uint8_t elec_enable;
	float trigger_angle;
	float trigger_speed;
	float last_motor_angle;
  uint16_t upper_clk;
	float shoot_delta_trigger_angle = 1.0f;
	float block_delta_trigger_angle = 0.02;
	float shoot_speed_10 = 13.06f,shoot_speed_16 = 18.5;
	uint8_t exp_spd_flag;
	float LIMIT_TRI_SPEED;
//初始化中间期望变量为当前拨弹盘位置
	float exp_tri_angle;


};



#endif

#endif