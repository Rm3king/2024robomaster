#ifdef TEMPLATE

#ifndef __REMOTE_CONTROLTASK_H__
#define __REMOTE_CONTROLTASK_H__

#include "Task.h"

class Robot;

class RemoteControlTask : public Task_Base
{
public:
  RemoteControlTask(Robot &robot0, timeus_t interval_tick_us0 = 1e6f / 60.0f) : Task_Base(robot0) // 400 Hz
  {
    this->interval_tick_us = interval_tick_us0;
  }
  float last_vx;
	float last_vy;
	uint8_t open_flag;
	uint8_t close_flag;
	uint8_t press_r_flag;
	uint8_t press_E_flag;
	uint8_t max_yaw_vel = 7;
	uint8_t double_press_flag = 0;
	uint16_t double_press_clk = 0;
  uint8_t shoot_heat_flag;
  virtual void init(void);
  virtual void update(timeus_t dT_us);
  virtual void uninit(void);
  
  uint8_t autoaim_flag = 1;//�����־λ�����ڵ�������������ȼ� 1:�ڱ� 2:���� 3��Ӣ��
  uint8_t autoaim_lock = 0;//�����־λ 
  uint16_t time_cnt;
};

#endif

#endif