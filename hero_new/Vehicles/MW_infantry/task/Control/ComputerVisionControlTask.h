#ifdef TEMPLATE
#ifndef COMPUTER_VISION_CONTROL_H
#define COMPUTER_VISION_CONTROL_H

#include "stdint.h"
#include "CommonMath.h"
#include "Task.h"
#include "Vector3.h"
#include "CANSendTask.h"
#include "Referee/RefereeSystemTask.h"

//#include "OffboardLink.h"

//#include "msgs/ChassisVelCommand.h"
//#include "msgs/ChassisVelYawRateCommand.h"
#include "GimbalConstantAimShootCommand.h"
#include "LockTargetSuggest.h"
#include "IMUData.h"
#include "ReferenceDataMini.h"
extern DMA_HandleTypeDef hdma_usart1_rx;
using namespace cmt;

#define CAN_MINIPC_RX_ID 0x500  // 接收
#define CV_RX_BUF_NUM 8u
#define CV_FRAME_LENGTH 4u

#define CV_BUF_NUM 64
#define CV_BUF_LEN 64
#define CV_FIFO_BUF_NUM				8
#define CV_FIFO_BUF_LEN 				64
//#define k1  0.0027
#define g 9.8011 
#define m 0.043
#define ex_number 25
//#define CAN_MINIPC_DWN_QUATERNION_SYNC_ID 0x403 // 四元数 W X 同步
//#define CAN_MINIPC_QUATERNION_WX_SYNC_ID 0x403 // 四元数 W X 同步
//#define CAN_MINIPC_QUATERNION_YZ_SYNC_ID 0x404 // 四元数 Y Z 同步
//#define CAN_MINIPC_ACCEL_XY_SYNC_ID 0x405 // 加速度 X Y 同步
//#define CAN_MINIPC_ACCEL_Z_GYRO_X_SYNC_ID 0x406 // 加速度 Z 角速度 X 同步
//#define CAN_MINIPC_GYRO_YZ_SYNC_ID 0x407 // 角速度 Y Z 同步

class IMUDataSyncTask;
class RefereeDataSyncTask;
class LockDataSyncTask;
typedef struct
{
  float shoot_freq;//射频
  Vector3f pos;//position

} MINIPC_RX_data_t;

union U1
{
    uint8_t s[4];
    float d;
};

class ComputerVisionControlTask : public Task_Base
{
public:
  friend class Robot;
  friend class RemoteControlTask;
  friend class MainControlTask;
	friend class ChassisControlTask;
	friend class GimbalControlTask;
  ComputerVisionControlTask
  (
    Robot &robot0,
    timeus_t interval_tick_us0 = 1e6f / 200.0f
  );
  MINIPC_RX_data_t getMiniPCControlData()
  {
    return minipc_ctrl_data;
  }

  virtual void init(void);
  virtual void update(timeus_t dT_us);
  virtual void uninit(void);
	void parseData(void);
	void processByte(uint8_t data);
	void pushToBuffer(uint16_t start_pos, uint16_t end_pos);
	void Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData,uint32_t Size);
	
	void chassisVelCommand(float vx, float vy, uint8_t frame);
	void chassisVelYawRateCommand(float vx, float vy, float yaw_rate, uint8_t frame);
//	void chassisVelYawCommand(float vx, float vy, float yaw, uint8_t frame);
	void gimbalAimCommand(float x, float y, float z, float freq, uint8_t frame);
//	void gimbalAimSingleCommand(float x, float y, float z, float freq, uint8_t frame);
//	void ammoControlCommand(uint8_t ammo_on, float bullet_spd);
//	void heartbeatCommand(float cpu_usage);
	
	uint8_t rec_buf[CV_BUF_NUM];
  uint8_t rec_fifo_buf[CV_BUF_NUM][CV_BUF_LEN];
	olk::GimbalConstantAimShootCommand get_gimbal_constant_aim_shoot_cmd()
	{
		return gimbal_constant_aim_shoot_cmd;
	}
protected:
	olk::Subscriber subscriber;
//	olk::ChassisVelCommand chassis_vel_cmd;
//	olk::ChassisVelYawRateCommand chassis_vel_yaw_rate_cmd;
	olk::GimbalConstantAimShootCommand gimbal_constant_aim_shoot_cmd;
	uint8_t fifo_count;
  uint8_t fifo_head_pos;
  uint8_t fifo_tail_pos;
  uint8_t referee_seq_num;

  MINIPC_RX_data_t minipc_ctrl_data;
  IMUDataSyncTask *imu_sync_data_task;
	RefereeDataSyncTask *ref_sync_data_task;
LockDataSyncTask *lock_sync_data_task;
	Transmit_DMAFunc transmit_dma_func;
	RobotRefereeStatus_t referee_sta;
	RobotRefereeStatus_t referee_sta_last;
	uint8_t send_sta;
	uint8_t data_buf[60];
	uint8_t rec_buffer[32];
	uint16_t index;
};

// 加速度计、陀螺仪、角速度计数据同步任务
class IMUDataSyncTask : public Task_Base
{
public:
  friend class Robot;

  IMUDataSyncTask
  (
    Robot &robot0,
    timeus_t interval_tick_us0 = 1e6f / 500.0f
  );


  virtual void init(void);
  virtual void update(timeus_t dT_us);
  virtual void uninit(void);

protected:
  uint8_t data_buf[26];
	olk::Publisher publisher;
	olk::IMUData imu_data;

};

class LockDataSyncTask : public Task_Base
{
public:
  friend class Robot;

  LockDataSyncTask
  (
    Robot &robot0,
    timeus_t interval_tick_us0 = 1e6f / 50.0f
  );


  virtual void init(void);
  virtual void update(timeus_t dT_us);
  virtual void uninit(void);

protected:
  uint8_t data_buf[26];
	olk::Publisher publisher;
	olk::LockTargetSuggest lock_target_suggest;

};

class RefereeDataSyncTask : public Task_Base
{
public:
  friend class Robot;

  RefereeDataSyncTask
  (
    Robot &robot0,
    timeus_t interval_tick_us0 = 1e6f / 10.0f
  );


  virtual void init(void);
  virtual void update(timeus_t dT_us);
  virtual void uninit(void);

protected:
	uint8_t send_sta;
  uint8_t data_buf[60];
	olk::Publisher publisher;
//	olk::GameState game_state;
//	olk::RobotsHealth robots_health;
//	olk::RobotStatus robot_status;
//	olk::PowerHeatData power_heat_data;
//	olk::Buff buff;
//	olk::RobotHurt robot_hurt;
//	olk::ShootData shoot_data;
//	olk::BulletRemaining bullet_remaining;
//	olk::RefereeWarning referee_warning;
	olk::ReferenceDataMini reference_data_mini;
//	olk::RobotAroundInformation robot_around_information;

	//RobotRefereeStatus_t referee_sta;
//	RobotRefereeStatus_t referee_sta_last;
};
#endif
#endif