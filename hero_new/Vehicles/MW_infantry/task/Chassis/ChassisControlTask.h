#ifdef TEMPLATE

#ifndef CHASSIS_CONTROL_TASK_H
#define CHASSIS_CONTROL_TASK_H

#include "Motor.h"
#include "Motor_RM_Tasks.h"
#include "Filters.h"
#include "/Robot/Params.h"

//#define CAN_CHASSIS_MOTOR_SUPERCAP_CTRL_ID      0x300
//#define CAN_CHASSIS_CTRL_ID                     0x200
//#define CAN_3508_MOTOR1_ID                      0x201
//#define CAN_3508_MOTOR2_ID                      0x202
//#define CAN_3508_MOTOR3_ID                      0x203
//#define CAN_3508_MOTOR4_ID                      0x204

#define INITIAL_HALF_WHEEL_BASE .42f// �����[m]
#define INITIAL_HALF_TREAD .42// ���־�[m]
#define INITIAL_ROTATE_RATIO_X .0f// ��ת������x�����ϵĹ�һ������
#define INITIAL_ROTATE_RATIO_Y .0f// ��ת������y�����ϵĹ�һ������
#define INITIAL_WHEEL_VEL_RATIO 1.0f// ��������
#define INITIAL_XY_Chassis_Slow 1.0f// //ƽ����������

struct suercap_RX_data_pack
{
	float supercap_voltage;
	float supercap_energy_percent;
};

typedef enum
{
  Chassis_OFF = 0, // ����ʧ��
  Chassis_FOLLOW = 1, // ���̸�����̨
  Chassis_TOP = 2, // С����
  Chassis_FREE = 3,//������̨����
	Chassis_45to_enemy=4,//45�ȶԵа�E
} Chassis_Control_Mode;
//�����ٶȾ���ֵ���
template<typename Type> 
Type _Chassis_Abs(Type x) {return ((x > 0) ? x : -x);}
//��ͣ ���ٺ���
template<typename Type> 
Type _Chassis_Minish(Type x,Type y)
{
    return ((x>0)?(x-y):(x+y));
}
class ChassisControlTask : public Task_Base
{
public:
  friend class Robot;
  friend class MainControlTask;
  friend class RemoteControlTask;

  ChassisControlTask(
    Robot &robot0,
    Motor_RM_Params_t *m1, PID_Params_t *m1_ang, PID_Params_t *m1_ang_vel,
    Motor_RM_Params_t *m2, PID_Params_t *m2_ang, PID_Params_t *m2_ang_vel,
    Motor_RM_Params_t *m3, PID_Params_t *m3_ang, PID_Params_t *m3_ang_vel,
    Motor_RM_Params_t *m4, PID_Params_t *m4_ang, PID_Params_t *m4_ang_vel,
		PID_Params_t *chassis_follow_ang,CAN_Rx_Data_Pack_t &_CAN_SUPERCAP_RX,
		CAN_Tx_Data_Link_t &_CAN_SUPERCAP_TX,
    timeus_t interval_tick_us0 = 0
  ) ;

  virtual ~ChassisControlTask(void) {};

  virtual void init(void);

  virtual void update(timeus_t dT_us);

  virtual void uninit(void);

	void setControlMode(Chassis_Control_Mode mode)
  {
    chassis_mode = mode;
  }
  
	Chassis_Control_Mode getControlMode()
  {
    return chassis_mode;
  }
	
	float getexp_vel(void)
	{
		return exp_vel[2];
	}
	
	Motor_RM_PIDControlTask *getMotor_RM_PIDControlTaskPointer(void)
	{
		return motor_3_pid_task;
	}	
	
	uint8_t *get_can_tx_data()
	{
		return can_tx_data;
	}
	
	LowPassFilter<float> get_lowpass_filter()
	{
		return lowpass_filter;
	}
//	PIDControlTask getChassis_follow_PIDTask()
//	{
//		return chassis_follow_pid_task;
//	}	
	suercap_RX_data_pack get_suercap_RX_data_pack()
	{
		return suercap_RX_data;
	}
	
	void set_Chassis_OFF_flag()
	{
		Chassis_OFF_flag = 1;
	}
	uint8_t get_Chassis_OFF_flag()
	{
		return Chassis_OFF_flag;
	}
	void clear_Chassis_OFF_flag()
	{
		Chassis_OFF_flag = 0; 
	}
	
	void set_Chassis_FOLLOW_flag()
	{
		Chassis_FOLLOW_flag = 1;
	}
		void set_Chassis_45toenemy_flag()
	{
		Chassis_45toenemy_flag = 1;
	}
		void clear_Chassis_45toenemy_flag()
	{
		Chassis_45toenemy_flag = 0; 
	}
	uint8_t get_Chassis_45toenemy_flag()
	{ 
		return Chassis_45toenemy_flag;
	}
	uint8_t get_Chassis_FOLLOW_flag()
	{ 
		return Chassis_FOLLOW_flag;
	}
	void clear_Chassis_FOLLOW_flag()
	{
		Chassis_FOLLOW_flag = 0; 
	}
	
	void set_Chassis_TOP_flag()
	{
		Chassis_TOP_flag = 1;
	}
	uint8_t get_Chassis_TOP_flag()
	{
		return Chassis_TOP_flag;
	}
	void clear_Chassis_TOP_flag()
	{
		Chassis_TOP_flag = 0; 
	}
	void set_Speed_flag()
    {
        speed_add_flag = 1;
    }
	uint8_t get_Speed_flag()
    {
        return speed_add_flag;
    }
    void clear_Speed_flag()
    {
        speed_add_flag = 0;
    }
    float get_max_speed(){
        return max_vx;
    }
	 float get_supercap_voltage_lf()
	 {
		 return supercap_voltage_lpf.getOutput();
	 }
	  float get_supercap_persent_lf()
	 {
		 return supercap_percent_lpf.getOutput();
	 }
	 
protected:
  // uint8_t can_tx_data[8];
  Motor_RM_PIDControlTask *motor_1_pid_task, *motor_2_pid_task, *motor_3_pid_task, *motor_4_pid_task;
	PIDControlTask  *chassis_follow_pid_task;
	Chassis_Control_Mode chassis_mode;
	
	uint8_t Chassis_OFF_flag;
	uint8_t Chassis_FOLLOW_flag;
	uint8_t Chassis_TOP_flag;
	uint8_t Chassis_45toenemy_flag;
	LowPassFilter<float> lowpass_filter;
	LowPassFilter<float> supercap_voltage_lpf, supercap_percent_lpf;
	CAN_Rx_Data_Pack_t CAN_SUPERCAP_RX_PACK;
	
	suercap_RX_data_pack suercap_RX_data;
	//KMJ
   uint8_t level_lock = 0;
	
	CAN_Tx_Data_Link_t CAN_SUPERCAP_TX_PACK;
	uint8_t can_tx_data[8];
   uint8_t speed_add_flag=0;
  float vx; // ��̨x�����ٶ�[m/s]
  float vy; // ��̨y�����ٶ�[m/s]
  float vw; // ��z����ٶ�[rad/s]
	float vx_resolve;//�����ĵ���x�����ٶ�
	float vy_resolve;//�����ĵ���y�����ٶ�
	float max_vx = 1.8f;
	float min_vx = -1.8f;
	float max_vy = 1.8f;
	float min_vy = -1.8f;
	
	uint8_t last_level = 1;
	
    float max_vw = 0.8f;
    float min_vw = -0.8f;
	float delta_vx = 0.005f;
	float delta_vy = 0.005f;
    float delta_vw = 0.01f;
  float vx_filter;//�˲���ĵ���x�����ٶ�[m/s]
	float vy_filter;//�˲���ĵ���x�����ٶ�[m/s]
	float chassis_angle;
	float super_chassis_slow = 1.0f;
	float super_vw_slow = 1.0f;
  float half_wheel_base; // �����[m]
  float half_tread; // ���־�[m]

  float rotate_ratio_x; // ��ת������x�����ϵĹ�һ������[-1.0, 1.0]��������Ϊ1.0��-1.0
  float rotate_ratio_y; // ��ת������y�����ϵĹ�һ������[-1.0, 1.0]��������Ϊ1.0��-1.0

  float wheel_vel_ratio; // ��ת��������
	float xy_chassis_slow;//ƽ����������
  float exp_vel[4] = {0}; // �ĸ�����������ٶ�[m/s]
	float rotate_ratio[4];
  int16_t motor_send[4] = {0}; // ���շ�������ĵ���ֵ
  /*     forward
            X              1.0
           /|\
       M2   |   M1
    Y<------|------        0.0
       M3   |   M4
            |
            |             -1.0
  1.0      0.0       -1.0
  */
};
#endif

#endif