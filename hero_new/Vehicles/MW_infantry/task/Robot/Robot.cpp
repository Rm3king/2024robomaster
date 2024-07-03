/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   Robot.cpp
** �ļ�˵����   �����˻���
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���לB     	   2022-07-10
***************************************************************************/
#include "/Robot/Robot.h"
#include "arm_math.h"
#include "Motor_RM_Tasks.h"
#include "Params.h"
#include "tim.h"
#include "UARTDriver.h"
/***********************************************************************
** �� �� ���� Robot::init()
** ����˵���� ������ϵͳ��ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Robot::init()
{

  // ��ʼ���������
  params.initMotorsParams();
 //��������
  attitude_solution_task_p = new AttitudeSolutionTask(*this); // ��̬����, ���ⲿ�ж��н���
  led_control_task_p = new LEDControlTask(*this); // LED����
  referee_system_task_p = new RefereeSystemTask(*this); // ����ϵͳ
  main_control_task_p = new MainControlTask(*this); // ������
  remote_control_task_p = new RemoteControlTask(*this);//ң��
	
	cv_control_task_p = new ComputerVisionControlTask(*this);// �Ӿ�
  bmi088_id = inertial_sensors.addBackend(new InertialSensor_BMI088(inertial_sensors, ROTATION_YAW_270)); // BMI088 �Ʊ���Z����ת180��

	
  rc_protocol.init();
  scheduler.init();
  can1_device.init(&hcan1);
  can2_device.init(&hcan2);

  ammo_task_p = new AmmoRM_ControlTask(
    *this,

    &params.motor_params.ammo_booster_motor_1,
    NULL,
    &params.motor_params.ammo_booster_motor_1_ang_vel,

    &params.motor_params.ammo_booster_motor_2,
    NULL,
    &params.motor_params.ammo_booster_motor_2_ang_vel,

    &params.motor_params.trigger_motor,
    &params.motor_params.trigger_motor_ang,
    &params.motor_params.trigger_motor_ang_vel
  );

  chassis_task_p = new ChassisControlTask(
    *this,

    &params.motor_params.chassis_motor_1,
    NULL,
    &params.motor_params.chassis_motor_1_ang_vel,

    &params.motor_params.chassis_motor_2,
    NULL,
    &params.motor_params.chassis_motor_2_ang_vel,

    &params.motor_params.chassis_motor_3,
    NULL,
    &params.motor_params.chassis_motor_3_ang_vel,

    &params.motor_params.chassis_motor_4,
    NULL,
    &params.motor_params.chassis_motor_4_ang_vel,
		
		&params.motor_params.chassis_follow_ang,
		params.motor_params.CAN_SUPERCAP_RX,
		params.motor_params.CAN_SUPERCAP_TX
  );

  gimbal_task_p = new GimbalControlTask(
    *this,

    &params.motor_params.gimbal_yaw,
    &params.motor_params.gimbal_yaw_ang,
    &params.motor_params.gimbal_yaw_ang_vel,

    &params.motor_params.gimbal_pitch,
    &params.motor_params.gimbal_pitch_ang,
    &params.motor_params.gimbal_pitch_ang_vel
  );

		/* can��������ʵ���� */
  can1_send_task_0x1ff_p = new CANSendTask(
    *this,
    &this->can1_device,
    0x1ff,
    params.motor_params.control_tasks_interval.can1_send_0x1ff_task_interval
  );
		
  can2_send_task_0x1ff_p = new CANSendTask(
    *this,
    &this->can2_device,
    0x1ff,
    params.motor_params.control_tasks_interval.can2_send_0x1ff_task_interval
  );
		
  can1_send_task_0x200_p = new CANSendTask(
    *this,
    &this->can1_device,
    0x200,
    params.motor_params.control_tasks_interval.can1_send_0x200_task_interval
  );
		
  can2_send_task_0x200_p = new CANSendTask(
    *this,
    &this->can2_device,
    0x200,
    params.motor_params.control_tasks_interval.can2_send_0x200_task_interval
  );
		
	can2_send_task_0x401_p = new CANSendTask(
    *this,
    &this->can2_device,
    0x401,
    params.motor_params.control_tasks_interval.can2_send_0x401_task_interval
  );
		/* can��������ʵ���� */

		/* �������ڳ�ʼ�� */
  ammo_task_p->setInterval(
    params.motor_params.control_tasks_interval.ammo_task_interval
  );
		
  chassis_task_p->setInterval(
    params.motor_params.control_tasks_interval.chassis_task_interval
  );
		
  gimbal_task_p->setInterval(
    params.motor_params.control_tasks_interval.gimbal_task_interval
  );
		
  led_control_task_p->setInterval(
    params.motor_params.control_tasks_interval.led_task_interval
  );
	
  referee_system_task_p->setInterval(
    params.motor_params.control_tasks_interval.referee_system_task_interval
  );
	 /* �������ڳ�ʼ�� */
		
	/* �����ʼ�� */
	//ע������
  ammo_task_p->init();
  chassis_task_p->init();
  gimbal_task_p->init();
	cv_control_task_p->init();
	//ע������
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);  
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1); 
  USART1_DMA_Tx_Init();
	
  led_control_task_p->init();

  attitude_solution_task_p->inited = true;
  attitude_solution_task_p->init();
	/* �����ʼ�� */
	
	/* ע������ */
  scheduler.registerTask(referee_system_task_p);
  scheduler.registerTask(main_control_task_p);
  scheduler.registerTask(cv_control_task_p);
	scheduler.registerTask(ammo_task_p);
  scheduler.registerTask(chassis_task_p);
  scheduler.registerTask(gimbal_task_p);
  scheduler.registerTask(can1_send_task_0x1ff_p);
  scheduler.registerTask(can1_send_task_0x200_p);
  scheduler.registerTask(can2_send_task_0x1ff_p);
  scheduler.registerTask(can2_send_task_0x200_p);
	scheduler.registerTask(can2_send_task_0x401_p);
	/* ע������ */
	
}

/***********************************************************************
** �� �� ���� Robot::run()
** ����˵���� ������ϵͳ���к������˺�����������ѭ����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Robot::run()
{
  scheduler.run();
}

void Robot::setParams()
{

}