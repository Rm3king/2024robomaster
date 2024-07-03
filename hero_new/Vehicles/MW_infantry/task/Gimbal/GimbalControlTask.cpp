/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   robo_template
** �� �� ����   GimbalControlTask.cpp
** �ļ�˵����   ��̨��������
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						     ���Ӻ�     	     2022-07-26
***************************************************************************/
#include "/Gimbal/GimbalControlTask.h"
#include "/Robot/Robot.h"
#include "UARTDriver.h"

/***********************************************************************
** �� �� ���� GimbalControlTask���캯��
** ����˵���� ָ��robot���ã�CAN�豸���ã�ָ��yaw��pitch����CAN����id
**            ��CAN�������ݰ�������ʼ�±ָ꣬���������ʱ������ͬʱ
**            ��yaw��pitch����ע��Motor_RM_PIDControlTask����
**---------------------------------------------------------------------
** ��������� robot���á�CAN�豸���á�yaw��pitch����CAN����id��
**            CAN�������ݰ�������ʼ�±ꡢ�������ʱ����
** ���ز����� ��
***********************************************************************/
GimbalControlTask::GimbalControlTask(
  Robot &robot0,
  Motor_RM_Params_t *yaw, PID_Params_t *yaw_ang, PID_Params_t *yaw_ang_vel,
  Motor_RM_Params_t *pitch, PID_Params_t *pitch_ang, PID_Params_t *pitch_ang_vel,
  timeus_t interval_tick_us0
) : Task_Base(robot0)
{
  this->interval_tick_us = interval_tick_us0;

  if(pitch != NULL)
  {
    if(pitch->canx == 1)
    {
      this->motor_pitch_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          pitch->can_rx_id,
          pitch->can_tx_id,
          pitch->can_tx_data_start_pos,
          RoboMaster_GM6020,
          pitch->interval);
    }
    else if(pitch->canx == 2)
    {
      this->motor_pitch_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          pitch->can_rx_id,
          pitch->can_tx_id,
          pitch->can_tx_data_start_pos,
          RoboMaster_GM6020,
          pitch->interval);
    }
    this->motor_pitch_pid_task->motor_backend_p->setParams(*pitch);
  }

  if(yaw != NULL)
  {
    if(yaw->canx == 1)
    {
      this->motor_yaw_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          yaw->can_rx_id,
          yaw->can_tx_id,
          yaw->can_tx_data_start_pos,
          RoboMaster_GM6020,
          yaw->interval);
    }
    else if(yaw->canx == 2)
    {
      this->motor_yaw_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          yaw->can_rx_id,
          yaw->can_tx_id,
          yaw->can_tx_data_start_pos,
          RoboMaster_GM6020,
          yaw->interval);
    }
    this->motor_yaw_pid_task->motor_backend_p->setParams(*yaw);
  }


  if(yaw_ang_vel != NULL)
  {
    this->motor_yaw_pid_task->getAngularVelocityTaskPointer()->setPIDControllerParams(*yaw_ang_vel);
    this->motor_yaw_pid_task->getAngularVelocityTaskPointer()->setInterval((*yaw_ang_vel).interval);
  }

  if(yaw_ang != NULL)
  {
    this->motor_yaw_pid_task->getAngleTaskPointer()->setPIDControllerParams(*yaw_ang);
    this->motor_yaw_pid_task->getAngleTaskPointer()->setInterval((*yaw_ang).interval);
  }


  if(pitch_ang_vel != NULL)
  {
    this->motor_pitch_pid_task->getAngularVelocityTaskPointer()->setPIDControllerParams(*pitch_ang_vel);
    this->motor_pitch_pid_task->getAngularVelocityTaskPointer()->setInterval((*pitch_ang_vel).interval);
  }

  if(pitch_ang != NULL)
  {
    this->motor_pitch_pid_task->getAngleTaskPointer()->setPIDControllerParams(*pitch_ang);
    this->motor_pitch_pid_task->getAngleTaskPointer()->setInterval((*pitch_ang).interval);
  }
}

float shift_Topi(float T)
{
	return  (T * PI) / 180.0;
}

float f(float theta,float D,float Z,float v1)
{
	return (m * g * D) / (k3 * v1 * cos(shift_Topi(theta))) + D * tan(shift_Topi(theta)) + ((m * m * g) / (k3 * k3)) * log(1 - (k3 * D) / (m * v1 * cos(shift_Topi(theta)))) - Z;
}

float root_between(float a, float b,float dx,float dy,float dz,float v2)
{
	float fa, fb, x, fx,tem;
	fa = f(a,sqrt(dx*dx+dy*dy),dz,v2);
	fb = f(b, sqrt(dx * dx + dy * dy), dz,v2);
	if (fabs(fa) < threshold) return a;//fabs()Ϊ����ֵ��������ֹȡ������ҲС��D,z=ֻҪС��D��������0��Ϊ��������1e-8��
	if (fabs(fb) < threshold) return b;
	while (fabs(b - a) > threshold)//ֻҪ������벻���㹻С�Ļ��ͼ���ѭ���� 
	{
		x = (a + b) / 2;
		fx = f(x, sqrt(dx * dx + dy * dy), dz,v2);
		if (fabs(fx) < threshold) return x;
		if (fx * fa < 0) { b = x; fb = fx; }
		else { a = x; fa = fx; }//���x��a����ֵ��ţ���ô�ڴ������ڣ�x�����µ�b������x��Ȼ��b��ţ�x���µ�a��
	}
			
	return  (a+b)/2;
}

float update_z(float xx_distance,float y_distance,float z_distance,float v0,float k1)
{
	int i=0;
	float ey=z_distance;
	float dx=sqrt(xx_distance*xx_distance+y_distance*y_distance);
	float dy;
	float a;
	float b;
	float upangle = 45.0;
	float lowangle = -25.0;
	b = 10.0;
	a = b / 57.3;
	dy = (m*g*dx) / (k1*v0*cos(a)) + dx * tan(a) + ((m*m*g) / (k1*k1))*log(1 - (k1*dx) / (m*v0*cos(a)));
	while (fabs(dy - ey) > 0.0001)
	{
		i++;
		if (dy < ey) //����ֵƫС
		{
			lowangle = b;
			b = (upangle + lowangle) / 2;
			a = b / 57.3;
			dy = (m*g*dx) / (k1*v0*cos(a)) + dx * tan(a) + ((m*m*g) / (k1*k1))*log(1 - (k1*dx) / (m*v0*cos(a)));//��Խ��ֵԽ��
		}

		else //����ֵƫ��
		{
			upangle = b;
			b = (upangle + lowangle) / 2;
			a = b / 57.3;
			dy = (m*g*dx) / (k1*v0*cos(a)) + dx * tan(a) + ((m*m*g) / (k1*k1))*log(1 - (k1*dx) / (m*v0*cos(a)));//��Խ��ֵԽ��
		}
		if(i>ex_number) return z_distance;
	}
	return tan(b/180*3.1415926)*dx;
}
/***********************************************************************
** �� �� ���� GimbalControlTask::init()
** ����˵���� ע����̨�������Motor_RM_PIDControlTask����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void GimbalControlTask::init(void)
{
  inited = true;
  if(motor_yaw_pid_task != NULL)
    robot.scheduler.registerTask(motor_yaw_pid_task);
  if(motor_pitch_pid_task != NULL)
    robot.scheduler.registerTask(motor_pitch_pid_task);

  if(motor_yaw_pid_task->getAngleTaskPointer() != NULL)
    robot.scheduler.registerTask(motor_yaw_pid_task->getAngleTaskPointer() );
  if(motor_yaw_pid_task->getAngularVelocityTaskPointer() != NULL)
    robot.scheduler.registerTask(motor_yaw_pid_task->getAngularVelocityTaskPointer() );

  if(motor_pitch_pid_task->getAngleTaskPointer() != NULL)
    robot.scheduler.registerTask(motor_pitch_pid_task->getAngleTaskPointer() );
  if(motor_pitch_pid_task->getAngularVelocityTaskPointer() != NULL)
    robot.scheduler.registerTask(motor_pitch_pid_task->getAngularVelocityTaskPointer() );

  expect_vec.set(1, 0, 0);//���ó�ʼ��������Ϊ(1��0��0)

  gimbal_limit_ang_max_cos = arm_cos_f32(gimbal_limit_ang_max);
  gimbal_limit_ang_min_cos = arm_cos_f32(gimbal_limit_ang_min);
  
	gimbal_mode = GIMBAL_RC_MODE;
  lp_pitch_out.setCutOffFreq(20);
//  lp_pitch_out.setSamplingCycle(this->interval_tick_us / 1e6f);
}

/***********************************************************************
** �� �� ���� GimbalControlTask::update(timeus_t dT_us)
** ����˵���� ͨ����̨�����������IMU������̨Yaw�������������
**            �Ȳ��ֵĿռ���̬��������̨��������������񣬲�����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void GimbalControlTask::update(timeus_t dT_us)
{
  float dT_s = dT_us / 1e6f;

  // ͨ��IMU�͵������������Yaw���������ϵ�͵�������ϵ

  // ��ȡIMUϵ
  coord_imu = robot.attitude_solution_task_p->getIMUCoord();

  // ͨ��Pitch���������ת���Yaw���������ϵ
  coord_yaw = coord_imu.rotateAboutAxis(coord_imu.j, - motor_pitch_pid_task->motor_backend_p->getCommonMeasurement().rotator.polar_angle);

  // ͨ��Yaw���������ת��õ�������ϵ
  coord_chassis = coord_yaw.rotateAboutAxis(coord_yaw.k, - motor_yaw_pid_task->motor_backend_p->getCommonMeasurement().rotator.polar_angle);


	distance = sqrt(robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.x*robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.x+robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.y*robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.y);

   if(gimbal_mode == GIMBAL_RC_MODE)//ң����ģʽ
  {

    expect_vec = expect_vec.rotateAboutAxis(coord_chassis.k, gimbal_yaw_vel * dT_s);
    expect_vec = expect_vec.rotateAboutAxis(coord_imu.j, gimbal_pitch_vel * dT_s);
		
  }
	else if(gimbal_mode == GIMABAL_HANGING_MODE)
	{
		expect_vec = expect_vec.rotateAboutAxis(coord_imu.j, 0.2*gimbal_pitch_vel * dT_s);
	}
	else if(gimbal_mode==GIMBAL_HANGING_LOCKALL_MODE){}
  else if(gimbal_mode == GIMBAL_AUTO_AIMING_MODE)
  {
    // ִֻ��80ms�ڷ���������
    if(micros() - robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.getTimestamp() < 50000)
    {
//			gimbal_ammo_auto_z =sqrt(robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.x*robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.x+
//			robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.y*robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.y)*
//			tan(shift_Topi(root_between(-30.0,45.0,robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.x,robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.y,robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.z,robot.referee_system_task_p->get_filter_speed())));
//�ֶ�����
//		pitch_vision +=1.233333*gimbal_pitch_vel*dT_s;
//		if(pitch_vision>2.5)   pitch_vision = 2.5;
//		else if(pitch_vision<-2.5) pitch_vision = -2.5;
//			gimbal_ammo_auto_z =robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.z-pitch_vision;
	//		gimbal_ammo_auto_z=update_z(robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.x,robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.y,robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.z,robot.referee_system_task_p->get_filter_speed(),robot.gimbal_task_p->k1);
//			robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.z = -0.243265;
     // expect_vec = Vector3f(robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.x,
        //                    robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.y, robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.z);
			expect_vec = Vector3f(robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.x,
                            robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.y,robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.z);
			
//			expect_vec = Vector3f(robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.x,
//                            robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.y,robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.z);
//			  expect_vec = Vector3f(robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.x,
                           //robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.y,robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.z);
    }
    else
    {
      expect_vec = expect_vec.rotateAboutAxis(coord_chassis.k, gimbal_yaw_vel * dT_s);
      expect_vec = expect_vec.rotateAboutAxis(coord_imu.j, gimbal_pitch_vel * dT_s);
    }
  }
//  coord_imu.normalize();
//  coord_yaw.normalize();
//  coord_chassis.normalize();
  expect_vec.normalize();
  gimbal_soft_limit();

  exp_cross = expect_vec.crossProduct(coord_chassis.k);//������������*��������ϵk��
  imu_cross = coord_imu.i.crossProduct(coord_chassis.k);//����

  exp_cross.normalize();
  imu_cross.normalize();

  float sine = imu_cross.mixedProduct(exp_cross, coord_chassis.k);
  float cosine = imu_cross.dotProduct(exp_cross);

  delta_yaw = -atan2f(sine, cosine);

  float pitch_exp, pitch_imu; // ����ϵ���pitch
  pitch_exp = coord_chassis.k.dotProduct(expect_vec);
  pitch_imu = coord_chassis.k.dotProduct(coord_imu.i);

  delta_pitch = -(acosf(pitch_exp) - acosf(pitch_imu)); //from aim to camera

//  // ���㵱ǰ������������ָ̨�������Ĳ��
//  delta_angle_vec = expect_vec.crossProduct(coord_imu.i);

//  // ����˵Ľ��ģ����Ϊ�������ǶȲ�
//  delta_angle_vec.uniformScaling(asin(delta_angle_vec.length()) / delta_angle_vec.length());

  // ����ʱ�Ĳ�����������ϵ��k���׵�ˣ��õ�yaw��������ת�Ǻ͵�ǰת�ǵļн�
  // ����ʱ�Ĳ��������imuϵ��j���׵�ˣ��õ�pitch��������ת�Ǻ͵�ǰת�ǵļн�
  // delta_yaw = delta_angle_vec.dotProduct(coord_chassis.k);
  // delta_pitch = delta_angle_vec.dotProduct(coord_imu.j);

  // ���ٶ�
  gyro_vec = Matrix3f(coord_imu.i, coord_imu.j, coord_imu.k);
  gyro_vec.i = gyro_vec.i.uniformScaling(robot.inertial_sensors.getGyro().x);
  gyro_vec.j = gyro_vec.j.uniformScaling(robot.inertial_sensors.getGyro().y);
  gyro_vec.k = gyro_vec.k.uniformScaling(robot.inertial_sensors.getGyro().z);

  // ����ʱIMU�Ľ��ٶ����������ϵ��k���׵�ˣ��õ�yaw�������ٶ�
  // ����ʱIMU�Ľ��ٶ�������imuϵ��j���׵�ˣ��õ�pitch�������ٶ�
  motor_yaw_pid_task->motor_backend_p->setAngularVelocityIMU(gyro_vec.i.dotProduct(coord_chassis.k) + gyro_vec.j.dotProduct(coord_chassis.k) + gyro_vec.k.dotProduct(coord_chassis.k));
  motor_pitch_pid_task->motor_backend_p->setAngularVelocityIMU(gyro_vec.i.dotProduct(coord_imu.j) + gyro_vec.j.dotProduct(coord_imu.j) + gyro_vec.k.dotProduct(coord_imu.j));

  /* ����yaw���� */

  motor_yaw_pid_task->angle_control_task_p->setPIDControllerFeedback(
    delta_yaw
  );
  motor_yaw_pid_task->angle_control_task_p->setPIDControllerExpect(
    0
  );
  motor_yaw_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(
    motor_yaw_pid_task->motor_backend_p->getCommonMeasurement().rotator.angular_velocity_imu
  );

  motor_yaw_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
    motor_yaw_pid_task->angle_control_task_p->getOutput()
  );

  motor_yaw_pid_task->angular_velocity_control_task_p->setPIDControllerFeedforward(
    motor_yaw_pid_task->angle_control_task_p->getOutput()
  );

  /* ����pitch���� */

  motor_pitch_pid_task->angle_control_task_p->setPIDControllerFeedback(
    delta_pitch
  );
  motor_pitch_pid_task->angle_control_task_p->setPIDControllerExpect(
    0
  );

  motor_pitch_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(
    motor_pitch_pid_task->motor_backend_p->getCommonMeasurement().rotator.angular_velocity_imu
  );

  motor_pitch_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
    motor_pitch_pid_task->angle_control_task_p->getOutput()
  );
  // Debug_Printf("%12.4f%12.4f\r\n", motor_yaw_pid_task->motor_backend_p->getCommonMeasurement().rotator.angular_velocity_imu*1000, motor_yaw_pid_task->angle_control_task_p->getOutput()*1000);


  // ����
  if(gimbal_mode == GIMBAL_OFF)
  {
    motor_yaw_pid_task->motor_backend_p->setMotorInput(
      0
    );
    motor_pitch_pid_task->motor_backend_p->setMotorInput(
      0
    );
		motor_pitch_pid_task->angular_velocity_control_task_p->clearPID();
		motor_yaw_pid_task->angular_velocity_control_task_p->clearPID();
		
  }
  else
  {
    motor_yaw_pid_task->motor_backend_p->setMotorInput(
      motor_yaw_pid_task->angular_velocity_control_task_p->getOutput()
    );
    motor_pitch_pid_task->motor_backend_p->setMotorInput(
    motor_pitch_pid_task->angular_velocity_control_task_p->getOutput()
    );
  }
}

/***********************************************************************
** �� �� ���� GimbalControlTask::uninit()
** ����˵���� ���񷴳�ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void GimbalControlTask::uninit(void) {}

/***********************************************************************
** �� �� ���� GimbalControlTask::gimbal_soft_limit()
** ����˵���� ��̨����λ,�ڿ�����̨�Ƕ�ǰ����
**---------------------------------------------------------------------
** ��������� ��/����̨������������/����̨��λ����������ϵk����(������)����̨����ϵj��������̨y�ᣩ
** ���ز����� ��
***********************************************************************/
void GimbalControlTask::gimbal_soft_limit()
{
  float dot_product;
  Vector3f cross_product;
  
  cross_product = (coord_chassis.k).crossProduct(expect_vec);
  dot_product = expect_vec.dotProduct(coord_chassis.k);
  cross_product.normalize(); // ����������������˹�һ��
  
  static uint8_t flg_max, flg_min;

  // С����С�Ƕ���λ
  if(dot_product >= gimbal_limit_ang_min_cos)
  {
    expect_vec = (coord_chassis.k).rotateAboutAxis(cross_product, gimbal_limit_ang_min);
  }
  // ������С�Ƕ���λ
  else if(dot_product <= gimbal_limit_ang_max_cos)
  {
    expect_vec = (coord_chassis.k).rotateAboutAxis(cross_product, gimbal_limit_ang_max);
  }

	
}



