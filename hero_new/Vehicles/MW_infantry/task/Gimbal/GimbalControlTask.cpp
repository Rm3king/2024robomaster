/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   GimbalControlTask.cpp
** 文件说明：   云台控制任务
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     林子涵     	     2022-07-26
***************************************************************************/
#include "/Gimbal/GimbalControlTask.h"
#include "/Robot/Robot.h"
#include "UARTDriver.h"

/***********************************************************************
** 函 数 名： GimbalControlTask构造函数
** 函数说明： 指定robot引用，CAN设备引用，指定yaw和pitch轴电机CAN总线id
**            和CAN总线数据包发送起始下标，指定任务更新时间间隔，同时
**            给yaw和pitch轴电机注册Motor_RM_PIDControlTask任务
**---------------------------------------------------------------------
** 输入参数： robot引用、CAN设备引用、yaw和pitch轴电机CAN总线id、
**            CAN总线数据包发送起始下标、任务更新时间间隔
** 返回参数： 无
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
	if (fabs(fa) < threshold) return a;//fabs()为绝对值函数，防止取到负数也小于D,z=只要小于D，便趋近0，为根（精度1e-8）
	if (fabs(fb) < threshold) return b;
	while (fabs(b - a) > threshold)//只要区间距离不是足够小的话就继续循环。 
	{
		x = (a + b) / 2;
		fx = f(x, sqrt(dx * dx + dy * dy), dz,v2);
		if (fabs(fx) < threshold) return x;
		if (fx * fa < 0) { b = x; fb = fx; }
		else { a = x; fa = fx; }//如果x与a函数值异号，那么在此区间内，x就是新的b；否则x必然和b异号，x是新的a。
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
		if (dy < ey) //计算值偏小
		{
			lowangle = b;
			b = (upangle + lowangle) / 2;
			a = b / 57.3;
			dy = (m*g*dx) / (k1*v0*cos(a)) + dx * tan(a) + ((m*m*g) / (k1*k1))*log(1 - (k1*dx) / (m*v0*cos(a)));//角越大值越大
		}

		else //计算值偏大
		{
			upangle = b;
			b = (upangle + lowangle) / 2;
			a = b / 57.3;
			dy = (m*g*dx) / (k1*v0*cos(a)) + dx * tan(a) + ((m*m*g) / (k1*k1))*log(1 - (k1*dx) / (m*v0*cos(a)));//角越大值越大
		}
		if(i>ex_number) return z_distance;
	}
	return tan(b/180*3.1415926)*dx;
}
/***********************************************************************
** 函 数 名： GimbalControlTask::init()
** 函数说明： 注册云台各个电机Motor_RM_PIDControlTask任务
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
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

  expect_vec.set(1, 0, 0);//设置初始期望向量为(1，0，0)

  gimbal_limit_ang_max_cos = arm_cos_f32(gimbal_limit_ang_max);
  gimbal_limit_ang_min_cos = arm_cos_f32(gimbal_limit_ang_min);
  
	gimbal_mode = GIMBAL_RC_MODE;
  lp_pitch_out.setCutOffFreq(20);
//  lp_pitch_out.setSamplingCycle(this->interval_tick_us / 1e6f);
}

/***********************************************************************
** 函 数 名： GimbalControlTask::update(timeus_t dT_us)
** 函数说明： 通过云台电机编码器和IMU计算云台Yaw轴机构、车底盘
**            等部分的空间姿态，更新云台各个电机控制任务，并发送
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void GimbalControlTask::update(timeus_t dT_us)
{
  float dT_s = dT_us / 1e6f;

  // 通过IMU和电机编码器计算Yaw轴机构坐标系和底盘坐标系

  // 获取IMU系
  coord_imu = robot.attitude_solution_task_p->getIMUCoord();

  // 通过Pitch轴编码器逆转获得Yaw轴机构坐标系
  coord_yaw = coord_imu.rotateAboutAxis(coord_imu.j, - motor_pitch_pid_task->motor_backend_p->getCommonMeasurement().rotator.polar_angle);

  // 通过Yaw轴编码器逆转获得底盘坐标系
  coord_chassis = coord_yaw.rotateAboutAxis(coord_yaw.k, - motor_yaw_pid_task->motor_backend_p->getCommonMeasurement().rotator.polar_angle);


	distance = sqrt(robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.x*robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.x+robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.y*robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.y);

   if(gimbal_mode == GIMBAL_RC_MODE)//遥控器模式
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
    // 只执行80ms内发来的数据
    if(micros() - robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.getTimestamp() < 50000)
    {
//			gimbal_ammo_auto_z =sqrt(robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.x*robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.x+
//			robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.y*robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.y)*
//			tan(shift_Topi(root_between(-30.0,45.0,robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.x,robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.y,robot.cv_control_task_p->gimbal_constant_aim_shoot_cmd.z,robot.referee_system_task_p->get_filter_speed())));
//手动补偿
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

  exp_cross = expect_vec.crossProduct(coord_chassis.k);//计算期望向量*底盘坐标系k轴
  imu_cross = coord_imu.i.crossProduct(coord_chassis.k);//计算

  exp_cross.normalize();
  imu_cross.normalize();

  float sine = imu_cross.mixedProduct(exp_cross, coord_chassis.k);
  float cosine = imu_cross.dotProduct(exp_cross);

  delta_yaw = -atan2f(sine, cosine);

  float pitch_exp, pitch_imu; // 底盘系里的pitch
  pitch_exp = coord_chassis.k.dotProduct(expect_vec);
  pitch_imu = coord_chassis.k.dotProduct(coord_imu.i);

  delta_pitch = -(acosf(pitch_exp) - acosf(pitch_imu)); //from aim to camera

//  // 计算当前期望向量和云台指向向量的叉乘
//  delta_angle_vec = expect_vec.crossProduct(coord_imu.i);

//  // 将叉乘的结果模长设为两向量角度差
//  delta_angle_vec.uniformScaling(asin(delta_angle_vec.length()) / delta_angle_vec.length());

  // 将此时的叉乘向量与底盘系的k基底点乘，得到yaw轴电机期望转角和当前转角的夹角
  // 将此时的叉乘向量与imu系的j基底点乘，得到pitch轴电机期望转角和当前转角的夹角
  // delta_yaw = delta_angle_vec.dotProduct(coord_chassis.k);
  // delta_pitch = delta_angle_vec.dotProduct(coord_imu.j);

  // 角速度
  gyro_vec = Matrix3f(coord_imu.i, coord_imu.j, coord_imu.k);
  gyro_vec.i = gyro_vec.i.uniformScaling(robot.inertial_sensors.getGyro().x);
  gyro_vec.j = gyro_vec.j.uniformScaling(robot.inertial_sensors.getGyro().y);
  gyro_vec.k = gyro_vec.k.uniformScaling(robot.inertial_sensors.getGyro().z);

  // 将此时IMU的角速度向量与底盘系的k基底点乘，得到yaw轴电机角速度
  // 将此时IMU的角速度向量与imu系的j基底点乘，得到pitch轴电机角速度
  motor_yaw_pid_task->motor_backend_p->setAngularVelocityIMU(gyro_vec.i.dotProduct(coord_chassis.k) + gyro_vec.j.dotProduct(coord_chassis.k) + gyro_vec.k.dotProduct(coord_chassis.k));
  motor_pitch_pid_task->motor_backend_p->setAngularVelocityIMU(gyro_vec.i.dotProduct(coord_imu.j) + gyro_vec.j.dotProduct(coord_imu.j) + gyro_vec.k.dotProduct(coord_imu.j));

  /* 控制yaw轴电机 */

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

  /* 控制pitch轴电机 */

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


  // 发送
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
** 函 数 名： GimbalControlTask::uninit()
** 函数说明： 任务反初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void GimbalControlTask::uninit(void) {}

/***********************************************************************
** 函 数 名： GimbalControlTask::gimbal_soft_limit()
** 函数说明： 云台软限位,在控制云台角度前运行
**---------------------------------------------------------------------
** 输入参数： 上/下云台期望向量、上/下云台限位、底盘坐标系k分量(法向量)、云台坐标系j分量（云台y轴）
** 返回参数： 无
***********************************************************************/
void GimbalControlTask::gimbal_soft_limit()
{
  float dot_product;
  Vector3f cross_product;
  
  cross_product = (coord_chassis.k).crossProduct(expect_vec);
  dot_product = expect_vec.dotProduct(coord_chassis.k);
  cross_product.normalize(); // 求法向量，这里别忘了归一化
  
  static uint8_t flg_max, flg_min;

  // 小于最小角度限位
  if(dot_product >= gimbal_limit_ang_min_cos)
  {
    expect_vec = (coord_chassis.k).rotateAboutAxis(cross_product, gimbal_limit_ang_min);
  }
  // 大于最小角度限位
  else if(dot_product <= gimbal_limit_ang_max_cos)
  {
    expect_vec = (coord_chassis.k).rotateAboutAxis(cross_product, gimbal_limit_ang_max);
  }

	
}



