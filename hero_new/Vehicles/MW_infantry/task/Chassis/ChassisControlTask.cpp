/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   ChassisControlTask.cpp
** 文件说明：   云台控制任务
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     林子涵     	     2022-07-26
***************************************************************************/
#include "/Chassis/ChassisControlTask.h"
#include "/Robot/Robot.h"
#include "/Robot/Params.h"
#include "UARTDriver.h"
#include "math.h"
/***********************************************************************
** 函 数 名： ChassisControlTask构造函数
** 函数说明： 指定robot引用，CAN设备引用，指定电机CAN总线id
**            和CAN总线数据包发送起始下标，指定任务更新时间间隔，同时
**            给电机注册Motor_RM_PIDControlTask任务
**---------------------------------------------------------------------
** 输入参数： robot引用、CAN设备引用、yaw和pitch轴电机CAN总线id、
**            CAN总线数据包发送起始下标、任务更新时间间隔
** 返回参数： 无
***********************************************************************/

ChassisControlTask::ChassisControlTask(
  Robot &robot0,
  Motor_RM_Params_t *m1, PID_Params_t *m1_ang, PID_Params_t *m1_ang_vel,
  Motor_RM_Params_t *m2, PID_Params_t *m2_ang, PID_Params_t *m2_ang_vel,
  Motor_RM_Params_t *m3, PID_Params_t *m3_ang, PID_Params_t *m3_ang_vel,
  Motor_RM_Params_t *m4, PID_Params_t *m4_ang, PID_Params_t *m4_ang_vel,
	PID_Params_t *chassis_follow_ang,CAN_Rx_Data_Pack_t &_CAN_SUPERCAP_RX,
	CAN_Tx_Data_Link_t &_CAN_SUPERCAP_TX,
  timeus_t interval_tick_us0
) : Task_Base(robot0)
{
  this->interval_tick_us = interval_tick_us0;

  if(m1 != NULL)
  {
    if(m1->canx == 1)
    {
      this->motor_1_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          m1->can_rx_id,
          m1->can_tx_id,
          m1->can_tx_data_start_pos,
          RoboMaster_3508,
          m1->interval);
    }
    else if(m1->canx == 2)
    {
      this->motor_1_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          m1->can_rx_id,
          m1->can_tx_id,
          m1->can_tx_data_start_pos,
          RoboMaster_3508,
          m1->interval);
    }
    this->motor_1_pid_task->motor_backend_p->setParams(*m1);
  }

  if(m2 != NULL)
  {
    if(m2->canx == 1)
    {
      this->motor_2_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          m2->can_rx_id,
          m2->can_tx_id,
          m2->can_tx_data_start_pos,
          RoboMaster_3508,
          m2->interval);
    }
    else if(m2->canx == 2)
    {
      this->motor_2_pid_task = new Motor_RM_PIDControlTask(robot0,
robot0.can2_device,
          m2->can_rx_id,
          m2->can_tx_id,
          m2->can_tx_data_start_pos,
          RoboMaster_3508,
          m2->interval);
    }
    this->motor_2_pid_task->motor_backend_p->setParams(*m2);

  }

  if(m3 != NULL)
  {
    if(m3->canx == 1)
    {
      this->motor_3_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          m3->can_rx_id,
          m3->can_tx_id,
          m3->can_tx_data_start_pos,
          RoboMaster_3508,
          m3->interval);
    }
    else if(m3->canx == 2)
    {
      this->motor_3_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          m3->can_rx_id,
          m3->can_tx_id,
          m3->can_tx_data_start_pos,
          RoboMaster_3508,
          m3->interval);
    }
    this->motor_3_pid_task->motor_backend_p->setParams(*m3);

  }
  if(m4 != NULL)
  {
    if(m4->canx == 1)
    {
      this->motor_4_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          m4->can_rx_id,
          m4->can_tx_id,
          m4->can_tx_data_start_pos,
          RoboMaster_3508,
          m1->interval);
    }
    else if(m4->canx == 2)
    {
      this->motor_4_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          m4->can_rx_id,
          m4->can_tx_id,
          m4->can_tx_data_start_pos,
          RoboMaster_3508,
          m4->interval);
    }
    this->motor_4_pid_task->motor_backend_p->setParams(*m4);
  }

  if(m1_ang_vel != NULL)
  {
    this->motor_1_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*m1_ang_vel);
    this->motor_1_pid_task->angular_velocity_control_task_p->setInterval((*m1_ang_vel).interval);
  }

  if(m1_ang != NULL)
  {
    this->motor_1_pid_task->angle_control_task_p->setPIDControllerParams(*m1_ang);
    this->motor_1_pid_task->angle_control_task_p->setInterval((*m1_ang).interval);
  }


  if(m2_ang_vel != NULL)
  {
    this->motor_2_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*m2_ang_vel);
    this->motor_2_pid_task->angular_velocity_control_task_p->setInterval((*m2_ang_vel).interval);
  }

  if(m2_ang != NULL)
  {
    this->motor_2_pid_task->angle_control_task_p->setPIDControllerParams(*m2_ang);
    this->motor_2_pid_task->angle_control_task_p->setInterval((*m2_ang).interval);
  }

  if(m3_ang_vel != NULL)
  {
    this->motor_3_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*m3_ang_vel);
    this->motor_3_pid_task->angular_velocity_control_task_p->setInterval((*m3_ang_vel).interval);
  }

  if(m3_ang != NULL)
  {
    this->motor_3_pid_task->angle_control_task_p->setPIDControllerParams(*m3_ang);
    this->motor_3_pid_task->angle_control_task_p->setInterval((*m3_ang).interval);
  }

  if(m4_ang_vel != NULL)
  {
    this->motor_4_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*m4_ang_vel);
    this->motor_4_pid_task->angular_velocity_control_task_p->setInterval((*m4_ang_vel).interval);
  }

  if(m4_ang != NULL)
  {
    this->motor_4_pid_task->angle_control_task_p->setPIDControllerParams(*m4_ang);
    this->motor_4_pid_task->angle_control_task_p->setInterval((*m4_ang).interval);
  }

	if(chassis_follow_ang != NULL)
  {
		chassis_follow_pid_task = new PIDControlTask(robot, 0);
    this->chassis_follow_pid_task->setPIDControllerParams(*chassis_follow_ang);
    this->chassis_follow_pid_task->setInterval((*chassis_follow_ang).interval);
  }
	
		
		CAN_SUPERCAP_RX_PACK = _CAN_SUPERCAP_RX;
		robot0.can2_device.addRxLink(&CAN_SUPERCAP_RX_PACK);
		
		CAN_SUPERCAP_TX_PACK = _CAN_SUPERCAP_TX;
		robot0.can2_device.addTxLink(CAN_SUPERCAP_TX_PACK.std_id,CAN_SUPERCAP_TX_PACK.dlc,0,can_tx_data,2);
		/*SSH*/
		supercap_voltage_lpf.setCutOffFreq(0.5);
		supercap_voltage_lpf.setSamplingCycle(this->interval_tick_us / 1e6f);
		supercap_percent_lpf.setCutOffFreq(0.5);
		supercap_percent_lpf.setSamplingCycle(this->interval_tick_us / 1e6f);
}

/***********************************************************************
** 函 数 名： ChassisControlTask::init()
** 函数说明： 注册云台各个电机Motor_RM_PIDControlTask任务
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void ChassisControlTask::init(void)
{
  inited = true;
	/* 电机传感器数据获取任务注册 */
  if(motor_1_pid_task != NULL)
    robot.scheduler.registerTask(motor_1_pid_task);
  if(motor_2_pid_task != NULL)
    robot.scheduler.registerTask(motor_2_pid_task);
  if(motor_3_pid_task != NULL)
    robot.scheduler.registerTask(motor_3_pid_task);
  if(motor_4_pid_task != NULL)
    robot.scheduler.registerTask(motor_4_pid_task);
	/* 电机传感器数据获取任务注册 */
	
	/* PID任务注册 */
	if(chassis_follow_pid_task != NULL)
		robot.scheduler.registerTask(chassis_follow_pid_task);

  if(motor_1_pid_task->angular_velocity_control_task_p != NULL)
    robot.scheduler.registerTask(motor_1_pid_task->angular_velocity_control_task_p);

  if(motor_2_pid_task->angular_velocity_control_task_p != NULL)
    robot.scheduler.registerTask(motor_2_pid_task->angular_velocity_control_task_p);

  if(motor_3_pid_task->angular_velocity_control_task_p != NULL)
    robot.scheduler.registerTask(motor_3_pid_task->angular_velocity_control_task_p);

  if(motor_4_pid_task->angular_velocity_control_task_p != NULL)
    robot.scheduler.registerTask(motor_4_pid_task->angular_velocity_control_task_p);
 	/* PID任务注册 */	
  half_wheel_base = INITIAL_HALF_WHEEL_BASE;
  half_tread = INITIAL_HALF_TREAD;
  rotate_ratio_x = INITIAL_ROTATE_RATIO_X;
  rotate_ratio_y = INITIAL_ROTATE_RATIO_Y;
  wheel_vel_ratio = INITIAL_WHEEL_VEL_RATIO;
	xy_chassis_slow = INITIAL_XY_Chassis_Slow;
	chassis_mode = Chassis_FOLLOW;
	
	rotate_ratio[0]=std::sqrtf(cmt::sq(half_wheel_base*(1.0f-rotate_ratio_x)) + cmt::sq(half_tread*(1.0f+rotate_ratio_y)));
	rotate_ratio[1]=std::sqrtf(cmt::sq(half_wheel_base*(1.0-rotate_ratio_x)) + cmt::sq(half_tread*(1.0f-rotate_ratio_y)));
	rotate_ratio[2]=std::sqrtf(cmt::sq(half_wheel_base*(1.0f+rotate_ratio_x)) + cmt::sq(half_tread*(1.0f-rotate_ratio_y)));
	rotate_ratio[3]=std::sqrtf(cmt::sq(half_wheel_base*(1.0f+rotate_ratio_x)) + cmt::sq(half_tread*(1.0f+rotate_ratio_y)));
}

/***********************************************************************
** 函 数 名： ChassisControlTask::update(timeus_t dT_us)
** 函数说明： 通过云台电机编码器和IMU计算云台Yaw轴机构、车底盘
**            等部分的空间姿态，更新云台各个电机控制任务，并发送
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void ChassisControlTask::update(timeus_t dT_us)
{
	//超级电容数据解析
	suercap_RX_data.supercap_voltage = (int32_t)((CAN_SUPERCAP_RX_PACK.data[0]<<24)|(CAN_SUPERCAP_RX_PACK.data[1]<<16)|(CAN_SUPERCAP_RX_PACK.data[2]<<8)|(CAN_SUPERCAP_RX_PACK.data[3]&0xFF))/10000.0;
	suercap_RX_data.supercap_energy_percent = (int32_t)((CAN_SUPERCAP_RX_PACK.data[4]<<24)|(CAN_SUPERCAP_RX_PACK.data[5]<<16)|(CAN_SUPERCAP_RX_PACK.data[6]<<8)|(CAN_SUPERCAP_RX_PACK.data[7]&0xFF))/10000.0;
	//SSH
	 supercap_voltage_lpf.calc(dT_us, suercap_RX_data.supercap_voltage);
	supercap_percent_lpf.calc(dT_us, suercap_RX_data.supercap_energy_percent);
	if(robot.rc_protocol.getRCData().sw_right == 2 && robot.rc_protocol.getRCData().sw_left == 2 )//键鼠控制
	{  
				//SHIFT加速
		if(robot.rc_protocol.getRCData().keyboard.key_bit.SHIFT)
		{
			robot.chassis_task_p->wheel_vel_ratio = 1.6f;
			robot.chassis_task_p->xy_chassis_slow = 1.8f;
		}
		else 
		{
		  robot.chassis_task_p->wheel_vel_ratio = 1.0f;
			robot.chassis_task_p->xy_chassis_slow = 1.0f;
		}
		if(robot.rc_protocol.getRCData().keyboard.key_bit.W ||robot.rc_protocol.getRCData().keyboard.key_bit.S)
		{			
			vx += (robot.rc_protocol.getRCData().keyboard.key_bit.W - robot.rc_protocol.getRCData().keyboard.key_bit.S)*delta_vx;//键鼠控制缓起步
			if(vx > max_vx){vx = max_vx;}
			else if(vx < min_vx){vx = min_vx;}
		}
		else 
        {if(_Chassis_Abs(vx)>0.1) vx = _Chassis_Minish(vx,delta_vx);
         else vx = 0;
        }
		if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0){
		if(robot.rc_protocol.getRCData().keyboard.key_bit.A ||robot.rc_protocol.getRCData().keyboard.key_bit.D)
		{		
			vy += (robot.rc_protocol.getRCData().keyboard.key_bit.A - robot.rc_protocol.getRCData().keyboard.key_bit.D)*delta_vy;
			if(vy > max_vy){vy = max_vy;}
			else if(vy < min_vy){vy = min_vy;}
		}
		else 
        {if(_Chassis_Abs(vy)>0.1) vy = _Chassis_Minish(vy,delta_vy);
         else vy = 0;
        }
		  
		 if(last_level != robot.getRefereeSystemTaskPointer()->getRobotRefereeStatus_t().game_robot_status.robot_level)
		 {
			 last_level = robot.getRefereeSystemTaskPointer()->getRobotRefereeStatus_t().game_robot_status.robot_level;
			 switch(robot.getRefereeSystemTaskPointer()->getRobotRefereeStatus_t().game_robot_status.robot_level)
			 {
				 case 1:
					  max_vx = 1.4;
                 max_vy = 1.4;
                 min_vx =-1.4;
                 min_vy =-1.4;
				 break;
				 
				 case 2:
					  max_vx = 1.65;
                 max_vy = 1.65;
                 min_vx =-1.65;
                 min_vy =-1.65;
				 break;
				 
				 case 3:
					  max_vx = 1.9;
                 max_vy = 1.9;
                 min_vx =-1.9;
                 min_vy =-1.9;
				 break;
			 }
			 
		 }

		if(robot.rc_protocol.getRCData().keyboard.key_bit.F)
        {
            max_vx +=0.0003;
            max_vy +=0.0003;
            min_vx -=0.0003;
            min_vy -=0.0003;
        }
    }
//        else if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 1){
//        if(robot.rc_protocol.getRCData().keyboard.key_bit.Z)
//        {
//            max_vx +=0.0003;
//            max_vy +=0.0003;
//            min_vx -=0.0003;
//            min_vy -=0.0003;
//        }
        if(robot.rc_protocol.getRCData().keyboard.key_bit.C)
        {
            max_vx -=0.0003;
            max_vy -=0.0003;
            min_vx +=0.0003;
            min_vy +=0.0003;
        }
        
//    }
		 //最大速度限幅
      if(max_vx>8.0){
          max_vx =8.0;
          max_vy =8.0;
          min_vx =-8.0;
          min_vy =-8.0;
      }
      else if(max_vx<0.6)
      {
          max_vx =0.6;
          max_vy =0.6;
          min_vx =-0.6;
          min_vy =-0.6;          
      }
	}
	if(chassis_mode == Chassis_OFF)
	{
//		vx = 0;
//		vy = 0;
//		vw = 0;
		motor_1_pid_task->angular_velocity_control_task_p->clearPID();
		motor_2_pid_task->angular_velocity_control_task_p->clearPID();
		motor_3_pid_task->angular_velocity_control_task_p->clearPID();
		motor_4_pid_task->angular_velocity_control_task_p->clearPID();
	}
	
	else if(chassis_mode == Chassis_FOLLOW)
	{
		chassis_follow_pid_task->setPIDControllerExpect(0);
		chassis_follow_pid_task->setPIDControllerFeedback(robot.gimbal_task_p->getYawPIDControlTaskPointer()->motor_backend_p->getMeasurement().ecd);
		vw = chassis_follow_pid_task->getOutput();
        chassis_angle = robot.getGimbalControlTaskPointer()->getYawPIDControlTaskPointer()->motor_backend_p->getMeasurement().ecd * 2 * PI /8192;
      if(vw > 1.8*max_vw){vw = 1.8*max_vw;}
			else if(vw < 1.8*min_vw){vw = 1.8*min_vw;}
	}
	else if(chassis_mode==Chassis_45to_enemy)
 {
	 
		chassis_follow_pid_task->setPIDControllerExpect(0.125*8192);
		chassis_follow_pid_task->setPIDControllerFeedback(robot.gimbal_task_p->getYawPIDControlTaskPointer()->motor_backend_p->getMeasurement().ecd);
		vw = chassis_follow_pid_task->getOutput();
        chassis_angle = robot.getGimbalControlTaskPointer()->getYawPIDControlTaskPointer()->motor_backend_p->getMeasurement().ecd * 2 * PI /8192;
      if(vw > 1.8*max_vw){vw = 1.8*max_vw;}
			else if(vw < 1.8*min_vw){vw = 1.8*min_vw;}
	 
 }
	
//	else if(chassis_mode == Chassis_TOP)
//	{
//		if(vx == 0 && vy == 0)
//		{
//			vw = 2.0f*robot.referee_system_task_p->getRobotRefereeStatus_t().game_robot_status.robot_level;
//		}
//		else
//		{
//			vw = 0.8f*robot.referee_system_task_p->getRobotRefereeStatus_t().game_robot_status.robot_level;
//		}
//		chassis_angle = robot.getGimbalControlTaskPointer()->getYawPIDControlTaskPointer()->motor_backend_p->getMeasurement().ecd * 2 * PI /8192;
//      if(vw > max_vw){vw = 1.2*max_vw;}
//			else if(vw < min_vw){vw = 1.2*min_vw;}
//	}
	else if(chassis_mode == Chassis_TOP)
	{
		vw+=delta_vw;
		if(vx == 0 && vy == 0)
		{
			if(vw>1.8*max_vx)
			{
			vw = 1.8*max_vx;
			}
		}
		else
		{
			if(vw>1.1*max_vx)
			{
			vw = 1.1*max_vx;
			}
		}
		chassis_angle = robot.getGimbalControlTaskPointer()->getYawPIDControlTaskPointer()->motor_backend_p->getMeasurement().ecd * 2 * PI /8192;
//      if(vw > max_vw){vw = 1.2*max_vw;}
//			else if(vw < min_vw){vw = 1.2*min_vw;}
	}
    //CTRL键系下A D转向
	else if(chassis_mode == Chassis_FREE)
    {
        
        if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL){
        if(robot.rc_protocol.getRCData().keyboard.key_bit.A ||robot.rc_protocol.getRCData().keyboard.key_bit.D)
		{		
			vw += (robot.rc_protocol.getRCData().keyboard.key_bit.A - robot.rc_protocol.getRCData().keyboard.key_bit.D)*delta_vw;
			if(vw > max_vw){vw = max_vw;}
			else if(vw < min_vw){vw = min_vw;}
		}
    }
       else {vw = 0;}
        chassis_angle = 0;
    }
	vx_resolve = vx * arm_cos_f32(-chassis_angle) - vy * (-chassis_angle);
	vy_resolve = vx * arm_sin_f32(-chassis_angle) + vy * arm_cos_f32(-chassis_angle);	
	//超级电容限电
		if(supercap_voltage_lpf.getOutput()<20&&supercap_voltage_lpf.getOutput()>7){
			super_chassis_slow =1.0f - (20.0f - supercap_voltage_lpf.getOutput()) / 12.4f;
			super_vw_slow = 1.0f - (20.0f - supercap_voltage_lpf.getOutput()) / 15.0f;
			if(super_chassis_slow<0.3)
			{
				super_chassis_slow = 0.3;
			}
			if(super_vw_slow<0.7)
			{
				super_vw_slow = 0.7;
			}
			
		}
		if(robot.getRefereeSystemTaskPointer()->getRobotRefereeStatus_t().power_heat_data.chassis_power_buffer<25 ){
		motor_1_pid_task->angular_velocity_control_task_p->clearPID();
		motor_2_pid_task->angular_velocity_control_task_p->clearPID();
		motor_3_pid_task->angular_velocity_control_task_p->clearPID();
		motor_4_pid_task->angular_velocity_control_task_p->clearPID();
		}
			
  exp_vel[0] = (-vx_resolve - vy_resolve)*xy_chassis_slow*super_chassis_slow- vw * rotate_ratio[0] * wheel_vel_ratio*super_vw_slow;
  exp_vel[1] = (vx_resolve - vy_resolve)*xy_chassis_slow *super_chassis_slow- vw * rotate_ratio[1] * wheel_vel_ratio*super_vw_slow;
  exp_vel[2] = (vx_resolve + vy_resolve)*xy_chassis_slow*super_chassis_slow - vw * rotate_ratio[2] * wheel_vel_ratio*super_vw_slow;
  exp_vel[3] = (-vx_resolve + vy_resolve)*xy_chassis_slow *super_chassis_slow- vw * rotate_ratio[3] * wheel_vel_ratio*super_vw_slow;

  motor_1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(exp_vel[0]);
  motor_2_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(exp_vel[1]);
  motor_3_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(exp_vel[2]);
  motor_4_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(exp_vel[3]);
	
  motor_1_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_1_pid_task->motor_backend_p->getCommonMeasurement().linear.velocity);
  motor_2_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_2_pid_task->motor_backend_p->getCommonMeasurement().linear.velocity);
  motor_3_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_3_pid_task->motor_backend_p->getCommonMeasurement().linear.velocity);
  motor_4_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_4_pid_task->motor_backend_p->getCommonMeasurement().linear.velocity);
  
  motor_1_pid_task->motor_backend_p->setMotorInput(motor_1_pid_task->angular_velocity_control_task_p->getOutput());
  motor_2_pid_task->motor_backend_p->setMotorInput(motor_2_pid_task->angular_velocity_control_task_p->getOutput());
  motor_3_pid_task->motor_backend_p->setMotorInput(motor_3_pid_task->angular_velocity_control_task_p->getOutput());
  motor_4_pid_task->motor_backend_p->setMotorInput(motor_4_pid_task->angular_velocity_control_task_p->getOutput());
  
}

/***********************************************************************
** 函 数 名： ChassisControlTask::uninit()
** 函数说明： 任务反初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void ChassisControlTask::uninit(void) {}
