#include "/Gimbal/GimbalControlTask.h"
#include "/Remote/RemoteControlTask.h"
#include "/Robot/Robot.h"

#define JOYSTICK_DEAD_ZONE 5
#define JOYSTICK_MAX 700
#define one_ball_heat 100
/* ----------------------- RC Switch Definition----------------------------- */
#define SWITCH_UP                ((uint16_t)1)
#define SWITCH_MID               ((uint16_t)3)
#define SWITCH_DWN               ((uint16_t)2)

/* ----------------------- ROBOT Mode Definition---------------------------- */
#define MOTOR_DISABLE										((uint16_t)1)
#define CHASSIS_FOLLOW_GIMBAL						((uint16_t)2)
#define CHASSIS_GYRO_ROTATION						((uint16_t)3)
#define CHASSIS_FREE										((uint16_t)4)

/***********************************************************************
** 函 数 名： isBeyondDeadZone()
** 函数说明： 判断是否越过死区
**---------------------------------------------------------------------
** 输入参数： rc_data
** 返回参数： 无
***********************************************************************/
bool isBeyondDeadZone(int16_t rc_data)
{
  if(rc_data >= JOYSTICK_DEAD_ZONE || rc_data <= -JOYSTICK_DEAD_ZONE)
    return true;
  else
    return false;
}

/***********************************************************************
** 函 数 名： RemoteControlTask::init()
** 函数说明： 判断是否完成初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void RemoteControlTask::init(void)
{
  inited = true;
}

/***********************************************************************
** 函 数 名： RemoteControlTask::update()
** 函数说明： 处理接收到的rc_data[]中的数据，完成遥控器控制
**---------------------------------------------------------------------
** 输入参数： 更新周期
** 返回参数： 无
***********************************************************************/
void RemoteControlTask::update(timeus_t dT_us)
{	
		/* 键鼠控制 */
	//比赛键鼠控制时，将遥控器左开关和右开关都拨到最下方
	if(robot.rc_protocol.getRCData().sw_right == SWITCH_DWN && robot.rc_protocol.getRCData().sw_left == SWITCH_DWN )
{
	
//	        //E键  开启吊射模式？
//        if(robot.rc_protocol.getRCData().keyboard.key_bit.E && autoaim_lock == 0)
//		{
//			autoaim_lock = 1;
//			switch(autoaim_flag)
//			{
//				case 1:
//             	autoaim_flag = 2;
//               break;
//				case 2:
//             	autoaim_flag = 3;
//               break; 				
//				case 3:
//             	autoaim_flag = 1;
//               break;	
//			}
//			robot.gimbal_task_p->gimbal_mode = GIMABAL_HANGING_MODE;
//		}
//		if(robot.rc_protocol.getRCData().keyboard.key_bit.E == 0 && time_cnt < 21 && autoaim_lock ==  1) //0.2秒最多手动切换一次
//		{
//			time_cnt ++;
//		}		
//			if(time_cnt == 21) //0.2秒最多手动切换一次
//		{
//			time_cnt = 0;
//			autoaim_lock = 0;
//		}	
	
        if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0){
		if(robot.rc_protocol.getRCData().keyboard.key_bit.Z)
        {
//			   robot.chassis_task_p->chassis_mode = Chassis_OFF;//底盘失能
//            robot.gimbal_task_p->gimbal_mode = GIMBAL_OFF;//云台失能
//            robot.ammo_task_p->booster_mode = BOOSTER_OFF;//发射机构失能
					robot.gimbal_task_p->gimbal_mode = GIMBAL_HANGING_LOCKALL_MODE;
					robot.chassis_task_p->chassis_mode=Chassis_45to_enemy;
		      robot.chassis_task_p->set_Chassis_45toenemy_flag();
					
//            robot.ammo_task_p->set_Booster_off_flag();
        }
				//robot.gimbal_task_p->gimbal_mode = GIMBAL_RC_MODE;
		//开启底盘跟随云台模式 //按键C开启底盘跟随云台
		if(robot.rc_protocol.getRCData().keyboard.key_bit.V)
		{
            robot.gimbal_task_p->gimbal_mode = GIMBAL_RC_MODE;//云台正常控制
			robot.chassis_task_p->chassis_mode = Chassis_FOLLOW;//底盘跟随云台
			robot.chassis_task_p->clear_Chassis_TOP_flag();
			robot.chassis_task_p->clear_Chassis_45toenemy_flag();
		}
        //底盘云台分离模式
//        if(robot.rc_protocol.getRCData().keyboard.key_bit.C){
//            robot.gimbal_task_p->gimbal_mode = GIMBAL_RC_MODE;//云台正常控制
//            robot.chassis_task_p->chassis_mode = Chassis_FREE;//底盘跟随云台
//			   robot.chassis_task_p->clear_Chassis_TOP_flag();
//        }
		//开启小陀螺模式,键盘G
		if(robot.rc_protocol.getRCData().keyboard.key_bit.G)
		{
			robot.chassis_task_p->chassis_mode = Chassis_TOP;
			robot.chassis_task_p->set_Chassis_TOP_flag();
		}
		
		//开启或停止摩擦轮,键盘Q开启，Ctrl+Q关闭
		if(robot.rc_protocol.getRCData().keyboard.key_bit.Q)
		{
			robot.ammo_task_p->booster_mode = TRIGGER_AUTO;
            robot.ammo_task_p->set_Booster_on_flag();
		}
	 if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL && robot.rc_protocol.getRCData().keyboard.key_bit.Q)
		{
			robot.ammo_task_p->booster_mode = BOOSTER_OFF;
            robot.ammo_task_p->set_Booster_off_flag();
			robot.ammo_task_p->elec_enable = 1;
		}
		//卡弹 退弹。但存在着机械限位
		if(robot.rc_protocol.getRCData().keyboard.key_bit.R)
		{
            robot.ammo_task_p->block_flag = 1;
			robot.ammo_task_p->shoot_freq = -1.0f;
				robot.ammo_task_p->moving_flag = 1;
			double_press_flag = 1;
		}
		else {robot.ammo_task_p->block_flag = 0;robot.ammo_task_p->block_one_flag = 0;}
    if(robot.rc_protocol.getRCData().mouse.press_l)//鼠标左键发射
            {
					 
                robot.ammo_task_p->shoot_flag = 1; //发射标志位
					 double_press_flag = 1;
						robot.ammo_task_p->moving_flag = 1;
            }	
        else {robot.ammo_task_p->shoot_flag = 0;robot.ammo_task_p->one_flag = 0;}
//		  if(double_press_flag == 1)
//		  {
//			  double_press_clk++;
//			 if(double_press_clk == 30) {
//			  double_press_clk = 0;
//			  double_press_flag = 0;
//		  }
//		  }
        //检测枪管冷却
//		  if(robot.rc_protocol.getRCData().keyboard.key_bit.B)
//		  {robot.ammo_task_p->shoot_b_flag = 1;} //按B之后 热量限制标志位不起作用
//		  
		if(robot.getRefereeSystemTaskPointer()->getRobotRefereeStatus_t().game_robot_status.shooter_id1_42mm_cooling_limit-robot.getRefereeSystemTaskPointer()->getRobotRefereeStatus_t().power_heat_data.shooter_id1_42mm_cooling_heat-one_ball_heat<0)
			 {
					robot.ammo_task_p->shoot_flag = 0; //热量标志位变0 无法发射
			 }
//        //刷新UI
//        if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0){
		if(robot.rc_protocol.getRCData().keyboard.key_bit.X)
		{
			robot.referee_system_task_p->set_num_0();
			robot.referee_system_task_p->set_t_0();
		}	
//    }
        //E键  开启吊射模式
        if(robot.rc_protocol.getRCData().keyboard.key_bit.E)
		{
            robot.gimbal_task_p->gimbal_mode = GIMABAL_HANGING_MODE;
						robot.chassis_task_p->chassis_mode=Chassis_45to_enemy;
		        robot.chassis_task_p->set_Chassis_45toenemy_flag();
		}	
      		//鼠标右键长按 开始自瞄
			else if(robot.rc_protocol.getRCData().mouse.press_r && !press_r_flag )
    {
      press_r_flag = 1;
      if(robot.gimbal_task_p->gimbal_mode == GIMBAL_RC_MODE)
      {
        robot.gimbal_task_p->gimbal_mode = GIMBAL_AUTO_AIMING_MODE;
        robot.gimbal_task_p->set_GIMBAL_AUTO_AIMING_MODE_flag();
      }
		}
		 else if(!robot.rc_protocol.getRCData().mouse.press_r&&robot.gimbal_task_p->gimbal_mode != GIMBAL_OFF)
     {
      press_r_flag = 0;
			 robot.gimbal_task_p->gimbal_mode = GIMBAL_RC_MODE;
     }
	 if(robot.rc_protocol.getRCData().mouse.vx != 0)
    {
      robot.gimbal_task_p->gimbal_yaw_vel = -robot.rc_protocol.getRCData().mouse.vx / 15.0f;
      if(robot.gimbal_task_p->gimbal_yaw_vel > max_yaw_vel)
      {
        robot.gimbal_task_p->gimbal_yaw_vel = max_yaw_vel;
      }
      else if(robot.gimbal_task_p->gimbal_yaw_vel < -max_yaw_vel)
      {
        robot.gimbal_task_p->gimbal_yaw_vel = -max_yaw_vel;
      }
    }
    else
    {
      robot.gimbal_task_p->gimbal_yaw_vel = 0;

    }

    if(robot.rc_protocol.getRCData().mouse.vy != 0)
    {
      robot.gimbal_task_p->gimbal_pitch_vel = robot.rc_protocol.getRCData().mouse.vy / 15.0f;
      if(robot.gimbal_task_p->gimbal_pitch_vel > max_yaw_vel)
      {
        robot.gimbal_task_p->gimbal_pitch_vel = max_yaw_vel;
      }
      else if(robot.gimbal_task_p->gimbal_pitch_vel < -max_yaw_vel)
      {
        robot.gimbal_task_p->gimbal_pitch_vel = -max_yaw_vel;
      }

    }
    else
    {
      robot.gimbal_task_p->gimbal_pitch_vel = 0;
    }
    //鼠标滚轮  Ctrl+z调整速度上限
 }
	else if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 1)
	{
		if(robot.rc_protocol.getRCData().keyboard.key_bit.Z)
		{
			robot.ammo_task_p->set_exp_spd_flag();
		}
	  if(robot.rc_protocol.getRCData().keyboard.key_bit.Q)
		{
			robot.ammo_task_p->booster_mode = BOOSTER_OFF;
			robot.ammo_task_p->set_Booster_off_flag();
		}
	}
}

	/* 遥控器控制 Begin
	 遥控器左开关上中下分别控制gimbal(上)、ammo(中)、chassis(下)，右开关负责控制每个模块的具体模式 */
	/* 遥控器控制 Begin
	 遥控器左开关上中下分别控制gimbal(上)、ammo(中)、chassis(下)，右开关负责控制每个模块的具体模式 */
	else
 {
  if(robot.rc_protocol.getRCData().sw_left == SWITCH_UP)//左上
  {
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_UP && robot.rc_protocol.getRCData().wheel<-500)
		{
			robot.gimbal_task_p->gimbal_mode = GIMBAL_OFF;//左上右上+滑轮=云台失能
		}
		
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_MID && robot.rc_protocol.getRCData().wheel<-500)
		{
			robot.gimbal_task_p->gimbal_mode = GIMBAL_RC_MODE;//左上右中+滑轮=云台正常控制
		}
		
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_DWN && robot.rc_protocol.getRCData().wheel<-500)
		{
			//robot.gimbal_task_p->gimbal_mode = GIMBAL_AUTO_AIMING_MODE;//左上右下+滑轮=云台自瞄
			robot.gimbal_task_p->gimbal_mode = GIMBAL_AUTO_AIMING_MODE;
		}
  }
	
  else if(robot.rc_protocol.getRCData().sw_left == SWITCH_MID)//左中
  {
    if(robot.rc_protocol.getRCData().sw_right == SWITCH_DWN && robot.rc_protocol.getRCData().wheel<-500)
		{
			robot.ammo_task_p->booster_mode = BOOSTER_OFF;//左中右下+滑轮=发射机构失能
		}
		
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_MID && robot.rc_protocol.getRCData().wheel<-500)
		{
            robot.ammo_task_p->booster_mode = TRIGGER_AUTO;//单发模式   英雄 只设置单发
		}
  }
	
  else if(robot.rc_protocol.getRCData().sw_left == SWITCH_DWN)//左下
  {
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_UP && robot.rc_protocol.getRCData().wheel<-500)
		{
			robot.chassis_task_p->chassis_mode = Chassis_OFF;//左下右上+滑轮=底盘失能
		}
		
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_MID && robot.rc_protocol.getRCData().wheel<-500)
		{
			robot.chassis_task_p->chassis_mode = Chassis_FOLLOW;//左下右中+滑轮=底盘跟随云台模式
		}
		
//		if(robot.rc_protocol.getRCData().sw_right == SWITCH_DWN && robot.rc_protocol.getRCData().wheel<-500)
//		{
////			robot.gimbal_task_p->gimbal_mode = GIMBAL_AUTO_AIMING_MODE;
//		}    
  }
	
  robot.gimbal_task_p->gimbal_pitch_vel = -robot.rc_protocol.getRCData().ch3 / 400.0f;//(-6.6,+6.6)
  robot.gimbal_task_p->gimbal_yaw_vel = -robot.rc_protocol.getRCData().ch2 / 400.0f;//(-6.6,+6.6)
  
	//滚轮控制发射
//  robot.ammo_task_p->shoot_freq = robot.rc_protocol.getRCData().wheel / 660.0f * 10.0f;//(-10,+10)
  if(robot.rc_protocol.getRCData().wheel > 500)//滚轮退弹
	{
		robot.ammo_task_p->block_flag = 1;
	}	
	else if(robot.rc_protocol.getRCData().sw_right == SWITCH_UP)
	{
		robot.ammo_task_p->shoot_flag = 1;
		robot.ammo_task_p->moving_flag = 1;
	}			
	else {robot.ammo_task_p->shoot_flag = 0;robot.ammo_task_p->one_flag = 0;robot.ammo_task_p->block_flag = 0;}
	
	//底盘速度  二次函数缓起 //最大值 ：
    
  robot.chassis_task_p->vx =  float(robot.rc_protocol.getRCData().ch1)/300.0f;


  robot.chassis_task_p->vy = -float(robot.rc_protocol.getRCData().ch0)/ 300.0f; //键鼠控制缓停，遥控器控制缓起缓停

	/* 遥控器控制 End*/
	
 }

	

}

void RemoteControlTask::uninit(void)
{

}
