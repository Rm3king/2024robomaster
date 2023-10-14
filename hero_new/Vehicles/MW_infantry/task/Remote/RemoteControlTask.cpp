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
** �� �� ���� isBeyondDeadZone()
** ����˵���� �ж��Ƿ�Խ������
**---------------------------------------------------------------------
** ��������� rc_data
** ���ز����� ��
***********************************************************************/
bool isBeyondDeadZone(int16_t rc_data)
{
  if(rc_data >= JOYSTICK_DEAD_ZONE || rc_data <= -JOYSTICK_DEAD_ZONE)
    return true;
  else
    return false;
}

/***********************************************************************
** �� �� ���� RemoteControlTask::init()
** ����˵���� �ж��Ƿ���ɳ�ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void RemoteControlTask::init(void)
{
  inited = true;
}

/***********************************************************************
** �� �� ���� RemoteControlTask::update()
** ����˵���� ������յ���rc_data[]�е����ݣ����ң��������
**---------------------------------------------------------------------
** ��������� ��������
** ���ز����� ��
***********************************************************************/
void RemoteControlTask::update(timeus_t dT_us)
{	
		/* ������� */
	//�����������ʱ����ң�����󿪹غ��ҿ��ض��������·�
	if(robot.rc_protocol.getRCData().sw_right == SWITCH_DWN && robot.rc_protocol.getRCData().sw_left == SWITCH_DWN )
{
	
//	        //E��  ��������ģʽ��
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
//		if(robot.rc_protocol.getRCData().keyboard.key_bit.E == 0 && time_cnt < 21 && autoaim_lock ==  1) //0.2������ֶ��л�һ��
//		{
//			time_cnt ++;
//		}		
//			if(time_cnt == 21) //0.2������ֶ��л�һ��
//		{
//			time_cnt = 0;
//			autoaim_lock = 0;
//		}	
	
        if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0){
		if(robot.rc_protocol.getRCData().keyboard.key_bit.Z)
        {
//			   robot.chassis_task_p->chassis_mode = Chassis_OFF;//����ʧ��
//            robot.gimbal_task_p->gimbal_mode = GIMBAL_OFF;//��̨ʧ��
//            robot.ammo_task_p->booster_mode = BOOSTER_OFF;//�������ʧ��
					robot.gimbal_task_p->gimbal_mode = GIMBAL_HANGING_LOCKALL_MODE;
					robot.chassis_task_p->chassis_mode=Chassis_45to_enemy;
		      robot.chassis_task_p->set_Chassis_45toenemy_flag();
					
//            robot.ammo_task_p->set_Booster_off_flag();
        }
				//robot.gimbal_task_p->gimbal_mode = GIMBAL_RC_MODE;
		//�������̸�����̨ģʽ //����C�������̸�����̨
		if(robot.rc_protocol.getRCData().keyboard.key_bit.V)
		{
            robot.gimbal_task_p->gimbal_mode = GIMBAL_RC_MODE;//��̨��������
			robot.chassis_task_p->chassis_mode = Chassis_FOLLOW;//���̸�����̨
			robot.chassis_task_p->clear_Chassis_TOP_flag();
			robot.chassis_task_p->clear_Chassis_45toenemy_flag();
		}
        //������̨����ģʽ
//        if(robot.rc_protocol.getRCData().keyboard.key_bit.C){
//            robot.gimbal_task_p->gimbal_mode = GIMBAL_RC_MODE;//��̨��������
//            robot.chassis_task_p->chassis_mode = Chassis_FREE;//���̸�����̨
//			   robot.chassis_task_p->clear_Chassis_TOP_flag();
//        }
		//����С����ģʽ,����G
		if(robot.rc_protocol.getRCData().keyboard.key_bit.G)
		{
			robot.chassis_task_p->chassis_mode = Chassis_TOP;
			robot.chassis_task_p->set_Chassis_TOP_flag();
		}
		
		//������ֹͣĦ����,����Q������Ctrl+Q�ر�
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
		//���� �˵����������Ż�е��λ
		if(robot.rc_protocol.getRCData().keyboard.key_bit.R)
		{
            robot.ammo_task_p->block_flag = 1;
			robot.ammo_task_p->shoot_freq = -1.0f;
				robot.ammo_task_p->moving_flag = 1;
			double_press_flag = 1;
		}
		else {robot.ammo_task_p->block_flag = 0;robot.ammo_task_p->block_one_flag = 0;}
    if(robot.rc_protocol.getRCData().mouse.press_l)//����������
            {
					 
                robot.ammo_task_p->shoot_flag = 1; //�����־λ
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
        //���ǹ����ȴ
//		  if(robot.rc_protocol.getRCData().keyboard.key_bit.B)
//		  {robot.ammo_task_p->shoot_b_flag = 1;} //��B֮�� �������Ʊ�־λ��������
//		  
		if(robot.getRefereeSystemTaskPointer()->getRobotRefereeStatus_t().game_robot_status.shooter_id1_42mm_cooling_limit-robot.getRefereeSystemTaskPointer()->getRobotRefereeStatus_t().power_heat_data.shooter_id1_42mm_cooling_heat-one_ball_heat<0)
			 {
					robot.ammo_task_p->shoot_flag = 0; //������־λ��0 �޷�����
			 }
//        //ˢ��UI
//        if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0){
		if(robot.rc_protocol.getRCData().keyboard.key_bit.X)
		{
			robot.referee_system_task_p->set_num_0();
			robot.referee_system_task_p->set_t_0();
		}	
//    }
        //E��  ��������ģʽ
        if(robot.rc_protocol.getRCData().keyboard.key_bit.E)
		{
            robot.gimbal_task_p->gimbal_mode = GIMABAL_HANGING_MODE;
						robot.chassis_task_p->chassis_mode=Chassis_45to_enemy;
		        robot.chassis_task_p->set_Chassis_45toenemy_flag();
		}	
      		//����Ҽ����� ��ʼ����
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
    //������  Ctrl+z�����ٶ�����
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

	/* ң�������� Begin
	 ң�����󿪹������·ֱ����gimbal(��)��ammo(��)��chassis(��)���ҿ��ظ������ÿ��ģ��ľ���ģʽ */
	/* ң�������� Begin
	 ң�����󿪹������·ֱ����gimbal(��)��ammo(��)��chassis(��)���ҿ��ظ������ÿ��ģ��ľ���ģʽ */
	else
 {
  if(robot.rc_protocol.getRCData().sw_left == SWITCH_UP)//����
  {
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_UP && robot.rc_protocol.getRCData().wheel<-500)
		{
			robot.gimbal_task_p->gimbal_mode = GIMBAL_OFF;//��������+����=��̨ʧ��
		}
		
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_MID && robot.rc_protocol.getRCData().wheel<-500)
		{
			robot.gimbal_task_p->gimbal_mode = GIMBAL_RC_MODE;//��������+����=��̨��������
		}
		
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_DWN && robot.rc_protocol.getRCData().wheel<-500)
		{
			//robot.gimbal_task_p->gimbal_mode = GIMBAL_AUTO_AIMING_MODE;//��������+����=��̨����
			robot.gimbal_task_p->gimbal_mode = GIMBAL_AUTO_AIMING_MODE;
		}
  }
	
  else if(robot.rc_protocol.getRCData().sw_left == SWITCH_MID)//����
  {
    if(robot.rc_protocol.getRCData().sw_right == SWITCH_DWN && robot.rc_protocol.getRCData().wheel<-500)
		{
			robot.ammo_task_p->booster_mode = BOOSTER_OFF;//��������+����=�������ʧ��
		}
		
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_MID && robot.rc_protocol.getRCData().wheel<-500)
		{
            robot.ammo_task_p->booster_mode = TRIGGER_AUTO;//����ģʽ   Ӣ�� ֻ���õ���
		}
  }
	
  else if(robot.rc_protocol.getRCData().sw_left == SWITCH_DWN)//����
  {
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_UP && robot.rc_protocol.getRCData().wheel<-500)
		{
			robot.chassis_task_p->chassis_mode = Chassis_OFF;//��������+����=����ʧ��
		}
		
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_MID && robot.rc_protocol.getRCData().wheel<-500)
		{
			robot.chassis_task_p->chassis_mode = Chassis_FOLLOW;//��������+����=���̸�����̨ģʽ
		}
		
//		if(robot.rc_protocol.getRCData().sw_right == SWITCH_DWN && robot.rc_protocol.getRCData().wheel<-500)
//		{
////			robot.gimbal_task_p->gimbal_mode = GIMBAL_AUTO_AIMING_MODE;
//		}    
  }
	
  robot.gimbal_task_p->gimbal_pitch_vel = -robot.rc_protocol.getRCData().ch3 / 400.0f;//(-6.6,+6.6)
  robot.gimbal_task_p->gimbal_yaw_vel = -robot.rc_protocol.getRCData().ch2 / 400.0f;//(-6.6,+6.6)
  
	//���ֿ��Ʒ���
//  robot.ammo_task_p->shoot_freq = robot.rc_protocol.getRCData().wheel / 660.0f * 10.0f;//(-10,+10)
  if(robot.rc_protocol.getRCData().wheel > 500)//�����˵�
	{
		robot.ammo_task_p->block_flag = 1;
	}	
	else if(robot.rc_protocol.getRCData().sw_right == SWITCH_UP)
	{
		robot.ammo_task_p->shoot_flag = 1;
		robot.ammo_task_p->moving_flag = 1;
	}			
	else {robot.ammo_task_p->shoot_flag = 0;robot.ammo_task_p->one_flag = 0;robot.ammo_task_p->block_flag = 0;}
	
	//�����ٶ�  ���κ������� //���ֵ ��
    
  robot.chassis_task_p->vx =  float(robot.rc_protocol.getRCData().ch1)/300.0f;


  robot.chassis_task_p->vy = -float(robot.rc_protocol.getRCData().ch0)/ 300.0f; //������ƻ�ͣ��ң�������ƻ���ͣ

	/* ң�������� End*/
	
 }

	

}

void RemoteControlTask::uninit(void)
{

}
