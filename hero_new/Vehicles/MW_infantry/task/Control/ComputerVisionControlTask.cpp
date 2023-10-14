#include "/Control/ComputerVisionControlTask.h"
#include "/Referee/RefereeSystemTask.h"
#include "/Robot/Robot.h"
#include "usart.h"



uint8_t ref_lock = 0;
//MINIPC_RX_data_t mini_pc_data;
/***********************************************************************
** 函 数 名： ComputerVisionControlTask::ComputerVisionControlTask
** 函数说明： ComputerVisionControlTask构造函数
**---------------------------------------------------------------------
** 输入参数： 
** 返回参数： 无
***********************************************************************/
ComputerVisionControlTask::ComputerVisionControlTask(
  Robot &robot0,
  timeus_t interval_tick_us0
) : Task_Base(robot0)
{
	index = 0;
  this->interval_tick_us = interval_tick_us0;
  imu_sync_data_task = new IMUDataSyncTask(robot0);//显式实例化
	ref_sync_data_task = new RefereeDataSyncTask(robot0);
	lock_sync_data_task = new LockDataSyncTask(robot0);
  robot0.scheduler.registerTask(imu_sync_data_task);//注册任务
	robot0.scheduler.registerTask(ref_sync_data_task);
	robot0.scheduler.registerTask(lock_sync_data_task);
}

/***********************************************************************
** 函 数 名： ComputerVisionControlTask::init
** 函数说明： 
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void ComputerVisionControlTask::init()
{
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR); //使能DMA串口接收
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  //使能空闲中断

  hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR); //DMA传输外设地址
  hdma_usart1_rx.Instance->M0AR = (uint32_t)(rec_buf); //DMA传输存储器地址

  __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, CV_BUF_LEN); //DMA传输数据的个数(循环模式)
  __HAL_DMA_ENABLE(&hdma_usart1_rx); //串口DMA使能
  inited = true;
	send_sta = 0;
	
//	subscriber.subscribe(0x01, &chassis_vel_cmd);
//	subscriber.subscribe(0x02, &chassis_vel_yaw_rate_cmd);
	subscriber.subscribe(0x04, &gimbal_constant_aim_shoot_cmd);
}

/***********************************************************************
** 函 数 名： ComputerVisionControlTask::Transmit_DMA
** 函数说明： 
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void ComputerVisionControlTask::Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData,uint32_t Size)
{
	robot.scheduler.startAsync(transmit_dma_func.setData(huart, pData, Size), Size *8.0f / 5000 * 1000);
}

/***********************************************************************
** 函 数 名： ComputerVisionControlTask::pushToBuffer
** 函数说明： 将UART1 DMA数据从缓存中push到rec_fifo_buf[]数组中
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void ComputerVisionControlTask::pushToBuffer(uint16_t start_pos, uint16_t end_pos)
{
	uint16_t size;

  if (end_pos >= start_pos)
  {
    size = end_pos - start_pos;
  }
  else
  {
    size = CV_FIFO_BUF_LEN - (start_pos - end_pos);
  }

  if ((size > 0) && (fifo_count < CV_FIFO_BUF_NUM))
  {
    //
    rec_fifo_buf[fifo_tail_pos][0] = size >> 8;
    rec_fifo_buf[fifo_tail_pos][1] = size;

    if (end_pos > start_pos)
    {
      memcpy(rec_fifo_buf[fifo_tail_pos] + 2, rec_buf + start_pos, size);
    }
    else
    {
      memcpy(rec_fifo_buf[fifo_tail_pos] + 2, rec_buf + start_pos, CV_FIFO_BUF_LEN - start_pos);
      memcpy(rec_fifo_buf[fifo_tail_pos] + 2 + (CV_FIFO_BUF_LEN - start_pos), rec_buf, end_pos);
    }


    fifo_tail_pos = (fifo_tail_pos + 1) % CV_FIFO_BUF_NUM;
    fifo_count++;
  }
	//USART1_DMA_Debug_Printf("0x%x\n", buf[0]);
}
//
/***********************************************************************
** 函 数 名： ComputerVisionControlTask::parseData
** 函数说明： 分析数据
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void ComputerVisionControlTask::parseData(void)
{
	int16_t remaining_n;

  uint8_t *pbuf;
  uint8_t *ptr; //用于float和uint64_t类型数据转换

  uint16_t data_len;
  uint16_t cmd;

  if (fifo_count > 0)
  {
    //
    remaining_n = rec_fifo_buf[fifo_head_pos][0] * 256 + rec_fifo_buf[fifo_head_pos][1];
    pbuf = rec_fifo_buf[fifo_head_pos] + 2;
		
		while (1)
    {
      //
      if (remaining_n >= 1)
			{
				processByte(pbuf[0]);
			}
			else //check frame_header & length error
      {
        break;
      }
			
			remaining_n = remaining_n - 1;//(data_len + 9);
      pbuf = pbuf + 1;//(data_len + 9);

      if (remaining_n <= 0)
      {
        break;
      }
		}
		
		fifo_head_pos = (fifo_head_pos + 1) % CV_FIFO_BUF_NUM;
    fifo_count--;
	}
}

/***********************************************************************
** 函 数 名： ComputerVisionControlTask::processByte
** 函数说明： 解析数据
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void ComputerVisionControlTask::processByte(uint8_t data)
{
	subscriber.processByte(data);
	return;
//	rec_buffer[index] = data;
//	index = index + 1;
//	if(index >= 2 && rec_buffer[index - 2] == 0xAA && data == 0xEE)
//  {
//      index = 2;
//      rec_buffer[0] = 0xAA;
//      rec_buffer[1] = 0xEE;
//  }
//	
//	 // vel cmd
//	if(index == 13 + 6 && rec_buffer[2] == 0x01)
//	{
//		uint8_t sum = 0, sum_sum = 0;
//    for(int i = 0; i < 13 + 4; i ++)
//    {
//       sum += rec_buffer[i];
//       sum_sum += sum;
//    }
//		if(chassis_vel_cmd.unpack(rec_buffer))
//		{
//			index = 0;
//			chassisVelCommand(chassis_vel_cmd.x, chassis_vel_cmd.y, chassis_vel_cmd.frame);
//		}
//		else
//		{
//			index = 0;
//		}
//	}
//	//else if(index == )
//	else if(index >= 13 + 6)
//	{
//		index = 0;
//	}
}

///***********************************************************************
//** 函 数 名： ComputerVisionControlTask::chassisVelCommand
//** 函数说明： 
//**---------------------------------------------------------------------
//** 输入参数： 无
//** 返回参数： 无
//***********************************************************************/
//void ComputerVisionControlTask::chassisVelCommand(float vx, float vy, uint8_t frame)
//{
//	if(frame == 0) // 云台IMU世界系
//	{
//		//robot.chassis_task_p->setVelWorldFromExt(vx, vy);
//	}
//	else // 云台IMU机体系
//	{
//		//TO DO
//	}
//}

///***********************************************************************
//** 函 数 名： ComputerVisionControlTask::chassisVelYawRateCommand
//** 函数说明： 
//**---------------------------------------------------------------------
//** 输入参数： 无
//** 返回参数： 无
//***********************************************************************/
//void ComputerVisionControlTask::chassisVelYawRateCommand(float vx, float vy, float yaw_rate, uint8_t frame)
//{
//	if(frame == 0) // 云台IMU世界系
//	{
//		//robot.chassis_task_p->setVelWorldYawRateFromExt(vx, vy, yaw_rate);
//	}
//	else // 云台IMU机体系
//	{
//		//TO DO
//	}
//}

///***********************************************************************
//** 函 数 名： ComputerVisionControlTask::gimbalAimCommand()
//** 函数说明： 获取上位机云台命令信息
//**---------------------------------------------------------------------
//** 输入参数： 无
//** 返回参数： 无
//***********************************************************************/
//void ComputerVisionControlTask::gimbalAimCommand(float x, float y, float z, float freq, uint8_t frame)
//{
//	if(frame == 0) // 云台IMU世界系
//	{
//		robot.getGimbalControlTaskPointer()->setGimbalExpVecWorldFromExt(Vector3f(x, y, z));
//	}
//	else // 云台IMU机体系
//	{
//		//TO DO
//	}
//}

/***********************************************************************
** 函 数 名： ComputerVisionControlTask::update
** 函数说明： 更新上位机数据，步兵不需要
**---------------------------------------------------------------------
** 输入参数： 执行周期
** 返回参数： 无
***********************************************************************/
void ComputerVisionControlTask::update(timeus_t dT_us)
{
	parseData();//解析数据
//	gimbal_constant_aim_shoot_cmd.z = update_z(gimbal_constant_aim_shoot_cmd.x,gimbal_constant_aim_shoot_cmd.y,gimbal_constant_aim_shoot_cmd.z,robot.referee_system_task_p->get_filter_speed());
	return;
//	referee_sta = robot.getRefereeSystemTaskPointer()->getRobotRefereeStatus_t();
//	
//	switch(send_sta)
//	{
//		// 比赛状态 0x10
//		case(0):
//		{
//			if(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_BUSY && referee_sta.game_status.timestamp > referee_sta_last.game_status.timestamp)
//			{
//				data_buf[0] = 0xAA; // 帧头
//				data_buf[1] = 0xFF; // 目标地址
//				data_buf[2] = 0x10; // 功能码
//				data_buf[3] = 4;
//				data_buf[4] = referee_sta.game_status.game_type;
//				data_buf[5] = referee_sta.game_status.game_progress;
//				data_buf[6] = referee_sta.game_status.stage_remain_time;
//				data_buf[7] = referee_sta.game_status.stage_remain_time >> 8;
//				
//				data_buf[data_buf[3] + 4] = 0;
//				data_buf[data_buf[3] + 5] = 0;
//				for(int i = 0; i < (data_buf[3] + 4); i++)
//				{
//					data_buf[data_buf[3] + 4] += data_buf[i];		//从帧头开始，对每一字节进行求和，直到DATA区结束
//					data_buf[data_buf[3] + 5] += data_buf[data_buf[3] + 4];		//每一字节的求和操作，进行一次sumcheck的累加
//				}
//				
//				HAL_UART_Transmit_DMA(&huart1, data_buf, data_buf[3] + 6);
//			}
//			break;
//		}
//		
//		// 机器人血量数据 0x11
//		case(1):
//		{
//			if(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_BUSY && referee_sta.game_robot_HP.timestamp > referee_sta_last.game_robot_HP.timestamp)
//			{
//				data_buf[0] = 0xAA; // 帧头
//				data_buf[1] = 0xFF; // 目标地址
//				data_buf[2] = 0x11; // 功能码
//				data_buf[3] = 32;
//				data_buf[4] = referee_sta.game_robot_HP.red_1_robot_HP;
//				data_buf[5] = referee_sta.game_robot_HP.red_1_robot_HP >> 8;
//				data_buf[6] = referee_sta.game_robot_HP.red_2_robot_HP;
//				data_buf[7] = referee_sta.game_robot_HP.red_2_robot_HP >> 8;
//				data_buf[8] = referee_sta.game_robot_HP.red_3_robot_HP;
//				data_buf[9] = referee_sta.game_robot_HP.red_3_robot_HP >> 8;
//				data_buf[10] = referee_sta.game_robot_HP.red_4_robot_HP;
//				data_buf[11] = referee_sta.game_robot_HP.red_4_robot_HP >> 8;
//				data_buf[12] = referee_sta.game_robot_HP.red_5_robot_HP;
//				data_buf[13] = referee_sta.game_robot_HP.red_5_robot_HP >> 8;
//				data_buf[14] = referee_sta.game_robot_HP.red_7_robot_HP;
//				data_buf[15] = referee_sta.game_robot_HP.red_7_robot_HP >> 8;
//				data_buf[16] = referee_sta.game_robot_HP.red_outpost_HP;
//				data_buf[17] = referee_sta.game_robot_HP.red_outpost_HP >> 8;
//				data_buf[18] = referee_sta.game_robot_HP.red_base_HP;
//				data_buf[19] = referee_sta.game_robot_HP.red_base_HP >> 8;
//				data_buf[20] = referee_sta.game_robot_HP.blue_1_robot_HP;
//				data_buf[21] = referee_sta.game_robot_HP.blue_1_robot_HP >> 8;
//				data_buf[22] = referee_sta.game_robot_HP.blue_2_robot_HP;
//				data_buf[23] = referee_sta.game_robot_HP.blue_2_robot_HP >> 8;
//				data_buf[24] = referee_sta.game_robot_HP.blue_3_robot_HP;
//				data_buf[25] = referee_sta.game_robot_HP.blue_3_robot_HP >> 8;
//				data_buf[26] = referee_sta.game_robot_HP.blue_4_robot_HP;
//				data_buf[27] = referee_sta.game_robot_HP.blue_4_robot_HP >> 8;
//				data_buf[28] = referee_sta.game_robot_HP.blue_5_robot_HP;
//				data_buf[29] = referee_sta.game_robot_HP.blue_5_robot_HP >> 8;
//				data_buf[30] = referee_sta.game_robot_HP.blue_7_robot_HP;
//				data_buf[31] = referee_sta.game_robot_HP.blue_7_robot_HP >> 8;
//				data_buf[32] = referee_sta.game_robot_HP.blue_outpost_HP;
//				data_buf[33] = referee_sta.game_robot_HP.blue_outpost_HP >> 8;
//				data_buf[34] = referee_sta.game_robot_HP.blue_base_HP;
//				data_buf[35] = referee_sta.game_robot_HP.blue_base_HP >> 8;
//				
//				data_buf[data_buf[3] + 4] = 0;
//				data_buf[data_buf[3] + 5] = 0;
//				for(int i = 0; i < (data_buf[3] + 4); i++)
//				{
//					data_buf[data_buf[3] + 4] += data_buf[i];		//从帧头开始，对每一字节进行求和，直到DATA区结束
//					data_buf[data_buf[3] + 5] += data_buf[data_buf[3] + 4];		//每一字节的求和操作，进行一次sumcheck的累加
//				}
//				
//				HAL_UART_Transmit_DMA(&huart1, data_buf, data_buf[3] + 6);
//				break;
//			}
//		}
//		// 机器人状态数据 0x12
//		case(2):
//		{
//			if(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_BUSY && referee_sta.game_robot_status.timestamp > referee_sta_last.game_robot_HP.timestamp)
//			{
//				data_buf[0] = 0xAA; // 帧头
//				data_buf[1] = 0xFF; // 目标地址
//				data_buf[2] = 0x12; // 功能码
//				data_buf[3] = 29;
//				data_buf[4] = referee_sta.game_robot_status.robot_id;
//				data_buf[5] = referee_sta.game_robot_status.robot_level;
//				data_buf[6] = referee_sta.game_robot_status.remain_HP;
//				data_buf[7] = referee_sta.game_robot_status.remain_HP >> 8;
//				data_buf[8] = referee_sta.game_robot_status.max_HP;
//				data_buf[9] = referee_sta.game_robot_status.max_HP >> 8;
//				data_buf[10] = referee_sta.game_robot_status.shooter_id1_17mm_cooling_rate;
//				data_buf[11] = referee_sta.game_robot_status.shooter_id1_17mm_cooling_rate >> 8;
//				data_buf[12] = referee_sta.game_robot_status.shooter_id1_17mm_cooling_limit;
//				data_buf[13] = referee_sta.game_robot_status.shooter_id1_17mm_cooling_limit >> 8;
//				data_buf[14] = referee_sta.game_robot_status.shooter_id1_17mm_speed_limit;
//				data_buf[15] = referee_sta.game_robot_status.shooter_id1_17mm_speed_limit >> 8;
//				data_buf[16] = referee_sta.game_robot_status.shooter_id2_17mm_cooling_rate;
//				data_buf[17] = referee_sta.game_robot_status.shooter_id2_17mm_cooling_rate >> 8;
//				data_buf[18] = referee_sta.game_robot_status.shooter_id2_17mm_cooling_limit;
//				data_buf[19] = referee_sta.game_robot_status.shooter_id2_17mm_cooling_limit >> 8;
//				data_buf[20] = referee_sta.game_robot_status.shooter_id2_17mm_speed_limit;
//				data_buf[21] = referee_sta.game_robot_status.shooter_id2_17mm_speed_limit >> 8;
//				data_buf[22] = referee_sta.game_robot_status.shooter_id1_42mm_cooling_rate;
//				data_buf[23] = referee_sta.game_robot_status.shooter_id1_42mm_cooling_rate >> 8;
//				data_buf[24] = referee_sta.game_robot_status.shooter_id1_42mm_cooling_limit;
//				data_buf[25] = referee_sta.game_robot_status.shooter_id1_42mm_cooling_limit >> 8;
//				data_buf[26] = referee_sta.game_robot_status.shooter_id1_42mm_speed_limit;
//				data_buf[27] = referee_sta.game_robot_status.shooter_id1_42mm_speed_limit >> 8;
//				data_buf[28] = referee_sta.game_robot_status.chassis_power_limit;
//				data_buf[29] = referee_sta.game_robot_status.chassis_power_limit >> 8;
//				data_buf[30] = referee_sta.game_robot_status.mains_power_gimbal_output;
//				data_buf[31] = referee_sta.game_robot_status.mains_power_chassis_output;
//				data_buf[32] = referee_sta.game_robot_status.mains_power_shooter_output;
//				
//				data_buf[data_buf[3] + 4] = 0;
//				data_buf[data_buf[3] + 5] = 0;
//				for(int i = 0; i < (data_buf[3] + 4); i++)
//				{
//					data_buf[data_buf[3] + 4] += data_buf[i];		//从帧头开始，对每一字节进行求和，直到DATA区结束
//					data_buf[data_buf[3] + 5] += data_buf[data_buf[3] + 4];		//每一字节的求和操作，进行一次sumcheck的累加
//				}
//				
//				HAL_UART_Transmit_DMA(&huart1, data_buf, data_buf[3] + 6);
//				break;
//			}
//		}
//		// 实时功率热量数据 0x13
//		case(3):
//		{
//			if(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_BUSY && referee_sta.power_heat_data.timestamp > referee_sta_last.power_heat_data.timestamp)
//			{
//				data_buf[0] = 0xAA; // 帧头
//				data_buf[1] = 0xFF; // 目标地址
//				data_buf[2] = 0x13; // 功能码
//				data_buf[3] = 14;
//				data_buf[4] = referee_sta.power_heat_data.chassis_volt;
//				data_buf[5] = referee_sta.power_heat_data.chassis_volt >> 8;
//				data_buf[6] = referee_sta.power_heat_data.chassis_current;
//				data_buf[7] = referee_sta.power_heat_data.chassis_current >> 8;
//				data_buf[8] = ((uint8_t *)(&referee_sta.power_heat_data.chassis_power))[0];
//				data_buf[9] = ((uint8_t *)(&referee_sta.power_heat_data.chassis_power))[1];
//				data_buf[10] = ((uint8_t *)(&referee_sta.power_heat_data.chassis_power))[2];
//				data_buf[11] = ((uint8_t *)(&referee_sta.power_heat_data.chassis_power))[3];
//				data_buf[12] = referee_sta.power_heat_data.shooter_id1_17mm_cooling_heat;
//				data_buf[13] = referee_sta.power_heat_data.shooter_id1_17mm_cooling_heat >> 8;
//				data_buf[14] = referee_sta.power_heat_data.shooter_id2_17mm_cooling_heat;
//				data_buf[15] = referee_sta.power_heat_data.shooter_id2_17mm_cooling_heat >> 8;
//				data_buf[16] = referee_sta.power_heat_data.shooter_id1_42mm_cooling_heat;
//				data_buf[17] = referee_sta.power_heat_data.shooter_id1_42mm_cooling_heat >> 8;
//				
//				data_buf[data_buf[3] + 4] = 0;
//				data_buf[data_buf[3] + 5] = 0;
//				for(int i = 0; i < (data_buf[3] + 4); i++)
//				{
//					data_buf[data_buf[3] + 4] += data_buf[i];		//从帧头开始，对每一字节进行求和，直到DATA区结束
//					data_buf[data_buf[3] + 5] += data_buf[data_buf[3] + 4];		//每一字节的求和操作，进行一次sumcheck的累加
//				}
//				
//				HAL_UART_Transmit_DMA(&huart1, data_buf, data_buf[3] + 6);
//				break;
//			}
//		}			
//		// 机器人增益数据 0x14
//		case(4):
//		{
//			if(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_BUSY && referee_sta.buff.timestamp > referee_sta_last.buff.timestamp)
//			{
//				data_buf[0] = 0xAA; // 帧头
//				data_buf[1] = 0xFF; // 目标地址
//				data_buf[2] = 0x14; // 功能码
//				data_buf[3] = 1;
//				data_buf[4] = referee_sta.buff.power_rune_buff;
//				
//				data_buf[data_buf[3] + 4] = 0;
//				data_buf[data_buf[3] + 5] = 0;
//				for(int i = 0; i < (data_buf[3] + 4); i++)
//				{
//					data_buf[data_buf[3] + 4] += data_buf[i];		//从帧头开始，对每一字节进行求和，直到DATA区结束
//					data_buf[data_buf[3] + 5] += data_buf[data_buf[3] + 4];		//每一字节的求和操作，进行一次sumcheck的累加
//				}
//				
//				HAL_UART_Transmit_DMA(&huart1, data_buf, data_buf[3] + 6);
//				break;
//			}
//		}
//		// 机器人增益数据 0x15
//		case(5):
//		{
//			if(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_BUSY && referee_sta.robot_hurt.timestamp > referee_sta_last.robot_hurt.timestamp)
//			{
//				data_buf[0] = 0xAA; // 帧头
//				data_buf[1] = 0xFF; // 目标地址
//				data_buf[2] = 0x15; // 功能码
//				data_buf[3] = 2;
//				data_buf[4] = referee_sta.robot_hurt.armor_id;
//				data_buf[5] = referee_sta.robot_hurt.hurt_type;
//				
//				data_buf[data_buf[3] + 4] = 0;
//				data_buf[data_buf[3] + 5] = 0;
//				for(int i = 0; i < (data_buf[3] + 4); i++)
//				{
//					data_buf[data_buf[3] + 4] += data_buf[i];		//从帧头开始，对每一字节进行求和，直到DATA区结束
//					data_buf[data_buf[3] + 5] += data_buf[data_buf[3] + 4];		//每一字节的求和操作，进行一次sumcheck的累加
//				}
//				
//				HAL_UART_Transmit_DMA(&huart1, data_buf, data_buf[3] + 6);
//				break;
//			}
//		}
//		// 伤害状态数据 0x16
//		case(6):
//		{
//			if(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_BUSY && referee_sta.shoot_data.timestamp > referee_sta_last.shoot_data.timestamp)
//			{
//				data_buf[0] = 0xAA; // 帧头
//				data_buf[1] = 0xFF; // 目标地址
//				data_buf[2] = 0x16; // 功能码
//				data_buf[3] = 7;
//				data_buf[4] = referee_sta.shoot_data.bullet_type;
//				data_buf[5] = referee_sta.shoot_data.shooter_id;
//				data_buf[6] = referee_sta.shoot_data.bullet_freq;
//				
//				data_buf[7] = ((uint8_t *)&referee_sta.shoot_data.bullet_speed)[0];
//				data_buf[8] = ((uint8_t *)&referee_sta.shoot_data.bullet_speed)[1];
//				data_buf[9] = ((uint8_t *)&referee_sta.shoot_data.bullet_speed)[2];
//				data_buf[10] = ((uint8_t *)&referee_sta.shoot_data.bullet_speed)[3];
//				
//				data_buf[data_buf[3] + 4] = 0;
//				data_buf[data_buf[3] + 5] = 0;
//				for(int i = 0; i < (data_buf[3] + 4); i++)
//				{
//					data_buf[data_buf[3] + 4] += data_buf[i];		//从帧头开始，对每一字节进行求和，直到DATA区结束
//					data_buf[data_buf[3] + 5] += data_buf[data_buf[3] + 4];		//每一字节的求和操作，进行一次sumcheck的累加
//				}
//				
//				HAL_UART_Transmit_DMA(&huart1, data_buf, data_buf[3] + 6);
//				break;
//			}
//		}
//		// 子弹剩余发射数 0x17
//		case(7):
//		{
//			if(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_BUSY && referee_sta.bullet_remaining.timestamp > referee_sta_last.bullet_remaining.timestamp)
//			{
//				data_buf[0] = 0xAA; // 帧头
//				data_buf[1] = 0xFF; // 目标地址
//				data_buf[2] = 0x17; // 功能码
//				data_buf[3] = 6;
//				data_buf[4] = referee_sta.bullet_remaining.bullet_remaining_num_17mm;
//				data_buf[5] = referee_sta.bullet_remaining.bullet_remaining_num_17mm >> 8;
//				data_buf[6] = referee_sta.bullet_remaining.bullet_remaining_num_42mm;
//				data_buf[7] = referee_sta.bullet_remaining.bullet_remaining_num_42mm >> 8;
//				data_buf[8] = referee_sta.bullet_remaining.coin_remaining_num;
//				data_buf[9] = referee_sta.bullet_remaining.coin_remaining_num >> 8;
//				
//				data_buf[data_buf[3] + 4] = 0;
//				data_buf[data_buf[3] + 5] = 0;
//				for(int i = 0; i < (data_buf[3] + 4); i++)
//				{
//					data_buf[data_buf[3] + 4] += data_buf[i];		//从帧头开始，对每一字节进行求和，直到DATA区结束
//					data_buf[data_buf[3] + 5] += data_buf[data_buf[3] + 4];		//每一字节的求和操作，进行一次sumcheck的累加
//				}
//				
//				HAL_UART_Transmit_DMA(&huart1, data_buf, data_buf[3] + 6);
//				break;
//			}
//		}
//		// 机器人RFID状态数据 0x18
//		case(8):
//		{
//			if(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_BUSY && referee_sta.rfid_status.timestamp > referee_sta_last.rfid_status.timestamp)
//			{
//				data_buf[0] = 0xAA; // 帧头
//				data_buf[1] = 0xFF; // 目标地址
//				data_buf[2] = 0x18; // 功能码
//				data_buf[3] = 1;
//				data_buf[4] = referee_sta.rfid_status.rfid_status;
//				
//				data_buf[data_buf[3] + 4] = 0;
//				data_buf[data_buf[3] + 5] = 0;
//				for(int i = 0; i < (data_buf[3] + 4); i++)
//				{
//					data_buf[data_buf[3] + 4] += data_buf[i];		//从帧头开始，对每一字节进行求和，直到DATA区结束
//					data_buf[data_buf[3] + 5] += data_buf[data_buf[3] + 4];		//每一字节的求和操作，进行一次sumcheck的累加
//				}
//				
//				HAL_UART_Transmit_DMA(&huart1, data_buf, data_buf[3] + 6);
//				break;
//			}
//		}
//		// 场地事件数据 0x19
//		case(9):
//		{
//			if(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_BUSY && referee_sta.event_data.timestamp > referee_sta_last.event_data.timestamp)
//			{
//				data_buf[0] = 0xAA; // 帧头
//				data_buf[1] = 0xFF; // 目标地址
//				data_buf[2] = 0x19; // 功能码
//				data_buf[3] = 4;
//				data_buf[4] = referee_sta.event_data.event_type;
//				data_buf[5] = referee_sta.event_data.event_type >> 8;
//				data_buf[6] = referee_sta.event_data.event_type >> 16;
//				data_buf[7] = referee_sta.event_data.event_type >> 24;
//				
//				data_buf[data_buf[3] + 4] = 0;
//				data_buf[data_buf[3] + 5] = 0;
//				for(int i = 0; i < (data_buf[3] + 4); i++)
//				{
//					data_buf[data_buf[3] + 4] += data_buf[i];		//从帧头开始，对每一字节进行求和，直到DATA区结束
//					data_buf[data_buf[3] + 5] += data_buf[data_buf[3] + 4];		//每一字节的求和操作，进行一次sumcheck的累加
//				}
//				
//				HAL_UART_Transmit_DMA(&huart1, data_buf, data_buf[3] + 6);
//				break;
//			}
//		}
//		// 裁判警告信息 0x20
//		case(10):
//		{
//			if(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_BUSY && referee_sta.referee_warning.timestamp > referee_sta_last.referee_warning.timestamp)
//			{
//				data_buf[0] = 0xAA; // 帧头
//				data_buf[1] = 0xFF; // 目标地址
//				data_buf[2] = 0x20; // 功能码
//				data_buf[3] = 2;
//				data_buf[5] = referee_sta.referee_warning.level;
//				data_buf[6] = referee_sta.referee_warning.foul_robot_id;
//				
//				data_buf[data_buf[3] + 4] = 0;
//				data_buf[data_buf[3] + 5] = 0;
//				for(int i = 0; i < (data_buf[3] + 4); i++)
//				{
//					data_buf[data_buf[3] + 4] += data_buf[i];		//从帧头开始，对每一字节进行求和，直到DATA区结束
//					data_buf[data_buf[3] + 5] += data_buf[data_buf[3] + 4];		//每一字节的求和操作，进行一次sumcheck的累加
//				}
//				
//				HAL_UART_Transmit_DMA(&huart1, data_buf, data_buf[3] + 6);
//				break;
//			}
//		}
//		default: break;
//	}
//	
//	if(send_sta ++ == 10) send_sta = 0;
//	referee_sta_last = referee_sta;
}


void ComputerVisionControlTask::uninit(void)
{

}

/***********************************************************************
** 函 数 名： IMUDataSyncTask::IMUDataSyncTask
** 函数说明： 加速度计、角速度计数据同步任务构造函数
**---------------------------------------------------------------------
** 输入参数： 
** 返回参数： 无
***********************************************************************/ 
IMUDataSyncTask::IMUDataSyncTask(
  Robot &robot0,
  timeus_t interval_tick_us0
) : Task_Base(robot0)
{
  this->interval_tick_us = interval_tick_us0;
}

/***********************************************************************
** 函 数 名： IMUDataSyncTask::init
** 函数说明： 
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/ 
void IMUDataSyncTask::init()
{

}

/***********************************************************************
** 函 数 名： IMUDataSyncTask::update
** 函数说明： 将角速度，加速度以及四元数数据发送给上位机
**---------------------------------------------------------------------
** 输入参数： 执行周期
** 返回参数： 无
***********************************************************************/ 
void IMUDataSyncTask::update(timeus_t dT_us)
{
  uint8_t imu_id = robot.getBMI088BackendId();

  // 24g
  int16_t * accel = ((InertialSensor_BMI088*)robot.getInertialSensorPointer().findBackendById(imu_id))->getAccelRawPointer();
  // 2000dps
  int16_t * gyro = ((InertialSensor_BMI088*)robot.getInertialSensorPointer().findBackendById(imu_id))->getGyroRawPointer();

  Quaternion q = robot.getAttitudeSolutionTaskPointer()->getIMUQuat();
	
	imu_data.w = q.w;
	imu_data.x = q.x;
	imu_data.y = q.y;
	imu_data.z = q.z;
	for(uint8_t i = 0; i < 3; i ++)
	{
		imu_data.accel[i] = accel[i];
		imu_data.gyro[i] = gyro[i];
	}
	
	publisher.publish(&imu_data);

//  int16_t quaternion[4];
//  quaternion[0] = q.w * 32760.0f;
//  quaternion[1] = q.x * 32760.0f;
//  quaternion[2] = q.y * 32760.0f;
//  quaternion[3] = q.z * 32760.0f;

//  data_buf[0] = 0xAA; // 帧头
//  data_buf[1] = 0xFF; // 目标地址
//  data_buf[2] = 0x01; // 功能码
//  data_buf[3] = 20; // 数据长度

//  data_buf[4]  = accel[0];
//  data_buf[5]  = accel[0] >> 8;
//  data_buf[6]  = accel[1];
//  data_buf[7]  = accel[1] >> 8;
//  data_buf[8]  = accel[2];
//  data_buf[9]  = accel[2] >> 8;

//  data_buf[10] = gyro[0];
//  data_buf[11] = gyro[0] >> 8;
//  data_buf[12] = gyro[1];
//  data_buf[13] = gyro[1] >> 8;
//  data_buf[14] = gyro[2];
//  data_buf[15] = gyro[2] >> 8;

//  data_buf[16] = quaternion[0];
//  data_buf[17] = quaternion[0] >> 8;
//  data_buf[18] = quaternion[1];
//  data_buf[19] = quaternion[1] >> 8;
//  data_buf[20] = quaternion[2];
//  data_buf[21] = quaternion[2] >> 8;
//  data_buf[22] = quaternion[3];
//  data_buf[23] = quaternion[3] >> 8;

//  data_buf[24] = 0;
//  data_buf[25] = 0;

//  for(int i = 0; i < (data_buf[3] + 4); i++)
//  {
//    data_buf[24] += data_buf[i];		//从帧头开始，对每一字节进行求和，直到DATA区结束
//    data_buf[25] += data_buf[24];		//每一字节的求和操作，进行一次sumcheck的累加
//  }

//  HAL_UART_Transmit_DMA(&huart1, data_buf, data_buf[3] + 6);
////  j++;
////  if(j > 100)
////    j = 0;
////  j = 10;
//  HAL_UART_Transmit_DMA(&huart1, j, 5);
}

void IMUDataSyncTask::uninit(void)
{

}
/***********************************************************************
** 函 数 名： IMUDataSyncTask::IMUDataSyncTask
** 函数说明： 加速度计、陀螺仪、角速度计数据同步任务构造函数
**---------------------------------------------------------------------
** 输入参数： 
** 返回参数： 无
***********************************************************************/ 
RefereeDataSyncTask::RefereeDataSyncTask(
  Robot &robot0,
  timeus_t interval_tick_us0
) : Task_Base(robot0)
{
  this->interval_tick_us = interval_tick_us0;
}

/***********************************************************************
** 函 数 名： IMUDataSyncTask::init
** 函数说明： 
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/ 
void RefereeDataSyncTask::init()
{

}
//void RefereeDataSyncTask::update(timeus_t dT_us)
//{
//	ref_lock = 1;
//	static uint64_t rec_cnt_last = 0;
//		reference_data_mini.game_state = 0;
//		reference_data_mini.bullet_speed_meas = robot.referee_system_task_p->get_filter_speed();
//		reference_data_mini.hurt_by_gimbal = 0;
//		reference_data_mini.remaining_bullet = 0;
//		reference_data_mini.remaining_time = 0;
//		reference_data_mini.self_color = robot.referee_system_task_p->get_robot_color();
//		reference_data_mini.self_health = 0;
//		reference_data_mini.shoot_heat = 0;
//		reference_data_mini.who_is_balance = robot.referee_system_task_p->get_who_is_balance();
//		publisher.publish(&reference_data_mini);
//	ref_lock = 0;
//}
void RefereeDataSyncTask::update(timeus_t dT_us)
{
  ref_lock = 1;

  RobotRefereeStatus_t ref_data = robot.getRefereeSystemTaskPointer()->getRobotRefereeStatus_t();
  static uint64_t who_is_balance = 0x00;
  static uint8_t my_color = 0x00;
  if(ref_data.game_robot_HP.recv_cnt > 0 && ref_data.game_status.stage_remain_time > 290)
  {
    if(ref_data.game_robot_status.robot_id > 100)
    {
      if(ref_data.game_robot_HP.red_3_robot_HP > 202)
      {
        who_is_balance = 3;
      }
      if(ref_data.game_robot_HP.red_4_robot_HP > 202)
      {
        who_is_balance = 4;
      }
      if(ref_data.game_robot_HP.red_5_robot_HP > 202)
      {
        who_is_balance = 5;
      }
    }
    else if(ref_data.game_robot_status.robot_id > 0)
    {
      if(ref_data.game_robot_HP.blue_3_robot_HP > 202)
      {
        who_is_balance = 3;
      }
      if(ref_data.game_robot_HP.blue_4_robot_HP > 202)
      {
        who_is_balance = 4;
      }
      if(ref_data.game_robot_HP.blue_5_robot_HP > 202)
      {
        who_is_balance = 5;
      }
    }
  }

  if(ref_data.game_robot_status.recv_cnt > 0)
  {
    if(ref_data.game_robot_status.robot_id > 100)
    {
      my_color = 2; // blue
    }
    else if(ref_data.game_robot_status.robot_id > 0)
    {
      my_color = 1; // red
    }
  }
  //if(mini.rec_cnt > rec_cnt_last)
  //{

  reference_data_mini.game_state = ref_data.game_status.game_progress;
  reference_data_mini.bullet_speed_meas = robot.referee_system_task_p->get_filter_speed();
  reference_data_mini.hurt_by_gimbal = 0;
  reference_data_mini.remaining_bullet = ref_data.bullet_remaining.bullet_remaining_num_42mm;
  reference_data_mini.remaining_time = ref_data.game_status.stage_remain_time;
  reference_data_mini.self_color = my_color;
  reference_data_mini.self_health = 0;
  reference_data_mini.shoot_heat = ref_data.power_heat_data.shooter_id1_42mm_cooling_heat;
  reference_data_mini.who_is_balance = who_is_balance;
  publisher.publish(&reference_data_mini);
  
  //}

//	for(int i = 0; i < 10; i ++)
//	{
//		if(robot_data[i].rec_cnt > robot_data[i].rec_cnt_last)
//		{
//			robot_around_information.robot_id = robot_data[i].id;
//			robot_around_information.remain_HP = robot_data[i].hp;
//			publisher.publish(&robot_around_information);
//		}
//
//		robot_data[i].rec_cnt_last = robot_data[i].rec_cnt;
//	}
//
//	rec_cnt_last = mini.rec_cnt;
  ref_lock = 0;
}
void RefereeDataSyncTask::uninit(void)
{

}

void LockDataSyncTask::init()
{

}
//void RefereeDataSyncTask::update(timeus_t dT_us)
//{
//	ref_lock = 1;
//	static uint64_t rec_cnt_last = 0;
//		reference_data_mini.game_state = 0;
//		reference_data_mini.bullet_speed_meas = robot.referee_system_task_p->get_filter_speed();
//		reference_data_mini.hurt_by_gimbal = 0;
//		reference_data_mini.remaining_bullet = 0;
//		reference_data_mini.remaining_time = 0;
//		reference_data_mini.self_color = robot.referee_system_task_p->get_robot_color();
//		reference_data_mini.self_health = 0;
//		reference_data_mini.shoot_heat = 0;
//		reference_data_mini.who_is_balance = robot.referee_system_task_p->get_who_is_balance();
//		publisher.publish(&reference_data_mini);
//	ref_lock = 0;
//}
void LockDataSyncTask::update(timeus_t dT_us)
{

  lock_target_suggest.lock_target = robot.getRemoteControlTask()->autoaim_flag;
  publisher.publish(&lock_target_suggest);
  
  //}

//	for(int i = 0; i < 10; i ++)
//	{
//		if(robot_data[i].rec_cnt > robot_data[i].rec_cnt_last)
//		{
//			robot_around_information.robot_id = robot_data[i].id;
//			robot_around_information.remain_HP = robot_data[i].hp;
//			publisher.publish(&robot_around_information);
//		}
//
//		robot_data[i].rec_cnt_last = robot_data[i].rec_cnt;
//	}
//
//	rec_cnt_last = mini.rec_cnt;
}
void LockDataSyncTask::uninit(void)
{

}

LockDataSyncTask::LockDataSyncTask(
  Robot &robot0,
  timeus_t interval_tick_us0
) : Task_Base(robot0)
{
  this->interval_tick_us = interval_tick_us0;
}
