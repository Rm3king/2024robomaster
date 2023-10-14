/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   RefereeSystemTask.cpp
** 文件说明：   裁判系统解析任务
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     林子涵     	     2022-07-18
**	 1.1							   补充注释						     赵钟源     	     2022-12-10
***************************************************************************/

#include "RefereeSystemTask.h"
#include "Scheduler.h"
#include "/Robot/Robot.h"

#include "UARTDriver.h"
#define DevX 0
#define DevY 0
#define UI_OPERATE_ADD							1
#define UI_OPERATE_UPDATE						2
#define UI_COLOR_RED_BLUE						0
#define UI_COLOR_YELLOW							1
#define UI_COLOR_GREEN							2
#define UI_COLOR_ORANGE							3
#define UI_COLOR_VIOLETRED					4
#define UI_COLOR_PINK								5
#define UI_COLOR_BLUEGREEN					6
#define UI_COLOR_BLACK							7
#define UI_COLOR_WHITE							8
//extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

void RefereeSystemTask::init(void)
{
  
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);

  SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR); //使能DMA串口接收
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);  //使能空闲中断

  hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR); //DMA传输外设地址
  hdma_usart6_rx.Instance->M0AR = (uint32_t)(referee_buf); //DMA传输存储器地址

  __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, REFEREE_BUF_LEN); //DMA传输数据的个数(循环模式)
  __HAL_DMA_ENABLE(&hdma_usart6_rx); //串口DMA使能
	filter_speed = 15.0;
}

/***********************************************************************
** 函 数 名： RefereeSystemTask::update()
** 函数说明： 裁判系统数据解析更新
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::update(timeus_t dT_us)
{
  parseData();
	Refresh_ClientUI();
}

void RefereeSystemTask::uninit(void)
{

}
float RefereeSystemTask::get_filter_speed()
{
	return filter_speed;
}
//红色：1 蓝色2
uint8_t RefereeSystemTask::get_robot_color()
{
	if(robot_referee_status.game_robot_status.robot_id>=100)
	{
		return 2;
	}
	else 
	{
		return 1;
	}
}	
uint64_t RefereeSystemTask::get_who_is_balance()
{
	return who_is_balance;
}
/***********************************************************************
** 函 数 名： RefereeSystemTask::pushToBuffer()
** 函数说明： 从串口DMA的接收缓冲循环数组中提取数据帧,压入裁判系统协议解析缓冲队列(在UART中断调用)
**---------------------------------------------------------------------
** 输入参数： 起始位置,结束位置
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::pushToBuffer(uint16_t start_pos, uint16_t end_pos)
{
  uint16_t size;

  if (end_pos >= start_pos)
  {
    size = end_pos - start_pos;
  }
  else
  {
    size = REFEREE_FIFO_BUF_LEN - (start_pos - end_pos);
  }

  if ((size > 0) && (fifo_count < REFEREE_FIFO_BUF_NUM))
  {
    //
    referee_fifo_buf[fifo_tail_pos][0] = size >> 8;
    referee_fifo_buf[fifo_tail_pos][1] = size;

    if (end_pos > start_pos)
    {
      memcpy(referee_fifo_buf[fifo_tail_pos] + 2, referee_buf + start_pos, size);
    }
    else
    {
      memcpy(referee_fifo_buf[fifo_tail_pos] + 2, referee_buf + start_pos, REFEREE_FIFO_BUF_LEN - start_pos);
      memcpy(referee_fifo_buf[fifo_tail_pos] + 2 + (REFEREE_FIFO_BUF_LEN - start_pos), referee_buf, end_pos);
    }


    fifo_tail_pos = (fifo_tail_pos + 1) % REFEREE_FIFO_BUF_NUM;
    fifo_count++;
  }
}

/***********************************************************************
** 函 数 名： RefereeSystemTask::parseData()
** 函数说明： 裁判系统数据解析
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::parseData(void)
{
  int16_t remaining_n;

  uint8_t *pbuf;
  uint8_t *ptr; //用于float和uint64_t类型数据转换

  uint16_t data_len;
  uint16_t cmd;

  if (fifo_count > 0)
  {
    //
    remaining_n = referee_fifo_buf[fifo_head_pos][0] * 256 + referee_fifo_buf[fifo_head_pos][1];
    pbuf = referee_fifo_buf[fifo_head_pos] + 2;

    while (1)
    {
      //
      if ((pbuf[0] == 0xA5) && (remaining_n >= 5)) //check frame_header & length
      {
        if (Verify_CRC8_Check_Sum(pbuf, 5)) //check head_crc8
        {
          data_len = pbuf[2] * 256 + pbuf[1];
          if (pbuf[3] != (uint8_t)(referee_seq_num + 1))
          {
            robot_referee_status.cmd_error_count[0]++;
          } //seq_num error
          referee_seq_num = pbuf[3];

          //
          if (remaining_n >= data_len + 9) //5+2+len+2 //check package length
          {
            if (Verify_CRC16_Check_Sum(pbuf, data_len + 9)) //check package_crc16
            {
              cmd = pbuf[6] * 256 + pbuf[5];

              //
              if (cmd == 0x0201) //10Hz,机器人状态数据
              {
                robot_referee_status.game_robot_status.recv_cnt++;
                robot_referee_status.game_robot_status.timestamp = micros();

                robot_referee_status.game_robot_status.robot_id = pbuf[7];
                robot_referee_status.game_robot_status.robot_level = pbuf[8];

                robot_referee_status.game_robot_status.remain_HP = pbuf[10] * 256 + pbuf[9];
                robot_referee_status.game_robot_status.max_HP = pbuf[12] * 256 + pbuf[11];

                robot_referee_status.game_robot_status.shooter_id1_17mm_cooling_rate = pbuf[14] * 256 + pbuf[13];
                robot_referee_status.game_robot_status.shooter_id1_17mm_cooling_limit = pbuf[16] * 256 + pbuf[15];
                robot_referee_status.game_robot_status.shooter_id1_17mm_speed_limit = pbuf[18] * 256 + pbuf[17];

                robot_referee_status.game_robot_status.shooter_id2_17mm_cooling_rate = pbuf[20] * 256 + pbuf[19];
                robot_referee_status.game_robot_status.shooter_id2_17mm_cooling_limit = pbuf[22] * 256 + pbuf[21];
                robot_referee_status.game_robot_status.shooter_id2_17mm_speed_limit = pbuf[24] * 256 + pbuf[23];

                robot_referee_status.game_robot_status.shooter_id1_42mm_cooling_rate = pbuf[26] * 256 + pbuf[25];
                robot_referee_status.game_robot_status.shooter_id1_42mm_cooling_limit = pbuf[28] * 256 + pbuf[27];
                robot_referee_status.game_robot_status.shooter_id1_42mm_speed_limit = pbuf[30] * 256 + pbuf[29];

                robot_referee_status.game_robot_status.chassis_power_limit = pbuf[32] * 256 + pbuf[31];

                robot_referee_status.game_robot_status.mains_power_gimbal_output = pbuf[33];
                robot_referee_status.game_robot_status.mains_power_chassis_output = pbuf[33] >> 1;
                robot_referee_status.game_robot_status.mains_power_shooter_output = pbuf[33] >> 2;
              }
              else if (cmd == 0x0202) //50Hz,实时功率热量数据
              {
                robot_referee_status.power_heat_data.recv_cnt++;
                robot_referee_status.power_heat_data.timestamp = micros();

                robot_referee_status.power_heat_data.chassis_volt = pbuf[8] * 256 + pbuf[7];
                robot_referee_status.power_heat_data.chassis_current = pbuf[10] * 256 + pbuf[9];
                ptr = (uint8_t *)&robot_referee_status.power_heat_data.chassis_power;
                ptr[0] = pbuf[11];
                ptr[1] = pbuf[12];
                ptr[2] = pbuf[13];
                ptr[3] = pbuf[14];
                robot_referee_status.power_heat_data.chassis_power_buffer = pbuf[16] * 256 + pbuf[15];

                robot_referee_status.power_heat_data.shooter_id1_17mm_cooling_heat = pbuf[18] * 256 + pbuf[17];
                robot_referee_status.power_heat_data.shooter_id1_17mm_cooling_heat = pbuf[20] * 256 + pbuf[19];
                robot_referee_status.power_heat_data.shooter_id1_42mm_cooling_heat = pbuf[22] * 256 + pbuf[21];
              }
              else if (cmd == 0x0203) //10Hz,机器人位置数据
              {
                robot_referee_status.game_robot_pos.recv_cnt++;
                robot_referee_status.game_robot_pos.timestamp = micros();

                ptr = (uint8_t *)&robot_referee_status.game_robot_pos.x;
                ptr[0] = pbuf[7];
                ptr[1] = pbuf[8];
                ptr[2] = pbuf[9];
                ptr[3] = pbuf[10];
                ptr = (uint8_t *)&robot_referee_status.game_robot_pos.y;
                ptr[0] = pbuf[11];
                ptr[1] = pbuf[12];
                ptr[2] = pbuf[13];
                ptr[3] = pbuf[14];
                ptr = (uint8_t *)&robot_referee_status.game_robot_pos.z;
                ptr[0] = pbuf[15];
                ptr[1] = pbuf[16];
                ptr[2] = pbuf[17];
                ptr[3] = pbuf[18];
                ptr = (uint8_t *)&robot_referee_status.game_robot_pos.yaw;
                ptr[0] = pbuf[19];
                ptr[1] = pbuf[20];
                ptr[2] = pbuf[21];
                ptr[3] = pbuf[22];
              }
              else if (cmd == 0x0204) //1Hz,机器人增益数据
              {
                robot_referee_status.buff.recv_cnt++;
                robot_referee_status.buff.timestamp = micros();

                robot_referee_status.buff.power_rune_buff = pbuf[7];
              }
              else if (cmd == 0x0206) //伤害发生后发送,伤害状态数据
              {
                robot_referee_status.robot_hurt.recv_cnt++;
                robot_referee_status.robot_hurt.timestamp = micros();

                robot_referee_status.robot_hurt.armor_id = pbuf[7];
                robot_referee_status.robot_hurt.hurt_type = pbuf[7] >> 4;
              }
              else if (cmd == 0x0207) //子弹发射后发送,实时射击数据
              {
                robot_referee_status.shoot_data.recv_cnt++;
                robot_referee_status.shoot_data.timestamp = micros();

                robot_referee_status.shoot_data.bullet_type = pbuf[7];
                robot_referee_status.shoot_data.shooter_id = pbuf[8];
                robot_referee_status.shoot_data.bullet_freq = pbuf[9];
                ptr = (uint8_t *)&robot_referee_status.shoot_data.bullet_speed;
                ptr[0] = pbuf[10];
                ptr[1] = pbuf[11];
                ptr[2] = pbuf[12];
                ptr[3] = pbuf[13];
								if(robot_referee_status.shoot_data.recv_cnt != robot_referee_status.shoot_data.recv_cnt_last )//上一发弹速
								{			
										filter_speed = 0.3*robot_referee_status.shoot_data.bullet_speed_last+0.7*robot_referee_status.shoot_data.bullet_speed;
										robot_referee_status.shoot_data.bullet_speed_last = robot_referee_status.shoot_data.bullet_speed;
								}
              }
              else if (cmd == 0x0208) //1Hz,子弹剩余发射数
              {
                robot_referee_status.bullet_remaining.recv_cnt++;
                robot_referee_status.bullet_remaining.timestamp = micros();

                robot_referee_status.bullet_remaining.bullet_remaining_num_17mm = pbuf[8] * 256 + pbuf[7];
                robot_referee_status.bullet_remaining.bullet_remaining_num_42mm = pbuf[10] * 256 + pbuf[9];
                robot_referee_status.bullet_remaining.coin_remaining_num = pbuf[12] * 256 + pbuf[11];
              }
              else if (cmd == 0x0209) //1Hz,机器人RFID状态数据
              {
                robot_referee_status.rfid_status.recv_cnt++;
                robot_referee_status.rfid_status.timestamp = micros();

                robot_referee_status.rfid_status.rfid_status = pbuf[9]; //(pbuf[12]<<24)+(pbuf[11]<<16)+(pbuf[10]<<8)+pbuf[9];
              }

              else if (cmd == 0x0001) //1Hz,比赛状态数据
              {
                robot_referee_status.game_status.recv_cnt++;
                robot_referee_status.game_status.timestamp = micros();

                //1机甲大师赛  2机甲大师单项赛  3人工智能挑战赛  4联盟赛3V3 5联盟赛1V1
                robot_referee_status.game_status.game_type = pbuf[7];

                //0未开始 1准备阶段 2自检阶段 3倒计时5s 4对战中 5比赛解算中
                robot_referee_status.game_status.game_progress = pbuf[7] >> 4;

                //剩余时间 秒为单位
                robot_referee_status.game_status.stage_remain_time = pbuf[9] * 256 + pbuf[8];

                //Unix时间
                ptr = (uint8_t *)&robot_referee_status.game_status.SyncTimeStamp;
                ptr[0] = pbuf[10];
                ptr[1] = pbuf[11];
                ptr[2] = pbuf[12];
                ptr[3] = pbuf[13];
                ptr[4] = pbuf[14];
                ptr[5] = pbuf[15];
                ptr[6] = pbuf[16];
                ptr[7] = pbuf[17];
              }
              else if (cmd == 0x0002) //比赛结束后发送,比赛结果数据
              {
                robot_referee_status.game_result.recv_cnt++;
                robot_referee_status.game_result.timestamp = micros();

                //0平局 1红方胜利 2蓝方胜利
                robot_referee_status.game_result.winner = pbuf[7];
              }
              else if (cmd == 0x0003) //1Hz,比赛机器人血量数据
              {
                robot_referee_status.game_robot_HP.recv_cnt++;
                robot_referee_status.game_robot_HP.timestamp = micros();

                robot_referee_status.game_robot_HP.red_1_robot_HP = pbuf[8] * 256 + pbuf[7];
                robot_referee_status.game_robot_HP.red_2_robot_HP = pbuf[10] * 256 + pbuf[9];
                robot_referee_status.game_robot_HP.red_3_robot_HP = pbuf[12] * 256 + pbuf[11];
                robot_referee_status.game_robot_HP.red_4_robot_HP = pbuf[14] * 256 + pbuf[13];
                robot_referee_status.game_robot_HP.red_5_robot_HP = pbuf[16] * 256 + pbuf[15];
                robot_referee_status.game_robot_HP.red_7_robot_HP = pbuf[18] * 256 + pbuf[17];
                robot_referee_status.game_robot_HP.red_outpost_HP = pbuf[20] * 256 + pbuf[19];
                robot_referee_status.game_robot_HP.red_base_HP = pbuf[22] * 256 + pbuf[21];
                robot_referee_status.game_robot_HP.blue_1_robot_HP = pbuf[24] * 256 + pbuf[23];
                robot_referee_status.game_robot_HP.blue_2_robot_HP = pbuf[26] * 256 + pbuf[25];
                robot_referee_status.game_robot_HP.blue_3_robot_HP = pbuf[28] * 256 + pbuf[27];
                robot_referee_status.game_robot_HP.blue_4_robot_HP = pbuf[30] * 256 + pbuf[29];
                robot_referee_status.game_robot_HP.blue_5_robot_HP = pbuf[32] * 256 + pbuf[31];
                robot_referee_status.game_robot_HP.blue_7_robot_HP = pbuf[34] * 256 + pbuf[33];
                robot_referee_status.game_robot_HP.blue_outpost_HP = pbuf[36] * 256 + pbuf[35];
                robot_referee_status.game_robot_HP.blue_base_HP = pbuf[38] * 256 + pbuf[37];
										if(robot_referee_status.game_robot_HP.recv_cnt > 0 && robot_referee_status.game_status.stage_remain_time > 290)
		{
			if(robot_referee_status.game_robot_status.robot_id > 100)
			{
				if(robot_referee_status.game_robot_HP.red_3_robot_HP > 202)
				{
					who_is_balance = 3;
				}
				if(robot_referee_status.game_robot_HP.red_4_robot_HP > 202)
				{
					who_is_balance = 4;
				}
				if(robot_referee_status.game_robot_HP.red_5_robot_HP > 202)
				{
					who_is_balance = 5;
				}
			}
			else if(robot_referee_status.game_robot_status.robot_id > 0)
			{
				if(robot_referee_status.game_robot_HP.blue_3_robot_HP > 202)
				{
					who_is_balance = 3;
				}
				if(robot_referee_status.game_robot_HP.blue_4_robot_HP > 202)
				{
					who_is_balance = 4;
				}
				if(robot_referee_status.game_robot_HP.blue_5_robot_HP > 202)
				{
					who_is_balance = 5;
				}
			}
		}
              }

              else if (cmd == 0x0101) //1Hz,场地事件数据
              {
                robot_referee_status.event_data.recv_cnt++;
                robot_referee_status.event_data.timestamp = micros();

                robot_referee_status.event_data.event_type = (pbuf[10] << 24) + (pbuf[9] << 16) + (pbuf[8] << 8) + pbuf[7];
              }
              else if (cmd == 0x0102) //动作改变后发送,场地补给站动作标识数据
              {
                robot_referee_status.supply_projectile_action.recv_cnt++;
                robot_referee_status.supply_projectile_action.timestamp = micros();

                robot_referee_status.supply_projectile_action.supply_projectile_id = pbuf[7];
                robot_referee_status.supply_projectile_action.supply_robot_id = pbuf[8];
                robot_referee_status.supply_projectile_action.supply_projectile_step = pbuf[9];
                robot_referee_status.supply_projectile_action.supply_projectile_num = pbuf[10];
              }
              else if (cmd == 0x0104) //警告发生后发送,裁判警告信息
              {
                robot_referee_status.referee_warning.recv_cnt++;
                robot_referee_status.referee_warning.timestamp = micros();

                robot_referee_status.referee_warning.level = pbuf[7];
                robot_referee_status.referee_warning.foul_robot_id = pbuf[8];
              }
              else //cmd error
              {
                robot_referee_status.cmd_error_count[5]++;
              }
            }
            else //check package_crc16 error
            {
              robot_referee_status.cmd_error_count[4]++;
              break;
            }
          }
          else //check package length error
          {
            robot_referee_status.cmd_error_count[3]++;
            break;
          }
        }
        else //check head_crc8 error
        {
          robot_referee_status.cmd_error_count[2]++;
          break;
        }
      }
      else //check frame_header & length error
      {
        robot_referee_status.cmd_error_count[1]++;
        break;
      }

      //
      remaining_n = remaining_n - (data_len + 9);
      pbuf = pbuf + (data_len + 9);

      if (remaining_n <= 0)
      {
        break;
      }
    }

    //
    fifo_head_pos = (fifo_head_pos + 1) % REFEREE_FIFO_BUF_NUM;
    fifo_count--;
  }
}

/*---------------------机器人间通信与客户端UI代码---------------------*/
/***********************************************************************
** 函 数 名： create_ui_interactive_package()
** 函数说明： 建立机器人和客户端UI通信数据帧内容(未填充数据段)
**---------------------------------------------------------------------
** 输入参数： 数据帧缓冲区地址, 数据帧缓冲区长度, 数据段内容id
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::create_ui_interactive_package(uint8_t *tx_buf, uint8_t tx_buf_len, uint16_t data_cmd_id)
{
  //4 bytes
  frame_header_t *p_frame_header = (frame_header_t *)(tx_buf + 0);
  p_frame_header->sof = 0xA5;
  p_frame_header->data_length = tx_buf_len - 9; //数据长度
  p_frame_header->seq = student_interactive_seq++;

  //1 bytes
  Append_CRC8_Check_Sum(tx_buf, 5);

  //2 bytes
  uint16_t cmd_id = 0x0301; //交互数据指令
  memcpy(tx_buf + 5, &cmd_id, sizeof(cmd_id));

  //6 bytes
  ext_student_interactive_header_data_t *p_student_interacitve_header = (ext_student_interactive_header_data_t *)(tx_buf + 7);
  p_student_interacitve_header->data_cmd_id = data_cmd_id;
  p_student_interacitve_header->sender_ID = robot_referee_status.game_robot_status.robot_id;
  p_student_interacitve_header->receiver_ID = (robot_referee_status.game_robot_status.robot_id + 0x0100);

  //数据段(需提前填充好)

  //2 bytes
  Append_CRC16_Check_Sum(tx_buf, tx_buf_len);
}

/***********************************************************************
** 函 数 名： set_clear_graphics_data()
** 函数说明： 设置UI数据段内容:删除图形
**---------------------------------------------------------------------
** 输入参数： UI数据段缓冲区地址,操作,图层
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::set_clear_graphics_data(uint8_t *tx_buf, uint8_t operate, uint8_t layer)
{
  ext_client_custom_graphic_delete_t *p_client_custom_graphic_delete = (ext_client_custom_graphic_delete_t *)(tx_buf + 0);

  p_client_custom_graphic_delete->operate_type = operate; //0-空操作;1-删除图层;2-删除所有
  p_client_custom_graphic_delete->layer = layer;          //图层数0-9
}

/***********************************************************************
** 函 数 名： set_line_graphics_data()
** 函数说明： 设置UI数据段内容:直线数据
**---------------------------------------------------------------------
** 输入参数： UI数据段缓冲区地址,图层名,图形名,操作,线宽,颜色,起点x,起点y,终点x,终点y
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::set_line_graphics_data(uint8_t *tx_buf, uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
  graphic_data_struct_t *p_graphic_data_struct = (graphic_data_struct_t *)(tx_buf + 0);

  p_graphic_data_struct->graphic_name[0] = name[0];
  p_graphic_data_struct->graphic_name[1] = name[1];
  p_graphic_data_struct->graphic_name[2] = name[2];
  p_graphic_data_struct->operate_tpye = operate;                      //0-空;1-增加;2-修改;3-删除
  p_graphic_data_struct->graphic_tpye = 0;                            //0-直线;1-矩形;2-整圆;3-椭圆;4-圆弧;5-浮点数;6-整形数;7-字符
  p_graphic_data_struct->layer = layer;                               //图层0-9
  p_graphic_data_struct->color = color;                               //0-红蓝主色;1-黄色;2-绿色;3-橙色;4-紫红色;5-粉色;6-青色;7-黑色;8-白色
  p_graphic_data_struct->start_angle = 0;                             //空
  p_graphic_data_struct->end_angle = 0;                               //空
  p_graphic_data_struct->width = width;                               //线条宽度
  p_graphic_data_struct->start_x = start_x + DevX;                    //起点x坐标
  p_graphic_data_struct->start_y = start_y + DevY;                    //起点y坐标
  p_graphic_data_struct->graphic_config_3.value.radius = 0;           //空
  p_graphic_data_struct->graphic_config_3.value.end_x = end_x + DevX; //终点x坐标
  p_graphic_data_struct->graphic_config_3.value.end_y = end_y + DevY; //终点y坐标
}

/***********************************************************************
** 函 数 名： set_rectangle_graphics_data()
** 函数说明： 设置UI数据段内容:矩形数据
**---------------------------------------------------------------------
** 输入参数： UI数据段缓冲区地址,图层名,图形名,操作,线宽,颜色,起点x,起点y,对角顶点x,对角顶点y
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::set_rectangle_graphics_data(uint8_t *tx_buf, uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t start_x, uint16_t start_y, uint16_t opposite_x, uint16_t opposite_y)
{
  graphic_data_struct_t *p_graphic_data_struct = (graphic_data_struct_t *)(tx_buf + 0);

  p_graphic_data_struct->graphic_name[0] = name[0];
  p_graphic_data_struct->graphic_name[1] = name[1];
  p_graphic_data_struct->graphic_name[2] = name[2];
  p_graphic_data_struct->operate_tpye = operate;                           //0-空;1-增加;2-修改;3-删除
  p_graphic_data_struct->graphic_tpye = 1;                                 //0-直线;1-矩形;2-整圆;3-椭圆;4-圆弧;5-浮点数;6-整形数;7-字符
  p_graphic_data_struct->layer = layer;                                    //图层0-9
  p_graphic_data_struct->color = color;                                    //0-红蓝主色;1-黄色;2-绿色;3-橙色;4-紫红色;5-粉色;6-青色;7-黑色;8-白色
  p_graphic_data_struct->start_angle = 0;                                  //空
  p_graphic_data_struct->end_angle = 0;                                    //空
  p_graphic_data_struct->width = width;                                    //线条宽度
  p_graphic_data_struct->start_x = start_x + DevX;                         //起点x坐标
  p_graphic_data_struct->start_y = start_y + DevY;                         //起点y坐标
  p_graphic_data_struct->graphic_config_3.value.radius = 0;                //空
  p_graphic_data_struct->graphic_config_3.value.end_x = opposite_x + DevX; //对角顶点x坐标
  p_graphic_data_struct->graphic_config_3.value.end_y = opposite_y + DevY; //对角顶点y坐标
}

/***********************************************************************
** 函 数 名： set_circle_graphics_data()
** 函数说明： 设置UI数据段内容:整圆数据
**---------------------------------------------------------------------
** 输入参数： UI数据段缓冲区地址,图层名,图形名,操作,线宽,颜色,圆心x,圆心y,半径
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::set_circle_graphics_data(uint8_t *tx_buf, uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t center_x, uint16_t center_y, uint16_t radius)
{
  graphic_data_struct_t *p_graphic_data_struct = (graphic_data_struct_t *)(tx_buf + 0);

  p_graphic_data_struct->graphic_name[0] = name[0];
  p_graphic_data_struct->graphic_name[1] = name[1];
  p_graphic_data_struct->graphic_name[2] = name[2];
  p_graphic_data_struct->operate_tpye = operate;                 //0-空;1-增加;2-修改;3-删除
  p_graphic_data_struct->graphic_tpye = 2;                       //0-直线;1-矩形;2-整圆;3-椭圆;4-圆弧;5-浮点数;6-整形数;7-字符
  p_graphic_data_struct->layer = layer;                          //图层0-9
  p_graphic_data_struct->color = color;                          //0-红蓝主色;1-黄色;2-绿色;3-橙色;4-紫红色;5-粉色;6-青色;7-黑色;8-白色
  p_graphic_data_struct->start_angle = 0;                        //空
  p_graphic_data_struct->end_angle = 0;                          //空
  p_graphic_data_struct->width = width;                          //线条宽度
  p_graphic_data_struct->start_x = center_x + DevX;              //圆心x坐标
  p_graphic_data_struct->start_y = center_y + DevY;              //圆心y坐标
  p_graphic_data_struct->graphic_config_3.value.radius = radius; //半径
  p_graphic_data_struct->graphic_config_3.value.end_x = 0;       //空
  p_graphic_data_struct->graphic_config_3.value.end_y = 0;       //空
}

/***********************************************************************
** 函 数 名： set_ellipse_graphics_data()
** 函数说明： 设置UI数据段内容:椭圆数据
**---------------------------------------------------------------------
** 输入参数： UI数据段缓冲区地址,图层名,图形名,操作,线宽,颜色,圆心x,圆心y,半轴x,半轴y
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::set_ellipse_graphics_data(uint8_t *tx_buf, uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t center_x, uint16_t center_y, uint16_t half_axis_x, uint16_t half_axis_y)
{
  graphic_data_struct_t *p_graphic_data_struct = (graphic_data_struct_t *)(tx_buf + 0);

  p_graphic_data_struct->graphic_name[0] = name[0];
  p_graphic_data_struct->graphic_name[1] = name[1];
  p_graphic_data_struct->graphic_name[2] = name[2];
  p_graphic_data_struct->operate_tpye = operate;                     //0-空;1-增加;2-修改;3-删除
  p_graphic_data_struct->graphic_tpye = 3;                           //0-直线;1-矩形;2-整圆;3-椭圆;4-圆弧;5-浮点数;6-整形数;7-字符
  p_graphic_data_struct->layer = layer;                              //图层0-9
  p_graphic_data_struct->color = color;                              //0-红蓝主色;1-黄色;2-绿色;3-橙色;4-紫红色;5-粉色;6-青色;7-黑色;8-白色
  p_graphic_data_struct->start_angle = 0;                            //空
  p_graphic_data_struct->end_angle = 0;                              //空
  p_graphic_data_struct->width = width;                              //线条宽度
  p_graphic_data_struct->start_x = center_x + DevX;                  //圆心x坐标
  p_graphic_data_struct->start_y = center_y + DevY;                  //圆心y坐标
  p_graphic_data_struct->graphic_config_3.value.radius = 0;          //空
  p_graphic_data_struct->graphic_config_3.value.end_x = half_axis_x; //半轴x长度
  p_graphic_data_struct->graphic_config_3.value.end_y = half_axis_y; //半轴y长度
}

/***********************************************************************
** 函 数 名： set_arc_graphics_data()
** 函数说明： 设置UI数据段内容:圆弧数据
**---------------------------------------------------------------------
** 输入参数： UI数据段缓冲区地址,图层名,图形名,操作,线宽,颜色,起始角度,终止角度,圆心x,圆心y,半轴x,半轴y
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::set_arc_graphics_data(uint8_t *tx_buf, uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t start_angle, uint16_t end_angle, uint16_t center_x, uint16_t center_y, uint16_t half_axis_x, uint16_t half_axis_y)
{
  graphic_data_struct_t *p_graphic_data_struct = (graphic_data_struct_t *)(tx_buf + 0);

  p_graphic_data_struct->graphic_name[0] = name[0];
  p_graphic_data_struct->graphic_name[1] = name[1];
  p_graphic_data_struct->graphic_name[2] = name[2];
  p_graphic_data_struct->operate_tpye = operate;                     //0-空;1-增加;2-修改;3-删除
  p_graphic_data_struct->graphic_tpye = 4;                           //0-直线;1-矩形;2-整圆;3-椭圆;4-圆弧;5-浮点数;6-整形数;7-字符
  p_graphic_data_struct->layer = layer;                              //图层0-9
  p_graphic_data_struct->color = color;                              //0-红蓝主色;1-黄色;2-绿色;3-橙色;4-紫红色;5-粉色;6-青色;7-黑色;8-白色
  p_graphic_data_struct->start_angle = start_angle;                  //起始角度
  p_graphic_data_struct->end_angle = end_angle;                      //终止角度
  p_graphic_data_struct->width = width;                              //线条宽度
  p_graphic_data_struct->start_x = center_x + DevX;                  //圆心x坐标
  p_graphic_data_struct->start_y = center_y + DevY;                  //圆心y坐标
  p_graphic_data_struct->graphic_config_3.value.radius = 0;          //空
  p_graphic_data_struct->graphic_config_3.value.end_x = half_axis_x; //半轴x长度
  p_graphic_data_struct->graphic_config_3.value.end_y = half_axis_y; //半轴y长度
}

/***********************************************************************
** 函 数 名： set_DrawString_data()
** 函数说明： 设置UI数据段内容:字符数据
**---------------------------------------------------------------------
** 输入参数： UI数据段缓冲区地址,图层名,图形名,操作,字体大小,颜色,起点x,起点y,格式化字符串
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::set_DrawString_data(uint8_t *tx_buf, uint8_t layer, char *name, uint8_t operate, uint8_t size, uint8_t color, uint16_t start_x, uint16_t start_y, const char *str, ...)
{
	uint8_t len = 0;
  ext_client_custom_character_t *p_client_custom_character = (ext_client_custom_character_t *)(tx_buf + 0);

	va_list ap;
  va_start(ap, str);
  len = vsprintf((char *)(p_client_custom_character->data), str, ap);
  va_end(ap);
	
	p_client_custom_character->grapic_data_struct.graphic_name[0] = name[0];
  p_client_custom_character->grapic_data_struct.graphic_name[1] = name[1];
  p_client_custom_character->grapic_data_struct.graphic_name[2] = name[2];
  p_client_custom_character->grapic_data_struct.operate_tpye = operate;            //0-空;1-增加;2-修改;3-删除
  p_client_custom_character->grapic_data_struct.graphic_tpye = 7;                  //0-直线;1-矩形;2-整圆;3-椭圆;4-圆弧;5-浮点数;6-整形数;7-字符
  p_client_custom_character->grapic_data_struct.layer = layer;                     //图层0-9
  p_client_custom_character->grapic_data_struct.color = color;                     //0-红蓝主色;1-黄色;2-绿色;3-橙色;4-紫红色;5-粉色;6-青色;7-黑色;8-白色
  p_client_custom_character->grapic_data_struct.start_angle = size * 10;           //字体大小
  p_client_custom_character->grapic_data_struct.end_angle = len;                   //字符长度
  p_client_custom_character->grapic_data_struct.width = size;                      //线条宽度
  p_client_custom_character->grapic_data_struct.start_x = start_x + DevX;          //起点x坐标
  p_client_custom_character->grapic_data_struct.start_y = start_y + DevY;          //起点y坐标
  p_client_custom_character->grapic_data_struct.graphic_config_3.value.radius = 0; //空
  p_client_custom_character->grapic_data_struct.graphic_config_3.value.end_x = 0;  //空
  p_client_custom_character->grapic_data_struct.graphic_config_3.value.end_y = 0;  //空
}
/***********************************************************************
** 函 数 名： ClientUI_ClearLayer()
** 函数说明： 清除客户端所选图层的UI图形
**---------------------------------------------------------------------
** 输入参数： 图层
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::ClientUI_ClearLayer(uint8_t layer)
{
  static uint8_t tx_buf[17] = {0};

  set_clear_graphics_data(tx_buf + 13, 1, layer);    //删除所选图层
  create_ui_interactive_package(tx_buf, 17, 0x0100); //客户端删除图形

  Transmit_DMA(&huart6, tx_buf, 17); //发送数据帧
}

/***********************************************************************
** 函 数 名： ClientUI_DrawLine()
** 函数说明： 客户端UI画一个直线
**---------------------------------------------------------------------
** 输入参数： 图层名,图形名,操作,线宽,颜色,起点x,起点y,终点x,终点y
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::ClientUI_DrawLine(uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
  static uint8_t tx_buf[30] = {0};

  set_line_graphics_data(tx_buf + 13, layer, name, operate, width, color, start_x, start_y, end_x, end_y); //设置直线绘制数据
  create_ui_interactive_package(tx_buf, 30, 0x0101);                                                       //客户端绘制一个图形

  Transmit_DMA(&huart6, tx_buf, 30); //发送数据帧
}

/***********************************************************************
** 函 数 名： ClientUI_DrawRectangle()
** 函数说明： 客户端UI画一个矩形
**---------------------------------------------------------------------
** 输入参数： 图层名,图形名,操作,线宽,颜色,起点x,起点y,对角顶点x,对角顶点y
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::ClientUI_DrawRectangle(uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t start_x, uint16_t start_y, uint16_t opposite_x, uint16_t opposite_y)
{
  static uint8_t tx_buf[30] = {0};

  set_rectangle_graphics_data(tx_buf + 13, layer, name, operate, width, color, start_x, start_y, opposite_x, opposite_y); //设置矩形绘制数据
  create_ui_interactive_package(tx_buf, 30, 0x0101);                                                                      //客户端绘制一个图形

  Transmit_DMA(&huart6, tx_buf, 30); //发送数据帧
}

/***********************************************************************
** 函 数 名： ClientUI_DrawCircle()
** 函数说明： 客户端UI画一个整圆
**---------------------------------------------------------------------
** 输入参数： 图层名,图形名,操作,线宽,颜色,圆心x,圆心y,半径
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::ClientUI_DrawCircle(uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t center_x, uint16_t center_y, uint16_t radius)
{
  static uint8_t tx_buf[30] = {0};

  set_circle_graphics_data(tx_buf + 13, layer, name, operate, width, color, center_x, center_y, radius); //设置整圆绘制数据
  create_ui_interactive_package(tx_buf, 30, 0x0101);                                                     //客户端绘制一个图形

  Transmit_DMA(&huart6, tx_buf, 30); //发送数据帧
}

/***********************************************************************
** 函 数 名： ClientUI_DrawEllipse()
** 函数说明： 客户端UI画一个椭圆
**---------------------------------------------------------------------
** 输入参数： 图层名,图形名,操作,线宽,颜色,圆心x,圆心y,半轴x,半轴y
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::ClientUI_DrawEllipse(uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t center_x, uint16_t center_y, uint16_t half_axis_x, uint16_t half_axis_y)
{
  static uint8_t tx_buf[30] = {0};

  set_ellipse_graphics_data(tx_buf + 13, layer, name, operate, width, color, center_x, center_y, half_axis_x, half_axis_y); //设置椭圆绘制数据
  create_ui_interactive_package(tx_buf, 30, 0x0101);                                                                        //客户端绘制一个图形

  Transmit_DMA(&huart6, tx_buf, 30); //发送数据帧
}

/***********************************************************************
** 函 数 名： ClientUI_DrawArc()
** 函数说明： 客户端UI画一个圆弧
**---------------------------------------------------------------------
** 输入参数： 图层名,图形名,操作,线宽,颜色,起始角度,终止角度,圆心x,圆心y,半轴x,半轴y
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::ClientUI_DrawArc(uint8_t layer, char *name, uint8_t operate, uint16_t width, uint8_t color, uint16_t start_angle, uint16_t end_angle, uint16_t center_x, uint16_t center_y, uint16_t half_axis_x, uint16_t half_axis_y)
{
  static uint8_t tx_buf[30] = {0};

  set_arc_graphics_data(tx_buf + 13, layer, name, operate, width, color, start_angle, end_angle, center_x, center_y, half_axis_x, half_axis_y); //设置圆弧绘制数据
  create_ui_interactive_package(tx_buf, 30, 0x0101);                                                                                            //客户端绘制一个图形

  Transmit_DMA(&huart6, tx_buf, 30); //发送数据帧
}

/***********************************************************************
** 函 数 名： ClientUI_DrawString()
** 函数说明： 客户端UI画格式化字符串
**---------------------------------------------------------------------
** 输入参数： 图层名,图形名,操作,字体大小,颜色,起点x,起点y,格式化字符串
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::ClientUI_DrawString(uint8_t layer, char *name, uint8_t operate, uint8_t size, uint8_t color, uint16_t start_x, uint16_t start_y, const char *str, ...)
{
  static uint8_t tx_buf[60] = {0};
  uint8_t len = 0;
	memset(tx_buf, 0, 60);
  //4 bytes
  frame_header_t *p_frame_header = (frame_header_t *)(tx_buf + 0);
  p_frame_header->sof = 0xA5;
  p_frame_header->data_length = 51; //数据长度
  p_frame_header->seq = student_interactive_seq++;

  //1 byztes
  Append_CRC8_Check_Sum(tx_buf, 5);

  //2 bytes
  uint16_t cmd_id = 0x0301;
  memcpy(tx_buf + 5, &cmd_id, sizeof(cmd_id));

  //6 bytes
  ext_student_interactive_header_data_t *p_student_interacitve_header = (ext_student_interactive_header_data_t *)(tx_buf + 7);
  p_student_interacitve_header->data_cmd_id = 0x0110; //客户端绘制字符图形
  p_student_interacitve_header->sender_ID = robot_referee_status.game_robot_status.robot_id;
  p_student_interacitve_header->receiver_ID = (robot_referee_status.game_robot_status.robot_id + 0x0100);

  //15+30 bytes
  ext_client_custom_character_t *p_client_custom_character = (ext_client_custom_character_t *)(tx_buf + 13);

	//计算字符串长度
  va_list ap;
  va_start(ap, str);
  len = vsprintf((char *)(p_client_custom_character->data), str, ap);
  va_end(ap);

  p_client_custom_character->grapic_data_struct.graphic_name[0] = name[0];
  p_client_custom_character->grapic_data_struct.graphic_name[1] = name[1];
  p_client_custom_character->grapic_data_struct.graphic_name[2] = name[2];
  p_client_custom_character->grapic_data_struct.operate_tpye = operate;            //0-空;1-增加;2-修改;3-删除
  p_client_custom_character->grapic_data_struct.graphic_tpye = 7;                  //0-直线;1-矩形;2-整圆;3-椭圆;4-圆弧;5-浮点数;6-整形数;7-字符
  p_client_custom_character->grapic_data_struct.layer = layer;                     //图层0-9
  p_client_custom_character->grapic_data_struct.color = color;                     //0-红蓝主色;1-黄色;2-绿色;3-橙色;4-紫红色;5-粉色;6-青色;7-黑色;8-白色
  p_client_custom_character->grapic_data_struct.start_angle = size * 10;           //字体大小
  p_client_custom_character->grapic_data_struct.end_angle = len;                   //字符长度
  p_client_custom_character->grapic_data_struct.width = size;                      //线条宽度
  p_client_custom_character->grapic_data_struct.start_x = start_x + DevX;          //起点x坐标
  p_client_custom_character->grapic_data_struct.start_y = start_y + DevY;          //起点y坐标
  p_client_custom_character->grapic_data_struct.graphic_config_3.value.radius = 0; //空
  p_client_custom_character->grapic_data_struct.graphic_config_3.value.end_x = 0;  //空
  p_client_custom_character->grapic_data_struct.graphic_config_3.value.end_y = 0;  //空

  //2 bytes
  Append_CRC16_Check_Sum(tx_buf, 60);

  //
  Transmit_DMA(&huart6, tx_buf, 60); //发送数据帧
}


/***********************************************************************
** 函 数 名： Transmit_DMA()
** 函数说明：  串口DMA发送
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/

void RefereeSystemTask::Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData,uint32_t Size)
{
	robot.scheduler.startAsync(transmit_dma_func.setData(huart, pData, Size), Size *8.0f / 5000 * 1000);
}


/***********************************************************************
** 函 数 名： RefreshClientUI()
** 函数说明： 刷新客户端UI界面
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::Refresh_ClientUI(void)
{
  //客户端屏幕尺寸为1920*1080, 左下角为(0,0), 中心点为(960,540), 点坐标(x,y)中 x为水平 y为竖直
//	static uint8_t num = 0 ;
//	static uint8_t t = 0;
	static uint8_t tx_buf[120] = {0};
	//限制时间
	if(micros() / 1000 < transmit_dma_func.delay_ms + transmit_dma_func.register_ms + 20) return; 
//	USART1_DMA_Debug_Printf("t %.2f %ld\n", micros() / 1000.0f, transmit_dma_func.delay_ms + transmit_dma_func.register_ms);
switch(num)
	{
		case 0:
		{
			ClientUI_ClearLayer(2);break;//清除图层2 静态图形
		}
		case 1:
		{
			ClientUI_ClearLayer(5);break;//清除图层3 数值数据
		}
		case 2:
		{
			ClientUI_ClearLayer(9);break;//清除图层5 动态UI
		}
		
		
		case 3:
		{ //底盘指示+超级电容
			set_line_graphics_data(tx_buf + 13, 5, "OG1", UI_OPERATE_ADD, 20, UI_COLOR_VIOLETRED, 500, 0,  1420, 0);
			set_line_graphics_data(tx_buf + 28, 5, "OS0", UI_OPERATE_ADD, 15, UI_COLOR_BLUEGREEN, 700, 25, 1175,25);
			set_line_graphics_data(tx_buf + 43, 5, "OS1", UI_OPERATE_ADD, 3, UI_COLOR_WHITE, 700, 33, 1175,33);
			set_line_graphics_data(tx_buf + 58, 5, "OS2", UI_OPERATE_ADD, 3, UI_COLOR_WHITE, 700, 18, 1175,18);
			set_line_graphics_data(tx_buf + 73, 5, "OS3", UI_OPERATE_ADD, 3, UI_COLOR_WHITE, 700, 33, 700,18);
			set_line_graphics_data(tx_buf + 88, 5, "OS4", UI_OPERATE_ADD, 3, UI_COLOR_WHITE, 700, 18, 1175,18);
			set_line_graphics_data(tx_buf + 103,5, "OS5", UI_OPERATE_ADD, 3, UI_COLOR_WHITE, 1175, 18, 1175,33);
			create_ui_interactive_package(tx_buf, 120, 0x0104);
			Transmit_DMA(&huart6, tx_buf, 120); //发送数据帧
			break;
		}
		case 4:
		{
            //底盘  判断哪一方
			if(robot_referee_status.game_robot_status.robot_id < 100)//RED
			{
				set_line_graphics_data(tx_buf + 13, 2, "S03", UI_OPERATE_ADD, 5, UI_COLOR_RED_BLUE, 500, 50, 700, 150);
				set_line_graphics_data(tx_buf + 28, 2, "S01", UI_OPERATE_ADD, 5, UI_COLOR_RED_BLUE, 700, 150, 900, 150);
				set_line_graphics_data(tx_buf + 43,	2, "S02", UI_OPERATE_ADD, 5, UI_COLOR_RED_BLUE, 1020, 150, 1220, 150);
				set_line_graphics_data(tx_buf + 58, 2, "L04", UI_OPERATE_ADD, 5, UI_COLOR_RED_BLUE, 1220, 150, 1420, 50);
			}
			
			else 
			{
				set_line_graphics_data(tx_buf + 13, 2, "S03", UI_OPERATE_ADD, 5, UI_COLOR_BLUEGREEN, 500, 50, 700, 150);
				set_line_graphics_data(tx_buf + 28, 2, "S01", UI_OPERATE_ADD, 5, UI_COLOR_BLUEGREEN, 700, 150, 900, 150);
				set_line_graphics_data(tx_buf + 43,	2, "S02", UI_OPERATE_ADD, 5, UI_COLOR_BLUEGREEN, 1020, 150, 1220, 150);
				set_line_graphics_data(tx_buf + 58, 2, "S04", UI_OPERATE_ADD, 5, UI_COLOR_BLUEGREEN, 1220, 150, 1420, 50);				
			}
			create_ui_interactive_package(tx_buf, 75, 0x0103);
			Transmit_DMA(&huart6, tx_buf, 75); //发送数据帧
			break;
		}
		case 5:
		{	
			//机器人详细调试信息
			ClientUI_DrawString( 9, "N1", UI_OPERATE_ADD, 2, UI_COLOR_WHITE, 625, 600, "AMO: 0");

			break;
		}
		case 6:
		{
			
		ClientUI_DrawString( 9, "N2", UI_OPERATE_ADD, 2, UI_COLOR_WHITE, 625, 560, "AutoAIM: 0");
			break;
		}
		case 7:
		{
          //set_line_graphics_data(tx_buf + 13, 2, "L03", UI_OPERATE_ADD, 5, UI_COLOR_RED_BLUE, 960, 140, 960, 520);
									//绘制横线
					//set_line_graphics_data(tx_buf + 28, 2, "L01", UI_OPERATE_ADD, 5, UI_COLOR_YELLOW, 930, 490, 990, 490);
					//set_line_graphics_data(tx_buf + 43,	2, "L02", UI_OPERATE_ADD, 5, UI_COLOR_YELLOW, 960, 490, 990, 490);//2.5
					//set_line_graphics_data(tx_buf + 58, 2, "L04"X, UI_OPERATE_ADD, 5, UI_COLOR_YELLOW, 960, 451, 995,451 );
									//set_line_graphics_data(tx_buf + 58, 2, "L05", UI_OPERATE_ADD, 5, UI_COLOR_RED_BLUE, 960, 413, 995, 413);
									//set_line_graphics_data(tx_buf + 58, 2, "L06", UI_OPERATE_ADD, 5, UI_COLOR_RED_BLUE, 960, 384, 995, 384);
									//set_line_graphics_data(tx_buf + 58, 2, "L07", UI_OPERATE_ADD, 5, UI_COLOR_RED_BLUE, 960, 420, 995, 420);//
			set_line_graphics_data(tx_buf + 13, 2, "L01", UI_OPERATE_ADD, 2, UI_COLOR_YELLOW, 950, 140, 950, 520);
			set_line_graphics_data(tx_buf + 28, 2, "L02", UI_OPERATE_ADD, 4, UI_COLOR_YELLOW, 960,  140, 960, 520);//改
			set_line_graphics_data(tx_buf + 43, 2, "L03", UI_OPERATE_ADD, 2, UI_COLOR_WHITE, 930, 460, 990,460 );//4
			set_line_graphics_data(tx_buf + 58, 2, "L04", UI_OPERATE_ADD, 2, UI_COLOR_WHITE, 930, 430, 990,430 );//5
			set_line_graphics_data(tx_buf + 73, 2, "L05", UI_OPERATE_ADD, 2, UI_COLOR_WHITE, 930, 400, 990,400 );//6
			set_line_graphics_data(tx_buf + 88, 2, "L06", UI_OPERATE_ADD, 2, UI_COLOR_WHITE, 930, 370, 990,370 );//7
			set_line_graphics_data(tx_buf + 103, 2, "L07", UI_OPERATE_ADD, 2, UI_COLOR_WHITE, 930, 296, 990,296 );//8P
			
			create_ui_interactive_package(tx_buf,120, 0x0104);
			Transmit_DMA(&huart6, tx_buf, 120); //发送数据帧p
			break;
		}
//        //case 8:
//        {
//				//set_line_graphics_data(tx_buf + 13, 2, "L08", UI_OPERATE_ADD, 5, UI_COLOR_RED_BLUE, 960, 315, 990, 315);
//				//set_line_graphics_data(tx_buf + 28, 2, "L09", UI_OPERATE_ADD, 5, UI_COLOR_RED_BLUE, 960, 275, 990, 275);
//				//set_line_graphics_data(tx_buf + 43,	2, "L10", UI_OPERATE_ADD, 5, UI_COLOR_RED_BLUE, 960, 225, 990, 225);
//				//set_line_graphics_data(tx_buf + 58, 2, "L11", UI_OPERATE_ADD, 5, UI_COLOR_RED_BLUE, 960, 236, 990, 236);
//                //set_line_graphics_data(tx_buf + 58, 2, "L12", UI_OPERATE_ADD, 5, UI_COLOR_RED_BLUE, 960, 217, 990, 217);
//                //set_line_graphics_data(tx_buf + 58, 2, "L13", UI_OPERATE_ADD, 5, UI_COLOR_RED_BLUE, 960, 207, 990, 207);
//               // set_line_graphics_data(tx_buf + 58, 2, "L14", UI_OPERATE_ADD, 5, UI_COLOR_RED_BLUE, 960, 197, 990, 197);
//			//create_ui_interactive_package(tx_buf, 120, 0x0104);
//			//Transmit_DMA(&huart6, tx_buf, 120); //发送数据帧
//			//break;  
//        }
        case 8:
        {
            ClientUI_DrawString( 9, "N3", UI_OPERATE_ADD, 2, UI_COLOR_RED_BLUE, 625, 520, "C_SPEED: %.2f",robot.chassis_task_p->get_max_speed());
			break;
        }
				case 9:
				{
//					if(robot.ammo_task_p->getExp_Spd() <15)
//					ClientUI_DrawString( 9, "N4", UI_OPERATE_ADD, 2, UI_COLOR_RED_BLUE, 70, 680, "AMMO_SPEED: 10");
//					else
//					ClientUI_DrawString( 9, "N4", UI_OPERATE_ADD, 2, UI_COLOR_RED_BLUE, 70, 680, "AMMO_SPEED: 16");	
					ClientUI_DrawString( 9, "N4", UI_OPERATE_ADD, 2, UI_COLOR_RED_BLUE, 625, 480, "SPEED: %.2f",robot.ammo_task_p->getExp_Spd());
					break;
				}
				case  10:
				{
					ClientUI_DrawString( 9, "N5", UI_OPERATE_ADD, 2, UI_COLOR_RED_BLUE, 625, 440, "GOAL: %.2f",robot.gimbal_task_p->get_goal_distance());
					break;
				}
				 case  11:
				{
					ClientUI_DrawString( 2, "N7", UI_OPERATE_ADD, 2, UI_COLOR_RED_BLUE, 625, 400, "AIM_MODE: %d",robot.getRemoteControlTask()->autoaim_flag);
					break;
				}
				
				case 12:
				{
					ClientUI_DrawString( 9, "N6", UI_OPERATE_ADD, 2, UI_COLOR_YELLOW, 625, 640, "CHASSIS_FOLLOW");
					break;
				}
				case 13:
				{
					ClientUI_DrawString( 9, "N8", UI_OPERATE_ADD, 2, UI_COLOR_PINK, 1295,640, "PITCH_ANGLE:%.2f",(-robot.gimbal_task_p->motor_pitch_pid_task->motor_backend_p->getCommonMeasurement().rotator.polar_angle)/PI*180.0+4.57);
					break;
				}
				case 14:
				{
					ClientUI_DrawString( 9, "N9", UI_OPERATE_ADD, 2, UI_COLOR_PINK, 625,360, "45ToEnenmy: %d",robot.chassis_task_p->get_Chassis_45toenemy_flag());
					t++;
				}
		default:
		{
			 RefreshDynamicClientUI();  
			break;
		}
	}
	
	if(num > 14) 
	{
		if(t <= 2) num = 3;
	}
	else num ++;
	
}
/***********************************************************************
** 函 数 名： RefreshDynamicClientUI()
** 函数说明： 刷新动态客户端UI界面
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void RefereeSystemTask::RefreshDynamicClientUI(void)
{
	static uint16_t state = 0;
	
	switch(state)
	{
		case 0:
		{
			//"AMO"UI字符
			if(robot.ammo_task_p->get_Booster_on_flag())
			{
				ClientUI_DrawString(9, (char *)"N1", UI_OPERATE_UPDATE, 2, UI_COLOR_YELLOW, 625, 600, "AMO:1");
				robot.ammo_task_p->clear_Booster_on_flag();
			}
			else if(robot.ammo_task_p->get_Booster_off_flag())
			{
				ClientUI_DrawString(9, (char *)"N1", UI_OPERATE_UPDATE, 2, UI_COLOR_WHITE, 625, 600, "AMO:0");
				robot.ammo_task_p->clear_Booster_off_flag();
			}
			break;
		}
		case 1:
		{
			//"AutoAIM"UI字符
			if(robot.gimbal_task_p->get_GIMBAL_RC_MODE_flag())
			{
				ClientUI_DrawString(9, (char *)"N2", UI_OPERATE_UPDATE, 2, UI_COLOR_YELLOW, 625, 560, "AutoAIM:1");
				robot.gimbal_task_p->clear_GIMBAL_RC_MODE_flag();
			}
			else if(robot.gimbal_task_p->get_GIMBAL_AUTO_AIMING_MODE_flag())
			{
				ClientUI_DrawString(9, (char *)"N2", UI_OPERATE_UPDATE, 2, UI_COLOR_WHITE, 625, 560, "AutoAIM:0");
				robot.gimbal_task_p->clear_GIMBAL_AUTO_AIMING_MODE_flag();
			}
			break;
		}
		case 2:
		{
			//底盘
			if(robot.chassis_task_p->get_Chassis_OFF_flag())
			{
				ClientUI_DrawLine(5, (char *)"OG1", UI_OPERATE_UPDATE, 20, UI_COLOR_VIOLETRED, 500, 0,  1420, 0);
				robot.chassis_task_p->clear_Chassis_OFF_flag();
			}
			else if(robot.chassis_task_p->get_Chassis_FOLLOW_flag())
			{
				ClientUI_DrawLine(5, (char *)"OG1", UI_OPERATE_UPDATE, 20, UI_COLOR_GREEN, 500, 0,  1420, 0);
				robot.chassis_task_p->clear_Chassis_FOLLOW_flag();
			}
			else if(robot.chassis_task_p->get_Chassis_TOP_flag())
			{
				ClientUI_DrawLine(5, (char *)"OG1", UI_OPERATE_UPDATE, 20, UI_COLOR_PINK, 500, 0,  1420, 0);
//				robot.chassis_task_p->clear_Chassis_TOP_flag();
			}
			break;
		}
		case 3:
		{
			//超级电容能量条更新
			if(robot.chassis_task_p->get_suercap_RX_data_pack().supercap_energy_percent > 20)
			{
				ClientUI_DrawLine(5, (char *)"OS0", UI_OPERATE_UPDATE, 15,UI_COLOR_BLUEGREEN, 700, 25, 700+5*robot.chassis_task_p->get_supercap_persent_lf()*1.0f,25);
			}
			else 
			{
				//ClientUI_DrawLine( 5, (char *)"OS0", UI_OPERATE_UPDATE, 15, UI_COLOR_ORANGE, 700, 25, 700+5*robot.chassis_task_p->get_suercap_RX_data_pack().supercap_energy_percent*0.95f,25);
				ClientUI_DrawLine( 5, (char *)"OS0", UI_OPERATE_UPDATE, 15, UI_COLOR_ORANGE, 700, 25, 700+5*robot.chassis_task_p->get_supercap_persent_lf()*1.0f,25);
			}
			break;
		}
        case 4:
        {
             ClientUI_DrawString( 9, "N3", UI_OPERATE_UPDATE, 2, UI_COLOR_RED_BLUE, 625, 520, "C_SPEED: %.2f",robot.chassis_task_p->get_max_speed());
            break;
        }
				case 5:
				{
					ClientUI_DrawString( 9, "N4", UI_OPERATE_UPDATE, 2, UI_COLOR_RED_BLUE, 625, 480, "SPEED: %.1f",robot.ammo_task_p->getExp_Spd());
					break;
				}
				case 6:
				{
					ClientUI_DrawString( 9, "N5", UI_OPERATE_UPDATE, 2, UI_COLOR_RED_BLUE, 625, 440, "GOAL: %.2f",robot.gimbal_task_p->get_goal_distance());
					break;
				}
				case 7:
				{
					if(robot.chassis_task_p->get_Chassis_TOP_flag())
					{
						ClientUI_DrawString( 9, "N6", UI_OPERATE_UPDATE, 2, UI_COLOR_RED_BLUE, 625, 640, "CHASSIS:TOP");
					}
					else
					{
						ClientUI_DrawString( 9, "N6", UI_OPERATE_UPDATE, 2, UI_COLOR_YELLOW, 625, 640, "CHASSIS:FOLLOW");
					}
					break;
				}
				case 8:
				{
					ClientUI_DrawString(2, "N7", UI_OPERATE_UPDATE, 2, UI_COLOR_RED_BLUE, 625, 400, "AIM_MODE: %d",robot.getRemoteControlTask()->autoaim_flag);
					break;
				}
				case 9:
				{
					ClientUI_DrawString( 9, "N8", UI_OPERATE_UPDATE, 2, UI_COLOR_YELLOW, 1295	, 640, "PITCH_ANGLE:%.2f",(- robot.gimbal_task_p->motor_pitch_pid_task->motor_backend_p->getCommonMeasurement().rotator.polar_angle)/PI*180.0+4.57);
					break;
				}
					case 10:
				{
					ClientUI_DrawString( 9, "N9", UI_OPERATE_UPDATE, 2, UI_COLOR_PINK, 625,360, "45ToEnenmy: %d",robot.chassis_task_p->get_Chassis_45toenemy_flag());
					break;
				}
		default: break;
	}
	
	
		
	if(state >= 10) state = 0;
	else state +=1;
	
	
	
}