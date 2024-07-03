/*****************************************************************
**                大连理工大学 凌BUG战队
**    没有什么难题是一个通宵解决不了的，如果有那就整两个！
**---------------------------------------------------------------
** 项目名称：   mxdemo_std_robot
** 日    期：   2021-04-15
** 作    者：   MasterWang
**---------------------------------------------------------------
** 文 件 名：   referee_system.h
** 文件说明：   裁判系统协议解析和客户端UI绘制
*****************************************************************/
#ifndef REFEREE_SYSTEM_H
#define REFEREE_SYSTEM_H

/*---------------------INCLUDES----------------------*/
#include "main.h"

#include <stdio.h>
#include <stdarg.h>

/*---------------------DEFINES-----------------------*/
#define REFEREE_FIFO_BUF_NUM				8
#define REFEREE_FIFO_BUF_LEN 				1024

//ID
#define RED_HERO_1_ROBOT						1
#define RED_ENGINEER_2_ROBOT				2
#define RED_INFANTRY_3_ROBOT				3
#define RED_INFANTRY_4_ROBOT				4
#define RED_INFANTRY_5_ROBOT				5
#define RED_AERIAL_6_ROBOT					6
#define RED_SENTRY_7_ROBOT					7
#define RED_DARTLAUNCH_8						8
#define RED_RADAR_9									9

#define RED_HERO_1_CLIENT						0x0101		//257
#define RED_ENGINEER_2_CLIENT				0x0102		//258
#define RED_INFANTRY_3_CLIENT				0x0103		//259
#define RED_INFANTRY_4_CLIENT				0x0104		//260
#define RED_INFANTRY_5_CLIENT				0x0105		//261
#define RED_AERIAL_6_CLIENT					0x0106		//262

#define BLUE_HERO_1_ROBOT						101
#define BLUE_ENGINEER_2_ROBOT				102
#define BLUE_INFANTRY_3_ROBOT				103
#define BLUE_INFANTRY_4_ROBOT				104
#define BLUE_INFANTRY_5_ROBOT				105
#define BLUE_AERIAL_6_ROBOT					106
#define BLUE_SENTRY_7_ROBOT					107
#define BLUE_DARTLAUNCH_8						108
#define BLUE_RADAR_9								109

#define BLUE_HERO_1_CLIENT					0x0165		//357
#define BLUE_ENGINEER_2_CLIENT			0x0166		//358
#define BLUE_INFANTRY_3_CLIENT			0x0167		//359
#define BLUE_INFANTRY_4_CLIENT			0x0168		//360
#define BLUE_INFANTRY_5_CLIENT			0x0169		//361
#define BLUE_AERIAL_6_CLIENT				0x016A		//362

//CLIENT UI
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

/*---------------------STRUCTS-----------------------*/
//帧头
typedef __packed struct
{
	uint8_t  sof;
	uint16_t data_length;
	uint8_t  seq;
	uint8_t  crc8;
} frame_header_t;

//0x0001,1Hz,比赛状态数据
typedef __packed struct
{ 
	uint32_t recv_cnt;
	uint8_t game_type:4; 
	uint8_t game_progress:4; 
	uint16_t stage_remain_time; 
	uint64_t SyncTimeStamp;
} ext_game_status_t;

//0x0002,比赛结束后发送,比赛结果数据
typedef __packed struct
{ 
	uint32_t recv_cnt;
	uint8_t winner; 
} ext_game_result_t;

//0x0003,1Hz,比赛机器人血量数据
typedef __packed struct
{ 
	uint32_t recv_cnt;
	uint16_t red_1_robot_HP; 
	uint16_t red_2_robot_HP; 
	uint16_t red_3_robot_HP; 
	uint16_t red_4_robot_HP; 
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP; 
	uint16_t red_outpost_HP; 
	uint16_t red_base_HP; 
	uint16_t blue_1_robot_HP; 
	uint16_t blue_2_robot_HP; 
	uint16_t blue_3_robot_HP; 
	uint16_t blue_4_robot_HP; 
	uint16_t blue_5_robot_HP; 
	uint16_t blue_7_robot_HP; 
	uint16_t blue_outpost_HP; 
	uint16_t blue_base_HP; 
} ext_game_robot_HP_t;


//0x0101,事件改变后发送,场地事件数据
typedef __packed struct
{
	uint32_t recv_cnt;
	uint32_t event_type; 
} ext_event_data_t;

//0x0102,动作改变后发送,场地补给站动作标识数据
typedef __packed struct
{
	uint32_t recv_cnt;
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id; 
	uint8_t supply_projectile_step; 
	uint8_t supply_projectile_num; 
} ext_supply_projectile_action_t;

//0x0104,警告发生后发送,裁判警告信息
typedef __packed struct
{ 
	uint32_t recv_cnt;
	uint8_t level; 
	uint8_t foul_robot_id; 
} ext_referee_warning_t;


//0x0201,10Hz,机器人状态数据
typedef __packed struct
{ 
	uint32_t recv_cnt;
	uint8_t robot_id; 
	uint8_t robot_level; 
	uint16_t remain_HP; 
	uint16_t max_HP; 
	uint16_t shooter_id1_42mm_cooling_rate; 
	uint16_t shooter_id1_42mm_cooling_limit;
	uint16_t shooter_id1_42mm_speed_limit;
	uint16_t chassis_power_limit; 
	uint8_t mains_power_gimbal_output:1; 
	uint8_t mains_power_chassis_output:1; 
	uint8_t mains_power_shooter_output:1; 
} ext_game_robot_status_t;

//0x0202,50Hz,实时功率热量数据
typedef __packed struct
{ 
	uint32_t recv_cnt;
	uint16_t chassis_volt; 
	uint16_t chassis_current; 
	float chassis_power; 
	uint16_t chassis_power_buffer; 
	uint16_t shooter_id1_42mm_cooling_heat; 
} ext_power_heat_data_t;

//0x0203,10Hz,机器人位置数据
typedef __packed struct
{ 
	uint32_t recv_cnt;
	float x; 
	float y; 
	float z; 
	float yaw; 
} ext_game_robot_pos_t;

//0x0204,增益状态改变后发送,机器人增益数据
typedef __packed struct
{ 
	uint32_t recv_cnt;
	uint8_t power_rune_buff; 
} ext_buff_t;

//0x0206,伤害发生后发送,伤害状态数据
typedef __packed struct
{ 
	uint32_t recv_cnt;
	uint8_t armor_id:4; 
	uint8_t hurt_type:4; 
} ext_robot_hurt_t;

//0x0207,子弹发射后发送,实时射击数据
typedef __packed struct
{ 
	uint32_t recv_cnt,recv_cnt_last;
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq; 
	float bullet_speed; 
	float bullet_speed_last;
} ext_shoot_data_t;

//0x0208,1Hz,子弹剩余发射数
typedef __packed struct
{ 
	uint32_t recv_cnt;
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

//0x0209,1Hz,机器人RFID状态数据
typedef __packed struct
{ 
	uint32_t recv_cnt;
	uint32_t rfid_status;
} ext_rfid_status_t;

//机器人裁判系统获取总状态数据
typedef __packed struct
{
	ext_game_status_t game_status; //0x0001
	ext_game_result_t game_result; //0x0002
	ext_game_robot_HP_t game_robot_HP; //0x0003
	
	ext_event_data_t event_data; //0x0101
	ext_supply_projectile_action_t supply_projectile_action; //0x0102
	ext_referee_warning_t referee_warning; //0x0104
	
	ext_game_robot_status_t game_robot_status; //0x0201
	ext_power_heat_data_t power_heat_data; //0x0202
	ext_game_robot_pos_t game_robot_pos; //0x0203
	ext_buff_t buff; //0x0204

	ext_robot_hurt_t robot_hurt; //0x0206
	ext_shoot_data_t shoot_data; //0x0207
	ext_bullet_remaining_t bullet_remaining; //0x0208
	ext_rfid_status_t rfid_status; //0x0209

	
	uint32_t cmd_error_count[6]; //0-seq_num error; 1-frame_header & length error; 2-head_crc8 error; 3-package length error; 4-package_crc16 error; 5-cmd error
	
} RobotRefereeStatus_t;


/*---------------------DECLARES----------------------*/
//
extern RobotRefereeStatus_t robot_referee_status;
//extern ClientUIStatus_t client_ui_status;

//
void PushToRefereeFIFOBuf(uint8_t *pdata, uint16_t max_size, uint16_t start_pos, uint16_t end_pos);
void ParseRefereeSystemData(void);

//
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8);
uint32_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif
