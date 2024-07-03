/*****************************************************************
**                大连理工大学 凌BUG战队
**    没有什么难题是一个通宵解决不了的，如果有那就整两个！
**---------------------------------------------------------------
** 项目名称：   mxdemo_std_robot
** 日    期：   2021-04-21
** 作    者：   MasterWang
**---------------------------------------------------------------
** 文 件 名：   referee_system.c
** 文件说明：   裁判系统协议解析和客户端UI绘制
*****************************************************************/

/*---------------------INCLUDES----------------------*/
#include "referee_system.h"
#include "usart.h"
#include "stdlib.h"
#include "string.h"

uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_NUM][REFEREE_FIFO_BUF_LEN] ;//__attribute__((section(".ARM.__at_0x10005000"))) = {0}; //8*1KB

uint8_t fifo_count ;//	__attribute__((section(".ARM.__at_0x10007C00")));//			__AT(0x10002000) = 0;
uint8_t fifo_head_pos ;//__attribute__((section(".ARM.__at_0x10007C04"))) = 0;
uint8_t fifo_tail_pos ;//__attribute__((section(".ARM.__at_0x10007C08"))) = 0;

uint8_t referee_seq_num ;//__attribute__((section(".ARM.__at_0x10007C10"))) = 0;
uint8_t student_interactive_seq;// __attribute__((section(".ARM.__at_0x10007C14"))) = 0;


RobotRefereeStatus_t robot_referee_status;// __attribute__((section(".ARM.__at_0x10007000"))) = {0}; //1KB


/***********************************************************************
** 函 数 名： PushToRefereeFIFOBuf()
** 函数说明： 从串口DMA的接收缓冲循环数组中提取数据帧,压入裁判系统协议解析缓冲队列(在UART中断调用)
**---------------------------------------------------------------------
** 输入参数： 串口DMA接收缓冲数组地址,数组最大长度,起始位置,结束位置
** 返回参数： 无
***********************************************************************/
void PushToRefereeFIFOBuf(uint8_t *pdata, uint16_t max_size, uint16_t start_pos, uint16_t end_pos)
{
	uint16_t size;
	
	if(end_pos>=start_pos){size=end_pos-start_pos;}
	else{size=max_size-(start_pos-end_pos);}
	
	if((size>0)&&(fifo_count<REFEREE_FIFO_BUF_NUM))
	{
		//
		referee_fifo_buf[fifo_tail_pos][0]=size>>8;
		referee_fifo_buf[fifo_tail_pos][1]=size;
		
		if(end_pos>start_pos){memcpy(referee_fifo_buf[fifo_tail_pos]+2, pdata+start_pos, size);}
		else{memcpy(referee_fifo_buf[fifo_tail_pos]+2, pdata+start_pos, max_size-start_pos); memcpy(referee_fifo_buf[fifo_tail_pos]+2+(max_size-start_pos), pdata, end_pos);}
		
		//
		fifo_tail_pos=(fifo_tail_pos+1)%REFEREE_FIFO_BUF_NUM;
		fifo_count++;
	}
}



/***********************************************************************
** 函 数 名： ParseRefereeSystemData()
** 函数说明： 裁判系统协议解析(在100Hz定时器中断调用)
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void ParseRefereeSystemData(void)
{
	int16_t remaining_n;
	
	uint8_t *pbuf;
	uint8_t *ptr; //用于float和uint64_t类型数据转换
	
	uint16_t data_len;
	uint16_t cmd;
	
	if(fifo_count>0)
	{
		//
		remaining_n = referee_fifo_buf[fifo_head_pos][0]*256+referee_fifo_buf[fifo_head_pos][1];
		pbuf = referee_fifo_buf[fifo_head_pos]+2;
		
		while(1)
		{
			//
     if((pbuf[0]==0xA5)&&(remaining_n>=5)) //check frame_header & length
			{
				if(Verify_CRC8_Check_Sum(pbuf,5)) //check head_crc8
				{
					data_len=pbuf[2]*256+pbuf[1];
					if(pbuf[3]!=(uint8_t)(referee_seq_num+1)){robot_referee_status.cmd_error_count[0]++;} //seq_num error
					referee_seq_num=pbuf[3];
				
					//
					if(remaining_n>=data_len+9) //5+2+len+2 //check package length
					{
						if(Verify_CRC16_Check_Sum(pbuf,data_len+9)) //check package_crc16
						{
							cmd = pbuf[6]*256+pbuf[5];
							
							//
							if(cmd==0x0201) //10Hz,机器人状态数据
							{
								robot_referee_status.game_robot_status.recv_cnt++;
								robot_referee_status.game_robot_status.robot_id = pbuf[7];
								robot_referee_status.game_robot_status.robot_level = pbuf[8];
								robot_referee_status.game_robot_status.remain_HP = pbuf[10]*256+pbuf[9];
								robot_referee_status.game_robot_status.max_HP = pbuf[12]*256+pbuf[11];
//								robot_referee_status.game_robot_status.shooter_id1_17mm_cooling_rate = pbuf[14]*256+pbuf[13];
//								robot_referee_status.game_robot_status.shooter_id1_17mm_cooling_limit = pbuf[16]*256+pbuf[15];
//								robot_referee_status.game_robot_status.shooter_id1_17mm_speed_limit = pbuf[18]*256+pbuf[17];
//								robot_referee_status.game_robot_status.shooter_id2_17mm_cooling_rate = pbuf[20]*256+pbuf[19];
//								robot_referee_status.game_robot_status.shooter_id2_17mm_cooling_limit = pbuf[22]*256+pbuf[21];
//								robot_referee_status.game_robot_status.shooter_id2_17mm_speed_limit = pbuf[24]*256+pbuf[23];
								robot_referee_status.game_robot_status.shooter_id1_42mm_cooling_rate = pbuf[26]*256+pbuf[25];
								robot_referee_status.game_robot_status.shooter_id1_42mm_cooling_limit = pbuf[28]*256+pbuf[27];
								robot_referee_status.game_robot_status.shooter_id1_42mm_speed_limit = pbuf[30]*256+pbuf[29];
								robot_referee_status.game_robot_status.chassis_power_limit = pbuf[32]*256+pbuf[31];
								robot_referee_status.game_robot_status.mains_power_gimbal_output = pbuf[33];
								robot_referee_status.game_robot_status.mains_power_chassis_output = pbuf[33]>>1;
								robot_referee_status.game_robot_status.mains_power_shooter_output = pbuf[33]>>2;					
							}
							else if(cmd==0x0202) //50Hz,实时功率热量数据
							{
								robot_referee_status.power_heat_data.recv_cnt++;
								robot_referee_status.power_heat_data.chassis_volt = pbuf[8]*256+pbuf[7];
								robot_referee_status.power_heat_data.chassis_current = pbuf[10]*256+pbuf[9];
								ptr=(uint8_t *)&robot_referee_status.power_heat_data.chassis_power; ptr[0]=pbuf[11]; ptr[1]=pbuf[12]; ptr[2]=pbuf[13]; ptr[3]=pbuf[14];
								robot_referee_status.power_heat_data.chassis_power_buffer = pbuf[16]*256+pbuf[15];
//								robot_referee_status.power_heat_data.shooter_id1_17mm_cooling_heat = pbuf[18]*256+pbuf[17];
//								robot_referee_status.power_heat_data.shooter_id2_17mm_cooling_heat = pbuf[20]*256+pbuf[19];
								robot_referee_status.power_heat_data.shooter_id1_42mm_cooling_heat = pbuf[22]*256+pbuf[21];
							}
							else if(cmd==0x0203) //10Hz,机器人位置数据
							{
								robot_referee_status.game_robot_pos.recv_cnt++;
								ptr=(uint8_t *)&robot_referee_status.game_robot_pos.x; ptr[0]=pbuf[7]; ptr[1]=pbuf[8]; ptr[2]=pbuf[9]; ptr[3]=pbuf[10];
								ptr=(uint8_t *)&robot_referee_status.game_robot_pos.y; ptr[0]=pbuf[11]; ptr[1]=pbuf[12]; ptr[2]=pbuf[13]; ptr[3]=pbuf[14];
								ptr=(uint8_t *)&robot_referee_status.game_robot_pos.z; ptr[0]=pbuf[15]; ptr[1]=pbuf[16]; ptr[2]=pbuf[17]; ptr[3]=pbuf[18];
								ptr=(uint8_t *)&robot_referee_status.game_robot_pos.yaw; ptr[0]=pbuf[19]; ptr[1]=pbuf[20]; ptr[2]=pbuf[21]; ptr[3]=pbuf[22];
							}
							else if(cmd==0x0204) //1Hz,机器人增益数据
							{
								robot_referee_status.buff.recv_cnt++;
								robot_referee_status.buff.power_rune_buff = pbuf[7];
							}
							else if(cmd==0x0205) //10Hz,空中机器人能量状态数据,只有空中机器人主控发送
							{
//								robot_referee_status.aerial_robot_energy.recv_cnt++;
//								robot_referee_status.aerial_robot_energy.attack_time = pbuf[7];
							}
							else if(cmd==0x0206) //伤害发生后发送,伤害状态数据
							{
								robot_referee_status.robot_hurt.recv_cnt++;
								robot_referee_status.robot_hurt.armor_id = pbuf[7];
								robot_referee_status.robot_hurt.hurt_type = pbuf[7]>>4;
							}
							else if(cmd==0x0207) //子弹发射后发送,实时射击数据
							{
								robot_referee_status.shoot_data.recv_cnt++;
								robot_referee_status.shoot_data.bullet_type = pbuf[7];
								robot_referee_status.shoot_data.shooter_id = pbuf[8];
								robot_referee_status.shoot_data.bullet_freq = pbuf[9];
								ptr=(uint8_t *)&robot_referee_status.shoot_data.bullet_speed; ptr[0]=pbuf[10]; ptr[1]=pbuf[11]; ptr[2]=pbuf[12]; ptr[3]=pbuf[13];
							}
							else if(cmd==0x0208) //1Hz,子弹剩余发射数
							{
								robot_referee_status.bullet_remaining.recv_cnt++;
//								robot_referee_status.bullet_remaining.bullet_remaining_num_17mm = pbuf[8]*256+pbuf[7];
								robot_referee_status.bullet_remaining.bullet_remaining_num_42mm = pbuf[10]*256+pbuf[9];
								robot_referee_status.bullet_remaining.coin_remaining_num = pbuf[12]*256+pbuf[11];
							}
							else if(cmd==0x0209) //1Hz,机器人RFID状态数据
							{
								robot_referee_status.rfid_status.recv_cnt++;
								robot_referee_status.rfid_status.rfid_status = pbuf[9]; //(pbuf[12]<<24)+(pbuf[11]<<16)+(pbuf[10]<<8)+pbuf[9];
							}
							else if(cmd==0x020A) //10Hz,飞镖机器人客户端指令数据
							{
//								robot_referee_status.dart_client_cmd.recv_cnt++;
//								robot_referee_status.dart_client_cmd.dart_launch_opening_status = pbuf[7];
//								robot_referee_status.dart_client_cmd.dart_attack_target = pbuf[8];
//								robot_referee_status.dart_client_cmd.target_change_time = pbuf[10]*256+pbuf[9];
//								robot_referee_status.dart_client_cmd.first_dart_speed = pbuf[11];
//								robot_referee_status.dart_client_cmd.second_dart_speed = pbuf[12];
//								robot_referee_status.dart_client_cmd.third_dart_speed = pbuf[13];
//								robot_referee_status.dart_client_cmd.fourth_dart_speed = pbuf[14];
//								robot_referee_status.dart_client_cmd.last_dart_launch_time = pbuf[16]*256+pbuf[15];
//								robot_referee_status.dart_client_cmd.operate_launch_cmd_time = pbuf[18]*256+pbuf[17];
							}
							else if(cmd==0x0001) //1Hz,比赛状态数据
							{
								robot_referee_status.game_status.recv_cnt++;
								robot_referee_status.game_status.game_type = pbuf[7];
								robot_referee_status.game_status.game_progress = pbuf[7]>>4;
								robot_referee_status.game_status.stage_remain_time = pbuf[9]*256+pbuf[8];
								ptr=(uint8_t *)&robot_referee_status.game_status.SyncTimeStamp; ptr[0]=pbuf[10]; ptr[1]=pbuf[11]; ptr[2]=pbuf[12]; ptr[3]=pbuf[13]; ptr[4]=pbuf[14]; ptr[5]=pbuf[15]; ptr[6]=pbuf[16]; ptr[7]=pbuf[17];
							}
							else if(cmd==0x0002) //比赛结束后发送,比赛结果数据
							{
								robot_referee_status.game_result.recv_cnt++;
								robot_referee_status.game_result.winner = pbuf[7];
							}
							else if(cmd==0x0003) //1Hz,比赛机器人血量数据
							{
								robot_referee_status.game_robot_HP.recv_cnt++;
								robot_referee_status.game_robot_HP.red_1_robot_HP = pbuf[8]*256+pbuf[7];
								robot_referee_status.game_robot_HP.red_2_robot_HP = pbuf[10]*256+pbuf[9];
								robot_referee_status.game_robot_HP.red_3_robot_HP = pbuf[12]*256+pbuf[11];
								robot_referee_status.game_robot_HP.red_4_robot_HP = pbuf[14]*256+pbuf[13];
								robot_referee_status.game_robot_HP.red_5_robot_HP = pbuf[16]*256+pbuf[15];
								robot_referee_status.game_robot_HP.red_7_robot_HP = pbuf[18]*256+pbuf[17];
								robot_referee_status.game_robot_HP.red_outpost_HP = pbuf[20]*256+pbuf[19];
								robot_referee_status.game_robot_HP.red_base_HP = pbuf[22]*256+pbuf[21];
								robot_referee_status.game_robot_HP.blue_1_robot_HP = pbuf[24]*256+pbuf[23];
								robot_referee_status.game_robot_HP.blue_2_robot_HP = pbuf[26]*256+pbuf[25];
								robot_referee_status.game_robot_HP.blue_3_robot_HP = pbuf[28]*256+pbuf[27];
								robot_referee_status.game_robot_HP.blue_4_robot_HP = pbuf[30]*256+pbuf[29];
								robot_referee_status.game_robot_HP.blue_5_robot_HP = pbuf[32]*256+pbuf[31];
								robot_referee_status.game_robot_HP.blue_7_robot_HP = pbuf[34]*256+pbuf[33];
								robot_referee_status.game_robot_HP.blue_outpost_HP = pbuf[36]*256+pbuf[35];
								robot_referee_status.game_robot_HP.blue_base_HP = pbuf[38]*256+pbuf[37];
							}
							else if(cmd==0x0004) //飞镖发射后发送,飞镖发射状态数据
							{
//								robot_referee_status.dart_status.recv_cnt++;
//								robot_referee_status.dart_status.dart_belong = pbuf[7];
//								robot_referee_status.dart_status.stage_remaining_time = pbuf[9]*256+pbuf[8];
							}
							else if(cmd==0x0005) //1Hz,人工智能挑战赛加成与惩罚区状态
							{
//								robot_referee_status.ICRA_buff_debuff_zone_status.recv_cnt++;
//								robot_referee_status.ICRA_buff_debuff_zone_status.F1_zone_status = pbuf[7];
//								robot_referee_status.ICRA_buff_debuff_zone_status.F1_zone_buff_debuff_status = pbuf[7]>>1;
//								robot_referee_status.ICRA_buff_debuff_zone_status.F2_zone_status = pbuf[7]>>4;
//								robot_referee_status.ICRA_buff_debuff_zone_status.F2_zone_buff_debuff_status = pbuf[7]>>5;
//								robot_referee_status.ICRA_buff_debuff_zone_status.F3_zone_status = pbuf[8];
//								robot_referee_status.ICRA_buff_debuff_zone_status.F3_zone_buff_debuff_status = pbuf[8]>>1;
//								robot_referee_status.ICRA_buff_debuff_zone_status.F4_zone_status = pbuf[8]>>4;
//								robot_referee_status.ICRA_buff_debuff_zone_status.F4_zone_buff_debuff_status = pbuf[8]>>5;
//								robot_referee_status.ICRA_buff_debuff_zone_status.F5_zone_status = pbuf[9];
//								robot_referee_status.ICRA_buff_debuff_zone_status.F5_zone_buff_debuff_status = pbuf[9]>>1;
//								robot_referee_status.ICRA_buff_debuff_zone_status.F6_zone_status = pbuf[9]>>4;
//								robot_referee_status.ICRA_buff_debuff_zone_status.F6_zone_buff_debuff_status = pbuf[9]>>5;
//								
//								robot_referee_status.ICRA_buff_debuff_zone_status.red1_bullet_left = pbuf[11]*256+pbuf[10];
//								robot_referee_status.ICRA_buff_debuff_zone_status.red2_bullet_left = pbuf[13]*256+pbuf[12];
//								robot_referee_status.ICRA_buff_debuff_zone_status.blue1_bullet_left = pbuf[15]*256+pbuf[14];
//								robot_referee_status.ICRA_buff_debuff_zone_status.blue2_bullet_left = pbuf[17]*256+pbuf[16];
							}
							else if(cmd==0x0101) //1Hz,场地事件数据
							{
								robot_referee_status.event_data.recv_cnt++;
								robot_referee_status.event_data.event_type = (pbuf[10]<<24)+(pbuf[9]<<16)+(pbuf[8]<<8)+pbuf[7];
							}
							else if(cmd==0x0102) //动作改变后发送,场地补给站动作标识数据
							{
								robot_referee_status.supply_projectile_action.recv_cnt++;
								robot_referee_status.supply_projectile_action.supply_projectile_id = pbuf[7];
								robot_referee_status.supply_projectile_action.supply_robot_id = pbuf[8];
								robot_referee_status.supply_projectile_action.supply_projectile_step = pbuf[9];
								robot_referee_status.supply_projectile_action.supply_projectile_num = pbuf[10];
							}
							else if(cmd==0x0104) //警告发生后发送,裁判警告信息
							{
								robot_referee_status.referee_warning.recv_cnt++;
								robot_referee_status.referee_warning.level = pbuf[7];
								robot_referee_status.referee_warning.foul_robot_id = pbuf[8];
							}
							else if(cmd==0x0105) //1Hz,飞镖发射口倒计时
							{
//								robot_referee_status.dart_remaining_time.recv_cnt++;
//								robot_referee_status.dart_remaining_time.dart_remaining_time = pbuf[7];
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
			remaining_n=remaining_n-(data_len+9);
			pbuf=pbuf+(data_len+9);
			
			if(remaining_n<=0){break;}
		}
		
		//
		fifo_head_pos=(fifo_head_pos+1)%REFEREE_FIFO_BUF_NUM;
		fifo_count--;
	}
}

/*---------------------DJI提供CRC校验代码---------------------*/
//crc8 generator polynomial:G(x)=x8+x5+x4+1
const uint8_t CRC8_INIT = 0xff;
const uint8_t CRC8_TAB[256] = { 0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 
	0x20, 0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 
	0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 
	0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 
	0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 
	0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 
	0xfb, 0x78, 0x26, 0xc4, 0x9a, 0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 
	0x45, 0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 
	0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 
	0xac, 0x2f, 0x71, 0x93, 0xcd, 0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 
	0x31, 0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 
	0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 
	0x12, 0x91, 0xcf, 0x2d, 0x73, 0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 
	0xea, 0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 
	0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 
	0xc9, 0x4a, 0x14, 0xf6, 0xa8, 0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 
	0x54, 0xd7, 0x89, 0x6b, 0x35};

/* 
** Descriptions: CRC8 checksum function 
** Input: Data to check,Stream length, initialized checksum 
** Output: CRC checksum 
*/
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8)
{
	uint8_t ucIndex;
	while(dwLength--)
	{
		ucIndex=ucCRC8^(*pchMessage++);
		ucCRC8=CRC8_TAB[ucIndex];
	}
	return(ucCRC8); 
}

/*
** Descriptions: CRC8 Verify function 
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result) 
*/
uint32_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint8_t ucExpected;
	ucExpected=Get_CRC8_Check_Sum(pchMessage,dwLength-1,CRC8_INIT);
	return (ucExpected==pchMessage[dwLength-1]);
}

/* 
** Descriptions: append CRC8 to the end of data 
** Input: Data to CRC and append,Stream length = Data + checksum 
** Output: True or False (CRC Verify Result) 
*/
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{ 
	uint8_t ucCRC; 
	ucCRC=Get_CRC8_Check_Sum(pchMessage,dwLength-1,CRC8_INIT); 
	pchMessage[dwLength-1]=ucCRC; 
}

//crc16
const uint16_t CRC16_INIT = 0xffff;
const uint16_t CRC16_Table[256] = {0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 
	 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 
	 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 
	 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 
	 0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 
	 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 
	 0x98e9, 0x8960, 0xbbfb, 0xaa72, 0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 
	 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 
	 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 
	 0x4e64, 0x5fed, 0x6d76, 0x7cff, 0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 
	 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 
	 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 
	 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 
	 0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 
	 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 
	 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 
	 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/* 
** Descriptions: CRC16 checksum function 
** Input: Data to check,Stream length, initialized checksum 
** Output: CRC checksum 
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) 
{
	uint8_t chData; 
	while(dwLength--)
	{ 
		chData=*pchMessage++; 
		wCRC=(wCRC>>8)^CRC16_Table[(wCRC^(uint16_t)(chData))&0x00ff]; 
	} 
	return wCRC; 
}

/*
** Descriptions: CRC16 Verify function 
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result) 
*/ 
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) 
{ 
	uint16_t wExpected; 
	wExpected=Get_CRC16_Check_Sum(pchMessage,dwLength-2,CRC16_INIT);
	return ((wExpected&0x00ff)==pchMessage[dwLength-2]&&((wExpected>>8)&0x00ff)==pchMessage[dwLength-1]);
}

/* 
** Descriptions: append CRC16 to the end of data 
** Input: Data to CRC and append,Stream length = Data + checksum 
** Output: True or False (CRC Verify Result) 
*/ 
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength) 
{ 
	uint16_t wCRC;
	wCRC = Get_CRC16_Check_Sum(pchMessage,dwLength-2,CRC16_INIT); 
	pchMessage[dwLength-2]=(wCRC&0x00ff); 
	pchMessage[dwLength-1]=((wCRC>>8)&0x00ff);
}
