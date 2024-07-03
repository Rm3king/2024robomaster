#pragma once

#include "OffboardLink.h"

#ifdef DEBUG_MODE
#include "UARTDriver.h"
#endif

namespace olk
{
	struct ReferenceDataMini : MessageBase
	{
		public:
		uint8_t game_state;
		uint16_t remaining_time;
		uint8_t self_color;
		uint16_t self_health;
		float bullet_speed_meas;
		uint16_t shoot_heat;
		uint8_t hurt_by_gimbal;
		uint16_t remaining_bullet;
		uint8_t who_is_balance;
			
		ReferenceDataMini() : MessageBase(0x40, 29)
		{
				
		}
			
		virtual void decode(uint8_t *buf) override
		{
	
		}
			
		virtual void packData(uint8_t *buf) override
		{
			buf[4] = game_state;
			buf[5] = remaining_time;
			buf[6] = remaining_time >> 8;
			buf[7] = self_color;
			buf[8] = self_health;
			buf[9] = self_health >> 8;
			uint16_t bullet_speed_temp = bullet_speed_meas / 50.0 * 32760.0;
			buf[10] = bullet_speed_temp;
			buf[11] = bullet_speed_temp >> 8;
			buf[12] = shoot_heat;
			buf[13] = shoot_heat >> 8;
			buf[14] = hurt_by_gimbal;
			buf[15] = remaining_bullet;
			buf[16] = remaining_bullet >> 8;
			buf[17] = who_is_balance;
		}
	};
}
