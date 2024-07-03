#pragma once

#include "OffboardLink.h"

#ifdef DEBUG_MODE
#include "UARTDriver.h"
#endif

namespace olk
{
	struct IMUData : public MessageBase
	{
		public:
		int16_t accel[3]; // 24g
		int16_t gyro[3]; // 2000dps
		float w;
		float x;
		float y;
		float z;
			
		IMUData() : MessageBase(0x01, 20)
		{
				
		}
			
		virtual void decode(uint8_t *buf) override
		{
//			for(uint8_t i = 0; i < 4; i ++)
//			{
//				h.u[i] = buf[OFFBOARDLINK_FRAME_HEAD_LEN + i];
//			}
//			x = h.f;
//			for(uint8_t i = 0; i < 4; i ++)
//			{
//				h.u[i] = buf[OFFBOARDLINK_FRAME_HEAD_LEN + i + 4];
//			}
//			y = h.f;
//			for(uint8_t i = 0; i < 4; i ++)
//			{
//				h.u[i] = buf[OFFBOARDLINK_FRAME_HEAD_LEN + i + 8];
//			}
//			z = h.f;
//			
//			frame = buf[OFFBOARDLINK_FRAME_HEAD_LEN + 12];
//			
//			#ifdef DEBUG_MODE
//			USART1_DMA_Debug_Printf("0x%x %f %f %f\r", frame, x, y, z);
//			#endif
		}
			
		virtual void packData(uint8_t *buf) override
		{
			int16_t quaternion[4];
			quaternion[0] = w * 32760.0f;
			quaternion[1] = x * 32760.0f;
			quaternion[2] = y * 32760.0f;
			quaternion[3] = z * 32760.0f;
			
			buf[4]  = accel[0];
			buf[5]  = accel[0] >> 8;
			buf[6]  = accel[1];
			buf[7]  = accel[1] >> 8;
			buf[8]  = accel[2];
			buf[9]  = accel[2] >> 8;

			buf[10] = gyro[0];
			buf[11] = gyro[0] >> 8;
			buf[12] = gyro[1];
			buf[13] = gyro[1] >> 8;
			buf[14] = gyro[2];
			buf[15] = gyro[2] >> 8;

			buf[16] = quaternion[0];
			buf[17] = quaternion[0] >> 8;
			buf[18] = quaternion[1];
			buf[19] = quaternion[1] >> 8;
			buf[20] = quaternion[2];
			buf[21] = quaternion[2] >> 8;
			buf[22] = quaternion[3];
			buf[23] = quaternion[3] >> 8;			
		}
	};
}
