#pragma once

#include "OffboardLink.h"

#ifdef DEBUG_MODE
#include "UARTDriver.h"
#endif

namespace olk
{
	struct LockTargetSuggest : MessageBase
	{
		public:
		uint8_t lock_target;
			
		LockTargetSuggest() : MessageBase(0x88, 1)
		{
				
		}
			
		virtual void decode(uint8_t *buf) override
		{
	
		}
			
		virtual void packData(uint8_t *buf) override
		{
			buf[4] = lock_target;
		}
	};
}
