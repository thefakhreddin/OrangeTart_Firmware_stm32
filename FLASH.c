#include "FLASH.h"

void FLASH_Init(void)
{
	 /* One wait state, 24 MHz < SYSCLK (32MHz) <= 48 MHz */
	FLASH->ACR |= FLASH_ACR_LATENCY;
}
