/*
 * stm32g441xbxx.ld
 *
 *  Created on: 02 feb. 2025
 *      Author: Ludo
 */

__flash_size__ = 128K;
__ccmsram_size__ = 10K;
__sram1_size__ = 16K;
__sram2_address__ = 0x20004000;
__sram2_size__ = 6K;

INCLUDE "stm32g4xx.ld"