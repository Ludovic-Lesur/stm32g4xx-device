/*
 * stm32g484xexx.ld
 *
 *  Created on: 02 feb. 2025
 *      Author: ARM
 */

__flash_size__ = 512K;
__ccmsram_size__ = 32K;
__sram1_size__ = 80K;
__sram2_address__ = 0x20014000;
__sram2_size__ = 16K;

INCLUDE "stm32g4xx.ld"