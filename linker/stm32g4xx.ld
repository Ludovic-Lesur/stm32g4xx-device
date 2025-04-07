/*
 * stm32g441xbxx.ld
 *
 *  Created on: 04 apr. 2025
 *      Author: Ludo
 */

__flash_page_size__ = 2K;

MEMORY
{
    /* Last FLASH page is reserved for EEPROM emulation */
    FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = (__flash_size__ - __flash_page_size__)
    EEPROM  (rw)  : ORIGIN = (ORIGIN(FLASH) + LENGTH(FLASH)), LENGTH = __flash_page_size__
    CCMSRAM (rwx) : ORIGIN = 0x10000000, LENGTH = __ccmsram_size__
    SRAM1   (rwx) : ORIGIN = 0x20000000, LENGTH = __sram1_size__
    SRAM2   (rwx) : ORIGIN = __sram2_address__, LENGTH = __sram2_size__
}

__flash_address__ = ORIGIN(FLASH);
__eeprom_page_address__ = ORIGIN(EEPROM);

ENTRY(Reset_Handler)

SECTIONS
{
    .text :
    {
        KEEP(*(.vectors))
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
        KEEP(*(.eh_frame*))
    } > FLASH

    .ARM.extab (READONLY) :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    __exidx_start = .;
    .ARM.exidx (READONLY) :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;

    .copy.table (READONLY) :
    {
        . = ALIGN(4);
        __copy_table_start__ = .;
        LONG(LOADADDR(.data));
        LONG(__data_sram1_start__);
        LONG(__data_sram1_end__ - __data_sram1_start__);
        LONG(LOADADDR(.data_sram2));
        LONG(__data_sram2_start__);
        LONG(__data_sram2_end__ - __data_sram2_start__);
        LONG(LOADADDR(.data_ccmsram));
        LONG(__data_ccmsram_start__);
        LONG(__data_ccmsram_end__ - __data_ccmsram_start__);
        __copy_table_end__ = .;
    } > FLASH

    .zero.table (READONLY) :
    {
        . = ALIGN(4);
        __zero_table_start__ = .;
        LONG (__bss_sram1_start__)
        LONG (__bss_sram1_end__ - __bss_sram1_start__)
        LONG (__bss_sram2_start__)
        LONG (__bss_sram2_end__ - __bss_sram2_start__)
        LONG (__bss_ccmsram_start__)
        LONG (__bss_ccmsram_end__ - __bss_ccmsram_start__)
        __zero_table_end__ = .;
    } > FLASH

    __flash_end__ = ALIGN(4);

    .data : AT(__flash_end__)
    {
        . = ALIGN(4);
        __data_sram1_start__ = .;
        *(vtable)
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        __data_sram1_end__ = .;
    } > SRAM1

    __flash_end_2__ = (__flash_end__ + __data_sram1_end__ - __data_sram1_start__);

    .data_sram2 : AT(__flash_end_2__)
    {
        . = ALIGN(4);
        __data_sram2_start__ = .;
        *(.data_sram2)
        *(.data_sram2.*)
        . = ALIGN(4);
        __data_sram2_end__ = .;
    } > SRAM2

    __flash_end_3__ = (__flash_end_2__ + __data_sram2_end__ - __data_sram2_start__);

    .data_ccmsram : AT(__flash_end_3__)
    {
        . = ALIGN(4);
        __data_ccmsram_start__ = .;
        *(.data_ccmsram)
        *(.data_ccmsram.*)
        . = ALIGN(4);
        __data_ccmsram_end__ = .;
    } > CCMSRAM

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_sram1_start__ = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_sram1_end__ = .;
    } > SRAM1

    .bss_sram2 (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_sram2_start__ = .;
        *(.bss_sram2)
        *(.bss_sram2.*)
        . = ALIGN(4);
        __bss_sram2_end__ = .;
    } > SRAM2

    .bss_ccmsram (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_ccmsram_start__ = .;
        *(.bss_ccmsram)
        *(.bss_ccmsram.*)
        . = ALIGN(4);
        __bss_ccmsram_end__ = .;
    } > CCMSRAM
  
    .heap (COPY) :
    {
        __heap_start__ = .;
        KEEP(*(.heap*))
        __heap_end__ = .;
    } > SRAM2

    .stack (COPY) :
    {
        KEEP(*(.stack*))
    } > SRAM2

    __stack_top__ = ORIGIN(SRAM2) + LENGTH(SRAM2);
    __stack_limit__ = __stack_top__ - SIZEOF(.stack);

    ASSERT((__stack_limit__ >= __heap_end__), "Region SRAM2 overflowed with stack")
}