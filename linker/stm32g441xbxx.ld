/*
 * stm32g441xbxx.ld
 *
 *  Created on: 02 feb. 2025
 *      Author: ARM
 */

__flash_size__ = 128K;
__flash_page_size__ = 2K;

MEMORY
{
    /* Last FLASH page is reserved for EEPROM emulation */
    FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = (__flash_size__ - __flash_page_size__)
    EEPROM  (rw)  : ORIGIN = (ORIGIN(FLASH) + LENGTH(FLASH)), LENGTH = __flash_page_size__
    CCMSRAM (rwx) : ORIGIN = 0x10000000, LENGTH = 10K
    SRAM1   (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
    SRAM2   (rwx) : ORIGIN = 0x20004000, LENGTH = 6K
}

__flash_address__ = ORIGIN(FLASH);
__eeprom_page_address__ = ORIGIN(EEPROM);

ENTRY(Reset_Handler)

SECTIONS
{
    .text :
    {
        KEEP(*(.vectors))
        KEEP(*(.ResetHandler))
        KEEP(*(.DefaultHandler))
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

    __etext = ALIGN(4);

    .data : AT(__etext)
    {
        . = ALIGN(4);
        __data_sram1_start__ = .;
        *(vtable)
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        __data_sram1_end__ = .;
    } > SRAM1

    __etext2 = ALIGN(4);

    .data_sram2 : AT(__etext2)
    {
        . = ALIGN(4);
        __data_sram2_start__ = .;
        *(.data_sram2)
        *(.data_sram2.*)
        . = ALIGN(4);
        __data_sram2_end__ = .;
    } > SRAM2

    __etext3 = ALIGN(4);

    .data_ccmsram : AT(__etext3)
    {
        . = ALIGN(4);
        __data_ccmsram_start__ = .;
        *(.data_ccmsram)
        *(.data_ccmsram.*)
        . = ALIGN(4);
        __data_ccmsram_end__ = .;
    } > CCMSRAM

    .bss_sram1 :
    {
        . = ALIGN(4);
        __bss_sram1_start__ = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_sram1_end__ = .;
    } > SRAM1 

    .bss_sram2 :
    {
        . = ALIGN(4);
        __bss_sram2_start__ = .;
        *(.bss_sram2)
        *(.bss_sram2.*)
        . = ALIGN(4);
        __bss_sram2_end__ = .;
    } > SRAM2 

    .bss_ccmsram :
    {
        . = ALIGN(4);
        __bss_ccmsram_start__ = .;
        *(.bss_ccmsram)
        *(.bss_ccmsram.*)
        . = ALIGN(4);
        __bss_ccmsram_end__ = .;
    } > CCMSRAM
  
    .heap (COPY):
    {
        __HeapBase = .;
        __end__ = .;
        end = __end__;
        KEEP(*(.heap*))
        __HeapLimit = .;
    } > SRAM2

    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy (COPY):
    {
        KEEP(*(.stack*))
    } > SRAM2

    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    __StackTop = ORIGIN(SRAM2) + LENGTH(SRAM2);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")
}
