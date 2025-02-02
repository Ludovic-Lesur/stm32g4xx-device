/*
 * stm32g441xbxx.ld
 *
 *  Created on: 02 feb. 2025
 *      Author: ARM
 */

MEMORY
{
    /* Last FLASH page is reserved for EEPROM emulation (128K-2K = 126K) */
    FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 126K
    CCMSRAM (rwx) : ORIGIN = 0x10000000, LENGTH = 10K
    SRAM1   (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
    SRAM2   (rwx) : ORIGIN = 0x20014000, LENGTH = 6K
}

__flash_address__ = ORIGIN(FLASH);
__eeprom_page_address__ = ORIGIN(FLASH) + LENGTH(FLASH);

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
        LONG(LOADADDR(.data_sram1));
        LONG(    ADDR(.data_sram1));
        LONG(  SIZEOF(.data_sram1) / 4);
        LONG(LOADADDR(.data_sram2));
        LONG(    ADDR(.data_sram2));
        LONG(  SIZEOF(.data_sram2) / 4);
        LONG(LOADADDR(.data_ccmsram));
        LONG(    ADDR(.data_ccmsram));
        LONG(  SIZEOF(.data_ccmsram) / 4);
        __copy_table_end__ = .;
    } > FLASH

    .zero.table (READONLY) :
    {
        . = ALIGN(4);
        __zero_table_start__ = .;
        LONG(    ADDR(.bss_sram1));
        LONG(  SIZEOF(.bss_sram1) / 4);
        LONG(    ADDR(.bss_sram2));
        LONG(  SIZEOF(.bss_sram2) / 4);
        LONG(    ADDR(.bss_ccmsram));
        LONG(  SIZEOF(.bss_ccmsram) / 4);
        __zero_table_end__ = .;
    } > FLASH

    __etext = .;

    .data_sram1 :
    {
        . = ALIGN(4);
        __data_sram1_start__ = .;
        *(vtable)
        *(.data_sram1)
        *(.data_sram1.*)
        . = ALIGN(4);
        __data_sram1_end__ = .;
    } > SRAM1

    .data_sram2 :
    {
        . = ALIGN(4);
        __data_sram2_start__ = .;
        *(.data_sram2)
        *(.data_sram2.*)
        . = ALIGN(4);
        __data_sram2_end__ = .;
    } > SRAM2

    .data_ccmsram :
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
        __bss_start__ = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > SRAM1 

    .bss_sram2 :
    {
        . = ALIGN(4);
        *(.bss_sram2)
        *(.bss_sram2.*)
        . = ALIGN(4);
    } > SRAM2 

    .bss_ccmsram :
    {
        . = ALIGN(4);
        *(.bss_ccmsram)
        *(.bss_ccmsram.*)
        . = ALIGN(4);
    } > CCMSRAM
  
    .heap (COPY):
    {
        __HeapBase = .;
        __end__ = .;
        end = __end__;
        KEEP(*(.heap*))
        __HeapLimit = .;
    } > SRAM1

    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy (COPY):
    {
        KEEP(*(.stack*))
    } > SRAM1

    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
    __StackTop = ORIGIN(SRAM1) + LENGTH(SRAM1);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")
}
