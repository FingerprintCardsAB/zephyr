/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Linker command/script file
 *
 * Linker script for the Cortex-M3 platform.
 */

#define _LINKER
#define _ASMLANGUAGE

#include <autoconf.h>
#include <linker/sections.h>
#include <generated_dts_board.h>

#include <linker/linker-defs.h>
#include <linker/linker-tool.h>

/* physical address of RAM */
#ifdef CONFIG_XIP
  #define ROMABLE_REGION TEXT
  #define RAMABLE_REGION SRAM
#else
  #define ROMABLE_REGION SRAM
  #define RAMABLE_REGION SRAM
#endif

#if defined(CONFIG_XIP)
  #define _DATA_IN_ROM __data_rom_start
#else
  #define _DATA_IN_ROM
#endif

#define ROM_ADDR (CONFIG_FLASH_BASE_ADDRESS + CONFIG_FLASH_LOAD_OFFSET)
#if CONFIG_FLASH_LOAD_SIZE > 0
  #define ROM_SIZE CONFIG_FLASH_LOAD_SIZE
#else
  #define ROM_SIZE (CONFIG_FLASH_SIZE*1K - CONFIG_FLASH_LOAD_OFFSET)
#endif

#if defined(CONFIG_XIP)
    #if defined(CONFIG_IS_BOOTLOADER)
        #define RAM_SIZE (CONFIG_BOOTLOADER_SRAM_SIZE * 1K)
        #define RAM_ADDR (CONFIG_SRAM_BASE_ADDRESS + \
            (CONFIG_SRAM_SIZE * 1K - RAM_SIZE))
    #else
        #define RAM_SIZE (CONFIG_SRAM_SIZE * 1K)
        #define RAM_ADDR CONFIG_SRAM_BASE_ADDRESS
    #endif
    #define FLASH_CONFIG_SIZE CONFIG_IVT_SECTION_OFFSET
    #define IVT_SIZE CONFIG_TEXT_SECTION_OFFSET - CONFIG_IVT_SECTION_OFFSET
#else
    #define RAM_SIZE (CONFIG_SRAM_SIZE * 1K - CONFIG_BOOTLOADER_SRAM_SIZE * 1K)
    #define RAM_ADDR CONFIG_SRAM_BASE_ADDRESS
#endif

MEMORY
    {
    FLASH_CONFIG          (rx) : ORIGIN = ROM_ADDR, LENGTH = FLASH_CONFIG_SIZE
    IVT                   (rx) : ORIGIN = ROM_ADDR + CONFIG_IVT_SECTION_OFFSET, LENGTH = IVT_SIZE
    TEXT                  (rx) : ORIGIN = ROM_ADDR + CONFIG_TEXT_SECTION_OFFSET, LENGTH = ROM_SIZE - CONFIG_TEXT_SECTION_OFFSET
#ifdef CONFIG_CCM_BASE_ADDRESS
    CCM                   (rw) : ORIGIN = CONFIG_CCM_BASE_ADDRESS, LENGTH = CONFIG_CCM_SIZE * 1K
#endif
    SRAM                  (wx) : ORIGIN = RAM_ADDR, LENGTH = RAM_SIZE

    /* Used by and documented in include/linker/intlist.ld */
    IDT_LIST              (wx) : ORIGIN = (RAM_ADDR + RAM_SIZE), LENGTH = 2K
    }

SECTIONS
    {
    GROUP_START(ROMABLE_REGION)

    _image_rom_start = ROM_ADDR;

    SECTION_PROLOGUE(.flash_config,,)
    {
    _flash_config_start = .;
    KEEP(* (.boot_hdr.conf))     /* flash config section */
    _flash_config_end = .;
    } GROUP_LINK_IN(FLASH_CONFIG)

    SECTION_PROLOGUE(.ivt,,)
    {
    _ivt_config_start = .;
    KEEP(* (.boot_hdr.ivt))           /* ivt section */
    KEEP(* (.boot_hdr.boot_data))     /* boot section */
    KEEP(* (.boot_hdr.dcd_data))      /* dcd section */
    _ivt_config_end = .;
    } GROUP_LINK_IN(IVT)

    SECTION_PROLOGUE(_TEXT_SECTION_NAME,,)
    {

#if defined(CONFIG_SW_VECTOR_RELAY)
    KEEP(*(.vector_relay_table))
    KEEP(*(".vector_relay_table.*"))
    KEEP(*(.vector_relay_handler))
    KEEP(*(".vector_relay_handler.*"))
#endif

    _vector_start = .;
    KEEP(*(.exc_vector_table))
    KEEP(*(".exc_vector_table.*"))

    KEEP(*(IRQ_VECTOR_TABLE))

    KEEP(*(.openocd_dbg))
    KEEP(*(".openocd_dbg.*"))

#ifdef CONFIG_GEN_SW_ISR_TABLE
    KEEP(*(SW_ISR_TABLE))
#endif
    _vector_end = .;
    _image_text_start = .;
    *(.text)
    *(".text.*")
    *(.gnu.linkonce.t.*)

#include <linker/priv_stacks-text.ld>
#include <linker/kobject-text.ld>

    } GROUP_LINK_IN(ROMABLE_REGION)

    _image_text_end = .;

    SECTION_PROLOGUE(.ARM.exidx,,)
    {
    /*
     * This section, related to stack and exception unwinding, is placed
     * explicitly to prevent it from being shared between multiple regions.
     * It  must be defined for gcc to support 64-bit math and avoid
     * section overlap.
     */
    __exidx_start = .;
#if defined (__GCC_LINKER_CMD__)
    *(.ARM.exidx* gnu.linkonce.armexidx.*)
#endif
    __exidx_end = .;
    } GROUP_LINK_IN(ROMABLE_REGION)

    _image_rodata_start = .;

#include <linker/common-rom.ld>

    SECTION_PROLOGUE(_RODATA_SECTION_NAME,,)
    {
    *(.rodata)
    *(".rodata.*")
    *(.gnu.linkonce.r.*)

#ifdef CONFIG_CUSTOM_RODATA_LD
/* Located in project source directory */
#include <custom-rodata.ld>
#endif

#include <linker/priv_stacks-rom.ld>
#include <linker/kobject-rom.ld>

    /*
     * For XIP images, in order to avoid the situation when __data_rom_start
     * is 32-bit aligned, but the actual data is placed right after rodata
     * section, which may not end exactly at 32-bit border, pad rodata
     * section, so __data_rom_start points at data and it is 32-bit aligned.
     *
     * On non-XIP images this may enlarge image size up to 3 bytes. This
     * generally is not an issue, since modern ROM and FLASH memory is
     * usually 4k aligned.
     */
    . = ALIGN(4);
    } GROUP_LINK_IN(ROMABLE_REGION)

    _image_rodata_end = .;
    _image_rom_end = .;

    GROUP_END(ROMABLE_REGION)

    GROUP_START(RAMABLE_REGION)

#if defined(CONFIG_SW_VECTOR_RELAY)
    /* Reserved 4 bytes to save vector table base address */
    SECTION_PROLOGUE(.vt_pointer,(NOLOAD),)
    {
        *(.vt_pointer_section)
        *(".vt_pointer_section.*")
    }
#endif

#ifdef CONFIG_APPLICATION_MEMORY
    SECTION_DATA_PROLOGUE(_APP_DATA_SECTION_NAME, (OPTIONAL),)
    {

#include <app_data_alignment.ld>

        __app_ram_start = .;
        __app_data_ram_start = .;
        _image_ram_start = .;
        APP_INPUT_SECTION(.data)
        APP_INPUT_SECTION(".data.*")
        __app_data_ram_end = .;
    } GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

    __app_data_rom_start = LOADADDR(_APP_DATA_SECTION_NAME);

    SECTION_PROLOGUE(_APP_BSS_SECTION_NAME, (NOLOAD OPTIONAL),)
    {
        __app_bss_start = .;
        APP_INPUT_SECTION(.bss)
        APP_INPUT_SECTION(".bss.*")
        APP_INPUT_SECTION(COMMON)
        __app_bss_end = .;
    } GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)

    __app_bss_num_words = (__app_bss_end - __app_bss_start) >> 2;

    SECTION_PROLOGUE(_APP_NOINIT_SECTION_NAME, (NOLOAD OPTIONAL),)
    {
        APP_INPUT_SECTION(.noinit)
        APP_INPUT_SECTION(".noinit.*")
    } GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)

    __app_last_address_used = .;

    /* Pad out application ram area to make MPU friendly */
    SECTION_PROLOGUE(app_pad, (NOLOAD OPTIONAL),)
    {
        . = ALIGN(_app_data_align);
    } GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)
    __app_ram_end = .;
    __app_ram_size = __app_ram_end - __app_ram_start;
#endif /* CONFIG_APPLICATION_MEMORY */

    SECTION_DATA_PROLOGUE(_BSS_SECTION_NAME,(NOLOAD),)
    {
        /*
         * For performance, BSS section is assumed to be 4 byte aligned and
         * a multiple of 4 bytes
         */
        . = ALIGN(4);
    __bss_start = .;
#ifndef CONFIG_APPLICATION_MEMORY
    _image_ram_start = .;
#endif
    __kernel_ram_start = .;

    KERNEL_INPUT_SECTION(.bss)
    KERNEL_INPUT_SECTION(".bss.*")
    KERNEL_INPUT_SECTION(COMMON)
    *(".kernel_bss.*")

        /*
         * As memory is cleared in words only, it is simpler to ensure the BSS
         * section ends on a 4 byte boundary. This wastes a maximum of 3 bytes.
         */
    __bss_end = ALIGN(4);
    } GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)

    SECTION_PROLOGUE(_NOINIT_SECTION_NAME,(NOLOAD),)
        {
        /*
         * This section is used for non-initialized objects that
         * will not be cleared during the boot process.
         */
        KERNEL_INPUT_SECTION(.noinit)
        KERNEL_INPUT_SECTION(".noinit.*")
    *(".kernel_noinit.*")

        } GROUP_LINK_IN(RAMABLE_REGION)

    SECTION_DATA_PROLOGUE(_DATA_SECTION_NAME,,)
    {
    __data_ram_start = .;
    KERNEL_INPUT_SECTION(.data)
    KERNEL_INPUT_SECTION(".data.*")
    *(".kernel.*")

#ifdef CONFIG_CUSTOM_RWDATA_LD
/* Located in project source directory */
#include <custom-rwdata.ld>
#endif

    } GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

    __data_rom_start = LOADADDR(_DATA_SECTION_NAME);

#include <linker/common-ram.ld>
#include <linker/priv_stacks.ld>
#include <linker/kobject.ld>

    __data_ram_end = .;


    /* Define linker symbols */

    _image_ram_end = .;
    _end = .; /* end of image */

    __kernel_ram_end = RAM_ADDR + RAM_SIZE;
    __kernel_ram_size = __kernel_ram_end - __kernel_ram_start;

    GROUP_END(RAMABLE_REGION)

#ifdef CONFIG_CCM_BASE_ADDRESS

    GROUP_START(CCM)

    SECTION_PROLOGUE(_CCM_BSS_SECTION_NAME, (NOLOAD OPTIONAL),SUBALIGN(4))
    {
        __ccm_start = .;
        __ccm_bss_start = .;
        *(.ccm_bss)
        *(".ccm_bss.*")
    } GROUP_LINK_IN(CCM)

    __ccm_bss_end = .;

    SECTION_PROLOGUE(_CCM_NOINIT_SECTION_NAME, (NOLOAD OPTIONAL),SUBALIGN(4))
    {
        __ccm_noinit_start = .;
        *(.ccm_noinit)
        *(".ccm_noinit.*")
    } GROUP_LINK_IN(CCM)

    __ccm_noinit_end = .;

    SECTION_PROLOGUE(_CCM_DATA_SECTION_NAME, (OPTIONAL),SUBALIGN(4))
    {
        __ccm_data_start = .;
        *(.ccm_data)
        *(".ccm_data.*")
    } GROUP_LINK_IN(CCM AT> ROMABLE_REGION)

    __ccm_data_end = .;
    __ccm_end = .;

    __ccm_data_rom_start = LOADADDR(_CCM_DATA_SECTION_NAME);

    GROUP_END(CCM)

#endif /* CONFIG_CCM_BASE_ADDRESS */

#ifdef CONFIG_CUSTOM_SECTIONS_LD
/* Located in project source directory */
#include <custom-sections.ld>
#endif

#ifdef CONFIG_GEN_ISR_TABLES
#include <linker/intlist.ld>
#endif

    }
