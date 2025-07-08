/******************************************************************************

 @file  lpf3_app_freertos.cmd

 @brief lpf3 family linker configuration file for FreeRTOS
        with Code Composer Studio.

        Imported Symbols
        Note: Linker defines are located in the CCS IDE project by placing them
        in
        Properties->Build->Linker->Advanced Options->Command File Preprocessing.

 Group: WCS, BTS
 Target Device: cc23xx

 ******************************************************************************
 
 Copyright (c) 2017-2024, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/


/* include to the file that generates the globals which holds the size of the regions */
#include "ti_utils_build_linker.cmd.genmap"

--stack_size=1024
--heap_size=0
--entry_point resetISR

/* Retain interrupt vector table variable                                    */
--retain "*(.resetVecs)"

/* Suppress warnings and errors:                                             */
/* - 10063: Warning about entry point not being _c_int00                     */
/* - 16011, 16012: 8-byte alignment errors. Observed when linking in object  */
/*   files compiled using Keil (ARM compiler)                                */
--diag_suppress=10063,16011,16012

/* Set severity of diagnostics to Remark instead of Warning                  */
/* - 10068: Warning about no matching log_ptr* sections                      */
--diag_remark=10068

/* The starting address of the application.  Normally the interrupt vectors  */
/* must be located at the beginning of the application.                      */

/*******************************************************************************
 * Memory Sizes
 */

#define FLASH_BASE              ti_utils_build_GenMap_FLASH0_BASE
#ifdef SECURE_BOOT
/* Setting an hardcoded flash size is a workaround since the flash size */
/* generated in ti_utils_build_linker.cmd.genmap when using Secure Boot */
/* is the image size and not the total flash size required in this file. */
#define FLASH_SIZE              0xE8000
#else
#define FLASH_SIZE              ti_utils_build_GenMap_FLASH0_SIZE
#endif
#define RAM_BASE                ti_utils_build_GenMap_RAM0_BASE
#define RAM_SIZE                ti_utils_build_GenMap_RAM0_SIZE

/* Not all LPF3 devices have S2RRAM */
#if defined(ti_utils_build_GenMap_S2RRAM_BASE) && \
    defined(ti_utils_build_GenMap_S2RRAM_SIZE)
#define S2RRAM_BASE             ti_utils_build_GenMap_S2RRAM_BASE
#define S2RRAM_SIZE             ti_utils_build_GenMap_S2RRAM_SIZE
#endif

#define CCFG_BASE               ti_utils_build_GenMap_CCFG_BASE
#define CCFG_SIZE               ti_utils_build_GenMap_CCFG_SIZE

/* Not all LPF3 devices have SCFG */
#if defined(ti_utils_build_GenMap_SCFG_BASE) && \
    defined(ti_utils_build_GenMap_SCFG_SIZE)
#define SCFG_BASE               ti_utils_build_GenMap_SCFG_BASE
#define SCFG_SIZE               ti_utils_build_GenMap_SCFG_SIZE
#endif

/* Not all LPF3 devices have HSMOTP */
#if defined(ti_utils_build_GenMap_HSMOTP_BASE) && \
    defined(ti_utils_build_GenMap_HSMOTP_SIZE)
#define HSMOTP_BASE             ti_utils_build_GenMap_HSMOTP_BASE
#define HSMOTP_SIZE             ti_utils_build_GenMap_HSMOTP_SIZE
#endif

#define NVS_SIZE                0x4000
#define NVS_BASE                FLASH_SIZE - NVS_SIZE


/*******************************************************************************
 * OAD - Not all LPF3 devices supports OAD
 * If they do support, there are regions to define and align differnetlly
 */

#if defined(OAD_APP_OFFCHIP) || defined(OAD_APP_ONCHIP) || defined(OAD_PERSISTENT) || defined(OAD_DUAL_IMAGE)
#ifdef SECURE_BOOT
#define MCU_HDR_SIZE    0x80
#else
#define MCU_HDR_SIZE    0x100
#define MCUBOOT_BASE    FLASH_BASE
#define MCUBOOT_SIZE    0x6000
#endif
#define APP_HDR_BASE    APP_HDR_ADDR
#define APP_BASE        (APP_HDR_BASE + MCU_HDR_SIZE)
#endif //defined(OAD_APP_OFFCHIP) || defined(OAD_APP_ONCHIP) || defined(OAD_PERSISTENT) || defined(OAD_DUAL_IMAGE)

#if defined(OAD_APP_ONCHIP)|| defined(OAD_PERSISTENT)
#ifdef SECURE_BOOT
#define PERSISTENT_HDR_BASE 0x00
#else
#define PERSISTENT_HDR_BASE 0x6000
#endif
#define PERSISTENT_BASE     (PERSISTENT_HDR_BASE + MCU_HDR_SIZE)
#define PERSISTENT_SIZE     (APP_HDR_BASE - PERSISTENT_BASE)
#define APP_SIZE            (FLASH_SIZE - APP_BASE - NVS_SIZE)
#endif //defined(OAD_APP_ONCHIP)|| defined(OAD_PERSISTENT)

#ifdef OAD_APP_OFFCHIP
#define APP_SIZE        (FLASH_SIZE - APP_BASE - NVS_SIZE)
#endif //OAD_APP_OFFCHIP

#ifdef OAD_DUAL_IMAGE
#ifdef SECURE_BOOT
#define APP_SIZE        ((FLASH_SIZE - NVS_SIZE)/2 - MCU_HDR_SIZE)
#else
#define APP_SIZE        ((FLASH_SIZE - NVS_SIZE - MCUBOOT_SIZE)/2 - MCU_HDR_SIZE)
#endif
#endif //OAD_DUAL_IMAGE

/*******************************************************************************
 * Flash
 */

/* PAGE_AlIGN: Align BLE stack boundary to a page boundary.
 * Aligns to Flash word boundary by default.
 */

#define WORD_SIZE                  4
#define PAGE_SIZE                  0x800

#ifdef PAGE_ALIGN
  #define FLASH_MEM_ALIGN          PAGE_SIZE
#else
  #define FLASH_MEM_ALIGN          WORD_SIZE
#endif /* PAGE_ALIGN */

/*******************************************************************************
 * Stack
 */

/* Create global constant that points to top of stack */
/* CCS: Change stack size under Project Properties    */
__STACK_TOP = __stack + __STACK_SIZE;


/*******************************************************************************
 * System memory map
 */
MEMORY
{

#if defined(OAD_APP_OFFCHIP)|| defined(OAD_APP_ONCHIP) || defined(OAD_PERSISTENT) || defined(OAD_DUAL_IMAGE)

#ifndef SECURE_BOOT
    MCUBOOT_SLOT(RX)       : origin = MCUBOOT_BASE        ,length = MCUBOOT_SIZE
#endif
    APP_HDR_SLOT(RX)       : origin = APP_HDR_BASE        ,length = MCU_HDR_SIZE
    APP_SLOT (RX)          : origin = APP_BASE            ,length = APP_SIZE

#if defined(OAD_APP_ONCHIP) || defined(OAD_PERSISTENT)

    PERSISTENT_HDR_SLOT(RX) : origin = PERSISTENT_HDR_BASE ,length = MCU_HDR_SIZE
    PERSISTENT_SLOT(RX)     : origin = PERSISTENT_BASE     ,length = PERSISTENT_SIZE

#endif //defined(OAD_APP_ONCHIP) || defined(OAD_PERSISTENT)

#else //Without mcuboot

#ifdef SECURE_BOOT
    /* Application stored in and executes from internal flash */
    FLASH (RX) : origin = FLASH_BASE, length = FLASH_SIZE - 0x80 //size of header
#else
    /* Application stored in and executes from internal flash */
    FLASH (RX) : origin = FLASH_BASE, length = (FLASH_SIZE - NVS_SIZE)
#endif


#endif //defined(OAD_APP_OFFCHIP)|| defined(OAD_APP_ONCHIP) || defined(OAD_PERSISTENT)

#ifdef SECURE_BOOT
    NVS_SLOT(RX) : origin = 0xE4000 ,length = NVS_SIZE
#else
    NVS_SLOT(RX) : origin = NVS_BASE ,length = NVS_SIZE
#endif


    /* Application uses internal RAM for data */
    SRAM (RWX) : origin = RAM_BASE, length = RAM_SIZE

#if defined(S2RRAM_BASE) && defined(S2RRAM_SIZE)
    /* S2RRAM is intended for the S2R radio module, but it can also be used by
     * the application with some limitations. Please refer to the s2rram example.
     */
    S2RRAM (RW) : origin = S2RRAM_BASE, length = S2RRAM_SIZE
#endif

    /* Configuration region */
    CCFG (R) : origin = CCFG_BASE, length = CCFG_SIZE

#if defined(SCFG_BASE) && defined(SCFG_SIZE)
    /* Security configuration region */
    SCFG (R): origin = SCFG_BASE, length = SCFG_SIZE
#endif

#if defined(HSMOTP_BASE) && defined(HSMOTP_SIZE)
    /* HSM OTP region */
    HSMOTP (R): origin = HSMOTP_BASE, length = HSMOTP_SIZE
#endif

    /* Explicitly placed off target for the storage of logging data.
     * The ARM memory map allocates 1 GB of external memory from 0x60000000 - 0x9FFFFFFF.
     * Unlikely that all of this will be used, so we are using the upper parts of the region.
     * ARM memory map: https://developer.arm.com/documentation/ddi0337/e/memory-map/about-the-memory-map*/

    LOG_DATA (R) : origin = 0x90000000, length = 0x40000        /* 256 KB */
    LOG_PTR  (R) : origin = 0x94000008, length = 0x40000        /* 256 KB */

}

/*******************************************************************************
 * Section allocation in memory
 */
SECTIONS
{
#if defined(OAD_APP_OFFCHIP) || defined(OAD_APP_ONCHIP) || defined(OAD_PERSISTENT) || defined(OAD_DUAL_IMAGE)

	.primary_hdr    :   > APP_HDR_SLOT, type = NOLOAD

#if defined(OAD_APP_OFFCHIP) || defined(OAD_APP_ONCHIP) || defined(OAD_DUAL_IMAGE)

    .resetVecs      :   > APP_BASE
    .text           :   > APP_SLOT
    .const          :   > APP_SLOT
    .constdata      :   > APP_SLOT
    .rodata         :   > APP_SLOT
    .binit          :   > APP_SLOT
    .cinit          :   > APP_SLOT
    .pinit          :   > APP_SLOT
    .init_array     :   > APP_SLOT
    .emb_text       :   > APP_SLOT

#else

    .resetVecs      :   > PERSISTENT_BASE
    .text           :   > PERSISTENT_SLOT
    .const          :   > PERSISTENT_SLOT
    .constdata      :   > PERSISTENT_SLOT
    .rodata         :   > PERSISTENT_SLOT
    .binit          :   > PERSISTENT_SLOT
    .cinit          :   > PERSISTENT_SLOT
    .pinit          :   > PERSISTENT_SLOT
    .init_array     :   > PERSISTENT_SLOT
    .emb_text       :   > PERSISTENT_SLOT

#endif //defined(OAD_APP_OFFCHIP) || defined(OAD_APP_ONCHIP) || defined(OAD_DUAL_IMAGE)

#else

    .resetVecs      :   > FLASH_BASE
    .text           :   > FLASH
    .TI.ramfunc     : {} load=FLASH, run=SRAM, table(BINIT)
    .const          :   > FLASH
    .constdata      :   > FLASH
    .rodata         :   > FLASH
    .binit          :   > FLASH
    .cinit          :   > FLASH
    .pinit          :   > FLASH
    .init_array     :   > FLASH
    .emb_text       :   > FLASH

#endif //defined(OAD_APP_OFFCHIP) || defined(OAD_APP_ONCHIP) || defined(OAD_PERSISTENT) || defined(OAD_DUAL_IMAGE)

    .ccfg           :   > CCFG

#if defined(SCFG_BASE) && defined(SCFG_SIZE)
    .scfg           :   > SCFG
#endif

#if defined(HSMOTP_BASE) && defined(HSMOTP_SIZE)
    .hsmotp         :   > HSMOTP
#endif

    .ramVecs        :   > SRAM, type = NOLOAD, ALIGN(256)
    .data           :   > SRAM
    .bss            :   > SRAM
    .sysmem         :   > SRAM
    .stack          :   > SRAM (HIGH)
    .nonretenvar    :   > SRAM

#if defined(S2RRAM_BASE) && defined(S2RRAM_SIZE)
    /* Placing the section .s2rram in S2RRAM region. Only uninitialized
     * objects may be placed in this section.
     */
    .s2rram         :   > S2RRAM, type = NOINIT
#endif

    .log_data       :   > LOG_DATA, type = COPY
    .log_ptr        : { *(.log_ptr*) } > LOG_PTR align 4, type = COPY
}
