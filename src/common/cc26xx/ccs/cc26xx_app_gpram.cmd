
/*
 * Filename: cc26xx_app_gpram.cmd
 *
 * Description: CCS linker file to define GPRAM region
 *
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Retain interrupt vector table variable                                    */
--retain=g_pfnVectors
/* Override default entry point.                                             */
--entry_point ResetISR
/* Suppress warnings and errors:                                             */
/* - 10063: Warning about entry point not being _c_int00                     */
/* - 16011, 16012: 8-byte alignment errors. Observed when linking in object  */ 
/*   files compiled using Keil (ARM compiler)                                */
--diag_suppress=10063,16011,16012

/* The following command line options are set as part of the CCS project.    */
	/* If you are building using the command line, or for some reason want to    */
/* define them here, you can uncomment and modify these lines as needed.     */
/* If you are using CCS for building, it is probably better to make any such */
/* modifications in your CCS project and leave this file alone.              */
/*                                                                           */
/* --heap_size=0                                                             */
/* --stack_size=256                                                          */
/* --library=rtsv7M3_T_le_eabi.lib                                           */

/* The starting address of the application.  Normally the interrupt vectors  */
/* must be located at the beginning of the application. Flash is 128KB, with */
/* sector length of 4KB                                                      */
#define FLASH_APP_BASE          0x00000000
#define FLASH_LEN               0x20000
#define FLASH_PAGE_LEN          0x1000

/* Last page of Flash is allocated to App: 0x1F000 - 0x1FFFF */
#define FLASH_LAST_PAGE_START   FLASH_LEN - FLASH_PAGE_LEN

/* RAM starts at 0x20000000 and is 20KB */
#define RAM_APP_BASE            0x20000000
#define RAM_LEN                 0x5000
/* RAM reserved by ROM code starts. */
#define RAM_RESERVED_OFFSET      0x4F00

/* GPRAM starts at 0x11000000 and is 8KB */
#define GPRAM_BASE				0x11000000
#define GPRAM_SIZE				0x2000

/* System memory map */

MEMORY
{
    /* EDITOR'S NOTE:
     * the FLASH and SRAM lengths can be changed by defining
     * ICALL_STACK0_START or ICALL_RAM0_START in
     * Properties->ARM Linker->Advanced Options->Command File Preprocessing.
     */

    /* Application stored in and executes from internal flash */
    /* Flash Size 128 KB */
    #ifdef ICALL_STACK0_START
        FLASH (RX) : origin = FLASH_APP_BASE, length = ICALL_STACK0_START - FLASH_APP_BASE
    #else // default
        FLASH (RX) : origin = FLASH_APP_BASE, length = FLASH_LEN - FLASH_PAGE_LEN
    #endif

    // CCFG Page, contains .ccfg code section and some application code.
    FLASH_LAST_PAGE (RX) :  origin = FLASH_LAST_PAGE_START, length = FLASH_PAGE_LEN

    /* Application uses internal RAM for data */
    /* RAM Size 16 KB */
    #ifdef ICALL_RAM0_START
        SRAM (RWX) : origin = RAM_APP_BASE, length = ICALL_RAM0_START - RAM_APP_BASE
    #else //default
        SRAM (RWX) : origin = RAM_APP_BASE, length = RAM_RESERVED_OFFSET
    #endif

    GPRAM (RWX) : origin = GPRAM_BASE, length = GPRAM_SIZE
}

/* Section allocation in memory */

SECTIONS
{
    .intvecs        :   >  FLASH_APP_BASE
    .text           :   >> FLASH | FLASH_LAST_PAGE
    .const          :   >> FLASH | FLASH_LAST_PAGE
    .constdata      :   >> FLASH | FLASH_LAST_PAGE
    .rodata         :   >> FLASH | FLASH_LAST_PAGE
    .cinit          :   >  FLASH | FLASH_LAST_PAGE
    .pinit          :   >> FLASH | FLASH_LAST_PAGE
    .init_array     :   >> FLASH | FLASH_LAST_PAGE
    .emb_text       :   >> FLASH | FLASH_LAST_PAGE
    .ccfg           :   >  FLASH_LAST_PAGE (HIGH)
    .bss			:	>  GPRAM


	GROUP > SRAM
	{
	    .data
		.vtable
	    .vtable_ram
	     vtable_ram
	    .sysmem
    	.nonretenvar
	} LOAD_END(heapStart)

	.stack          :   >  SRAM (HIGH) LOAD_START(heapEnd)
}

/* Create global constant that points to top of stack */
/* CCS: Change stack size under Project Properties    */
__STACK_TOP = __stack + __STACK_SIZE;

/* Allow main() to take args */
--args 0x8
