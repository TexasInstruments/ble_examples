/******************************************************************************

 @file  cc23x0_app_freertos.cmd

 @brief cc23x0R5 linker configuration file for FreeRTOS
        with Code Composer Studio.

        Imported Symbols
        Note: Linker defines are located in the CCS IDE project by placing them
        in
        Properties->Build->Linker->Advanced Options->Command File Preprocessing.

        ICALL_RAM0_START:   RAM start of BLE stack.
        ICALL_STACK0_START: Flash start of BLE stack.
        PAGE_AlIGN:         Align BLE stack boundary to a page boundary.
                            Aligns to Flash word boundary by default.

 Group: WCS, BTS
 $Target Device: DEVICES $

 ******************************************************************************
 $License: BSD3 2017 $
 ******************************************************************************
 $Release Name: PACKAGE NAME $
 $Release Date: PACKAGE RELEASE DATE $
 *****************************************************************************/

/*******************************************************************************
 * CCS Linker configuration
 */

/* Override default entry point.                                             */
--entry_point=resetISR

/* Retain interrupt vector table variable                                    */
--retain "*(.resetVecs)"

/* Suppress warnings and errors:                                             */
/* - 10063: Warning about entry point not being _c_int00                     */
/* - 16011, 16012: 8-byte alignment errors. Observed when linking in object  */
/*   files compiled using Keil (ARM compiler)                                */
--diag_suppress=10063,16011,16012
--heap_size=0
--stack_size=800
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
/*******************************************************************************
 * Memory Sizes
 */
#define FLASH_BASE   0x00000000
#define RAM_BASE     0x20000000
#define FLASH_SIZE   0x00080000
#define RAM_SIZE     0x00009000
#define CCFG_BASE    0x4E020000
#define CCFG_SIZE    0x800
#define NVS_SIZE     0x4000
#define NVS_BASE     (FLASH_SIZE - NVS_SIZE)

#if defined(OAD_APP_OFFCHIP) || defined(OAD_APP_ONCHIP) || defined(OAD_PERSISTENT) || defined(OAD_DUAL_IMAGE)
#define MCU_HDR_SIZE    0x100
#define MCUBOOT_BASE    FLASH_BASE
#define MCUBOOT_SIZE    0x6000
#define APP_HDR_BASE    APP_HDR_ADDR
#define APP_BASE        (APP_HDR_BASE + MCU_HDR_SIZE)
#endif //defined(OAD_APP_OFFCHIP) || defined(OAD_APP_ONCHIP) || defined(OAD_PERSISTENT) || defined(OAD_DUAL_IMAGE)

#if defined(OAD_APP_ONCHIP)|| defined(OAD_PERSISTENT)
#define PERSISTENT_HDR_BASE 0x6000
#define PERSISTENT_BASE     (PERSISTENT_HDR_BASE + MCU_HDR_SIZE)
#define PERSISTENT_SIZE     (APP_HDR_BASE - PERSISTENT_BASE)
#define APP_SIZE            (FLASH_SIZE - APP_BASE - NVS_SIZE)
#endif //defined(OAD_APP_ONCHIP)|| defined(OAD_PERSISTENT)

#ifdef OAD_APP_OFFCHIP
#define APP_SIZE        (FLASH_SIZE - APP_BASE - NVS_SIZE)
#endif //OAD_APP_OFFCHIP

#ifdef OAD_DUAL_IMAGE
#define APP_SIZE        ((FLASH_SIZE - NVS_SIZE - MCUBOOT_SIZE)/2 - MCU_HDR_SIZE)
#endif //OAD_DUAL_IMAGE

/*******************************************************************************
 * Memory Definitions
 ******************************************************************************/

/*******************************************************************************
 * RAM
 */
#define RAM_START      (RAM_BASE)
#ifdef ICALL_RAM0_START
  #define RAM_END      (ICALL_RAM0_START - 1)
#else
  #define RAM_END      (RAM_BASE + RAM_SIZE - 1)
#endif /* ICALL_RAM0_START */

/*******************************************************************************
 * Flash
 */

#define FLASH_START                FLASH_BASE
#define WORD_SIZE                  4

#define PAGE_SIZE                  0x800

#ifdef PAGE_ALIGN
  #define FLASH_MEM_ALIGN          PAGE_SIZE
#else
  #define FLASH_MEM_ALIGN          WORD_SIZE
#endif /* PAGE_ALIGN */

#define PAGE_MASK                  0xFFFFE000

/* The last Flash page is reserved for the application. */
#define NUM_RESERVED_FLASH_PAGES   1
#define RESERVED_FLASH_SIZE        (NUM_RESERVED_FLASH_PAGES * PAGE_SIZE)

/* Check if page alingment with the Stack image is required.  If so, do not link
 * into a page shared by the Stack.
 */
#ifdef ICALL_STACK0_START
  #ifdef PAGE_ALIGN
    #define ADJ_ICALL_STACK0_START (ICALL_STACK0_START * PAGE_MASK)
  #else
    #define ADJ_ICALL_STACK0_START ICALL_STACK0_START
  #endif /* PAGE_ALIGN */

  #define FLASH_END                (ADJ_ICALL_STACK0_START - 1)
#else
  #define FLASH_END                (FLASH_BASE + FLASH_SIZE - RESERVED_FLASH_SIZE - 1)
#endif /* ICALL_STACK0_START */

#define FLASH_LAST_PAGE_START      (FLASH_SIZE - PAGE_SIZE)

/*******************************************************************************
 * Stack
 */

/* Create global constant that points to top of stack */
/* CCS: Change stack size under Project Properties    */
__STACK_TOP = __stack + __STACK_SIZE;

/*******************************************************************************
 * ROV
 * These symbols are used by ROV2 to extend the valid memory regions on device.
 * Without these defines, ROV will encounter a Java exception when using an
 * autosized heap. This is a posted workaround for a known limitation of
 * RTSC/rta. See: https://bugs.eclipse.org/bugs/show_bug.cgi?id=487894
 *
 * Note: these do not affect placement in RAM or FLASH, they are only used
 * by ROV2, see the BLE Stack User's Guide for more info on a workaround
 * for ROV Classic
 *
 */
__UNUSED_SRAM_start__ = RAM_BASE;
__UNUSED_SRAM_end__ = RAM_BASE + RAM_SIZE;

__UNUSED_FLASH_start__ = FLASH_BASE;
__UNUSED_FLASH_end__ = FLASH_BASE + FLASH_SIZE;

/*******************************************************************************
 * Main arguments
 */

/* Allow main() to take args */
/* --args 0x8 */

/*******************************************************************************
 * System Memory Map
 ******************************************************************************/
MEMORY
{
  /* EDITOR'S NOTE:
   * the FLASH and SRAM lengths can be changed by defining
   * ICALL_STACK0_START or ICALL_RAM0_START in
   * Properties->ARM Linker->Advanced Options->Command File Preprocessing.
   */

#if defined(OAD_APP_OFFCHIP)|| defined(OAD_APP_ONCHIP) || defined(OAD_PERSISTENT) || defined(OAD_DUAL_IMAGE)

    MCUBOOT_SLOT(RX)       : origin = MCUBOOT_BASE        ,length = MCUBOOT_SIZE
    APP_HDR_SLOT(RX)       : origin = APP_HDR_BASE        ,length = MCU_HDR_SIZE
    APP_SLOT (RX)          : origin = APP_BASE            ,length = APP_SIZE

#if defined(OAD_APP_ONCHIP) || defined(OAD_PERSISTENT)

    PERSISTENT_HDR_SLOT(RX) : origin = PERSISTENT_HDR_BASE ,length = MCU_HDR_SIZE
    PERSISTENT_SLOT(RX)     : origin = PERSISTENT_BASE     ,length = PERSISTENT_SIZE

#endif //defined(OAD_APP_ONCHIP) || defined(OAD_PERSISTENT)

#else //Without mcuboot

    FLASH (RX)   : origin = 0x0,      length = (FLASH_SIZE - NVS_SIZE)

#endif //defined(OAD_APP_OFFCHIP)|| defined(OAD_APP_ONCHIP) || defined(OAD_PERSISTENT)

  NVS_SLOT(RX) : origin = NVS_BASE ,length = NVS_SIZE
  /* Application uses internal RAM for data */
  SRAM (RWX) : origin = RAM_START, length = (RAM_END - RAM_START + 1)

  CCFG (RW) : origin = CCFG_BASE, length = CCFG_SIZE
}

/*******************************************************************************
 * Section Allocation in Memory
 ******************************************************************************/
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
  .resetVecs      :   >  FLASH_START
  .text           :   >> FLASH
  .const          :   >> FLASH
  .constdata      :   >> FLASH
  .rodata         :   >> FLASH
  .cinit          :   >  FLASH
  .pinit          :   >> FLASH
  .init_array     :   >  FLASH
  .emb_text       :   >> FLASH


#endif //defined(OAD_APP_OFFCHIP) || defined(OAD_APP_ONCHIP) || defined(OAD_PERSISTENT) || defined(OAD_DUAL_IMAGE)

  .ccfg           :   > CCFG
  .ramVecs        :   > SRAM, type = NOLOAD, ALIGN(256)
  .data           :   > SRAM
  .bss            :   > SRAM
  .sysmem         :   > SRAM
  .nonretenvar    :   > SRAM
  .heap           :   > SRAM
  .stack          :   > SRAM (HIGH)

}
