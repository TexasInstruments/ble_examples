/**
  @file  main.c
  @brief main entry of the BLE stack sample application.

  <!--
  Copyright 2013 - 2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED ``AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
  -->
*/
#include <xdc/runtime/Error.h>

#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/BIOS.h>

#include "icall.h"
#include "bcomdef.h"
#include "peripheral.h"
#include "security_examples_peripheral.h"

/* Header files required to enable instruction fetch cache */
#include <inc/hw_memmap.h>
#include <driverlib/vims.h>

#ifndef USE_DEFAULT_USER_CFG

#include "ble_user_config.h"

// BLE user defined configuration
bleUserCfg_t user0Cfg = BLE_USER_CFG;

#endif // USE_DEFAULT_USER_CFG

/**
 * Exception handler
 */
void exceptionHandler()
{
	volatile uint8_t i = 1;
    while(i){}
}

#ifdef FEATURE_OAD
#if defined(__IAR_SYSTEMS_ICC__)
extern uint32_t __vector_table;
#elif defined (__TI_COMPILER_VERSION__)
extern uint32_t ti_sysbios_family_arm_m3_Hwi_resetVectors;
#endif //Compiler
#endif //FEATURE_OAD

/*
 *  ======== main ========
 */
int main()
{
  PIN_init(BoardGpioInitTable);

#ifndef POWER_SAVING
    /* Set constraints for Standby, powerdown and idle mode */
    Power_setConstraint(Power_SB_DISALLOW);
    Power_setConstraint(Power_IDLE_PD_DISALLOW);
#endif // POWER_SAVING
    
    /* Initialize ICall module */
    ICall_init();

    /* Start tasks of external images - Priority 5 */
    ICall_createRemoteTasks();
    
    /* Kick off profile - Priority 3 */
    GAPRole_createTask();
    
    SimpleBLEPeripheral_createTask();

#ifdef FEATURE_OAD
    {
      uint8_t counter;
      uint32_t *vectorTable =  (uint32_t*) 0x20000000;
#if defined(__IAR_SYSTEMS_ICC__)
      uint32_t *flashVectors = &__vector_table;
#elif defined(__TI_COMPILER_VERSION__)
      uint32_t *flashVectors = &ti_sysbios_family_arm_m3_Hwi_resetVectors;
#endif //Compiler.
      
      // Write image specific interrupt vectors into RAM vector table.
      for(counter = 0; counter < 15; ++counter)
      {
        *vectorTable++ = *flashVectors++;
      }
    }
#endif //FEATURE_OAD
    
    /* enable interrupts and start SYS/BIOS */
    BIOS_start();
    
    return 0;
}

/**
 * Error handled to be hooked into TI-RTOS
 */
Void smallErrorHook(Error_Block *eb)
{
  for (;;);
}

/**
 * HAL assert handler required by OSAL memory module.
 */
void halAssertHandler(void)
{
  for (;;);
}
