/*
 Copyright (c) 2013-2016, Texas Instruments Incorporated
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
 */

/*
 *  ======== lcdSmartRF06EB.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <driverlib/ioc.h>

/* TI-RTOS Header files */
#include <ti/mw/lcd/LCDDogm1286.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

#include <inc/hw_fcfg1.h>

#include "Board.h"

// RF Driver
#include <ti/drivers/rf/RF.h>
#include "smartrf_settings.h"

/***** Defines *****/
#define BEACON_TASK_STACK_SIZE 1024
#define BEACON_TASK_PRIORITY   2

/* TX Configuration */
#define NUMBER_OF_PACKETS               1000
#define PAYLOAD_LENGTH                  30
#define PACKET_INTERVAL 	        50*4000   //50 ms @ 4 MHz
#define BBEACON_DELAY                   4000*4000 //4 seconds @4 MHz
#define ABEACON_COUNT                   20
#define ADV_PERIOD                      (8000 * 4000) - ((PACKET_INTERVAL * ABEACON_COUNT) + BBEACON_DELAY)
#define ADVLEN                          31

/***** Prototypes *****/
static void beaconTaskFunction(UArg arg0, UArg arg1);

/***** Variable declarations *****/
static Task_Params beaconTaskParams;
static Task_Struct beaconTask;
static uint8_t beaconTaskStack[BEACON_TASK_STACK_SIZE];

static RF_Object rfObject;
static RF_Handle rfHandle;

uint32_t time;
static uint8_t advCount = 1;

static char aBeaconData[ADVLEN] =
{
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
  0xAA,
};

static char bBeaconData[ADVLEN] = 
{
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
  0xBB,
};

void beaconTask_init(void) {

    Task_Params_init(&beaconTaskParams);
    beaconTaskParams.stackSize = BEACON_TASK_STACK_SIZE;
    beaconTaskParams.priority = BEACON_TASK_PRIORITY;
    beaconTaskParams.stack = &beaconTaskStack;
    beaconTaskParams.arg0 = (UInt)1000000;

    Task_construct(&beaconTask, beaconTaskFunction, &beaconTaskParams, NULL);
}


static void beaconTaskFunction(UArg arg0, UArg arg1)
{
    uint32_t time;
    RF_Params rfParams;
    RF_Params_init(&rfParams);
    
    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_ble, (RF_RadioSetup*)&RF_cmdBleRadioSetup, &rfParams);
    
    /* Get current time */
    time = RF_getCurrentTime();
    
    while(1)
    {
#ifdef EVERY_OTHER
      if (advCount)
      {
        //set adv data
        RF_cmdAdvNc0.pParams->pAdvData = (uint8_t*)aBeaconData;
        RF_cmdAdvNc0.pParams->advLen = ADVLEN;
        //switch
        advCount = 0;
      }
      else
      {
        //set advertising data
        RF_cmdAdvNc0.pParams->pAdvData = (uint8_t*)bBeaconData;
        RF_cmdAdvNc0.pParams->advLen = ADVLEN;
        //switch 
        advCount = 1;
      }
      time += PACKET_INTERVAL;
#elif // periodic use case  
      if (advCount == 1) //first aBeacon packet
      {
        //set adv data
        RF_cmdAdvNc0.pParams->pAdvData = (uint8_t*)aBeaconData;
        RF_cmdAdvNc0.pParams->advLen = ADVLEN;
        //increment aBeacon counter
        advCount++;
        //set time to next advertisement
        time += PACKET_INTERVAL;
      }
      else if (advCount < ABEACON_COUNT) // aBeacon packet
      {
        //set time for next packet
        time += PACKET_INTERVAL;
        //increment aBeacon counter
        advCount++;
      }
      else if (advCount == ABEACON_COUNT)  //last aBeacon packet
      {
        //delay before bBeacon packet
        time += BBEACON_DELAY;
        //set advertising data
        RF_cmdAdvNc0.pParams->pAdvData = (uint8_t*)bBeaconData;
        RF_cmdAdvNc0.pParams->advLen = ADVLEN;
        //increment aBeacon counter
        advCount++;
      }
      else                              //bBeacon packet
      {
        //reset aBeacon counter
        advCount = 1;
        //don't advertise again until end of period
        time += ADV_PERIOD;
      }
#endif
      
      RF_cmdAdvNc0.startTime = time;
      
      /* Send packet */
      RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdAdvNc0, RF_PriorityNormal, NULL, 0);
      if (!(result & RF_EventLastCmdDone))
      {
        /* Error */
        while(1);
      }
    }
}

/*
 *  ======== main ========
 *
 */
int main(void)
{
    /* Call board init functions. */
    PIN_init(BoardGpioInitTable);

#ifndef POWER_SAVING
    /* Set constraints for Standby, powerdown and idle mode */
    Power_setConstraint(PowerCC26XX_SB_DISALLOW);
    Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
#endif //POWER_SAVING

    /* Initialize task */
    beaconTask_init();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
