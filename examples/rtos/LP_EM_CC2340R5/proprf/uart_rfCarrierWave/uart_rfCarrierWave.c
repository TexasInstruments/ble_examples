/*
 * Copyright (c) 2013-2022, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* The following example is provided as-is, with any and all faults,
 * this is not meant to be a production quality example in line with
 * the SDK examples. The following example is meant to demonstrate one
 * way to use UART to select specs for rfCarrierWave.
 *
 * KNOWN ISSUES:
 * The program can become unresponsive (to UART inputs) after idling for enough time.
 *
 * This example is based off of the 7.20.01.10 F3 SDK ported and tested on the 7.40.00.64 SDK
 * Note that in the 7.40 SDK the PHY can be generated directly in the syscfg, however this code
 * still uses the setup folder with the various PHYs from SmartRF Studio 8 (default examples) is used.
 *
 * IMPORTANT: When using this example, use PuTTY and force line editing on under the terminal before opening
 * a session.
 *
 */

/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

/* TI Drivers */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/rcl/RCL.h>
#include <ti/drivers/rcl/RCL_Scheduler.h>
#include <ti/drivers/rcl/commands/generic.h>
#include "ti_drivers_config.h"
#include <ti/drivers/UART2.h>
#include <ti/devices/cc23x0r5/cmsis/cc23x0r5.h>
#include <ti/devices/cc23x0r5/cmsis/core/core_cm0plus.h>


/***** Variable Declarations *****/
/* RCL Commands */
RCL_CmdGenericTxTest   txCmd;           // TX Test command

/* RCL Client used to open RCL */
static RCL_Client  rclClient;

/* Counters for RCL event callback */
volatile uint32_t gCmdDone = 0;         // Command done



/***** Callback functions *****/
void defaultCallback(RCL_Command *cmd, LRF_Events lrfEvents, RCL_Events rclEvents)
{
    if (rclEvents.lastCmdDone)
    {
        gCmdDone += 1;
        GPIO_toggle(CONFIG_GPIO_LED_0);
    }
}
/* MsgNextString looks for the space in a string, example if "hello world" is entered, then mns will return just "world" */
char *MsgNextString(char *buffer) 
{
    char *mns;
    if(buffer == NULL)
        return NULL;
    mns = strchr(buffer, ' ');

    if(!mns)
        return NULL;
    while(*mns == ' ')
        mns++;
    if(!*mns)
        return NULL;
    return mns;
}

/***** Globals *****/
uint32_t phy   = 0;
uint32_t freq  = 0;
uint32_t freqM = 0;
uint32_t txpow = 0;
uint32_t start = 0;
RCL_Handle rclHandle = 0;
/***** Keyword Commands, add more or less as needed *****/
const char* valPhy = "PHY";
const char* valFrq = "FRQ";
const char* valTxp = "TXP";
const char* valSrt = "SRT";
const char* stop   = "STP";
const char* reset  = "RST";
/***** Function definitions *****/
void *mainThread(void *arg0)
{
    char input[32]; // note that the inputs buffer is only 32 bytes, so if more is needed configure this as needed
    memset(input, 0, strlen(input));
    const char echoPrompt[] = "rfCarrier Selector\r\nType in the three letter keywords in CAPS found below to select the following menu options:\r\nType PHY to select communication protocol\r\nType FRQ to select frequency\r\nType TXP to select transmit power\r\nType SRT to start the test\r\nType STP to stop the test\r\nType RST to reset the device\r\n";
    const char numberPrompt[] = "Please enter a number\r\n";
    const char dividerPrompt[] = "--------------------------------------------------\r\n";
    UART2_Handle uart;
    UART2_Params uartParams;
    size_t bytesRead;
    size_t bytesWritten = 0;
    uint32_t status     = UART2_STATUS_SUCCESS;

    /* Initialize and open RCL */
    RCL_init();
    GPIO_init();
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW); //The GPIO is not used much in this project, could ignore and remove
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    /* Create a UART where the default read and write mode is BmnsKING */
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200; //BaudRate, make sure to match this in PuTTY

    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL)
    {
        /* UART2_open() failed */
        while (1) {}
    }
    UART2_write(uart, echoPrompt, sizeof(echoPrompt), &bytesWritten); //Print out of the menu
    UART2_write(uart, dividerPrompt, sizeof(dividerPrompt), &bytesWritten); //Print out the menu divider
    while(1) //The UART is read/handled in a while(1)
    {
               bytesRead = 0;
               status = UART2_read(uart, input, sizeof(input), &bytesRead);
               UART2_flushRx(uart);
               char buffer[32];
               memset(buffer, 0, strlen(buffer));
               strcpy(buffer, input);
               if (status != UART2_STATUS_SUCCESS)
               {
                   /* UART2_read() failed */
                   while (1) {}
               }
               if(strncmp(buffer, valPhy, 3) == 0) //PHY select command that is used when "PHY #" is typed in UART
               {
                   const char phyPrompt[] = "Phy Selected: ";
                   char *mns;
                   mns = MsgNextString(buffer); //look for a number that lies in between 0-4.
                   if(!mns)
                       UART2_write(uart, "\r\nPut a number in\r\n", 18, &bytesWritten);

                   if(strncmp(mns, "0", 1) == 0)
                   {
                       phy = 0;
                       UART2_write(uart, phyPrompt, sizeof(phyPrompt), &bytesWritten);
                       UART2_write(uart, mns, 1, &bytesWritten);
                       UART2_write(uart, ", 1 mbps BLE", 12, &bytesWritten);
                       UART2_write(uart, "\r\n", 2, &bytesWritten);
                       memset(mns, 0, strlen(mns));
                   }
                   else if(strncmp(mns, "1", 1) == 0)
                   {
                       phy = 1;
                       UART2_write(uart, phyPrompt, sizeof(phyPrompt), &bytesWritten);
                       UART2_write(uart, mns, 1, &bytesWritten);
                       UART2_write(uart, ", 250 kbps msk", 14, &bytesWritten);
                       UART2_write(uart, "\r\n", 2, &bytesWritten);
                       memset(mns, 0, strlen(mns));
                   }
                   else if(strncmp(mns, "2", 1) == 0)
                   {
                       phy = 2;
                       UART2_write(uart, phyPrompt, sizeof(phyPrompt), &bytesWritten);
                       UART2_write(uart, mns, 1, &bytesWritten);
                       UART2_write(uart, ", 250 kbps fec msk", 18, &bytesWritten);
                       UART2_write(uart, "\r\n", 2, &bytesWritten);
                       memset(mns, 0, strlen(mns));
                   }
                   else if(strncmp(mns, "3", 1) == 0)
                   {
                       phy = 3;
                       UART2_write(uart, phyPrompt, sizeof(phyPrompt), &bytesWritten);
                       UART2_write(uart, mns, 1, &bytesWritten);
                       UART2_write(uart, ", 2 mbps BLE", 12, &bytesWritten);
                       UART2_write(uart, "\r\n", 2, &bytesWritten);
                       memset(mns, 0, strlen(mns));
                   }
                   else if(strncmp(mns, "4", 1) == 0)
                   {
                       phy = 4;
                       UART2_write(uart, phyPrompt, sizeof(phyPrompt), &bytesWritten);
                       UART2_write(uart, mns, 1, &bytesWritten);
                       UART2_write(uart, ", coded BLE", 11, &bytesWritten);
                       UART2_write(uart, "\r\n", 2, &bytesWritten);
                       memset(mns, 0, strlen(mns));
                   }
                   else
                       UART2_write(uart, "Please type a number: 0,1,2,3,4 in\r\n", 32, &bytesWritten);
                   memset(buffer, 0, strlen(buffer));
                   memset(input, 0, strlen(input));
                   memset(mns, 0, strlen(mns));
                   UART2_flushRx(uart);
               }
               else if(strncmp(buffer, valFrq, 3) == 0) //Frequency select command that is used when "FRQ ####" is typed in UART, where #### is your frequency in MHz
               {
                   const char echoPrompt2[] = "You Entered a Frequency of: ";
                   UART2_write(uart, echoPrompt2, sizeof(echoPrompt2), &bytesWritten);
                   char *mns;
                   mns = MsgNextString(buffer);
                   if(!mns)
                       UART2_write(uart, numberPrompt, sizeof(numberPrompt), &bytesWritten);
                   else
                   {
                       freqM = atoi(mns);
                       freq = freqM * 1000000; //Multiply UART input by a million, if you type in 2440, it turns into 2.440 GHz (or 2440 MHz).
                   }
                   UART2_write(uart, mns, sizeof(mns), &bytesWritten);
                   UART2_write(uart, " MHz", 4, &bytesWritten);
                   UART2_write(uart, "\r\n", 2, &bytesWritten);
                   memset(buffer, 0, strlen(buffer));
                   memset(input, 0, strlen(input));
                   UART2_flushRx(uart);
               }
               else if(strncmp(buffer, valTxp, 3) == 0) //TX Power command that is used when "TXP MAX/MIN" is typed in UART
               {
                   const char echoPrompt3[] = "You Selected: TXP ";
                   UART2_write(uart, echoPrompt3, sizeof(echoPrompt3), &bytesWritten);
                   char *mns;
                   mns = MsgNextString(buffer);
                   if(!mns)
                       UART2_write(uart, numberPrompt, sizeof(numberPrompt), &bytesWritten);
                   else if(strncmp(mns, "MAX", 3) == 0) //there are two predefined options for TxPower, being Min and Max
                   {
                       txpow = 1;
                   }
                   else if(strncmp(mns, "MIN", 3) == 0)
                   {
                       txpow = 0;
                   }
                   UART2_write(uart, mns, sizeof(mns), &bytesWritten);
                   UART2_write(uart, "\r\n", 2, &bytesWritten);
                   memset(buffer, 0, strlen(buffer));
                   memset(input, 0, strlen(input));
                   UART2_flushRx(uart);
               }
               else if(strncmp(buffer, valSrt, 3) == 0) //start rfCarrierTX when "SRT" is typed in UART
               {
                   const char startPrompt[] = "Start RF Carrier\r\n";
                   UART2_write(uart, startPrompt, sizeof(startPrompt), &bytesWritten);
                   memset(buffer, 0, strlen(buffer));
                   memset(input, 0, strlen(input));

                   /* The following six lines are found from the rcl_settings_...*/
                   extern const LRF_Config       LRF_configMsk250Kbps;
                   extern const LRF_Config       LRF_configMsk250KbpsFec;
                   extern const LRF_Config       LRF_configBle;
                   #define RCL_PHY_FEATURE_SUB_PHY_1_MBPS_BLE      0x0000
                   #define RCL_PHY_FEATURE_SUB_PHY_2_MBPS_BLE      0x0001
                   #define RCL_PHY_FEATURE_SUB_PHY_CODED_BLE       0x0002

                   /* Initialize and open RCL */
                   if(phy == 0)  //these are the selectable PHYs
                   {
                       txCmd.common.phyFeatures = RCL_PHY_FEATURE_SUB_PHY_1_MBPS_BLE;
                       rclHandle = RCL_open(&rclClient, &LRF_configBle);
                   }
                   else if(phy == 1)
                   {
                       rclHandle = RCL_open(&rclClient, &LRF_configMsk250Kbps); 
                   }
                   else if(phy == 2)
                   {
                       rclHandle = RCL_open(&rclClient, &LRF_configMsk250KbpsFec);
                   }
                   else if(phy == 3)
                   {
                       txCmd.common.phyFeatures = RCL_PHY_FEATURE_SUB_PHY_2_MBPS_BLE ;
                       rclHandle = RCL_open(&rclClient, &LRF_configBle);
                   }
                   else if(phy == 4)
                   {
                       txCmd.common.phyFeatures = RCL_PHY_FEATURE_SUB_PHY_CODED_BLE;
                       rclHandle = RCL_open(&rclClient, &LRF_configBle);
                   }

                   /* Setup generic transmit test command */
                   txCmd = RCL_CmdGenericTxTest_DefaultRuntime();

                   /* Set RF frequency */
                   txCmd.rfFrequency = freq; 

                   /* Start command as soon as possible */
                   txCmd.common.scheduling = RCL_Schedule_Now;
                   txCmd.common.status = RCL_CommandStatus_Idle;

                   txCmd.config.sendCw = 1U;       // Send CW
                   txCmd.config.whitenMode = 1U;   // Default whitening
                   txCmd.config.txWord = 0U;       // Repeated word to transmit
                   txCmd.config.fsOff = 1;         // Turn off FS
                   if(txpow == 1)
                       txCmd.txPower = LRF_TxPower_Use_Max;          // Set TXPower to its maximum
                   else
                       txCmd.txPower = LRF_TxPower_Use_Min;          // Set TXPower to its minimum

                   /* Callback triggers on last command done */
                   txCmd.common.runtime.callback = defaultCallback;
                   txCmd.common.runtime.rclCallbackMask.value = RCL_EventLastCmdDone.value;

                   /* Submit command */
                   RCL_Command_submit(rclHandle, &txCmd);
                   char sbuffer[64]; //buffer for sprintf function
                   memset(sbuffer, 0, strlen(sbuffer)); //set to zero to clean the buffer just in case some data was leftover
                   sprintf(sbuffer, "Freq %u MHz, TxPower ", freqM); //place the frequency global (MHz) into the buffer)
                   UART2_write(uart, sbuffer, sizeof(sbuffer), &bytesWritten);
                   if(txpow == 1) //write either MAX or MIN txpower based off of "txpow" global
                       UART2_write(uart, "MAX", 3, &bytesWritten);
                   else
                       UART2_write(uart, "MIN", 3, &bytesWritten);

                   if(phy == 0) //write which PHY is selected using "phy" global
                       UART2_write(uart, ", 1 mbps BLE", 12, &bytesWritten);
                   else if(phy == 1)
                       UART2_write(uart, ", 250 kbps msk", 14, &bytesWritten);
                   else if(phy == 2)
                       UART2_write(uart, ", 250 kbps fec msk", 18, &bytesWritten);
                   else if(phy == 3)
                       UART2_write(uart, ", 2 mbps BLE", 12, &bytesWritten);
                   else if(phy == 4)
                       UART2_write(uart, ", coded BLE", 11, &bytesWritten);
                   UART2_write(uart, "\r\n", 2, &bytesWritten);
                   memset(sbuffer, 0, strlen(sbuffer));
                   memset(buffer, 0, strlen(buffer));
                   memset(input, 0, strlen(input));
                   UART2_flushRx(uart);
               }
               else if(strncmp(buffer, stop, 3) == 0) //when "STP" is called, stop the RCL, then close RCL
               {
                   RCL_Command_stop(&txCmd, RCL_StopType_Hard);
                   RCL_close(rclHandle);
                   const char stopPrompt[] = "Stopped rfCarrierTX\r\n";
                   UART2_write(uart, stopPrompt, sizeof(stopPrompt), &bytesWritten);
                   memset(buffer, 0, strlen(buffer));
                   memset(input, 0, strlen(input));
                   UART2_flushRx(uart);
               }
               else if(strncmp(buffer, reset, 3) == 0) //when "RST" is called set all globals to zero, then use the NVIC_SystemReset to reset the device
               {
                   const char resetPrompt[] = "Values reset\r\n";
                   UART2_write(uart, resetPrompt, sizeof(resetPrompt), &bytesWritten);
                   memset(buffer, 0, strlen(buffer));
                   memset(input, 0, strlen(input));
                   UART2_flushRx(uart);
                   __NVIC_SystemReset();
               }
               memset(buffer, 0, strlen(buffer));
               memset(input, 0, strlen(input));
               UART2_flushRx(uart);
             bytesWritten = 0;

    }
}
