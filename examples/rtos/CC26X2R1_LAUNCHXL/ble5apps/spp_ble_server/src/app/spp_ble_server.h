/******************************************************************************

 @file  spp_ble_server.h

 @brief This file contains the SPP BLE Server sample application
        definitions and prototypes.

 Group: CMCU, LPRF
 Target Device: CC2652

 ******************************************************************************

 Copyright (c) 2013-2018, Texas Instruments Incorporated
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

 *****************************************************************************/

#ifndef SPP_BLE_SERVER_H
#define SPP_BLE_SERVER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */
// External Events for RTOS application
#define SBS_UART_DATA_EVT                     0x0010
#define SBS_UART_ERROR_EVT                    0x0020

//LED parameters
#define Board_LED_TOGGLE                      3
#define BLINK_DURATION                        500 // Milliseconds

 /* Delay */
#define delay_ms(i) ( CPUdelay(12000*(i)) )

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the SPP BLE Server.
 */
extern void SPPBLEServer_createTask(void);

/*
 * Functions for menu action
 */
/* Actions for Menu: Choose connection to work with */
bool SPPBLEServer_doSelectConn(uint8 index);
extern char* convInt32ToText(int32 value);
/* Actions for Menu: Set PHY - Select */
bool SPPBLEServer_doSetConnPhy(uint8 index);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SPP_BLE_SERVER_H */
