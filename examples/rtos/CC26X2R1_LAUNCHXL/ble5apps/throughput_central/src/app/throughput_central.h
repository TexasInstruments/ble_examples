/******************************************************************************

 @file  throughput_central.h

 @brief This file contains the Throughput Central sample application
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

#ifndef THROUGHPUTCENTRAL_H
#define THROUGHPUTCENTRAL_H

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

// Maximum number of scan results.
// Note: this value cannot be greater than the number of items reserved in
// scMenuConnect (See throughput_central_menu.c)
// This cannot exceed 27 (two-button menu's constraint)
#define DEFAULT_MAX_SCAN_RES                 8

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */
/*
 * Task creation function for the Throughput Central.
 */
extern void ThroughputCentral_createTask(void);

/*
 * Functions for menu action
 */

/* Action for Menu: Set Scanning PHY */
bool ThroughputCentral_doSetScanPhy(uint8_t index);

/* Action for Menu: Enable Scanning */
bool ThroughputCentral_doDiscoverDevices(uint8_t index);

/* Action for Menu: Disable Scanning */
bool ThroughputCentral_doStopDiscovering(uint8_t index);

/* Action for Menu: Connect */
bool ThroughputCentral_doConnect(uint8_t index);

/* Action for Menu: Cancel Connecting */
bool ThroughputCentral_doCancelConnecting(uint8_t index);

/* Action for Menu: Select Connection */
bool ThroughputCentral_doSelectConn(uint8_t index);

/* Action for Menu: GATT Read */
bool ThroughputCentral_doGattRead(uint8_t index);

/* Action for Menu: GATT Write */
bool ThroughputCentral_doGattWrite(uint8_t index);

/* Action for Menu: Initiate Connection Update Procedure */
bool ThroughputCentral_doConnUpdate(uint8_t index);

/* Action for Menu: Set Connection PHY */
bool ThroughputCentral_doSetConnPhy(uint8_t index);

/* Action for Menu: Set Connection PDU */
bool ThroughputCentral_doSetDLEPDU(uint8_t index);

/* Action for Menu: Toggle Throughput */
bool ThroughputCentral_doToggleThroughput(uint8_t index);

/* Action for Menu: Disconnect */
bool ThroughputCentral_doDisconnect(uint8_t index);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* THROUGHPUTCENTRAL_H */
