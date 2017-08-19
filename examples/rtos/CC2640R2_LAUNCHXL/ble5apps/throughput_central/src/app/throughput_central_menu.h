/******************************************************************************

 @file  throughput_central_menu.h

 @brief This file contains menu objects for throughput_central.

 Group: WCS BTS
 Target Device: CC2640R2

 ******************************************************************************

 Copyright (c) 2016-2017, Texas Instruments Incorporated
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
 Release Name: simplelink_cc2640r2_sdk_1_35_00_07_eng
 Release Date: 2017-03-23 10:36:21
 *****************************************************************************/

#ifndef SIMPLE_CENTRAL_MENU_H
#define SIMPLE_CENTRAL_MENU_H

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * Menus Declarations
 */

/* Main Menu Object */
extern tbmMenuObj_t sbcMenuMain;

/* Items of (Main) */
extern tbmMenuObj_t sbcMenuSetPhy;
extern tbmMenuObj_t sbcMenuSetDLEPDU;
extern tbmMenuObj_t sbcMenuScanandConnect;

/* Items of (sbcMenuScanandConnect) */
extern bool SimpleBLECentral_doScanAndConnect(uint8 index);

/* Items of (SetPhy) */
extern bool SimpleBLECentral_doSetPhy(uint8 index);

/* Items of (SetDLEPDU) */
extern bool SimpleBLECentral_doSetDLEPDU(uint8 index);

/* Items of (sbcMenuMain) */
extern bool SimpleBLECentral_doToggleRSSI(uint8 index);
extern bool SimpleBLECentral_doDisconnect(uint8 index);

/* Action items are defined in simple_central_menu.c */

#ifdef __cplusplus
}
#endif

#endif /* SIMPLE_CENTRAL_MENU_H */

