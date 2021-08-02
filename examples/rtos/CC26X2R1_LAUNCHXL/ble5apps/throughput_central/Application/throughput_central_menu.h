/******************************************************************************

 @file  throughput_central_menu.h

 @brief This file contains menu objects for throughput_central.

 Group: WCS BTS
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2016-2021, Texas Instruments Incorporated
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

#ifndef THROUGHPUT_CENTRAL_MENU_H
#define THROUGHPUT_CENTRAL_MENU_H

// Duplicate menu item indices.
#define TC_ITEM_NONE        TBM_ITEM_NONE
#define TC_ITEM_ALL         TBM_ITEM_ALL

// Note: The defines should be updated accordingly if there is any change
//       in the order of the items of the menu objects the items belong to.
#define TC_ITEM_SELECTCONN      TBM_ITEM(0)  // "Work With"
#define TC_ITEM_CONNECT         TBM_ITEM(1)       // "Connect To"
#define TC_ITEM_CANCELCONN      TBM_ITEM(2)   // "Stop Discovering"
#define TC_ITEM_STARTDISC       TBM_ITEM(3)   // "Discover Devices"
#define TC_ITEM_STOPDISC        TBM_ITEM(4)       // "Stop Discovering"
#define TC_ITEM_SCANPHY         TBM_ITEM(5)     // "Set Scanning PHY"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Menus Declarations
 */

/* Main Menu Object */
extern tbmMenuObj_t tcMenuMain;
extern tbmMenuObj_t tcMenuConnect;
extern tbmMenuObj_t tcMenuScanPhy;
extern tbmMenuObj_t tcMenuAutoConnect;
extern tbmMenuObj_t tcMenuConnPhy;
extern tbmMenuObj_t tcMenuSelectConn;
extern tbmMenuObj_t tcMenuPerConn;
extern tbmMenuObj_t tcMenuGattWrite;

void ThroughputCentral_buildMenu(void);

/* Items of (Main) */
/* Action items are defined in throughput_central_menu.c */

#ifdef __cplusplus
}
#endif

#endif /* THROUGHPUT_CENTRAL_MENU_H */

