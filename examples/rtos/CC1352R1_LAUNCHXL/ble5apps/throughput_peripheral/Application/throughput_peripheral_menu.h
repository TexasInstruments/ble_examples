/******************************************************************************

 @file  throughput_peripheral_menu.h

 @brief This file contains menu objects for throughput_peripheral.

 Group: WCS BTS
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2016-2020, Texas Instruments Incorporated
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

#ifndef THROUGHPUT_PERIPHERAL_MENU_H
#define THROUGHPUT_PERIPHERAL_MENU_H

#ifdef __cplusplus
extern "C"
{
#endif

// Menu item indices.
#define TP_ITEM_NONE            TBM_ITEM_NONE
#define TP_ITEM_ALL             TBM_ITEM_ALL

// Note: The defines should be updated accordingly if there is any change
//       in the order of the items of the menu objects the items belong to.
#define TP_ITEM_SELECT_CONN     TBM_ITEM(0)  // "Work with"
#define TP_ITEM_AUTOCONNECT     TBM_ITEM(1)  // "Auto Connect"

/*
 * Menus Declarations
 */

/* Main Menu Object */
extern tbmMenuObj_t spMenuMain;

/* Items of (Main) */
/* Action items are defined in simple_peripheral_menu.c */

/*
 * Menus Declarations
 */

/* Main Menu Object */
extern tbmMenuObj_t tpMenuMain;
extern tbmMenuObj_t tpMenuSelectConn;
extern tbmMenuObj_t tpMenuPerConn;
extern tbmMenuObj_t tpMenuChangeAddrType;
extern tbmMenuObj_t tpMenuConnPhy;

void ThroughputPeripheral_buildMenu(void);

#ifdef __cplusplus
}
#endif

#endif /* THROUGHPUT_PERIPHERAL_MENU_H */

