/******************************************************************************

 @file       uble_bcast_scan_menu.h

 @brief This file contains macros, type definitions, and function prototypes
        for two-button menu implementation.

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

 *****************************************************************************/

#ifndef UBLE_BCAST_SCAN_MENU_H
#define UBLE_BCAST_SCAN_MENU_H

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * Menus Declarations
 */

/* Main Menu Object */
extern tbmMenuObj_t ubsMenuMain;

/* Items of Main */
extern tbmMenuObj_t ubsMenuInit;
extern tbmMenuObj_t ubsMenuTxPower;
extern tbmMenuObj_t ubsMenuAdvInterval;
extern tbmMenuObj_t ubsMenuAdvChanMap;
extern tbmMenuObj_t ubsMenuTimeToPrepare;
extern tbmMenuObj_t ubsMenuAdvData;
extern tbmMenuObj_t ubsMenuBcastDuty;
extern tbmMenuObj_t ubsMenuBcastControl;

/* Items of (Init) */
extern tbmMenuObj_t ubsMenuAddrStatic;
/* Action items are defined in uble_bcast_scan.h */

/* Items of (TX Power) */
/* Action items are defined in uble_bcast_scan.h */

/* Items of (Adv Interval) */
/* Action items are defined in uble_bcast_scan.h */

/* Items of (Adv Channel Map) */
/* Action items are defined in uble_bcast_scan.h */

/* Items of (Time to Adv) */
/* Action items are defined in uble_bcast_scan.h */

/* Items of (Adv Data) */
extern tbmMenuObj_t ubsMenuAdvData;
/* Action items are defined in uble_bcast_scan.h */

/* Items of (Update Option) */
extern tbmMenuObj_t ubsMenuUpdateOption;
/* Action items are defined in uble_bcast_scan.h */

/* Items of (Bcast Duty) */
extern tbmMenuObj_t ubsMenuBcastDutyOffTime;
extern tbmMenuObj_t ubsMenuBcastDutyOnTime;

/* Items of (Bcast Duty - Off Time) */
/* Action items are defined in uble_bcast_scan.h */

/* Items of (Bcast Duty - On Time) */
/* Action items are defined in uble_bcast_scan.h */

/* Items of (Bcast Control) */
/* Action items are defined in uble_bcast_scan.h */

#ifdef __cplusplus
}
#endif

#endif /* UBLE_BCAST_SCAN_MENU_H */

