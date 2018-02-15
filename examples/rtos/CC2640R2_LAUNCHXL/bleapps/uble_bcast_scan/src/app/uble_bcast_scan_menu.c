/******************************************************************************

 @file       uble_bcast_scan_menu.c

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

#include <bcomdef.h>
#include <ti/display/Display.h>
#include <menu/two_btn_menu.h>
#include "uble_bcast_scan_menu.h"
#include "uble_bcast_scan.h"

/*
 * Menu Lists Initializations
 */

/* Menu: Main
     7 submenus, no upper */
MENU_OBJ(ubsMenuMain, "Main Menu", 7, NULL)
  MENU_ITEM_SUBMENU(&ubsMenuBcastControl)
  MENU_ITEM_SUBMENU(&ubsMenuTxPower)
  MENU_ITEM_SUBMENU(&ubsMenuAdvInterval)
  MENU_ITEM_SUBMENU(&ubsMenuAdvChanMap)
  MENU_ITEM_SUBMENU(&ubsMenuTimeToPrepare)
  MENU_ITEM_SUBMENU(&ubsMenuAdvData)
  MENU_ITEM_SUBMENU(&ubsMenuBcastDuty)
MENU_OBJ_END

/* Menu: TX Power */
MENU_OBJ(ubsMenuTxPower, "Set TX Power", 2, &ubsMenuMain)
  MENU_ITEM_ACTION("Inc TX Power", UBLEBcastScan_doTxPower)
  MENU_ITEM_ACTION("Dec TX Power", UBLEBcastScan_doTxPower)
MENU_OBJ_END

/* Menu: Adv Interval */
MENU_OBJ(ubsMenuAdvInterval, "Set Adv Interval", 2, &ubsMenuMain)
  MENU_ITEM_ACTION("Inc Adv Interval", UBLEBcastScan_doAdvInterval)
  MENU_ITEM_ACTION("Dec Adv Interval", UBLEBcastScan_doAdvInterval)
MENU_OBJ_END

/* Menu: Adv Channel Map */
MENU_OBJ(ubsMenuAdvChanMap, "Set Adv Chan Map", 2, &ubsMenuMain)
  MENU_ITEM_ACTION("Inc Adv Chan Map", UBLEBcastScan_doAdvChanMap)
  MENU_ITEM_ACTION("Dec Adv Chan Map", UBLEBcastScan_doAdvChanMap)
MENU_OBJ_END

/* Menu: Time to Adv */
MENU_OBJ(ubsMenuTimeToPrepare, "Set TimeToPrepare", 2, &ubsMenuMain)
  MENU_ITEM_ACTION("Inc TimeToPrepare", UBLEBcastScan_doTimeToPrepare)
  MENU_ITEM_ACTION("Dec TimeToPrepare", UBLEBcastScan_doTimeToPrepare)
MENU_OBJ_END

/* Menu: Adv Data */
MENU_OBJ(ubsMenuAdvData, "Set Adv Data", 5, &ubsMenuMain)
  MENU_ITEM_SUBMENU(&ubsMenuUpdateOption)
  MENU_ITEM_ACTION("Eddystone UID", UBLEBcastScan_doSetFrameType)
  MENU_ITEM_ACTION("Eddystone URL", UBLEBcastScan_doSetFrameType)
  MENU_ITEM_ACTION("Eddystone TLM", UBLEBcastScan_doSetFrameType)
  MENU_ITEM_SUBMENU(&ubsMenuTimeToPrepare)
MENU_OBJ_END

/* Menu: Update Option */
MENU_OBJ(ubsMenuUpdateOption, "Update Option", 2, &ubsMenuAdvData)
  MENU_ITEM_ACTION("Immediately", UBLEBcastScan_doUpdateOption)
  MENU_ITEM_ACTION("Upon Prepare Evt", UBLEBcastScan_doUpdateOption)
MENU_OBJ_END

/* Menu: Bcast Duty */
MENU_OBJ(ubsMenuBcastDuty, "Bcast Duty Control", 2, &ubsMenuMain)
  MENU_ITEM_SUBMENU(&ubsMenuBcastDutyOnTime)
  MENU_ITEM_SUBMENU(&ubsMenuBcastDutyOffTime)
MENU_OBJ_END

/* Menu: Bcast Duty - On Time */
MENU_OBJ(ubsMenuBcastDutyOnTime, "Set Duty On Time", 3, &ubsMenuBcastDuty)
  MENU_ITEM_ACTION("Inc Duty On Time", UBLEBcastScan_doBcastDutyOnTime)
  MENU_ITEM_ACTION("Dec Duty On Time", UBLEBcastScan_doBcastDutyOnTime)
  MENU_ITEM_SUBMENU(&ubsMenuMain)
MENU_OBJ_END

/* Menu: Bcast Duty - Off Time */
MENU_OBJ(ubsMenuBcastDutyOffTime, "Set Duty Off Time", 3, &ubsMenuBcastDuty)
  MENU_ITEM_ACTION("Inc Duty Off Time", UBLEBcastScan_doBcastDutyOffTime)
  MENU_ITEM_ACTION("Dec Duty Off Time", UBLEBcastScan_doBcastDutyOffTime)
  MENU_ITEM_SUBMENU(&ubsMenuMain)
MENU_OBJ_END

/* Menu: Bcast Control */
MENU_OBJ(ubsMenuBcastControl, "Bcast Start/Stop", 3, &ubsMenuMain)
  MENU_ITEM_ACTION("Start(Indef. Adv)", UBLEBcastScan_doBcastStart)
  MENU_ITEM_ACTION("Start(100 Adv\'s)", UBLEBcastScan_doBcastStart)
  MENU_ITEM_ACTION("Stop", UBLEBcastScan_doBcastStop)
MENU_OBJ_END

