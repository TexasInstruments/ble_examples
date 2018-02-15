/******************************************************************************

 @file       uble_bcast_scan.h

 @brief This file contains macros, type definitions, and function prototypes
        for the uble_bcast_scan app.

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

#ifndef MICROBLETEST_H
#define MICROBLETEST_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <menu/two_btn_menu.h>

/*********************************************************************
*  EXTERNAL VARIABLES
*/
/* Main Menu Object */
extern tbmMenuObj_t ubsMenuMain;

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      UBLEBcastScan_doTxPower
 *
 * @brief   Increment/Decrement the TX Power level
 *
 * @param   index - index of the menu item used to determine increment/decrement
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doTxPower(uint8 index);

/*********************************************************************
 * @fn      UBLEBcastScan_doAdvInterval
 *
 * @brief   Increment/Decrement the advertising interval
 *
 * @param   index - index of the menu item used to determine increment/decrement
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doAdvInterval(uint8 index);

/*********************************************************************
 * @fn      UBLEBcastScan_doAdvChanMap
 *
 * @brief   Enable/disable various advertising channels (37, 38, 39)
 *
 * @param   index - index of the menu item used to determine increment/decrement
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doAdvChanMap(uint8 index);

/*********************************************************************
 * @fn      UBLEBcastScan_doTimeToPrepare
 *
 * @brief   Increment/Decrement the time when the application is
 *          notified before an advertisement event
 *
 * @param   index - index of the menu item used to determine increment/decrement
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doTimeToPrepare(uint8 index);

/*********************************************************************
 * @fn      UBLEBcastScan_doSetFrameType
 *
 * @brief   Update advertisement data with a given Eddystone frame type
 *
 * @param   index - index of the menu item used to choose frame offset
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doSetFrameType(uint8 index);

/*********************************************************************
 * @fn      UBLEBcastScan_doUpdateOption
 *
 * @brief   Toggle the update of the advertisement data to be immediate
 *          or on an event.
 *
 * @param   index - Used to choose whether immediate update or at event
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doUpdateOption(uint8 index);

/*********************************************************************
 * @fn      UBLEBcastScan_doBcastDutyOnTime
 *
 * @brief   Change the "on" duty cycle of the broadcast
 *
 * @param   index - index of the menu item used to determine increment/decrement
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doBcastDutyOnTime(uint8 index);

/*********************************************************************
 * @fn      UBLEBcastScan_doBcastDutyOffTime
 *
 * @brief   Change the "off" duty cycle of the broadcast
 *
 * @param   index - index of the menu item used to determine increment/decrement
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doBcastDutyOffTime(uint8 index);

/*********************************************************************
 * @fn      UBLEBcastScan_doBcastStart
 *
 * @brief   Start broadcasting
 *
 * @param   index - Used to select number of adv vs indefinite
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doBcastStart(uint8 index);

/*********************************************************************
 * @fn      UBLEBcastScan_doBcastStop
 *
 * @brief   Stop broadcasting
 *
 * @param   index - unused
 *
 * @return  status - true if successful false otherwise
 */
bool UBLEBcastScan_doBcastStop(uint8 index);

/*********************************************************************
 * @fn      UBLEBcastScan_createTask
 *
 * @brief   Create Micro BLE stack broadcaster and observer task
 */
extern void UBLEBcastScan_createTask(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* MICROBLETEST_H */
