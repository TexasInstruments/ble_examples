/******************************************************************************

@file  app_central.c

@brief This example file demonstrates how to activate the central role with
the help of BLEAppUtil APIs.

Two structures are used for event handling, one for connection events and one
for scanning events.
In each, eventMask is used to specify the events that will be received
and handled.
In addition, structures must be provided for scan init parameters,
scan start and connection init.

In the events handler functions, write what actions are done after each event.

In the Central_start() function at the bottom of the file, registration,
initialization and activation are done using the BLEAppUtil API functions,
using the structures defined in the file.

More details on the functions and structures can be seen next to the usage.

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2023, Texas Instruments Incorporated
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

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include <stdarg.h>

#include "ti_ble_config.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>

//*****************************************************************************
//! Prototypes
//*****************************************************************************

void Central_ScanEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
void Central_addScanRes(GapScan_Evt_AdvRpt_t *pScanRpt);

//*****************************************************************************
//! Globals
//*****************************************************************************

// Events handlers struct, contains the handlers and event masks
// of the application central role module
BLEAppUtil_EventHandler_t centralScanHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_SCAN_TYPE,
    .pEventHandler  = Central_ScanEventHandler,
    .eventMask      = BLEAPPUTIL_SCAN_ENABLED |
                      BLEAPPUTIL_SCAN_DISABLED
};

BLEAppUtil_ConnectParams_t centralConnParams =
{
    .phys = INIT_PHY_1M,
    .timeout = 0
};

const BLEAppUtil_ScanInit_t centralScanInitParams =
{
    /*! Opt SCAN_PRIM_PHY_1M | SCAN_PRIM_PHY_CODED */
    .primPhy                    = DEFAULT_SCAN_PHY,

    /*! Opt SCAN_TYPE_ACTIVE | SCAN_TYPE_PASSIVE */
    .scanType                   = DEFAULT_SCAN_TYPE,

    /*! Scan interval shall be greater than or equal to scan window */
    .scanInterval               = DEFAULT_SCAN_INTERVAL, /* Units of 625 us */

    /*! Scan window shall be less than or equal to scan interval */
    .scanWindow                 = DEFAULT_SCAN_WINDOW, /* Units of 625 us */

    /*! Select which fields of an advertising report will be stored */
    /*! in the AdvRptList, For mor field see @ref Gap_scanner.h     */
    .advReportFields            = ADV_RPT_FIELDS,

    /*! Opt SCAN_PRIM_PHY_1M | SCAN_PRIM_PHY_CODED */
    .scanPhys                   = DEFAULT_SCAN_PHY,

    /*! Opt SCAN_FLT_POLICY_ALL | SCAN_FLT_POLICY_AL |   */
    /*! SCAN_FLT_POLICY_ALL_RPA | SCAN_FLT_POLICY_AL_RPA */
    .fltPolicy                  = SCANNER_FILTER_POLICY,

    /*! For more filter PDU @ref Gap_scanner.h */
    .fltPduType                 = SCANNER_FILTER_PDU_TYPE,

    /*! Opt SCAN_FLT_RSSI_ALL | SCAN_FLT_RSSI_NONE */
    .fltMinRssi                 = SCANNER_FILTER_MIN_RSSI,

    /*! Opt SCAN_FLT_DISC_NONE | SCAN_FLT_DISC_GENERAL | SCAN_FLT_DISC_LIMITED
     *  | SCAN_FLT_DISC_ALL | SCAN_FLT_DISC_DISABLE */
    .fltDiscMode                = SCANNER_FILTER_DISC_MODE,

    /*! Opt SCAN_FLT_DUP_ENABLE | SCAN_FLT_DUP_DISABLE | SCAN_FLT_DUP_RESET */
    .fltDup                     = SCANNER_DUPLICATE_FILTER
};

const BLEAppUtil_ConnParams_t centralConnInitParams =
{
     /*! Opt INIT_PHY_ALL | INIT_PHY_1M | INIT_PHY_2M | INIT_PHY_CODED */
    .initPhys              = DEFAULT_INIT_PHY,

    .scanInterval          = INIT_PHYPARAM_SCAN_INT,      /* Units of 0.625ms */
    .scanWindow            = INIT_PHYPARAM_SCAN_WIN,      /* Units of 0.625ms */
    .minConnInterval       = INIT_PHYPARAM_MIN_CONN_INT,  /* Units of 1.25ms  */
    .maxConnInterval       = INIT_PHYPARAM_MAX_CONN_INT,  /* Units of 1.25ms  */
    .connLatency           = INIT_PHYPARAM_CONN_LAT,
    .supTimeout            = INIT_PHYPARAM_SUP_TO         /* Units of 10ms */
};

static App_scanResults centralScanRes[APP_MAX_NUM_OF_ADV_REPORTS];
static uint8 centralScanIndex = 0;

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      Central_ScanEventHandler
 *
 * @brief   The purpose of this function is to handle scan events
 *          that rise from the GAP and were registered in
 *          @ref BLEAppUtil_registerEventHandler
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void Central_ScanEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    BLEAppUtil_ScanEventData_t *scanMsg = (BLEAppUtil_ScanEventData_t *)pMsgData;

    switch (event)
    {
        case BLEAPPUTIL_SCAN_ENABLED:
        {
            centralScanIndex = 0;
            MenuModule_printf(APP_MENU_SCAN_EVENT, 0, "Scan status: Scan started...");

            break;
        }

        case BLEAPPUTIL_SCAN_DISABLED:
        {
            uint8 i;

            for(i = 0; i < APP_MAX_NUM_OF_ADV_REPORTS; i++)
            {
                memset(&centralScanRes[i], 0, sizeof(App_scanResults));
            }

            // Go over the advertise reports that was saved in the host level and save it
            for (i = 0; i < scanMsg->pBuf->pScanDis.numReport; i++)
            {
              GapScan_Evt_AdvRpt_t advReport;
              // Get the address from the report
              GapScan_getAdvReport(i, &advReport);
              // Add the report to the scan list
              Central_addScanRes(&advReport);
            }

            MenuModule_printf(APP_MENU_SCAN_EVENT, 0, "Scan status: Scan disabled - "
                              "Reason: " MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET
                              "Num results: " MENU_MODULE_COLOR_YELLOW "%d " MENU_MODULE_COLOR_RESET,
                              scanMsg->pBuf->pScanDis.reason,
                              scanMsg->pBuf->pScanDis.numReport);
            break;
        }

        default:
        {
            break;
        }

    }
}

/*********************************************************************
 * @fn      Central_addScanRes
 *
 * @brief   Add a scan result to the scan results list
 *
 * @param   pScanRpt - the adv report to take the data from
 *
 * @return  none
 */
void Central_addScanRes(GapScan_Evt_AdvRpt_t *pScanRpt)
{
    if(centralScanIndex < APP_MAX_NUM_OF_ADV_REPORTS)
    {
        centralScanRes[centralScanIndex].addressType = pScanRpt->addrType;
        memcpy(centralScanRes[centralScanIndex].address, pScanRpt->addr, B_ADDR_LEN);
        centralScanIndex++;
    }
}

/*********************************************************************
 * @fn      Scan_getScanResList
 *
 * @brief   Get the scan result list
 *
 * @param   scanRes - a scan list pointer
 *
 * @return  The number of results in the list
 */
uint8 Scan_getScanResList(App_scanResults **scanRes)
{
    *scanRes = centralScanRes;
    return centralScanIndex;
}

/*********************************************************************
 * @fn      Central_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the central
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Central_start()
{
    bStatus_t status = SUCCESS;

    status = BLEAppUtil_registerEventHandler(&centralScanHandler);
    if(status != SUCCESS)
    {
        // Return status value
        return(status);
    }

    status = BLEAppUtil_scanInit(&centralScanInitParams);
    if(status != SUCCESS)
    {
        // Return status value
        return(status);
    }

    status = BLEAppUtil_setConnParams(&centralConnInitParams);
    if(status != SUCCESS)
    {
        // Return status value
        return(status);
    }

    // Return status value
    return(status);
}

#endif // ( HOST_CONFIG & ( CENTRAL_CFG ) )
