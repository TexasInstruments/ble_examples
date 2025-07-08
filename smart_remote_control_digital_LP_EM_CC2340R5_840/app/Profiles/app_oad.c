/******************************************************************************

@file  app_oad.c

@brief This file contains the device info application functionality

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2025, Texas Instruments Incorporated
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

 Copyright (c) 2022-2025, Texas Instruments Incorporated
 All rights reserved.

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.

 ******************************************************************************
 *****************************************************************************/

#include <string.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <app_main.h>
#include "ti/bleapp/profiles/oad/oad_profile.h"
#include "ti/bleapp/util/sw_update/sw_update.h"


OADProfile_AppCommand_e App_OADCallback(OADProfile_App_Msg_e msg)
{
    OADProfile_AppCommand_e cmd = OAD_PROFILE_PROCEED;
    switch(msg)
    {
        case OAD_PROFILE_MSG_REVOKE_IMG_HDR:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: Revoke image header");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_NEW_IMG_IDENDIFY:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: New image identify");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_START_DOWNLOAD:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: Download new image");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_FINISH_DOWNLOAD:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: Download complete");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_RESET_REQ:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: Reset device");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
    }
    return (cmd);
}

/*********************************************************************
 * @fn      OAD_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the OAD profile and service.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t OAD_start(void)
{
  bStatus_t status = SUCCESS;

  OADProfile_start(&App_OADCallback);

  //APP_HDR_ADDR is the place in the flash memory of APP header, and it is imported from the predefined symbols
  //SwUpdate_GetSWVersion function extract image version struct from given address
  struct image_version * img_ver = (struct image_version *)SwUpdate_GetSWVersion(APP_HDR_ADDR);
  MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "App version: %d.%d.%d.%d",
                    img_ver->iv_major,
                    img_ver->iv_minor,
                    img_ver->iv_revision,
                    img_ver->iv_build_num);
  return ( status );
}
