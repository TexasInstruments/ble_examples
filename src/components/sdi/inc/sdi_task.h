/******************************************************************************

 @file  sdi_task.h

  SDI is a TI RTOS Application Thread that provides a
  common Network Processor Interface framework

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************

 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _SDITask_H
#define _SDITask_H

#ifdef __cplusplus
extern "C"
{
#endif

// ****************************************************************************
// includes
// ****************************************************************************
#include "inc/sdi_data.h"
#include <ti/sysbios/knl/Semaphore.h>
// ****************************************************************************
// defines
// ****************************************************************************
#define DEBUG(x) SDITask_sendToUART(x, strlen((const char*)x));
#define DEBUG_NEWLINE() DEBUG("\n\r")
// ****************************************************************************
// typedefs
// ****************************************************************************

// -----------------------------------------------------------------------------
//! \brief      Typedef for call back function mechanism to reroute incoming SDI
//!             messages.
//!             NOTE: Definer MUST copy contents to local buffer.  SDI task will
//!             free this memory.
//!             NOTE: The contained message buffer does NOT include any "framing"
//!             bytes, ie. SOF, FCS etc.
//! \param[in]  pMsg   Pointer to "unframed" message buffer.
//!
//! \return     void
// -----------------------------------------------------------------------------
typedef void (*sdiIncomingEventCBack_t)(uint8_t event, uint8_t *pMsg, uint8_t len);

//*****************************************************************************
// globals
//*****************************************************************************

//*****************************************************************************
// function prototypes
//*****************************************************************************

// -----------------------------------------------------------------------------
//! \brief      SDI task creation function
//!
//! \return     void
// -----------------------------------------------------------------------------
Void SDITask_createTask(void);

// -----------------------------------------------------------------------------
//! \brief      Register callback function to reroute incoming (from UART)
//!             SDI messages.
//!
//! \param[in]  appRxCB   Callback fucntion.
//!
//! \return     void
// -----------------------------------------------------------------------------
extern void SDITask_registerIncomingRXEventAppCB(sdiIncomingEventCBack_t appRxCB);

// -----------------------------------------------------------------------------
//! \brief      API for application task to send a message to the Host.
//!             NOTE: It's assumed all message traffic to the stack will use
//!             other (ICALL) APIs/Interfaces.
//!
//! \param[in]  pMsg    Pointer to "unframed" message buffer.
//! \param[in]  length  Length of buffer
//!
//! \return     void
// -----------------------------------------------------------------------------
extern void SDITask_sendToUART(uint8_t *pMsg, uint16_t length);

// -----------------------------------------------------------------------------
//! \brief      API for application task to set packet data size to send over the air.
//!
//! \param[in]  mtuSize    GATT MTU size.
//!
//! \return     void
// -----------------------------------------------------------------------------
extern void SDITask_setAppDataSize(uint16_t mtuSize);

#ifdef __cplusplus
{
#endif // extern "C"

#endif // end of _SDITask_H definition
