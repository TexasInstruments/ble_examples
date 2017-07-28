/******************************************************************************

 @file  sdi_tl.c

 SDI Transport Layer API

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


// ****************************************************************************
// includes
// ****************************************************************************
#include <string.h>
#include <xdc/std.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "Board.h"
#include "ICall.h"
#include "hal_types.h"
#include "inc/sdi_tl.h"
#include "inc/sdi_config.h"

// ****************************************************************************
// defines
// ****************************************************************************

#if defined(SDI_USE_SPI)
#include "inc/sdi_tl_spi.h"
#elif defined(SDI_USE_UART)
#include "inc/sdi_tl_uart.h"
#else
#error Must define an underlying serial bus for SDI
#endif


// ****************************************************************************
// typedefs
// ****************************************************************************

//*****************************************************************************
// globals
//*****************************************************************************

//! \brief Flag for low power mode
static volatile bool sdiPMSetConstraint = FALSE;

//! \brief Flag for ongoing SDI TX
static volatile bool sdiTxActive = FALSE;

//! \brief The packet that was being sent when HWI of MRDY going low was received
static volatile uint32 mrdyPktStamp = 0;

//! \brief Packets transmitted counter
static uint32 txPktCount = 0;

//! \brief SDI Transport Layer receive buffer
static Char sdiRxBuf[SDI_TL_BUF_SIZE];

//! \brief Index to last byte written into SDI Transport Layer receive buffer
static uint16_t sdiRxBufTail = 0;

//! \brief Index to first byte to be read from SDI Transport Layer receive buffer
static uint16_t sdiRxBufHead = 0;

//! \brief SDI Transport Layer transmit buffer
static Char sdiTxBuf[SDI_TL_BUF_SIZE];

//! \brief Number of bytes in SDI Transport Layer transmit buffer
static uint16_t sdiTxBufLen = 0;

//! \brief Call back function in SDI Task for transmit complete
static sdiRtosCB_t taskTxCB = NULL;

//! \brief Call back function in SDI Task for receive complete
static sdiRtosCB_t taskRxCB = NULL;

//! \brief The remainder of any message that is fragmented
static uint8 *msgFrag = NULL;

//! \brief The length of the remaining message fragment
static uint16 msgFragLen = 0;

#ifdef POWER_SAVING
//! \brief Call back function in SDI Task for MRDY hardware interrupt
static sdiMrdyRtosCB_t taskMrdyCB = NULL;

//! \brief PIN Config for Mrdy and Srdy signals
static PIN_Config sdiHandshakePinsCfg[] =
{
    MRDY_PIN | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP,
    SRDY_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

//! \brief PIN State for Mrdy and Srdy signals
static PIN_State sdiHandshakePins;

//! \brief PIN Handles for Mrdy and Srdy signals
static PIN_Handle hSdiHandshakePins;

//! \brief No way to detect whether positive or negative edge with PIN Driver
//!             Use a flag to keep track of state
static uint8 mrdy_state;
#endif

//*****************************************************************************
// function prototypes
//*****************************************************************************

//! \brief Call back function provided to underlying serial interface to be
//              invoked upon the completion of a transmission
static void SDITL_transmissionCallBack(uint16 Rxlen, uint16 Txlen);

#ifdef POWER_SAVING
//! \brief HWI interrupt function for MRDY
static void SDITL_MRDYPinHwiFxn(PIN_Handle hPin, PIN_Id pinId);
#endif

// -----------------------------------------------------------------------------
//! \brief      This routine initializes the transport layer and opens the port
//!             of the device. Note that based on project defines, either the
//!             UART, or SPI driver can be used.
//!
//! \param[in]  sdiCBTx - Call back function for TX complete event
//! \param[in]  sdiCBRx - Call back function for RX event
//! \param[in]  sdiCBMrdy - Call back function for MRDY event
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITL_initTL(sdiRtosCB_t sdiCBTx, sdiRtosCB_t sdiCBRx, sdiRtosCB_t sdiCBMrdy)
{
    ICall_CSState key;
    key = ICall_enterCriticalSection();
    
    taskTxCB = sdiCBTx;
    taskRxCB = sdiCBRx;
#ifdef POWER_SAVING
    taskMrdyCB = sdiCBMrdy;
#endif

    transportInit(sdiRxBuf,sdiTxBuf, SDITL_transmissionCallBack);

#ifdef POWER_SAVING
    SRDY_DISABLE();

    // Initialize SRDY/MRDY. Enable int after callback registered
    hSdiHandshakePins = PIN_open(&sdiHandshakePins, sdiHandshakePinsCfg);
    PIN_registerIntCb(hSdiHandshakePins, SDITL_MRDYPinHwiFxn);
    PIN_setConfig(hSdiHandshakePins, PIN_BM_IRQ, MRDY_PIN | PIN_IRQ_BOTHEDGES);

    // Enable wakeup
    PIN_setConfig(hSdiHandshakePins, PINCC26XX_BM_WAKEUP, MRDY_PIN | PINCC26XX_WAKEUP_NEGEDGE);
    
    mrdy_state = PIN_getInputValue(MRDY_PIN);
#endif // ! POWER_SAVING

#ifndef POWER_SAVING
    // This call will start repeated Uart Reads when Power Savings is disabled
    transportRead();
#endif //!POWER_SAVING

    ICall_leaveCriticalSection(key);
    
    return;
}

// -----------------------------------------------------------------------------
//! \brief      This routine returns the state of transmission on SDI
//!
//! \return     bool - state of SDI transmission - 1 - active, 0 - not active
// -----------------------------------------------------------------------------
bool SDITL_checkSdiBusy(void)
{
#ifdef POWER_SAVING

    return !PIN_getOutputValue(SRDY_PIN);
#else
    return sdiTxActive;
#endif
}

#ifdef POWER_SAVING

// -----------------------------------------------------------------------------
//! \brief      This routine is used to set constraints on power manager
//!
//! \return     void
// -----------------------------------------------------------------------------
static void SDITL_setPM(void)
{
    if( sdiPMSetConstraint )
    {
        return;
    }
    // set constraints for Standby and idle mode
    Power_setConstraint(PowerCC26XX_SB_DISALLOW);
    Power_setConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
    sdiPMSetConstraint = TRUE;
}
#endif

#ifdef POWER_SAVING
// -----------------------------------------------------------------------------
//! \brief      This routine is used to release constraints on power manager
//!
//! \return     void
// -----------------------------------------------------------------------------
static void SDITL_relPM(void)
{

    if ( ! sdiPMSetConstraint )
    {
        return;
    }
    // release constraints for Standby and idle mode
    Power_releaseConstraint(PowerCC26XX_SB_DISALLOW);
    Power_releaseConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
    sdiPMSetConstraint = FALSE;
}
#endif

#ifdef POWER_SAVING
// -----------------------------------------------------------------------------
//! \brief      This routine is used to handle an MRDY transition from a task
//!             context. Certain operations such as UART_read() cannot be
//!             performed from a HWI context
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITL_handleMrdyEvent(void)
{
    ICall_CSState key;
    key = ICall_enterCriticalSection();
  
    // Check to make sure this event is not occuring during the next packet
    // transmission
    if ( PIN_getInputValue(MRDY_PIN) == 0 || 
        (sdiTxActive && mrdyPktStamp == txPktCount ) )
    {
        transportMrdyEvent();
        SRDY_ENABLE();
    }
    
    ICall_leaveCriticalSection(key);
}

// -----------------------------------------------------------------------------
//! \brief      This is a HWI function handler for the MRDY pin. Some MRDY
//!             functionality can execute from this HWI context. Others
//!             must be executed from task context hence the taskMrdyCB()
//!
//! \param[in]  hPin - PIN Handle
//! \param[in]  pinId - ID of pin that triggered HWI
//!
//! \return     void
// -----------------------------------------------------------------------------
static void SDITL_MRDYPinHwiFxn(PIN_Handle hPin, PIN_Id pinId)
{
    // The pin driver does not currently support returning whether the int
    // was neg or pos edge so we must use a variable to keep track of state. 
    // If the physical state of the pin was used then a very quick toggle of
    // of MRDY could be missed.
    mrdy_state ^= 1;
    
    if(mrdy_state == 0)
    {
        mrdyPktStamp = txPktCount;
        SDITL_setPM();
        if ( taskMrdyCB )
        {

            taskMrdyCB();
        }
    }
    else
    {
        transportStopTransfer();
    }
}
#endif

// -----------------------------------------------------------------------------
//! \brief      This callback is invoked on the completion of one transmission
//!             to/from the host MCU. Any bytes receives will be [0,Rxlen) in
//!             sdiRxBuf.
//!             If bytes were receives or transmitted, this function notifies
//!             the SDI task via registered call backs
//!
//! \param[in]  Rxlen   - lenth of the data received
//! \param[in]  Txlen   - length of the data transferred
//!
//! \return     void
// -----------------------------------------------------------------------------
static void SDITL_transmissionCallBack(uint16 Rxlen, uint16 Txlen)
{

    sdiRxBufHead = 0;
    sdiRxBufTail = Rxlen;

    if(Rxlen)
    {
        if ( taskRxCB )
        {
            taskRxCB(Rxlen);
        }
    }
    if(Txlen)
    {
        sdiTxActive = FALSE;
        // Only perform call back if SDI Task has been registered
        // and if there is not another fragment to send of this message
        if ( taskTxCB && !msgFragLen )
        {
            taskTxCB(Txlen);
        }
    }

#ifdef POWER_SAVING
    // Reset mrdy state in case of missed HWI
    mrdy_state = PIN_getInputValue(MRDY_PIN);
    
    SDITL_relPM();
    SRDY_DISABLE();
#endif
    
    // If there is another fragment to send, begin write without notifying 
    // higher level tasks
    if ( msgFragLen )
    {
        SDITL_writeTL(msgFrag,msgFragLen);
    }
}

// -----------------------------------------------------------------------------
//! \brief      This routine reads data from the transport layer based on len,
//!             and places it into the buffer.
//!
//! \param[in]  buf - Pointer to buffer to place read data.
//! \param[in]  len - Number of bytes to read.
//!
//! \return     uint16 - the number of bytes read from transport
// -----------------------------------------------------------------------------
uint16 SDITL_readTL(uint8 *buf, uint16 len)
{
    // Only copy the lowest number between len and bytes remaining in buffer
    len = (len > SDITL_getRxBufLen()) ? SDITL_getRxBufLen() : len;

    memcpy(buf, &sdiRxBuf[sdiRxBufHead], len);
    sdiRxBufHead += len;
    return len;
}

// -----------------------------------------------------------------------------
//! \brief      This routine writes data from the buffer to the transport layer.
//!
//! \param[in]  buf - Pointer to buffer to write data from.
//! \param[in]  len - Number of bytes to write.
//!
//! \return     uint16 - the number of bytes written to transport
// -----------------------------------------------------------------------------
uint16 SDITL_writeTL(uint8 *buf, uint16 len)
{
    ICall_CSState key;
    key = ICall_enterCriticalSection();
    
    // Writes are atomic at transport layer
    if ( SDITL_checkSdiBusy() )
    {
        ICall_leaveCriticalSection(key);
        return 0;
    }
    
    // If len of message is greater than fragment size
    // then message must be sent over the span of multiple 
    // fragments
    if ( len > SDI_MAX_FRAG_SIZE )
    {
      msgFrag = buf + SDI_MAX_FRAG_SIZE;
      msgFragLen = len - SDI_MAX_FRAG_SIZE;
      len = SDI_MAX_FRAG_SIZE;
    }
    else
    {
      msgFrag = NULL;
      msgFragLen = 0;
    }

    memcpy(sdiTxBuf, buf, len);
    sdiTxBufLen = len;
    sdiTxActive = TRUE;
    txPktCount++;

    len = transportWrite(sdiTxBufLen);

#ifdef POWER_SAVING
    SRDY_ENABLE();
#endif
    
    ICall_leaveCriticalSection(key);

    return len;
}

// -----------------------------------------------------------------------------
//! \brief      This routine returns the max size receive buffer.
//!
//! \return     uint16 - max size of the receive buffer
// -----------------------------------------------------------------------------
uint16 SDITL_getMaxRxBufSize(void)
{
    return(SDI_TL_BUF_SIZE);
}

// -----------------------------------------------------------------------------
//! \brief      This routine returns the max size transmit buffer.
//!
//! \return     uint16 - max size of the transmit buffer
// -----------------------------------------------------------------------------
uint16 SDITL_getMaxTxBufSize(void)
{
    return(SDI_TL_BUF_SIZE);
}

// -----------------------------------------------------------------------------
//! \brief      Returns number of bytes that are unread in RxBuf
//!
//! \return     uint16 - number of unread bytes
// -----------------------------------------------------------------------------
uint16 SDITL_getRxBufLen(void)
{
    return ((sdiRxBufTail - sdiRxBufHead) + SDI_TL_BUF_SIZE) % SDI_TL_BUF_SIZE;
}
