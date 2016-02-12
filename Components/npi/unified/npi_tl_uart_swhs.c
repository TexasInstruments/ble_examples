/*
 * npi_tl_uart_swhs.c
 *
 * NPI Transport Layer Module for UART with Software Handshake
 *
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
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "Board.h"
#include "hal_types.h"
#include "hal_defs.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

#include "inc/npi_util.h"
#include "inc/npi_data_swhs.h"
#include "inc/npi_tl_uart_swhs.h"
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

// ****************************************************************************
// defines
// ****************************************************************************

//! \brief NPI UART Message Indexes and Constants
//
#define NPI_UART_MSG_NON_PAYLOAD_LEN                 0x05
#define NPI_UART_MSG_SOF_LEN                         0x01
#define NPI_UART_MSG_HDR_LEN                         0x04
#define NPI_UART_MSG_SOF                             0xFE
#define NPI_UART_MSG_SOF_IDX                         0x00

//! \brief NPI UART SW Handshaking contents and sizes
//! \brief Note that all HS transactions are 1 byte
#define NPI_UART_HS_LEN                              0x01
#define NPI_UART_HS_RST                              0xFA
#define NPI_UART_HS_CHIRP                            0x55
#define NPI_UART_HS_JUNK                             0xAA

//! \brief NPI UART SW state flags in handshakingState var
#define NPI_UART_HS_CHIRP_QUEUED                     0x80
#define NPI_UART_HS_RST_QUEUED                       0x40
#define NPI_UART_HS_RST_SENT                         0x20
#define NPI_UART_HS_CHIRP_SENT                       0x10
#define NPI_UART_HS_COMPLETE                         0x08
#define NPI_UART_HS_INITIATOR                        0x04
#define NPI_UART_HS_WAIT                             0x02
#define NPI_UART_HS_RST_STATE                        0x01

//! \brief NPI UART Read States
typedef enum
{
  NPITLUART_READ_SOF = 0x00,
  NPITLUART_READ_RST,
  NPITLUART_READ_CHIRP,
  NPITLUART_READ_HDR,
  NPITLUART_READ_PLD,
  NPITLUART_IGNORE
} npiTLUart_readState;


// ****************************************************************************
// typedefs
// ****************************************************************************

//*****************************************************************************
// globals
//*****************************************************************************
//! \brief UART Handle for UART Driver
static UART_Handle uartHandle;

//! \brief NPI TL call back function for the end of a UART transaction
static npiTransmissionCompleteCB_t npiTransmitCB = NULL;

//! \brief NPI TL call back function for complete handshake
static npiHandshakeCompleteCB_t npiHandshakeCB = NULL;

//! \brief Flag sigaling software handshake status, default to RST
static uint8_t handshakingState = NPI_UART_HS_RST_STATE;


//! \brief Length of bytes received
static uint16_t TransportRxLen = 0;

//! \brief Length of bytes to send from NPI TL Tx Buffer
static uint16_t TransportTxLen = 0;

//! \brief Handshaking TX/RX data bytes
static uint8_t *hsTxByte;
static uint8_t *hsRxByte;


//! \brief NPI Transport Layer Buffer variables defined in npi_tl.c
extern uint8_t *npiRxBuf;
extern uint8_t *npiTxBuf;
extern uint16_t npiBufSize;

//*****************************************************************************
// function prototypes
//*****************************************************************************

//! \brief UART Callback invoked after UART write completion
static void NPITLUART_writeCallBack(UART_Handle handle, void *ptr, size_t size);

//! \brief UART Callback invoked after readsize has been read or timeout
static void NPITLUART_readCallBack(UART_Handle handle, void *ptr, size_t size);

//! \brief Check for whether a complete and valid packet has been received
static uint8_t NPITLUART_validPacketFound();

//! \brief Calculate FCS over the given length of buf
static uint8_t NPITLUART_calcFCS(uint8_t *buf, uint16_t len);

// -----------------------------------------------------------------------------
//! \brief      This routine initializes the transport layer and opens the port
//!             of the device.
//!
//! \param[in]  portID      ID value for board specific UART port
//! \param[in]  portParams  Parameters used to initialize UART port
//! \param[in]  npiTransmissionCompleteCB       TL Transmission complete CB
//! \param[in]  npiHandshakeCompleteCB          TL handshake complete CB
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLUART_initTransport(UART_Params *portParams,
                             npiTransmissionCompleteCB_t npiCBack,
                             npiHandshakeCompleteCB_t npiHSCback)
{
  //Initialize TL Layer callabacks
  npiTransmitCB = npiCBack;
  npiHandshakeCB = npiHSCback;
  // Add call backs UART parameters.
  portParams->readCallback = NPITLUART_readCallBack;
  portParams->writeCallback = NPITLUART_writeCallBack;
  
  hsTxByte = NPIUTIL_MALLOC(NPI_UART_HS_LEN);
  memset(hsTxByte, NPI_UART_HS_JUNK, NPI_UART_HS_LEN);
  hsRxByte = NPIUTIL_MALLOC(NPI_UART_HS_LEN);
  memset(hsRxByte, NPI_UART_HS_JUNK, NPI_UART_HS_LEN);
  //Don't open the UART yet, this will be handled at a per message basis
  //in the open transport function
}
// -----------------------------------------------------------------------------
//! \brief      This routine opens the port
//!             of the device.
//!
//! \param[in]  portID      ID value for board specific UART port
//! \param[in]  portParams  Parameters used to initialize UART port
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLUART_openTransport(uint8_t portID, UART_Params *portParams,
                             hsTransactionRole role)
{
  hsRxByte[0] = NPI_UART_HS_JUNK;
  hsTxByte[0] = NPI_UART_HS_JUNK;
  // Open / power on the UART.
  uartHandle = UART_open(portID, portParams);
  //Remove any mystery bytes from the FIFO...i.e. 0xF8
  volatile uint16_t count = 0;
  while( count < 32000)
  {
    UARTCharGetNonBlocking(((UARTCC26XX_HWAttrs const *)(uartHandle->hwAttrs))->baseAddr);
    count++;
  }
  NPITLUART_doHandshake(uartHandle,role);
}
// -----------------------------------------------------------------------------
//! \brief      This routine re-sends a chirp if the remote proc 
//!             missed the previously sent one
//!
//! \param[in]  role the hsTransactionRole
//!
//! \return     None
// -----------------------------------------------------------------------------
void NPITLUART_resendChirp(hsTransactionRole role)
{
  _npiCSKey_t key;
  key = NPIUtil_EnterCS();
  hsTxByte[0] = NPI_UART_HS_CHIRP;
  handshakingState |= (NPI_UART_HS_CHIRP_QUEUED | NPI_UART_HS_INITIATOR
                    | NPI_UART_HS_WAIT);
  UART_write(uartHandle, hsTxByte, NPI_UART_HS_LEN);
  NPIUtil_ExitCS(key);
}

// -----------------------------------------------------------------------------
//! \brief      This routine closes Transport Layer port           
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLUART_closeTransport(void)
{
  UART_readCancel(uartHandle);
  UART_close(uartHandle);
}

#ifdef POWER_SAVING
// -----------------------------------------------------------------------------
//! \brief      This routine stops any pending reads
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLUART_stopTransfer(void)
{
  // This is a dummy function for the UART Master implementation
  // The transfer will end once the master has finished sending and has 
  // received a valid packet from Slave or no data at all
  //TODO: SML-remove this function?
}
#endif //POWER_SAVING
// -----------------------------------------------------------------------------
//! \brief      This routine kicks off the handshaking process in a given role
//!
//! \param[in]  hanlde - handle to UART port recently opened
//! \param[in]  role - the handshaking role, initiator or responder
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLUART_doHandshake(UART_Handle handle, hsTransactionRole role)
{
  _npiCSKey_t key;
  key = NPIUtil_EnterCS();
  //If we are in the reset state, we need to perform reset handshake
  //Else we should perform normal HS
  if(handshakingState & NPI_UART_HS_RST_STATE)
  {
    //Regardless of role, we want to write a chirp and read remote
    //Reset initiator is determined by who sends RST byte first
    //Indicate we have queued a chirp, and are waiting for
    // a chirp in response
    handshakingState |= (NPI_UART_HS_CHIRP_QUEUED | NPI_UART_HS_WAIT);
    
    UART_read(handle, hsRxByte, NPI_UART_HS_LEN);
    //Write to chirp back to the Remote proc to indicate we're awake
    hsTxByte[0] = NPI_UART_HS_CHIRP;
    UART_write(handle, hsTxByte, NPI_UART_HS_LEN);
  }
  else
  {
    //Set HS byte to send to be a chirp
    hsTxByte[0] = NPI_UART_HS_CHIRP;
    
    if(HS_INITIATOR == role)
    {
      //Indicate we are the initiator, have queued a chirp, and are waiting for
      // a chirp in response

      //Kick off remote chirp read first to ensure we don't miss it
      UART_read(handle, hsRxByte, NPI_UART_HS_LEN);
      //Write to chirp back to the Remote proc to indicate we're awake
      handshakingState = (NPI_UART_HS_CHIRP_QUEUED | NPI_UART_HS_INITIATOR
                    | NPI_UART_HS_WAIT);
      UART_write(handle, hsTxByte, NPI_UART_HS_LEN);

    }
    else
    {
      //Indicate we are the not the initiator, and are not waiting on any resp
      //from remote, once write is confirmed, we are clear to read NPI msg
      handshakingState = (NPI_UART_HS_CHIRP_QUEUED);
      //We have already rx'd a chirp to wake us up, we just need to respond
      UART_write(handle, hsTxByte, NPI_UART_HS_LEN);
      //kickoff NPI frame read
      NPITLUART_readTransport();
    }
  }
  NPIUtil_ExitCS(key);
}
// -----------------------------------------------------------------------------
//! \brief      This callback is invoked on Write completion
//!
//! \param[in]  handle - handle to the UART port
//! \param[in]  ptr    - pointer to data to be transmitted
//! \param[in]  size   - size of the data
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLUART_writeCallBack(UART_Handle handle, void *ptr, size_t size)
{
  _npiCSKey_t key;
  key = NPIUtil_EnterCS();
  uint16_t localRxLen = TransportRxLen;
  //We should only process NPI packets when HS is complete
  if(!(handshakingState & NPI_UART_HS_CHIRP_QUEUED) &&
     !(handshakingState & NPI_UART_HS_RST_QUEUED))
  { 
    //If we haven't read anything yet, we aren't going to
    //cancel the read so we can release PM and close the UART
    if(TransportRxLen == 0)
    {
      UART_readCancel(uartHandle);
    }
    else
    {
      //else check to see if we have rx'd a valid packet
      //if we have, we are safe to sleep, if not need to stay awake
      if (NPITLUART_validPacketFound() == NPI_SUCCESS)
      {
        // Decrement as to not include trailing FCS byte
        TransportRxLen--;
      }
      else
      {
        //Since the largest valid NPI packet size is 4096
        //valid RxLen fields should only be up to 0x0FFF
        //by setting a bogus RxLen, we let the TL know that 
        //a Rx is in process w/o an extra parameter
        localRxLen |= 0x1000;
      }
    }
    if (npiTransmitCB) 
    {   
      npiTransmitCB(localRxLen,TransportTxLen);
    }
  }
  //Else we have just sent a CHIRP or RST
  else
  {
    //If we are not in reset and just sent a chirp
    if(!(handshakingState & NPI_UART_HS_RST_STATE) && 
       (handshakingState & NPI_UART_HS_CHIRP_QUEUED))
    {
      //If we aren't the initiator it is safe to trigger the HS_complete
      //NPI should be reading packet by now
      if(!(handshakingState & NPI_UART_HS_INITIATOR))
      {
        handshakingState = NPI_UART_HS_COMPLETE;
        handshakingState &= ~NPI_UART_HS_CHIRP_QUEUED;
        handshakingState |= NPI_UART_HS_CHIRP_SENT;
      }
      else
      {
        //Chirp is now confirmed to have been sent
        handshakingState &= ~NPI_UART_HS_CHIRP_QUEUED;
        handshakingState |= NPI_UART_HS_CHIRP_SENT;
      }
    }
    else //we are in reset mode
    {
      
      if(handshakingState & NPI_UART_HS_CHIRP_QUEUED)
      {
        handshakingState &= ~NPI_UART_HS_CHIRP_QUEUED;
        handshakingState |= NPI_UART_HS_CHIRP_SENT;
      }
      if(handshakingState & NPI_UART_HS_RST_QUEUED)
      {
        handshakingState &= ~NPI_UART_HS_RST_QUEUED;
        handshakingState |= NPI_UART_HS_RST_SENT;
      }
    }
  }

  NPIUtil_ExitCS(key);
}

// -----------------------------------------------------------------------------
//! \brief      This callback is invoked on Read completion of readSize/receive
//!             timeout
//!
//! \param[in]  handle - handle to the UART port
//! \param[in]  ptr    - pointer to buffer to read data into
//! \param[in]  size   - size of the data
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLUART_readCallBack(UART_Handle handle, void *ptr, size_t size)
{
  static npiTLUart_readState readState = NPITLUART_READ_SOF;
  static uint16_t payloadLen = 0;
  _npiCSKey_t key;
  key = NPIUtil_EnterCS();
  
  //Set local readState variable according to HS state
  //If we have rx'd a reset, that takes precedence
  if((handshakingState & NPI_UART_HS_RST_STATE) || (npiRxBuf[0] == NPI_UART_HS_RST))
  {
    readState = NPITLUART_READ_RST;
    //The below case is for when we thought we completed the HS, but rx'd a RST
    //we need to force RST conditions
    if(npiRxBuf[0] == NPI_UART_HS_RST)
    {
      handshakingState = NPI_UART_HS_RST_STATE;
      hsRxByte[0] = NPI_UART_HS_RST;
      npiRxBuf[0] = NPI_UART_HS_JUNK;
    }
  }
  else if(!(handshakingState & NPI_UART_HS_COMPLETE))
  {
    readState = NPITLUART_READ_CHIRP;
  }
  //Case statement to crawl over the various parts of the HS logic and NPI frame
  switch (readState)
  {
    case NPITLUART_READ_RST:
      //Check to see that the data we rx'd is a valid RST byte
      if (size == NPI_UART_HS_LEN && ((hsRxByte[0] == NPI_UART_HS_RST) 
                                      | (hsRxByte[0] == NPI_UART_HS_CHIRP)))
      {
          if(hsRxByte[0] == NPI_UART_HS_CHIRP)
          {
            //We have now written and rx'd a chirp, lets send a reset
            hsTxByte[0] = NPI_UART_HS_RST;
            handshakingState |= (NPI_UART_HS_RST_QUEUED | NPI_UART_HS_WAIT);
            UART_read(handle, hsRxByte, NPI_UART_HS_LEN);
            UART_write(handle, hsTxByte, NPI_UART_HS_LEN);
          }
          else //we've read a RST
          {
            hsTxByte[0] = NPI_UART_HS_RST;
            //If we read a reset and have already sent one, we are initiator
            if(handshakingState & NPI_UART_HS_RST_SENT)
            {
              hsTxByte[0] = NPI_UART_HS_CHIRP;
              //We have completed RS_HS
              handshakingState &= ~NPI_UART_HS_RST_STATE;
              handshakingState &= ~NPI_UART_HS_CHIRP_SENT;
              //It is safe to clear out any pending reads at this point
              UART_readCancel(uartHandle);
              //Start regular chirp HS
              //Indicate we are the initiator, have queued a chirp, and are waiting for
              // a chirp in response
              handshakingState |= (NPI_UART_HS_CHIRP_QUEUED | NPI_UART_HS_INITIATOR
                                  | NPI_UART_HS_WAIT);
              //Kick off remote chirp read first to ensure we don't miss it
              UART_read(handle, hsRxByte, NPI_UART_HS_LEN);
              //Write to chirp back to the Remote proc to indicate we're awake
              UART_write(handle, hsTxByte, NPI_UART_HS_LEN);
            }
            else
            {
              hsTxByte[0] = NPI_UART_HS_RST;
              //Indicate we have queued a RST for sending
              handshakingState |= (NPI_UART_HS_RST_QUEUED);
              //We have completed RS_HS
              handshakingState &= ~NPI_UART_HS_RST_STATE;
              handshakingState &= ~NPI_UART_HS_CHIRP_SENT;
              //It is safe to clear out any pending reads at this point
              UART_readCancel(uartHandle);
              //We're not initiator since other proc beat us to sending RST
              handshakingState &= ~NPI_UART_HS_INITIATOR;
              //Kick off remote chirp read first to ensure we don't miss it
              UART_read(handle, hsRxByte, NPI_UART_HS_LEN);
              //We have already rx'd a RST , we just need to respond
              UART_write(handle, hsTxByte, NPI_UART_HS_LEN);
            }
              
          }
      }
      else
      {
        //If we haven't read anything the remote proc may still be coming up
        //we need to stay awake until we get something.
        UART_read(handle, hsRxByte, NPI_UART_HS_LEN);
      }
      break;
    case NPITLUART_READ_CHIRP:
      //Check to see that we have rx'd a valid chirp
      if ((size == NPI_UART_HS_LEN) && (hsRxByte[0] == NPI_UART_HS_CHIRP))
      {
        //If we are the initiator, and have sent a chirp, then HS is complete
        //if we are responder, have already sent a chirp and rx'd a chirp
        //we need to reset the HS logic because remote is out of sync
        if((handshakingState & NPI_UART_HS_INITIATOR) && 
           (handshakingState & NPI_UART_HS_CHIRP_SENT))
        {
          //Set appropriate flags and kick off callback
          handshakingState = NPI_UART_HS_COMPLETE;
          readState = NPITLUART_READ_SOF;
          if (npiHandshakeCB) 
          {
            npiHandshakeCB(HS_INITIATOR);
          }
        }
        else if(!(handshakingState & NPI_UART_HS_INITIATOR) && 
           !(handshakingState & NPI_UART_HS_CHIRP_SENT))
        {
          //We have not sent a chirp, and did not initiate,but have rx'd one
          //once we respond, then the HS will be complete
          //Indicate we are the not the initiator, and are not waiting on any resp
          //from remote, once write is confirmed, we are clear to read NPI msg
          handshakingState = (NPI_UART_HS_CHIRP_QUEUED);
          hsTxByte[0] = NPI_UART_HS_CHIRP;
          //We have already rx'd a chirp to wake us up, we just need to respond
          UART_write(handle, hsTxByte, NPI_UART_HS_LEN);
          //kickoff NPI frame read
          handshakingState |= NPI_UART_HS_COMPLETE;
          readState = NPITLUART_READ_SOF;
          NPITLUART_readTransport();
        }
        else if(!(handshakingState & NPI_UART_HS_INITIATOR) && 
           (handshakingState & NPI_UART_HS_CHIRP_SENT))
        {
          //We have seemingly already completed the HS, but we received another
          //chirp, something is wrong
          handshakingState = NPI_UART_HS_RST_STATE;
          //Regardless of role, we want to write a chirp and read remote
          //Reset initiator is determined by who sends RST byte first
          //Indicate we have queued a chirp, and are waiting for
          // a chirp in response
          hsTxByte[0] = NPI_UART_HS_RST;
          handshakingState = (NPI_UART_HS_CHIRP_QUEUED | NPI_UART_HS_WAIT);
          UART_read(handle, hsRxByte, NPI_UART_HS_LEN);
          //Write to chirp back to the Remote proc to indicate we're awake
          UART_write(handle, hsTxByte, NPI_UART_HS_LEN);
        }
        
      }
      //If we somehow missed a chirp and received a SOF, we must keep reading
      //this is a bi-directional transfer beginning
      else if(size == NPI_UART_MSG_SOF_LEN && hsRxByte[0] == NPI_UART_MSG_SOF)
      {
        //Note that we can mark the HS as complete in this state, but do not
        //need CB
        handshakingState |= NPI_UART_HS_COMPLETE;
        TransportRxLen = 0;
        // Recevied SOF, Read HDR next. Do not save SOF byte
        UART_read(uartHandle, npiRxBuf, NPI_UART_MSG_HDR_LEN);
        readState = NPITLUART_READ_HDR;
      }
      break;
    case NPITLUART_READ_SOF:
      // Should only have read one byte in this state. 
      if (size == NPI_UART_MSG_SOF_LEN && npiRxBuf[0] == NPI_UART_MSG_SOF)
      {
        // Recevied SOF, Read HDR next. Do not save SOF byte
        UART_read(uartHandle, npiRxBuf, NPI_UART_MSG_HDR_LEN);
        readState = NPITLUART_READ_HDR;
      }
      break;
    
    case NPITLUART_READ_HDR:
      if (size == NPI_UART_MSG_HDR_LEN)
      {
        // Header has been read. Increment RxLen
        TransportRxLen += size;
        
        // Determine length of remainder of packet, add an extra byte for FCS
        payloadLen = BUILD_UINT16(npiRxBuf[0],npiRxBuf[1]) + 1;
        
        // Check to see if payload can fit in the remainder of the RxBuf
        if (payloadLen <= npiBufSize - TransportRxLen)
        {
          // Read remainder of packet
          UART_read(uartHandle, &npiRxBuf[TransportRxLen], payloadLen);
          readState = NPITLUART_READ_PLD;
        }
        else
        {
          // Read remainder of packet bytes but ignore them
          UART_read(uartHandle, npiRxBuf, npiBufSize);
          readState = NPITLUART_IGNORE;
        }
      }
      else
      {
        // Error has occured. Reset read state
        readState = NPITLUART_READ_SOF;
      }
    break;
    
    case NPITLUART_READ_PLD:
      if (payloadLen == size)
      {
        // All bytes are read
        TransportRxLen += size;
        
        // Check if FCS is valid
        if (NPITLUART_validPacketFound() == NPI_SUCCESS)
        {
          // Valid Packet. Decrement RxLen to not include FCS since it is
          // checked and valid
          TransportRxLen--;
          
          if (npiTransmitCB) 
          {
            npiTransmitCB(TransportRxLen,0);
          }
        }
      }

      // Reset State. Full packet has been read or error has occurred
      readState = NPITLUART_READ_SOF;
      break;
    
    case NPITLUART_IGNORE:
      if (payloadLen == size)
      {
        // All bytes of oversized payload have been read. Reset state
        readState = NPITLUART_READ_SOF;
      }
      else
      {
        // Bytes remaining to be read
        payloadLen -= size;
        
        if (payloadLen > npiBufSize)
        {
          UART_read(uartHandle, npiRxBuf, npiBufSize);
        }
        else
        {
          UART_read(uartHandle, npiRxBuf, payloadLen);
        }
      }
      break;
    
    default:
      // Should not get here. If so reset read state
      readState = NPITLUART_READ_SOF;
      break;
  }
  
  NPIUtil_ExitCS(key);
}

// -----------------------------------------------------------------------------
//! \brief      This routine reads data from the UART
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLUART_readTransport(void)
{
  _npiCSKey_t key;
  key = NPIUtil_EnterCS();
  

  TransportRxLen = 0;
  
  // UART driver will automatically reject this read if already in use
  UART_read(uartHandle, npiRxBuf, NPI_UART_MSG_SOF_LEN);
  
  NPIUtil_ExitCS(key);
}


// -----------------------------------------------------------------------------
//! \brief      This routine writes copies buffer addr to the transport layer.
//!
//! \param[in]  len - Number of bytes to write.
//!
//! \return     uint16_t - number of bytes written to transport
// -----------------------------------------------------------------------------
uint16_t NPITLUART_writeTransport(uint16_t len)
{
  _npiCSKey_t key;
  key = NPIUtil_EnterCS();
  //Build message
  npiTxBuf[NPI_UART_MSG_SOF_IDX] = NPI_UART_MSG_SOF;
  npiTxBuf[len + 1] = NPITLUART_calcFCS((uint8_t *)&npiTxBuf[1],len);
  TransportTxLen = len + 2;
  //read before writing to ensure we don't miss anything
  NPITLUART_readTransport();
  //write data
  if(UART_write(uartHandle, npiTxBuf, TransportTxLen) == UART_ERROR)
  {
    len = 0;
  }

  NPIUtil_ExitCS(key);
  
  return len;
}

// -----------------------------------------------------------------------------
//! \brief      Check for whether a complete and valid packet has been received.
//!             If packet incomplete do nothing. If packet is invalid, flush
//!             the Rx buffer
//!
//! \return     uint8_t - NPI_SUCCESS if Success, NPI_INCOMPLETE_PACKET if not
//!                       enough bytes recieved, NPI_INVALID_PACKET if incorrect 
//!                       format or FCS
// -----------------------------------------------------------------------------
uint8_t NPITLUART_validPacketFound(void)
{
  uint16_t payloadLen;
  uint8_t fcs;
  
  // SOF has already been removed from npiRxBuf
  payloadLen = (uint16) npiRxBuf[0];
  payloadLen += ((uint16) npiRxBuf[1]) << 8;
  
  // Check to make sure we have received all bytes of this message
  if (TransportRxLen < (payloadLen + NPI_UART_MSG_NON_PAYLOAD_LEN))
  {
    return NPI_INCOMPLETE_PKT;
  }
  
  // Calculate FCS of this message
  fcs = NPITLUART_calcFCS((uint8_t *)npiRxBuf, payloadLen + NPI_UART_MSG_HDR_LEN);

  if (fcs != npiRxBuf[payloadLen + NPI_UART_MSG_HDR_LEN])
  {
    // Invalid FCS, Flush RX buffer before returning error
    TransportRxLen = 0;
    
    return NPI_INVALID_PKT;
  }
  
  return NPI_SUCCESS;
}

// -----------------------------------------------------------------------------
//! \brief      Calculate FCS over the given length of buf
//!
//! \param[in]  buf - Pointer to first byte to use for FCS
//!             len - Number of bytes to calculate FCS over.
//!
//! \return     uint8_t - FCS value
// -----------------------------------------------------------------------------
uint8_t NPITLUART_calcFCS(uint8_t *buf, uint16_t len)
{   
    uint16_t i;
    uint8_t fcs = 0;
    
    for (i = 0; i < len; i++)
    {
        fcs ^= buf[i];
    }
    
    return fcs;
}
