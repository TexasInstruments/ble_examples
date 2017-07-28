/******************************************************************************

 * Filename: sdi_task.c

 SDI is a TI RTOS Application Thread that provides a common serial data interface
 framework

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

#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>

#include <string.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

#include "inc/sdi_task.h"
#include "inc/sdi_data.h"
#include "inc/sdi_rxbuf.h"
#include "inc/sdi_tl.h"

// ****************************************************************************
// defines
// ****************************************************************************

//! \brief Transport layer RX Event (ie. bytes received, RX ISR etc.)
#define SDITASK_TRANSPORT_RX_EVENT      Event_Id_00

//! \brief Transmit Complete Event (likely associated with TX ISR etc.)
#define SDITASK_TRANSPORT_TX_DONE_EVENT Event_Id_01

//! \brief A framed message buffer is ready to be sent to the transport layer.
#define SDITASK_TX_READY_EVENT          Event_Id_02

//! \brief MRDY Received Event
#define SDITASK_MRDY_EVENT              Event_Id_03

//! \brief Size of stack created for SDI RTOS task
#ifndef Display_DISABLE_ALL
#ifdef __TI_COMPILER_VERSION__
#define SDITASK_STACK_SIZE 752      /* in order to optimize memory, this value should be a multiple of 8 bytes */
#else // !__TI_COMPILER_VERSION__
#define SDITASK_STACK_SIZE 656      /* in order to optimize memory, this value should be a multiple of 8 bytes */
#endif // __TI_COMPILER_VERSION__
#else // Display_DISABLE_ALL
#ifdef __TI_COMPILER_VERSION__
#define SDITASK_STACK_SIZE 752      /* in order to optimize memory, this value should be a multiple of 8 bytes */
#else // !__TI_COMPILER_VERSION__
#define SDITASK_STACK_SIZE 608      /* in order to optimize memory, this value should be a multiple of 8 bytes */
#endif // __TI_COMPILER_VERSION__
#endif // Display_DISABLE_ALL

//! \brief Task priority for SDI RTOS task
#define SDITASK_PRIORITY 2

//! \brief Max bytes received from UART send to App
#define DEFAULT_APP_DATA_LENGTH 20

// ****************************************************************************
// typedefs
// ****************************************************************************

//! \brief Queue record structure
//!
typedef struct SDI_QueueRec_t
{
    Queue_Elem _elem;
    SDIMSG_msg_t *sdiMsg;
} SDI_QueueRec;


//*****************************************************************************
// globals
//*****************************************************************************
//! \brief RTOS task structure for SDI task
//!
static Task_Struct sdiTaskStruct;

//! \brief Allocated memory block for SDI task's stack
//!
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(sdiTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t sdiTaskStack[SDITASK_STACK_SIZE];

//! \brief Handle for the ASYNC TX Queue
//!
static Queue_Handle sdiTxQueue;

//! \brief Pointer to last tx message.  This is free'd once confirmation is
//!        is received that the buffer has been transmitted
//!        (ie. SDITASK_TRANSPORT_TX_DONE_EVENT)
//!
static uint8_t *lastQueuedTxMsg;

Event_Struct uartEvent;
Event_Handle hUartEvent; //!< Event used to control the UART thread

//! \brief Pointer to Application RX event callback function for optional
//!        rerouting of messages to application.
//!
static sdiIncomingEventCBack_t incomingRXEventAppCBFunc = NULL;

//! \brief Data buffer to send to application
//!
static uint8 buf[SDI_TL_BUF_SIZE] ={0x00,};
static uint16 length;
static uint8 lengthRead;

//! \brief Size of data to send to application
//!
static uint16 maxAppDataSize = DEFAULT_APP_DATA_LENGTH;

//*****************************************************************************
// function prototypes
//*****************************************************************************

//! \brief      SDI main event processing loop.
//!
static void SDITask_process(void);

//! \brief Callback function registered with Transport Layer
//!
static void SDITask_transportRXCallBack(int size);

//! \brief Callback function registered with Transport Layer
//!
static void SDITask_transportTxDoneCallBack(int size);

//! \brief Callback function registered with Transport Layer
//!
static void SDITask_MRDYEventCB(int size);

//! \brief ASYNC TX Q Processing function.
//!
static void SDITask_ProcessTXQ(void);

// -----------------------------------------------------------------------------
//! \brief      Initialization for the SDI Thread
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITask_setAppDataSize(uint16 mtuSize)
{
  maxAppDataSize = mtuSize - 3; //subtract GATT Notification overhead: 1 byte opcode, 2 bytes conn. handle
}

// -----------------------------------------------------------------------------
//! \brief      Initialization for the SDI Thread
//!
//! \return     void
// -----------------------------------------------------------------------------
static void SDITask_inititializeTask(void)
{
    lastQueuedTxMsg = NULL;

    // create a Tx Queue instance
    sdiTxQueue = Queue_create(NULL, NULL);

    Event_Params evParams;
    Event_Params_init(&evParams);

    Event_construct(&uartEvent, &evParams);
    hUartEvent = Event_handle(&uartEvent);

    // Initialize Network Processor Interface (SDI) and Transport Layer
    SDITL_initTL( &SDITask_transportTxDoneCallBack,
                  &SDITask_transportRXCallBack,
                  &SDITask_MRDYEventCB );

}

// -----------------------------------------------------------------------------
//! \brief      SDI main event processing loop.
//!
//! \return     void
// -----------------------------------------------------------------------------
static void SDITask_process(void)
{
  UInt postedEvents;

    /* Forever loop */
    for (;; )
    {
        /* Wait for response message */
        postedEvents = Event_pend(hUartEvent, Event_Id_NONE, SDITASK_MRDY_EVENT | SDITASK_TX_READY_EVENT | SDITASK_TRANSPORT_RX_EVENT | SDITASK_TRANSPORT_TX_DONE_EVENT, BIOS_WAIT_FOREVER);

        {
            // Capture the ISR events flags now within this task loop.

            // MRDY event
            if (postedEvents & SDITASK_MRDY_EVENT)
            {
#ifdef POWER_SAVING
                SDITL_handleMrdyEvent();
#endif //POWER_SAVING

            }

            // An ASYNC message is ready to send to the Host
            if(postedEvents & SDITASK_TX_READY_EVENT)
            {

                if ((!Queue_empty(sdiTxQueue)) && !SDITL_checkSdiBusy())
                {
                    SDITask_ProcessTXQ();
                }

                if (Queue_empty(sdiTxQueue))
                {
                    // Q is empty, no action.

                }
                else
                {
                    // Q is not empty, there's more to handle so preserve the
                    // flag and repost to the task event.
                    Event_post(hUartEvent, SDITASK_TX_READY_EVENT);
                }
            }

            // The Transport Layer has received some bytes
            if(postedEvents & SDITASK_TRANSPORT_RX_EVENT)
            {
                length = SDIRxBuf_GetRxBufCount();

                if(length > maxAppDataSize)
                {
                  lengthRead = (maxAppDataSize & 0xFF);
                }else
                {
                  lengthRead = (length & 0xFF);
                }

                //Do custom app processing
                SDIRxBuf_ReadFromRxBuf(buf, lengthRead);

                //Echo back via UART
                //SDITask_sendToUART(buf, length);

                if (incomingRXEventAppCBFunc != NULL)
                {
                  incomingRXEventAppCBFunc( UART_DATA_EVT , buf, lengthRead);
                }

                if(length > maxAppDataSize)
                {
                    // Additional bytes to collect, preserve the flag and repost
                    // to the event
                    Event_post(hUartEvent, SDITASK_TRANSPORT_RX_EVENT);
                }
            }

            // The last transmission to the host has completed.
            if(postedEvents & SDITASK_TRANSPORT_TX_DONE_EVENT)
            {
                // Current TX is done.

                    if (!Queue_empty(sdiTxQueue))
                    {
                        // There are pending ASYNC messages waiting to be sent
                        // to the host.  Post to event.

                        Event_post(hUartEvent, SDITASK_TX_READY_EVENT);
                    }
            }
        }
    }
}

// -----------------------------------------------------------------------------
//! \brief      SDI Task function called from within SDITask_Fxn
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITask_task(void)
{
    // Initialize application
    SDITask_inititializeTask();

    // No return from TestProfile2 process
    SDITask_process();
}


// -----------------------------------------------------------------------------
// Exported Functions


// -----------------------------------------------------------------------------
//! \brief      SDI task entry point.
//!
//! \return     void
// -----------------------------------------------------------------------------
Void SDITask_Fxn(UArg a0, UArg a1)
{
    SDITask_task();
}

// -----------------------------------------------------------------------------
//! \brief      Task creation function for SDI
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITask_createTask(void)
{
    memset(&sdiTaskStack, 0xDD, sizeof(sdiTaskStack));

    // Configure and create the SDI task.
    Task_Params sdiTaskParams;
    Task_Params_init(&sdiTaskParams);
    sdiTaskParams.stack = sdiTaskStack;
    sdiTaskParams.stackSize = SDITASK_STACK_SIZE;
    sdiTaskParams.priority = SDITASK_PRIORITY;

    Task_construct(&sdiTaskStruct, SDITask_Fxn, &sdiTaskParams, NULL);
}

// -----------------------------------------------------------------------------
//! \brief      Register callback function to reroute incoming (from UART)
//!             SDI messages.
//!
//! \param[in]  appRxCB   Callback fucntion.
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITask_registerIncomingRXEventAppCB(sdiIncomingEventCBack_t appRxCB)
{
    incomingRXEventAppCBFunc = appRxCB;
}

// -----------------------------------------------------------------------------
//! \brief      API for application task to send a message to the Host.
//!             NOTE: It's assumed all message traffic to the stack will use
//!             other (ICALL) APIs/Interfaces.
//!
//! \param[in]  pMsg    Pointer to "unframed" message buffer.
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITask_sendToUART(uint8_t *pMsg, uint16 length)
{
    ICall_CSState key;
    SDI_QueueRec *recPtr;

    SDIMSG_msg_t *pSDIMsg =(SDIMSG_msg_t *)ICall_malloc( sizeof(SDIMSG_msg_t));

    key = ICall_enterCriticalSection();

    if(pSDIMsg)
    {
      pSDIMsg->msgType = SDIMSG_Type_ASYNC;
      pSDIMsg->pBuf = (uint8 *)ICall_allocMsg(length);
      pSDIMsg->pBufSize = length;

      if(pSDIMsg->pBuf)
      {
          // Payload
          memcpy(pSDIMsg->pBuf, pMsg, length);
      }

      recPtr = ICall_malloc(sizeof(SDI_QueueRec));

      recPtr->sdiMsg = pSDIMsg;
    }

    switch (pSDIMsg->msgType)
    {
        case SDIMSG_Type_ASYNC:
        {
            Queue_enqueue(sdiTxQueue, &recPtr->_elem);
            Event_post(hUartEvent, SDITASK_TX_READY_EVENT);
            break;
        }
        default:
        {
            //error
            break;
        }
    }
    ICall_leaveCriticalSection(key);
}


// -----------------------------------------------------------------------------
//! \brief      Dequeue next message in the ASYNC TX Queue and send to serial
//!             interface.
//!
//! \return     void
// -----------------------------------------------------------------------------
static void SDITask_ProcessTXQ(void)
{
    ICall_CSState key;
    SDI_QueueRec *recPtr = NULL;

    // Processing of any TX Queue should only be done
    // in a critical section since any application
    // task can enqueue items freely
    key = ICall_enterCriticalSection();

    if (!Queue_empty(sdiTxQueue))
    {
    recPtr = Queue_dequeue(sdiTxQueue);

    if (recPtr != NULL)
    {
		lastQueuedTxMsg = recPtr->sdiMsg->pBuf;

		SDITL_writeTL(recPtr->sdiMsg->pBuf, recPtr->sdiMsg->pBufSize);

		//free the Queue record
		ICall_free(recPtr->sdiMsg);
		ICall_free(recPtr);
      }
    }

    ICall_leaveCriticalSection(key);
}

// -----------------------------------------------------------------------------
// Call Back Functions

// -----------------------------------------------------------------------------
//! \brief      Call back function for TX Done event from transport layer.
//!
//! \param[in]  size    Number of bytes transmitted.
//!
//! \return     void
// -----------------------------------------------------------------------------
static void SDITask_transportTxDoneCallBack(int size)
{

    if(lastQueuedTxMsg)
    {
        //Deallocate most recent message being transmitted.
        ICall_freeMsg(lastQueuedTxMsg);

        lastQueuedTxMsg = NULL;
    }

    // Post the event to the SDI task thread.
    Event_post(hUartEvent, SDITASK_TRANSPORT_TX_DONE_EVENT);
}

// -----------------------------------------------------------------------------
//! \brief      RX Callback provided to Transport Layer for RX Event (ie.Bytes
//!             received).
//!
//! \param[in]  size    Number of bytes received.
//!
//! \return     void
// -----------------------------------------------------------------------------
static void SDITask_transportRXCallBack(int size)
{
    if ( size < SDIRxBuf_GetRxBufAvail() )
    {
    	SDIRxBuf_Read(size);
    }
    else
    {
        // Trap here for pending buffer overflow. If SDI_FLOW_CTRL is
        // enabled, increase size of RxBuf to handle larger frames from host.
        for(;;);
    }
    Event_post(hUartEvent, SDITASK_TRANSPORT_RX_EVENT);
}

// -----------------------------------------------------------------------------
//! \brief      RX Callback provided to Transport Layer for MRDY Event
//!
//! \param[in]  size    N/A
//!
//! \return     void
// -----------------------------------------------------------------------------
static void SDITask_MRDYEventCB(int size)
{
    Event_post(hUartEvent, SDITASK_MRDY_EVENT);
}

