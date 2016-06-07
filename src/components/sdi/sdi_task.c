//******************************************************************************
//! \file           sdi_task.c
//! \brief          SDI is a TI RTOS Application Thread that provides a
//! \brief          common Serial Data Interface framework.
//
//   Revised        $Date: 2015-02-15 11:57:15 -0800 (Sun, 15 Feb 2015) $
//   Revision:      $Revision: 42612 $
//
//  Copyright 2015 Texas Instruments Incorporated. All rights reserved.
//
// IMPORTANT: Your use of this Software is limited to those specific rights
// granted under the terms of a software license agreement between the user
// who downloaded the software, his/her employer (which must be your employer)
// and Texas Instruments Incorporated (the "License").  You may not use this
// Software unless you agree to abide by the terms of the License. The License
// limits your use, and you acknowledge, that the Software may not be modified,
// copied or distributed unless used solely and exclusively in conjunction with
// a Texas Instruments radio frequency device, which is integrated into
// your product.  Other than for the foregoing purpose, you may not use,
// reproduce, copy, prepare derivative works of, modify, distribute, perform,
// display or sell this Software and/or its documentation for any purpose.
//
//  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
//  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,l
//  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
//  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
//  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
//  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
//  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
//  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
//  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
//  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
//  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
//
//  Should you have any questions regarding your right to use this Software,
//  contact Texas Instruments Incorporated at www.TI.com.
//******************************************************************************

// ****************************************************************************
// includes
// ****************************************************************************

#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
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
#define SDITASK_STACK_SIZE 512

//! \brief Task priority for SDI RTOS task
#define SDITASK_PRIORITY 2


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

uint8 buf[SDI_TL_BUF_SIZE] ={0x00,};
uint16 length;
uint8 lengthRead;

//! \brief ICall ID for stack which will be sending SDI messages
//!
//static uint32_t stackServiceID = 0x0000;

//! \brief RTOS task structure for SDI task
//!
static Task_Struct sdiTaskStruct;

//! \brief Allocated memory block for SDI task's stack
//!
Char sdiTaskStack[SDITASK_STACK_SIZE];

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

//! \brief SDI ICall Application Entity ID.
//!
ICall_EntityID sdiAppEntityID = 0;

//! \brief Pointer to Application RX event callback function for optional
//!        rerouting of messages to application.
//!
static sdiIncomingEventCBack_t incomingRXEventAppCBFunc = NULL;

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
    while (1)
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
         
                length = SDIRxBuf_GetRxBufLen();
                
                if(length > 20)
                {
                  lengthRead = 20;
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
                
                if(length > 20)
                {
                    // Additional bytes to collect, preserve the flag and repost
                    // to the semaphore
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
    SDI_QueueRec *recPtr;

    SDIMSG_msg_t *pSDIMsg =(SDIMSG_msg_t *)ICall_malloc( sizeof(SDIMSG_msg_t));
    
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
}


// -----------------------------------------------------------------------------
//! \brief      Dequeue next message in the ASYNC TX Queue and send to serial
//!             interface.
//!
//! \return     void
// -----------------------------------------------------------------------------
static void SDITask_ProcessTXQ(void)
{

    SDI_QueueRec *recPtr = NULL;

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
    SDIRxBuf_Read(size);
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

