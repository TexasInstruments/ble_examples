/*
 * Filename: uart_logs.c
 *
 * Description: This is the simple_central example modified to receive
 * data over BLE at a high throughput.
 *
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

/*********************************************************************
 * INCLUDES
 */
#include <ti/drivers/UART.h>
#include <ti/sysbios/hal/Hwi.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Text.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Timestamp.h>

#include <driverlib/aon_rtc.h>

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/*********************************************************************
 * CONSTANTS
 */
#ifndef UARTLOG_NUM_EVT_BUF
#  define UARTLOG_NUM_EVT_BUF     16  /* Max log records in real-time buffer  */
#endif

#ifndef UARTLOG_OUTBUF_LEN
#  define UARTLOG_OUTBUF_LEN     168  /* Size of buffer used for log printout */
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */
/* Local reduced copy of Log_EventRec */
typedef struct
{
  //xdc_runtime_Types_Timestamp64 tstamp; /* Provided by some Logger modules */
  uint32_t tstamp_cust;                   /* 16.16 fractional from RTC       */
  //xdc_Bits32 serial;                    /* Provided by some Logger modules */
  //xdc_runtime_Types_Event evt;          /* EventID and ModuleID            */
  uint16_t serial_cust;                   /* Only 16-bit serial in this impl.*/
  uint16_t evt_cust;                      /* Only event, no module ID.       */
  __TA_xdc_runtime_Log_EventRec__arg arg; /* Arguments given to log_xx(...); */
} uartLog_EventRec;

/*********************************************************************
* LOCAL VARIABLES
*/
#if (defined(xdc_runtime_Log_DISABLE_ALL) && (xdc_runtime_Log_DISABLE_ALL == 0)) || !defined(xdc_runtime_Log_DISABLE_ALL)
static UART_Handle hUart = NULL;

char              uartLog_outBuf[UARTLOG_OUTBUF_LEN + 4];
uartLog_EventRec  uartLog_evBuf[UARTLOG_NUM_EVT_BUF];
uint8_t           uartLog_tail = 0;
uint8_t           uartLog_head = 0;
uint8_t           uartLog_evBufIsEmpty = true;
uint16_t          uartLog_evtNum = 1;
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
#if (defined(xdc_runtime_Log_DISABLE_ALL) && (xdc_runtime_Log_DISABLE_ALL == 0)) || !defined(xdc_runtime_Log_DISABLE_ALL)
static void uartLog_doPrint(uartLog_EventRec *er);
#endif

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      UartLog_init
 *
 * @brief   Initializes module with the handle to the UART.
 *
 * @param   handle - UART driver handle to an initialized and opened UART.
 *
 * @return  None.
 */
void UartLog_doInit(UART_Handle handle)
{
#if (defined(xdc_runtime_Log_DISABLE_ALL) && (xdc_runtime_Log_DISABLE_ALL == 0)) || !defined(xdc_runtime_Log_DISABLE_ALL)
  hUart = handle;
#endif
}

/*********************************************************************
 * SYSTEM HOOK FUNCTIONS
 */

/*********************************************************************
 * @fn      uartLog_outputFxn
 *
 * @brief   User supplied LoggerCallback output function.
 *          typedef Void  (*LoggerCallback_OutputFxn)(UArg,Log_EventRec*,Int);
 *
 *          This function is called whenever the LoggerCallback module needs
 *          to output a log event.
 *
 *          This implementation fills a very basic ring-buffer with log records,
 *          with relatively low overhead, and relies on another function to
 *          convert log records to text and transmit the string out on UART.
 *
 *          Requires LoggerCallback to be the ILogger provider module.
 *          Initialized via LoggerCallback.outputFxn = "&uartLog_outputFxn";
 *          in the TI-RTOS configuration script.
 *
 *          Note that Log must be included as well to use the interface, and
 *          a LoggerCallback instance must be made and assigned as default.
 *          In total, therefore:
 *
 *            var Log = xdc.useModule('xdc.runtime.Log');
 *            var LoggerCallback = xdc.useModule('xdc.runtime.LoggerCallback');
 *
 *            LoggerCallback.outputFxn = "&uartPrintf_logOutputFxn";
 *
 *            var loggerParams = new LoggerCallback.Params();
 *            Defaults.common$.logger = LoggerCallback.create(loggerParams);
 *
 * @param   a0 - User supplied argument when creating the LoggerCallback logger.
 *          pRec - Pointer to Log_EventRec created by LoggerCallback
 *          numArgs - Number of arguments used in pRec.
 *
 * @return  None.
 *
 * @post    ::uartLog_head is incremented, and if it becomes equal to _tail,
 *          both are moved together, discarding the oldest record.
 */
void uartLog_outputFxn(UArg a0, Log_EventRec *pRec, int32_t numArgs)
{
#if (defined(xdc_runtime_Log_DISABLE_ALL) && (xdc_runtime_Log_DISABLE_ALL == 0)) || !defined(xdc_runtime_Log_DISABLE_ALL)
  unsigned int key;
  uint8_t moveTail = 0;

  /* Disable interrupts while adding record */
  key = Hwi_disable();

  /* Copy into current head */
  //uartLog_evBuf[uartLog_head].tstamp = pRec->tstamp; /* If real Log_EvtRec */
  uartLog_evBuf[uartLog_head].tstamp_cust = AONRTCCurrentCompareValueGet();
  uartLog_evBuf[uartLog_head].serial_cust = uartLog_evtNum;
  uartLog_evtNum++;
  //uartLog_evBuf[uartLog_head].serial = pRec->serial;  /* If LogEvtRec */
  uartLog_evBuf[uartLog_head].evt_cust = pRec->evt >> 16; /* Ignore ModuleID */
  memcpy(uartLog_evBuf[uartLog_head].arg,
         pRec->arg,
         sizeof(__TA_xdc_runtime_Log_EventRec__arg)); /* Copy log arguments */

  /* Discard oldest if buffer is full */
  if ( !uartLog_evBufIsEmpty && (uartLog_head == uartLog_tail) )
  {
    moveTail = 1;
  }

  /* Increment head with wrap */
  uartLog_head += 1;
  if (uartLog_head == UARTLOG_NUM_EVT_BUF)
  {
    uartLog_head = 0;
  }

  if (moveTail)
  {
    uartLog_tail = uartLog_head;
  }

  /* This is used to discern whether head==tail means empty or full*/
  uartLog_evBufIsEmpty = false;

  /* Let mayhem commence */
  Hwi_restore(key);
#endif
};


/*********************************************************************
 * @fn      uartLog_flush
 *
 * @brief   Log-buffer flush function
 *
 *          In this implementation it is intended to be called by the
 *          Idle task when nothing else is running.
 *
 *          This is achieved by setting up the Idle task in the TI-RTOS
 *          configuration script like so:
 *
 *          var Idle = xdc.useModule('ti.sysbios.knl.Idle');
 *          Idle.addFunc('&uartLog_flush');
 *
 *    NOTE: This must be added _before_ the Power driver is included, in order
 *          to output the pending log messages before going to sleep.
 *
 *          Uses a utility function to convert a log record to a user-friendlier
 *          string which is then printed to UART.
 *
 * @param   None. Relies on global state.
 *
 * @return  None.
 *
 * @post    ::uartLog_tail is incremented to where uartLog_head is, then returns
 */
void uartLog_flush()
{
#if (defined(xdc_runtime_Log_DISABLE_ALL) && (xdc_runtime_Log_DISABLE_ALL == 0)) || !defined(xdc_runtime_Log_DISABLE_ALL)
  unsigned int key;

  /* Local copy of current event record. To keep atomic section short. */
  uartLog_EventRec curRec;

  /* If we don't have UART, don't bother. */
  if (NULL == hUart)
    return;

  /* In the Idle function (this) send all messages. Will be preempted. */
  while(!uartLog_evBufIsEmpty)
  {
    /* Atomic section while manipulating the buffer. */
    key = Hwi_disable();

    /* Extract oldest and move tail */
    curRec = uartLog_evBuf[uartLog_tail];
    uartLog_tail = (uartLog_tail + 1) % UARTLOG_NUM_EVT_BUF;
    if (uartLog_tail == uartLog_head)
      uartLog_evBufIsEmpty = true;

    /* Let the others play. */
    Hwi_restore(key);

    /* Prepare log string from record, and print to UART. */
    uartLog_doPrint(&curRec);
  }
#endif
}


/*********************************************************************
 * INTERNAL FUNCTIONS
 */
/*********************************************************************
 * @fn      uartLog_doPrint
 *
 * @brief   Converts log records to strings.
 *
 *          This is a copy of ti.xdc.runtime.Log's doPrint method, but
 *          instead of calling System_printf, it writes into a static buffer
 *          which is then sent to the UART driver as an atomic unit.
 *
 * @param   er - Log_EventRecord to be parsed and output.
 *
 * @return  None.
 */
#if (defined(xdc_runtime_Log_DISABLE_ALL) && (xdc_runtime_Log_DISABLE_ALL == 0)) || !defined(xdc_runtime_Log_DISABLE_ALL)
static void uartLog_doPrint(uartLog_EventRec *er)
{
  Text_RopeId rope;
  char       *fmt;
  //uint32_t    hi, lo;
  char       *bufPtr = uartLog_outBuf;
  char       *bufEndPtr = uartLog_outBuf + UARTLOG_OUTBUF_LEN - 2; // Less 2 for \r\n

  /* print serial number if there is one; 0 isn't a valid serial number */
  if (er->serial_cust) {
    System_snprintf(bufPtr, (bufEndPtr - bufPtr), "#%06u ", er->serial_cust);
    bufPtr = uartLog_outBuf + strlen(uartLog_outBuf);
  }

  /* print timestamp if there is one; ~0 isn't a valid timestamp value */
  /* Formatting of Log_EvtRec timestamps. But LoggerCallback doesn't provide.
  hi = er->tstamp.hi;
  lo = er->tstamp.lo;
  if (lo != ~0 && hi != ~0) {
    System_snprintf(bufPtr, (bufEndPtr - bufPtr), "[t=0x");
    bufPtr = uartLog_outBuf + strlen(uartLog_outBuf);
    if (hi) {
      System_snprintf(bufPtr, (bufEndPtr - bufPtr), HI, hi);
      bufPtr = uartLog_outBuf + strlen(uartLog_outBuf);
    }
    System_snprintf(bufPtr, (bufEndPtr - bufPtr), LO, lo);
    bufPtr = uartLog_outBuf + strlen(uartLog_outBuf);
  }
  */

  uint16_t seconds = er->tstamp_cust>>16;
  uint16_t ifraction = er->tstamp_cust & 0xFFFF;
  int fraction = (int)((double)ifraction/65536 * 1000);  // Get 3 decimals

  System_snprintf(bufPtr, (bufEndPtr - bufPtr), "[ %d.%03u ] ", seconds, fraction);
  bufPtr = uartLog_outBuf + strlen(uartLog_outBuf);

  /* print module name. This is ignored in this implementation.
  Text_putMod((Text_RopeId)Types_getModuleId(er->evt), &bufPtr, -1);
  System_snprintf(bufPtr, (bufEndPtr - bufPtr), ": ");
  bufPtr = uartLog_outBuf + strlen(uartLog_outBuf);
  */

  /* Ouput everything till now and start over in the buffer. */
  UART_write(hUart, uartLog_outBuf, (bufPtr - uartLog_outBuf));
  bufPtr = uartLog_outBuf;

  /* print event */
  /* rope = Types_getEventId(er->evt); */  /* the event id is the message rope */
  rope = er->evt_cust; /* Don't need getEventId as we already have it */
  if (rope == 0) {
    /* Log_print() event */
    System_snprintf(bufPtr, (bufEndPtr - bufPtr), (String)iargToPtr(er->arg[0]),
                    er->arg[1], er->arg[2], er->arg[3], er->arg[4],
                    er->arg[5], er->arg[6], 0, 0);
    bufPtr = uartLog_outBuf + strlen(uartLog_outBuf);
  }
  else {
    /* Log_write() event */
    fmt = Text_ropeText(rope);

    if (Text_isLoaded) {
      System_snprintf(bufPtr, (bufEndPtr - bufPtr), fmt, er->arg[0], er->arg[1],
                      er->arg[2], er->arg[3], er->arg[4], er->arg[5],
                      er->arg[6], er->arg[7]);
      bufPtr = uartLog_outBuf + strlen(uartLog_outBuf);
    }
    else {
      System_snprintf(bufPtr, (bufEndPtr - bufPtr),
                      "{evt: fmt=%p, args=[0x%x, 0x%x ...]}",
                      fmt, er->arg[0], er->arg[1]);
      bufPtr = uartLog_outBuf + strlen(uartLog_outBuf);
    }
  }

  *bufPtr++ = '\r';
  *bufPtr++ = '\n';


  UART_write(hUart, uartLog_outBuf, (bufPtr - uartLog_outBuf));
}
#endif
