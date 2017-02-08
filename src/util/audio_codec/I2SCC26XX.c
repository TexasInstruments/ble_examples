/*
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <xdc/runtime/Error.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Types.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

// TODO: Create correct path
//#include <ti/drivers/i2s/I2SCC26XX.h>
#include "I2SCC26XX.h"

#include <string.h>

/* driverlib header files */
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <inc/hw_types.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/ioc.h>
#include <driverlib/rom.h>
#include <driverlib/prcm.h>
#include <driverlib/i2s.h>

/* I2SCC26XX functions */
void            I2SCC26XX_init(I2SCC26XX_Handle handle);
I2SCC26XX_Handle  I2SCC26XX_open(I2SCC26XX_Handle handle, I2SCC26XX_Params *params);
void            I2SCC26XX_close(I2SCC26XX_Handle handle);
bool            I2SCC26XX_startStream(I2SCC26XX_Handle handle);
bool            I2SCC26XX_stopStream(I2SCC26XX_Handle handle);
bool            I2SCC26XX_requestBuffer(I2SCC26XX_Handle handle, I2SCC26XX_BufferRequest *bufferRequest);
void            I2SCC26XX_releaseBuffer(I2SCC26XX_Handle handle, I2SCC26XX_BufferRelease *bufferRelease);
/* Internal functions */
static void I2SCC26XX_initHw(I2SCC26XX_Handle handle);
static bool I2SCC26XX_initIO(I2SCC26XX_Handle handle);
static void I2SCC26XX_hwiFxn (UArg arg);
static void I2SCC26XX_callback(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *msg);
static void I2SCC26XX_cleanUpQueues();

#ifdef I2S_DEBUG
    static int32_t dbgCntDmaIn = 0, dbgCntDmaOut = 0, dbgCntPtrErr = 0;
    static int32_t dbgCntI2SReqBuf = 0;
    static int32_t dbgCntI2SRelBuf = 0;
#endif //I2S_DEBUG

#ifdef I2S_DEBUG
typedef enum {
    NODE_INDICATOR_NO_QUEUE,       // 0: not part of queue
    NODE_INDICATOR_AVAIL_QUEUE,    // 1: part of available
    NODE_INDICATOR_READY_QUEUE,    // 2: part of ready,
    NODE_INDICATOR_NEXT,           // 3: next
    NODE_INDICATOR_ACTIVE,         // 4: active
    NODE_INDICATOR_USER,           // 5: handled by user
    NODE_INDICATOR_REUSED,         // 6: reusable
} queueIndicator_t;
#endif
typedef struct
{
  Queue_Elem  _elem;    // queue element
#ifdef I2S_DEBUG
  queueIndicator_t queueIndicator;
#endif //I2S_DEBUG
  void *buf;        // Buffer pointer, allocated by caller
  // TODO: Add output buffer
} queueNodeI2S_t;

//queueNodeI2S_t nodeTable[NUM_OF_NODES];

#ifdef I2S_DEBUG
static queueNodeI2S_t * shadowQueueNodeTable[I2SCC26XX_QUEUE_SIZE];
#endif //I2S_DEBUG

static queueNodeI2S_t *i2sBlockActiveIn = NULL; // Reference to the element which is currently being filled by I2S DMA In
static queueNodeI2S_t *i2sBlockNextIn = NULL; // Reference to the next element which will be filled by I2S DMA In
static queueNodeI2S_t *i2sBlockRepeatIn = NULL; //  Reference to an reusable element which will be filled by I2S DMA In

static queueNodeI2S_t *i2sBlockActiveOut = NULL; // Reference to the element which is currently being used by I2S DMA Out
static queueNodeI2S_t *i2sBlockNextOut = NULL; // Reference to the next element which will be used by I2S DMA Out
static queueNodeI2S_t *i2sBlockRepeatOut = NULL; // Reference to an reusable element which will be used by I2S DMA Out

static Queue_Struct i2sBlockAvailIn;
static Queue_Handle i2sBlockAvailInQueue;

static Queue_Struct i2sBlockReadyIn;
static Queue_Handle i2sBlockReadyInQueue;

static Queue_Struct i2sBlockAvailOut;
static Queue_Handle i2sBlockAvailOutQueue;

static Queue_Struct i2sBlockReadyOut;
static Queue_Handle i2sBlockReadyOutQueue;

I2SControlTable g_ControlTable;

/*
 *  ======== I2SCC26XX_init ========
 *  @pre    Function assumes that the handle is not NULL
 */
void I2SCC26XX_init(I2SCC26XX_Handle handle) {
    I2SCC26XX_Object         *object;

    /* Get the pointer to the object */
    object = handle->object;

    /* Mark the object as available */
    object->isOpen = false;

    /* Make sure struct in driverlib I2S driver is initialized */
    g_pControlTable = &g_ControlTable;
}

/*
 *  ======== I2SCC26XX_open ========
 *  @pre    Function assumes that the handle is not NULL
 */
I2SCC26XX_Handle I2SCC26XX_open(I2SCC26XX_Handle handle, I2SCC26XX_Params *params) {
    /* Use union to save on stack allocation */
    union {
        Semaphore_Params semParams;
        Hwi_Params hwiParams;
    } paramsUnion;
    I2SCC26XX_Object         *object;
    I2SCC26XX_HWAttrs const  *hwAttrs;
    unsigned int             key;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Disable preemption while checking if the I2S is open. */
    key = Hwi_disable();

    /* Check if the I2S is open already with the base addr. */
    if (object->isOpen == true) {
        Hwi_restore(key);

        Log_warning1("I2S:(%p) already in use.", hwAttrs->baseAddr);

        return (NULL);
    }

    /* Mark the handle as being used */
    object->isOpen = true;
    Hwi_restore(key);

    /* Initialize the I2S object */
    object->currentStream = NULL;
    object->requestMode                 = params->requestMode;
    object->ui32requestTimeout          = params->ui32requestTimeout;
    object->i32SampleRate               = params->i32SampleRate;
    object->audioClkCfg                 = params->audioClkCfg;
    object->audioPinCfg                 = params->audioPinCfg;
    object->audioFmtCfg                 = params->audioFmtCfg;
    object->blockSize                   = params->blockSize;
    object->pvContBuffer                = params->pvContBuffer;
    object->ui32conBufTotalSize         = params->ui32conBufTotalSize;
    object->pvContMgtBuffer             = params->pvContMgtBuffer;
    object->ui32conMgtBufTotalSize      = params->ui32conMgtBufTotalSize;
    object->currentStream               = params->currentStream;
    object->currentStream->status       = I2SCC26XX_STREAM_IDLE;

    /* The following are constants that apply to PDM */
    object->i32SampleRate = 0;//I2S_SAMPLE_RATE_16K;                             /* If negative then use user configured clock division */
    object->audioClkCfg.wclkDiv = 250; //16                                  /* I2S Word Clock divider override*/
    object->audioClkCfg.sampleOnPositiveEdge = I2SCC26XX_SampleEdge_Postive; /* I2S Sample Edge */
    object->audioClkCfg.wclkPhase = I2SCC26XX_WordClockPhase_Dual;           /* I2S Word Clock Phase */
    object->audioClkCfg.wclkInverted = I2SCC26XX_ClockSource_Normal;         /* I2S Invert Word Clock */
    object->audioClkCfg.wclkSource = I2SCC26XX_WordClockSource_Int;          /* I2S Word Clock source */
    object->audioClkCfg.bclkDiv = 6;//94;                                    /* I2S Bit Clock divider override */
    object->audioClkCfg.reserved = 0;
    object->audioClkCfg.bclkSource = I2SCC26XX_BitClockSource_Int;           /* I2S Bit Clock source */
    object->audioClkCfg.mclkDiv = 4;                                         /* I2S Master Clock divider override */

    object->audioPinCfg.bitFields.ad1Usage = I2SCC26XX_ADUsageDisabled;      /* I2S AD1 usage (0: Disabled, 1: Input, 2: Output) */
    object->audioPinCfg.bitFields.enableMclkPin = I2SCC26XX_GENERIC_DISABLED;/* I2S Enable Master clock output on pin */
    object->audioPinCfg.bitFields.reserved = 0;
    object->audioPinCfg.bitFields.ad1NumOfChannels = 0;                      /* I2S AD1 number of channels (1 - 8). !Must match channel mask */
    object->audioPinCfg.bitFields.ad1ChannelMask = I2SCC26XX_DISABLED_MODE;  /* I2S AD1 Channel Mask */
#ifdef AUDIO_RECEIVER
    object->audioPinCfg.bitFields.ad0Usage = I2SCC26XX_ADUsageOutput;        /* I2S AD0 usage (0: Disabled, 1: Input, 2: Output) */
#else //AUDIO_TRANSMITTER
    object->audioPinCfg.bitFields.ad0Usage = I2SCC26XX_ADUsageInput;         /* I2S AD0 usage (0: Disabled, 1: Input, 2: Output) */
#endif
    object->audioPinCfg.bitFields.enableWclkPin = I2SCC26XX_GENERIC_ENABLED; /* I2S Enable Word clock output on pin */
    object->audioPinCfg.bitFields.enableBclkPin = I2SCC26XX_GENERIC_ENABLED; /* I2S Enable Bit clock output on pin */
    object->audioPinCfg.bitFields.ad0NumOfChannels = 1;                      /* I2S AD0 number of channels (1 - 8). !Must match channel mask. \sa PDM_NUM_OF_CHANNELS */
    object->audioPinCfg.bitFields.ad0ChannelMask = I2SCC26XX_MONO_MODE;      /* I2S AD0 Channel Mask */

    object->audioFmtCfg.wordLength = I2SCC26XX_WordLength16;                 /* Number of bits per word (8-24). Exact for single phase, max for dual phase */
    object->audioFmtCfg.sampleEdge = I2SCC26XX_PositiveEdge;                 /* Data and Word clock is samples, and clocked out, on opposite edges of BCLK */
    object->audioFmtCfg.dualPhase = I2SCC26XX_DualPhase;                     /* Selects dual- or single phase format (0: Single, 1: Dual) */
    object->audioFmtCfg.memLen = I2SCC26XX_MemLen16bit;                      /* Size of each word stored to or loaded from memory (0: 16, 1: 24) */
    object->audioFmtCfg.dataDelay = I2SCC26XX_FormatI2SandDSP;               /* Number of BCLK perids between a WCLK edge and MSB of the first word in a phase */
    // Find out how many channels are In and Out respectively
    uint8_t ui8TotalNumberOfChannelsIn = 0;
    uint8_t ui8TotalNumberOfChannelsOut = 0;
    switch (object->audioPinCfg.bitFields.ad0Usage)
    {
    case I2SCC26XX_ADUsageInput:
        ui8TotalNumberOfChannelsIn += object->audioPinCfg.bitFields.ad0NumOfChannels;
        break;
    case I2SCC26XX_ADUsageOutput:
        ui8TotalNumberOfChannelsOut += object->audioPinCfg.bitFields.ad0NumOfChannels;
        break;
    }
    switch (object->audioPinCfg.bitFields.ad1Usage)
    {
    case I2SCC26XX_ADUsageInput:
        ui8TotalNumberOfChannelsIn += object->audioPinCfg.bitFields.ad1NumOfChannels;
        break;
    case I2SCC26XX_ADUsageOutput:
        ui8TotalNumberOfChannelsOut += object->audioPinCfg.bitFields.ad1NumOfChannels;
        break;
    }

    uint32_t ui32BlockSizeInBytesIn = (object->blockSize * ( (object->audioFmtCfg.memLen) ? 3 : 2 ) * ui8TotalNumberOfChannelsIn);
    uint32_t ui32BlockSizeInBytesOut = (object->blockSize * ( (object->audioFmtCfg.memLen) ? 3 : 2 ) * ui8TotalNumberOfChannelsOut);
    uint32_t ui32NumberOfBlocks = (object->ui32conBufTotalSize/(ui32BlockSizeInBytesIn + ui32BlockSizeInBytesOut));
    if (ui32NumberOfBlocks < I2SCC26XX_MIN_ALLOWED_QUEUE_SIZE) {
        Log_warning2("I2S:(%p) Buffer size %d too small.",
                     hwAttrs->baseAddr,
                     ui32BlockSizeInBytesIn * ui32NumberOfBlocks);

        /* Release power dependency - i.e. potentially power down serial domain. */
        //Power_releaseDependency(hwAttrs->powerMngrId);

        /* Mark the module as available */
        key = Hwi_disable();
        object->isOpen = false;
        Hwi_restore(key);

        /* Signal back to application that I2S driver was not succesfully opened */
        return (NULL);
    }
    /* Check if we have enough buffer for management */
    uint32_t ui32TotalNumberOfBlocks = 0;
    if (ui32BlockSizeInBytesIn) {
        ui32TotalNumberOfBlocks += ui32NumberOfBlocks;
    }
    if (ui32BlockSizeInBytesOut) {
        ui32TotalNumberOfBlocks += ui32NumberOfBlocks;
    }
    if ( (ui32TotalNumberOfBlocks * sizeof(queueNodeI2S_t)) < object->ui32conMgtBufTotalSize ) {
        /* Not enough memory has been allocated */
        Log_warning0("Not enough memory provided");

        /* Release power dependency - i.e. potentially power down serial domain. */
        //Power_releaseDependency(hwAttrs->powerMngrId);

        /* Mark the module as available */
        key = Hwi_disable();
        object->isOpen = false;
        Hwi_restore(key);

        /* Signal back to application that I2S driver was not succesfully opened */
        return (NULL);
    }

    Log_print2(Diags_USER2,"I2S:(%p) Found memory for %d blocks",
                            hwAttrs->baseAddr, ui32NumberOfBlocks);

    /* Register power dependency - i.e. power up and enable clock for I2S. */
    Power_setDependency(hwAttrs->powerMngrId);
    /* Add dependency on uDMA to keep bus alive in IDLE */
    Power_setDependency(PowerCC26XX_PERIPH_UDMA);

    /* Configure IOs after hardware has been initialized so that IOs aren't */
    /* toggled unnecessary and make sure it was successful */
    if (!I2SCC26XX_initIO(handle)) {
        /* Trying to use I2S driver when some other driver or application
        *  has already allocated these pins, error! */
        Log_warning0("Could not allocate I2S pins, already in use.");

        /* Release power dependency - i.e. potentially power down serial domain. */
        Power_releaseDependency(hwAttrs->powerMngrId);

        /* Mark the module as available */
        key = Hwi_disable();
        object->isOpen = false;
        Hwi_restore(key);

        /* Signal back to application that I2S driver was not succesfully opened */
        return (NULL);
    }

    /* First part of the buffer is used for input */
    if (ui32BlockSizeInBytesIn) {
        int i = 0;
        /* Setup queues now that we now whether they are needed */
        Queue_construct(&i2sBlockReadyIn, NULL);
        i2sBlockReadyInQueue = Queue_handle(&i2sBlockReadyIn);

        Queue_construct(&i2sBlockAvailIn, NULL);
        i2sBlockAvailInQueue = Queue_handle(&i2sBlockAvailIn);

        for (i = 0; i < ui32NumberOfBlocks ; i++) {
            queueNodeI2S_t * tmpNode = (queueNodeI2S_t *)&((uint8_t *)object->pvContMgtBuffer)[i * sizeof(queueNodeI2S_t)];
            // Split continuous buffer
            tmpNode->buf = &((uint8_t *)object->pvContBuffer)[i * ui32BlockSizeInBytesIn];
            if (i == (ui32NumberOfBlocks - 1)) {
                i2sBlockRepeatIn = tmpNode;
#ifdef I2S_DEBUG
                tmpNode->queueIndicator = NODE_INDICATOR_REUSED;
#endif //I2S_DEBUG
            }
            else {
                Queue_enqueue(i2sBlockAvailInQueue, &tmpNode->_elem);
#ifdef I2S_DEBUG
                tmpNode->queueIndicator = NODE_INDICATOR_AVAIL_QUEUE;
#endif //I2S_DEBUG
            }
#ifdef I2S_DEBUG
            shadowQueueNodeTable[i] = tmpNode;
#endif //I2S_DEBUG
        }
    }
    else {
        i2sBlockReadyInQueue = NULL;
    }
    /* Second part of the buffer is used for output */
    if (ui32BlockSizeInBytesOut) {
        int i = 0;
        /* Setup queues now that we now whether they are needed */
        Queue_construct(&i2sBlockReadyOut, NULL);
        i2sBlockReadyOutQueue = Queue_handle(&i2sBlockReadyOut);

        Queue_construct(&i2sBlockAvailOut, NULL);
        i2sBlockAvailOutQueue = Queue_handle(&i2sBlockAvailOut);

        uint32_t ui32NumberOfBlocksIn = object->ui32conBufTotalSize/ui32BlockSizeInBytesIn;
        for (i = 0; i < ui32NumberOfBlocks ; i++) {
            queueNodeI2S_t * tmpNode = (queueNodeI2S_t *)&((uint8_t *)object->pvContMgtBuffer)[(i * sizeof(queueNodeI2S_t)) + (ui32NumberOfBlocksIn * sizeof(queueNodeI2S_t))];
            // Split continuous buffer
            tmpNode->buf = &((uint8_t *)object->pvContBuffer)[(i * ui32BlockSizeInBytesOut) + (ui32NumberOfBlocks * ui32BlockSizeInBytesIn)];
            if (i == (ui32NumberOfBlocks - 1)) {
                i2sBlockRepeatOut = tmpNode;
#ifdef I2S_DEBUG
                tmpNode->queueIndicator = NODE_INDICATOR_REUSED;
#endif //I2S_DEBUG
            }
            else {
                Queue_enqueue(i2sBlockAvailOutQueue, &tmpNode->_elem);
#ifdef I2S_DEBUG
                tmpNode->queueIndicator = NODE_INDICATOR_AVAIL_QUEUE;
#endif //I2S_DEBUG
            }
#ifdef I2S_DEBUG
            shadowQueueNodeTable[i] = tmpNode;
#endif //I2S_DEBUG
        }
    }
    else {
        i2sBlockReadyOutQueue = NULL;
    }

    /* Create the Hwi for this I2S peripheral. */
    Hwi_Params_init(&paramsUnion.hwiParams);
    paramsUnion.hwiParams.arg = (UArg) handle;
    paramsUnion.hwiParams.priority = hwAttrs->intPriority;
    Hwi_construct(&(object->hwi), (int) hwAttrs->intNum, I2SCC26XX_hwiFxn, &paramsUnion.hwiParams, NULL);

    /* Create a semaphore to block task execution while stopping the stream */
    Semaphore_Params_init(&paramsUnion.semParams);
    paramsUnion.semParams.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&(object->semStopping), 0, &paramsUnion.semParams);

    /* Check the transfer mode */
    if (object->requestMode == I2SCC26XX_MODE_BLOCKING) {
        Log_print1(Diags_USER2, "I2S:(%p) in I2SCC26XX_MODE_BLOCKING mode",
                                 hwAttrs->baseAddr);

        /* Create a semaphore to block task execution for the duration of the
         * I2S transfer */
        Semaphore_Params_init(&paramsUnion.semParams);
        paramsUnion.semParams.mode = Semaphore_Mode_COUNTING;
        Semaphore_construct(&(object->blockComplete), 0, &paramsUnion.semParams);

        /* Store internal callback function */
        object->callbackFxn = I2SCC26XX_callback;
    }
    else {
        Log_print1(Diags_USER2, "I2S:(%p) in I2SCC26XX_MODE_NONBLOCKING mode", hwAttrs->baseAddr);

        /* Check to see if a callback function was defined for async mode */
        Assert_isTrue(params->callbackFxn != NULL, NULL);

        /* Save the callback function pointer */
        object->callbackFxn = params->callbackFxn;
    }

    Log_print1(Diags_USER2, "I2S:(%p) opened", hwAttrs->baseAddr);

    /* Register notification functions */
//    Power_registerNotify(&object->i2sPostObj, Power_AWAKE_STANDBY, (Fxn)i2sPostNotify, (UInt32)handle, NULL );

    return (handle);
}

/*
 *  ======== I2SCC26XX_close ========
 *  @pre    Function assumes that the handle is not NULL
 */
void I2SCC26XX_close(I2SCC26XX_Handle handle) {
    unsigned int                 key;
    I2SCC26XX_Object             *object;
    I2SCC26XX_HWAttrs const      *hwAttrs;

    /* Get the pointer to the object and hwAttrs */
    hwAttrs = handle->hwAttrs;
    object = handle->object;

    /* Deallocate pins */
    PIN_close(object->pinHandle);

    /* Disable the I2S */
    I2SDisable(hwAttrs->baseAddr);

    /* Destroy the Hwi */
    Hwi_destruct(&(object->hwi));

    /* Release power dependency on I2S. */
    Power_releaseDependency(hwAttrs->powerMngrId);
    /* Release power dependency on UDMA. */
    Power_releaseDependency(PowerCC26XX_PERIPH_UDMA);

    /* Destroy the stopping semaphore */
    Semaphore_destruct(&(object->semStopping));

    if (object->requestMode == I2SCC26XX_MODE_BLOCKING) {
        /* Destroy the block complete semaphore */
        Semaphore_destruct(&(object->blockComplete));
    }

    /* Clean up queues before destroying queues*/
    I2SCC26XX_cleanUpQueues();

    if (i2sBlockReadyInQueue) {
        Queue_destruct(&i2sBlockReadyIn);
        Queue_destruct(&i2sBlockAvailIn);
    }

    i2sBlockActiveIn = NULL;
    i2sBlockNextIn = NULL;

    if (i2sBlockReadyOutQueue) {
        Queue_destruct(&i2sBlockReadyOut);
        Queue_destruct(&i2sBlockAvailOut);
    }

    i2sBlockActiveOut = NULL;
    i2sBlockNextOut = NULL;

    /* Mark the module as available */
    key = Hwi_disable();
    object->isOpen = false;
    Hwi_restore(key);

    Log_print1(Diags_USER2, "I2S:(%p) closed", hwAttrs->baseAddr);
}

/*
 *  ======== I2SCC26XX_hwiFxn ========
 *  ISR for the I2S
 */
static void I2SCC26XX_hwiFxn (UArg arg) {
    I2SCC26XX_StreamNotification *notification;
    I2SCC26XX_Object        *object;
    I2SCC26XX_HWAttrs const *hwAttrs;
    uint32_t                intStatus;

    /* Get the pointer to the object and hwAttrs */
    object = ((I2SCC26XX_Handle)arg)->object;
    hwAttrs = ((I2SCC26XX_Handle)arg)->hwAttrs;

    Log_print1(Diags_USER2, "I2S:(%p) interrupt context start", hwAttrs->baseAddr);

    /* Get the interrupt status of the I2S controller */
    intStatus = I2SIntStatus(hwAttrs->baseAddr, true);
    I2SIntClear(hwAttrs->baseAddr, intStatus);

    if (intStatus & I2S_IRQMASK_AIF_DMA_IN) {
#ifdef I2S_DEBUG
        dbgCntDmaIn++;
        if (dbgCntDmaIn == 2) {
            asm(" NOP");
        }
        if (dbgCntDmaIn == 300) {
            asm(" NOP");
        }
#endif //I2S_DEBUG
        /* Move completed buffer to ready queue, if not reusable buffer */
        if (i2sBlockActiveIn != i2sBlockRepeatIn) {
          /* Move completed buffer to ready queue */
          Queue_put(i2sBlockReadyInQueue, &i2sBlockActiveIn->_elem);
#ifdef I2S_DEBUG
          i2sBlockActiveIn->queueIndicator = NODE_INDICATOR_READY_QUEUE;
#endif //I2S_DEBUG
        }
        /* Setup next active buffer */
        i2sBlockActiveIn = i2sBlockNextIn;
        /* Mark next buffer as empty*/
        i2sBlockNextIn = NULL;
#ifdef I2S_DEBUG
        // Avoid null-pointer exception
        if (i2sBlockActiveIn) {
            i2sBlockActiveIn->queueIndicator = NODE_INDICATOR_ACTIVE;
        }
        else {
            // This can happen if stream is stopping
            asm(" NOP");
        }
#endif //I2S_DEBUG

        if (object->currentStream->status == I2SCC26XX_STREAM_STOPPING) {
            /* Part of shut down sequence*/
            object->currentStream->status = I2SCC26XX_STREAM_STOPPED;
        }
        else if (object->currentStream->status != I2SCC26XX_STREAM_STOPPED) {
            if (!Queue_empty(i2sBlockAvailInQueue)) {
                i2sBlockNextIn = Queue_get(i2sBlockAvailInQueue);
#ifdef I2S_DEBUG
                i2sBlockNextIn->queueIndicator = NODE_INDICATOR_NEXT;
#endif //I2S_DEBUG
                I2SPointerSet(hwAttrs->baseAddr, true, (uint32_t *)i2sBlockNextIn->buf);
                /* Buffer is ready */
                object->currentStream->status = I2SCC26XX_STREAM_BUFFER_READY;
            }
            else
            {
                //ITM_Port32(2) = 0xDEADABBA;
                object->currentStream->status = I2SCC26XX_STREAM_BUFFER_READY_BUT_NO_AVAILABLE_BUFFERS;
                // Send reusable frames
                i2sBlockNextIn = i2sBlockRepeatIn;
#ifdef I2S_DEBUG
                i2sBlockNextIn->queueIndicator = NODE_INDICATOR_REUSED;
#endif //I2S_DEBUG
                I2SPointerSet(hwAttrs->baseAddr, true, (uint32_t *)i2sBlockNextIn->buf);
            }

            /* Use a temporary stream pointer in case the callback function
            * attempts to perform another I2SCC26XX_bufferRequest call
            */
            notification = object->currentStream;

            /* Notify caller about availability of buffer */
            object->callbackFxn((I2SCC26XX_Handle)arg, notification);
        }
    }

    if (intStatus & I2S_IRQMASK_AIF_DMA_OUT) {
#ifdef I2S_DEBUG
        dbgCntDmaOut++;
        if (dbgCntDmaOut == 2) {
            asm(" NOP");
        }
        if (dbgCntDmaOut == 300) {
            asm(" NOP");
        }
#endif //I2S_DEBUG
        /* Move completed buffer to ready queue, if not reusable buffer */
        if (i2sBlockActiveOut != i2sBlockRepeatOut) {
            Queue_put(i2sBlockReadyOutQueue, &i2sBlockActiveOut->_elem);
#ifdef I2S_DEBUG
            i2sBlockActiveOut->queueIndicator = NODE_INDICATOR_READY_QUEUE;
#endif //I2S_DEBUG
        }
        /* Setup next active buffer */
        i2sBlockActiveOut = i2sBlockNextOut;
        /* Mark next buffer as empty*/
        i2sBlockNextOut = NULL;
#ifdef I2S_DEBUG
        // Avoid null-pointer exception
        if (i2sBlockActiveOut) {
            i2sBlockActiveOut->queueIndicator = NODE_INDICATOR_ACTIVE;
        }
        else {
            // This can happen if stream is stopping
            asm(" NOP");
        }
#endif //I2S_DEBUG

        if (object->currentStream->status == I2SCC26XX_STREAM_STOPPING) {
            /* Part of shut down sequence*/
            object->currentStream->status = I2SCC26XX_STREAM_STOPPED;
        }
        else if (object->currentStream->status != I2SCC26XX_STREAM_STOPPED) {
            if ((dbgCntI2SRelBuf > 3) && !Queue_empty(i2sBlockAvailOutQueue))
            {
                i2sBlockNextOut = Queue_get(i2sBlockAvailOutQueue);
#ifdef I2S_DEBUG
                i2sBlockNextOut->queueIndicator = NODE_INDICATOR_NEXT;
#endif //I2S_DEBUG
                I2SPointerSet(hwAttrs->baseAddr, false, (uint32_t *)i2sBlockNextOut->buf);
                /* Buffer is ready */
                object->currentStream->status = I2SCC26XX_STREAM_BUFFER_READY;
            }
            else
            {
                //ITM_Port32(2) = 0xDEADABBA;
                object->currentStream->status = I2SCC26XX_STREAM_BUFFER_READY_BUT_NO_AVAILABLE_BUFFERS;
                if (i2sBlockReadyOutQueue) {
                  // Send reusable frames
                  i2sBlockNextOut = i2sBlockRepeatOut;
                  // Copy data from previous frame, best to repeat this
                  memcpy(i2sBlockRepeatOut->buf, i2sBlockActiveOut->buf, object->blockSize * ((object->audioFmtCfg.memLen) ? 3 : 2 ));
#ifdef I2S_DEBUG
                  i2sBlockNextOut->queueIndicator = NODE_INDICATOR_REUSED;
#endif //I2S_DEBUG
                  I2SPointerSet(hwAttrs->baseAddr, false, (uint32_t *)i2sBlockNextOut->buf);
                }
            }

            /* Use a temporary stream pointer in case the callback function
            * attempts to perform another I2SCC26XX_bufferRequest call
            */
            notification = object->currentStream;
            object->currentStream->arg = shadowQueueNodeTable;

            /* Notify caller about availability of buffer */
            object->callbackFxn((I2SCC26XX_Handle)arg, notification);
        }
    }

    /* Error handling:
    * Overrun in the RX Fifo -> at least one sample in the shift
    * register has been discarded  */
    if (intStatus & I2S_IRQMASK_PTR_ERR) {
#ifdef I2S_DEBUG
        dbgCntPtrErr++;
#endif //I2S_DEBUG
        /* disable the interrupt */
        I2SIntDisable(hwAttrs->baseAddr, I2S_INT_PTR_ERR);
        /* Check if we are expecting this interrupt as part of stopping */
        if ( object->currentStream->status == I2SCC26XX_STREAM_STOPPED ) {
            /* This happened because I2SCC26XX_stopStream was called for some reason
             * Post the semaphore
             */
            Semaphore_post(Semaphore_handle(&(object->semStopping)));
        }
        else {
            asm(" NOP");
            object->currentStream->status = I2SCC26XX_STREAM_ERROR;
            /* Use a temporary stream pointer in case the callback function
            * attempts to perform another I2SCC26XX_bufferRequest call
            */
            notification = object->currentStream;
            /* Notify caller about availability of buffer */
            object->callbackFxn((I2SCC26XX_Handle)arg, notification);
            Log_print1(Diags_USER2, "I2S missing next pointer: (%p) !\n", hwAttrs->baseAddr);
        }
    }
    else if (intStatus & (I2S_INT_TIMEOUT | I2S_INT_BUS_ERR | I2S_INT_WCLK_ERR)) {
        asm(" NOP"); // Any other error
    }

    Log_print1(Diags_USER2, "I2S:(%p) interrupt context end",
               hwAttrs->baseAddr);
}

/*
 *  ======== I2SCC26XX_startStream ========
 *  @pre    Function assumes that handle is not NULL
 */
bool I2SCC26XX_startStream(I2SCC26XX_Handle handle) {
    unsigned int            key;
    I2SCC26XX_Object        *object;
    I2SCC26XX_HWAttrs const *hwAttrs;

    /* Get the pointer to the object and hwAttr*/
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Disable preemption while checking if a transfer is in progress */
    key = Hwi_disable();
    if (object->currentStream->status != I2SCC26XX_STREAM_IDLE) {
        Hwi_restore(key);

        Log_error1("I2S:(%p) stream still in progress",
                ((I2SCC26XX_HWAttrs const *)(handle->hwAttrs))->baseAddr);

        /* Flag that the transfer failed to start */
        object->currentStream->status = I2SCC26XX_STREAM_FAILED;

        /* Transfer is in progress */
        return (false);
    }

    /* Make sure to flag that a stream is now active */
    object->currentStream->status = I2SCC26XX_STREAM_STARTED;

    if (i2sBlockReadyOutQueue) {
      /* Clear repeat buffer */
      memset(i2sBlockRepeatOut->buf, 0, object->blockSize * ((object->audioFmtCfg.memLen) ? 3 : 2 ));
    }

    Hwi_restore(key);

    I2SCC26XX_cleanUpQueues();

    /* Set constraint during streaming */
    Power_setConstraint(PowerCC26XX_SB_DISALLOW);

    /* Configure the hardware module */
    I2SCC26XX_initHw(handle);

    /* Kick off clocks */
    PRCMAudioClockEnable();
    PRCMLoadSet();

    //TODO: Rename from I2S to AIF
    // Enable samplestamp
    //I2SSampleStampEnable(hwAttrs->baseAddr);

    if ((i2sBlockActiveIn == NULL) && i2sBlockReadyInQueue) {
        i2sBlockActiveIn = Queue_dequeue(i2sBlockAvailInQueue);
#ifdef I2S_DEBUG
        i2sBlockActiveIn->queueIndicator = NODE_INDICATOR_ACTIVE;
#endif //I2S_DEBUG
    }
    if ((i2sBlockNextIn == NULL) && i2sBlockReadyInQueue) {
        i2sBlockNextIn = Queue_dequeue(i2sBlockAvailInQueue);
#ifdef I2S_DEBUG
        i2sBlockNextIn->queueIndicator = NODE_INDICATOR_NEXT;
#endif //I2S_DEBUG
    }

    if ((i2sBlockActiveOut == NULL) && i2sBlockReadyOutQueue) {
        i2sBlockActiveOut = Queue_dequeue(i2sBlockAvailOutQueue);
#ifdef I2S_DEBUG
        i2sBlockActiveOut->queueIndicator = NODE_INDICATOR_ACTIVE;
#endif //I2S_DEBUG
    }
    if ((i2sBlockNextOut == NULL) && i2sBlockReadyOutQueue) {
        i2sBlockNextOut = Queue_dequeue(i2sBlockAvailOutQueue);
#ifdef I2S_DEBUG
        i2sBlockNextOut->queueIndicator = NODE_INDICATOR_NEXT;
#endif //I2S_DEBUG
        // Move at least one buffer to ready queue
        if (!Queue_empty(i2sBlockAvailOutQueue)) {
            queueNodeI2S_t *availNode = Queue_dequeue(i2sBlockAvailOutQueue);
            /* Move buffer to ready queue */
            Queue_enqueue(i2sBlockReadyOutQueue, &availNode->_elem);
#ifdef I2S_DEBUG
            availNode->queueIndicator = NODE_INDICATOR_READY_QUEUE;
#endif //I2S_DEBUG
        }
    }

    /* Configure buffers */
    I2SBufferConfig(hwAttrs->baseAddr,
                    (i2sBlockActiveIn) ? (uint32_t)i2sBlockActiveIn->buf : NULL,
                    (i2sBlockActiveOut) ? (uint32_t)i2sBlockActiveOut->buf : NULL, object->blockSize,
                    I2SCC26XX_DEFAULT_SAMPLE_STAMP_MOD);
    /* Enable the I2S module. This will set first buffer and DMA length */
    I2SEnable(hwAttrs->baseAddr);
    /* Second buffer is then set in hardware after DMA length is set */
    if (i2sBlockReadyInQueue) {
        I2SPointerSet(hwAttrs->baseAddr, true, (uint32_t *)i2sBlockNextIn->buf);
    }
    if (i2sBlockReadyOutQueue) {
        I2SPointerSet(hwAttrs->baseAddr, false, (uint32_t *)i2sBlockNextOut->buf);
    }

    /* Enable the RX overrun interrupt in the I2S module */
    if (i2sBlockReadyInQueue) {
        I2SIntEnable(hwAttrs->baseAddr, I2S_INT_DMA_IN | I2S_INT_PTR_ERR);
    }
    if (i2sBlockReadyOutQueue) {
        I2SIntEnable(hwAttrs->baseAddr, I2S_INT_DMA_OUT | I2S_INT_PTR_ERR | I2S_INT_ALL);
    }

    /* Clear internal pending interrupt flags */
    I2SIntClear(I2S0_BASE, I2S_INT_ALL);

#ifdef I2S_DEBUG
    dbgCntDmaIn = 0;
    dbgCntDmaOut = 0;
    dbgCntI2SReqBuf = 0;
    dbgCntI2SRelBuf = 0;
#endif //I2S_DEBUG

    /* Configuring sample stamp generator will trigger the audio stream to start */
    I2SSampleStampConfigure(hwAttrs->baseAddr, (i2sBlockReadyInQueue) ? true : false, (i2sBlockReadyOutQueue) ? true : false);

    /* Enable samplestamp */
    I2SSampleStampEnable(hwAttrs->baseAddr);

    /* Clear potential pending I2S interrupt to CM3 */
    Hwi_clearInterrupt(INT_I2S_IRQ);

    /* Enable I2S interrupt to CM3 */
    Hwi_enableInterrupt(INT_I2S_IRQ);

    return (true);
}

/*
 *  ======== I2SCC26XX_stopStream ========
 *  @pre    Function assumes that handle is not NULL
 */
bool I2SCC26XX_stopStream(I2SCC26XX_Handle handle) {
    I2SCC26XX_Object                    *object;
    I2SCC26XX_HWAttrs const             *hwAttrs;
    unsigned int                        key;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Check if a transfer is in progress */
    key = Hwi_disable();

    /* Check if there is an active stream */
    if ( (object->currentStream->status == I2SCC26XX_STREAM_STOPPING) ||
         (object->currentStream->status == I2SCC26XX_STREAM_STOPPED) ||
         (object->currentStream->status == I2SCC26XX_STREAM_IDLE) ) {
        Hwi_restore(key);
        return false;
    }

    /* Begin stopping sequence, if not stopped because of error */
    if (object->currentStream->status != I2SCC26XX_STREAM_ERROR) {
        object->currentStream->status = I2SCC26XX_STREAM_STOPPING;

        /* Reenable the interrupt as it may have been disabled during an error*/
        I2SIntEnable(hwAttrs->baseAddr, I2S_INT_PTR_ERR);

        Hwi_restore(key);

        /* Wait for I2S module to complete all buffers*/
        //    if (!Semaphore_pend(Semaphore_handle(&(object->semStopping)), BIOS_WAIT_FOREVER)) {
        if (!Semaphore_pend(Semaphore_handle(&(object->semStopping)), 40000)) {
            object->currentStream->status = I2SCC26XX_STREAM_FAILED_TO_STOP;
            return false;
        }
    }

    /* restore HWI */
    Hwi_restore(key);

    /* Disable the I2S module */
    I2SDisable(hwAttrs->baseAddr);

    /* Disable and clear any pending interrupts */
    I2SIntDisable(hwAttrs->baseAddr, I2S_INT_ALL);
    I2SIntClear(hwAttrs->baseAddr, I2S_INT_ALL);
    Hwi_clearInterrupt(INT_I2S_IRQ);
    Hwi_disableInterrupt(INT_I2S_IRQ);

    /* Flush any unprocessed I2S data */
    I2SCC26XX_cleanUpQueues();

    /* Indicate we are done with this stream */
    object->currentStream->status = I2SCC26XX_STREAM_IDLE;

    Log_print2(Diags_USER2,"I2S:(%p) stream: %p stopped",
                            hwAttrs->baseAddr, (UArg)object->currentStream);

    /* Release constraint after streaming */
    Power_releaseConstraint(PowerCC26XX_SB_DISALLOW);

    /* Stream was successfully stopped */
    return true;
}

/*
 *  ======== I2SCC26XX_requestBuffer ========
 *  @pre    Function assumes that stream has started and that bufferRequest is not NULL
 */
bool I2SCC26XX_requestBuffer(I2SCC26XX_Handle handle, I2SCC26XX_BufferRequest *bufferRequest) {
    I2SCC26XX_Object         *object;
    bool                     retVal = false;
    /* Get the pointer to the object */
    object = handle->object;

#ifdef I2S_DEBUG
    dbgCntI2SReqBuf++;
#endif //I2S_DEBUG

    if (object->requestMode == I2SCC26XX_MODE_BLOCKING) {
        Log_print1(Diags_USER2, "I2S:(%p) request pending on blockComplete "
                                "semaphore",
                ((I2SCC26XX_HWAttrs const *)(handle->hwAttrs))->baseAddr);

        if (!Semaphore_pend(Semaphore_handle(&(object->blockComplete)), object->ui32requestTimeout)) {
            /* Stop stream, if we experience a timeout */
            I2SCC26XX_stopStream(handle);

            bufferRequest->status = object->currentStream->status;

            return false;
        }
    };
    bufferRequest->bufferHandleIn = NULL;
    bufferRequest->bufferHandleOut = NULL;
    /* When in callback mode we typically expect the user to call this
    * after being notified of available buffers. Hence we may directly
    * check queue and dequeue buffer
    */
    if (i2sBlockReadyInQueue) {
        if (!Queue_empty(i2sBlockReadyInQueue)) {
            queueNodeI2S_t *readyNode = Queue_get(i2sBlockReadyInQueue);
            bufferRequest->bufferIn = readyNode->buf;
            bufferRequest->status = object->currentStream->status;
            bufferRequest->bufferHandleIn = readyNode;
#ifdef I2S_DEBUG
            readyNode->queueIndicator = NODE_INDICATOR_USER;
#endif //I2S_DEBUG
            retVal = true;
        }
    }

    if (i2sBlockReadyOutQueue) {
        if (!Queue_empty(i2sBlockReadyOutQueue)) {
            queueNodeI2S_t *readyNode = Queue_get(i2sBlockReadyOutQueue);
            bufferRequest->bufferOut = readyNode->buf;
            bufferRequest->status = object->currentStream->status;
            bufferRequest->bufferHandleOut = readyNode;
#ifdef I2S_DEBUG
            readyNode->queueIndicator = NODE_INDICATOR_USER;
#endif //I2S_DEBUG
            retVal = true;
        }
    }

    return retVal;
}

/*
 *  ======== I2SCC26XX_releaseBuffer ========
 *  @pre    Function assumes bufferRelease contains a valid bufferHandle (identical to
 *          the one provided in the bufferRequest in I2SCC26XX_requestBuffer
 */
void I2SCC26XX_releaseBuffer(I2SCC26XX_Handle handle, I2SCC26XX_BufferRelease *bufferRelease)
{
    unsigned int key;
#ifdef I2S_DEBUG
    dbgCntI2SRelBuf++;
#endif //I2S_DEBUG
    if (bufferRelease->bufferHandleIn) {
        /* Place released buffer back in available queue */
        Queue_put(i2sBlockAvailInQueue, &((queueNodeI2S_t *)bufferRelease->bufferHandleIn)->_elem);
#ifdef I2S_DEBUG
        ((queueNodeI2S_t *)bufferRelease->bufferHandleIn)->queueIndicator = NODE_INDICATOR_AVAIL_QUEUE;
#endif //I2S_DEBUG
    }
    if (bufferRelease->bufferHandleOut) {
        key = Hwi_disable();
        /* Check if we current have the reusable buffer as next, replace it */
        if (i2sBlockNextOut == i2sBlockRepeatOut)
        {
            i2sBlockNextOut = ((queueNodeI2S_t *)bufferRelease->bufferHandleOut);
#ifdef I2S_DEBUG
            ((queueNodeI2S_t *)bufferRelease->bufferHandleOut)->queueIndicator = NODE_INDICATOR_NEXT;
#endif //I2S_DEBUG
        }
        else {
            /* Place released buffer back in available queue */
            Queue_put(i2sBlockAvailOutQueue, &((queueNodeI2S_t *)bufferRelease->bufferHandleOut)->_elem);
#ifdef I2S_DEBUG
            ((queueNodeI2S_t *)bufferRelease->bufferHandleOut)->queueIndicator = NODE_INDICATOR_AVAIL_QUEUE;
#endif //I2S_DEBUG
        }
        Hwi_restore(key);
    }
}

/*
 *  ======== I2SCC26XX_callback ========
 *  Callback function for when the I2S is in I2SCC26XX_MODE_BLOCKING
 *
 *  @pre    Function assumes that the handle is not NULL
 */
static void I2SCC26XX_callback(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *msg) {
    I2SCC26XX_Object         *object;

    Log_print1(Diags_USER2, "I2S DMA:(%p) posting block complete semaphore",
                ((I2SCC26XX_HWAttrs const *)(handle->hwAttrs))->baseAddr);

    /* Get the pointer to the object */
    object = handle->object;

    /* Post the semaphore */
    Semaphore_post(Semaphore_handle(&(object->blockComplete)));
}

/*
*  ======== I2SCC26XX_hwInit ========
*  This functions initializes the I2S hardware module.
*
*  @pre    Function assumes that the I2S handle is pointing to a hardware
*          module which has already been opened.
*/
static void I2SCC26XX_initHw(I2SCC26XX_Handle handle) {
    I2SCC26XX_Object        *object;
    I2SCC26XX_HWAttrs const *hwAttrs;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Disable I2S operation */
    I2SDisable(hwAttrs->baseAddr);

    /* Configure Audio format */
    I2SAudioFormatConfigure(hwAttrs->baseAddr,
                            *(uint32_t *)&object->audioFmtCfg,
                            object->audioFmtCfg.dataDelay);

    /* Configure Channels */
    I2SChannelConfigure(hwAttrs->baseAddr,
                        object->audioPinCfg.driverLibParams.ad0,
                        object->audioPinCfg.driverLibParams.ad1,
                        ~I2SCC26XX_DIR_CHA_M);

    /* Turn on I2S clocks */
//    PRCMPeripheralRunEnable(PRCM_PERIPH_I2S);

    /* Configure Clocks*/
    uint32_t clkCfg = object->audioClkCfg.wclkSource;
    clkCfg |= (object->audioClkCfg.wclkInverted) ? I2S_INVERT_WCLK : 0;
    I2SClockConfigure(hwAttrs->baseAddr, clkCfg);
    uint32_t ui32MstDiv = object->audioClkCfg.mclkDiv;
    uint32_t ui32BitDiv = object->audioClkCfg.bclkDiv;
    uint32_t ui32WordDiv = object->audioClkCfg.wclkDiv;
    clkCfg = (object->audioClkCfg.wclkPhase << PRCM_I2SCLKCTL_WCLK_PHASE_S);
    clkCfg |= (object->audioClkCfg.sampleOnPositiveEdge << PRCM_I2SCLKCTL_SMPL_ON_POSEDGE_S);
    if ( (object->i32SampleRate >= I2S_SAMPLE_RATE_16K) &&
        (object->i32SampleRate <= I2S_SAMPLE_RATE_48K)) {
        PRCMAudioClockConfigSet(clkCfg, object->i32SampleRate);
    }
    else {
        PRCMAudioClockConfigSetOverride(clkCfg, ui32MstDiv, ui32BitDiv, ui32WordDiv);
    }
    // TODO: Replace this with Driverlib code
    HWREG(PRCM_BASE + PRCM_O_I2SBCLKSEL) = (object->audioClkCfg.bclkSource & PRCM_I2SBCLKSEL_SRC_M);
    /* Apply configuration */
    PRCMLoadSet();

    /* Disable I2S module interrupts */
//    uint32_t ui32IntFlags
    I2SIntDisable(hwAttrs->baseAddr, I2S_INT_ALL);

    /* Print the configuration */
    Log_print1(Diags_USER2, "I2S:(%p) Configured", hwAttrs->baseAddr);
}

/*
*  ======== I2SCC26XX_initIO ========
*  This functions initializes the I2S IOs.
*
*  @pre    Function assumes that the I2S handle is pointing to a hardware
*          module which has already been opened.
*/
static bool I2SCC26XX_initIO(I2SCC26XX_Handle handle) {
    I2SCC26XX_Object        *object;
    I2SCC26XX_HWAttrs const *hwAttrs;
    PIN_Config              i2sPinTable[6];
    uint32_t                i=0;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Configure IOs */
    /* Build local list of pins, allocate through PIN driver and map HW ports */
    if (object->audioPinCfg.bitFields.enableMclkPin) {
      i2sPinTable[i++] = hwAttrs->mclkPin | IOC_STD_OUTPUT;
    }
    if (object->audioPinCfg.bitFields.enableWclkPin) {
      i2sPinTable[i++] = hwAttrs->wclkPin | IOC_STD_OUTPUT;
    }
    if (object->audioPinCfg.bitFields.enableBclkPin) {
      i2sPinTable[i++] = hwAttrs->bclkPin | IOC_STD_OUTPUT;
    }
    if (object->audioPinCfg.bitFields.ad0Usage == I2SCC26XX_ADUsageInput) {
      i2sPinTable[i++] = hwAttrs->ad0Pin | PIN_INPUT_EN | PIN_NOPULL;
    }
    else if (object->audioPinCfg.bitFields.ad0Usage == I2SCC26XX_ADUsageOutput) {
      i2sPinTable[i++] = hwAttrs->ad0Pin | IOC_STD_OUTPUT;
    }
    if (object->audioPinCfg.bitFields.ad1Usage == I2SCC26XX_ADUsageInput) {
      i2sPinTable[i++] = hwAttrs->ad1Pin | IOC_STD_INPUT;
    }
    else if (object->audioPinCfg.bitFields.ad1Usage == I2SCC26XX_ADUsageOutput) {
      i2sPinTable[i++] = hwAttrs->ad1Pin | IOC_STD_OUTPUT;
    }
    i2sPinTable[i++] = PIN_TERMINATE;

    /* Open and assign pins through pin driver */
    if (!(object->pinHandle = PIN_open(&(object->pinState), i2sPinTable))) {
      return false;
    }

    /* Set IO muxing for the I2S pins */

    if (object->audioPinCfg.bitFields.enableMclkPin) {
      PINCC26XX_setMux(object->pinHandle, hwAttrs->mclkPin,  IOC_PORT_MCU_I2S_MCLK);
    }
    if (object->audioPinCfg.bitFields.enableWclkPin) {
      PINCC26XX_setMux(object->pinHandle, hwAttrs->wclkPin,  IOC_PORT_MCU_I2S_WCLK);
    }
    if (object->audioPinCfg.bitFields.enableBclkPin) {
      PINCC26XX_setMux(object->pinHandle, hwAttrs->bclkPin,  IOC_PORT_MCU_I2S_BCLK);
    }
    if (object->audioPinCfg.bitFields.ad0Usage != I2SCC26XX_ADUsageDisabled) {
      PINCC26XX_setMux(object->pinHandle, hwAttrs->ad0Pin,  IOC_PORT_MCU_I2S_AD0);
    }
    if (object->audioPinCfg.bitFields.ad1Usage != I2SCC26XX_ADUsageDisabled) {
      PINCC26XX_setMux(object->pinHandle, hwAttrs->ad1Pin,  IOC_PORT_MCU_I2S_AD1);
    }

    return true;
}
/*
*  ======== I2SCC26XX_cleanUpQueues ========
*  This functions cleans up the queues.
*
*  @pre    Function assumes that the I2S driver has been opened.
*/
static void I2SCC26XX_cleanUpQueues() {
    if (i2sBlockReadyInQueue)
    {
        /* Clean up unused queue elements */
        while (!Queue_empty(i2sBlockReadyInQueue)) {
            queueNodeI2S_t *readyNode = Queue_dequeue(i2sBlockReadyInQueue);
            /* Move buffer to avail queue */
            Queue_enqueue(i2sBlockAvailInQueue, &readyNode->_elem);
#ifdef I2S_DEBUG
            readyNode->queueIndicator = NODE_INDICATOR_AVAIL_QUEUE;
#endif //I2S_DEBUG
        }
    }
    if (i2sBlockReadyOutQueue) {
        /* Clean up unused queue elements */
        while (!Queue_empty(i2sBlockReadyOutQueue)) {
            queueNodeI2S_t *readyNode = Queue_dequeue(i2sBlockReadyOutQueue);
            /* Move buffer to avail queue */
            Queue_enqueue(i2sBlockAvailOutQueue, &readyNode->_elem);
#ifdef I2S_DEBUG
            readyNode->queueIndicator = NODE_INDICATOR_AVAIL_QUEUE;
#endif //I2S_DEBUG
        }
    }
}

/*
 *  ======== i2sPostNotify ========
 *  This functions is called to notify the I2S driver of an ongoing transition
 *  out of sleep mode.
 *
 *  @pre    Function assumes that the I2S handle (clientArg) is pointing to a
 *          hardware module which has already been opened.
 */
int i2sPostNotify(char eventType, uint32_t clientArg) {
    I2SCC26XX_Handle i2sHandle;

    /* Get the pointers to I2S objects */
    i2sHandle = (I2SCC26XX_Handle) clientArg;

    /* Reconfigure the hardware when returning from standby */
    I2SCC26XX_initHw(i2sHandle);

    return Power_NOTIFYDONE;
}
