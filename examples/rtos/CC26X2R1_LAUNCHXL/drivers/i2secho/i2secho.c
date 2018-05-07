/*
 * Copyright (c) 2015-2018, Texas Instruments Incorporated
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

/*
 *  ======== i2secho.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* Driver Header files */
#include <ti/drivers/i2s/I2SCC26XX.h>
#include <ti/display/Display.h>
#include <ti/audiocodec/audiocodec.h>

#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/BIOS.h>

/* Example/Board Header files */
#include "Board.h"

/* I2S Constants */
#define SAMPLE_RATE                     16000   // Warning: Only 16kHz supported
#define INPUT_OPTION                    AUDIO_CODEC_MIC_LINE_IN
#define OUTPUT_OPTION                   AUDIO_CODEC_SPEAKER_HP
#define NUM_CHAN                        2

/* Configure for a 20ms frame (20ms/16kHz) */
#define FRAME_SIZE                      320

#define I2S_TOTAL_QUEUE_MEM_SZ         (I2S_BLOCK_OVERHEAD_IN_BYTES *           \
                                        I2SCC26XX_QUEUE_SIZE *                  \
                                        NUM_CHAN)

#define I2S_SAMPLE_MEMORY_SZ           (FRAME_SIZE *                            \
                                        I2SCC26XX_QUEUE_SIZE *                  \
                                        NUM_CHAN)

/* Application defines */
#define I2S_FRAME_EVENT                 Event_Id_00
#define I2S_ERROR_EVENT                 Event_Id_01
#define I2S_NO_MEM                      Event_Id_02
#define ALL_EVENTS                      (I2S_FRAME_EVENT | I2S_ERROR_EVENT)

/* Forward Declarations */
static void i2sCallbackFxn(I2SCC26XX_Handle handle,
                            I2SCC26XX_StreamNotification *notification);

/* I2S Variables */
static I2SCC26XX_Handle i2sHandle = NULL;
static I2SCC26XX_StreamNotification i2sStream;
static uint8_t  i2sQueueMemory[I2S_TOTAL_QUEUE_MEM_SZ];
static uint16_t i2sSampleBuffer[I2S_SAMPLE_MEMORY_SZ];

static I2SCC26XX_Params i2sParams =
{
    .requestMode            = I2SCC26XX_CALLBACK_MODE,
    .ui32requestTimeout     = BIOS_WAIT_FOREVER,
    .callbackFxn            = i2sCallbackFxn,
    .blockSize              = FRAME_SIZE,
    .pvContBuffer           = (void *)i2sSampleBuffer,
    .ui32conBufTotalSize    = (sizeof(int16_t) * I2S_SAMPLE_MEMORY_SZ),
    .pvContMgtBuffer        = (void *)i2sQueueMemory,
    .ui32conMgtBufTotalSize = I2S_TOTAL_QUEUE_MEM_SZ,
    .currentStream          = &i2sStream
};

/* RTOS event */
Event_Handle i2sTaskEvent;

/* Handle for display */
Display_Handle dispHandle = NULL;

/*
 *  ======== i2sCallbackFxn ========
 */
static void i2sCallbackFxn(I2SCC26XX_Handle handle,
                            I2SCC26XX_StreamNotification *notification)
{
    if (notification->status == I2SCC26XX_STREAM_ERROR)
    {
        /* I2S_ERROR_EVENT */
        Event_post(i2sTaskEvent, I2S_ERROR_EVENT);
    }
    else if (notification->status == I2SCC26XX_STREAM_BUFFER_READY)
    {
        /* I2S_FRAME_EVENT */
        Event_post(i2sTaskEvent, I2S_FRAME_EVENT);
    }
    else if (notification->status ==                                            \
                I2SCC26XX_STREAM_BUFFER_READY_BUT_NO_AVAILABLE_BUFFERS)
    {
        /* I2S_FRAME_EVENT */
        Event_post(i2sTaskEvent, I2S_NO_MEM);
    }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    uint32_t events;
    uint8_t status;

    /* Create event */
    i2sTaskEvent = Event_create(NULL, NULL);

    /* Open Display Interface */
    dispHandle = Display_open(Display_Type_UART, NULL);
    Display_printf(dispHandle, 0, 0, "\f");
    Display_printf(dispHandle, 0,0, "I2S Echo App");
    Display_printf(dispHandle, 1,0, "Sample rate: %d", SAMPLE_RATE);
    Display_printf(dispHandle, 2,0, "Samples/Frame: %d", FRAME_SIZE);
    Display_printf(dispHandle, 3,0, "Audio input: line in");
    Display_printf(dispHandle, 4,0, "Audio output: line out");

    /* Initialize TLV320AIC3254 Codec on Audio BP */
    status = AudioCodecOpen();

    if( AUDIO_CODEC_STATUS_SUCCESS != status)
    {
        Display_printf(dispHandle, 3,0, "Can't open codec, status: %d", status);
    }

    /* Configure Codec */
    status =  AudioCodecConfig(AUDIO_CODEC_TI_3254, AUDIO_CODEC_16_BIT,
                                SAMPLE_RATE, AUDIO_CODEC_STEREO, OUTPUT_OPTION,
                                INPUT_OPTION);

    if( AUDIO_CODEC_STATUS_SUCCESS != status)
    {
        Display_printf(dispHandle, 3,0, "Can't open codec, status: %d", status);
    }

    AudioCodecSpeakerVolCtrl(AUDIO_CODEC_TI_3254, OUTPUT_OPTION, 75);
    /* Volume control */
    AudioCodecMicVolCtrl(AUDIO_CODEC_TI_3254, INPUT_OPTION, 75);

    /* Initialize I2S driver */
    i2sHandle = (I2SCC26XX_Handle)&(I2SCC26XX_config);
    I2SCC26XX_init(i2sHandle);
    I2SCC26XX_Handle i2sHandleTmp = NULL;
    i2sHandleTmp = I2SCC26XX_open(i2sHandle, &i2sParams);
    if(i2sHandleTmp == NULL)
    {
        Display_printf(dispHandle, 3,0,
                        "Can't open I2S, check memory allocation");
    }

    /* Start Streaming */
    I2SCC26XX_startStream(i2sHandle);

    for (;;)
    {
        events = Event_pend(i2sTaskEvent, Event_Id_NONE,
                            I2S_FRAME_EVENT+I2S_ERROR_EVENT,
                            BIOS_WAIT_FOREVER);

        /* Check for Events */
        if(events)
        {
            if(events & I2S_ERROR_EVENT)
            {
                /* Stop the stream if an error occurred */
                I2SCC26XX_stopStream(i2sHandle);

                /* Close the I2C interface to the codec */
                AudioCodecClose();

                Display_printf(dispHandle, 3,0, "I2S Error");
            }

            if(events & I2S_NO_MEM)
            {
                /* Stop the stream if an error occurred */
                I2SCC26XX_stopStream(i2sHandle);

                /* Close the I2C interface to the codec */
                AudioCodecClose();

                Display_printf(dispHandle, 3,0, "I2S out of memory");
            }

            if(events & I2S_FRAME_EVENT)
            {
                /* Since we are looping back, request input and output buffer */
                I2SCC26XX_BufferRequest bufferRequest;
                I2SCC26XX_BufferRelease bufferRelease;
                bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN_AND_OUT;

                bool gotBuffer = I2SCC26XX_requestBuffer(i2sHandle,
                                                            &bufferRequest);

                /* If we received a buffer, pass to output stream */
                if(gotBuffer)
                {
                    /* Do frame specific processing here */

                    /* Copy the frame directly to the output buffer */
                    memcpy(bufferRequest.bufferOut, bufferRequest.bufferIn,
                           FRAME_SIZE);
                }

                /* Release the buffer back to the I2S driver */
                bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
                bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
                I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);

            }
        }
    }
}
