/******************************************************************************

@file  audio.c

@brief This file contains both the AMIC and DMIC main functionalities.

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

#ifdef USE_AMIC

#include <string.h>
#include "audio.h"
#include <ti/drivers/SPI.h>
#include <ti/drivers/utils/List.h>
#include "ti_drivers_config.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/ADCBuf.h>
#include "Audio/Filter/digital_filter.h"

#ifdef DEBUG_PRINT_OUT
#include <ti/drivers/UART2.h>
#endif //DEBUG_PRINT_OUT

#ifdef DEBUG_PRINT_OUT
UART2_Handle uart;
#endif //DEBUG_PRINT_OUT

#ifdef DEBUG_PRINT_OUT
void uartCallbackFxn(UART2_Handle handle, void *buffer, size_t count, void *userArg, int_fast16_t status)
{
    // Note: The callback could be modified to handle a "queue" of UART operations
}
#endif //DEBUG_PRINT_OUT


#ifdef USE_ADPCM_CODEC
    #include <adpcm/Codec1.h>
    #ifndef OVERSAMPLING
        #define ADCSAMPLESIZE           (32)
    #endif
    #ifdef OVERSAMPLING
        #define OVERSAMPLING_FACTOR      8
        #define ADCSAMPLESIZE           (32*OVERSAMPLING_FACTOR)
    #endif
#else
#ifdef USE_MSBC_CODEC
    #include <sbc/Codec2.h>
    #define ADCSAMPLESIZE           (120)
#endif
#endif

#ifdef USE_MSBC_CODEC
int16_t pcmData[144];
uint8_t pcmDataIndex = 0;
#endif

uint16_t sampleBufferOne[ADCSAMPLESIZE];
uint16_t sampleBufferTwo[ADCSAMPLESIZE];
int16_t fullBuffer[ADCSAMPLESIZE];

#ifdef USE_ANALOG_FILTER
int16_t finalbuffer[32];
int16_t filterdecimatedBuffer[32];
#endif

#ifdef USE_ADPCM_CODEC
// The ADPCM buffers hold compressed PCM data (compression ratio is 4:1)
static uint8_t sequenceNumber = 0;
int8_t            si = 0;
int16_t           pv = 0;
#endif

ADCBuf_Handle adcBuf;
ADCBuf_Params adcBufParams;
ADCBuf_Conversion continuousConversion;
extern uint8_t bleStreamIsEnabled;

extern void Application_DoAudioEvent();

void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion, void *completedADCBuffer, uint32_t completedChannel, int_fast16_t status)
{
    memcpy(fullBuffer, completedADCBuffer, ADCSAMPLESIZE*sizeof(int16_t));
    BLEAppUtil_invokeFunctionNoData(Application_DoAudioEvent);
}

uint8_t Audio_init(void)
{
    GPIO_init();
    ADCBuf_init();
    #ifdef USE_ADPCM_CODEC
    // Do nothing
    #else
    #ifdef USE_MSBC_CODEC
    Codec2_init();
    #endif
    #endif

#ifdef DEBUG_PRINT_OUT
    #ifndef Display_DISABLE_ALL
        #error "Resource conflict! In order to printout PCM or PDM data, the BLEStack should not use the Display module should be disabled u"
    #endif
    /* Create a UART in CALLBACK write mode */
    UART2_Params uartParams;
    UART2_Params_init(&uartParams);
    uartParams.writeMode     = UART2_Mode_NONBLOCKING;
    uartParams.writeCallback = uartCallbackFxn;
    uartParams.baudRate      = 1000000;

    uart = UART2_open(CONFIG_DISPLAY_UART, &uartParams);
#endif //DEBUG_PRINT_OUT

    return 0;
}

uint8_t Audio_enable(void)
{
    uint8_t retVal = 0;
    #ifdef USE_ADPCM_CODEC
    // Do nothing
    #else
    #ifdef USE_MSBC_CODEC
    Codec2_init();
    #endif
    #endif

    GPIO_write(CONFIG_GPIO_MIC,1);

    //Configure & Open ADC Buffer driver
    adcBufParams.callbackFxn = adcBufCallback;
    adcBufParams.recurrenceMode = ADCBuf_RECURRENCE_MODE_CONTINUOUS;
    adcBufParams.returnMode = ADCBuf_RETURN_MODE_CALLBACK;
    #ifndef OVERSAMPLING
    adcBufParams.samplingFrequency = 16000;
    #endif
    #ifdef OVERSAMPLING
    // Oversampling with a Factor (k) = 8 - Theoretical SNR improvement of 9 dB and 1.5 Extra bit of resolution.
    adcBufParams.samplingFrequency = 16000 * OVERSAMPLING_FACTOR; // Oversampling frequency = Nyq freq * 4 = 16 * 4 = 64 Hz
    #endif

    adcBuf = ADCBuf_open(CONFIG_ADCBUF_0, &adcBufParams);

    //GPIO_setConfig(CONFIG_ADCBUF_0, GPIO_CFG_PULL_DOWN_INTERNAL);

    //Configure the conversion struct
    continuousConversion.arg = NULL;
    continuousConversion.adcChannel = CONFIG_ADCBUF_0;
    continuousConversion.sampleBuffer = sampleBufferOne;
    continuousConversion.sampleBufferTwo = sampleBufferTwo;
    continuousConversion.samplesRequestedCount = ADCSAMPLESIZE;

    if (ADCBuf_convert(adcBuf, &continuousConversion, 1) != 0) {
            uint8_t retVal = 1;
    }

    return retVal;
}

uint8_t Audio_disable(void)
{
    uint8_t retVal = 0;
    //if (bleStreamIsEnabled == 1){
        /* Stop converting. */
        if (ADCBuf_convertCancel(adcBuf) != 0) {
            uint8_t retVal = 1;
        }
        ADCBuf_close(adcBuf);
    //}
    GPIO_write(CONFIG_GPIO_MIC,0);

    return retVal;
}

#ifdef OVERSAMPLING

#ifdef USE_ANALOG_FILTER

#define ORDER 4
int32_t integrator[ORDER] = {0};
int32_t comb[ORDER] = {0};
int16_t decimated_dcnotch_buffer[32];

uint8_t Audio_Decimate(int16_t* input_buffer, int16_t* output_buffer){

    int32_t decimated_counter = 0;

    #ifdef USE_AVERAGE_FILTER
    uint8_t decimated_counter = 0;
    for(int i=0; i<=ADCSAMPLESIZE-OVERSAMPLING_FACTOR; i=i+OVERSAMPLING_FACTOR){
        buffer[decimated_counter] = ((buffer[i] + buffer[i+1] + buffer[i+2] + buffer[i+3] + buffer[i+4] + buffer[i+5] + buffer[i+6] + buffer[i+7])/OVERSAMPLING_FACTOR);
        decimated_counter++;
    }
    #endif

    #ifdef USE_CIC_FILTER
    uint8_t count = 0;
    //Scale Factor = OVERSAMPLING_FACTOR ^ ORDER
    int64_t scale_factor = 4096;
    //int64_t scale_factor = 65536;
    for (int i = 0; i<ADCSAMPLESIZE; i++) {

       int32_t input_sample = (int32_t)input_buffer[i];

        // Integrator stage
        integrator[0] += input_sample;
        for (int j = 1; j < ORDER; j++) {
             integrator[j] += integrator[j - 1];
        }

        // Decimation by OVERSAMPLING_FACTOR
        count++;
        if (count < OVERSAMPLING_FACTOR) {
        // Jump to next iteration
           continue;
        }

        count = 0;
        // Comb stage
        int64_t temp = integrator[ORDER - 1];
        for (int j = 0; j < ORDER; j++) {
            int64_t comb_input = comb[j];
            comb[j] = temp;
            temp -= comb_input;
        }

        // Store the output sample
        output_buffer[decimated_counter++] = (int16_t)(temp/scale_factor);
     }
    #endif

    return decimated_counter;
}
#endif

#endif

uint8_t Audio_getCompressedBuffer(uint8_t* encBuf)
{
    uint8_t dataLength = AUDIO_FAIL_NO_DATA;
    uint8_t decimated_size = 0;

    //Using Digital Filter
    #ifdef USE_ANALOG_FILTER

    #ifdef OVERSAMPLING
    //DC_notch(fullBuffer, ADCSAMPLESIZE);
    decimated_size = Audio_Decimate(fullBuffer, decimated_dcnotch_buffer);
    #else
    decimated_size = ADCSAMPLESIZE;
    #endif

    //applyIIRFilter(decimated_dcnotch_buffer, finalbuffer, decimated_size);
    //applyIIRFilter2(filterdecimatedBuffer, finalbuffer, decimated_size);

    #ifdef DEBUG_PRINT_OUT
        static int8_t uart_buf2[32*sizeof(int16_t)];
        memcpy(uart_buf2, decimated_dcnotch_buffer, 32*sizeof(int16_t));
        UART2_write(uart, uart_buf2, sizeof(uart_buf2), NULL);
    #endif

    #ifdef USE_ADPCM_CODEC
    // ADPCM Encode
    #ifdef OVERSAMPLING
    dataLength = Codec1_encodeBuff(encBuf, decimated_dcnotch_buffer, decimated_size, &si, &pv);
    #endif
    #ifndef OVERSAMPLING
    dataLength = Codec1_encodeBuff(encBuf, fullBuffer, decimated_size, &si, &pv);
    #endif

    #endif
    #ifdef USE_MSBC_CODEC
    // MSBC Encode
    dataLength = Codec2_encodeBuff(encBuf, fullBuffer, AUDIO_ENC_FRAME_SIZE, AUDIO_PCM_FRAME_SIZE);
    #endif

    #endif

    //Without using Digital Filter
    #ifndef USE_ANALOG_FILTER
    #ifdef USE_ADPCM_CODEC
    // ADPCM Encode
    dataLength = Codec1_encodeBuff(encBuf, fullBuffer, ADCSAMPLESIZE, &si, &pv);
    #endif
    #ifdef USE_MSBC_CODEC
    dataLength = Codec2_encodeBuff(encBuf, fullBuffer, AUDIO_ENC_FRAME_SIZE, AUDIO_PCM_FRAME_SIZE);
    #endif

    #endif

    return dataLength;
}
#endif

#ifdef USE_DMIC

#include "audio.h"
#include <ti/drivers/SPI.h>
#include <ti/drivers/utils/List.h>
#include "ti_drivers_config.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/drivers/GPIO.h>
#ifdef USE_ADPCM_CODEC
#include <adpcm/Codec1.h>
#else
#ifdef USE_MSBC_CODEC
#include <sbc/Codec2.h>
#endif
#endif

#define SPI_NUMBUFS 6
#ifdef USE_BW_24KHz
#define SPI_BUFSIZE 256*3
#define PCM_BUF_SIZE 32*3
#else
#define SPI_BUFSIZE 256
#define PCM_BUF_SIZE 32
#endif



List_List pdmFilledSpiTransactionsList;

// SPI buffers hold PDM data (1-bit, 1 MHz)
uint8_t buf1[SPI_BUFSIZE];
uint8_t buf2[SPI_BUFSIZE];
uint8_t buf3[SPI_BUFSIZE];
uint8_t buf4[SPI_BUFSIZE];
uint8_t buf5[SPI_BUFSIZE];
uint8_t buf6[SPI_BUFSIZE];

uint8_t* spiBufTable[SPI_NUMBUFS] = {buf1, buf2, buf3, buf4, buf5, buf6};

SPI_Transaction spiTransaction1;
SPI_Transaction spiTransaction2;
SPI_Transaction spiTransaction3;
SPI_Transaction spiTransaction4;
SPI_Transaction spiTransaction5;
SPI_Transaction spiTransaction6;
SPI_Transaction* spiTransactionTable[SPI_NUMBUFS] = {&spiTransaction1, &spiTransaction2, &spiTransaction3, &spiTransaction4, &spiTransaction5, &spiTransaction6};

SPI_Handle spiHandle;

extern void Application_DoAudioEvent();

extern uint8_t bleStreamIsEnabled;

// This is for the decimation filter (decimation from PDM data to PCM data)
#define FILTER_STATE_SIZE 16
int32_t filterState[FILTER_STATE_SIZE + 4];

// Example biquad coefficient definition
const int32_t pBqCoeffs[] = {
    // Halfband, DC notch, CIC droop compensation (2*Fs)
    689,   121,  1024,  -209,   919,    // First stage of correction filter
    407,   765,  1024,  -376,   470,    // Second stage of correction filter
    392,     0, -1024, -1265,   245,    // Third stage of correction filter
    450,  1345,  1024,  -495,   269,    // Fourth stage of correction filter
    507,   400,  1024,  -296,   690,    // Fifth stage of correction filter
    554,   209,  1024,  -240,   870,    // Sixth stage of correction filter
    0,                                  // Terminate correction filter
    // No EQ filter implemented (Fs)
    0                                   // Terminate EQ filter and coefficient list
};
#ifdef USE_ADPCM_CODEC
// The ADPCM buffers hold compressed PCM data (compression ratio is 4:1)
static uint8_t sequenceNumber = 0;
int8_t            si = 0;
int16_t           pv = 0;
// The PCM buffers hold PCM data (16-bit, 16 kHz)
int16_t pcmData[PCM_BUF_SIZE];
uint8_t pcmDataIndex = 0;
#endif

#ifdef USE_MSBC_CODEC
#ifdef USE_BW_24KHz
//int16_t pcmData[432];
int16_t pcmData[144];
#else
int16_t pcmData[144];
#endif
uint16_t pcmDataIndex = 0;
#endif

//extern uint8_t SimpleSerialSocketServer_postApplicationEvent(uint8_t event, uint8_t *pData, uint16_t size);
extern bool pdm2pcm16k(const void* pIn, int32_t* pState, int32_t* pBqCoeffs, int16_t* pOut);
void spiCallbackFxn(SPI_Handle handle, SPI_Transaction *transaction)
{
    if(NULL != transaction)
    {
            List_put(&pdmFilledSpiTransactionsList, (List_Elem*)transaction);
            /* Trigger the treatment of the data */
            BLEAppUtil_invokeFunctionNoData(Application_DoAudioEvent);
    }
}

uint8_t Audio_init(void)
{
    #ifdef USE_ADPCM_CODEC
    // Do nothing
    #else
    #ifdef USE_MSBC_CODEC
        Codec2_init();
    #endif
    #endif

    SPI_init();

    return 0;
}

uint8_t Audio_enable(void)
{
    SPI_Params spiParams;
    /* Open SPI as master (default) */
    SPI_Params_init(&spiParams);
    spiParams.frameFormat = SPI_POL1_PHA1;
    #ifdef USE_BW_24KHz
    spiParams.bitRate = 3000000; //1024000;
    #else
    spiParams.bitRate = 1024000;
    #endif
    spiParams.transferMode = SPI_MODE_CALLBACK;
    spiParams.transferCallbackFxn = spiCallbackFxn;
    spiHandle = SPI_open(CONFIG_SPI_0, &spiParams);

    GPIO_write(CONFIG_GPIO_MIC,1);
    uint8_t retVal = AUDIO_FAIL_NOT_OPENED;
    if (bleStreamIsEnabled == 1){
        GPIO_write(CONFIG_GPIO_LED_GREEN,1);
        if(spiHandle != 0)
        {
            List_clearList(&pdmFilledSpiTransactionsList);

            /* Prepare the SPI transactions */
            int k;
            for(k = 0; k < SPI_NUMBUFS; k++) {
                spiTransactionTable[k]->count = SPI_BUFSIZE;
                spiTransactionTable[k]->txBuf = (void *) NULL;
                spiTransactionTable[k]->rxBuf = (void *) spiBufTable[k];
            }

            /* State Struct: zero-initialize before use */
            int j = 0;
            for(j = 0; j < FILTER_STATE_SIZE; j++)
            {
                filterState[j] = 0;
            }

            /* Start SPI */
            for(k = 0; k < SPI_NUMBUFS; k++)
            {
               SPI_transfer(spiHandle, spiTransactionTable[k]);
            }

            retVal = AUDIO_SUCCESS;
        }
    }

    return retVal;
}

uint8_t Audio_disable(void)
{
    if (bleStreamIsEnabled == 1){
        SPI_transferCancel(spiHandle);
        SPI_close(spiHandle);
    }
    GPIO_write(CONFIG_GPIO_MIC,0);
    return 0;
}

extern uint8_t stop_audio;

uint8_t Audio_getCompressedBuffer(uint8_t* encBuf)
{
    SPI_Transaction* transactionToTreat = (SPI_Transaction*) List_head(&pdmFilledSpiTransactionsList);
    uint16_t dataLength = AUDIO_FAIL_NO_DATA;

    if(transactionToTreat != NULL)
    {
        dataLength = 0;
        // Decimation
        pdm2pcm16k(transactionToTreat->rxBuf, filterState, pBqCoeffs, &pcmData[pcmDataIndex]);
        pcmDataIndex += PCM_BUF_SIZE;

        if(pcmDataIndex >= AUDIO_PCM_FRAME_SIZE)
        {
            #ifdef USE_ADPCM_CODEC
            // ADPCM Encode
            dataLength = Codec1_encodeBuff(encBuf, pcmData, PCM_BUF_SIZE, &si, &pv);
            #else
            #ifdef USE_MSBC_CODEC
            // mSBC Encode
            #ifdef USE_BW_24KHz
            dataLength = Codec2_encodeBuff(encBuf, pcmData, AUDIO_ENC_FRAME_SIZE, AUDIO_PCM_FRAME_SIZE);
            #else
            dataLength = Codec2_encodeBuff(encBuf, pcmData, AUDIO_ENC_FRAME_SIZE, AUDIO_PCM_FRAME_SIZE);
            #endif

            #endif
            #endif

            // Move the PCM data not used in the last decimation to the begining of the buffer
            #ifdef USE_BW_24KHz
            memcpy(&pcmData[0], &pcmData[AUDIO_PCM_FRAME_SIZE], (pcmDataIndex - AUDIO_PCM_FRAME_SIZE)*sizeof(int16_t));
            pcmDataIndex -= AUDIO_PCM_FRAME_SIZE;
            #else
            memcpy(&pcmData[0], &pcmData[AUDIO_PCM_FRAME_SIZE], (pcmDataIndex - AUDIO_PCM_FRAME_SIZE)*sizeof(int16_t));
            pcmDataIndex -= AUDIO_PCM_FRAME_SIZE;
            #endif
        }

        /* Trigger next sampling */
        List_remove(&pdmFilledSpiTransactionsList, (List_Elem*)transactionToTreat);
        transactionToTreat->count = SPI_BUFSIZE;
        if (stop_audio == 0){
            SPI_transfer(spiHandle, transactionToTreat);
        }
    }

    return dataLength;
}
#endif
