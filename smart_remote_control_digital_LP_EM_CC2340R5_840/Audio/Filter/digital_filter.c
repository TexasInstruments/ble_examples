/******************************************************************************

@file  digital_filter.c

@brief This file contains the digital filtering functions for both AMIC and DMIC functionalities.

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

#include <stdint.h>
#include <stdio.h>

#ifdef USE_ANALOG_FILTER

#define R_MUL 0x7FFE // 32766 (1-2^(-15)) Q32:1.31
#define R_DIV 15 // 32767 (2^15 - shift)

static int16_t x_prev=0;
static int16_t y_prev=0;

// H(z) = (1 - z-1)/(1-Rz-1)
// Equivalent difference equation: y[n] = (x[n] - x[n-1]) + R*y[n-1]

void DC_notch(int16_t *pcmIn, int16_t size){

    for (int i=0;i<size;i++) {
        int16_t sampleIn, delta_x, sampleOut;
        int16_t Ry_prev;

        sampleIn = (int16_t)*pcmIn;

        // Calculate x[n] - x[n-1]
        delta_x = sampleIn - x_prev;

        // Calculate R * y[n-1] using fixed-point multiplication (A1 is Q31, y_prev is integer)
        Ry_prev = (int16_t)(((int32_t)R_MUL * y_prev) >> R_DIV);

        // Calculate y[n] = (x[n] - x[n-1]) + R*y[n-1]
        sampleOut = delta_x + Ry_prev;

        // Update state variables
        x_prev = sampleIn;
        y_prev = (int16_t)sampleOut;

        // Update output
        *pcmIn = (int16_t)sampleOut;

        pcmIn++;
    }
}

#include <stdint.h>

// Filter coefficients - Lowpass
// This is the number of delays for a second-order section
#define BIQUAD_ORDER 2

const int8_t q = 14; // Scaling factor in bits

// Filter coefficients (scaled by 2^q)
// These should be the quantized coefficients from your Python script
const int16_t b_stage1[] = {5682, 11365, 5682}; // b0, b1, b2 for stage 1
const int16_t a_stage1[] = {16384, 14015, 3436}; // a0, a1, a2 for stage 1 (a0 is 16384 = 2^14)
const int16_t b_stage2[] = {16384, 32768, 16384}; // b0, b1, b2 for stage 2
const int16_t a_stage2[] = {16384, 18236, 9405}; // a0, a1, a2 for stage 2 (a0 is 16384 = 2^14)

// State variables for each biquad stage (Direct Form II Transposed)
// Size is BIQUAD_ORDER (2) for a second-order section
static int32_t w1[BIQUAD_ORDER] = {0}; // State variables for stage 1
static int32_t w2[BIQUAD_ORDER] = {0}; // State variables for stage 2

//See function and graph: https://www.advsolned.com/difference-between-iir-and-fir-filters-a-practical-design-guide/

// Function to apply IIR filter (two cascaded biquad stages - Direct Form II Transposed)
void applyIIRFilter(int16_t *input, int16_t *output, int length) {

    for (int i = 0; i < length; i++) {

        int32_t current_input = (int32_t)input[i];

        // --- Stage 1 (Direct Form II Transposed) ---
        // y[n] = b0*w[n] + b1*w[n-1] + b2*w[n-2]
        // w[n] = x[n] - a1*w[n-1] - a2*w[n-2] / a0 (since a0=1 in scaled form, no division needed)

        int32_t stage1_output = 0;
        int32_t w1_n = 0; // w[n] for stage 1

        // Calculate w[n] = x[n] - a1*w[n-1] - a2*w[n-2]
        // Note: a0 is assumed to be 2^q (16384), so the division by a0 is handled by the scaling (q).
        // We use the scaled coefficients directly in the multiplications.
        w1_n = current_input;
        w1_n -= (int32_t)a_stage1[1] * w1[0] >> q; // - a1 * w[n-1]
        w1_n -= (int32_t)a_stage1[2] * w1[1] >> q; // - a2 * w[n-2]

        // Calculate y[n] = b0*w[n] + b1*w[n-1] + b2*w[n-2]
        stage1_output = (int32_t)b_stage1[0] * w1_n >> q; // + b0 * w[n]
        stage1_output += (int32_t)b_stage1[1] * w1[0] >> q; // + b1 * w[n-1]
        stage1_output += (int32_t)b_stage1[2] * w1[1] >> q; // + b2 * w[n-2]

        // Update state variables for stage 1 (shift old values)
        w1[1] = w1[0]; // w[n-2] = w[n-1]
        w1[0] = w1_n;  // w[n-1] = w[n]


        // --- Stage 2 (Direct Form II Transposed) ---
        // Input to stage 2 is the output of stage 1
        int32_t w2_n = 0; // w[n] for stage 2

        // Calculate w[n] = x[n] - a1*w[n-1] - a2*w[n-2]
        w2_n = stage1_output; // Input is stage1_output
        w2_n -= (int32_t)a_stage2[1] * w2[0] >> q; // - a1 * w[n-1]
        w2_n -= (int32_t)a_stage2[2] * w2[1] >> q; // - a2 * w[n-2]

        // Calculate y[n] = b0*w[n] + b1*w[n-1] + b2*w[n-2]
        int32_t stage2_output = 0;
        stage2_output = (int32_t)b_stage2[0] * w2_n >> q; // + b0 * w[n]
        stage2_output += (int32_t)b_stage2[1] * w2[0] >> q; // + b1 * w[n-1]
        stage2_output += (int32_t)b_stage2[2] * w2[1] >> q; // + b2 * w[n-2]

        // Update state variables for stage 2 (shift old values)
        w2[1] = w2[0]; // w[n-2] = w[n-1]
        w2[0] = w2_n;  // w[n-1] = w[n]


        // --- Saturation and Final Output ---

        const int32_t satVal_pos = 32767;
        const int32_t satVal_neg = -32768; // Correct minimum value for int16_t

        int32_t final_output = stage2_output;

        // Saturate the final output to valid 16b range
        if (final_output > satVal_pos) {
            final_output = satVal_pos;
        } else if (final_output < satVal_neg) {
            final_output = satVal_neg;
        }

        // Store the final 16-bit output
        output[i] = (int16_t)final_output;
    }
}

/*#include <stdint.h>

// Filter coefficients - Lowpass
#define FILTER_ORDER 2
const int8_t q = 14;
// Filter coefficients (scaled by 2^q)
// Assuming these are coefficients for two cascaded stages
const int16_t b_stage1[] = {12671, 25341, 12671}; // MATLAB coefficients for stage 1 numerator
const int16_t a_stage1[] = {16384, 27230, 11380}; // MATLAB coefficients for stage 1 denominator (a[0] is 16384 = 2^14)
const int16_t b_stage2[] = {16384, 32768, 16384}; // MATLAB coefficients for stage 2 numerator (b[0] = 16384 = 2^14)
const int16_t a_stage2[] = {16384, 29906, 14108}; // MATLAB coefficients for stage 2 denominator (a[0] is 16384 = 2^14)

// State variables for each stage
static int32_t x_history1[FILTER_ORDER + 1] = {0}; // Input history for stage 1
static int32_t y_history1[FILTER_ORDER + 1] = {0}; // Output history for stage 1

static int32_t x_history2[FILTER_ORDER + 1] = {0}; // Input history for stage 2
static int32_t y_history2[FILTER_ORDER + 1] = {0}; // Output history for stage 2


// Function to apply IIR filter (two cascaded stages)
void applyIIRFilter(int16_t *input, int16_t *output, int length) {

    for (int i = 0; i < length; i++) {

        // --- Stage 1 ---

        // Shift history for stage 1
        for (int j = FILTER_ORDER; j > 0; j--) {
            x_history1[j] = x_history1[j - 1];
            y_history1[j] = y_history1[j - 1];
        }

        // Store current input for stage 1 (convert to 32-bit)
        x_history1[0] = (int32_t)input[i];

        // Calculate output for stage 1 (using 32-bit intermediate sum)
        int32_t stage1_output = 0;

        // Contribution from input terms (b coefficients)
        for (int j = 0; j <= FILTER_ORDER; j++) {
            stage1_output += (int32_t)b_stage1[j] * x_history1[j];
        }

        // Contribution from feedback terms (a coefficients, excluding a[0])
        for (int j = 1; j <= FILTER_ORDER; j++) {
            stage1_output -= (int32_t)a_stage1[j] * y_history1[j];
        }

        // Scale the output back (fixed-point scaling)
        stage1_output >>= q;


        // --- Stage 2 ---

        // Shift history for stage 2
        for (int j = FILTER_ORDER; j > 0; j--) {
            x_history2[j] = x_history2[j - 1];
            y_history2[j] = y_history2[j - 1];
        }

        // The input to stage 2 is the output of stage 1
        x_history2[0] = stage1_output;

        // Calculate output for stage 2 (using 32-bit intermediate sum)
        int32_t stage2_output = 0;

        // Contribution from input terms (b coefficients)
        for (int j = 0; j <= FILTER_ORDER; j++) {
            stage2_output += (int32_t)b_stage2[j] * x_history2[j];
        }

        // Contribution from feedback terms (a coefficients, excluding a[0])
        for (int j = 1; j <= FILTER_ORDER; j++) {
            stage2_output -= (int32_t)a_stage2[j] * y_history2[j];
        }

        // Scale the output back (fixed-point scaling)
        stage2_output >>= q;

        // Update output history for stage 1 (store the calculated output)
        y_history1[0] = stage1_output; // Store output before saturation

        // Update output history for stage 2 (store the calculated output)
        y_history2[0] = stage2_output; // Store output before saturation

        // --- Saturation and Final Output ---

        const int32_t satVal = 32767;
        int32_t final_output = stage2_output;

        // Saturate the final output to valid 16b range
        if (final_output > satVal) {
            final_output = satVal;
        } else if (final_output < -satVal - 1) { // Account for -32768 being valid
            final_output = -satVal - 1;
        }

        // Store the final 16-bit output
        output[i] = (int16_t)final_output;
    }
}*/


/*
// Function to apply IIR filter
void applyIIRFilter(int16_t *input, int16_t *output, int length) {

    for (int i = 0; i < length; i++) {

        // Update input history and compute contributions from past inputs and outputs
        for (int j=FILTER_ORDER; j>0; j--) {
            x_history1[j] = x_history1[j-1];  // Shift the input to the right // x[n-1] = x[n-2], x[n-2] = x[n-3], ... x[1] = x[0].
            y_history1[j] = y_history1[j-1]; // Shift the output to the right  // y[n-1] = y[n-2], y[n-2] = y[n-3], ... y[1] = y[0], y[0] = OUTPUT.
        }

        // Current input
        x_history1[0] = input[i];

        output_val = 0;
        // Update input history and compute contributions from past inputs and outputs
        for (int j=0; j<=FILTER_ORDER; j++) {
            // Add contributions from past inputs and outputs
            // y[n] = b*x[n]-a*y[n-1]
            output_val += (int16_t)(((int32_t)b_stage1[j] * x_history1[j]) >> q);      //b[0] * x[0] + b[1] * x[1]...+b[2] * x[2]
            if (j > 0) { // Skip a[0]
                output_val -= (int16_t)(((int32_t)a_stage1[j] * y_history1[j]) >> q);  //a[1] * y[5] + a[2] * y[4]...+a[6] * y[0]
            }
        }

        // Store the output in the output array
        output[i] = output_val;
        y_history1[0] = output[i];  // This is y[n], now stored as the most recent output
    }
}


// Function to apply IIR filter
void applyIIRFilter2(int16_t *input, int16_t *output, int length) {

    for (int i = 0; i < length; i++) {

        // Update input history and compute contributions from past inputs and outputs
        for (int j=FILTER_ORDER; j>0; j--) {
            x_history2[j] = x_history2[j-1];  // Shift the input to the right // x[n-1] = x[n-2], x[n-2] = x[n-3], ... x[1] = x[0].
            y_history2[j] = y_history2[j-1]; // Shift the output to the right  // y[n-1] = y[n-2], y[n-2] = y[n-3], ... y[1] = y[0], y[0] = OUTPUT.
        }

        // Current input
        x_history2[0] = input[i];

        output_val = 0;
        // Update input history and compute contributions from past inputs and outputs
        for (int j=0; j<=FILTER_ORDER; j++) {
            // Add contributions from past inputs and outputs
            // y[n] = b*x[n]-a*y[n-1]
            output_val += (int16_t)(((int32_t)b_stage2[j] * x_history2[j]) >> q);      //b[0] * x[6] + b[1] * x[5]...+b[6] * x[0]
            if (j > 0) { // Skip a[0]
                output_val -= (int16_t)(((int32_t)a_stage2[j] * y_history2[j]) >> q);  //a[1] * y[5] + a[2] * y[4]...+a[6] * y[0]
            }
        }

        // Store the output in the output array
        output[i] = output_val;
        y_history2[0] = output[i];  // This is y[n], now stored as the most recent output
    }
}*/
#endif
#endif

#ifdef USE_DMIC

#include <stdint.h>
#include <stdbool.h>

/* Driver configuration */
//#include "ti_drivers_config.h"

// Define byte endianess of input stream
#define BITSTREAM_BE
//#define BITSTREAM_LE16

// LUT for order 4 CIC decimator. The LUT index corresponds to 8 bits of
// input, MSB to LSB (followed by 0 zero bits). Each LUT entry contains a
// packed word that contains the increment value to apply to each accumulator
// for the given input word:
//    [ 3: 0] 4 bit increment value for ACC0 (max val 8)
//    [ 9: 4] 6 bit increment value for ACC1 (max val 36)
//    [16:10] 7 bit increment value for ACC2 (max val 120)
//    [25:17] 9 bit increment value for ACC3 (max val 330)
//    [31:26] zeroes
// In addition each accumulator needs to be incremented based on the other
// accumulator values so the whole operation becomes:
//    ACC3 +=   120*ACC0 +    36*ACC1 +     8*ACC2 +       LUT[x][25:17]
//    ACC2 +=    36*ACC0 +     8*ACC1 +       LUT[x][16:10]
//    ACC1 +=     8*ACC0 +       LUT[x][ 9: 4]
//    ACC0 +=       LUT[x][ 3: 0]
static const uint32_t cicOrd4Bits8Pad0Lut[] =
{
    0x00000000, 0x00020411, 0x00080C21, 0x000A1032, 0x00141831, 0x00161C42, 0x001C2452, 0x001E2863, // [   0]
    0x00282841, 0x002A2C52, 0x00303462, 0x00323873, 0x003C4072, 0x003E4483, 0x00444C93, 0x004650A4, // [   8]
    0x00463C51, 0x00484062, 0x004E4872, 0x00504C83, 0x005A5482, 0x005C5893, 0x006260A3, 0x006464B4, // [  16]
    0x006E6492, 0x007068A3, 0x007670B3, 0x007874C4, 0x00827CC3, 0x008480D4, 0x008A88E4, 0x008C8CF5, // [  24]
    0x00705461, 0x00725872, 0x00786082, 0x007A6493, 0x00846C92, 0x008670A3, 0x008C78B3, 0x008E7CC4, // [  32]
    0x00987CA2, 0x009A80B3, 0x00A088C3, 0x00A28CD4, 0x00AC94D3, 0x00AE98E4, 0x00B4A0F4, 0x00B6A505, // [  40]
    0x00B690B2, 0x00B894C3, 0x00BE9CD3, 0x00C0A0E4, 0x00CAA8E3, 0x00CCACF4, 0x00D2B504, 0x00D4B915, // [  48]
    0x00DEB8F3, 0x00E0BD04, 0x00E6C514, 0x00E8C925, 0x00F2D124, 0x00F4D535, 0x00FADD45, 0x00FCE156, // [  56]
    0x00A87071, 0x00AA7482, 0x00B07C92, 0x00B280A3, 0x00BC88A2, 0x00BE8CB3, 0x00C494C3, 0x00C698D4, // [  64]
    0x00D098B2, 0x00D29CC3, 0x00D8A4D3, 0x00DAA8E4, 0x00E4B0E3, 0x00E6B4F4, 0x00ECBD04, 0x00EEC115, // [  72]
    0x00EEACC2, 0x00F0B0D3, 0x00F6B8E3, 0x00F8BCF4, 0x0102C4F3, 0x0104C904, 0x010AD114, 0x010CD525, // [  80]
    0x0116D503, 0x0118D914, 0x011EE124, 0x0120E535, 0x012AED34, 0x012CF145, 0x0132F955, 0x0134FD66, // [  88]
    0x0118C4D2, 0x011AC8E3, 0x0120D0F3, 0x0122D504, 0x012CDD03, 0x012EE114, 0x0134E924, 0x0136ED35, // [  96]
    0x0140ED13, 0x0142F124, 0x0148F934, 0x014AFD45, 0x01550544, 0x01570955, 0x015D1165, 0x015F1576, // [ 104]
    0x015F0123, 0x01610534, 0x01670D44, 0x01691155, 0x01731954, 0x01751D65, 0x017B2575, 0x017D2986, // [ 112]
    0x01872964, 0x01892D75, 0x018F3585, 0x01913996, 0x019B4195, 0x019D45A6, 0x01A34DB6, 0x01A551C7, // [ 120]
    0x00F09081, 0x00F29492, 0x00F89CA2, 0x00FAA0B3, 0x0104A8B2, 0x0106ACC3, 0x010CB4D3, 0x010EB8E4, // [ 128]
    0x0118B8C2, 0x011ABCD3, 0x0120C4E3, 0x0122C8F4, 0x012CD0F3, 0x012ED504, 0x0134DD14, 0x0136E125, // [ 136]
    0x0136CCD2, 0x0138D0E3, 0x013ED8F3, 0x0140DD04, 0x014AE503, 0x014CE914, 0x0152F124, 0x0154F535, // [ 144]
    0x015EF513, 0x0160F924, 0x01670134, 0x01690545, 0x01730D44, 0x01751155, 0x017B1965, 0x017D1D76, // [ 152]
    0x0160E4E2, 0x0162E8F3, 0x0168F103, 0x016AF514, 0x0174FD13, 0x01770124, 0x017D0934, 0x017F0D45, // [ 160]
    0x01890D23, 0x018B1134, 0x01911944, 0x01931D55, 0x019D2554, 0x019F2965, 0x01A53175, 0x01A73586, // [ 168]
    0x01A72133, 0x01A92544, 0x01AF2D54, 0x01B13165, 0x01BB3964, 0x01BD3D75, 0x01C34585, 0x01C54996, // [ 176]
    0x01CF4974, 0x01D14D85, 0x01D75595, 0x01D959A6, 0x01E361A5, 0x01E565B6, 0x01EB6DC6, 0x01ED71D7, // [ 184]
    0x019900F2, 0x019B0503, 0x01A10D13, 0x01A31124, 0x01AD1923, 0x01AF1D34, 0x01B52544, 0x01B72955, // [ 192]
    0x01C12933, 0x01C32D44, 0x01C93554, 0x01CB3965, 0x01D54164, 0x01D74575, 0x01DD4D85, 0x01DF5196, // [ 200]
    0x01DF3D43, 0x01E14154, 0x01E74964, 0x01E94D75, 0x01F35574, 0x01F55985, 0x01FB6195, 0x01FD65A6, // [ 208]
    0x02076584, 0x02096995, 0x020F71A5, 0x021175B6, 0x021B7DB5, 0x021D81C6, 0x022389D6, 0x02258DE7, // [ 216]
    0x02095553, 0x020B5964, 0x02116174, 0x02136585, 0x021D6D84, 0x021F7195, 0x022579A5, 0x02277DB6, // [ 224]
    0x02317D94, 0x023381A5, 0x023989B5, 0x023B8DC6, 0x024595C5, 0x024799D6, 0x024DA1E6, 0x024FA5F7, // [ 232]
    0x024F91A4, 0x025195B5, 0x02579DC5, 0x0259A1D6, 0x0263A9D5, 0x0265ADE6, 0x026BB5F6, 0x026DBA07, // [ 240]
    0x0277B9E5, 0x0279BDF6, 0x027FC606, 0x0281CA17, 0x028BD216, 0x028DD627, 0x0293DE37, 0x0295E248, // [ 248]
};



// Implements PDM to PCM conversion (64x OSR, 4th order DS)
//
// Reads in 256B block of PDM data (1b/sample @ Fs*64) and outputs
// 32 samples of PCM data (16b/sample @ Fs)
//
// Signal chain details:
// * Decimate-by-32 (4th order CIC decimation from 64*Fs to 2*Fs)
// * N biquad IIR filters (@2*Fs) typically implement DC notch, halfband
//   filter and CIC passband droop compensation
// * Decimate-by-2 (accumulate and dump from 2*Fs to Fs)
// * Optionally, M biquad filters (@Fs) to implement application-specific
//   filtering such as user-adjustable EQ or enclosure compensation)
//
// \param[in,out] pIn
//   Pointer to input bitstream (endianess BITSTREAM_BE, BITSTREAM_LE16).
//   Bitstream overwritten with intermediate values during filtering
// \param[in,out] pState
//   Pointer to state structure for filters (zero-init before first call)
//   Six 32b words for CIC + two 32b words per filter stage in pBqCoeffs
// \param[in] pBqCoeffs
//   Pointer to list of biquad stage coefficients. Five 32b coefficients per
//   stage in the sequence G, B1, B2, A1, A2 implementing transfer function:
//                 1 + B1*z^-1 + B2*z^-2
//     H(z) = G * -----------------------
//                 1 + A1*z^-1 + A2*z^-2
//   All coefficients has the format SI.FFFF_FFFF_FF format, i.e 21 sign bits,
//   one integer bit and ten fractional bits and must remain within a range of
//   (-2,+2). For example, the integer value 1024 corresponds to a coefficient
//   value of +1.0.
//   The list must contain N*5 coefficients followed by a terminating zero
//   value, then optionally M*5 coefficients, then a a terminating zero value.
//   E.g.:
//     G_1, B1_1, B2_1, A1_1, A2_1,     // Corrective filter, first stage
//     G_2, B1_2, B2_2, A1_2, A2_2,     // Corrective filter, second stage
//     G_3, B1_3, B2_3, A1_1, A2_3,     // Corrective filter, third stage
//     G_4, B1_4, B2_4, A1_2, A2_4,     // Corrective filter, fourth stage
//     G_5, B1_5, B2_5, A1_1, A2_5,     // Corrective filter, fifth stage
//     0,                               // Terminate corrective filter
//    (G_1, B1_1, B2_1, A1_1, A2_1,)    // Optional EQ filter, first stage
//    (G_2, B1_2, B2_2, A1_2, A2_2,)    // Optional EQ filter, second stage
//    (G_3, B1_3, B2_3, A1_1, A2_3,)    // Optional EQ filter, third stage
//     0                                // Terminate coefficient list
// \param[out] pOut
//   Pointer to where 16b PCM samples are output
//
// \note
//   1b PDM samples must be stored MSB first in each input byte. If bytes are
//   stored in same order as received over PDM interface use the BITSTREAM_BE
//   define. If bytes are received as 16b words and stored to memory using
//   little-endian memory ordering use the BITSTREAM_LE16 define.
//


void pdm2pcm16k(
    uint8_t* pIn,
    void* pState,
    const int32_t* pBqCoeffs,
    int16_t* pOut
) {
    // ***********************************************************************
    // *     FIRST STAGE: 4th order decimate-by-32 CIC (64*FS --> 2*Fs)      *
    // ***********************************************************************

    // Read in accumulator/differentiator state
    uint32_t acc0  = ((uint32_t*)pState)[0];
    uint32_t acc1  = ((uint32_t*)pState)[1];
    uint32_t acc2  = ((uint32_t*)pState)[2];
    uint32_t diff2 = ((uint32_t*)pState)[3];
    uint32_t diff1 = ((uint32_t*)pState)[4];
    uint32_t diff0 = ((uint32_t*)pState)[5];

    // Process 2048 1b input samples @64*Fs, producing 64 32b output samples @2*Fs
    uint32_t nLoopsRem = 256*8/32;
    do {
        // To save one differentiator and two state variables reset innermost
        // accumulator each decimation period
        uint32_t acc3 = 0;

        // Run over 32 input samples, 8b at a time (according to byte endianess)
        #define CIC_8b(index) do { \
            /* Update accumulator values for 8 samples disregarding input */ \
            /* bits which we can, thanks to superposition, handle below) */ \
            acc3 += 120U * acc0; \
            acc3 += 36U  * acc1; \
            acc3 += 8U   * acc2; \
            acc2 += 36U  * acc0; \
            acc2 += 8U   * acc1; \
            acc1 += 8U   * acc0; \
                                 \
            /* Read first 8 samples, get LUT value for affect on accX */ \
            uint32_t lut = cicOrd4Bits8Pad0Lut[pIn[index]]; \
            /* LUT[ 3: 0] 4 bit increment value for ACC0 (max val 8)   */ \
            /* LUT[ 9: 4] 6 bit increment value for ACC1 (max val 36)  */ \
            /* LUT[16:10] 7 bit increment value for ACC2 (max val 120) */ \
            /* LUT[25:17] 9 bit increment value for ACC3 (max val 330) */ \
            /* LUT[31:26] zero */ \
            acc0 += lut & ((1<<4)-1); \
            lut >>= 4; \
            acc1 += lut & ((1<<6)-1); \
            lut >>= 6; \
            acc2 += lut & ((1<<7)-1); \
            lut >>= 7; \
            acc3 += lut; \
        } while (0)

        #if defined(BITSTREAM_BE)
            CIC_8b(0);
            CIC_8b(1);
            CIC_8b(2);
            CIC_8b(3);
        #elif defined(BITSTREAM_LE16)
            CIC_8b(1);
            CIC_8b(0);
            CIC_8b(3);
            CIC_8b(2);
        #else
            #error Define either BITSTREAM_BE or BITSTREAM_LE16 byte endianess
        #endif
        #undef CIC_8b

        // End of decimation period, perform differentiation and output sample
        uint32_t diff3Out = acc3     - 0;
        uint32_t diff2Out = diff3Out - diff2;
        diff2 = diff3Out;
        uint32_t diff1Out = diff2Out - diff1;
        diff1 = diff2Out;
        uint32_t diff0Out = diff1Out - diff0;
        diff0 = diff1Out;

        // Convert output to SI3.F17 format and 2s complement
        int32_t output = (diff0Out>>2) - 0x20000;

        // Store decimated sample back on top of bitstream data, increment pIn
        *((int32_t*)pIn) = output;
        pIn += 4;
    } while (--nLoopsRem);

    // Update accumulator/differentiator state
    ((uint32_t*)pState)[0] = acc0;
    ((uint32_t*)pState)[1] = acc1;
    ((uint32_t*)pState)[2] = acc2;
    ((uint32_t*)pState)[3] = diff2;
    ((uint32_t*)pState)[4] = diff1;
    ((uint32_t*)pState)[5] = diff0;

    // Spool back pIn to beginning of block
    pIn -= 2048/8;

    // ***********************************************************************
    // *      SECOND STAGE: Apply N stages of biquad filtering (@ 2*Fs)      *
    // ***********************************************************************
    // Move pointer to state of first stage
    #ifdef USE_DIGITAL_FILTER_1
    pState = &((uint32_t*)pState)[6];

    // Loop over N stages (N given by coefficient array)
    while (1) {
        // Define aliases for coefficients for this stage
        #define G  pBqCoeffs[0]
        #define B1 pBqCoeffs[1]
        #define B2 pBqCoeffs[2]
        #define A1 pBqCoeffs[3]
        #define A2 pBqCoeffs[4]

        // Check if last stage of filter reached
        if (G == 0) {
            break;
        }

        // Load state for next stage
        int32_t s_m1 = ((int32_t*)pState)[0];
        int32_t s_m2 = ((int32_t*)pState)[1];

        // Loop over samples in block
        uint32_t samples = 32*2;
        int32_t* pSamples = (void*)pIn;
        do {
            int32_t x       = *pSamples;            // Read input sample in place
            int32_t g_x     = (G*x) >> 10;          // g_x      = g*x
            int32_t y       = g_x + s_m1;           // y        = g*x + s[-1]
            *pSamples++     = y;                    // Write output sample in place

            int32_t g_b1_x  = (g_x*B1)>>10;         // g_b1_x   = g*b1*x
            int32_t a1_y    = (A1*y) >> 10;         // a1_y     = a1*y
            s_m1            = s_m2 + g_b1_x - a1_y; // s'[-1]   = s[-2] + g*b1*x - a1*y

            int32_t g_b2_x  = (g_x*B2)>>10;         // g_b2_x   = g*b2*x
            int32_t a2_y    = (A2*y) >> 10;         // a2_y     = a2*y
            s_m2            = g_b2_x - a2_y;        // s'[-2]   = g*b2*x - a2*y
        } while (--samples);
        #undef G
        #undef B1
        #undef B2
        #undef A1
        #undef A2

        // Save state for current stage and increment pointers
        ((uint32_t*)pState)[0] = s_m1;
        ((uint32_t*)pState)[1] = s_m2;
        pState = &((uint32_t*)pState)[2];
        pBqCoeffs += 5;
    }
    #endif //USE_DIGITAL_FILTER_1


    // ***********************************************************************
    // *              THIRD STAGE: Decimate by 2 (2*Fs -> Fs)                *
    // ***********************************************************************
    // Process 64 32b input samples @2*Fs, producing 32 32b output samples @Fs

    for (uint32_t i = 0; i<32; i++) {
        ((int32_t*)pIn)[i] =
            ( ((int32_t*)pIn)[2*i+0] + ((int32_t*)pIn)[2*i+1] ) >> 1;
    }


    // ***********************************************************************
    // *  FOURTH STAGE: Optionally apply M stages of biquad filtering (@ Fs) *
    // ***********************************************************************
    // Loop over M stages (M given by coefficient array)
    #ifdef USE_DIGITAL_FILTER_2
    while (1) {
        // Define aliases for coefficients for this stage
        #define G  pBqCoeffs[0]
        #define B1 pBqCoeffs[1]
        #define B2 pBqCoeffs[2]
        #define A1 pBqCoeffs[3]
        #define A2 pBqCoeffs[4]

        // Check if last stage of filter reached
        if (G == 0) {
            break;
        }

        // Load state for next stage
        int32_t s_m1 = ((int32_t*)pState)[0];
        int32_t s_m2 = ((int32_t*)pState)[1];

        // Loop over samples in block
        uint32_t samples = 32;
        int32_t* pSamples = (void*)pIn;
        do {
            int32_t x       = *pSamples;            // Read input sample in place
            int32_t g_x     = (G*x) >> 10;          // g_x      = g*x
            int32_t y       = g_x + s_m1;           // y        = g*x + s[-1]
            *pSamples++     = y;                    // Write output sample in place

            int32_t g_b1_x  = (g_x*B1)>>10;         // g_b1_x   = g*b1*x
            int32_t a1_y    = (A1*y) >> 10;         // a1_y     = a1*y
            s_m1            = s_m2 + g_b1_x - a1_y; // s'[-1]   = s[-2] + g*b1*x - a1*y

            int32_t g_b2_x  = (g_x*B2)>>10;         // g_b2_x   = g*b2*x
            int32_t a2_y    = (A2*y) >> 10;         // a2_y     = a2*y
            s_m2            = g_b2_x - a2_y;        // s'[-2]   = g*b2*x - a2*y
        } while (--samples);
        #undef G
        #undef B1
        #undef B2
        #undef A1
        #undef A2

        // Save state for current stage and increment pointers
        ((uint32_t*)pState)[0] = s_m1;
        ((uint32_t*)pState)[1] = s_m2;
        pState = &((uint32_t*)pState)[2];
        pBqCoeffs += 5;
    }
    #endif //USE_DIGITAL_FILTER_2

    // ***********************************************************************
    // *     FIFHT STAGE: Saturate results to 16b signed and output (@ Fs)   *
    // ***********************************************************************
    nLoopsRem = 32;
    const int32_t satVal = 32767;
    int32_t* pSamples = (void*)pIn;
    do {
        int32_t val = *pSamples++;
        // Saturate output to valid 16b range
        if (val > satVal) {
            val = satVal;
        } else if (val < -satVal) {
            val = -satVal;
        }
        *pOut++ = (int16_t)val;
    } while (--nLoopsRem);

}
#endif
