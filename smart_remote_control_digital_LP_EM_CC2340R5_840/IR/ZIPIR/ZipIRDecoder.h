/*****************************************************************************
 ** COPYRIGHT 2020 UNIVERSAL ELECTRONICS INC (“UEI”). ALL RIGHTS RESERVED.
 **
 ** These materials (“Materials”) are the intellectual property of UEI, and are
 ** provided by UEI on an “As-Is” basis without warranties or representations.
 **
 ** The Materials are provided to the direct recipient of the Materials
 ** ("Recipient") and such Recipient's authorized partners and customers for
 ** internal development, evaluation and implementation purposes only
 ** ("Authorized Purpose").
 **
 ** The user of the Materials may not:
 ** (i) modify or create any derivative works of the Materials;
 ** (ii) decompile, disassemble, reverse engineer, or otherwise attempt to
 ** derive or extract any part of the Materials; or
 ** (iii) redistribute, sell, lease, sublicense, or otherwise transfer ownership
 ** of the Materials without the prior written consent of UEI.
 **
 ** By accessing and using the Materials, the user agrees to use the Materials
 ** solely for the Authorized Purpose only.
 ********************************************************************************
 * File : ZipIRDecoder.h
 * Revision : $Revision: $
 * Date : $Date: $
 * Updated by : $Author: $
 * Description :
 *
 ******************************************************************************/
#ifdef USE_ZIPIR
#ifndef _ZIP_IR_DECODER_H_
#define _ZIP_IR_DECODER_H_
#include "typedefs.h"

/* MACRO DEFINITIONS */
#define CHAR_BIT 8
#define DEVICE_TYPE_OFFSET 9
#define DEVICE_INFO 5
#define NRS_SIZE 6
#define MAX_LONG_TIMINGS 16 // NLT must be <= MAX_LONG_TIMINGS
#define MAX_TIMINGS 28 // NCT + 1 must be <= MAX_TIMINGS
#define MAX_REP_SEQ 60 // NRS + 1 must be < MAX_REP_SEQ
#define MAX_REP_SEQ_SIZE 20 // SRS[i] + 1 must be < MAX_REP_SEQ_SIZE
#define MAX_WAVEFORM_NUM 384 // 128*3 - 128 keys with 3 WS per key
// NWS + 1 must be <= MAX_WAVEFORM_NUM
#define BITMASK_DEFERRED_REPEAT_FRAME (0x1 << 6) // Bit6 => 0x40 (keep repeating)
#define BITMASK_DEFERRED_BREAK_FRAME (0x1 << 5) // Bit5 => 0x20 (just once)

/* TYPE DEFINITIONS */
// For one CodeSet
typedef struct ZIPIrSignalInfo
{
    U8 *codeSet;
    U32 codeOffset;
    U16 funOffset;
    U8 ver;
    U8 em;
    U8 timLongTbl[MAX_LONG_TIMINGS]; // Long time for IR timing mode may overflow
    // U16
    U16 timTbl[MAX_TIMINGS];
    U16 carrierPeriod;
    U8 dutyCycle;
    U8 numTime;
    U8 longTime;
    U8 numWaveformSubs;
    U8 bwWaveformSubs; // Bit width for waveform substitution block
    U8 waveformSubs[MAX_WAVEFORM_NUM];
    U8 numRepSeq;
    U8 repSeqSize[MAX_REP_SEQ];
    U8 repSeq[MAX_REP_SEQ][MAX_REP_SEQ_SIZE];
    U8 sqc;
    U8 wsp; // Multiple functions
    U8 wsIndexWidth;
    U8 wsIndex[3];
} ZIPIrSignalInfo_t;

/* PUBLIC VARIABLE DECLARATIONS */
extern ZIPIrSignalInfo_t ZIPZcsdata;

/* PUBLIC FUNCTION DECLARATIONS */
BOOL DecodeZSFData(void);
BOOL DecodeZCLData(U8 device, U16 codeSet);
U32 GetBits(int length);
U8 GetSQC(void);
void GetWSFunction(U8 function, U8 seq);
void InitBitBuffer(U8 *pbuf, int bit, U8 bucket);
void LocateWS(int wsIndex);

#endif // _ZIP_IR_DECODER_H_

#endif
