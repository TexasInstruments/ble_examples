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
 * File : ZipIRDecoder.c
 * Revision : $Revision: $
 * Date : $Date: $
 * Updated by : $Author: $
 * Description :
 *
 ******************************************************************************/

#ifdef USE_ZIPIR

/* INCLUDE FILES */
#include "ZipIRDecoder.h"
#include "ZipIRInterface.h"
/* PUBLIC VARIABLE DEFINITIONS */
ZIPIrSignalInfo_t ZIPZcsdata;
/* LOCAL VARIABLE DEFINITIONS */
static U8 bitBucket = 0, savebitBucket;
static int bitLeft = 0, savebitLeft;
static U8 *bitBuffer, *savebitBuffer;
/* LOCAL FUNCTION DEFINITIONS */
static BOOL DecodeZCSData(void);
static U8 GetDeviceCodeSet(U8 device, U16 codeSet);
static BOOL GetTimingInfo(void);
static BOOL FindRepeatSequence(void);
static BOOL FindWaveFormPattern(void);
static void IrConvertMarkSpace(void);
static U8* GetBitBufferPtr(void);
static void SkipBytes(int length);
static void SaveBitReadPoint(void);
static void RetrieveReadPoint(void);
static U8 PeekByte(void);
static void SkipBits(int length);
static U32 GetBytes(int length);
//=============================================================================
// Decoder functions
//=============================================================================
/**
 *******************************************************************************
 * @fn DecodeZCLData
 *
 * @brief Decodes compressed ZCL data block.
 *
 * @param[in] device - mode of code set
 * @param[in] codeset - number of code set
 * @param[out] None
 *
 * @return TRUE = Success
 * FALSE = Failed
 *
 ******************************************************************************/
BOOL DecodeZCLData(U8 device, U16 codeSet)
{
    // Skip over the versioning data at the beginning of the ZCL data
    SkipBytes(DEVICE_TYPE_OFFSET);
    // Check device type and code set ID
    if (GetDeviceCodeSet(device, codeSet))
    {
        // Decode the timing table, repeat sequence, and wave subs for a code set.
        return DecodeZCSData();
    }
    return FALSE;
}
/**
 *******************************************************************************
 * @fn DecodeZSFData
 *
 * @brief Decodes compressed ZSF data block.
 *
 * @param[in] None
 * @param[out] None
 *
 * @return TRUE = Success
 * FALSE = Failure
 *
 ******************************************************************************/
BOOL DecodeZSFData(void)
{
    // Is this ZSF data?
    if (GetBytes(1) == ZSF_FILE_ID)
    {
        // Skip over the Device ID byte
        SkipBytes(1);
        // The third byte is the ZCS length minus 1
        ZIPZcsdata.funOffset = GetBytes(1) + 1;
        // Decode the timing table, repeat sequence, and wave subs for a code set.
        return DecodeZCSData();
    }
    return FALSE;
}
/**
 *******************************************************************************
 * @fn DecodeZCSData
 *
 * @brief Decodes compressed ZCS data block.
 *
 * @param[in] None
 * @param[out] None
 *
 * @return TRUE = Success
 * FALSE = Failed
 *
 ******************************************************************************/
static BOOL DecodeZCSData(void)
{
    // Code for version handling
    ZIPZcsdata.codeSet = GetBitBufferPtr();
    ZIPZcsdata.ver = GetBits(4);
    ZIPZcsdata.em = GetBits(4);
    // Get Carrier Freq
    ZIPZcsdata.carrierPeriod = GetBits(12);
    // Get Duty cycle
    ZIPZcsdata.dutyCycle = GetBits(4);
    // Get Timing info
    if (!GetTimingInfo())
    {
        return FALSE;
    }
    // Get the sequence info
    if (!FindRepeatSequence())
    {
        return FALSE;
    }
    // Construct wave form
    if (!FindWaveFormPattern())
    {
        return FALSE;
    }
    IrConvertMarkSpace();
    return TRUE;
}
/**
 *******************************************************************************
 * @fn GetDeviceCodeSet
 *
 * @brief Locates a code-set if the compressed data block contains it.
 *
 * @param[in] device - code-set mode, 0 to ignore both the mode and number of
 * the code-set
 * @param[in] codeset - code-set number
 * @param[out] None
 *
 * @return TRUE = Success
 * FALSE = Failed
 *
 ******************************************************************************/
static U8 GetDeviceCodeSet(U8 device, U16 codeSet)
{
    int codeCount, tempData, result = FALSE, tc;
    // Extract the number of device types supported
    tc = GetBytes(1);
    // Loop until the specified device type is found, or use the first entry
    // if input param 'device' is set to 0
    while (tc--)
    {
        if ((device == 0) || (PeekByte() == device))
        {
            // Skip over device info blocks until we find a device type = 0,
            // which indicates the final 'dummy' entry in the Device Type Table
            while (PeekByte())
            {
                SkipBytes(DEVICE_INFO);
            }
            // Now skip over the final, delimiting 'dummy' entry
            SkipBytes(DEVICE_INFO);
            // Extract the number of code sets stored within this data
            codeCount = GetBytes(2);
            // Loop until the specified code set ID is found, or use the first
            // entry if input param 'codeset' is set to 0
            while (codeCount--)
            {
                // Read the next code set ID
                tempData = GetBytes(2);
                // If the selected code set ID has been found, store its ZCS
                // and ZFT offsets
                if ((device == 0) || (codeSet == tempData))
                {
                    ZIPZcsdata.codeOffset = GetBytes(4);
                    ZIPZcsdata.funOffset = GetBytes(2);
// Skip over code set info blocks until we find a code set
                    // ID = 0xFFFF, which indicates the final 'dummy' entry in
// the Code Set Lookup Table
                    while (tempData != 0xFFFF)
                    {
                        tempData = GetBytes(2);
                        SkipBytes(6);
                    }
                    // Position the buffer read pointer to the beginning of the
// selected ZCS code set data block
                    SkipBytes(ZIPZcsdata.codeOffset);
                    result = TRUE;
                    break;
                }
                else
                {
                    // Not found? Skip over the current entry's ZCS and ZFT offsets
                    SkipBytes(6);
                }
            }
            break;
        }
        else
        {
            // Not found? Skip over the current device info block
            SkipBytes(DEVICE_INFO);
        }
    }
    return result;
}
/**
 *******************************************************************************
 * @fn GetTimingInfo
 *
 * @brief Locates the timing info if compressed data block contains it.
 *
 * @param[in] None
 * @param[out] None
 *
 * @return TRUE = Success
 * FALSE = Failed
 *
 ******************************************************************************/
static BOOL GetTimingInfo(void)
{
    int i;
    // Extract and save the total number of timings & the number of long timings
    ZIPZcsdata.numTime = GetBits(6) + 1;
    ZIPZcsdata.longTime = GetBits(4);
    if ((ZIPZcsdata.numTime > MAX_TIMINGS)
            || (ZIPZcsdata.longTime > MAX_LONG_TIMINGS))
    {
        return FALSE;
    }
    // Extract and save the long timings
    for (i = 0; i < ZIPZcsdata.longTime; i++)
    {
        ZIPZcsdata.timTbl[i] = GetBits(16);
    }
    // Extract and save the normal timings
    for (; i < ZIPZcsdata.numTime; i++)
    {
        ZIPZcsdata.timTbl[i] = GetBits(12);
    }
    return TRUE;
}
/**
 *******************************************************************************
 * @fn FindRepeatSequence
 *
 * @brief Decodes a Repeat Sequence.
 *
 * @param[in] None
 * @param[out] None
 *
 * @return TRUE = Success
 * FALSE = Failed
 *
 ******************************************************************************/
static BOOL FindRepeatSequence(void)
{
    int number = ZIPZcsdata.numTime - 1, length, count = 1, i, j;
    // Determine the number of bits req'd to store a Count Time index
    while (number = number / 2)
    {
        count++;
    }
    // Extract the number of Repeat Sequences
    number = GetBits(NRS_SIZE) + 1;
    if (number >= MAX_REP_SEQ)
    {
        return FALSE;
    }
    // Extract the number of bits req'd to store the Repeat Sequence sizes
    length = GetBits(2) + 3;
    // Extract and save the Repeat Sequence sizes
    for (i = 0; i < number; i++)
    {
        ZIPZcsdata.repSeqSize[i] = GetBits(length) + 1;
        if (ZIPZcsdata.repSeqSize[i] >= MAX_REP_SEQ_SIZE)
        {
            return FALSE;
        }
    }
    // For each Repeat Sequence, extract and save its sequence elements (which
    // are Count Time indices)
    for (i = 0; i < number; i++)
    {
        for (j = 0; j < ZIPZcsdata.repSeqSize[i]; j++)
        {
            ZIPZcsdata.repSeq[i][j] = GetBits(count);
        }
    }
    // Save the number of Repeat Sequences
    ZIPZcsdata.numRepSeq = number;
    // Determine the number of bits req'd to store a Repeat Sequence index
    number--;
    count = 1;
    while (number = number / 2)
    {
        count++;
    }
    // Save the bit width of Repeat Sequence indices
    ZIPZcsdata.bwWaveformSubs = count;
    return TRUE;
}
/**
 *******************************************************************************
 * @fn FindWaveFormPattern
 *
 * @brief Decodes a Waveform Pattern.
 *
 * @param[in] None
 * @param[out] None
 *
 * @return TRUE = Success
 * FALSE = Failed
 *
 ******************************************************************************/
static BOOL FindWaveFormPattern(void)
{
    int count = 1, length, number, i;
    // Extract the number of Waveform Substitutions
    number = GetBits(10) + 1;
    if (number > MAX_WAVEFORM_NUM)
    {
        return FALSE;
    }
    // Extract the number of bits req'd to store the Waveform Substitution sizes
    length = GetBits(3) + 3;
    // Extract and save the Waveform Substitution sizes
    for (i = 0; i < number; i++)
    {
        ZIPZcsdata.waveformSubs[i] = GetBits(length) + 1;
    }
    // Save the number of Waveform Substitutions
    ZIPZcsdata.numWaveformSubs = number;
    // Determine the number of bits req'd to store a Waveform Substitution index
    number--;
    while (number = number / 2)
    {
        count++;
    }
    // Save the bit width of Waveform Substitution indices
    ZIPZcsdata.wsIndexWidth = count;
    // Preserve the current bit/byte/buffer parameters, which point to the
    // beginning of the Waveform Substitution arrays
    SaveBitReadPoint();
    // Reinitialize the read buffer to point to the start of the ZFT
    InitBitBuffer(ZIPZcsdata.codeSet + ZIPZcsdata.funOffset, 0, 0);
    // Skip over the Waveform Substitution Index bit width parameter because it
    // was previously computed and saved above
    SkipBits(4);
    // Extract and save the number of Transmission Sequences
    ZIPZcsdata.sqc = GetBits(4) + 1;
    return TRUE;
}
/**
 *******************************************************************************
 * @fn IrConvertMarkSpace
 *
 * @brief Calculates mark and space durations according to timer or carrier
 * mode.
 *
 * @param[in] None
 * @param[out] None
 *
 * @return None
 *
 ******************************************************************************/
static void IrConvertMarkSpace(void)
{
    int i = 0;
    U32 temp;
    if (ZIPZcsdata.carrierPeriod)
    {
        U32 nCarrierPulses;
        U32 half_carrier_period = (U32) ZIPZcsdata.carrierPeriod >> 1;
        while (i < ZIPZcsdata.longTime)
        {
            temp = (U32)(ZIPZcsdata.timTbl[i]) * 8 * 100 + half_carrier_period;
            nCarrierPulses = temp / (U32) ZIPZcsdata.carrierPeriod;
            ZIPZcsdata.timLongTbl[i] = nCarrierPulses >> 15;
            ZIPZcsdata.timTbl[i] = nCarrierPulses & 0x7FFF;
            i++;
        }
        while (i < ZIPZcsdata.numTime)
        {
            temp = (U32)(ZIPZcsdata.timTbl[i]) * 4 * 100 + half_carrier_period;
            ZIPZcsdata.timTbl[i] = temp / (U32) ZIPZcsdata.carrierPeriod;
            i++;
        }
    }
    else
    {
        while (i < ZIPZcsdata.longTime)
        {
            temp = (U32)(ZIPZcsdata.timTbl[i]) * 8;
            ZIPZcsdata.timLongTbl[i] = temp >> 15;
            ZIPZcsdata.timTbl[i] = temp & 0x7FFF;
            i++;
        }
        while (i < ZIPZcsdata.numTime)
        {
            ZIPZcsdata.timTbl[i] = ZIPZcsdata.timTbl[i] * 4;
            i++;
        }
    }
}
U8 lkupTable[8] = { 0, // WSP = 0b000 ==> no substitutions present
        1, // WSP = 0b001 ==> 1 sub present (break)
        1, // WSP = 0b010 ==> 1 sub present (repeat)
        2, // WSP = 0b011 ==> 2 subs present (break & repeat)
        1, // WSP = 0b100 ==> 1 sub present (make)
        2, // WSP = 0b101 ==> 2 subs present (break & make)
        2, // WSP = 0b110 ==> 2 subs present (repeat & make)
        3 // WSP = 0b111 ==> 3 subs present (break, repeat, & make)
        };
/**
 *******************************************************************************
 * @fn GetWSFunction
 *
 * @brief Decodes a Waveform Substitution Index.
 *
 * @param[in] function – the active function
 * @param[in] seq – the active sequence
 * @param[out] None
 *
 * @return None
 *
 ******************************************************************************/
void GetWSFunction(U8 function, U8 seq)
{
    U8 temp;
    int i, k, nbits;
    // Re-init bit buffer for decoding
    InitBitBuffer(ZIPZcsdata.codeSet + ZIPZcsdata.funOffset + 1, 0, 0);
    for (i = 0; i < function; i++)
    {
        for (k = 0; k < ZIPZcsdata.sqc; k++)
        {
            temp = GetBits(3);
            nbits = lkupTable[temp];
            SkipBits(nbits * ZIPZcsdata.wsIndexWidth);
        }
    }
    // Find out which sequence should be used
    seq %= ZIPZcsdata.sqc;
    for (k = 0; k < seq; k++)
    {
        temp = GetBits(3);
        nbits = lkupTable[temp];
        SkipBits(nbits * ZIPZcsdata.wsIndexWidth);
    }
    ZIPZcsdata.wsp = GetBits(3);
    temp = 0x04;
    for (i = 0; i < 3; i++)
    {
        if (ZIPZcsdata.wsp & temp)
        {
            ZIPZcsdata.wsIndex[i] = GetBits(ZIPZcsdata.wsIndexWidth);
        }
        temp >>= 1;
    }
}
/**
 *******************************************************************************
 * @fn LocateWS
 *
 * @brief Locates the reading point for the input Waveform Substitution Index.
 *
 * @param[in] wsIndex - Waveform Substitution Index
 * @param[out] None
 *
 * @return None
 *
 ******************************************************************************/
void LocateWS(int wsIndex)
{
    int i, bitOffset = 0;
    for (i = 0; i < wsIndex; i++)
    {
        bitOffset += ZIPZcsdata.waveformSubs[i];
    }
    bitOffset *= ZIPZcsdata.bwWaveformSubs;
    RetrieveReadPoint();
    SkipBits(bitOffset);
}
/**
 *******************************************************************************
 * @fn GetSQC
 *
 * @brief Gets the Sequence Count decoded from the compressed ZCL data block.
 *
 * @param[in] None
 * @param[out] None
 *
 * @return Sequence Count
 *
 ******************************************************************************/
U8 GetSQC(void)
{
    return ZIPZcsdata.sqc;
}
//=============================================================================
// Bit processor functions
//=============================================================================
/**
 *******************************************************************************
 * @fn GetBitBufferPtr
 *
 * @brief Gets a pointer from current reading location.
 *
 * @param[in] None
 * @param[out] None
 *
 * @return pointer from current reading location
 *
 ******************************************************************************/
static U8* GetBitBufferPtr(void)
{
    return bitBuffer;
}
/**
 *******************************************************************************
 * @fn SkipBytes
 *
 * @brief Skips a number of bytes from the currrent reading location.
 *
 * @param[in] length - number of bytes to skip
 * @param[out] None
 *
 * @return None
 *
 ******************************************************************************/
static void SkipBytes(int length)
{
    bitBuffer += length;
}
/**
 *******************************************************************************
 * @fn InitBitBuffer
 *
 * @brief Initializes the reading location for the bit processor module.
 *
 * @param[in] pBuf - current buffer read location
 * @param[in] bit - current number of residue bits
 * @param[in] bucket - byte that holds the residue bits
 * @param[out] None
 *
 * @return None
 *
 ******************************************************************************/
void InitBitBuffer(U8 *pbuf, int bit, U8 bucket)
{
    bitBuffer = pbuf;
    bitLeft = bit;
    bitBucket = bucket;
}
/**
 *******************************************************************************
 * @fn SaveBitReadPoint
 *
 * @brief Saves the current reading location for the bit processor module.
 *
 * @param[in] None
 * @param[out] None
 *
 * @return None
 *
 ******************************************************************************/
static void SaveBitReadPoint(void)
{
    savebitBuffer = bitBuffer;
    savebitLeft = bitLeft;
    savebitBucket = bitBucket;
}
/**
 *******************************************************************************
 * @fn RetrieveReadPoint
 *
 * @brief Retrieves the saved reading location for the bit processor module.
 *
 * @param[in] None
 * @param[out] None
 *
 * @return None
 *
 ******************************************************************************/
static void RetrieveReadPoint(void)
{
    bitBuffer = savebitBuffer;
    bitLeft = savebitLeft;
    bitBucket = savebitBucket;
}
/**
 *******************************************************************************
 * @fn PeekByte
 *
 * @brief Gets a byte from the current reading location without advancing it.
 *
 * @param[in] None
 * @param[out] None
 *
 * @return a byte from current reading location
 *
 ******************************************************************************/
static U8 PeekByte(void)
{
    return *bitBuffer;
}
/**
 *******************************************************************************
 * @fn SkipBits
 *
 * @brief Advances the current reading location by the number of bits
 * specified.
 *
 * @param[in] length - number of bits to skip
 * @param[out] None
 *
 * @return None
 *
 ******************************************************************************/
static void SkipBits(int length)
{
    if (length < bitLeft)
    {
        bitLeft -= length;
        return;
    }
    length -= bitLeft;
    while (length >= CHAR_BIT)
    {
        bitBuffer++;
        length -= CHAR_BIT;
    }
    bitBucket = *bitBuffer++;
    bitLeft = CHAR_BIT;
    if (length)
    {
        bitLeft -= length;
    }
}
/**
 *******************************************************************************
 * @fn GetBytes
 *
 * @brief Reads a number of bytes (4 max) from the current reading location
 * and advances it.
 *
 * @param[in] length - number of bytes to read
 * @param[out] None
 *
 * @return None
 *
 ******************************************************************************/
static U32 GetBytes(int length)
{
    register unsigned long result = 0;
    // Byte boundary aligned/bitLeft = 0 assumed
    while (length--)
    {
        result = (result << CHAR_BIT) | *bitBuffer++;
    }
    return result;
}
/**
 *******************************************************************************
 * @fn GetBits
 *
 * @brief Reads a number of bits (32 max) from the current reading location
 * and advances it.
 *
 * @param[in] length - number of bits to read
 * @param[out] None
 *
 * @return None
 *
 ******************************************************************************/
U32 GetBits(int length)
{
    register unsigned long result;
    if (length < bitLeft)
    {
        result = (bitBucket & ((1 << bitLeft) - 1)) >> (bitLeft - length);
        bitLeft -= length;
        return result;
    }
    result = bitBucket & ((1 << bitLeft) - 1);
    length -= bitLeft;
    while (length >= CHAR_BIT)
    {
        result = (result << CHAR_BIT) | *bitBuffer++;
        length -= CHAR_BIT;
    }
    bitBucket = *bitBuffer++;
    bitLeft = CHAR_BIT;
    if (length)
    {
        result = (result << length) | (bitBucket >> (CHAR_BIT - length));
        bitLeft -= length;
    }
    return result;
}

#endif
