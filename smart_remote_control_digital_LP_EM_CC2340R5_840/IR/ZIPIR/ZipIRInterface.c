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
 * File : ZipIRInterface.c
 * Revision : $Revision: $
 * Date : $Date: $
 * Updated by : $Author: $
 * Description : Utilities for searching ZIP code set from the database and
 * identify the functions picked.
 ******************************************************************************/
#ifdef USE_ZIPIR
/* INCLUDE FILES */
#include "ZipIR_API.h"
#include "ZipIRInterface.h"

/* LOCAL FUNCTION DEFINITIONS */
static U8 BitProcessor(U8 **source, U8 position, S16 size, U8 *dest,
                       U8 dataType);

/**
 *******************************************************************************
 * @fn ZipIR_CheckFunction
 *
 * @brief Checks whether the specified function is picked.
 *
 * @param[in] codeset - codeset location in RAM
 * @param[in] function - function to be checked; expressed as function offset
 * @return TRUE = Function is picked
 * FALSE = Function is not picked.
 *
 ******************************************************************************/
BOOL ZipIR_CheckFunction(U8 *codeset, U8 function)
{
    U8 BWS, SQC, byte, index, size, prevPos, i, j, k;
    U16 word;
    U32 dummy;
    U8 *tmpPtr, *ptrLimit;
    BOOL status = FALSE;
    if (codeset == NULL)
    {
        return FALSE;
    }
    if (ZSF_FILE_ID == codeset[0])
    {
        return (0 == function) ? TRUE : FALSE;
    }
    // Get the codeset size
    dummy = codeset[ZCL_FSZ_OFS];
    dummy = (dummy << 8) | codeset[ZCL_FSZ_OFS + 1];
    dummy = (dummy << 8) | codeset[ZCL_FSZ_OFS + 2];
    dummy = (dummy << 8) | codeset[ZCL_FSZ_OFS + 3];
    // Point just past the last codeset byte
    ptrLimit = codeset + dummy;
    // Get the offset from ZCS to ZFT
    word = ((U16) codeset[ZCL_FTO_OFS] << 8) + (U16) codeset[ZCL_FTO_OFS + 1];
    // Get the offset from start of ZCL to ZFT
    word += ZCS_SECT_OFS;
    // Point to the ZFT area
    tmpPtr = codeset + word;
    // Extract BWS|SQC byte
    byte = *tmpPtr++;
    // Size of waveform substitutions (in # of bits)
    BWS = byte >> 4;
    // # of transmission sequences
    SQC = (byte & 0x0F) + 1;
    index = 0;
    prevPos = 8;
    while (tmpPtr < ptrLimit)
    {
        // Function found?
        if (index == function)
        {
            prevPos = BitProcessor(&tmpPtr, prevPos, WSP_SIZE, &byte,
                                   sizeof(byte));
            if (byte & WSP_MASK)
            {
                status = TRUE;
            }
            break;
        }
        for (k = 0; k < SQC; k++)
        {
            // Get waveform substitution present (WSP) - 3 bits
            prevPos = BitProcessor(&tmpPtr, prevPos, WSP_SIZE, &byte,
                                   sizeof(byte));
            // Calculate offset to the next function
            for (i = j = 0; i < 3; i++)
            {
                if (byte & 0x01)
                {
                    j++;
                }
                byte >>= 1;
            }
            size = j * BWS;
            // Offset to the next function
            prevPos = BitProcessor(&tmpPtr, prevPos, size, (U8*) &dummy,
                                   sizeof(dummy));
        }
        index++;
    }
    return status;
}

/**
 *******************************************************************************
 * @fn BitProcessor
 *
 * @brief Extracts the data bits from a specified location and position.
 *
 * @param[in] source – pointer to the input buffer; updated dynamically
 * @param[in] position - bit position
 * @param[in] size - data size (# of bits)
 * @param[out] dest – output data buffer
 * @param[in] dataType - # of bytes to be output
 *
 * @return Current bit position
 *
 ******************************************************************************/
static U8 BitProcessor(U8 **source, U8 position, S16 size, U8 *dest,
                       U8 dataType)
{
    U8 tmpBuf[4], bit = 0x01, i = 0, index, bitPosition;
    U8 bitAdjust = (8 - position + size) % 8;
    U8 *data = *source + (8 - position + size - 1) / 8;
    if (bitAdjust)
    {
        bitAdjust = 8 - bitAdjust;
    }
    index = (size - 1) / 8;
    bitPosition = 0x01 << bitAdjust;
    tmpBuf[index] = 0;
    while (i++ < size)
    {
        if (*data & bitPosition)
        {
            tmpBuf[index] |= bit;
        }
        bitPosition <<= 1;
        position--;
        if (!bitPosition)
        {
            bitPosition = 0x01;
            data--;
        }
        if (!position)
        {
            (*source)++;
            position = 8;
        }
        if (i % 8 == 0)
        {
            bit = 0x01;
            if (index)
            {
                index--;
                tmpBuf[index] = 0;
            }
        }
        else
        {
            bit <<= 1;
        }
    }
    switch (dataType)
    {
    case sizeof(U8):
        *dest = tmpBuf[0];
        break;
    case sizeof(U16):
        *dest = tmpBuf[1];
        *(dest + 1) = tmpBuf[0];
        break;
    case sizeof(U32):
        *dest = tmpBuf[3];
        *(dest + 1) = tmpBuf[2];
        *(dest + 2) = tmpBuf[1];
        *(dest + 3) = tmpBuf[0];
        break;
    default:
        break;
    }
    return position;
}

/**
 *******************************************************************************
 * @fn ZipIR_GetKeyMap
 *
 * @brief Scans the specified code set and lists all the keys having
 * IR functions assigned.
 *
 * @param[in] codeset – address of code set in RAM
 * @param[in] keyMapSize - max number of key functions to be stored in keyMap
 * @param[out] keyMap – output buffer where the key functions will be stored;
 * keyMap[0] = # of functions found
 *
 * @return TRUE = the function was found within the codeset.
 * FALSE = the function was not found.
 *
 ******************************************************************************/
BOOL ZipIR_GetKeyMap(U8 *codeset, U8 keyMapSize, U8 *keyMap)
{
    U8 BWS, SQC, byte, index, size, prevPos, i, j, k, keyCount;
    U16 word;
    U32 dummy;
    U8 *tmpPtr, *ptrLimit;
    if ((codeset == NULL) || (keyMap == NULL))
    {
        return FALSE;
    }
    // Get the codeset size
    dummy = codeset[ZCL_FSZ_OFS];
    dummy = (dummy << 8) | codeset[ZCL_FSZ_OFS + 1];
    dummy = (dummy << 8) | codeset[ZCL_FSZ_OFS + 2];
    dummy = (dummy << 8) | codeset[ZCL_FSZ_OFS + 3];
    // Point just past the last codeset byte
    ptrLimit = codeset + dummy;
    // Get the offset from ZCS to ZFT
    word = ((U16) codeset[ZCL_FTO_OFS] << 8) + (U16) codeset[ZCL_FTO_OFS + 1];
    // Get the offset from start of ZCL to ZFT
    word += ZCS_SECT_OFS;
    // Point to the ZFT area
    tmpPtr = codeset + word;
    // Extract BWS\SQC byte
    byte = *tmpPtr++;
    // Size of waveform substitutions (in # of bits)
    BWS = byte >> 4;
    // # of transmission sequences
    SQC = (byte & 0x0F) + 1;
    index = keyCount = 0;
    prevPos = 8;
    // Update key mapping
    while (tmpPtr < ptrLimit)
    {
        // Get waveform substitution present (WSP) - 3 bits
        prevPos = BitProcessor(&tmpPtr, prevPos, WSP_SIZE, &byte, sizeof(byte));
        // Check whether function is present
        if (byte & WSP_MASK)
        {
            if (keyCount < keyMapSize - 1)
            {
                // Save function found to key map array
                *(keyMap + index + 1) = index;
            }
            keyCount++;
        }
        for (k = 0; k < SQC; k++)
        {
            // Calculate offset to the next function
            for (i = j = 0; i < 3; i++)
            {
                if (byte & 0x01)
                {
                    j++;
                }
                byte >>= 1;
            }
            size = j * BWS;
            // Offset to the next function
            prevPos = BitProcessor(&tmpPtr, prevPos, size, (U8*) &dummy,
                                   sizeof(dummy));
        }
        index++;
    }
    // Record total functions found
    *keyMap = keyCount;
    if (keyCount)
    {
        return TRUE;
    }
    return FALSE;
}

/**
 *******************************************************************************
 * @fn ZipIR_GetVersion
 *
 * @brief Retrieves the IR API version information.
 *
 * @param[out] Version Major
 * @param[out] Version Minor
 * @param[out] Version Released
 * @param[out] code set ID format
 * @param[out] Chip name
 * @param[out] Chip ID
 *
 * @return None
 *
 ******************************************************************************/
void ZipIR_GetVersion(IrVersion_t *irVersion)
{
    if (irVersion != NULL)
    {
        irVersion->versionMajor = VER_MAJOR;
        irVersion->versionMinor = VER_MINOR;
        irVersion->versionRelease = VER_RELEASE;
        irVersion->codesetIdFormat = FORMAT_ZIPIR;
        irVersion->chipName = CN_GENERIC;
        irVersion->chipID = CID_GENERIC;
    }
}
#endif
