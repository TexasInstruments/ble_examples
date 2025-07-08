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
 * File : ZipIR_API.h
 * Revision : $Revision: $
 * Date : $Date: $
 * Updated by : $Author: $
 * Description : Header file to export ZipIR functionality.
 *
 ******************************************************************************/

#ifndef _ZIP_IR_API_H_
#define _ZIP_IR_API_H_

#ifdef USE_ZIPIR

/* INCLUDE FILES */
#include "typedefs.h"

/* TYPE DEFINITIONS */
typedef struct
{
    U8 keyFlag;
    U8 keyCounter;
    U8 function;
    U8 dummy; // Required to make *codesetID at even addr
    U8 *codesetId;
    void (*irCallback)(void);
    U32 duration;
} IrInput_t;

typedef struct
{
    U8 versionMajor;
    U8 versionMinor;
    U8 versionRelease;
    U8 codesetIdFormat;
    U8 chipName;
    U8 chipID;
} IrVersion_t;


/* MACRO DEFINITIONS */
// IrInput_t.keyFlag definitions
#define KF_XMIT_TYPE 0x01
#define NORMAL_XMIT 0x00
#define MACRO_XMIT 0x01
// IrVersion_t member definitions
#define VER_MAJOR 1 // IrVersion_t.versionMajor
#define VER_MINOR 9 // IrVersion_t.versionMinor
#define VER_RELEASE 0 // IrVersion_t.versionRelease
#define FORMAT_ZIPIR 0x00 // IrVersion_t.codesetIdFormat
#define CN_GENERIC 0x00 // IrVersion_t.chipName
#define CID_GENERIC 0x00 // IrVersion_t.chipID
// ZipIR_Stop() 'stopCmd' input parameter definitions
#define STOP_IR 0x00 // Stop IR transmission at the
// end of the frame
#define SUSPEND_IR 0x01 // Stop IR transmission right
// away; not yet implemented
// ZipIR_Init() 'gpio' input parameter definitions
#define IR_PIN38 0x00
#define IR_PIN1 0x01
#define IR_PIN4 0x02


/* PUBLIC FUNCTION DECLARATIONS */
void ZipIR_Init(U8 gpio, U8 pwmChannel);
BOOL ZipIR_Active(void);
U8 ZipIR_Transmit(IrInput_t *ir_input);
void ZipIR_Stop(U8 stopCmd);
void ZipIR_StartTimer(U32 duration);
BOOL ZipIR_CheckFunction(U8 *codeset, U8 function);
BOOL ZipIR_GetKeyMap(U8 *codeset, U8 keyMapSize, U8 *keyMap);
void ZipIR_GetVersion(IrVersion_t *irVersion);

#if TOBYDEV_20250318
void IrTxTimerCallback(void);
#endif

#endif // _ZIP_IR_API_H_

#endif
