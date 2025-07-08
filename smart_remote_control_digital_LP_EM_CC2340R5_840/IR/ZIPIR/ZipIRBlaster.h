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
 * File : ZipIRBlaster.h
 * Revision : $Revision: $
 * Date : $Date: $
 * Updated by : $Author: $
 * Description :
 *
 ******************************************************************************/

#ifdef USE_ZIPIR

#ifndef _ZIP_IR_BLASTER_H_
#define _ZIP_IR_BLASTER_H_
/* MACRO DEFINITIONS */
#define MAX_IRTX_SIZE 512
#define APPIR_HIGH 0x8000 // Bit15 = 1: bit 15 set to one to
// indicate logic high in data
// duration
#define APPIR_LOW 0x0000 // Bit15 = 0: bit 15 set to zero to
// indicate logic low in data
// duration
#define MACRO_IR_DURATION_INTERVAL 260 // 260 ms
#define MAX_IR_DURATION_INTERVAL 600000 // 10 minutes
// Timing adjustments for end pulse and code gap
#define ADJUST_TIME 200 // 200 us to compensate for the time
// Conditional compilation switch – uncomment to truncate final frame timing to
// compensate for potential transmission callback delay
//#define CALLBACK_DELAY
// Conditional compilation switch - uncomment to allow 0-duration Characteristic
// Times to be skipped over, effectively concatenating the preceding and succeeding
// Characteristic Times without toggling between Mark and Space
//#define SKIP_0_TIMINGS
// irState definitions
enum
{
    UEIRTX_IDLE = 0, UEIRTX_STARTED, UEIRTX_REPEATING, UEIRTX_END
};
// irState2 definitions
enum
{
    IS2_WAIT_FOR_IDLE = 0x01,
    IS2_READY_TO_XMIT = 0x02,
    IS2_STOP_COMMAND = 0x04,
    IS2_REPEAT_FLAG = 0x08,
    IS2_IRTX_ACTIVE = 0x10
};
// IR frame definitions
enum
{
    IR_MAKE_FRAME, IR_REPEAT_FRAME, IR_BREAK_FRAME
};
#endif // _ZIP_IR_BLASTER_H_

#endif
