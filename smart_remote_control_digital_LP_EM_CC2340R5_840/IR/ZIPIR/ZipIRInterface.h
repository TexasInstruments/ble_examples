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
* File : ZipIRInterface.h
* Revision : $Revision: $
* Date : $Date: $
* Updated by : $Author: $
* Description : Zip IR utility header file
*
******************************************************************************/
#ifdef USE_ZIPIR
#ifndef _ZIP_IR_INTERFACE_H_
#define _ZIP_IR_INTERFACE_H_

/* MACRO DEFINITIONS */

// ZCL Definitions
#define ZCL_VER_OFS 0 // 0
#define ZCL_FSZ_OFS (1 + ZCL_VER_OFS) // 1
#define ZCL_FCD_OFS (4 + ZCL_FSZ_OFS) // 5
#define ZCL_TC_OFS (4 + ZCL_FCD_OFS) // 9
#define ZCL_DT_OFS (1 + ZCL_TC_OFS) // 10
#define ZCL_DCC_OFS (1 + ZCL_DT_OFS) // 11
#define ZCL_DCSI_OFS (2 + ZCL_DCC_OFS) // 13
#define ZCL_CC_OFS (5 + 2 + ZCL_DCSI_OFS) // 20
#define ZCL_CID_OFS (2 + ZCL_CC_OFS) // 22
#define ZCL_CSS_OFS (2 + ZCL_CID_OFS) // 24
#define ZCL_FTO_OFS (4 + ZCL_CSS_OFS) // 28
#define ZCS_SECT_OFS (8 + 2 + ZCL_FTO_OFS) // 38

// ZFT Definitions
#define ZFT_BWS_SQC_OFS 0
#define ZFT_FUNCTION_OFS ZFT_BWS_SQC_OFS
#define WSP_SIZE 3
#define WSP_MASK 0x07

// ZSF definitions
#define ZSF_FILE_ID 0x82
#define ZSF_FILE_ID_OFS 0 // 0
#define ZSF_DEVICE_ID_OFS (1 + ZSF_FILE_ID_OFS) // 1
#define ZSF_ZCS_SIZE_M1_OFS (1 + ZSF_DEVICE_ID_OFS) // 2
#define ZSF_ZCS_OFS (1 + ZSF_ZCS_SIZE_M1_OFS) // 3

#endif // _ZIP_IR_INTERFACE_H_

#endif
