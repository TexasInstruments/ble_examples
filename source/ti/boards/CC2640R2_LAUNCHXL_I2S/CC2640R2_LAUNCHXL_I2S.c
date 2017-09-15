/*
 * Copyright (c) 2016, Texas Instruments Incorporated
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
 *  ====================== CC2640R2_LAUNCHXL_I2S.c ===========================
 *  This board file is an addendum to CC2640R2_LAUNCHXL.c. It adds an I2S
 *  object to interface the CC2640R2F and a booster pack.
 *  ============================================================================
 */

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

#include "CC2640R2_LAUNCHXL_I2S.h"
/*
 *============================= I2S begin =====================================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(I2SCC26XX_config, ".const:I2SCC26XX_config")
#pragma DATA_SECTION(i2sCC26XXHWAttrs, ".const:i2sCC26XXHWAttrs")
#endif

#include <ti/drivers/i2s/I2SCC26XX.h>

I2SCC26XX_Object i2sCC26XXObject;

const I2SCC26XX_HWAttrs i2sCC26XXHWAttrs = {
    .baseAddr = I2S0_BASE,
    .intNum = INT_I2S_IRQ,
    .intPriority = ~0,
    .powerMngrId = PowerCC26XX_PERIPH_I2S,
    .mclkPin = CC2640R2_LAUNCHXL_I2S_MCLK,
    .bclkPin = CC2640R2_LAUNCHXL_I2S_BCLK,
    .wclkPin = CC2640R2_LAUNCHXL_I2S_WCLK,
    .ad0Pin = CC2640R2_LAUNCHXL_I2S_ADO,
    .ad1Pin = CC2640R2_LAUNCHXL_I2S_ADI,
};

/* I2S configuration structure */
const I2SCC26XX_Config I2SCC26XX_config[] = {
    {
        .object = &i2sCC26XXObject,
        .hwAttrs = &i2sCC26XXHWAttrs
    },
    {NULL, NULL}
};
