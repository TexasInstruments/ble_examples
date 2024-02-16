/*
 * Copyright (c) 2021-2023, Texas Instruments Incorporated
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

/****************************************************************
 * Function Name : openFile
 * Description   : Opens the file
 * Returns       : NULL on failure
 * Params        @file: Path to the file to be opened
 ****************************************************************/
FILE *openFile(const char *file)
{
    FILE *fp = fopen(file, "rb");
    return(fp);
}
/****************************************************************
 * Function Name : openFileCcfg
 * Description   : Opens the CCFG-file
 * Returns       : NULL on failure
 * Params        @file: Path to the file to be opened
 ****************************************************************/
FILE *openFileCcfg(const char *file)
{

    /*open file*/ 
    printf("OpenCCFGfile");
    FILE *fp = fopen(file, "w+b");
    return(fp);
    
}


/****************************************************************
 * Function Name : getFileSize
 * Description   : Gets the size of the file to be read
 * Returns       : 0 on failure
 * Params        @fp: File descriptor
 ****************************************************************/
long int getFileSize(FILE *fp)
{
    /* Go to end of file */
    if(fseek(fp, 0L, SEEK_END))
        return (0);

    /* Get the size */
    long int sz = ftell(fp);

    /* Put the curser back to 0 */
    if(fseek(fp,0L,SEEK_SET))
        return (0);
    return sz;
}

/****************************************************************
 * Function Name : closeFile
 * Description   : closes the file
 * Returns       : EOF on failure
 * Params        @fp: file descriptor
 ****************************************************************/
int closeFile(FILE *fp)
{
    return(fclose(fp));
}

