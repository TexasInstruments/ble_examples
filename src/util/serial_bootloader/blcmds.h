//------------------------------------------------------------------------------
//  Copyright (C) 2011-2016 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//  Content        : The list of commands and return messages supported by the
//                   boot loader.
//
//------------------------------------------------------------------------------
#ifndef __BLCMDS_H__
#define __BLCMDS_H__

//*****************************************************************************
//
//! \defgroup blcmds Boot Loader Commands
//!
//! This section describes the list of commands that can be sent to the boot
//! loader. The first byte of the data in a packet should always be one of the
//! commands defined here followed by data or parameters as determined
//! by the command that is sent.
//!
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! This command is used to receive an acknowledge from the the boot loader
//! proving that communication has been established. This command is a single
//! byte.
//!
//! The format of the buffer is the following:
//!
//! <code>
//! uint8_t Command[1];
//!
//! Data[0] = COMMAND_PING;
//! </code>
//!
//
//*****************************************************************************
#define COMMAND_PING            0x20

//*****************************************************************************
//
//! This command is sent to the boot loader to indicate where to store data
//! and how many bytes will be sent by the COMMAND_SEND_DATA commands that
//! follow. The command consists of two 32-bit values that are both
//! transferred MSB first. The first 32-bit value is the address to start
//! programming data into, while the second is the 32-bit size of the data that
//! will be sent. This command should be followed by a COMMAND_GET_STATUS
//! to ensure that the program address and program size were valid for the
//! microcontroller running the boot loader.
//!
//! The format of the packet to send this command is as follows:
//!
//! <code>
//! uint8_t Command[9];
//!
//! Command[0] = COMMAND_DOWNLOAD;
//!
//! Command[1] = Program Address [31:24];
//!
//! Command[2] = Program Address [23:16];
//!
//! Command[3] = Program Address [15:8];
//!
//! Command[4] = Program Address [7:0];
//!
//! Command[5] = Program Size [31:24];
//!
//! Command[6] = Program Size [23:16];
//!
//! Command[7] = Program Size [15:8];
//!
//! Command[8] = Program Size [7:0];
//! </code>
//
//*****************************************************************************
#define COMMAND_DOWNLOAD        0x21

//*****************************************************************************
//
//! This command returns the status of the last command that was issued.
//! Typically this command should be received after every command is sent to
//! ensure that the previous command was successful or, if unsuccessful, to
//! properly respond to a failure. The command requires one byte in the data
//! of the packet and the boot loader should respond by sending a packet with
//! one byte of data that contains the current status code.
//!
//! The format of the packet that is received is as follows:
//!
//! <code>
//! uint8_t Command[1];
//!
//! Command[0]= COMMAND_GET_STATUS;
//! </code>
//!
//! The following are the definitions for the possible status values that can
//! be returned from the boot loader when <tt>COMMAND_GET_STATUS</tt> is sent
//! to the microcontroller.
//!
//! <code>
//! \#define COMMAND_RET_SUCCESS         0x40
//!
//! \#define COMMAND_RET_UNKNOWN_CMD     0x41
//!
//! \#define COMMAND_RET_INVALID_CMD     0x42
//!
//! \#define COMMAND_RET_INVALID_ADD     0x43
//!
//! \#define COMMAND_RET_FLASH_FAIL      0x44
//! </code>
//!
//
//*****************************************************************************
#define COMMAND_GET_STATUS      0x23

//*****************************************************************************
//
//! This command should only follow a COMMAND_DOWNLOAD command or another
//! COMMAND_SEND_DATA command, if more data is needed. Consecutive send data
//! commands automatically increment the address and continue programming from
//! the previous location. The caller should allow the device to finish
//! the flash programming before issuing another command in order to avoid
//! overflow input buffers of the serial interfaces. The command terminates
//! programming once the number of bytes indicated by the COMMAND_DOWNLOAD
//! command has been received. Each time this function is called, it should be
//! followed by a COMMAND_GET_STATUS command to ensure that the data was
//! successfully programmed into the flash. If the boot loader sends a NAK to
//! this command, the boot loader will not increment the current address to
//! allow retransmission of the previous data.
//!
//! The format of the packet to send this command is as follows:
//!
//! <code>
//! uint8_t Command[9];
//!
//! Command[0] = COMMAND_SEND_DATA
//!
//! Command[1] = Data[0];
//!
//! Command[2] = Data[1];
//!
//! Command[3] = Data[2];
//!
//! Command[4] = Data[3];
//!
//! Command[5] = Data[4];
//!
//! Command[6] = Data[5];
//!
//! Command[7] = Data[6];
//!
//! Command[8] = Data[7];
//! </code>
//
//*****************************************************************************
#define COMMAND_SEND_DATA       0x24

//*****************************************************************************
//
//! This command is used to tell the boot loader to reset. This can be is used
//! after downloading a new image to the device to cause the new application
//! to start from a reset. The normal boot sequence occurs and the image runs
//! as if from a hardware reset.
//! It can also be used to reset the boot loader if a critical error
//! occurs and the host device wants to restart communication with the boot
//! loader.
//!
//! The format of the command is as follows:
//!
//! <code>
//! uint8_t Command[1];
//!
//! Command[0] = COMMAND_RESET;
//! </code>
//!
//! The boot loader responds with an ACK signal to the host device before
//! actually executing the system reset of the device running the
//! boot loader. This informs the updater application that the command was
//! received successfully and the part will be reset.
//
//*****************************************************************************
#define COMMAND_RESET           0x25

//*****************************************************************************
//
//! This command is sent to the boot loader to erase the specified flash sector.
//! The command consists of one 32-bit value that is transferred MSB first.
//! This 32-bit value is the start address of the flash sector to be erased.
//!
//! The format of the packet to send this command is as follows:
//!
//! <code>
//! uint8_t Command[5];
//!
//! Command[0] = COMMAND_SECTOR_ERASE;
//!
//! Command[1] = Memory Address [31:24];
//!
//! Command[2] = Memory Address [23:16];
//!
//! Command[3] = Memory Address [15:8];
//!
//! Command[4] = Memory Address [7:0];
//!
//! </code>
//
//*****************************************************************************
#define COMMAND_SECTOR_ERASE    0x26

//*****************************************************************************
//
//! This command is sent to the boot loader to get the calculated CRC32 for a
//! specified memory area. The command consists of three 32-bit values that are
//! both transferred MSB first. The first 32-bit value is the memory address to
//! start the CRC calculation from, the second is the 32-bit size of the
//! memory area that shall be included in the CRC calculation, while the third
//! is the number of repeated reads of the memory addresses. For a normal CRC
//! calculation this value should be set to 0.
//!
//! The format of the packet to send this command is as follows:
//!
//! <code>
//! uint8_t Command[13];
//!
//! Command[0] = COMMAND_CRC32;
//!
//! Command[1] = Memory Address [31:24];
//!
//! Command[2] = Memory Address [23:16];
//!
//! Command[3] = Memory Address [15:8];
//!
//! Command[4] = Memory Address [7:0];
//!
//! Command[5] = Memory Area Size [31:24];
//!
//! Command[6] = Memory Area Size [23:16];
//!
//! Command[7] = Memory Area Size [15:8];
//!
//! Command[8] = Memory Area Size [7:0];
//!
//! Command[9] = Repeat Count [31:24];
//!
//! Command[10] = Repeat Count [23:16];
//!
//! Command[11] = Repeat Count [15:8];
//!
//! Command[12] = Repeat Count [7:0];
//! </code>
//
//*****************************************************************************
#define COMMAND_CRC32           0x27

//*****************************************************************************
//
//! This command is sent to the boot loader to get the Usesr ID of the device.
//!
//! The format of the packet to send this command is as follows:
//!
//! <code>
//! uint8_t Command[1];
//!
//! Command[0] = COMMAND_GET_CHIP_ID;
//!
//! </code>
//
//*****************************************************************************
#define COMMAND_GET_CHIP_ID     0x28

//*****************************************************************************
//
//! This command is sent to the boot loader to read data from a specified
//! memory area. The command consists of one 32-bit value that is transferred
//! MSB first holding the read start address within the memory map.
//! A second 8-bit value specifies the read access width (0=8-bits/1=32-bits).
//! A third 8-bit value specifies number of read accesses. Max valid value is
//! 63 for 32-bits access width and 253 for 8-bit access width.
//!
//! The format of the packet to send this command is as follows:
//!
//! <code>
//! uint8_t Command[7];
//!
//! Command[0] = COMMAND_MEMORY_READ;
//!
//! Command[1] = Address [31:24];
//!
//! Command[2] = Address [23:16];
//!
//! Command[3] = Address [15:8];
//!
//! Command[4] = Address [7:0];
//!
//! Command[5] = Access Type [7:0];
//!
//! Command[6] = Number of accesses [7:0];
//!
//! </code>
//
//*****************************************************************************
#define COMMAND_MEMORY_READ    0x2A

//*****************************************************************************
//
//! This command is sent to the boot loader to write data to a specified
//! memory area. The command consists of one 32-bit value that is transferred
//! MSB first holding the write start address within the memory map.
//! A following 8-bit value specifies the write access width (0=8-bit/1=32-bit).
//! The number of data bytes received is given by the packet size byte. Max
//! number of data bytes for access width 0 is 249 and 248 for access width 1.
//!
//! The format of the packet to send this command is as follows:
//!
//! <code>
//! uint8_t Command[6 + no of data bytes];
//!
//! Command[0] = COMMAND_MEMORY_WRITE;
//!
//! Command[1] = Address [31:24];
//!
//! Command[2] = Address [23:16];
//!
//! Command[3] = Address [15:8];
//!
//! Command[4] = Address [7:0];
//!
//! Command[5] = Access Type [7:0];
//!
//! Command[6] = Data [7:0];
//!     .
//!     .
//!     .
//! Command[6 + (packet size - 7)] = Data [7:0] or Data[31:24];
//!
//! </code>
//
//*****************************************************************************
#define COMMAND_MEMORY_WRITE   0x2B

//*****************************************************************************
//
//! This command is used to perform a bank erase of the on chip flash. All
//! main flash bank sectors not protected by FCFG1 and CCFG protect bits will
//! be erased.
//!
//! The format of the command is as follows:
//!
//! <code>
//! uint8_t Command[1];
//!
//! Command[0] = COMMAND_BANK_ERASE;
//! </code>
//!
//
//*****************************************************************************
#define COMMAND_BANK_ERASE      0x2C

//*****************************************************************************
//
//! This command is sent to the boot loader to program a specified value into
//! a specified parameter within the CCFG flash sector. The command consists
//! of two 32-bit values that are both transferred MSB first.
//! The first 32-bit value is the ID value of the wanted CCFG parameter to
//! program and the second is the 32-bit value of the CCFG parameter.
//! Dependent on the size of the CCFG parameter that is requested to be
//! programmed one or more (max 4) bytes of the input value parameter is
//! programmed.
//! Only the CCFG field read during ROM boot can be modified by this command.
//!
//! The format of the packet to send this command is as follows:
//!
//! <code>
//! uint8_t Command[9];
//!
//! Command[0] = COMMAND_SET_CCFG;
//!
//! Command[1] = Field Id [31:24];
//!
//! Command[2] = Field Id [23:16];
//!
//! Command[3] = Field Id [15:8];
//!
//! Command[4] = Field Id [7:0];
//!
//! Command[5] = Field Value [31:24];
//!
//! Command[6] = Field Value [23:16];
//!
//! Command[7] = Field Value [15:8];
//!
//! Command[8] = Field Value [7:0];
//!
//! </code>
//
//*****************************************************************************
#define COMMAND_SET_CCFG        0x2D

//*****************************************************************************
//
//! This command is sent to the boot loader to brick the device.
//! The command consists of one 32-bit value that is transferred MSB first.
//! This 32-bit value is the key required to enable the brick operation.
//!
//! The format of the packet to send this command is as follows:
//!
//! <code>
//! uint8_t Command[5];
//!
//! Command[0] = COMMAND_BRICK_DEVICE;
//!
//! Command[1] = Key [31:24];
//!
//! Command[2] = Key [23:16];
//!
//! Command[3] = Key [15:8];
//!
//! Command[4] = Key [7:0];
//!
//! </code>
//
//*****************************************************************************
#define COMMAND_BRICK_DEVICE    0x2E

//*****************************************************************************
//
//! This is returned in response to a COMMAND_GET_STATUS command and indicates
//! that the previous command completed successful. 
//
//*****************************************************************************
#define COMMAND_RET_SUCCESS     0x40

//*****************************************************************************
//
//! This is returned in response to a COMMAND_GET_STATUS command and indicates
//! that the command sent was an unknown command.
//
//*****************************************************************************
#define COMMAND_RET_UNKNOWN_CMD 0x41

//*****************************************************************************
//
//! This is returned in response to a COMMAND_GET_STATUS command and indicates
//! that the previous command was formatted incorrectly.
//
//*****************************************************************************
#define COMMAND_RET_INVALID_CMD 0x42

//*****************************************************************************
//
//! This is returned in response to a COMMAND_GET_STATUS command and indicates
//! that the previous download command contained an invalid address value.
//
//*****************************************************************************
#define COMMAND_RET_INVALID_ADR 0x43

//*****************************************************************************
//
//! This is returned in response to a COMMAND_GET_STATUS command and indicates
//! that an attempt to program or erase the flash has failed.
//
//*****************************************************************************
#define COMMAND_RET_FLASH_FAIL  0x44

//*****************************************************************************
//
//! This is the value that is sent to acknowledge a packet.
//
//*****************************************************************************
#define COMMAND_ACK             0xcc

//*****************************************************************************
//
//! This is the value that is sent to not-acknowledge a packet.
//
//*****************************************************************************
#define COMMAND_NAK             0x33

//*****************************************************************************
//
//! Defines for the Field ID parameter used by the COMMAND_SET_CCFG command.
//
//*****************************************************************************
enum{
        ID_SECTOR_PROT = 0x00000000,
        ID_IMAGE_VALID,
        ID_TEST_TAP_LCK,
        ID_PRCM_TAP_LCK,
        ID_CPU_DAP_LCK,
        ID_WUC_TAP_LCK,
        ID_PBIST1_TAP_LCK,
        ID_PBIST2_TAP_LCK,
        ID_BANK_ERASE_DIS,
        ID_CHIP_ERASE_DIS,
        ID_TI_BACKDOOR_LCK,
        ID_BL_BACKDOOR_DIS,
        ID_BL_BACKDOOR_PIN,
        ID_BL_BACKDOOR_LEVEL,
        ID_BL_ENABLE,
        NO_OF_CCFG_PARAMS
    };

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

#endif // __BLCMDS_H__
