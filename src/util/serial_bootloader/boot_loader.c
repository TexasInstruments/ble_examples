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
//  Content        : The file holds the main control loop of the boot loader.
//
//-----------------------------------------------------------------------------

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_nvic.h>
#include <inc/hw_ioc.h>
#include <inc/hw_prcm.h>
#include <inc/hw_aon_ioc.h>
#include <inc/hw_aon_rtc.h>
#include <inc/hw_fcfg1.h>
#include <inc/hw_ccfg.h>
#include <driverlib/ioc.h>
#include <blcmds.h>
#include <boot_loader.h>
#include <driverlib/flash.h>

//*****************************************************************************
//
// Define for input parameter to the ConfigurePads() function.
//
//*****************************************************************************
#define BL_INPUT_PINS          true
#define BL_OUTPUT_PINS         false

//*****************************************************************************
//
// Defines for access types used by the COMMAND_MEMORY_READ and
// COMMAND_MEMORY_WRITE.
//
//*****************************************************************************
#define ACC_TYPE_8_BIT         0
#define ACC_TYPE_32_BIT        1

//*****************************************************************************
//
// Forward declaration of static funtions
//
//*****************************************************************************
static uint32_t        CommandInvalid(void);
static void            CommandGetStatus(void);
static void            CommandGetChipId(void);
static void            CommandReset(void);
static uint32_t        CommandPing(void);
static void            CommandCrc32(const uint32_t ui32Size,
                                    const uint8_t *pui8DataBuffer);

static uint32_t        CommandBankErase(void);
static uint32_t        CommandSectorErase(const uint32_t ui32Size,
                                          const uint8_t *pui8DataBuffer);
static uint32_t        CommandSetCcfg(uint32_t ui32Size, const uint8_t *pui8DataBuffer);
static uint32_t        CommandSendData(uint32_t ui32Size,
                                       const uint8_t *pui32DataBuffer);
static uint32_t        CommandDownload(const uint32_t ui32Size,
                                       const uint8_t *pui8DataBuffer);
static uint32_t        CommandMemoryRead(uint32_t ui32Size,
                                         const uint8_t *pui8DataBuffer);
static uint32_t        CommandMemoryWrt(uint32_t ui32Size,
                                        const uint8_t *pui8DataBuffer);
static void            ConfigureIO(uint32_t ui32Interface,
                                   bool bInput);
static void            SelectSSP(void);
static void            SelectUART(void);
static uint32_t        SwapWord(const uint8_t *pui8Data);


//
// Table used by the COMMAND_SET_CCFG command.
//
static const uint32_t ui32SetCcfgTab[NO_OF_CCFG_PARAMS][3] =
{
    // No of    --- Param value ---- Addr offset
    // bytes to --- mask        ---- in CCFG
    // program  ---             ----
    //
    {    4,        0x000000FF,     CCFG_O_CCFG_PROT_31_0                                                      }, // ID_SECTOR_PROT
    {    4,        0xFFFFFFFF,     CCFG_O_IMAGE_VALID_CONF                                                    }, // ID_IMAGE_VALID
    {    1,        0x000000FF,    (CCFG_O_CCFG_TAP_DAP_0   + (CCFG_CCFG_TAP_DAP_0_TEST_TAP_ENABLE_S / 8))     }, // ID_TEST_TAP_LCK
    {    1,        0x000000FF,    (CCFG_O_CCFG_TAP_DAP_0   + (CCFG_CCFG_TAP_DAP_0_PRCM_TAP_ENABLE_S / 8))     }, // ID_PRCM_TAP_LCK
    {    1,        0x000000FF,    (CCFG_O_CCFG_TAP_DAP_0   + (CCFG_CCFG_TAP_DAP_0_CPU_DAP_ENABLE_S / 8))      }, // ID_CPU_DAP_LCK
    {    1,        0x000000FF,    (CCFG_O_CCFG_TAP_DAP_1   + (CCFG_CCFG_TAP_DAP_1_WUC_TAP_ENABLE_S / 8))      }, // ID_WUC_TAP_LCK
    {    1,        0x000000FF,    (CCFG_O_CCFG_TAP_DAP_1   + (CCFG_CCFG_TAP_DAP_1_PBIST1_TAP_ENABLE_S / 8))   }, // ID_PBIST1_TAP_LCK
    {    1,        0x000000FF,    (CCFG_O_CCFG_TAP_DAP_1   + (CCFG_CCFG_TAP_DAP_1_PBIST2_TAP_ENABLE_S / 8))   }, // ID_PBIST2_TAP_LCK
    {    1,        0x00000001,    (CCFG_O_ERASE_CONF       + (CCFG_ERASE_CONF_BANK_ERASE_DIS_N_S / 8))        }, // ID_BANK_ERASE_DIS
    {    1,        0x00000001,    (CCFG_O_ERASE_CONF       + (CCFG_ERASE_CONF_CHIP_ERASE_DIS_N_S / 8))        }, // ID_CHIP_ERASE_DIS
    {    1,        0x000000FF,    (CCFG_O_CCFG_TI_OPTIONS  + (CCFG_CCFG_TI_OPTIONS_TI_FA_ENABLE_S / 8))       }, // ID_TI_BACKDOOR_LCK
    {    1,        0x000000FF,    (CCFG_O_BL_CONFIG        + (CCFG_BL_CONFIG_BL_ENABLE_S / 8))                }, // ID_BL_BACKDOOR_DIS
    {    1,        0x000000FF,    (CCFG_O_BL_CONFIG        + (CCFG_BL_CONFIG_BL_PIN_NUMBER_S / 8))            }, // ID_BL_BACKDOOR_PIN
    {    1,        0x00000001,    (CCFG_O_BL_CONFIG        + (CCFG_BL_CONFIG_BL_LEVEL_S / 8))                 }, // ID_BL_BACKDOOR_LEVEL
    {    1,        0x000000FF,    (CCFG_O_BL_CONFIG        + (CCFG_BL_CONFIG_BOOTLOADER_ENABLE_S / 8))        }  // ID_BL_ENABLE
};

//*****************************************************************************
//
// A pointer to the function that sends data to the selected serial interface.
//
//*****************************************************************************
void (*SendData)(const uint8_t *pui8Data, uint32_t ui32Size);

//*****************************************************************************
//
// A pointer to the function that receives data from the selected serial
// interface.
//
//*****************************************************************************
void (*ReceiveData)(uint8_t *pui8Data, uint32_t ui32Size);

//*****************************************************************************
//
// A pointer to the function that waits for transmit data to be drained.
//
//*****************************************************************************
void (*Flush)(void);

//*****************************************************************************
//
// Holds the current status of the last command that was issued to the boot
// loader.
//
//
//*****************************************************************************
static uint32_t g_ui32Status;

//*****************************************************************************
//
// This holds the current remaining size in bytes to be downloaded.
//
//*****************************************************************************
static uint32_t g_ui32TransferSize;

//*****************************************************************************
//
// This holds the current address that is being written to during a download
// command.
//
//*****************************************************************************
static uint32_t g_ui32TransferAddress;

//*****************************************************************************
//
// This is the data buffer assigned for transfers to the boot loader.
// The actual buffer area being used starts at byte 3 of the first 32-bit
// word of the assigned buffer by later on allocating a pointer to this
// position.
// This makes a received command byte to be stored in byte 3 and the following
// data portion (of a SendData command) to be word aligned.
//
//*****************************************************************************
static uint32_t g_pui32DataBuffer[65];

//*****************************************************************************
//
//! This function selects the serial interface to be used.
//!
//! One of two serial interfaces can be used by the boot loader: SSP and UART.
//!
//! \return None
//
//*****************************************************************************
void
PickInterface(void)
{
    uint32_t ui32Ratio;
    uint32_t ui32Interface;
    uint32_t ui32Delay;

    // Enable power on Peripheral domain; GPIO module (and GPTM, DMA, I2S, SEC)
    HWREG(PRCM_BASE + PRCM_O_PDCTL0PERIPH) = PRCM_PDCTL0PERIPH_ON;

    // Wait for stable power
    while((HWREG(PRCM_BASE + PRCM_O_PDSTAT0) &
            PRCM_PDSTAT0_PERIPH_ON) == 0)
    {
    }

    // Enable GPIO clock
    HWREG(PRCM_BASE + PRCM_O_GPIOCLKGR) = PRCM_GPIOCLKGR_CLK_EN;
    HWREG(PRCM_BASE + PRCM_O_CLKLOADCTL)  = PRCM_CLKLOADCTL_LOAD;

    // Enable power on SSP and UART module (and I2C)
    HWREG(PRCM_BASE + PRCM_O_PDCTL0SERIAL) = PRCM_PDCTL0SERIAL_ON;

    // Wait for stable power
    while((HWREG(PRCM_BASE + PRCM_O_PDSTAT0) &
            PRCM_PDSTAT0_SERIAL_ON) == 0)
    {
    }

	// Enable SSP0 clock
	HWREG(PRCM_BASE + PRCM_O_SSICLKGR) = 1;
	HWREG(PRCM_BASE + PRCM_O_CLKLOADCTL) = PRCM_CLKLOADCTL_LOAD;

	ConfigureIO(SSP_MODE, BL_INPUT_PINS);  // Config SSP input IOs

	// Wait for stable SSP0 clock
	while(!(HWREG(PRCM_BASE + PRCM_O_CLKLOADCTL) & PRCM_CLKLOADCTL_LOAD_DONE))
	{
	}

	// Configure SSP0
	SSPConfig();

    // Setup SysTick as a free-running timer with maximum length, used to time
    // UART edges for the auto-baud detection.
    HWREG(NVIC_ST_RELOAD) = 0xFFFFFF;
    HWREG(NVIC_ST_CTRL)  |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_ENABLE;

    // There is more than one serial interface available for use, so perform
    // autobauding.  If the UART is not used, this will simply wait until
    // SSP is accessed.
    while((ui32Interface =
                UARTAutoBaud(&ui32Ratio)) == 0)
    {
    }

    // See if the SSP interface was accessed.
    if(ui32Interface == SSP_MODE)
    {
        // Configure SSP output pads
        ConfigureIO(SSP_MODE, BL_OUTPUT_PINS);

        // The SSP interface should be used.
        SelectSSP();
    }

    // Otherwise, the UART interface was accessed.
    else if(ui32Interface == UART_MODE)
    {
        // Configure the UART interface pads and UART module.
        HWREG(PRCM_BASE + PRCM_O_UARTCLKGR) = 1;          // Enable UART0 clock
        HWREG(PRCM_BASE + PRCM_O_CLKLOADCTL)  = PRCM_CLKLOADCTL_LOAD;
        ConfigureIO(UART_MODE, BL_INPUT_PINS);
        ConfigureIO(UART_MODE, BL_OUTPUT_PINS);

        // Wait for stable UART0 clock
        while(!(HWREG(PRCM_BASE + PRCM_O_CLKLOADCTL) & PRCM_CLKLOADCTL_LOAD_DONE))
        {
        }

        // Configure UART0
        UARTConfig(ui32Ratio);

        // Turn off the SSP0 clock
        HWREG(PRCM_BASE + PRCM_O_SSICLKGR) = 0;
        HWREG(PRCM_BASE + PRCM_O_CLKLOADCTL) = PRCM_CLKLOADCTL_LOAD;

        // The UART interface should be used.
        SelectUART();

        // Delay a time corresponding to one UART frame (i.e. 10 bits) on the
        // selected UART speed, in order for host to recover from possible
        // framing error that migth ocurre if the CC26xx TX line is floating
        // while host transmits the autobaud pulses, until CC26xx enabled
        // the UART module.
        // The calculation below is based on the formula for calculating the
        // baud rate divisor:
        // Baud rate divisor = UART_Clk / (16 * Baud rate)
        // which equals:
        // Baud rate = UART_Clk / (16 * ((64 * IBRD + FBRD) / 64))
        // which gives:
        // usec pr frame = ((16 * ((64 * IBRD + FBRD) / 64)) / UART_Clk) *
        //                 (1 * 10exp6) * 10
        //               = ((10240 * IBRD + 160 * FBRD) / 64) / UART_Clk_MHz
        //               = ((10240 * (ui32Ratio >> 6) +
        //                   160 * (ui32Ratio & 0x3F)) / 64) / UART_Clk_MHz
        //
        // One loop in the while() below is 3 cycles: 3 * 1/48MHz = 62.5 nsec
        // which gives 16 loops pr. usec
        // Note! UART_Clk_MHz is 48 for silicon and 12 for FPGA
        ui32Delay = (((10240 * (ui32Ratio >> 6) + (160 * (ui32Ratio & 0x3F))) /
                     64) / 48);
        ui32Delay = (ui32Delay + 1) * 16; // Add 1 usec to round up calculation
        while(ui32Delay > 0)
        {
            ui32Delay--;
        }

        // An ack needs to be sent via the UART so that the host knows that
        // autobauding has succeeded.
        AckPacket();
    }
}

//*****************************************************************************
//
//! The main control loop of the boot loader
//!
//! This functions handles the boot loader commands received from the host.
//! This function will never return.
//!
//! \return None
//
//*****************************************************************************
void
Update(void)
{
    uint32_t ui32Size;
    uint8_t *pui8DataBuffer;

    //
    // Initialize status.
    //
    g_ui32Status = COMMAND_RET_SUCCESS;

    //
    // Insure that the COMMAND_SEND_DATA cannot be sent to erase the boot
    // loader before the application is erased.
    //
    g_ui32TransferSize = 0;

    //
    // Offset the data buffer pointer so that the command resides in the final
    // byte of a word, and the data portion of the packet is word aligned.
    //
    pui8DataBuffer = (uint8_t *)((uint32_t)g_pui32DataBuffer + 3);

    //
    // Read any data from the serial port in use.
    //
    while(1)
    {
        //
        // Receive a packet from the port in use.
        //
        if(ReceivePacket(pui8DataBuffer, &ui32Size) != 0)
        {
            continue;
        }

        //
        // The first byte of the data buffer has the command and determines
        // the format of the rest of the bytes.
        //
        switch(pui8DataBuffer[0])
        {
            //
            // This was a simple ping command.
            //
        case COMMAND_PING:
        {
            g_ui32Status = CommandPing();
            break;
        }

        //
        // This command indicates the start of a download sequence.
        //
        case COMMAND_DOWNLOAD:
        {
            g_ui32Status = CommandDownload(ui32Size, pui8DataBuffer);
            break;
        }

        //
        // This command is sent to transfer data to the device following
        // a download command.
        //
        case COMMAND_SEND_DATA:
        {
            g_ui32Status = CommandSendData(ui32Size, pui8DataBuffer);
            break;
        }

        //
        // This command returns the status of the last command that
        // was sent.
        //
        case COMMAND_GET_STATUS:
        {
            CommandGetStatus();
            break;
        }

        //
        // This command returns the Chip ID
        //
        case COMMAND_GET_CHIP_ID:
        {
            CommandGetChipId();
            break;
        }

        //
        // This command is used to reset the device.
        //
        case COMMAND_RESET:
        {
            CommandReset();
            break;
        }

        //
        // This command performs a CRC32 calculation.
        //
        case COMMAND_CRC32:
        {
            CommandCrc32(ui32Size, pui8DataBuffer);
            break;
        }

        //
        // This command will perform sector erase of the flash.
        //
        case COMMAND_SECTOR_ERASE:
        {
            g_ui32Status = CommandSectorErase(ui32Size, pui8DataBuffer);
            break;
        }

        //
        // This command will perform bank erase of the flash.
        //
        case COMMAND_BANK_ERASE:
        {
            g_ui32Status = CommandBankErase();
            break;
        }

        //
        // This command will program a specified parameter within the
        // CCFG flash sector.
        //
        case COMMAND_SET_CCFG:
        {
            g_ui32Status = CommandSetCcfg(ui32Size, pui8DataBuffer);
            break;
        }

        //
        // This command will read and return data from specified memory
        // address area.
        //
        case COMMAND_MEMORY_READ:
        {
            g_ui32Status = CommandMemoryRead(ui32Size, pui8DataBuffer);
            break;
        }

        //
        // This command will write the received data to the specified
        // memory address area.
        //
        case COMMAND_MEMORY_WRITE:
        {
            g_ui32Status = CommandMemoryWrt(ui32Size, pui8DataBuffer);
            break;
        }

        //
        // Acknowledge the command and set the error to indicate that
        // a bad command was sent.
        //
        default:
        {
            g_ui32Status = CommandInvalid();
            break;
        }
        }
    }
}

//*****************************************************************************
//
//! Handles invalid Boot loader command
//!
//! This function will return the appropriate status for an invalid boot loader
//! command received from the host.
//!
//! \return Returns \b COMMAND_RET_UNKNOWN_CMD
//
//*****************************************************************************
static uint32_t
CommandInvalid(void)
{
    //
    // Acknowledge that this command was received correctly. This does not
    // indicate success, just that the command was received.
    //
    AckPacket();

    //
    // Indicate that a bad comand was sent.
    //
    return(COMMAND_RET_UNKNOWN_CMD);
}

//*****************************************************************************
//
//! Handles GET CHIP_ID command
//!
//! This function handles the Get Chip ID command received from the host.
//!
//! \return None
//
//*****************************************************************************
static void
CommandGetChipId(void)
{
    uint32_t ui32Id;
    uint32_t ui32Temp;

    //
    // Acknowledge that this command was received correctly.  This
    // does not indicate success, just that the command was
    // received.
    //
    AckPacket();

    //
    // Get ID. This is the USER_ID written to FCFG1 during production.
    //
    ui32Id = HapiGetChipId();

    //
    // Send the packet.
    //
    ui32Temp = SwapWord((uint8_t *)&ui32Id);
    SendPacket((uint8_t *)&ui32Temp, 4);
}

//*****************************************************************************
//
//! Handles GET STATUS command
//!
//! This function handles the Get Status command received from the host.
//!
//! \return None
//
//*****************************************************************************
static void
CommandGetStatus(void)
{
    //
    // Acknowledge that this command was received correctly.  This
    // does not indicate success, just that the command was
    // received.
    //
    AckPacket();

    //
    // Send the status.
    //
    SendPacket((uint8_t *)&g_ui32Status, 1);
}

//*****************************************************************************
//
//! Handles PING command
//!
//! This function handles the Ping command received from the host. The purpose
//! of this command is just to inform the host that connection is established
//! on the boot loader serial interface.
//!
//! \return Returns \b COMMAND_RET_SUCCESS
//
//*****************************************************************************
static uint32_t
CommandPing(void)
{
    //
    // Just acknowledge that the command was received.
    //
    AckPacket();

    return(COMMAND_RET_SUCCESS);
}

//*****************************************************************************
//
//! Handles CRC32 command
//!
//! \param ui32Size is number of bytes of the received packet
//! \param *pui8DataBuffer is the pointer to the buffer holding the packet
//!
//! This function will calculate the CRC32 checksum over the specified area.
//! The calculated checksum is transferred to host on the boot loader
//! interface. If \b ui32Size has incorrect value the function will return a
//! crc value of 0xFFFFFFFF.
//!
//! \return None
//
//*****************************************************************************
static void
CommandCrc32(const uint32_t ui32Size,
             const uint8_t *pui8DataBuffer)
{
    uint32_t ui32Temp;
    uint32_t ui32MemAddr;
    uint32_t ui32MemSize;
    uint32_t ui32RepeatCnt;
    uint32_t ui32CalculatedCrc;

    ui32CalculatedCrc = 0xFFFFFFFF;

    //
    // Get the start address, size and read repeat count from the command.
    //
    ui32MemAddr = SwapWord(pui8DataBuffer + 1);
    ui32MemSize = SwapWord(pui8DataBuffer + 5);
    ui32RepeatCnt = SwapWord(pui8DataBuffer + 9);

    //
    // See if a full packet was received.
    //
    if(ui32Size == 13)
    {
        //
        // Full packet. Calculate CRC.
        //
        ui32CalculatedCrc = HapiCrc32((uint8_t *)ui32MemAddr,
                                      ui32MemSize,
                                      ui32RepeatCnt);
    }

    //
    // Acknowledge that this command was received.
    //
    AckPacket();

    //
    // Send the calculated crc.
    //
    ui32Temp = SwapWord((uint8_t *)&ui32CalculatedCrc);
    SendPacket((uint8_t *)&ui32Temp, 4);
}

//*****************************************************************************
//
//! Handles SECTOR ERASE command
//!
//! \param ui32Size is number of bytes of the received packet
//! \param *pui8DataBuffer is the pointer to the buffer holding the packet
//!
//! This function will erase the flash sector specified within the command
//! packet.
//!
//! \return Returns \b COMMAND_RET_SUCCESS if command succeeded or
//! \b COMMAND_RET_FLASH_FAIL if erase operation failed or
//! \b COMMAND_RET_INVALID_CMD if invalid command or \b COMMAND_RET_INVALID_ADR
//! if sector address is invalid.
//
//*****************************************************************************
static uint32_t
CommandSectorErase(const uint32_t ui32Size,
                   const uint8_t *pui8DataBuffer)
{
    uint32_t ui32Status;
    uint32_t ui32MemAddr;
    uint32_t ui32FlashSize;

    //
    // Get the sector start address from the command.
    //
    ui32MemAddr   = SwapWord(pui8DataBuffer + 1);
    ui32FlashSize = FlashSizeGet();

    //
    // See if a full packet was received.
    //
    if(ui32Size != 5)
    {
        //
        // Indicate that an invalid command was received.
        //
        ui32Status = COMMAND_RET_INVALID_CMD;
    }

    //
    // Check if sector address is valid for the device.
    //
    else if((ui32MemAddr >= (FLASHMEM_BASE + ui32FlashSize)) ||
            ((ui32MemAddr & (FlashSectorSizeGet() - 1)) != 0))
    {
        //
        // Set the code to an error to indicate that the last
        // command failed. This informs the updater program
        // that the erase command failed.
        //
        ui32Status = COMMAND_RET_INVALID_ADR;
    }
    else
    {
        //
        // Initialize status.
        //
        ui32Status = COMMAND_RET_SUCCESS;

        //
        //  Erase the required flash sector.
        //
        if(HapiSectorErase(ui32MemAddr) != 0)
        {
            ui32Status = COMMAND_RET_FLASH_FAIL;
        }
    }

    //
    // Acknowledge that this command was received correctly. This does not
    // indicate success, just that the command was received.
    //
    AckPacket();

    return(ui32Status);
}

//*****************************************************************************
//
//! Handles BANK ERASE command
//!
//! This function will erase all unprotected main flash bank sectors.
//!
//! \return b\ COMMAND_RET_SUCCESS if command succeeded. If erase operation
//! failed b\ COMMAND_RET_FLASH_FAIL is returned. If command is disabled
//! b\ COMMAND_RET_INVALID_CMD is returned.
//
//*****************************************************************************
static uint32_t
CommandBankErase(void)
{
    uint32_t ui32Status;

    //
    // Check if command is disabled in CCFG with config bit set to 0.
    //
    if(HWREG(FLASH_CFG_BASE + CCFG_OFFSET + CCFG_O_ERASE_CONF) &
            CCFG_ERASE_CONF_BANK_ERASE_DIS_N)
    {
        //
        // Command not disabled. Initialize status.
        //
        ui32Status = COMMAND_RET_SUCCESS;

        //
        // Erase the bank.
        //
        //if(HapiBankErase() != 0)
        {
            ui32Status = COMMAND_RET_FLASH_FAIL;
        }
    }
    else
    {
        //
        // Command disabled. Update status.
        //
        ui32Status = COMMAND_RET_INVALID_CMD;
    }

    //
    // Acknowledge that this command was received correctly.  This
    // does not indicate success, just that the command was
    // received.
    //
    AckPacket();

    return(ui32Status);
}

//*****************************************************************************
//
//! Handles SET CCFG command
//!
//! \param ui32Size is number of bytes of the received packet
//! \param *pui8DataBuffer is the pointer to the buffer holding the packet
//!
//! This function handles the Set CCFG command received from the host. The
//! specified configuration parameter in CCFG (top flash sector) will be
//! programmed.
//!
//! \return b\ COMMAND_RET_SUCCESS if programming succeeded or
//! b\ COMMAND_RET_FLASH_FAIL if programming fail or \b COMMAND_RET_INVALID_ADR
//! if invalid parameter value was transferred from host.
//
//*****************************************************************************
static uint32_t
CommandSetCcfg(uint32_t ui32Size, const uint8_t *pui8DataBuffer)
{
    uint32_t ui32Status;
    uint32_t ui32FieldId;
    uint32_t ui32FieldValue;
    uint8_t pui8ProgBuf[4];
    uint32_t ui32ProgBufSize;
    uint32_t ui32CcfgFieldAddr;
    uint32_t ui32CurrValue;

    ui32Status = COMMAND_RET_SUCCESS;

    //
    // Get the field ID and field ID value.
    //
    ui32FieldId    = SwapWord(pui8DataBuffer + 1);
    ui32FieldValue = SwapWord(pui8DataBuffer + 5);

    //
    // See if a full packet and valid CCFG Id was received.
    //
    if((ui32Size != 9) ||
            (ui32FieldId >= NO_OF_CCFG_PARAMS))
    {
        // Indicate that an invalid command was received.
        ui32Status = COMMAND_RET_INVALID_CMD;
    }
    else
    {
        // Find address to start programming from.
        ui32CcfgFieldAddr = FLASHMEM_BASE + FlashSizeGet() -
                            FlashSectorSizeGet() +
                            ui32SetCcfgTab[ui32FieldId][2];

        // Find number of bytes to be programmed.
        ui32ProgBufSize = ui32SetCcfgTab[ui32FieldId][0];

        // Fill buffer with value to be programmed.
        if(ui32ProgBufSize == 1)
        {
            ui32CurrValue = (uint32_t)(*(uint8_t *)ui32CcfgFieldAddr);
        }
        else
        {
            ui32CurrValue = *(uint32_t *)ui32CcfgFieldAddr;
        }

        *(uint32_t *)pui8ProgBuf =
            ((ui32FieldValue & ui32SetCcfgTab[ui32FieldId][1]) |
             ~ui32SetCcfgTab[ui32FieldId][1]) &
            ui32CurrValue;

        //
        // Check if current parameter ID requires additional handling
        //
        if(ui32FieldId == ID_SECTOR_PROT)
        {
            // Adjust CCFG address to the 32-bit CCFG word holding the
            // protect-bit for the specified sector number.
            ui32CcfgFieldAddr += ((ui32FieldValue >> 5) * 4);

            // Find value to program by setting the protect-bit which
            // corresponds to specified sector number to 0.
            // Leave other protect-bits unchanged.
            *(uint32_t *)pui8ProgBuf = (~(1 << (ui32FieldValue & 0x1F))) &
                                       *(uint32_t *)ui32CcfgFieldAddr;
        }

        //
        // Program specified value into specified CCFG parameter.
        //
        if(HapiProgramFlash(pui8ProgBuf, ui32CcfgFieldAddr, ui32ProgBufSize))
        {
            ui32Status = COMMAND_RET_FLASH_FAIL;
        }
    }

    //
    // Acknowledge that this command was received correctly. This
    // does not indicate success, just that the command was
    // received.
    //
    AckPacket();

    return(ui32Status);
}

//*****************************************************************************
//
//! Handles SEND DATA command
//!
//! \param ui32Size is number of bytes of the received packet
//! \param *pui8DataBuffer is the pointer to the buffer holding the packet
//!
//! This function handles the Send Data command received from the host. The
//! received data will be programmed in NVM memory.
//!
//! \return b\ COMMAND_RET_SUCCESS if programming succeeded or
//! b\ COMMAND_RET_FLASH_FAIL if programming fail or \b COMMAND_RET_INVALID_ADR
//! if too much data was transferred from host.
//
//*****************************************************************************
static uint32_t
CommandSendData(uint32_t ui32Size, const uint8_t *pui8DataBuffer)
{
    uint32_t ui32Status;

    //
    // Set the status to success, to be changed if an error is
    // encountered.
    //
    ui32Status = COMMAND_RET_SUCCESS;

    //
    // Take one byte off for the command.
    //
    ui32Size = ui32Size - 1;

    //
    // Check if there are any more bytes to receive.
    //
    if(g_ui32TransferSize >= ui32Size)
    {
        //
        // Program the bytes into the NVM memory as they are received.
        // For OTP single-bit errors are allowed while multi-bit errors are repaired.
        //
        if(HapiProgramFlash((uint8_t *)(pui8DataBuffer + 1),
                            g_ui32TransferAddress,
                            ui32Size))
        {
            //
            // Indicate that the NVM memory programming failed.
            //
            ui32Status = COMMAND_RET_FLASH_FAIL;
        }
        else
        {
            //
            // Now update the address to program.
            //
            g_ui32TransferSize    -= ui32Size;
            g_ui32TransferAddress += ui32Size;
        }
    }
    else
    {
        //
        // This indicates that too much data is being sent to the
        // device.
        //
        ui32Status = COMMAND_RET_INVALID_ADR;
    }

    //
    // Acknowledge that this command was received correctly.  This
    // does not indicate success, just that the command was
    // received.
    //
    AckPacket();

    return(ui32Status);
}

//*****************************************************************************
//
//! Handles DOWNLOAD command
//!
//! \param ui32Size is number of bytes of the received packet.
//! \param *pui8DataBuffer is the pointer to the buffer holding the packet.
//!
//! This function handles the Download command. This command prepares for
//! programming the flash with data transferred by following Send Data
//! commands. This function checks if the specified flash start address and
//! specified data size is valid for the device.
//!
//! \return Returns b\ COMMAND_RET_SUCCESS if command succeded or
//! b\ COMMAND_RET_INVALID_CMD if command was invalid or
//! b\ COMMAND_RET_INVALID_ADR if specified start address and data size were
//! invalid.
//
//*****************************************************************************
static uint32_t
CommandDownload(const uint32_t ui32Size, const uint8_t *pui8DataBuffer)
{
    uint32_t ui32Status, ui32FlashSize;

    //
    // A simple do/while(0) control loop to make error exits
    // easier.
    //
    do
    {
        //
        // See if a full packet was received.
        //
        if(ui32Size != 9)
        {
            //
            // Setting g_ui32TransferSize to zero makes
            // COMMAND_SEND_DATA fail to accept any more data.
            //
            g_ui32TransferSize = 0;

            //
            // Indicate that an invalid command was received.
            //
            ui32Status = COMMAND_RET_INVALID_CMD;

            //
            // This packet has been handled.
            //
            break;
        }

        //
        // Get the address and size from the command.
        //
        g_ui32TransferAddress = SwapWord(pui8DataBuffer + 1);
        g_ui32TransferSize    = SwapWord(pui8DataBuffer + 5);

        //
        // This determines the size of the flash available.
        //
        ui32FlashSize = FlashSizeGet();

        //
        // Test if the size of the download is too large for the device.
        //
        if((g_ui32TransferAddress                      >= (FLASHMEM_BASE + ui32FlashSize)) ||
                ((g_ui32TransferAddress + g_ui32TransferSize) > (FLASHMEM_BASE + ui32FlashSize)))
        {
            //
            // Setting g_ui32TransferSize to zero makes
            // COMMAND_SEND_DATA fail to accept any more data.
            //
            g_ui32TransferSize = 0;

            //
            // Set the code to an error to indicate that the last
            // command failed.  This informs the updater program
            // that the download command failed.
            //
            ui32Status = COMMAND_RET_INVALID_ADR;

            //
            // This packet has been handled.
            //
            break;
        }

        //
        // Set ok status.
        //
        ui32Status = COMMAND_RET_SUCCESS;

        //
        // This packet has been handled.
        //
        break;
    }
    while(0);

    //
    // Acknowledge that this command was received correctly.  This
    // does not indicate success, just that the command was
    // received.
    //
    AckPacket();

    return(ui32Status);
}

//*****************************************************************************
//
//! Handles RESET command
//!
//! This function handles the Reset command which resets the device.
//! This function will not return.
//!
//! \return None.
//
//*****************************************************************************
static void
CommandReset(void)
{
    //
    // Acknowledge that this command was received correctly.  This
    // does not indicate success, just that the command was
    // received.
    //
    AckPacket();

    //
    // Make sure that the ACK packet has been sent.
    //
    Flush();


    //
    // Perform a system reset of the device.
    //
    HapiResetDevice();
}

//*****************************************************************************
//
//! Handles MEMORY READ command
//!
//! \param ui32Size is number of bytes of the received packet
//! \param *pui8DataBuffer is the pointer to the buffer holding the packet
//!
//! This function handles the Memory Read command received from the host. The
//! read data from specified start address will be returned.
//!
//! \return b\ COMMAND_RET_SUCCESS if memory write was executed or
//! \b COMMAND_RET_INVALID_CMD if invalid packet or \b COMMAND_RET_INVALID_ADR
//! if invalid command parameters.
//
//*****************************************************************************
static uint32_t
CommandMemoryRead(uint32_t ui32Size, const uint8_t *pui8DataBuffer)
{
    uint32_t ui32Status;
    uint32_t ui32StartAddr;
    uint8_t ui8AccType;
    uint8_t ui8NoOfElements;
    uint32_t ui32Index;
    uint8_t pui8ReadData[255];

    ui8AccType = 0;
    ui8NoOfElements = 0;

    //
    // Set the status to success, to be changed if an error is
    // encountered.
    //
    ui32Status = COMMAND_RET_SUCCESS;

    //
    // See if a full packet was received.
    //
    if(ui32Size != 7)
    {
        //
        // Indicate that an invalid command was received.
        //
        ui32Status = COMMAND_RET_INVALID_CMD;
    }
    else
    {
        //
        // Get start address to read
        //
        ui32StartAddr = SwapWord(pui8DataBuffer + 1);

        //
        // Get access type
        //
        ui8AccType = *(pui8DataBuffer + 5);

        //
        // Get number of elements
        //
        ui8NoOfElements = *(pui8DataBuffer + 6);

        //
        // Read from memory if valid command
        //
        if(((ui8AccType == ACC_TYPE_32_BIT) && (ui8NoOfElements <= 63)) ||
                ((ui8AccType == ACC_TYPE_8_BIT) && (ui8NoOfElements <= 253)))
        {
            if(ui8AccType == ACC_TYPE_32_BIT)
            {
                //
                // Read 32-bits elements
                //
                for(ui32Index = 0; ui32Index < ui8NoOfElements; ui32Index++)
                {
                    *((uint32_t *)(pui8ReadData + (ui32Index * 4))) =
                        HWREG(ui32StartAddr + (ui32Index * 4));
                }
            }
            else
            {
                //
                // Read 8-bits elements
                //
                for(ui32Index = 0; ui32Index < ui8NoOfElements; ui32Index++)
                {
                    *(pui8ReadData + ui32Index) = HWREGB(ui32StartAddr + ui32Index);
                }
            }
        }
        else
        {
            //
            // Invalid parameters
            //
            ui32Status = COMMAND_RET_INVALID_ADR;
        }
    }

    //
    // Acknowledge that this command was received.
    //
    AckPacket();

    //
    // Send data if valid command
    //
    if(ui32Status == COMMAND_RET_SUCCESS)
    {
        if(ui8AccType == ACC_TYPE_32_BIT)
        {
            SendPacket(pui8ReadData, (ui8NoOfElements * 4));
        }
        else
        {
            SendPacket(pui8ReadData, ui8NoOfElements);
        }
    }

    //
    // Return status
    //
    return(ui32Status);
}

//*****************************************************************************
//
//! Handles MEMORY WRITE command
//!
//! \param ui32Size is number of bytes of the received packet
//! \param *pui8DataBuffer is the pointer to the buffer holding the packet
//!
//! This function handles the Memory Write command received from the host. The
//! received data will be written from specified start address.
//!
//! \return b\ COMMAND_RET_SUCCESS if memory write was executed or
//! \b COMMAND_INVALID_COMMAND if invalid command parameters.
//
//*****************************************************************************
static uint32_t
CommandMemoryWrt(uint32_t ui32Size, const uint8_t *pui8DataBuffer)
{
    uint32_t ui32Status;
    uint32_t ui32StartAddr;
    uint32_t ui32Index;
    uint32_t ui32Data;
    uint8_t ui8AccType;
    uint8_t ui8Data;

    //
    // Set the status to success, to be changed if an error is
    // encountered.
    //
    ui32Status = COMMAND_RET_SUCCESS;

    //
    // Take one byte off for the command, four for start address and one for
    // access type.
    //
    ui32Size = ui32Size - 6;

    //
    // Get start address to write
    //
    ui32StartAddr = SwapWord(pui8DataBuffer + 1);

    //
    // Get access type
    //
    ui8AccType = *(pui8DataBuffer + 5);

    //
    // Write to memory if valid command
    //
    if((ui8AccType == ACC_TYPE_32_BIT) && (ui32Size <= 244) && (ui32Size >= 4) &&
            ((ui32Size & 0x03) == 0))
    {
        // Do 32-bits write accesses
        for(ui32Index = 0; ui32Index < ui32Size; ui32Index += 4)
        {
            ui32Data = (pui8DataBuffer[9 + ui32Index] << 24) |
                       (pui8DataBuffer[8 + ui32Index] << 16) |
                       (pui8DataBuffer[7 + ui32Index] <<  8) |
                       (pui8DataBuffer[6 + ui32Index]);
            HWREG(ui32StartAddr + ui32Index) = ui32Data;
        }
    }
    else if((ui8AccType == ACC_TYPE_8_BIT) && (ui32Size <= 247) && (ui32Size > 0))
    {
        // Do 8-bit write accesses
        for(ui32Index = 0; ui32Index < ui32Size; ui32Index++)
        {
            ui8Data = (pui8DataBuffer[6 + ui32Index]);
            HWREGB(ui32StartAddr + ui32Index) = ui8Data;
        }
    }
    else
    {
        // Report invalid command
        ui32Status = COMMAND_RET_INVALID_CMD;
    }

    //
    // Acknowledge that this command was received.
    //
    AckPacket();

    //
    // Return status
    //
    return(ui32Status);
}


//*****************************************************************************
//
//! Configures the IOs used by the boot loader.
//!
//! \param ui32Interface specifies the interface to configure.
//! \param bInput selects if input or output signals on the boot loader
//! interface shall be configured.
//!
//! This function configures the IOs used by the boot loader interfaces.
//!
//! \return None
//
//*****************************************************************************
static void
ConfigureIO(uint32_t ui32Interface, bool bInput)
{
    // Avoid IO glitches during IO configuration.
    HWREG(AON_IOC_BASE + AON_IOC_O_IOCLATCH) = 0;
    HWREG(AON_RTC_BASE + AON_RTC_O_SYNC);

    // Configure IOC Signal Select Registers for outputs/inputs to/from pads.
    if(ui32Interface == UART_MODE)
    {
        if(bInput)
        {
            // Configure pad for UART_RX with input enable and pull-up.
            HWREG(IOC_BASE + IOC_O_IOCFG0 + (PIN_UART_RXD * 4)) = IOC_IOCFG0_IE |
                    IOC_IOPULL_UP |
                    IOC_PORT_MCU_UART0_RX;
        }
        else
        {
            // UART interface is to be used.
            // Can not have SSP interface while having UART TX.
            // Reconfigure SSP input pads to their reset values.
            HWREG(IOC_BASE + IOC_O_IOCFG0 + (PIN_SSP_MISO * 4)) = IOC_NO_IOPULL;
            HWREG(IOC_BASE + IOC_O_IOCFG0 + (PIN_SSP_CSN * 4)) = IOC_NO_IOPULL;
            HWREG(IOC_BASE + IOC_O_IOCFG0 + (PIN_SSP_CLK * 4)) = IOC_NO_IOPULL;

            // Disable the SSP interface
            SSPDisableInterface();

            // Configure pad for UART_TX with no pull.
            HWREG(IOC_BASE + IOC_O_IOCFG0 + (PIN_UART_TXD * 4)) = (IOC_NO_IOPULL |
                    IOC_PORT_MCU_UART0_TX) &
                    ~IOC_IOCFG0_IE;
        }
    }
    else if(ui32Interface == SSP_MODE)
    {
        if(bInput)
        {
            // Configure pad for SSP_RX with input enable and pull-up.
            HWREG(IOC_BASE + IOC_O_IOCFG0 + (PIN_SSP_MOSI * 4)) = IOC_IOCFG0_IE |
                    IOC_IOPULL_UP  |
                    IOC_PORT_MCU_SSI0_RX;

            // Configure pad for SSP_FSS with input enable and pull-up.
            HWREG(IOC_BASE + IOC_O_IOCFG0 + (PIN_SSP_CSN * 4)) = IOC_IOCFG0_IE |
                    IOC_IOPULL_UP  |
                    IOC_PORT_MCU_SSI0_FSS;

            // Configure pad for SSP_CLK with input enable and pull-up.
            HWREG(IOC_BASE + IOC_O_IOCFG0 + (PIN_SSP_CLK * 4)) = IOC_IOCFG0_IE |
                    IOC_IOPULL_UP  |
                    IOC_PORT_MCU_SSI0_CLK;
        }
        else
        {
            // Configure pad for SSP_TX with no pull.
            HWREG(IOC_BASE + IOC_O_IOCFG0 + (PIN_SSP_MISO * 4)) =
                (IOC_NO_IOPULL |
                 IOC_PORT_MCU_SSI0_TX) &
                ~IOC_IOCFG0_IE;
        }
    }

    HWREG(AON_IOC_BASE + AON_IOC_O_IOCLATCH) = AON_IOC_IOCLATCH_EN;
    HWREG(AON_RTC_BASE + AON_RTC_O_SYNC);

}

//*****************************************************************************
//
//! Selects SSP as the interface to be used for communication with the host.
//!
//! This function will configure the function pointers controlling the boot
//! loader serial interface to access the SSP functions.
//!
//! \return None
//
//*****************************************************************************
static void
SelectSSP(void)
{
    //
    // Select the SSI send and receive functions so that the remainder of the
    // boot loader's operation occurs via SSI.
    //
    SendData    = SSPSend;
    ReceiveData = SSPReceive;
    Flush       = SSPFlush;
}

//*****************************************************************************
//
//! Selects UART as the interface to be used for communication with the host.
//!
//! This function will configure the function pointers controlling the boot
//! loader serial interface to access the UART functions.
//!
//! \return None
//
//*****************************************************************************
static void
SelectUART(void)
{
    //
    // Select the UART send and receive functions so that the remainder of the
    // boot loader's operation occurs via UART.
    //
    SendData    = UARTSend;
    ReceiveData = UARTReceive;
    Flush       = UARTFlush;
}

//*****************************************************************************
//
//! Converts a word from big endian to little endian.
//!
//! \param pui8Data is a pointer to the data to be converted.
//!
//! Reads a 32-bit value in big endian format and returns it in little endian
//! (i.e. native) format.
//!
//! \return The 32-bit little endian value
//
//*****************************************************************************
static uint32_t
SwapWord(const uint8_t *pui8Data)
{
    //
    // Align the bytes in the buffer into a 32-bit value.
    //
    return((pui8Data[0] << 24) | (pui8Data[1] << 16) | (pui8Data[2] << 8) |
           (pui8Data[3]));
}

//*****************************************************************************
//
//! Checks whether the bootloader should be entered, by checking a pin.
//!
//! \return Whether the bootloader should be entered
//
//*****************************************************************************
int
bootloaderOpened(void)
{
    uint32_t ui32ResetVal;
    uint32_t ui32PinLevel;
    uint32_t ui32BackdoorPin = PIN_BL_ENTER;

    // Enable power on Peripheral domain; GPIO module (and GPTM, DMA, I2S, SEC)
    HWREG(PRCM_BASE + PRCM_O_PDCTL0PERIPH) = PRCM_PDCTL0PERIPH_ON;

    // Wait for stable power
    while((HWREG(PRCM_BASE + PRCM_O_PDSTAT0) &
            PRCM_PDSTAT0_PERIPH_ON) == 0)
    {
    }

    // Enable GPIO clock
    HWREG(PRCM_BASE + PRCM_O_GPIOCLKGR) = PRCM_GPIOCLKGR_CLK_EN;
    HWREG(PRCM_BASE + PRCM_O_CLKLOADCTL)  = PRCM_CLKLOADCTL_LOAD;

    // Configure selected pin pad as input with pull enabled
    ui32ResetVal = HWREG(IOC_BASE + IOC_O_IOCFG0 + (ui32BackdoorPin * 4));
    HWREG(IOC_NONBUF_BASE + IOC_O_IOCFG0 + (ui32BackdoorPin * 4)) =
        IOC_IOCFG0_IE | (PIN_BL_ENTER_ACTIVE?IOC_IOPULL_DOWN:IOC_IOPULL_UP);

    // Delay to let pad stabilize, by executing a read access
    HWREG(GPIO_BASE + GPIO_O_DIN31_0);

    // Read level on backdoor pin
    ui32PinLevel = (HWREG(GPIO_BASE + GPIO_O_DIN31_0) & (1 << ui32BackdoorPin))
                   >> ui32BackdoorPin;

    // Reconfigure selected pin pad to its reset value
    HWREG(IOC_NONBUF_BASE + IOC_O_IOCFG0 + (ui32BackdoorPin * 4)) = ui32ResetVal;

    // Disable GPIO module
    HWREG(PRCM_BASE + PRCM_O_GPIOCLKGR) = 0; // Disable GPIO clock
    HWREG(PRCM_BASE + PRCM_O_CLKLOADCTL) = PRCM_CLKLOADCTL_LOAD; // Load settings
    HWREG(PRCM_BASE + PRCM_O_PDCTL0PERIPH) = 0; // Disable power on GPIO

    while(HWREG(PRCM_BASE + PRCM_O_PDSTAT0) & PRCM_PDSTAT0_PERIPH_ON)
    {
    }

    return ui32PinLevel == PIN_BL_ENTER_ACTIVE;
}
