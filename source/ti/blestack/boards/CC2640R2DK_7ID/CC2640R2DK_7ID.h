/*
 * Copyright (c) 2017, Texas Instruments Incorporated
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

/** ============================================================================
 *  @file       CC2640R2DK_7ID.h
 *
 *  @brief      CC2640R2DK_7ID Board Specific header file.
 *
 *  The CC2640R2DK_7ID header file should be included in an application as
 *  follows:
 *  @code
 *  #include "CC2640R2DK_7ID.h"
 *  @endcode
 *
 *  This board file is made for the 7x7 mm QFN package, to convert this board
 *  file to use for other smaller device packages please refer to the table
 *  below which lists the max IOID values supported by each package. All other
 *  unused pins should be set to IOID_UNUSED.
 *
 *  Furthermore the board file is also used
 *  to define a symbol that configures the RF front end and bias.
 *  See the comments below for more information.
 *  For an in depth tutorial on how to create a custom board file, please refer
 *  to the section "Running the SDK on Custom Boards" with in the Software
 *  Developer's Guide.
 *
 *  Refer to the datasheet for all the package options and IO descriptions:
 *  http://www.ti.com/lit/ds/symlink/cc2640r2f.pdf
 *
 *  +-----------------------+------------------+-----------------------+
 *  |     Package Option    |  Total GPIO Pins |   MAX IOID            |
 *  +=======================+==================+=======================+
 *  |     7x7 mm QFN        |     31           |   IOID_30             |
 *  +-----------------------+------------------+-----------------------+
 *  |     5x5 mm QFN        |     15           |   IOID_14             |
 *  +-----------------------+------------------+-----------------------+
 *  |     4x4 mm QFN        |     10           |   IOID_9              |
 *  +-----------------------+------------------+-----------------------+
 *  |     2.7 x 2.7 mm WCSP |     14           |   IOID_13             |
 *  +-----------------------+------------------+-----------------------+
 *  ============================================================================
 */
#ifndef __CC2640R2DK_7ID_BOARD_H__
#define __CC2640R2DK_7ID_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include <ti/drivers/PIN.h>
#include <ti/devices/cc26x0r2/driverlib/ioc.h>

/* Externs */
extern const PIN_Config BoardGpioInitTable[];

/* Defines */
#ifndef CC2640R2DK_7ID
  #define CC2640R2DK_7ID
#endif /* CC2640R2DK_7ID */

/*
 *  ============================================================================
 *  RF Front End and Bias configuration symbols for TI reference designs and
 *  kits. This symbol sets the RF Front End configuration in ble_user_config.h
 *  and selects the appropriate PA table in ble_user_config.c.
 *  Other configurations can be used by editing these files.
 *
 *  Define only one symbol:
 *  CC2650EM_7ID    - Differential RF and internal biasing
                      (default for CC2640R2 LaunchPad)
 *  CC2650EM_5XD    – Differential RF and external biasing
 *  CC2650EM_4XS    – Single-ended RF on RF-P and external biasing
 *  CC2640R2DK_CXS  - WCSP: Single-ended RF on RF-N and external biasing
 *                    (Note that the WCSP is only tested and characterized for
 *                     single ended configuration, and it has a WCSP-specific
 *                     PA table)
 *
 *  Note: CC2650EM_xxx reference designs apply to all CC26xx devices.
 *  ==========================================================================
 */
#define CC2650EM_7ID

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>                <pin mapping>
 */

/* Accelerometer */
#define CC2640R2DK_7ID_ACC_PWR                       IOID_20         /* P2.8 */
#define CC2640R2DK_7ID_ACC_CSN                       IOID_24         /* P2.10 */

/* Ambient Light Sensor */
#define CC2640R2DK_7ID_ALS_OUT                       IOID_23         /* P2.5 */
#define CC2640R2DK_7ID_ALS_PWR                       IOID_26         /* P2.6 */

/* Analog Capable DIO's */
#define CC2640R2DK_7ID_DIO23_ANALOG                  IOID_23
#define CC2640R2DK_7ID_DIO24_ANALOG                  IOID_24
#define CC2640R2DK_7ID_DIO25_ANALOG                  IOID_25
#define CC2640R2DK_7ID_DIO26_ANALOG                  IOID_26
#define CC2640R2DK_7ID_DIO27_ANALOG                  IOID_27
#define CC2640R2DK_7ID_DIO28_ANALOG                  IOID_28
#define CC2640R2DK_7ID_DIO29_ANALOG                  IOID_29
#define CC2640R2DK_7ID_DIO30_ANALOG                  IOID_30

/* Button Board */
#define CC2640R2DK_7ID_KEY_SELECT                    IOID_11         /* P1.14 */
#define CC2640R2DK_7ID_KEY_UP                        IOID_19         /* P1.10 */
#define CC2640R2DK_7ID_KEY_DOWN                      IOID_12         /* P1.12 */
#define CC2640R2DK_7ID_KEY_LEFT                      IOID_15         /* P1.6  */
#define CC2640R2DK_7ID_KEY_RIGHT                     IOID_18         /* P1.8  */

/* GPIO */
#define CC2640R2DK_7ID_GPIO_LED_ON                   1
#define CC2640R2DK_7ID_GPIO_LED_OFF                  0

/* LEDs */
#define CC2640R2DK_7ID_PIN_LED_ON                    1
#define CC2640R2DK_7ID_PIN_LED_OFF                   0
#define CC2640R2DK_7ID_PIN_LED1                      IOID_25         /* P2.11 */
#define CC2640R2DK_7ID_PIN_LED2                      IOID_27         /* P2.13 */
#define CC2640R2DK_7ID_PIN_LED3                      IOID_7          /* P1.2  */
#define CC2640R2DK_7ID_PIN_LED4                      IOID_6          /* P1.4  */

/* LCD  Board */
#define CC2640R2DK_7ID_LCD_MODE                      IOID_4          /* P1.11 */
#define CC2640R2DK_7ID_LCD_RST                       IOID_5          /* P1.13 */
#define CC2640R2DK_7ID_LCD_CSN                       IOID_14         /* P1.17 */

/* SD Card */
#define CC2640R2DK_7ID_SDCARD_CSN                    IOID_30         /* P2.12 */

/* SPI Board */
#define CC2640R2DK_7ID_SPI0_MISO                     IOID_8          /* P1.20 */
#define CC2640R2DK_7ID_SPI0_MOSI                     IOID_9          /* P1.18 */
#define CC2640R2DK_7ID_SPI0_CLK                      IOID_10         /* P1.16 */
#define CC2640R2DK_7ID_SPI0_CSN                      PIN_UNASSIGNED  /* P1.14, separate CSn for LCD, SDCARD, and ACC */
#define CC2640R2DK_7ID_SPI1_MISO                     IOID_24         /* RF2.10 for testing only */
#define CC2640R2DK_7ID_SPI1_MOSI                     IOID_23         /* RF2.5  for testing only */
#define CC2640R2DK_7ID_SPI1_CLK                      IOID_30         /* RF2.12 for testing only */
#define CC2640R2DK_7ID_SPI1_CSN                      PIN_UNASSIGNED  /* RF2.6  for testing only */

/* Power Board */
#define CC2640R2DK_7ID_3V3_EN                        IOID_13         /* P1.15 */

/* PWM Outputs */
#define CC2640R2DK_7ID_PWMPIN0                       CC2640R2DK_7ID_PIN_LED1
#define CC2640R2DK_7ID_PWMPIN1                       CC2640R2DK_7ID_PIN_LED2
#define CC2640R2DK_7ID_PWMPIN2                       PIN_UNASSIGNED
#define CC2640R2DK_7ID_PWMPIN3                       PIN_UNASSIGNED
#define CC2640R2DK_7ID_PWMPIN4                       PIN_UNASSIGNED
#define CC2640R2DK_7ID_PWMPIN5                       PIN_UNASSIGNED
#define CC2640R2DK_7ID_PWMPIN6                       PIN_UNASSIGNED
#define CC2640R2DK_7ID_PWMPIN7                       PIN_UNASSIGNED

/* UART Board */
#define CC2640R2DK_7ID_UART_RX                       IOID_2          /* P1.7 */
#define CC2640R2DK_7ID_UART_TX                       IOID_3          /* P1.9 */
#define CC2640R2DK_7ID_UART_CTS                      IOID_0          /* P1.3 */
#define CC2640R2DK_7ID_UART_RTS                      IOID_21         /* P2.18 */

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 */
void CC2640R2DK_7ID_initGeneral(void);

/*!
 *  @def    CC2640R2DK_7ID_ADCBufName
 *  @brief  Enum of ADCBufs
 */
typedef enum CC2640R2DK_7ID_ADCBufName {
    CC2640R2DK_7ID_ADCBUF0 = 0,

    CC2640R2DK_7ID_ADCBUFCOUNT
} CC2640R2DK_7ID_ADCBufName;

/*!
 *  @def    CC2640R2DK_7ID_ADCBuf0SourceName
 *  @brief  Enum of ADCBuf channels
 */
typedef enum CC2640R2DK_7ID_ADCBuf0ChannelName {
    CC2640R2DK_7ID_ADCBUF0CHANNEL0 = 0,
    CC2640R2DK_7ID_ADCBUF0CHANNEL1,
    CC2640R2DK_7ID_ADCBUF0CHANNEL2,
    CC2640R2DK_7ID_ADCBUF0CHANNEL3,
    CC2640R2DK_7ID_ADCBUF0CHANNEL4,
    CC2640R2DK_7ID_ADCBUF0CHANNEL5,
    CC2640R2DK_7ID_ADCBUF0CHANNEL6,
    CC2640R2DK_7ID_ADCBUF0CHANNEL7,
    CC2640R2DK_7ID_ADCBUF0CHANNELVDDS,
    CC2640R2DK_7ID_ADCBUF0CHANNELDCOUPL,
    CC2640R2DK_7ID_ADCBUF0CHANNELVSS,

    CC2640R2DK_7ID_ADCBUF0CHANNELCOUNT
} CC2640R2DK_7ID_ADCBuf0ChannelName;

/*!
 *  @def    CC2640R2DK_7ID_ADCName
 *  @brief  Enum of ADCs
 */
typedef enum CC2640R2DK_7ID_ADCName {
    CC2640R2DK_7ID_ADC0 = 0,
    CC2640R2DK_7ID_ADC1,
    CC2640R2DK_7ID_ADC2,
    CC2640R2DK_7ID_ADC3,
    CC2640R2DK_7ID_ADC4,
    CC2640R2DK_7ID_ADC5,
    CC2640R2DK_7ID_ADC6,
    CC2640R2DK_7ID_ADC7,
    CC2640R2DK_7ID_ADCDCOUPL,
    CC2640R2DK_7ID_ADCVSS,
    CC2640R2DK_7ID_ADCVDDS,

    CC2640R2DK_7ID_ADCCOUNT
} CC2640R2DK_7ID_ADCName;

/*!
 *  @def    CC2640R2DK_7ID_CryptoName
 *  @brief  Enum of Crypto names
 */
typedef enum CC2640R2DK_7ID_CryptoName {
    CC2640R2DK_7ID_CRYPTO0 = 0,

    CC2640R2DK_7ID_CRYPTOCOUNT
} CC2640R2DK_7ID_CryptoName;

/*!
 *  @def    CC2640R2DK_7ID_GPIOName
 *  @brief  Enum of GPIO names
 */
typedef enum CC2640R2DK_7ID_GPIOName {
    CC2640R2DK_7ID_GPIO_S1 = 0,
    CC2640R2DK_7ID_GPIO_S2,
    CC2640R2DK_7ID_GPIO_LED1,
    CC2640R2DK_7ID_GPIO_LED2,
    CC2640R2DK_7ID_GPIO_LED3,
    CC2640R2DK_7ID_GPIO_LED4,

    CC2640R2DK_7ID_GPIOCOUNT
} CC2640R2DK_7ID_GPIOName;

/*!
 *  @def    CC2640R2DK_7ID_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum CC2640R2DK_7ID_GPTimerName {
    CC2640R2DK_7ID_GPTIMER0A = 0,
    CC2640R2DK_7ID_GPTIMER0B,
    CC2640R2DK_7ID_GPTIMER1A,
    CC2640R2DK_7ID_GPTIMER1B,
    CC2640R2DK_7ID_GPTIMER2A,
    CC2640R2DK_7ID_GPTIMER2B,
    CC2640R2DK_7ID_GPTIMER3A,
    CC2640R2DK_7ID_GPTIMER3B,

    CC2640R2DK_7ID_GPTIMERPARTSCOUNT
} CC2640R2DK_7ID_GPTimerName;

/*!
 *  @def    CC2640R2DK_7ID_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum CC2640R2DK_7ID_GPTimers {
    CC2640R2DK_7ID_GPTIMER0 = 0,
    CC2640R2DK_7ID_GPTIMER1,
    CC2640R2DK_7ID_GPTIMER2,
    CC2640R2DK_7ID_GPTIMER3,

    CC2640R2DK_7ID_GPTIMERCOUNT
} CC2640R2DK_7ID_GPTimers;

/*!
 *  @def    CC2640R2DK_7ID_PWM
 *  @brief  Enum of PWM outputs
 */
typedef enum CC2640R2DK_7ID_PWMName {
    CC2640R2DK_7ID_PWM0 = 0,
    CC2640R2DK_7ID_PWM1,
    CC2640R2DK_7ID_PWM2,
    CC2640R2DK_7ID_PWM3,
    CC2640R2DK_7ID_PWM4,
    CC2640R2DK_7ID_PWM5,
    CC2640R2DK_7ID_PWM6,
    CC2640R2DK_7ID_PWM7,

    CC2640R2DK_7ID_PWMCOUNT
} CC2640R2DK_7ID_PWMName;

/*!
 *  @def    CC2640R2DK_7ID_SPIName
 *  @brief  Enum of SPI names
 */
typedef enum CC2640R2DK_7ID_SPIName {
    CC2640R2DK_7ID_SPI0 = 0,
    CC2640R2DK_7ID_SPI1,

    CC2640R2DK_7ID_SPICOUNT
} CC2640R2DK_7ID_SPIName;

/*!
 *  @def    CC2640R2DK_7ID_UARTName
 *  @brief  Enum of UARTs
 */
typedef enum CC2640R2DK_7ID_UARTName {
    CC2640R2DK_7ID_UART0 = 0,

    CC2640R2DK_7ID_UARTCOUNT
} CC2640R2DK_7ID_UARTName;

/*!
 *  @def    CC2640R2DK_7ID_UDMAName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC2640R2DK_7ID_UDMAName {
    CC2640R2DK_7ID_UDMA0 = 0,

    CC2640R2DK_7ID_UDMACOUNT
} CC2640R2DK_7ID_UDMAName;

/*!
 *  @def    CC2640R2DK_7ID_WatchdogName
 *  @brief  Enum of Watchdogs
 */
typedef enum CC2640R2DK_7ID_WatchdogName {
    CC2640R2DK_7ID_WATCHDOG0 = 0,

    CC2640R2DK_7ID_WATCHDOGCOUNT
} CC2640R2DK_7ID_WatchdogName;

/*!
 *  @def    CC2650_7ID_TRNGName
 *  @brief  Enum of TRNG names on the board
 */
typedef enum CC2640R2DK_7ID_TRNGName {
    CC2640R2DK_7ID_TRNG0 = 0,
    CC2640R2DK_7ID_TRNGCOUNT
} CC2640R2DK_7ID_TRNGName;

#ifdef __cplusplus
}
#endif

#endif /* __CC2640R2DK_7ID_BOARD_H__ */
