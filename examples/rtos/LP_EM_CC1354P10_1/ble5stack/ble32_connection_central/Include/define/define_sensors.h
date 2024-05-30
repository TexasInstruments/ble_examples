/******************************************************************************

@file  define_sensors.h

Group: WCS, BTS
Target Device: cc13xx cc26xx

******************************************************************************

 Copyright (c) 2022-2024, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

******************************************************************************
*****************************************************************************/

// *****************************************************************************
// define_sensors.h
// *****************************************************************************
#ifndef DEFINE_SENSORS_H_
#define DEFINE_SENSORS_H_

enum SENSOR_VALUES { SENSOR_TEMPERATURE, SENSOR_LIGHT, SENSOR_PRESSURE, SENSOR_HUMIDITY, SENSOR_MOTION, SENSOR_GPS, SENSOR_BATTERY, SENSOR_HUB };
enum SENSOR_TYPE {
    SENSOR_HUB_DATA, SENSOR_GPS_DATA, SENSOR_BATTERY_DATA, SENSOR_RSSI_DATA, SENSOR_IMU_DATA, SENSOR_PIR_DATA,
    SENSOR_PRESSURE_DATA, SENSOR_LIGHT_DATA, SENSOR_HUMIDITY_DATA, SENSOR_AMBIENT_TEMPERATURE_DATA, SENSOR_IR_TEMPERATURE_DATA, SENSOR_MOTION_DATA,
    SENSOR_UPDATE_FLAG, SENSOR_GET_UPDATE_INTERVAL, SENSOR_SET_UPDATE_INTERVAL, SENSOR_GET_PRINT_INTERVAL, SENSOR_SET_PRINT_INTERVAL,
    SENSOR_HUB_GET_PROTOCOL, SENSOR_HUB_SET_PROTOCOL, SENSOR_GPS_COMMAND, JOYSTICK_GET_PROTOCOL, JOYSTICK_SET_PROTOCOL, SENSOR_GPS_PASS_THROUGH,
    SENSOR_READ, SENSOR_WRITE, SENSOR_DEVICE, GET_NUMBER_OF_BATTERIES, ENABLE_BATTERY, GET_BATTERY_STATUS,
    SENSOR_RTC_GET_DATA, SENSOR_RTC_SET_DATA, SENSOR_RTC_SET_MATCH, GET_GPS_STATUS, GPS_FILE_WRITE, SENSOR_IMU_COMMAND, SENSOR_PIR_COMMAND
};

#if defined ( USE_IMU )
enum IMU_CMD_VALUES {
    CMD_IMU_SEND_CONTROL, CMD_IMU_PRINT_CONTROL, CMD_IMU_SET_SAMPLE_RATE, CMD_IMU_SET_UPDATE_RATE, CMD_IMU_NUMBER_OF_MESSAGES
};
#endif

#if defined ( USE_PIR )
enum PIR_CMD_VALUES {
    CMD_PIR_TEXT_CONTROL = 1, CMD_PIR_MOTION_CONTROL, CMD_PIR_PRINT_CONTROL, CMD_PIR_SEND_CONTROL, CMD_PIR_SELECT_CHANNEL_MASK,
    CMD_PIR_SET_THRESHOLD, CMD_PIR_SET_UPDATE_RATE, CMD_PIR_GET_STATUS, CMD_PIR_MOTION_CHANGE, CMD_PIR_GET_IRQ_REQUEST,
    CMD_PIR_BUFFER_LEVEL, CMD_PIR_BUFFER_DATA, CMD_MSG_NUMBER_OF_PIR_MESSAGES
};
#endif

#if defined ( USE_GPS_SONY )
enum SONY_GPS_PROGRAM_STATES {
    SONY_GPS_PROG_STATE_FILE1, SONY_GPS_PROG_STATE_FILE2, SONY_GPS_PROG_STATE_FILE3, SONY_GPS_PROG_STATE_FILE4, SONY_GPS_PROG_STATE_EXEC
};

enum SONY_GPS_RUNNING_STATES {
    SONY_GPS_RUN_STATE_IDLE, SONY_GPS_RUN_STATE_RUNNING
};

enum SONY_GPS_STATE_MODES {
    SONY_GPS_COLD_START, SONY_GPS_WARM_START, SONY_GPS_HOT_START
};

enum SONY_GPS_CMDS {
    CMD_MSG_GPS_SEND_BSSL, CMD_MSG_GPS_SEND_BUP, CMD_MSG_GPS_SEND_FER, CMD_MSG_GPS_SEND_GCD, CMD_MSG_GPS_SEND_GNS, CMD_MSG_GPS_SEND_GPOS, CMD_MSG_GPS_SEND_GPTC,
    CMD_MSG_GPS_SEND_GSOP, CMD_MSG_GPS_SEND_GSR, CMD_MSG_GPS_SEND_GSTP, CMD_MSG_GPS_SEND_GSW, CMD_MSG_GPS_SEND_GTE, CMD_MSG_GPS_SEND_GTIM, CMD_MSG_GPS_SEND_GTR,
    CMD_MSG_GPS_SEND_GTS, CMD_MSG_GPS_SEND_VER, CMD_MSG_GPS_SEND_WUP, CMD_MSG_GPS_SEND_SLP, CMD_MSG_GPS_GET_STATUS, CMD_MSG_GPS_SEND_PROGRAM_FIRMWARE,
    CMD_MSG_NUMBER_OF_MESSAGES
};
#endif

// *****************************************************************************
// UART
// *****************************************************************************
#define MAX_NUMBER_OF_BYTES_IN_UART_CMD_BUFFER       256

// *****************************************************************************
// OPT3001
// *****************************************************************************
// Slave address
#ifdef BOARD_ID_SENSOR_TAG_1350
#define OPT3001_I2C_ADDRESS             0x45
#else
#define OPT3001_I2C_ADDRESS             0x47
#endif

/* Register addresses */
#define REG_RESULT                      0x00
#define REG_CONFIGURATION               0x01
#define REG_LOW_LIMIT                   0x02
#define REG_HIGH_LIMIT                  0x03

#define REG_MANUFACTURER_ID             0x7E
#define REG_DEVICE_ID                   0x7F

/* Register values */
#define MANUFACTURER_ID                 0x5449  // TI
#define DEVICE_ID                       0x3001  // Opt 3001
#define CONFIG_RESET                    0xC810                   
#define CONFIG_TEST                     0xCC10
#define CONFIG_ENABLE                   0x10C4 // 0xC410   - 100 ms, continuous               
#define CONFIG_DISABLE                  0x10C0 // 0xC010   - 100 ms, shutdown

/* Bit values */
#define DATA_RDY_BIT                    0x0080  // Data ready

// *****************************************************************************
// MSP430FR2633
// *****************************************************************************
/* Slave address */
#define FR2633_I2C_ADDRESS              0x0A

// *****************************************************************************
// TMP007
// *****************************************************************************
/* Slave address */
#ifdef BOARD_ID_SENSOR_TAG_1350
#define TMP007_I2C_ADDRESS              0x44
#else
#define TMP007_I2C_ADDRESS              0x40
#endif

/* TMP006 register addresses */
#define TMP007_REG_ADDR_VOLTAGE         0x00
#define TMP007_REG_ADDR_LOCAL_TEMP      0x01
#define TMP007_REG_ADDR_CONFIG          0x02
#define TMP007_REG_ADDR_OBJ_TEMP        0x03
#define TMP007_REG_ADDR_STATUS          0x04
#define TMP007_REG_ADDR_STATUS_ENABLE   0x05
#define TMP007_REG_ADDR_TC0_COEFFICIENT 0x11
#define TMP007_REG_ADDR_TC1_COEFFICIENT 0x12
#define TMP007_REG_PROD_ID              0x1F

/* TMP006 register values */
#define TMP007_VAL_CONFIG_ON            0x1040  // Sensor on state (1 sec) with Output Filtering On
#define TMP007_VAL_CONFIG_OFF           0x0000  // Sensor off state
#define TMP007_VAL_CONFIG_RESET         0x8000
#define TMP007_VAL_PROD_ID              0x0078  // Product ID
#define TMP007_VAL_TC0					0x0000  //TC0 Coefficient Register
#define TMP007_VAL_TC1					0x0000  //TC1 Coefficient Register

#define TMP007_VAL_STATUS_MASK_ALERT    0x8000  // Enable Alert Pin
#define TMP007_VAL_STATUS_MASK_TEMP     0x4000  // Enable Temperature Conversion Ready Enable

/* Bit values */
#define CONV_RDY_BIT                    0x4000  // Conversion ready 

// *****************************************************************************
// BME280
// *****************************************************************************
// ***************************************************
// I2C ADDRESS DEFINITIONS
// **************************************************
#define BME280_I2C_ADDRESS1                  (0x76)
#define BME280_I2C_ADDRESS2                  (0x77)
// ***************************************************
// POWER MODE DEFINITIONS
// **************************************************
//  Sensor Specific constants
#define BME280_SLEEP_MODE                    (0x00)
#define BME280_FORCED_MODE                   (0x01)
#define BME280_NORMAL_MODE                   (0x03)
#define BME280_SOFT_RESET_CODE               (0xB6)

// ***************************************************
// OVER SAMPLING DEFINITIONS
// **************************************************
#define BME280_OVERSAMP_SKIPPED          (0x00)
#define BME280_OVERSAMP_1X               (0x01)
#define BME280_OVERSAMP_2X               (0x02)
#define BME280_OVERSAMP_4X               (0x03)
#define BME280_OVERSAMP_8X               (0x04)
#define BME280_OVERSAMP_16X              (0x05)

// ***************************************************
// STANDBY DEFINITIONS
// **************************************************
#define BME280_STANDBY_TIME_1_MS              (0x00)
#define BME280_STANDBY_TIME_63_MS             (0x01)
#define BME280_STANDBY_TIME_125_MS			  (0x02)
#define BME280_STANDBY_TIME_250_MS            (0x03)
#define BME280_STANDBY_TIME_500_MS            (0x04)
#define BME280_STANDBY_TIME_1000_MS           (0x05)
#define BME280_STANDBY_TIME_10_MS             (0x06)
#define BME280_STANDBY_TIME_20_MS             (0x07)

// *****************************************************************************
// Generic
// *****************************************************************************
/* Byte swap of 16-bit register value */
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)
#define SWAP(v) ( (LO_UINT16(v) << 8) | HI_UINT16(v) )

// ****************************************************************************
// MPU9150
// ****************************************************************************
#define PRINT_SKIP_COUNT        10

#endif /* DEFINE_H_ */

// *****************************************************************************
// end of file
// *****************************************************************************
