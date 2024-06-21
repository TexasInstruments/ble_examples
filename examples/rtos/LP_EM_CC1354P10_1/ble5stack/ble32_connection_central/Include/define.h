// *****************************************************************************
// define.h
// *****************************************************************************
#include "define/define_gps.h"
#include "define/define_sensors.h"

#ifndef TRUE
enum BOOL_FLAG { FALSE, TRUE };
#endif

// *****************************************************************************
// Common typedefs
// *****************************************************************************
typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned long ulong;

enum CMD_CTRL_VALUES {
    CMD_TEXT_CONTROL = 1, CMD_FILE_CONTROL, CMD_LED_CONTROL, CMD_AUDIO_CONTROL, CMD_VIDEO_CONTROL, CMD_SENSOR_CONTROL, CMD_MOTOR_CONTROL,
    CMD_WIFI_CONTROL, CMD_VIDEO_NEW_FRAME, CMD_BLE_CONTROL, CMD_ZIGBEE_CONTROL, CMD_CONSOLE_CONTROL, CMD_RADIO_CONTROL, CMD_BOOTLOADER_CONTROL,
    CMD_BOARD_CONTROL, CMD_VOICE_CONTROL, CMD_CODEC_CONTROL, CMD_INA_CONTROL, CMD_FLIR_CONTROL, CMD_ACK_RESPONSE, CMD_NACK_RESPONSE
};

enum CMD_BOARD {
    CMD_BOARD_GET_STATE, CMD_BOARD_GET_STATE_RESPONSE, CMD_BOARD_GET_IP_SERVERS, CMD_BOARD_GET_IP_SERVERS_RESPONSE,
    CMD_BOARD_SET_AUX_STATE, CMD_BOARD_SET_AUX_IP_SERVERS
};

enum PROTOCOL_TYPE {
    NET_PROTO_TCP, NET_PROTO_UDP, NET_PROTO_MULTICAST, NET_PROTO_RADIO, NET_PROTO_SERIAL_PORT
};
enum IP_PROTO_FLAGS { TCP_PROTO_FLAG = 1, UDP_PROTO_FLAG = 2, MC_PROTO_FLAG = 4 };

enum CMD_WIFI {
    CMD_WIFI_SEND_BROADCAST, CMD_WIFI_CREATE_UDP_CLIENT_SOCKET, CMD_WIFI_GET_UDP_CLIENT_SOCKET, CMD_WIFI_START_TX_PER_TEST, CMD_WIFI_SEND_PER_PACKET,
    CMD_WIFI_RESET_TX_PER_RESULTS, CMD_WIFI_GET_TX_PER_STATUS, CMD_WIFI_RESET_RX_PER_RESULTS, CMD_WIFI_GET_RX_PER_STATUS,
    CMD_WIFI_GET_PROTOCOL, CMD_WIFI_SET_PROTOCOL, CMD_WIFI_WRITE_TX_PER_REGS, CMD_WIFI_READ_TX_PER_REGS,
    CMD_WIFI_PER_ENABLE_AUTO_UPDATE, CMD_WIFI_PER_ENABLE_PRINT, CMD_WIFI_READ_SSID_AND_PASSWORD, CMD_WIFI_WRITE_SSID_AND_PASSWORD,
    CMD_WIFI_CONNECT_TO_ROUTER, CMD_WIFI_DISCONNECT_FROM_ROUTER, CMD_WIFI_GET_RADIO_STATE, CMD_WIFI_GET_BOARD_ID,
    CMD_WIFI_BUTTON_CONTROL, CMD_WIFI_VOICE_PROCESSING, CMD_WIFI_HIBERNATE, CMD_WIFI_DEEP_SLEEP,
    CMD_WIFI_GATEWAY_SEND_AUDIO_PLAYBACK, CMD_WIFI_GATEWAY_SEND_AUDIO_UDP, CMD_WIFI_GATEWAY_AUDIO_LOOPBACK,
    CMD_WIFI_GATEWAY_SEND_AUDIO_CAPTURE, CMD_WIFI_GATEWAY_ADPCM_ENCODE, CMD_WIFI_GET_AUDIO_CAPTURE_IP, CMD_WIFI_SET_AUDIO_CAPTURE_IP,
    CMD_WIFI_GET_AUDIO_PLAYBACK_IP, CMD_WIFI_SET_AUDIO_PLAYBACK_IP, CMD_WIFI_GET_VIDEO_CAPTURE_IP, CMD_WIFI_SET_VIDEO_CAPTURE_IP
};

//enum CMD_BLE_32X {
//    CMD_BLE_32X_DISCOVER, CMD_BLE_32X_CONNECT, CMD_BLE_32X_LED, CMD_BLE_32X_PRINT_STRING, CMD_BLE_32X_WRITE_DATA_BLOCK,
//    CMD_BLE_32X_READ_DATA_BLOCK, CMD_BLE_32X_TX_CONTROL, CMD_BLE_32X_AUTO_UPDATE, CMD_BLE_32X_GET_STATUS, CMD_BLE_32X_STATUS_RESULT,
//    CMD_BLE_32X_CONNECT_RESULT, CMD_BLE_32X_SCAN_RESULT, CMD_BLE_32X_MTU_UPDATE, CMD_BLE_32X_NOTIFY, CMD_BLE_32X_START_GATT, CMD_BLE_32X_STOP_GATT
//};

enum CMD_BLE_32X {
    CMD_BLE_32X_DISCOVER, CMD_BLE_32X_CONNECT, CMD_BLE_32X_LED, CMD_BLE_32X_PRINT_STRING, CMD_BLE_32X_GATT_DATA_BLOCK,
    CMD_BLE_32X_GET_STATUS, CMD_BLE_32X_STATUS_RESULT, CMD_BLE_32X_CONNECT_RESULT, CMD_BLE_32X_SCAN_RESULT,
    CMD_BLE_32X_MTU_UPDATE, CMD_BLE_32X_NOTIFY, CMD_BLE_32X_START_GATT, CMD_BLE_32X_STOP_GATT, CMD_BLE_32X_AUTO_GATT, CMD_BLE_32X_LATENCY_RSSI
};

#if defined ( USE_INA_23X )
enum CMD_INA_VALUES
{
    CMD_INA_SET_UPDATE_INTERVAL, CMD_INA_ENABLE_READ_DATA, CMD_INA_GET_STATUS, CMD_INA_AUTO_UPDATE_FLAG, CMD_INA_PRINT_FLAG, CMD_INA_NEW_UPDATE,
    CMD_INA_READ_WARNING_LIMITS, CMD_INA_WRITE_WARNING_LIMITS
};
#endif

enum CMD_RADIO {
    CMD_RADIO_SEND_BROADCAST, CMD_RADIO_ACK_BROADCAST, CMD_RADIO_SEND_PACKET, CMD_RADIO_RETRANSMIT_PACKET, CMD_RADIO_SEND_DATA_TEST,
    CMD_RADIO_TEST_TX_CONTINUOUS, CMD_RADIO_TEST_RX_CONTINUOUS, CMD_RADIO_PROCESS_PACKET, CMD_RADIO_START_PER_TEST, CMD_RADIO_STREAM_PER_DATA,
    CMD_RADIO_GET_VERSION, CMD_RADIO_GET_LOCAL_PHY, CMD_RADIO_SET_LOCAL_PHY, CMD_RADIO_SET_REMOTE_PHY, CMD_RADIO_ACK_TO_SET_REMOTE_PHY, CMD_RADIO_SET_POWER_LEVEL,
    CMD_RADIO_GET_LOCAL_FREQUENCY, CMD_RADIO_SET_LOCAL_FREQUENCY, CMD_RADIO_SET_REMOTE_FREQUENCY, CMD_RADIO_ACK_TO_SET_REMOTE_FREQUENCY,
    CMD_RADIO_WRITE_TX_PER_REGS, CMD_RADIO_RESET_TX_PER_STATUS, CMD_RADIO_GET_SENSOR_DATA, CMD_RADIO_DISPLAY_SENSOR_DATA,
    CMD_RADIO_RESET_RX_PER_STATUS,CMD_RADIO_READ_TX_PER_REGS, CMD_RADIO_ACK_CONTROL, CMD_RADIO_SET_ACK_MAX_RETRANSMITS, CMD_RADIO_GET_ACK_MAX_RETRANSMITS,
    CMD_RADIO_GET_TX_PER_STATUS, CMD_RADIO_GET_RX_PER_STATUS, CMD_RADIO_DISPLAY_TX_PER_STATUS, CMD_RADIO_DISPLAY_RX_PER_STATUS,
    CMD_RADIO_GET_ACK_RETRANSMISSION_ARRAY, CMD_RADIO_DISPLAY_ACK_RETRANSMISSION_ARRAY, CMD_RADIO_GET_ACK_STATUS, CMD_RADIO_DISPLAY_ACK_STATUS,
    CMD_RADIO_TX_PER_ENABLE_AUTO_UPDATE, CMD_RADIO_TX_PER_ENABLE_PRINT, CMD_RADIO_RX_PER_ENABLE_AUTO_UPDATE, CMD_RADIO_RX_PER_ENABLE_PRINT,
    CMD_RADIO_SEND_PER_PACKET, CMD_RADIO_SEND_TEST_PACKET, CMD_RADIO_SEND_DATA_PACKET,
    CMD_RADIO_READ_REG, CMD_RADIO_WRITE_REG, CMD_RADIO_GET_PROTOCOL, CMD_RADIO_SET_PROTOCOL, CMD_RADIO_GET_STATE, CMD_RADIO_GET_BOARD_ID,
    CMD_RADIO_BUTTON_CONTROL, CMD_RADIO_SEND_RAW_AUDIO, CMD_RADIO_SEND_ADPCM_AUDIO, CMD_RADIO_SEND_ENCODED_AUDIO, CMD_RADIO_SEND_OPUS_AUDIO,
    CMD_RADIO_STREAM_FILE, CMD_RADIO_SEND_FLIR_FRAME, CMD_RADIO_SEND_H264_FRAME, CMD_RADIO_FRAME_IN_PROCESS, CMD_RADIO_KEEP_ALIVE,
    CMD_RADIO_GET_GPS_STATUS, CMD_RADIO_DISPLAY_GPS_STATUS, CMD_RADIO_BROADCAST_GPS_STATUS, CMD_RADIO_AUTO_PHY_RESET
};

enum CMD_MSG_RADIO_VALUES {
    CMD_MSG_RADIO_PROCESS_MESSAGE = 1, CMD_MSG_RADIO_PRINT_CONSOLE, CMD_MSG_RADIO_PRINT_REMOTE, CMD_MSG_RADIO_SEND_FRAME, CMD_MSG_RADIO_SEND_FILE,
    CMD_MSG_RADIO_RECEIVE_FILE, CMD_MSG_RADIO_SEND_FLIR_IMAGE, CMD_MSG_RADIO_SEND_H264_IMAGE, CMD_MSG_RADIO_SEND_AUDIO_FRAME,
    CMD_MSG_RADIO_WAIT_ON_EOF_TO_SEND, CMD_MSG_RADIO_SET_QUALITY_PARAMS, CMD_MSG_RADIO_SET_QUALITY_LEVEL, CMD_MSG_RADIO_ANTENNA_TEST,
    CMD_MSG_RADIO_PER_RECEIVE
};

enum CMD_MSG_VALUES { CMD_MSG_OPEN_ROUTER = 1, CMD_MSG_CLOSE_ROUTER, CMD_MSG_OPEN_TCP_SOCKET, CMD_MSG_CLOSE_TCP_SOCKET,
    CMD_MSG_OPEN_UDP_SOCKET, CMD_MSG_CLOSE_UDP_SOCKET, CMD_MSG_OPEN_MULTICAST_SOCKET, CMD_MSG_CLOSE_MULTICAST_SOCKET, CMD_MSG_WRITE_FLASH,
    CMD_MSG_SENSOR_DATA, CMD_MSG_START_AUDIO, CMD_MSG_SEND_TEXT_STRING, CMD_MSG_SEND_AUDIO_PCM, CMD_MSG_SEND_AUDIO_DATA, CMD_MSG_SEND_VIDEO_DATA,
    CMD_MSG_SEND_DATA, CMD_MSG_RESEND_DATA, CMD_MSG_SEND_FRAME, CMD_MSG_SEND_FLIR_IMAGE, CMD_MSG_SEND_VIDEO_FILE,
    CMD_MSG_SEND_OMV_VIDEO_FRAME, CMD_MSG_SEND_OMV_VIDEO_STATUS, CMD_MSG_SEND_OMV_AUDIO_FRAME, CMD_MSG_SEND_OMV_AUDIO_STATUS, CMD_MSG_SEND_FLIR_STATUS,
    CMD_MSG_START_TX_PER_TEST, CMD_MSG_PRINT_VOICE_TEXT, CMD_MSG_LPDS_MODE, CMD_MSG_HIBERNATE_MODE, CMD_MSG_BUTTON_1, CMD_MSG_BUTTON_2,
    CMD_MSG_REINIT_AUDIO_SOCKETS, CMD_MSG_PRINT_IMU_STATUS, CMD_MSG_SEND_WIFI_STATUS, CMD_MSG_OPEN_AUDIO_CAPTURE_CLIENT, CMD_MSG_OPEN_AUDIO_PLAYBACK_SERVER,
    CMD_MSG_CREATE_VIDEO_FRAME, CMD_MSG_RECEIVE_FILE
};

#if defined ( USE_STEPPER_MOTOR )
enum MOTOR_SM_CMD_VALUES {
    CMD_MOTOR_SM_TEXT_CONTROL = 1, CMD_MOTOR_SM_STATE, CMD_MOTOR_SM_SLEEP_MODE, CMD_MOTOR_SM_DIRECTION_MODE, CMD_MOTOR_SM_CONTROL_MODE, CMD_MOTOR_SM_DECAY_MODE,
    CMD_MOTOR_SM_TORQUE_MODE, CMD_MOTOR_SM_SPEED, CMD_MOTOR_SM_STEPS, CMD_MOTOR_SM_OUTPUT_MODE, CMD_MOTOR_SM_STEP_MODE_INDEX, CMD_MOTOR_SM_CURRENT, CMD_MOTOR_SM_GET_STATUS,
    CMD_MOTOR_SM_START
};
#endif

enum STATE_SPI {
    STATE_SPI_INIT, STATE_SPI_INITIATING_XFER, STATE_SPI_WAITING_ON_XFER, STATE_SPI_PROCESS_INCOMING_MESSAGE, STATE_SPI_WAITING_ON_MESSAGE,
    STATE_SPI_PROCESS_COMMAND, STATE_SPI_SEND_DATA_TO_NETWORK, STATE_SPI_REQUESTING_READ_TRANSFER, STATE_SPI_PROCESSING_ERROR
};

enum COMMAND_STATES { COMMAND_STATE_SYNC, COMMAND_STATE_HEADER, COMMAND_STATE_PAYLOAD };

enum CMD_SPI {
    CMD_SPI_SEND_CMD, CMD_SPI_READ_DATA, CMD_SPI_SEND_ADPCM
};

enum CMD_MSG_BLE_VALUES {
    CMD_MSG_ADD_CONNECTION, CMD_MSG_REMOVE_CONNECTION, CMD_MSG_SEND_MESSAGE, CMD_MSG_SEND_GATT_DATA, CMD_MSG_CONNECT_UPDATE, CMD_MSG_PRINT_MESSAGE, CMD_MSG_PROCESS_SPI_MESSAGE
};

enum CMD_FILE_VALUES {
    CMD_FILE_GET_DIR, CMD_FILE_DISPLAY_DIR, CMD_FILE_DELETE_FILE, CMD_FILE_RENAME_FILE, CMD_FILE_ACK_CONTROL, CMD_FILE_MAX_RETRANSMISSIONS,
    CMD_FILE_SET_BITRATE, CMD_FILE_PAYLOAD_SIZE, CMD_FILE_SEND_PROTOCOL, CMD_FILE_DESTINATION_BOARD_ID,
    CMD_FILE_RECEIVE_FILE, CMD_FILE_PLAY_FILE_TO_AUDDEC, CMD_FILE_SEND_FILE, CMD_FILE_STOP_FILE_SEND, CMD_FILE_GET_TX_PARAMS, CMD_FILE_GET_ACK_STATUS,
    CMD_FILE_GET_ACK_HISTOGRAM
};

enum CMD_RADIO_PER_TEST { CMD_RADIO_PER_TEST_START, CMD_RADIO_PER_TEST_TERMINATE };
enum RADIO_STATES { RADIO_STATE_IDLE, RADIO_STATE_RECEIVE, RADIO_STATE_TRANSMIT, RX_FIFO_ERRORx, TX_FIFO_ERRORx };
enum RADIO_PHYS {
    RADIO_PHY_5KHZ, RADIO_PHY_50KHZ, RADIO_PHY_200KHZ, RADIO_PHY_500KHZ, RADIO_PHY_1MHZ, RADIO_PHY_MAX
};
enum RADIO_PER_MODE { PER_SINGLE_PROPRIETARY, PER_SMARTRF_STUDIO_MODE, PER_MULTI_PROPRIETARY };

//#define DEFAULT_RADIO_PHY   RADIO_PHY_5KHZ
//#define DEFAULT_RADIO_PHY   RADIO_PHY_50KHZ
//#define DEFAULT_RADIO_PHY   RADIO_PHY_200KHZ
//#define DEFAULT_RADIO_PHY   RADIO_PHY_500KHZ
#define DEFAULT_RADIO_PHY   RADIO_PHY_1MHZ

// ****************************************************************************
// Device ID Values
// ****************************************************************************
#define ID_MCU_MASK         0b00000000000000000000000000001111
#define ID_3220             0b00000000000000000000000000000001
#define ID_1352             0b00000000000000000000000000000010
#define ID_2652             0b00000000000000000000000000000011
#define ID_MSP432E401       0b00000000000000000000000000000100
#define ID_MSP432E411       0b00000000000000000000000000000101
#define ID_AUX_C6747        0b00000000000000000000000000000110
#define ID_AUX_C5545        0b00000000000000000000000000000111

#define ID_RADIO_MASK       0b00000000000000000000000001110000
#define ID_WIFI             0b00000000000000000000000000010000
#define ID_SUB1G            0b00000000000000000000000000100000
#define ID_BLE              0b00000000000000000000000000110000
#define ID_ZIGBEE           0b00000000000000000000000001000000

#define ID_SAV_AV_BP        0b00000000000000000000000010000000
#define ID_AUDIO_BP         0b00000000000000000000000100000000
#define ID_SENSOR_HUB_BP    0b00000000000000000000001000000000
#define ID_BATTERY_BP       0b00000000000000000000010000000000
#define ID_SD_CARD          0b00000000000000000000100000000000
#define ID_MOTOR            0b00000000000000000001000000000000
#define ID_FLIR             0b00000000000000000010000000000000
#define ID_GPS              0b00000000000000000100000000000000
#define ID_JOY_STICK        0b00000000000000001000000000000000

#define ID_ACODEC_MASK      0b00000000111111110000000000000000
#define ID_ACODEC_ADCM      0b00000000000000010000000000000000
#define ID_ACODEC_OPUS      0b00000000000000100000000000000000
#define ID_ACODEC_MP3_ENC   0b00000000000001000000000000000000
#define ID_ACODEC_MP3_DEC   0b00000000000010000000000000000000
#define ID_ACODEC_AAC_ENC   0b00000000000100000000000000000000
#define ID_ACODEC_AAC_DEC   0b00000000001000000000000000000000
#define ID_ACODEC_LC3_ENC   0b00000000010000000000000000000000
#define ID_ACODEC_LC3_DEC   0b00000000100000000000000000000000

#define ID_AUX_MASK         0b00000001000000000000000000000000

#define ID_AUX_SERIAL_MASK  0b00000110000000000000000000000000
#define ID_AUX_SERIAL_UART  0b00000010000000000000000000000000
#define ID_AUX_SERIAL_SPI   0b00000100000000000000000000000000
#define ID_PIR              0b00001000000000000000000000000000

#define SL_IPV4_BYTE( x, y) ( ( x >> ( y * 8)) & 0xFF )

#define PACKET_SYNC_CODE                0xAA
#define CLIENT_TCP_SERVER_PORT          10000
#define CLIENT_UDP_SERVER_PORT          10001
#define CLIENT_MULTICAST_SERVER_PORT    10002
#define TCP_PACKET_RECEIVE_SIZE         1200

// ****************************************************************************
// I2C Addresses
// ****************************************************************************
#define INA233_I2C_ADDRESS      0x40
#define INA236_I2C_ADDRESS      0x40
#define WIT_IMU_I2C_ADDRESS     0x50

// *****************************************************************************
// Histogram constants
// *****************************************************************************
#define HISTOGRAM_MAX_NUMBER_OF_TIMING_BINS     8
#define HISTOGRAM_MAX_NUMBER_OF_RSSI_BINS       8

// *****************************************************************************
// SPI constants
// *****************************************************************************
#define SPI_MAX_NUMBER_OF_BUFFERS               16
#define SPI_TRANSFER_SIZE                       ( 256 + 16 )
#define SPI_MAX_BUFFER_SIZE                     192 + 16

// *****************************************************************************
// UART constants
// *****************************************************************************
#define MAX_NUMBER_OF_BYTES_IN_COMMAND_BUFFER   256
#define UART_CMD_BAUD_RATE                      115200
#define UART_BUFFER_SIZE                        512
#define MAX_NUMBER_OF_UART_TEXT_STRINGS         8
#define MAX_SIZE_OF_UART_PRINT_BUFFER           512
#define MAX_NUMBER_OF_BYTES_IN_UART_CMD_BUFFER  256

// *****************************************************************************
// BLE constants
// *****************************************************************************
#define MAX_NUMBER_OF_BLE_ARRAYS                2
#define MAX_NUMBER_OF_BLE_DEVICES               32

// *****************************************************************************
// INA constants
// *****************************************************************************
#define MAX_NUMBER_OF_INA_SAMPLES               32

// *****************************************************************************
// SD Card Definitions
// *****************************************************************************
//#define BOARD_ID_SD_CARD
#define MAX_SDCARD_FILENAME_TEXT_LENGTH         16
#define MAX_SIZE_OF_DIR_STORAGE                 128
#define MAX_SIZE_OF_FILE_STORAGE                512
#define MAX_SDCARD_NUMBER_OF_FILES              10

// *****************************************************************************
// Radio constants
// *****************************************************************************
#define RADIO_PER_PKT_LENGTH                    16              // Packet length has to be within fifo limits ( 1 - 127 bytes)
#define MAX_NUMBER_OF_BOARD_IDS                 8
#define MAX_NUMBER_OF_ACK_RETRANSMITS           16
#define MAX_INDEX_OF_ACKS                       4
#define MAX_NUMBER_OF_LOG_LOST_PACKETS          32
#define MAX_RADIO_PAYLOAD_LENGTH                256
#define MAX_INDEX_OF_BYTES_PER_PACKET           2
#define PER_STREAM_MASK_RSSI                    0b00000001
#define PER_STREAM_MASK_BIT_RATE                0b00000010

// *****************************************************************************
// Color codes for terminal output
// *****************************************************************************
#define CLEAR_SCREEN                "\033[2J"
#define CURSOR_HOME                 "\033[H"
#define RED_COLOR                   "\033[0;31m"
#define BOLD_RED_COLOR              "\033[1;31m"
#define GREEN_COLOR                 "\033[0;32m"
#define BOLD_GREEN_COLOR            "\033[1;32m"
#define YELLOW_COLOR                "\033[0;33m"
#define BOLD_YELLOW_COLOR           "\033[1;33m"
#define BLUE_COLOR                  "\033[0;34m"
#define BOLD_BLUE_COLOR             "\033[1;34m"
#define MAGENTA_COLOR               "\033[0;35m"
#define BOLD_MAGENTA_COLOR          "\033[1;35m"
#define CYAN_COLOR                  "\033[0;36m"
#define BOLD_CYAN_COLOR             "\033[1;36m"
#define RESET_COLOR                 "\033[0m"

#define CURSOR_SET_TO_SOL           "\033[0A"
#define CURSOR_ERASE_CURRENT_LINE   "\033[2K"
#define CURSOR_ERASE_TO_EOL         "\033[K"
#define BACKUP_1_LINE               "\033[1A"
#define BACKUP_2_LINES              "\033[2A"

// *****************************************************************************
// end of file
// *****************************************************************************
