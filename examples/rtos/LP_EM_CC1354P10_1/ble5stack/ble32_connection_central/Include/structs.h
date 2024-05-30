/******************************************************************************

@file  structs.h

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
// structs.h
// *****************************************************************************

// *****************************************************************************
// Commands
// *****************************************************************************
struct __attribute__((packed)) CMD_PACKET_HEADER
{
    unsigned SyncByte : 8;
    unsigned Command : 5;
    unsigned PayloadBytes : 11;
    unsigned AckFlag : 1;
    unsigned SendProtocol : 3;
    unsigned CCTR : 3;
    unsigned Unused : 1;
    unsigned SourceAddress : 8;
    unsigned DestinationAddress : 8;
    unsigned SubCommand : 6;
    unsigned StartOfFrameFlag : 1;
    unsigned EndOfFrameFlag : 1;
    unsigned CheckSum : 8;
};
struct CMD_PACKET_DATAGRAM
{
    struct CMD_PACKET_HEADER Header;
    uint8_t Payload[ ];
};

struct __attribute__((packed)) RADIO_ACK_HEADER
{
    unsigned SyncByte : 8;
    unsigned Command : 5;
    unsigned ContinuityCtr : 3;
    unsigned BoardID : 8;
    signed RSSI : 8;
};

// ************************************************************************
// Buffer Structures
// ************************************************************************
struct BUFFER_INFO
{
    uint8_t *BottomPtr;
    uint8_t *TopPtr;
    uint8_t *BeginPtr;
    uint8_t *WritePtr;
    uint8_t *ReadPtr;
    int32_t WrapValue, ProcessingFlag;
    int32_t TotalBufferSize, FloorSize, WriteBufferSize;
    int32_t BufferLevel, InitialStartProcessLevel, StartProcessLevel, RunningLowLevel;
    int32_t TotalBytesWritten, TotalBytesRead;
    int32_t ReadByteIndex,  WriteByteIndex, EndianType;
    struct FILE_INFO *FileInfoPtr;
};

// *****************************************************************************
// Board
// *****************************************************************************
struct BOARD_IP_SERVER_INFO
{
    uint16_t IP_Flags;
    uint16_t TCP_Port;
    uint16_t UDP_Port;
    uint16_t MulticastPort;
    uint32_t TCP_UDP_IPAddress;
    uint32_t MulitcastIPAddress;
};

struct BOARD_INFO
{
    uint32_t State;
    uint8_t ID;
    uint8_t Protocol;
    uint8_t Unused1;
    uint8_t Unused2;
};

struct RADIO_DEVICE_INFO
{
    struct BOARD_INFO MyBoard;
    struct BOARD_INFO AuxBoard;
};

struct BOARD_ID_LIST
{
    struct BOARD_INFO BoardInfo;
    int8_t RSSI;
};
//} BoardElement;

struct BOARD_QUEUE_INFO
{
    uint32_t SelectedBoardIDListIndex, NumberOfActiveBoardIDs;
    struct BOARD_INFO SelectedBoardInfo;
    struct BOARD_ID_LIST BoardIDList[ MAX_NUMBER_OF_BOARD_IDS ];
};


// *****************************************************************************
// Messages
// *****************************************************************************
typedef struct MsgObj
{
    uint16_t Message;
    uint16_t FrameBytes;
    char *FramePtr;
    uint16_t ShortWord1;
    uint16_t ShortWord2;
    uint32_t LongWord;
} MsgObj, *Msg;             /* Use Msg as pointer to MsgObj */

// ************************************************************************
// Generic Structures
// ************************************************************************
struct TIME_INFO
{
    uint32_t Seconds;
    uint32_t Minutes;
    uint32_t Hours;
};

struct COMP_DATA
{
    int32_t Integer;
    uint32_t Fraction;
};

// *****************************************************************************
// PER
// *****************************************************************************
struct PER_INFO
{
    bool TxPERInProcessFlag, TxIPCreateFlag, TxAutoUpdateFlag;
    bool TxSinglePERTaskRunningFlag, TxMultiPERTaskRunningFlag;
    bool RxPERInProcessFlag, RxIPCreateFlag, RxAutoUpdateFlag;
    bool InfiniteRxFlag, InfiniteTxFlag, StreamFlag, ContinuousPERFlag;
    uint8_t TestMode, TxIPProtocol, RxIPProtocol, StreamDataMask, FilePrefix;
    uint8_t CCValue, DestinationAddress;
    uint8_t ContinuousTxMaxSeconds, ContinuousTxSeconds, ContinuousRxMaxSeconds, ContinuousRxSeconds;
    uint16_t TxIPPort, RxIPPort;
    uint32_t TxIPAddress, RxIPAddress;
    uint16_t TxBytesPerPacket, TestSequenceCtr, PERFileCtr;
    uint32_t TxTotalBytes, TxTotalPackets, TxPacketsToSend, TxPacketCtr;
    uint32_t TxDesiredBitRate, TxActualBitRate, TxMinBitRate, TxMaxBitRate;
    uint32_t TxTime_sec, DeltaClockTicks;
    uint32_t RxSecByteCtr, RxPacketsPerSecCtr;
    uint32_t RxTotalPackets, RxPacketsToSend, RxDuplicatePackets, RxBytesPerPacket, RxLastPacketCount, RxTotalBytes, RxLostPackets;
    uint32_t RxDesiredBitRate, RxActualBitRate, RxMinBitRate, RxMaxBitRate;
    uint32_t RxTime_sec, StartOfTestTicks, StartOfValidPacketTicks, StartOfReceivedPacketTicks;
    uint32_t TxStartTime;
    uint32_t RxLastTime, RxStartTime, RxPerSecTime, RxNextRateTime;
    struct timespec TimeSpecStartOfTest, TimeSpecStartOfValidPacket, TimeSpecStartOfReceivePacket;
    struct timespec TimeSpecRxStart, TimeSpecRxPerSecond, TimeSpecRxLast;
    uint16_t LostPacketLog[ MAX_NUMBER_OF_LOG_LOST_PACKETS ];
    uint8_t TxBuffer[ 48 ];
    uint8_t NetBuffer[ 48 ];
};

// *****************************************************************************
// WiFi
// *****************************************************************************
#if defined ( USE_WIFI_RADIO )
struct WIFI_IP_INFO
{
    uint16_t IPPort;
    uint32_t IPAddress;
};

struct WIFI_FLAGS
{
    unsigned UseRouterPasswordFlag : 1;
    unsigned AutoConnectRouterFlag : 1;
    unsigned RouterConnectedFlag : 1;
    unsigned AutoConnectServerFlag : 1;
    unsigned ServerConnectedFlag : 1;
    unsigned AutoStoreToFLASHFlag : 1;
    unsigned AutoCaptureStreamFlag : 1;
    unsigned IPProtocol : 3;
    unsigned Unused : 22;
};

struct WIFI_CONNECT_INFO
{
    struct WIFI_FLAGS Flags;
    uint8_t SecurityType;
    uint16_t IPPort;
    uint32_t IPAddressNumber;
    char IPAddressText[ 16 ];
    char RouterSSIDName[ 32 ];
    char RouterPassword[ 32 ];
};

struct WIFI_INFO
{
    int32_t State, Protocol;
    uint32_t ResentPackets, LostPackets;
    struct WIFI_IP_INFO TCPConnectInfo, UDPConnectInfo, MulticastConnectInfo;
    struct WIFI_CONNECT_INFO StartupConnectInfo, RunningConnectInfo;
};

struct NET_PACKET_INFO
{
    unsigned int TotalPackets, TotalBytes;
};

struct NET_INFO
{
    int32_t LocalSocket, RemoteSocket;
    int32_t LocalPort, RemotePort, AddressSize;
    char LocalAddressTextBuffer[ 24 ], RemoteAddressTextBuffer[ 24 ];
    struct sockaddr_in LocalIPAddress, RemoteIPAddress;
    struct NET_PACKET_INFO TxPacketInfo;
    struct NET_PACKET_INFO RxPacketInfo;
};
#endif

// ************************************************************************
// Timing Structures
// ************************************************************************
struct HISTOGRAM_TIMING_INFO
{
    uint32_t Min, Max, BinInterval, StartTicks, DeltaTicks, MinBinValue, MaxBinValue, TotalBinWidth;
    uint32_t PERPacketInterval;
    struct timespec TimeSpecStart;
    uint32_t Histogram[ HISTOGRAM_MAX_NUMBER_OF_TIMING_BINS ];
};

struct HISTOGRAM_RSSI_INFO
{
    int8_t Min, Max, MinBinValue, MaxBinValue;
    int32_t Avg;
    uint32_t BinInterval, TotalBinWidth;
    uint32_t Histogram[ HISTOGRAM_MAX_NUMBER_OF_RSSI_BINS ];
};

// ************************************************************************
// Radio PHY Structures
// ************************************************************************
struct PHY_INFO
{
    char *TextStringPtr;
    uint32_t ActualBitRate;
    uint32_t NumberOfPacketsToSend;
    uint16_t BytesPerPacketArray[ MAX_INDEX_OF_BYTES_PER_PACKET ];
    uint32_t PERBitRate[ 2 ][ MAX_INDEX_OF_BYTES_PER_PACKET ];
};

// ************************************************************************
// Radio Structures
// ************************************************************************
struct RADIO_CHANNEL_INFO
{
    uint8_t AutoSendFlag, PrintFlag;
    int8_t RSSI;
    uint32_t UnderFlowCtr, OverFlowCtr;
    uint32_t TotalPackets, TotalBytes, Bandwidth, ActualBitRate, Time_sec;
//    uint8_t FIFOBuffer[ 128 ];
};

struct RADIO_ACK_INFO
{
    bool AckEnableFlag, AckReceivedFlag;
    uint8_t ContinuityCounter, PacketRetransmitCtr, MaxNumberOfPacketRetransmissions;
    uint32_t TotalSentPacketCtr, TotalReceivedPacketCtr, TotalAckTimeoutPacketCtr, TotalValidCCPacketCtr, TotalInvalidCCPacketCtr;
    uint32_t TotalNumberOfPacketRetransmissions, TotalNumberOfRetransmissionFailures;
    uint32_t RetransmitHistogram[ MAX_NUMBER_OF_ACK_RETRANSMITS + 2 ];
    uint32_t SemaWaitClockTicks[ RADIO_PHY_MAX ];
};

struct RADIO_INFO
{
    bool RunningFlag, ConsolePrintFlag, WaitToSendOnEOFFlag, RemotePHYAckFlag, EndOfPhyChangeMessagesFlag;
    bool RemoteFrequencyAckFlag, EndOfFrequencyChangeMessagesFlag;
    bool AutoResetPhyFlag, RadioTxPacketFlag, RadioRxPacketFlag, BroadcastReceivedFlag, BroadcastGPSFlag;
    char NorthSouthDirection, EastWestDirection;
    uint8_t SendProtocol, TxWriteBankIndex, TxReadBankIndex, ContinuityCounter, ChangePHYCounter, ChangeFrequencyCounter;
    uint8_t DefaultPHYIndex, PHYIndex, ResetPhyCounter;
    uint8_t MotorBoardID;
    int8_t RSSI, TxPowerLevel;
    uint16_t Address;
    int32_t State, Device, Version, ID, XOscFreq, BandwidthSelect;
    uint32_t ResentPackets, LostPackets;
    uint32_t TotalSentPackets;
    uint32_t AudioEncoderPackets, AudioEncoderBytes, AudioEncoderFrames, RadioAudioEncoderBytes;
    uint32_t AudioDecoderPackets, AudioDecoderBytes, AudioDecoderFrames, RadioAudioDecoderBytes;
    uint32_t VideoPackets, VideoBytes, VideoFrames, RadioVideoBytes;
    uint32_t FilePackets, FilePayloadBytes, RadioFileBytes;
    double fFrequency, fDistance, fBearing;
    double fLocalLatitude, fLocalLongitude;
    double fRemoteLatitude, fRemoteLongitude;
    float fTxSecPerPacket;
    char VersionTextString[ 32 ];
    struct COMP_DATA LocalLatitude, LocalLongitude;
    struct COMP_DATA RemoteLatitude, RemoteLongitude;
//    SPI_Handle hMasterSPI;
    struct PER_INFO PERInfo;
    uint8_t TxBuffer[ 2 ][ MAX_RADIO_PAYLOAD_LENGTH ];
    struct RADIO_CHANNEL_INFO TxChannel, RxChannel;
    struct RADIO_ACK_INFO AckInfo;
    struct HISTOGRAM_TIMING_INFO HistogramTimingInfo;
    struct HISTOGRAM_RSSI_INFO HistogramRSSIInfo;
//    struct PRINT_INFO PrintInfo;
};

struct RX_STATS_INFO
{
    uint32_t OkPacketCtr, BadCRCPacketCtr, QuasiOkPacketCtr, AbortPacketCtr;
};

struct CMD_EVENT_COUNTER
{
    uint32_t CmdDoneCtr;
    uint32_t LastCmdDoneCtr;
    uint32_t RxEntryDoneCtr;
    uint32_t TxEntryDoneCtr;
    uint32_t CancelCmdCtr;
    uint32_t AbortCmdCtr;
    uint32_t StopCmdCtr;
};

struct CMD_STATUS_COUNTER
{
    uint32_t OKCtr;
    uint32_t RxTimeOutCtr;
    uint32_t BreakCtr;
    uint32_t EndTriggerCtr;
    uint32_t StoppedCtr;
    uint32_t AbortCtr;
    uint32_t CRCErrorCtr;
    uint32_t CSIdleCtr;
    uint32_t CSBusyCtr;
    uint32_t CSIdleTimeOutCtr;
    uint32_t CSBusyTimeOutCtr;
    uint32_t IllegalParamCtr;
    uint32_t NoRxBufferCtr;
    uint32_t RxBufferFullCtr;
    uint32_t NoSetupCtr;
    uint32_t FSNotProgCtr;
    uint32_t RxOverflowCtr;
    uint32_t TxUnderflowCtr;
    uint32_t UnknownCtr;
};

struct PACKET_INFO
{
    bool TxSuccessFlag, RxSuccessFlag;
    int8_t RxRSSIValue, RxStatusValue;
    uint8_t TxLocalRetransmitCtr, RxContinuityCounter, TxContinuityCounter;
    uint32_t TxPacketCtr, TxRetransmitOverrunCtr, TxTotalRetransmitCtr;
    uint32_t RxPacketCtr, RxTimeOutCtr, RxBadCC, ErrorCtr, CSBranchSkipCtr;
    struct RX_STATS_INFO RxStatsInfo;
    struct CMD_EVENT_COUNTER CmdEvent;
    struct CMD_STATUS_COUNTER CSCmdStatus;
    struct CMD_STATUS_COUNTER RxCmdStatus;
    struct CMD_STATUS_COUNTER TxCmdStatus;
};

// *****************************************************************************
// BLE
// *****************************************************************************
#if defined ( USE_BLE_RADIO )

struct PRINT_LIST {
    Queue_Elem elem;
    uint8_t *TextPtr;
    uint32_t Size;
};

struct PRINT_INFO
{
    bool CurrentlyPrintingFlag;
    Queue_Handle PrintQueue;
    uint32_t ListIndex;
    struct PRINT_LIST PrintList[ MAX_NUMBER_OF_UART_TEXT_STRINGS ];
    uint32_t BufferIndex;
    uint8_t PrintBuffer[ MAX_SIZE_OF_UART_PRINT_BUFFER ];
};

struct BLE_INFO
{
    uint32_t TotalSentPackets, TotalReceivedPackets;
    struct PRINT_INFO PrintInfo;
};

struct BLE_DEVICE_INFO
{
    unsigned Address_0 : 8;
    unsigned Address_1 : 8;
    unsigned Address_2 : 8;
    unsigned Address_3 : 8;
    unsigned Address_4 : 8;
    unsigned Address_5 : 8;
    unsigned JoinedFlag : 1;
    unsigned UnuusedFlags : 7;
    unsigned JoinedCtr : 8;
    unsigned UnjoinedCtr : 8;
    signed RSSI : 8;
    unsigned Handle : 16;
    unsigned ScanNumber : 16;
    unsigned Latency : 16;
};

struct BLE_CENTRAL_INFO
{
    bool EnableMTUFlag, EnableNotificationFlag, EnableGATTWriteFlag;
    uint32_t CurrentJoinedDevices, TotalJoinedCtr, TotalUnjoinedCtr, TotalConnectedDevices, TotalScans, DeviceArrayIndex;
    uint32_t RSSIReceivedMask;
    struct BLE_DEVICE_INFO DeviceInfo[ MAX_NUMBER_OF_BLE_DEVICES ];
    uint8_t DataArray[ MAX_NUMBER_OF_BLE_ARRAYS ][ MAX_NUMBER_OF_BLE_DEVICES ][ 64 ];
    uint32_t StartTicks[ MAX_NUMBER_OF_BLE_DEVICES ];
    uint32_t Latency[ MAX_NUMBER_OF_BLE_DEVICES ];
};

#endif

// ************************************************************************
// Sensor Generic Structures
// ************************************************************************
struct COMP_STATUS
{
    unsigned DataReceivedFlag : 1;
    unsigned DataAvailableFlag : 1;
    unsigned InitFlag : 1;
    unsigned AutoSendFlag : 1;
    unsigned PrintFlag : 1;
    unsigned FileWriteFlag : 1;
    unsigned EnableFlag : 1;
    unsigned UnusedFlags : 1;
    unsigned ErrorStatus : 8;
    unsigned Unused : 16;
};

struct MODE_INFO
{
    uint8_t NativeProtocol;
    uint8_t CommandProtocol;
    uint8_t DestinationAddress;
    unsigned ClockUpdateFlag : 1;
    unsigned SensorHubConnectedFlag : 1;
    unsigned BatteryConnectedFlag : 1;
    unsigned GPSConnectedFlag : 1;
    unsigned BLE_RSSIUpdateFlag : 1;
    unsigned JoyStickConnectedFlag : 1;
    unsigned UnusedFlags : 26;
};

struct SENSOR_CMD_INFO
{
// Auto Update Flags
    unsigned AutoUpdateGPSFlag : 1;
    unsigned AutoUpdateBatteryFlag : 1;
    unsigned AutoUpdateRSSIFlag : 1;
    unsigned AutoUpdatePressureFlag : 1;
    unsigned AutoUpdateHumidityFlag : 1;
    unsigned AutoUpdateAmbientTemperatureFlag : 1;
    unsigned AutoUpdateIRTemperatureFlag : 1;
    unsigned AutoUpdateLightFlag : 1;
    unsigned AutoUpdateMotionFlag : 1;
    unsigned AutoUpdateIMUFlag : 1;
    unsigned AutoUpdatePIRFlag : 1;
// Unused Auto Update Flags
    unsigned UnusedAutoUpdateFlags : 5;
// Print Flags
    unsigned PrintGPSFlag : 1;
    unsigned PrintBatteryFlag : 1;
    unsigned PrintRSSIFlag : 1;
    unsigned PrintPressureFlag : 1;
    unsigned PrintHumidityFlag : 1;
    unsigned PrintAmbientTemperatureFlag : 1;
    unsigned PrintIRTemperatureFlag : 1;
    unsigned PrintLightFlag : 1;
    unsigned PrintMotionFlag : 1;
    unsigned PrintIMUFlag : 1;
    unsigned PrintPIRFlag : 1;
// Unused Print Flags
    unsigned UnusedPrintFlags : 5;
};

// ***********************************************************************************************************************************
// IMU Structures
// ***********************************************************************************************************************************
struct IMU_INFO
{
    bool IRQFlag;
    uint8_t UpdateRateCtr, UpdateRate, TicksPerUpdate;
    uint16_t SampleRateIndex;
    int16_t Roll, Pitch, Yaw;
    int16_t AngularVelocity_X, AngularVelocity_Y, AngularVelocity_Z;
    struct COMP_STATUS Status;
};

// ***********************************************************************************************************************************
// PIR Structures
// ***********************************************************************************************************************************
struct PIR_INFO
{
    bool MotionEnableFlag, LocalPrintFlag, PrintRawDataFlag, PrintMotionFlag, PrintSendRawDataFlag;
    bool SendRawDataFlag, SendMotionFlag, IRQFlag;
    uint8_t UpdateRate, Threshold, State, ChannelMask;
    int16_t MotionData, BytesToRead;
    uint32_t WriteIndex, ReadIndex, BufferLevel, PacketsSent, NumberOfSamples, MotionStartCtr, MotionEndCtr;
    uint32_t NumberOfIRQs, TotalNumberOfIRQs;
    uint8_t Buffer[ MAX_NUMBER_OF_BYTES_IN_UART_CMD_BUFFER ];
    struct COMP_STATUS Status;
//    UART_Handle hUART;
    struct BUFFER_INFO CommandBufferInfo;
};

// ***********************************************************************************************************************************
// GPS Structures
// ***********************************************************************************************************************************
struct GPS_INFO
{
    bool NMEASentencePrintFlag, NMEA_StringReceivedFlag, NMEA_StringCompleteFlag, NMEA_StringPrintFlag, TTFF_DetectFlag;
    uint8_t State, CheckSum, BufferBankIndex, BufferByteIndex, CheckSumByteIndex, Error, NMEA_StringIndex;
    uint8_t FixAvailableFlag, PassThroughFlag, SatellitesInView, EastWestDirection, NorthSouthDirection, NumberOfMessages;
    uint16_t UpdateRate, PrintRate;
    uint32_t TextCtr, StartTime, TimeToFirstFix;
    char Buffer[ GPS_BUFFERS ][ GPS_MSG_MAX_SIZE ];
    char CheckSumBuffer[ 2 ];
    char *TextStrings[24];
    char LatitudeTextInteger[8], LatitudeTextFraction[6];
    char LongitudeTextInteger[8], LongitudeTextFraction[6];
    char UTCTextInteger[8], UTCTextFraction[6];
    char VelocityTextInteger[6], VelocityTextFraction[6];
    char AltitudeTextInteger[6], AltitudeTextFraction[6];
    char NMEAFileNameString[ 32 ];
    char NMEA_String[ MAX_SIZE_OF_NMEA_STRING ];
#if defined ( USE_SD_CARD )
    char FirmwareFileNameString[ 16 ];
    char SD_NMEAFileNameString[ 32 ];
    uint32_t NMEAFileBytesWritten;
    FILE *FileNMEAStringPtr;
#endif
    struct COMP_STATUS Status;
    struct COMP_DATA ChipLatitude;
    struct COMP_DATA ChipLongitude;
    struct COMP_DATA UTCTime;
    struct COMP_DATA Velocity;
    struct COMP_DATA Altitude;
    struct timespec TimeSpecStartOfTask;
};

#if defined (USE_GPS_UART ) || defined ( USE_GPS_SONY)
//*****************************************************************************
// GPS Message Buffers
//*****************************************************************************
typedef struct gpsBufferStruct
{
    char GPGLLBuffer[ GPS_MSG_MAX_SIZE ];
    char GPRMCBuffer[ GPS_MSG_MAX_SIZE ];
    char GPGGABuffer[ GPS_MSG_MAX_SIZE ];
    char GPVTGBuffer[ GPS_MSG_MAX_SIZE ];
    char GPGSABuffer[ GPS_MSG_MAX_SIZE ];
    char GPGSVBuffer[ GPS_GSV_MAX_MSGS ][ GPS_MSG_MAX_SIZE ];
    char CFWVERBuffer[ GPS_MSG_MAX_SIZE ];
    char GPSVERBuffer[ GPS_MSG_MAX_SIZE ];

} GPSBuffer;
#endif

// ************************************************************************
// INA Structures
// ************************************************************************
struct INA_SAMPLES
{
    uint16_t Current;
    uint16_t Power;
};

struct INA_INFO
{
    bool RunningFlag, ReadDataFlag, AutoUpdateFlag, ConsolePrintFlag, IRQFlag;
    uint8_t UpdateIntervalIndex, DestinationAddress;
    uint16_t OverCurrentLimit, OverVoltageLimit, UnderVoltageLimit, OverPowerLimit;
    uint16_t ADCConfigReg;
    uint32_t SampleIndex, SamplesPerSec, SamplesPerSecCtr, NumberOfIRQs, TotalNumberOfIRQs;
    double fCurrent_mA_PerLSB, fBusVoltsPerLSB, fShuntVoltsPerLSB, fPower_mW_PerLSB;
    struct INA_SAMPLES SampleBuffer[ MAX_NUMBER_OF_INA_SAMPLES ];
};

// ************************************************************************
// Battery2
// ************************************************************************
struct BQ27441_INFO
{
    uint8_t SendProtocol;
    uint16_t AtRateTimeToEmpty, Temperature, Voltage;
    uint16_t NominalAvailableCapacity, FullAvailableCapacity, RemainingCapacity;
    uint16_t FullChargeCapacity, AverageCurrent, TimeToEmpty, StandbyCurrent, StandbyTimeToEmpty;
    uint16_t StateOfHealth, CycleCount, StateOfCharge, InstantaneousCurrent;
    uint16_t InternalTemperature;
    uint16_t UpdateRate, PrintRate;
    uint32_t TickCtr;
    float fTemperature, fVoltage, fAverageCurrent, fStandbyCurrent, fInstantaneousCurrent, fTimeToEmpty, fStandbyTimeToEmpty;
    float fInternalTemperature;
    struct COMP_STATUS Status;
    struct COMP_DATA TemperatureData, VoltageData, AverageCurrentData, StandbyCurrentData, InstantaneousCurrentData, InternalTemperatureData;
};

// ************************************************************************
// Sensor Command Structures
// ************************************************************************
struct SENSOR_INFO
{
    struct MODE_INFO ModeInfo;
    //  struct SENSOR_HUB_INFO SensorHubInfo;
#if defined( USE_GPS_UART ) || defined ( USE_GPS_SONY ) || defined( USE_GPS_I2C )
    struct GPS_INFO GPSInfo;
#endif
#if defined ( USE_GPS_SONY )
    struct SONY_GPS_INFO SonyGPSInfo;
#endif
#if defined BOARD_ID_BATTERY_BP_1
    struct BQ27510_INFO BatteryInfo;
#elif defined BOARD_ID_BATTERY_BP_2
    struct BQ27441_INFO BatteryInfo;
#endif
#if defined BOARD_ID_SENSOR_HUB_1
    struct BMP180_INFO PressureInfo;
    struct SHT21_INFO HumidityInfo;
    struct TMP006_INFO TemperatureInfo;
    struct ISL29023_INFO LightInfo;
    struct MPU9150_INFO MotionInfo;
#elif defined( BOARD_ID_SENSOR_HUB_2 ) || defined( BOARD_ID_SENSOR_TAG_1350 )
    struct BME280_INFO PressureInfo;
    struct BME280_INFO HumidityInfo;
    struct TMP007_INFO TemperatureInfo;
    struct OPT3001_INFO LightInfo;
    struct BMI160_INFO MotionInfo;
#elif defined( USE_SAV_BP)
    struct OPT3004_INFO LightInfo;
#endif
#if defined ( USE_IMU )
    struct IMU_INFO IMUInfo;
#endif
#if defined ( USE_PIR )
    struct PIR_INFO PIRInfo;
#endif
    uchar SendSensorBuffer[256];
};

// *****************************************************************************
// SPI
// *****************************************************************************
#if defined( USE_SPI_CMD )
struct SPI_TASK_INFO
{
    bool XferCompleteFlag;
    uint32_t State, BytesToTransfer, NumberOfReadRequests, NumberOfMessages, NumberOfMasterReqs;
    uint32_t TxBufferIndex, RxBufferIndex;
    uint32_t ValidSPITransfers, InvalidSPITransfers, XferCallbackCtr, AuxReadReqISRCtr;
    uint32_t ValidSPICommands, InvalidSPICommands, MessagesSent;
    uint32_t ValidCommandsReceived, InvalidCommandsReceived, MessageSendDataCtr, MessageReceiveDataCtr;
//    SPI_Handle hSlaveSPI;
    uint8_t RcvBuffer[ SPI_MAX_NUMBER_OF_BUFFERS ][ SPI_TRANSFER_SIZE ];
    uint8_t TxtBuffer[ SPI_MAX_NUMBER_OF_BUFFERS ][ SPI_TRANSFER_SIZE ];
};
#endif

// ***********************************************************************************************************************************
// UART Structures
// ***********************************************************************************************************************************
#if defined( USE_UART_CMD )
struct UART_CMD_INFO
{
    uint8_t UpdateRate, Threshold, State, ChannelMask;
    int16_t BytesToRead;
    uint32_t WriteIndex, ReadIndex, BufferLevel, PacketsSent, NumberOfSamples;
    uint8_t Buffer[ MAX_NUMBER_OF_BYTES_IN_COMMAND_BUFFER ];
    UART2_Handle hUART;
    struct BUFFER_INFO CommandBufferInfo;
};
#endif

//#if !defined ( USE_BLE_RADIO )
// ************************************************************************
// I2C Structures
// ************************************************************************
#if defined ( USE_I2C )
struct I2C_INFO
{
    uint32_t InitCounter;
    I2C_Handle hI2CDevice;
    pthread_mutex_t hMutex;
    uint8_t TxBuffer[ 16 ];
    uint8_t RxBuffer[ 32 ];
};
#endif
//#endif

// ************************************************************************
// SPI Structures
// ************************************************************************
#if defined ( USE_HOST_OVER_SPI )
    struct SPI_TASK_INFO
    {
        bool XferCompleteFlag;
        uint32_t State, BytesToTransfer, NumberOfReadRequests, NumberOfMessages, NumberOfSlaveAcks, NumberOfMasterReqs;
        uint32_t TxBufferIndex, RxBufferIndex;
        uint32_t ValidSPITransfers, InvalidSPITransfers, XferCallbackCtr, AuxReadReqISRCtr;
        uint32_t ValidSPICommands, InvalidSPICommands, MessagesSent;
        uint32_t ValidCommandsReceived, InvalidCommandsReceived, MessageSendDataCtr, MessageReceiveDataCtr;
        SPI_Handle hSlaveSPI;
        uint8_t RcvBuffer[ SPI_MAX_NUMBER_OF_BUFFERS ][ SPI_TRANSFER_SIZE ];
        uint8_t TxtBuffer[ SPI_MAX_NUMBER_OF_BUFFERS ][ SPI_TRANSFER_SIZE ];
    };
#endif
// ************************************************************************
// SD Card Structure
// ************************************************************************
struct SD_CARD_DIR_FILE_INFO
{
    bool SDCardFoundFlag;
    int16_t Directories;
    int16_t Files;
    int16_t DirStringIndex, FileStringIndex;
    char DirString[ MAX_SIZE_OF_DIR_STORAGE ];
    char FileString[ MAX_SIZE_OF_FILE_STORAGE ];
    uint16_t SizeOfCommandBuffer;
    uint8_t *CommandBufferPtr;
};

struct SDCARD_FILE_INFO
{
    uint8_t FileNameText[ MAX_SDCARD_FILENAME_TEXT_LENGTH ];
    uint32_t FileNameLength;
    uint32_t FileSize;
};

// ************************************************************************
// File Streaming Structures
// ************************************************************************
struct FILE_STREAMING_INFO
{
    bool RunningFlag, RequestToStopFlag, StartOfFileFlag, EndOfFileFlag;
    uint8_t RxContinuityCtr, TxContinuityCtr, DestinationBoardID, SendProtocol;
    uint16_t PayloadSize;
    uint32_t TxDesiredBitRate, TxFileNameLength, TxFileSize;
    uint32_t RxNewFileNameLength, RxFileNameLength, RxFileSize, RxFileOffset, RxPackets, RxFilePrintThreshold;
    uint32_t RxStartTicks, RxTimeSec, RxActualBitRate, ReceiveTotalFrameBytes;
    uint32_t RxTotalPacketCtr, RxTotalPacketBytes;
    uint32_t TxTotalPacketCtr, TxTotalPacketBytes;
    uint32_t MissingCRCCtr, UDPBankIndex, UDPByteIndex;
    FILE *TxFilePtr, *RxFilePtr;
    char TxFileName[ 32 ], RxFileName[ 32 ], RxNewFileName[ 32 ];
#if defined ( USE_HOST_OVER_SPI )
    uint8_t UDPBuffer[ 2 ][ 1500 ];
#endif
    struct BUFFER_INFO BufferInfo;
};


// ************************************************************************
// Audio
// ************************************************************************
struct AUDIO_FILE_INFO
{
    uint32_t NumberOfMediaFiles;
    uint32_t FilePlayerIndex;
    struct SDCARD_FILE_INFO CaptureFileInfo;
    struct SDCARD_FILE_INFO PlaybackFileInfo;
    struct SDCARD_FILE_INFO EncoderOutputFileInfo;
    struct SDCARD_FILE_INFO DecoderOutputFileInfo;
    struct SDCARD_FILE_INFO *MediaFileInfoPtr[ MAX_SDCARD_NUMBER_OF_FILES ];
};

struct __attribute__((packed)) AUDIO_PACKET_HEADER
{
    unsigned SyncByte : 8;
    unsigned Command : 5;
    unsigned PayloadBytes : 11;
    unsigned AckFlag : 1;
    unsigned SendProtocol : 3;
    unsigned CCTR : 4;
    unsigned SourceAddress : 8;
    unsigned DestinationAddress : 8;
    unsigned SubCommand : 8;
    unsigned ContinuityCtr : 3;
    unsigned PlayFlag : 1;
    unsigned StartOfFrameFlag : 1;
    unsigned EndOfFrameFlag : 1;
    unsigned InitFlag : 1;
    unsigned EndFlag : 1;
    unsigned SampleRateCode : 4;
    unsigned StereoFlag : 1;
    unsigned CodecType : 3;
    unsigned FrameSize : 16;
    unsigned CheckSum : 8;
};

struct __attribute__((packed)) AUDIO_SOF_HEADER
{
    unsigned FrameSize : 16;
    unsigned TimeStamp : 32;
};

struct AUDIO_PACKET_DATAGRAM
{
    struct AUDIO_PACKET_HEADER Header;
    uint8_t ByteArray[4];
};

// ************************************************************************
// Video
// ************************************************************************
struct __attribute__((packed)) VIDEO_PACKET_HEADER
{
    unsigned SyncByte : 8;
    unsigned Command : 5;
    unsigned PayloadBytes : 11;
    unsigned AckFlag : 1;
    unsigned SendProtocol : 3;
    unsigned CCTR : 4;
    unsigned SourceAddress : 8;
    unsigned DestinationAddress : 8;
    unsigned SubCommand : 8;
    unsigned CheckSum : 8;
    unsigned StartOfFrameFlag : 1;
    unsigned EndOfFrameFlag : 1;
    unsigned StartOfFileFlag : 1;
    unsigned EndOfFileFlag : 1;
    unsigned InitFlag : 1;
    unsigned PictureType : 1;
    unsigned CodecType : 2;
    unsigned ImageType : 4;
    unsigned FrameSize : 20;
};

struct VIDEO_PACKET_DATAGRAM
{
    struct VIDEO_PACKET_HEADER Header;
    uint8_t Payload[ ];
};

// *****************************************************************************
// end of file
// *****************************************************************************
