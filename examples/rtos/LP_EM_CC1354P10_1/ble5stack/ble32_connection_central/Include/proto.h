// *****************************************************************************
// proto.h
// *****************************************************************************

void socket_ClientExecute(void* pValue);
void socket_ServerExecute(void* pValue);

int32_t ProcessCommand( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr, struct CMD_PACKET_DATAGRAM *DestPacketDatagramPtr, int SendProtocol );
int32_t SendCommandDataToNetwork( uint16_t Message, struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr, uint8_t WirelessProtocol );

// *****************************************************************************
// Threads
// *****************************************************************************
int32_t CreateUARTConsoleThread( void );
void *UARTConsoleThread(void *args);

int32_t CreateMessageThread( void );
void *MessageThread(void *args);

int32_t CreateHostSPIThread( void );
void *pThreadTaskSPIMaster( void *arg );

int32_t CreateTimerThread( void );
void *TimerThread(void *args);

int32_t CreateTCPListenThread( void );
void *TCPListenThread( void *args );
void *TCPServerReceiveThread( void *args );
void *TCPServerTransmitThread( void *args );

int32_t CreateUDPBroadcastThread( void );
void *UDPBroadcastThread( void *args );

int32_t CreateSPISlaveThread( void );
void *SPISlaveThread( void *args );

int32_t CreateUARTCommandThread( void );
void *UARTCommandThread( void *args );

int32_t CreateBLEMessageThread( void );
void *BLEMessageThread(void *args);

int32_t CreateBLETimerThread( void );
void *BLETimerThread( void *args );

int32_t CreateSlaveSPIThread( void );
void *SlaveSPIThread( void *arg );

int32_t CreateRadioRxThread( void );
void *RadioRxThread(void *arg0);

int32_t CreateINA23xThread( void );
void *pThreadTaskINA236( void *arg );

int32_t CreateRadioTxAndRxThreads( void );
void *pThreadTaskRadioReceive( void *arg );
void *pThreadTaskRadio( void *arg );
void *pThreadTaskRadioSinglePERTest( void *arg );
void *pThreadTaskRadioMultiPERTest( void *arg );

int32_t CreateGPSThread( void );
void *pThreadTaskUART_GPS( void *arg );

int32_t CreateSensorThread( void );
void *pThreadSensors( void *arg );

int32_t CreateHostSPIThread( void );
void *HostSPIThread( void *arg );

int32_t CreateFileReadThread( void );
void *FileReadThread( void *arg );

// *****************************************************************************
// Time utils
// *****************************************************************************
uint32_t GetDelta_msec( uint32_t Start_msec );
uint32_t GetDeltaTicks( uint32_t StartTicks );
uint32_t GetDelta_usec( struct timespec *NewTimeSpecPtr, struct timespec *PreviousTimeSpecPtr );
void Convert_msecToTimeSpec( uint32_t msec, struct timespec *TimeSpecPtr );
void Convert_usecToTimeSpec( uint32_t usec, struct timespec *TimeSpecPtr );
void DeltaTimeSpec( struct timespec *t1, struct timespec *t2, struct timespec *td );
double GetTimeSpecElapsedSeconds( struct timespec *TimeSpecPtr );
double GetTimeSpecElapsedMilliSeconds( struct timespec *TimeSpecPtr );

// *****************************************************************************
// Buffers
// *****************************************************************************
uint8_t *GetBufferReadPtr(struct FILE_INFO *FileInfoPtr, struct BUFFER_INFO *BufferInfoPtr, int BytesToRead);
uint8_t *GetBufferWritePtr(struct BUFFER_INFO *BufferInfoPtr, int BytesToWrite);
int GetBufferLevel( struct BUFFER_INFO *BufferInfoPtr );
int AllocateBufferInfo(struct BUFFER_INFO *BufferInfoPtr, int TotalBufferSize, int FloorSize);
void AdvanceBufferReadPtr(struct BUFFER_INFO *BufferInfoPtr, int Bytes);
void AdvanceBufferWritePtr(struct BUFFER_INFO *BufferInfoPtr, int Bytes);
uint8_t *GetContiguousReadPtr( struct BUFFER_INFO *BufferInfoPtr, int BytesToRead );
uint8_t *GetContiguousAuxReadPtr( struct BUFFER_INFO *BufferInfoPtr, uint8_t *DestPtr, int BytesToRead );
uint32_t CreateMessageWithBuffer( uint8_t Command, uint8_t SubCommand, uint8_t DestinationAddress, uint8_t *CmdPtr, uint8_t *SrcPtr,
                                  uint32_t BytesToTransfer, uint8_t SendProtocol );
uint32_t AppendMessageWithBuffer( uint8_t *CmdPtr, uint8_t *SrcPtr, uint32_t BytesToTransfer );
void SwapBytes( uint8_t *CharPtr, int Bytes );

// *****************************************************************************
// UART
// *****************************************************************************
UART2_Handle OpenUARTChannel( uint8_t UARTIndex );
UART2_Handle OpenUARTChannelAtBaudRate( uint8_t UARTIndex, uint32_t BaudRate );
bool ProcessConsoleCommand( char Key, int32_t NumberOfParameters, char **ParametersPtr );
int32_t DisplayAppBanner( char* appName, char* appVersion );
void DisplayColorPrintString( char *ColorStringPtr, char *TextStringPtr, uint32_t StringLength, char *ColorString );

// *****************************************************************************
// SPI
// *****************************************************************************
void ToggleSPIHostIRQRequestToRead( void );

// *****************************************************************************
// I2C Routines
// *****************************************************************************
bool InitI2C( void );
bool I2CWrite( uint8_t SlaveAddress, uint8_t RegAddress, uint8_t *DataPtr, uint8_t NumberOfBytes );
bool I2CRead( uint8_t SlaveAddress, uint8_t RegAddress, uint8_t *DataPtr, uint8_t NumberOfBytes );
bool I2CReadOnly( uint8_t SlaveAddress, uint8_t *DataPtr, uint8_t NumberOfBytes );
bool I2C_Write(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data, uint32_t ui8ByteCount);
bool I2C_Read(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data, uint32_t ui8ByteCount);
bool I2C_WriteWithoutRegister( uint8_t SlaveAddress, uint8_t *DataPtr, uint32_t NumberOfBytes );
bool I2C_ReadWithoutRegister( uint8_t SlaveAddress, uint8_t *DataPtr, uint32_t NumberOfBytes );

// *****************************************************************************
// Network
// *****************************************************************************
struct NET_INFO *CreateUDPServer( uint16_t Port );
int SendUDPData( struct NET_INFO *UDPInfoPtr, uint8_t *DataPtr, uint32_t Size );
struct NET_INFO *CreateUDPClient( uint32_t IPAddress, uint16_t Port );
int CloseUDPClientSocket( struct NET_INFO *UDPInfoPtr );
unsigned long convert_inet_address_string_to_number( char *AddressString );

// *****************************************************************************
// Radio
// *****************************************************************************
int ProcessRadioCommand( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr, struct CMD_PACKET_DATAGRAM *DestPacketDatagramPtr, int SendProtocol );
int ProcessLocalRadioCommand( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr, int SendProtocol );
void CreateRadioTransmitCommandChain( void );
void CreateRadioReceiveCommandChain( void );
uint8_t *SendRadioSmartRFStudioPERPacket( uint16_t SequenceNumber, uint8_t PacketLength );
struct CMD_PACKET_DATAGRAM *SendRadioPERPacket( uint8_t AckIndex, uint8_t FileIndex );
void SendBufferToRadioPacket( uint8_t *DataPtr, uint8_t Size );
void CopyToRadioBufferAndSendRadioPacket( uint8_t *DataPtr, uint32_t Size );
uint8_t CreateRadioAckPacket( uint8_t *BufferPtr, uint8_t ContinuityCounter );
uint32_t SendRadioTxPERRegs( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr, struct CMD_PACKET_DATAGRAM *DestPacketDatagramPtr, uint8_t SendProtocol );
uint32_t DisplayRadioTxPERRegs( struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr );
void SendRadioBLEAdvertisingPacket( void );
void SendRadioBroadcastPacket( uint8_t SendProtocol );
uint32_t SendRadioBroadcastAckPacket( struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr );
void SendRadioChangeLocalPHY( uint8_t PHYIndex );
void SendRadioChangeRemotePHYPacket( uint8_t PHYIndex );
uint32_t SendRadioChangePHYAckPacket( struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr );
void SendRadioChangeRemoteFrequencyPacket( double Frequency );
uint32_t SendRadioChangeFrequencyAckPacket( struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr );
void AddBroadcastBoardIDToQueue( struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr );
int SendRadioProtocol( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr, struct CMD_PACKET_DATAGRAM *DestPacketDatagramPtr, uint8_t SendProtocol );
int SendRadioLocalPHY( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr, struct CMD_PACKET_DATAGRAM *DestPacketDatagramPtr, uint8_t SendProtocol );
uint32_t SendRadioTxPERStatus( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr, struct CMD_PACKET_DATAGRAM *DestPacketDatagramPtr, uint8_t SendProtocol );
uint32_t DisplayRadioTxPERStatus( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr );
uint32_t SendRadioRxPERStatus( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr, struct CMD_PACKET_DATAGRAM *DestPacketDatagramPtr, uint8_t SendProtocol );
uint32_t DisplayRadioRxPERStatus( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr );
uint32_t SendRadioAckRetransmissionArray( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr, struct CMD_PACKET_DATAGRAM *DestPacketDatagramPtr, uint8_t SendProtocol );
uint32_t DisplayRadioAckRetransmissionArray( struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr );
void SendRadioAckPacket( uint8_t ContinuityCtr );
uint32_t SendRadioAckStatus( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr, struct CMD_PACKET_DATAGRAM *DestPacketDatagramPtr, uint8_t SendProtocol );
uint32_t DisplayRadioAckStatus( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr );
void SendRadioBroadcastGPSPacket( uint8_t SendProtocol );
uint32_t SendRadioTxGPSStatus( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr, struct CMD_PACKET_DATAGRAM *DestPacketDatagramPtr, uint8_t SendProtocol );
uint32_t DisplayRadioTxGPSStatus( struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr );
void SendRadioTestPacket( uint16_t RadioAddress );
void SendRadioTextPacket( void );
void SendRadioLEDPacket( void );
void SendRadioStartSinglePERTestPacket( void );
void StartRadioSinglePERTest( void );
int SendRadioCommandData( struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr, int WirelessProtocol );
int SendHostCommandData( struct CMD_PACKET_DATAGRAM *CmdPacketDatagramPtr, uint32_t BytesToSend, uint8_t WirelessProtocol );
int SendRadioAudioData( struct AUDIO_PACKET_DATAGRAM *AudioPacketDatagramPtr, int WirelessProtocol );
int SendRadioVideoData( struct VIDEO_PACKET_DATAGRAM *VideoPacketDatagramPtr, int WirelessProtocol );
void CreateRadioGenericMessage( uint8_t Command, uint8_t SubCommand, uint8_t DestinationAddress, uint8_t *CmdPtr, uint8_t *SrcPtr,
                                uint32_t BytesToTransfer, uint8_t SendProtocol );
int SendRadioGenericPacket( struct CMD_PACKET_DATAGRAM *SrcPacketDatagramPtr, struct CMD_PACKET_DATAGRAM *DestPacketDatagramPtr,
                            uint8_t Command, uint8_t SubCommand, uint8_t *SrcPtr, uint32_t BytesToTransfer, uint8_t SendProtocol );
void UpdateCommandEventCounter( struct CMD_EVENT_COUNTER *CmdEventCounter, uint64_t EventMask );
void UpdateCommandPacketCounter( struct CMD_STATUS_COUNTER *CmdStatusCounter, uint32_t Status );
void WritePERTransmitterTestResults( uint8_t AckIndex, uint8_t BytePerPacketIndex );
void WritePERReceiverTestResults( uint32_t TestSequenceCtr, uint32_t TotalPackets, uint8_t AckIndex, uint8_t BytePerPacketIndex, uint8_t FilePhaseIndex );
uint8_t GetBytesPerPacketIndex( uint16_t BytesPerPacket );
void TaskRadioPER_MultiTest( void );
void SetPER_Parameters( bool AckEnableFlag, uint8_t AckIndex, uint8_t BytesPerPacketIndex );
void RunRadioPER_MultiTest( uint8_t AckIndex, uint8_t PhaseIndex );
void SendPacketToRadioSoC( uint8_t *DataPtr, uint8_t Size );
void ToggleSPIHostIRQRequestToRead( void );
void DisplayRadioStatus( char *TextStringPtr, char *ColorStringPtr );
uint16_t Return16BitRandomValue( void );

// *****************************************************************************
// BLE
// *****************************************************************************
void BLEHostCommand( uint32_t Command, uint32_t SubCommand );
struct BLE_DEVICE_INFO *GetDeviceInfo( uint16_t Handle, uint8_t *IndexPtr );

// *****************************************************************************
// Data
// *****************************************************************************
void ConvertDoubleToCompositeData( double fData, uint32_t SignificantDigits, struct COMP_DATA *CompDataPtr );
void ConvertCompositeDataToDouble( struct COMP_DATA *CompDataPtr, uint32_t SignificantDigits, double *fDataPtr );
int32_t CompareFiles( char *FileName1, char *FileName2 );

// *****************************************************************************
// SD Card
// *****************************************************************************
bool ValidateFileType( char *FileName, char *ExtensionStringPtr );
int32_t fatfs_getFatTime( void );
int32_t ScanForFileType( char *ExtensionStringPtr );
int32_t scan_files( char *path );
int32_t GetDirAndFiles( void );

// *****************************************************************************
// INA236
// *****************************************************************************
bool InitINA236( void );

// *****************************************************************************
// Time
// *****************************************************************************
uint32_t osi_GetTimeMS( void );
uint32_t GetDelta_msec( uint32_t Start_msec );

// *****************************************************************************
// Messages
// *****************************************************************************
uint32_t CreateMessageOnly( uint8_t Command, uint8_t SubCommand, uint8_t DestinationAddress, uint8_t *CmdPtr, uint8_t SendProtocol );
uint32_t CreateMessageWithBuffer( uint8_t Command, uint8_t SubCommand, uint8_t DestinationAddress, uint8_t *CmdPtr, uint8_t *SrcPtr, uint32_t BytesToTransfer, uint8_t SendProtocol );
uint32_t AppendMessageWithBuffer( uint8_t *CmdPtr, uint8_t *SrcPtr, uint32_t BytesToTransfer );
uint32_t CreateMessageWithStringAndLength( uint8_t Command, uint8_t SubCommand, uint8_t DestinationAddress, uint8_t *CmdPtr, uint8_t *TextPtr, uint32_t TextLength, uint8_t SendProtocol );
void SendHostFrameShortMessage( uint16_t Message, char *FramePtr, uint16_t FrameBytes, uint16_t ShortValue );

// *****************************************************************************
// Sensors
// *****************************************************************************
int SendSensorData( uchar *BufferPtr );

// *****************************************************************************
// GPS
// *****************************************************************************
void GpsAppsMenu( void );
void GPSDataDisplayMenu( void );
void ProcessGPSByte( uchar Byte );
int ParseGPSDataString( char *GPSTextPtr );
void SendPMTKCommand( int Command, int Value );
int ComputeGPSCheckSum( char *BufferPtr );
uint8_t CalculateNMEA_StringChecksum( char *StringPtr, uint8_t Length );
float GPSStringToDecimalDegrees( const char* NMEATextString, char quadrant );
double CalcDistanceGPS( double Lat1, double Long1, double Lat2, double Long2, double *BearingPtr );
double GetDistanceGPS( double Lat1, double Long1, double Lat2, double Long2 );

// *****************************************************************************
// end of file
// *****************************************************************************
