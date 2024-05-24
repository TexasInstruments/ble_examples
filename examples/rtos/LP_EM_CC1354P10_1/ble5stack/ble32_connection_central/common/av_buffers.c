// ****************************************************************************
// av_buffers.c 
// ****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#if defined ( USE_WIFI_RADIO )
#include "lwip/sockets.h"
#endif
#if defined ( USE_BLE_RADIO )
#include <ti/sysbios/knl/Mailbox.h>
#endif

#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>
#include <ti/display/Display.h>
#include <ti/drivers/SDFatFS.h>
#include <third_party/fatfs/ffcio.h>

#include <semaphore.h>

#include "define.h"
#include "structs.h"
#include "externs.h"
#include "proto.h"

void SwapBytesInShortWord( uint16_t *ShortPtr, int NumberOfShortWords )
{
	int Index;
	uint8_t TempByte;
	uint8_t *BytePtr;

	BytePtr = ( uint8_t * )ShortPtr;
	for( Index = 0; Index < NumberOfShortWords; Index++ )
	{
		TempByte = BytePtr[ 0 ];
		BytePtr[ 0 ] = BytePtr[ 1 ];
		BytePtr[ 1 ] = TempByte;
		BytePtr += 2;

	}
	return;
}

/*****************************************************************************************************/
// This routine swaps bytes within the same buffer.  It assumes that data is read in words at a time 
// and takes care of residual at the end of the buffer. 
/*****************************************************************************************************/
void SwapBytes( uint8_t *CharPtr, int Bytes )
{
	int i,j,Words,Residual;
	uint8_t Byte[4];

	Words = Bytes/4;
	Residual = Bytes % 4;
	if(Residual > 0)
		Words++;
	for(i=0; i<Words; i++)
	{
		for(j=0; j<4; j++)
			Byte[j] = CharPtr[j];
		for(j=0; j<4; j++)
			CharPtr[j] = Byte[3-j];
		CharPtr += 4;
	}
/*
	Residual = Bytes % 4;
	if(Residual > 0)
	{
		for(j=0; j<Residual; j++)
			Byte[j] = CharPtr[j];
		j=0;
		for(j=0; j<Residual; j++)
			CharPtr[j] = Byte[3-j];
	}
*/
}
/*
struct AUDIO_INFO *AllocateAudioInfoMemory(int BufferSize, int FloorSize)
{
	struct AUDIO_INFO *AudioInfoPtr;

	AudioInfoPtr = (struct AUDIO_INFO *)calloc(1,sizeof(struct AUDIO_INFO));
	if(AudioInfoPtr == NULL)
		return NULL;
	AudioInfoPtr->BitBufferSize = AllocateBufferInfo(&AudioInfoPtr->BufferInfo, BufferSize, FloorSize);
	if(AudioInfoPtr->BitBufferSize < 0)
		return NULL;
	else
	{
		AudioInfoPtr->BytesPerSec.Min = AudioInfoPtr->FramesPerSec.Min = 0xFFFFFFFF;
		AudioInfoPtr->PlayTime = 0.0;
//		AudioInfoPtr->MinBytesPerSec = 0xFFFFFFFF;
		return AudioInfoPtr;
	}
//	AudioInfoPtr->BitBufferPtr = (char *)AudioInfoPtr->BufferStruct.BeginPtr;
//	return 0;
}

int ResetAudioInfoMemory( struct AUDIO_INFO *AudioInfoPtr, int BufferSize, int FloorSize )
{
	free( AudioInfoPtr->BufferInfo.BottomPtr );
	ZeroMemory( AudioInfoPtr, sizeof( struct AUDIO_INFO ) );
	AudioInfoPtr->BitBufferSize = AllocateBufferInfo(&AudioInfoPtr->BufferInfo, BufferSize, FloorSize );
	if(AudioInfoPtr->BitBufferSize < 0)
		return -1;
	else
	{
		AudioInfoPtr->BytesPerSec.Min = AudioInfoPtr->FramesPerSec.Min = 0xFFFFFFFF;
		AudioInfoPtr->PlayTime = 0.0;
//		AudioInfoPtr->MinBytesPerSec = 0xFFFFFFFF;
		return 0;
	}
//	AudioInfoPtr->BitBufferPtr = (char *)AudioInfoPtr->BufferStruct.BeginPtr;
//	return 0;
}
*/
int AllocateBufferInfo(struct BUFFER_INFO *BufferInfoPtr, int TotalBufferSize, int FloorSize)
{
	int Buffers,Residual;

	if( FloorSize != 0 )
	{
		Buffers = TotalBufferSize/FloorSize;
		Residual = TotalBufferSize % FloorSize;
		if( Residual != 0 )
			TotalBufferSize = ( Buffers * FloorSize ) + FloorSize;
		if( TotalBufferSize < 2 * FloorSize )
			TotalBufferSize = 2 * FloorSize;
	}
	if( BufferInfoPtr->BottomPtr == NULL )
	{
		BufferInfoPtr->BottomPtr = calloc( 1, TotalBufferSize );
		if(BufferInfoPtr->BottomPtr == NULL)
			return -1;
	}
	BufferInfoPtr->TopPtr = BufferInfoPtr->BottomPtr + TotalBufferSize;
	BufferInfoPtr->BeginPtr = BufferInfoPtr->BottomPtr + FloorSize;
	BufferInfoPtr->WritePtr = BufferInfoPtr->BeginPtr;
	BufferInfoPtr->ReadPtr = BufferInfoPtr->BeginPtr;
	BufferInfoPtr->WriteBufferSize = TotalBufferSize - FloorSize;
	BufferInfoPtr->TotalBufferSize = TotalBufferSize;
	BufferInfoPtr->FloorSize = FloorSize;
	BufferInfoPtr->ReadByteIndex = 0;
	BufferInfoPtr->WriteByteIndex = 0;
	BufferInfoPtr->TotalBytesWritten = 0;
	BufferInfoPtr->WrapValue = 0;
	BufferInfoPtr->BufferLevel = 0;
	return TotalBufferSize;
}

void ResetBuffer(struct BUFFER_INFO *BufferInfoPtr)
{
	BufferInfoPtr->WritePtr = BufferInfoPtr->BeginPtr;
	BufferInfoPtr->ReadPtr = BufferInfoPtr->BeginPtr;
	BufferInfoPtr->ReadByteIndex = 3;
	BufferInfoPtr->WriteByteIndex = 3;
	BufferInfoPtr->WrapValue = 0;
	BufferInfoPtr->BufferLevel = 0;
	BufferInfoPtr->TotalBytesWritten = 0;
	BufferInfoPtr->TotalBytesRead = 0;
}


void ZeroBuffer(struct BUFFER_INFO *BufferInfoPtr, int BytesToWrite)
{
	int BytesToTop;

	BufferInfoPtr->BufferLevel += BytesToWrite;
	BufferInfoPtr->TotalBytesWritten += BytesToWrite;
	BytesToTop = (int)(BufferInfoPtr->TopPtr - BufferInfoPtr->WritePtr);
	if(BytesToWrite > BytesToTop)
	{
		memset( BufferInfoPtr->WritePtr, 0, BytesToTop);
//		if(BufferInfoPtr->FileInfoPtr != NULL)
//			fwrite(BufferInfoPtr->WritePtr, 1, BytesToTop, BufferInfoPtr->FileInfoPtr->FilePtr);
		BytesToWrite -= BytesToTop;
		memset( BufferInfoPtr->BeginPtr, 0, BytesToWrite);
//		if(BufferInfoPtr->FileInfoPtr != NULL)
//			fwrite(BufferInfoPtr->BeginPtr, 1, BytesToWrite, BufferInfoPtr->FileInfoPtr->FilePtr);
		BufferInfoPtr->WritePtr = BufferInfoPtr->BeginPtr + BytesToWrite;
		BufferInfoPtr->WrapValue = BufferInfoPtr->WriteBufferSize;
	}
	else
	{
		memset( BufferInfoPtr->WritePtr, 0, BytesToWrite);
//		if(BufferInfoPtr->FileInfoPtr != NULL)
//			fwrite(BufferInfoPtr->WritePtr, 1, BytesToWrite, BufferInfoPtr->FileInfoPtr->FilePtr);
		BufferInfoPtr->WritePtr += BytesToWrite;
	}
}

uint8_t *GetBufferWritePtr(struct BUFFER_INFO *BufferInfoPtr, int BytesToWrite)
{
	int BytesToTop;

	BytesToTop = (int)(BufferInfoPtr->TopPtr - BufferInfoPtr->WritePtr);
	if(BytesToWrite > BytesToTop)
	{
		if(BytesToTop == 0)
		{
			BufferInfoPtr->WritePtr = BufferInfoPtr->BeginPtr;
			BufferInfoPtr->WrapValue = BufferInfoPtr->WriteBufferSize;
		}
		else
			return NULL;
	}
	return BufferInfoPtr->WritePtr;
}

void WriteToBuffer(struct BUFFER_INFO *BufferInfoPtr, unsigned char *SrcPtr, int BytesToWrite)
{
	int BytesToTop, LeftOverBytes;

//	if(BufferInfoPtr->FileInfoPtr != NULL)
//		fwrite(SrcPtr, 1, BytesToWrite, BufferInfoPtr->FileInfoPtr->FilePtr);
	BufferInfoPtr->TotalBytesWritten += BytesToWrite;
	BytesToTop = (int)(BufferInfoPtr->TopPtr - BufferInfoPtr->WritePtr);
	if(BytesToWrite >= BytesToTop)
	{
		if( BytesToTop > 0 )
		{
			memcpy( BufferInfoPtr->WritePtr, SrcPtr, BytesToTop);
			SrcPtr += BytesToTop;
		}
		LeftOverBytes = BytesToWrite - BytesToTop;
		BufferInfoPtr->WritePtr = BufferInfoPtr->BeginPtr;
		if( LeftOverBytes > 0 )
		{
			memcpy( BufferInfoPtr->WritePtr, SrcPtr, LeftOverBytes);
			BufferInfoPtr->WritePtr += LeftOverBytes;
		}
		BufferInfoPtr->WrapValue = BufferInfoPtr->WriteBufferSize;
	}
	else
	{
		memcpy( BufferInfoPtr->WritePtr, SrcPtr, BytesToWrite);
		BufferInfoPtr->WritePtr += BytesToWrite;
	}
	BufferInfoPtr->BufferLevel += BytesToWrite;
}

int ReadFromBuffer( struct BUFFER_INFO *BufferInfoPtr, unsigned char *DestPtr, int BytesToRead )
{
    int BytesToTop, BytesAtBottom;
//    int BufferLevel;

//	BufferLevel = GetBufferLevel( BufferInfoPtr );
//	if(BytesToRead > BufferLevel)
//		BytesToRead = BufferLevel;
    if( BytesToRead > BufferInfoPtr->BufferLevel )
        BytesToRead = BufferInfoPtr->BufferLevel;
	BytesToTop = (int)( BufferInfoPtr->TopPtr - BufferInfoPtr->ReadPtr );
	if( BytesToRead >= BytesToTop )
	{
		if( BytesToTop > 0 )
		{
			memcpy( DestPtr, BufferInfoPtr->ReadPtr, BytesToTop );
			DestPtr += BytesToTop;
		}
		BytesAtBottom = BytesToRead - BytesToTop;
		BufferInfoPtr->ReadPtr = BufferInfoPtr->BeginPtr;
		if( BytesAtBottom > 0 )
		{
			memcpy( DestPtr, BufferInfoPtr->ReadPtr, BytesAtBottom );
			BufferInfoPtr->ReadPtr += BytesAtBottom;
		}
		BufferInfoPtr->WrapValue = 0;
	}
	else
	{
		memcpy( DestPtr, BufferInfoPtr->ReadPtr, BytesToRead );
		BufferInfoPtr->ReadPtr += BytesToRead;
	}
	BufferInfoPtr->BufferLevel -= BytesToRead;
	BufferInfoPtr->TotalBytesRead += BytesToRead;
	return BytesToRead;
}

uint8_t *GetEitherOrBufferReadPtr(struct BUFFER_INFO *BufferInfoPtr, unsigned char *DestPtr, int BytesToRead)
{
//	int BufferLevel;
	int BytesToTop, BytesAtBottom;
	unsigned char *ReadPtr;

//	BufferLevel = GetBufferLevel(BufferInfoPtr);  // BufferInfoPtr->BufferLevel; //
//	if(BytesToRead > BufferLevel)
//		BytesToRead = BufferLevel;
//	if(BytesToRead == 0)
//		return NULL;
	BytesToTop = (int)(BufferInfoPtr->TopPtr - BufferInfoPtr->ReadPtr);
	if(BytesToRead > BytesToTop)
	{
		ReadPtr = DestPtr;
		memcpy( DestPtr, BufferInfoPtr->ReadPtr, BytesToTop);
		DestPtr += BytesToTop;
		BytesAtBottom = BytesToRead - BytesToTop;
		memcpy( DestPtr, BufferInfoPtr->BeginPtr, BytesAtBottom);
//		BufferInfoPtr->ReadPtr = BufferInfoPtr->BeginPtr + BytesAtBottom;
//		BufferInfoPtr->WrapValue = 0;
	}
	else
	{
//		memcpy( DestPtr, BufferInfoPtr->ReadPtr, BytesToRead);
		ReadPtr = BufferInfoPtr->ReadPtr;
//		BufferInfoPtr->ReadPtr += BytesToRead;
	}
//	BufferInfoPtr->BufferLevel -= BytesToRead;
//	BufferInfoPtr->TotalBytesRead += BytesToRead;
	return ReadPtr;
}

int GetAvailableBufferSpace( struct BUFFER_INFO *BufferInfoPtr )
{
	int AvailSpace;

//	AvailSpace = (int)(BufferInfoPtr->TopPtr - BufferInfoPtr->WritePtr);
	AvailSpace = BufferInfoPtr->ReadPtr - BufferInfoPtr->WritePtr;
    if( AvailSpace < 0 )
    {
        AvailSpace = BufferInfoPtr->TopPtr - BufferInfoPtr->ReadPtr;
        AvailSpace += BufferInfoPtr->WritePtr - BufferInfoPtr->BeginPtr;
    }
	return AvailSpace;
}

int GetBufferLevel( struct BUFFER_INFO *BufferInfoPtr )
{
	int BufferLevel;
	unsigned char *CharPtr;

	CharPtr = BufferInfoPtr->WritePtr + BufferInfoPtr->WrapValue;
	BufferLevel = (int)(CharPtr - BufferInfoPtr->ReadPtr);
	return BufferLevel;
}

void AdvanceBufferWritePtr(struct BUFFER_INFO *BufferInfoPtr, int Bytes)
{
	int BytesOverrun;

	BufferInfoPtr->WritePtr += Bytes;
	if(BufferInfoPtr->WritePtr >= BufferInfoPtr->TopPtr)
	{
		BytesOverrun = (int)(BufferInfoPtr->WritePtr - BufferInfoPtr->TopPtr);
		BufferInfoPtr->WritePtr = BufferInfoPtr->BeginPtr + BytesOverrun;
		BufferInfoPtr->WrapValue = BufferInfoPtr->WriteBufferSize;
	}
	BufferInfoPtr->BufferLevel += Bytes;
	if( BufferInfoPtr->BufferLevel > BufferInfoPtr->WriteBufferSize )
		BufferInfoPtr->BufferLevel = BufferInfoPtr->WriteBufferSize;
	BufferInfoPtr->TotalBytesWritten += Bytes;
}

void AdvanceBufferReadPtr(struct BUFFER_INFO *BufferInfoPtr, int Bytes)
{
	int BytesOverrun;

	BufferInfoPtr->ReadPtr += Bytes;
	if(BufferInfoPtr->ReadPtr >= BufferInfoPtr->TopPtr)
	{
		BytesOverrun = (int)(BufferInfoPtr->ReadPtr - BufferInfoPtr->TopPtr);
		BufferInfoPtr->ReadPtr = BufferInfoPtr->BeginPtr + BytesOverrun;
		BufferInfoPtr->WrapValue = 0;
	}
	BufferInfoPtr->BufferLevel -= Bytes;
	if( BufferInfoPtr->BufferLevel < 0 )
		BufferInfoPtr->BufferLevel = 0;
	BufferInfoPtr->TotalBytesRead += Bytes;
}

uint8_t *GetContiguousWritePtr(struct BUFFER_INFO *BufferInfoPtr, int BytesToWrite)
{
	int BytesToTop;

	BytesToTop = (int)(BufferInfoPtr->TopPtr - BufferInfoPtr->WritePtr);
	if(BytesToWrite > BytesToTop)
	{
		BufferInfoPtr->WritePtr = BufferInfoPtr->BeginPtr;
		BufferInfoPtr->WrapValue = BufferInfoPtr->WriteBufferSize;
	}
	return BufferInfoPtr->WritePtr;
}

uint8_t *GetAvailableBufferReadPtr( struct BUFFER_INFO *BufferInfoPtr, int BytesToRead, int *BytesAvailablePtr )
{
	int BytesToTop;
	
	BytesToTop = (int)(BufferInfoPtr->TopPtr - BufferInfoPtr->ReadPtr);
	if( BytesToRead > BytesToTop )
		*BytesAvailablePtr = BytesToTop;
	else
		*BytesAvailablePtr = BytesToRead;
	return BufferInfoPtr->ReadPtr;
}

uint8_t *GetContiguousAuxReadPtr( struct BUFFER_INFO *BufferInfoPtr, uint8_t *DestPtr, int BytesToRead )
{
    int BytesToTop;

    BytesToTop = (int)( BufferInfoPtr->TopPtr - BufferInfoPtr->ReadPtr );
    if( BytesToRead > BytesToTop )
    {
        memcpy( (char *)DestPtr, (char *)BufferInfoPtr->ReadPtr, BytesToTop );
        memcpy( (char *)&DestPtr[ BytesToTop ], (char *)BufferInfoPtr->BeginPtr, BytesToRead - BytesToTop );
        return DestPtr;
    }
    else
    	return BufferInfoPtr->ReadPtr;
}

uint8_t *GetContiguousReadPtr( struct BUFFER_INFO *BufferInfoPtr, int BytesToRead )
{
	uint8_t *CharPtr;
	int BytesToTop;

	BytesToTop = (int)(BufferInfoPtr->TopPtr - BufferInfoPtr->ReadPtr);
//	if(BytesToTop > BufferInfoPtr->FloorSize)
//		return BufferInfoPtr->ReadPtr;
	if(BytesToRead > BytesToTop)
	{
/*
		if(BytesToTop > BufferInfoPtr->FloorSize)
		{
			CharPtr = FrameBufferPtr;
			memcpy(CharPtr, BufferInfoPtr->ReadPtr, BytesToTop);
			CharPtr += BytesToTop;
			memcpy(CharPtr, BufferInfoPtr->BeginPtr, BytesToRead - BytesToTop);
			CharPtr = FrameBufferPtr;
			return CharPtr;
		}
		else
		{
*/
			CharPtr = BufferInfoPtr->BeginPtr - BytesToTop;
			if(CharPtr < BufferInfoPtr->BottomPtr)
			{
                Display_printf( hDisplay, 0, 0, "Trying to read an amount of data from BUFFER_INFO that is greater than Buffer Floor Size - you are going to crash." );
			}
//            Display_printf( hDisplay, 0, 0, "TopBuf" );
			memcpy( CharPtr, BufferInfoPtr->ReadPtr, BytesToTop );
			BufferInfoPtr->ReadPtr = CharPtr;
			BufferInfoPtr->WrapValue = 0;
//		}
	}
	return BufferInfoPtr->ReadPtr;
}

// ****************************************************************************
// end of file 
// ****************************************************************************
