'''
/*
 * Filename: pySBC27.py
 *
 * Description: This tool is used to decode audio frames from the
 * CC2650ARC, the CC2650STK development kits and the CC2650 LaunchPad with 
 * CC3200AUDBOOST booster pack. These frames will saved to a wav file for 
 * playback. This script expects audio compressed in mSBC format.
 *
 * Copyright (C) 2016-2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
'''

import ctypes;
from ctypes import *
from struct import unpack, pack
import wave
from serial import Serial
from serial import SerialException
from time import time
import time
import winsound
import pyaudio
import speech_recognition as sr

runOnce = 0
compressedSize = 57
pcmFrameLen = 120
pcmFrameSize = 240
pcmSampleWidth = 2
sampleRate = 16000
written = c_uint(0)
MsbcFrame = c_ubyte * compressedSize
firstFrame = MsbcFrame(0xad, 0x00, 0x00, 0x00, 0xea, 0x97, 0x77, 0x67, 0x7f, 0x7b, 0x55, 0x60, 0x1e, 0xd5, 0x58, 0x48, 0x35, 0x56, 0x26, 0x2c, 0x51, 0xa9, 0xa5, 0x81, 0xb7, 0x0b, 0x21, 0x98, 0x08, 0xb5, 0x54, 0x7d, 0xed, 0x55, 0x42, 0x7b, 0x55, 0x6e, 0x5e, 0xd5, 0x5e, 0x17, 0xb5, 0x56, 0x05, 0xed, 0x55, 0x1f, 0x7b, 0x55, 0x50, 0x9e, 0xd5, 0x5b, 0x97, 0xb5, 0x54)
secondFrame = MsbcFrame(0xad, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x01, 0x12, 0xe1, 0xeb, 0x31, 0x60, 0x76, 0xcd, 0x61, 0xf3, 0x40, 0xe5, 0x09, 0x38, 0xc4, 0xba, 0xa3, 0xa2, 0x38, 0x7b, 0x09, 0xb8, 0x1d, 0xdf, 0x30, 0x7c, 0xd1, 0xa2, 0x42, 0x4b, 0xe5, 0xae, 0xa9, 0x15, 0x9e, 0x1e, 0xc1, 0x62, 0x07, 0x6e, 0xb5, 0x1f, 0x33, 0x56, 0x90, 0x92, 0xf9, 0x7b, 0xaa, 0x35, 0xe0)
msbcFrame = MsbcFrame()
PcmFrame = c_short * pcmFrameLen
pcmFrame = PcmFrame()

p = pyaudio.PyAudio()

stream = p.open(format=p.get_format_from_width(pcmSampleWidth),
                channels=1,
                rate=sampleRate,
                output=True)

def beep(sound):
    print "Playing sound"
    winsound.PlaySound('%s.wav' % sound, winsound.SND_FILENAME)
    
def start_wave():
    global filename
    filename = time.strftime("samples/pdm_test_%Y-%m-%d_%H-%M-%S_wave")
    print "Opening wave file for writing -- " + filename
    global w
    w = wave.open(filename + ".wav", "w")
    w.setnchannels(1)
    w.setframerate(sampleRate)
    w.setsampwidth(pcmSampleWidth)

def play_stream(pcmArray):
    stream.write(pcmArray)

def speech_to_text(filename):
    audio_file = filename + ".wav"
    r = sr.Recognizer()
    with sr.AudioFile(audio_file) as source:
        audio = r.record(source)
    try:
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        # instead of `r.recognize_google(audio)`
        print "\nGoogle Speech Recognition thinks you said \n" + r.recognize_google(audio) + "\n"
    except sr.UnknownValueError:
        print "Google Speech Recognition could not understand audio"
    except sr.RequestError as e:
        print "Could not request results from Google Speech Recognition service; {0}".format(e)

def write_to_wave(_buf, bytesWritten):
    global w    
    pcmArray = b''
##    print "Writing {0} bytes, {1} samples, to wave file".format(bytesWritten, len(_buf)))
    for pcm in _buf:
        pcmArray += pack('h', pcm)
    w.writeframes(pcmArray)
    play_stream(pcmArray)

def close_wave():
    global w
    global filename
    print "Closing wave file"
##    beep(filename)
    speech_to_text(filename)
    w.close()

sbcDll = cdll.LoadLibrary(".\SBC.dll");
print "Initializing decoder, mSBC"
sbcDll.c_init()

indata = b''
inbuffer = b''
lastByteTime = 0
startTime = 0

start_wave()

print "Waiting for stream"

waitingForEnd = 0

try:
    ser = None
    ser = Serial("COM91", 460800, timeout=0.2)
    readSoFar = 0
    readNow = 0
    msbcData = bytearray(compressedSize)
    index = 0

#    startTime = time.time()
#    endTime = startTime + 100

    while True:
        indata = ser.read(compressedSize - readSoFar)
        readNow = len(indata)
        readSoFar = readSoFar + readNow
        msbcData[index:(index + readNow)] = indata
##        indata = ser.read(388)
        if not indata and len(inbuffer): # or time.time() - startTime > 10:
            if time.time() - lastByteTime > 2 and waitingForEnd == 1:
                print "Stream ended"
                startTime = time.time()
                close_wave()
                waitingForEnd = 0
                start_wave()
        elif indata:
            limit = min(compressedSize,readSoFar)
##            print "Read {0}, total {1} HEADER:[{2}, {3}, {4}], Current HEADER:[{5}, {6}, {7}], limit {8}".format(readNow, readSoFar,
##                                                                                                               indata[0], indata[1] if (readNow > 1) else 0,
##                                                                                                               indata[2] if (readNow > 2) else 0,
##                                                                                                               msbcData[0], msbcData[1], msbcData[2],
##                                                                                                               limit))
##          Find header [0xAD seqNum 0x00], start over
            index = 0
            for byte in msbcData:
                if (byte == 173):
                    if (msbcData[index+2] == 0):
##                        print "Found header, index {0}".format(index))
                        break # Found wanted header
                if (index < (limit - 2)):
                    index = index + 1
                else:
##                    print "Did not find header, index {0}".format(index))
                    break # Did not find header
            if (index > 0):
##                print "Need to remove {0} of {1} bytes".format(index, readSoFar))
##              Received incorrect header, remove it
                readSoFar = readSoFar - index
                i = 0
                for j in range(index,limit):
                    msbcData[i] = msbcData[j]
                    i = i + 1
                index = i

            if (readSoFar == compressedSize):
                if (readSoFar == compressedSize):
                    waitingForEnd = 1
                    for i in range(0, compressedSize):
                        msbcFrame[i] = msbcData[i]
##                        print "msbcFrame[{0}]={1} -- firstFrame[{2}]={3}".format(i, msbcFrame[i], i, firstFrame[i]))

                    sbcDll.c_decode(msbcFrame, compressedSize, pcmFrame, pcmFrameSize, byref(written))
                    
##                    print "Packet #{0}, HEADER:[{1}, {2}, {3}], decoded {4} bytes".format(msbcData[1], msbcData[0], msbcData[1], msbcData[2], written.value))
                    if runOnce == 0:
##                        sbcDll.c_decode(firstFrame, compressedSize, pcmFrame, pcmFrameSize, byref(written))
                        # Dump first frame, simple removal of noise. It's only 7.5ms anyway
                        runOnce = 1
                    else:
                        write_to_wave(pcmFrame, written.value)
##                        sbcDll.c_decode(secondFrame, compressedSize, pcmFrame, pcmFrameSize, byref(written))

                    readSoFar = 0
                inbuffer += indata[4:]
                lastByteTime = time.time()
            
##            print "Reading up to {0} bytes".format(compressedSize - readSoFar))
    
except SerialException as e:
    print "Serial port error"
    print(e)

finally:
    if ser is not None: ser.close()
    stream.stop_stream()
    stream.close()

    p.terminate()

##sbcDll.c_decode(secondFrame, len(secondFrame), pcmFrame, pcmFrameSize, byref(written))
##for pcm in pcmFrame:
##    print(str(pcm))

