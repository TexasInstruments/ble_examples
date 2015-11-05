#!C:\Python27
#/*
# * SNP_software_handshaking.py
# *
# * Python script for demoing software handshaking capbilities from PC, 
# * sets up simple profile on SNP
# *
# * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/ 
# * 
# * 
# *  Redistribution and use in source and binary forms, with or without 
# *  modification, are permitted provided that the following conditions 
# *  are met:
# *
# *    Redistributions of source code must retain the above copyright 
# *    notice, this list of conditions and the following disclaimer.
# *
# *    Redistributions in binary form must reproduce the above copyright
# *    notice, this list of conditions and the following disclaimer in the 
# *    documentation and/or other materials provided with the   
# *    distribution.
# *
# *    Neither the name of Texas Instruments Incorporated nor the names of
# *    its contributors may be used to endorse or promote products derived
# *    from this software without specific prior written permission.
# *
# *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
# *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
# *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
# *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
# *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
# *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
# *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# *
#*/

###############################################################################################################################
#SNP Frame fromat is described below:
#SOF: Fixed 1 byte start of frame 0xFE
#Length: 2 bytes describing the size of the payload. Message payloads can range from 0-4096 bytes, NOTE: field is little endian
#CMD0: Bits 7:5 are message type, bits 4:0 are subsystem which is always 0x15 for BLE SNP
#CMD1: The message opcode 
#Payload: variable length parameters from 0-4096 bytes
#FCS: A frame check that is calculated as the successive XOR of all message fields excluding SOF, see calculateFCS function
###############################################################################################################################

#This dictionary stores the associated CMD0, and CMD1 values for each command
#Command names are taken from the SNP API Guide, and values are taken form there as well. 
snpMessageDictionary = {
	#Format:										 	       CMD0,CMD1
	#Device Subgroup Commands#############################################
	'SNP Mask Event' 										: '\x35\x02',
	'SNP Get Revision' 										: '\x35\x03',
	'SNP Encapsulated HCI' 									: '\x55\x04',
	'SNP Get Status' 										: '\x35\x06',
	'SNP Test' 												: '\x35\x10',
	#Device Subgroup Events###############################################
	'SNP Power Up'	 										: '\x55\x01',
	'SNP Mask Event Response'  								: '\x75\x02',
	'SNP Get Revision Response' 							: '\x75\x03',
	'SNP HCI Command Response' 								: '\x55\x04',
	'SNP Event Indication'  								: '\x55\x05',
	'SNP Get Status Response'  								: '\x75\x06',
	'SNP Invalid Synchronous Command Indication'  			: '\x75\x07',
	'SNP Test Response'  									: '\x75\x10',
	#GAP Subgroup Commands################################################
	'SNP Start Advertisement'  								: '\x55\x42',
	'SNP Set Advertisement Data'  							: '\x55\x43',
	'SNP Stop Advertisement' 								: '\x55\x44',
	'SNP Update Connection Parameters'  					: '\x55\x45',
	'SNP Terminate Connection'  							: '\x55\x46',
	'SNP Set GAP Parameter'  								: '\x35\x48',
	'SNP Get Gap Parameter'  								: '\x35\x49',
	#GAP Subgroup Events##################################################
	'SNP Set Advertisement Data Response'  					: '\x55\x43',
	'SNP Update Connection Parameter Response' 				: '\x55\x45',
	'SNP Set Parameter Response'  							: '\x75\x48',
	'SNP Get Parameter Response'  							: '\x75\x49',
	#GATT Subgroup Commands###############################################
	'SNP Add Service'  										: '\x35\x81', 
	'SNP Add Characteristic Value Declaration'  			: '\x35\x82', 
	'SNP Add Characteristic Descriptor Declaration'  		: '\x35\x83', 
	'SNP Register Service'  								: '\x35\x84', 
	'SNP Get Attribute Value'  								: '\x35\x85', 
	'SNP Set Attribute Value'  								: '\x35\x86', 
	'SNP Characteristic Read Confirmation' 					: '\x55\x87', 
	'SNP Characteristic Write Confirmation'  				: '\x55\x88', 
	'SNP Send Notification Indication' 						: '\x55\x89', 
	'SNP CCCD Updated Confirmation'  						: '\x55\x8B', 
	'SNP Set GATT Parameter'  								: '\x35\x8C',
	'SNP Get GATT Parameter'  								: '\x35\x8D',
	#GATT Subgroup Events#################################################
	'SNP Add Service Response'  							: '\x75\x81',
	'SNP Add Characteristic Value Declaration Response'  	: '\x75\x82',
	'SNP Add Characteristic Descriptor Declaration Response': '\x75\x83',
	'SNP Register Service Response'  						: '\x75\x84',
	'SNP Get Attribute Value Response'  					: '\x75\x85',
	'SNP Set Attribute Value Response'  					: '\x75\x86',
	'SNP Characteristic Read Indication' 					: '\x55\x87',
	'SNP Characteristic Write Indication' 					: '\x55\x88',
	'SNP Send Notification Indication Response' 			: '\x55\x89',
	'SNP CCCD Updated Indication'  							: '\x55\x8B',
	'SNP Set GATT Parameter Response'  						: '\x75\x8C',
	'SNP Get GATT Parameter Response'  						: '\x75\x8D'
}
snpErrorCodeDictionary = {
	'0x83' : 'SNP Failure',
	'0x84' : 'SNP Invalid Parameters',
	'0x85' : 'Command Already in Progress',
	'0x86' : 'Command Rejected',
	'0x87' : 'Out of Resources',
	'0x88' : 'Unknown Attribute',
	'0x89' : 'Unknown Service',
	'0x8A' : 'Already Advertising'
}
#convience function to search the dictionary values to find key associated with it
#all key value pairs are 1:1, so no risk of finding multiples
def dictLookupKey(search_value):
	for key, value in snpMessageDictionary.iteritems():
	    if value == search_value:
	        return key

#convience function that displays SNP message data to the console
def printRxSNPMessageData(msgName,cmd0,length, payload, validFCS):
	print "***************************************************** "
	print "Message Name: " + msgName
	print "Valid FCS: " + str(validFCS)
	if cmd0 == '\x35':
		msgType = "Synchronous Request"
	elif cmd0 == '\x55':
		msgType = "Asynchronous"
	else:
		msgType = "Synchronous Response"
	print "Message Type: " + msgType
	if(length > 0):
		print "Payload: " + binascii.b2a_hex(payload)
#This function will pack an SNP message for you
#It uses a a dictionary to look up the paramters associated with the selected command
def buildSNPMessage(msgString, Payload):

	SOF = struct.pack('B', 0xFE)
	payloadLen = len(Payload)
	len_LSB = struct.pack('B', payloadLen & 0x00FF)
	len_MSB = struct.pack('B', (payloadLen & 0xFF00) >> 8)
	CMD0_CMD1 = snpMessageDictionary[msgString]
	msg = SOF + len_LSB + len_MSB + CMD0_CMD1
	if(payloadLen > 0):
		msg += Payload
	
	#print msg
	FCS = struct.pack('B', calculateFCS(msg))
	msg += FCS
	return msg

#Calculates the Frame check by successively XORing all fields of the message, SOF is excluded
def calculateFCS(SNPbyteString):
	#Function takes a byte_string as a parameter 
	#calculates and returns the FCS
	
	FCS = 0
	for x in SNPbyteString:
		int_data = struct.unpack('B', x)[0]
		#Note FCS is not calculated over the SOF=0xFE
		if 0xFE == int_data:
			continue
		else:
			FCS ^= int_data
	return FCS
#Initialzes the serial port using the base SNP parameters
#Note that you must pass in a port string that corresponds to the port your SNP device is connected to
def serialInit(portString):
	ser = serial.Serial(portString, 115200, timeout=1)
	return ser
#Writes the system defined chirp of 0x55, and prints dialog
def writeChirp():
	ser.write(struct.pack('B', 0x55))
	#print("STATUS: Writing Chirp")
#Blocks script execution until a chirp is received
def waitForChirp():
	serialData = ser.read(1)
	while (struct.pack('B', 0x55) != serialData):
		serialData = ser.read(1)
	#print("STATUS: Chirp found")
#Unpacks a SNP message to determine what is found
def snpReadMsg():
	serialData = ser.read(1)
	if("" == serialData):
		return
	elif(struct.pack('B', 0xFE) == serialData):
		#if we found a start of frame, read the length bytes
		#note that length is little endian
		#print("Found SOF")
		length = ser.read(2)
		#This next line unpacks a two byte string in little endian format 
		#into a single integer in a tuple, read from first tuple element
		int_length = int(struct.unpack('<H', length)[0])
		#print("Cmd Length is: " + str(int_legnth))
		#read cmd0,cmd1
		cmd0 = ser.read(1)
		cmd1 = ser.read(1)
		CMD0_CMD1 = cmd0 + cmd1
		#find the message name in the dictionary
		msgName = dictLookupKey(CMD0_CMD1)
		#Read the payload
		if(int_length > 0):
			payload = ser.read(int_length)
		else:
			payload = ""
		#Read the FCS and verify it
		FCS = ser.read(1)
		FCS_int = struct.unpack('B', FCS)[0]
		msg = length + cmd0 + cmd1
		if(int_length > 0):
			msg = msg + payload
		recievedFCS = calculateFCS(msg)
		if(recievedFCS == FCS_int):
			validFCS = True
		else:
			validFCS = False
		#print message information
		printRxSNPMessageData(msgName,cmd0,str(int_length), payload, validFCS)
		#return msg name
		return msgName
	#Should not get here.
	else:
		return ""
#This function is a blocking wait for event
def waitForEvt(evtName):
	#Wait for chirp
	print "> Waiting for event: ", evtName
	waitForChirp()
	#Send chirp to indicate we're awake
	writeChirp()
	#Wait for event indication
	while evtName != snpReadMsg():
		pass
#Sends a synch request and waits for its response. The AP (this script) cannot send any more messages until response is rx'd
def sendSynchRequest(msgName, payload):
	#Send Chirp to wake SNP
	writeChirp()
	#wait for return chirp
	waitForChirp()
	#now clear to send Start Avertisement command
	SNP_message = buildSNPMessage(msgName, payload)
	ser.write(SNP_message)
	#wait for return chirp
	waitForChirp()
	#Send Chirp to wake SNP
	writeChirp()
	while msgName + ' Response' != snpReadMsg():
		pass
#Sends an asynch request
def sendAsynch(msgName, payload):
	#Send Chirp to wake SNP
	writeChirp()
	#wait for return chirp
	waitForChirp()
	#now clear to send Start Avertisement command
	SNP_message = buildSNPMessage(msgName, payload)
	ser.write(SNP_message)
#Determines message type and calls appropriate function
def sendSNPMessage(msgName, payload):
	CMD0_CMD1 = snpMessageDictionary[msgName]
	cmd0 = CMD0_CMD1[0]
	if cmd0 == '\x35':
		sendSynchRequest(msgName, payload)
	else:
		sendAsynch(msgName, payload)






	
#This is the main loop of the program.
#Test cases are carried out in order here. 
#This script will setup the SimpleProfile on the SNP
def testLoop():
	while True:
		waitForEvt('SNP Power Up')
		time.sleep(1)
		#Start to build up the SimpleService On the SNP
		#service has 16bit UUID of 0xFFF0, UUID is LSB first
		payload = b'\x01\xF0\xFF'
		sendSNPMessage('SNP Add Service', payload)
		#Char 1 has read and write props and permissions, it is 1B with a 16 bit UUID of 0xFFF1
		payload = b'\x03\x00\x0A\x00\x00\x01\xF1\xFF'
		sendSNPMessage('SNP Add Characteristic Value Declaration' , payload)
		#Char 2 has read props and permissions, it is 1B with a 16 bit UUID of 0xFFF2
		payload = b'\x01\x00\x02\x00\x00\x01\xF2\xFF'
		sendSNPMessage('SNP Add Characteristic Value Declaration' , payload)
		#Char 3 has write props and permissions, it is 1B with a 16 bit UUID of 0xFFF3
		payload = b'\x02\x00\x08\x00\x00\x01\xF3\xFF'
		sendSNPMessage('SNP Add Characteristic Value Declaration' , payload)
		#Char 4 has notify props and permissions, it is 1B with a 16 bit UUID of 0xFFF4
		payload = b'\x00\x00\x10\x00\x00\x01\xF4\xFF'
		sendSNPMessage('SNP Add Characteristic Value Declaration' , payload)
		#Char 4 has a CCC that controls enabling/disabling the notifications, the CCC is R/W
		payload = b'\x04\x03'
		sendSNPMessage('SNP Add Characteristic Descriptor Declaration' , payload)
		#Char 5 has read props and permissions, it is 5B with a 16 bit UUID of 0xFFF5
		payload = b'\x01\x00\x02\x00\x00\x05\xF5\xFF'
		sendSNPMessage('SNP Add Characteristic Value Declaration' , payload)
		#Register the Simple Profile service with the GATT server
		payload = ""
		sendSNPMessage('SNP Register Service' , payload)
		snpReadMsg()
		#Build payload for starting advertisement
		payload = b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02'
		sendSNPMessage('SNP Start Advertisement', payload)
		waitForEvt('SNP Event Indication')
		#Wait for connection
		waitForEvt('SNP Event Indication')
		waitForEvt('SNP Event Indication')
		waitForEvt('SNP Event Indication')
		waitForEvt('SNP Event Indication')
		break





#This script uses these 3 modules. 
#serial must be imported to send data over the serial port
#struct is useful for packing data to/from C structs
#time is used for sleeping the script
import serial
import struct
import time
import binascii

#open the serial port
ser = serialInit('COM4')
#Note that the script will run the testloop procedure until the script exists or CTRL+C is pressed 
print("Beginning SNP test, Ctrl+C to quit\n")
try:
	testLoop()
except KeyboardInterrupt:
	ser.close()
	exit()
