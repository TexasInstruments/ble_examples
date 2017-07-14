"""
/*
 * Filename: hidlegacy.py
 *
 * Description: Voice over HID over GATT with the legacy HID service
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
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
"""

from transport import TransportBase
import logging
import pywinusb.hid as hid
from struct import pack
from time import sleep
import time


class HIDTransport(object):

    """
    This set must be overridden by any derived classes!!
    This set should contain a list of all report_ids for which required for the voice HID reports
    """
    _VENDOR_IDS = set([])

    def __init__(self, callback, vendor_id=None, product_id=None):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("Init")

        """ Designate all instance variables """
        self.product_id = None
        self.vendor_id = None
        self.hid_device = None

        """ Apply values as defined by the instance """
        self.__hid_callbackFxn = callback
        self.setHIDFilter(vendor_id, product_id)

    def setHIDFilter(self, vendor_id, product_id):
        self.logger.info("Setting Vendor ID: 0x%04X" % vendor_id)
        self.vendor_id = vendor_id
        self.logger.info("Setting Product ID: 0x%04X" % product_id)
        self.product_id = product_id
        return

    def __findSupportedDevices(self, devices):
        targetdevice = None

        for device in devices:
            #print(device)
            device.open()

            """ Find all report IDs in this device """
            report_ids = list()
            input_reports = device.find_input_reports()
            for report in input_reports:
                report_ids.append(report.report_id)

            #print(report_ids)
            """ If we have all the report IDs required, then we found the device we need to talk with """
            if self._VENDOR_IDS.issubset(report_ids):
                targetdevice = device
                device.close()
                break

            device.close()

        self.logger.debug("Found these devices: ")
        self.logger.debug(targetdevice)
        self.hid_device = targetdevice
        return

    def open(self):
        devices = hid.HidDeviceFilter(vendor_id=self.vendor_id,
                                      product_it=self.product_id).get_devices()

        self.__findSupportedDevices(devices)
        if self.hid_device is not None:
            self.hid_device.open()
            self.hid_device.set_raw_data_handler(self.__hid_callbackFxn)
            self.logger.info("Opened")
            return True
        else:
            self.logger.info("Could not open!")
            return False

    def close(self):
        if self.hid_device is not None:
            self.logger.info("Closed")
            self.hid_device.close()

    def isConnected(self):
        if self.hid_device is not None:
            return self.hid_device.is_plugged()
        return False


class HIDLegacy(HIDTransport, TransportBase):

    _VENDOR_IDS = set([3])

    _START_REPORT = [4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    _STOP_REPORT  = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    _SEQUENCE_ORDER = (  1,   9,  17,  25,  33,  41,  49,  57,  65,  73,
                        81,  89,  97, 105, 113, 121, 129, 137, 145, 153,
                       161, 169, 177, 185, 193, 201, 209, 217, 225, 233,
                       241, 249)

    class Event(object):
        Start = "Start"
        Stop = "Stop"
        Data = "Data"

    def __init__(self, callbackFunction, vendor_id=None, product_id=None):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("Init")

        HIDTransport.__init__(self,
                              callback=self._callback,
                              vendor_id=vendor_id,
                              product_id=product_id)

        self._callbackFxn = callbackFunction
        self._audioData = bytearray()
        self._frameCnt = 0
        self._checkData = False
        self._state = HIDLegacy.Event.Stop
        self._sequenceIndex = 1
        return

    def _callback(self, data):
        #self.logger.debug("HID Callback: Data: %s" % data)

        if data[1:21] == HIDLegacy._START_REPORT:
            self.logger.info("Start")

            if self._checkData:
                self.__sequenceIndex = 1
                if self._state is not HIDLegacy.Event.Stop:
                    self.logger.error("Received \"Start\" while in \"%s\" state!" % self._state)

            self._state = HIDLegacy.Event.Start

            self._frameCnt = 0
            self._audioData = b''

        elif data[1:21] == HIDLegacy._STOP_REPORT:
            self.logger.info("Stop")

            if self._checkData:
                if self._state is not HIDLegacy.Event.Data:
                    self.logger.error("Received \"Data\" while in \"%s\" state!" % self._state)
                if self._frameCnt != 0:
                    self.logger.error("Received \"Stop\" without having all of the audio frames!")

            self._state = HIDLegacy.Event.Stop

            """ Perform supplied application function callback"""
            if self._callbackFxn is not None:
                self._callbackFxn(self._audioData)

        else:
            self.logger.debug("Data[%d]" % self._frameCnt)

            if self._checkData:
                if self._state is HIDLegacy.Event.Start and self._frameCnt != 0:
                    self.logger.warn("Received \"Data\" where frameCnt is \"%d\"" % self._frameCnt)

            self._state = HIDLegacy.Event.Data

            if self._frameCnt == 0:
                self.logger.debug("%s" % data)

                """ Experimental check for lost voice frames; not yet validated! """
                if self._checkData:
                    if data[1] is not HIDLegacy._SEQUENCE_ORDER[self._sequenceIndex]:
                        for i, x in enumerate(HIDLegacy._SEQUENCE_ORDER):
                            if x == data[1]:
                                self.logger.debug("Found pair: %d, %d" % (i, x))
                                self.logger.debug("Expected pair: %d, %d" % (self._sequenceIndex, HIDLegacy._SEQUENCE_ORDER[self._sequenceIndex]))

                                if i > self._sequenceIndex:
                                    delta = i - self._sequenceIndex
                                else:
                                    delta = i + len(HIDLegacy._SEQUENCE_ORDER) - self._sequenceIndex

                                self.logger.warn("Dropped %d voice frame(s)" % delta)
                                self._sequenceIndex = i
                                self.logger.debug("Setting index to %d; delta found %d" % (self._sequenceIndex, delta))
                                break

                    self._sequenceIndex += 1
                    if self._sequenceIndex == len(HIDLegacy._SEQUENCE_ORDER):
                        self._sequenceIndex = 0

                it = iter(data[5:21])
                for x in it:
                    self._audioData += pack('B', x)
            else:
                it = iter(data[1:21])
                for x in it:
                    self._audioData += pack('B', x)

            self._frameCnt += 1
            if self._frameCnt == 5:
                self._frameCnt = 0


    def setDataValidation(self, boolean):
        self._checkData = boolean


class HIDVoHoGP(HIDTransport, TransportBase):

    HID_RPT_ID_VOICE_START_IN = 10
    HID_RPT_ID_VOICE_DATA_IN = 11

    _VENDOR_IDS = set([HID_RPT_ID_VOICE_START_IN, HID_RPT_ID_VOICE_DATA_IN])

    _SEQUENCE_ORDER = (  1,   9,  17,  25,  33,  41,  49,  57,  65,  73,
                        81,  89,  97, 105, 113, 121, 129, 137, 145, 153,
                       161, 169, 177, 185, 193, 201, 209, 217, 225, 233,
                       241, 249)

    _START_REPORT = [4, 0, 0, 0, 0]
    _STOP_REPORT  = [0, 0, 0, 0, 0]

    class Event(object):
        Start = "Start"
        Stop = "Stop"
        Data = "Data"

    def __init__(self, callbackFunction, vendor_id=None, product_id=None):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("Init")

        HIDTransport.__init__(self,
                              callback=self._callback,
                              vendor_id=vendor_id,
                              product_id=product_id)

        self._callbackFxn = callbackFunction
        self._audioData = bytearray()
        self._frameCnt = 0
        self._checkData = False
        self._state = HIDVoHoGP.Event.Stop
        self._sequenceIndex = 1
        self._rawFile = None
        return

    def _callback(self, data):
        #self.logger.debug("HID Callback: Data: %s" % data)

        if data[0] == HIDVoHoGP.HID_RPT_ID_VOICE_START_IN:
            if data[1:6] == HIDVoHoGP._START_REPORT:
                self.logger.info("Start")

                if self._checkData:
                    self.__sequenceIndex = 1
                    if self._state is not HIDVoHoGP.Event.Stop:
                        self.logger.error("Received \"Start\" while in \"%s\" state!" % self._state)
                    if self._rawFile is not None:
                        self._rawFile.close()
                    self._rawFile = open(time.strftime("raw_%Y-%m-%d_%H-%M-%S")+".bin", "w")
                    self._rawFile.write("ST %02X %s\n" % (data[1], data[2:6]))

                self._state = HIDLegacy.Event.Start

                self._frameCnt = 0
                self._audioData = b''

            elif data[1:6] == HIDVoHoGP._STOP_REPORT:
                self.logger.info("Stop")

                if self._checkData:
                    if self._state is not HIDVoHoGP.Event.Data:
                        self.logger.error("Received \"Data\" while in \"%s\" state!" % self._state)
                    if self._frameCnt != 0:
                        self.logger.error("Received \"Stop\" without having all of the audio frames!")
                    self._rawFile.write("SP %02X %s\n" % (data[1], data[2:6]))
                    self._rawFile.close()
                    self._rawFile = None

                self._state = HIDVoHoGP.Event.Stop

                """ Perform supplied application function callback"""
                if self._callbackFxn is not None:
                    self._callbackFxn(self._audioData)
            else:
                self.logger.error("Invalid packet: %s" % data[1:6])
                self.logger.error("Data: %s" % data)
                raise "Invalid packet"

        elif data[0] == HIDVoHoGP.HID_RPT_ID_VOICE_DATA_IN:
            #self.logger.debug("Data[%d]: %s" % (self._frameCnt, data[1:21]))

            if self._checkData:
                if self._state is HIDVoHoGP.Event.Start and self._frameCnt != 0:
                    self.logger.warn("Received \"Data\" where frameCnt is \"%d\"" % self._frameCnt)

            self._state = HIDVoHoGP.Event.Data

            """ Experimental check for lost voice frames; not yet validated! """
            if self._checkData:
                if self._frameCnt == 0:
                    if data[1] is not HIDVoHoGP._SEQUENCE_ORDER[self._sequenceIndex]:
                        for i, x in enumerate(HIDVoHoGP._SEQUENCE_ORDER):
                            if x == data[1]:
                                self.logger.debug("Found pair: %d, %d" % (i, x))
                                self.logger.debug("Expected pair: %d, %d" % (self._sequenceIndex, HIDVoHoGP._SEQUENCE_ORDER[self._sequenceIndex]))

                                if i > self._sequenceIndex:
                                    delta = i - self._sequenceIndex
                                else:
                                    delta = i + len(HIDVoHoGP._SEQUENCE_ORDER) - self._sequenceIndex

                                self.logger.warn("Dropped %d voice frame(s)" % delta)
                                self._sequenceIndex = i
                                self.logger.debug("Setting index to %d; delta found %d" % (self._sequenceIndex, delta))
                                break

                    self._sequenceIndex += 1
                    if self._sequenceIndex == len(HIDVoHoGP._SEQUENCE_ORDER):
                        self._sequenceIndex = 0

                    string = 'D0 {seqNum:3d} {array:>120}\n'.format(seqNum=data[1],
                                                                    array=', '.join('0x{0:02X}'.format(x) for x in data[5:]))
                else:
                    string = 'D{frameCnt}     {array:>120}\n'.format(frameCnt=self._frameCnt,
                                                                     array=', '.join('0x{0:02X}'.format(x) for x in data[1:]))
                self._rawFile.write(string)

            if self._frameCnt == 0:
                it = iter(data[5:21])
            else:
                it = iter(data[1:21])

            for x in it:
                self._audioData += pack('B', x)

            self._frameCnt += 1
            if self._frameCnt == 5:
                self._frameCnt = 0
        else:
            raise "HID Report ID"

    def setDataValidation(self, boolean):
        self._checkData = boolean


def set_debug_level(loggers, formatter):

    ch = logging.StreamHandler()
    ch.setFormatter(formatter)

    for log in loggers:

        logger = logging.getLogger(log)

        if "HIDVoHoGP" in log:
            logger.setLevel(logging.CRITICAL)
            logger.setLevel(logging.ERROR)
            logger.setLevel(logging.WARNING)
            logger.setLevel(logging.INFO)
            logger.setLevel(logging.DEBUG)

        if "HIDLegacy" in log:
            logger.setLevel(logging.CRITICAL)
            logger.setLevel(logging.ERROR)
            logger.setLevel(logging.WARNING)
            logger.setLevel(logging.INFO)
            logger.setLevel(logging.DEBUG)

        logger.addHandler(ch)


def main():
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    set_debug_level(["HIDLegacy", "HIDVoHoGP"], formatter)

    appLogger = logging.getLogger("Test App")
    appLogger.setLevel(logging.INFO)
    ch = logging.StreamHandler()
    ch.setFormatter(formatter)
    appLogger.addHandler(ch)

    def testfunction(data):
        appLogger.info("Test Callback with %s bytes" % len(data))

    #hid = HIDLegacy(testfunction, vendor_id=0x000d, product_id=0x0000)
    hid = HIDVoHoGP(testfunction, vendor_id=0x000d, product_id=0x0000)
    hid.setDataValidation(True)
    hid.open()

    while hid.isConnected():
        sleep(1)

    hid.close()


if __name__ == '__main__':
    main()
