"""
/*
 * Filename: voice_hogp.py
 *
 * Description: Voice of HID over GATT sample script
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


from transport.hidtransport import HIDLegacy, HIDVoHoGP
from codec.adpcm import ADPCM
import logging
import argparse
from time import sleep
import time
import wave


def set_debug_level(loggers, formatter):

    ch = logging.StreamHandler()
    ch.setFormatter(formatter)

    for log in loggers:

        logger = logging.getLogger(log)

        if "ADPCM" in log:
            logger.setLevel(logging.CRITICAL)
            logger.setLevel(logging.ERROR)
            #logger.setLevel(logging.WARNING)
            #logger.setLevel(logging.INFO)
            #logger.setLevel(logging.DEBUG)

        if "HIDLegacy" in log:
            logger.setLevel(logging.CRITICAL)
            logger.setLevel(logging.ERROR)
            #logger.setLevel(logging.WARNING)
            logger.setLevel(logging.INFO)
            #logger.setLevel(logging.DEBUG)

        if "HIDVoHoGP" in log:
            logger.setLevel(logging.CRITICAL)
            logger.setLevel(logging.ERROR)
            logger.setLevel(logging.WARNING)
            logger.setLevel(logging.INFO)
            #logger.setLevel(logging.DEBUG)


        logger.addHandler(ch)


def main():

    """ Add argument parsing to the application """
    parser = argparse.ArgumentParser(description="Voice test script")
    parser.add_argument('--verbose', '-v', action='store_true', help="enable verbose output")
    parser.add_argument('--transport_validation', action='store_true', help="enable transport level data validation; used for testing")
    parser.add_argument('--transport', '-t', choices=["legacy", "vohogp"], default="vohogp", help="transport method used to get voice data")

    args = vars(parser.parse_args())

    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    if args["verbose"]:
        set_debug_level(["ADPCM", "HIDLegacy", "HIDVoHoGP"], formatter)

    appLogger = logging.getLogger("APP")
    appLogger.setLevel(logging.INFO)

    ch = logging.StreamHandler()
    ch.setFormatter(formatter)
    appLogger.addHandler(ch)

    def testfunction(data):
        appLogger.info("Test Callback with %s bytes" % len(data))
        codec = ADPCM()
        decoded = codec.decode(data)

        filename = time.strftime("pdm_test_%Y-%m-%d_%H-%M-%S_adpcm")+".wav"
        w = wave.open(filename, "w")
        w.setnchannels(1)
        w.setframerate(16000)
        w.setsampwidth(2)
        w.writeframes(decoded)
        w.close()
        appLogger.info("Created %s" % filename)

    if args["transport"] is "vohogp":
        transport = HIDVoHoGP(testfunction, vendor_id=0x000d, product_id=0x0000)
    elif args["transport"] is "legacy":
        transport = HIDLegacy(testfunction, vendor_id=0x000d, product_id=0x0000)
    else:
        raise "Invalid transport selection"

    transport.setDataValidation(args["transport_validation"])

    transport.open()

    while transport.isConnected():
        sleep(1)

    transport.close()

if __name__ == '__main__':
    main()
