"""
/*
 * Filename: codecs.py
 *
 * Description: Various codes used by the audio script
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

from codec import CodecBase
import logging
from struct import unpack, pack


class ADPCM(CodecBase):
    """
    Implementation of a Codec class for TI's PDM driver
    """

    __STEPSIZE_LUT = [
            7,     8,     9,    10,    11,    12,    13,    14,    16,   17,   19,   21,    23,    25,    28,    31,
           34,    37,    41,    45,    50,    55,    60,    66,    73,   80,   88,   97,   107,   118,   130,   143,
          157,   173,   190,   209,   230,   253,   279,   307,   337,  371,  408,  449,   494,   544,   598,   658,
          724,   796,   876,   963,  1060,  1166,  1282,  1411,  1552, 1707, 1878, 2066,  2272,  2499,  2749,  3024,
         3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,  7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
        15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
    ]

    __INDEXLUT = [
        -1, -1, -1, -1, 2, 4, 6, 8,
        -1, -1, -1, -1, 2, 4, 6, 8
    ]

    def __init__(self):
        """
        Constructor
        """
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("Init")

        self.__SI_Dec = 0
        self.__PV_Dec = 0

    def decode(self, buffer):
        """
        Function to decode the buffer

        :param buffer: coded buffer
        :return: The decoded data
        """
        decoded = bytearray()

        for b in buffer:
            b, = unpack('B', b)
            self.logger.debug("decode: 0x%02X", b)
            sample1 = self.__tic1_DecodeSingle(b & 0x0F)
            sample2 = self.__tic1_DecodeSingle((b >> 4) & 0xF)
            decoded += pack('@hh', sample1, sample2)

        return decoded

    def __tic1_DecodeSingle(self, nibble):

        self.logger.debug("__tic1_DecodeSingle: 0x%X", nibble)

        step = ADPCM.__STEPSIZE_LUT[self.__SI_Dec]
        cum_diff = step >> 3

        self.__SI_Dec += ADPCM.__INDEXLUT[nibble]

        if self.__SI_Dec < 0:
            self.__SI_Dec = 0
        if self.__SI_Dec > 88:
            self.__SI_Dec = 88

        if nibble & 4:
            cum_diff += step
        if nibble & 2:
            cum_diff += step >> 1
        if nibble & 1:
            cum_diff += step >> 2

        if nibble & 8:
            if self.__PV_Dec < (-32767+cum_diff):
                self.__PV_Dec = -32767
            else:
                self.__PV_Dec -= cum_diff
        else:
            if self.__PV_Dec > (0x7fff-cum_diff):
                self.__PV_Dec = 0x7fff
            else:
                self.__PV_Dec += cum_diff

        return self.__PV_Dec


if __name__ == '__main__':
    logger = logging.getLogger("ADPCM")
    logger.setLevel(logging.CRITICAL)
    logger.setLevel(logging.ERROR)
    logger.setLevel(logging.WARNING)
    logger.setLevel(logging.INFO)
    logger.setLevel(logging.DEBUG)

    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    ch = logging.StreamHandler()
    ch.setFormatter(formatter)
    logger.addHandler(ch)

    print ('Subclass:', issubclass(ADPCM, CodecBase))
    print ('Instance:', isinstance(ADPCM(), CodecBase))

    test = ADPCM()