/*
 *
 *  Bluetooth low-complexity, subband codec (SBC) library
 *
 *  Copyright (C) 2008-2010  Nokia Corporation
 *  Copyright (C) 2012-2014  Intel Corporation
 *  Copyright (C) 2004-2010  Marcel Holtmann <marcel@holtmann.org>
 *  Copyright (C) 2004-2005  Henryk Ploetz <henryk@ploetzli.ch>
 *  Copyright (C) 2005-2006  Brad Midgley <bmidgley@xmission.com>
 *
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef __MSBC_LIBRARY_H
#define __MSBC_LIBRARY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @mainpage Modified Subband Codec (mSBC)
 *
 * mSBC is a low complexity constant bitrate codec targeting speech applications.
 * The modified subband codec is defined in Appendix A of the [Hands Free Profile
 * (HFP) specification](https://www.bluetooth.org/docman/handlers/downloaddoc.ashx?doc_id=457090).
 * The original SBC specification is defined as part of the [A2DP profile](https://www.bluetooth.org/docman/handlers/downloaddoc.ashx?doc_id=457083).
 *
 * TI has taken the open source version of the [SBC Codec](http://www.bluez.org/sbc-13/)
 * provided as a part of BlueZ and modified it to support mSBC only, thus
 * reducing the footprint and optimizing it for SimpleLink wireless
 * micro controllers.
 *
 * mSBC uses the following settings:
 *
 * | Parameter           | Value                |
 * | :-----------------: | :------------------: |
 * | Channel mode        | Mono                 |
 * | Sampling rate       | 16 kHz               |
 * | Allocation method   | Loudness             |
 * | Subbands            | 8                    |
 * | Blocklength         | 15                   |
 * | Bitpool             | 26                   |
 * | Bit depth           | 16 bit               |
 *
 *  See the @ref msbc  for information about usage.
 *
 *
 * @defgroup msbc mSBC API
 * @{
 */

/*!
 *  @brief ssize_t type definition
 *
 *  This type is a 32-bit unsigned integer on the TI SimpleLink platform
 */
typedef uint32_t size_t;

/*!
 *  @brief ssize_t type definition
 *
 *  This type is a 32-bit signed integer on the TI SimpleLink platform
 */
typedef int32_t ssize_t;

/*!
 *  @brief  Structure to hold internal memory and state required by an
 *          instance of the mSBC codec.
 */
typedef struct sbc_struct {
    /*! Holds internal flags of the encoder/decoder*/
    unsigned long flags;
    /*! Pointer to the private allocation used internal by mSBC */
    void *priv;
    /*! Pointer to the base address of private allocation */
    void *priv_alloc_base;
}sbc_t;

/*!
 *  @brief  Allocate and initialize SBC structures
 *
 *  This function will initialize the @ref sbc_t structure and allocate from
 *  heap private memory used required by the mSBC codec. These structures
 *  are set to a known state.
 *
 *  @param [in]  sbc      An instance of the @ref sbc_t structure
 *
 *  @param [in]  flags    This parameter is unused.
 *
 *
 *  @return 0 on success, @ref sbc_errorcodes otherwise.
 *
 */
int sbc_init_msbc(sbc_t *sbc, unsigned long flags);

/*!
 *  @brief  Decodes ONE input block into ONE output block
 *
 *  @pre    sbc_init_msbc() has been called.
 *
 *  @param [in]  sbc        An instance of the @ref sbc_t structure
 *
 *  @param [in]  input      Pointer to the input bit stream, will be
 *                          uncompressed
 *  @param [in]  input_len  Length of the input array in *bytes*
 *  @param [out] output     Pointer to memory where output PCM frame will be
 *                          written
 *  @param [in] output_len  Size of output PCM frame buffer in *bytes*
 *  @param [out] written    Number of bytes written into output buffer
 *
 * @return Returns the length in bytes of the packed input frame, or a negative
 * value on error. The error codes are:
 *
 *  -1   Data stream too short
 *  -2   Sync byte incorrect
 *  -3   CRC8 incorrect
 *  -4   Bitpool value out of bounds
 */
ssize_t sbc_decode(sbc_t *sbc, const void *input, size_t input_len,
            void *output, size_t output_len, size_t *written);

/*!
 *  @brief  Encodes ONE input block into ONE output block
 *
 *  @pre    sbc_init_msbc() has been called.
 *
 *  @param [in]  sbc        An instance of the @ref sbc_t structure
 *
 *  @param [in]  input      Pointer to the input PCM frame, will be compressed
 *  @param [in]  input_len  Length of the input PCM frame in *bytes*
 *  @param [out] output     Pointer to memory where output bitstream will be
 *                          written
 *  @param [in] output_len  Size of output buffer in *bytes*
 *  @param [out] written    Number of bytes written into output buffer
 *
 * @return Returns the length in bytes of the input PCM frame,
 *         @ref sbc_errorcodes otherwise.
 */
ssize_t sbc_encode(sbc_t *sbc, const void *input, size_t input_len,
            void *output, size_t output_len, size_t *written);

/*!
 *  @brief  Reset state and free memory
 *
 *  This function frees the memory allocated by @ref sbc_init_msbc and
 *  resets the sbc structure to a known state.
 *
 *  @pre    sbc_init_msbc() has been called.
 *
 *  @param [in]  sbc        An instance of the @ref sbc_t structure
 */
void sbc_finish(sbc_t *sbc);

/*! @}*/

#ifdef __cplusplus
}
#endif

#endif /* __MSBC_LIBRARY_H */
