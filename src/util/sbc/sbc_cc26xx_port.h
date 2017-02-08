/*
 *
 *  Bluetooth low-complexity, subband codec (SBC) library
 *
 *  Copyright (C) 2016-2017  Texas Instruments
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
#ifndef SBC_CC26XX_PORT_H
#define SBC_CC26XX_PORT_H

#include <stdint.h>

#ifndef SBC_CC26XX
#define SBC_CC26XX
#endif //SBC_CC26XX
#define ONLY_MSBC
//#define MSBC_ENCODE
//#define xMSBC_DECODE
#ifdef AUDIO_TRANSMITTER
  #ifndef MSBC_ENCODE
    #define MSBC_ENCODE
  #endif //MSBC_ENCODE
  #ifdef MSBC_DECODE
    #undef MSBC_DECODE
  #endif //MSBC_DECODE
#endif //AUDIO_TRANSMITTER

#ifdef AUDIO_RECEIVER
  #ifndef MSBC_DECODE
    #define MSBC_DECODE
  #endif //MSBC_DECODE
  #ifdef MSBC_ENCODE
    #undef MSBC_ENCODE
  #endif //MSBC_ENCODE
#endif //AUDIO_RECEIVER


#define EIO       (-1)
#define ENOMEM    (-1)
#define EINVAL    (-1)
#define ENOSPC    (-1)

typedef int32_t ssize_t;

#endif //SBC_CC26XX_PORT_H
