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

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef int32_t ssize_t;

struct sbc_struct {
	unsigned long flags;

	void *priv;
	void *priv_alloc_base;
};

typedef struct sbc_struct sbc_t;

int sbc_init_msbc(sbc_t *sbc, unsigned long flags);

/* Decodes ONE input block into ONE output block */
ssize_t sbc_decode(sbc_t *sbc, const void *input, size_t input_len,
			void *output, size_t output_len, size_t *written);

/* Encodes ONE input block into ONE output block */
ssize_t sbc_encode(sbc_t *sbc, const void *input, size_t input_len,
			void *output, size_t output_len, ssize_t *written);

void sbc_finish(sbc_t *sbc);

#ifdef __cplusplus
}
#endif

#endif /* __MSBC_LIBRARY_H */
