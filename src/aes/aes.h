/* aes.h */
/*
    Copyright (C) 2008  bg nerilex (bg@nerilex.org)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/**
 * \file     aes.h
 * \email    bg@nerilex.org
 * \author   bg nerilex
 * \date     2008-12-30
 * \license  GPLv3 or later
 *
 */
#ifndef AES_H_
#define AES_H_

#include <stdint.h>

#include <stdint.h>

typedef struct __attribute__((packed))
{
    uint8_t ks[16];
} aes_roundkey_t;

typedef struct __attribute__((packed))
{
    aes_roundkey_t key[10 + 1];
} aes128_ctx_t;

typedef struct __attribute__((packed))
{
    aes_roundkey_t key[12 + 1];
} aes192_ctx_t;

typedef struct __attribute__((packed))
{
    aes_roundkey_t key[14 + 1];
} aes256_ctx_t;

typedef struct __attribute__((packed))
{
    aes_roundkey_t key[15]; /* just to avoid the warning */
} aes_genctx_t;

typedef struct __attribute__((packed))
{
    uint8_t s[16];
} aes_cipher_state_t;

void aes128_enc(void *buffer, aes128_ctx_t *ctx);
void aes192_enc(void *buffer, aes192_ctx_t *ctx);
void aes256_enc(void *buffer, aes256_ctx_t *ctx);

/**
 * \brief initialize the keyschedule
 *
 * This function computes the keyschedule from a given key with a given length
 * and stores it in the context variable
 * \param key       pointer to the key material
 * \param keysize_b length of the key in bits (valid are 128, 192 and 256)
 * \param ctx       pointer to the context where the keyschedule should be
 *stored
 */
void aes_init(const void *key, uint16_t keysize_b, aes_genctx_t *ctx);

/**
 * \brief initialize the keyschedule for 128 bit key
 *
 * This function computes the keyschedule from a given 128 bit key
 * and stores it in the context variable
 * \param key       pointer to the key material
 * \param ctx       pointer to the context where the keyschedule should be
 *stored
 */
void aes128_init(const void *key, aes128_ctx_t *ctx);

/**
 * \brief initialize the keyschedule for 192 bit key
 *
 * This function computes the keyschedule from a given 192 bit key
 * and stores it in the context variable
 * \param key       pointer to the key material
 * \param ctx       pointer to the context where the keyschedule should be
 *stored
 */
void aes192_init(const void *key, aes192_ctx_t *ctx);

/**
 * \brief initialize the keyschedule for 256 bit key
 *
 * This function computes the keyschedule from a given 256 bit key
 * and stores it in the context variable
 * \param key       pointer to the key material
 * \param ctx       pointer to the context where the keyschedule should be
 *stored
 */
void aes256_init(const void *key, aes256_ctx_t *ctx);

extern const uint8_t aes_sbox[256];

#endif
