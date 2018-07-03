/* aes_enc.c */
/*
    Copyright (C) 2006-2010  bg nerilex (bg@nerilex.org)

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
 * \file     aes_enc.c
 * \email    bg@nerilex.org
 * \author   bg nerilex
 * \date     2008-12-30
 * \license  GPLv3 or later
 *
 */

#include <stdint.h>
#include <string.h>
#include "aes.h"

static uint32_t T0[256], T1[256], T2[256], T3[256];

/*
 *   Precompute the T-Tables in this function and
 *   use the global arrays T0, ..., T3 to store the tables.
 */
static void precompute_tables()
{
    static int counter = 0;
    uint32_t tmp, a, b, c;
    const uint32_t poly = 0x11b;
    if (counter > 0) {
        return;
    }
    do {
        /* a =  01 * sbox[counter] */
        a = aes_sbox[counter];
        /* b =  02 * sbox[counter] */
        b = (a << 1) ^ (poly & (0 - (a >> 7))); /* reduction step */
        /* c =  03 * sbox[counter] */
        c = b  ^ a;
        tmp = (((((b << 8) | a) << 8) | a) << 8) | c;
        T0[counter] = tmp;
        tmp = (tmp >> 8) | (tmp << 24);
        T1[counter] = tmp;
        tmp = (tmp >> 8) | (tmp << 24);
        T2[counter] = tmp;
        tmp = (tmp >> 8) | (tmp << 24);
        T3[counter] = tmp;
    } while(++counter < 256);
}

static void aes_encrypt_core(const aes_genctx_t *ks, uint8_t plaintext[16], uint8_t ciphertext[16], unsigned rounds)
{
    int_fast8_t round, i, j;
    uint8_t tmp[16], tmp2[4];
    uint32_t a[4];
    /* indices to be used wit the T-Table */
    const uint8_t idx_c[4][4] = {
            {  0,  5, 10, 15},
            {  4,  9, 14,  3},
            {  8, 13,  2,  7},
            { 12,  1,  6, 11}
    };

    precompute_tables();
    /* KeyWhitening & copy to local memory */
    for (i = 0; i < 16; ++i) {
        tmp[i] = plaintext[i] ^ ks->key[0].ks[i];
    }

    /* main rounds */
    for (round = 1; round < rounds; ++round) {
        for(j = 0; j < 4; ++j) {
            a[j] = T0[tmp[idx_c[j][0]]] ^ T1[tmp[idx_c[j][1]]] ^ T2[tmp[idx_c[j][2]]] ^ T3[tmp[idx_c[j][3]]];
        }
        /* AddKey */
        for (i = 0; i < 16; ++i) {
            tmp[i] = ks->key[round].ks[i] ^ a[i / 4] >> (24 - (i % 4) * 8);
        }
    }

    /* SubBytes */
    for (i = 0; i < 16; ++i) {
        tmp[i] = aes_sbox[tmp[i]];
    }

    /* ShiftRows */
    for (j = 1; j < 4; ++j) {
        for (i = 0; i < 4; ++i) {
            tmp2[i] = tmp[4 * i + j];
        }
        for (i = 0; i < 4; ++i) {
            tmp[4 * i + j] = tmp2[(i + j) % 4];
        }
    }

    /* AddKey */
    for (i = 0; i < 16; ++i) {
        tmp[i] ^= ks->key[round].ks[i];
    }

    /* copy result to output array */
    for (i = 0; i < 16; ++i) {
        ciphertext[i] = tmp[i];
    }
}

void aes128_enc(void *buffer, aes128_ctx_t *ctx)
{
    aes_encrypt_core((aes_genctx_t *)ctx, buffer, buffer, 10);
}


void aes192_enc(void *buffer, aes192_ctx_t *ctx)
{
    aes_encrypt_core((aes_genctx_t *)ctx, buffer, buffer, 12);
}


void aes256_enc(void *buffer, aes256_ctx_t *ctx)
{
    aes_encrypt_core((aes_genctx_t *)ctx, buffer, buffer, 14);
}
