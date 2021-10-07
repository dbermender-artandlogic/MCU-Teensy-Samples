/*
Copyright © 2017 Silvair Sp. z o.o. All Rights Reserved.
 
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:
 
The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.
 
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "CRC.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Arduino.h"


/**< CRC configuration */
#define CRC16_POLYNOMIAL 0x8005u
#define CRC32_POLYNOMIAL 0xEDB88320u

/**< SHA256 configuration */
#define SHA256_CHUNK_SIZE 64
#define SHA256_TOTAL_LEN_LEN 8


static const uint32_t sha256_k[] = {0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
                                    0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
                                    0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
                                    0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
                                    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
                                    0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
                                    0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
                                    0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
                                    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
                                    0xc67178f2};

static const uint32_t sha256_h[] =
    {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};

typedef struct __SHA256_State_Tag
{
    const uint8_t *p;
    size_t         len;
    size_t         total_len;
    bool           single_one_delivered;
    bool           total_len_delivered;
} __SHA256_State_T;


/*
 *  Internal CRC16 calculations
 */
static uint16_t __calcCRC16(uint8_t data, uint16_t crc);

/**
 * Internal CRC16 calculations, byte reflection
 */
static uint8_t __calcCRC16_ReflectByte(uint8_t crc);

/*
 *  Internal SHA256 calculations
 */
static void __calcSHA256(uint32_t hash[32], const void *input, size_t len);

/*
 *  Internal SHA256 chunk calculations
 */
static bool __calcSHA256_Chunk(uint8_t chunk[SHA256_CHUNK_SIZE], __SHA256_State_T *state);

/*
 *  Internal SHA256 right rotation
 */
static inline uint32_t __calcSHA256_RightRotation(uint32_t value, unsigned int count);


uint16_t CalcCRC16(uint8_t *data, size_t len, uint16_t init_val)
{
    uint16_t crc = init_val;
    for (size_t i = 0; i < len; i++)
    {
        crc = __calcCRC16(data[i], crc);
    }

    return crc;
}

uint16_t CalcCRC16_Modbus(uint8_t *data, size_t len, uint16_t init_val)
{
    uint16_t crc = init_val;
    for (size_t i = 0; i < len; i++)
    {
        crc = __calcCRC16(__calcCRC16_ReflectByte(data[i]), crc);
    }

    uint16_t result = 0;
    result |= (uint16_t)__calcCRC16_ReflectByte(lowByte(crc));
    result |= (uint16_t)__calcCRC16_ReflectByte(highByte(crc)) << 8;

    return result;
}

uint32_t CalcCRC32(uint8_t *data, size_t len, uint32_t init_val)
{
    uint32_t crc = init_val;
    for (uint32_t i = 0; i < len; i++)
    {
        crc = crc ^ data[i];
        for (uint32_t j = 8; j > 0; j--)
        {
            crc = (crc >> 1) ^ (CRC32_POLYNOMIAL & ((crc & 1) ? 0xFFFFFFFF : 0));
        }
    }
    return ~crc;
}

void CalcSHA256(uint8_t *data, size_t len, uint8_t *sha256)
{
    __calcSHA256((uint32_t *)sha256, data, len);
}


static uint16_t __calcCRC16(uint8_t data, uint16_t crc)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (((crc & 0x8000) >> 8) ^ (data & 0x80))
        {
            crc = (crc << 1) ^ CRC16_POLYNOMIAL;
        }
        else
        {
            crc = (crc << 1);
        }
        data <<= 1;
    }

    return crc;
}

static uint8_t __calcCRC16_ReflectByte(uint8_t crc)
{
    uint8_t result = 0;

    result |= (uint8_t)(crc & 0x01u) << 7;
    result |= (uint8_t)(crc & 0x02u) << 5;
    result |= (uint8_t)(crc & 0x04u) << 3;
    result |= (uint8_t)(crc & 0x08u) << 1;
    result |= (uint8_t)(crc & 0x10u) >> 1;
    result |= (uint8_t)(crc & 0x20u) >> 3;
    result |= (uint8_t)(crc & 0x40u) >> 5;
    result |= (uint8_t)(crc & 0x80u) >> 7;

    return result;
}

static void __calcSHA256(uint32_t hash[32], const void *input, size_t len)
{
    size_t i, j;

    memcpy(hash, sha256_h, 32);

    uint8_t chunk[64];

    __SHA256_State_T state = {
        .p                    = (uint8_t *)input,
        .len                  = len,
        .total_len            = len,
        .single_one_delivered = 0,
        .total_len_delivered  = 0,
    };

    while (__calcSHA256_Chunk(chunk, &state))
    {
        uint32_t ah[8];

        uint32_t       w[64];
        const uint8_t *p = chunk;

        memset(w, 0x00, sizeof w);
        for (i = 0; i < 16; i++)
        {
            w[i] = (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | (uint32_t)p[3];
            p += 4;
        }

        for (i = 16; i < 64; i++)
        {
            const uint32_t s0 = __calcSHA256_RightRotation(w[i - 15], 7) ^ __calcSHA256_RightRotation(w[i - 15], 18) ^
                                (w[i - 15] >> 3);
            const uint32_t s1 = __calcSHA256_RightRotation(w[i - 2], 17) ^ __calcSHA256_RightRotation(w[i - 2], 19) ^
                                (w[i - 2] >> 10);

            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }

        for (i = 0; i < 8; i++)
        {
            ah[i] = hash[i];
        }

        for (i = 0; i < 64; i++)
        {
            const uint32_t s1 = __calcSHA256_RightRotation(ah[4], 6) ^ __calcSHA256_RightRotation(ah[4], 11) ^
                                __calcSHA256_RightRotation(ah[4], 25);

            const uint32_t ch    = (ah[4] & ah[5]) ^ (~ah[4] & ah[6]);
            const uint32_t temp1 = ah[7] + s1 + ch + sha256_k[i] + w[i];
            const uint32_t s0    = __calcSHA256_RightRotation(ah[0], 2) ^ __calcSHA256_RightRotation(ah[0], 13) ^
                                __calcSHA256_RightRotation(ah[0], 22);

            const uint32_t maj   = (ah[0] & ah[1]) ^ (ah[0] & ah[2]) ^ (ah[1] & ah[2]);
            const uint32_t temp2 = s0 + maj;

            ah[7] = ah[6];
            ah[6] = ah[5];
            ah[5] = ah[4];
            ah[4] = ah[3] + temp1;
            ah[3] = ah[2];
            ah[2] = ah[1];
            ah[1] = ah[0];
            ah[0] = temp1 + temp2;
        }

        for (i = 0; i < 8; i++)
        {
            hash[i] += ah[i];
        }
    }

    for (i = 0, j = 0; i < 8; i++)
    {
        uint32_t word          = hash[i];
        ((uint8_t *)hash)[j++] = (uint8_t)(word >> 24);
        ((uint8_t *)hash)[j++] = (uint8_t)(word >> 16);
        ((uint8_t *)hash)[j++] = (uint8_t)(word >> 8);
        ((uint8_t *)hash)[j++] = (uint8_t)word;
    }
}

static bool __calcSHA256_Chunk(uint8_t chunk[SHA256_CHUNK_SIZE], __SHA256_State_T *state)
{
    size_t space_in_chunk;

    if (state->total_len_delivered)
    {
        return false;
    }

    if (state->len >= SHA256_CHUNK_SIZE)
    {
        memcpy(chunk, state->p, SHA256_CHUNK_SIZE);
        state->p += SHA256_CHUNK_SIZE;
        state->len -= SHA256_CHUNK_SIZE;
        return true;
    }

    memcpy(chunk, state->p, state->len);
    chunk += state->len;
    space_in_chunk = SHA256_CHUNK_SIZE - state->len;
    state->p += state->len;
    state->len = 0;

    if (!state->single_one_delivered)
    {
        *chunk++ = 0x80;
        space_in_chunk -= 1;
        state->single_one_delivered = 1;
    }

    if (space_in_chunk >= SHA256_TOTAL_LEN_LEN)
    {
        const size_t left = space_in_chunk - SHA256_TOTAL_LEN_LEN;
        size_t       len  = state->total_len;
        int          i;
        memset(chunk, 0x00, left);
        chunk += left;

        chunk[7] = (uint8_t)(len << 3);
        len >>= 5;
        for (i = 6; i >= 0; i--)
        {
            chunk[i] = (uint8_t)len;
            len >>= 8;
        }
        state->total_len_delivered = 1;
    }
    else
    {
        memset(chunk, 0x00, space_in_chunk);
    }

    return true;
}

static inline uint32_t __calcSHA256_RightRotation(uint32_t value, unsigned int count)
{
    return value >> count | value << (32 - count);
}
