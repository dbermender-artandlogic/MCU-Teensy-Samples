/*
Copyright © 2021 Silvair Sp. z o.o. All Rights Reserved.

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

#include "RingBuffer.h"

static bool     IsOverflow(RingBuffer_T *p_ring_buffer, uint16_t len);
static uint16_t MaxQueueBufferLen(RingBuffer_T *p_ring_buffer, uint16_t table_len);

void RingBuffer_Init(RingBuffer_T *p_ring_buffer, uint8_t *p_buf_pointer, size_t buf_len)
{
    p_ring_buffer->p_buf   = p_buf_pointer;
    p_ring_buffer->buf_len = buf_len;
    p_ring_buffer->wr      = 0;
    p_ring_buffer->rd      = 0;
}

bool RingBuffer_isEmpty(RingBuffer_T *p_ring_buffer)
{
    return p_ring_buffer->wr == p_ring_buffer->rd;
}

void RingBuffer_SetWrIndex(RingBuffer_T *p_ring_buffer, uint16_t value)
{
    p_ring_buffer->wr = (value) % p_ring_buffer->buf_len;
}

void RingBuffer_IncrementRdIndex(RingBuffer_T *p_ring_buffer, uint16_t value)
{
    p_ring_buffer->rd = (p_ring_buffer->rd + value) % p_ring_buffer->buf_len;
}

bool RingBuffer_DequeueByte(RingBuffer_T *p_ring_buffer, uint8_t *read_byte)
{
    if (RingBuffer_isEmpty(p_ring_buffer))
    {
        return false;
    }
    *read_byte = p_ring_buffer->p_buf[(p_ring_buffer->rd)++];

    if (p_ring_buffer->rd >= p_ring_buffer->buf_len)
    {
        p_ring_buffer->rd = 0;
    }

    return true;
}

bool RingBuffer_QueueBytes(RingBuffer_T *p_ring_buffer, uint8_t *table, uint16_t table_len)
{
    if (IsOverflow(p_ring_buffer, table_len))
    {
        return false;
    }

    uint16_t cpy_len = MaxQueueBufferLen(p_ring_buffer, table_len);
    memcpy(&p_ring_buffer->p_buf[p_ring_buffer->wr], table, cpy_len);

    if (cpy_len != table_len)
    {
        uint16_t rest_of_table_len = table_len - cpy_len;
        memcpy(p_ring_buffer->p_buf, &table[cpy_len], rest_of_table_len);
    }

    RingBuffer_SetWrIndex(p_ring_buffer, p_ring_buffer->wr + table_len);

    return true;
}

uint8_t *RingBuffer_GetMaxContinuousBuffer(RingBuffer_T *p_ring_buffer, uint16_t *buf_len)
{
    if (p_ring_buffer->rd + RingBuffer_DataLen(p_ring_buffer) > p_ring_buffer->buf_len)
    {
        *buf_len = p_ring_buffer->buf_len - p_ring_buffer->rd;
    }
    else
    {
        *buf_len = RingBuffer_DataLen(p_ring_buffer);
    }

    return &p_ring_buffer->p_buf[p_ring_buffer->rd];
}

uint16_t RingBuffer_DataLen(RingBuffer_T *p_ring_buffer)
{
    return (p_ring_buffer->wr - p_ring_buffer->rd) % p_ring_buffer->buf_len;
}

static bool IsOverflow(RingBuffer_T *p_ring_buffer, uint16_t len)
{
    return (len + RingBuffer_DataLen(p_ring_buffer)) > p_ring_buffer->buf_len;
}

static uint16_t MaxQueueBufferLen(RingBuffer_T *p_ring_buffer, uint16_t table_len)
{
    if (IsOverflow(p_ring_buffer, table_len))
    {
        return 0;
    }

    if (p_ring_buffer->wr + table_len > p_ring_buffer->buf_len)
    {
        return p_ring_buffer->buf_len - p_ring_buffer->wr;
    }
    else
    {
        return table_len;
    }
}
