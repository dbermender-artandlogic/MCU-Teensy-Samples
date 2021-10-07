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

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include "Config.h"

typedef struct RingBuffer_Tag
{
    uint8_t *p_buf;
    size_t   buf_len;
    size_t   wr;
    size_t   rd;
} RingBuffer_T;

/*
 *  Initialize ring buffer.
 *
 *  @param p_ring_buffer  Pointer to ring buffer instance @def RingBuffer_T
 *  @param bufPointer     Pointer to initialized uint8_t[] buffer
 *  @param bufLen         Length of the buffer.
 *  @return               void
 */
void RingBuffer_Init(RingBuffer_T *p_ring_buffer, uint8_t *bufPointer, size_t bufLen);

/*
 *  Get information if any bytes are present in the ring buffer.
 *
 *  @param ring_buffer    Pointer to ring buffer instance @def RingBuffer_T
 *  @return               True if it's empty, false otherwise
 */
bool RingBuffer_isEmpty(RingBuffer_T *ring_buffer);

/*
 *  Write bytes from table to ring buffer
 *
 *  @param p_ring_buffer  Pointer to ring buffer instance @def RingBuffer_T
 *  @param table          pointer to table with bytes to be queued
 *  @param table_len      length of table
 *  @return               True if success, false if queue this table could cause
 *                        overflow (table is not queued) 
 */
bool RingBuffer_QueueBytes(RingBuffer_T *p_ring_buffer, uint8_t *table, uint16_t table_len);

/*
 *  Get byte from ring buffer.
 *
 *  @param p_ring_buffer    Pointer to ring buffer instance @def RingBuffer_T
 *  @return                 True if success, false if empty
 */
bool RingBuffer_DequeueByte(RingBuffer_T *p_ring_buffer, uint8_t *read_byte);

/*
 *  Set RingBuffer wr index (needed for DMA).
 *
 *  @param p_ring_buffer  Pointer to ring buffer instance @def RingBuffer_T
 *  @param value          number of queued bytes
 *  @return               True if success, false if buffer is overflowed
 */
void RingBuffer_SetWrIndex(RingBuffer_T *p_ring_buffer, uint16_t value);

/*
 *  Inform Ring Buffer how many bytes were dequeued from it without using
 *  RingBuffer_DequeueByte (needed for DMA).
 *
 *  @param p_ring_buffer  Pointer to ring buffer instance @def RingBuffer_T
 *  @param value          number of dequeued bytes
 *  @return               True if success, false otherwise
 */
void RingBuffer_IncrementRdIndex(RingBuffer_T *p_ring_buffer, uint16_t value);

/*
 *  Get information about length of queued data.
 *
 *  @param p_ring_buffer  Pointer to ring buffer instance @def RingBuffer_T
 *  @return               length of data
 */
uint16_t RingBuffer_DataLen(RingBuffer_T *p_ring_buffer);

/*
 *  Get pointer to first element of buffer, and maximum length that could be
 *  sent like normal buffer
 *
 *  @param p_ring_buffer  Pointer to ring buffer instance @def RingBuffer_T
 *  @param buf_len        output, length of continuous buffer
 * 
 *  @return               pointer to first element of buffer
 */
uint8_t *RingBuffer_GetMaxContinuousBuffer(RingBuffer_T *p_ring_buffer, uint16_t *buf_len);

#endif    //RINGBUFFER_H
