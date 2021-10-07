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

#ifndef MESH_TIME_H_
#define MESH_TIME_H_

#include <stddef.h>
#include <stdint.h>

#include "Config.h"
#include "RTC.h"

typedef struct __attribute__((packed))
{
    uint8_t         instance_index;
    struct TimeDate date;
} TimeSourceSetReq_T;

typedef struct __attribute__((packed))
{
    uint8_t instance_index;
} TimeSourceGetReq_T;

typedef struct __attribute__((packed))
{
    uint8_t         instance_index;
    struct TimeDate date;
} TimeSourceGetResp_T;

typedef struct __attribute__((packed))
{
    uint8_t instance_index;
} TimeSourceSetResp_T;

typedef struct __attribute__((packed))
{
    uint8_t instance_index;
} TimeGetReq_T;

typedef struct __attribute__((packed))
{
    uint8_t  instance_index;
    uint64_t tai_seconds : 40;
    uint8_t  subsecond;
    uint16_t tai_utc_delta;
    uint8_t  time_zone_offset;
} TimeGetResp_T;

typedef struct
{
    uint32_t local_sync_timestamp_ms;
    uint64_t tai_seconds : 40;
    uint8_t  subsecond;
    uint16_t tai_utc_delta;
    uint8_t  time_zone_offset;
} MeshTimeLastSync_T;

static_assert(sizeof(TimeSourceSetReq_T) == 10, "Wrong size of the struct TimeSourceSetReq_T");
static_assert(sizeof(TimeSourceGetReq_T) == 1, "Wrong size of the struct TimeSourceGetReq_T");
static_assert(sizeof(TimeSourceGetResp_T) == 10, "Wrong size of the struct TimeSourceGetResp_T");
static_assert(sizeof(TimeSourceSetResp_T) == 1, "Wrong size of the struct TimeSourceSetResp_T");
static_assert(sizeof(TimeGetReq_T) == 1, "Wrong size of the struct TimeGetReq_T");
static_assert(sizeof(TimeGetResp_T) == 10, "Wrong size of the struct TimeGetResp_T");

void MeshTime_ProcessTimeSourceSetRequest(uint8_t *p_payload, uint8_t len);

void MeshTime_ProcessTimeSourceGetRequest(uint8_t *p_payload, uint8_t len);

void MeshTime_ProcessTimeGetResponse(uint8_t *p_payload, uint8_t len);

MeshTimeLastSync_T *MeshTime_GetLastSyncTime(void);

void LoopMeshTimeSync(void);

#endif    // MESH_TIME_H_
