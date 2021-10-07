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

#ifndef LIGHTELTESTSRV_H_
#define LIGHTELTESTSRV_H_

#include <stdint.h>
#include <stdio.h>

#include "Mesh.h"
/**
 * Default properties
 */
#define LIGHT_EL_TEST_SRV_EMERGENCY_MANUFACTURER_MIN_LIGHTNESS_DC 0x0000
#define LIGHT_EL_TEST_SRV_EMERGENCY_MANUFACTURER_MAX_LIGHTNESS_DC 0xFFFF

/**
 * UART frame message structures
 */
typedef struct __attribute__((packed))
{
    uint8_t mid;
} LightElTestSrv_EmergencyLightingModeGet_T;

typedef struct __attribute__((packed))
{
    uint8_t mid;
    uint8_t emergency_lighting_mode;
} LightElTestSrv_ChangeEmergencyLightingMode_v1_T;

typedef struct __attribute__((packed))
{
    uint8_t mid;
    uint8_t emergency_lighting_mode;
    uint8_t mode_parameter;
} LightElTestSrv_ChangeEmergencyLightingMode_v2_T;

typedef struct __attribute__((packed))
{
    uint8_t mid;
    uint8_t emergency_lighting_mode;
} LightElTestSrv_ChangeEmergencyLightingModeUnack_v1_T;

typedef struct __attribute__((packed))
{
    uint8_t mid;
    uint8_t emergency_lighting_mode;
    uint8_t mode_parameter;
} LightElTestSrv_ChangeEmergencyLightingModeUnack_v2_T;

typedef struct __attribute__((packed))
{
    uint8_t mid;
    uint8_t test_identifier;
} LightElTestSrv_StartTest_T;

typedef struct __attribute__((packed))
{
    uint8_t mid;
    uint8_t test_identifier;
} LightElTestSrv_StartTestUnack_T;

typedef struct __attribute__((packed))
{
    uint8_t mid;
    uint8_t emergency_lighting_mode;
} LightElTestSrv_EmergencyLightingModeStatus_T;

typedef struct __attribute__((packed))
{
    uint8_t mid;
} LightElTestSrv_LastFunctionTestResultGet_T;

typedef struct __attribute__((packed))
{
    uint8_t  mid;
    uint8_t  result;
    uint64_t tai_seconds : 40;
    uint8_t  time_zone_offset;
    uint16_t tai_utc_delta;
} LightElTestSrv_LastFunctionTestResultStatus_T;

typedef struct __attribute__((packed))
{
    uint8_t mid;
} LightElTestSrv_LastDurationTestResultGet_T;

typedef struct __attribute__((packed))
{
    uint8_t  mid;
    uint8_t  result;
    uint16_t test_length;
    uint64_t tai_seconds : 40;
    uint8_t  time_zone_offset;
    uint16_t tai_utc_delta;
} LightElTestSrv_LastDurationTestResultStatus_T;

/**
 * Validate UART frame message structures size
 */
static_assert(sizeof(LightElTestSrv_EmergencyLightingModeGet_T) == 1,
              "Wrong size of the struct LightElTestSrv_EmergencyLightingModeGet_T");
static_assert(sizeof(LightElTestSrv_ChangeEmergencyLightingMode_v1_T) == 2,
              "Wrong size of the struct LightElTestSrv_ChangeEmergencyLightingMode_v1_T");
static_assert(sizeof(LightElTestSrv_ChangeEmergencyLightingMode_v2_T) == 3,
              "Wrong size of the struct LightElTestSrv_ChangeEmergencyLightingMode_v2_T");
static_assert(sizeof(LightElTestSrv_ChangeEmergencyLightingModeUnack_v1_T) == 2,
              "Wrong size of the struct LightElTestSrv_ChangeEmergencyLightingModeUnack_v1_T");
static_assert(sizeof(LightElTestSrv_ChangeEmergencyLightingModeUnack_v2_T) == 3,
              "Wrong size of the struct LightElTestSrv_ChangeEmergencyLightingModeUnack_v2_T");
static_assert(sizeof(LightElTestSrv_StartTest_T) == 2, "Wrong size of the struct LightElTestSrv_StartTest_T");
static_assert(sizeof(LightElTestSrv_StartTestUnack_T) == 2, "Wrong size of the struct LightElTestSrv_StartTestUnack_T");
static_assert(sizeof(LightElTestSrv_EmergencyLightingModeStatus_T) == 2,
              "Wrong size of the struct LightElTestSrv_EmergencyLightingModeStatus_T");
static_assert(sizeof(LightElTestSrv_LastFunctionTestResultGet_T) == 1,
              "Wrong size of the struct LightElTestSrv_LastFunctionTestResultGet_T");
static_assert(sizeof(LightElTestSrv_LastFunctionTestResultStatus_T) == 10,
              "Wrong size of the struct LightElTestSrv_LastFunctionTestResultStatus_T");
static_assert(sizeof(LightElTestSrv_LastDurationTestResultGet_T) == 1,
              "Wrong size of the struct LightElTestSrv_LastDurationTestResultGet_T");
static_assert(sizeof(LightElTestSrv_LastDurationTestResultStatus_T) == 12,
              "Wrong size of the struct LightElTestSrv_LastDurationTestResultStatus_T");

/*
 *  Light EL Test Server message process
 *
 *  @param * p_header    Mesh Message Request1 header
 *  @param * p_payload   Pointer to mesh message payload
 *  @param len           Payload length
 */
void LightElTestSrv_ProcessMessage(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);

/*
 *  Loop Light EL Test simulator
 */
void LoopLightElTest(void);

#endif    // LIGHTELTESTSRV_H_
