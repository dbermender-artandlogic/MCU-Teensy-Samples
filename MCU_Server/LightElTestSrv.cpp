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

#include "LightElTestSrv.h"

#include "Log.h"
#include "MeshTime.h"
#include "TAILocalTimeConverter.h"
#include "Timestamp.h"
#include "UARTProtocol.h"

/**
 * Light EL Test Simulation check period
 */
#define LIGHT_EL_TEST_SIMULATION_CHECK_PERIOD_MS (1000 * 60)

/**
 * Supported Light EL Test Server SubOpcodes definitions
 */
#define LIGHT_EL_TEST_SRV_SUBOPCODE_EMERGENCY_LIGHTING_MODE_GET 0x00
#define LIGHT_EL_TEST_SRV_SUBOPCODE_CHANGE_EMERGENCY_LIGHTING_MODE 0X01
#define LIGHT_EL_TEST_SRV_SUBOPCODE_CHANGE_EMERGENCY_LIGHTING_MODE_UNACK 0x02
#define LIGHT_EL_TEST_SRV_SUBOPCODE_CANCEL_REST 0x03
#define LIGHT_EL_TEST_SRV_SUBOPCODE_CANCEL_REST_UNACKNOWLEDGED 0x04
#define LIGHT_EL_TEST_SRV_SUBOPCODE_START_TEST 0x05
#define LIGHT_EL_TEST_SRV_SUBOPCODE_START_TEST_UNACKNOWLEDGED 0x06
#define LIGHT_EL_TEST_SRV_SUBOPCODE_EMERGENCY_LIGHTING_MODE_STATUS 0x07
#define LIGHT_EL_TEST_SRV_SUBOPCODE_EMERGENCY_LIGHTING_PROPERTY_GET 0x08
#define LIGHT_EL_TEST_SRV_SUBOPCODE_EMERGENCY_LIGHTING_PROPERTY_SET 0x09
#define LIGHT_EL_TEST_SRV_SUBOPCODE_EMERGENCY_LIGHTING_PROPERTY_SET_UNACK 0x0A
#define LIGHT_EL_TEST_SRV_SUBOPCODE_EMERGENCY_LIGHTING_PROPERTY_STATUS 0x0B
#define LIGHT_EL_TEST_SRV_SUBOPCODE_LAST_FUNCTION_TEST_RESULT_GET 0x0C
#define LIGHT_EL_TEST_SRV_SUBOPCODE_LAST_FUNCTION_TEST_RESULT_CLEAR 0x0D
#define LIGHT_EL_TEST_SRV_SUBOPCODE_LAST_FUNCTION_TEST_RESULT_CLEAR_UNACK 0x0E
#define LIGHT_EL_TEST_SRV_SUBOPCODE_LAST_FUNCTION_TEST_RESULT_STATUS 0x0F
#define LIGHT_EL_TEST_SRV_SUBOPCODE_LAST_DURATION_TEST_RESULT_GET 0x10
#define LIGHT_EL_TEST_SRV_SUBOPCODE_LAST_DURATION_TEST_RESULT_CLEAR 0x11
#define LIGHT_EL_TEST_SRV_SUBOPCODE_LAST_DURATION_TEST_RESULT_CLEAR_UNACK 0x12
#define LIGHT_EL_TEST_SRV_SUBOPCODE_LAST_DURATION_TEST_RESULT_STATUS 0x13

/*
 *  Change Emergency lighting mode
 */
#define CHANGE_EMERGENCY_LIGHTING_MODE_NORMAL 0x00
#define CHANGE_EMERGENCY_LIGHTING_MODE_RESET 0x03
#define CHANGE_EMERGENCY_LIGHTING_MODE_INHIBIT 0x04

/*
 *  Emergency lighting mode
 */
#define EMERGENCY_LIGHTING_MODE_STATUS_NORMAL 0x00
#define EMERGENCY_LIGHTING_MODE_STATUS_EMERGENCY 0x01
#define EMERGENCY_LIGHTING_MODE_STATUS_PROLONGED_EMERGENCY 0x02
#define EMERGENCY_LIGHTING_MODE_STATUS_REST 0x03
#define EMERGENCY_LIGHTING_MODE_STATUS_INHIBIT 0x04
#define EMERGENCY_LIGHTING_MODE_STATUS_DURATION_TEST_IN_PROGRESS 0x05
#define EMERGENCY_LIGHTING_MODE_STATUS_FUNCTION_TEST_IN_PROGRESS 0x06
#define EMERGENCY_LIGHTING_MODE_STATUS_BATTERY_DISCHARGED 0x07

/*
 *  Test Identifier
 */
#define TEST_IDENTIFIER_FUNCTIONAL_TEST 0x00
#define TEST_IDENTIFIER_DURATION_TEST 0x01

/*
 *  Test Result
 */
#define TEST_RESULT_TEST_COMPLETE 0x00
#define TEST_RESULT_TEST_COMPLETE_BATTERY_FAULT 0x02
#define TEST_RESULT_TEST_COMPLETE_BATTERY_DISCHARGED 0x04
#define TEST_RESULT_TEST_COMPLETE_CHARGIN_FAULT 0x08
#define TEST_RESULT_TEST_COMPLETE_LAMP_DRIVER_FAULT 0x10
#define TEST_RESULT_TEST_COMPLETE_OVER_TEMPERATURE_EVENT 0x20
#define TEST_RESULT_TEST_COMPLETE_OVER_VOLTAGE_EVENT 0x40

#define TEST_RESULT_TEST_NOT_COMPLETE 0x01
#define TEST_RESULT_TEST_NOT_COMPLETE_TEST_CANCELLED (0x01 | 0x02)
#define TEST_RESULT_TEST_NOT_COMPLETE_TEST_CANCELLED_EMERGENCY (0x01 | 0x04)
#define TEST_RESULT_TEST_NOT_COMPLETE_TEST_NOT_RUN_BATTERY_NIT_CHARGE (0x01 | 0x08)
#define TEST_RESULT_TEST_NOT_COMPLETE_TEST_NOT_COMPLETED_BATTERY_DISCHARGED (0x01 | 0x10)

/*
 *  Emergency Lighting State structure
 */

typedef struct
{
    uint8_t  el_lighting_mode_state;
    uint8_t  last_test_result;
    uint16_t last_test_length;
    uint64_t last_test_tai_seconds : 40;
    uint8_t  last_test_time_zone_offset;
    uint16_t last_test_tai_utc_delta;
} LightingElState_T;

/*
 *  Emergency Lighting Mode Get message handler
 *
 *  @param * p_header    Mesh Message Request1 header
 *  @param * p_payload   Pointer to mesh message payload
 *  @param len           Payload length
 */
static void EmergencyLightingModeGetHandler(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);

/*
 *  Change Emergency Lighting Mode message handler
 *
 *  @param * p_header    Mesh Message Request1 header
 *  @param * p_payload   Pointer to mesh message payload
 *  @param len           Payload length
 */
static void ChangeEmergencyLightingModeHandler(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);

/*
 *  Start Test message handler
 *
 *  @param * p_header    Mesh Message Request header
 *  @param * p_payload   Pointer to mesh message payload
 *  @param len           Payload length
 */
static void StartTestHandler(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);

/*
 *  Last Function Test Result Get message handler
 *
 *  @param * p_header    Mesh Message Request1 header
 *  @param * p_payload   Pointer to mesh message payload
 *  @param len           Payload length
 */
static void LastFunctionTestResultGetHandler(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);

/*
 *  Last Duration Test Result Get message handler
 *
 *  @param * p_header    Mesh Message Request1 header
 *  @param * p_payload   Pointer to mesh message payload
 *  @param len           Payload length
 */
static void LastDurationTestResultGetHandler(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);

/*
 *  Send Mesh Message Request1 message
 *
 *  @param * p_header    Mesh Message Request1 header
 *  @param subopcode     Subopcode
 *  @param * p_payload   Pointer to mesh message payload
 *  @param len           Payload length
 */
static void MeshMessageRequest1Send(Mesh_MeshMessageRequest1Cmd_T *p_header,
                                    uint8_t                        subopcode,
                                    uint8_t *                      p_payload,
                                    size_t                         len);
/*
 *  Light EL Test Simulate
 */
static void LightElTestSimulate(void);

LightingElState_T EmergencyLightingModeState;

void LightElTestSrv_ProcessMessage(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if ((p_payload == NULL) || (p_header == NULL) || (len < 1))
    {
        return;
    }

    uint8_t subopcode = p_payload[0];

    LOG_INFO("LightElTestSrv_ProcessMessage subopcode: 0x%02X", subopcode);

    switch (subopcode)
    {
        case LIGHT_EL_TEST_SRV_SUBOPCODE_EMERGENCY_LIGHTING_MODE_GET:
            EmergencyLightingModeGetHandler(p_header, p_payload + 1, len - 1);
            break;

        case LIGHT_EL_TEST_SRV_SUBOPCODE_CHANGE_EMERGENCY_LIGHTING_MODE:
            ChangeEmergencyLightingModeHandler(p_header, p_payload + 1, len - 1);
            break;

        case LIGHT_EL_TEST_SRV_SUBOPCODE_START_TEST:
            StartTestHandler(p_header, p_payload + 1, len - 1);
            break;

        case LIGHT_EL_TEST_SRV_SUBOPCODE_LAST_FUNCTION_TEST_RESULT_GET:
            LastFunctionTestResultGetHandler(p_header, p_payload + 1, len - 1);
            break;

        case LIGHT_EL_TEST_SRV_SUBOPCODE_LAST_DURATION_TEST_RESULT_GET:
            LastDurationTestResultGetHandler(p_header, p_payload + 1, len - 1);
            break;

        default:
            break;
    }
}

void LoopLightElTest(void)
{
    MeshTimeLastSync_T *last_time_sync            = MeshTime_GetLastSyncTime();
    static uint32_t     last_check_period_time_ms = 0;
    static uint8_t      last_test_day             = 0;

    if (last_time_sync->tai_seconds == TIME_TAI_SECONDS_TIME_UNKNOWN)
    {
        return;
    }

    if (Timestamp_GetTimeElapsed(last_check_period_time_ms, Timestamp_GetCurrent()) <
        LIGHT_EL_TEST_SIMULATION_CHECK_PERIOD_MS)
    {
        last_check_period_time_ms = Timestamp_GetCurrent() + LIGHT_EL_TEST_SIMULATION_CHECK_PERIOD_MS;
        return;
    }

    uint64_t actual_tai_ms = last_time_sync->tai_seconds * 1000 + TIME_SUBSECONDS_TO_MS(last_time_sync->subsecond) +
                             Timestamp_GetTimeElapsed(last_time_sync->local_sync_timestamp_ms, Timestamp_GetCurrent());

    int16_t time_zone_offset_minutes = TIME_ZONE_OFFSET_STATE_TO_MIN((int16_t)last_time_sync->time_zone_offset);
    int16_t leap_seconds             = TIME_TAI_UTC_DELTA_STATE_TO_SEC((int16_t)last_time_sync->tai_utc_delta);
    struct LocalTime local_time      = TAILocalTimeConverter_TAIToLocalTime(actual_tai_ms / 1000,
                                                                       time_zone_offset_minutes,
                                                                       leap_seconds);
    // Prevent test simulator execution just after startup
    if (last_test_day == 0)
    {
        last_test_day = local_time.day;
        return;
    }

    // Execute EL Test at every day just after 00:00:00
    if (local_time.day != last_test_day)
    {
        last_test_day = local_time.day;
        LightElTestSimulate();
    }
}

static void EmergencyLightingModeGetHandler(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (sizeof(LightElTestSrv_EmergencyLightingModeGet_T) != len)
    {
        return;
    }

    LightElTestSrv_EmergencyLightingModeGet_T *frame = (LightElTestSrv_EmergencyLightingModeGet_T *)p_payload;

    LOG_INFO("LightElTestSrv_EmergencyLightingModeGetHandler");

    LightElTestSrv_EmergencyLightingModeStatus_T resp = {
        .mid                     = frame->mid,
        .emergency_lighting_mode = EmergencyLightingModeState.el_lighting_mode_state,
    };

    MeshMessageRequest1Send(p_header,
                            LIGHT_EL_TEST_SRV_SUBOPCODE_EMERGENCY_LIGHTING_MODE_STATUS,
                            (uint8_t *)&resp,
                            sizeof(resp));
}

static void ChangeEmergencyLightingModeHandler(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    uint8_t mid = 0;
    if (sizeof(LightElTestSrv_ChangeEmergencyLightingMode_v1_T) == len)
    {
        LightElTestSrv_ChangeEmergencyLightingMode_v1_T *frame = (LightElTestSrv_ChangeEmergencyLightingMode_v1_T *)
            p_payload;

        mid = frame->mid;
        LOG_INFO("ChangeEmergencyLightingModeHandler v1");
    }
    else if (sizeof(LightElTestSrv_ChangeEmergencyLightingMode_v2_T) == len)
    {
        LightElTestSrv_ChangeEmergencyLightingMode_v2_T *frame = (LightElTestSrv_ChangeEmergencyLightingMode_v2_T *)
            p_payload;

        mid = frame->mid;

        LOG_INFO("ChangeEmergencyLightingModeHandler v2");
    }
    else
    {
        return;
    }

    LightElTestSrv_EmergencyLightingModeStatus_T resp = {
        .mid                     = mid,
        .emergency_lighting_mode = EmergencyLightingModeState.el_lighting_mode_state,
    };

    MeshMessageRequest1Send(p_header,
                            LIGHT_EL_TEST_SRV_SUBOPCODE_EMERGENCY_LIGHTING_MODE_STATUS,
                            (uint8_t *)&resp,
                            sizeof(resp));
}

static void StartTestHandler(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (sizeof(LightElTestSrv_StartTest_T) != len)
    {
        return;
    }

    LightElTestSrv_StartTest_T *frame = (LightElTestSrv_StartTest_T *)p_payload;

    LOG_INFO("StartTestHandler");

    LightElTestSrv_EmergencyLightingModeStatus_T resp = {
        .mid                     = frame->mid,
        .emergency_lighting_mode = EmergencyLightingModeState.el_lighting_mode_state,
    };

    LightElTestSimulate();

    MeshMessageRequest1Send(p_header,
                            LIGHT_EL_TEST_SRV_SUBOPCODE_EMERGENCY_LIGHTING_MODE_STATUS,
                            (uint8_t *)&resp,
                            sizeof(resp));
}

static void LastFunctionTestResultGetHandler(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (sizeof(LightElTestSrv_LastFunctionTestResultGet_T) != len)
    {
        return;
    }

    LightElTestSrv_LastFunctionTestResultGet_T *frame = (LightElTestSrv_LastFunctionTestResultGet_T *)p_payload;

    LOG_INFO("LastFunctionTestResultGetHandler");

    LightElTestSrv_LastFunctionTestResultStatus_T resp = {
        .mid              = frame->mid,
        .result           = EmergencyLightingModeState.last_test_result,
        .tai_seconds      = EmergencyLightingModeState.last_test_tai_seconds,
        .time_zone_offset = EmergencyLightingModeState.last_test_time_zone_offset,
        .tai_utc_delta    = EmergencyLightingModeState.last_test_tai_utc_delta,
    };

    MeshMessageRequest1Send(p_header,
                            LIGHT_EL_TEST_SRV_SUBOPCODE_LAST_FUNCTION_TEST_RESULT_STATUS,
                            (uint8_t *)&resp,
                            sizeof(resp));
}

static void LastDurationTestResultGetHandler(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (sizeof(LightElTestSrv_LastDurationTestResultGet_T) != len)
    {
        return;
    }

    LightElTestSrv_LastDurationTestResultGet_T *frame = (LightElTestSrv_LastDurationTestResultGet_T *)p_payload;

    LOG_INFO("LastDurationTestResultGetHandler");

    LightElTestSrv_LastDurationTestResultStatus_T resp = {
        .mid              = frame->mid,
        .result           = TEST_RESULT_TEST_COMPLETE,
        .test_length      = EmergencyLightingModeState.last_test_length,
        .tai_seconds      = EmergencyLightingModeState.last_test_tai_seconds,
        .time_zone_offset = EmergencyLightingModeState.last_test_time_zone_offset,
        .tai_utc_delta    = EmergencyLightingModeState.last_test_tai_utc_delta,
    };

    MeshMessageRequest1Send(p_header,
                            LIGHT_EL_TEST_SRV_SUBOPCODE_LAST_DURATION_TEST_RESULT_STATUS,
                            (uint8_t *)&resp,
                            sizeof(resp));
}

static void MeshMessageRequest1Send(Mesh_MeshMessageRequest1Cmd_T *p_header,
                                    uint8_t                        subopcode,
                                    uint8_t *                      p_payload,
                                    size_t                         len)
{
    size_t size = sizeof(p_header->instance_index) + sizeof(p_header->instance_subindex) + sizeof(subopcode) +
                  p_header->mesh_cmd_size + len;

    uint8_t buff[size];
    size_t  index = 0;

    buff[index++] = p_header->instance_index;
    buff[index++] = p_header->instance_subindex;

    if (p_header->mesh_cmd_size == 3)
    {
        buff[index++] = (uint8_t)(p_header->mesh_cmd >> 16);
        buff[index++] = (uint8_t)(p_header->mesh_cmd >> 8);
        buff[index++] = (uint8_t)(p_header->mesh_cmd);
    }
    else if (p_header->mesh_cmd_size == 2)
    {
        buff[index++] = (uint8_t)(p_header->mesh_cmd >> 8);
        buff[index++] = (uint8_t)(p_header->mesh_cmd);
    }
    else
    {
        buff[index++] = (uint8_t)(p_header->mesh_cmd);
    }

    buff[index++] = subopcode;

    memcpy(buff + index, p_payload, len);
    index += len;

    UART_SendMeshMessageRequest1(buff, index);
}

static void LightElTestSimulate(void)
{
    LOG_INFO("LightElTestSimulate");

    // Emulate different states
    EmergencyLightingModeState.el_lighting_mode_state++;
    if (EmergencyLightingModeState.el_lighting_mode_state == EMERGENCY_LIGHTING_MODE_STATUS_BATTERY_DISCHARGED)
    {
        EmergencyLightingModeState.el_lighting_mode_state = EMERGENCY_LIGHTING_MODE_STATUS_NORMAL;
    }

    // Emulate different test results
    if (EmergencyLightingModeState.last_test_result == TEST_RESULT_TEST_COMPLETE)
    {
        EmergencyLightingModeState.last_test_result = TEST_RESULT_TEST_COMPLETE_BATTERY_FAULT;
    }
    else if (EmergencyLightingModeState.last_test_result < TEST_RESULT_TEST_COMPLETE_OVER_VOLTAGE_EVENT)
    {
        EmergencyLightingModeState.last_test_result <<= 1;
    }
    else
    {
        EmergencyLightingModeState.last_test_result = TEST_RESULT_TEST_COMPLETE;
    }

    // Emulate different test length
    if (EmergencyLightingModeState.last_test_length <= 60)
    {
        EmergencyLightingModeState.last_test_length++;
    }
    else
    {
        EmergencyLightingModeState.last_test_length = 1;
    }

    // Calculate time from last sync
    MeshTimeLastSync_T *last_sync_time = MeshTime_GetLastSyncTime();

    if (last_sync_time->tai_seconds == 0)
    {
        EmergencyLightingModeState.last_test_tai_seconds      = 0;
        EmergencyLightingModeState.last_test_time_zone_offset = 0;
        EmergencyLightingModeState.last_test_tai_utc_delta    = 0;
    }
    else
    {
        EmergencyLightingModeState
            .last_test_tai_seconds = last_sync_time->tai_seconds +

                                     Timestamp_GetTimeElapsed(last_sync_time->local_sync_timestamp_ms,
                                                              Timestamp_GetCurrent()) /
                                         1000;
        EmergencyLightingModeState.last_test_time_zone_offset = last_sync_time->time_zone_offset;
        EmergencyLightingModeState.last_test_tai_utc_delta    = last_sync_time->tai_utc_delta;
    }
}
