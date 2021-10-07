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


#include "Mesh.h"

#include "Arduino.h"
#include "Log.h"
#include "Timestamp.h"
#include "UARTProtocol.h"
#include "Utils.h"

/**
 * Supported Mesh Opcodes definitions
 */
#define MESH_MESSAGE_GENERIC_ONOFF_GET 0x8201
#define MESH_MESSAGE_GENERIC_ONOFF_SET 0x8202
#define MESH_MESSAGE_GENERIC_ONOFF_SET_UNACKNOWLEDGED 0x8203
#define MESH_MESSAGE_GENERIC_ONOFF_STATUS 0x8204
#define MESH_MESSAGE_GENERIC_LEVEL_GET 0x8205
#define MESH_MESSAGE_GENERIC_LEVEL_SET 0x8206
#define MESH_MESSAGE_GENERIC_LEVEL_SET_UNACKNOWLEDGED 0x8207
#define MESH_MESSAGE_GENERIC_LEVEL_STATUS 0x8208
#define MESH_MESSAGE_GENERIC_DELTA_SET 0x8209
#define MESH_MESSAGE_GENERIC_DELTA_SET_UNACKNOWLEDGED 0x820A
#define MESH_MESSAGE_LIGHT_L_GET 0x824B
#define MESH_MESSAGE_LIGHT_L_SET 0x824C
#define MESH_MESSAGE_LIGHT_L_SET_UNACKNOWLEDGED 0x824D
#define MESH_MESSAGE_LIGHT_L_STATUS 0x824E
#define MESH_MESSAGE_LIGHT_LC_MODE_GET 0x8291
#define MESH_MESSAGE_LIGHT_LC_MODE_SET 0x8292
#define MESH_MESSAGE_LIGHT_LC_MODE_SET_UNACKNOWLEDGED 0x8293
#define MESH_MESSAGE_LIGHT_LC_MODE_STATUS 0x8294
#define MESH_MESSAGE_SENSOR_STATUS 0x0052
#define MESH_MESSAGE_LEVEL_STATUS 0x8208
#define MESH_MESSAGE_LIGHT_CTL_TEMPERATURE_STATUS 0x8266
#define MESH_MESSAGE_LEVEL_GET 0x8205
#define MESH_MESSAGE_LIGHT_EL_TEST 0xFF3601

/**
 * Used Mesh Messages len
 */
#define MESH_MESSAGE_LIGHT_L_GET_LEN 4
#define MESH_MESSAGE_GENERIC_ONOFF_SET_LEN 8
#define MESH_MESSAGE_LIGHT_L_SET_LEN 9
#define MESH_MESSAGE_GENERIC_DELTA_SET_LEN 11
#define MESH_MESSAGE_GENERIC_LEVEL_SET_LEN 9

/*
 * Mesh time conversion definitions
 */
#define MESH_NUMBER_OF_MS_IN_100_MS (100UL)
#define MESH_NUMBER_OF_MS_IN_1S (10 * MESH_NUMBER_OF_MS_IN_100_MS)
#define MESH_NUMBER_OF_MS_IN_10S (10 * MESH_NUMBER_OF_MS_IN_1S)
#define MESH_NUMBER_OF_MS_IN_10MIN (60 * MESH_NUMBER_OF_MS_IN_10S)
#define MESH_TRANSITION_TIME_STEP_RESOLUTION_MASK 0xC0
#define MESH_TRANSITION_TIME_STEP_RESOLUTION_100_MS 0x00
#define MESH_TRANSITION_TIME_STEP_RESOLUTION_1_S 0x40
#define MESH_TRANSITION_TIME_STEP_RESOLUTION_10_S 0x80
#define MESH_TRANSITION_TIME_STEP_RESOLUTION_10_MIN 0xC0
#define MESH_TRANSITION_TIME_NUMBER_OF_STEPS_MASK 0x3F
#define MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE 0x3F
#define MESH_DELAY_TIME_STEP_MS 5

/*
 * Sensor status description
 */
#define SS_FORMAT_MASK 0x01
#define SS_SHORT_LEN_MASK 0x1E
#define SS_SHORT_LEN_OFFSET 1
#define SS_SHORT_PROP_ID_LOW_MASK 0xE0
#define SS_SHORT_PROP_ID_LOW_OFFSET 5
#define SS_SHORT_PROP_ID_HIGH_OFFSET 3
#define SS_LONG_LEN_MASK 0xFE
#define SS_LONG_LEN_OFFSET 1

/**
 * Default communication properties
 */
#define MESH_REPEATS_INTERVAL_MS 20
#define MESH_MESSAGES_QUEUE_LENGTH 10

/**
 * Opcode size mask
 */
#define MESH_OPCODE_SIZE_3_OCTET_MASK 0xC0
#define MESH_OPCODE_SIZE_2_OCTET_MASK 0x80
#define MESH_OPCODE_SIZE_RFU_MASK 0x7F
#define MESH_OPCODE_SIZE_1_OCTET_MASK 0x00

typedef enum
{
    GENERIC_ON_OFF_SET_MSG,
    GENERIC_DELTA_SET_MSG,
    LIGHT_L_SET_MSG,
    GENERIC_LEVEL_SET_MSG
} MsgType_T;

typedef struct
{
    uint8_t onoff;
    uint8_t tid;
    uint8_t transition_time;
    uint8_t delay;
} GenericOnOffSetMsg_T;

typedef struct
{
    uint32_t delta_level;
    uint8_t  tid;
    uint8_t  transition_time;
    uint8_t  delay;
} GenericDeltaSetMsg_T;

typedef struct
{
    uint16_t lightness;
    uint8_t  tid;
    uint8_t  transition_time;
    uint8_t  delay;
} LightLSetMsg_T;

typedef struct
{
    int16_t value;
    uint8_t tid;
    uint8_t transition_time;
    uint8_t delay;
} GenericLevelSetMsg_T;

typedef struct
{
    MsgType_T msg_type;
    uint8_t   instance_idx;
    void *    mesh_msg;
    uint32_t  dispatch_time;
} EnqueuedMsg_T;

EnqueuedMsg_T *MeshMsgsQueue[MESH_MESSAGES_QUEUE_LENGTH];


/*
 *  Convert time from mesh format to miliseconds
 *
 *  @param time_mesh_format   Time in mesh format
 *  @param * p_time_ms        Pointer to result
 *  @return                   True if success, false otherwise
 */
static bool MeshInternal_ConvertFromMeshFormatToMsTransitionTime(uint8_t time_mesh_format, uint32_t *p_time_ms);

/*
 *  Process Light Lightness Status mesh message
 *
 *  @param * p_payload   Pointer mesh message payload
 *  @param len           Payload length
 */
static void MeshInternal_ProcessLightLStatus(uint8_t *p_payload, size_t len);

/*
 *  Process Generic Level Status mesh message
 *
 *  @param * p_payload   Pointer mesh message payload
 *  @param len           Payload length
 */
static void MeshInternal_ProcessLevelStatus(uint8_t *p_payload, size_t len);

/*
 *  Process Light CTL Status mesh message
 *
 *  @param * p_payload   Pointer mesh message payload
 *  @param len           Payload length
 */
static void MeshInternal_ProcessLightCTLTempStatus(uint8_t *p_payload, size_t len);

/*
 *  Send Generic OnOff Set message
 *
 *  @param instance_idx      Instance index
 *  @param value             Generic OnOff value
 *  @param transition_time   Transition time (mesh format)
 *  @param delay_ms          Delay in miliseconds
 *  @param is_new            Is it a new transation?
 */
static void MeshInternal_SendGenericOnOffSet(uint8_t instance_idx, GenericOnOffSetMsg_T *message);

/*
 *  Send Light Lightness Set message
 *
 *  @param instance_idx      Instance index
 *  @param value             Light Lightness value
 *  @param transition_time   Transition time (mesh format)
 *  @param delay_ms          Delay in miliseconds
 *  @param is_new            Is it a new transation?
 */
static void MeshInternal_SendLightLSet(uint8_t instance_idx, LightLSetMsg_T *message);

/*
 *  Send Generic Level Set message
 *
 *  @param instance_idx      Instance index
 *  @param value             Generic Level value
 *  @param transition_time   Transition time (mesh format)
 *  @param delay_ms          Delay in miliseconds
 *  @param is_new            Is it a new transation?
 */
static void MeshInternal_SendGenericLevelSet(uint8_t instance_idx, GenericLevelSetMsg_T *message);

/*
 *  Send Generic Delta Set message
 *
 *  @param instance_idx      Instance index
 *  @param value             Generic OnOff value
 *  @param transition_time   Transition time (mesh format)
 *  @param delay_ms          Delay in miliseconds
 *  @param is_new            Is it a new transation?
 */
static void MeshInternal_SendGenericDeltaSet(uint8_t instance_idx, GenericDeltaSetMsg_T *message);

/*
 *  Convert time from miliseconds to mesh format
 *
 *  @param time_ms   Time in miliseconds
 *  @return          Time in mesh format
 */
static uint8_t MeshInternal_ConvertFromMsToMeshFormat(uint32_t time_ms);

/*
 *  Process Sensor Status message
 *
 *  @param * p_payload    Pointer to message p_payload
 *  @param len            Payload length
 */
static void MeshInternal_ProcessSensorStatus(uint8_t *p_payload, size_t len);

/*
 *  Process Sensor Property
 *
 *  @param property_id   Property ID
 *  @param * p_payload   Pointer to message p_payload
 *  @param len           Payload length
 *  @param src_addr      Source address
 */
static void MeshInternal_ProcessSensorProperty(uint16_t property_id, uint8_t *p_payload, size_t len, uint16_t src_addr);

/*
 *  Process PIR update
 *
 *  @param * p_payload    Pointer to message p_payload
 *  @param len            Payload length
 *  @param src_addr       Source address
 */
static void MeshInternal_ProcessPresenceDetected(uint8_t *p_payload, size_t len, uint16_t src_addr);

/*
 *  Process ALS update
 *
 *  @param * p_payload    Pointer to message p_payload
 *  @param len            Payload length
 *  @param src_addr       Source address
 */
static void MeshInternal_ProcessPresentAmbientLightLevel(uint8_t *p_payload, size_t len, uint16_t src_addr);

/*
 *  Process Power Sensor update
 *
 *  @param * p_payload    Pointer to message p_payload
 *  @param len            Payload length
 *  @param src_addr       Source address
 */
static void MeshInternal_ProcessDeviceInputPower(uint8_t *p_payload, size_t len, uint16_t src_addr);

/*
 *  Process Current Sensor update
 *
 *  @param * p_payload    Pointer to message p_payload
 *  @param len            Payload length
 *  @param src_addr       Source address
 */
static void MeshInternal_ProcessPresentInputCurrent(uint8_t *p_payload, size_t len, uint16_t src_addr);

/*
 *  Process Voltage Sensor update
 *
 *  @param * p_payload    Pointer to message p_payload
 *  @param len            Payload length
 *  @param src_addr       Source address
 */
static void MeshInternal_ProcessPresentInputVoltage(uint8_t *p_payload, size_t len, uint16_t src_addr);

/*
 *  Process Energy Sensor update
 *
 *  @param * p_payload    Pointer to message p_payload
 *  @param len            Payload length
 *  @param src_addr       Source address
 */
static void MeshInternal_ProcessTotalDeviceEnergyUse(uint8_t *p_payload, size_t len, uint16_t src_addr);

/*
 *  Process Precise Energy Sensor update
 *
 *  @param * p_payload    Pointer to message p_payload
 *  @param len            Payload length
 *  @param src_addr       Source address
 */
static void MeshInternal_ProcessPreciseTotalDeviceEnergyUse(uint8_t *p_payload, size_t len, uint16_t src_addr);


bool Mesh_IsModelAvailable(uint8_t *p_payload, uint8_t len, uint16_t expected_model_id)
{
    for (size_t index = 0; index < len;)
    {
        uint16_t model_id = ((uint16_t)p_payload[index++]);
        model_id |= ((uint16_t)p_payload[index++] << 8);

        if (expected_model_id == model_id)
        {
            return true;
        }
    }
    return false;
}

void Mesh_ProcessMeshCommand(uint8_t *p_payload, size_t len)
{
    size_t   index             = 0;
    uint8_t  instance_index    = p_payload[index++];
    uint8_t  instance_subindex = p_payload[index++];
    uint16_t mesh_cmd          = ((uint16_t)p_payload[index++]);
    mesh_cmd |= ((uint16_t)p_payload[index++] << 8);

    LOG_DEBUG("Process Mesh Command [%d %d 0x%02X]", instance_index, instance_subindex, mesh_cmd);

    switch (mesh_cmd)
    {
        case MESH_MESSAGE_SENSOR_STATUS:
        {
            MeshInternal_ProcessSensorStatus(p_payload + index, len - index);
            break;
        }
        case MESH_MESSAGE_LIGHT_L_STATUS:
        {
            MeshInternal_ProcessLightLStatus(p_payload + index, len - index);
            break;
        }
        case MESH_MESSAGE_LEVEL_STATUS:
        {
            MeshInternal_ProcessLevelStatus(p_payload + index, len - index);
            break;
        }
        case MESH_MESSAGE_LIGHT_CTL_TEMPERATURE_STATUS:
        {
            MeshInternal_ProcessLightCTLTempStatus(p_payload + index, len - index);
            break;
        }
    }
}

void Mesh_SendLightLGet(uint8_t instance_idx)
{
    uint8_t buf[MESH_MESSAGE_LIGHT_L_GET_LEN];
    size_t  index = 0;

    buf[index++] = instance_idx;
    buf[index++] = 0x00;
    buf[index++] = lowByte(MESH_MESSAGE_LIGHT_L_GET);
    buf[index++] = highByte(MESH_MESSAGE_LIGHT_L_GET);

    UART_SendMeshMessageRequest(buf, sizeof(buf));
}

void Mesh_Loop(void)
{
    for (int i = 0; i < MESH_MESSAGES_QUEUE_LENGTH; i++)
    {
        if (MeshMsgsQueue[i] == NULL)
            continue;
        if (Timestamp_Compare(Timestamp_GetCurrent(), MeshMsgsQueue[i]->dispatch_time))
            continue;

        switch (MeshMsgsQueue[i]->msg_type)
        {
            case GENERIC_ON_OFF_SET_MSG:
            {
                MeshInternal_SendGenericOnOffSet(MeshMsgsQueue[i]->instance_idx,
                                                 (GenericOnOffSetMsg_T *)MeshMsgsQueue[i]->mesh_msg);
                free(MeshMsgsQueue[i]->mesh_msg);
                free(MeshMsgsQueue[i]);
                MeshMsgsQueue[i] = NULL;
                break;
            }
            case GENERIC_DELTA_SET_MSG:
            {
                MeshInternal_SendGenericDeltaSet(MeshMsgsQueue[i]->instance_idx,
                                                 (GenericDeltaSetMsg_T *)MeshMsgsQueue[i]->mesh_msg);
                free(MeshMsgsQueue[i]->mesh_msg);
                free(MeshMsgsQueue[i]);
                MeshMsgsQueue[i] = NULL;
                break;
            }
            case LIGHT_L_SET_MSG:
            {
                MeshInternal_SendLightLSet(MeshMsgsQueue[i]->instance_idx,
                                           (LightLSetMsg_T *)MeshMsgsQueue[i]->mesh_msg);
                free(MeshMsgsQueue[i]->mesh_msg);
                free(MeshMsgsQueue[i]);
                MeshMsgsQueue[i] = NULL;
                break;
            }
            case GENERIC_LEVEL_SET_MSG:
            {
                MeshInternal_SendGenericLevelSet(MeshMsgsQueue[i]->instance_idx,
                                                 (GenericLevelSetMsg_T *)MeshMsgsQueue[i]->mesh_msg);
                free(MeshMsgsQueue[i]->mesh_msg);
                free(MeshMsgsQueue[i]);
                MeshMsgsQueue[i] = NULL;
                break;
            }
        }
    }
}

void Mesh_SendGenericOnOffSet(uint8_t  instance_idx,
                              bool     value,
                              uint32_t transition_time,
                              uint32_t delay_ms,
                              uint8_t  num_of_repeats,
                              bool     is_new_transaction)
{
    static uint8_t tid = 0;

    if (is_new_transaction)
        tid++;

    for (int i = 0; i <= num_of_repeats; i++)
    {
        GenericOnOffSetMsg_T *p_msg = (GenericOnOffSetMsg_T *)calloc(1, sizeof(GenericOnOffSetMsg_T));
        p_msg->onoff                = value;
        p_msg->tid                  = tid;
        p_msg->transition_time      = MeshInternal_ConvertFromMsToMeshFormat(transition_time);
        p_msg->delay = ((num_of_repeats - i) * MESH_REPEATS_INTERVAL_MS + delay_ms) / MESH_DELAY_TIME_STEP_MS;

        EnqueuedMsg_T *p_enqueued_msg = (EnqueuedMsg_T *)calloc(1, sizeof(EnqueuedMsg_T));
        p_enqueued_msg->instance_idx  = instance_idx;
        p_enqueued_msg->mesh_msg      = p_msg;
        p_enqueued_msg->dispatch_time = Timestamp_GetCurrent() + i * MESH_REPEATS_INTERVAL_MS;
        p_enqueued_msg->msg_type      = GENERIC_ON_OFF_SET_MSG;

        for (int i = 0; i < MESH_MESSAGES_QUEUE_LENGTH; i++)
        {
            if (MeshMsgsQueue[i] == NULL)
            {
                MeshMsgsQueue[i] = p_enqueued_msg;
                break;
            }
        }
    }
}

void Mesh_SendLightLSet(uint8_t  instance_idx,
                        uint16_t value,
                        uint32_t transition_time,
                        uint32_t delay_ms,
                        uint8_t  num_of_repeats,
                        bool     is_new_transaction)
{
    static uint8_t tid = 0;

    if (is_new_transaction)
        tid++;

    for (int i = 0; i <= num_of_repeats; i++)
    {
        LightLSetMsg_T *p_msg  = (LightLSetMsg_T *)calloc(1, sizeof(LightLSetMsg_T));
        p_msg->lightness       = value;
        p_msg->tid             = tid;
        p_msg->transition_time = MeshInternal_ConvertFromMsToMeshFormat(transition_time);
        p_msg->delay           = ((num_of_repeats - i) * MESH_REPEATS_INTERVAL_MS + delay_ms) / MESH_DELAY_TIME_STEP_MS;

        EnqueuedMsg_T *p_enqueued_msg = (EnqueuedMsg_T *)calloc(1, sizeof(EnqueuedMsg_T));
        p_enqueued_msg->instance_idx  = instance_idx;
        p_enqueued_msg->mesh_msg      = p_msg;
        p_enqueued_msg->dispatch_time = Timestamp_GetCurrent() + i * MESH_REPEATS_INTERVAL_MS;
        p_enqueued_msg->msg_type      = LIGHT_L_SET_MSG;

        for (int i = 0; i < MESH_MESSAGES_QUEUE_LENGTH; i++)
        {
            if (MeshMsgsQueue[i] == NULL)
            {
                MeshMsgsQueue[i] = p_enqueued_msg;
                break;
            }
        }
    }
}

void Mesh_SendGenericLevelSet(uint8_t  instance_idx,
                              uint16_t value,
                              uint32_t transition_time,
                              uint32_t delay_ms,
                              uint8_t  num_of_repeats,
                              bool     is_new_transaction)
{
    static uint8_t tid = 0;

    if (is_new_transaction)
        tid++;

    for (int i = 0; i <= num_of_repeats; i++)
    {
        GenericLevelSetMsg_T *p_msg = (GenericLevelSetMsg_T *)calloc(1, sizeof(GenericLevelSetMsg_T));
        p_msg->value                = value;
        p_msg->tid                  = tid;
        p_msg->transition_time      = MeshInternal_ConvertFromMsToMeshFormat(transition_time);
        p_msg->delay = ((num_of_repeats - i) * MESH_REPEATS_INTERVAL_MS + delay_ms) / MESH_DELAY_TIME_STEP_MS;

        EnqueuedMsg_T *p_enqueued_msg = (EnqueuedMsg_T *)calloc(1, sizeof(EnqueuedMsg_T));
        p_enqueued_msg->instance_idx  = instance_idx;
        p_enqueued_msg->mesh_msg      = p_msg;
        p_enqueued_msg->dispatch_time = Timestamp_GetCurrent() + i * MESH_REPEATS_INTERVAL_MS;
        p_enqueued_msg->msg_type      = GENERIC_LEVEL_SET_MSG;

        for (int i = 0; i < MESH_MESSAGES_QUEUE_LENGTH; i++)
        {
            if (MeshMsgsQueue[i] == NULL)
            {
                MeshMsgsQueue[i] = p_enqueued_msg;
                break;
            }
        }
    }
}

void Mesh_SendGenericDeltaSet(uint8_t  instance_idx,
                              int32_t  value,
                              uint32_t transition_time,
                              uint32_t delay_ms,
                              uint8_t  num_of_repeats,
                              bool     is_new_transaction)
{
    static uint8_t tid = 0;

    if (is_new_transaction)
        tid++;

    for (int i = 0; i <= num_of_repeats; i++)
    {
        GenericDeltaSetMsg_T *p_msg = (GenericDeltaSetMsg_T *)calloc(1, sizeof(GenericDeltaSetMsg_T));
        p_msg->delta_level          = value;
        p_msg->tid                  = tid;
        p_msg->transition_time      = MeshInternal_ConvertFromMsToMeshFormat(transition_time);
        p_msg->delay = ((num_of_repeats - i) * MESH_REPEATS_INTERVAL_MS + delay_ms) / MESH_DELAY_TIME_STEP_MS;

        EnqueuedMsg_T *p_enqueued_msg = (EnqueuedMsg_T *)calloc(1, sizeof(EnqueuedMsg_T));
        p_enqueued_msg->instance_idx  = instance_idx;
        p_enqueued_msg->mesh_msg      = p_msg;
        p_enqueued_msg->dispatch_time = Timestamp_GetCurrent() + i * MESH_REPEATS_INTERVAL_MS;
        p_enqueued_msg->msg_type      = GENERIC_DELTA_SET_MSG;

        for (int i = 0; i < MESH_MESSAGES_QUEUE_LENGTH; i++)
        {
            if (MeshMsgsQueue[i] == NULL)
            {
                MeshMsgsQueue[i] = p_enqueued_msg;
                break;
            }
        }
    }
}


static void MeshInternal_ProcessLightLStatus(uint8_t *p_payload, size_t len)
{
    size_t   index = 0;
    uint16_t present_value;
    uint16_t target_value;
    uint32_t transition_time_ms;

    present_value = ((uint16_t)p_payload[index++]);
    present_value |= ((uint16_t)p_payload[index++] << 8);

    if (index < len)
    {
        target_value = ((uint16_t)p_payload[index++]);
        target_value |= ((uint16_t)p_payload[index++] << 8);

        bool is_valid = MeshInternal_ConvertFromMeshFormatToMsTransitionTime(p_payload[index++], &transition_time_ms);
        if (!is_valid)
        {
            LOG_INFO("Rejected Transition Time");
            return;
        }
    }
    else
    {
        target_value       = present_value;
        transition_time_ms = 0;
    }

    ProcessTargetLightness(present_value, target_value, transition_time_ms);
}

static void MeshInternal_ProcessLevelStatus(uint8_t *p_payload, size_t len)
{
    size_t   index = 0;
    int16_t  target_value;
    int16_t  present_value;
    uint32_t transition_time_ms;

    present_value = ((uint16_t)p_payload[index++]);
    present_value |= ((uint16_t)p_payload[index++] << 8);

    if (index < len)
    {
        target_value = ((uint16_t)p_payload[index++]);
        target_value |= ((uint16_t)p_payload[index++] << 8);

        bool is_valid = MeshInternal_ConvertFromMeshFormatToMsTransitionTime(p_payload[index++], &transition_time_ms);
        if (!is_valid)
        {
            LOG_INFO("Rejected Transition Time");
            return;
        }
    }
    else
    {
        target_value       = present_value;
        transition_time_ms = 0;
    }

    uint16_t present_lightness = present_value - INT16_MIN;
    uint16_t target_lightness  = target_value - INT16_MIN;

    ProcessTargetLightness(present_lightness, target_lightness, transition_time_ms);
}

static void MeshInternal_ProcessLightCTLTempStatus(uint8_t *p_payload, size_t len)
{
    size_t   index = 0;
    uint16_t present_temperature;
    uint16_t present_delta_uv;
    uint16_t target_temperature;
    uint16_t target_delta_uv;
    uint32_t transition_time_ms;

    present_temperature = ((uint16_t)p_payload[index++]);
    present_temperature |= ((uint16_t)p_payload[index++] << 8);

    present_delta_uv = ((uint16_t)p_payload[index++]);
    present_delta_uv |= ((uint16_t)p_payload[index++] << 8);

    if (index < len)
    {
        target_temperature = ((uint16_t)p_payload[index++]);
        target_temperature |= ((uint16_t)p_payload[index++] << 8);

        target_delta_uv = ((uint16_t)p_payload[index++]);
        target_delta_uv |= ((uint16_t)p_payload[index++] << 8);

        bool is_valid = MeshInternal_ConvertFromMeshFormatToMsTransitionTime(p_payload[index++], &transition_time_ms);
        if (!is_valid)
        {
            LOG_INFO("Rejected Transition Time");
            return;
        }
    }
    else
    {
        target_temperature = present_temperature;
        target_delta_uv    = present_delta_uv;
        transition_time_ms = 0;
    }

    ProcessTargetLightnessTemp(present_temperature, target_temperature, transition_time_ms);
}

static bool MeshInternal_ConvertFromMeshFormatToMsTransitionTime(uint8_t time_mesh_format, uint32_t *p_time_ms)
{
    uint32_t number_of_steps = (time_mesh_format & MESH_TRANSITION_TIME_NUMBER_OF_STEPS_MASK);
    uint32_t step_resolution = (time_mesh_format & MESH_TRANSITION_TIME_STEP_RESOLUTION_MASK);

    if (MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE == number_of_steps)
    {
        *p_time_ms = 0;
        return false;
    }
    else
    {
        switch (step_resolution)
        {
            case MESH_TRANSITION_TIME_STEP_RESOLUTION_10_MIN:
            {
                *p_time_ms = (uint32_t)MESH_NUMBER_OF_MS_IN_10MIN * number_of_steps;
                break;
            }
            case MESH_TRANSITION_TIME_STEP_RESOLUTION_10_S:
            {
                *p_time_ms = (uint32_t)MESH_NUMBER_OF_MS_IN_10S * number_of_steps;
                break;
            }
            case MESH_TRANSITION_TIME_STEP_RESOLUTION_1_S:
            {
                *p_time_ms = (uint32_t)MESH_NUMBER_OF_MS_IN_1S * number_of_steps;
                break;
            }
            case MESH_TRANSITION_TIME_STEP_RESOLUTION_100_MS:
            {
                *p_time_ms = (uint32_t)MESH_NUMBER_OF_MS_IN_100_MS * number_of_steps;
                break;
            }
            default:
            {
                *p_time_ms = (uint32_t)MESH_NUMBER_OF_MS_IN_100_MS * number_of_steps;
                break;
            }
        }
    }
    return true;
}

static void MeshInternal_SendGenericOnOffSet(uint8_t instance_idx, GenericOnOffSetMsg_T *message)
{
    uint8_t buf[MESH_MESSAGE_GENERIC_ONOFF_SET_LEN];
    size_t  index = 0;

    buf[index++] = instance_idx;
    buf[index++] = 0x00;
    buf[index++] = lowByte(MESH_MESSAGE_GENERIC_ONOFF_SET_UNACKNOWLEDGED);
    buf[index++] = highByte(MESH_MESSAGE_GENERIC_ONOFF_SET_UNACKNOWLEDGED);
    buf[index++] = message->onoff;
    buf[index++] = message->tid;
    buf[index++] = message->transition_time;
    buf[index++] = message->delay;

    UART_SendMeshMessageRequest(buf, sizeof(buf));
}

static void MeshInternal_SendLightLSet(uint8_t instance_idx, LightLSetMsg_T *message)
{
    uint8_t buf[MESH_MESSAGE_LIGHT_L_SET_LEN];
    size_t  index = 0;

    buf[index++] = instance_idx;
    buf[index++] = 0x00;
    buf[index++] = lowByte(MESH_MESSAGE_LIGHT_L_SET_UNACKNOWLEDGED);
    buf[index++] = highByte(MESH_MESSAGE_LIGHT_L_SET_UNACKNOWLEDGED);
    buf[index++] = lowByte(message->lightness);
    buf[index++] = highByte(message->lightness);
    buf[index++] = message->tid;
    buf[index++] = message->transition_time;
    buf[index++] = message->delay;

    UART_SendMeshMessageRequest(buf, sizeof(buf));
}

static void MeshInternal_SendGenericLevelSet(uint8_t instance_idx, GenericLevelSetMsg_T *message)
{
    uint8_t buf[MESH_MESSAGE_GENERIC_LEVEL_SET_LEN];
    size_t  index = 0;

    buf[index++] = instance_idx;
    buf[index++] = 0x00;
    buf[index++] = lowByte(MESH_MESSAGE_GENERIC_LEVEL_SET_UNACKNOWLEDGED);
    buf[index++] = highByte(MESH_MESSAGE_GENERIC_LEVEL_SET_UNACKNOWLEDGED);
    buf[index++] = lowByte(message->value);
    buf[index++] = highByte(message->value);
    buf[index++] = message->tid;
    buf[index++] = message->transition_time;
    buf[index++] = message->delay;

    UART_SendMeshMessageRequest(buf, sizeof(buf));
}

static void MeshInternal_SendGenericDeltaSet(uint8_t instance_idx, GenericDeltaSetMsg_T *message)
{
    uint8_t buf[MESH_MESSAGE_GENERIC_DELTA_SET_LEN];
    size_t  index = 0;

    buf[index++] = instance_idx;
    buf[index++] = 0x00;
    buf[index++] = lowByte(MESH_MESSAGE_GENERIC_DELTA_SET_UNACKNOWLEDGED);
    buf[index++] = highByte(MESH_MESSAGE_GENERIC_DELTA_SET_UNACKNOWLEDGED);
    buf[index++] = ((message->delta_level >> 0) & 0xFF);
    buf[index++] = ((message->delta_level >> 8) & 0xFF);
    buf[index++] = ((message->delta_level >> 16) & 0xFF);
    buf[index++] = ((message->delta_level >> 24) & 0xFF);
    buf[index++] = message->tid;
    buf[index++] = message->transition_time;
    buf[index++] = message->delay;

    UART_SendMeshMessageRequest(buf, sizeof(buf));
}

static uint8_t MeshInternal_ConvertFromMsToMeshFormat(uint32_t time_ms)
{
    if ((time_ms / MESH_NUMBER_OF_MS_IN_100_MS) < MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
    {
        return (MESH_TRANSITION_TIME_STEP_RESOLUTION_100_MS | (time_ms / MESH_NUMBER_OF_MS_IN_100_MS));
    }
    else if ((time_ms / MESH_NUMBER_OF_MS_IN_1S) < MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
    {
        return (MESH_TRANSITION_TIME_STEP_RESOLUTION_1_S | (time_ms / MESH_NUMBER_OF_MS_IN_1S));
    }
    else if ((time_ms / MESH_NUMBER_OF_MS_IN_10S) < MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
    {
        return (MESH_TRANSITION_TIME_STEP_RESOLUTION_10_S | (time_ms / MESH_NUMBER_OF_MS_IN_10S));
    }
    else if ((time_ms / MESH_NUMBER_OF_MS_IN_10MIN) < MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
    {
        return (MESH_TRANSITION_TIME_STEP_RESOLUTION_10_MIN | (time_ms / MESH_NUMBER_OF_MS_IN_10MIN));
    }
    else
    {
        return (MESH_TRANSITION_TIME_STEP_RESOLUTION_10_MIN |
                (MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE & MESH_TRANSITION_TIME_NUMBER_OF_STEPS_MASK));
    }
}


static void MeshInternal_ProcessSensorStatus(uint8_t *p_payload, size_t len)
{
    if (len < 2)
    {
        LOG_INFO("Received empty Sensor Status message");
        return;
    }

    uint16_t src_addr = ((uint16_t)p_payload[len - 2]);
    src_addr |= ((uint16_t)p_payload[len - 1] << 8);

    if (len <= 2)
    {
        LOG_INFO("Received empty Sensor Status message from: %d", src_addr);
        return;
    }

    len -= 2;
    size_t index = 0;
    while (index < len)
    {
        LOG_INFO("ProcessSensorStatus index: %d", index);
        if (p_payload[index] & SS_FORMAT_MASK)
        {
            /* Length field in Sensor Status message is 0-based */
            size_t   message_len = ((p_payload[index++] & SS_LONG_LEN_MASK) >> SS_LONG_LEN_OFFSET) + 1;
            uint16_t property_id = ((uint16_t)p_payload[index++]);
            property_id |= ((uint16_t)p_payload[index++] << 8);

            MeshInternal_ProcessSensorProperty(property_id, p_payload + index, message_len, src_addr);
            index += message_len;
        }
        else
        {
            /* Length field in Sensor Status message is 0-based */
            size_t   message_len = ((p_payload[index] & SS_SHORT_LEN_MASK) >> SS_SHORT_LEN_OFFSET) + 1;
            uint16_t property_id = (p_payload[index++] & SS_SHORT_PROP_ID_LOW_MASK) >> SS_SHORT_PROP_ID_LOW_OFFSET;
            property_id |= ((uint16_t)p_payload[index++]) << SS_SHORT_PROP_ID_HIGH_OFFSET;

            MeshInternal_ProcessSensorProperty(property_id, p_payload + index, message_len, src_addr);
            index += message_len;
        }
    }
}

static void MeshInternal_ProcessSensorProperty(uint16_t property_id, uint8_t *p_payload, size_t len, uint16_t src_addr)
{
    switch (property_id)
    {
        case PRESENCE_DETECTED:
        {
            MeshInternal_ProcessPresenceDetected(p_payload, len, src_addr);
            break;
        }
        case PRESENT_AMBIENT_LIGHT_LEVEL:
        {
            MeshInternal_ProcessPresentAmbientLightLevel(p_payload, len, src_addr);
            break;
        }
        case PRESENT_DEVICE_INPUT_POWER:
        {
            MeshInternal_ProcessDeviceInputPower(p_payload, len, src_addr);
            break;
        }
        case PRESENT_INPUT_CURRENT:
        {
            MeshInternal_ProcessPresentInputCurrent(p_payload, len, src_addr);
            break;
        }
        case PRESENT_INPUT_VOLTAGE:
        {
            MeshInternal_ProcessPresentInputVoltage(p_payload, len, src_addr);
            break;
        }
        case TOTAL_DEVICE_ENERGY_USE:
        {
            MeshInternal_ProcessTotalDeviceEnergyUse(p_payload, len, src_addr);
            break;
        }
        case PRECISE_TOTAL_DEVICE_ENERGY_USE:
        {
            MeshInternal_ProcessPreciseTotalDeviceEnergyUse(p_payload, len, src_addr);
            break;
        }
        default:
        {
            LOG_INFO("Invalid property id");
        }
    }
}

static void MeshInternal_ProcessPresenceDetected(uint8_t *p_payload, size_t len, uint16_t src_addr)
{
    if (len != 1)
    {
        LOG_INFO("Invalid Length Sensor Status message");
        return;
    }

    SensorValue_T sensor_value = {.pir = p_payload[0]};

    SensorOutput_ProcessPresenceDetected(src_addr, sensor_value);
}

static void MeshInternal_ProcessPresentAmbientLightLevel(uint8_t *p_payload, size_t len, uint16_t src_addr)
{
    if (len != 3)
    {
        LOG_INFO("Invalid Length Sensor Status message");
        return;
    }

    SensorValue_T sensor_value = {.als = ((uint32_t)p_payload[0]) | ((uint32_t)p_payload[1] << 8) |
                                         ((uint32_t)p_payload[2] << 16)};

    SensorOutput_ProcessPresentAmbientLightLevel(src_addr, sensor_value);
}

static void MeshInternal_ProcessDeviceInputPower(uint8_t *p_payload, size_t len, uint16_t src_addr)
{
    if (len != 3)
    {
        LOG_INFO("Invalid Length Sensor Status message");
        return;
    }

    SensorValue_T sensor_value = {.power = ((uint32_t)p_payload[0]) | ((uint32_t)p_payload[1] << 8) |
                                           ((uint32_t)p_payload[2] << 16)};

    SensorOutput_ProcessPresentDeviceInputPower(src_addr, sensor_value);
}

static void MeshInternal_ProcessPresentInputCurrent(uint8_t *p_payload, size_t len, uint16_t src_addr)
{
    if (len != 2)
    {
        LOG_INFO("Invalid Length Sensor Status message");
        return;
    }

    SensorValue_T sensor_value;
    sensor_value.current = ((uint16_t)p_payload[0]) | ((uint16_t)p_payload[1] << 8);

    SensorOutput_ProcessPresentInputCurrent(src_addr, sensor_value);
}

static void MeshInternal_ProcessPresentInputVoltage(uint8_t *p_payload, size_t len, uint16_t src_addr)
{
    if (len != 2)
    {
        LOG_INFO("Invalid Length Sensor Status message");
        return;
    }

    SensorValue_T sensor_value;
    sensor_value.voltage = ((uint16_t)p_payload[0]) | ((uint16_t)p_payload[1] << 8);

    SensorOutput_ProcessPresentInputVoltage(src_addr, sensor_value);
}

static void MeshInternal_ProcessTotalDeviceEnergyUse(uint8_t *p_payload, size_t len, uint16_t src_addr)
{
    if (len != 3)
    {
        LOG_INFO("Invalid Length Sensor Status message");
        return;
    }

    SensorValue_T sensor_value = {
        .energy = (((uint32_t)p_payload[0]) | ((uint32_t)p_payload[1] << 8) | ((uint32_t)p_payload[2] << 16))};

    SensorOutput_ProcessTotalDeviceEnergyUse(src_addr, sensor_value);
}

static void MeshInternal_ProcessPreciseTotalDeviceEnergyUse(uint8_t *p_payload, size_t len, uint16_t src_addr)
{
    if (len != 4)
    {
        LOG_INFO("Invalid Length Sensor Status message");
        return;
    }

    SensorValue_T sensor_value = {.precise_energy = ((uint32_t)p_payload[0]) | ((uint32_t)p_payload[1] << 8) |
                                                    ((uint32_t)p_payload[2] << 16) | ((uint32_t)p_payload[3] << 24)};

    SensorOutput_ProcessPreciseTotalDeviceEnergyUse(src_addr, sensor_value);
}
