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
#include "Config.h"
#include "UARTProtocol.h"


/**
 * Supported Mesh Opcodes definitions
 */
#define MESH_MESSAGE_LIGHT_L_GET 0x824B
#define MESH_MESSAGE_LIGHT_L_SET 0x824C
#define MESH_MESSAGE_LIGHT_L_SET_UNACKNOWLEDGED 0x824D
#define MESH_MESSAGE_LIGHT_L_STATUS 0x824E
#define MESH_MESSAGE_SENSOR_STATUS 0x0052
#define MESH_MESSAGE_LEVEL_STATUS 0x8208
#define MESH_MESSAGE_LIGHT_CTL_TEMPERATURE_STATUS 0x8266
#define MESH_MESSAGE_LEVEL_GET 0x8205

/**
 * Used Mesh Messages len
 */
#define MESH_MESSAGE_LIGHT_L_GET_LEN 4

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

/**
 * Property IDs description
 */
#define PRESENCE_DETECTED_PROP_ID 0x004D
#define PRESENT_AMBIENT_LIGHT_LEVEL 0x004E


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

    DEBUG("Process Mesh Command [%d %d 0x%02X]\n", instance_index, instance_subindex, mesh_cmd);
    switch (mesh_cmd)
    {
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
            INFO("Rejected Transition Time\n");
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
            INFO("Rejected Transition Time\n");
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
            INFO("Rejected Transition Time\n");
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
