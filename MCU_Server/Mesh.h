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

#ifndef MESH_H_
#define MESH_H_


#include "stddef.h"
#include "stdint.h"


/**
 * Supported Mesh Model IDs definitions
 */
#define MESH_MODEL_ID_LIGHT_LC_SERVER 0x130F
#define MESH_MODEL_ID_LIGHT_CTL_SERVER 0x1303
#define MESH_MODEL_ID_SENSOR_SERVER 0x1100
#define MESH_MODEL_ID_HEALTH_SERVER 0x0002

/**
* Supported Mesh Property IDs definitions
*/
#define MESH_PROPERTY_ID_PRESENCE_DETECTED 0x004D
#define MESH_PROPERTY_ID_PRESENT_AMBIENT_LIGHT_LEVEL 0x004E
#define MESH_PROPERTY_ID_PRESENT_INPUT_CURRENT 0x0057
#define MESH_PROPERTY_ID_PRESENT_INPUT_VOLTAGE 0x0059
#define MESH_PROPERTY_ID_PRESENT_DEVICE_INPUT_POWER 0x0052
#define MESH_PROPERTY_ID_PRECISE_TOTAL_DEVICE_ENERGY_USE 0x0072


/*
 *  Search for model ID in a message
 *
 *  @param p_payload            Pointer to payload
 *  @param len                  Payload len
 *  @param expected_model_id    Expected model ID
 *  @return                     True if found, false otherwise
 */
bool Mesh_IsModelAvailable(uint8_t *p_payload, uint8_t len, uint16_t expected_model_id);

/*
 *  Process Mesh Message Request command
 *
 *  @param p_payload    Pointer to payload
 *  @param len          Payload len
 */
void Mesh_ProcessMeshCommand(uint8_t *p_payload, size_t len);

/*
 *  Send Light Lightness Get message
 *
 *  @param instance_idx    Instance index
 */
void Mesh_SendLightLightnessGet(uint8_t instance_idx);

/*
 *  Process new target lightness
 *
 *  @param current             Current lightness value
 *  @param target              Target lightness value
 *  @param transition_time     Transition time
 */
extern void ProcessTargetLightness(uint16_t current, uint16_t target, uint32_t transition_time);

/*
 *  Process new target lightness temperature
 *
 *  @param current             Current lightness temperature value
 *  @param target              Target lightness temperature value
 *  @param transition_time     Transition time
 */
extern void ProcessTargetLightnessTemp(uint16_t current, uint16_t target, uint32_t transition_time);

#endif    // MESH_H_
