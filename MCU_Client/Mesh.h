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


#include "MCU_Sensor.h"
#include "stddef.h"
#include "stdint.h"


/**
 * Supported Mesh Model IDs definitions
 */
#define MESH_MODEL_ID_GENERIC_ONOFF_CLIENT 0x1001
#define MESH_MODEL_ID_SENSOR_CLIENT 0x1102
#define MESH_MODEL_ID_GENERIC_LEVEL_CLIENT 0x1003
#define MESH_MODEL_ID_LIGHT_LIGHTNESS_CLIENT 0x1302
#define MESH_MODEL_ID_LIGHT_LC_CLIENT 0x1311
#define MESH_MODEL_ID_LIGHT_CTL_CLIENT 0x1305


/*
 *  Whis function should be called in Arduino main loop
 */
void Mesh_Loop(void);

/*
 *  Search for model ID in a message
 *
 *  @param p_payload            Pointer to p_payload
 *  @param len                  Payload len
 *  @param expected_model_id    Expected model ID
 *  @return                     True if found, false otherwise
 */
bool Mesh_IsModelAvailable(uint8_t *p_payload, uint8_t len, uint16_t expected_model_id);

/*
 *  Process Mesh Message Request command
 *
 *  @param p_payload    Pointer to p_payload
 *  @param len          Payload len
 */
void Mesh_ProcessMeshCommand(uint8_t *p_payload, size_t len);

/*
 *  Send Generic OnOff Set Unacknowledged message with repeats.
 *
 *  @param instance_idx        Instance index.
 *  @param value               Generic OnOff target value.
 *  @param transition_time     Transition time (mesh format).
 *  @param delay_ms            Delay in miliseconds.
 *  @param num_of_repeats      Number of message repeats.
 *  @param is_new_transaction  Is it a new transaction?
 */
void Mesh_SendGenericOnOffSet(uint8_t  instance_idx,
                              bool     value,
                              unsigned transition_time,
                              unsigned delay_ms,
                              uint8_t  num_of_repeats,
                              bool     is_new_transaction);

/*
 *  Send Light Lightness Set Unacknowledged message with repeats.
 *
 *  @param instance_idx        Instance index.
 *  @param value               Light Lightness target value.
 *  @param transition_time     Transition time (mesh format).
 *  @param delay_ms            Delay in miliseconds.
 *  @param num_of_repeats      Number of message repeats.
 *  @param is_new_transaction  Is it a new transaction?
 */
void Mesh_SendLightLightnessSet(uint8_t  instance_idx,
                                uint16_t value,
                                unsigned transition_time,
                                unsigned delay_ms,
                                uint8_t  num_of_repeats,
                                bool     is_new_transaction);

/*
 *  Send Generic Delta Set Unacknowledged message with repeats.
 *
 *  @param instance_idx        Instance index.
 *  @param value               Delta change of the value.
 *  @param transition_time     Transition time (mesh format).
 *  @param delay_ms            Delay in miliseconds.
 *  @param num_of_repeats      Number of message repeats.
 *  @param is_new_transaction  Is it a new transaction?
 */
void Mesh_SendGenericDeltaSet(uint8_t  instance_idx,
                              int32_t  value,
                              unsigned transition_time,
                              unsigned delay_ms,
                              uint8_t  num_of_repeats,
                              bool     is_new_transaction);

/*
 *  Send Generic Level Set Unacknowledged message with repeats.
 *
 *  @param instance_idx        Instance index.
 *  @param value               Generic Level target value.
 *  @param transition_time     Transition time (mesh format).
 *  @param delay_ms            Delay in miliseconds.
 *  @param num_of_repeats      Number of message repeats.
 *  @param is_new_transaction  Is it a new transaction?
 */
void Mesh_SendGenericLevelSet(uint8_t  instance_idx,
                              uint16_t value,
                              unsigned transition_time,
                              unsigned delay_ms,
                              uint8_t  num_of_repeats,
                              bool     is_new_transaction);

/*
 *  Process ALS update
 *
 *  @param value_clux  New ALS value in clux
 *  @param src_addr    Source address
 */
extern void ProcessPresentAmbientLightLevel(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process PIR update
 *
 *  @param value       New PIR value
 *  @param src_addr    Source address
 */
extern void ProcessPresenceDetected(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process Power value update
 *
 *  @param value       New Power value
 *  @param src_addr    Source address
 */
extern void ProcessPresentDeviceInputPower(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process Current value update
 *
 *  @param value       New Current value
 *  @param src_addr    Source address
 */
extern void ProcessPresentInputCurrent(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process Voltage value update
 *
 *  @param value       New Voltage value
 *  @param src_addr    Source address
 */
extern void ProcessPresentInputVoltage(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process Energy value update
 *
 *  @param value       New Energy value
 *  @param src_addr    Source address
 */
extern void ProcessTotalDeviceEnergyUse(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process Precise Energy value update
 *
 *  @param value       New Energy value
 *  @param src_addr    Source address
 */
extern void ProcessPreciseTotalDeviceEnergyUse(uint16_t src_addr, SensorValue_T sensor_value);

#endif    // MESH_H_
