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

#ifndef SENSOR_OUTPUT_H
#define SENSOR_OUTPUT_H


#include <stdint.h>


typedef union
{
    uint32_t als;
    uint8_t  pir;
    uint32_t power;
    uint16_t current;
    uint16_t voltage;
    uint32_t energy;
    uint32_t precise_energy;
} SensorValue_T;

typedef enum
{
    PRESENCE_DETECTED               = 0x004D,
    PRESENT_AMBIENT_LIGHT_LEVEL     = 0x004E,
    PRESENT_DEVICE_INPUT_POWER      = 0x0052,
    PRESENT_INPUT_CURRENT           = 0x0057,
    PRESENT_INPUT_VOLTAGE           = 0x0059,
    TOTAL_DEVICE_ENERGY_USE         = 0x006A,
    PRECISE_TOTAL_DEVICE_ENERGY_USE = 0x0072
} SensorProperty_T;


#define MESH_PROP_PRESENT_AMBIENT_LIGHT_LEVEL_UNKNOWN_VAL 0xFFFFFF
#define MESH_PROP_PRESENT_DEVICE_INPUT_POWER_UNKNOWN_VAL 0xFFFFFF
#define MESH_PROP_PRESENT_INPUT_CURRENT_UNKNOWN_VAL 0xFFFF
#define MESH_PROP_PRESENT_INPUT_VOLTAGE_UNKNOWN_VAL 0xFFFF
#define MESH_PROP_TOTAL_DEVICE_ENERGY_USE_UNKNOWN_VAL 0xFFFFFF
#define MESH_PROP_PRECISE_TOTAL_DEVICE_ENERGY_USE_UNKNOWN_VAL 0xFFFFFFFF
#define MESH_PROP_PRECISE_TOTAL_DEVICE_ENERGY_USE_NOT_VALID_VAL 0xFFFFFFFE


/*
 *  Set Sensor Output instance index
 *
 *  @param idx  Lightness value
 */
void SensorOutput_SetInstanceIdx(uint8_t idx);

/*
 *  Get Sensor Output instance index
 *
 *  @return     Lightness value
 */
uint8_t SensorOutput_GetInstanceIdx(void);

/*
 *  Process ALS value update
 *
 *  @param src_addr            Source address
 *  @param sensor_value        New sensor value
 */
void SensorOutput_ProcessPresentAmbientLightLevel(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process PIR value update
 *
 *  @param src_addr            Source address
 *  @param sensor_value        New sensor value
 */
void SensorOutput_ProcessPresenceDetected(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process Power value update
 *
 *  @param sensor_value       New Power value
 *  @param src_addr           Source address
 */
void SensorOutput_ProcessPresentDeviceInputPower(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process Current value update
 *
 *  @param sensor_value       New Current value
 *  @param src_addr           Source address
 */
void SensorOutput_ProcessPresentInputCurrent(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process Voltage value update
 *
 *  @param sensor_value       New Voltage value
 *  @param src_addr           Source address
 */
void SensorOutput_ProcessPresentInputVoltage(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process Energy value update
 *
 *  @param sensor_value       New Energy value
 *  @param src_addr           Source address
 */
void SensorOutput_ProcessTotalDeviceEnergyUse(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process Precise Energy value update
 *
 *  @param sensor_value       New Energy value
 *  @param src_addr           Source address
 */
void SensorOutput_ProcessPreciseTotalDeviceEnergyUse(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Setup sensor server hardware
 */
void SensorOutput_Setup(void);

#endif    // SENSOR_OUTPUT_H
