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


#include "SensorOutput.h"

#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Arduino.h"
#include "LCD.h"
#include "Log.h"
#include "Timestamp.h"
#include "UARTProtocol.h"


static uint8_t SensorOutputIdx = INSTANCE_INDEX_UNKNOWN;


void SensorOutput_SetInstanceIdx(uint8_t idx)
{
    SensorOutputIdx = idx;
}

uint8_t SensorOutput_GetInstanceIdx(void)
{
    return SensorOutputIdx;
}

void SensorOutput_ProcessPresentAmbientLightLevel(uint16_t src_addr, SensorValue_T sensor_value)
{
    LOG_INFO("Decoded Sensor Status message from 0x%04X [%d ms], PRESENT AMBIENT LIGHT LEVEL with value of: "
             "%d.%02d",
             src_addr,
             Timestamp_GetCurrent(),
             sensor_value.als / 100,
             sensor_value.als % 100);
    LCD_UpdateSensorValue(PRESENT_AMBIENT_LIGHT_LEVEL, sensor_value);
}

void SensorOutput_ProcessPresenceDetected(uint16_t src_addr, SensorValue_T sensor_value)
{
    LOG_INFO("Decoded Sensor Status message from 0x%04X [%d ms], PRESENCE DETECTED with value of: %d",
             src_addr,
             Timestamp_GetCurrent(),
             sensor_value.pir);
    LCD_UpdateSensorValue(PRESENCE_DETECTED, sensor_value);
}

void SensorOutput_ProcessPresentDeviceInputPower(uint16_t src_addr, SensorValue_T sensor_value)
{
    LOG_INFO("Decoded Sensor Status message from 0x%04X [%d ms], PRESENT DEVICE INPUT POWER with value of: %d.%02d",
             src_addr,
             Timestamp_GetCurrent(),
             sensor_value.power / 10,
             sensor_value.power % 10);
    LCD_UpdateSensorValue(PRESENT_DEVICE_INPUT_POWER, sensor_value);
}

void SensorOutput_ProcessPresentInputCurrent(uint16_t src_addr, SensorValue_T sensor_value)
{
    LOG_INFO("Decoded Sensor Status message from 0x%04X [%d ms], PRESENT INPUT CURRENT with value of: %d.%02d",
             src_addr,
             Timestamp_GetCurrent(),
             sensor_value.current / 100,
             sensor_value.current % 100);
    LCD_UpdateSensorValue(PRESENT_INPUT_CURRENT, sensor_value);
}

void SensorOutput_ProcessPresentInputVoltage(uint16_t src_addr, SensorValue_T sensor_value)
{
    LOG_INFO("Decoded Sensor Status message from 0x%04X [%d ms], PRESENT INPUT VOLTAGE with value of: %d.%02d",
             src_addr,
             Timestamp_GetCurrent(),
             sensor_value.voltage / 64,
             (sensor_value.voltage % 64) * 100 / 64);
    LCD_UpdateSensorValue(PRESENT_INPUT_VOLTAGE, sensor_value);
}

void SensorOutput_ProcessTotalDeviceEnergyUse(uint16_t src_addr, SensorValue_T sensor_value)
{
    LOG_INFO("Decoded Sensor Status message from 0x%04X [%d ms], TOTAL DEVICE ENERGY USE with value of: "
             "%d.%02d Wh",
             src_addr,
             Timestamp_GetCurrent(),
             sensor_value.energy);
    LCD_UpdateSensorValue(TOTAL_DEVICE_ENERGY_USE, sensor_value);
}

void SensorOutput_ProcessPreciseTotalDeviceEnergyUse(uint16_t src_addr, SensorValue_T sensor_value)
{
    LOG_INFO("Decoded Sensor Status message from 0x%04X [%d ms], PRECISE TOTAL DEVICE ENERGY USE with value of: "
             "%d.%02d Wh",
             src_addr,
             Timestamp_GetCurrent(),
             sensor_value.precise_energy);
    LCD_UpdateSensorValue(PRECISE_TOTAL_DEVICE_ENERGY_USE, sensor_value);
}

void SensorOutput_Setup(void)
{
    LOG_INFO("Sensor output initialization");
}
