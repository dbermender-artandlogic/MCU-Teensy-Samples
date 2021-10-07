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


#include "SensorInput.h"

#include <TimerOne.h>
#include <TimerThree.h>
#include <math.h>

#include "Log.h"
#include "Mesh.h"
#include "SDM.h"
#include "Timestamp.h"
#include "UARTProtocol.h"

#define ALS_CONVERSION_COEFFICIENT 14UL     /**< light sensor coefficient [centilux / millivolt] */
#define ALS_MAX_MODEL_VALUE (0xFFFFFF - 1)  /**<  Maximal allowed value of ALS reading passed to model */
#define PIR_DEBOUNCE_TIME_MS 20             /**< Maximal PIR debounce time in milliseconds */
#define PIR_INERTIA_MS 4000                 /**< PIR inertia in milliseconds */
#define SENSOR_UPDATE_INTV_PIR 200          /**< sensor update in milliseconds for PIR Sensor */
#define SENSOR_UPDATE_INTV_ALS 200          /**< sensor update in milliseconds for ALS Sensor */
#define SENSOR_UPDATE_INTV_CURR_ENERGY 1000 /**< sensor update in milliseconds for Current and Precise Energy Sensor */
#define SENSOR_UPDATE_INTV_VOLT_POWER 1000  /**< sensor update in milliseconds for Voltage and Power Sensor */
#define ALS_REPORT_THRESHOLD 500            /**< sensor threshold in centilux */
#define ANALOG_REFERENCE_VOLTAGE_MV 3300    /**< ADC reference voltage in millivolts */
#define ANALOG_MIN 0                        /**< lower range of analog measurements. */
#define ANALOG_MAX 1023                     /**< uppper range of analog measurements. */


static bool              IsEnabled                       = false;
static volatile uint32_t PirTimestamp                    = 0;
static uint8_t           SensorInputPirIdx               = INSTANCE_INDEX_UNKNOWN;
static uint8_t           SensorInputAlsIdx               = INSTANCE_INDEX_UNKNOWN;
static uint8_t           SensorInputCurrPreciseEnergyIdx = INSTANCE_INDEX_UNKNOWN;
static uint8_t           SensorInputVoltPowIdx           = INSTANCE_INDEX_UNKNOWN;


/**
 * Convert floating point value to Voltage Characteristic
 *
 * @param voltage   value
 * @return          encoded value
 */
static uint16_t ConvertFloatToVoltage(float voltage);

/**
 * Convert floating point value to Electric Current Characteristic
 *
 * @param voltage   value
 * @return          encoded value
 */
static uint16_t ConvertFloatToCurrent(float voltage);

/**
 * Convert floating point value to Power Characteristic
 *
 * @param voltage   value
 * @return          encoded value
 */
static uint32_t ConvertFloatToPower(float power);

/**
 * Convert floating point value to Energy Characteristic
 *
 * @param voltage   value
 * @return          encoded value
 */
static uint32_t ConvertFloatToPreciseEnergy(float energy);

/**
 * Process PIR update
 */
static void ProcessPIR(void);

/**
 * Process ALS update
 */
static void ProcessALS(void);

/**
 * Process voltage, current update
 */
static void ProcessCurrPreciseEnergy(void);

/**
 * Process power and energy update
 */
static void ProcessVoltPow(void);


void SensorInput_SetAlsIdx(uint8_t idx)
{
    SensorInputAlsIdx = idx;
}

uint8_t SensorInput_GetAlsIdx(void)
{
    return SensorInputAlsIdx;
}

void SensorInput_SetPirIdx(uint8_t idx)
{
    SensorInputPirIdx = idx;
}

uint8_t SensorInput_GetPirIdx(void)
{
    return SensorInputPirIdx;
}

void SensorInput_SetCurrPreciseEnergyIdx(uint8_t idx)
{
    SensorInputCurrPreciseEnergyIdx = idx;
}

uint8_t SensorInput_GetCurrPreciseEnergyIdx(void)
{
    return SensorInputCurrPreciseEnergyIdx;
}

void SensorInput_SetVoltPowIdx(uint8_t idx)
{
    SensorInputVoltPowIdx = idx;
}

uint8_t SensorInput_GetVoltPowIdx(void)
{
    return SensorInputVoltPowIdx;
}

void InterruptPIR(void)
{
    PirTimestamp = Timestamp_GetCurrent();
}

void SensorInput_Setup(void)
{
    pinMode(PIN_PIR, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_PIR), InterruptPIR, RISING);
    IsEnabled = true;
}

void SensorInput_Loop(void)
{
    if (!IsEnabled)
        return;

    static unsigned long timestamp_pir         = 0;
    static unsigned long timestamp_als         = 0;
    static unsigned long timestamp_curr_energy = 0;
    static unsigned long timestamp_volt_power  = 0;

    if (Timestamp_GetTimeElapsed(timestamp_pir, Timestamp_GetCurrent()) >= SENSOR_UPDATE_INTV_PIR)
    {
        timestamp_pir = Timestamp_GetCurrent();
        ProcessPIR();
    }
    if (Timestamp_GetTimeElapsed(timestamp_als, Timestamp_GetCurrent()) >= SENSOR_UPDATE_INTV_ALS)
    {
        timestamp_als = Timestamp_GetCurrent();
        ProcessALS();
    }
    if (Timestamp_GetTimeElapsed(timestamp_curr_energy, Timestamp_GetCurrent()) >= SENSOR_UPDATE_INTV_CURR_ENERGY)
    {
        timestamp_curr_energy = Timestamp_GetCurrent();
        ProcessCurrPreciseEnergy();
    }
    if (Timestamp_GetTimeElapsed(timestamp_volt_power, Timestamp_GetCurrent()) >= SENSOR_UPDATE_INTV_VOLT_POWER)
    {
        timestamp_volt_power = Timestamp_GetCurrent();
        ProcessVoltPow();
    }
}


static void ProcessPIR(void)
{
    if (SensorInput_GetPirIdx() != INSTANCE_INDEX_UNKNOWN)
    {
        bool pir = digitalRead(PIN_PIR) ||
                   (Timestamp_GetTimeElapsed(PirTimestamp, Timestamp_GetCurrent()) < PIR_INERTIA_MS);

        uint8_t pir_buf[] = {
            SensorInputPirIdx,
            lowByte(MESH_PROP_ID_PRESENCE_DETECTED),
            highByte(MESH_PROP_ID_PRESENCE_DETECTED),
            pir,
        };
        UART_SendSensorUpdateRequest(pir_buf, sizeof(pir_buf));
    }
}

static void ProcessALS(void)
{
    if (SensorInput_GetAlsIdx() != INSTANCE_INDEX_UNKNOWN)
    {
        uint32_t als_adc_val    = analogRead(PIN_ALS);
        uint32_t als_millivolts = (als_adc_val * ANALOG_REFERENCE_VOLTAGE_MV) / ANALOG_MAX;
        uint32_t als_centilux   = als_millivolts * ALS_CONVERSION_COEFFICIENT;

        /*
        * Sensor Server can be configured to report on change. In one mode report is triggered by
        * percentage change from the actual value. In case of small measurement, it can generate heavy traffic.
        */
        if (als_centilux < ALS_REPORT_THRESHOLD)
        {
            als_centilux = 0;
        }

        if (als_centilux > ALS_MAX_MODEL_VALUE)
        {
            als_centilux = ALS_MAX_MODEL_VALUE;
        }

        uint8_t als_buf[] = {
            SensorInputAlsIdx,
            lowByte(MESH_PROP_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
            highByte(MESH_PROP_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
            (uint8_t)als_centilux,
            (uint8_t)(als_centilux >> 8),
            (uint8_t)(als_centilux >> 16),
        };

        UART_SendSensorUpdateRequest(als_buf, sizeof(als_buf));
    }
}

static void ProcessCurrPreciseEnergy(void)
{
    uint16_t current;
    uint32_t energy;

    const SDM_State_T *p_sdm_state = SDM_GetState();

    if (p_sdm_state != NULL)
    {
        current = ConvertFloatToCurrent(p_sdm_state->current);
        energy  = ConvertFloatToPreciseEnergy(p_sdm_state->total_active_energy);
    }
    else
    {
        current = MESH_PROP_PRESENT_INPUT_CURRENT_UNKNOWN_VAL;
        energy  = MESH_PROP_PRECISE_TOTAL_DEVICE_ENERGY_USE_UNKNOWN_VAL;
    }

    if (SensorInput_GetCurrPreciseEnergyIdx() != INSTANCE_INDEX_UNKNOWN)
    {
        uint8_t currpreciseenergy_buf[] = {SensorInputCurrPreciseEnergyIdx,
                                           lowByte(MESH_PROP_ID_PRESENT_INPUT_CURRENT),
                                           highByte(MESH_PROP_ID_PRESENT_INPUT_CURRENT),
                                           (uint8_t)current,
                                           (uint8_t)(current >> 8),
                                           lowByte(MESH_PROP_ID_PRECISE_TOTAL_DEVICE_ENERGY_USE),
                                           highByte(MESH_PROP_ID_PRECISE_TOTAL_DEVICE_ENERGY_USE),
                                           (uint8_t)energy,
                                           (uint8_t)(energy >> 8),
                                           (uint8_t)(energy >> 16),
                                           (uint8_t)(energy >> 24)};
        UART_SendSensorUpdateRequest(currpreciseenergy_buf, sizeof(currpreciseenergy_buf));
    }
}

static void ProcessVoltPow(void)
{
    uint16_t voltage;
    uint32_t power;

    const SDM_State_T *p_sdm_state = SDM_GetState();

    if (p_sdm_state != NULL)
    {
        voltage = ConvertFloatToVoltage(p_sdm_state->voltage);
        power   = ConvertFloatToPower(p_sdm_state->active_power);
    }
    else
    {
        voltage = MESH_PROP_PRESENT_INPUT_VOLTAGE_UNKNOWN_VAL;
        power   = MESH_PROP_PRESENT_DEVICE_INPUT_POWER_UNKNOWN_VAL;
    }

    if (SensorInput_GetVoltPowIdx() != INSTANCE_INDEX_UNKNOWN)
    {
        uint8_t voltpow_buf[] = {
            SensorInputVoltPowIdx,
            lowByte(MESH_PROP_ID_PRESENT_INPUT_VOLTAGE),
            highByte(MESH_PROP_ID_PRESENT_INPUT_VOLTAGE),
            (uint8_t)voltage,
            (uint8_t)(voltage >> 8),
            lowByte(MESH_PROP_ID_PRESENT_DEVICE_INPUT_POWER),
            highByte(MESH_PROP_ID_PRESENT_DEVICE_INPUT_POWER),
            (uint8_t)power,
            (uint8_t)(power >> 8),
            (uint8_t)(power >> 16),
        };
        UART_SendSensorUpdateRequest(voltpow_buf, sizeof(voltpow_buf));
    }
}

static uint16_t ConvertFloatToVoltage(float voltage)
{
    return (uint16_t)(voltage * 64);
}

static uint16_t ConvertFloatToCurrent(float voltage)
{
    return (uint16_t)(voltage * 100);
}

static uint32_t ConvertFloatToPower(float power)
{
    return (uint32_t)(power * 10);
}

static uint32_t ConvertFloatToPreciseEnergy(float energy)
{
    return (uint32_t)(energy * 1000);
}
