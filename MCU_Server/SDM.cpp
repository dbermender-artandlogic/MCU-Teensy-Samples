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


#include "SDM.h"

#include "Arduino.h"
#include "Log.h"
#include "MODBUS.h"
#include "Timestamp.h"
#include "UARTProtocol.h"


/**
 * SDM communication configuration
 */
#define SDM_QUERY_TIMEOUT 200
#define SDM_DEFAULT_ADDRESS 1
#define SDM_MAX_TIMEOUTS_IN_ROW_ALLOWED 10

/**
 * SDM register addresses
 */
#define SDM_INPUT_REG_VOLTAGE 0x0000
#define SDM_INPUT_REG_CURRENT 0x0006
#define SDM_INPUT_REG_ACTIVE_POWER 0x000C
#define SDM_INPUT_REG_APPARENT_POWER 0x0012
#define SDM_INPUT_REG_REACTIVE_POWER 0x0018
#define SDM_INPUT_REG_POWER_FACTOR 0x001E
#define SDM_INPUT_REG_FREQUENCY 0x0046
#define SDM_INPUT_REG_IMPORT_ACTIVE_ENERGY 0x0048
#define SDM_INPUT_REG_EXPORT_ACTIVE_ENERGY 0x004A
#define SDM_INPUT_REG_IMPORT_REACTIVE_ENERGY 0x004C
#define SDM_INPUT_REG_EXPORT_REACTIVE_ENERGY 0x004E
#define SDM_INPUT_REG_TOTAL_SYSTEM_POWER_DEMAND 0x0054
#define SDM_INPUT_REG_MAX_TOTAL_SYSTEM_POWER_DEMAND 0x0056
#define SDM_INPUT_REG_IMPORT_SYSTEM_POWER_DEMAND 0x0058
#define SDM_INPUT_REG_MAX_IMPORT_SYSTEM_POWER_DEMAND 0x005A
#define SDM_INPUT_REG_EXPORT_SYSTEM_POWER_DEMAND 0x005C
#define SDM_INPUT_REG_MAX_EXPORT_SYSTEM_POWER_DEMAND 0x005E
#define SDM_INPUT_REG_CURRENT_DEMAND 0x0102
#define SDM_INPUT_REG_MAX_CURRENT_DEMAND 0x0108
#define SDM_INPUT_REG_TOTAL_ACTIVE_ENERGY 0x0156
#define SDM_INPUT_REG_TOTAL_REACTIVE_ENERGY 0x0158
#define SDM_HOLDING_REG_RELAY_PULSE_WIDTH 0x000C
#define SDM_HOLDING_REG_NETWORK_PARITY_STOP 0x0012
#define SDM_HOLDING_REG_METER_ID 0x0014
#define SDM_HOLDING_REG_BAUD_RATE 0x001C
#define SDM_HOLDING_REG_CT_PRIMARY_CURRENT 0x0032
#define SDM_HOLDING_REG_PULSE_1_OUTPUT_MODE 0x0056
#define SDM_HOLDING_REG_TIME_OF_SCROLL_DISPLAY 0xF900
#define SDM_HOLDING_REG_PULSE_1_OUTPUT 0xF910
#define SDM_HOLDING_REG_MEASUREMENT_MODE 0xF920

/**
 * SDM no query value
 */
#define SDM_NO_QUERY 0xFFFF


static const uint16_t input_query_table[] = {
    SDM_INPUT_REG_VOLTAGE,
    SDM_INPUT_REG_CURRENT,
    SDM_INPUT_REG_ACTIVE_POWER,
    SDM_INPUT_REG_TOTAL_ACTIVE_ENERGY,
};
static const size_t input_query_entries = sizeof(input_query_table) / sizeof(*input_query_table);

static const uint16_t holding_query_table[] = {};
static const size_t   holding_query_entries = sizeof(holding_query_table) / sizeof(*holding_query_table);

static bool        is_enabled           = false;
static uint8_t     slave_address        = SDM_DEFAULT_ADDRESS;
static SDM_State_T state                = {0};
static uint16_t    waiting_for_query    = SDM_NO_QUERY;
static uint8_t     query_index          = 0;
static uint32_t    last_query_timestamp = 0;
static uint32_t    timeouts_in_row      = SDM_MAX_TIMEOUTS_IN_ROW_ALLOWED + 1;


/**
 * Request input register value
 *
 * @param address   Register address
 */
static void SDM_SendRequestInput(uint16_t address);

/**
 * Request holding register value
 *
 * @param address   Register address
 */
static void SDM_SendRequestHolding(uint16_t address);

/**
 * Request float value write
 *
 * @param value     Value to be written
 * @param address   Address to write
 */
static void SDM_SendSetFloat(float value, uint16_t address);

/**
 * Request word value write
 *
 * @param value     Value to be written
 * @param address   Address to write
 */
static void SDM_SendSetHEX(uint16_t value, uint16_t address);

/**
 * Process incoming float data and write it to address
 *
 * @param data_len  Incoming data len
 * @param p_data    Pointer to incoming register
 * @param p_dest    Pointer to write float
 */
static void SDM_ProcessFloatData(size_t data_len, uint16_t *p_data, float *p_dest);

/**
 * Process incoming word data and write it to address
 *
 * @param data_len  Incoming data len
 * @param p_data    Pointer to incoming register
 * @param p_dest    Pointer to write word
 */
static void SDM_ProcessUint16Data(size_t data_len, uint16_t *p_data, uint16_t *p_dest);

/**
 * Process incoming byte in float data and write it to address
 *
 * @param data_len  Incoming data len
 * @param p_data    Pointer to incoming register
 * @param p_dest    Pointer to write byte
 */
static void SDM_ProcessUint8InsideFloatData(size_t data_len, uint16_t *p_data, uint8_t *p_dest);


void LoopSDM(void)
{
    if (!is_enabled)
        return;

    if (waiting_for_query != SDM_NO_QUERY)
    {
        if (Timestamp_GetTimeElapsed(last_query_timestamp, Timestamp_GetCurrent()) >= SDM_QUERY_TIMEOUT)
        {
            timeouts_in_row++;

            waiting_for_query = SDM_NO_QUERY;
            MODBUS_ClearBuffer();
        }
        else
        {
            MODBUS_ProcessIncoming();
        }
    }

    if (waiting_for_query == SDM_NO_QUERY)
    {
        switch (query_index)
        {
            case 0 ... input_query_entries - 1:
            {
                uint16_t query_address = input_query_table[query_index];

                SDM_SendRequestInput(query_address);
                waiting_for_query    = query_address;
                last_query_timestamp = Timestamp_GetCurrent();

                query_index++;
                break;
            }
#if holding_query_entries == 0
            case input_query_entries:
#else
            case input_query_entries ... input_query_entries + (holding_query_entries - 1):
#endif
            {
                uint16_t query_address = holding_query_table[query_index - input_query_entries];

                SDM_SendRequestHolding(query_address);
                waiting_for_query    = query_address;
                last_query_timestamp = Timestamp_GetCurrent();

                query_index++;
                break;
            }
            default:
            {
                query_index       = 0;
                waiting_for_query = SDM_NO_QUERY;

                break;
            }
        }
    }
}

void SetupSDM(void)
{
    MODBUS_INTERFACE.begin(MODBUS_INTERFACE_BAUDRATE);
    MODBUS_INTERFACE.transmitterEnable(PIN_MODBUS_RTS);
    // Waits for debug interface initialization.
    delay(1000);
    is_enabled = true;
}

const SDM_State_T *SDM_GetState(void)
{
    if (timeouts_in_row >= SDM_MAX_TIMEOUTS_IN_ROW_ALLOWED)
        return NULL;

    return &state;
}

void SDM_SetRelayPulseWidth(uint8_t relay_pulse_width)
{
    SDM_SendSetFloat((float)relay_pulse_width, SDM_HOLDING_REG_RELAY_PULSE_WIDTH);
}

void SDM_SetNetworkParityStop(uint8_t network_parity_stop)
{
    SDM_SendSetFloat((float)network_parity_stop, SDM_HOLDING_REG_NETWORK_PARITY_STOP);
}

void SDM_SetMeterID(uint8_t meter_id)
{
    SDM_SendSetFloat((float)meter_id, SDM_HOLDING_REG_METER_ID);
}

void SDM_SetBaudRate(uint8_t baud_rate)
{
    SDM_SendSetFloat((float)baud_rate, SDM_HOLDING_REG_BAUD_RATE);
}

void SDM_SetCTPrimaryCurrent(uint16_t ct_primary_current)
{
    SDM_SendSetFloat((float)ct_primary_current, SDM_HOLDING_REG_CT_PRIMARY_CURRENT);
}

void SDM_SetPulse1OutputMode(uint8_t pulse1_output_mode)
{
    SDM_SendSetHEX(pulse1_output_mode, SDM_HOLDING_REG_PULSE_1_OUTPUT_MODE);
}

void SDM_SetTimeOfScrollDisplay(uint16_t time_of_scroll_display)
{
    SDM_SendSetHEX(time_of_scroll_display, SDM_HOLDING_REG_TIME_OF_SCROLL_DISPLAY);
}

void SDM_SetPulse1Output(uint16_t pulse1_output)
{
    SDM_SendSetHEX(pulse1_output, SDM_HOLDING_REG_PULSE_1_OUTPUT);
}

void SDM_SetMeasurementMode(uint16_t measurement_mode)
{
    SDM_SendSetHEX(measurement_mode, SDM_HOLDING_REG_MEASUREMENT_MODE);
}

void MODBUS_ProcessReadInputRegisters(uint8_t slave_address, size_t data_len, uint16_t *p_data)
{
    LOG_DEBUG("Processing query: %04X, len %d", waiting_for_query, data_len);

    switch (waiting_for_query)
    {
        case SDM_INPUT_REG_VOLTAGE:
            SDM_ProcessFloatData(data_len, p_data, &state.voltage);
            break;

        case SDM_INPUT_REG_CURRENT:
            SDM_ProcessFloatData(data_len, p_data, &state.current);
            break;

        case SDM_INPUT_REG_ACTIVE_POWER:
            SDM_ProcessFloatData(data_len, p_data, &state.active_power);
            break;

        case SDM_INPUT_REG_APPARENT_POWER:
            SDM_ProcessFloatData(data_len, p_data, &state.apparent_power);
            break;

        case SDM_INPUT_REG_REACTIVE_POWER:
            SDM_ProcessFloatData(data_len, p_data, &state.reactive_power);
            break;

        case SDM_INPUT_REG_POWER_FACTOR:
            SDM_ProcessFloatData(data_len, p_data, &state.power_factor);
            break;

        case SDM_INPUT_REG_FREQUENCY:
            SDM_ProcessFloatData(data_len, p_data, &state.frequency);
            break;

        case SDM_INPUT_REG_IMPORT_ACTIVE_ENERGY:
            SDM_ProcessFloatData(data_len, p_data, &state.import_active_energy);
            break;

        case SDM_INPUT_REG_EXPORT_ACTIVE_ENERGY:
            SDM_ProcessFloatData(data_len, p_data, &state.export_active_energy);
            break;

        case SDM_INPUT_REG_IMPORT_REACTIVE_ENERGY:
            SDM_ProcessFloatData(data_len, p_data, &state.import_reactive_energy);
            break;

        case SDM_INPUT_REG_EXPORT_REACTIVE_ENERGY:
            SDM_ProcessFloatData(data_len, p_data, &state.export_reactive_energy);
            break;

        case SDM_INPUT_REG_TOTAL_SYSTEM_POWER_DEMAND:
            SDM_ProcessFloatData(data_len, p_data, &state.total_system_power_demand);
            break;

        case SDM_INPUT_REG_MAX_TOTAL_SYSTEM_POWER_DEMAND:
            SDM_ProcessFloatData(data_len, p_data, &state.max_total_system_power_demand);
            break;

        case SDM_INPUT_REG_IMPORT_SYSTEM_POWER_DEMAND:
            SDM_ProcessFloatData(data_len, p_data, &state.import_system_power_demand);
            break;

        case SDM_INPUT_REG_MAX_IMPORT_SYSTEM_POWER_DEMAND:
            SDM_ProcessFloatData(data_len, p_data, &state.max_import_system_power_demand);
            break;

        case SDM_INPUT_REG_EXPORT_SYSTEM_POWER_DEMAND:
            SDM_ProcessFloatData(data_len, p_data, &state.export_system_power_demand);
            break;

        case SDM_INPUT_REG_MAX_EXPORT_SYSTEM_POWER_DEMAND:
            SDM_ProcessFloatData(data_len, p_data, &state.max_export_system_power_demand);
            break;

        case SDM_INPUT_REG_CURRENT_DEMAND:
            SDM_ProcessFloatData(data_len, p_data, &state.current_demand);
            break;

        case SDM_INPUT_REG_MAX_CURRENT_DEMAND:
            SDM_ProcessFloatData(data_len, p_data, &state.max_current_demand);
            break;

        case SDM_INPUT_REG_TOTAL_ACTIVE_ENERGY:
            SDM_ProcessFloatData(data_len, p_data, &state.total_active_energy);
            break;

        case SDM_INPUT_REG_TOTAL_REACTIVE_ENERGY:
            SDM_ProcessFloatData(data_len, p_data, &state.total_reactive_energy);
            break;

        default:
            break;
    }

    waiting_for_query = SDM_NO_QUERY;
    timeouts_in_row   = 0;
}

void MODBUS_ProcessReadHoldingRegisters(uint8_t slave_address, size_t data_len, uint16_t *p_data)
{
    LOG_DEBUG("Processing query: %04X, len %d", waiting_for_query, data_len);

    switch (waiting_for_query)
    {
        case SDM_HOLDING_REG_RELAY_PULSE_WIDTH:
            SDM_ProcessUint8InsideFloatData(data_len, p_data, &state.relay_pulse_width);
            break;

        case SDM_HOLDING_REG_NETWORK_PARITY_STOP:
            SDM_ProcessUint8InsideFloatData(data_len, p_data, &state.network_parity_stop);
            break;

        case SDM_HOLDING_REG_METER_ID:
            SDM_ProcessUint8InsideFloatData(data_len, p_data, &state.meter_id);
            break;

        case SDM_HOLDING_REG_BAUD_RATE:
            SDM_ProcessUint8InsideFloatData(data_len, p_data, &state.baud_rate);
            break;

        case SDM_HOLDING_REG_CT_PRIMARY_CURRENT:
            SDM_ProcessUint8InsideFloatData(data_len, p_data, &state.ct_primary_current);
            break;

        case SDM_HOLDING_REG_PULSE_1_OUTPUT_MODE:
            SDM_ProcessUint8InsideFloatData(data_len, p_data, &state.pulse1_output_mode);
            break;

        case SDM_HOLDING_REG_TIME_OF_SCROLL_DISPLAY:
            SDM_ProcessUint16Data(data_len, p_data, &state.time_of_scroll_display);
            break;

        case SDM_HOLDING_REG_PULSE_1_OUTPUT:
            SDM_ProcessUint16Data(data_len, p_data, &state.pulse1_output);
            break;

        case SDM_HOLDING_REG_MEASUREMENT_MODE:
            SDM_ProcessUint16Data(data_len, p_data, &state.measurement_mode);
            break;

        default:
            break;
    }

    waiting_for_query = SDM_NO_QUERY;
    timeouts_in_row   = 0;
}


void MODBUS_ProcessReadPresetSingleRegister(uint8_t slave_address, uint16_t register_address, uint16_t preset_data)
{
    /// Not implemented
}

void MODBUS_ProcessReadPresetMultipleRegisters(uint8_t  slave_address,
                                               uint16_t starting_address,
                                               uint16_t num_of_registers)
{
    /// Not implemented
}

void MODBUS_ProcessException(uint8_t slave_address, uint8_t original_function_code, uint8_t error_code)
{
    /// Not implemented
    LOG_DEBUG("Received MODBUS exception");

    waiting_for_query = SDM_NO_QUERY;
}

static void SDM_SendRequestInput(uint16_t address)
{
    MODBUS_SendReadInputRegisters(slave_address, address, sizeof(float) / sizeof(uint16_t));
}

static void SDM_SendRequestHolding(uint16_t address)
{
    MODBUS_SendReadHoldingRegisters(slave_address, address, sizeof(float) / sizeof(uint16_t));
}


static void SDM_SendSetFloat(float value, uint16_t address)
{
    uint16_t  to_send[sizeof(float) / sizeof(uint16_t)];
    uint16_t *p_value = (uint16_t *)&value;

    to_send[0] = p_value[1];
    to_send[1] = p_value[0];

    MODBUS_SendPresetMultipleRegisters(slave_address, address, 2, to_send);
}

static void SDM_SendSetHEX(uint16_t value, uint16_t address)
{
    MODBUS_SendPresetSingleRegister(slave_address, address, value);
}

static void SDM_ProcessFloatData(size_t data_len, uint16_t *p_data, float *p_dest)
{
    if (data_len < (sizeof(*p_dest) / sizeof(uint16_t)))
    {
        LOG_DEBUG("Cannot update float, data_len: %d", data_len);
        MODBUS_ClearBuffer();
        return;
    }

    uint16_t *p_dest_uint16 = (uint16_t *)p_dest;
    p_dest_uint16[1]        = p_data[0];
    p_dest_uint16[0]        = p_data[1];

    LOG_DEBUG("Updated %p to %f", p_dest, *p_dest);
}

static void SDM_ProcessUint16Data(size_t data_len, uint16_t *p_data, uint16_t *p_dest)
{
    if (data_len < (sizeof(*p_dest) / sizeof(uint16_t)))
    {
        MODBUS_ClearBuffer();
        return;
    }

    *p_dest = p_data[0];
}

static void SDM_ProcessUint8InsideFloatData(size_t data_len, uint16_t *p_data, uint8_t *p_dest)
{
    float data;
    SDM_ProcessFloatData(data_len, p_data, &data);
    *p_dest = (uint8_t)(data + 0.5f);
}
