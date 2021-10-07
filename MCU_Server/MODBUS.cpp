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


#include "MODBUS.h"

#include "CRC.h"
#include "Config.h"
#include "Log.h"

#define MIN_RX_MODBUS_MESSAGE_LEN 4u
#define MAX_RX_MODBUS_MESSAGE_LEN 255u
#define MODBUS_TX_MESSAGE_LEN(_payload_len) (4 + _payload_len)
#define MODBUS_BASIC_COMMAND_PAYLOAD_LEN 4u
#define MODBUS_READ_HOLDING_REGISTERS_PAYLOAD_LEN 4u
#define MODBUS_READ_INPUT_REGISTERS_PAYLOAD_LEN 4u
#define MODBUS_PRESET_SINGLE_REGISTER_PAYLOAD_LEN 4u
#define MODBUS_PRESET_MULTIPLE_REGISTERS_PAYLOAD_LEN(_num_of_registers) (3 + (_num_of_registers * 2))

#define MODBUS_READ_MULTIPLE_REGISTERS_PAYLOAD_SIZE(_byte_count) (1u + _byte_count)
#define MODBUS_READ_SINGLE_REGISTER_PAYLOAD_SIZE 4u
#define MODBUS_EXCEPTION_PAYLOAD_SIZE 1u
#define MODBUS_CRC_SIZE sizeof(uint16_t)

#define MODBUS_SLAVE_ADDR_OFFSET 0u
#define MODBUS_FUNCTION_CODE_OFFSET (MODBUS_SLAVE_ADDR_OFFSET + 1u)
#define MODBUS_PAYLOAD_START_OFFSET (MODBUS_FUNCTION_CODE_OFFSET + 1u)

#define MODBUS_READ_COIL_STATUS 0x01u
#define MODBUS_READ_INPUT_STATUS 0x02u
#define MODBUS_READ_HOLDING_REGISTERS 0x03u
#define MODBUS_READ_INPUT_REGISTERS 0x04u
#define MODBUS_FORCE_SINGLE_COIL 0x05u
#define MODBUS_PRESET_SINGLE_REGISTER 0x06u
#define MODBUS_READ_EXCEPTION_STATUS 0x07u
#define MODBUS_DIAGNOSTICS 0x08u
#define MODBUS_FETCH_COEVENT_CTR 0x0Bu
#define MODBUS_FETCH_COEVENT_LOG 0x0Cu
#define MODBUS_FORCE_MULTIPLE_COILS 0x0Fu
#define MODBUS_PRESET_MULTIPLE_REGS 0x10u
#define MODBUS_REPORT_SLAVE_ID 0x11u
#define MODBUS_READ_GENERAL_REFERENCE 0x14u
#define MODBUS_WRITE_GENERAL_REFERENCE 0x15u
#define MODBUS_MASK_WRITE_4X_REGISTER 0x16u
#define MODBUS_READ_WRITE_4X_REGISTERS 0x17u
#define MODBUS_READ_FIFO_QUEUE 0x18u

#define MODBUS_ERROR_FIRST_ID 0x80u
#define MODBUS_ERROR_LAST_ID 0xFFu


typedef struct MODBUS_State_Tag
{
    uint8_t payload[MAX_RX_MODBUS_MESSAGE_LEN];
    uint8_t already_received;
    size_t  expected_len;
} MODBUS_State_T;

typedef struct MODBUS_Frame_Tag
{
    uint8_t  slave_address;
    uint8_t  function_code;
    size_t   len;
    uint8_t *p_payload;
} MODBUS_Frame_T;

static MODBUS_State_T state = {0};


/**
 * Determing if function code is supported or not.
 *
 * @param function_code     Function code to be checked
 * @return                  true if supported, false otherwise
 */
static bool MODBUS_IsFunctionCodeSupported(uint8_t function_code);

/**
 * Calculate expected message len based on function code and first byte of payload.
 *
 * @param function_code         Function code
 * @param first_payload_byte    First byte of payload
 * @return                      Expected message len
 */
static size_t MODBUS_ExpectedMessageLen(uint8_t function_code, uint8_t first_payload_byte);

/**
 * Process incoming MODBUS frame.
 *
 * @param payload   Complete received payload
 * @param len       Payload size
 */
static void MODBUS_ProcessFrame(uint8_t *payload, size_t len);

/**
 * Determine if message's CRC matches expected
 *
 * @param buffer    Buffer with raw message
 * @param len       Data len
 * @return          true is CRC OK, false otherwise
 */
static bool MODBUS_IsValidMessage(uint8_t *buffer, size_t len);

/**
 * Process MODBUS repsponse based on parsed frame.
 *
 * @param p_frame   Pointer to parsed frame
 */
static void MODBUS_ProcessResponse(MODBUS_Frame_T *p_frame);

/**
 * Send basic command - two words payload
 *
 * @param slave_address     Destination address
 * @param data1             First word
 * @param data2             Second word
 * @param funtion_code      Function code
 */
static void MODBUS_SendBasicCommand(uint8_t slave_address, uint16_t data1, uint16_t data2, uint8_t funtion_code);

/**
 * Send MODBUS frame.
 *
 * @param p_frame   Pointer to frame
 */
static void MODBUS_SendFrame(MODBUS_Frame_T *p_frame);


void MODBUS_ProcessIncoming(void)
{
    while (MODBUS_INTERFACE.available())
    {
        if (state.already_received == MAX_RX_MODBUS_MESSAGE_LEN)
            state.already_received = 0;

        state.payload[state.already_received] = MODBUS_INTERFACE.read();

        switch (state.already_received)
        {
            case MODBUS_SLAVE_ADDR_OFFSET:
            {
                LOG_DEBUG("Received slave address");
                state.already_received++;
                break;
            }
            case MODBUS_FUNCTION_CODE_OFFSET:
            {
                LOG_DEBUG("Received function code");
                if (MODBUS_IsFunctionCodeSupported(state.payload[MODBUS_FUNCTION_CODE_OFFSET]))
                    state.already_received++;
                else
                    state.already_received = 0;

                break;
            }
            case MODBUS_PAYLOAD_START_OFFSET:
            {
                state.already_received++;
                state.expected_len = MODBUS_ExpectedMessageLen(state.payload[MODBUS_FUNCTION_CODE_OFFSET],
                                                               state.payload[MODBUS_PAYLOAD_START_OFFSET]);
                LOG_DEBUG("Received first payload byte, expected len: %d", state.expected_len);
                break;
            }
            default:
            {
                if (state.expected_len > state.already_received)
                {
                    LOG_DEBUG("Received next payload byte");
                    state.already_received++;
                }

                if (state.expected_len == state.already_received)
                {
                    MODBUS_ProcessFrame(state.payload, state.already_received);

                    state.already_received = 0;
                }

                break;
            }
        }
    }
}

void MODBUS_ClearBuffer(void)
{
    state.already_received = 0;
}

void MODBUS_SendReadHoldingRegisters(uint8_t slave_address, uint16_t starting_address, uint16_t num_of_points)
{
    MODBUS_SendBasicCommand(slave_address, starting_address, num_of_points, MODBUS_READ_HOLDING_REGISTERS);
}

void MODBUS_SendReadInputRegisters(uint8_t slave_address, uint16_t starting_address, uint16_t num_of_points)
{
    MODBUS_SendBasicCommand(slave_address, starting_address, num_of_points, MODBUS_READ_INPUT_REGISTERS);
}

void MODBUS_SendPresetSingleRegister(uint8_t slave_address, uint16_t register_address, uint16_t preset_data)
{
    MODBUS_SendBasicCommand(slave_address, register_address, preset_data, MODBUS_PRESET_SINGLE_REGISTER);
}

void MODBUS_SendPresetMultipleRegisters(uint8_t   slave_address,
                                        uint16_t  starting_address,
                                        uint8_t   register_count,
                                        uint16_t *p_registers)
{
    MODBUS_Frame_T frame;
    uint8_t        buffer[MODBUS_PRESET_MULTIPLE_REGISTERS_PAYLOAD_LEN(register_count)];
    size_t         index = 0;

    buffer[index++] = highByte(starting_address);
    buffer[index++] = lowByte(starting_address);
    buffer[index++] = register_count;

    for (size_t i = 0; i < register_count; i++)
    {
        buffer[index++] = highByte(p_registers[i]);
        buffer[index++] = lowByte(p_registers[i]);
    }

    frame.slave_address = slave_address;
    frame.function_code = MODBUS_PRESET_MULTIPLE_REGS;
    frame.len           = sizeof(buffer);
    frame.p_payload     = buffer;

    MODBUS_SendFrame(&frame);
}


static void MODBUS_ProcessResponse(MODBUS_Frame_T *p_frame)
{
    LOG_DEBUG("Process function code: %02X", p_frame->function_code);

    switch (p_frame->function_code)
    {
        case MODBUS_READ_INPUT_REGISTERS:
        case MODBUS_READ_HOLDING_REGISTERS:
        {
            size_t   index      = 0;
            size_t   num_of_reg = p_frame->p_payload[index++] / sizeof(uint16_t);
            uint16_t registers[num_of_reg];

            for (size_t i = 0; i < num_of_reg; i++)
            {
                registers[i] = ((uint16_t)p_frame->p_payload[index++]) << 8;
                registers[i] |= ((uint16_t)p_frame->p_payload[index++]);
            }

            switch (p_frame->function_code)
            {
                case MODBUS_READ_INPUT_REGISTERS:
                    MODBUS_ProcessReadInputRegisters(p_frame->slave_address, num_of_reg, registers);
                    break;

                case MODBUS_READ_HOLDING_REGISTERS:
                    MODBUS_ProcessReadHoldingRegisters(p_frame->slave_address, num_of_reg, registers);
                    break;

                default:
                    break;
            }

            break;
        }
        case MODBUS_PRESET_SINGLE_REGISTER:
        case MODBUS_PRESET_MULTIPLE_REGS:
        {
            size_t   index   = 0;
            uint16_t address = ((uint16_t)p_frame->p_payload[index++] << 8);
            address |= ((uint16_t)p_frame->p_payload[index++]);
            uint16_t data = ((uint16_t)p_frame->p_payload[index++] << 8);
            data |= ((uint16_t)p_frame->p_payload[index++]);

            switch (p_frame->function_code)
            {
                case MODBUS_PRESET_SINGLE_REGISTER:
                    MODBUS_ProcessReadPresetSingleRegister(p_frame->slave_address, address, data);
                    break;

                case MODBUS_PRESET_MULTIPLE_REGS:
                    MODBUS_ProcessReadPresetMultipleRegisters(p_frame->slave_address, address, data);
                    break;

                default:
                    break;
            }

            break;
        }
        case MODBUS_ERROR_FIRST_ID ... MODBUS_ERROR_LAST_ID:
        {
            size_t  index                 = 0;
            uint8_t original_funtion_code = p_frame->function_code - MODBUS_ERROR_FIRST_ID;
            uint8_t error_code            = p_frame->p_payload[index++];
            MODBUS_ProcessException(p_frame->slave_address, original_funtion_code, error_code);

            break;
        }

        case MODBUS_READ_COIL_STATUS:
        case MODBUS_READ_INPUT_STATUS:
        case MODBUS_FORCE_SINGLE_COIL:
        case MODBUS_READ_EXCEPTION_STATUS:
        case MODBUS_DIAGNOSTICS:
        case MODBUS_FETCH_COEVENT_CTR:
        case MODBUS_FETCH_COEVENT_LOG:
        case MODBUS_FORCE_MULTIPLE_COILS:
        case MODBUS_REPORT_SLAVE_ID:
        case MODBUS_READ_GENERAL_REFERENCE:
        case MODBUS_WRITE_GENERAL_REFERENCE:
        case MODBUS_MASK_WRITE_4X_REGISTER:
        case MODBUS_READ_WRITE_4X_REGISTERS:
        case MODBUS_READ_FIFO_QUEUE:
        default:
        {
            /// Not supported
            break;
        }
    }
}

static void MODBUS_SendFrame(MODBUS_Frame_T *p_frame)
{
    size_t  index = 0;
    uint8_t buffer[MODBUS_TX_MESSAGE_LEN(p_frame->len)];

    buffer[index++] = p_frame->slave_address;
    buffer[index++] = p_frame->function_code;
    memcpy(buffer + index, p_frame->p_payload, p_frame->len);
    index += p_frame->len;

    uint16_t crc = CalcCRC16_Modbus(buffer, index, CRC16_INIT_VAL);

    buffer[index++] = highByte(crc);
    buffer[index++] = lowByte(crc);

    MODBUS_INTERFACE.write(buffer, sizeof(buffer));
}

static bool MODBUS_IsValidMessage(uint8_t *buffer, size_t len)
{
    uint16_t expected_crc = CalcCRC16_Modbus(buffer, len - sizeof(uint16_t), CRC16_INIT_VAL);
    uint16_t actual_crc   = buffer[len - 1];
    actual_crc |= (uint16_t)buffer[len - 2] << 8;

    LOG_DEBUG("Expected CRC: %04X, actual CRC: %04X", expected_crc, actual_crc);

    return expected_crc == actual_crc;
}

static void MODBUS_SendBasicCommand(uint8_t slave_address, uint16_t data1, uint16_t data2, uint8_t funtion_code)
{
    MODBUS_Frame_T frame;
    uint8_t        buffer[MODBUS_BASIC_COMMAND_PAYLOAD_LEN];
    size_t         index = 0;

    buffer[index++] = highByte(data1);
    buffer[index++] = lowByte(data1);
    buffer[index++] = highByte(data2);
    buffer[index++] = lowByte(data2);

    frame.slave_address = slave_address;
    frame.function_code = funtion_code;
    frame.len           = sizeof(buffer);
    frame.p_payload     = buffer;

    MODBUS_SendFrame(&frame);
}

static size_t MODBUS_ExpectedMessageLen(uint8_t function_code, uint8_t first_payload_byte)
{
    switch (function_code)
    {
        case MODBUS_READ_INPUT_REGISTERS:
        case MODBUS_READ_HOLDING_REGISTERS:
        {
            return MODBUS_PAYLOAD_START_OFFSET + MODBUS_READ_MULTIPLE_REGISTERS_PAYLOAD_SIZE(first_payload_byte) +
                   MODBUS_CRC_SIZE;
        }
        case MODBUS_PRESET_SINGLE_REGISTER:
        case MODBUS_PRESET_MULTIPLE_REGS:
        {
            return MODBUS_PAYLOAD_START_OFFSET + MODBUS_READ_SINGLE_REGISTER_PAYLOAD_SIZE + MODBUS_CRC_SIZE;
        }
        case MODBUS_ERROR_FIRST_ID ... MODBUS_ERROR_LAST_ID:
        {
            return MODBUS_PAYLOAD_START_OFFSET + MODBUS_EXCEPTION_PAYLOAD_SIZE + MODBUS_CRC_SIZE;
        }
        default:
        {
            /// Not supported opcode, should not get there

            return 0;
        }
    }
}

static bool MODBUS_IsFunctionCodeSupported(uint8_t function_code)
{
    switch (function_code)
    {
        case MODBUS_READ_HOLDING_REGISTERS:
        case MODBUS_READ_INPUT_REGISTERS:
        case MODBUS_PRESET_SINGLE_REGISTER:
        case MODBUS_PRESET_MULTIPLE_REGS:
        case MODBUS_ERROR_FIRST_ID ... MODBUS_ERROR_LAST_ID:
        {
            return true;
        }
        default:
        {
            return false;
        }
    }
}

static void MODBUS_ProcessFrame(uint8_t *payload, size_t len)
{
    if (MODBUS_IsValidMessage(payload, len))
    {
        MODBUS_Frame_T frame;
        size_t         index = 0;

        frame.slave_address = payload[index++];
        frame.function_code = payload[index++];
        frame.len           = len - index;
        frame.p_payload     = payload + index;

        LOG_DEBUG("Received MODBUS frame");

        MODBUS_ProcessResponse(&frame);
    }
    else
    {
        LOG_DEBUG("Message not valid");
    }
}
