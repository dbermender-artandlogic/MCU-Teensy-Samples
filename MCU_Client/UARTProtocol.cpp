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

#include "UARTProtocol.h"

#include "Arduino.h"
#include "CRC.h"
#include "Log.h"
#include "MeshTime.h"
#include "UARTDriver.h"
#include "Utils.h"

/**< UART Command Codes definitions */
#define UART_CMD_PING_REQUEST 0x01u
#define UART_CMD_PONG_RESPONSE 0x02u
#define UART_CMD_INIT_DEVICE_EVENT 0x03u
#define UART_CMD_CREATE_INSTANCES_REQUEST 0x04u
#define UART_CMD_CREATE_INSTANCES_RESPONSE 0x05u
#define UART_CMD_INIT_NODE_EVENT 0x06u
#define UART_CMD_MESH_MESSAGE_REQUEST 0x07u
#define UART_CMD_START_NODE_REQUEST 0x09u
#define UART_CMD_START_NODE_RESPONSE 0x0Bu
#define UART_CMD_FACTORY_RESET_REQUEST 0x0Cu
#define UART_CMD_FACTORY_RESET_RESPONSE 0x0Du
#define UART_CMD_FACTORY_RESET_EVENT 0x0Eu
#define UART_CMD_MESH_MESSAGE_RESPONSE 0x0Fu
#define UART_CMD_CURRENT_STATE_REQUEST 0x10u
#define UART_CMD_CURRENT_STATE_RESPONSE 0x11u
#define UART_CMD_ERROR 0x12u
#define UART_CMD_MODEM_FIRMWARE_VERSION_REQUEST 0x13u
#define UART_CMD_MODEM_FIRMWARE_VERSION_RESPONSE 0x14u
#define UART_CMD_SENSOR_UPDATE_REQUEST 0x15u
#define UART_CMD_ATTENTION_EVENT 0x16u
#define UART_CMD_SOFTWARE_RESET_REQUEST 0x17u
#define UART_CMD_SOFTWARE_RESET_RESPONSE 0x18u
#define UART_CMD_SENSOR_UPDATE_RESPONSE 0x19u
#define UART_CMD_DEVICE_UUID_REQUEST 0x1Au
#define UART_CMD_DEVICE_UUID_RESPONSE 0x1Bu
#define UART_CMD_SET_FAULT_REQUEST 0x1Cu
#define UART_CMD_SET_FAULT_RESPONSE 0x1Du
#define UART_CMD_CLEAR_FAULT_REQUEST 0x1Eu
#define UART_CMD_CLEAR_FAULT_RESPONSE 0x1Fu
#define UART_CMD_START_TEST_REQ 0x20u
#define UART_CMD_START_TEST_RESP 0x21u
#define UART_CMD_TEST_FINISHED_REQ 0x22u
#define UART_CMD_TEST_FINISHED_RESP 0x23u
#define UART_CMD_FIRMWARE_VERSION_SET_REQ 0x24u
#define UART_CMD_FIRMWARE_VERSION_SET_RESP 0x25u
#define UART_CMD_BATTERY_STATUS_SET_REQ 0x26u
#define UART_CMD_BATTERY_STATUS_SET_RESP 0x27u
#define UART_CMD_MESH_MESSAGE_REQUEST_1 0x28u
#define UART_CMD_TIME_SOURCE_SET_REQ 0x29u
#define UART_CMD_TIME_SOURCE_SET_RESP 0x2Au
#define UART_CMD_TIME_SOURCE_GET_REQ 0x2Bu
#define UART_CMD_TIME_SOURCE_GET_RESP 0x2Cu
#define UART_CMD_TIME_GET_REQ 0x2Du
#define UART_CMD_TIME_GET_RESP 0x2Eu

#define UART_CMD_DFU_INIT_REQ 0x80u
#define UART_CMD_DFU_INIT_RESP 0x81u
#define UART_CMD_DFU_STATUS_REQ 0x82u
#define UART_CMD_DFU_STATUS_RESP 0x83u
#define UART_CMD_DFU_PAGE_CREATE_REQ 0x84u
#define UART_CMD_DFU_PAGE_CREATE_RESP 0x85u
#define UART_CMD_DFU_WRITE_DATA_EVENT 0x86u
#define UART_CMD_DFU_PAGE_STORE_REQ 0x87u
#define UART_CMD_DFU_PAGE_STORE_RESP 0x88u
#define UART_CMD_DFU_STATE_CHECK_REQ 0x89u
#define UART_CMD_DFU_STATE_CHECK_RESP 0x8Au
#define UART_CMD_DFU_CANCEL_REQ 0x8Bu
#define UART_CMD_DFU_CANCEL_RESP 0x8Cu

#define UART_CMD_DFU_OFFSET 0x80

/**< Preamble definition */
#define PREAMBLE_BYTE_1 0xAAu
#define PREAMBLE_BYTE_2 0x55u

/**< UART Message description */
#define HEADER_LEN 4u
#define CRC_LEN 2u
#define PACKET_LEN(len) (HEADER_LEN + len + CRC_LEN)
#define PREAMBLE_BYTE_1_OFFSET 0u
#define PREAMBLE_BYTE_2_OFFSET 1u
#define LEN_OFFSET 2u
#define CMD_OFFSET 3u
#define PAYLOAD_OFFSET 4u
#define CRC_BYTE_1_OFFSET(len) (PAYLOAD_OFFSET + (len))
#define CRC_BYTE_2_OFFSET(len) (PAYLOAD_OFFSET + (len) + 1)


typedef struct RxFrame_tag
{
    uint8_t len;
    uint8_t cmd;
    uint8_t p_payload[MAX_PAYLOAD_SIZE];
} RxFrame_t;

static bool UART_PingsEnabled = true; /**< If true, device will send and respond to pings. Default it should work */

/*
 *  Received data from UART
 *
 *  @param rx_frame    Pointer to frame to be filled with received data
 *  @return            True if frame's CRC is valid, false otherwise
 */
static bool ExtractFrameFromBuffer(RxFrame_t *rx_frame);

/*
 *  Send message over UART
 *
 *  @param len        Message length
 *  @param cmd        Message command
 *  @param p_payload  Message payload
 */
static void UARTInternal_Send(uint8_t len, uint8_t cmd, uint8_t *p_payload);

/*
 *  Print debug message
 *
 *  @param *dir    Direction description
 *  @param len     Command length
 *  @param cmd     Command code
 *  @param *buf    Pointer to message
 *  @param crc     CRC
 */
static void PrintDebug(const char *dir, uint8_t len, uint8_t cmd, uint8_t *buf, uint16_t crc);

/*
 *  Calc CRC16
 *
 *  @param len    Data length
 *  @param cmd    Command code
 *  @param *data  Pointer to data buffer
 */
static uint16_t UARTInternal_CalcCRC16(uint8_t len, uint8_t cmd, uint8_t *data);

void UART_Init(void)
{
    UARTDriver_Init();
}

void UART_EnablePings(void)
{
    LOG_INFO("Pings enabled");
    UART_PingsEnabled = true;
}

void UART_DisablePings(void)
{
    LOG_INFO("Pings disabled");
    UART_PingsEnabled = false;
}

void UART_SendPingRequest(void)
{
    if (UART_PingsEnabled)
    {
        UARTInternal_Send(0, UART_CMD_PING_REQUEST, NULL);
    }
}

void UART_SendPongResponse(uint8_t *p_payload, uint8_t len)
{
    if (UART_PingsEnabled)
    {
        UARTInternal_Send(len, UART_CMD_PONG_RESPONSE, p_payload);
    }
}

void UART_SendSoftwareResetRequest(void)
{
    UARTInternal_Send(0, UART_CMD_SOFTWARE_RESET_REQUEST, NULL);
}

void UART_SendCreateInstancesRequest(uint8_t *model_id, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_CREATE_INSTANCES_REQUEST, model_id);
}

void UART_SendMeshMessageRequest(uint8_t *p_payload, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_MESH_MESSAGE_REQUEST, p_payload);
}

void UART_SendMeshMessageRequest1(uint8_t *p_payload, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_MESH_MESSAGE_REQUEST_1, p_payload);
}

void UART_SendSensorUpdateRequest(uint8_t *p_payload, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_SENSOR_UPDATE_REQUEST, p_payload);
}

void UART_StartNodeRequest(void)
{
    UARTInternal_Send(0, UART_CMD_START_NODE_REQUEST, NULL);
}

void UART_ModemFirmwareVersionRequest(void)
{
    UARTInternal_Send(0, UART_CMD_MODEM_FIRMWARE_VERSION_REQUEST, NULL);
}

void UART_SendSetFaultRequest(uint8_t *p_payload, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_SET_FAULT_REQUEST, p_payload);
}

void UART_SendClearFaultRequest(uint8_t *p_payload, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_CLEAR_FAULT_REQUEST, p_payload);
}

void UART_SendTestStartResponse(uint8_t *p_payload, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_START_TEST_RESP, p_payload);
}

void UART_SendTestFinishedRequest(uint8_t *p_payload, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_TEST_FINISHED_REQ, p_payload);
}

void UART_SendDfuInitResponse(uint8_t *p_payload, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_DFU_INIT_RESP, p_payload);
}

void UART_SendDfuStatusResponse(uint8_t *p_payload, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_DFU_STATUS_RESP, p_payload);
}

void UART_SendDfuPageCreateResponse(uint8_t *p_payload, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_DFU_PAGE_CREATE_RESP, p_payload);
}

void UART_SendDfuPageStoreResponse(uint8_t *p_payload, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_DFU_PAGE_STORE_RESP, p_payload);
}

void UART_SendDfuStateCheckRequest(uint8_t *p_payload, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_DFU_STATE_CHECK_REQ, p_payload);
}

void UART_SendDfuCancelRequest(uint8_t *p_payload, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_DFU_CANCEL_REQ, p_payload);
}

void UART_SendFirmwareVersionSetRequest(uint8_t *p_payload, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_FIRMWARE_VERSION_SET_REQ, p_payload);
}

void UART_ProcessIncomingCommand(void)
{
    static RxFrame_t rx_frame;

    UARTDriver_RxDMAPoll();

    if (!ExtractFrameFromBuffer(&rx_frame))
    {
        return;
    }

    switch (rx_frame.cmd)
    {
        case UART_CMD_PING_REQUEST:
        {
            UART_SendPongResponse(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_INIT_DEVICE_EVENT:
        {
            ProcessEnterInitDevice(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_CREATE_INSTANCES_RESPONSE:
        {
            ProcessEnterDevice(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_INIT_NODE_EVENT:
        {
            ProcessEnterInitNode(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_START_NODE_RESPONSE:
        {
            ProcessEnterNode(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_MESH_MESSAGE_REQUEST:
        {
            ProcessMeshCommand(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_ATTENTION_EVENT:
        {
            ProcessAttention(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_ERROR:
        {
            ProcessError(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_MODEM_FIRMWARE_VERSION_RESPONSE:
        {
            ProcessModemFirmwareVersion(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_START_TEST_REQ:
        {
            ProcessStartTest(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_DFU_INIT_REQ:
        {
            ProcessDfuInitRequest(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_DFU_STATUS_REQ:
        {
            ProcessDfuStatusRequest(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_DFU_PAGE_CREATE_REQ:
        {
            ProcessDfuPageCreateRequest(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_DFU_WRITE_DATA_EVENT:
        {
            ProcessDfuWriteDataEvent(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_DFU_PAGE_STORE_REQ:
        {
            ProcessDfuPageStoreRequest(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_DFU_STATE_CHECK_RESP:
        {
            ProcessDfuStateCheckResponse(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_DFU_CANCEL_RESP:
        {
            ProcessDfuCancelResponse(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_FIRMWARE_VERSION_SET_RESP:
        {
            ProcessFirmwareVersionSetResponse();
            break;
        }
        case UART_CMD_FACTORY_RESET_EVENT:
        {
            ProcessFactoryResetEvent();
            break;
        }
        case UART_CMD_TIME_SOURCE_SET_REQ:
        {
            MeshTime_ProcessTimeSourceSetRequest(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_TIME_SOURCE_GET_REQ:
        {
            MeshTime_ProcessTimeSourceGetRequest(rx_frame.p_payload, rx_frame.len);
            break;
        }
        case UART_CMD_TIME_GET_RESP:
        {
            MeshTime_ProcessTimeGetResponse(rx_frame.p_payload, rx_frame.len);
            break;
        }
    }
}

static bool ExtractFrameFromBuffer(RxFrame_t *rx_frame)
{
    bool            isCRCValid = false;
    static uint16_t crc        = 0;
    static size_t   count      = 0;
    uint8_t         received_byte;

    if (!UARTDriver_ReadByte(&received_byte))
    {
        return isCRCValid;
    }

    if (count == PREAMBLE_BYTE_1_OFFSET)
    {
        if (received_byte == PREAMBLE_BYTE_1)
        {
            count++;
        }
        else
        {
            count = 0;
        }
    }
    else if (count == PREAMBLE_BYTE_2_OFFSET)
    {
        if (received_byte == PREAMBLE_BYTE_2)
        {
            count++;
        }
        else
        {
            count = 0;
        }
    }
    else if (count == LEN_OFFSET)
    {
        if (received_byte <= MAX_PAYLOAD_SIZE)
        {
            rx_frame->len = received_byte;
            count++;
        }
        else
        {
            count = 0;
        }
    }
    else if (count == CMD_OFFSET)
    {
        rx_frame->cmd = received_byte;
        count++;
    }
    else if ((CMD_OFFSET < count) && (count < CRC_BYTE_1_OFFSET(rx_frame->len)))
    {
        rx_frame->p_payload[count - PAYLOAD_OFFSET] = received_byte;
        count++;
    }
    else if (count == CRC_BYTE_1_OFFSET(rx_frame->len))
    {
        crc = received_byte;
        count++;
    }
    else if (count == CRC_BYTE_2_OFFSET(rx_frame->len))
    {
        crc += ((uint16_t)received_byte) << 8;
        isCRCValid = (crc == UARTInternal_CalcCRC16(rx_frame->len, rx_frame->cmd, rx_frame->p_payload));
        count      = 0;
    }

    if (isCRCValid)
    {
        PrintDebug("Received", rx_frame->len, rx_frame->cmd, rx_frame->p_payload, crc);
    }

    return isCRCValid;
}

void UART_SendTimeSourceGetResponse(uint8_t instance_idx, TimeDate *time)
{
    TimeSourceGetResp_T msg = {0};

    msg.instance_index = instance_idx;
    msg.date           = *time;

    UARTInternal_Send(sizeof(msg), UART_CMD_TIME_SOURCE_GET_RESP, (uint8_t *)&msg);
}

void UART_SendTimeSourceSetResponse(uint8_t instance_idx)
{
    TimeSourceSetResp_T msg = {0};

    msg.instance_index = instance_idx;

    UARTInternal_Send(sizeof(msg), UART_CMD_TIME_SOURCE_SET_RESP, (uint8_t *)&msg);
}

void UART_SendTimeGetRequest(uint8_t instance_idx)
{
    TimeGetReq_T msg = {0};

    msg.instance_index = instance_idx;

    UARTInternal_Send(sizeof(msg), UART_CMD_TIME_GET_REQ, (uint8_t *)&msg);
}

void UART_SendBatteryStatusSetRequest(uint8_t *p_payload, uint8_t len)
{
    UARTInternal_Send(len, UART_CMD_BATTERY_STATUS_SET_REQ, p_payload);
}


static void UARTInternal_Send(uint8_t len, uint8_t cmd, uint8_t *p_payload)
{
    uint16_t crc;
    uint8_t  msg[PACKET_LEN(len)];

    msg[PREAMBLE_BYTE_1_OFFSET] = PREAMBLE_BYTE_1;
    msg[PREAMBLE_BYTE_2_OFFSET] = PREAMBLE_BYTE_2;
    msg[LEN_OFFSET]             = len;
    msg[CMD_OFFSET]             = cmd;

    memcpy(&msg[PAYLOAD_OFFSET], p_payload, len);

    crc                         = UARTInternal_CalcCRC16(len, cmd, p_payload);
    msg[CRC_BYTE_1_OFFSET(len)] = lowByte(crc);
    msg[CRC_BYTE_2_OFFSET(len)] = highByte(crc);

    UARTDriver_WriteBytes(msg, PACKET_LEN(len));

    PrintDebug("Sent", len, cmd, p_payload, crc);
}

static void PrintDebug(const char *dir, uint8_t len, uint8_t cmd, uint8_t *buf, uint16_t crc)
{
#if LOG_DEBUG_ENABLE == 1
    const char *cmdName[] = {
        "Unknown",                      /* 0x00 */
        "PingRequest",                  /* 0x01 */
        "PongResponse",                 /* 0x02 */
        "InitDeviceEvent",              /* 0x03 */
        "CreateInstancesRequest",       /* 0x04 */
        "CreateInstancesResponse",      /* 0x05 */
        "InitNodeEvent",                /* 0x06 */
        "MeshMessageRequest",           /* 0x07 */
        "Unknown",                      /* 0x08 */
        "StartNodeRequest",             /* 0x09 */
        "Unknown",                      /* 0x0A */
        "StartNodeResponse",            /* 0x0B */
        "FactoryResetRequest",          /* 0x0C */
        "FactoryResetResponse",         /* 0x0D */
        "FactoryResetEvent",            /* 0x0E */
        "MeshMessageResponse",          /* 0x0F */
        "CurrentStateRequest",          /* 0x10 */
        "CurrentStateResponse",         /* 0x11 */
        "Error",                        /* 0x12 */
        "ModemFirmwareVersionRequest",  /* 0x13 */
        "ModemFirmwareVersionResponse", /* 0x14 */
        "SensorUpdateRequest",          /* 0x15 */
        "AttentionEvent",               /* 0x16 */
        "SoftwareResetRequest",         /* 0x17 */
        "SoftwareResetResponse",        /* 0x18 */
        "SensorUpdateResponse",         /* 0x19 */
        "DeviceUUIDRequest",            /* 0x1A */
        "DeviceUUIDResponse",           /* 0x1B */
        "SetFaultRequest",              /* 0x1C */
        "SetFaultResponse",             /* 0x1D */
        "ClearFaultRequest",            /* 0x1E */
        "ClearFaultResponse",           /* 0x1F */
        "StartTestRequest",             /* 0x20 */
        "StartTestResponse",            /* 0x21 */
        "TestFinishedRequest",          /* 0x22 */
        "TestFinishedResponse",         /* 0x23 */
        "FirmwareVersionSetRequest",    /* 0x24 */
        "FirmwareVersionSetResponse",   /* 0x25 */
        "BatteryStatusSetRequest",      /* 0x26 */
        "BatteryStatusSetResponse",     /* 0x27 */
        "MeshMessageRequest1",          /* 0x28 */
        "TimeSourceSetRequest",         /* 0x29 */
        "TimeSourceSetResponse",        /* 0x2A */
        "TimeSourceGetRequest",         /* 0x2B */
        "TimeSourceGetResponse",        /* 0x2C */
        "TimeGetRequest",               /* 0x2D */
        "TimeGetResponse",              /* 0x2E */
    };

    const char *dfuCmdName[] = {
        "DfuInitRequest",        /* 0x80 */
        "DfuInitResponse",       /* 0x81 */
        "DfuStatusRequest",      /* 0x82 */
        "DfuStatusResponse",     /* 0x83 */
        "DfuPageCreateRequest",  /* 0x84 */
        "DfuPageCreateResponse", /* 0x85 */
        "DfuWriteDataEvent",     /* 0x86 */
        "DfuPageStoreRequest",   /* 0x87 */
        "DfuPageStoreResponse",  /* 0x88 */
        "DfuStateCheckRequest",  /* 0x89 */
        "DfuStateCheckResponse", /* 0x8A */
        "DfuCancelRequest",      /* 0x8B */
        "DfuCancelResponse",     /* 0x8C */
    };

    const char unknown_command_name[] = "Unknown";

    const char *command_name;
    if (cmd < ARRAY_SIZE(cmdName))
    {
        command_name = cmdName[cmd];
    }
    else
    {
        if (cmd >= UART_CMD_DFU_OFFSET && cmd < UART_CMD_DFU_OFFSET + ARRAY_SIZE(dfuCmdName))
        {
            command_name = dfuCmdName[cmd - UART_CMD_DFU_OFFSET];
        }
        else
        {
            command_name = unknown_command_name;
        }
    }

    LOG_DEBUG("%s %s command", dir, command_name);
    LOG_DEBUG("\t Len: 0x%02X", len);
    LOG_DEBUG("\t Cmd: 0x%02X", cmd);
    LOG_DEBUG_HEXBUF("\t Data:", buf, len);
    LOG_DEBUG("\t CRC: 0x%02X%02X", lowByte(crc), highByte(crc));
#endif
}

static uint16_t UARTInternal_CalcCRC16(uint8_t len, uint8_t cmd, uint8_t *data)
{
    uint16_t crc = CRC16_INIT_VAL;
    crc          = CalcCRC16(&len, sizeof(len), crc);
    crc          = CalcCRC16(&cmd, sizeof(cmd), crc);
    crc          = CalcCRC16(data, len, crc);
    return crc;
}
