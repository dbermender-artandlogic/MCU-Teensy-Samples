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


#include <limits.h>
#include <string.h>

#include "LCD.h"
#include "Log.h"
#include "MCU_Attention.h"
#include "MCU_DFU.h"
#include "MCU_Definitions.h"
#include "MCU_Switch.h"
#include "Mesh.h"
#include "MeshTime.h"
#include "RTC.h"
#include "SensorOutput.h"
#include "UARTProtocol.h"


#define RTC_TIME_ACCURACY_PPB 10000 /**<  Accuracy of PCF8253 RTC */


static ModemState_t ModemState        = MODEM_STATE_UNKNOWN;
static bool         LastDfuInProgress = false;
static bool         RtcEnabled        = false;


/*
 *  Setup debug interface
 */
void SetupDebug(void);

/*
 *  Process Init Device Event command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessEnterInitDevice(uint8_t *p_payload, uint8_t len);

/*
 *  Process Create Instances Response command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessEnterDevice(uint8_t *p_payload, uint8_t len);

/*
 *  Process Init Node Event command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessEnterInitNode(uint8_t *p_payload, uint8_t len);

/*
 *  Process Start Node Response command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessEnterNode(uint8_t *p_payload, uint8_t len);

/*
 *  Process Mesh Message Request command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessMeshCommand(uint8_t *p_payload, uint8_t len);

/*
 *  Process Error command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessError(uint8_t *p_payload, uint8_t len);

/*
 *  Process Start Test Request command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessStartTest(uint8_t *p_payload, uint8_t len);

/*
 *  Process new target lightness
 *
 *  @param current             Current lightness value
 *  @param target              Target lightness value
 *  @param transition_time     Transition time
 */
void ProcessTargetLightness(uint16_t current, uint16_t target, uint32_t transition_time);

/*
 *  Process new target lightness temperature
 *
 *  @param current             Current lightness temperature value
 *  @param target              Target lightness temperature value
 *  @param transition_time     Transition time
 */
void ProcessTargetLightnessTemp(uint16_t current, uint16_t target, uint32_t transition_time);

/*
 *  Send Firmware Version Set request
 */
void SendFirmwareVersionSetRequest(void);

/*
 *  Process FactoryResetEvent
 */
void ProcessFactoryResetEvent(void);

/*
 *  Process Firmware Version set response
 */
void ProcessFirmwareVersionSetResponse(void);

/*
 *  Print current UART Modem state on LCD
 */
void PrintStateOnLCD(void);

/*
 *  Update everything on LCD
 */
void UpdateLCD(void);

/*
 *  Check if DFU state was changed.
 */
bool IsDfuStateChanged(void);

/*
 *  Main Arduino setup
 */
void setup();

/*
 *  Main Arduino loop
 */
void loop();


void SetupDebug(void)
{
    DEBUG_INTERFACE.begin(DEBUG_INTERFACE_BAUDRATE);
    // Waits for debug interface initialization.
    delay(1000);
}

void ProcessEnterInitDevice(uint8_t *p_payload, uint8_t len)
{
    LOG_INFO("Init Device State");
    ModemState = MODEM_STATE_INIT_DEVICE;
    LCD_UpdateModemState(ModemState);
    AttentionStateSet(false);

    SetInstanceIdxCtl(INSTANCE_INDEX_UNKNOWN);
    SetInstanceIdxLc(INSTANCE_INDEX_UNKNOWN);
    SensorOutput_SetInstanceIdx(INSTANCE_INDEX_UNKNOWN);
    SetTimeServerInstanceIdx(INSTANCE_INDEX_UNKNOWN);

    if (!Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_LIGHT_LC_CLIENT))
    {
        LOG_INFO("Modem does not support Light Lightness Controler Client");
        return;
    }

    if (!Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_SENSOR_CLIENT))
    {
        LOG_INFO("Modem does not support Sensor Client");
        return;
    }

    SendFirmwareVersionSetRequest();

    if (RtcEnabled && RTC_IsBatteryDetected())
    {
        uint8_t model_ids[] = {
            // Light Lightness controller client
            lowByte(MESH_MODEL_ID_LIGHT_LC_CLIENT),
            highByte(MESH_MODEL_ID_LIGHT_LC_CLIENT),

            // Time Server with RTC
            lowByte(MESH_MODEL_ID_TIME_SERVER),
            highByte(MESH_MODEL_ID_TIME_SERVER),
            RTC_WITH_BATTERY_ATTACHED,
            lowByte(RTC_TIME_ACCURACY_PPB),
            highByte(RTC_TIME_ACCURACY_PPB),

            // Light CTL client
            lowByte(MESH_MODEL_ID_LIGHT_CTL_CLIENT),
            highByte(MESH_MODEL_ID_LIGHT_CTL_CLIENT),
            // Sensor client
            lowByte(MESH_MODEL_ID_SENSOR_CLIENT),
            highByte(MESH_MODEL_ID_SENSOR_CLIENT),
        };

        UART_SendCreateInstancesRequest(model_ids, sizeof(model_ids));
    }
    else if (RtcEnabled)
    {
        uint8_t model_ids[] = {
            // Light Lightness controller client
            lowByte(MESH_MODEL_ID_LIGHT_LC_CLIENT),
            highByte(MESH_MODEL_ID_LIGHT_LC_CLIENT),

            // Time Server with RTC
            lowByte(MESH_MODEL_ID_TIME_SERVER),
            highByte(MESH_MODEL_ID_TIME_SERVER),
            RTC_WITHOUT_BATTERY_ATTACHED,
            lowByte(RTC_TIME_ACCURACY_PPB),
            highByte(RTC_TIME_ACCURACY_PPB),

            // Light CTL client
            lowByte(MESH_MODEL_ID_LIGHT_CTL_CLIENT),
            highByte(MESH_MODEL_ID_LIGHT_CTL_CLIENT),
            // Sensor client
            lowByte(MESH_MODEL_ID_SENSOR_CLIENT),
            highByte(MESH_MODEL_ID_SENSOR_CLIENT),
        };

        UART_SendCreateInstancesRequest(model_ids, sizeof(model_ids));
    }
    else
    {
        uint8_t model_ids[] = {
            // Light Lightness controller client
            lowByte(MESH_MODEL_ID_LIGHT_LC_CLIENT),
            highByte(MESH_MODEL_ID_LIGHT_LC_CLIENT),

            // Time Server without RTC
            lowByte(MESH_MODEL_ID_TIME_SERVER),
            highByte(MESH_MODEL_ID_TIME_SERVER),
            RTC_NOT_ATTACHED,
            0x00,    // Accuracy not used = 0x0000
            0x00,
            // Light CTL client
            lowByte(MESH_MODEL_ID_LIGHT_CTL_CLIENT),
            highByte(MESH_MODEL_ID_LIGHT_CTL_CLIENT),
            // Sensor client
            lowByte(MESH_MODEL_ID_SENSOR_CLIENT),
            highByte(MESH_MODEL_ID_SENSOR_CLIENT),
        };

        UART_SendCreateInstancesRequest(model_ids, sizeof(model_ids));
    }

    UART_ModemFirmwareVersionRequest();
}

void ProcessEnterDevice(uint8_t *p_payload, uint8_t len)
{
    LOG_INFO("Device State");
    ModemState = MODEM_STATE_DEVICE;
    LCD_UpdateModemState(ModemState);
}

void ProcessEnterInitNode(uint8_t *p_payload, uint8_t len)
{
    LOG_INFO("Init Node State");
    ModemState = MODEM_STATE_INIT_NODE;
    LCD_UpdateModemState(ModemState);
    AttentionStateSet(false);

    SetInstanceIdxCtl(INSTANCE_INDEX_UNKNOWN);
    SetInstanceIdxLc(INSTANCE_INDEX_UNKNOWN);
    SensorOutput_SetInstanceIdx(INSTANCE_INDEX_UNKNOWN);
    SetTimeServerInstanceIdx(INSTANCE_INDEX_UNKNOWN);

    for (size_t index = 0; index < len;)
    {
        uint16_t model_id = ((uint16_t)p_payload[index++]);
        model_id |= ((uint16_t)p_payload[index++] << 8);
        uint16_t current_model_id_instance_index = index / 2;

        if (MESH_MODEL_ID_LIGHT_LC_CLIENT == model_id)
        {
            SetInstanceIdxLc(current_model_id_instance_index);
        }

        if (MESH_MODEL_ID_TIME_SERVER == model_id)
        {
            SetTimeServerInstanceIdx(current_model_id_instance_index);
        }

        if (MESH_MODEL_ID_SENSOR_CLIENT == model_id)
        {
            SensorOutput_SetInstanceIdx(current_model_id_instance_index);
        }

        if (MESH_MODEL_ID_LIGHT_CTL_CLIENT == model_id)
        {
            SetInstanceIdxCtl(current_model_id_instance_index);
        }
    }

    if (GetInstanceIdxLc() == INSTANCE_INDEX_UNKNOWN)
    {
        ModemState = MODEM_STATE_UNKNOWN;
        LCD_UpdateModemState(ModemState);
        LOG_INFO("Light Lightness Controller Client model id not found in init node message");
        return;
    }

    if (GetInstanceIdxCtl() == INSTANCE_INDEX_UNKNOWN)
    {
        ModemState = MODEM_STATE_UNKNOWN;
        LCD_UpdateModemState(ModemState);
        LOG_INFO("Light CTL Client model id not found in init node message");
        return;
    }

    if (SensorOutput_GetInstanceIdx() == INSTANCE_INDEX_UNKNOWN)
    {
        ModemState = MODEM_STATE_UNKNOWN;
        LCD_UpdateModemState(ModemState);
        LOG_INFO("Sensor Client model id not found in init node message");
        return;
    }

    if ((GetTimeServerInstanceIdx() == INSTANCE_INDEX_UNKNOWN) && RtcEnabled)
    {
        ModemState = MODEM_STATE_UNKNOWN;
        LCD_UpdateModemState(ModemState);
        LOG_INFO("Time Server model id not found in init node message");
        return;
    }

    SendFirmwareVersionSetRequest();
    UART_StartNodeRequest();
    UART_ModemFirmwareVersionRequest();
}

void ProcessEnterNode(uint8_t *p_payload, uint8_t len)
{
    LOG_INFO("Node State");
    ModemState = MODEM_STATE_NODE;
    LCD_UpdateModemState(ModemState);
}

void ProcessMeshCommand(uint8_t *p_payload, uint8_t len)
{
    Mesh_ProcessMeshCommand(p_payload, len);
}

void ProcessError(uint8_t *p_payload, uint8_t len)
{
    LOG_INFO("Error %d", p_payload[0]);
}

void ProcessModemFirmwareVersion(uint8_t *p_payload, uint8_t len)
{
    LOG_INFO("Process Modem Firmware Version");
    LCD_UpdateModemFwVersion((char *)p_payload, len);
}

bool IsDfuStateChanged(void)
{
    return LastDfuInProgress ^ MCU_DFU_IsInProgress();
}

void SendFirmwareVersionSetRequest(void)
{
    const char *p_firmware_version = BUILD_NUMBER;

    UART_SendFirmwareVersionSetRequest((uint8_t *)p_firmware_version, strlen(p_firmware_version));
}

void ProcessFirmwareVersionSetResponse(void)
{
}

void ProcessFactoryResetEvent(void)
{
    LCD_EraseSensorsValues();
}

void ProcessStartTest(uint8_t *p_payload, uint8_t len)
{
}

void ProcessTargetLightness(uint16_t current, uint16_t target, uint32_t transition_time)
{
}

void ProcessTargetLightnessTemp(uint16_t current, uint16_t target, uint32_t transition_time)
{
}

void setup()
{
    SetupDebug();

    SetupAttention();
    LCD_Setup();

    SetupSwitch();
    SensorOutput_Setup();
    RtcEnabled = RTC_Init(UART_SendTimeSourceGetResponse, UART_SendTimeSourceSetResponse);

    UART_Init();
    UART_SendSoftwareResetRequest();

    SetupDFU();
}

void loop()
{
    Mesh_Loop();
    LCD_Loop();
    LoopAttention();
    UART_ProcessIncomingCommand();

    switch (ModemState)
    {
        case MODEM_STATE_UNKNOWN:
        case MODEM_STATE_INIT_DEVICE:
        case MODEM_STATE_INIT_NODE:
            break;

        case MODEM_STATE_DEVICE:
        case MODEM_STATE_NODE:

            if (IsDfuStateChanged())
            {
                LastDfuInProgress = MCU_DFU_IsInProgress();
                LCD_UpdateDfuState(LastDfuInProgress);
            }

            LoopRTC();
            LoopSwitch();
            LoopMeshTimeSync();
            break;
    }
}
