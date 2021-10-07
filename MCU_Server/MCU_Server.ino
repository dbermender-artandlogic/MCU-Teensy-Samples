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


#include <math.h>
#include <string.h>

#include "LightElTestSrv.h"
#include "Log.h"
#include "MCU_Attention.h"
#include "MCU_DFU.h"
#include "MCU_Definitions.h"
#include "MCU_Health.h"
#include "MCU_Lightness.h"
#include "Mesh.h"
#include "MeshTime.h"
#include "RTC.h"
#include "SDM.h"
#include "SensorInput.h"
#include "UARTProtocol.h"

#define RTC_TIME_ACCURACY_PPB 10000 /**<  Accuracy of PCF8253 RTC */

static const uint8_t ctl_registration[] = {
    lowByte(MESH_MODEL_ID_LIGHT_CTL_SERVER),
    highByte(MESH_MODEL_ID_LIGHT_CTL_SERVER),

    lowByte(LIGHT_CTL_TEMP_RANGE_MIN),
    highByte(LIGHT_CTL_TEMP_RANGE_MIN),
    lowByte(LIGHT_CTL_TEMP_RANGE_MAX),
    highByte(LIGHT_CTL_TEMP_RANGE_MAX),
};

static const uint8_t time_with_battery_registration[] = {
    lowByte(MESH_MODEL_ID_TIME_SERVER),
    highByte(MESH_MODEL_ID_TIME_SERVER),
    RTC_WITH_BATTERY_ATTACHED,
    lowByte(RTC_TIME_ACCURACY_PPB),
    highByte(RTC_TIME_ACCURACY_PPB),
};

static const uint8_t time_without_battery_registration[] = {
    lowByte(MESH_MODEL_ID_TIME_SERVER),
    highByte(MESH_MODEL_ID_TIME_SERVER),
    RTC_WITHOUT_BATTERY_ATTACHED,
    lowByte(RTC_TIME_ACCURACY_PPB),
    highByte(RTC_TIME_ACCURACY_PPB),
};

static const uint8_t time_without_rtc_registration[] = {
    lowByte(MESH_MODEL_ID_TIME_SERVER),
    highByte(MESH_MODEL_ID_TIME_SERVER),
    RTC_NOT_ATTACHED,
    0x00,
    0x00,
};

static const uint8_t light_el_server_registration[] = {
    lowByte(MESH_MODEL_ID_LIGHT_EL_SERVER),
    highByte(MESH_MODEL_ID_LIGHT_EL_SERVER),
    lowByte(LIGHT_EL_TEST_SRV_EMERGENCY_MANUFACTURER_MIN_LIGHTNESS_DC),
    highByte(LIGHT_EL_TEST_SRV_EMERGENCY_MANUFACTURER_MIN_LIGHTNESS_DC),
    lowByte(LIGHT_EL_TEST_SRV_EMERGENCY_MANUFACTURER_MAX_LIGHTNESS_DC),
    highByte(LIGHT_EL_TEST_SRV_EMERGENCY_MANUFACTURER_MAX_LIGHTNESS_DC),
};

static const uint8_t lightness_registration[] = {
    lowByte(MESH_MODEL_ID_LIGHT_LC_SERVER),
    highByte(MESH_MODEL_ID_LIGHT_LC_SERVER),
};

static const uint8_t pir_registration[] = {
    lowByte(MESH_MODEL_ID_SENSOR_SERVER),
    highByte(MESH_MODEL_ID_SENSOR_SERVER),

    0x01,    //Number of sensors

    lowByte(MESH_PROP_ID_PRESENCE_DETECTED),
    highByte(MESH_PROP_ID_PRESENCE_DETECTED),
    lowByte(PIR_POSITIVE_TOLERANCE),
    highByte(PIR_POSITIVE_TOLERANCE),
    lowByte(PIR_NEGATIVE_TOLERANCE),
    highByte(PIR_NEGATIVE_TOLERANCE),
    PIR_SAMPLING_FUNCTION,
    PIR_MEASUREMENT_PERIOD,
    PIR_UPDATE_INTERVAL,
};

static const uint8_t als_registration[] = {
    lowByte(MESH_MODEL_ID_SENSOR_SERVER),
    highByte(MESH_MODEL_ID_SENSOR_SERVER),

    0x01,    //Number of sensors

    lowByte(MESH_PROP_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
    highByte(MESH_PROP_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
    lowByte(ALS_POSITIVE_TOLERANCE),
    highByte(ALS_POSITIVE_TOLERANCE),
    lowByte(ALS_NEGATIVE_TOLERANCE),
    highByte(ALS_NEGATIVE_TOLERANCE),
    ALS_SAMPLING_FUNCTION,
    ALS_MEASUREMENT_PERIOD,
    ALS_UPDATE_INTERVAL,
};

static const uint8_t current_precise_energy_registration[] = {
    lowByte(MESH_MODEL_ID_SENSOR_SERVER),
    highByte(MESH_MODEL_ID_SENSOR_SERVER),

    0x02,    //Number of sensors

    lowByte(MESH_PROP_ID_PRESENT_INPUT_CURRENT),
    highByte(MESH_PROP_ID_PRESENT_INPUT_CURRENT),
    lowByte(CURRENT_SENSOR_POSITIVE_TOLERANCE),
    highByte(CURRENT_SENSOR_POSITIVE_TOLERANCE),
    lowByte(CURRENT_SENSOR_NEGATIVE_TOLERANCE),
    highByte(CURRENT_SENSOR_NEGATIVE_TOLERANCE),
    CURRENT_SENSOR_SAMPLING_FUNCTION,
    CURRENT_SENSOR_MEASUREMENT_PERIOD,
    CURRENT_SENSOR_UPDATE_INTERVAL,

    lowByte(MESH_PROP_ID_PRECISE_TOTAL_DEVICE_ENERGY_USE),
    highByte(MESH_PROP_ID_PRECISE_TOTAL_DEVICE_ENERGY_USE),
    lowByte(ENERGY_SENSOR_POSITIVE_TOLERANCE),
    highByte(ENERGY_SENSOR_POSITIVE_TOLERANCE),
    lowByte(ENERGY_SENSOR_NEGATIVE_TOLERANCE),
    highByte(ENERGY_SENSOR_NEGATIVE_TOLERANCE),
    ENERGY_SENSOR_SAMPLING_FUNCTION,
    ENERGY_SENSOR_MEASUREMENT_PERIOD,
    ENERGY_SENSOR_UPDATE_INTERVAL,
};

static const uint8_t voltage_power_registration[] = {
    lowByte(MESH_MODEL_ID_SENSOR_SERVER),
    highByte(MESH_MODEL_ID_SENSOR_SERVER),

    0x02,    //Number of sensors

    lowByte(MESH_PROP_ID_PRESENT_INPUT_VOLTAGE),
    highByte(MESH_PROP_ID_PRESENT_INPUT_VOLTAGE),
    lowByte(VOLTAGE_SENSOR_POSITIVE_TOLERANCE),
    highByte(VOLTAGE_SENSOR_POSITIVE_TOLERANCE),
    lowByte(VOLTAGE_SENSOR_NEGATIVE_TOLERANCE),
    highByte(VOLTAGE_SENSOR_NEGATIVE_TOLERANCE),
    VOLTAGE_SENSOR_SAMPLING_FUNCTION,
    VOLTAGE_SENSOR_MEASUREMENT_PERIOD,
    VOLTAGE_SENSOR_UPDATE_INTERVAL,

    lowByte(MESH_PROP_ID_PRESENT_DEVICE_INPUT_POWER),
    highByte(MESH_PROP_ID_PRESENT_DEVICE_INPUT_POWER),
    lowByte(POWER_SENSOR_POSITIVE_TOLERANCE),
    highByte(POWER_SENSOR_POSITIVE_TOLERANCE),
    lowByte(POWER_SENSOR_NEGATIVE_TOLERANCE),
    highByte(POWER_SENSOR_NEGATIVE_TOLERANCE),
    POWER_SENSOR_SAMPLING_FUNCTION,
    POWER_SENSOR_MEASUREMENT_PERIOD,
    POWER_SENSOR_UPDATE_INTERVAL,
};

static const uint8_t health_registration[] = {
    lowByte(MESH_MODEL_ID_HEALTH_SERVER),
    highByte(MESH_MODEL_ID_HEALTH_SERVER),

    0x01,    //Number of company IDs

    lowByte(SILVAIR_ID),
    highByte(SILVAIR_ID),
};

static ModemState_t ModemState = MODEM_STATE_UNKNOWN;

#if ENABLE_CTL == 1 && ENABLE_LC == 1
#undef ENABLE_LC
#define ENABLE_LC 0
#endif

static bool       CTLEnabled    = (ENABLE_CTL != 0);
static bool       LCEnabled     = (ENABLE_LC != 0);
static bool       RtcEnabled    = false;
static const bool PIRALSEnabled = (ENABLE_PIRALS != 0);
static const bool ENERGYEnabled = (ENABLE_ENERGY != 0);
static const bool ELTestEnabled = (ENABLE_LC != 0);


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
 *  Process ALS update
 *
 *  @param value_clux  New ALS value in clux
 *  @param src_addr    Source address
 */
void ProcessPresentAmbientLightLevel(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process PIR update
 *
 *  @param value       New PIR value
 *  @param src_addr    Source address
 */
void ProcessPresenceDetected(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process Power value update
 *
 *  @param value       New Power value
 *  @param src_addr    Source address
 */
void ProcessPresentDeviceInputPower(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process Current value update
 *
 *  @param value       New Current value
 *  @param src_addr    Source address
 */
void ProcessPresentInputCurrent(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process Voltage value update
 *
 *  @param value       New Voltage value
 *  @param src_addr    Source address
 */
void ProcessPresentInputVoltage(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process Energy value update
 *
 *  @param value       New Energy value
 *  @param src_addr    Source address
 */
void ProcessTotalDeviceEnergyUse(uint16_t src_addr, SensorValue_T sensor_value);

/*
 *  Process Precise Energy value update
 *
 *  @param value       New Energy value
 *  @param src_addr    Source address
 */
void ProcessPreciseTotalDeviceEnergyUse(uint16_t src_addr, SensorValue_T sensor_value);

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
 *  Main Arduino setup
 */
void setup(void);

/*
 *  Main Arduino loop
 */
void loop(void);


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
    AttentionStateSet(false);

    if (!Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_LIGHT_CTL_SERVER) && CTLEnabled)
    {
        LOG_INFO("Modem does not support Light CTL Server");

        if (Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_LIGHT_LC_SERVER))
        {
            LOG_INFO("Disabling CTL activating LC");
            CTLEnabled = false;
            LCEnabled  = true;
        }
    }

    if (!Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_TIME_SERVER) && RtcEnabled)
    {
        LOG_INFO("Modem does not support Time Server");
        return;
    }

    if (!Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_SENSOR_SERVER) && (PIRALSEnabled || ENERGYEnabled))
    {
        LOG_INFO("Modem does not support Sensor Server");
        return;
    }

    if (!Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_SENSOR_SERVER) && (PIRALSEnabled || ENERGYEnabled))
    {
        LOG_INFO("Modem does not support Sensor Server");
        return;
    }

    size_t payload_len      = 0;
    bool   battery_detected = RTC_IsBatteryDetected();

    if (CTLEnabled)
    {
        payload_len += sizeof(ctl_registration);
    }
    else if (LCEnabled)
    {
        payload_len += sizeof(lightness_registration);
    }

    if (RtcEnabled && battery_detected)
    {
        payload_len += sizeof(time_with_battery_registration);
    }
    else if (RtcEnabled)
    {
        payload_len += sizeof(time_without_battery_registration);
    }
    else
    {
        payload_len += sizeof(time_without_rtc_registration);
    }

    if (ELTestEnabled)
    {
        payload_len += sizeof(light_el_server_registration);
    }

    if (PIRALSEnabled)
    {
        payload_len += sizeof(pir_registration) + sizeof(als_registration);
    }

    if (ENERGYEnabled)
    {
        payload_len += sizeof(current_precise_energy_registration) + sizeof(voltage_power_registration);
    }

    payload_len += sizeof(health_registration);

    uint8_t model_ids[payload_len];
    size_t  index = 0;

    if (CTLEnabled)
    {
        memcpy(model_ids + index, ctl_registration, sizeof(ctl_registration));
        index += sizeof(ctl_registration);
    }
    else if (LCEnabled)
    {
        memcpy(model_ids + index, lightness_registration, sizeof(lightness_registration));
        index += sizeof(lightness_registration);
    }

    if (RtcEnabled && battery_detected)
    {
        memcpy(model_ids + index, time_with_battery_registration, sizeof(time_with_battery_registration));
        index += sizeof(time_with_battery_registration);
    }
    else if (RtcEnabled)
    {
        memcpy(model_ids + index, time_without_battery_registration, sizeof(time_without_battery_registration));
        index += sizeof(time_without_battery_registration);
    }
    else
    {
        memcpy(model_ids + index, time_without_rtc_registration, sizeof(time_without_rtc_registration));
        index += sizeof(time_without_rtc_registration);
    }

    if (ELTestEnabled)
    {
        memcpy(model_ids + index, light_el_server_registration, sizeof(light_el_server_registration));
        index += sizeof(light_el_server_registration);
    }

    if (PIRALSEnabled)
    {
        memcpy(model_ids + index, pir_registration, sizeof(pir_registration));
        index += sizeof(pir_registration);
        memcpy(model_ids + index, als_registration, sizeof(als_registration));
        index += sizeof(als_registration);
    }

    if (ENERGYEnabled)
    {
        memcpy(model_ids + index, current_precise_energy_registration, sizeof(current_precise_energy_registration));
        index += sizeof(current_precise_energy_registration);
        memcpy(model_ids + index, voltage_power_registration, sizeof(voltage_power_registration));
        index += sizeof(voltage_power_registration);
    }

    memcpy(model_ids + index, health_registration, sizeof(health_registration));

    SendFirmwareVersionSetRequest();
    UART_SendCreateInstancesRequest(model_ids, sizeof(model_ids));
}

void ProcessEnterDevice(uint8_t *p_payload, uint8_t len)
{
    LOG_INFO("Device State");

    EnableStartupSequence();
    ModemState = MODEM_STATE_DEVICE;
}

void ProcessEnterInitNode(uint8_t *p_payload, uint8_t len)
{
    LOG_INFO("Init Node State");
    ModemState = MODEM_STATE_INIT_NODE;
    AttentionStateSet(false);

    SetLightnessServerIdx(INSTANCE_INDEX_UNKNOWN);
    SensorInput_SetPirIdx(INSTANCE_INDEX_UNKNOWN);
    SensorInput_SetAlsIdx(INSTANCE_INDEX_UNKNOWN);
    SetTimeServerInstanceIdx(INSTANCE_INDEX_UNKNOWN);
    SensorInput_SetCurrPreciseEnergyIdx(INSTANCE_INDEX_UNKNOWN);
    SensorInput_SetVoltPowIdx(INSTANCE_INDEX_UNKNOWN);

    uint8_t sensor_server_model_id_occurency = 0;

    for (size_t index = 0; index < len;)
    {
        uint16_t model_id = ((uint16_t)p_payload[index++]);
        model_id |= ((uint16_t)p_payload[index++] << 8);

        if (MESH_MODEL_ID_LIGHT_CTL_SERVER == model_id || MESH_MODEL_ID_LIGHT_LC_SERVER == model_id)
        {
            uint16_t current_model_id_instance_index = index / 2;
            SetLightnessServerIdx(current_model_id_instance_index);
            SetLightCTLSupport(model_id == MESH_MODEL_ID_LIGHT_CTL_SERVER);
        }

        if (MESH_MODEL_ID_TIME_SERVER == model_id)
        {
            uint16_t current_model_id_instance_index = index / 2;
            SetTimeServerInstanceIdx(current_model_id_instance_index);
        }

        if (model_id == MESH_MODEL_ID_SENSOR_SERVER)
        {
            uint16_t current_model_id_instance_index = index / 2;
            sensor_server_model_id_occurency++;

            if (sensor_server_model_id_occurency == PIR_REGISTRATION_ORDER && PIRALSEnabled)
            {
                SensorInput_SetPirIdx(current_model_id_instance_index);
            }
            else if (sensor_server_model_id_occurency == ALS_REGISTRATION_ORDER && PIRALSEnabled)
            {
                SensorInput_SetAlsIdx(current_model_id_instance_index);
            }
            else if (sensor_server_model_id_occurency == CURR_ENERGY_REGISTRATION_ORDER && ENERGYEnabled)
            {
                SensorInput_SetCurrPreciseEnergyIdx(current_model_id_instance_index);
            }
            else if (sensor_server_model_id_occurency == VOLT_POWER_REGISTRATION_ORDER && ENERGYEnabled)
            {
                SensorInput_SetVoltPowIdx(current_model_id_instance_index);
            }
        }

        if (model_id == MESH_MODEL_ID_HEALTH_SERVER)
        {
            uint16_t current_model_id_instance_index = index / 2;
            SetHealthSrvIdx(current_model_id_instance_index);
        }
    }

    if (GetLightnessServerIdx() == INSTANCE_INDEX_UNKNOWN && (LCEnabled || CTLEnabled))
    {
        ModemState = MODEM_STATE_UNKNOWN;
        LOG_INFO("Light CTL/LC Server model id not found in init node message");
        return;
    }

    if (SensorInput_GetPirIdx() == INSTANCE_INDEX_UNKNOWN && PIRALSEnabled)
    {
        ModemState = MODEM_STATE_UNKNOWN;
        LOG_INFO("Sensor server (PIR) model id not found in init node message");
        return;
    }

    if (SensorInput_GetAlsIdx() == INSTANCE_INDEX_UNKNOWN && PIRALSEnabled)
    {
        ModemState = MODEM_STATE_UNKNOWN;
        LOG_INFO("Sensor server (ALS) model id not found in init node message");
        return;
    }

    if (SensorInput_GetCurrPreciseEnergyIdx() == INSTANCE_INDEX_UNKNOWN && ENERGYEnabled)
    {
        ModemState = MODEM_STATE_UNKNOWN;
        LOG_INFO("Sensor server (Voltage Current) model id not found in init node message");
        return;
    }

    if (SensorInput_GetVoltPowIdx() == INSTANCE_INDEX_UNKNOWN && ENERGYEnabled)
    {
        ModemState = MODEM_STATE_UNKNOWN;
        LOG_INFO("Sensor server (Power Energy) model id not found in init node message");
        return;
    }

    if (GetHealthSrvIdx() == INSTANCE_INDEX_UNKNOWN)
    {
        ModemState = MODEM_STATE_UNKNOWN;
        LOG_INFO("Health Server model id not found in init node message");
        return;
    }

    SendFirmwareVersionSetRequest();
    UART_StartNodeRequest();
}

void ProcessEnterNode(uint8_t *p_payload, uint8_t len)
{
    LOG_INFO("Node State");
    ModemState = MODEM_STATE_NODE;

    SynchronizeLightness();
}

void ProcessMeshCommand(uint8_t *p_payload, uint8_t len)
{
    Mesh_ProcessMeshCommand(p_payload, len);
}

void ProcessMeshMessageRequest1(uint8_t *p_payload, uint8_t len)
{
    Mesh_ProcessMeshMessageRequest1(p_payload, len);
}

void ProcessError(uint8_t *p_payload, uint8_t len)
{
    LOG_INFO("Error %d", p_payload[0]);
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
}

void ProcessModemFirmwareVersion(uint8_t *p_payload, uint8_t len)
{
}

void ProcessPresentAmbientLightLevel(uint16_t src_addr, SensorValue_T sensor_value)
{
}

void ProcessPresenceDetected(uint16_t src_addr, SensorValue_T sensor_value)
{
}

void ProcessPresentDeviceInputPower(uint16_t src_addr, SensorValue_T sensor_value)
{
}

void ProcessPresentInputCurrent(uint16_t src_addr, SensorValue_T sensor_value)
{
}

void ProcessPresentInputVoltage(uint16_t src_addr, SensorValue_T sensor_value)
{
}

void ProcessTotalDeviceEnergyUse(uint16_t src_addr, SensorValue_T sensor_value)
{
}

void ProcessPreciseTotalDeviceEnergyUse(uint16_t src_addr, SensorValue_T sensor_value)
{
}

void setup(void)
{
    SetupDebug();
    LOG_INFO("Server Sample");
    SetupAttention();
    SetupHealth();

    if (LCEnabled || CTLEnabled)
        SetupLightnessServer();
    if (PIRALSEnabled)
        SensorInput_Setup();
    if (ENERGYEnabled)
        SetupSDM();

    UART_Init();
    UART_SendSoftwareResetRequest();
    RtcEnabled = RTC_Init(UART_SendTimeSourceGetResponse, UART_SendTimeSourceSetResponse);

    SetupDFU();
}

void loop(void)
{
    UART_ProcessIncomingCommand();

    LoopHealth();
    if (!IsTestInProgress())
    {
        // Health and Attention shares Status LED
        LoopAttention();
    }

    LoopLightnessServer();
    LoopSDM();

    if (MODEM_STATE_NODE == ModemState)
    {
        SensorInput_Loop();
        LoopRTC();
    }

    if (ELTestEnabled)
    {
        LoopLightElTest();
    }
    LoopMeshTimeSync();
}
