/*
Copyright © 2021 Silvair Sp. z o.o. All Rights Reserved.

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

#include "RTC.h"

#include "Log.h"
#include "MCU_Health.h"
#include "PCF8523.h"
#include "Timestamp.h"
#include "UARTProtocol.h"
#include "Utils.h"


#define BATTERY_MEASUREMENT_PERIOD_MS 60000
#define BATTERY_CURVE_STEP_PERCENT 10
#define VOLTAGE_DIVIDER_COEFFICIENT 2
#define ANALOG_MAX_READOUT 1023
#define ANALOG_REFERENCE_VOLTAGE_MV 3300

#define PCF8523_CURRENT_CONSUMPTION_NA 1200
#define CR1220_BATTER_CAPACITANCE_MAH 37
#define BATTERY_DISCHARGE_TIME_PER_PERCENT_IN_MINUTES \
    ((((CR1220_BATTER_CAPACITANCE_MAH * 1000000) / PCF8523_CURRENT_CONSUMPTION_NA) * 60) / 100)
#define BATTERY_LEVEL_LOW_PERCENT 30
#define BATTERY_LEVEL_CRITICAL_LOW_PERCENT 10
#define BATTERY_NOT_DETECTED_THRESHOLD_PERCENT 0

#define BATTERY_TIME_TO_DISCHARGE_UNKNOWN 0xFFFFFF
#define BATTERY_TIME_TO_CHARGE_UNKNOWN 0xFFFFFF

#define BATTERY_FLAGS_PRESENCE_NOT_PRESENT (0b00 << 0)
#define BATTERY_FLAGS_PRESENCE_PRESENT_AND_REMOVABLE (0b01 << 0)
#define BATTERY_FLAGS_PRESENCE_PRESENT_AND_NON_REMOVABLE (0b10 << 0)
#define BATTERY_FLAGS_PRESENCE_UNKNOWN (0b11 << 0)

#define BATTERY_FLAGS_INDICATOR_CRITICALLY_LOW_LEVEL (0b00 << 2)
#define BATTERY_FLAGS_INDICATOR_LOW_LEVEL (0b01 << 2)
#define BATTERY_FLAGS_INDICATOR_GOOD_LEVEL (0b10 << 2)
#define BATTERY_FLAGS_INDICATOR_UNKNOWN (0b11 << 2)

#define BATTERY_FLAGS_CHARGING_IS_NOT_CHARGEABLE (0b00 << 4)
#define BATTERY_FLAGS_CHARGING_IS_CHARGEABLE_AND_IS_NOT_CHARGING (0b01 << 4)
#define BATTERY_FLAGS_CHARGING_IS_CHARGEABLE_AND_IS_CHARGING (0b10 << 4)
#define BATTERY_FLAGS_CHARGING_UNKNOWN (0b11 << 4)

#define BATTERY_FLAGS_SERVICEABILITY_RFU (0b00 << 6)
#define BATTERY_FLAGS_SERVICEABILITY_BATTERY_DOES_NOT_REQUIRE_SERVICE (0b01 << 6)
#define BATTERY_FLAGS_SERVICEABILITY_BATTERY_REQUIRES_SERVICE (0b10 << 6)
#define BATTERY_FLAGS_SERVICEABILITY_UNKNOWN (0b11 << 6)

#define HEALTH_FAULT_ID_BATTERY_LOW_WARNING 0x01
#define HEALTH_FAULT_ID_BATTERY_LOW_ERROR 0x02
#define HEALTH_FAULT_ID_RTC_ERROR 0xA1


static PCF8523 *                     pSelf           = NULL;
volatile static bool                 ReceivedTimeGet = false;
static SendTimeSourceGetRespCallback TimeSourceGetRespCallback;
static SendTimeSourceSetRespCallback TimeSourceSetRespCallback;
static uint8_t                       LastBatteryLevelPercent    = 0;
static bool                          IsBatteryDetected          = false;
static bool                          IsBatteryLevelEverMeasured = false;

static const uint16_t cr1220_battery_curve_mv[] = {
    0,    /* 0 % of battery capacity */
    2600, /* 10 % of battery capacity */
    2750, /* 20 % of battery capacity */
    2810, /* 30 % of battery capacity */
    2860, /* 40 % of battery capacity */
    2900, /* 50 % of battery capacity */
    2900, /* 60 % of battery capacity */
    2900, /* 70 % of battery capacity */
    2900, /* 80 % of battery capacity */
    2900, /* 90 % of battery capacity */
    2900, /* 100 % of battery capacity */
};

static uint8_t TimeServerInstanceIdx = INSTANCE_INDEX_UNKNOWN;

static struct
{
    uint32_t  end_time;
    DateTime *set_time;
} TimeSetParams;

static void OnSecondElapsed(void);
static void ConfigureIntEverySecondAndInternalCapacitors(void);
static void ConfigureBatterySwitchOver(void);
static void MeasureBatteryLevel(void);
static void UpdateBatteryStatus(void);
static void UpdateHealthFaultStatus(void);


bool RTC_Init(SendTimeSourceGetRespCallback get_resp_callback, SendTimeSourceSetRespCallback set_resp_callback)
{
    if (pSelf != NULL)
    {
        return true;
    }

    pSelf = new PCF8523;
    pSelf->begin();
    pSelf->rtcStart();

    if (pSelf->rtcReadReg(PCF8523_TMR_B_FREQ_CTRL) == __UINT8_MAX__)
    {
        LOG_INFO("RTC is not connected");
        delete pSelf;
        pSelf = NULL;
        return false;
    }

    ConfigureIntEverySecondAndInternalCapacitors();
    ConfigureBatterySwitchOver();
    TimeSourceGetRespCallback = get_resp_callback;
    TimeSourceSetRespCallback = set_resp_callback;
    pinMode(PIN_RTC_INT1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_RTC_INT1), OnSecondElapsed, FALLING);

    return true;
}

void RTC_SetTime(TimeDate *time)
{
    if ((TimeServerInstanceIdx == INSTANCE_INDEX_UNKNOWN) || (pSelf == NULL))
    {
        return;
    }

    if (time->milliseconds != 0)
    {
        TimeSetParams.end_time = Timestamp_GetDelayed(Timestamp_GetCurrent(), (1000 - time->milliseconds));
        DateTime current_time  = DateTime(time->year, time->month, time->day, time->hour, time->minute, time->seconds);
        TimeSetParams.set_time = new DateTime(current_time + TimeSpan(1));    // next second
    }
    else
    {
        pSelf->setTime(DateTime(time->year, time->month, time->day, time->hour, time->minute, time->seconds));
        TimeSourceSetRespCallback(TimeServerInstanceIdx);
    }
}

void RTC_GetTime(void)
{
    if ((TimeServerInstanceIdx == INSTANCE_INDEX_UNKNOWN) || (pSelf == NULL))
    {
        return;
    }

    ReceivedTimeGet = true;
}

bool RTC_IsBatteryDetected(void)
{
    MeasureBatteryLevel();

    if (IsBatteryDetected)
    {
        LOG_INFO("Battery detected");
    }
    else
    {
        LOG_INFO("Battery not detected");
    }

    return IsBatteryDetected;
}

void SetTimeServerInstanceIdx(uint8_t instance_index)
{
    TimeServerInstanceIdx = instance_index;
}

uint8_t GetTimeServerInstanceIdx(void)
{
    return TimeServerInstanceIdx;
}

void LoopRTC(void)
{
    if ((TimeServerInstanceIdx == INSTANCE_INDEX_UNKNOWN) || (pSelf == NULL))
    {
        return;
    }

    MeasureBatteryLevel();

    if (TimeSetParams.set_time == NULL)
    {
        return;
    }

    if (Timestamp_Compare(TimeSetParams.end_time, Timestamp_GetCurrent()))
    {
        pSelf->setTime(*TimeSetParams.set_time);
        TimeSourceSetRespCallback(TimeServerInstanceIdx);

        delete (TimeSetParams.set_time);
        TimeSetParams.set_time = NULL;
    }
}

static void OnSecondElapsed(void)
{
    if ((TimeServerInstanceIdx == INSTANCE_INDEX_UNKNOWN) || (pSelf == NULL))
    {
        return;
    }

    if (ReceivedTimeGet)
    {
        DateTime        cur_time = pSelf->readTime();
        struct TimeDate date;
        date.year         = cur_time.year();
        date.month        = cur_time.month();
        date.day          = cur_time.day();
        date.hour         = cur_time.hour();
        date.minute       = cur_time.minute();
        date.seconds      = cur_time.second();
        date.milliseconds = 0;

        if (date.month > 12)
        {
            // In case of connection error with RTC the library returns the month equal to 165.
            // All the other data is also invalid
            LOG_INFO("RTC connection error");
            MCU_Health_SendSetFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_RTC_ERROR, TimeServerInstanceIdx);
            return;
        }

        MCU_Health_SendClearFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_RTC_ERROR, TimeServerInstanceIdx);
        TimeSourceGetRespCallback(TimeServerInstanceIdx, &date);
        ReceivedTimeGet = false;
    }
}

static void ConfigureIntEverySecondAndInternalCapacitors(void)
{
    if (pSelf == NULL)
    {
        return;
    }

    pSelf->rtcWriteReg(PCF8523_TMR_CLKOUT_CTRL, (1 << PCF8523_TMR_CLKOUT_CTRL_TAM_BIT) | RTC_CLKOUT_DISABLED);
    pSelf->rtcWriteReg(PCF8523_CONTROL_1, (1 << PCF8523_CONTROL_1_SIE_BIT) | (1 << PCF8523_CONTROL_1_CAP_SEL_BIT));
}

static void ConfigureBatterySwitchOver(void)
{
    if (pSelf == NULL)
    {
        return;
    }

    pSelf->rtcWriteReg(PCF8523_CONTROL_3, 0x00);
}

static void MeasureBatteryLevel(void)
{
    static unsigned long last_measurement_timestamp = 0;

    if ((IsBatteryLevelEverMeasured == true) && (IsBatteryDetected == false))
    {
        return;
    }

    if ((Timestamp_GetTimeElapsed(last_measurement_timestamp, Timestamp_GetCurrent()) >
         BATTERY_MEASUREMENT_PERIOD_MS) ||
        (last_measurement_timestamp == 0))
    {
        uint16_t adc_readout        = analogRead(PIN_RTC_BATTERY);
        uint16_t battery_voltage_mv = (adc_readout * VOLTAGE_DIVIDER_COEFFICIENT * ANALOG_REFERENCE_VOLTAGE_MV) /
                                      ANALOG_MAX_READOUT;

        LastBatteryLevelPercent = 100;
        for (size_t i = 0; i < ARRAY_SIZE(cr1220_battery_curve_mv) - 1; i++)
        {
            if (battery_voltage_mv < cr1220_battery_curve_mv[i])
            {
                LastBatteryLevelPercent = (i - 1) * BATTERY_CURVE_STEP_PERCENT;
                break;
            }
        }

        if (IsBatteryDetected)
        {
            UpdateBatteryStatus();
            UpdateHealthFaultStatus();
        }
        LOG_INFO("RTC battery voltage: %d mV (%d%%)", battery_voltage_mv, LastBatteryLevelPercent);

        last_measurement_timestamp = millis();

        if (!IsBatteryLevelEverMeasured && (LastBatteryLevelPercent > BATTERY_NOT_DETECTED_THRESHOLD_PERCENT))
        {
            IsBatteryDetected = true;
        }
        IsBatteryLevelEverMeasured = true;
    }
}

static void UpdateBatteryStatus(void)
{
    uint32_t time_to_discharge_minutes = LastBatteryLevelPercent * BATTERY_DISCHARGE_TIME_PER_PERCENT_IN_MINUTES;

    uint8_t battery_flags = BATTERY_FLAGS_PRESENCE_PRESENT_AND_REMOVABLE | BATTERY_FLAGS_CHARGING_IS_NOT_CHARGEABLE;
    if (LastBatteryLevelPercent <= BATTERY_LEVEL_CRITICAL_LOW_PERCENT)
    {
        battery_flags |= BATTERY_FLAGS_INDICATOR_CRITICALLY_LOW_LEVEL;
        battery_flags |= BATTERY_FLAGS_SERVICEABILITY_BATTERY_REQUIRES_SERVICE;
    }
    else if (LastBatteryLevelPercent <= BATTERY_LEVEL_LOW_PERCENT)
    {
        battery_flags |= BATTERY_FLAGS_INDICATOR_LOW_LEVEL;
        battery_flags |= BATTERY_FLAGS_SERVICEABILITY_BATTERY_REQUIRES_SERVICE;
    }
    else
    {
        battery_flags |= BATTERY_FLAGS_INDICATOR_GOOD_LEVEL;
        battery_flags |= BATTERY_FLAGS_SERVICEABILITY_BATTERY_DOES_NOT_REQUIRE_SERVICE;
    }

    uint8_t payload[] = {
        TimeServerInstanceIdx,
        LastBatteryLevelPercent,
        ((uint8_t)(time_to_discharge_minutes & 0xFF)),
        ((uint8_t)((time_to_discharge_minutes >> 8) & 0xFF)),
        ((uint8_t)((time_to_discharge_minutes >> 16) & 0xFF)),
        (BATTERY_TIME_TO_CHARGE_UNKNOWN & 0xFF),
        ((BATTERY_TIME_TO_CHARGE_UNKNOWN >> 8) & 0xFF),
        ((BATTERY_TIME_TO_CHARGE_UNKNOWN >> 16) & 0xFF),
        battery_flags,
    };

    UART_SendBatteryStatusSetRequest(payload, sizeof(payload));
}

static void UpdateHealthFaultStatus(void)
{
    if (LastBatteryLevelPercent <= BATTERY_LEVEL_CRITICAL_LOW_PERCENT)
    {
        MCU_Health_SendSetFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_BATTERY_LOW_WARNING, TimeServerInstanceIdx);
        MCU_Health_SendSetFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_BATTERY_LOW_ERROR, TimeServerInstanceIdx);
    }
    else if (LastBatteryLevelPercent <= BATTERY_LEVEL_LOW_PERCENT)
    {
        MCU_Health_SendSetFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_BATTERY_LOW_WARNING, TimeServerInstanceIdx);
        MCU_Health_SendClearFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_BATTERY_LOW_ERROR, TimeServerInstanceIdx);
    }
    else
    {
        MCU_Health_SendClearFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_BATTERY_LOW_WARNING, TimeServerInstanceIdx);
        MCU_Health_SendClearFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_BATTERY_LOW_ERROR, TimeServerInstanceIdx);
    }
}
