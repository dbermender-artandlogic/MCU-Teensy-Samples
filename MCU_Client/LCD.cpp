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


#include "LCD.h"

#include "Arduino.h"
#include "Config.h"
#include "LiquidCrystal_I2C.h"
#include "Log.h"
#include "MeshTime.h"
#include "TAILocalTimeConverter.h"
#include "Timestamp.h"
#include "UARTProtocol.h"

#define LCD_SCREEN_SWITCH_INTV_MS 5000 /**< Defines LCD screen switch interval. */
#define LCD_ROWS_NUMBER 4              /**< Defines number of LCD rows. */
#define LCD_COLUMNS_NUMBER 20          /**< Defines number of LCD columns. */

#define LCD_PIR_VALUE_EXP_MS 60000     /**< PIR Sensor value expiration time in milliseconds. */
#define LCD_ALS_VALUE_EXP_MS 60000     /**< ALS Sensor value expiration time in milliseconds. */
#define LCD_POWER_VALUE_EXP_MS 60000   /**< Power Sensor value expiration time in milliseconds. */
#define LCD_CURRENT_VALUE_EXP_MS 60000 /**< Current Sensor value expiration time in milliseconds. */
#define LCD_VOLTAGE_VALUE_EXP_MS 60000 /**< Voltage Sensor value expiration time in milliseconds. */
#define LCD_ENERGY_VALUE_EXP_MS 60000  /**< Energy Sensor value expiration time in milliseconds. */

#define LCD_DATE_AND_TIME_UPDATE_PERIOD_MS 1000 /**< Date and Time displayed time update in milliseconds. */


typedef enum
{
    SCREEN_TYPE_FIRST,
    SCREEN_TYPE_MODEM_STATE_PIR_ALS = SCREEN_TYPE_FIRST,
    SCREEN_TYPE_ENERGY_SENSORS,
    SCREEN_TYPE_FW_VERSION,
    SCREEN_TYPE_DFU,
    SCREEN_TYPE_DATE_AND_TIME,
    SCREEN_TYPE_LAST = SCREEN_TYPE_DATE_AND_TIME,
} ScreenType_T;


static LiquidCrystal_I2C Lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

static bool          LCD_DfuInProgress                      = false;
static ModemState_t  LCD_ModemState                         = MODEM_STATE_UNKNOWN;
static char          LCD_ModemFwVersion[LCD_COLUMNS_NUMBER] = "Unknown";
static ScreenType_T  LCD_CurrentScreen                      = SCREEN_TYPE_FIRST;
static unsigned long LCD_CurrentScreenTimestamp             = 0;
static bool          LCD_NeedsUpdate                        = false;

typedef enum
{
    SENSOR_VALUE_UNKNOWN,
    SENSOR_VALUE_ACTUAL,
    SENSOR_VALUE_EXPIRED,
} LCD_SensorValueState_T;

typedef struct
{
    SensorProperty_T       property;
    SensorValue_T          value;
    unsigned long          value_timestamp;
    unsigned long          value_expiration_time;
    LCD_SensorValueState_T value_state;
} LCD_Sensor_T;

static LCD_Sensor_T LCD_PirSensor = {
    .property = PRESENCE_DETECTED,
    .value =
        {
            .pir = 0,
        },
    .value_timestamp       = 0,
    .value_expiration_time = LCD_PIR_VALUE_EXP_MS,
    .value_state           = SENSOR_VALUE_UNKNOWN,
};

static LCD_Sensor_T LCD_AlsSensor = {
    .property = PRESENT_AMBIENT_LIGHT_LEVEL,
    .value =
        {
            .als = 0,
        },
    .value_timestamp       = 0,
    .value_expiration_time = LCD_ALS_VALUE_EXP_MS,
    .value_state           = SENSOR_VALUE_UNKNOWN,
};

static LCD_Sensor_T LCD_PowerSensor = {
    .property = PRESENT_DEVICE_INPUT_POWER,
    .value =
        {
            .power = 0,
        },
    .value_timestamp       = 0,
    .value_expiration_time = LCD_POWER_VALUE_EXP_MS,
    .value_state           = SENSOR_VALUE_UNKNOWN,
};

static LCD_Sensor_T LCD_CurrentSensor = {
    .property = PRESENT_INPUT_CURRENT,
    .value =
        {
            .current = 0,
        },
    .value_timestamp       = 0,
    .value_expiration_time = LCD_CURRENT_VALUE_EXP_MS,
    .value_state           = SENSOR_VALUE_UNKNOWN,
};

static LCD_Sensor_T LCD_VoltageSensor = {
    .property = PRESENT_INPUT_VOLTAGE,
    .value =
        {
            .voltage = 0,
        },
    .value_timestamp       = 0,
    .value_expiration_time = LCD_VOLTAGE_VALUE_EXP_MS,
    .value_state           = SENSOR_VALUE_UNKNOWN,
};

static LCD_Sensor_T LCD_EnergySensor = {
    .property = TOTAL_DEVICE_ENERGY_USE,
    .value =
        {
            .energy = 0,
        },
    .value_timestamp       = 0,
    .value_expiration_time = LCD_ENERGY_VALUE_EXP_MS,
    .value_state           = SENSOR_VALUE_UNKNOWN,
};

static LCD_Sensor_T LCD_PreciseEnergySensor = {
    .property = PRECISE_TOTAL_DEVICE_ENERGY_USE,
    .value =
        {
            .precise_energy = 0,
        },
    .value_timestamp       = 0,
    .value_expiration_time = LCD_ENERGY_VALUE_EXP_MS,
    .value_state           = SENSOR_VALUE_UNKNOWN,
};

/*
 *  Display line of text on LCD screen
 */
static void DisplayLine(size_t line, const char *text);

/*
 *  Display Modem State on LCD screen
 */
static void DisplayModemState(uint8_t lineNumber, ModemState_t modemState);

/*
 *  Display LCD screen
 */
static void DisplayScreen(uint8_t screenNum);

/*
 *  Switch to another Screen
 */
static void ScreenIterate(void);

/*
 *  Check if Sensor values expired
 */
static void CheckSensorValuesExpiration(void);

/*
 *  Check if Time Display needs to be updated
 */
static void CheckTimeDisplayNeedUpdate(void);


void LCD_Setup(void)
{
    Lcd.begin(LCD_COLUMNS_NUMBER, LCD_ROWS_NUMBER);

    DisplayScreen(LCD_CurrentScreen);
}


void LCD_UpdateModemState(ModemState_t modemState)
{
    if (LCD_ModemState != modemState && LCD_CurrentScreen == SCREEN_TYPE_MODEM_STATE_PIR_ALS)
        LCD_NeedsUpdate = true;

    LCD_ModemState = modemState;
}


void LCD_UpdateModemFwVersion(char *fwVersion, uint8_t fwVerLen)
{
    if (fwVerLen > LCD_COLUMNS_NUMBER)
        fwVerLen = LCD_COLUMNS_NUMBER;

    if (strncmp(LCD_ModemFwVersion, fwVersion, fwVerLen) && LCD_CurrentScreen == SCREEN_TYPE_FW_VERSION)
        LCD_NeedsUpdate = true;
    strncpy(LCD_ModemFwVersion, fwVersion, fwVerLen);
}


void LCD_UpdateSensorValue(SensorProperty_T sensorProperty, SensorValue_T sensorValue)
{
    switch (sensorProperty)
    {
        case PRESENCE_DETECTED:
        {
            if (LCD_PirSensor.value.pir != sensorValue.pir && LCD_CurrentScreen == SCREEN_TYPE_MODEM_STATE_PIR_ALS)
                LCD_NeedsUpdate = true;

            LCD_PirSensor.value.pir       = sensorValue.pir;
            LCD_PirSensor.value_state     = SENSOR_VALUE_ACTUAL;
            LCD_PirSensor.value_timestamp = Timestamp_GetCurrent();
            break;
        }
        case PRESENT_AMBIENT_LIGHT_LEVEL:
        {
            if (LCD_AlsSensor.value.als != sensorValue.als && LCD_CurrentScreen == SCREEN_TYPE_MODEM_STATE_PIR_ALS)
                LCD_NeedsUpdate = true;

            LCD_AlsSensor.value.als   = sensorValue.als;
            LCD_AlsSensor.value_state = (sensorValue.als == MESH_PROP_PRESENT_AMBIENT_LIGHT_LEVEL_UNKNOWN_VAL)
                                            ? SENSOR_VALUE_UNKNOWN
                                            : SENSOR_VALUE_ACTUAL;
            LCD_AlsSensor.value_timestamp = Timestamp_GetCurrent();
            break;
        }
        case PRESENT_DEVICE_INPUT_POWER:
        {
            if (LCD_PowerSensor.value.power != sensorValue.power && LCD_CurrentScreen == SCREEN_TYPE_ENERGY_SENSORS)
                LCD_NeedsUpdate = true;

            LCD_PowerSensor.value.power = sensorValue.power;
            LCD_PowerSensor.value_state = (sensorValue.power == MESH_PROP_PRESENT_DEVICE_INPUT_POWER_UNKNOWN_VAL)
                                              ? SENSOR_VALUE_UNKNOWN
                                              : SENSOR_VALUE_ACTUAL;
            LCD_PowerSensor.value_timestamp = Timestamp_GetCurrent();
            break;
        }
        case PRESENT_INPUT_CURRENT:
        {
            if (LCD_CurrentSensor.value.current != sensorValue.current &&
                LCD_CurrentScreen == SCREEN_TYPE_ENERGY_SENSORS)
                LCD_NeedsUpdate = true;

            LCD_CurrentSensor.value.current = sensorValue.current;
            LCD_CurrentSensor.value_state   = (sensorValue.current == MESH_PROP_PRESENT_INPUT_CURRENT_UNKNOWN_VAL)
                                                ? SENSOR_VALUE_UNKNOWN
                                                : SENSOR_VALUE_ACTUAL;
            LCD_CurrentSensor.value_timestamp = Timestamp_GetCurrent();
            break;
        }
        case PRESENT_INPUT_VOLTAGE:
        {
            if (LCD_VoltageSensor.value.voltage != sensorValue.voltage &&
                LCD_CurrentScreen == SCREEN_TYPE_ENERGY_SENSORS)
                LCD_NeedsUpdate = true;

            LCD_VoltageSensor.value.voltage = sensorValue.voltage;
            LCD_VoltageSensor.value_state   = (sensorValue.voltage == MESH_PROP_PRESENT_INPUT_VOLTAGE_UNKNOWN_VAL)
                                                ? SENSOR_VALUE_UNKNOWN
                                                : SENSOR_VALUE_ACTUAL;
            LCD_VoltageSensor.value_timestamp = Timestamp_GetCurrent();
            break;
        }
        case TOTAL_DEVICE_ENERGY_USE:
        {
            if (LCD_EnergySensor.value.energy != sensorValue.energy && LCD_CurrentScreen == SCREEN_TYPE_ENERGY_SENSORS)
                LCD_NeedsUpdate = true;

            LCD_EnergySensor.value.energy = sensorValue.energy;
            LCD_EnergySensor.value_state  = (sensorValue.energy == MESH_PROP_TOTAL_DEVICE_ENERGY_USE_UNKNOWN_VAL)
                                               ? SENSOR_VALUE_UNKNOWN
                                               : SENSOR_VALUE_ACTUAL;
            LCD_EnergySensor.value_timestamp = Timestamp_GetCurrent();
            break;
        }
        case PRECISE_TOTAL_DEVICE_ENERGY_USE:
        {
            if (LCD_PreciseEnergySensor.value.precise_energy != sensorValue.precise_energy &&
                LCD_CurrentScreen == SCREEN_TYPE_ENERGY_SENSORS)
                LCD_NeedsUpdate = true;

            LCD_PreciseEnergySensor.value.precise_energy = sensorValue.precise_energy;
            LCD_PreciseEnergySensor.value_state          = ((sensorValue.precise_energy ==
                                                    MESH_PROP_PRECISE_TOTAL_DEVICE_ENERGY_USE_UNKNOWN_VAL) ||
                                                   (sensorValue.precise_energy ==
                                                    MESH_PROP_PRECISE_TOTAL_DEVICE_ENERGY_USE_NOT_VALID_VAL))
                                                      ? SENSOR_VALUE_UNKNOWN
                                                      : SENSOR_VALUE_ACTUAL;
            LCD_PreciseEnergySensor.value_timestamp = Timestamp_GetCurrent();
            break;
        }
    }
}

void LCD_UpdateDfuState(bool dfuInProgress)
{
    LCD_DfuInProgress = dfuInProgress;
}


void LCD_Loop(void)
{
    bool switchScreen = (Timestamp_GetTimeElapsed(LCD_CurrentScreenTimestamp, Timestamp_GetCurrent()) >=
                         LCD_SCREEN_SWITCH_INTV_MS);
    CheckSensorValuesExpiration();
    CheckTimeDisplayNeedUpdate();

    if (switchScreen)
        ScreenIterate();
    if (switchScreen | LCD_NeedsUpdate)
        DisplayScreen(LCD_CurrentScreen);
}


void LCD_Reinit(void)
{
    Lcd.begin(LCD_COLUMNS_NUMBER, LCD_ROWS_NUMBER);
    LCD_NeedsUpdate = true;
}

void LCD_EraseSensorsValues(void)
{
    LCD_PirSensor.value_state     = SENSOR_VALUE_UNKNOWN;
    LCD_AlsSensor.value_state     = SENSOR_VALUE_UNKNOWN;
    LCD_PowerSensor.value_state   = SENSOR_VALUE_UNKNOWN;
    LCD_CurrentSensor.value_state = SENSOR_VALUE_UNKNOWN;
    LCD_VoltageSensor.value_state = SENSOR_VALUE_UNKNOWN;
    LCD_EnergySensor.value_state  = SENSOR_VALUE_UNKNOWN;

    LCD_NeedsUpdate = true;
}


static void DisplayLine(size_t line, const char *text)
{
    if(getToggleLCD()){
        LOG_INFO(text);
    }

    if (strlen(text) > LCD_COLUMNS_NUMBER)
    {
        LOG_INFO("Trying to write too long string on LCD: %s", text);
        return;
    }

    Lcd.setCursor(0, line);
    Lcd.print(text);
}


static void DisplayScreen(uint8_t screenNum)
{
    switch (screenNum)
    {
        case SCREEN_TYPE_DFU:
        {
            Lcd.clear();

            if (LCD_DfuInProgress)
            {
                DisplayLine(0, "DFU in progress");
            }

            break;
        }

        case SCREEN_TYPE_MODEM_STATE_PIR_ALS:
        {
            char text[LCD_COLUMNS_NUMBER] = {0};

            Lcd.clear();

            DisplayModemState(0, LCD_ModemState);

            strcpy(text, "ALS: ");
            if (LCD_AlsSensor.value_state != SENSOR_VALUE_UNKNOWN)
            {
                if (LCD_AlsSensor.value_state == SENSOR_VALUE_EXPIRED)
                    strcpy(text + strlen(text), "(");
                itoa(LCD_AlsSensor.value.als / 100, text + strlen(text), 10);
                strcpy(text + strlen(text), ".00");
                itoa(LCD_AlsSensor.value.als % 100,
                     text + strlen(text) - (LCD_AlsSensor.value.als % 100 < 10 ? 1 : 2),
                     10);
                strcpy(text + strlen(text), " lux");
                if (LCD_AlsSensor.value_state == SENSOR_VALUE_EXPIRED)
                    strcpy(text + strlen(text), ")");
            }
            else
            {
                strcpy(text + strlen(text), "Unknown");
            }
            DisplayLine(2, text);

            strcpy(text, "PIR: ");
            if (LCD_PirSensor.value_state != SENSOR_VALUE_UNKNOWN)
            {
                if (LCD_PirSensor.value_state == SENSOR_VALUE_EXPIRED)
                    strcpy(text + strlen(text), "(");
                LCD_PirSensor.value.pir ? strcpy(text + strlen(text), "True") : strcpy(text + strlen(text), "False");
                if (LCD_PirSensor.value_state == SENSOR_VALUE_EXPIRED)
                    strcpy(text + strlen(text), ")");
            }
            else
            {
                strcpy(text + strlen(text), "Unknown");
            }
            DisplayLine(3, text);

            break;
        }

        case SCREEN_TYPE_ENERGY_SENSORS:
        {
            char text[LCD_COLUMNS_NUMBER] = {0};

            Lcd.clear();

            strcpy(text, "Power:   ");
            if (LCD_PowerSensor.value_state != SENSOR_VALUE_UNKNOWN)
            {
                if (LCD_PowerSensor.value_state == SENSOR_VALUE_EXPIRED)
                    strcpy(text + strlen(text), "(");
                itoa(LCD_PowerSensor.value.power / 10, text + strlen(text), 10);
                strcpy(text + strlen(text), ".");
                itoa(LCD_PowerSensor.value.power % 10, text + strlen(text), 10);
                strcpy(text + strlen(text), " W");
                if (LCD_PowerSensor.value_state == SENSOR_VALUE_EXPIRED)
                    strcpy(text + strlen(text), ")");
            }
            else
            {
                strcpy(text + strlen(text), "Unknown");
            }
            DisplayLine(0, text);

            strcpy(text, "Energy:  ");
            if (LCD_EnergySensor.value_state != SENSOR_VALUE_UNKNOWN ||
                LCD_PreciseEnergySensor.value_state != SENSOR_VALUE_UNKNOWN)
            {
                if (Timestamp_Compare(LCD_EnergySensor.value_timestamp, LCD_PreciseEnergySensor.value_timestamp))
                {
                    if (LCD_PreciseEnergySensor.value_state == SENSOR_VALUE_EXPIRED)
                        strcpy(text + strlen(text), "(");
                    itoa(LCD_PreciseEnergySensor.value.precise_energy, text + strlen(text), 10);
                    strcpy(text + strlen(text), " Wh");
                    if (LCD_PreciseEnergySensor.value_state == SENSOR_VALUE_EXPIRED)
                        strcpy(text + strlen(text), ")");
                }
                else
                {
                    if (LCD_EnergySensor.value_state == SENSOR_VALUE_EXPIRED)
                        strcpy(text + strlen(text), "(");
                    itoa(LCD_EnergySensor.value.energy, text + strlen(text), 10);
                    strcpy(text + strlen(text), " kWh");
                    if (LCD_EnergySensor.value_state == SENSOR_VALUE_EXPIRED)
                        strcpy(text + strlen(text), ")");
                }
            }
            else
            {
                strcpy(text + strlen(text), "Unknown");
            }
            DisplayLine(1, text);

            strcpy(text, "Voltage: ");
            if (LCD_VoltageSensor.value_state != SENSOR_VALUE_UNKNOWN)
            {
                if (LCD_VoltageSensor.value_state == SENSOR_VALUE_EXPIRED)
                    strcpy(text + strlen(text), "(");
                itoa(LCD_VoltageSensor.value.voltage / 64, text + strlen(text), 10);
                strcpy(text + strlen(text), ".00");
                itoa(LCD_VoltageSensor.value.voltage % 64,
                     text + strlen(text) - (LCD_VoltageSensor.value.voltage % 64 < 10 ? 1 : 2),
                     10);
                strcpy(text + strlen(text), " V");
                if (LCD_VoltageSensor.value_state == SENSOR_VALUE_EXPIRED)
                    strcpy(text + strlen(text), ")");
            }
            else
            {
                strcpy(text + strlen(text), "Unknown");
            }
            DisplayLine(2, text);

            strcpy(text, "Current: ");
            if (LCD_CurrentSensor.value_state != SENSOR_VALUE_UNKNOWN)
            {
                if (LCD_CurrentSensor.value_state == SENSOR_VALUE_EXPIRED)
                    strcpy(text + strlen(text), "(");
                itoa(LCD_CurrentSensor.value.current / 100, text + strlen(text), 10);
                strcpy(text + strlen(text), ".00");
                itoa(LCD_CurrentSensor.value.current % 100,
                     text + strlen(text) - (LCD_CurrentSensor.value.current % 100 < 10 ? 1 : 2),
                     10);
                strcpy(text + strlen(text), " A");
                if (LCD_CurrentSensor.value_state == SENSOR_VALUE_EXPIRED)
                    strcpy(text + strlen(text), ")");
            }
            else
            {
                strcpy(text + strlen(text), "Unknown");
            }
            DisplayLine(3, text);

            break;
        }

        case SCREEN_TYPE_FW_VERSION:
        {
            Lcd.clear();

            DisplayLine(0, "Modem FW version");
            DisplayLine(1, LCD_ModemFwVersion);
            DisplayLine(2, "MCU FW version");
            DisplayLine(3, BUILD_NUMBER);

            break;
        }

        case SCREEN_TYPE_DATE_AND_TIME:
        {
            Lcd.clear();

            MeshTimeLastSync_T *last_time_sync = MeshTime_GetLastSyncTime();

            DisplayLine(0, "Date:");
            DisplayLine(2, "Time:");

            if (last_time_sync->tai_seconds == TIME_TAI_SECONDS_TIME_UNKNOWN)
            {
                DisplayLine(2, "NotFound");
                DisplayLine(1, "Unknown");
                DisplayLine(3, "Unknown");
            }
            else
            {
                uint64_t actual_tai_ms = last_time_sync->tai_seconds * 1000 +
                                         TIME_SUBSECONDS_TO_MS(last_time_sync->subsecond) +
                                         Timestamp_GetTimeElapsed(last_time_sync->local_sync_timestamp_ms,
                                                                  Timestamp_GetCurrent());

                int16_t time_zone_offset_minutes = TIME_ZONE_OFFSET_STATE_TO_MIN(
                    (int16_t)last_time_sync->time_zone_offset);
                int16_t          leap_seconds = TIME_TAI_UTC_DELTA_STATE_TO_SEC((int16_t)last_time_sync->tai_utc_delta);
                struct LocalTime local_time   = TAILocalTimeConverter_TAIToLocalTime(actual_tai_ms / 1000,
                                                                                   time_zone_offset_minutes,
                                                                                   leap_seconds);

                char date_str[16];
                char time_str[16];

                sprintf(date_str, "%04u-%02u-%02u", local_time.year, local_time.month + 1, local_time.day);
                sprintf(time_str, "%02u:%02u:%02u", local_time.hour, local_time.minutes, local_time.seconds);

                DisplayLine(1, date_str);
                DisplayLine(3, time_str);
            }

            break;
        }
    }

    LCD_NeedsUpdate = false;
}

static void DisplayModemState(uint8_t lineNumber, ModemState_t modemState)
{
    char const *modem_states[] = {
        "Init Device state",
        "Device state",
        "Init Node state",
        "Node state",
        "Unknown state",
    };

    DisplayLine(lineNumber, modem_states[modemState]);
}

static void ScreenIterate(void)
{
    LCD_CurrentScreen = (ScreenType_T)(LCD_CurrentScreen + 1);
    if (LCD_CurrentScreen == SCREEN_TYPE_DFU && !LCD_DfuInProgress)
        LCD_CurrentScreen = (ScreenType_T)(LCD_CurrentScreen + 1);
    if (LCD_CurrentScreen > SCREEN_TYPE_LAST)
        LCD_CurrentScreen = SCREEN_TYPE_FIRST;

    LCD_CurrentScreenTimestamp = Timestamp_GetCurrent();
}

static void CheckSensorValuesExpiration(void)
{
    if ((Timestamp_GetTimeElapsed(LCD_PirSensor.value_timestamp, Timestamp_GetCurrent()) >
         LCD_PirSensor.value_expiration_time) &&
        LCD_PirSensor.value_state == SENSOR_VALUE_ACTUAL)
    {
        LCD_PirSensor.value_state = SENSOR_VALUE_EXPIRED;
        LCD_NeedsUpdate           = true;
    }

    if ((Timestamp_GetTimeElapsed(LCD_AlsSensor.value_timestamp, Timestamp_GetCurrent()) >
         LCD_AlsSensor.value_expiration_time) &&
        LCD_AlsSensor.value_state == SENSOR_VALUE_ACTUAL)
    {
        LCD_AlsSensor.value_state = SENSOR_VALUE_EXPIRED;
        LCD_NeedsUpdate           = true;
    }

    if ((Timestamp_GetTimeElapsed(LCD_PowerSensor.value_timestamp, Timestamp_GetCurrent()) >
         LCD_PowerSensor.value_expiration_time) &&
        LCD_PowerSensor.value_state == SENSOR_VALUE_ACTUAL)
    {
        LCD_PowerSensor.value_state = SENSOR_VALUE_EXPIRED;
        LCD_NeedsUpdate             = true;
    }

    if ((Timestamp_GetTimeElapsed(LCD_CurrentSensor.value_timestamp, Timestamp_GetCurrent()) >
         LCD_CurrentSensor.value_expiration_time) &&
        LCD_CurrentSensor.value_state == SENSOR_VALUE_ACTUAL)
    {
        LCD_CurrentSensor.value_state = SENSOR_VALUE_EXPIRED;
        LCD_NeedsUpdate               = true;
    }

    if ((Timestamp_GetTimeElapsed(LCD_VoltageSensor.value_timestamp, Timestamp_GetCurrent()) >
         LCD_VoltageSensor.value_expiration_time) &&
        LCD_VoltageSensor.value_state == SENSOR_VALUE_ACTUAL)
    {
        LCD_VoltageSensor.value_state = SENSOR_VALUE_EXPIRED;
        LCD_NeedsUpdate               = true;
    }

    if ((Timestamp_GetTimeElapsed(LCD_EnergySensor.value_timestamp, Timestamp_GetCurrent()) >
         LCD_EnergySensor.value_expiration_time) &&
        LCD_EnergySensor.value_state == SENSOR_VALUE_ACTUAL)
    {
        LCD_EnergySensor.value_state = SENSOR_VALUE_EXPIRED;
        LCD_NeedsUpdate              = true;
    }
}

static void CheckTimeDisplayNeedUpdate(void)
{
    static uint32_t time_update_timestamp = 0;
    if (Timestamp_GetTimeElapsed(time_update_timestamp, Timestamp_GetCurrent()) > LCD_DATE_AND_TIME_UPDATE_PERIOD_MS)
    {
        time_update_timestamp += LCD_DATE_AND_TIME_UPDATE_PERIOD_MS;
        LCD_NeedsUpdate = true;
    }
}
