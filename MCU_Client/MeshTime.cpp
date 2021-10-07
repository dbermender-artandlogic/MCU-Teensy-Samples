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

#include "MeshTime.h"

#include "Log.h"
#include "Timestamp.h"
#include "UARTProtocol.h"

#define SYNC_TIME_PERIOD_MS (1000 * 10)

#define FEBRUARY_MONTH 2

static const uint8_t      days_in_month[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
static MeshTimeLastSync_T last_sync_time;

static bool IsLeapYear(uint16_t year)
{
    if (year % 4 != 0)
    {
        return false;
    }
    if (year % 100 != 0)
    {
        return true;
    }
    if (year % 400 != 0)
    {
        return false;
    }
    return true;
}

static bool ValidateMonthDay(TimeDate *time_date)
{
    if (time_date->month != FEBRUARY_MONTH)
    {
        return time_date->day <= days_in_month[time_date->month];
    }

    uint8_t num_days_feb = days_in_month[FEBRUARY_MONTH];

    if (IsLeapYear(time_date->year))
    {
        num_days_feb += 1;
    }

    return time_date->day <= num_days_feb;
}


static bool ValidateTimeValuesRange(TimeDate *time_date)
{
    return (((time_date->year <= 36841) && (time_date->year > 0) && (time_date->month <= 12) &&
             (time_date->month > 0) && (time_date->day <= 31) && (time_date->day > 0) && (time_date->hour <= 23) &&
             (time_date->minute <= 59) && (time_date->seconds <= 59) && (time_date->milliseconds <= 999)) ||
            ((time_date->year == 0) && (time_date->month == 0) && (time_date->day == 0) && (time_date->hour == 0) &&
             (time_date->minute == 0) && (time_date->seconds == 0) && (time_date->milliseconds == 0)));
}

void MeshTime_ProcessTimeSourceSetRequest(uint8_t *p_payload, uint8_t len)
{
    if ((p_payload == NULL) || (len != sizeof(TimeSourceSetReq_T)))
    {
        return;
    }

    TimeSourceSetReq_T *msg = (TimeSourceSetReq_T *)p_payload;

    if (msg->instance_index != GetTimeServerInstanceIdx())
    {
        return;
    }

    if (!ValidateTimeValuesRange(&msg->date) || !ValidateMonthDay(&msg->date))
    {
        return;
    }
    RTC_SetTime(&msg->date);
}

void MeshTime_ProcessTimeSourceGetRequest(uint8_t *p_payload, uint8_t len)
{
    if ((p_payload == NULL) || (len != sizeof(TimeSourceGetReq_T)))
    {
        return;
    }

    TimeSourceGetReq_T *msg = (TimeSourceGetReq_T *)p_payload;

    if (msg->instance_index != GetTimeServerInstanceIdx())
    {
        return;
    }

    RTC_GetTime();
}

void MeshTime_ProcessTimeGetResponse(uint8_t *p_payload, uint8_t len)
{
    if ((p_payload == NULL) || (len != sizeof(TimeGetResp_T)))
    {
        return;
    }

    TimeGetResp_T *msg = (TimeGetResp_T *)p_payload;

    if (msg->instance_index != GetTimeServerInstanceIdx())
    {
        return;
    }

    last_sync_time.local_sync_timestamp_ms = Timestamp_GetCurrent();
    last_sync_time.tai_seconds             = msg->tai_seconds;
    last_sync_time.subsecond               = msg->subsecond;
    last_sync_time.tai_utc_delta           = msg->tai_utc_delta;
    last_sync_time.time_zone_offset        = msg->time_zone_offset;
}

MeshTimeLastSync_T *MeshTime_GetLastSyncTime(void)
{
    return &last_sync_time;
}

void LoopMeshTimeSync(void)
{
    static uint32_t last_sync_time_ms = 0;

    if (Timestamp_GetTimeElapsed(last_sync_time_ms, Timestamp_GetCurrent()) > SYNC_TIME_PERIOD_MS)
    {
        LOG_INFO("LoopMeshTimeSync");

        last_sync_time_ms += SYNC_TIME_PERIOD_MS;
        UART_SendTimeGetRequest(GetTimeServerInstanceIdx());
    }
}
