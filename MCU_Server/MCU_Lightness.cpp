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


#include "MCU_Lightness.h"

#include <TimerOne.h>
#include <TimerThree.h>
#include <math.h>

#include "Log.h"
#include "Mesh.h"
#include "Timestamp.h"
#include "UARTProtocol.h"


#define PWM_OUTPUT_MAX UINT16_MAX
#define PWM_RESOLUTION 16 /**< Defines PWM resolution value */

#if ENABLE_1_10_V
#define PWM_OUTPUT_MIN (uint16_t)(0.12 * PWM_OUTPUT_MAX)
#else
#define PWM_OUTPUT_MIN 0u
#endif
/**
 * Light Lightness Controller Server configuration
 */
#define DIINTERRUPT_TIME_MS 5u /**< Dimming control interrupt interval definition [ms]. */
#define DIINTERRUPT_TIME_US (DIINTERRUPT_TIME_MS * 1000)
#define POW(a) ((a) * (a))
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define ATTENTION_LIGHTNESS_ON 0xFFFF
#define ATTENTION_LIGHTNESS_OFF (0xFFFF * 4 / 10)

#define DEVICE_STARTUP_SEQ_STAGE_1_DURATION_MS 3000
#define DEVICE_STARTUP_SEQ_STAGE_2_DURATION_MS 1000
#define DEVICE_STARTUP_SEQ_STAGE_3_DURATION_MS 1000
#define DEVICE_STARTUP_SEQ_STAGE_4_DURATION_MS 1000

#define DEVICE_STARTUP_SEQ_STAGE_1_LIGHTNESS 0xB504
#define DEVICE_STARTUP_SEQ_STAGE_2_LIGHTNESS 0x0001
#define DEVICE_STARTUP_SEQ_STAGE_3_LIGHTNESS 0xFFFF
#define DEVICE_STARTUP_SEQ_STAGE_4_LIGHTNESS 0x0001
#define DEVICE_STARTUP_SEQ_STAGE_OFF_LIGHTNESS 0xFFFF


struct Transition
{
    uint16_t target_value;
    uint16_t start_value;
    uint32_t start_timestamp;
    uint32_t transition_time;
};

typedef enum
{
    DEVICE_SEQUENCE_STAGE_1,
    DEVICE_SEQUENCE_STAGE_2,
    DEVICE_SEQUENCE_STAGE_3,
    DEVICE_SEQUENCE_STAGE_4,
    DEVICE_SEQUENCE_STAGE_OFF,
} DeviceStartupSequence_T;


/*
 *  Convert Lightness Actual to Lightness Linear (based on Spec Model, chapter 6.1.2.2.1)
 *
 *  @param val     Lightness Actual value
 */
static inline uint32_t ConvertLightnessActualToLinear(uint16_t val);

/*
 *  Dimming interrupt handler.
 */
static void DimmInterrupt(void);

/*
 *  Calculate present transition value
 *
 *  @param p_transition     Pointer to transition
 */
static uint16_t GetPresentValue(Transition *p_transition);

/*
 *  Calculate slope ans sets PWM output to specific lightness
 *
 *  @param val     Lightness value
 */
static void SetLightnessOutput(uint16_t val);

/*
 *  Calculate new transition
 *
 *  @param present          Present value
 *  @param target           Target value
 *  @param transition_time  Transition time
 *  @param p_transition     Pointer to transition
 */
static void UpdateTransition(uint16_t present, uint16_t target, uint32_t transition_time, Transition *p_transition);

/*
 *  Calculate current stage on Startup Sequence
 *
 *  @param time_since_sequence_start  time since the beginning of sequence
 */
static DeviceStartupSequence_T GetStartupSequenceStage(unsigned long time_since_sequence_start);

static Transition Light = {
    .target_value    = 0,
    .start_value     = 0,
    .start_timestamp = 0,
    .transition_time = 0,
};

static Transition Temperature = {
    .target_value    = (LIGHT_CTL_TEMP_RANGE_MAX - LIGHT_CTL_TEMP_RANGE_MIN) / 2 + LIGHT_CTL_TEMP_RANGE_MIN,
    .start_value     = LIGHT_CTL_TEMP_RANGE_MIN,
    .start_timestamp = 0,
    .transition_time = 0,
};

static bool          IsEnabled                       = false;
static bool          CTLSupport                      = false;
static uint8_t       LightLSrvIdx                    = INSTANCE_INDEX_UNKNOWN;
static volatile bool AttentionLedState               = false;
static bool          UnprovisionedSequenceEnableFlag = false;


static inline uint32_t ConvertLightnessActualToLinear(uint16_t val)
{
    return (LIGHTNESS_MAX * POW((val * UINT8_MAX) / LIGHTNESS_MAX)) / UINT16_MAX;
}

static void DimmInterrupt(void)
{
    if (!AttentionLedState)
    {
        SetLightnessOutput(GetPresentValue(&Light));
    }
}

static uint16_t GetPresentValue(Transition *p_transition)
{
    uint32_t delta_time = Timestamp_GetTimeElapsed(p_transition->start_timestamp, Timestamp_GetCurrent());

    if (delta_time > p_transition->transition_time)
    {
        p_transition->start_value = p_transition->target_value;
        return p_transition->start_value;
    }

    int32_t delta_transition = ((int64_t)p_transition->target_value - p_transition->start_value) * delta_time /
                               p_transition->transition_time;

    return p_transition->start_value + delta_transition;
}

static void SetLightnessOutput(uint16_t val)
{
    const uint32_t coefficient = ((uint32_t)(PWM_OUTPUT_MAX - PWM_OUTPUT_MIN) * UINT16_MAX) /
                                 (LIGHTNESS_MAX - LIGHTNESS_MIN);
    uint32_t pwm_out;

    if (val == 0)
    {
        pwm_out = 0;
    }
    else
    {
        // calc value to PWM OUTPUT
        pwm_out = ConvertLightnessActualToLinear(val);
        // Calculate value depend of 0-10 V
        pwm_out = ((coefficient * (pwm_out - LIGHTNESS_MIN)) / UINT16_MAX) + PWM_OUTPUT_MIN;
    }

    if (CTLSupport)
    {
        uint64_t warm;
        uint64_t cold;

        uint16_t temperature = GetPresentValue(&Temperature);

        cold = (LIGHT_CTL_TEMP_RANGE_MAX - temperature) * pwm_out;
        cold /= LIGHT_CTL_TEMP_RANGE_MAX - LIGHT_CTL_TEMP_RANGE_MIN;

        warm = (temperature - LIGHT_CTL_TEMP_RANGE_MIN) * pwm_out;
        warm /= LIGHT_CTL_TEMP_RANGE_MAX - LIGHT_CTL_TEMP_RANGE_MIN;

        analogWrite(PIN_PWM_WARM, warm);
        analogWrite(PIN_PWM_COLD, cold);
    }
    else
    {
        analogWrite(PIN_PWM_COLD, pwm_out);
        analogWrite(PIN_PWM_WARM, 0);
    }
}

static void UpdateTransition(uint16_t present, uint16_t target, uint32_t transition_time, Transition *p_transition)
{
    noInterrupts();
    p_transition->start_value     = present;
    p_transition->target_value    = target;
    p_transition->transition_time = transition_time;
    p_transition->start_timestamp = Timestamp_GetCurrent();
    interrupts();
}

static DeviceStartupSequence_T GetStartupSequenceStage(unsigned long time_since_sequence_start)
{
    long startup_sequence_duration_ms[] = {DEVICE_STARTUP_SEQ_STAGE_1_DURATION_MS,
                                           DEVICE_STARTUP_SEQ_STAGE_2_DURATION_MS,
                                           DEVICE_STARTUP_SEQ_STAGE_3_DURATION_MS,
                                           DEVICE_STARTUP_SEQ_STAGE_4_DURATION_MS};

    for (size_t i = 0; i < ARRAY_SIZE(startup_sequence_duration_ms); i++)
    {
        if (time_since_sequence_start / startup_sequence_duration_ms[i] >= 1)
        {
            time_since_sequence_start -= startup_sequence_duration_ms[i];
        }
        else
        {
            return (DeviceStartupSequence_T)i;
        }
    }
    return DEVICE_SEQUENCE_STAGE_OFF;
}

static void PerformStartupSequenceIfNeeded(void)
{
    static DeviceStartupSequence_T present_startup_sequence_stage = DEVICE_SEQUENCE_STAGE_OFF;
    static long                    sequence_start                 = UINT32_MAX;
    uint16_t                       startup_sequence_lightness[]   = {DEVICE_STARTUP_SEQ_STAGE_1_LIGHTNESS,
                                             DEVICE_STARTUP_SEQ_STAGE_2_LIGHTNESS,
                                             DEVICE_STARTUP_SEQ_STAGE_3_LIGHTNESS,
                                             DEVICE_STARTUP_SEQ_STAGE_4_LIGHTNESS,
                                             DEVICE_STARTUP_SEQ_STAGE_OFF_LIGHTNESS};
    if (UnprovisionedSequenceEnableFlag)
    {
        sequence_start                  = Timestamp_GetCurrent();
        UnprovisionedSequenceEnableFlag = false;
        present_startup_sequence_stage  = DEVICE_SEQUENCE_STAGE_1;

        ProcessTargetLightness(0, startup_sequence_lightness[present_startup_sequence_stage], 0);
    }

    unsigned long           sequence_duration = Timestamp_GetTimeElapsed(sequence_start, Timestamp_GetCurrent());
    DeviceStartupSequence_T calculated_stage  = GetStartupSequenceStage(sequence_duration);

    if (present_startup_sequence_stage != DEVICE_SEQUENCE_STAGE_OFF &&
        present_startup_sequence_stage != calculated_stage)
    {
        present_startup_sequence_stage = calculated_stage;
        ProcessTargetLightness(0, startup_sequence_lightness[present_startup_sequence_stage], 0);
    }
}


void SetLightnessServerIdx(uint8_t idx)
{
    if (!IsEnabled)
        return;
    LightLSrvIdx = idx;
}

void SetLightCTLSupport(bool support)
{
    if (!IsEnabled)
        return;
    CTLSupport = support;
}

uint8_t GetLightnessServerIdx(void)
{
    return LightLSrvIdx;
}

void IndicateAttentionLightness(bool attention_state, bool led_state)
{
    if (!IsEnabled)
        return;

    if (attention_state)
    {
        uint16_t led_lightness = led_state ? ATTENTION_LIGHTNESS_ON : ATTENTION_LIGHTNESS_OFF;
        SetLightnessOutput(led_lightness);
    }
    AttentionLedState = attention_state;
}

void ProcessTargetLightness(uint16_t present, uint16_t target, uint32_t transition_time)
{
    if (!IsEnabled)
        return;

    LOG_INFO("Lightness: %d -> %d, transition_time %d", present, target, transition_time);

    UpdateTransition(present, target, transition_time, &Light);
}

void ProcessTargetLightnessTemp(uint16_t present, uint16_t target, uint32_t transition_time)
{
    if (!IsEnabled)
        return;

    LOG_INFO("Temperature: %d-> %d, transition_time %d", present, target, transition_time);

    UpdateTransition(present, target, transition_time, &Temperature);
}

void SetupLightnessServer(void)
{
    IsEnabled = true;
    pinMode(PIN_PWM_WARM, OUTPUT);
    pinMode(PIN_PWM_COLD, OUTPUT);
    analogWriteResolution(PWM_RESOLUTION);
    Timer1.initialize(DIINTERRUPT_TIME_US);
    Timer1.attachInterrupt(DimmInterrupt);
}

void LoopLightnessServer(void)
{
    if (!IsEnabled)
        return;

    PerformStartupSequenceIfNeeded();
}

void EnableStartupSequence(void)
{
    if (!IsEnabled)
        return;

    UnprovisionedSequenceEnableFlag = true;
}

void SynchronizeLightness(void)
{
    if (!IsEnabled)
        return;

    Mesh_SendLightLGet(GetLightnessServerIdx());
}
