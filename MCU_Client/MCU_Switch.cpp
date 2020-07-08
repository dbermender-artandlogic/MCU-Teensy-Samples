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


#include "MCU_Switch.h"

#include <limits.h>
#include <stdint.h>

#include "Arduino.h"
#include "Config.h"
#include "Encoder.h"
#include "LCD.h"
#include "Mesh.h"
#include "UARTProtocol.h"


/**
 * Buttons placement definitions
 */
#define PB_ON_1 PIN_SW_1           /**< Defines ON button location. */
#define PB_OFF_1 PIN_SW_2          /**< Defines OFF button location. */
#define PB_ON_2 PIN_SW_3           /**< Defines ON button location. */
#define PB_OFF_2 PIN_SW_4          /**< Defines OFF button location. */
#define PB_ENCODER_A PIN_ENCODER_A /**< Defines Encoder A pin location. */
#define PB_ENCODER_B PIN_ENCODER_B /**< Defines Encoder B pin location. */

/**
 * Generic Level messages for Light CTL Client configuration
 */

#define GENERIC_LEVEL_MIN INT16_MIN     /**< Defines lower range of light lightness. */
#define GENERIC_LEVEL_MAX INT16_MAX     /**< Defines upper range of light lightness. */
#define GENERIC_LEVEL_MIN_CHANGE 0x200u /**< Defines the lowest reported changed. */
#define GENERIC_LEVEL_INTVL_MS 50u      /**< Defines the shortest interval between two LightL messages. */

/**
 * On Off communication properties
 */
#define ON_OFF_TRANSITION_TIME_MS 0 /**< Defines transition time */
#define ON_OFF_NUMBER_OF_REPEATS 2  /**< Defines number of repeats while sending mesh message request */
#define ON_OFF_DELAY_TIME_MS 0      /**< Defines message delay in milliseconds */

#define GENERIC_OFF 0x00 /**< Defines Generic OnOff Off payload */
#define GENERIC_ON 0x01  /**< Defines Generic OnOff On payload */

/**
 * Generic Delta Client step
 */
#define DELTA_STEP_VALUE 0x500         /**< Defines Generic Delta minimal step */
#define DELTA_INTVL_MS 100             /**< Defines the shortest interval beetwen two Delta Set messages. */
#define DELTA_NEW_TID_INTVL 350        /**< Defines the shortest interval beetwen generation of new TID. */
#define DELTA_TRANSITION_TIME_MS 100   /**< Defines transition time */
#define DELTA_DELAY_TIME_MS 40         /**< Defines message delay in milliseconds */
#define DELTA_DELAY_TIME_MS_LAST 0     /**< Defines message delay in milliseconds */
#define DELTA_NUMBER_OF_REPEATS 0      /**< Defines number of repeats while sending mesh message request */
#define DELTA_NUMBER_OF_REPEATS_LAST 2 /**< Defines number of repeats while sending mesh message request */

/**
 * Default communication properties
 */
#define GENERIC_LEVEL_NUMBER_OF_REPEATS 0 /**< Defines default number of repeats while sending mesh message request */
#define GENERIC_LEVEL_TRANSITION_TIME_MS 100 /**< Defines default transition time */
#define GENERIC_LEVEL_DELAY_TIME_MS 0        /**< Defines default delay time */

#define POT_DEADBAND 20 /**< Potentiometer deadband. About 2% of full range */
#define ANALOG_MIN 0    /**< Defines lower range of analog measurements. */
#define ANALOG_MAX 1023 /**< Defines uppper range of analog measurements. */


static volatile bool On1                       = false; /**< Implies if Generic ON button has been pushed */
static volatile bool Off1                      = false; /**< Implies if Generic OFF button has been pushed */
static volatile bool On2                       = false; /**< Implies if Generic ON button has been pushed */
static volatile bool Off2                      = false; /**< Implies if Generic OFF button has been pushed */
static volatile bool Reinit                    = false; /**< Implies if LCD reinit button has been pushed */
static uint8_t       LightCtlClientInstanceIdx = INSTANCE_INDEX_UNKNOWN;
static uint8_t       LightLcClientInstanceIdx  = INSTANCE_INDEX_UNKNOWN;
static Encoder       DeltaEncoder(PB_ENCODER_B, PB_ENCODER_A);


/*
 *  Generic On 1 button interrupt handler
 */
static void InterruptOn1PBClick(void);

/*
 *  Generic Off 1 button interrupt handler
 */
static void InterruptOff1PBClick(void);

/*
 *  Generic On 2 button interrupt handler
 */
static void InterruptOn2PBClick(void);

/*
 *  Generic Off 2 button interrupt handler
 */
static void InterruptOff2PBClick(void);

/*
 *  LCD reinit button interrupt handler
 */
static void InterruptLcdReinit(void);

/*
 *  Check if temperature need an update
 *
 *  @param gen_level Generic Level of temperature
 */
static bool HasGenLevelChanged(int gen_level);

/*
 *  Get Generic Level value
 */
static int16_t GenericLevelGet(void);

/*
 *  Print Generic Level of Temperature on debug interface
 *
 *  @param gen_level Generic Level value
 */
static void PrintGenericLevelTemperature(int gen_level);


static void InterruptOn1PBClick(void)
{
    delay(BUTTON_DEBOUNCE_TIME_MS);
    if (digitalRead(PB_ON_1))
        return;

    On1 = true;
}

static void InterruptOff1PBClick(void)
{
    delay(BUTTON_DEBOUNCE_TIME_MS);
    if (digitalRead(PB_OFF_1))
        return;

    Off1 = true;
}

static void InterruptOn2PBClick(void)
{
    delay(BUTTON_DEBOUNCE_TIME_MS);
    if (digitalRead(PB_ON_2))
        return;

    On2 = true;
}

static void InterruptOff2PBClick(void)
{
    delay(BUTTON_DEBOUNCE_TIME_MS);
    if (digitalRead(PB_OFF_2))
        return;

    Off2 = true;
}

static void InterruptLcdReinit(void)
{
    delay(BUTTON_DEBOUNCE_TIME_MS);
    if (digitalRead(PIN_ENCODER_SW))
        return;

    Reinit = true;
}

static bool HasGenLevelChanged(int gen_level)
{
    static int  prev_gen_level          = GENERIC_LEVEL_MIN;
    static bool is_prev_gen_level_valid = false;

    if (!is_prev_gen_level_valid)
    {
        prev_gen_level          = gen_level;
        is_prev_gen_level_valid = true;
        return false;
    }
    else if ((abs(gen_level - prev_gen_level) > GENERIC_LEVEL_MIN_CHANGE) ||
             ((gen_level == 0) && (prev_gen_level != 0)))
    {
        prev_gen_level = gen_level;
        return true;
    }
    else
    {
        return false;
    }
}

static int16_t GenericLevelGet(void)
{
    uint16_t analog_measurement = analogRead(PIN_ANALOG);

    if (analog_measurement < ANALOG_MIN + POT_DEADBAND)
    {
        analog_measurement = ANALOG_MIN + POT_DEADBAND;
    }
    if (analog_measurement > ANALOG_MAX - POT_DEADBAND)
    {
        analog_measurement = ANALOG_MAX - POT_DEADBAND;
    }

    return map(analog_measurement,
               ANALOG_MIN + POT_DEADBAND,
               ANALOG_MAX - POT_DEADBAND,
               GENERIC_LEVEL_MIN,
               GENERIC_LEVEL_MAX);
}

static void PrintGenericLevelTemperature(int gen_level)
{
    unsigned percentage = ((gen_level - GENERIC_LEVEL_MIN) * 100.0) / (GENERIC_LEVEL_MAX - GENERIC_LEVEL_MIN);
    INFO("Current Generic Level of Temperature is %d (%d%)\n", gen_level, percentage);
}


void SetInstanceIdxCtl(uint8_t idx)
{
    LightCtlClientInstanceIdx = idx;
}

uint8_t GetInstanceIdxCtl(void)
{
    return LightCtlClientInstanceIdx;
}

void SetInstanceIdxLc(uint8_t idx)
{
    LightLcClientInstanceIdx = idx;
}

uint8_t GetInstanceIdxLc(void)
{
    return LightLcClientInstanceIdx;
}

void SetupSwitch(void)
{
    INFO("Switch initialization.\n");
    pinMode(PB_ON_1, INPUT_PULLUP);
    pinMode(PB_OFF_1, INPUT_PULLUP);
    pinMode(PB_ON_2, INPUT_PULLUP);
    pinMode(PB_OFF_2, INPUT_PULLUP);
    pinMode(PIN_ENCODER_SW, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PB_ON_1), InterruptOn1PBClick, FALLING);
    attachInterrupt(digitalPinToInterrupt(PB_OFF_1), InterruptOff1PBClick, FALLING);
    attachInterrupt(digitalPinToInterrupt(PB_ON_2), InterruptOn2PBClick, FALLING);
    attachInterrupt(digitalPinToInterrupt(PB_OFF_2), InterruptOff2PBClick, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_SW), InterruptLcdReinit, FALLING);
}

void LoopSwitch(void)
{
    if (On1)
    {
        On1 = false;
        INFO("Generic ON 1\n");
        Mesh_SendGenericOnOffSet(LightLcClientInstanceIdx,
                                 GENERIC_ON,
                                 ON_OFF_TRANSITION_TIME_MS,
                                 ON_OFF_DELAY_TIME_MS,
                                 ON_OFF_NUMBER_OF_REPEATS,
                                 true);
    }

    if (Off1)
    {
        Off1 = false;
        INFO("Generic OFF 1\n");
        Mesh_SendGenericOnOffSet(LightLcClientInstanceIdx,
                                 GENERIC_OFF,
                                 ON_OFF_TRANSITION_TIME_MS,
                                 ON_OFF_DELAY_TIME_MS,
                                 ON_OFF_NUMBER_OF_REPEATS,
                                 true);
    }

    if (On2)
    {
        On2 = false;
        INFO("Generic ON 2\n");
        Mesh_SendGenericOnOffSet(LightCtlClientInstanceIdx,
                                 GENERIC_ON,
                                 ON_OFF_TRANSITION_TIME_MS,
                                 ON_OFF_DELAY_TIME_MS,
                                 ON_OFF_NUMBER_OF_REPEATS,
                                 true);
    }

    if (Off2)
    {
        Off2 = false;
        INFO("Generic OFF 2\n");
        Mesh_SendGenericOnOffSet(LightCtlClientInstanceIdx,
                                 GENERIC_OFF,
                                 ON_OFF_TRANSITION_TIME_MS,
                                 ON_OFF_DELAY_TIME_MS,
                                 ON_OFF_NUMBER_OF_REPEATS,
                                 true);
    }

    if (Reinit)
    {
        Reinit = false;
        INFO("LCD Reinit\n");

        LCD_Reinit();
    }

    static int long last_message_time = ULONG_MAX;

    long encoder_pos;
    encoder_pos = DeltaEncoder.read();

    if (encoder_pos != 0)
    {
        if (last_message_time + DELTA_INTVL_MS <= millis())
        {
            static uint32_t last_delta_message_time = UINT32_MAX;
            static int      delta                   = 0;

            bool is_new_tid = (last_delta_message_time + DELTA_NEW_TID_INTVL) < millis();
            if (is_new_tid)
                delta = 0;

            delta += encoder_pos;
            Mesh_SendGenericDeltaSet(LightLcClientInstanceIdx,
                                     DELTA_STEP_VALUE * delta,
                                     DELTA_TRANSITION_TIME_MS,
                                     DELTA_DELAY_TIME_MS,
                                     DELTA_NUMBER_OF_REPEATS,
                                     is_new_tid);

            if (is_new_tid)
                INFO("Delta Continue %d \n\n", delta);
            else
                INFO("Delta Start %d \n\n", delta);

            DeltaEncoder.write(0);
            last_delta_message_time = millis();
            last_message_time       = millis();
        }
    }

    if (last_message_time + GENERIC_LEVEL_INTVL_MS <= millis())
    {
        int16_t gen_level = GenericLevelGet();
        if (HasGenLevelChanged(gen_level))
        {
            INFO("Temperature changed");
            PrintGenericLevelTemperature(gen_level);
            Mesh_SendGenericLevelSet(LightCtlClientInstanceIdx,
                                     gen_level,
                                     GENERIC_LEVEL_TRANSITION_TIME_MS,
                                     GENERIC_LEVEL_DELAY_TIME_MS,
                                     GENERIC_LEVEL_NUMBER_OF_REPEATS,
                                     true);

            last_message_time = millis();
        }
    }
}
