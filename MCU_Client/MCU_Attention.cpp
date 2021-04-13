/*
Copyright © 2019 Silvair Sp. z o.o. All Rights Reserved.
 
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


#include "MCU_Attention.h"

#include "Arduino.h"
#include "Config.h"
#include "UARTProtocol.h"


static bool     AttentionState;
static bool     AttentionLedValue;
static uint32_t AttentionToggleTimestamp;


void SetupAttention(void)
{
    pinMode(PIN_LED_STATUS, OUTPUT);
    AttentionStateSet(false);
}

void LoopAttention(void)
{
    if (!AttentionState)
        return;

    uint32_t timestamp = millis();
    uint32_t duration  = timestamp - AttentionToggleTimestamp;

    if (duration >= ATTENTION_TIME_MS)
    {
        AttentionLedValue        = !AttentionLedValue;
        AttentionToggleTimestamp = millis();

        digitalWrite(PIN_LED_STATUS, AttentionLedValue);
    }
}

void AttentionStateSet(bool state)
{
    AttentionToggleTimestamp = millis();
    AttentionState           = state;
    AttentionLedValue        = false;
    digitalWrite(PIN_LED_STATUS, AttentionLedValue);
}

void ProcessAttention(uint8_t *p_payload, uint8_t len)
{
    INFO("Attention State %d\n\n.", p_payload[0]);
    AttentionStateSet(p_payload[0] == 0x01);
}
