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

#include "Timestamp.h"

#include "Arduino.h"

const uint32_t TIMESTAMP_MAX                 = UINT32_MAX;
const uint32_t TIMESTAMP_MAX_COMPARABLE_DIFF = UINT32_MAX / 2;

uint32_t Timestamp_GetCurrent(void)
{
    return millis();
}

bool Timestamp_Compare(uint32_t timestamp_lhs, uint32_t timestamp_rhs)
{
    return (timestamp_rhs - timestamp_lhs) <= TIMESTAMP_MAX_COMPARABLE_DIFF;
}

uint32_t Timestamp_GetTimeElapsed(uint32_t timestamp_earlier, uint32_t timestamp_further)
{
    if (timestamp_further < timestamp_earlier)
    {
        return timestamp_further + (TIMESTAMP_MAX - timestamp_earlier) + 1;
    }

    return timestamp_further - timestamp_earlier;
}

uint32_t Timestamp_GetDelayed(uint32_t timestamp, uint32_t delay)
{
    return (timestamp + delay);
}
