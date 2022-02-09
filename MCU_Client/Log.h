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


#ifndef LOG_H_
#define LOG_H_

#include <stdint.h>
#include <stdio.h>

#ifndef CMAKE_UNIT_TEST
#include "Config.h"
#endif

#include "Utils.h"

#define LOG_INFO_ENABLE 1 /**< Enables INFO level logs */
#define LOG_DEBUG_ENABLE \
    1 /**< Enables DEBUG level logs Enabling this make DFU impossible, due to implementation of UART in Arduino */

static bool toggleLED = false;
static int ledToggleCount = 0;


#ifdef CMAKE_UNIT_TEST
#define _PRINTF(format, ...) printf(format, ##__VA_ARGS__)
#else
#define _PRINTF(format, ...)                     \
    DEBUG_INTERFACE.printf(format, ##__VA_ARGS__)
#endif

#define _LOG(format, ...)               \
    do                                  \
    {                                   \
        _PRINTF(format, ##__VA_ARGS__); \
        _PRINTF("\n");                  \
    } while (0)

void setToggleLCD(bool val);
bool getToggleLCD();

static inline void _LOG_NULL(const char *format, ...)
{
    UNUSED(format);
}

static inline void _LOG_HEXBUF_NULL(const char *text, const void *buf, size_t len)
{
    UNUSED(text);
    UNUSED(buf);
    UNUSED(len);
}

void _LOG_HEXBUF(const char *text, const void *buf, size_t len);

#if LOG_INFO_ENABLE == 1
#define LOG_INFO(format, ...) _LOG(format, ##__VA_ARGS__)
#define LOG_INFO_HEXBUF(text, buff, len) _LOG_HEXBUF(text, buff, len)
#else
#define LOG_INFO(format, ...) _LOG_NULL(format, ##__VA_ARGS__)
#define LOG_INFO_HEXBUF(text, buff, len) _LOG_HEXBUF_NULL(text, buff, len)
#endif

#if LOG_DEBUG_ENABLE == 1
#define LOG_DEBUG(format, ...) _LOG(format, ##__VA_ARGS__)
#define LOG_DEBUG_HEXBUF(text, buff, len) _LOG_HEXBUF(text, buff, len)
#else
#define LOG_DEBUG(format, ...) _LOG_NULL(format, ##__VA_ARGS__)
#define LOG_DEBUG_HEXBUF(text, buff, len) _LOG_HEXBUF_NULL(text, buff, len)
#endif

#endif /* LOG_H_ */
