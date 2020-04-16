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

#ifndef CONFIG_H_
#define CONFIG_H_


#include "Arduino.h"


#define ENABLE_LC 1     /**< Enable LC support */
#define ENABLE_CTL 0    /**< Enable CTL support */
#define ENABLE_PIRALS 1 /**< Enable PIR and ALS support */
#define ENABLE_ENERGY 1 /**< Enable energy monitoring support */
#define ENABLE_1_10_V 0 /**< Define for calculate lightness for 0-10 V (value 0) or 1-10 V (value 1) */

#define BUILD_NUMBER "0000"            /**< Defines firmware build number. */
#define DFU_VALIDATION_STRING "server" /**< Defines string to be expected in app data */

#define INSTANCE_INDEX_UNKNOWN UINT8_MAX /**< Defines unknown instance index value */

#define ATTENTION_TIME_MS 500 /**< Defines attention state change time in milliseconds. */
#define TEST_TIME_MS 1500     /**< Defines fake test duration in milliseconds. */

#define DEBUG_INTERFACE (Serial)        /**< Defines serial port to print debug messages */
#define DEBUG_INTERFACE_BAUDRATE 115200 /**< Defines baudrate of debug interface */
#define UART_INTERFACE_BAUDRATE 57600   /**< Defines baudrate of modem interface */
#define MODBUS_INTERFACE (Serial3)      /**< Defines serial port to communicate with modem */
#define MODBUS_INTERFACE_BAUDRATE 2400  /**< Defines baudrate of modem interface */

#define PIN_LED_1 11      /**< Defines led 1 pin. */
#define PIN_LED_2 12      /**< Defines led 2 pin. */
#define PIN_LED_STATUS 13 /**< Defines status led pin. */
#define PIN_PWM_WARM 23   /**< Defines pwm pin. */
#define PIN_PWM_COLD 16   /**< Defines pwm pin. */
#define PIN_SW_1 22       /**< Defines switch 1 pin. */
#define PIN_SW_2 21       /**< Defines switch 2 pin. */
#define PIN_SW_3 20       /**< Defines switch 3 pin. */
#define PIN_SW_4 15       /**< Defines switch 4 pin. */
#define PIN_ENCODER_SW 14 /**< Defines encoder switch pin. */
#define PIN_ENCODER_A 6   /**< Defines encoder A pin. */
#define PIN_ENCODER_B 4   /**< Defines encoder B pin. */
#define PIN_ANALOG 9      /**< Defines analog measurement pin. */

#define BUTTON_DEBOUNCE_TIME_MS 20 /**< Defines buttons debounce time in milliseconds. */

#if ENABLE_PIRALS == 1
#define PIR_REGISTRATION_ORDER 1 /**< Defines sensor servers registration order */
#define ALS_REGISTRATION_ORDER 2 /**< Defines sensor servers registration order */
#else
#define PIR_REGISTRATION_ORDER 0 /**< Defines sensor servers registration order */
#define ALS_REGISTRATION_ORDER 0 /**< Defines sensor servers registration order */
#endif

#define CURR_ENERGY_REGISTRATION_ORDER (ALS_REGISTRATION_ORDER + 1) /**< Defines sensor servers registration order */
#define VOLT_POWER_REGISTRATION_ORDER \
    (CURR_ENERGY_REGISTRATION_ORDER + 1) /**< Defines sensor servers registration order */

#define PWM_RESOLUTION 16 /**< Defines PWM resolution value */

#define LOG_INFO_ENABLE 0 /**< Enables INFO level logs */
#define LOG_DEBUG_ENABLE \
    0 /**< Enables DEBUG level logs Enabling this make DFU impossible, due to implementation of UART in Arduino. */

#endif    // CONFIG_H_
