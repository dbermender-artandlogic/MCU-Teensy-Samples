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


#define BUILD_NUMBER "0.0.0"           /**< Defines firmware build number. */
#define DFU_VALIDATION_STRING "client" /**< Defines string to be expected in app data */

#define INSTANCE_INDEX_UNKNOWN UINT8_MAX /**< Defines unknown instance index value. */


#ifdef CMAKE_UNIT_TEST

#define DEBUG_INTERFACE (Serial)        /**< Defines serial port to print debug messages. */
#define DEBUG_INTERFACE_BAUDRATE 115200 /**< Defines baudrate of debug interface. */
#define UART_INTERFACE_BAUDRATE 57600   /**< Defines baudrate of modem interface. */

#else

#define DEBUG_INTERFACE (Serial1)       /**< Defines serial port to print debug messages. */
#define DEBUG_INTERFACE_BAUDRATE 115200 /**< Defines baudrate of debug interface. */
#define UART_INTERFACE_BAUDRATE 57600   /**< Defines baudrate of modem interface. */

#endif

/**
+------------------------------------------------------------------------------------------------------------------------+
|                                                    Teensy LC pinout                                                    |
+------------+-----------------------------------------------------------------------------------------------------------+
| Pin number | Description                                                                                               |
+------------+-----------------------------------------------------------------------------------------------------------+
|          0 | UART Debug interface RX.                                                                                  |
|          1 | UART Debug interface TX.                                                                                  |
|          2 | RTS pin for Energy Sensor. Defined as PIN_MODBUS_RTS.                                                     |
|          3 | PWM warm output. Defined as a PIN_PWM_WARM.                                                               |
|          4 | Encoder B input. Defined as a PIN_ENCODER_B.                                                              |
|          5 | PIR Sensor input. Defined as a PIN_PIR.                                                                   |
|          6 | Encoder A input. Defined as a PIN_ENCODER_A.                                                              |
|          7 | UART Energy Sensor RX.                                                                                    |
|          8 | UART Energy Sensor TX.                                                                                    |
|          9 | UART Communication interface RX pin. Used without define in DMA UART Driver located in UARTDriver.cpp/.h. |
|         10 | UART Communication interface TX pin. Used without define in DMA UART Driver located in UARTDriver.cpp/.h. |
|         11 | INT1 pin of RTC PCF8523 module. Defined as a PIN_RTC_INT1.                                                |
|         12 | Not used.                                                                                                 |
|         13 | Status LED output. Defined as a PIN_LED_STATUS.                                                           |
|         14 | Switch encoder input. Defined as a PIN_ENCODER_SW.                                                        |
|         15 | Switch 4 input. Defined as a PIN_SW_4.                                                                    |
|         16 | PWM cold output. Defined as a PIN_PWM_COLD.                                                               |
|         17 | ALS sensor input. Defined as a PIN_ALS.                                                                   |
|         18 | SDA of I2C_0 interface. Used in I2C library module.                                                       |
|         19 | SCL of I2C_0 interface. Used in I2C library module.                                                       |
|         20 | Switch 3 input. Defined as a PIN_SW_3.                                                                    |
|         21 | Switch 2 input. Defined as a PIN_SW_2.                                                                    |
|         22 | Switch 1 input. Defined as a PIN_SW_1.                                                                    |
|         23 | Analog potentiometer input. Defined as a PIN_ANALOG.                                                      |
|         24 | Not used.                                                                                                 |
|         25 | RTC battery measurement ADC pin. Defined as a PIN_RTC_BATTERY.                                            |
|         26 | Not used.                                                                                                 |
+------------+-----------------------------------------------------------------------------------------------------------+
**/
#define PIN_MODBUS_RTS 2
#define PIN_PWM_WARM 3
#define PIN_ENCODER_B 4
#define PIN_PIR 5
#define PIN_ENCODER_A 6
#define PIN_RTC_INT1 11
#define PIN_LED_STATUS 13
#define PIN_ENCODER_SW 14
#define PIN_SW_4 15
#define PIN_PWM_COLD 16
#define PIN_ALS 17
#define PIN_SW_3 20
#define PIN_SW_2 21
#define PIN_SW_1 22
#define PIN_ANALOG 23
#define PIN_RTC_BATTERY 25

#endif    // CONFIG_H_
