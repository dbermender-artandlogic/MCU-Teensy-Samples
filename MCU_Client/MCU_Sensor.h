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

#ifndef MCU_SENSOR_H
#define MCU_SENSOR_H

/********************************************
 * INCLUDES                                 *
 ********************************************/

#include <stdint.h>

/********************************************
 * EXPORTED FUNCTIONS PROTOTYPES            *
 ********************************************/

/*
 *  Set Sensor Client instance index
 *
 *  @param idx  Lightness value
 */
void SetInstanceIdxSensor(uint8_t idx);

/*
 *  Get Sensor Client instance index
 *
 *  @return     Lightness value
 */
uint8_t GetInstanceIdxSensor(void);

/*
 *  Process ALS value update
 *
 *  @param src_addr     Source address
 *  @param value        New sensor value
 */
void ProcessPresentAmbientLightLevel(uint16_t src_addr, uint32_t value_clux);

/*
 *  Process PIR value update
 *
 *  @param src_addr     Source address
 *  @param value        New sensor value
 */
void ProcessPresenceDetected(uint16_t src_addr, bool value);

/*
 *  Setup sensor server hardware
 */
void SetupSensor(void);

/*
 *  Sensor server main function, should be called in Arduino main loop
 */
void LoopSensor(void);

#endif  // MCU_SENSOR_H