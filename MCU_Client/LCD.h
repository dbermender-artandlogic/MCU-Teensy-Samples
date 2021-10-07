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

#ifndef MCU_LCD_H
#define MCU_LCD_H


#include "MCU_Definitions.h"
#include "SensorOutput.h"
#include "stdint.h"
#include "stdlib.h"


/*
 *  Setup LCD hardware
 */
void LCD_Setup(void);

/*
 *  Update displayed Modem state
 */
void LCD_UpdateModemState(ModemState_t modemState);

/*
 *  Update displayed Modem FW version
 */
void LCD_UpdateModemFwVersion(char *fwVersion, uint8_t fwVerLen);

/*
 *  Update displayed Sensor value
 */
void LCD_UpdateSensorValue(SensorProperty_T sensorProperty, SensorValue_T sensorValue);

/*
 *  Update DFU in progress state
 */
void LCD_UpdateDfuState(bool dfuInProgress);

/*
 *  LCD refresh function, should be called in Arduino main loop
 */
void LCD_Loop(void);

/*
 *  Reinit LCD in case of LCD driver failure
 */
void LCD_Reinit(void);

/*
 *	Sets Sensors values to Unknown state on LCD screen
 */
void LCD_EraseSensorsValues(void);

#endif    // MCU_LCD
