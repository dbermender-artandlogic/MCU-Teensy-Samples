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

#ifndef MCU_HEALTH_H
#define MCU_HEALTH_H


#include <stdint.h>


/** Silvair Mesh Model Company Id */
#define SILVAIR_ID 0x0136u


/*
 *  Check if there is test in progress.
 */
bool IsTestInProgress(void);

/*
 * Setup health hardware
 */
void SetupHealth(void);

/*
 * Health main function, should be called in Arduino main loop
 */
void LoopHealth(void);

/*
 * Set index of registered Health Server model
 */
void SetHealthSrvIdx(uint8_t idx);

/*
 * Get index of registered Health Server model
 */
uint8_t GetHealthSrvIdx(void);

#endif    // MCU_HEALTH_H
