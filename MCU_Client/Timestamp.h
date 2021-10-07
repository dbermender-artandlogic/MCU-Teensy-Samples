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

#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Get current Timestamp.
 *
 *  This value overflows approx. every 48 days.
 *
 *  @return Current timestamp in milliseconds.
 */
uint32_t Timestamp_GetCurrent(void);


/** @brief Less-than operator for timestamps.
 *
 * @note Values should not differ more than half of the range of UINT32_MAX.
 *
 *  @param [in] timestamp_lhs Timestamp on the left hand side
 *  @param [in] timestamp_rhs Timestamp on the right hand side
 *
 *  @return true if timestamp_lhs lies 'behind' timestamp_rhs on the clock half-face
 */
bool Timestamp_Compare(uint32_t timestamp_lhs, uint32_t timestamp_rhs);


/** @brief Get Time elapsed between two timestamps.
 *
 * @note timestamps values should not be bigger than Max Timestamp that can be obtained using Timestamp_GetMax()
 *
 *  @param [in] timestamp_earlier Older timestamp
 *  @param [in] timestamp_further Newer timestamp
 *
 *  @return Time elapsed between timestamps in milliseconds.
 */
uint32_t Timestamp_GetTimeElapsed(uint32_t timestamp_earlier, uint32_t timestamp_further);


/** @brief Apply delay to timestamp.
 *
 *  @note timestamp and delay values should not be bigger than Max Timestamp that can be obtained using Timestamp_GetMax()
 *
 *  @param [in] timestamp Timestamp to be delayed
 *  @param [in] delay Delay to be applied
 *
 *  @return Timestamp after applying delay
 */
uint32_t Timestamp_GetDelayed(uint32_t timestamp, uint32_t delay);

#endif /* #ifndef TIMESTAMP_H */
