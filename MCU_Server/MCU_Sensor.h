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

#ifndef MCU_SENSOR_SERVER_H
#define MCU_SENSOR_SERVER_H


#include <stdint.h>


#define MESH_TOLERANCE(_error) ((uint16_t)((4095 * _error) / 100))

/*
 * Sensor Positive Tolerance: 0 percent
 */
#define PIR_POSITIVE_TOLERANCE MESH_TOLERANCE(0)
/*
 * Sensor Negative Tolerance: 0 percent
 */
#define PIR_NEGATIVE_TOLERANCE MESH_TOLERANCE(0)
/*
 * Sensor Sampling Function: Instantaneous
 *
 * Sensor Sampling Functions:
 * 0x00 - Unspecified
 * 0x01 - Instantaneous
 * 0x02 - Arithmetic Mean
 * 0x03 - RMS
 * 0x04 - Maximum
 * 0x05 - Minimum
 * 0x06 - Accumulated
 * 0x07 - Count
 * 0x08 - 0xFF - RFU
 */
#define PIR_SAMPLING_FUNCTION 0x01
/*
 * Sensor Measurement Period: Not Applicable
 *
 * Calculated using formula: value = 1.1^(n-64) [s]
 *
 * Value of 0x00 means 'Not Applicable'
 */
#define PIR_MEASUREMENT_PERIOD 0x00
/*
 * Sensor Update Interval: 1 second
 *
 * Calculated using formula: value = 1.1^(n-64) [s]
 *
 * Value of 0x00 means 'Not Applicable'
 */
#define PIR_UPDATE_INTERVAL 0x40

/*
 * Sensor Positive Tolerance: 0 percent
 */
#define ALS_POSITIVE_TOLERANCE MESH_TOLERANCE(0)
/*
 * Sensor Negative Tolerance: 0 percent
 */
#define ALS_NEGATIVE_TOLERANCE MESH_TOLERANCE(0)
/*
 * Sensor Sampling Function: Instantaneous
 *
 * Sensor Sampling Functions:
 * 0x00 - Unspecified
 * 0x01 - Instantaneous
 * 0x02 - Arithmetic Mean
 * 0x03 - RMS
 * 0x04 - Maximum
 * 0x05 - Minimum
 * 0x06 - Accumulated
 * 0x07 - Count
 * 0x08 - 0xFF - RFU
 */
#define ALS_SAMPLING_FUNCTION 0x01
/*
 * Sensor Measurement Period: Not Applicable
 *
 * Calculated using formula: value = 1.1^(n-64) [s]
 *
 * Value of 0x00 means 'Not Applicable'
 */
#define ALS_MEASUREMENT_PERIOD 0x00
/*
 * Sensor Update Interval: 1 second
 *
 * Calculated using formula: value = 1.1^(n-64) [s]
 *
 * Value of 0x00 means 'Not Applicable'
 */
#define ALS_UPDATE_INTERVAL 0x40

/*
 * Sensor Positive Tolerance: 0.5 percent
 */
#define VOLTAGE_SENSOR_POSITIVE_TOLERANCE MESH_TOLERANCE(0.5)
/*
 * Sensor Negative Tolerance: 0.5 percent
 */
#define VOLTAGE_SENSOR_NEGATIVE_TOLERANCE MESH_TOLERANCE(0.5)
/*
 * Sensor Sampling Function: RMS
 *
 * Sensor Sampling Functions:
 * 0x00 - Unspecified
 * 0x01 - Instantaneous
 * 0x02 - Arithmetic Mean
 * 0x03 - RMS
 * 0x04 - Maximum
 * 0x05 - Minimum
 * 0x06 - Accumulated
 * 0x07 - Count
 * 0x08 - 0xFF - RFU
 */
#define VOLTAGE_SENSOR_SAMPLING_FUNCTION 0x03
/*
 * Sensor Measurement Period: Not Applicable
 *
 * Calculated using formula: value = 1.1^(n-64) [s]
 *
 * Value of 0x00 means 'Not Applicable'
 */
#define VOLTAGE_SENSOR_MEASUREMENT_PERIOD 0x00
/*
 * Sensor Update Interval: 1 second
 *
 * Calculated using formula: value = 1.1^(n-64) [s]
 *
 * Value of 0x00 means 'Not Applicable'
 */
#define VOLTAGE_SENSOR_UPDATE_INTERVAL 0x40

/*
 * Sensor Positive Tolerance: 0.5 percent
 */
#define CURRENT_SENSOR_POSITIVE_TOLERANCE MESH_TOLERANCE(0.5)
/*
 * Sensor Negative Tolerance: 0.5 percent
 */
#define CURRENT_SENSOR_NEGATIVE_TOLERANCE MESH_TOLERANCE(0.5)
/*
 * Sensor Sampling Function: RMS
 *
 * Sensor Sampling Functions:
 * 0x00 - Unspecified
 * 0x01 - Instantaneous
 * 0x02 - Arithmetic Mean
 * 0x03 - RMS
 * 0x04 - Maximum
 * 0x05 - Minimum
 * 0x06 - Accumulated
 * 0x07 - Count
 * 0x08 - 0xFF - RFU
 */
#define CURRENT_SENSOR_SAMPLING_FUNCTION 0x03
/*
 * Sensor Measurement Period: Not Applicable
 *
 * Calculated using formula: value = 1.1^(n-64) [s]
 *
 * Value of 0x00 means 'Not Applicable'
 */
#define CURRENT_SENSOR_MEASUREMENT_PERIOD 0x00
/*
 * Sensor Update Interval: 1 second
 *
 * Calculated using formula: value = 1.1^(n-64) [s]
 *
 * Value of 0x00 means 'Not Applicable'
 */
#define CURRENT_SENSOR_UPDATE_INTERVAL 0x40

/*
 * Sensor Positive Tolerance: 1 percent
 */
#define POWER_SENSOR_POSITIVE_TOLERANCE MESH_TOLERANCE(1)
/*
 * Sensor Negative Tolerance: 1 percent
 */
#define POWER_SENSOR_NEGATIVE_TOLERANCE MESH_TOLERANCE(1)
/*
 * Sensor Sampling Function: RMS
 *
 * Sensor Sampling Functions:
 * 0x00 - Unspecified
 * 0x01 - Instantaneous
 * 0x02 - Arithmetic Mean
 * 0x03 - RMS
 * 0x04 - Maximum
 * 0x05 - Minimum
 * 0x06 - Accumulated
 * 0x07 - Count
 * 0x08 - 0xFF - RFU
 */
#define POWER_SENSOR_SAMPLING_FUNCTION 0x03
/*
 * Sensor Measurement Period: Not Applicable
 *
 * Calculated using formula: value = 1.1^(n-64) [s]
 *
 * Value of 0x00 means 'Not Applicable'
 */
#define POWER_SENSOR_MEASUREMENT_PERIOD 0x00
/*
 * Sensor Update Interval: 1 second
 *
 * Calculated using formula: value = 1.1^(n-64) [s]
 *
 * Value of 0x00 means 'Not Applicable'
 */
#define POWER_SENSOR_UPDATE_INTERVAL 0x40

/*
 * Sensor Positive Tolerance: 1 percent
 */
#define ENERGY_SENSOR_POSITIVE_TOLERANCE MESH_TOLERANCE(1)
/*
 * Sensor Negative Tolerance: 1 percent
 */
#define ENERGY_SENSOR_NEGATIVE_TOLERANCE MESH_TOLERANCE(1)
/*
 * Sensor Sampling Function: RMS
 *
 * Sensor Sampling Functions:
 * 0x00 - Unspecified
 * 0x01 - Instantaneous
 * 0x02 - Arithmetic Mean
 * 0x03 - RMS
 * 0x04 - Maximum
 * 0x05 - Minimum
 * 0x06 - Accumulated
 * 0x07 - Count
 * 0x08 - 0xFF - RFU
 */
#define ENERGY_SENSOR_SAMPLING_FUNCTION 0x03
/*
 * Sensor Measurement Period: Not Applicable
 *
 * Calculated using formula: value = 1.1^(n-64) [s]
 *
 * Value of 0x00 means 'Not Applicable'
 */
#define ENERGY_SENSOR_MEASUREMENT_PERIOD 0x00
/*
 * Sensor Update Interval: 1 second
 *
 * Calculated using formula: value = 1.1^(n-64) [s]
 *
 * Value of 0x00 means 'Not Applicable'
 */
#define ENERGY_SENSOR_UPDATE_INTERVAL 0x40

#define MESH_PROPERTY_PRESENT_AMBIENT_LIGHT_LEVEL_UNKNOWN_VAL 0xFFFFFF
#define MESH_PROPERTY_PRESENT_DEVICE_INPUT_POWER_UNKNOWN_VAL 0xFFFFFF
#define MESH_PROPERTY_PRESENT_INPUT_CURRENT_UNKNOWN_VAL 0xFFFF
#define MESH_PROPERTY_PRESENT_INPUT_VOLTAGE_UNKNOWN_VAL 0xFFFF
#define MESH_PROPERTY_PRECISE_TOTAL_DEVICE_ENERGY_USE_UNKNOWN_VAL 0xFFFFFFFF


/*
 *  Sensor Server ALS instance index setter
 */
void SetSensorServerALSIdx(uint8_t idx);

/*
 *  Sensor Server ALS instance index getter
 *
 *  @return  Instance index
 */
uint8_t GetSensorServerALSIdx(void);

/*
 *  Sensor Server PIR instance index setter
 */
void SetSensorServerPIRIdx(uint8_t idx);

/*
 *  Sensor Server PIR instance index getter
 *
 *  @return  Instance index
 */
uint8_t GetSensorServerPIRIdx(void);

/*
 *  Sensor Server Voltage Current instance index setter
 */
void SetSensorServerCurrPreciseEnergyIdx(uint8_t idx);

/*
 *  Sensor Server Voltage Current instance index getter
 *
 *  @return  Instance index
 */
uint8_t GetSensorServerCurrPreciseEnergyIdx(void);

/*
 *  Sensor Server Power Energy instance index setter
 */
void SetSensorServerVoltPowIdx(uint8_t idx);

/*
 *  Sensor Server Power Energy instance index getter
 *
 *  @return  Instance index
 */
uint8_t GetSensorServerVoltPowIdx(void);

/*
 *  Setup sensor server hardware
 */
void SetupSensorServer(void);

/*
 *  Sensor server main function, should be called in Arduino main loop
 */
void LoopSensorServer(void);

#endif    // MCU_SENSOR_SERVER_H
