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

#ifndef SENSOR_INPUT_H
#define SENSOR_INPUT_H


#include <stdint.h>

#include "Config.h"


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

#define MESH_PROP_PRESENT_AMBIENT_LIGHT_LEVEL_UNKNOWN_VAL 0xFFFFFF
#define MESH_PROP_PRESENT_DEVICE_INPUT_POWER_UNKNOWN_VAL 0xFFFFFF
#define MESH_PROP_PRESENT_INPUT_CURRENT_UNKNOWN_VAL 0xFFFF
#define MESH_PROP_PRESENT_INPUT_VOLTAGE_UNKNOWN_VAL 0xFFFF
#define MESH_PROP_PRECISE_TOTAL_DEVICE_ENERGY_USE_UNKNOWN_VAL 0xFFFFFFFF


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


typedef union
{
    uint32_t als;
    uint8_t  pir;
    uint32_t power;
    uint16_t current;
    uint16_t voltage;
    uint32_t energy;
    uint32_t precise_energy;
} SensorValue_T;

typedef enum
{
    PRESENCE_DETECTED               = 0x004D,
    PRESENT_AMBIENT_LIGHT_LEVEL     = 0x004E,
    PRESENT_DEVICE_INPUT_POWER      = 0x0052,
    PRESENT_INPUT_CURRENT           = 0x0057,
    PRESENT_INPUT_VOLTAGE           = 0x0059,
    TOTAL_DEVICE_ENERGY_USE         = 0x006A,
    PRECISE_TOTAL_DEVICE_ENERGY_USE = 0x0072
} SensorProperty_T;


/*
 *  Sensor Input ALS instance index setter
 */
void SensorInput_SetAlsIdx(uint8_t idx);

/*
 *  Sensor Input ALS instance index getter
 *
 *  @return  Instance index
 */
uint8_t SensorInput_GetAlsIdx(void);

/*
 *  Sensor Input PIR instance index setter
 */
void SensorInput_SetPirIdx(uint8_t idx);

/*
 *  Sensor Input PIR instance index getter
 *
 *  @return  Instance index
 */
uint8_t SensorInput_GetPirIdx(void);

/*
 *  Sensor Input Voltage Current instance index setter
 */
void SensorInput_SetCurrPreciseEnergyIdx(uint8_t idx);

/*
 *  Sensor Input Voltage Current instance index getter
 *
 *  @return  Instance index
 */
uint8_t SensorInput_GetCurrPreciseEnergyIdx(void);

/*
 *  Sensor Input Power Energy instance index setter
 */
void SensorInput_SetVoltPowIdx(uint8_t idx);

/*
 *  Sensor Input Power Energy instance index getter
 *
 *  @return  Instance index
 */
uint8_t SensorInput_GetVoltPowIdx(void);

/*
 *  Setup Sensor Input hardware
 */
void SensorInput_Setup(void);

/*
 *  Sensor Input main function, should be called in Arduino main loop
 */
void SensorInput_Loop(void);

#endif    // SENSOR_INPUT_H
