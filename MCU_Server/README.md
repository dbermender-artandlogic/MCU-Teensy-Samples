# Lightness and Sensor Server

## IMPORTANT!
'Serial2' class from Teensyduino library is not supported (project has own implementation of UART1 based on DMA). Using
it may cause application problems.

## Usage
Lightness Server receives LightL Status messages from UART Modem and adjust PIN_PWM output accordingly to received data.
Sensor Server measures sensor states and sends SensorUpdateRequest to UART Modem periodically.