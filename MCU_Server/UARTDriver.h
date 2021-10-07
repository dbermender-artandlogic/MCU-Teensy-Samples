/*
Copyright © 2021 Silvair Sp. z o.o. All Rights Reserved.

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

#ifndef UARTDRIVER_H
#define UARTDRIVER_H

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

/*
 *  Initialize UART Driver.
 */
void UARTDriver_Init(void);

/*
 *  Write bytes from table to transmit buffer.
 *
 *  @param table        pointer to table with bytes that 
 *                      you want to write to transmit buffer 
 *  @param len          length of table
 *  
 *  @return             False if overflow in TX buffer occured, true otherwise
 */
bool UARTDriver_WriteBytes(uint8_t *table, uint16_t len);

/*
 *  Read Byte from Receive Buffer.
 *
 *  @param read_byte        pointer for received byte 
 *  
 *  @return                 False if RX buffer is empty, true otherwise 
 */
bool UARTDriver_ReadByte(uint8_t *read_byte);

/*
 *  Function for polling received bytes from UART DMA buffer
 */
void UARTDriver_RxDMAPoll(void);

#endif    //UARTDRIVER_H
