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

#include "UARTDriver.h"

#include <DMAChannel.h>

#include "Config.h"
#include "RingBuffer.h"
#include "kinetis.h"

#define RX_BUFFER_LEN 512
#define TX_BUFFER_LEN 512

#define C2_RX_ENABLE (UART_C2_TE | UART_C2_RE | UART_C2_RIE)
#define C2_TX_ACTIVE (UART_C2_TIE)
#define C2_TX_INACTIVE (~C2_TX_ACTIVE)
#define C3_ERROR_ISR_ENABLED (UART_C3_ORIE | UART_C3_NEIE | UART_C3_FEIE | UART_C3_PEIE)
#define C4_UART_DMA_ENABLED (UART_C5_TDMAS | UART_C5_RDMAS)

#define COUNTER_SIZE (DMA_DSR_BCR_BCR(RX_BUFFER_LEN))

static DMAChannel rx_dma;
static DMAChannel tx_dma;

static RingBuffer_T rx_dma_buffer;
static RingBuffer_T tx_dma_buffer;

/*
 *  According to http://cache.freescale.com/files/microcontrollers/doc/ref_manual/KL26P121M48SF4RM.pdf
 *  page 380, DMA buffers must be aligned to a 0-modulo-(circular buffer size) boundary.
 */
static __attribute__((section(".dmabuffers"), aligned(TX_BUFFER_LEN))) uint8_t tx_buf[TX_BUFFER_LEN];
static __attribute__((section(".dmabuffers"), aligned(RX_BUFFER_LEN))) uint8_t rx_buf[RX_BUFFER_LEN];

static uint16_t cur_tx_message_len = 0;

static void DMA_TransmitRequest();
static void DMA_OnTXCompletion();
static void DMA_OnRXCompletion();
static bool IsTXActive();

void UARTDriver_Init()
{
    // Connect clock to UART1
    SIM_SCGC4 |= SIM_SCGC4_UART1;

    // Setup baud rate
    uint32_t baud_rate = BAUD2DIV2(UART_INTERFACE_BAUDRATE);
    UART1_BDH          = (baud_rate >> 8) & 0x1F;
    UART1_BDL          = baud_rate & 0xFF;

    // Initialize pins
    CORE_PIN9_CONFIG  = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);
    CORE_PIN10_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);

    // TX DMA configuration
    RingBuffer_Init(&tx_dma_buffer, tx_buf, TX_BUFFER_LEN);
    tx_dma.destination(UART1_D);
    tx_dma.interruptAtCompletion();
    tx_dma.disableOnCompletion();
    tx_dma.attachInterrupt(DMA_OnTXCompletion);
    tx_dma.triggerAtHardwareEvent(DMAMUX_SOURCE_UART1_TX);

    // RX DMA configuration
    RingBuffer_Init(&rx_dma_buffer, rx_buf, RX_BUFFER_LEN);
    rx_dma.source(UART1_D);
    rx_dma.destinationCircular(rx_buf, RX_BUFFER_LEN);
    rx_dma.disableOnCompletion();
    rx_dma.transferCount(COUNTER_SIZE);
    rx_dma.attachInterrupt(DMA_OnRXCompletion);
    rx_dma.interruptAtCompletion();
    rx_dma.triggerAtHardwareEvent(DMAMUX_SOURCE_UART1_RX);
    rx_dma.enable();

    // UART Register settings
    UART1_C1 = 0;
    UART1_C2 = C2_RX_ENABLE;
    UART1_C3 = C3_ERROR_ISR_ENABLED;
    UART1_MA1 |= C4_UART_DMA_ENABLED;    // UART1_MA1 is UART1_C4 register (bug with address mapping in teensyduino libraries)
}

bool UARTDriver_WriteBytes(uint8_t *table, uint16_t table_len)
{
    if (!RingBuffer_QueueBytes(&tx_dma_buffer, table, table_len))
    {
        return false;
    }

    DMA_TransmitRequest();
    return true;
}

void DMA_TransmitRequest()
{
    __disable_irq();
    if (IsTXActive())
    {
        __enable_irq();
        return;
    }

    uint8_t *tx_begin_pointer = RingBuffer_GetMaxContinuousBuffer(&tx_dma_buffer, &cur_tx_message_len);
    if (cur_tx_message_len == 0)
    {
        __enable_irq();
        return;
    }

    tx_dma.sourceBuffer(tx_begin_pointer, cur_tx_message_len);
    UART1_C2 |= C2_TX_ACTIVE;
    tx_dma.enable();
    __enable_irq();
}

bool UARTDriver_ReadByte(uint8_t *read_byte)
{
    return RingBuffer_DequeueByte(&rx_dma_buffer, read_byte);
}

void UARTDriver_RxDMAPoll()
{
    __disable_irq();
    RingBuffer_SetWrIndex(&rx_dma_buffer, COUNTER_SIZE - DMA_DSR_BCR_BCR(DMA_DSR_BCR0));
    __enable_irq();
}

static bool IsTXActive()
{
    return (UART1_C2 & C2_TX_ACTIVE);
}

static void DMA_OnTXCompletion()
{
    tx_dma.clearInterrupt();
    UART1_C2 &= C2_TX_INACTIVE;
    RingBuffer_IncrementRdIndex(&tx_dma_buffer, cur_tx_message_len);

    if (!RingBuffer_isEmpty(&tx_dma_buffer))
    {
        DMA_TransmitRequest();
    }
}

static void DMA_OnRXCompletion()
{
    rx_dma.clearInterrupt();
    rx_dma.transferCount(COUNTER_SIZE);

    if ((UART1_S1 & UART_S1_OR) != 0)
    {
        (void)UART1_D;
    }

    rx_dma.enable();
}
