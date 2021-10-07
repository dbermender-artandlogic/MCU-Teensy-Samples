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


#include "MCU_DFU.h"

#include <string.h>

#include "CRC.h"
#include "Flasher.h"
#include "Log.h"
#include "UARTProtocol.h"
#include "Utils.h"


#define SHA256_SIZE 32u
#define MAX_PAGE_SIZE 1024UL

#define DFU_INVALID_CODE 0x00
#define DFU_SUCCESS 0x01
#define DFU_OPCODE_NOT_SUPPORTED 0x02
#define DFU_INVALID_PARAMETER 0x03
#define DFU_INSUFFICIENT_RESOURCES 0x04
#define DFU_INVALID_OBJECT 0x05
#define DFU_UNSUPPORTED_TYPE 0x07
#define DFU_OPERATION_NOT_PERMITTED 0x08
#define DFU_OPERATION_FAILED 0x0A
#define DFU_FIRMWARE_ALREADY_UP_TO_DATE 0x80
#define DFU_FIRMWARE_SUCCESSFULLY_UPDATED 0xFF

#define DFU_STATUS_IN_PROGRESS 0x00
#define DFU_STATUS_NOT_IN_PROGRESS 0x01

/**< CRC configuration */
#define CRC_POLYNOMIAL 0x8005u
#define CRC_INIT_VAL 0xFFFFu

/**< Defines string that forces update */
#define DFU_VALIDATION_IGNORE_STRING "ignore"


static uint8_t DfuInProgress             = 0;
static size_t  FirmwareSize              = 0;
static size_t  FirmwareOffset            = 0;
static uint8_t Sha256[SHA256_SIZE]       = {0};
static uint8_t PageBuffer[MAX_PAGE_SIZE] = {0};
static size_t  PageOffset                = 0;
static size_t  PageSize                  = 0;


/*
 *  Validate Application Data
 */
static uint8_t MCU_DFU_AppData_Validate(uint8_t *p_app_data, uint8_t app_data_len);

/*
 *  Clear DFU states
 */
static void MCU_DFU_ClearStates(void);

/*
 *  Calculate CRC of data saved in flash and ram
 */
static uint32_t MCU_DFU_CalcCRC(void);


void SetupDFU(void)
{
    MCU_DFU_ClearStates();

    LOG_INFO("DFU space start addr: %016X", Flasher_GetSpaceAddr());
    LOG_INFO("DFU available bytes:  %d", Flasher_GetSpaceSize());
}

bool MCU_DFU_IsInProgress(void)
{
    return (bool)DfuInProgress;
}

void ProcessDfuInitRequest(uint8_t *p_payload, uint8_t len)
{
    MCU_DFU_ClearStates();

    size_t index = 0;

    FirmwareSize = ((uint32_t)p_payload[index++]);
    FirmwareSize |= ((uint32_t)p_payload[index++] << 8);
    FirmwareSize |= ((uint32_t)p_payload[index++] << 16);
    FirmwareSize |= ((uint32_t)p_payload[index++] << 24);

    for (size_t i = 0; i < SHA256_SIZE; i++)
    {
        Sha256[SHA256_SIZE - i - 1] = p_payload[index++];
    }

    uint8_t  app_data_len = p_payload[index++];
    uint8_t *p_app_data   = p_payload + index;

    uint8_t init_status = MCU_DFU_AppData_Validate(p_app_data, app_data_len);
    if (init_status != DFU_SUCCESS)
    {
        UART_SendDfuInitResponse(&init_status, sizeof(init_status));

        MCU_DFU_ClearStates();

        LOG_INFO("DFU Rejected");

        return;
    }

    size_t available = Flasher_GetSpaceSize();
    if (available > FirmwareSize)
    {
        Flasher_EraseSpace();

        uint8_t init_status[] = {DFU_SUCCESS};
        UART_SendDfuInitResponse(init_status, sizeof(init_status));

        DfuInProgress = 1;
    }
    else
    {
        uint8_t init_status[] = {DFU_INSUFFICIENT_RESOURCES};
        UART_SendDfuInitResponse(init_status, sizeof(init_status));

        MCU_DFU_ClearStates();
    }

    LOG_INFO("DFU Init:");
    LOG_INFO("Size: %d", FirmwareSize);
    LOG_INFO("Available:%d", available);
    LOG_INFO_HEXBUF("SHA256:", Sha256, SHA256_SIZE);
}

void ProcessDfuStatusRequest(uint8_t *p_payload, uint8_t len)
{
    uint32_t offset = FirmwareOffset + PageOffset;
    uint32_t crc    = MCU_DFU_CalcCRC();

    uint8_t response[] = {
        DFU_SUCCESS,

        (uint8_t)MAX_PAGE_SIZE,
        (uint8_t)(MAX_PAGE_SIZE >> 8),
        (uint8_t)(MAX_PAGE_SIZE >> 16),
        (uint8_t)(MAX_PAGE_SIZE >> 24),

        (uint8_t)offset,
        (uint8_t)(offset >> 8),
        (uint8_t)(offset >> 16),
        (uint8_t)(offset >> 24),

        (uint8_t)crc,
        (uint8_t)(crc >> 8),
        (uint8_t)(crc >> 16),
        (uint8_t)(crc >> 24),
    };

    UART_SendDfuStatusResponse(response, sizeof(response));

    LOG_INFO("DFU Status:");
    LOG_INFO("Max page: %08X", MAX_PAGE_SIZE);
    LOG_INFO("offset: %08X", offset);
    LOG_INFO("crc: %08X", crc);
}

void ProcessDfuPageCreateRequest(uint8_t *p_payload, uint8_t len)
{
    if (!DfuInProgress)
    {
        uint8_t response[] = {DFU_OPERATION_NOT_PERMITTED};
        UART_SendDfuPageCreateResponse(response, sizeof(response));
        UART_SendDfuCancelRequest(NULL, 0);
        LOG_INFO("DFU Page, dfu not in progress");
        return;
    }

    size_t   index = 0;
    uint32_t req_page_size;
    req_page_size = ((uint32_t)p_payload[index++]);
    req_page_size |= ((uint32_t)p_payload[index++] << 8);
    req_page_size |= ((uint32_t)p_payload[index++] << 16);
    req_page_size |= ((uint32_t)p_payload[index++] << 24);

    if (req_page_size <= MAX_PAGE_SIZE)
    {
        PageOffset = 0;
        PageSize   = req_page_size;

        uint8_t response[] = {DFU_SUCCESS};
        UART_SendDfuPageCreateResponse(response, sizeof(response));
        LOG_INFO("DFU Page Created:");
        LOG_INFO("Size: %08X", req_page_size);
    }
    else
    {
        uint8_t response[] = {DFU_INSUFFICIENT_RESOURCES};
        UART_SendDfuPageCreateResponse(response, sizeof(response));
        LOG_INFO("DFU Page Invalid Size:");
        LOG_INFO("Size: %08X", req_page_size);
    }
}

void ProcessDfuWriteDataEvent(uint8_t *p_payload, uint8_t len)
{
    if (!DfuInProgress)
    {
        UART_SendDfuCancelRequest(NULL, 0);
        LOG_INFO("DFU Write data, dfu not in progress");
        return;
    }

    size_t   index     = 0;
    uint8_t  image_len = p_payload[index++];
    uint8_t *p_image   = p_payload + index;

    if (PageOffset + image_len <= PageSize)
    {
        memcpy(PageBuffer + PageOffset, p_image, image_len);
        PageOffset += image_len;
    }
}

void ProcessDfuPageStoreRequest(uint8_t *p_payload, uint8_t len)
{
    if (!DfuInProgress)
    {
        uint8_t response[] = {DFU_OPERATION_NOT_PERMITTED};
        UART_SendDfuPageStoreResponse(response, sizeof(response));
        UART_SendDfuCancelRequest(NULL, 0);
        LOG_INFO("DFU Write data, dfu not in progress");
        return;
    }

    if (PageOffset == 0)
    {
        uint8_t response[] = {DFU_SUCCESS};
        UART_SendDfuPageStoreResponse(response, sizeof(response));
        LOG_INFO("DFU Page not stored");
        return;
    }

    if (PageOffset != PageSize)
    {
        uint8_t response[] = {DFU_OPERATION_NOT_PERMITTED};
        UART_SendDfuPageStoreResponse(response, sizeof(response));

        LOG_INFO("DFU Page store failed, size doesn't match");
        return;
    }

    uint32_t page_store_address = Flasher_GetSpaceAddr() + FirmwareOffset;
    uint8_t  ret_val            = Flasher_SaveMemoryToFlash(page_store_address, (uint32_t *)PageBuffer, PageSize / 4);
    if (ret_val != FLASHER_SUCCESS)
    {
        uint8_t response[] = {DFU_OPERATION_FAILED};
        UART_SendDfuPageStoreResponse(response, sizeof(response));

        LOG_INFO("DFU Page not stored, flasher fail");
        return;
    }

    FirmwareOffset += PageOffset;
    PageOffset = 0;
    PageSize   = 0;

    if (FirmwareOffset != FirmwareSize)
    {
        uint8_t response[] = {DFU_SUCCESS};
        UART_SendDfuPageStoreResponse(response, sizeof(response));

        uint32_t crc = CalcCRC32((uint8_t *)((uintptr_t)Flasher_GetSpaceAddr()), FirmwareOffset, CRC_INIT_VAL);
        LOG_INFO("DFU Page store success, CRC %08X", crc);
        return;
    }

    uint8_t calculated_sha256[SHA256_SIZE];
    CalcSHA256((uint8_t *)((uintptr_t)Flasher_GetSpaceAddr()), FirmwareOffset, calculated_sha256);
    bool is_object_valid = (0 == memcmp(calculated_sha256, Sha256, SHA256_SIZE));

    if (!is_object_valid)
    {
        uint8_t response[] = {DFU_INVALID_OBJECT};
        UART_SendDfuPageStoreResponse(response, sizeof(response));

        LOG_INFO("DFU Invalid object");
        MCU_DFU_ClearStates();
        return;
    }

    uint8_t response[] = {DFU_FIRMWARE_SUCCESSFULLY_UPDATED};
    UART_SendDfuPageStoreResponse(response, sizeof(response));

    LOG_INFO("DFU Firmware updated");
    DEBUG_INTERFACE.flush();

    size_t FwSizeWords = FirmwareSize / sizeof(uint32_t);

    MCU_DFU_ClearStates();
    Flasher_UpdateFirmware(FwSizeWords);

    //Should not get there
    for (;;)
    {
        digitalWrite(PIN_LED_STATUS, 0);
        delay(1000);
        digitalWrite(PIN_LED_STATUS, 1);
        delay(1000);
    }
}

void ProcessDfuStateCheckResponse(uint8_t *p_payload, uint8_t len)
{
    size_t  index  = 0;
    uint8_t status = p_payload[index++];

    if ((status == DFU_STATUS_IN_PROGRESS) != (DfuInProgress))
    {
        UART_SendDfuCancelRequest(NULL, 0);
        LOG_INFO("DFU Canceling");
    }
}

void ProcessDfuCancelResponse(uint8_t *p_payload, uint8_t len)
{
    MCU_DFU_ClearStates();
    LOG_INFO("DFU Cancelled");
}

static uint8_t MCU_DFU_AppData_Validate(uint8_t *p_app_data, uint8_t app_data_len)
{
    LOG_INFO("Application Data length: %d", app_data_len);
    LOG_INFO_HEXBUF("Application Data:", p_app_data, app_data_len);

    if (app_data_len == strlen(DFU_VALIDATION_IGNORE_STRING) &&
        memcmp(p_app_data, DFU_VALIDATION_IGNORE_STRING, app_data_len) == 0)
    {
        /* Application Data contains special string that always validates the package */
        return DFU_SUCCESS;
    }

    /* Valid Application Data is in format: DFU_VALIDATION_STRING/BUILD_NUMBER */
    uint8_t *delimiter = (uint8_t *)memchr(p_app_data, '/', app_data_len);
    if (delimiter == NULL)
    {
        /* Application Data does not contain delimiter */
        return DFU_INVALID_OBJECT;
    }
    uint8_t *fw_type      = p_app_data;
    uint8_t  fw_type_len  = delimiter - p_app_data;
    uint8_t *fw_build     = &p_app_data[fw_type_len + 1];
    uint8_t  fw_build_len = app_data_len - fw_type_len - 1;

    if (strncmp((char *)fw_type, DFU_VALIDATION_STRING, fw_type_len) || (fw_type_len != strlen(DFU_VALIDATION_STRING)))
    {
        /* DFU package contains different type of firmware */
        return DFU_INVALID_OBJECT;
    }

    if (!strncmp((char *)fw_build, BUILD_NUMBER, fw_build_len) && (fw_build_len == strlen(BUILD_NUMBER)))
    {
        /* Application Data contains the same firmware that exists on the device */
        return DFU_FIRMWARE_ALREADY_UP_TO_DATE;
    }

    return DFU_SUCCESS;
}


static void MCU_DFU_ClearStates(void)
{
    DfuInProgress  = 0;
    FirmwareSize   = 0;
    FirmwareOffset = 0;
    PageOffset     = 0;
    PageSize       = 0;

    memset(Sha256, 0, SHA256_SIZE);
    memset(PageBuffer, 0, MAX_PAGE_SIZE);
}

static uint32_t MCU_DFU_CalcCRC(void)
{
    uint32_t crc = ~CRC32_INIT_VAL;
    if (FirmwareOffset != 0)
    {
        crc = CalcCRC32((uint8_t *)((uintptr_t)Flasher_GetSpaceAddr()), FirmwareOffset, ~crc);
    }
    if (PageOffset != 0)
    {
        crc = CalcCRC32(PageBuffer, PageOffset, ~crc);
    }
    return crc;
}
