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

#ifndef FLASHER_H_
#define FLASHER_H_


#include <stddef.h>
#include <stdint.h>


#ifndef CMAKE_UNIT_TEST
#ifndef __MKL26Z64__
#error "Flash support available only on MKL26Z64"
#endif
#endif

/**< RAMFUNC attribute definition. Used to place function in RAM */
#define RAMFUNC __attribute__((section(".fastrun"), noinline, noclone, optimize("Os")))

/**< Flasher return codes*/
#define FLASHER_SUCCESS 0
#define FLASHER_ERROR_ALIGNMENT 1
#define FLASHER_ERROR_RANGE 2
#define FLASHER_ERROR_NOT_ERASED 3
#define FLASHER_ERROR_VERIFY 4
#define FLASHER_ERROR_COLLISION 5
#define FLASHER_ERROR_ACCESS 6
#define FLASHER_ERROR_PROTECTION 7
#define FLASHER_ERROR_CONTROLLER 8
#define FLASHER_ERROR_UNSAFE 9


/*
 *  Copy stored firmware to the beggining of flash and reboots.
 *  If success will never return.
 *
 *  @param num_of_words    Size of firmware image.
 *  @return                Flasher return code.
 */
RAMFUNC int Flasher_UpdateFirmware(uint32_t num_of_words);

/*
 *  Save word to flash.
 *
 *  @param address       Destination pointer
 *  @param word_value    Value to flash
 *  @param reenable_irq  If true will leave IRQ enabled, disabled if false.
 *  @return              Flasher return code
 */
RAMFUNC int Flasher_FlashWord(uint32_t address, uint32_t word_value, bool reenable_irq);

/*
 *  Get pointer to beggining of storage space.
 *
 *  @return        storage space address.
 */
uint32_t Flasher_GetSpaceAddr(void);

/*
 *  Get number of available bytes in storage space.
 *
 *  @return        number of bytes available.
 */
size_t Flasher_GetSpaceSize(void);

/*
 *  Erase whole storage space.
 *
 *  @return        Flasher return code
 */
int Flasher_EraseSpace(void);

/*
 *  Saves words to flash.
 *  Destination should be already erased with Flasher_EraseSpace.
 *
 *  @param address         Destination pointer
 *  @param src             Source pointer
 *  @param num_of_words    Size of data to copy
 *  @return                Flasher return code
 */
int Flasher_SaveMemoryToFlash(uint32_t address, const uint32_t *src, uint32_t num_of_words);

#endif    // FLASHER_H_