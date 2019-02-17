/** \copyright
 * Copyright (c) 2014, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file ApplicationChecksum.hxx
 *
 * Helper function to check a falshed application's checksum for correctness.
 *
 * @author Balazs Racz
 * @date 14 May 2017
 */

#ifndef _OPENLCB_APPLICATIONCHECKSUM_HXX_
#define _OPENLCB_APPLICATIONCHECKSUM_HXX_

#include "freertos/bootloader_hal.h"

extern "C" {
/** @returns true if the application checksum currently in flash is correct. */
bool check_application_checksum()
{
    uint32_t checksum[CHECKSUM_COUNT];
    const void *flash_min;
    const void *flash_max;
    const struct app_header *app_header_ptr;
    get_flash_boundaries(&flash_min, &flash_max, &app_header_ptr);

    struct app_header app_header;
    memcpy(&app_header, app_header_ptr, sizeof(app_header));

    uint32_t pre_size = reinterpret_cast<const uint8_t *>(app_header_ptr) -
        static_cast<const uint8_t *>(flash_min);
    checksum_data(flash_min, pre_size, checksum);
    if (memcmp(app_header.checksum_pre, checksum, sizeof(checksum)))
    {
        return false;
    }
    size_t flash_size = (size_t)flash_max - (size_t)flash_min;
    if (app_header.app_size > flash_size) return false;
    uint32_t post_offset = sizeof(struct app_header) +
        (reinterpret_cast<const uint8_t *>(app_header_ptr) -
         static_cast<const uint8_t *>(flash_min));
    uint32_t post_size = (post_offset < app_header.app_size)
        ? app_header.app_size - post_offset
        : 0;
    checksum_data(app_header_ptr + 1, post_size, checksum);
    if (memcmp(app_header.checksum_post, checksum, sizeof(checksum)))
    {
        return false;
    }
    return true;
}
}

#endif // _OPENLCB_APPLICATIONCHECKSUM_HXX_
