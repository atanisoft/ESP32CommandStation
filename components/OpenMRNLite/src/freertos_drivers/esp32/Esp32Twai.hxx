/** \copyright
 * Copyright (c) 2020, Mike Dunston
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
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
 * \file Esp32Twai.hxx
 *
 * TWAI driver implementation for OpenMRN. This leverages the ESP-IDF HAL API
 * rather than the TWAI driver to allow 
 *
 * @author Mike Dunston
 * @date 24 September 2020
 */

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32TWAI_HXX_

// Target platform check via the sdkconfig.h file which is only present on the
// ESP32 platform.
#if defined __has_include
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif
#endif // defined __has_include

// Only define the Esp32Twai interface if we are compiling for a supported
// platform.
#if defined(CONFIG_IDF_TARGET)

#include <driver/gpio.h>

#if __has_include(<esp_idf_version.h>)
#include <esp_idf_version.h>
#else
#include <esp_system.h>
#endif

// If we are using IDF 4.2+ we can leverage the TWAI HAL layer
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,0,0)
#define ESP32_TWAI_DRIVER_SUPPORTED 1
#else
#warning Unsupported IDF version
#endif // IDF 4.0+

#endif // CONFIG_IDF_TARGET

#if ESP32_TWAI_DRIVER_SUPPORTED

#include "utils/macros.h"

namespace openmrn_arduino
{

class Esp32Twai
{
public:
    /// Constructor.
    ///
    /// @param vfsPath is the VFS path to register the TWAI driver to.
    /// @param rxPin is the pin connected to the external transceiver RX pin.
    /// @param txPin is the pin connected to the external transceiver TX pin.
    Esp32Twai(const char *vfsPath, int rxPin, int txPin, bool report = true);

    /// Destructor.
    ///
    /// Cleans up resources allocated by the TWAI driver.
    ~Esp32Twai();

    /// Initializes the VFS adapter and TWAI driver
    void hw_init();
private:
    DISALLOW_COPY_AND_ASSIGN(Esp32Twai);

    /// VFS Mount point.
    const char *vfsPath_;

    /// GPIO pin connected to the external transceiver RX pin.
    const gpio_num_t rxPin_;

    /// GPIO pin connected to the external transceiver TX pin.
    const gpio_num_t txPin_;
};

} // namespace openmrn_arduino
using openmrn_arduino::Esp32Twai;
#endif

#endif // _FREERTOS_DRIVERS_ESP32_ESP32TWAI_HXX_