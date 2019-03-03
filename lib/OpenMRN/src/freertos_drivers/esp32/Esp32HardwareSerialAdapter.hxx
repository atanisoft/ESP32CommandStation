/** \copyright
 * Copyright (c) 2019, Mike Dunston
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
 * \file Esp32HardwareSerialAdapter.hxx
 *
 * On the ESP32 the HardwareSerial code does not have a read(const char *,
 * size_t) method so it is necessary to wrap it here.
 *
 * @author Mike Dunston
 * @date 20 January 2019
 */

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32SERIAL_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32SERIAL_HXX_

#include <HardwareSerial.h>

namespace openmrn_arduino {

class Esp32HardwareSerialAdapter
{
public:
    /// Constructor.
    ///
    /// @param serial HardwareSerial device to use as a bridge to the OpenMRN system.
    Esp32HardwareSerialAdapter(HardwareSerial &serial)
        : serial_(serial)
    {
    }

    /// Returns the usable capacity of the underlying HardwareSerial transmit buffer.
    size_t availableForWrite()
    {
        return serial_.availableForWrite();
    }

    /// Writes a byte stream to the underlying HardwareSerial device.
    ///
    /// @param buffer byte stream to be transmitted.
    /// @param len length of byte stream to be transmitted.
    size_t write(const char *buffer, size_t len)
    {
        return serial_.write((uint8_t *)buffer, len);
    }

    /// @return the number of bytes available to read from the underlying HardwareSerial device.
    size_t available()
    {
        return serial_.available();
    }

    /// Reads a byte stream from the underlying HardwareSerial device.
    ///
    /// @param buffer buffer to read into.
    /// @param len size of the buffer to read into.
    size_t read(const char *buffer, size_t len)
    {
        return serial_.readBytes((char *)buffer, len);
    }

private:
    /// HardwareSerial device being wrapped.
    HardwareSerial &serial_;
};

} // namespace openmrn_arduino

using openmrn_arduino::Esp32HardwareSerialAdapter;

#endif /* _FREERTOS_DRIVERS_ESP32_ESP32SERIAL_HXX_ */
