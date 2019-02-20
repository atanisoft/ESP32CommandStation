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
 * \file Esp32WiFiClientAdapter.hxx
 *
 * ESP32 adapter code using the WiFiClient provided by the WiFiServer code
 * for interfacing with the OpenMRN stack.
 *
 * @author Mike Dunston
 * @date 13 January 2019
 */

#ifndef _FREERTOS_DRIVERS_ESP32_ESP32WIFI_HXX_
#define _FREERTOS_DRIVERS_ESP32_ESP32WIFI_HXX_

#include <Arduino.h>
#include <WiFi.h>

class Esp32WiFiClientAdapter
{
public:
    /// Constructor.
    ///
    /// @param client is the client returned from the ESP32 WiFiServer.
    Esp32WiFiClientAdapter(WiFiClient client)
        : client_(client)
    {
        client_.setNoDelay(true);
    }

    /// This is how many bytes we return as writeable when select says the
    /// socket is write active.
    static constexpr unsigned WRITE_PACKET_SIZE = 512;

    /// @return the capacity of the write buffer for the underlying WiFiClient.
    size_t availableForWrite()
    {
        if (!client_.connected())
        {
            return 0;
        }
        int fd = client_.fd();
        fd_set set;
        struct timeval tv;
        FD_ZERO(&set);    // empties the set
        FD_SET(fd, &set); // adds FD to the set
        tv.tv_sec = 0;
        tv.tv_usec = 0;

        if (select(fd + 1, NULL, &set, NULL, &tv) < 0)
        {
            return 0;
        }

        if (FD_ISSET(fd, &set))
        {
            return WRITE_PACKET_SIZE;
        }
        else
        {
            return 0;
        }
    }

    /// Writes a byte stream to the underlying WiFiClient.
    ///
    /// @param buffer byte stream to be transmitted.
    /// @param len length of byte stream to be transmitted.
    size_t write(const char *buffer, size_t len)
    {
        if (client_.connected())
        {
            return client_.write(buffer, len);
        }
        return 0;
    }

    /// @return the number of bytes available to read from the underlying WiFiClient.
    size_t available()
    {
        if (client_.connected())
        {
            return client_.available();
        }
        return 0;
    }

    /// Reads a byte stream from the underlying WiFiClient.
    ///
    /// @param buffer buffer to read into.
    /// @param len size of the buffer to read into.
    size_t read(const char *buffer, size_t len)
    {
        size_t bytesRead = 0;
        if (client_.connected())
        {
            bytesRead = client_.read((uint8_t *)buffer, len);
        }
        return bytesRead;
    }

private:
    /// WiFiClient being wrapped.
    WiFiClient client_;
};

#endif /* _FREERTOS_DRIVERS_ARDUINO_ESP32WIFI_HXX_ */
