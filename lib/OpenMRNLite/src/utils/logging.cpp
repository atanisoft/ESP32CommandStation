/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file logging.cxx
 * Facility to do debug printf's on a configurable loglevel. 
 *
 * @author Balazs Racz
 * @date 3 August 2013
 */

#include "logging.h"

#if defined(__linux__) || defined(__MACH__)
char logbuffer[4096];
#else
/// Temporary buffer to sprintf() the log lines into.
char logbuffer[256];
#endif

#ifdef LOCKED_LOGGING
os_mutex_t g_log_mutex = OS_MUTEX_INITIALIZER;
#endif

#ifdef MBED_USE_STDIO_LOGGING  // TARGET_LPC1768

extern "C" { void send_stdio_serial_message(const char* data); }

void log_output(char* buf, int size) {
    if (size <= 0) return;
    buf[size] = '\0';
    send_stdio_serial_message(buf);
}

#elif defined(__linux__) || defined(__MACH__) || defined(__EMSCRIPTEN__) || defined(ESP32)

#include "utils/stdio_logging.h"

#else

/// Prints a line of log to the log output destination.
///
/// @param buf is the logging buffer. Guaranteed to be available until this
/// function returns.
/// @param size is now many bytes are there in the logging buffer. There is
/// never a terminating \n in the log buffer. There is a terminating zero at
/// buf[size].
__attribute__((weak)) void log_output(char* buf, int size) {}

#endif
