/** \copyright
 * Copyright (c) 2022, Mike Dunston
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
 * \file WiThrottleConstants.cpp
 *
 * Default constants for the WiThrottle Server.
 *
 * @author Mike Dunston
 * @date 8 Feb 2022
 */

#ifdef ESP32
#include "sdkconfig.h"
#endif // ESP32

#include <utils/constants.hxx>

namespace withrottle
{

#ifndef CONFIG_WITHROTTLE_MAX_LOCOMOTIVES
#define CONFIG_WITHROTTLE_MAX_LOCOMOTIVES 10
#endif // CONFIG_WITHROTTLE_MAX_LOCOMOTIVES

#ifndef CONFIG_WITHROTTLE_MAX_CONNECTIONS
#define CONFIG_WITHROTTLE_MAX_CONNECTIONS 5
#endif // CONFIG_WITHROTTLE_MAX_CONNECTIONS

#ifndef CONFIG_WITHROTTLE_PORT
#define CONFIG_WITHROTTLE_PORT 12090
#endif // CONFIG_WITHROTTLE_PORT

#ifndef CONFIG_WITHROTTLE_SOCKET_TIMEOUT_MS
#define CONFIG_WITHROTTLE_SOCKET_TIMEOUT_MS 50
#endif // CONFIG_WITHROTTLE_SOCKET_TIMEOUT_MS

#ifndef CONFIG_WITHROTTLE_READ_BUFFER_SIZE
#define CONFIG_WITHROTTLE_READ_BUFFER_SIZE 128
#endif // CONFIG_WITHROTTLE_READ_BUFFER_SIZE

#ifndef CONFIG_WITHROTTLE_CLIENT_READ_TIMEOUT_MS
#define CONFIG_WITHROTTLE_CLIENT_READ_TIMEOUT_MS 50
#endif // CONFIG_WITHROTTLE_CLIENT_READ_TIMEOUT_MS

///////////////////////////////////////////////////////////////////////////////
// WiThrottle constants
///////////////////////////////////////////////////////////////////////////////

DEFAULT_CONST(withrottle_server_stack_size, 2048);
DEFAULT_CONST(withrottle_server_priority, 0);
DEFAULT_CONST(withrottle_socket_timeout_ms,
    CONFIG_WITHROTTLE_SOCKET_TIMEOUT_MS);
DEFAULT_CONST(withrottle_read_size,
    CONFIG_WITHROTTLE_READ_BUFFER_SIZE);
DEFAULT_CONST(withrottle_client_timeout_ms,
    CONFIG_WITHROTTLE_CLIENT_READ_TIMEOUT_MS);
DEFAULT_CONST(withrottle_max_client_locomotives,
    CONFIG_WITHROTTLE_MAX_LOCOMOTIVES);
DEFAULT_CONST(withrottle_max_connections,
    CONFIG_WITHROTTLE_MAX_CONNECTIONS);
DEFAULT_CONST(withrottle_default_port, CONFIG_WITHROTTLE_PORT);
} // namespace http