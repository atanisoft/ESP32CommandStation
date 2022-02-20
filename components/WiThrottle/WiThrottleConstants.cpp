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

#include "sdkconfig.h"
#include <utils/constants.hxx>

namespace withrottle
{

///////////////////////////////////////////////////////////////////////////////
// WiThrottle constants
///////////////////////////////////////////////////////////////////////////////

DEFAULT_CONST(withrottle_server_stack_size, 4096);
DEFAULT_CONST(withrottle_server_priority, 0);
DEFAULT_CONST(withrottle_socket_timeout_ms, 50);
DEFAULT_CONST(withrottle_read_size, 128);
DEFAULT_CONST(withrottle_client_timeout_ms, 50);
DEFAULT_CONST(withrottle_max_client_locomotives,
    CONFIG_WITHROTTLE_MAX_LOCOMOTIVES);
DEFAULT_CONST(withrottle_max_connections,
    CONFIG_WITHROTTLE_MAX_CONNECTIONS);
} // namespace http