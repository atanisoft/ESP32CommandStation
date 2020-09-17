/** \copyright
 * Copyright (c) 2019-2020, Mike Dunston
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
 * \file HttpdConstants.cpp
 *
 * Default constants for the DNS and Http servers.
 *
 * @author Mike Dunston
 * @date 13 Sept 2019
 */

#include <utils/constants.hxx>

namespace http
{

///////////////////////////////////////////////////////////////////////////////
// Httpd constants
///////////////////////////////////////////////////////////////////////////////

DEFAULT_CONST(httpd_server_stack_size, 6144);
DEFAULT_CONST(httpd_server_priority, 0);
DEFAULT_CONST(httpd_header_chunk_size, 512);
DEFAULT_CONST(httpd_body_chunk_size, 3072);
DEFAULT_CONST(httpd_response_chunk_size, 2048);
DEFAULT_CONST(httpd_max_header_size, 1024);
DEFAULT_CONST(httpd_max_req_size, 4194304);
DEFAULT_CONST(httpd_max_req_per_connection, 5);
DEFAULT_CONST(httpd_req_timeout_ms, 5);
DEFAULT_CONST(httpd_socket_timeout_ms, 50);
DEFAULT_CONST(httpd_websocket_timeout_ms, 200);
DEFAULT_CONST(httpd_websocket_max_frame_size, 256);
DEFAULT_CONST(httpd_websocket_max_read_attempts, 2);
DEFAULT_CONST(httpd_cache_max_age_sec, 300);

///////////////////////////////////////////////////////////////////////////////
// Dnsd constants
///////////////////////////////////////////////////////////////////////////////

DEFAULT_CONST(dnsd_stack_size, 3072);
DEFAULT_CONST(dnsd_buffer_size, 512);

} // namespace http