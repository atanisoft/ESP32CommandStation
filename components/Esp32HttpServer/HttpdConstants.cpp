/**********************************************************************
ESP32 HTTP Server

COPYRIGHT (c) 2019-2020 Mike Dunston

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses
**********************************************************************/

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