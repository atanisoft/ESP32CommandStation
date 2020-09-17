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
 * \file HttpResponse.cpp
 *
 * Implementation of various AbstractHttpResponse classes.
 *
 * @author Mike Dunston
 * @date 13 Sept 2019
 */

#include "Httpd.h"

namespace http
{

extern string HTTP_BUILD_TIME;

extern std::map<HttpHeader, string> well_known_http_headers;

static std::map<HttpStatusCode, string> http_code_strings =
{
  {STATUS_CONTINUE, "Continue"},
  {STATUS_SWITCH_PROTOCOL, "Switching Protocols"},

  {STATUS_OK, "OK"},
  {STATUS_CREATED, "Created"},
  {STATUS_ACCEPTED, "Accepted"},
  {STATUS_NON_AUTH_INFO, "Non-Authoritative Information"},
  {STATUS_NO_CONTENT, "No Content"},
  {STATUS_RESET_CONTENT, "Reset Content"},
  {STATUS_PARTIAL_CONTENT, "Partial Content"},

  {STATUS_MULTIPLE_CHOICES, "Multiple Choices"},
  {STATUS_MOVED_PERMANENTLY, "Moved Permanently"},
  {STATUS_FOUND, "Found"},
  {STATUS_SEE_OTHER, "See Other"},
  {STATUS_NOT_MODIFIED, "Not Modified"},
  {STATUS_USE_PROXY, "Use Proxy"},
  {STATUS_TEMP_REDIRECT, "Temporary Redirect"},

  {STATUS_BAD_REQUEST, "Bad Request"},
  {STATUS_NOT_AUTHORIZED, "Unauthorized"},
  {STATUS_PAYMENT_REQUIRED, "Payment Required"},
  {STATUS_FORBIDDEN, "Forbidden"},
  {STATUS_NOT_FOUND, "Not Found"},
  {STATUS_NOT_ALLOWED, "Method Not Allowed"},
  {STATUS_NOT_ACCEPTABLE, "Not Acceptable"},
  {STATUS_PROXY_AUTH_REQ, "Proxy Authentication Required"},
  {STATUS_TIMEOUT, "Request Time-out"},
  {STATUS_CONFLICT, "Conflict"},
  {STATUS_GONE, "Gone"},
  {STATUS_LENGTH_REQ, "Length Required"},
  {STATUS_PRECOND_FAIL, "Precondition Failed"},
  {STATUS_ENTITY_TOO_LARGE, "Request Entity Too Large"},
  {STATUS_URI_TOO_LARGE, "Request-URI Too Large"},
  {STATUS_UNSUPPORTED_MEDIA_TYPE, "Unsupported Media Type"},
  {STATUS_RANGE_NOT_SATISFIABLE, "Requested range not satisfiable"},
  {STATUS_EXPECATION_FAILED, "Expectation Failed"},

  {STATUS_SERVER_ERROR, "Internal Server Error"},
  {STATUS_NOT_IMPLEMENTED, "Not Implemented"},
  {STATUS_BAD_GATEWAY, "Bad Gateway"},
  {STATUS_SERVICE_UNAVAILABLE, "Service Unavailable"},
  {STATUS_GATEWAY_TIMEOUT, "Gateway Time-out"},
  {STATUS_HTTP_VERSION_UNSUPPORTED, "HTTP Version not supported"}
};

AbstractHttpResponse::AbstractHttpResponse(HttpStatusCode code
                                         , const string &mime_type)
                                         : code_(code), mime_type_(mime_type)
                                         , encoded_headers_("")
{
  // seed default headers
  header(HttpHeader::CACHE_CONTROL, HTTP_CACHE_CONTROL_NO_CACHE);
}

AbstractHttpResponse::~AbstractHttpResponse()
{
  encoded_headers_.clear();
  headers_.clear();
}

uint8_t *AbstractHttpResponse::get_headers(size_t *len, bool keep_alive
                                         , bool add_keep_alive)
{
  to_string(false, keep_alive, add_keep_alive);
  *len = encoded_headers_.size();
  return (uint8_t *)encoded_headers_.c_str();
}

string AbstractHttpResponse::to_string(bool include_body, bool keep_alive
                                     , bool add_keep_alive)
{
  encoded_headers_.assign(StringPrintf("HTTP/1.1 %d %s%s", code_
                                      , http_code_strings[code_].c_str()
                                      , HTML_EOL));
  for (auto &ent : headers_)
  {
    LOG(CONFIG_HTTP_RESP_LOG_LEVEL, "[resp-header] %s -> %s", ent.first.c_str()
      , ent.second.c_str());
    encoded_headers_.append(
      StringPrintf("%s: %s%s", ent.first.c_str(), ent.second.c_str()
                  , HTML_EOL));
  }

  if (get_body_length())
  {
    LOG(CONFIG_HTTP_RESP_LOG_LEVEL, "[resp-header] %s -> %zu"
      , well_known_http_headers[HttpHeader::CONTENT_LENGTH].c_str()
      , get_body_length());
    encoded_headers_.append(
      StringPrintf("%s: %zu%s"
                 , well_known_http_headers[HttpHeader::CONTENT_LENGTH].c_str()
                 , get_body_length(), HTML_EOL));
    LOG(CONFIG_HTTP_RESP_LOG_LEVEL, "[resp-header] %s -> %s"
      , well_known_http_headers[HttpHeader::CONTENT_TYPE].c_str()
      , get_body_mime_type().c_str());
    encoded_headers_.append(
      StringPrintf("%s: %s%s"
                 , well_known_http_headers[HttpHeader::CONTENT_TYPE].c_str()
                 , get_body_mime_type().c_str(), HTML_EOL));
  }

  if (add_keep_alive)
  {
    string connection = keep_alive ? HTTP_CONNECTION_CLOSE
                                  : HTTP_CONNECTION_KEEP_ALIVE;
    LOG(CONFIG_HTTP_RESP_LOG_LEVEL, "[resp-header] %s -> %s"
      , well_known_http_headers[HttpHeader::CONNECTION].c_str()
      , connection.c_str());
    encoded_headers_.append(
      StringPrintf("%s: %s%s"
                 , well_known_http_headers[HttpHeader::CONNECTION].c_str()
                 , connection.c_str(), HTML_EOL));
  }

  // leave blank line after headers before the body
  encoded_headers_.append(HTML_EOL);

  return encoded_headers_;
}

void AbstractHttpResponse::header(const string &header, const string &value)
{
  headers_[header] = std::move(value);
}

void AbstractHttpResponse::header(const HttpHeader header, const string &value)
{
  headers_[well_known_http_headers[header]] = std::move(value);
}

RedirectResponse::RedirectResponse(const string &target_uri)
  : AbstractHttpResponse(HttpStatusCode::STATUS_FOUND)
{
  header(HttpHeader::LOCATION, target_uri);
}

StringResponse::StringResponse(const string &response, const string &mime_type)
  : AbstractHttpResponse(HttpStatusCode::STATUS_OK, mime_type)
  , response_(std::move(response))
{
}

StaticResponse::StaticResponse(const uint8_t *payload, const size_t length
                             , const std::string mime_type
                             , const std::string encoding)
                             : AbstractHttpResponse(STATUS_OK, mime_type)
                             , payload_(payload), length_(length)
{
  if (!encoding.empty())
  {
    header(HttpHeader::CONTENT_ENCODING, encoding);
  }
  header(HttpHeader::LAST_MODIFIED, HTTP_BUILD_TIME);
  // update the default cache strategy to set the must-revalidate and max-age
  header(HttpHeader::CACHE_CONTROL
       , StringPrintf("%s, %s, %s=%d"
                    , HTTP_CACHE_CONTROL_NO_CACHE
                    , HTTP_CACHE_CONTROL_MUST_REVALIDATE
                    , HTTP_CACHE_CONTROL_MAX_AGE
                    , config_httpd_cache_max_age_sec()));
}

} // namespace http
