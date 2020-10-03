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
 * \file HttpRequest.cpp
 *
 * Implementation of the HttpRequest class.
 *
 * @author Mike Dunston
 * @date 13 Sept 2019
 */

#include "Httpd.h"

namespace http
{

static constexpr const char * HTTP_METHOD_DELETE = "DELETE";
static constexpr const char * HTTP_METHOD_GET = "GET";
static constexpr const char * HTTP_METHOD_HEAD = "HEAD";
static constexpr const char * HTTP_METHOD_POST = "POST";
static constexpr const char * HTTP_METHOD_PATCH = "PATCH";
static constexpr const char * HTTP_METHOD_PUT = "PUT";

std::map<HttpHeader, string> well_known_http_headers =
{
  { ACCEPT, "Accept" }
, { CACHE_CONTROL, "Cache-Control" }
, { CONNECTION, "Connection" }
, { CONTENT_ENCODING, "Content-Encoding" }
, { CONTENT_TYPE, "Content-Type" }
, { CONTENT_LENGTH, "Content-Length" }
, { CONTENT_DISPOSITION, "Content-Disposition" }
, { EXPECT, "Expect" }
, { HOST, "Host" }
, { IF_MODIFIED_SINCE, "If-Modified-Since" }
, { LAST_MODIFIED, "Last-Modified" }
, { LOCATION, "Location" }
, { ORIGIN, "Origin" }
, { UPGRADE, "Upgrade" }
, { WS_VERSION, "Sec-WebSocket-Version" }
, { WS_KEY, "Sec-WebSocket-Key" }
, { WS_ACCEPT, "Sec-WebSocket-Accept"}
};

HttpRequest::HttpRequest() : headers_(config_httpd_max_header_count())
                           , params_(config_httpd_max_param_count())
{
}

void HttpRequest::method(const string &value)
{
  LOG(CONFIG_HTTP_REQ_LOG_LEVEL
    , "[HttpReq %p] Setting Method: %s", this, value.c_str());
  raw_method_.assign(std::move(value));
  if (!raw_method_.compare(HTTP_METHOD_DELETE))
  {
    method_ = HttpMethod::DELETE;
  }
  else if (!raw_method_.compare(HTTP_METHOD_GET))
  {
    method_ = HttpMethod::GET;
  }
  else if (!raw_method_.compare(HTTP_METHOD_HEAD))
  {
    method_ = HttpMethod::HEAD;
  }
  else if (!raw_method_.compare(HTTP_METHOD_POST))
  {
    method_ = HttpMethod::POST;
  }
  else if (!raw_method_.compare(HTTP_METHOD_PATCH))
  {
    method_ = HttpMethod::PATCH;
  }
  else if (!raw_method_.compare(HTTP_METHOD_PUT))
  {
    method_ = HttpMethod::PUT;
  }
}

HttpMethod HttpRequest::method()
{
  return method_;
}

const string &HttpRequest::raw_method()
{
  return raw_method_;
}

const string &HttpRequest::uri()
{
  return uri_;
}

void HttpRequest::uri(const string &value)
{
  LOG(CONFIG_HTTP_REQ_LOG_LEVEL
    , "[HttpReq %p] Setting URI: %s", this, value.c_str());
  uri_.assign(std::move(value));
}

void HttpRequest::param(const string &name, const string &value)
{
  if (params_.size() < params_.max_size())
  {
    LOG(CONFIG_HTTP_REQ_LOG_LEVEL
      , "[HttpReq %p] Adding param: %s: %s", this, name.c_str()
      , value.c_str());
    params_[name] = value;
  }
  else
  {
    LOG_ERROR("[HttpReq %p] Discarding parameter '%s' as max parameter count "
              "has been reached!", this, name.c_str());
  }
}

void HttpRequest::header(const string &name, const string &value)
{
  if (headers_.size() < headers_.max_size())
  {
    LOG(CONFIG_HTTP_REQ_LOG_LEVEL
      , "[HttpReq %p] Adding header: %s: %s", this, name.c_str()
      , value.c_str());
    headers_[name] = value;
  }
  else
  {
    LOG_ERROR("[HttpReq %p] Discarding header '%s' as maximum header limit "
              "has been reached!", this, name.c_str());
  }
}

void HttpRequest::header(HttpHeader header, std::string value)
{
  if (has_header(header))
  {
    LOG(CONFIG_HTTP_REQ_LOG_LEVEL
      , "[HttpReq %p] Replacing header: %s: %s (old: %s)", this
      , well_known_http_headers[header].c_str(), value.c_str()
      , headers_[well_known_http_headers[header]].c_str());
  }
  else if (headers_.size() < headers_.max_size())
  {
    LOG(CONFIG_HTTP_REQ_LOG_LEVEL
      , "[HttpReq %p] Adding header: %s: %s", this
      , well_known_http_headers[header].c_str(), value.c_str());
  }
  else
  {
    LOG_ERROR("[HttpReq %p] Discarding header '%s' as maximum header limit "
              "has been reached!", this
            , well_known_http_headers[header].c_str());
    return;
  }
  headers_[well_known_http_headers[header]] = value;
}

bool HttpRequest::has_header(const string &name)
{
  return headers_.find(name) != headers_.end();
}

bool HttpRequest::has_header(const HttpHeader name)
{
  return has_header(well_known_http_headers[name]);
}

const string &HttpRequest::header(const string name)
{
  if (!has_header(name))
  {
    return no_value_;
  }
  return headers_[name];
}

const string &HttpRequest::header(const HttpHeader name)
{
  return header(well_known_http_headers[name]);
}

void HttpRequest::reset()
{
  LOG(CONFIG_HTTP_REQ_LOG_LEVEL
    , "[HttpReq %p] Resetting to blank request", this);
  headers_.clear();
  params_.clear();
  raw_method_.clear();
  method_ = HttpMethod::UNKNOWN_METHOD;
  uri_.clear();
  error_ = false;
}

bool HttpRequest::keep_alive()
{
  if (!has_header(HttpHeader::CONNECTION))
  {
    return false;
  }
  return header(HttpHeader::CONNECTION).compare(HTTP_CONNECTION_CLOSE);
}

void HttpRequest::error(bool value)
{
  LOG(CONFIG_HTTP_REQ_LOG_LEVEL
    , "[HttpReq %p] Setting error flag to %d", this, value);
  error_ = value;
}

bool HttpRequest::error()
{
  return error_;
}

ContentType HttpRequest::content_type()
{
  static const string MULTIPART_FORM = "multipart/form-data";
  static const string FORM_URLENCODED = "application/x-www-form-urlencoded";
  // For a multipart/form-data the Content-Type value will look like:
  // multipart/form-data; boundary=----WebKitFormBoundary4Aq7x8166jGWkA0q
  if (!header(HttpHeader::CONTENT_TYPE).compare(0, MULTIPART_FORM.size()
                                              , MULTIPART_FORM))
  {
    return ContentType::MULTIPART_FORMDATA;
  }
  if (!header(HttpHeader::CONTENT_TYPE).compare(0, FORM_URLENCODED.size()
                                              , FORM_URLENCODED))
  {
    return ContentType::FORM_URLENCODED;
  }
  return ContentType::UNKNOWN_TYPE;
}

void HttpRequest::set_status(HttpStatusCode code)
{
  status_ = code;
}

size_t HttpRequest::params()
{
  return params_.size();
}

string HttpRequest::param(string name)
{
  if (params_.find(name) != params_.end())
  {
    LOG(CONFIG_HTTP_REQ_LOG_LEVEL
      , "[Req %p] Param %s -> %s", this, name.c_str(), params_[name].c_str());
    return params_[name];
  }
  LOG(CONFIG_HTTP_REQ_LOG_LEVEL
    , "[Req %p] Param %s doesn't exist", this, name.c_str());
  return no_value_;
}

bool HttpRequest::param(string name, bool def)
{
  if (params_.find(name) != params_.end())
  {
    LOG(CONFIG_HTTP_REQ_LOG_LEVEL
      , "[Req %p] Param %s -> %s", this, name.c_str(), params_[name].c_str());
    auto value = params_[name];
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    return value.compare("false");
  }
  return def;
}

int HttpRequest::param(string name, int def)
{
  if (params_.find(name) != params_.end())
  {
    LOG(CONFIG_HTTP_REQ_LOG_LEVEL
      , "[Req %p] Param %s -> %s", this, name.c_str(), params_[name].c_str());
    return std::stoi(params_[name]);
  }
  return def;
}

bool HttpRequest::has_param(string name)
{
  return params_.find(name) != params_.end();
}

string HttpRequest::to_string()
{
  string res = StringPrintf("[HttpReq %p] method:%s uri:%s,error:%d,"
                            "header-count:%zu,param-count:%zu"
                          , this, raw_method_.c_str(), uri_.c_str(), error_
                          , headers_.size(), params_.size());
  for (auto &ent : headers_)
  {
    res.append(
      StringPrintf("\nheader: %s: %s%s", ent.first.c_str(), ent.second.c_str()
                  , HTML_EOL));
  }
  for (auto &ent : params_)
  {
    res.append(
      StringPrintf("\nparam: %s: %s%s", ent.first.c_str(), ent.second.c_str()
                  , HTML_EOL));
  }
  return res;
}

} // namespace http
