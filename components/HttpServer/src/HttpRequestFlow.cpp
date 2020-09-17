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
 * \file HttpRequestFlow.cpp
 *
 * Implementation of the HttpRequestFlow StateFlow.
 *
 * @author Mike Dunston
 * @date 13 Sept 2019
 */
#include "Httpd.h"
#include "HttpStringUtils.h"

namespace http
{

static vector<string> captive_portal_uris =
{
  "/generate_204",                  // Android
  "/gen_204",                       // Android 9.0
  "/mobile/status.php",             // Android 8.0 (Samsung s9+)
  "/ncsi.txt",                      // Windows
  "/success.txt",                   // OSX / FireFox
  "/hotspot-detect.html",           // iOS 8/9
  "/hotspotdetect.html",            // iOS 8/9
  "/library/test/success.html"      // iOS 8/9
  "/kindle-wifi/wifiredirect.html"  // Kindle
  "/kindle-wifi/wifistub.html"      // Kindle
};

HttpRequestFlow::HttpRequestFlow(Httpd *server, int fd
                               , uint32_t remote_ip)
                               : StateFlowBase(server)
                               , server_(server)
                               , fd_(fd)
                               , remote_ip_(remote_ip)
{
  start_flow(STATE(start_request));
  LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL, "[Httpd fd:%d] Connected.", fd_);
}

HttpRequestFlow::~HttpRequestFlow()
{
  if (close_)
  {
    LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL, "[Httpd fd:%d] Closed", fd_);
    ::close(fd_);
  }
}

StateFlowBase::Action HttpRequestFlow::start_request()
{
  LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL, "[Httpd fd:%d] reading header", fd_);
  req_.reset();
  part_boundary_.assign("");
  part_filename_.assign("");
  part_type_.assign("");
  raw_header_.assign("");
  start_time_ = esp_timer_get_time();
  buf_.resize(header_read_size_);
  return read_repeated_with_timeout(&helper_, timeout_, fd_, buf_.data()
                                  , header_read_size_
                                  , STATE(parse_header_data));
}

StateFlowBase::Action HttpRequestFlow::parse_header_data()
{
  if (helper_.hasError_)
  {
    return call_immediately(STATE(abort_request));
  }
  else if (helper_.remaining_ == header_read_size_ || buf_.empty())
  {
    return yield_and_call(STATE(read_more_data));
  }

  raw_header_.append((char *)buf_.data()
                   , header_read_size_ - helper_.remaining_);

  // if we don't have an EOL string in the data yet, get more data
  if (raw_header_.find(HTML_EOL) == string::npos)
  {
    return yield_and_call(STATE(read_more_data));
  }

  // parse the data we have into delimited lines of header data, this will
  // leave some data in the raw_header_ which will need to be pushed back
  // into the buf_ after we reach the body segment.
  vector<string> lines;
  size_t parsed = tokenize(raw_header_, lines, HTML_EOL, false);

  // drop whatever has been tokenized so we don't process it again
  raw_header_.erase(0, parsed);

  // the first line is always the request line
  if (req_.raw_method().empty())
  {
    vector<string> request;
    tokenize(lines[0], request, " ");
    if (request.size() != 3)
    {
      LOG_ERROR("[Httpd fd:%d] Malformed request: %s.", fd_, lines[0].c_str());
      req_.set_status(HttpStatusCode::STATUS_BAD_REQUEST);
      return call_immediately(STATE(abort_request_with_response));
    }
    req_.method(request[0]);
    vector<string> uri_parts;
    tokenize(request[1], uri_parts, "?");
    req_.uri(uri_parts[0]);
    uri_parts.erase(uri_parts.begin());
    for (string &part : uri_parts)
    {
      vector<string> params;
      tokenize(part, params, "&");
      for (auto param : params)
      {
        req_.param(break_string(param, "="));
      }
    }
    // remove the first line since we processed it
    lines.erase(lines.begin());
  }

  // process any remaining lines as headers until we reach a blank line
  size_t processed_lines = 0;
  for (auto &line : lines)
  {
    processed_lines++;
    LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
      , "[Httpd fd:%d,uri:%s] line(%zu/%zu): ||%s||", fd_, req_.uri().c_str()
      , processed_lines, lines.size(), line.c_str());
    // check if we have reached a blank line, this is immediately after the
    // last header in the request.
    if (line.empty())
    {
      // Now that we have the request headers parsed we can check if the
      // request exceeds the size limits of the server.
      if (server_->is_request_too_large(&req_))
      {
        // If the request has the EXPECT header we can reject the request with
        // the EXPECTATION_FAILED (417) status, otherwise reject it with
        // BAD_REQUEST (400).
        req_.set_status(HttpStatusCode::STATUS_BAD_REQUEST);
        if (req_.has_header(HttpHeader::EXPECT))
        {
          req_.set_status(HttpStatusCode::STATUS_EXPECATION_FAILED);
        }
        LOG_ERROR("[Httpd fd:%d,uri:%s] Request body is too large, "
                  "aborting with status %d"
                , fd_, req_.uri().c_str(), res_->code_);
        return call_immediately(STATE(abort_request_with_response));
      }

      if (!server_->is_servicable_uri(&req_))
      {
        // check if it is a captive portal request
        if (server_->captive_active_ &&
            std::find_if(captive_portal_uris.begin(), captive_portal_uris.end()
                      , [&](const string &ent) {return !ent.compare(req_.uri());})
            != captive_portal_uris.end() && remote_ip_)
        {
          if (!server_->captive_auth_.count(remote_ip_) ||
              (server_->captive_auth_[remote_ip_] > server_->captive_timeout_ &&
              server_->captive_timeout_ != UINT32_MAX))
          {
            // new client or authentication expired, send the canned response
            res_.reset(new StringResponse(server_->captive_response_
                                        , MIME_TYPE_TEXT_HTML));
          }
          else if (req_.uri().find("_204") > 0 ||
                  req_.uri().find("status.php") > 0)
          {
            // These URIs require a generic response with code 204
            res_.reset(
              new AbstractHttpResponse(HttpStatusCode::STATUS_NO_CONTENT));
          }
          else if (req_.uri().find("ncsi.txt") > 0)
          {
            // Windows success page content
            res_.reset(
              new StringResponse("Microsoft NCSI", MIME_TYPE_TEXT_PLAIN));
          }
          else if (req_.uri().find("success.txt") > 0)
          {
            // Generic success.txt page content
            res_.reset(
              new StringResponse("success", MIME_TYPE_TEXT_PLAIN));
          }
          else
          {
            // iOS success page content
            res_.reset(
              new StringResponse("<HTML><HEAD><TITLE>Success</TITLE></HEAD>"
                                "<BODY>Success</BODY></HTML>"
                              , MIME_TYPE_TEXT_HTML));
          }
        }
        else if (server_->captive_active_ &&
                !server_->captive_auth_uri_.compare(req_.uri()))
        {
          server_->captive_auth_[remote_ip_] =
            esp_timer_get_time() + server_->captive_timeout_;
          res_.reset(new AbstractHttpResponse(HttpStatusCode::STATUS_OK));
        }
        else
        {
          LOG_ERROR("[Httpd fd:%d,uri:%s] Unknown URI, sending 404", fd_
                  , req_.uri().c_str());
          res_.reset(new UriNotFoundResponse(req_.uri()));
        }
        req_.error(true);
        return call_immediately(STATE(send_response_headers));
      }

      // Check if we have processed all parsed lines, if not add them back to
      // the buffer for deferred processing.
      if (processed_lines != lines.size())
      {
        LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
          , "[Httpd fd:%d,uri:%s] Processed %zu/%zu lines, adding remaining "
            "lines to body payload"
          , fd_, req_.uri().c_str(), processed_lines, lines.size());
        string unprocessed = string_join(lines.begin() + processed_lines
                                       , lines.end(), HTML_EOL) + raw_header_;
        raw_header_.assign(unprocessed);
      }

      return yield_and_call(STATE(process_request));
    }
    else
    {
      // it appears to be a header entry

      // split header into name/value pair
      auto h = break_string(line, ": ");

      // URL decode the header name and value
      h.first = url_decode(h.first);
      h.second = url_decode(h.second);

      // stash the header for later retrieval
      req_.header(h);
    }
  }

  return yield_and_call(STATE(read_more_data));
}

StateFlowBase::Action HttpRequestFlow::process_request()
{
  LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL, "[Httpd fd:%d] %s", fd_
    , req_.to_string().c_str());

  if (!req_.header(HttpHeader::UPGRADE).compare(HTTP_UPGRADE_HEADER_WEBSOCKET))
  {
    // upgrade to websocket!
    if (!req_.has_header(HttpHeader::WS_VERSION) ||
        !req_.has_header(HttpHeader::WS_KEY))
    {
      LOG_ERROR("[Httpd fd:%d,uri:%s] Missing required websocket header(s)\n%s"
              , fd_, req_.uri().c_str(), req_.to_string().c_str());
      req_.set_status(HttpStatusCode::STATUS_BAD_REQUEST);
      return call_immediately(STATE(abort_request_with_response));
    }
    return call_immediately(STATE(upgrade_to_websocket));
  }

  // if it is a PUT or POST and there is a content-length header we need to
  // read the body of the message in chunks and pass it off to the request
  // handler.
  if (req_.method() == HttpMethod::POST || req_.method() == HttpMethod::PUT)
  {
    // move the raw unparsed header bits back to the buffer
    if (!raw_header_.empty())
    {
      // resize for the reading of the body payload
      buf_.clear();
      buf_.reserve(body_read_size_);
      std::move(raw_header_.begin(), raw_header_.end(), std::back_inserter(buf_));
      raw_header_.assign("");
    }

    // If we do not have a Content-Length header outright reject the request
    // as there is no telling how big the payload is without reading it in
    // full.
    if (!req_.has_header(HttpHeader::CONTENT_LENGTH))
    {
      LOG_ERROR("[Httpd fd:%d] Request does not have the Content-Type "
                "header, aborting!\n%s", fd_, req_.to_string().c_str());
      req_.set_status(HttpStatusCode::STATUS_BAD_REQUEST);
      return call_immediately(STATE(abort_request_with_response));
    }

    // extract the body length from the content length header
    body_len_ = std::stoul(req_.header(HttpHeader::CONTENT_LENGTH));
    LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
      , "[Httpd fd:%d,uri:%s] body (header): %zu", fd_, req_.uri().c_str()
      , body_len_);

    if (req_.content_type() == ContentType::MULTIPART_FORMDATA)
    {
      // If we do not have a streaming handler for the URI abort the request.
      if (!server_->stream_handler(req_.uri()))
      {
        LOG_ERROR("[Httpd fd:%d] No streaming handler to receive payload, "
                  "aborting!", fd_);
        req_.set_status(HttpStatusCode::STATUS_SERVER_ERROR);
        return call_immediately(STATE(abort_request_with_response));
      }
      // cache the stream handler for this URI
      part_stream_ = server_->stream_handler(req_.uri());

      // force request to be concluded at end of processing
      req_.header(HttpHeader::CONNECTION, HTTP_CONNECTION_CLOSE);

      LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
        , "Converting to multipart/form-data req");
      return call_immediately(STATE(start_multipart_processing));
    }
    else if (req_.content_type() == ContentType::FORM_URLENCODED)
    {
      // convert the request body from form url-encoded to parameters and send
      // it for processing
      LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
        , "Converting to application/x-www-form-urlencoded req");
      return call_immediately(STATE(read_form_data));
    }
    else if (server_->stream_handler(req_.uri()))
    {
      // cache the stream handler for this URI
      part_stream_ = server_->stream_handler(req_.uri());

      // we have some of the body already read in, process it before requesting
      // more data
      if (!buf_.empty())
      {
        return yield_and_call(STATE(stream_body));
      }
      else
      {
        // read the payload and process it in chunks
        return read_repeated_with_timeout(&helper_, timeout_, fd_
                                        , buf_.data()
                                        , std::min(body_len_, body_read_size_)
                                        , STATE(stream_body));
      }
    }
    else if (req_.has_header(HttpHeader::CONTENT_LENGTH) &&
             std::stol(req_.header(HttpHeader::CONTENT_LENGTH)) > 0)
    {
      // the POST/PUT request has a payload but unrecognized Content-Type,
      // abort the request as we can't process it.
      LOG_ERROR("[Httpd fd:%d] Unrecognized Content-Type, aborting!\n%s", fd_
              , req_.to_string().c_str());
      req_.set_status(HttpStatusCode::STATUS_SERVER_ERROR);
      return call_immediately(STATE(abort_request_with_response));
    }
  }

  return yield_and_call(STATE(process_request_handler));
}

StateFlowBase::Action HttpRequestFlow::process_request_handler()
{
  if (server_->have_known_response(req_.uri()))
  {
    res_ = server_->response(&req_);
  }
  else
  {
    auto handler = server_->handler(req_.method(), req_.uri());
    auto res = handler(&req_);
    if (res && !res_)
    {
      res_.reset(res);
    }
  }

  return yield_and_call(STATE(send_response_headers));
}

StateFlowBase::Action HttpRequestFlow::read_more_data()
{
  // we need more data to parse the request
  LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
    , "[Httpd fd:%d] Requesting more data to process request", fd_);
  return read_repeated_with_timeout(&helper_, timeout_, fd_, buf_.data()
                              , header_read_size_, STATE(parse_header_data));
}

StateFlowBase::Action HttpRequestFlow::stream_body()
{
  if (helper_.hasError_)
  {
    return call_immediately(STATE(abort_request));
  }
  HASSERT(part_stream_);
  size_t data_len = body_read_size_ - helper_.remaining_;
  // if we received some data pass it on to the handler
  if (data_len)
  {
    bool abort_req = false;
    auto res = part_stream_(&req_, "", body_len_, buf_.data(), data_len
                          , body_offs_, (body_offs_ + data_len) >= body_len_
                          , &abort_req);
    body_offs_ += data_len;
    if (res && !res_)
    {
      res_.reset(res);
    }
    if (abort_req)
    {
      return call_immediately(STATE(send_response_headers));
    }
  }
  if (body_offs_ < body_len_)
  {
    size_t data_req = std::min(body_len_ - body_offs_, body_read_size_);
    return read_repeated_with_timeout(&helper_, timeout_, fd_, buf_.data()
                                    , data_req, STATE(stream_body));
  }
  return yield_and_call(STATE(send_response_headers));
}

StateFlowBase::Action HttpRequestFlow::start_multipart_processing()
{
  // check if the request has the "Expect: 100-continue" header. If it does
  // send the 100/continue line so the client starts streaming data.
  if (!req_.header(HttpHeader::EXPECT).compare("100-continue"))
  {
    LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL, "[Httpd fd:%d,uri:%s] Sending:%s", fd_
      , req_.uri().c_str(), multipart_res_.c_str());
    return write_repeated(&helper_, fd_, multipart_res_.c_str()
                        , multipart_res_.length()
                        , STATE(read_multipart_headers));
  }
  return call_immediately(STATE(read_multipart_headers));
}

extern std::map<HttpHeader, string> well_known_http_headers;
StateFlowBase::Action HttpRequestFlow::parse_multipart_headers()
{
  // https://tools.ietf.org/html/rfc7578
  // https://tools.ietf.org/html/rfc2388 (obsoleted by above)
  if (helper_.hasError_)
  {
    return call_immediately(STATE(abort_request));
  }
  else if (helper_.remaining_ == header_read_size_ && buf_.empty())
  {
    return yield_and_call(STATE(read_multipart_headers));
  }

  // parse the boundary marker on our first pass through the headers
  if (part_boundary_.empty())
  {
    string type = req_.header(HttpHeader::CONTENT_TYPE);
    if (type.find('=') == string::npos)
    {
      LOG_ERROR("[Httpd fd:%d] Unable to find multipart/form-data bounary "
                "marker, giving up:\n%s", fd_, req_.to_string().c_str());
      req_.set_status(HttpStatusCode::STATUS_BAD_REQUEST);
      return call_immediately(STATE(abort_request_with_response));
    }
    part_boundary_.assign(type.substr(type.find_last_of('=') + 1));
    LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
      , "[Httpd fd:%d] multipart/form-data boundary: %s (%zu)"
      , fd_, part_boundary_.c_str(), part_boundary_.length());
    part_count_ = 0;
  }

  // add any data that has been received to the parse buffer
  size_t bytes_received = header_read_size_ - helper_.remaining_;
  raw_header_.append((char *)buf_.data(), bytes_received);
  buf_.clear();
  
  // if we don't have an EOL string in the data yet, get more data
  if (raw_header_.find(HTML_EOL) == string::npos)
  {
    if (raw_header_.length() < config_httpd_max_header_size())
    {
      LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
        , "[Httpd fd:%d] no EOL character yet, reading more data: %zu\n%s"
        , fd_, raw_header_.size(), raw_header_.c_str());
      return yield_and_call(STATE(read_multipart_headers));
    }
    LOG_ERROR("[Httpd fd:%d] Received %zu bytes without being able to parse "
              "headers, aborting.", fd_, raw_header_.length());
    LOG_ERROR("request:%s", req_.to_string().c_str());
    LOG_ERROR(raw_header_.c_str());
    req_.set_status(HttpStatusCode::STATUS_BAD_REQUEST);
    return call_immediately(STATE(abort_request_with_response));
  }

  // parse the data we have into delimited lines of header data, this will
  // leave some data in the raw_header_ which will need to be pushed back
  // into the buf_ after we reach the body segment.
  vector<string> lines;
  size_t parsed = tokenize(raw_header_, lines, HTML_EOL, false);
  // drop whatever has been tokenized so we don't process it again
  LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
    , "[Httpd fd:%d,uri:%s] body: %zu, parsed: %zu, header: %zu"
    , fd_, req_.uri().c_str(), body_len_, parsed, raw_header_.length());
  raw_header_.erase(0, parsed);

  // reduce the body size by the amount of data we have successfully parsed.
  body_len_ -= parsed;

  LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
    , "[Httpd fd:%d,uri:%s] parsed: %zu, body: %zu", fd_, req_.uri().c_str()
    , parsed, body_len_);

  // process any remaining lines as headers until we reach a blank line
  int processed_lines = 0;
  for (auto &line : lines)
  {
    processed_lines++;
    LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
      , "[Httpd fd:%d,uri:%s] line(%zu/%zu): ||%s||"
      , fd_, req_.uri().c_str(), processed_lines, lines.size(), line.c_str());
    if (line.empty())
    {
      if (found_part_boundary_)
      {
        LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
          , "[Httpd fd:%d,uri:%s] found blank line, starting body stream"
          , fd_, req_.uri().c_str());
        process_part_body_ = true;
        // Check if we have processed all parsed lines, if not add them back to
        // the buffer for deferred processing.
        if (processed_lines != lines.size())
        {
          LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
            , "[Httpd fd:%d,uri:%s] Processed %zu/%zu lines, adding remaining "
              "lines to body payload"
            , fd_, req_.uri().c_str(), processed_lines, lines.size());
          // add the unprocessed lines back to the buffer
          raw_header_.insert(0, string_join(lines.begin() + processed_lines
                                          , lines.end(), HTML_EOL));
        }
        break;
      }
      else
      {
        LOG_ERROR("[Httpd fd:%d,uri:%s] Missing part boundary marker, aborting "
                  "request", fd_, req_.uri().c_str());
        req_.set_status(HttpStatusCode::STATUS_BAD_REQUEST);
        return call_immediately(STATE(abort_request_with_response));
      }
    }
    else if (line.find(part_boundary_) != string::npos)
    {
      // skip the first two bytes as the line will be: --<boundary>
      if (found_part_boundary_)
      {
        LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
          , "[Httpd fd:%d,uri:%s] End of multipart/form-data segment(%d)"
          , fd_, req_.uri().c_str(), part_count_);
        found_part_boundary_ = false;
        if (line.find_last_of("--") == line.length() - 2)
        {
          LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
            , "[Httpd fd:%d,uri:%s] End of last segment reached"
            , fd_, req_.uri().c_str());
          return yield_and_call(STATE(send_response_headers));
        }
      }
      else
      {
        LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
          , "[Httpd fd:%d,uri:%s] Start of multipart/form-data segment(%d)"
          , fd_, req_.uri().c_str(), part_count_);
        found_part_boundary_ = true;
        // reset part specific data
        part_len_ = 0;
        part_offs_ = 0;
        part_count_++;
        part_filename_.assign("");
        part_type_.assign("text/plain");
      }
    }
    else if (found_part_boundary_)
    {
      std::pair<string, string> parts = break_string(line, ": ");
      if (!parts.first.compare(
            well_known_http_headers[HttpHeader::CONTENT_TYPE]))
      {
        part_type_.assign(parts.second);
        LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
          , "[Httpd fd:%d,uri:%s] multipart/form-data segment(%d) uses %s:%s"
          , fd_, req_.uri().c_str(), part_count_
          , well_known_http_headers[HttpHeader::CONTENT_TYPE].c_str()
          , part_type_.c_str());
      }
      else if (!parts.first.compare(
                  well_known_http_headers[HttpHeader::CONTENT_DISPOSITION]))
      {
        // extract filename if present
        vector<string> disposition;
        tokenize(parts.second, disposition, "; ");
        for (auto &part : disposition)
        {
          if (part.find("filename") != string::npos)
          {
            vector<string> file_parts;
            tokenize(part, file_parts, "\"");
            part_filename_.assign(file_parts[1]);
            LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
              , "[Httpd fd:%d,uri:%s] multipart/form-data segment(%d) "
                "filename detected as '%s'"
              , fd_, req_.uri().c_str(), part_count_, part_filename_.c_str());
          }
          else if (part.find("form-data") != string::npos)
          {
            // ignored field
          }
          else if (part.find("name") != string::npos)
          {
            // ignored field (for now)
          }
          else
          {
            LOG_ERROR(
              "[Httpd fd:%d,uri:%s] Unrecognized field in segment(%d): %s"
              , fd_, req_.uri().c_str(), part_count_, part.c_str());
          }
        }
      }
      else
      {
        LOG_ERROR("[Httpd fd:%d,uri:%s] Received unexpected header in segment, "
                  "aborting\n%s", fd_, req_.uri().c_str(), line.c_str());
        req_.set_status(HttpStatusCode::STATUS_BAD_REQUEST);
        return call_immediately(STATE(abort_request_with_response));
      }
    }
  }
  if (process_part_body_)
  {
    // if there was some data leftover from the parsing of the headers,
    // transfer it back to the pending buffer and start streaming it.
    if (!raw_header_.empty())
    {
      buf_.clear();
      std::move(raw_header_.begin(), raw_header_.end(), std::back_inserter(buf_));
      LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
        , "[Httpd fd:%d,uri:%s] segment(%d) size %zu/%zu"
        , fd_, req_.uri().c_str(), part_count_, buf_.size(), body_len_);
      body_len_ += buf_.size();
      raw_header_.assign("");
    }

    // size is expanded to account for "--" and "\r\n", default to expecting a
    // single part. If there is leftover body_len_ after we process this chunk
    // we will return to this state to get the next part.
    part_len_ = body_len_ - (part_boundary_.size() + 4);
    LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
      , "[Httpd fd:%d,uri:%s] segment(%d) size %zu/%zu"
      , fd_, req_.uri().c_str(), part_count_, part_len_, body_len_);
    size_t data_req = std::min(part_len_, body_read_size_ - buf_.size());
    
    LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
      , "[Httpd fd:%d,uri:%s] Requesting %zu/%zu bytes for segment(%d)"
      , fd_, req_.uri().c_str(), data_req, part_len_, part_count_);
    // read the payload and process it in chunks
    return read_repeated_with_timeout(&helper_, timeout_, fd_
                                    , buf_.data() + buf_.size(), data_req
                                    , STATE(stream_multipart_body));
  }
  return yield_and_call(STATE(read_multipart_headers));
}

StateFlowBase::Action HttpRequestFlow::read_multipart_headers()
{
  // If we have data in the buffer already and it is more than the size of
  // what we are trying to read, call directly into the parser to clear the
  // data.
  if (!buf_.empty() && buf_.size() >= header_read_size_)
  {
    return call_immediately(STATE(parse_multipart_headers));
  }

  // We either have no data or less bytes of data than header_read_size_.
  // Request enough data for at least header_read_size_ number of bytes.
  LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
    , "[Httpd fd:%d,uri:%s] requesting %zu bytes for multipart/form-data "
      "processing", fd_, req_.uri().c_str(), header_read_size_ - buf_.size());
  return read_repeated_with_timeout(&helper_, timeout_, fd_
                                  , buf_.data() + buf_.size()
                                  , header_read_size_ - buf_.size()
                                  , STATE(parse_multipart_headers));
}

StateFlowBase::Action HttpRequestFlow::stream_multipart_body()
{
  if (helper_.hasError_)
  {
    return call_immediately(STATE(abort_request));
  }
  HASSERT(part_stream_);
  size_t data_len = body_read_size_ - helper_.remaining_;
  if (data_len)
  {
    if (body_len_ > data_len)
    {
      body_len_ -= data_len;
    }
    else
    {
      body_len_ = 0;
    }
    LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
      , "[Httpd fd:%d,uri:%s] Received %zu/%zu bytes", fd_, req_.uri().c_str()
      , part_offs_, part_len_);
    bool abort_req = false;
    auto res = part_stream_(&req_, part_filename_, part_len_, buf_.data()
                          , data_len, part_offs_
                          , (part_offs_ + data_len) >= part_len_, &abort_req);
    part_offs_ += data_len;
    if (res && !res_)
    {
      res_.reset(res);
    }
    if (abort_req)
    {
      return yield_and_call(STATE(send_response_headers));
    }
  }
  // if we didn't receive any data or we need to read more data to reach the
  // end of the part, try to retrieve more data.
  if (part_offs_ < part_len_)
  {
    size_t data_req = std::min(part_len_ - part_offs_, body_read_size_);
    LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
      , "[Httpd fd:%d,uri:%s] Requesting %zu bytes", fd_, req_.uri().c_str()
      , data_req);
    return read_repeated_with_timeout(&helper_, timeout_, fd_, buf_.data()
                                    , data_req, STATE(stream_multipart_body));
  }
  if (body_len_)
  {
    return yield_and_call(STATE(read_multipart_headers));
  }

  return yield_and_call(STATE(send_response_headers));
}

StateFlowBase::Action HttpRequestFlow::read_form_data()
{
  // Request enough data for at least header_read_size_ number of bytes.
  LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
    , "[Httpd fd:%d,uri:%s] requesting %zu bytes for form-data processing", fd_
    , req_.uri().c_str(), header_read_size_ - buf_.size());
  return read_repeated_with_timeout(&helper_, timeout_, fd_
                                  , buf_.data() + buf_.size()
                                  , header_read_size_ - buf_.size()
                                  , STATE(parse_form_data));
}

StateFlowBase::Action HttpRequestFlow::parse_form_data()
{
  if (helper_.hasError_)
  {
    return call_immediately(STATE(abort_request));
  }
  else if (helper_.remaining_ == header_read_size_ || buf_.empty())
  {
    return yield_and_call(STATE(read_form_data));
  }

  raw_header_.append((char *)buf_.data()
                   , header_read_size_ - helper_.remaining_);
  buf_.clear();

  LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL, "body: %zu, received: %zu", body_len_
    , (header_read_size_ - helper_.remaining_));

  // track how much we have read in of the body payload
  if (body_len_ > (header_read_size_ - helper_.remaining_))
  {
    body_len_ -= (header_read_size_ - helper_.remaining_);
  }
  else
  {
    body_len_ = 0;
  }

  vector<string> params;
  size_t parsed = tokenize(raw_header_, params, "&", body_len_ == 0);

  // drop whatever has been tokenized so we don't process it again
  raw_header_.erase(0, parsed);

  for (auto param : params)
  {
    auto p = break_string(param, "=");

    // URL decode the parameter name and value
    p.first = url_decode(p.first);
    p.second = url_decode(p.second);

    LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
      , "param: %s, value: %s", p.first.c_str(), p.second.c_str());
    // add the parameter to the request
    req_.param(p);
  }

  // If there is more body payload to read request more data before processing
  // the request fully.
  if (body_len_)
  {
    size_t data_req = std::min(body_len_, header_read_size_);
    LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
      , "[Httpd fd:%d,uri:%s] Requesting %zu bytes", fd_, req_.uri().c_str()
      , data_req);
    return read_repeated_with_timeout(&helper_, timeout_, fd_, buf_.data()
                                    , data_req, STATE(parse_form_data));
  }

  return yield_and_call(STATE(process_request_handler));
}

StateFlowBase::Action HttpRequestFlow::send_response_headers()
{
  // when there is no response object created, default to the status code from
  // the request.
  if (!res_)
  {
    LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
      , "[Httpd fd:%d,uri:%s] no response body, creating one with status:%d"
      , fd_, req_.uri().c_str(), req_.status_);
    res_.reset(new AbstractHttpResponse(req_.status_));
  }
  size_t len = 0;
  bool keep_alive = req_.keep_alive() &&
                    req_count_ < config_httpd_max_req_per_connection();
  uint8_t *payload = res_->get_headers(&len, keep_alive);
  LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
    , "[Httpd fd:%d,uri:%s] Sending headers using %zu bytes (%d)."
    , fd_, req_.uri().c_str(), len, res_->code_);
  return write_repeated(&helper_, fd_, payload, len, STATE(send_response_body));
}

StateFlowBase::Action HttpRequestFlow::send_response_body()
{
  if (req_.method() == HttpMethod::HEAD)
  {
    LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
      , "[Httpd fd:%d,uri:%s] HEAD request, no body required.", fd_
      , req_.uri().c_str());
  }
  else if (res_->get_body_length())
  {
    LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
      , "[Httpd fd:%d,uri:%s] Sending body of %d bytes.", fd_
      , req_.uri().c_str(), res_->get_body_length());
    // check if we can send the entire response in one call or not.
    if (res_->get_body_length() > config_httpd_response_chunk_size())
    {
      LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
        , "[Httpd fd:%d,uri:%s] Converting to streamed response.", fd_
        , req_.uri().c_str());
      response_body_offs_ = 0;
      LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
        , "[Httpd fd:%d,uri:%s] Sending [%d-%d/%d]", fd_
        , req_.uri().c_str(), response_body_offs_
        , config_httpd_response_chunk_size(), res_->get_body_length());
      return write_repeated(&helper_, fd_, res_->get_body()
                          , config_httpd_response_chunk_size()
                          , STATE(send_response_body_split));
    }
    return write_repeated(&helper_, fd_, res_->get_body()
                        , res_->get_body_length(), STATE(request_complete));
  }
  return yield_and_call(STATE(request_complete));
}

StateFlowBase::Action HttpRequestFlow::send_response_body_split()
{
  // check if there has been an error and abort if needed
  if (helper_.hasError_)
  {
    return yield_and_call(STATE(abort_request));
  }
  response_body_offs_ += (config_httpd_response_chunk_size() -
                          helper_.remaining_);
  if (response_body_offs_ >= res_->get_body_length())
  {
    // the body has been sent fully
    return yield_and_call(STATE(request_complete));
  }
  size_t remaining = res_->get_body_length() - response_body_offs_;
  if (remaining > config_httpd_response_chunk_size())
  {
    remaining = config_httpd_response_chunk_size();
  }
  LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
    , "[Httpd fd:%d,uri:%s] Sending [%d-%d/%d]", fd_, req_.uri().c_str()
    , response_body_offs_, response_body_offs_ + remaining
    , res_->get_body_length());
  return write_repeated(&helper_, fd_, res_->get_body() + response_body_offs_
                      , remaining, STATE(send_response_body_split));
}

StateFlowBase::Action HttpRequestFlow::request_complete()
{
#if CONFIG_HTTP_REQ_FLOW_LOG_LEVEL == VERBOSE
  if (!req_.uri().empty())
  {
    uint32_t proc_time = USEC_TO_MSEC(esp_timer_get_time() - start_time_);
    if (res_->get_body_length())
    {
      LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
        , "[Httpd fd:%d,uri:%s] Processed in %d ms (%d, %zu bytes).", fd_
        , req_.uri().c_str(), proc_time, res_->code_, res_->get_body_length());
    }
    else
    {
      LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
        , "[Httpd fd:%d,uri:%s] Processed in %d ms (%d).", fd_
        , req_.uri().c_str(), proc_time, res_->code_);
    }
  }
#endif // CONFIG_HTTP_REQ_FLOW_LOG_LEVEL == VERBOSE
  req_count_++;
  // If the connection setting is not keep-alive, or there was an error during
  // processing, or we have processed more than the configured number of
  // requests, or the URI was empty (parse failure?), or the result code is a
  // redirect shutdown the socket immediately. FireFox will not follow the
  // redirect request if the connection is kept open.
  if (!req_.keep_alive() || req_.error() ||
      req_count_ >= config_httpd_max_req_per_connection() ||
      req_.uri().empty() ||
      res_->code_ == HttpStatusCode::STATUS_FOUND ||
      res_->code_ == HttpStatusCode::STATUS_MOVED_PERMANENTLY)
  {
    req_.reset();
    server_->schedule_cleanup(this);
    return exit();
  }

  return call_immediately(STATE(start_request));
}

StateFlowBase::Action HttpRequestFlow::upgrade_to_websocket()
{
  // keep the socket open since we will reuse it as the websocket
  close_ = false;
  LOG(CONFIG_HTTP_REQ_FLOW_LOG_LEVEL
    , "[Httpd fd:%d,uri:%s] Upgrading to WebSocket", fd_, req_.uri().c_str());
  new WebSocketFlow(server_, fd_, remote_ip_, req_.header(HttpHeader::WS_KEY)
                  , req_.header(HttpHeader::WS_VERSION)
                  , server_->ws_handler(req_.uri()));
  req_.reset();
  server_->schedule_cleanup(this);
  return exit();
}

StateFlowBase::Action HttpRequestFlow::abort_request_with_response()
{
  req_.error(true);
  return call_immediately(STATE(send_response_headers));
}

StateFlowBase::Action HttpRequestFlow::abort_request()
{
  req_.error(true);
  return call_immediately(STATE(request_complete));
}

} // namespace http

