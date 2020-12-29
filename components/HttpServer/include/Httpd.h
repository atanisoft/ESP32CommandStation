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
 * \file Httpd.h
 *
 * Main include file for all HttpServer declarations.
 *
 * @author Mike Dunston
 * @date 13 Sept 2019
 */

/// @file HTTP server with Captive Portal support.

#ifndef HTTPD_H_
#define HTTPD_H_

#include <algorithm>
#include <map>
#include <mutex>
#include <stdint.h>

#include <executor/Service.hxx>
#include <executor/StateFlow.hxx>

#include <os/MDNS.hxx>
#include <os/OS.hxx>
#include <utils/Base64.hxx>
#include <utils/constants.hxx>
#include <utils/format_utils.hxx>
#include <utils/Singleton.hxx>
#include <utils/socket_listener.hxx>
#include <utils/StringPrintf.hxx>
#include <utils/Uninitialized.hxx>

#include "Dnsd.h"

/// Namespace for HTTP related functionality.
namespace http
{

/// FreeRTOS task stack size for the httpd Executor.
DECLARE_CONST(httpd_server_stack_size);

/// FreeRTOS task priority for the httpd Executor.
DECLARE_CONST(httpd_server_priority);

/// Max number of bytes to read in a single chunk when reading the HTTP request
/// headers.
DECLARE_CONST(httpd_header_chunk_size);

/// Max number of bytes to read in a single chunk when reading the HTTP request
/// body payload.
DECLARE_CONST(httpd_body_chunk_size);

/// Max number of bytes to write in a single chunk when sending the HTTP
/// response to the client.
DECLARE_CONST(httpd_response_chunk_size);

/// Maximum size of the HTTP headers before which the request will be aborted
/// with a BAD_REQUEST (400) error code.
DECLARE_CONST(httpd_max_header_size);

/// This controls how many headers will be allowed in the http request before
/// ignoring new ones. Default is 20.
DECLARE_CONST(httpd_max_header_count);

/// This controls how many request parameters will be allowed before ignroing
/// new ones. Default is 20.
DECLARE_CONST(httpd_max_param_count);

/// Maximum size of the HTTP request body payload. Any request which exceeds
/// this limit will be forcibly aborted.
DECLARE_CONST(httpd_max_req_size);

/// This is the maximum number of HTTP requests which should be processed for a
/// single connection with keep-alive active before the connection will be
/// closed with the "Connection: close" header.
DECLARE_CONST(httpd_max_req_per_connection);

/// This is the maximum wait time for receiving a single chunk of data from the
/// HTTP request.
DECLARE_CONST(httpd_req_timeout_ms);

/// This is the number of milliseconds to use for the socket send and receive
/// timeouts.
DECLARE_CONST(httpd_socket_timeout_ms);

/// This is the number of milliseconds to use as the read timeout for all
/// websocket connections. When this limit and the max_XX_attempts limit have
/// been exceeded the send/receive attempt is aborted and the inverse operation
/// will be attempted.
DECLARE_CONST(httpd_websocket_timeout_ms);

/// This is the maximum data size to send/receive in a single operation. If a
/// websocket frame is received exceeding this limit it will be processed in
/// chunks of this size.
DECLARE_CONST(httpd_websocket_max_frame_size);

/// This controls how many attempts will be allowed to receive a websocket
/// frame before attempting to send out a websocket frame.
DECLARE_CONST(httpd_websocket_max_read_attempts);

/// This controls how many websocket URIs that can be registered at one time.
/// Default is 1.
DECLARE_CONST(httpd_websocket_max_uris);

/// This controls how many websocket clients can connect at one time.
/// Default is 10.
DECLARE_CONST(httpd_websocket_max_clients);

/// This controls how many pending text frames will be cached before discarding
/// additional frames.
/// Default is 20.
DECLARE_CONST(httpd_websocket_max_pending_frames);

/// This controls the Cache-Control: max-age=XXX value in the response headers
/// for static content.
DECLARE_CONST(httpd_cache_max_age_sec);

/// Commonly used HTTP status codes.
/// @enum HttpStatusCode
enum HttpStatusCode
{
  STATUS_CONTINUE=100,
  STATUS_SWITCH_PROTOCOL=101,

  STATUS_OK=200,
  STATUS_CREATED=201,
  STATUS_ACCEPTED=202,
  STATUS_NON_AUTH_INFO=203,
  STATUS_NO_CONTENT=204,
  STATUS_RESET_CONTENT=205,
  STATUS_PARTIAL_CONTENT=206,

  STATUS_MULTIPLE_CHOICES=300,
  STATUS_MOVED_PERMANENTLY=301,
  STATUS_FOUND=302,
  STATUS_SEE_OTHER=303,
  STATUS_NOT_MODIFIED=304,
  STATUS_USE_PROXY=305,
  STATUS_TEMP_REDIRECT=307,

  STATUS_BAD_REQUEST=400,
  STATUS_NOT_AUTHORIZED=401,
  STATUS_PAYMENT_REQUIRED=402,
  STATUS_FORBIDDEN=403,
  STATUS_NOT_FOUND=404,
  STATUS_NOT_ALLOWED=405,
  STATUS_NOT_ACCEPTABLE=406,
  STATUS_PROXY_AUTH_REQ=407,
  STATUS_TIMEOUT=408,
  STATUS_CONFLICT=409,
  STATUS_GONE=410,
  STATUS_LENGTH_REQ=411,
  STATUS_PRECOND_FAIL=412,
  STATUS_ENTITY_TOO_LARGE=413,
  STATUS_URI_TOO_LARGE=414,
  STATUS_UNSUPPORTED_MEDIA_TYPE=415,
  STATUS_RANGE_NOT_SATISFIABLE=416,
  STATUS_EXPECATION_FAILED=417,

  STATUS_SERVER_ERROR=500,
  STATUS_NOT_IMPLEMENTED=501,
  STATUS_BAD_GATEWAY=502,
  STATUS_SERVICE_UNAVAILABLE=503,
  STATUS_GATEWAY_TIMEOUT=504,
  STATUS_HTTP_VERSION_UNSUPPORTED=505
};

/// Default HTTP listener port
static constexpr uint16_t DEFAULT_HTTP_PORT = 80;

/// Commonly used and well-known HTTP methods.
enum HttpMethod
{
  /// Request is for deleting a resource.
  DELETE          = BIT(1),
  /// Request is for retrieving a resource.
  GET             = BIT(2),
  /// Request is for retrieving the headers for a resource. 
  HEAD            = BIT(3),
  /// Request is for creating a resource.
  POST            = BIT(4),
  /// Request is for patching an existing resource.
  PATCH           = BIT(5),
  /// Request is for applying an update to an existing resource.
  PUT             = BIT(6),
  /// Request type was not understood by the server.
  UNKNOWN_METHOD  = BIT(7),
};

/// Commonly used and well-known HTTP headers
enum HttpHeader
{
  ACCEPT,
  CACHE_CONTROL,
  CONNECTION,
  CONTENT_ENCODING,
  CONTENT_TYPE,
  CONTENT_LENGTH,
  CONTENT_DISPOSITION,
  EXPECT,
  HOST,
  IF_MODIFIED_SINCE,
  LAST_MODIFIED,
  LOCATION,
  ORIGIN,
  UPGRADE,
  WS_VERSION,
  WS_KEY,
  WS_ACCEPT,
};

/// Commonly used and well-known values for the Content-Type HTTP header.
///
/// These are currently only used for POST and PUT HTTP requests. All other
/// types will receive a stream of the body of the HTTP request.
enum ContentType
{
  MULTIPART_FORMDATA,
  FORM_URLENCODED,
  UNKNOWN_TYPE
};

/// WebSocket events for the @ref WebSocketHandler callback.
typedef enum
{
  /// A new Websocket connection has been established.
  WS_EVENT_CONNECT,
  
  /// A WebSocket connection has been closed.
  WS_EVENT_DISCONNECT,

  /// A TEXT message has been received from a WebSocket. Note that it may be
  /// sent to the handler in pieces.
  WS_EVENT_TEXT,
  
  /// A BINARY message has been received from a WebSocket. Note that it may be
  /// sent to the handler in pieces.
  WS_EVENT_BINARY
} WebSocketEvent;

// Values for Cache-Control
// TODO: introduce enum constants for these
static constexpr const char * HTTP_CACHE_CONTROL_NO_CACHE = "no-cache";
static constexpr const char * HTTP_CACHE_CONTROL_NO_STORE = "no-store";
static constexpr const char * HTTP_CACHE_CONTROL_NO_TRANSFORM = "no-transform";
static constexpr const char * HTTP_CACHE_CONTROL_MAX_AGE = "max-age";
static constexpr const char * HTTP_CACHE_CONTROL_PUBLIC = "public";
static constexpr const char * HTTP_CACHE_CONTROL_PRIVATE = "private";
static constexpr const char * HTTP_CACHE_CONTROL_MUST_REVALIDATE = "must-revalidate";
static constexpr const char * HTTP_CACHE_CONTROL_IMMUTABLE = "immutable";

// Values for Connection header
// TODO: introduce enum constants for these
static constexpr const char * HTTP_CONNECTION_CLOSE = "close";
static constexpr const char * HTTP_CONNECTION_KEEP_ALIVE = "keep-alive";
static constexpr const char * HTTP_CONNECTION_UPGRADE = "Upgrade";

// TODO: introduce enum constant for this value
static constexpr const char * HTTP_UPGRADE_HEADER_WEBSOCKET = "websocket";

// HTTP end of line characters
static constexpr const char * HTML_EOL = "\r\n";

// Common mime-types
// TODO: introduce enum constants for these
static constexpr const char * MIME_TYPE_NONE = "";
static constexpr const char * MIME_TYPE_TEXT_CSS = "text/css";
static constexpr const char * MIME_TYPE_TEXT_HTML = "text/html";
static constexpr const char * MIME_TYPE_TEXT_JAVASCRIPT = "text/javascript";
static constexpr const char * MIME_TYPE_TEXT_PLAIN = "text/plain";
static constexpr const char * MIME_TYPE_TEXT_XML = "text/XML";
static constexpr const char * MIME_TYPE_IMAGE_GIF = "image/gif";
static constexpr const char * MIME_TYPE_IMAGE_PNG = "image/png";
static constexpr const char * MIME_TYPE_APPLICATION_JSON = "application/json";
static constexpr const char * MIME_TYPE_OCTET_STREAM = "application/octet-stream";

// TODO: introduce enum constants for these
static constexpr const char * HTTP_ENCODING_NONE = "";
static constexpr const char * HTTP_ENCODING_GZIP = "gzip";

/// Forward declaration of the WebSocketFlow so it can access internal methods
/// of various classes.
class WebSocketFlow;

/// Forward declaration of the HttpdRequestFlow so it can access internal
/// methods of various classes.
class HttpRequestFlow;

/// Forward declaration of the Httpd so it can access internal methods of
/// various classes.
class Httpd;

/// This is the base class for an HTTP response.
class AbstractHttpResponse
{
public:
  /// Constructor.
  ///
  /// @param code is the @ref HttpStatusCode to use for the
  /// response.
  /// @param mime_type is the mime type to send as part of the Content-Type
  /// header.
  AbstractHttpResponse(HttpStatusCode code=STATUS_NOT_FOUND
                     , const std::string &mime_type=MIME_TYPE_TEXT_PLAIN);

  /// Destructor.
  ~AbstractHttpResponse();

  /// Encodes the HTTP response headers for transmission to the client.
  ///
  /// @param len is the length of the headers after encoding. This is an
  /// OUTPUT parameter.
  /// @param keep_alive is used to control the "Connection: [keep-alive|close]"
  /// header.
  /// @param add_keep_alive is used to control if the Connection header will be
  /// included in the response.
  ///
  /// @return a buffer containing the HTTP response. This buffer is owned by
  /// the @ref AbstractHttpResponse and does not need to be
  /// freed by the caller.
  uint8_t *get_headers(size_t *len, bool keep_alive=false
                     , bool add_keep_alive=true);

  /// Encodes the HTTP response into a string which can be printed. This is
  /// used by @ref get_headers internally with include_body set to false.
  ///
  /// @param include_body will include the body of the response as well as any
  /// headers.
  /// @param keep_alive is used to control the "Connection: [keep-alive|close]"
  /// header.
  /// @param add_keep_alive is used to control if the Connection header will be
  /// included in the response.
  ///
  /// @return the request as a printable string.
  std::string to_string(bool include_body=false, bool keep_alive=false
                      , bool add_keep_alive=true);

  /// @return the response body as a buffer which can be streamed. This buffer
  /// is owned by the @ref AbstractHttpResponse and does not
  /// need to be freed by the caller.
  ///
  /// Note: this method should be overriden by sub-classes to supply the
  /// response body.
  virtual const uint8_t *get_body()
  {
    return nullptr;
  }

  /// @return the size of the body payload.
  ///
  /// Note: this method should be overriden by sub-classes to supply the
  /// response body.
  virtual size_t get_body_length()
  {
    return 0;
  }

  /// @return the mime type to include in the HTTP response header.
  ///
  /// Note: this method should be overriden by sub-classes to supply the
  /// response body.
  std::string get_body_mime_type()
  {
    return mime_type_;
  }

protected:
  /// Adds an arbitrary HTTP header to the response object.
  ///
  /// @param name is the HTTP header name to add.
  /// @param value is the HTTP header value to add.
  void header(const std::string &name, const std::string &value);

  /// Adds a well-known HTTP header to the response object.
  ///
  /// @param name is the HTTP header name to add.
  /// @param value is the HTTP header value to add.
  void header(const HttpHeader name, const std::string &value);

private:
  /// Collection of headers and values for this HTTP response.
  std::map<std::string, std::string> headers_;

  /// @ref HttpStatusCode for this HTTP response.
  HttpStatusCode code_;

  /// Content-Type header value.
  std::string mime_type_;

  /// Temporary storage for the HTTP response headers in an encoded format
  /// that is ready for transmission to the client.
  std::string encoded_headers_;

  /// Gives @ref WebSocketFlow access to protected/private
  /// members.
  friend class WebSocketFlow;

  /// Gives @ref HttpRequestFlow access to protected/private
  /// members.
  friend class HttpRequestFlow;
};

/// HTTP Response object for URI not found.
class UriNotFoundResponse : public AbstractHttpResponse
{
public:
  /// Constructor.
  ///
  /// @param uri is the URI that was requested that is not known by the server.
  UriNotFoundResponse(const std::string &uri)
    : body_(StringPrintf("URI %s was not found.", uri.c_str()))
  {
  }

  /// @return the pre-formatted body of this response.
  const uint8_t *get_body() override
  {
    return (const uint8_t *)(body_.c_str());
  }

  /// @return the size of the pre-formatted body of this response.
  size_t get_body_length() override
  {
    return body_.length();
  }

private:
  /// Temporary storage for the response body.
  const std::string body_;
};

/// HTTP Response object used to redirect the client to a different location
/// via the HTTP Header: "Location: uri".
class RedirectResponse : public AbstractHttpResponse
{
public:
  /// Constructor.
  ///
  /// @param target_url is the target which the client should be redirected to.
  RedirectResponse(const std::string &target_url);
};

/// HTTP Response object which is used to return a static payload to a client
/// when the URI is accessed.
///
/// Note: The payload must remain alive as long as the URI is referenced by the
/// @ref Httpd server.
class StaticResponse : public AbstractHttpResponse
{
public:
  /// Constructor.
  ///
  /// @param payload is the body of the response to send.
  /// @param length is the length of the body payload.
  /// @param mime_type is the value to send in the Content-Type HTTP header.
  /// @param encoding is the optional encoding to send in the Content-Encoding
  /// HTTP Header.
  /// @param cached allows control of the Cache-Control header, when true the
  /// response will be cached by the client and when false it will not.
  StaticResponse(const uint8_t *payload, const size_t length
               , const std::string mime_type
               , const std::string encoding = HTTP_ENCODING_NONE
               , bool cached = true);

  /// @return the pre-formatted body of this response.
  const uint8_t *get_body() override
  {
    return payload_;
  }

  /// @return the size of the pre-formatted body of this response.
  size_t get_body_length() override
  {
    return length_;
  }

private:
  /// Pointer to the payload to return for this URI.
  const uint8_t *payload_;

  /// Length of the payload to return for this URI.
  const size_t length_;
};

/// HTTP Response object which can be used to return a string based response to
/// a given URI.
class StringResponse : public AbstractHttpResponse
{
public:
  /// Constructor.
  ///
  /// @param response is the response string to send as the HTTP response body.
  /// @param mime_type is the value to use for the Content-Type HTTP header.
  ///
  /// Note: The ownership of the response object passed into this method will
  /// be transfered to this class instance and will be cleaned up after it has
  /// been sent to the client.
  StringResponse(const std::string &response, const std::string &mime_type);

  /// @return the pre-formatted body of this response.
  const uint8_t *get_body() override
  {
    return (uint8_t *)response_.c_str();
  }

  /// @return the size of the pre-formatted body of this response.
  size_t get_body_length() override
  {
    return response_.length();
  }

private:
  /// Temporary storage of the response payload.
  std::string response_;
};

/// Specialized @ref StringResponse type for json payloads.
class JsonResponse : public StringResponse
{
public:
  /// Constructor.
  ///
  /// @param response is the response string to send as the HTTP response body.
  ///
  /// This calls into @ref StringResponse passing in
  /// @ref MIME_TYPE_APPLICATION_JSON as mime_type.
  ///
  /// Note: The ownership of the response object passed into this method will
  /// be transfered to this class instance and will be cleaned up after it has
  /// been sent to the client.
  JsonResponse(const std::string &response)
  : StringResponse(response, MIME_TYPE_APPLICATION_JSON)
  {
  }
};

/// Runtime state of an HTTP Request.
class HttpRequest
{
public:
  /// @return the parsed well-known @ref HttpMethod.
  HttpMethod method();

  /// @return the unparsed HTTP method.
  const std::string &raw_method();

  /// @return the URI that this request is for.
  const std::string &uri();

  /// @return true if the named HTTP Header exists.
  /// @param name of the parameter to check for.
  bool has_header(const std::string &name);

  /// @return true if the well-known @ref HttpHeader exists.
  bool has_header(const HttpHeader name);

  /// @return the value of the named HTTP Header or a blank string if it does
  /// not exist.
  const std::string &header(const std::string name);

  /// @return the value of the well-known @ref HttpHeader or a blank string if
  /// it does not exist.
  const std::string &header(const HttpHeader name);

  /// @return true if the well-known @ref HttpHeader::CONNECTION header exists
  /// with a value of "keep-alive".
  bool keep_alive();

  /// @return true if the request could not be parsed successfully.
  bool error();

  /// @return the parsed @ref ContentType for this request.
  ContentType content_type();

  /// Sets the request response status code.
  ///
  /// @param code is the @ref HttpStatusCode to respond with upon conclusion of
  /// this @ref HttpRequest.
  ///
  /// This can be used by the @ref RequestProcessor to set the response code
  /// when no response body is required.
  void set_status(HttpStatusCode code);

  /// @return number of parameters to this @ref HttpRequest.
  size_t params();

  /// @return parameter value or a blank string if parameter is not present.
  /// @param name is the name of the parameter to return.
  std::string param(std::string name);

  /// @return parameter value or default value is parameter is not present.
  /// @param name is the name of the parameter to return as a boolean value.
  /// @param def is the default value to return if the parameter does not exist
  /// or is otherwise non-convertable to a boolean.
  bool param(std::string name, bool def);

  /// @return parameter value or default value is parameter is not present.
  /// @param name is the name of the parameter to return as an integer value.
  /// @param def is the default value to return if the parameter does not exist
  /// or is otherwise non-convertable to an integer.
  int param(std::string name, int def);

  /// @return true if the parameter is present.
  /// @param name is the name of the parameter to return.
  bool has_param(std::string name);

  /// @return string form of the request, this is headers only.
  std::string to_string();

private:
  /// Gives @ref HttpRequestFlow access to protected/private members.
  friend class HttpRequestFlow;

  /// Sets the @ref HttpMethod if it is well-known, otherwise only the unparsed
  /// value will be available.
  ///
  /// @param value is the raw value parsed from the first line of the HTTP
  /// request stream.
  void method(const std::string &value);

  /// Sets the URI of the HttpRequest.
  ///
  /// @param value is the value of the URI.
  void uri(const std::string &value);

  /// Adds a URI parameter to the request.
  ///
  /// @param name is the name of the parameter.
  /// @param value is the value of the parameter.
  void param(const string &name, const string &value);

  /// Adds an HTTP Header to the request.
  ///
  /// @param name is the name of the header.
  /// @param value is the value of the header.
  void header(const string &name, const string &value);

  /// Adds/replaces a HTTP Header to the request.
  ///
  /// @param header is the @ref HttpHeader to add/replace.
  /// @param value is the value for the header.
  void header(HttpHeader header, std::string value);

  /// Resets the internal state of the @ref HttpRequest to defaults so it can
  /// be reused for subsequent requests.
  void reset();

  /// Sets/Resets the parse error flag.
  ///
  /// @param value should be true if there is a parse failure, false otherwise.
  void error(bool value);

  /// default return value when a requested header or parameter is not known.
  const std::string no_value_{""};

  /// Collection of HTTP Headers that have been parsed from the HTTP request
  /// stream.
  std::map<std::string, std::string> headers_;

  /// Collection of parameters supplied with the HTTP Request after the URI.
  std::map<std::string, std::string> params_;

  /// Parsed @ref HttpMethod for this @ref HttpRequest.
  HttpMethod method_;
  
  /// Raw unparsed HTTP method for this @ref HttpRequest.
  std::string raw_method_;
  
  /// Parsed URI of this @ref HttpRequest.
  std::string uri_;

  /// Parse error flag.
  bool error_;

  /// @ref HttpStatusCode to return by default when this @ref HttpRequest has
  /// completed and there is no @ref AbstractHttpResponse to return.
  HttpStatusCode status_{HttpStatusCode::STATUS_SERVER_ERROR};
};

/// URI processing handler that will be invoked for requests which have no body
/// payload to stream. Currently the only requests which have a body payload 
///
/// When this function is invoked it has the option to return a pointer to a
/// @ref AbstractHttpResponse which will be sent to the client or it can call
/// @ref HttpRequest::set_status if no response body is required.
typedef std::function<
  http::AbstractHttpResponse *(http::HttpRequest * /** request*/)> RequestProcessor;

#define HTTP_HANDLER(name) \
http::AbstractHttpResponse * name (http::HttpRequest *);

#define HTTP_HANDLER_IMPL(name, request) \
http::AbstractHttpResponse * name (http::HttpRequest * request)

/// URI processing handler which will be invoked for POST/PUT requests that
/// have a body payload.
///
/// When this function is invoked the "abort" parameter can be set to true and
/// the request will be aborted immediately and an error returned to the
/// client. The function has the same option of calling
/// @ref HttpRequest::set_status or returning a pointer to a
/// @ref AbstractHttpResponse.
typedef std::function<
  http::AbstractHttpResponse *(http::HttpRequest * /** request */
                             , const std::string & /** filename*/
                             , size_t              /** size    */
                             , const uint8_t *     /** data    */
                             , size_t              /** length  */
                             , size_t              /** offset  */
                             , bool                /** final   */
                             , bool *              /** abort   */
                             )> StreamProcessor;

#define HTTP_STREAM_HANDLER(name)                                             \
http::AbstractHttpResponse * name (http::HttpRequest *request                 \
                             , const std::string &filename, size_t size       \
                             , const uint8_t *data, size_t length             \
                             , size_t offset, bool final, bool *abort)

#define HTTP_STREAM_HANDLER_IMPL(name, request, filename, size, data, length  \
                               , offset, final, abort)                        \
http::AbstractHttpResponse * name (http::HttpRequest * request                \
                                 , const std::string & filename, size_t size  \
                                 , const uint8_t * data, size_t length        \
                                 , size_t offset, bool final, bool * abort)

/// WebSocket processing Handler.
///
/// This method will be invoked when there is an event to be processed.
///
/// When @ref WebSocketEvent is @ref WebSocketEvent::WS_EVENT_CONNECT or
/// @ref WebSocketEvent::WS_EVENT_DISCONNECT data will be
/// nullptr and data length will be zero.
/// 
/// When @ref WebSocketEvent is @ref WebSocketEvent::WS_EVENT_TEXT or
/// @ref WebSocketEvent::WS_EVENT_BINARY the data parameter
/// will be a buffer of data length bytes of text or binary data to be
/// processed by the handler.
///
/// The handler can invoke the @ref WebSocketFlow parameter to retrieve
/// additional details about the WebSocket client, queue response text or
/// binary data for delivery at next available opportunity, or request the
/// WebSocket connection to be closed.
typedef std::function<void(http::WebSocketFlow *  /* websocket */
                         , http::WebSocketEvent   /* event */
                         , uint8_t *              /* data */
                         , size_t                 /* data length */
                         )> WebSocketHandler;

#define WEBSOCKET_STREAM_HANDLER(name)                                      \
void name (http::WebSocketFlow *, http::WebSocketEvent, const uint8_t *     \
         , size_t);

#define WEBSOCKET_STREAM_HANDLER_IMPL(name, websocket, event, data, length) \
void name (http::WebSocketFlow * websocket, http::WebSocketEvent event      \
         , const uint8_t * data, size_t length)

/// HTTP Server implementation
class Httpd : public Service, public Singleton<Httpd>
{
public:
  /// Constructor.
  ///
  /// @param mdns is the @ref MDNS instance to use for publishing mDNS records
  /// when the server is active. This is disabled by default.
  /// @param port is the port to listen for HTTP requests on, default is 80.
  /// @param name is the name to use for the executor, default is "httpd".
  /// @param service_name is the mDNS service name to advertise when the server
  /// is active, default is _http._tcp.
  Httpd(MDNS *mdns = nullptr, uint16_t port = DEFAULT_HTTP_PORT
      , const std::string &name = "httpd"
      , const std::string service_name = "_http._tcp");

  /// Constructor.
  ///
  /// @param executor is the @ref Executor to use for all http requests.
  /// @param mdns is the @ref MDNS instance to use for publishing mDNS records
  /// when the server is active. This is disabled by default.
  /// @param port is the port to listen for HTTP requests on, default is 80.
  /// @param name is the name to use for the executor, default is "httpd".
  /// @param service_name is the mDNS service name to advertise when the server
  /// is active, default is _http._tcp.
  Httpd(ExecutorBase *executor, MDNS *mdns = nullptr
      , uint16_t port = DEFAULT_HTTP_PORT, const std::string &name = "httpd"
      , const std::string service_name = "_http._tcp");

  /// Destructor.
  ~Httpd();

  /// Registers a URI with the provided handler.
  ///
  /// @param uri is the URI to call the provided handler for.
  /// @param method_mask is the @ref HttpMethod for this URI, when multiple
  /// @ref HttpMethod values are required they must be ORed together.
  /// @param handler is the @ref RequestProcessor to invoke when this URI is
  /// requested. When there is a request body and the method is POST/PUT this
  /// function will not be invoked.
  /// @param stream_handler is the @ref StreamProcessor to invoke when this URI
  /// is requested as POST/PUT and the request has a body payload.
  void uri(const std::string &uri, const size_t method_mask
         , RequestProcessor handler
         , StreamProcessor stream_handler = nullptr);

  /// Registers a URI with the provided handler that can process all
  /// @ref HttpMethod values. Note that any request with a body payload will
  /// result in an error being returned to the client for this URI since there
  /// is no stream handler defined by this method.
  ///
  /// @param uri is the URI to call the provided handler for.
  /// @param handler is the @ref RequestProcessor to invoke when this URI is
  /// requested. When there is a request body and the method is POST/PUT this
  /// function will not be invoked.
  void uri(const std::string &uri, RequestProcessor handler);

  /// Registers a URI to redirect to another location.
  ///
  /// @param source is the URI which should trigger the redirect.
  /// @param target is where the request should be routed instead.
  ///
  /// Note: This will result in the client receiving an HTTP 302 response
  /// with an updated Location value provided.
  void redirect_uri(const std::string &source, const std::string &target);

  /// Registers a static response URI.
  ///
  /// @param uri is the URI to serve the static content for.
  /// @param content is the content to send back to the client when this uri is
  /// requested. Note that large content blocks will be broken into smaller
  /// pieces for transmission to the client and thus must remain in memory.
  /// @param length is the length of the content to send to the client.
  /// @param mime_type is the Content-Type parameter to return to the client.
  /// @param encoding is the encoding for the content, if not specified the
  /// Content-Encoding header will not be transmitted.
  /// @param cached allows control of the Cache-Control header, when true the
  /// response will be cached by the client and when false it will not.
  void static_uri(const std::string &uri, const uint8_t *content
                , const size_t length, const std::string &mime_type
                , const std::string &encoding = HTTP_ENCODING_NONE
                , bool cached = true);

  /// Registers a WebSocket handler for a given URI.  ///
  /// @param uri is the URI to process as a WebSocket endpoint.
  /// @param handler is the @ref WebSocketHandler to invoke when this URI is
  /// requested.
  void websocket_uri(const std::string &uri, WebSocketHandler handler);

  /// Sends a binary message to a single WebSocket.
  ///
  /// @param id is the ID of the WebSocket to send the data to.
  /// @param data is the binary data to send to the websocket client.
  /// @param length is the length of the binary data to send to the websocket
  /// client.
  ///
  /// Note: this is currently unimplemented.
  void send_websocket_binary(int id, uint8_t *data, size_t length);

  /// Sends a text message to a single WebSocket.
  /// 
  /// @param id is the ID of the WebSocket to send the text to.
  /// @param text is the text to send to the WebSocket client.
  void send_websocket_text(int id, std::string &text);

  /// Broadcasts a text message to all connected WebSocket clients.
  ///
  /// @param text is the text to send to all WebSocket clients.
  void broadcast_websocket_text(std::string &text);

  /// Creates a new @ref HttpRequestFlow for the provided socket handle.
  ///
  /// @param fd is the socket handle.
  void new_connection(int fd);

  /// Enables processing of known captive portal endpoints.
  ///
  /// @param first_access_response is the HTML response to send upon first
  /// access from a client.
  /// @param auth_uri is the callback URI that the client should access to
  /// bypass the captive portal redirection. Note this URI will be processed
  /// internally by @ref Httpd before invoking the @ref RequestProcessor. If
  /// there is no @ref RequestProcessor for this URI a redirect to / will be
  /// sent instead.
  /// @param auth_timeout is the number of seconds to cache the source IP
  /// address for a client before forcing a re-authentication to occur. A value
  /// of UINT32_MAX will disable the timeout.
  ///
  /// The following URIs will be processed as captive portal endpoints and
  /// force redirect to the captive portal when accessed and the source IP
  /// has not previously accessed the auth_uri:
  /// | URI | Environment |
  /// | --- | ----------- |
  /// | /generate_204 | Android |
  /// | /gen_204 | Android 9.0 |
  /// | /mobile/status.php | Android 8.0 (Samsung s9+) |
  /// | /ncsi.txt | Windows |
  /// | /success.txt | OSX / FireFox |
  /// | /hotspot-detect.html | iOS 8/9 |
  /// | /hotspotdetect.html | iOS 8/9 |
  /// | /library/test/success.html | iOS 8/9 |
  /// | /kindle-wifi/wifiredirect.html | Kindle |
  /// | /kindle-wifi/wifistub.html | Kindle |
  ///
  void captive_portal(std::string first_access_response
                    , std::string auth_uri = "/captiveauth"
                    , uint64_t auth_timeout = UINT32_MAX);

private:
  /// Gives @ref WebSocketFlow access to protected/private members.
  friend class WebSocketFlow;

  /// Gives @ref HttpRequestFlow access to protected/private members.
  friend class HttpRequestFlow;

  /// Initializes the server.
  void init_server();

  /// Schedules the Executable to be cleaned up in an asynchronous fashion.
  void schedule_cleanup(Executable *flow);

  /// Starts the HTTP socket listener.
  void start_http_listener();

  /// Starts the DNS socket listener.
  ///
  /// @param ip is the IP address to redirect all DNS requests to. This should
  /// be in HOST native order, the DNS server will convert to network order.
  void start_dns_listener(uint32_t ip);

  /// Stops the HTTP socket listener (if active).
  void stop_http_listener();

  /// Stops the DNS socket listener (if active).
  void stop_dns_listener();

  /// Registers a new @ref WebSocketFlow with the server to allow sending
  /// text or binary messages based on the WebSocket ID.
  ///
  /// @param id is the ID of the WebSocket client.
  /// @param ws is the @ref WebSocketFlow managing the WebSocket client.
  /// @return true if the websocket was recorded, false otherwise.
  bool add_websocket(int id, WebSocketFlow *ws);

  /// Removes a previously registered WebSocket client.
  ///
  /// @param id of the WebSocket client to remove.
  void remove_websocket(int id);

  /// @return the @ref RequestProcessor for a URI.
  /// @param uri is the URI to retrieve the @ref RequestProcessor for.
  RequestProcessor handler(HttpMethod method, const std::string &uri);

  /// @return the @ref StreamProcessor for a URI.
  /// @param uri is the URI to retrieve the @ref StreamProcessor for.
  StreamProcessor stream_handler(const std::string &uri);

  /// @return the @ref WebSocketHandler for the provided URI.
  /// @param uri is the URI to retrieve the @ref WebSocketHandler for.
  WebSocketHandler ws_handler(const std::string &uri);

  /// @return true if there is a @ref AbstractHttpResponse for the URI.
  /// @param uri is the URI to check.
  bool have_known_response(const std::string &uri);

  /// @return the @ref AbstractHttpResponse for the @param request. This will
  /// evaluate the request for static_uri and redirect registered endpoints.
  ///
  /// For a static_uri endpoint the request headers will be evaluated for the
  /// presence of @ref HttpHeader::IF_MODIFIED_SINCE and will return either the
  /// requested resource or a @ref AbstractHttpResponse with
  /// @ref HttpStatusCode::STATUS_NOT_MODIFIED as the code if the resource has
  /// not been modified.
  std::shared_ptr<AbstractHttpResponse> response(HttpRequest *request);

  /// @return true if the @param request is too large to be processed. Size
  /// is configured via httpd_max_req_size.
  bool is_request_too_large(HttpRequest *request);

  /// @return true if the @param request can be serviced by this @ref Httpd.
  bool is_servicable_uri(HttpRequest *request);

  /// Name to use for the @ref Httpd server.
  const std::string name_;

  /// @ref MDNS instance to use for publishing mDNS records when the @ref Httpd
  /// server is active;
  MDNS *mdns_;

  /// mDNS service name to advertise when the @ref Httpd Server is running.
  const std::string mdns_service_;

  /// @ref Executor that manages all @ref StateFlow for the @ref Httpd server.
  Executor<1> executor_;

  /// Internal flag to indicate if this class owns the executor or if it is
  /// externally managed.
  bool externalExecutor_{false};

  /// TCP/IP port to listen for HTTP requests on.
  uint16_t port_;

  /// @ref SocketListener that will accept() the socket connections and call
  /// the @ref Httpd when a new client is available.
  uninitialized<SocketListener> listener_;

  /// @ref Dnsd that will serve DNS responses when enabled.
  uninitialized<Dnsd> dns_;

  /// Internal state flag for the listener_ being active.
  bool http_active_{false};

  /// Internal state flag for the dns_ being active.
  bool dns_active_{false};

  /// Internal map of all registered @ref RequestProcessor handlers.
  std::map<std::string, std::pair<size_t, RequestProcessor>> handlers_;

  /// Internal map of all registered @ref RequestProcessor handlers.
  std::map<std::string, StreamProcessor> stream_handlers_;

  /// Internal map of all registered static URIs to use when the client does
  /// not specify the @ref HttpHeader::IF_MODIFIED_SINCE or the value is not
  /// the current version.
  std::map<std::string, std::shared_ptr<AbstractHttpResponse>> static_uris_;

  /// Internal map of all registeres static URIs to use when resource has not
  /// been modified since the client last retrieved it.
  std::map<std::string, std::shared_ptr<AbstractHttpResponse>> static_cached_;

  /// Internal map of all redirected URIs.
  std::map<std::string, std::shared_ptr<AbstractHttpResponse>> redirect_uris_;

  /// Internal map of all registered @ref WebSocketHandler URIs.
  std::map<std::string, WebSocketHandler> websocket_uris_;

  /// Internal map of active @ref WebSocketFlow instances.
  std::map<int, WebSocketFlow *> websockets_;

  /// Lock object for websockets_.
  std::mutex websocketsLock_;

  /// Captive portal response for HTTP 204 NO CONTENT.
  std::shared_ptr<AbstractHttpResponse> captive_no_content_;

  /// Captive portal response for HTTP 200 OK.
  std::shared_ptr<AbstractHttpResponse> captive_ok_;

  /// Captive portal response for Microsoft NCSI.
  std::shared_ptr<AbstractHttpResponse> captive_msft_ncsi_;

  /// Captive portal response for generic success.txt.
  std::shared_ptr<AbstractHttpResponse> captive_success_;

  /// Captive portal response for iOS devices.
  std::shared_ptr<AbstractHttpResponse> captive_success_ios_;

  /// Captive portal response for first access.
  std::shared_ptr<AbstractHttpResponse> captive_response_;

  /// Captive portal authentication URL.
  std::string captive_auth_uri_;

  /// Timeout for captive portal authenticated clients.
  uint32_t captive_timeout_{UINT32_MAX};

  /// Internal state flag for the captive portal.
  bool captive_active_{false};

  /// Tracking entries for authenticated captive portal clients.
  std::map<uint32_t, uint64_t> captive_auth_;

  /// timeval to use for newly connected sockets for SO_RCVTIMEO and
  /// SO_SNDTIMEO.
  struct timeval socket_timeout_;

  DISALLOW_COPY_AND_ASSIGN(Httpd);
};

/// HTTP Request parser that implements the @ref StateFlowBase interface.
class HttpRequestFlow : private StateFlowBase
{
public:
  /// Constructor.
  ///
  /// @param server is the @ref Httpd server owning this request.
  /// @param fd is the socket handle.
  /// @param remote_ip is the remote IP address of the client.
  HttpRequestFlow(Httpd *server, int fd, uint32_t remote_ip);

  /// Destructor.
  ~HttpRequestFlow();

private:
  /// @ref StateFlowTimedSelectHelper which assists in reading/writing of the
  /// request data stream.
  StateFlowTimedSelectHelper helper_{this};

  /// Timeout value to use for reading data while processing the request.
  const long long timeout_{MSEC_TO_NSEC(config_httpd_req_timeout_ms())};

  /// Maximum read size for a single read() call when processing the headers.
  const size_t header_read_size_{(size_t)config_httpd_header_chunk_size()};

  /// Maximum read size for a single read() call when processing the request
  /// body.
  const size_t body_read_size_{(size_t)config_httpd_body_chunk_size()};

  /// @ref Httpd instance that owns this request.
  Httpd *server_;

  /// Underlying socket handle for this request.
  int fd_;

  /// Remote client IP (if known).
  uint32_t remote_ip_;

  /// @ref HttpRequest data holder.
  HttpRequest req_;

  /// Flag to indicate that the underlying socket handle should be closed when
  /// this @ref HttpRequestFlow is deleted. In the case of a WebSocket the
  /// socket needs to be preserved.
  bool close_{true};

  /// Temporary buffer used for reading the HTTP request.
  std::vector<uint8_t> buf_;

  /// Index into @ref buf_ for partial reads.
  size_t body_offs_;
  
  /// Total size of the request body.
  size_t body_len_;

  /// Temporary accumulator for the HTTP header data as it is being parsed.
  std::string raw_header_;

  /// @ref AbstractHttpResponse that represents the response to this request.
  std::shared_ptr<AbstractHttpResponse> res_;

  /// Index into the response body payload.
  size_t response_body_offs_{0};

  /// Request start time.
  uint64_t start_time_;

  /// Current request number for this client connection.
  uint8_t req_count_{0};

  /// Count of multipart/form-data segments that have been encountered and
  /// processing started.
  size_t part_count_{0};

  /// Temporary storage of multipart encoded form data boundary.
  std::string part_boundary_;

  /// Internal flag to indicate we have found the boundary marker for a
  /// multipart/form-data encoded POST/PUT body payload.
  bool found_part_boundary_{false};

  /// Internal flag to indicate we should process the body segment of a
  /// multipart/form-data encoded POST/PUT body payload.
  bool process_part_body_{false};

  /// Temporary storage of the filename for the multipart/form-data part.
  std::string part_filename_;

  /// Temporary storage of the content-type for a multipart/form-data part.
  std::string part_type_;

  /// Temporary storage of the length for a multipart/form-data part.
  size_t part_len_{0};

  /// Index into @ref buf_ for partial reads for a multipart/form-data part.
  size_t part_offs_{0};

  /// Temporary holder of the @ref StreamProcessor to avoid subsequent lookups
  /// during streaming of the parts.
  StreamProcessor part_stream_{nullptr};

  /// Response to send when a PUT/POST of multipart/form-data is received this
  /// needs to be sent before the client will send the content to be processed.
  std::string multipart_res_{"HTTP/1.1 100 Continue\r\n\r\n"};

  STATE_FLOW_STATE(start_request);
  STATE_FLOW_STATE(read_more_data);
  STATE_FLOW_STATE(parse_header_data);
  STATE_FLOW_STATE(process_request);
  STATE_FLOW_STATE(process_request_handler);
  STATE_FLOW_STATE(stream_body);
  STATE_FLOW_STATE(start_multipart_processing);
  STATE_FLOW_STATE(parse_multipart_headers);
  STATE_FLOW_STATE(read_multipart_headers);
  STATE_FLOW_STATE(stream_multipart_body);
  STATE_FLOW_STATE(read_form_data);
  STATE_FLOW_STATE(parse_form_data);
  STATE_FLOW_STATE(send_response);
  STATE_FLOW_STATE(send_response_headers);
  STATE_FLOW_STATE(send_response_body);
  STATE_FLOW_STATE(send_response_body_split);
  STATE_FLOW_STATE(request_complete);
  STATE_FLOW_STATE(upgrade_to_websocket);
  STATE_FLOW_STATE(abort_request_with_response);
  STATE_FLOW_STATE(abort_request);
};

/// WebSocket processor implementing the @ref StateFlowBase interface.
class WebSocketFlow : private StateFlowBase
{
public:
  /// Constructor.
  ///
  /// @param server is the @ref Httpd server owning this WebSocket.
  /// @param fd is the socket handle.
  /// @param remote_ip is the remote IP address (if known).
  /// @param ws_key is the "Sec-WebSocket-Key" HTTP Header from the initial
  /// request.
  /// @param ws_version is the "Sec-WebSocket-Version" HTTP header from the
  /// initial request.
  /// @param handler is the @ref WebSocketHandler that will process the events
  /// as they are raised.
  WebSocketFlow(Httpd *server, int fd, uint32_t remote_ip
              , const std::string &ws_key, const std::string &ws_version
              , WebSocketHandler handler);

  /// Destructor.
  ~WebSocketFlow();

  /// Sends text to this WebSocket at the next possible interval.
  ///
  /// @param text is the text to send.
  void send_text(std::string &text);

  /// @return the ID of the WebSocket.
  int id();

  /// @return the IP address of the remote side of the WebSocket.
  ///
  /// Note: This may return zero in which case the remote IP address is not
  /// known.
  uint32_t ip();

  /// This will trigger an orderly shutdown of the WebSocket at the next
  /// opportunity. This will trigger the @ref WebSocketHandler with the
  /// @ref WebSocketEvent set to @ref WebSocketEvent::WS_EVENT_DISCONNECT.
  void request_close();

private:
  /// @ref StateFlowTimedSelectHelper which assists in reading/writing of the
  /// request data stream.
  StateFlowTimedSelectHelper helper_{this};

  /// @ref Httpd instance that owns this request.
  Httpd *server_;

  /// Underlying socket handle for this request.
  int fd_;
  
  /// Remote client IP (if known).
  uint32_t remote_ip_;

  /// WebSocket read/write timeout for a data frame.
  const uint64_t timeout_;

  /// Maximum size to read/write of a frame in one call.
  const uint64_t max_frame_size_;

  /// Temporary buffer used for reading/writing WebSocket frame data.
  uint8_t *data_;

  /// Size of the used data in the temporary buffer.
  size_t data_size_;
  
  /// Temporary holder for the WebSocket handshake response.
  std::string handshake_;

  /// @ref WebSocketHandler to be invoked when a @ref WebSocketEvent needs to be
  /// processed.
  WebSocketHandler handler_;

  /// internal frame header data
  uint16_t header_;

  /// Parsed op code from the header data.
  uint8_t opcode_;

  /// When true the frame data is XOR masked with a 32bit XOR mask.
  bool masked_;

  /// internal flag indicating the frame length type.
  uint8_t frameLenType_;

  /// Temporary holder for 16bit frame length data.
  uint16_t frameLength16_;

  /// Length of the WebSocket Frame data.
  uint64_t frameLength_;

  /// 32bit XOR mask to apply to the data when @ref masked_ is true.
  uint32_t maskingKey_;

  /// Lock for the @ref textFrames_.
  OSMutex textLock_;

  /// Text frames that are pending delivery.
  std::vector<std::string> textFrames_;

  /// When set to true the @ref WebSocketFlow will attempt to shutdown the
  /// WebSocket connection at it's next opportunity.
  bool close_requested_{false};

  /// Flag to indicate that the @ref WebSocketHandler has been invoked for
  /// connect and the disconnect event should be sent.
  bool connect_event_sent_{false};

  // helper for fully reading a data block with a timeout so we can close the
  // socket if there are too many timeout errors
  uint8_t *buf_{nullptr};
  size_t buf_size_;
  size_t buf_offs_;
  size_t buf_remain_;
  uint8_t buf_attempts_;
  Callback buf_next_;
  Callback buf_next_timeout_;
  Action read_fully_with_timeout(void *buf, size_t size, size_t attempts
                               , Callback success, Callback timeout);
  STATE_FLOW_STATE(data_received);

  /// Internal state flow for reading a websocket frame of data and sending
  /// back a response, possibly broken into chunks.
  STATE_FLOW_STATE(send_handshake);
  STATE_FLOW_STATE(handshake_sent);
  STATE_FLOW_STATE(read_frame_header);
  STATE_FLOW_STATE(frame_header_received);
  STATE_FLOW_STATE(frame_data_len_received);
  STATE_FLOW_STATE(start_recv_frame_data);
  STATE_FLOW_STATE(recv_frame_data);
  STATE_FLOW_STATE(shutdown_connection);
  STATE_FLOW_STATE(send_frame_header);
  STATE_FLOW_STATE(frame_sent);
};

} // namespace http

#endif // HTTPD_H_
