/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2021 Mike Dunston

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

#include "sdkconfig.h"
#include "ESP32TrainDatabase.h"

#include <AllTrainNodes.hxx>
#include <cJSON.h>
#include <DCCppProtocol.h>
#include <DCCProgrammer.h>
#include <dcc/Loco.hxx>
#include <Dnsd.h>
#include <DCCSignalVFS.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <FileSystemManager.h>
#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>
#include <Httpd.h>
#include <JsonConstants.h>
#include <LCCStackManager.h>
#include <LCCWiFiManager.h>
#include <mutex>
#include <nvs.hxx>
#include <openlcb/MemoryConfigClient.hxx>
#include <Turnouts.h>
#include <utils/FileUtils.hxx>
#include <utils/SocketClientParams.hxx>
#include <utils/StringPrintf.hxx>
#include "OTAMonitor.h"

#if CONFIG_GPIO_OUTPUTS
#include <Outputs.h>
#endif // CONFIG_GPIO_OUTPUTS

#if CONFIG_GPIO_SENSORS
#include <Sensors.h>
#include <RemoteSensors.h>
#if CONFIG_GPIO_S88
#include <S88Sensors.h>
#endif // CONFIG_GPIO_S88
#endif // CONFIG_GPIO_SENSORS

using commandstation::AllTrainNodes;
using dcc::SpeedType;
using http::Httpd;
using http::HttpMethod;
using http::HttpRequest;
using http::HttpStatusCode;
using http::AbstractHttpResponse;
using http::StringResponse;
using http::JsonResponse;
using http::WebSocketFlow;
using http::MIME_TYPE_TEXT_HTML;
using http::MIME_TYPE_TEXT_JAVASCRIPT;
using http::MIME_TYPE_TEXT_PLAIN;
using http::MIME_TYPE_TEXT_XML;
using http::MIME_TYPE_TEXT_CSS;
using http::MIME_TYPE_IMAGE_PNG;
using http::MIME_TYPE_IMAGE_GIF;
using http::HTTP_ENCODING_GZIP;
using http::HTTP_ENCODING_NONE;
using http::WebSocketEvent;
using http::WebSocketFlow;
using openlcb::DatagramClient;
using openlcb::Defs;
using openlcb::MemoryConfigClient;
using openlcb::MemoryConfigClientRequest;
using openlcb::MemoryConfigDefs;
using openlcb::NodeHandle;

class WebSocketClient : public DCCPPProtocolConsumer
{
public:
  WebSocketClient(int clientID, uint32_t remoteIP)
    : DCCPPProtocolConsumer(), _id(clientID), _remoteIP(remoteIP)
  {
    LOG(INFO, "[WS %s] Connected", name().c_str());
  }
  virtual ~WebSocketClient()
  {
    LOG(INFO, "[WS %s] Disconnected", name().c_str());
  }
  int id()
  {
    return _id;
  }
  std::string name()
  {
    return StringPrintf("%s/%d", ipv4_to_string(_remoteIP).c_str(), _id);
  }
private:
  uint32_t _id;
  uint32_t _remoteIP;
};

// Captive Portal landing page
static constexpr const char * const CAPTIVE_PORTAL_HTML = R"!^!(
<html>
 <head>
  <title>ESP32 Command Station v%s</title>
  <meta http-equiv="refresh" content="30;url='/captiveauth'" />
 </head>
 <body>
  <h1>Welcome to the ESP32 Command Station</h1>
  <h2>Open your browser and navigate to any website and it will open the ESP32 Command Station</h2>
  <p>Click <a href="/captiveauth">here</a> to close this portal page if it does not automatically close.</p>
 </body>
</html>)!^!";

std::mutex webSocketLock;
std::vector<std::unique_ptr<WebSocketClient>> webSocketClients;
WEBSOCKET_STREAM_HANDLER(process_ws);
WEBSOCKET_STREAM_HANDLER(process_wsjson);
HTTP_STREAM_HANDLER(process_ota);
HTTP_HANDLER(process_power);
HTTP_HANDLER(process_config);
HTTP_HANDLER(process_prog);
HTTP_HANDLER(process_turnouts);
HTTP_HANDLER(process_loco);
HTTP_HANDLER(process_outputs);
HTTP_HANDLER(process_sensors);
HTTP_HANDLER(process_remote_sensors);
HTTP_HANDLER(process_s88);

extern const uint8_t indexHtmlGz[] asm("_binary_index_html_gz_start");
extern const size_t indexHtmlGz_size asm("index_html_gz_length");

extern const uint8_t loco32x32[] asm("_binary_loco_32x32_png_start");
extern const size_t loco32x32_size asm("loco_32x32_png_length");

extern const uint8_t cashJsGz[] asm("_binary_cash_min_js_gz_start");
extern const size_t cashJsGz_size asm("cash_min_js_gz_length");

extern const uint8_t spectreCssGz[] asm("_binary_spectre_min_css_gz_start");
extern const size_t spectreCssGz_size asm("spectre_min_css_gz_length");

extern const uint8_t normalizeCssGz[] asm("_binary_normalize_min_css_gz_start");
extern const size_t normalizeCssGz_size asm("normalize_min_css_gz_length");

namespace openlcb
{
  extern const size_t CDI_SIZE;
}

struct CDIClientRequest : public CallableFlowRequestBase
{
  enum ReadCmd
  {
    READ
  };
  
  enum WriteCmd
  {
    WRITE
  };

  enum UpdateCompleteCmd
  {
      UPDATE_COMPLETE
  };

  void reset(ReadCmd, NodeHandle target_node, WebSocketFlow *socket
           , uint32_t req_id, size_t offs, size_t size, string target
           , string type)
  {
    reset_base();
    cmd = CMD_READ;
    this->target_node = target_node;
    this->socket = socket;
    this->req_id = req_id;
    this->offs = offs;
    this->size = size;
    this->target = target;
    this->type = type;
    value.clear();
  }

  void reset(WriteCmd, NodeHandle target_node, WebSocketFlow *socket
           , uint32_t req_id, size_t offs, size_t size, string target
           , string value)
  {
    reset_base();
    cmd = CMD_WRITE;
    this->target_node = target_node;
    this->socket = socket;
    this->req_id = req_id;
    this->offs = offs;
    this->size = size;
    this->target = target;
    type.clear();
    this->value = std::move(value);
  }

  void reset(UpdateCompleteCmd, NodeHandle target_node, WebSocketFlow *socket
           , uint32_t req_id)
  {
    reset_base();
    cmd = CMD_UPDATE_COMPLETE;
    this->target_node = target_node;
    this->socket = socket;
    this->req_id = req_id;
    type.clear();
    value.clear();
  }

  enum Command : uint8_t
  {
      CMD_READ,
      CMD_WRITE,
      CMD_UPDATE_COMPLETE
  };

  Command cmd;
  WebSocketFlow *socket;
  NodeHandle target_node;
  uint32_t req_id;
  size_t offs;
  size_t size;
  string target;
  string type;
  string value;
};

class CDIClient : public CallableFlow<CDIClientRequest>
{
public:
  CDIClient(Service *service, MemoryConfigClient *memory_client)
          : CallableFlow<CDIClientRequest>(service), client_(memory_client)
  {
  }

private:
  MemoryConfigClient *client_;

  StateFlowBase::Action entry() override
  {
    request()->resultCode = DatagramClient::OPERATION_PENDING;
    switch (request()->cmd)
    {
      case CDIClientRequest::CMD_READ:
        LOG(VERBOSE
          , "[CDI:%d] Requesting %zu bytes from %s at offset %zu"
          , request()->req_id, request()->size
          , uint64_to_string_hex(request()->target_node.id).c_str()
          , request()->offs);
        return invoke_subflow_and_wait(client_, STATE(read_complete)
                                     , MemoryConfigClientRequest::READ_PART
                                     , request()->target_node
                                     , MemoryConfigDefs::SPACE_CONFIG
                                     , request()->offs, request()->size);
      case CDIClientRequest::CMD_WRITE:
        LOG(VERBOSE
          , "[CDI:%d] Writing %zu bytes to %s at offset %zu"
          , request()->req_id, request()->size
          , uint64_to_string_hex(request()->target_node.id).c_str()
          , request()->offs);
        return invoke_subflow_and_wait(client_, STATE(write_complete)
                                     , MemoryConfigClientRequest::WRITE
                                     , request()->target_node
                                     , MemoryConfigDefs::SPACE_CONFIG
                                     , request()->offs, request()->value);
      case CDIClientRequest::CMD_UPDATE_COMPLETE:
        LOG(VERBOSE, "[CDI:%d] Sending update-complete to %s"
          , request()->req_id
          , uint64_to_string_hex(request()->target_node.id).c_str());
        return invoke_subflow_and_wait(client_, STATE(update_complete)
                                     , MemoryConfigClientRequest::UPDATE_COMPLETE
                                     , request()->target_node);
    }
    return return_with_error(Defs::ERROR_UNIMPLEMENTED_SUBCMD);
  }

  StateFlowBase::Action read_complete()
  {
    auto b = get_buffer_deleter(full_allocation_result(client_));
    LOG(VERBOSE, "[CDI:%d] read bytes request returned with code: %d"
      , request()->req_id, b->data()->resultCode);
    string response;
    if (b->data()->resultCode)
    {
      LOG(VERBOSE, "[CDI:%d] non-zero result code, sending error response."
        , request()->req_id);
      response =
        StringPrintf(
          R"!^!({"res":"error","error":"request failed: %d","id":%d}\m)!^!"
        , b->data()->resultCode, request()->req_id);
    }
    else
    {
      LOG(VERBOSE, "[CDI:%d] Received %zu bytes from offset %zu"
        , request()->req_id, request()->size, request()->offs);
      if (request()->type == "str")
      {
        response =
          StringPrintf(
              R"!^!({"res":"field","tgt":"%s","val":"%s","type":"%s","id":%d})!^!"
            , request()->target.c_str(), b->data()->payload.c_str()
            , request()->type.c_str(), request()->req_id);
      }
      else if (request()->type == "int")
      {
        uint32_t data = b->data()->payload.data()[0];
        if (request()->size == 2)
        {
          uint16_t data16 = 0;
          memcpy(&data16, b->data()->payload.data(), sizeof(uint16_t));
          data = be16toh(data16);
        }
        else if (request()->size == 4)
        {
          uint32_t data32 = 0;
          memcpy(&data32, b->data()->payload.data(), sizeof(uint32_t));
          data = be32toh(data32);
        }
        response =
          StringPrintf(
              R"!^!({"res":"field","tgt":"%s","val":"%d","type":"%s","id":%d})!^!"
          , request()->target.c_str(), data, request()->type.c_str()
          , request()->req_id);
      }
      else if (request()->type == "evt")
      {
        uint64_t event_id = 0;
        memcpy(&event_id, b->data()->payload.data(), sizeof(uint64_t));
        response =
          StringPrintf(
              R"!^!({"res":"field","tgt":"%s","val":"%s","type":"%s","id":%d})!^!"
          , request()->target.c_str()
          , uint64_to_string_hex(be64toh(event_id)).c_str()
          , request()->type.c_str(), request()->req_id);
      }
    }
    LOG(VERBOSE, "[CDI-READ] %s", response.c_str());
    request()->socket->send_text(response);
    return return_with_error(b->data()->resultCode);
  }

  StateFlowBase::Action write_complete()
  {
    auto b = get_buffer_deleter(full_allocation_result(client_));
    LOG(VERBOSE, "[CDI:%d] write bytes request returned with code: %d"
      , request()->req_id, b->data()->resultCode);
    string response;
    if (b->data()->resultCode)
    {
      LOG(VERBOSE, "[CDI:%d] non-zero result code, sending error response."
        , request()->req_id);
      response =
          StringPrintf(
              R"!^!({"res":"error","error":"request failed: %d","id":%d})!^!"
            , b->data()->resultCode, request()->req_id);
    }
    else
    {
      LOG(VERBOSE, "[CDI:%d] Write request processed successfully."
        , request()->req_id);
      response =
        StringPrintf(R"!^!({"res":"saved","tgt":"%s","id":%d})!^!"
                    , request()->target.c_str(), request()->req_id);
    }
    LOG(VERBOSE, "[CDI-WRITE] %s", response.c_str());
    request()->socket->send_text(response);
    return return_with_error(b->data()->resultCode);
  }

  StateFlowBase::Action update_complete()
  {
    auto b = get_buffer_deleter(full_allocation_result(client_));
    LOG(VERBOSE, "[CDI:%d] update-complete request returned with code: %d"
      , request()->req_id, b->data()->resultCode);
    string response;
    if (b->data()->resultCode)
    {
      LOG(VERBOSE, "[CDI:%d] non-zero result code, sending error response."
        , request()->req_id);
      response =
        StringPrintf(
            R"!^!({"res":"error","error":"request failed: %d","id":%d}\m)!^!"
          , b->data()->resultCode, request()->req_id);
    }
    else
    {
      LOG(VERBOSE, "[CDI:%d] update-complete request processed successfully."
        , request()->req_id);
      response =
        StringPrintf(R"!^!({"res":"update-complete","id":%d})!^!"
                   , request()->req_id);
    }
    LOG(VERBOSE, "[CDI-UPDATE-COMPLETE] %s", response.c_str());
    request()->socket->send_text(response);
    return return_with_error(b->data()->resultCode);
  }

};

#define GET_LOCO_VIA_EXECUTOR(NAME, address)                                        \
  openlcb::TrainImpl *NAME = nullptr;                                               \
  Singleton<esp32cs::LCCStackManager>::instance()->stack()->executor()->sync_run(   \
  ([&]()                                                                            \
  {                                                                                 \
    NAME = Singleton<commandstation::AllTrainNodes>::instance()->get_train_impl(    \
                                      commandstation::DccMode::DCC_128, address);   \
  }));

#define REMOVE_LOCO_VIA_EXECUTOR(addr)                                              \
  Singleton<esp32cs::LCCStackManager>::instance()->stack()->executor()->sync_run(   \
  ([&]()                                                                            \
  {                                                                                 \
    Singleton<commandstation::AllTrainNodes>::instance()->remove_train_impl(addr);  \
  }));

std::unique_ptr<CDIClient> cdi_client;
NodeHandle cs_node_handle;

void init_webserver()
{
  auto stack = Singleton<esp32cs::LCCStackManager>::instance();
  auto httpd = Singleton<Httpd>::instance();
  cs_node_handle = NodeHandle(stack->node()->node_id());
  cdi_client.reset(
    new CDIClient(stack->service(), stack->memory_config_client()));
  httpd->captive_portal(
    StringPrintf(CAPTIVE_PORTAL_HTML
               , esp_ota_get_app_description()->version));
  httpd->static_uri("/", indexHtmlGz, indexHtmlGz_size
                  , MIME_TYPE_TEXT_HTML, HTTP_ENCODING_GZIP, false);
  httpd->static_uri("/loco-32x32.png", loco32x32, loco32x32_size
                  , MIME_TYPE_IMAGE_PNG);
  httpd->static_uri("/cash.min.js", cashJsGz, cashJsGz_size
                  , MIME_TYPE_TEXT_JAVASCRIPT, HTTP_ENCODING_GZIP);
  httpd->static_uri("/spectre.min.css", spectreCssGz, spectreCssGz_size
                  , MIME_TYPE_TEXT_CSS, HTTP_ENCODING_GZIP);
  httpd->static_uri("/cdi.xml", (const uint8_t *)openlcb::CDI_DATA
                  , openlcb::CDI_SIZE - 1, MIME_TYPE_TEXT_XML
                  , HTTP_ENCODING_NONE, false);
  httpd->websocket_uri("/ws", process_ws);
  httpd->websocket_uri("/wsjson", process_wsjson);
  httpd->uri("/update", HttpMethod::POST, nullptr, process_ota);
  httpd->uri("/fs", HttpMethod::GET,
  [&](HttpRequest *request) -> AbstractHttpResponse *
  {
    string path = request->param("path");
    struct stat statbuf;
    // verify that the requested path exists
    if (!stat(path.c_str(), &statbuf))
    {
      string data = read_file_to_string(path);
      string mimetype = http::MIME_TYPE_TEXT_PLAIN;
      if (path.find(".xml") != string::npos)
      {
        mimetype = MIME_TYPE_TEXT_XML;
      }
      else if (path.find(".json") != string::npos)
      {
        mimetype = http::MIME_TYPE_APPLICATION_JSON;
      }
      return new StringResponse(data, mimetype);
    }
    request->set_status(HttpStatusCode::STATUS_NOT_FOUND);
    return nullptr;
  });
  httpd->uri("/power", HttpMethod::GET | HttpMethod::PUT, process_power);
  httpd->uri("/programmer", HttpMethod::GET | HttpMethod::POST, process_prog);
  httpd->uri("/turnouts"
           , HttpMethod::GET | HttpMethod::POST |
             HttpMethod::PUT | HttpMethod::DELETE
           , process_turnouts);
  httpd->uri("/locomotive"
           , HttpMethod::GET | HttpMethod::POST |
             HttpMethod::PUT | HttpMethod::DELETE
           , process_loco);
  httpd->uri("/locomotive/roster"
           , HttpMethod::GET | HttpMethod::POST |
             HttpMethod::PUT | HttpMethod::DELETE
           , process_loco);
  httpd->uri("/locomotive/estop"
           , HttpMethod::GET | HttpMethod::POST |
             HttpMethod::PUT | HttpMethod::DELETE
           , process_loco);
#if CONFIG_GPIO_OUTPUTS
  httpd->uri("/outputs"
           , HttpMethod::GET | HttpMethod::POST |
             HttpMethod::PUT | HttpMethod::DELETE
           , process_outputs);
#endif // CONFIG_GPIO_OUTPUTS
#if CONFIG_GPIO_SENSORS
  httpd->uri("/sensors"
          , HttpMethod::GET | HttpMethod::POST | HttpMethod::DELETE
          , process_sensors);
  httpd->uri("/remoteSensors"
          , HttpMethod::GET | HttpMethod::POST | HttpMethod::DELETE
          , process_remote_sensors);
#if CONFIG_GPIO_S88
  httpd->uri("/s88sensors"
          , HttpMethod::GET | HttpMethod::POST | HttpMethod::DELETE
          , process_s88);
#endif // CONFIG_GPIO_S88
#endif // CONFIG_GPIO_SENSORS
}

WEBSOCKET_STREAM_HANDLER_IMPL(process_ws, client, event, data
                            , data_len)
{
  LOG(VERBOSE, "[WS %p/%d] handler invoked: %d, %p, %zu", client, client->id(), event, data, data_len);
  if (event == WebSocketEvent::WS_EVENT_CONNECT)
  {
    const std::lock_guard<std::mutex> lock(webSocketLock);
    webSocketClients.push_back(
      std::make_unique<WebSocketClient>(client->id(), client->ip()));
  }
  else if (event == WebSocketEvent::WS_EVENT_DISCONNECT)
  {
    const std::lock_guard<std::mutex> lock(webSocketLock);
    webSocketClients.erase(std::remove_if(webSocketClients.begin()
                                        , webSocketClients.end()
    , [client](const auto &inst) -> bool
      {
        return inst->id() == client->id();
      }));
  }
  else if (event == WebSocketEvent::WS_EVENT_TEXT)
  {
    auto ent = std::find_if(webSocketClients.begin(), webSocketClients.end()
    , [client](const auto &inst) -> bool
      {
        return inst->id() == client->id();
      }
    );
    if (ent != webSocketClients.end())
    {
      LOG(VERBOSE, "[WS %p/%d] feeding the DCC++ code", client, client->id());
      // TODO: remove cast that drops const
      auto res = (*ent)->feed((uint8_t *)data, data_len);
      if (res.length())
      {
        LOG(VERBOSE, "[WS %p/%d] sending %zu bytes as response", client, client->id(), res.length());
        client->send_text(res);
        LOG(VERBOSE, "[WS %p/%d] sent", client, client->id());
      }
      else
      {
        LOG(VERBOSE, "[WS %p/%d] no response to send", client, client->id());
      }
    }
  }
}

#ifdef CONFIG_GPIO_S88
#define GPIO_S88_CONFIG "true"
#define GPIO_S88_BASE CONFIG_GPIO_S88_FIRST_SENSOR
#else
#define GPIO_S88_CFG "false"
#define GPIO_S88_BASE 0
#endif

#ifdef CONFIG_GPIO_OUTPUTS
#define GPIO_OUTPUTS_CFG "true"
#else
#define GPIO_OUTPUTS_CFG "false"
#endif

#ifdef CONFIG_GPIO_SENSORS
#define GPIO_SENSORS_CFG "true"
#else
#define GPIO_SENSORS_CFG "false"
#endif

WEBSOCKET_STREAM_HANDLER_IMPL(process_wsjson, socket, event, data, len)
{
  if (event == WebSocketEvent::WS_EVENT_TEXT)
  {
    uint64_t node_id =
      Singleton<esp32cs::LCCStackManager>::instance()->node()->node_id();
    string response = R"!^!({"res":"error","error":"Request not understood"})!^!";
    string req = string((char *)data, len);
    LOG(VERBOSE, "[WS] MSG: %s", req.c_str());
    cJSON *root = cJSON_Parse(req.c_str());
    cJSON *req_type = cJSON_GetObjectItem(root, "req");
    cJSON *req_id = cJSON_GetObjectItem(root, "id");
    if (req_type == NULL || req_id == NULL)
    {
      // NO OP, the websocket is outbound only to trigger events on the client side.
      LOG(INFO, "[WSJSON] Failed to parse:%s", req.c_str());
    }
    else if (!strcmp(req_type->valuestring, "nodeid"))
    {
      if (!cJSON_HasObjectItem(root, "val"))
      {
        response =
          StringPrintf(R"!^!({"res":"error","error":"The 'val' field must be provided","id":%d})!^!"
                    , req_id->valueint);
      }
      else
      {
        std::string value = cJSON_GetObjectItem(root, "val")->valuestring;
        if (set_node_id(string_to_uint64(value)))
        {
          LOG(INFO, "[WSJSON:%d] Node ID updated to: %s, reboot pending"
            , req_id->valueint, value.c_str());
          response =
            StringPrintf(R"!^!({"res":"nodeid","id":%d})!^!", req_id->valueint);
          Singleton<esp32cs::LCCStackManager>::instance()->reboot_node();
        }
        else
        {
          LOG(INFO, "[WSJSON:%d] Node ID update failed", req_id->valueint);
          response =
            StringPrintf(R"!^!({"res":"error","error":"Failed to update node-id","id":%d})!^!"
                      , req_id->valueint);
        }
      }
    }
    else if (!strcmp(req_type->valuestring, "info"))
    {
      const esp_app_desc_t *app_data = esp_ota_get_app_description();
      const esp_partition_t *partition = esp_ota_get_running_partition();
      response =
        StringPrintf(R"!^!({"res":"info","timestamp":"%s %s","ota":"%s","snip_name":"%s","snip_hw":"%s","snip_sw":"%s","node_id":"%s","s88":%s,"sensorIDBase":%d,"outputs":%s,"sensors":%s,"bootloader":%s,"id":%d})!^!"
          , app_data->date, app_data->time, partition->label
          , openlcb::SNIP_STATIC_DATA.model_name
          , openlcb::SNIP_STATIC_DATA.hardware_version
          , openlcb::SNIP_STATIC_DATA.software_version
          , uint64_to_string_hex(node_id).c_str()
          , GPIO_S88_CFG, GPIO_S88_BASE, GPIO_OUTPUTS_CFG, GPIO_SENSORS_CFG
#if CONFIG_LCC_CAN_RX_PIN != -1 && CONFIG_LCC_CAN_TX_PIN != -1
          , "true"
#else
          , "false"
#endif
          , req_id->valueint);
    }
    else if (!strcmp(req_type->valuestring, "update-complete"))
    {
      BufferPtr<CDIClientRequest> b(cdi_client->alloc());
      b->data()->reset(CDIClientRequest::UPDATE_COMPLETE, cs_node_handle
                     , socket, req_id->valueint);
      b->data()->done.reset(EmptyNotifiable::DefaultInstance());
      LOG(VERBOSE, "[WSJSON:%d] Sending UPDATE_COMPLETE to queue"
        , req_id->valueint);
      cdi_client->send(b->ref());
      cJSON_Delete(root);
      return;
    }
    else if (!strcmp(req_type->valuestring, "cdi"))
    {
      if (!cJSON_HasObjectItem(root, "ofs") ||
          !cJSON_HasObjectItem(root, "type") ||
          !cJSON_HasObjectItem(root, "sz") ||
          !cJSON_HasObjectItem(root, "tgt"))
      {
        LOG_ERROR("[WSJSON:%d] One or more required parameters are missing: %s"
                , req_id->valueint, req.c_str());
        response =
          StringPrintf(
            R"!^!({"res":"error", "error":"request is missing one (or more) required parameters","id":%d})!^!"
          , req_id->valueint);
      }
      else
      {
        size_t offs = cJSON_GetObjectItem(root, "ofs")->valueint;
        std::string param_type =
            cJSON_GetObjectItem(root, "type")->valuestring;
        size_t size = cJSON_GetObjectItem(root, "sz")->valueint;
        string target = cJSON_GetObjectItem(root, "tgt")->valuestring;
        BufferPtr<CDIClientRequest> b(cdi_client->alloc());

        if (!cJSON_HasObjectItem(root, "val"))
        {
          LOG(VERBOSE
            , "[WSJSON:%d] Sending CDI READ: offs:%zu size:%zu type:%s tgt:%s"
            , req_id->valueint, offs, size, param_type.c_str()
            , target.c_str());
          b->data()->reset(CDIClientRequest::READ, cs_node_handle, socket
                         , req_id->valueint, offs, size, target, param_type);
        }
        else
        {
          string value = "";
          cJSON *raw_value = cJSON_GetObjectItem(root, "val");
          if (param_type == "str")
          {
            // copy of up to the reported size.
            value = string(raw_value->valuestring, size);
            // ensure value is null terminated
            value += '\0';
          }
          else if (param_type == "int")
          {
            if (size == 1)
            {
              uint8_t data8 = std::stoi(raw_value->valuestring);
              value.clear();
              value.push_back(data8);
            }
            else if (size == 2)
            {
              uint16_t data16 = std::stoi(raw_value->valuestring);
              value.clear();
              value.push_back((data16 >> 8) & 0xFF);
              value.push_back(data16 & 0xFF);
            }
            else
            {
              uint32_t data32 = std::stoul(raw_value->valuestring);
              value.clear();
              value.push_back((data32 >> 24) & 0xFF);
              value.push_back((data32 >> 16) & 0xFF);
              value.push_back((data32 >> 8) & 0xFF);
              value.push_back(data32 & 0xFF);
            }
          }
          else if (param_type == "evt")
          {
            uint64_t data = string_to_uint64(string(raw_value->valuestring));
            value.clear();
            value.push_back((data >> 56) & 0xFF);
            value.push_back((data >> 48) & 0xFF);
            value.push_back((data >> 40) & 0xFF);
            value.push_back((data >> 32) & 0xFF);
            value.push_back((data >> 24) & 0xFF);
            value.push_back((data >> 16) & 0xFF);
            value.push_back((data >> 8) & 0xFF);
            value.push_back(data & 0xFF);
          }
          LOG(VERBOSE
            , "[WSJSON:%d] Sending CDI WRITE: offs:%zu value:%s tgt:%s"
            , req_id->valueint, offs, raw_value->valuestring, target.c_str());
          b->data()->reset(CDIClientRequest::WRITE, cs_node_handle, socket
                         , req_id->valueint, offs, size, target, value);
        }
        b->data()->done.reset(EmptyNotifiable::DefaultInstance());
        cdi_client->send(b->ref());
        cJSON_Delete(root);
        return;
      }
    }
    else if (!strcmp(req_type->valuestring, "factory-reset"))
    {
      LOG(VERBOSE, "[WSJSON:%d] Factory reset received", req_id->valueint);
      if (force_factory_reset())
      {
        Singleton<esp32cs::LCCStackManager>::instance()->reboot_node();
        response =
          StringPrintf(R"!^!({"res":"factory-reset","id":%d})!^!"
                    , req_id->valueint);
      }
      else
      {
        LOG(INFO, "[WSJSON:%d] Factory reset update failed", req_id->valueint);
        response =
          StringPrintf(R"!^!({"res":"error","error":"Failed to record factory reset request","id":%d})!^!"
                    , req_id->valueint);
      }
    }
    else if (!strcmp(req_type->valuestring, "bootloader"))
    {
      LOG(VERBOSE, "[WSJSON:%d] bootloader request received", req_id->valueint);
      enter_bootloader();
      // NOTE: This response may not get sent to the client.
      response =
        StringPrintf(R"!^!({"res":"bootloader","id":%d})!^!", req_id->valueint);
    }
    else if (!strcmp(req_type->valuestring, "reset-events"))
    {
      LOG(VERBOSE, "[WSJSON:%d] Reset event IDs received", req_id->valueint);
      //Singleton<esp32cs::LCCStackManager>::instance()->reset_events();
      response =
        StringPrintf(R"!^!({"res":"reset-events","id":%d})!^!"
                   , req_id->valueint);
    }
    else if (!strcmp(req_type->valuestring, "event"))
    {
      if (!cJSON_HasObjectItem(root, "evt"))
      {
        LOG_ERROR("[WSJSON:%d] One or more required parameters are missing: %s"
                , req_id->valueint, req.c_str());
        response =
          StringPrintf(R"!^!({"res":"error","error":"The 'evt' field must be provided","id":%d})!^!"
                    , req_id->valueint);
      }
      else
      {
        string value = cJSON_GetObjectItem(root, "evt")->valuestring;
        LOG(VERBOSE, "[WSJSON:%d] Sending event: %s", req_id->valueint
          , value.c_str());
        uint64_t eventID = string_to_uint64(value);
        Singleton<esp32cs::LCCStackManager>::instance()->stack()->send_event(eventID);
        response =
          StringPrintf(R"!^!({"res":"event","evt":"%s","id":%d})!^!"
                    , value.c_str(), req_id->valueint);
      }
    }
    else if (!strcmp(req_type->valuestring, "function"))
    {
      if (!cJSON_HasObjectItem(root, "addr") ||
          !cJSON_HasObjectItem(root, "fn") ||
          !cJSON_HasObjectItem(root, "state"))
      {
        LOG_ERROR("[WSJSON:%d] One or more required parameters are missing: %s"
                , req_id->valueint, req.c_str());
        response =
          StringPrintf(R"!^!({"res":"error","error":"One (or more) required fields are missing.","id":%d})!^!"
                    , req_id->valueint);
      }
      else
      {
        uint16_t address = cJSON_GetObjectItem(root, "addr")->valueint;
        uint8_t function = cJSON_GetObjectItem(root, "fn")->valueint;
        uint8_t state = cJSON_IsTrue(cJSON_GetObjectItem(root, "state"));
        LOG(VERBOSE, "[WSJSON:%d] Setting function %d on loco %d to %d"
          , req_id->valueint, address, function, state);
        GET_LOCO_VIA_EXECUTOR(train, address);
        train->set_fn(function, state);
        response =
          StringPrintf(R"!^!({"res":"function","id":%d,"fn":%d,"state":%s})!^!"
                    , req_id->valueint, function
                    , train->get_fn(function) == 1 ? "true" : "false");
      }
    }
    else if (!strcmp(req_type->valuestring, "loco"))
    {
      if (!cJSON_HasObjectItem(root, "addr"))
      {
        LOG_ERROR("[WSJSON:%d] One or more required parameters are missing: %s"
                , req_id->valueint, req.c_str());
        response =
          StringPrintf(R"!^!({"res":"error","error":"The 'addr' field must be provided","id":%d})!^!"
                    , req_id->valueint);
      }
      else
      {
        uint16_t address = cJSON_GetObjectItem(root, "addr")->valueint;
        GET_LOCO_VIA_EXECUTOR(train, address);
        auto req_speed = train->get_speed();
        SpeedType direction = req_speed.direction();
        if (cJSON_HasObjectItem(root, "s"))
        {
          uint8_t speed = cJSON_GetObjectItem(root, "spd")->valueint;
          LOG(VERBOSE, "[WSJSON:%d] Setting loco %d speed to %d"
            , req_id->valueint, address, speed);
          req_speed.set_mph(speed);
        }
        if (cJSON_HasObjectItem(root, "dir"))
        {
          bool direction = cJSON_IsTrue(cJSON_GetObjectItem(root, "dir"));
          LOG(VERBOSE, "[WSJSON:%d] Setting loco %d direction to %s"
            , req_id->valueint, address, direction ? "REV" : "FWD");
          req_speed.set_direction(direction);
        }
        train->set_speed(req_speed);
        response =
          StringPrintf(R"!^!({"res":"loco","addr":%d,"spd":%d,"dir":%s,"id":%d})!^!"
                    , address, (int)req_speed.mph()
                    , req_speed.direction() ? "true" : "false", req_id->valueint);
      }
    }
    else if (!strcmp(req_type->valuestring, "turnout"))
    {
      if (!cJSON_HasObjectItem(root, "addr") ||
          !cJSON_HasObjectItem(root, "act"))
      {
        LOG_ERROR("[WSJSON:%d] One or more required parameters are missing: %s"
                , req_id->valueint, req.c_str());
        response =
          StringPrintf(R"!^!({"res":"error","error":"One (or more) required fields are missing.","id":%d})!^!"
                    , req_id->valueint);
      }
      else
      {
        uint16_t address = cJSON_GetObjectItem(root, "addr")->valueint;
        string action = cJSON_GetObjectItem(root, "act")->valuestring;
        string target = cJSON_HasObjectItem(root, "tgt") ?
            cJSON_GetObjectItem(root, "tgt")->valuestring : "";
        TurnoutType type = cJSON_HasObjectItem(root, "type") ?
            (TurnoutType)cJSON_GetObjectItem(root, "type")->valueint :
            TurnoutType::NO_CHANGE;
        if (action == "save")
        {
          LOG(VERBOSE, "[WSJSON:%d] Saving turnout %d as type %d"
            , req_id->valueint, address, type);
          Singleton<TurnoutManager>::instance()->createOrUpdate(address, type);
        }
        else if (action == "toggle")
        {
          LOG(VERBOSE, "[WSJSON:%d] Toggling turnout %d", req_id->valueint
            , address);
          Singleton<TurnoutManager>::instance()->toggle(address);
        }
        else if (action == "delete")
        {
          LOG(VERBOSE, "[WSJSON:%d] Deleting turnout %d", req_id->valueint
            , address);
          Singleton<TurnoutManager>::instance()->remove(address);
        }
        response =
          StringPrintf(R"!^!({"res":"turnout","act":"%s","addr":%d,"tgt":"%s","id":%d})!^!"
                    , action.c_str(), address, target.c_str(), req_id->valueint);
      }
    }
    else if (!strcmp(req_type->valuestring, "ping"))
    {
      LOG(VERBOSE, "[WSJSON:%d] PING received", req_id->valueint);
      response =
        StringPrintf(R"!^!({"res":"pong","id":%d})!^!", req_id->valueint);
    }
    else
    {
      LOG_ERROR("Unrecognized request: %s", req.c_str());
    }
    cJSON_Delete(root);
    LOG(VERBOSE, "[Web] WS: %s -> %s", req.c_str(), response.c_str());
    socket->send_text(response);
  }
}

esp_ota_handle_t otaHandle;
esp_partition_t *ota_partition = nullptr;
HTTP_STREAM_HANDLER_IMPL(process_ota, request, filename, size, data, length
                       , offset, final, abort_req)
{
  if (!offset)
  {
    esp_log_level_set("esp_image", ESP_LOG_VERBOSE);
    ota_partition = (esp_partition_t *)esp_ota_get_next_update_partition(NULL);
    esp_err_t err = ESP_ERROR_CHECK_WITHOUT_ABORT(
      esp_ota_begin(ota_partition, size, &otaHandle));
    if (err != ESP_OK)
    {
      LOG_ERROR("[WebSrv] OTA start failed, aborting!");
      Singleton<OTAMonitorFlow>::instance()->report_failure(err);
      request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
      *abort_req = true;
      return nullptr;
    }
    LOG(INFO, "[WebSrv] OTA Update starting (%zu bytes, target:%s)...", size
      , ota_partition->label);
    esp32cs::disable_track_outputs();
    Singleton<OTAMonitorFlow>::instance()->report_start();
  }
  HASSERT(ota_partition);
  ESP_ERROR_CHECK(esp_ota_write(otaHandle, data, length));
  Singleton<OTAMonitorFlow>::instance()->report_progress(length);
  if (final)
  {
    esp_err_t err = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_end(otaHandle));
    if (err != ESP_OK)
    {
      LOG_ERROR("[WebSrv] OTA end failed, aborting!");
      Singleton<OTAMonitorFlow>::instance()->report_failure(err);
      request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
      *abort_req = true;
      return nullptr;
    }
    LOG(INFO, "[WebSrv] OTA binary received, setting boot partition: %s"
      , ota_partition->label);
    err = ESP_ERROR_CHECK_WITHOUT_ABORT(
      esp_ota_set_boot_partition(ota_partition));
    if (err != ESP_OK)
    {
      LOG_ERROR("[WebSrv] OTA end failed, aborting!");
      Singleton<OTAMonitorFlow>::instance()->report_failure(err);
      request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
      *abort_req = true;
      return nullptr;
    }
    LOG(INFO, "[WebSrv] OTA Update Complete!");
    Singleton<OTAMonitorFlow>::instance()->report_success();
    request->set_status(HttpStatusCode::STATUS_OK);
    return new StringResponse("OTA Upload Complete", MIME_TYPE_TEXT_PLAIN);
  }
  return nullptr;
}

HTTP_HANDLER_IMPL(process_power, request)
{
  string response = "{}";
  request->set_status(HttpStatusCode::STATUS_OK);
  if (request->method() == HttpMethod::GET)
  {
    response.assign(esp32cs::get_track_state_json());
  }
  else if (request->method() == HttpMethod::PUT)
  {
    if (request->param(JSON_STATE_NODE, false))
    {
      esp32cs::enable_ops_track_output();
    }
    else
    {
      esp32cs::disable_track_outputs();
    }
  }
  return new JsonResponse(response);
}

HTTP_HANDLER_IMPL(process_prog, request)
{
  request->set_status(HttpStatusCode::STATUS_OK);
  if (!request->has_param(JSON_PROG_ON_MAIN))
  {
    request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
  }
  else if (request->method() == HttpMethod::GET)
  {
    if (request->param(JSON_PROG_ON_MAIN, false))
    {
      request->set_status(HttpStatusCode::STATUS_NOT_ALLOWED);
    }
    else if (request->has_param(JSON_IDENTIFY_NODE))
    {
      int16_t decoderConfig = readCV(CV_NAMES::DECODER_CONFIG);
      if (decoderConfig > 0)
      {
        uint16_t decoderAddress = 0;
        string response = "{";
        if ((decoderConfig & DECODER_CONFIG_BITS::DECODER_TYPE) == DECODER_CONFIG_BITS::DECODER_TYPE)
        {
          uint8_t decoderManufacturer = readCV(CV_NAMES::DECODER_MANUFACTURER);
          int16_t addrMSB = readCV(CV_NAMES::ACCESSORY_DECODER_MSB_ADDRESS);
          int16_t addrLSB = readCV(CV_NAMES::SHORT_ADDRESS);
          if (addrMSB >= 0 && addrLSB >= 0)
          {
            if (decoderManufacturer == 0xA5)
            {
              // MERG uses 7 bit LSB
              decoderAddress = (uint16_t)(((addrMSB & 0x07) << 7) | (addrLSB & 0x7F));
            }
            else if(decoderManufacturer == 0x19)
            {
              // Team Digital uses 8 bit LSB and 4 bit MSB
              decoderAddress = (uint16_t)(((addrMSB & 0x0F) << 8) | addrLSB);
            }
            else
            {
              // NMRA spec shows 6 bit LSB
              decoderAddress = (uint16_t)(((addrMSB & 0x07) << 6) | (addrLSB & 0x1F));
            }
            response += StringPrintf("\"%s\":\"%s\",", JSON_ADDRESS_MODE_NODE
                                   , JSON_VALUE_LONG_ADDRESS);
          }
          else
          {
            LOG(WARNING, "[WebSrv] Failed to read address MSB/LSB");
            request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
          }
        }
        else
        {
          if ((decoderConfig & DECODER_CONFIG_BITS::SHORT_OR_LONG_ADDRESS) == DECODER_CONFIG_BITS::SHORT_OR_LONG_ADDRESS)
          {
            int16_t addrMSB = readCV(CV_NAMES::LONG_ADDRESS_MSB_ADDRESS);
            int16_t addrLSB = readCV(CV_NAMES::LONG_ADDRESS_LSB_ADDRESS);
            if (addrMSB >= 0 && addrLSB >= 0)
            {
              decoderAddress = (uint16_t)(((addrMSB & 0xFF) << 8) | (addrLSB & 0xFF));
              response += StringPrintf("\"%s\":\"%s\",", JSON_ADDRESS_MODE_NODE
                                     , JSON_VALUE_LONG_ADDRESS);

            }
            else
            {
              LOG(WARNING, "[WebSrv] Unable to read address MSB/LSB");
              request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
            }
          }
          else
          {
            int16_t shortAddr = readCV(CV_NAMES::SHORT_ADDRESS);
            if (shortAddr > 0)
            {
              decoderAddress = shortAddr;
              response += StringPrintf("\"%s\":\"%s\",", JSON_ADDRESS_MODE_NODE
                                     , JSON_VALUE_SHORT_ADDRESS);
            }
            else
            {
              LOG(WARNING, "[WebSrv] Unable to read short address CV");
              request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
            }
          }
          response += StringPrintf("\"%s\":%s,", JSON_SPEED_TABLE_NODE
                                 , (decoderConfig & DECODER_CONFIG_BITS::SPEED_TABLE) == DECODER_CONFIG_BITS::SPEED_TABLE ?
                                           JSON_VALUE_ON : JSON_VALUE_OFF);
        }
        response += StringPrintf("\"%s\":%d,", JSON_ADDRESS_NODE
                               , decoderAddress);
        // if it is a mobile decoder *AND* we are requested to create it, send
        // the decoder address to the train db to create an entry.
        if (request->param(JSON_CREATE_NODE, false) &&
           (decoderConfig & BIT(DECODER_CONFIG_BITS::DECODER_TYPE)) == 0)
        {
          auto traindb = Singleton<esp32cs::Esp32TrainDatabase>::instance();
          traindb->create_if_not_found(decoderAddress);
        }
        response += "}";
        return new JsonResponse(response);
      }
      else
      {
        LOG(WARNING, "[WebSrv] Failed to read decoder configuration");
        request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
      }
    }
    else
    {
      uint16_t cvNumber = request->param(JSON_CV_NODE, 0);
      if (cvNumber == 0)
      {
        request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
      }
      else
      {
        int16_t cvValue = readCV(cvNumber);
        if (cvValue < 0)
        {
          request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
        }
        else
        {
          return new JsonResponse(
            StringPrintf("{\"%s\":%d,\"%s\":%d}", JSON_CV_NODE, cvNumber
                      , JSON_VALUE_NODE, cvValue));
        }
      }
    }
  }
  else if (request->method() == HttpMethod::POST)
  {
    uint16_t cv_num = request->param(JSON_CV_NODE, 0);
    uint16_t cv_value = 0;
    uint8_t cv_bit = request->param(JSON_CV_BIT_NODE, 0);
    bool pom = request->param(JSON_PROG_ON_MAIN, false);
    if (request->has_param(JSON_CV_BIT_NODE))
    {
      cv_value = request->param(JSON_VALUE_NODE, false);
    }
    else
    {
      cv_value = request->param(JSON_VALUE_NODE, 0);
    }

    if (cv_num == 0)
    {
      request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
    }
    else if (pom)
    {
      uint16_t address = request->param(JSON_ADDRESS_NODE, 0);
      if (request->has_param(JSON_CV_BIT_NODE))
      {
        writeOpsCVBit(address, cv_num, cv_bit, cv_value);
      }
      else
      {
        writeOpsCVByte(address, cv_num, cv_value);
      }
    }
    else if (request->has_param(JSON_CV_BIT_NODE) &&
             !writeProgCVBit(cv_num, cv_bit,cv_value))
    {
      request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
    }
    else if (!writeProgCVByte(cv_num, cv_value))
    {
      request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
    }
  }
  return nullptr;
}

// GET /turnouts - full list of turnouts, note that turnout state is STRING type for display
// GET /turnouts?readbleStrings=[0,1] - full list of turnouts, turnout state will be returned as true/false (boolean) when readableStrings=0.
// GET /turnouts?address=<address> - retrieve turnout by DCC address
// PUT /turnouts?address=<address> - toggle turnout by DCC address
// POST /turnouts?address=<address>&type=<type> - creates a new turnout
// DELETE /turnouts?address=<address> - delete turnout by DCC address
//
// For successful requests the result code will be 200 and either an array of turnouts or single turnout will be returned.
// For unsuccessful requests the result code will be 400 (bad request, missing args), 404 (not found), 500 (server failure).
//
HTTP_HANDLER_IMPL(process_turnouts, request)
{
  auto turnoutMgr = Singleton<TurnoutManager>::instance();
  if (request->method() == HttpMethod::GET &&
     !request->has_param(JSON_ADDRESS_NODE))
  {
    bool readable = request->param(JSON_TURNOUTS_READABLE_STRINGS_NODE, false);
    return new JsonResponse(turnoutMgr->getStateAsJson(readable));
  }

  uint16_t address = request->param(JSON_ADDRESS_NODE, 0);
  if (address < 1 || address > 2044)
  {
    request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
  }
  else if (request->method() == HttpMethod::GET)
  {
    auto turnout = turnoutMgr->get(address);
    if (turnout)
    {
      return new JsonResponse(turnout->toJson());
    }
    request->set_status(HttpStatusCode::STATUS_NOT_FOUND);
  }
  else if (request->method() == HttpMethod::POST)
  {
    TurnoutType type = (TurnoutType)request->param(JSON_TYPE_NODE
                                                 , TurnoutType::LEFT);
    auto turnout = turnoutMgr->createOrUpdate(address, type);
    if (turnout)
    {
      return new JsonResponse(turnout->toJson());
    }
    request->set_status(HttpStatusCode::STATUS_NOT_FOUND);
  }
  else if (request->method() == HttpMethod::DELETE)
  {
    if (turnoutMgr->remove(address))
    {
      request->set_status(HttpStatusCode::STATUS_NO_CONTENT);
    }
    else
    {
      request->set_status(HttpStatusCode::STATUS_NOT_FOUND);
    }
  }
  else if (request->method() == HttpMethod::PUT)
  {
    turnoutMgr->toggle(address);
    request->set_status(HttpStatusCode::STATUS_NO_CONTENT);
  }
  return nullptr;
}

string convert_loco_to_json(openlcb::TrainImpl *t)
{
  if (!t)
  {
    return "{}";
  }
  string res =
    StringPrintf("{\"%s\":%d,\"%s\":%d,\"%s\":\"%s\",\"%s\":["
               , JSON_ADDRESS_NODE, t->legacy_address()
               , JSON_SPEED_NODE, (int)t->get_speed().mph()
               , JSON_DIRECTION_NODE
               , t->get_speed().direction() == dcc::SpeedType::REVERSE ? JSON_VALUE_REVERSE
                                                                       : JSON_VALUE_FORWARD
               , JSON_FUNCTIONS_NODE);

  for (size_t funcID = 0; funcID < commandstation::DCC_MAX_FN; funcID++)
  {
    if (funcID)
    {
      res += ",";
    }
    res += StringPrintf("{\"%s\":%d,\"%s\":%d}", JSON_ID_NODE, funcID, JSON_STATE_NODE, t->get_fn(funcID));
  }
  res += "]}";
  return res;
}

// method - url pattern - meaning
// ANY /locomotive/estop - send emergency stop to all locomotives
// GET /locomotive/roster - roster
// GET /locomotive/roster?address=<address> - get roster entry
// PUT /locomotive/roster?address=<address> - update roster entry
// POST /locomotive/roster?address=<address> - create roster entry
// DELETE /locomotive/roster?address=<address> - delete roster entry
// GET /locomotive - get active locomotives
// POST /locomotive?address=<address> - add locomotive to active management
// GET /locomotive?address=<address> - get locomotive state
// PUT /locomotive?address=<address>&speed=<speed>&dir=[FWD|REV]&fX=[true|false] - Update locomotive state, fX is short for function X where X is 0-28.
// DELETE /locomotive?address=<address> - removes locomotive from active management
HTTP_HANDLER_IMPL(process_loco, request)
{
  string url = request->uri();
  auto traindb = Singleton<esp32cs::Esp32TrainDatabase>::instance();
  request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);

  // check if we have an eStop command, we don't care how this gets sent to the
  // command station (method) so check it first
  if (url.find("/estop") != string::npos)
  {
    esp32cs::toggle_estop();
    request->set_status(HttpStatusCode::STATUS_OK);
  }
  else if (url.find("/roster")  != string::npos)
  {
    if (request->method() == HttpMethod::GET &&
       !request->has_param(JSON_ADDRESS_NODE))
    {
      return new JsonResponse(traindb->get_all_entries_as_json());
    }
    else if (request->has_param(JSON_ADDRESS_NODE))
    {
      uint16_t address = request->param(JSON_ADDRESS_NODE, 0);
      if (request->method() == HttpMethod::DELETE)
      {
        traindb->delete_entry(address);
        request->set_status(HttpStatusCode::STATUS_NO_CONTENT);
      }
      else
      {
        traindb->create_if_not_found(address, std::to_string(address));
        if (request->has_param(JSON_NAME_NODE))
        {
          auto name = request->param(JSON_NAME_NODE);
          if (name.length() > 16)
          {
            LOG_ERROR("[WebSrv] Received locomotive name that is too long, "
                      "returning error.\n%s", request->to_string().c_str());
            request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
            return nullptr;
          }
          else if (name.empty())
          {
            name = integer_to_string(address);
          }
          traindb->set_train_name(address, name);
        }
        if (request->has_param(JSON_IDLE_ON_STARTUP_NODE))
        {
          traindb->set_train_auto_idle(address
                                     , request->param(JSON_IDLE_ON_STARTUP_NODE
                                                    , false));
        }
        if (request->has_param(JSON_DEFAULT_ON_THROTTLE_NODE))
        {
          traindb->set_train_show_on_limited_throttle(
            address, request->param(JSON_DEFAULT_ON_THROTTLE_NODE, false));
        }
        // search for and remap functions if present
        for (uint8_t fn = 0; fn <= 28; fn++)
        {
          string fArg = StringPrintf("f%d", fn);
          if (request->has_param(fArg.c_str()))
          {
            commandstation::Symbols label =
            static_cast<commandstation::Symbols>(
              request->param(fArg, commandstation::Symbols::FN_UNKNOWN));
            traindb->set_train_function_label(address, fn, label);
          }
        }
        if (request->has_param(JSON_MODE_NODE))
        {
          int8_t mode = request->param(JSON_MODE_NODE, -1);
          if (mode < 0)
          {
            LOG_ERROR("[WebSrv] Invalid parameters for setting mode:\n%s"
                    , request->to_string().c_str());
            request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
            return nullptr;
          }
          commandstation::DccMode drive_mode =
            static_cast<commandstation::DccMode>(mode);
          traindb->set_train_drive_mode(address, drive_mode);
        }
        return new JsonResponse(traindb->get_entry_as_json(address));
      }
    }
  }
  else
  {
    // Since it is not an eStop or roster command we need to check the request
    // method and ensure it contains the required arguments otherwise the
    // request should be rejected
    if (request->method() == HttpMethod::GET && 
       !request->has_param(JSON_ADDRESS_NODE))
    {
      // get all active locomotives
      string res = "[";
      auto trains = Singleton<commandstation::AllTrainNodes>::instance();
      for (size_t id = 0; id < trains->size(); id++)
      {
        auto nodeid = trains->get_train_node_id_ext(id, false);
        if (nodeid)
        {
          auto loco = trains->get_train_impl(nodeid, false);
          if (loco)
          {
            if (res.length() > 1)
            {
              res += ",";
            }
            res += convert_loco_to_json(loco);
          }
        }
      }
      res += "]";
      return new JsonResponse(res);
    }
    else if (request->has_param(JSON_ADDRESS_NODE))
    {
      uint16_t address = request->param(JSON_ADDRESS_NODE, 0);
      if (request->method() == HttpMethod::PUT ||
          request->method() == HttpMethod::POST)
      {
        GET_LOCO_VIA_EXECUTOR(loco, address);
        // Creation / Update of active locomotive
        if (request->has_param(JSON_IDLE_NODE))
        {
          loco->set_speed(dcc::SpeedType(0));
        }
        if (request->has_param(JSON_SPEED_NODE))
        {
          bool forward = true;
          if (request->has_param(JSON_DIRECTION_NODE))
          {
            forward = !request->param(JSON_DIRECTION_NODE).compare(JSON_VALUE_FORWARD);
          }
          auto speed = dcc::SpeedType::from_mph(request->param(JSON_SPEED_NODE, 0));
          if (!forward)
          {
            speed.set_direction(dcc::SpeedType::REVERSE);
          }
          loco->set_speed(speed);
        }
        else if (request->has_param(JSON_DIRECTION_NODE))
        {
          bool forward =
            !request->param(JSON_DIRECTION_NODE).compare(JSON_VALUE_FORWARD);
          auto upd_speed = loco->get_speed();
          upd_speed.set_direction(forward ? dcc::SpeedType::FORWARD
                                          : dcc::SpeedType::REVERSE);
          loco->set_speed(upd_speed);
        }
        
        for (uint8_t funcID = 0; funcID <= 28; funcID++)
        {
          string fArg = StringPrintf("f%d", funcID);
          if (request->has_param(fArg.c_str()))
          {
            loco->set_fn(funcID, request->param(fArg, false));
          }
        }
        return new JsonResponse(convert_loco_to_json(loco));
      }
      else if (request->method() == HttpMethod::DELETE)
      {
        REMOVE_LOCO_VIA_EXECUTOR(address)
#if CONFIG_NEXTION
        static_cast<NextionThrottlePage *>(nextionPages[THROTTLE_PAGE])->invalidateLocomotive(address);
#endif
        request->set_status(HttpStatusCode::STATUS_NO_CONTENT);
      }
      else
      {
        GET_LOCO_VIA_EXECUTOR(loco, address);
        return new JsonResponse(convert_loco_to_json(loco));
      }
    }
  }
  return nullptr;
}

#if CONFIG_GPIO_OUTPUTS
HTTP_HANDLER_IMPL(process_outputs, request)
{
  if (request->method() == HttpMethod::GET && !request->params())
  {
    return new JsonResponse(OutputManager::getStateAsJson());
  }
  request->set_status(HttpStatusCode::STATUS_OK);
  int16_t output_id = request->param(JSON_ID_NODE, -1);
  if (output_id < 0)
  {
    request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
  }
  else if (request->method() == HttpMethod::GET)
  {
    auto output = OutputManager::getOutput(output_id);
    if (output)
    {
      return new JsonResponse(output->toJson());
    }
    request->set_status(HttpStatusCode::STATUS_NOT_FOUND);
  }
  else if (request->method() == HttpMethod::POST)
  {
    int8_t pin = request->param(JSON_PIN_NODE, GPIO_NUM_NC);
    bool inverted = request->param(JSON_INVERTED_NODE, false);
    bool forceState = request->param(JSON_FORCE_STATE_NODE, false);
    bool defaultState = request->param(JSON_DEFAULT_STATE_NODE, false);
    uint8_t outputFlags = 0;
    if (inverted)
    {
      outputFlags &= OUTPUT_IFLAG_INVERT;
    }
    if (forceState)
    {
      outputFlags &= OUTPUT_IFLAG_RESTORE_STATE;
      if (defaultState)
      {
        outputFlags &= OUTPUT_IFLAG_FORCE_STATE;
      }
    }
    if (pin < 0)
    {
      request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
    }
    else if (!OutputManager::createOrUpdate(output_id, (gpio_num_t)pin, outputFlags))
    {
      request->set_status(HttpStatusCode::STATUS_NOT_ALLOWED);
    }
  }
  else if (request->method() == HttpMethod::DELETE &&
          !OutputManager::remove(output_id))
  {
    request->set_status(HttpStatusCode::STATUS_NOT_FOUND);
  }
  else if(request->method() == HttpMethod::PUT)
  {
    OutputManager::toggle(output_id);
  }
  return nullptr;
}
#endif // CONFIG_GPIO_OUTPUTS

#if CONFIG_GPIO_SENSORS
HTTP_HANDLER_IMPL(process_sensors, request)
{
  request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
  if (request->method() == HttpMethod::GET &&
     !request->has_param(JSON_ID_NODE))
  {
    return new JsonResponse(SensorManager::getStateAsJson());
  }
  else if (!request->has_param(JSON_ID_NODE))
  {
    request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
  }
  else
  {
    int16_t id = request->param(JSON_ID_NODE, -1);
    if (id < 0)
    {
      request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
    }
    else if (request->method() == HttpMethod::GET)
    {
      auto sensor = SensorManager::getSensor(id);
      if (sensor)
      {
        return new JsonResponse(sensor->toJson());
      }
      request->set_status(HttpStatusCode::STATUS_NOT_FOUND);
    }
    else if (request->method() == HttpMethod::POST)
    {
      int8_t pin = request->param(JSON_PIN_NODE, NON_STORED_SENSOR_PIN);
      bool pull = request->param(JSON_PULLUP_NODE, false);
      if (pin < 0)
      {
        request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
      }
      else if (!SensorManager::createOrUpdate(id, (gpio_num_t)pin, pull))
      {
        request->set_status(HttpStatusCode::STATUS_NOT_ALLOWED);
      }
      else
      {
        request->set_status(HttpStatusCode::STATUS_OK);
      }
    }
    else if (request->method() == HttpMethod::DELETE)
    {
      if (SensorManager::getSensorPin(id) < 0)
      {
        // attempt to delete S88/RemoteSensor
        request->set_status(HttpStatusCode::STATUS_NOT_ALLOWED);
      }
      else if (!SensorManager::remove(id))
      {
        request->set_status(HttpStatusCode::STATUS_NOT_FOUND);
      }
      else
      {
        request->set_status(HttpStatusCode::STATUS_NO_CONTENT);
      }
    }
  }
  
  return nullptr;
}

HTTP_HANDLER_IMPL(process_remote_sensors, request)
{
  request->set_status(HttpStatusCode::STATUS_OK);
  if (request->method() == HttpMethod::GET)
  {
    return new JsonResponse(RemoteSensorManager::getStateAsJson());
  }
  else if (request->method() == HttpMethod::POST)
  {
    RemoteSensorManager::createOrUpdate(request->param(JSON_ID_NODE, 0),
                                        request->param(JSON_VALUE_NODE, 0));
  }
  else if (request->method() == HttpMethod::DELETE)
  {
    RemoteSensorManager::remove(request->param(JSON_ID_NODE, 0));
  }
  return nullptr;
}

#if CONFIG_GPIO_S88
HTTP_HANDLER_IMPL(process_s88, request)
{
  request->set_status(HttpStatusCode::STATUS_OK);
  if (request->method() == HttpMethod::GET)
  {
    return new JsonResponse(S88BusManager::instance()->get_state_as_json());
  }
  else if (request->method() == HttpMethod::POST)
  {
    if(!S88BusManager::instance()->createOrUpdateBus(
      request->param(JSON_ID_NODE, 0),
      (gpio_num_t)request->param(JSON_PIN_NODE, 0),
      request->param(JSON_COUNT_NODE, 0)))
    {
      // duplicate pin/id
      request->set_status(HttpStatusCode::STATUS_NOT_ALLOWED);
    }
  }
  else if (request->method() == HttpMethod::DELETE)
  {
    S88BusManager::instance()->removeBus(request->param(JSON_ID_NODE, 0));
  }
  return nullptr;
}
#endif // CONFIG_GPIO_S88

#endif // CONFIG_GPIO_SENSORS
