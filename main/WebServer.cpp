/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2020 Mike Dunston

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
#include <FileSystemManager.h>
#include <DCCppProtocol.h>
#include <DCCProgrammer.h>
#include <dcc/Loco.hxx>
#include <Dnsd.h>
#include <DCCSignalVFS.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>
#include <Httpd.h>
#include <JsonConstants.h>
#include <LCCStackManager.h>
#include <LCCWiFiManager.h>
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
using http::WebSocketEvent;
using openlcb::TcpClientDefaultParams;

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

OSMutex webSocketLock;
std::vector<std::unique_ptr<WebSocketClient>> webSocketClients;
WEBSOCKET_STREAM_HANDLER(process_websocket_event);
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

extern const uint8_t jqueryJsGz[] asm("_binary_jquery_min_js_gz_start");
extern const size_t jqueryJsGz_size asm("jquery_min_js_gz_length");

extern const uint8_t jqueryMobileJsGz[] asm("_binary_jquery_mobile_1_5_0_rc1_min_js_gz_start");
extern const size_t jqueryMobileJsGz_size asm("jquery_mobile_1_5_0_rc1_min_js_gz_length");

extern const uint8_t jqueryMobileCssGz[] asm("_binary_jquery_mobile_1_5_0_rc1_min_css_gz_start");
extern const size_t jqueryMobileCssGz_size asm("jquery_mobile_1_5_0_rc1_min_css_gz_length");

extern const uint8_t jquerySimpleWebSocketGz[] asm("_binary_jquery_simple_websocket_min_js_gz_start");
extern const size_t jquerySimpleWebSocketGz_size asm("jquery_simple_websocket_min_js_gz_length");

extern const uint8_t jqClockGz[] asm("_binary_jqClock_lite_min_js_gz_start");
extern const size_t jqClockGz_size asm("jqClock_lite_min_js_gz_length");

extern const uint8_t ajaxLoader[] asm("_binary_ajax_loader_gif_start");
extern const size_t ajaxLoader_size asm("ajax_loader_gif_length");

// CDI Helper which sets the provided path if it is different than the value
// passed in.
#define CDI_COMPARE_AND_SET(name, PATH, fd, value, valuestr, updated) \
  {                                                                   \
    auto current = PATH().read(fd);                                   \
    if (current != value)                                             \
    {                                                                 \
      LOG(INFO, "[CDI] %s set to: %s", name, valuestr);               \
      PATH().write(fd, value);                                        \
      updated = true;                                                 \
    }                                                                 \
  }

// Helper which will trigger a config reload event to be queued when
// updated is true.
#define MAYBE_TRIGGER_UPDATE(updated)                                 \
  if (updated)                                                        \
  {                                                                   \
    Singleton<esp32cs::LCCStackManager>::instance()->update_config(); \
  }

class WebConfigListener : public DefaultConfigUpdateListener
{
public:
  WebConfigListener(const esp32cs::Esp32ConfigDef &cfg) : fd_(-1), cfg_(cfg)
  {
    cachedConfig_.reserve(512);
  }

  UpdateAction apply_configuration(int fd, bool initial_load
                                 , BarrierNotifiable *done) override
  {
    AutoNotify n(done);

    // cache the file descriptor for use in the update methods.
    // TODO: switch to MemoryConfigClient instead.
    fd_ = fd;

    cache_config();

    return UpdateAction::UPDATED;
  }

  void cache_config()
  {
    auto wifi = cfg_.seg().wifi_lcc();
    auto ops = cfg_.seg().hbridge().entry(esp32cs::OPS_CDI_TRACK_OUTPUT_IDX);
    auto prog = cfg_.seg().hbridge().entry(esp32cs::PROG_CDI_TRACK_OUTPUT_IDX);
    bool uplink = false;

    if (Singleton<esp32cs::LCCWiFiManager>::exists())
    {
      uplink = Singleton<esp32cs::LCCWiFiManager>::instance()->is_uplink_enabled();
    }

    int8_t current_power = 0;
    esp_wifi_get_max_tx_power(&current_power);

    // populate the cached config
    cachedConfig_ =
      StringPrintf(CONFIG_JSON_FMT
                 , CDI_READ_TRIM_DEFAULT(wifi.hub().enable, fd_) ? "true" : "false"
                 , CDI_READ_TRIM_DEFAULT(wifi.tx_power, fd_)
                 , current_power != 0 ? current_power : CDI_READ_TRIM_DEFAULT(wifi.tx_power, fd_)
                 , uplink ? "true" : "false"
                 , wifi.uplink().auto_address().service_name().read(fd_).c_str()
                 , wifi.uplink().manual_address().ip_address().read(fd_).c_str()
                 , CDI_READ_TRIM_DEFAULT(wifi.uplink().manual_address().port, fd_)
                 , uint64_to_string_hex(ops.event_short().read(fd_)).c_str()
                 , uint64_to_string_hex(ops.event_short_cleared().read(fd_)).c_str()
                 , uint64_to_string_hex(ops.event_shutdown().read(fd_)).c_str()
                 , uint64_to_string_hex(ops.event_shutdown_cleared().read(fd_)).c_str()
                 , uint64_to_string_hex(prog.event_short().read(fd_)).c_str()
                 , uint64_to_string_hex(prog.event_short_cleared().read(fd_)).c_str()
                 , uint64_to_string_hex(prog.event_shutdown().read(fd_)).c_str()
                 , uint64_to_string_hex(prog.event_shutdown_cleared().read(fd_)).c_str());
  }

  void factory_reset(int fd) override
  {
    // no-op
  }

  void reconfigure_hbridge(uint8_t hbridge_index
                         , string short_on, string short_off
                         , string shutdown_on, string shutdown_off)
  {
    bool upd = false;
    auto hbridge = cfg_.seg().hbridge().entry(hbridge_index);
    CDI_COMPARE_AND_SET("hbridge-short", hbridge.event_short, fd_
                      , string_to_uint64(short_on), short_on.c_str(), upd);
    CDI_COMPARE_AND_SET("hbridge-short-clear", hbridge.event_short_cleared, fd_
                      , string_to_uint64(short_off), short_off.c_str(), upd);
    CDI_COMPARE_AND_SET("hbridge-shutdown", hbridge.event_shutdown, fd_
                      , string_to_uint64(shutdown_on), shutdown_on.c_str()
                      , upd);
    CDI_COMPARE_AND_SET("hbridge-shutdown-clear"
                      , hbridge.event_shutdown_cleared, fd_
                      , string_to_uint64(shutdown_off), shutdown_off.c_str()
                      , upd);
    MAYBE_TRIGGER_UPDATE(upd);
  }

  void clear_last_uplink()
  {
    auto uplink = cfg_.seg().wifi_lcc().uplink();
    uplink.last_address().ip_address().write(fd_, "");
    MAYBE_TRIGGER_UPDATE(true);
  }

  void reconfigure_uplink(string uplink_service_name
                        , string manual_hostname
                        , uint16_t manual_port)
  {
    auto wifi = cfg_.seg().wifi_lcc();
    bool upd = false;
    CDI_COMPARE_AND_SET("uplink-auto-addr"
                      , wifi.uplink().auto_address().service_name, fd_
                      , uplink_service_name, uplink_service_name.c_str(), upd);
    CDI_COMPARE_AND_SET("uplink-manual-addr"
                      , wifi.uplink().manual_address().ip_address, fd_
                      , manual_hostname, manual_hostname.c_str(), upd);
    CDI_COMPARE_AND_SET("uplink-manual-port"
                      , wifi.uplink().manual_address().port, fd_, manual_port
                      , integer_to_string(manual_port).c_str(), upd);
    MAYBE_TRIGGER_UPDATE(upd);
  }

  void reconfigure_lcc_hub(bool enabled)
  {
    bool upd = false;
    CDI_COMPARE_AND_SET("lcc-hub", cfg_.seg().wifi_lcc().hub().enable, fd_
                      , enabled, enabled ? "true" : "false", upd);
    MAYBE_TRIGGER_UPDATE(upd);
  }

  void reconfigure_wifi_tx_power(uint8_t value)
  {
    bool upd = false;
    CDI_COMPARE_AND_SET("wifi-tx-power", cfg_.seg().wifi_lcc().tx_power, fd_
                      , value, integer_to_string(value).c_str(), upd);
    MAYBE_TRIGGER_UPDATE(upd);
  }

  string get_config_json()
  {
    return cachedConfig_;
  }
private:
  int fd_;
  const esp32cs::Esp32ConfigDef cfg_;
  string cachedConfig_;
  static constexpr const char * const CONFIG_JSON_FMT = R"!^!(  "hub":%s,
  "tx_power":%d,
  "tx_power_inuse":%d,
  "uplink":{
    "enabled":%s,
    "auto_service":"%s",
    "manual_host":"%s",
    "manual_port":%d
  },
  "hbridges":[
    {
      "short":"%s",
      "short_clear":"%s",
      "shutdown":"%s",
      "shutdown_clear":"%s"
    },
    {
      "short":"%s",
      "short_clear":"%s",
      "shutdown":"%s",
      "shutdown_clear":"%s"
    }
  ])!^!";
};

std::unique_ptr<WebConfigListener> configListener;

#ifndef CONFIG_GPIO_S88_FIRST_SENSOR
#define CONFIG_GPIO_S88_FIRST_SENSOR 0
#endif
#ifndef CONFIG_GPIO_S88
#define CONFIG_GPIO_S88 false
#endif
#ifndef CONFIG_GPIO_OUTPUTS
#define CONFIG_GPIO_OUTPUTS false
#endif
#ifndef CONFIG_GPIO_SENSORS
#define CONFIG_GPIO_SENSORS false
#endif

void init_webserver(const esp32cs::Esp32ConfigDef &cfg)
{
  configListener.reset(new WebConfigListener(cfg));

  auto httpd = Singleton<Httpd>::instance();
  httpd->redirect_uri("/", "/index.html");
  // if the soft AP interface is enabled, setup the captive portal
  if (Singleton<esp32cs::LCCWiFiManager>::instance()->is_softap_enabled())
  {
    httpd->captive_portal(
      StringPrintf(CAPTIVE_PORTAL_HTML
                 , esp_ota_get_app_description()->version));
  } 
  httpd->static_uri("/index.html", indexHtmlGz, indexHtmlGz_size
                  , MIME_TYPE_TEXT_HTML, HTTP_ENCODING_GZIP);
  httpd->static_uri("/loco-32x32.png", loco32x32, loco32x32_size
                  , MIME_TYPE_IMAGE_PNG);
  httpd->static_uri("/jquery.min.js", jqueryJsGz, jqueryJsGz_size
                  , MIME_TYPE_TEXT_JAVASCRIPT, HTTP_ENCODING_GZIP);
  httpd->static_uri("/jquery.mobile-1.5.0-rc1.min.js", jqueryMobileJsGz
                  , jqueryMobileJsGz_size, MIME_TYPE_TEXT_JAVASCRIPT
                  , HTTP_ENCODING_GZIP);
  httpd->static_uri("/jquery.mobile-1.5.0-rc1.min.css", jqueryMobileCssGz
                  , jqueryMobileCssGz_size, MIME_TYPE_TEXT_CSS
                  , HTTP_ENCODING_GZIP);
  httpd->static_uri("/jquery.simple.websocket.min.js"
                  , jquerySimpleWebSocketGz, jquerySimpleWebSocketGz_size
                  , MIME_TYPE_TEXT_JAVASCRIPT, HTTP_ENCODING_GZIP);
  httpd->static_uri("/jqClock-lite.min.js", jqClockGz, jqClockGz_size
                  , MIME_TYPE_TEXT_JAVASCRIPT, HTTP_ENCODING_GZIP);
  httpd->static_uri("/images/ajax-loader.gif", ajaxLoader, ajaxLoader_size
                  , MIME_TYPE_IMAGE_GIF);
  httpd->websocket_uri("/ws", process_websocket_event);
  httpd->uri("/update", HttpMethod::POST, nullptr, process_ota);
  httpd->uri("/features", [&](HttpRequest *req)
  {
    string features =
      StringPrintf("{\"%s\":%d,\"%s\":%s,\"%s\":%s,\"%s\":%s}"
                 , JSON_S88_SENSOR_BASE_NODE, CONFIG_GPIO_S88_FIRST_SENSOR
                 , JSON_S88_NODE, CONFIG_GPIO_S88 ? "true" : "false"
                 , JSON_OUTPUTS_NODE, CONFIG_GPIO_OUTPUTS ? "true" : "false"
                 , JSON_SENSORS_NODE, CONFIG_GPIO_SENSORS ? "true" : "false");
    return new JsonResponse(features);
  });
  httpd->uri("/version", [&](HttpRequest *req)
  {
    const esp_app_desc_t *app_data = esp_ota_get_app_description();
    const esp_partition_t *partition = esp_ota_get_running_partition();
    string version =
      StringPrintf("{\"version\":\"%s\",\"build\":\"%s\","
                    "\"timestamp\":\"%s %s\",\"ota\":\"%s\",\"uptime\":%llu}"
                 , CONFIG_ESP32CS_SW_VERSION, app_data->version
                 , app_data->date, app_data->time
                 , partition->label, esp_timer_get_time());
    return new JsonResponse(version);
  });
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
  httpd->uri("/config", HttpMethod::GET | HttpMethod::POST, process_config);
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

WEBSOCKET_STREAM_HANDLER_IMPL(process_websocket_event, client, event, data
                            , data_len)
{
  OSMutexLock h(&webSocketLock);
  if (event == WebSocketEvent::WS_EVENT_CONNECT)
  {
    webSocketClients.push_back(
      std::make_unique<WebSocketClient>(client->id(), client->ip()));
  }
  else if (event == WebSocketEvent::WS_EVENT_DISCONNECT)
  {
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
      // TODO: remove cast that drops const
      auto res = (*ent)->feed((uint8_t *)data, data_len);
      if (res.length())
      {
        client->send_text(res);
      }
    }
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

HTTP_HANDLER_IMPL(process_config, request)
{
  auto stackManager = Singleton<esp32cs::LCCStackManager>::instance();
  auto wifiManager = Singleton<esp32cs::LCCWiFiManager>::instance();
  bool needReboot = false;

  if (request->has_param("reset"))
  {
    // this will wipe all persistent config
    Singleton<FileSystemManager>::instance()->force_factory_reset();
    needReboot = true;
  }
  else if (request->has_param("scan"))
  {
    return new JsonResponse(wifiManager->wifi_scan_json(true));
  }
  if (request->has_param("ssid"))
  {
    wifiManager->reconfigure_station(request->param("ssid"), request->param("password"));
    needReboot = true;
  }
  if (request->has_param("mode"))
  {
    wifiManager->reconfigure_mode(request->param("mode"));
    needReboot = true;
  }
  if (request->has_param("nodeid") &&
      stackManager->set_node_id(request->param("nodeid")))
  {
    needReboot = true;
  }
  if (request->has_param("lcc-can"))
  {
    stackManager->reconfigure_can(request->param("lcc-can", false));
    needReboot = true;
  }
  if (request->has_param("lcc-hub"))
  {
    configListener->reconfigure_lcc_hub(request->param("lcc-hub", false));
  }
  if (request->has_param("tx_power"))
  {
    configListener->reconfigure_wifi_tx_power(request->param("tx_power", 78));
  }
  if (request->has_param("uplink-clear-last"))
  {
    configListener->clear_last_uplink();
  }
  if (request->has_param("uplink-enabled"))
  {
    if (request->param("uplink-enabled", true))
    {
      Singleton<esp32cs::LCCWiFiManager>::instance()->enable_uplink();
    }
    else
    {
      Singleton<esp32cs::LCCWiFiManager>::instance()->disable_uplink();
    }
    configListener->cache_config();
  }
  if (request->has_param("uplink-service") &&
      request->has_param("uplink-manual") &&
      request->has_param("uplink-manual-port"))
  {
    // WiFi uplink settings do not require a reboot
    string uplink_service = request->param("uplink-service");
    uint16_t manual_port =
      request->param("uplink-manual-port", TcpClientDefaultParams::DEFAULT_PORT);
    string manual_host = request->param("uplink-manual");
    configListener->reconfigure_uplink(uplink_service, manual_host, manual_port);
  }
  if (request->has_param("ops-short") &&
      request->has_param("ops-short-clear") &&
      request->has_param("ops-shutdown") &&
      request->has_param("ops-shutdown-clear"))
  {
    configListener->reconfigure_hbridge(esp32cs::OPS_CDI_TRACK_OUTPUT_IDX
                                      , request->param("ops-short")
                                      , request->param("ops-short-clear")
                                      , request->param("ops-shutdown")
                                      , request->param("ops-shutdown-clear"));
  }
  if (request->has_param("prog-short") &&
      request->has_param("prog-short-clear") &&
      request->has_param("prog-shutdown") &&
      request->has_param("prog-shutdown-clear"))
  {
    configListener->reconfigure_hbridge(esp32cs::PROG_CDI_TRACK_OUTPUT_IDX
                                      , request->param("prog-short")
                                      , request->param("prog-short-clear")
                                      , request->param("prog-shutdown")
                                      , request->param("prog-shutdown-clear"));
  }

  if (needReboot)
  {
    // send a string back to the client rather than SEND_GENERIC_RESPONSE
    // so we don't return prior to calling reboot.
    return new JsonResponse("{\"restart\":\"ESP32CommandStation Restarting!\"}");
  }

  string response =
    StringPrintf("{%s,\n  %s,\n%s\n}", stackManager->get_config_json().c_str()
                , wifiManager->get_config_json().c_str()
                , configListener->get_config_json().c_str());
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

  for (uint8_t funcID = 0; funcID < DCC_MAX_FN; funcID++)
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

#define GET_LOCO_VIA_EXECUTOR(NAME, address)                                          \
  openlcb::TrainImpl *NAME = nullptr;                                                 \
  {                                                                                   \
    Singleton<esp32cs::LCCStackManager>::instance()->stack()->executor()->sync_run(   \
    [&]()                                                                             \
    {                                                                                 \
      NAME = Singleton<commandstation::AllTrainNodes>::instance()->get_train_impl(    \
                                        commandstation::DccMode::DCC_128, address);   \
    });                                                                               \
  }

#define REMOVE_LOCO_VIA_EXECUTOR(address)                                             \
  {                                                                                   \
    Singleton<esp32cs::LCCStackManager>::instance()->stack()->executor()->sync_run(   \
    [&]()                                                                             \
    {                                                                                 \
      Singleton<commandstation::AllTrainNodes>::instance()->remove_train_impl(address);\
    });                                                                               \
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
    esp32cs::initiate_estop();
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
