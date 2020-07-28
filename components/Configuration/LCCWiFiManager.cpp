/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2020 Mike Dunston

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

#include "LCCWiFiManager.h"
#include "LCCStackManager.h"
#include "JsonConstants.h"

#include "sdkconfig.h"

#include <algorithm>
#include <FileSystemManager.h>
#include <esp_sntp.h>
#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>
#include <HttpStringUtils.h>
#include <openlcb/SimpleStack.hxx>
#include <utime.h>

namespace esp32cs
{

static constexpr const char WIFI_STATION_CFG[] = "wifi-sta";
static constexpr const char WIFI_STATION_DNS_CFG[] = "wifi-dns";
static constexpr const char WIFI_STATION_IP_CFG[] = "wifi-ip";
static constexpr const char WIFI_SOFTAP_CFG[] = "wifi-ap";

#ifndef CONFIG_WIFI_STATIC_IP_ADDRESS
#define CONFIG_WIFI_STATIC_IP_ADDRESS ""
#endif

#ifndef CONFIG_WIFI_STATIC_IP_GATEWAY
#define CONFIG_WIFI_STATIC_IP_GATEWAY ""
#endif

#ifndef CONFIG_WIFI_STATIC_IP_SUBNET
#define CONFIG_WIFI_STATIC_IP_SUBNET ""
#endif

#ifndef CONFIG_WIFI_STATIC_IP_DNS
#define CONFIG_WIFI_STATIC_IP_DNS ""
#endif

#ifndef CONFIG_WIFI_SOFTAP_SSID
#define CONFIG_WIFI_SOFTAP_SSID "esp32cs"
#endif

#ifndef CONFIG_WIFI_SOFTAP_PASSWORD
#define CONFIG_WIFI_SOFTAP_PASSWORD "esp32cs"
#endif

#ifndef CONFIG_WIFI_SSID
#define CONFIG_WIFI_SSID "esp32cs"
#endif

#ifndef CONFIG_WIFI_PASSWORD
#define CONFIG_WIFI_PASSWORD "esp32cs"
#endif

#if CONFIG_SNTP

static bool sntp_callback_called_previously = false;
static void sntp_received(struct timeval *tv)
{
  // if this is the first time we have been called, check if the modification
  // timestamp on the LCC_CDI_XML is older than 2020-01-01 and if so reset the
  // modification/access timestamp to current.
  if (!sntp_callback_called_previously)
  {
    struct stat statbuf;
    stat(LCC_CDI_XML, &statbuf);
    time_t mod = statbuf.st_mtime;
    struct tm timeinfo;
    localtime_r(&mod, &timeinfo);
    // check if the timestamp on LCC_CDI_XML is prior to 2020 and if so, force
    // the access/modified timestamps to the current time.
    if (timeinfo.tm_year < (2020 - 1900))
    {
      LOG(INFO, "[SNTP] Updating timestamps on CDI XML files");
      utime(LCC_CDI_XML, NULL);
      utime("/cfg/LCC/train.xml", NULL);
      utime("/cfg/LCC/tmptrain.xml", NULL);
    }
    sntp_callback_called_previously = true;
  }
  time_t new_time = tv->tv_sec;
  LOG(INFO, "[SNTP] Received time update, new localtime: %s", ctime(&new_time));

#if CONFIG_FASTCLOCK_REALTIME
  Singleton<LCCWiFiManager>::instance()->real_time_clock_sync(new_time);
#endif // CONFIG_FASTCLOCK_REALTIME
}
#endif // CONFIG_SNTP

LCCWiFiManager::LCCWiFiManager(openlcb::SimpleStackBase *stack
                             , const esp32cs::Esp32ConfigDef &cfg)
                             : stack_(stack), cfg_(cfg)
{
  auto fs = Singleton<FileSystemManager>::instance();
  if (!fs->exists(WIFI_SOFTAP_CFG) && !fs->exists(WIFI_STATION_CFG))
  {
#if defined(CONFIG_WIFI_MODE_SOFTAP)
    reconfigure_mode(JSON_VALUE_WIFI_MODE_SOFTAP_ONLY, false);
#elif defined(CONFIG_WIFI_MODE_SOFTAP_STATION)
    reconfigure_mode(JSON_VALUE_WIFI_MODE_SOFTAP_STATION, false);
#else
    reconfigure_mode(JSON_VALUE_WIFI_MODE_STATION_ONLY, false);
#endif
#if defined(CONFIG_WIFI_MODE_SOFTAP_STATION) || \
    defined(CONFIG_WIFI_MODE_STATION)
    reconfigure_station(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD
                      , CONFIG_WIFI_STATIC_IP_ADDRESS
                      , CONFIG_WIFI_STATIC_IP_GATEWAY
                      , CONFIG_WIFI_STATIC_IP_SUBNET
                      , CONFIG_WIFI_STATIC_IP_DNS, false);
#endif // CONFIG_WIFI_MODE_SOFTAP_STATION || CONFIG_WIFI_MODE_STATION
  }

  LOG(INFO, "[WiFi] Loading configuration");
  if (fs->exists(WIFI_SOFTAP_CFG) && !fs->exists(WIFI_STATION_CFG))
  {
    mode_ =  WIFI_MODE_AP;
    string station_cfg = fs->load(WIFI_SOFTAP_CFG);
    std::pair<string, string> cfg = http::break_string(station_cfg, "\n");
    ssid_ = cfg.first;
    password_ = cfg.second;
    LOG(INFO, "[WiFi] SoftAP only (ssid: %s)", ssid_.c_str());
  }
  else if (fs->exists(WIFI_SOFTAP_CFG) &&
           fs->exists(WIFI_STATION_CFG))
  {
    LOG(INFO, "[WiFi] SoftAP and Station");
    mode_ =  WIFI_MODE_APSTA;
  }
  else if (fs->exists(WIFI_STATION_CFG))
  {
    LOG(INFO, "[WiFi] Station only");
    mode_ =  WIFI_MODE_STA;
  }
  else
  {
    LOG(INFO
      , "[WiFi] Unable to locate SoftAP or Station configuration, enabling "
        "SoftAP: %s", CONFIG_WIFI_SOFTAP_SSID);
    mode_ =  WIFI_MODE_AP;
    ssid_ = CONFIG_WIFI_SOFTAP_SSID;
    password_ = CONFIG_WIFI_SOFTAP_PASSWORD;
  }
  if (mode_ != WIFI_MODE_AP)
  {
    string station_cfg = fs->load(WIFI_STATION_CFG);
    std::pair<string, string> cfg = http::break_string(station_cfg, "\n");
    ssid_ = cfg.first;
    password_ = cfg.second;
    if (fs->exists(WIFI_STATION_IP_CFG))
    {
      std::vector<string> ip_parts;
      string ip_cfg = fs->load(WIFI_STATION_IP_CFG);
      http::tokenize(ip_cfg, ip_parts, "\n");
      stationIP_.reset(new tcpip_adapter_ip_info_t());
      stationIP_->ip.addr = ipaddr_addr(ip_parts[0].c_str());
      stationIP_->gw.addr = ipaddr_addr(ip_parts[1].c_str());
      stationIP_->netmask.addr = ipaddr_addr(ip_parts[2].c_str());
      LOG(INFO, "[WiFi] Static IP:" IPSTR ", gateway:" IPSTR ",netmask:" IPSTR,
        IP2STR(&stationIP_->ip), IP2STR(&stationIP_->gw), IP2STR(&stationIP_->netmask));
    }
    if (fs->exists(WIFI_STATION_DNS_CFG))
    {
      string dns = fs->load(WIFI_STATION_DNS_CFG);
      stationDNS_.u_addr.ip4.addr = ipaddr_addr(dns.c_str());
      LOG(INFO, "[WiFi] DNS configured: " IPSTR
        , IP2STR(&stationDNS_.u_addr.ip4));
    }
  }
  else if (fs->exists(WIFI_SOFTAP_CFG))
  {
    string station_cfg = fs->load(WIFI_SOFTAP_CFG);
    std::pair<string, string> cfg = http::break_string(station_cfg, "\n");
    ssid_ = cfg.first;
    password_ = cfg.second;
  }

  LOG(INFO, "[WiFi] Starting WiFiManager");
  wifi_.reset(
    new Esp32WiFiManager(ssid_.c_str(), password_.c_str()
                       , stack_, cfg_.seg().wifi(), CONFIG_HOSTNAME_PREFIX
                       , mode_, stationIP_.get(), stationDNS_
                       , CONFIG_WIFI_SOFT_AP_CHANNEL));

#if CONFIG_WIFI_DEBUG_OUTPUT
  wifi_->enable_verbose_logging();
#endif // CONFIG_WIFI_DEBUG_OUTPUT

  // When operating as both SoftAP and Station mode it is not necessary to wait
  // for the station to be UP during CS startup.
  if (mode_ == WIFI_MODE_APSTA)
  {
    wifi_->wait_for_ssid_connect(false);
  }

#if CONFIG_FASTCLOCK_REALTIME
  LOG(INFO, "[FastClock] Creating Real-Time fast clock instance with ID %s"
    , uint64_to_string_hex(CONFIG_FASTCLOCK_REALTIME_ID).c_str());
  realTimeClock_.reset(
    new openlcb::BroadcastTimeServer(stack_->node()
                                  , UINT64_C(CONFIG_FASTCLOCK_REALTIME_ID)));
  realTimeClock_->set_rate_quarters(4);
#endif // CONFIG_FASTCLOCK_REALTIME

#if CONFIG_SNTP
  if (mode_ == WIFI_MODE_APSTA || mode_ == WIFI_MODE_STA)
  {
    LOG(INFO, "[SNTP] Polling %s for time updates", CONFIG_SNTP_SERVER);
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, CONFIG_SNTP_SERVER);
    sntp_set_time_sync_notification_cb(sntp_received);
    sntp_init();
  }
#endif // CONFIG_SNTP
}

void LCCWiFiManager::shutdown()
{
  wifi_.reset(nullptr);
}

void LCCWiFiManager::reconfigure_mode(string mode, bool restart)
{
  auto fs = Singleton<FileSystemManager>::instance();
  if (!mode.compare(JSON_VALUE_WIFI_MODE_SOFTAP_ONLY))
  {
    fs->remove(WIFI_STATION_CFG);
    string soft_ap_cfg = StringPrintf("%s\n%s", CONFIG_WIFI_SOFTAP_SSID
                                    , CONFIG_WIFI_SOFTAP_PASSWORD);
    fs->store(WIFI_SOFTAP_CFG, soft_ap_cfg);
  }
  else if (!mode.compare(JSON_VALUE_WIFI_MODE_SOFTAP_STATION))
  {
    string soft_ap_cfg = StringPrintf("%s\n%s", CONFIG_WIFI_SOFTAP_SSID
                                    , CONFIG_WIFI_SOFTAP_PASSWORD);
    fs->store(WIFI_SOFTAP_CFG, soft_ap_cfg);
  }
  else
  {
    fs->remove(WIFI_SOFTAP_CFG);
  }
  if (restart)
  {
    stack_->executor()->add(new CallbackExecutable([]()
    {
      reboot();
    }));
  }
}

void LCCWiFiManager::reconfigure_station(string ssid, string password
                                       , string ip, string gateway
                                       , string subnet, string dns
                                       , bool restart)
{
  LOG(VERBOSE, "[WiFi] reconfigure_station(%s,%s,%s,%s,%s,%s,%d)", ssid.c_str()
    , password.c_str(), ip.c_str(), gateway.c_str(), subnet.c_str()
    , dns.c_str(), restart);
  auto fs = Singleton<FileSystemManager>::instance();
  string station_cfg = StringPrintf("%s\n%s", ssid.c_str(), password.c_str());
  fs->store(WIFI_STATION_CFG, station_cfg);

  if (!ip.empty() && !gateway.empty() && !subnet.empty())
  {
    string ip_cfg = StringPrintf("%s\n%s\n%s\n", ip.c_str()
                                , gateway.c_str(), subnet.c_str());
    fs->store(WIFI_STATION_IP_CFG, ip_cfg);
  }
  else
  {
    fs->remove(WIFI_STATION_IP_CFG);
  }
  if (!dns.empty())
  {
    fs->store(WIFI_STATION_DNS_CFG, dns);
  }
  else
  {
    fs->remove(WIFI_STATION_DNS_CFG);
  }

  if (restart)
  {
    stack_->executor()->add(new CallbackExecutable([]()
    {
      reboot();
    }));
  }
}

string LCCWiFiManager::wifi_scan_json(bool ignore_duplicates)
{
  string result = "[";
  SyncNotifiable n;
  wifi_->start_ssid_scan(&n);
  n.wait_for_notification();
  size_t num_found = wifi_->get_ssid_scan_result_count();
  vector<string> seen_ssids;
  for (int i = 0; i < num_found; i++)
  {
    auto entry = wifi_->get_ssid_scan_result(i);
    if (ignore_duplicates)
    {
      if (std::find_if(seen_ssids.begin(), seen_ssids.end()
        , [entry](string &s)
          {
            return s == (char *)entry.ssid;
          }) != seen_ssids.end())
      {
        // filter duplicate SSIDs
        continue;
      }
      seen_ssids.push_back((char *)entry.ssid);
    }
    if (result.length() > 1)
    {
      result += ",";
    }
    LOG(VERBOSE, "auth:%d,rssi:%d,ssid:%s", entry.authmode, entry.rssi, entry.ssid);
    result += StringPrintf("{\"auth\":%d,\"rssi\":%d,\"ssid\":\"%s\"}",
                           entry.authmode, entry.rssi,
                           http::url_encode((char *)entry.ssid).c_str());
  }
  result += "]";
  wifi_->clear_ssid_scan_results();
  return result;
}

string LCCWiFiManager::get_config_json()
{
  auto fs = Singleton<FileSystemManager>::instance();
  string config = StringPrintf("\"wifi\":{"
                                "\"mode\":\"%s\""
                              , mode_ == WIFI_MODE_AP ? "softap"
                              : mode_ == WIFI_MODE_APSTA ? "softap-station" : "station");
  if (mode_ != WIFI_MODE_AP)
  {
    config += StringPrintf(",\"station\":{\"ssid\":\"%s\",\"password\":\"%s\""
                         , ssid_.c_str(), password_.c_str());
    if (fs->exists(WIFI_STATION_DNS_CFG))
    {
      config += ",\"dns\":\"" + fs->load(WIFI_STATION_DNS_CFG) + "\"";
    }
    if (stationIP_.get() != nullptr)
    {
      config += StringPrintf(",\"ip\":\"" IPSTR "\",\"gateway\":\"" IPSTR "\","
                             "\"netmask\":\"" IPSTR "\""
                            , IP2STR(&stationIP_->ip), IP2STR(&stationIP_->gw)
                            , IP2STR(&stationIP_->netmask));
    }
    config += "}";
  }
  config += "}";
  return config;
}

#if CONFIG_FASTCLOCK_REALTIME
void LCCWiFiManager::real_time_clock_sync(time_t sync_time)
{
  LOG(INFO, "[FastClock] Time sync: %s", ctime(&sync_time));
  struct tm timeinfo;
  localtime_r(&sync_time, &timeinfo);
  realTimeClock_->set_time(timeinfo.tm_hour, timeinfo.tm_min);
  realTimeClock_->set_date(timeinfo.tm_mon + 1, timeinfo.tm_mday);
  realTimeClock_->set_year(timeinfo.tm_year + 1900);
  if (!realTimeClock_->is_running())
  {
    realTimeClock_->start();
    LOG(INFO, "[FastClock] Starting real-time clock");
  }
  else
  {
    LOG(INFO, "[FastClock] real-time clock synced");
  }
}
#endif // CONFIG_FASTCLOCK_REALTIME

} // namespace esp32cs