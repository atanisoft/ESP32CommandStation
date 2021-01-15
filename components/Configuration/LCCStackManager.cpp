/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2020-2021 Mike Dunston

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

#include "LCCStackManager.h"
#include "FileSystemManager.h"
#include "nvs.hxx"
#include <freertos_drivers/esp32/Esp32HardwareTwai.hxx>
#include <openlcb/MemoryConfigClient.hxx>
#include <openlcb/SimpleStack.hxx>
#include <utils/AutoSyncFileFlow.hxx>

namespace esp32cs
{

#if CONFIG_LCC_CAN_RX_PIN != -1 && CONFIG_LCC_CAN_TX_PIN != -1
Esp32HardwareTwai twai(CONFIG_LCC_CAN_RX_PIN, CONFIG_LCC_CAN_TX_PIN);

extern "C" void enter_bootloader()
{
    node_config_t config;
    if (load_config(&config) != ESP_OK)
    {
        default_config(&config);
    }
    config.bootloader_req = true;
    save_config(&config);
    LOG(INFO, "[Bootloader] Rebooting into bootloader");
    reboot();
}
#endif // CONFIG_LCC_CAN_RX_PIN != -1 && CONFIG_LCC_CAN_TX_PIN != -1

LCCStackManager::LCCStackManager(const esp32cs::Esp32ConfigDef &cfg
                               , const uint64_t node_id, bool factory_reset)
                               : cfg_(cfg), nodeID_(node_id)
{
  struct stat statbuf;

  // If we are not currently forcing a factory reset, verify if the LCC config
  // file is the correct size. If it is not the expected size force a factory
  // reset.
  if (!factory_reset &&
      stat(LCC_CONFIG_FILE, &statbuf) != 0 &&
      statbuf.st_size != openlcb::CONFIG_FILE_SIZE)
  {
    LOG(WARNING
      , "[LCC] Corrupt/missing configuration file detected, %s is not the "
        "expected size: %lu vs %zu bytes."
      , LCC_CONFIG_FILE, statbuf.st_size, openlcb::CONFIG_FILE_SIZE);
    factory_reset = true;
  }
  else
  {
    LOG(VERBOSE, "[LCC] node config file(%s) is expected size %lu bytes"
      , LCC_CONFIG_FILE, statbuf.st_size);
  }

  // if the factory reset marker is present or we need to reset due to config
  // file size changes take care of it now.
  if (factory_reset && !stat(LCC_CONFIG_FILE, &statbuf))
  {
      LOG(WARNING, "[LCC] Forcing regeneration of %s", LCC_CONFIG_FILE);
      ERRNOCHECK(LCC_CONFIG_FILE, unlink(LCC_CONFIG_FILE));
  }

  LOG(INFO, "[LCC] Initializing Stack (node-id: %s)"
    , uint64_to_string_hex(nodeID_).c_str());
#ifdef CONFIG_LCC_TCP_STACK
  stack_ = new openlcb::SimpleTcpStack(nodeID_);
#else
  stack_ = new openlcb::SimpleCanStack(nodeID_);
#if CONFIG_LCC_CAN_RX_PIN != -1 && CONFIG_LCC_CAN_TX_PIN != -1
  stack_->executor()->add(new CallbackExecutable([&]
  {
      // Initialize the TWAI driver
      twai.hw_init();
      ((openlcb::SimpleCanStackBase *)stack_)->add_can_port_async("/dev/twai/twai0");
  }));
#endif // CONFIG_LCC_CAN_RX_PIN != -1 && CONFIG_LCC_CAN_TX_PIN != -1
#endif // CONFIG_LCC_TCP_STACK
  memory_client_ =
    new openlcb::MemoryConfigClient(node(), memory_config_handler());
}

openlcb::SimpleStackBase *LCCStackManager::stack()
{
  return stack_;
}

ExecutorBase *LCCStackManager::executor()
{
  return stack_->executor();
}

Service *LCCStackManager::service()
{
  return stack_->service();
}

openlcb::Node *LCCStackManager::node()
{
  return stack_->node();
}

openlcb::SimpleInfoFlow *LCCStackManager::info_flow()
{
  return stack_->info_flow();
}

openlcb::MemoryConfigClient *LCCStackManager::memory_config_client()
{
  return memory_client_;
}

openlcb::MemoryConfigHandler *LCCStackManager::memory_config_handler()
{
  return stack_->memory_config_handler();
}

#ifndef CONFIG_LCC_SD_FSYNC_SEC
#define CONFIG_LCC_SD_FSYNC_SEC 10
#endif

void LCCStackManager::start(bool is_sd, bool reset_event_ids)
{
  // Create the default internal configuration file if it doesn't already exist.
  fd_ =
    stack_->create_config_file_if_needed(cfg_.seg().internal_config()
                                       , CDI_VERSION
                                       , openlcb::CONFIG_FILE_SIZE);
  LOG(INFO, "[LCC] Config file opened using fd:%d", fd_);

  if (is_sd)
  {
    // ESP32 FFat library uses a 512b cache in memory by default for the SD VFS
    // adding a periodic fsync call for the LCC configuration file ensures that
    // config changes are saved since the LCC config file is less than 512b.
    LOG(INFO, "[LCC] Creating automatic fsync(%d) calls every %d seconds."
      , fd_, CONFIG_LCC_SD_FSYNC_SEC);
    configAutoSync_ =
      new AutoSyncFileFlow(stack_->service(), fd_
                         , SEC_TO_USEC(CONFIG_LCC_SD_FSYNC_SEC));
  }
#if defined(CONFIG_LCC_PRINT_ALL_PACKETS) && !defined(CONFIG_LCC_TCP_STACK)
  LOG(INFO, "[LCC] Configuring LCC packet printer");
  ((openlcb::SimpleCanStack *)stack_)->print_all_packets();
#endif

  if (reset_event_ids)
  {
    reset_events();
  }
}

void LCCStackManager::shutdown()
{
  // Shutdown the auto-sync handler if it is running before unmounting the FS.
  if (configAutoSync_ != nullptr)
  {
    LOG(INFO, "[LCC] Disabling automatic fsync(%d) calls...", fd_);
    SyncNotifiable n;
    configAutoSync_->shutdown(&n);
    LOG(INFO, "[LCC] Waiting for sync to stop");
    n.wait_for_notification();
    configAutoSync_ = nullptr;
  }

  // shutdown the executor so that no more tasks will run
  LOG(INFO, "[LCC] Shutting down executor");
  stack_->executor()->shutdown();

  // close the config file if it is open
  if (fd_ >= 0)
  {
    LOG(INFO, "[LCC] Closing config file.");
    ::close(fd_);
  }
}

void LCCStackManager::reboot_node()
{
  stack_->executor()->add(new CallbackExecutable([](){ reboot(); }));
}

void LCCStackManager::reset_events()
{
  stack_->factory_reset_all_events(cfg_.seg().internal_config(), nodeID_, fd_);
}

void LCCStackManager::send_event(uint64_t event)
{
  uint64_t evt = event;
  stack_->executor()->add(new CallbackExecutable(
  [evt, this](){
    LOG(INFO, "[LCC] Sending event: %s", uint64_to_string_hex(evt).c_str());
    stack_->send_event(evt);
  }));
}

} // namespace esp32cs
