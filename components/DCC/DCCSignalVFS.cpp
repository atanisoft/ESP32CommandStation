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

#include "PrioritizedUpdateLoop.hxx"
#include "TrackOutputDescriptor.hxx"
#include <hardware.hxx>
#include <os/Gpio.hxx>

#include <AllTrainNodes.hxx>
#include <dcc/DccOutput.hxx>
#include <dcc/LocalTrackIf.hxx>
#include <dcc/ProgrammingTrackBackend.hxx>
#include <dcc/RailCom.hxx>
#include <dcc/RailcomHub.hxx>
#include <dcc/RailcomPortDebug.hxx>
#include <driver/periph_ctrl.h>
#include <driver/rmt.h>
#include <driver/timer.h>
#include <driver/uart.h>
#include <esp_vfs.h>
#include <executor/PoolToQueueFlow.hxx>
#include <freertos_drivers/arduino/DummyGPIO.hxx>
#include <freertos_drivers/esp32/Esp32Gpio.hxx>
#include <map>
#include <openlcb/CallbackEventHandler.hxx>
#include <openlcb/EventHandlerTemplates.hxx>
#include <openlcb/MemoryConfig.hxx>
#include <openlcb/Node.hxx>
#include <openlcb/RefreshLoop.hxx>
#include <RMTTrackDevice.hxx>
#include <utils/GpioInitializer.hxx>
#include <utils/logging.h>

namespace esp32cs
{
/// RMT channel to use for track output.
static constexpr rmt_channel_t TRACK_RMT_CHANNEL = RMT_CHANNEL_0;

/// RailCom driver instance.
static NoRailcomDriver railComDriver;
static esp32cs::RMTTrackDevice<DccHwDefs> track(&railComDriver);
static uninitialized<dcc::LocalTrackIf> track_interface;
static uninitialized<esp32cs::PrioritizedUpdateLoop> track_update_loop;
static uninitialized<PoolToQueueFlow<Buffer<dcc::Packet>>> track_flow;
static uninitialized<dcc::RailcomHubFlow> railcom_hub;
#if CONFIG_OPS_RAILCOM_DUMP_PACKETS
static uninitialized<dcc::RailcomPrintfFlow> railcom_dumper;
#endif
static uninitialized<openlcb::CallbackEventHandler> track_events;

/// ESP32 VFS ::write() impl for the RMTTrackDevice.
/// @param fd is the file descriptor being written to.
/// @param data is the data to write.
/// @param size is the size of data.
/// @returns number of bytes written.
static ssize_t dcc_vfs_write(int fd, const void *data, size_t size)
{
  return track.write(fd, data, size);
}

/// ESP32 VFS ::open() impl for the RMTTrackDevice
/// @param path is the file location to be opened.
/// @param flags is not used.
/// @param mode is not used.
/// @returns file descriptor for the opened file location.
static int dcc_vfs_open(const char *path, int flags, int mode)
{
  int fd = TRACK_RMT_CHANNEL;
  LOG(INFO, "[Track:%d] Connecting track interface", fd);
  return fd;
}

/// ESP32 VFS ::close() impl for the RMTTrackDevice.
/// @param fd is the file descriptor to close.
/// @returns the status of the close() operation, only returns zero.
static int dcc_vfs_close(int fd)
{
  LOG(INFO, "[Track:%d] Disconnecting track interface", fd);
  return 0;
}

/// ESP32 VFS ::ioctl() impl for the RMTTrackDevice.
/// @param fd is the file descriptor to operate on.
/// @param cmd is the ioctl command to execute.
/// @param args are the arguments to ioctl.
/// @returns the result of the ioctl command, zero on success, non-zero will
/// set errno.
static int dcc_vfs_ioctl(int fd, int cmd, va_list args)
{
  return track.ioctl(fd, cmd, args);
}

/// RMT transmit complete callback.
///
/// @param channel is the RMT channel that has completed transmission.
/// @param ctx is unused.
///
/// This is called automatically by the RMT peripheral when it reaches the end
/// of TX data.
static void rmt_tx_callback(rmt_channel_t channel, void *ctx)
{
  if (channel == TRACK_RMT_CHANNEL)
  {
    track.rmt_transmit_complete();
  }
}

/// Initializes the RMT based signal generation
///
/// @param param unused.
///
/// Note: this is necessary to ensure the RMT ISRs are started on the second
/// core of the ESP32.
static void init_rmt_outputs(void *param)
{
  Notifiable *notif = static_cast<Notifiable *>(param);

  // Connect our callback into the RMT so we can queue up the next packet for
  // transmission when needed.
  rmt_register_tx_end_callback(rmt_tx_callback, nullptr);

  track.hw_init();

  // tell the main task that the RMT drivers are ready.
  notif->notify();

  // this is a one-time task, shutdown the task before returning
  vTaskDelete(nullptr);
}

// TODO: move this into TrainSearchProtocol
class EStopPacketSource : public dcc::NonTrainPacketSource
{
public:
  bool is_enabled()
  {
    return enabled_;
  }

  void enable()
  {
    LOG(INFO, "[eStop] Received eStop request, sending eStop to all trains.");
    // TODO: add helper method on AllTrainNodes for this.
    auto trains = Singleton<commandstation::AllTrainNodes>::instance();
    for (size_t id = 0; id < trains->size(); id++)
    {
      auto node = trains->get_train_node_id_ext(id, false);
      if (node)
      {
        trains->get_train_impl(node)->set_emergencystop();
      }
    }
    packet_processor_add_refresh_source(this, dcc::UpdateLoopBase::ESTOP_PRIORITY);
    enabled_ = true;
  }
  void disable()
  {
    LOG(INFO, "[eStop] Received eStop clear request.");
    packet_processor_remove_refresh_source(this);
    enabled_ = false;
  }
  void get_next_packet(unsigned code, dcc::Packet* packet)
  {
    packet->set_dcc_speed14(dcc::DccShortAddress(0), true, false
                          , dcc::Packet::EMERGENCY_STOP);
  }
private:
  bool enabled_{false};
} estop_packet_source;

void track_event_received(const openlcb::EventRegistryEntry &registry_entry,
                          openlcb::EventReport *report, BarrierNotifiable *done)
{
  AutoNotify n(done);
  switch(report->event)
  {
    case openlcb::Defs::EMERGENCY_OFF_EVENT:
      DccHwDefs::Output1::clear_disable_reason(
            DccOutput::DisableReason::GLOBAL_EOFF);
      DccHwDefs::Output2::clear_disable_reason(
            DccOutput::DisableReason::GLOBAL_EOFF);
    break;
    case openlcb::Defs::CLEAR_EMERGENCY_OFF_EVENT:
      DccHwDefs::Output1::set_disable_reason(
            DccOutput::DisableReason::GLOBAL_EOFF);
      DccHwDefs::Output2::set_disable_reason(
            DccOutput::DisableReason::GLOBAL_EOFF);
    break;
    case openlcb::Defs::EMERGENCY_STOP_EVENT:
      estop_packet_source.enable();
    break;
    case openlcb::Defs::CLEAR_EMERGENCY_STOP_EVENT:
     estop_packet_source.disable();
    break;
  }
}

/// Initializes the ESP32 VFS adapter for the DCC track interface and the short
/// detection devices.
/// @param node is the OpenLCB node to bind to.
/// @param service is the OpenLCB @ref Service to use for recurring tasks.
/// @param cfg is the CDI element for the track output.
void init_dcc(openlcb::Node *node, Service *svc, const TrackOutputConfig &cfg)
{
  // register the VFS handler as the LocalTrackIf uses this to route DCC
  // packets to the track.
  esp_vfs_t vfs;
  memset(&vfs, 0, sizeof(vfs));
  vfs.flags = ESP_VFS_FLAG_DEFAULT;
  vfs.ioctl = dcc_vfs_ioctl;
  vfs.open = dcc_vfs_open;
  vfs.close = dcc_vfs_close;
  vfs.write = dcc_vfs_write;

  LOG(INFO, "[Track] Registering %s VFS interface", CONFIG_DCC_VFS_MOUNT_POINT);
  ESP_ERROR_CHECK(esp_vfs_register(CONFIG_DCC_VFS_MOUNT_POINT, &vfs, nullptr));

  SyncNotifiable notif;
  // initialize the track signal generators on the second core so that the ISR
  // is bound to that core instead of the first core.
  xTaskCreatePinnedToCore(&init_rmt_outputs, "DCC RMT Init", 2048, &notif,
                          uxTaskPriorityGet(NULL) + 1, nullptr, APP_CPU_NUM);

  // block until the RMT init task completes
  notif.wait_for_notification();

  track_interface.emplace(svc, CONFIG_DCC_PACKET_POOL_SIZE);
  int track = ::open(CONFIG_DCC_VFS_MOUNT_POINT, O_WRONLY);
  track_interface->set_fd(track);
  track_update_loop.emplace(svc, track_interface.get_mutable());

  // Attach the DCC update loop to the track interface
  track_flow.emplace(svc, track_interface->pool()
                   , track_update_loop.get_mutable());

  railcom_hub.emplace(svc);
#if CONFIG_OPS_RAILCOM
//  opsRailComDriver.hw_init(railcom_hub.operator->());
#if CONFIG_OPS_RAILCOM_DUMP_PACKETS
  railcom_dumper.emplace(railcom_hub.operator->());
#endif
#endif // CONFIG_OPS_RAILCOM

  track_events.emplace(node, track_event_received, nullptr);
  track_events->add_entry(openlcb::Defs::EMERGENCY_STOP_EVENT,
    openlcb::CallbackEventHandler::RegistryEntryBits::IS_CONSUMER);
  track_events->add_entry(openlcb::Defs::CLEAR_EMERGENCY_STOP_EVENT,
    openlcb::CallbackEventHandler::RegistryEntryBits::IS_CONSUMER);
  track_events->add_entry(openlcb::Defs::EMERGENCY_OFF_EVENT,
    openlcb::CallbackEventHandler::RegistryEntryBits::IS_CONSUMER);
  track_events->add_entry(openlcb::Defs::CLEAR_EMERGENCY_OFF_EVENT,
    openlcb::CallbackEventHandler::RegistryEntryBits::IS_CONSUMER);

  DccHwDefs::Output1::clear_disable_reason(
        DccOutput::DisableReason::INITIALIZATION_PENDING);
#if CONFIG_ENERGIZE_TRACK_ON_STARTUP
  DccHwDefs::Output1::enable_output();
#endif // CONFIG_ENERGIZE_TRACK_ON_STARTUP
}

void shutdown_dcc()
{
  // disconnect the RMT TX complete callback so that no more DCC packets will
  // be sent to the tracks.
  rmt_register_tx_end_callback(nullptr, nullptr);

  // Disable all track outputs
  DccHwDefs::Output1::set_disable_reason(
        DccOutput::DisableReason::INITIALIZATION_PENDING);
  DccHwDefs::Output2::set_disable_reason(
        DccOutput::DisableReason::INITIALIZATION_PENDING);
}

} // namespace esp32cs

// this must be declared in the global namespace
DccOutput *get_dcc_output(DccOutput::Type type)
{
  switch (type)
  {
    case DccOutput::TRACK:
      return DccOutputImpl<DccHwDefs::Output1>::instance();
    case DccOutput::PGM:
      return DccOutputImpl<DccHwDefs::Output2>::instance();
    case DccOutput::LCC:
      return DccOutputImpl<DccHwDefs::Output3>::instance();
  }
  return nullptr;
}