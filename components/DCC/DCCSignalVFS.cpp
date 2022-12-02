/*
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: GPL-3.0
 * 
 * This file is part of ESP32 Command Station.
 */

#include "sdkconfig.h"
#include <hardware.hxx>

#if CONFIG_RAILCOM_CUT_OUT_ENABLED
#include "Esp32RailComDriver.hxx"
#endif 
#include "PrioritizedUpdateLoop.hxx"
#include "TrackOutputDescriptor.hxx"
#include "TrackPowerHandler.hxx"

#include <AccessoryDecoderDatabase.hxx>
#include <locomgr/LocoManager.hxx>
#include <dcc/DccOutput.hxx>
#include <dcc/LocalTrackIf.hxx>
#include <dcc/ProgrammingTrackBackend.hxx>
#include <dcc/RailCom.hxx>
#include <dcc/RailcomHub.hxx>
#include <dcc/RailcomPortDebug.hxx>
#include <driver/adc.h>
#include <driver/rtc_cntl.h>
#include <driver/rmt.h>
#include <driver/timer.h>
#include <driver/uart.h>
#include <esp_vfs.h>
#include <EventBroadcastHelper.hxx>
#include <executor/PoolToQueueFlow.hxx>
#include <freertos_drivers/arduino/DummyGPIO.hxx>
#include <freertos_drivers/esp32/Esp32Gpio.hxx>
#include <map>
#include <openlcb/EventHandlerTemplates.hxx>
#include <openlcb/MemoryConfig.hxx>
#include <openlcb/Node.hxx>
#include <openlcb/RefreshLoop.hxx>
#include <os/Gpio.hxx>
#include <RMTTrackDevice.hxx>
#include <soc/rtc_cntl_reg.h>
#include <StatusDisplay.hxx>
#include <utils/StringUtils.hxx>
#include <UlpAdc.hxx>
#include <utils/GpioInitializer.hxx>
#include <utils/logging.h>

namespace esp32cs
{

#if CONFIG_PROG_TRACK_ENABLED
/// Disables the OPS track output and enables the PROG track output.
static void enable_programming_track()
{
  DccHwDefs::InternalBoosterOutput::set_disable_reason(
    DccOutput::DisableReason::PGM_TRACK_LOCKOUT);
  DccHwDefs::OpenLCBBoosterOutput::set_disable_reason(
    DccOutput::DisableReason::PGM_TRACK_LOCKOUT);

  PROG_ENABLE_Pin::set(true);
}

/// Disables the PROG track output and enables the OPS track output.
static void disable_programming_track()
{
  PROG_ENABLE_Pin::set(false);

  DccHwDefs::InternalBoosterOutput::clear_disable_reason(
    DccOutput::DisableReason::PGM_TRACK_LOCKOUT);
  DccHwDefs::OpenLCBBoosterOutput::clear_disable_reason(
    DccOutput::DisableReason::PGM_TRACK_LOCKOUT);
}
#endif // CONFIG_PROG_TRACK_ENABLED

class EStopPacketSource : public dcc::NonTrainPacketSource,
                          public openlcb::BitEventInterface
{
public:
  EStopPacketSource(openlcb::Node *node)
    : openlcb::BitEventInterface(openlcb::Defs::EMERGENCY_STOP_EVENT,
                                 openlcb::Defs::CLEAR_EMERGENCY_STOP_EVENT),
    node_(node)
  {
    LOG(INFO, "[eStop] Registering OpenLCB event consumer (On:%s, Off:%s)",
        utils::event_id_to_string(event_on()).c_str(),
        utils::event_id_to_string(event_off()).c_str());
  }

  bool is_enabled()
  {
    return enabled_;
  }

  openlcb::EventState get_current_state() override
  {
    LOG(VERBOSE, "[eStop] Query event state: %d", is_enabled());
    if (is_enabled())
    {
      LOG(VERBOSE, "[eStop] ON (%s)",
          utils::event_id_to_string(event_on()).c_str());
      return openlcb::EventState::VALID;
    }
    LOG(VERBOSE, "[eStop] OFF (%s)",
        utils::event_id_to_string(event_off()).c_str());
    return openlcb::EventState::INVALID;
  }

  void set_state(bool new_value) override
  {
    if (enabled_ == new_value)
    {
      // discard attempt to set the state to the current state
      return;
    }
    if (new_value)
    {
      LOG(INFO, "[eStop] Received eStop request, sending eStop to all trains.");
      Singleton<locomgr::LocoManager>::instance()->estop_all_trains();
      packet_processor_add_refresh_source(this, dcc::UpdateLoopBase::ESTOP_PRIORITY);
    }
    else
    {
      LOG(INFO, "[eStop] Received eStop clear request.");
      packet_processor_remove_refresh_source(this);
    }
    enabled_ = new_value;
  }

  openlcb::Node *node()
  {
    return node_;
  }

  void get_next_packet(unsigned code, dcc::Packet* packet)
  {
    packet->set_dcc_speed14(dcc::DccShortAddress(0), true, false,
                            dcc::Packet::EMERGENCY_STOP);
  }
private:
  bool enabled_{false};
  openlcb::Node *node_;
};

#if CONFIG_RAILCOM_CUT_OUT_ENABLED
static uninitialized<dcc::RailcomHubFlow> railcom_hub;
#if CONFIG_RAILCOM_DUMP_PACKETS
static uninitialized<dcc::RailcomPrintfFlow> railcom_dumper;
#endif // CONFIG_RAILCOM_DUMP_PACKETS
static esp32cs::Esp32RailComDriver<RailComHwDefs, DccHwDefs::InternalBoosterOutput, DccHwDefs::OpenLCBBoosterOutput> railComDriver;
#else
static NoRailcomDriver railComDriver;
#endif // CONFIG_RAILCOM_CUT_OUT_ENABLED
static esp32cs::RMTTrackDevice<DccHwDefs, DccHwDefs::InternalBoosterOutput, DccHwDefs::OpenLCBBoosterOutput> track(&railComDriver);
static uninitialized<dcc::LocalTrackIf> track_interface;
static uninitialized<esp32cs::PrioritizedUpdateLoop> track_update_loop;
static uninitialized<PoolToQueueFlow<Buffer<dcc::Packet>>> track_flow;
static uninitialized<TrackPowerBit<DccHwDefs::InternalBoosterOutput, DccHwDefs::OpenLCBBoosterOutput>> track_power;
static uninitialized<openlcb::BitEventConsumer> track_power_consumer;
static uninitialized<EStopPacketSource> estop_packet_source;
static uninitialized<openlcb::BitEventConsumer> estop_consumer;
#if CONFIG_PROG_TRACK_ENABLED
static uninitialized<ProgrammingTrackBackend> prog_backend;
#endif
static uninitialized<esp32cs::AccessoryDecoderDB> accessory_db;

#if CONFIG_OPS_TRACK_ENABLED
class TrackMonitorFlow : public StateFlowBase, public DefaultConfigUpdateListener
{
public:
  TrackMonitorFlow(Service *service, const TrackOutputConfig &cfg)
    : StateFlowBase(service), cfg_(cfg)
  {
    auto status = Singleton<StatusDisplay>::instance();
    status->track_power("Track: Off");
    start_flow(STATE(sleep));
  }

  UpdateAction apply_configuration(int fd, bool initial_load,
                                   BarrierNotifiable *done) override
  {
    AutoNotify n(done);
    shortEvent_ = cfg_.event_short().read(fd);
    shutdownEvent_ = cfg_.event_shutdown().read(fd);
    return UPDATED;
  }

  void factory_reset(int fd) override
  {
    // TODO: move this out of track monitor
    CDI_FACTORY_RESET(cfg_.ops_current_limit);
    CDI_FACTORY_RESET(cfg_.advanced().enable_railcom);
    CDI_FACTORY_RESET(cfg_.advanced().enable_railcom_receiver);
    CDI_FACTORY_RESET(cfg_.advanced().ops_preamble_bits);
    CDI_FACTORY_RESET(cfg_.advanced().prog_preamble_bits);
  }

private:
  StateFlowTimer timer_{this};
  TrackOutputConfig cfg_;
  openlcb::EventId shortEvent_;
  openlcb::EventId shutdownEvent_;
  static constexpr uint64_t INTERVAL = SEC_TO_NSEC(15);

  Action check()
  {
    auto status = Singleton<StatusDisplay>::instance();
    uint16_t last_reading = esp32cs::get_last_ops_reading();
    uint16_t short_limit = esp32cs::get_ops_short_threshold();
    uint16_t shutdown_limit = esp32cs::get_ops_shutdown_threshold();
    uint16_t warn_limit = esp32cs::get_ops_warning_threshold();
    uint8_t disable_reason =
      DccHwDefs::InternalBoosterOutput::outputDisableReasons_;
    bool was_shorted =
      (disable_reason & (uint8_t)DccOutput::DisableReason::SHORTED);

    // If the last reading is under the short limit and the output is diabled
    // due to the short condition, clear it.
    if (was_shorted && last_reading < short_limit)
    {
      DccHwDefs::InternalBoosterOutput::clear_disable_reason(
        DccOutput::DisableReason::SHORTED);
    }

    if (last_reading > short_limit)
    {
      status->track_power("Track: Short!");
      if (last_reading < shutdown_limit)
      {
        Singleton<esp32cs::EventBroadcastHelper>::instance()->send_event(shortEvent_);
      }
      else
      {
        Singleton<esp32cs::EventBroadcastHelper>::instance()->send_event(shutdownEvent_);
      }
    }
    else if (estop_packet_source->is_enabled())
    {
      status->track_power("Track: e-stop");
    }
    else if (DccHwDefs::InternalBoosterOutput::should_be_enabled())
    {
      LOG(INFO, "[Track] Usage: %" PRIu16 "/%" PRIu16 ", %" PRIu32 " mA",
          last_reading, short_limit, esp32cs::get_ops_load());
      status->track_power("Track: %d mA%c", esp32cs::get_ops_load(),
                          last_reading > warn_limit ? '!' : ' ');
    }
    else
    {
      status->track_power("Track: Off");
    }
    return call_immediately(STATE(sleep));
  }

  Action sleep()
  {
      return sleep_and_call(&timer_, INTERVAL, STATE(check));
  }
};

static uninitialized<TrackMonitorFlow> track_monitor;
#endif // CONFIG_OPS_TRACK_ENABLED

/// ESP32 VFS ::write() impl for the RMTTrackDevice.
///
/// @param fd is the file descriptor being written to.
/// @param data is the data to write.
/// @param size is the size of data.
/// @returns number of bytes written.
static ssize_t dcc_vfs_write(int fd, const void *data, size_t size)
{
  return track.write(fd, data, size);
}

/// ESP32 VFS ::open() impl for the RMTTrackDevice
///
/// @param path is the file location to be opened.
/// @param flags is not used.
/// @param mode is not used.
///
/// @returns file descriptor for the opened file location.
static int dcc_vfs_open(const char *path, int flags, int mode)
{
  int fd = DccHwDefs::RMT_CHANNEL;
  LOG(INFO, "[Track:%d] Connecting track interface", fd);
  return fd;
}

/// ESP32 VFS ::close() impl for the RMTTrackDevice.
///
/// @param fd is the file descriptor to close.
///
/// @returns the status of the close() operation, only returns zero.
static int dcc_vfs_close(int fd)
{
  LOG(INFO, "[Track:%d] Disconnecting track interface", fd);
  return 0;
}

/// ESP32 VFS ::ioctl() impl for the RMTTrackDevice.
///
/// @param fd is the file descriptor to operate on.
/// @param cmd is the ioctl command to execute.
/// @param args are the arguments to ioctl.
///
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
  if (channel == DccHwDefs::RMT_CHANNEL)
  {
    track.rmt_transmit_complete();
  }
}

/// Initializes the ESP32 VFS adapter for the DCC track interface and the short
/// detection devices.
///
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

  // Connect our callback into the RMT so we can queue up the next packet for
  // transmission when needed.
  rmt_register_tx_end_callback(rmt_tx_callback, nullptr);

  // Initialize the RMT signal generator.
  track.hw_init();

#if CONFIG_OPS_TRACK_ENABLED
  LOG(INFO, "[OPS] EN/PWM: %d,"
#if CONFIG_DCC_TRACK_BRAKE_PIN != -1
            " Brake Pin: %d,"
#endif // CONFIG_DCC_TRACK_BRAKE_PIN != -1
#if CONFIG_OPSTRACK_ADC_I2C_CHANNEL_1
            " Current Sense I2C: 1"
#elif CONFIG_OPSTRACK_ADC_I2C_CHANNEL_2
            " Current Sense I2C: 2"
#elif CONFIG_OPSTRACK_ADC_I2C_CHANNEL_3
            " Current Sense I2C: 3"
#else
            " Current Sense Pin: %d (ADC1:%d)"
#endif // OPSTRACK_ADC_I2C_CHANNEL_1 / 2 / 3
    , CONFIG_OPS_TRACK_ENABLE_PIN
#if CONFIG_DCC_TRACK_BRAKE_PIN != -1
    , CONFIG_DCC_TRACK_BRAKE_PIN
#endif // CONFIG_DCC_TRACK_BRAKE_PIN != -1
#if !USE_I2C_FOR_OPS_CURRENT_SENSE
    , OPS_CURRENT_SENSE_Pin::pin()
    , OPS_CURRENT_SENSE_Pin::channel()
#endif // !USE_I2C_FOR_OPS_CURRENT_SENSE
);
#endif // CONFIG_OPS_TRACK_ENABLED

#if CONFIG_PROG_TRACK_ENABLED
  LOG(INFO, "[PROG] EN/PWM: %d,"
#if CONFIG_PROGTRACK_ADC_I2C_CHANNEL_1
            " Current Sense I2C: 1"
#elif CONFIG_PROGTRACK_ADC_I2C_CHANNEL_2
            " Current Sense I2C: 2"
#elif CONFIG_PROGTRACK_ADC_I2C_CHANNEL_3
            " Current Sense I2C: 3"
#else
            " Current Sense Pin: %d (ADC1:%d)"
#endif // PROGTRACK_ADC_I2C_CHANNEL_1 / 2 / 3
    , CONFIG_PROG_TRACK_ENABLE_PIN
#if !USE_I2C_FOR_PROG_CURRENT_SENSE
    , PROG_CURRENT_SENSE_Pin::pin()
    , PROG_CURRENT_SENSE_Pin::channel()
#endif // !USE_I2C_FOR_OPS_CURRENT_SENSE
);
#endif // CONFIG_OPS_TRACK_ENABLED

  track_interface.emplace(svc, CONFIG_DCC_PACKET_POOL_SIZE);
  track_interface->set_fd(open(CONFIG_DCC_VFS_MOUNT_POINT, O_WRONLY));
  track_update_loop.emplace(svc, track_interface.operator->());

  // Attach the DCC update loop to the track interface
  track_flow.emplace(svc, track_interface->pool(),
                     track_update_loop.operator->());

#if CONFIG_RAILCOM_CUT_OUT_ENABLED
  railcom_hub.emplace(svc);
  railComDriver.hw_init(railcom_hub.operator->());
#if CONFIG_RAILCOM_DUMP_PACKETS
  railcom_dumper.emplace(railcom_hub.operator->());
#endif
#else // cut-out disabled
  get_dcc_output(DccOutput::Type::TRACK)->set_railcom_cutout_enabled(
    DccOutput::RailcomCutout::DISABLED);
  get_dcc_output(DccOutput::Type::LCC)->set_railcom_cutout_enabled(
    DccOutput::RailcomCutout::DISABLED);
#endif // CONFIG_RAILCOM_CUT_OUT_ENABLED
  track_power.emplace(node);
  track_power_consumer.emplace(track_power.operator->());
  estop_packet_source.emplace(node);
  estop_consumer.emplace(estop_packet_source.operator->());
#if CONFIG_PROG_TRACK_ENABLED
  prog_backend.emplace(svc, enable_programming_track,
                       disable_programming_track);
#endif
  accessory_db.emplace(node, svc, track_interface.operator->());
#if CONFIG_OPS_TRACK_ENABLED
  track_monitor.emplace(svc, cfg);
#endif // CONFIG_OPS_TRACK_ENABLED

#if CONFIG_ENERGIZE_TRACK_ON_STARTUP
  DccHwDefs::InternalBoosterOutput::clear_disable_reason(
        DccOutput::DisableReason::GLOBAL_EOFF);
  DccHwDefs::OpenLCBBoosterOutput::clear_disable_reason(
        DccOutput::DisableReason::GLOBAL_EOFF);
#else
  DccHwDefs::InternalBoosterOutput::set_disable_reason(
        DccOutput::DisableReason::GLOBAL_EOFF);
  DccHwDefs::OpenLCBBoosterOutput::set_disable_reason(
        DccOutput::DisableReason::GLOBAL_EOFF);
#endif // CONFIG_ENERGIZE_TRACK_ON_STARTUP

  // Clear the initialization pending flag now that everything is configured.
  get_dcc_output(DccOutput::Type::TRACK)->clear_disable_output_for_reason(
    DccOutput::DisableReason::INITIALIZATION_PENDING);
  get_dcc_output(DccOutput::Type::LCC)->clear_disable_output_for_reason(
    DccOutput::DisableReason::INITIALIZATION_PENDING);
}

void shutdown_dcc()
{
  // disconnect the RMT TX complete callback so that no more DCC packets will
  // be sent to the tracks.
  rmt_register_tx_end_callback(nullptr, nullptr);

  // TODO: disable RMT driver?
  // TODO: disable VFS?

  // Disable all track outputs
  DccHwDefs::InternalBoosterOutput::set_disable_reason(
        DccOutput::DisableReason::INITIALIZATION_PENDING);
  DccHwDefs::OpenLCBBoosterOutput::set_disable_reason(
        DccOutput::DisableReason::INITIALIZATION_PENDING);
}

} // namespace esp32cs

// this must be declared in the global namespace
DccOutput *get_dcc_output(DccOutput::Type type)
{
  switch (type)
  {
    case DccOutput::TRACK:
      return DccOutputImpl<DccHwDefs::InternalBoosterOutput>::instance();
    case DccOutput::PGM:
      return DccOutputImpl<DccHwDefs::ProgBoosterOutput>::instance();
    case DccOutput::LCC:
      return DccOutputImpl<DccHwDefs::OpenLCBBoosterOutput>::instance();
  }
  return nullptr;
}