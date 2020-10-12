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

#include "DuplexedTrackIf.h"
#include "EStopHandler.h"
#include "Esp32RailComDriver.h"
#include "PrioritizedUpdateLoop.hxx"
#include "RMTTrackDevice.h"
#include "TrackPowerBitInterface.h"

#include <dcc/DccOutput.hxx>
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
#include <openlcb/EventHandlerTemplates.hxx>
#include <openlcb/RefreshLoop.hxx>
#include <StatusDisplay.h>
#include <StatusLED.h>
#include <utils/GpioInitializer.hxx>
#include <utils/logging.h>

namespace esp32cs
{
/// RMT channel to use for the OPS track output.
static constexpr rmt_channel_t OPS_RMT_CHANNEL = RMT_CHANNEL_0;

/// RMT channel to use for the PROG track output.
static constexpr rmt_channel_t PROG_RMT_CHANNEL = RMT_CHANNEL_3;

/// OPS Track signal pin.
GPIO_PIN(OPS_SIGNAL, GpioOutputSafeLow, CONFIG_OPS_HBRIDGE_SIGNAL_PIN);

/// OPS Track h-bridge enable pin.
GPIO_PIN(OPS_ENABLE, GpioOutputSafeLow, CONFIG_OPS_HBRIDGE_ENABLE_PIN);

#if defined(CONFIG_OPS_BRAKE_PIN)
/// OPS Track h-bridge brake pin, active HIGH.
GPIO_PIN(OPS_BRAKE, GpioOutputSafeHigh, CONFIG_OPS_HBRIDGE_BRAKE_PIN);
#else
/// OPS Track h-bridge brakde pin, not connected to actual hardware.
typedef DummyPin OPS_BRAKE_Pin;
#endif

/// RailCom driver instance for the PROG track, unused.
NoRailcomDriver progRailComDriver;

/// PROG Track signal pin.
GPIO_PIN(PROG_SIGNAL, GpioOutputSafeLow, CONFIG_PROG_HBRIDGE_SIGNAL_PIN);

/// PROG Track h-bridge enable pin.
GPIO_PIN(PROG_ENABLE, GpioOutputSafeLow, CONFIG_PROG_HBRIDGE_ENABLE_PIN);

// Sanity check that the preamble bits are within the supported range.
#ifndef CONFIG_OPS_DCC_PREAMBLE_BITS
#warning CONFIG_OPS_DCC_PREAMBLE_BITS is not defined and has been set to 11.
#define CONFIG_OPS_DCC_PREAMBLE_BITS 11
#elif CONFIG_OPS_DCC_PREAMBLE_BITS < 11
#warning CONFIG_OPS_DCC_PREAMBLE_BITS is set too low and has been reset to 11.
#undef CONFIG_OPS_DCC_PREAMBLE_BITS
#define CONFIG_OPS_DCC_PREAMBLE_BITS 11
#elif CONFIG_OPS_DCC_PREAMBLE_BITS > 20
#warning CONFIG_OPS_DCC_PREAMBLE_BITS is set too high and has been reset to 20.
#undef CONFIG_OPS_DCC_PREAMBLE_BITS
#define CONFIG_OPS_DCC_PREAMBLE_BITS 20
#endif

// Sanity check that the preamble bits are within range.
#ifndef CONFIG_PROG_DCC_PREAMBLE_BITS
#define CONFIG_PROG_DCC_PREAMBLE_BITS 22
#elif CONFIG_PROG_DCC_PREAMBLE_BITS < 22 || CONFIG_PROG_DCC_PREAMBLE_BITS > 75
#undef CONFIG_PROG_DCC_PREAMBLE_BITS
#define CONFIG_PROG_DCC_PREAMBLE_BITS 22
#endif

#if CONFIG_OPS_RAILCOM

#if CONFIG_OPS_DCC_PREAMBLE_BITS < 16
#warning CONFIG_OPS_DCC_PREAMBLE_BITS is set too low for RailCom support and has been reset to 16.
#undef CONFIG_OPS_DCC_PREAMBLE_BITS
#define CONFIG_OPS_DCC_PREAMBLE_BITS 16
#endif

#if !defined(CONFIG_OPS_RAILCOM_FEEDBACK_QUEUE) || \
    (CONFIG_OPS_RAILCOM_FEEDBACK_QUEUE < CONFIG_OPS_PACKET_QUEUE_SIZE)
#define CONFIG_OPS_RAILCOM_FEEDBACK_QUEUE (CONFIG_OPS_PACKET_QUEUE_SIZE * 2)
#endif // CONFIG_OPS_RAILCOM_FEEDBACK_QUEUE

/// RailCom detector enable pin, active HIGH.
GPIO_PIN(OPS_RAILCOM_ENABLE, GpioOutputSafeLow, CONFIG_OPS_RAILCOM_ENABLE_PIN);

/// RailCom detector data pin.
GPIO_PIN(OPS_RAILCOM_DATA, GpioInputPU, CONFIG_OPS_RAILCOM_UART_RX_PIN);

/// RailCom hardware definition
struct RailComHW
{
#if CONFIG_OPS_RAILCOM_UART1
  static constexpr uart_port_t UART = UART_NUM_1;
  static constexpr uart_dev_t *UART_BASE = &UART1;
  static constexpr periph_module_t UART_PERIPH = PERIPH_UART1_MODULE;
  static constexpr int UART_ISR_SOURCE = ETS_UART1_INTR_SOURCE;
  static constexpr uint32_t UART_MATRIX_IDX = U1RXD_IN_IDX;
  static constexpr uint32_t UART_CLOCK_EN_BIT = DPORT_UART1_CLK_EN;
  static constexpr uint32_t UART_RESET_BIT = DPORT_UART1_RST;
#elif CONFIG_OPS_RAILCOM_UART2
  static constexpr uart_port_t UART = UART_NUM_2;
  static constexpr uart_dev_t *UART_BASE = &UART2;
  static constexpr periph_module_t UART_PERIPH = PERIPH_UART2_MODULE;
  static constexpr int UART_ISR_SOURCE = ETS_UART2_INTR_SOURCE;
  static constexpr uint32_t UART_MATRIX_IDX = U2RXD_IN_IDX;
  static constexpr uint32_t UART_CLOCK_EN_BIT = DPORT_UART2_CLK_EN;
  static constexpr uint32_t UART_RESET_BIT = DPORT_UART2_RST;
#else
  #error Unsupported UART selected for OPS RailCom!
#endif

  using DATA = OPS_RAILCOM_DATA_Pin;
  using HB_BRAKE = OPS_BRAKE_Pin;
  using HB_ENABLE = OPS_ENABLE_Pin;
  using RC_ENABLE = OPS_RAILCOM_ENABLE_Pin;

  static void hw_init()
  {
    DATA::hw_init();
    HB_BRAKE::hw_init();
    RC_ENABLE::hw_init();

    // initialize the UART
    periph_module_enable(UART_PERIPH);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[DATA::pin()], PIN_FUNC_GPIO);
    gpio_matrix_in(DATA::pin(), UART_MATRIX_IDX, 0);
  }

  static constexpr timg_dev_t *TIMER_BASE = &TIMERG0;
  static constexpr timer_idx_t TIMER_IDX = TIMER_0;
  static constexpr timer_group_t TIMER_GRP = TIMER_GROUP_0;
  static constexpr periph_module_t TIMER_PERIPH = PERIPH_TIMG0_MODULE;
  static constexpr int TIMER_ISR_SOURCE = ETS_TG0_T0_LEVEL_INTR_SOURCE + TIMER_IDX;

  /// Number of microseconds to wait after the final packet bit completes
  /// before disabling the ENABLE pin on the h-bridge.
  static constexpr uint32_t RAILCOM_TRIGGER_DELAY_USEC = 1;

  /// Number of microseconds to wait for railcom data on channel 1.
  static constexpr uint32_t RAILCOM_MAX_READ_DELAY_CH_1 =
    177 - RAILCOM_TRIGGER_DELAY_USEC;

  /// Number of microseconds to wait for railcom data on channel 2.
  static constexpr uint32_t RAILCOM_MAX_READ_DELAY_CH_2 =
    454 - RAILCOM_MAX_READ_DELAY_CH_1;
};

/// RailCom driver instance for the OPS track.
Esp32RailComDriver<RailComHW> opsRailComDriver(CONFIG_OPS_RAILCOM_FEEDBACK_QUEUE);

#else

/// RailCom driver instance for the OPS track.
NoRailcomDriver opsRailComDriver;

#endif // CONFIG_OPS_RAILCOM

/// Initializer for all GPIO pins.
typedef GpioInitializer<
  OPS_SIGNAL_Pin, OPS_ENABLE_Pin
, PROG_SIGNAL_Pin, PROG_ENABLE_Pin
> DCCGpioInitializer;

static std::unique_ptr<openlcb::RefreshLoop> dcc_poller;
static std::unique_ptr<RMTTrackDevice> track[RMT_CHANNEL_MAX];
static std::unique_ptr<HBridgeShortDetector> track_mon[RMT_CHANNEL_MAX];
static std::unique_ptr<openlcb::BitEventConsumer> power_event;
static std::unique_ptr<EStopHandler> estop_handler;
static std::unique_ptr<ProgrammingTrackBackend> prog_track_backend;
static std::unique_ptr<esp32cs::DuplexedTrackIf> track_interface;
static std::unique_ptr<esp32cs::PrioritizedUpdateLoop> track_update_loop;
static std::unique_ptr<PoolToQueueFlow<Buffer<dcc::Packet>>> track_flow;

#if CONFIG_OPS_RAILCOM
static std::unique_ptr<dcc::RailcomHubFlow> railcom_hub;
static std::unique_ptr<dcc::RailcomPrintfFlow> railcom_dumper;
#endif // CONFIG_OPS_RAILCOM

/// Updates the status display with the current state of the track outputs.
static void update_status_display()
{
#if !CONFIG_DISPLAY_TYPE_NONE
  auto status = Singleton<StatusDisplay>::instance();
  status->track_power("%s:%s %s:%s", CONFIG_OPS_TRACK_NAME
                    , OPS_ENABLE_Pin::instance()->is_set() ? "On" : "Off"
                    , CONFIG_PROG_TRACK_NAME
                    , PROG_ENABLE_Pin::instance()->is_set() ? "On" : "Off");
#endif
}

/// Triggers an estop event to be sent
void initiate_estop()
{
  // TODO: add event publish
  estop_handler->set_state(true);
}

/// Returns true if the OPS track output is enabled
bool is_ops_track_output_enabled()
{
  return OPS_ENABLE_Pin::get();
}

/// Enables the OPS track output
void enable_ops_track_output()
{
  if (!is_ops_track_output_enabled())
  {
    LOG(INFO, "[Track] Enabling track output: %s", CONFIG_OPS_TRACK_NAME);
    OPS_ENABLE_Pin::set(true);
#if CONFIG_STATUS_LED
    Singleton<StatusLED>::instance()->setStatusLED(
          StatusLED::LED::OPS_TRACK, StatusLED::COLOR::GREEN);
#endif // CONFIG_STATUS_LED
    update_status_display();
  }
}

/// Enables the OPS track output
void disable_ops_track_output()
{
  LOG(INFO, "[Track] Disabling track output: %s (if enabled)", CONFIG_OPS_TRACK_NAME);
  OPS_ENABLE_Pin::set(false);
#if CONFIG_STATUS_LED
  Singleton<StatusLED>::instance()->setStatusLED(
        StatusLED::LED::OPS_TRACK, StatusLED::COLOR::OFF);
#endif // CONFIG_STATUS_LED
  update_status_display();
}

/// Enables the PROG track output
static void enable_prog_track_output()
{
  if (!PROG_ENABLE_Pin::get())
  {
    LOG(INFO, "[Track] Enabling track output: %s", CONFIG_PROG_TRACK_NAME);
    PROG_ENABLE_Pin::set(true);
    track_mon[PROG_RMT_CHANNEL]->enable_prog_response(true);
#if CONFIG_STATUS_LED
    Singleton<StatusLED>::instance()->setStatusLED(
          StatusLED::LED::PROG_TRACK, StatusLED::COLOR::GREEN);
#endif // CONFIG_STATUS_LED
    update_status_display();
  }
}

/// Disables the PROG track outputs
static void disable_prog_track_output()
{
  LOG(INFO, "[Track] Disabling track output: %s (if enabled)", CONFIG_PROG_TRACK_NAME);
  PROG_ENABLE_Pin::set(false);
  track_mon[PROG_RMT_CHANNEL]->enable_prog_response(false);
#if CONFIG_STATUS_LED
  Singleton<StatusLED>::instance()->setStatusLED(
        StatusLED::LED::PROG_TRACK, StatusLED::COLOR::OFF);
#endif // CONFIG_STATUS_LED
  update_status_display();
}

/// Disables all track outputs
void disable_track_outputs()
{
  disable_ops_track_output();
  disable_prog_track_output();
}

/// ESP32 VFS ::write() impl for the RMTTrackDevice.
/// @param fd is the file descriptor being written to.
/// @param data is the data to write.
/// @param size is the size of data.
/// @returns number of bytes written.
static ssize_t dcc_vfs_write(int fd, const void *data, size_t size)
{
  HASSERT(track[fd] != nullptr);
  return track[fd]->write(fd, data, size);
}

/// ESP32 VFS ::open() impl for the RMTTrackDevice
/// @param path is the file location to be opened.
/// @param flags is not used.
/// @param mode is not used.
/// @returns file descriptor for the opened file location.
static int dcc_vfs_open(const char *path, int flags, int mode)
{
  int fd;
  if (!strcasecmp(path + 1, CONFIG_OPS_TRACK_NAME))
  {
    fd = OPS_RMT_CHANNEL;
  }
  else if (!strcasecmp(path + 1, CONFIG_PROG_TRACK_NAME))
  {
    fd = PROG_RMT_CHANNEL;
  }
  else
  {
    LOG_ERROR("[Track] Attempt to open unknown track interface: %s", path + 1);
    errno = ENOTSUP;
    return -1;
  }
  LOG(INFO, "[Track] Connecting track interface (track:%s, fd:%d)", path + 1, fd);
  return fd;
}

/// ESP32 VFS ::close() impl for the RMTTrackDevice.
/// @param fd is the file descriptor to close.
/// @returns the status of the close() operation, only returns zero.
static int dcc_vfs_close(int fd)
{
  HASSERT(track[fd] != nullptr);
  LOG(INFO, "[Track] Disconnecting track interface (track:%s, fd:%d)"
    , track[fd]->name(), fd);
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
  HASSERT(track[fd] != nullptr);
  return track[fd]->ioctl(fd, cmd, args);
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
  if (track[channel] != nullptr)
  {
    track[channel]->rmt_transmit_complete();
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
  AutoNotify n(notif);
  // Connect our callback into the RMT so we can queue up the next packet for
  // transmission when needed.
  rmt_register_tx_end_callback(rmt_tx_callback, nullptr);

  track[OPS_RMT_CHANNEL].reset(
    new RMTTrackDevice(CONFIG_OPS_TRACK_NAME, OPS_RMT_CHANNEL
                     , CONFIG_OPS_DCC_PREAMBLE_BITS
                     , CONFIG_OPS_PACKET_QUEUE_SIZE, OPS_SIGNAL_Pin::pin()
                     , reinterpret_cast<RailcomDriver *>(&opsRailComDriver)));

  track[PROG_RMT_CHANNEL].reset(
    new RMTTrackDevice(CONFIG_PROG_TRACK_NAME, PROG_RMT_CHANNEL
                     , CONFIG_PROG_DCC_PREAMBLE_BITS
                     , CONFIG_PROG_PACKET_QUEUE_SIZE, PROG_SIGNAL_Pin::pin()
                     , &progRailComDriver));

  // this is a one-time task, shutdown the task before returning
  vTaskDelete(nullptr);
}

/// Initializes the ESP32 VFS adapter for the DCC track interface and the short
/// detection devices.
/// @param node is the OpenLCB node to bind to.
/// @param service is the OpenLCB @ref Service to use for recurring tasks.
/// @param ops_cfg is the CDI element for the OPS track output.
/// @param prog_cfg is the CDI element for the PROG track output.
void init_dcc(openlcb::Node *node, Service *service
            , const esp32cs::TrackOutputConfig &ops_cfg
            , const esp32cs::TrackOutputConfig &prog_cfg)
{
  // register the VFS handler as the LocalTrackIf uses this to route DCC
  // packets to the track.
  esp_vfs_t vfs;
  memset(&vfs, 0, sizeof(vfs));
  vfs.flags = ESP_VFS_FLAG_DEFAULT;
  vfs.ioctl = &esp32cs::dcc_vfs_ioctl;
  vfs.open = &esp32cs::dcc_vfs_open;
  vfs.close = &esp32cs::dcc_vfs_close;
  vfs.write = &esp32cs::dcc_vfs_write;

  LOG(INFO, "[Track] Registering /dev/track VFS interface");
  ESP_ERROR_CHECK(esp_vfs_register("/dev/track", &vfs, nullptr));

  DCCGpioInitializer::hw_init();

#if defined(CONFIG_OPS_RAILCOM)
  railcom_hub.reset(new dcc::RailcomHubFlow(service));
  opsRailComDriver.hw_init(railcom_hub.get());
#if defined(CONFIG_OPS_RAILCOM_DUMP_PACKETS)
  railcom_dumper.reset(new dcc::RailcomPrintfFlow(railcom_hub.get()));
#endif
#endif // CONFIG_OPS_RAILCOM

  track_mon[OPS_RMT_CHANNEL].reset(
    new HBridgeShortDetector(node, (adc1_channel_t)CONFIG_OPS_ADC
                           , OPS_ENABLE_Pin::instance()
                           , CONFIG_OPS_HBRIDGE_LIMIT_MILLIAMPS
                           , CONFIG_OPS_HBRIDGE_MAX_MILLIAMPS
                           , CONFIG_OPS_TRACK_NAME
                           , CONFIG_OPS_HBRIDGE_TYPE_NAME
                           , ops_cfg));

  track_mon[PROG_RMT_CHANNEL].reset(
    new HBridgeShortDetector(node, (adc1_channel_t)CONFIG_PROG_ADC
                           , PROG_ENABLE_Pin::instance()
                           , CONFIG_PROG_HBRIDGE_MAX_MILLIAMPS
                           , CONFIG_PROG_TRACK_NAME
                           , CONFIG_PROG_HBRIDGE_TYPE_NAME
                           , prog_cfg));

  SyncNotifiable notif;
  // initialize the track signal generators on the second core so that the ISR
  // is bound to that core instead of the first core.
  xTaskCreatePinnedToCore(&init_rmt_outputs, "DCC RMT Init", 2048, &notif, 2
                        , nullptr, APP_CPU_NUM);

  // block until the RMT init task completes
  notif.wait_for_notification();

  LOG(INFO
    , "[Track] Registering LCC EventConsumer for Track Power (On:%s, Off:%s)"
    , uint64_to_string_hex(openlcb::Defs::CLEAR_EMERGENCY_OFF_EVENT).c_str()
    , uint64_to_string_hex(openlcb::Defs::EMERGENCY_OFF_EVENT).c_str());
  power_event.reset(
    new openlcb::BitEventConsumer(
      new TrackPowerBit(node, OPS_ENABLE_Pin::instance())));

  // Initialize the e-stop event handler
  estop_handler.reset(new EStopHandler(node));

  // Initialize the Programming Track backend handler
  prog_track_backend.reset(
    new ProgrammingTrackBackend(service
                              , &enable_prog_track_output
                              , &disable_prog_track_output));

  // Configure h-bridge polling
  dcc_poller.reset(new openlcb::RefreshLoop(node,
    { track_mon[OPS_RMT_CHANNEL].get()
    , track_mon[PROG_RMT_CHANNEL].get()
  }));

  track_interface.reset(
    new esp32cs::DuplexedTrackIf(service, CONFIG_DCC_PACKET_POOL_SIZE
                               , CONFIG_OPS_TRACK_NAME
                               , CONFIG_PROG_TRACK_NAME
                               , "/dev/track"));
  track_update_loop.reset(
    new esp32cs::PrioritizedUpdateLoop(service, track_interface.get()));

  // Attach the DCC update loop to the track interface
  track_flow.reset(
    new PoolToQueueFlow<Buffer<dcc::Packet>>(service, track_interface->pool()
                                           , track_update_loop.get()));

#if defined(CONFIG_OPS_ENERGIZE_ON_STARTUP)
  // with everything up and running it's time to energize the track if it is
  // set to default to ON during startup.
  power_event->set_state(true);
#endif

  update_status_display();
}

void shutdown_dcc_vfs()
{
  // disconnect the RMT TX complete callback so that no more DCC packets will
  // be sent to the tracks.
  rmt_register_tx_end_callback(nullptr, nullptr);

  // stop any future polling of the DCC outputs
  dcc_poller->stop();

  // Note that other objects are not released at this point since they may
  // still be called by other systems until the reboot occurs.
}

/// @return string containing a two element json array of the track monitors.
std::string get_track_state_json()
{
  return StringPrintf("[%s,%s]"
                    , track_mon[OPS_RMT_CHANNEL]->getStateAsJson().c_str()
                    , track_mon[PROG_RMT_CHANNEL]->getStateAsJson().c_str());
}

/// @return DCC++ status data from the OPS track only.
std::string get_track_state_for_dccpp()
{
  return track_mon[OPS_RMT_CHANNEL]->get_state_for_dccpp();
}

/// Enables or disables track power via event state.
/// @param new_value is the requested status.
void TrackPowerBit::set_state(bool new_value)
{
  if (new_value)
  {
    enable_ops_track_output();
  }
  else
  {
    disable_track_outputs();
  }
}

/*
struct DccHardware
{
  typedef DummyPin PROG_RAILCOM_ENABLE_Pin;

  using OPSDccOutput = DccOutputHwReal<DccOutput::Type::TRACK, OPS_ENABLE_Pin
                                     , OPS_RAILCOM_ENABLE_Pin, 0, 0, 0>;
  using PROGDccOutput = DccOutputHwReal<DccOutput::Type::PGM, PROG_ENABLE_Pin
                                      , PROG_RAILCOM_ENABLE_Pin, 0, 0, 0>;
  using LCCDccOutput = DccOutputHwDummy<DccOutput::Type::LCC>;
};
*/

} // namespace esp32cs

// This needs to be declared in the global namespace
// TODO
DccOutput *get_dcc_output(DccOutput::Type type)
{
  /*
  switch (type)
  {
    case DccOutput::Type::TRACK:
      return DccOutputImpl<esp32cs::DccHardware::OPSDccOutput>::instance();
    case DccOutput::Type::PGM:
      return DccOutputImpl<esp32cs::DccHardware::PROGDccOutput>::instance();
    case DccOutput::Type::LCC:
      return DccOutputImpl<esp32cs::DccHardware::LCCDccOutput>::instance();
  }
  */
  return nullptr;
}
