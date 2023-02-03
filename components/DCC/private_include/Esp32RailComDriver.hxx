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

#ifndef ESP32_RAILCOM_DRIVER_HXX_
#define ESP32_RAILCOM_DRIVER_HXX_

#include <dcc/RailCom.hxx>
#include <dcc/RailcomHub.hxx>
#include <esp_intr_alloc.h>

#include <esp_rom_gpio.h>
#if ESP_IDF_VERSION_MAJOR >= 5
#include <esp_private/esp_clk.h>
#if defined(CONFIG_IDF_TARGET_ESP32)
#include <esp32/rom/ets_sys.h>
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#include <esp32s2/rom/ets_sys.h>
#endif // ESP variant
#elif CONFIG_IDF_TARGET_ESP32
#include <esp32/clk.h>
#elif CONFIG_IDF_TARGET_ESP32S3
#include <esp32s3/clk.h>
#endif // IDF v5+
#include <freertos_drivers/arduino/DeviceBuffer.hxx>
#include <freertos_drivers/arduino/RailcomDriver.hxx>
#include <hal/uart_types.h>
#include <os/Gpio.hxx>
#include <soc/dport_reg.h>
#include <soc/gpio_periph.h>
#include <soc/timer_periph.h>
#include <soc/uart_periph.h>
#include <stdint.h>
#include <utils/logging.h>

namespace esp32cs
{

template <class HW, class DCC_BOOSTER>
static bool esp32_railcom_timer_tick(void *param);

//template <class HW, class DCC_BOOSTER>
//static void esp32_railcom_timer_tick(void *param);

template <class HW, class DCC_BOOSTER>
static void esp32_railcom_uart_isr(void *param);

static portMUX_TYPE esp32_uart_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE esp32_timer_mux = portMUX_INITIALIZER_UNLOCKED;

static constexpr uint32_t ESP32_UART_CLEAR_ALL_INTERRUPTS = 0xFFFFFFFF;
static constexpr uint32_t ESP32_UART_DISABLE_ALL_INTERRUPTS = 0x00000000;
static constexpr uint32_t ESP32_UART_RX_INTERRUPT_BITS =
  UART_RXFIFO_FULL_INT_ENA | UART_RXFIFO_TOUT_INT_ENA;

template <class DCC_HW, class RAILCOM_HW>
class Esp32RailComDriver : public RailcomDriver
{
public:
  Esp32RailComDriver()
    : railComFeedbackBuffer_(DeviceBuffer<dcc::RailcomHubData>::create(RAILCOM_HW::PACKET_Q_SIZE))
  {
  }

  void hw_init(dcc::RailcomHubFlow *hubFlow)
  {
    railComHubFlow_ = hubFlow;

#if CONFIG_RAILCOM_DATA_ENABLED
    RAILCOM_HW::hw_init();
    LOG(INFO, "[RailCom] Initializing detector using UART %d, RX pin: %d",
        RAILCOM_HW::UART, RAILCOM_HW::RAILCOM_DATA_PIN);

    portENTER_CRITICAL_SAFE(&esp32_uart_mux);
    // enable the UART clock and disable the reset bit
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, RAILCOM_HW::UART_CLOCK_EN_BIT);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, RAILCOM_HW::UART_RESET_BIT);

    // TODO: switch to UART HAL in IDF v4.2+
    uint32_t baud_clock = (((esp_clk_apb_freq()) << 4) / 250000L);
    RAILCOM_HW::UART_BASE->clk_div.div_int = (baud_clock >> 4);
    RAILCOM_HW::UART_BASE->clk_div.div_frag = (baud_clock & 0xf);
    RAILCOM_HW::UART_BASE->conf0.val = 0x800001c;       // configure for 8N1 mode
    RAILCOM_HW::UART_BASE->conf1.val = 0;
    RAILCOM_HW::UART_BASE->conf1.rx_tout_thrhd = 2;     // two bit times
    RAILCOM_HW::UART_BASE->conf1.rxfifo_full_thrhd = 6; // max length of channel 2 data
    RAILCOM_HW::UART_BASE->conf1.rx_tout_en = 1;        // rx timeout
    RAILCOM_HW::UART_BASE->idle_conf.val = 0;           // tx configuration (unused)
    RAILCOM_HW::UART_BASE->rs485_conf.val = 0;          // RS485 configuration (unused)
    RAILCOM_HW::UART_BASE->int_clr.val = ESP32_UART_CLEAR_ALL_INTERRUPTS;
    RAILCOM_HW::UART_BASE->int_ena.val = ESP32_UART_DISABLE_ALL_INTERRUPTS;

    portEXIT_CRITICAL_SAFE(&esp32_uart_mux);
    ESP_ERROR_CHECK(
      esp_intr_alloc(RAILCOM_HW::UART_ISR_SOURCE, ESP_INTR_FLAG_LOWMED,
                     esp32_railcom_uart_isr<DCC_HW, RAILCOM_HW>,
                     this, nullptr));
#endif // CONFIG_RAILCOM_DATA_ENABLED

    LOG(INFO, "[RailCom] Configuring hardware timer (%d:%d)...",
        RAILCOM_HW::TIMER_GRP, RAILCOM_HW::TIMER_IDX);

    timer_config_t timer_config =
    {
      .alarm_en = TIMER_ALARM_EN,
      .counter_en = TIMER_PAUSE,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = TIMER_AUTORELOAD_DIS,
      .divider = 80,
#if SOC_TIMER_GROUP_SUPPORT_XTAL
      .clk_src = TIMER_SRC_CLK_APB,
#endif
    };
    ESP_ERROR_CHECK(
      timer_init(RAILCOM_HW::TIMER_GRP, RAILCOM_HW::TIMER_IDX, &timer_config));
    timer_isr_callback_add(RAILCOM_HW::TIMER_GRP, RAILCOM_HW::TIMER_IDX,
                           esp32_railcom_timer_tick<DCC_HW, RAILCOM_HW>, this, 0);

/*
    // TODO: switch to TIMER HAL in IDF v4.2+
    periph_module_enable(RAILCOM_HW::TIMER_PERIPH);
    configure_timer(false, 80, false, true, 1, true);
    ESP_ERROR_CHECK(
      esp_intr_alloc_intrstatus(RAILCOM_HW::TIMER_ISR_SOURCE, ESP_INTR_FLAG_LOWMED,
                                TIMG_INT_ST_TIMERS_REG(RAILCOM_HW::TIMER_GRP),
                                BIT(RAILCOM_HW::TIMER_IDX),
                                esp32_railcom_timer_tick<DCC_HW, RAILCOM_HW>,
                                this, nullptr));
*/
  }

  void feedback_sample() override
  {
    // NOOP
  }

  void start_cutout() override
  {
#if CONFIG_RAILCOM_CUT_OUT_ENABLED

    ets_delay_us(DCC_HW::InternalBoosterOutput::start_railcom_cutout_phase1());

#if CONFIG_RAILCOM_DATA_ENABLED
    portENTER_CRITICAL_SAFE(&esp32_uart_mux);
    // flush the uart queue of any pending data
    rx_to_buf(nullptr, 0);

    // clear all pending interrupts and enable default RX interrupts.
    SET_PERI_REG_MASK(UART_INT_CLR_REG(RAILCOM_HW::UART), ESP32_UART_RX_INTERRUPT_BITS);
    SET_PERI_REG_MASK(UART_INT_ENA_REG(RAILCOM_HW::UART), ESP32_UART_RX_INTERRUPT_BITS);
    portEXIT_CRITICAL_SAFE(&esp32_uart_mux);
#endif // CONFIG_RAILCOM_DATA_ENABLED

    // enable the RailCom detector
    ets_delay_us(DCC_HW::InternalBoosterOutput::start_railcom_cutout_phase2());

    //portENTER_CRITICAL_SAFE(&esp32_timer_mux);
    // set our phase and start the timer
    railcomPhase_ = RailComPhase::CUTOUT_PHASE1;
    timer_group_set_alarm_value_in_isr(RAILCOM_HW::TIMER_GRP, RAILCOM_HW::TIMER_IDX, RAILCOM_HW::RAILCOM_MAX_READ_DELAY_CH_1);
    timer_group_set_counter_enable_in_isr(RAILCOM_HW::TIMER_GRP, RAILCOM_HW::TIMER_IDX, TIMER_START);
    //start_timer(RAILCOM_HW::RAILCOM_MAX_READ_DELAY_CH_1);
    //portEXIT_CRITICAL_SAFE(&esp32_timer_mux);
#endif // CONFIG_RAILCOM_CUT_OUT_ENABLED
  }

  void middle_cutout() override
  {
    // NO OP, handled in ISR
  }

  void no_cutout() override
  {
    // NO OP
  }

  void end_cutout() override
  {
#if CONFIG_RAILCOM_CUT_OUT_ENABLED

#if CONFIG_RAILCOM_DATA_ENABLED
    portENTER_CRITICAL_SAFE(&esp32_uart_mux);
    // disable the UART RX interrupts
    RAILCOM_HW::UART_BASE->int_clr.val = ESP32_UART_CLEAR_ALL_INTERRUPTS;
    RAILCOM_HW::UART_BASE->int_ena.val = ESP32_UART_DISABLE_ALL_INTERRUPTS;
    portEXIT_CRITICAL_SAFE(&esp32_uart_mux);
#endif // CONFIG_RAILCOM_DATA_ENABLED

    // disable the RailCom detector
    ets_delay_us(DCC_HW::InternalBoosterOutput::stop_railcom_cutout_phase1());
    DCC_HW::InternalBoosterOutput::stop_railcom_cutout_phase2();

    if (DCC_HW::InternalBoosterOutput::should_be_enabled())
    {
      ets_printf("ENABLE\n");
      DCC_HW::InternalBoosterOutput::enable_output();
    }
#endif // CONFIG_RAILCOM_CUT_OUT_ENABLED
  }

  void set_feedback_key(uint32_t key) override
  {
    railcomFeedbackKey_ = key;
  }

  void timer_tick()
  {
    timer_group_set_counter_enable_in_isr(RAILCOM_HW::TIMER_GRP, RAILCOM_HW::TIMER_IDX, TIMER_PAUSE);
    // portENTER_CRITICAL_SAFE(&esp32_timer_mux);
    // clear the interrupt status register for our timer
    RAILCOM_HW::TIMER_BASE->int_clr_timers.val = BIT(RAILCOM_HW::TIMER_IDX);

    if (railcomPhase_ == RailComPhase::CUTOUT_PHASE1)
    {
      middle_cutout();

      railcomPhase_ = RailComPhase::CUTOUT_PHASE2;
      timer_group_set_alarm_value_in_isr(RAILCOM_HW::TIMER_GRP, RAILCOM_HW::TIMER_IDX, RAILCOM_HW::RAILCOM_MAX_READ_DELAY_CH_2);

      //start_timer(RAILCOM_HW::RAILCOM_MAX_READ_DELAY_CH_2);
    }
    else if (railcomPhase_ == RailComPhase::CUTOUT_PHASE2)
    {
      end_cutout();
      railcomPhase_ = RailComPhase::PRE_CUTOUT;
    }
    timer_group_set_counter_enable_in_isr(RAILCOM_HW::TIMER_GRP, RAILCOM_HW::TIMER_IDX, TIMER_START);
    //portEXIT_CRITICAL_SAFE(&esp32_timer_mux);
  }

  typedef enum : uint8_t
  {
    PRE_CUTOUT,
    CUTOUT_PHASE1,
    CUTOUT_PHASE2
  } RailComPhase;

  RailComPhase railcom_phase()
  {
    return railcomPhase_;
  }

  dcc::RailcomHubData *railcom_buffer()
  {
    dcc::RailcomHubData *data = nullptr;
    if (railComFeedbackBuffer_->data_write_pointer(&data) > 0)
    {
      data->reset(railcomFeedbackKey_);
    }
    return data;
  }

  void advance_railcom_buffer()
  {
    railComFeedbackBuffer_->advance(1);
    railComFeedbackBuffer_->signal_condition_from_isr();
  }

  size_t rx_to_buf(uint8_t *buf, size_t max_len)
  {
    size_t rx_bytes = 0;
#if CONFIG_RAILCOM_DATA_ENABLED
    // NOTE: Due to a hardware issue when flushing the RX FIFO it is necessary
    // to read the FIFO until the RX count is zero *AND* read/write addresses
    // in the RX buffer are the same.
    while(RAILCOM_HW::UART_BASE->status.rxfifo_cnt ||
          (RAILCOM_HW::UART_BASE->mem_rx_status.wr_addr !=
           RAILCOM_HW::UART_BASE->mem_rx_status.rd_addr))
    {
      uint8_t ch = RAILCOM_HW::UART_BASE->fifo.rw_byte;
      if (rx_bytes < max_len)
      {
        buf[rx_bytes++] = ch;
      }
    }
#endif // CONFIG_RAILCOM_DATA_ENABLED
    return rx_bytes;
  }

private:
  void configure_timer(bool reload, uint16_t divider, bool enable, bool count_up, uint64_t alarm, bool alarm_en)
  {
    portENTER_CRITICAL_SAFE(&esp32_timer_mux);
    // make sure the ISR is disabled and that the status is cleared before
    // reconfiguring the timer.
#if CONFIG_IDF_TARGET_ESP32 && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5,0,0)
    RAILCOM_HW::TIMER_BASE->int_ena.val &= (~BIT(RAILCOM_HW::TIMER_IDX));
    RAILCOM_HW::TIMER_BASE->int_clr_timers.val = BIT(RAILCOM_HW::TIMER_IDX);
    RAILCOM_HW::TIMER_BASE->hw_timer[RAILCOM_HW::TIMER_IDX].config.autoreload = reload;
    RAILCOM_HW::TIMER_BASE->hw_timer[RAILCOM_HW::TIMER_IDX].config.divider = divider;
    RAILCOM_HW::TIMER_BASE->hw_timer[RAILCOM_HW::TIMER_IDX].config.increase = count_up;

    RAILCOM_HW::TIMER_BASE->hw_timer[RAILCOM_HW::TIMER_IDX].config.enable = enable;
#endif
    start_timer(alarm, true, false);

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5,0,0)
    // enable the ISR now that the timer has been configured
    RAILCOM_HW::TIMER_BASE->int_ena.val |= BIT(RAILCOM_HW::TIMER_IDX);
#else
    // enable the ISR now that the timer has been configured
    RAILCOM_HW::TIMER_BASE->int_ena_timers.val |= BIT(RAILCOM_HW::TIMER_IDX);
#endif    
    portEXIT_CRITICAL_SAFE(&esp32_timer_mux);
  }

  void start_timer(uint32_t usec, bool enable_alarm = true, bool enable_timer = true)
  {
#if CONFIG_IDF_TARGET_ESP32 && ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5,0,0)
    // disable the timer since we will reconfigure it
    RAILCOM_HW::TIMER_BASE->hw_timer[RAILCOM_HW::TIMER_IDX].config.enable = 0;

    // reload the timer with a default count of zero
    RAILCOM_HW::TIMER_BASE->hw_timer[RAILCOM_HW::TIMER_IDX].load_high = 0UL;
    RAILCOM_HW::TIMER_BASE->hw_timer[RAILCOM_HW::TIMER_IDX].load_low = 0UL;
    RAILCOM_HW::TIMER_BASE->hw_timer[RAILCOM_HW::TIMER_IDX].reload = 1;

    // set the next alarm period
    RAILCOM_HW::TIMER_BASE->hw_timer[RAILCOM_HW::TIMER_IDX].alarm_high = 0;
    RAILCOM_HW::TIMER_BASE->hw_timer[RAILCOM_HW::TIMER_IDX].alarm_low = usec;
    RAILCOM_HW::TIMER_BASE->hw_timer[RAILCOM_HW::TIMER_IDX].config.alarm_en = enable_alarm;

    // start the timer
    RAILCOM_HW::TIMER_BASE->hw_timer[RAILCOM_HW::TIMER_IDX].config.enable = enable_timer;
#endif
  }

  uintptr_t railcomFeedbackKey_{0}; 
  dcc::RailcomHubFlow *railComHubFlow_;
  DeviceBuffer<dcc::RailcomHubData> *railComFeedbackBuffer_;
  RailComPhase railcomPhase_{RailComPhase::PRE_CUTOUT};
  bool enabled_{false};
};

template <class DCC_HW, class RAILCOM_HW>
static bool esp32_railcom_timer_tick(void *param)
{
  Esp32RailComDriver<DCC_HW, RAILCOM_HW> *driver =
    reinterpret_cast<Esp32RailComDriver<DCC_HW, RAILCOM_HW> *>(param);
  driver->timer_tick();
  return false;
}

/*template <class DCC_HW, class RAILCOM_HW>
static void esp32_railcom_timer_tick(void *param)
{
  Esp32RailComDriver<DCC_HW, RAILCOM_HW> *driver =
    reinterpret_cast<Esp32RailComDriver<DCC_HW, RAILCOM_HW> *>(param);
  driver->timer_tick();
}*/

template <class DCC_HW, class RAILCOM_HW>
static void esp32_railcom_uart_isr(void *param)
{
  portENTER_CRITICAL_SAFE(&esp32_uart_mux);
  Esp32RailComDriver<DCC_HW, RAILCOM_HW> *driver =
    reinterpret_cast<Esp32RailComDriver<DCC_HW, RAILCOM_HW> *>(param);
  dcc::RailcomHubData *fb = driver->railcom_buffer();
  uint8_t rx_buf[6] = {0, 0, 0, 0, 0, 0};
  size_t rx_bytes = driver->rx_to_buf(rx_buf, 6);
  if (fb)
  {
    for (size_t idx = 0; idx < rx_bytes; idx++)
    {
      if (driver->railcom_phase() ==
          Esp32RailComDriver<DCC_HW, RAILCOM_HW>::RailComPhase::CUTOUT_PHASE1)
      {
        fb->add_ch1_data(rx_buf[idx]);
      }
      else
      {
        fb->add_ch2_data(rx_buf[idx]);
      }
    }
    driver->advance_railcom_buffer();
  }
  // clear interrupt status
  RAILCOM_HW::UART_BASE->int_clr.val = ESP32_UART_CLEAR_ALL_INTERRUPTS;
  portEXIT_CRITICAL_SAFE(&esp32_uart_mux);
}

} // namespace esp32cs

#endif // ESP32_RAILCOM_DRIVER_HXX_