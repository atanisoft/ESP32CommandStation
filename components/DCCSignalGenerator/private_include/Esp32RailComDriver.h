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

#ifndef ESP32_RAILCOM_DRIVER_H_
#define ESP32_RAILCOM_DRIVER_H_

#include <dcc/RailCom.hxx>
#include <dcc/RailcomHub.hxx>
#include <esp_intr_alloc.h>
#include <esp32/clk.h>
#include <esp32/rom/gpio.h>
#include <freertos_drivers/arduino/RailcomDriver.hxx>
#include <os/Gpio.hxx>
#include <soc/gpio_periph.h>
#include <soc/timer_periph.h>
#include <soc/uart_periph.h>
#include <stdint.h>
#include <utils/logging.h>

namespace esp32cs
{

template <class HW>
static void esp32_railcom_timer_tick(void *param);

template <class HW>
static void esp32_railcom_uart_isr(void *arg);

template <class HW>
class Esp32RailComDriver : public RailcomDriver
{
public:
  Esp32RailComDriver()
  {
  }

  void hw_init(dcc::RailcomHubFlow *hubFlow)
  {
    railComHubFlow_ = hubFlow;

    HW::hw_init();
    LOG(INFO, "[RailCom] Initializing detector using UART %d", HW::UART);
    // TODO: with IDF v4.1+ switch to UART HAL instead of direct access.
    uint32_t baud_clock = (((esp_clk_apb_freq()) << 4) / 250000L);
    HW::UART_BASE->conf1.rx_flow_en = 0;
    HW::UART_BASE->conf0.tx_flow_en = 0;
    HW::UART_BASE->conf0.val = 0;
    HW::UART_BASE->conf0.parity = 0;
    HW::UART_BASE->conf0.bit_num = 3;
    HW::UART_BASE->conf0.stop_bit_num = 1;
    HW::UART_BASE->conf1.rx_tout_en = 1;
    HW::UART_BASE->clk_div.div_int = (baud_clock >> 4);
    HW::UART_BASE->clk_div.div_frag = (baud_clock & 0xf);
    HW::UART_BASE->idle_conf.tx_idle_num = 0;
    HW::UART_BASE->rs485_conf.dl1_en = 0;
    // disable TX interrupts since we won't be transmitting
    HW::UART_BASE->int_ena.val &=
      ~(UART_TXFIFO_EMPTY_INT_ENA | UART_TX_DONE_INT_ENA);

    ESP_ERROR_CHECK(
      esp_intr_alloc(HW::UART_ISR_SOURCE, ESP_INTR_FLAG_LOWMED
                   , esp32_railcom_uart_isr<HW>, this, nullptr));

    LOG(INFO, "[RailCom] Configuring hardware timer (%d:%d)...", HW::TIMER_GRP
      , HW::TIMER_IDX);
    periph_module_enable(HW::TIMER_PERIPH);
    configure_timer(false, 80, false, true, HW::RAILCOM_TRIGGER_DELAY_USEC, true);
    ESP_ERROR_CHECK(
      esp_intr_alloc_intrstatus(HW::TIMER_ISR_SOURCE, ESP_INTR_FLAG_LOWMED
                              , TIMG_INT_ST_TIMERS_REG(HW::TIMER_GRP)
                              , BIT(HW::TIMER_IDX)
                              , esp32_railcom_timer_tick<HW>, this, nullptr));
  }

  void feedback_sample() override
  {
    // NOOP
  }

  void disable_output()
  {
    // Enable the BRAKE pin on the h-bridge to force it into coast mode
    HW::HB_BRAKE::set(true);
 
    // Inject a small delay to ensure the brake pin and enable pin are transitioned
    // concurrently. This can lead to issues in the LMD18200
    ets_delay_us(1);

    // cache the current state of the pin so we can restore it after the
    // cutout.
    enabled_= HW::HB_ENABLE::get();
    HW::HB_ENABLE::set(false);
  }

  void enable_output()
  {
    HW::HB_ENABLE::set(enabled_);

    // Inject a small delay to ensure the brake pin and enable pin are transitioned
    // concurrently. This can lead to issues in the LMD18200
    ets_delay_us(1);

    HW::HB_BRAKE::set(false);
  }

  void start_cutout() override
  {
    disable_output();

    ets_delay_us(HW::RAILCOM_TRIGGER_DELAY_USEC);

    // flush the uart queue of any pending data
    reset_uart_fifo();
    
    // enable the UART RX interrupts
    SET_PERI_REG_MASK(
      UART_INT_CLR_REG(HW::UART)
                     , (UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR));
    SET_PERI_REG_MASK(
      UART_INT_ENA_REG(HW::UART)
                     , (UART_RXFIFO_FULL_INT_ENA | UART_RXFIFO_TOUT_INT_ENA));

    // enable the RailCom detector
    HW::RC_ENABLE::set(true);

    // set our phase and start the timer
    railcomPhase_ = RailComPhase::CUTOUT_PHASE1;
    start_timer(HW::RAILCOM_MAX_READ_DELAY_CH_1);
  }

  void middle_cutout() override
  {
    // NO OP, handled in ISR
  }

  void end_cutout() override
  {
    // disable the UART RX interrupts
    CLEAR_PERI_REG_MASK(
      UART_INT_ENA_REG(HW::UART)
                     , (UART_RXFIFO_FULL_INT_ENA | UART_RXFIFO_TOUT_INT_ENA));

    // disable the RailCom detector
    HW::RC_ENABLE::set(false);

    // ets_delay_us(0);
  }

  void set_feedback_key(uint32_t key) override
  {
    if (railComFeedback_)
    {
      // send the feedback to the hub
      railComHubFlow_->send(railComFeedback_);
    }
    // allocate a feedback packet
    railComFeedback_ = railComHubFlow_->alloc();
    if (railComFeedback_)
    {
      railComFeedback_->data()->reset(key);
      railcomFeedbackKey_ = key;
    }
  }

  void timer_tick()
  {
    // clear the interrupt status register for our timer
    HW::TIMER_BASE->int_clr_timers.val = BIT(HW::TIMER_IDX);

    if (railcomPhase_ == RailComPhase::CUTOUT_PHASE1)
    {
      middle_cutout();

      railcomPhase_ = RailComPhase::CUTOUT_PHASE2;
      start_timer(HW::RAILCOM_MAX_READ_DELAY_CH_2);
    }
    else if (railcomPhase_ == RailComPhase::CUTOUT_PHASE2)
    {
      end_cutout();
      railcomPhase_ = RailComPhase::PRE_CUTOUT;
      enable_output();
    }
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

  Buffer<dcc::RailcomHubData> *buf()
  {
    return railComFeedback_;
  }

private:

  void configure_timer(bool reload, uint16_t divider, bool enable, bool count_up, uint64_t alarm, bool alarm_en)
  {
    // make sure the ISR is disabled and that the status is cleared before
    // reconfiguring the timer.
    HW::TIMER_BASE->int_ena.val &= (~BIT(HW::TIMER_IDX));
    HW::TIMER_BASE->int_clr_timers.val = BIT(HW::TIMER_IDX);

    HW::TIMER_BASE->hw_timer[HW::TIMER_IDX].config.autoreload = reload;
    HW::TIMER_BASE->hw_timer[HW::TIMER_IDX].config.divider = divider;
    HW::TIMER_BASE->hw_timer[HW::TIMER_IDX].config.enable = enable;
    HW::TIMER_BASE->hw_timer[HW::TIMER_IDX].config.increase = count_up;
    HW::TIMER_BASE->hw_timer[HW::TIMER_IDX].config.level_int_en = 1;
    HW::TIMER_BASE->hw_timer[HW::TIMER_IDX].config.edge_int_en = 0;
    start_timer(alarm, true, false);

    // enable the ISR now that the timer has been configured
    HW::TIMER_BASE->int_ena.val |= BIT(HW::TIMER_IDX);
  }

  void start_timer(uint32_t usec, bool enable_alarm = true, bool enable_timer = true)
  {
    // disable the timer since we will reconfigure it
    HW::TIMER_BASE->hw_timer[HW::TIMER_IDX].config.enable = 0;

    // reload the timer with a default count of zero
    HW::TIMER_BASE->hw_timer[HW::TIMER_IDX].load_high = 0UL;
    HW::TIMER_BASE->hw_timer[HW::TIMER_IDX].load_low = 0UL;
    HW::TIMER_BASE->hw_timer[HW::TIMER_IDX].reload = 1;

    // set the next alarm period
    HW::TIMER_BASE->hw_timer[HW::TIMER_IDX].alarm_high = 0;
    HW::TIMER_BASE->hw_timer[HW::TIMER_IDX].alarm_low = usec;
    HW::TIMER_BASE->hw_timer[HW::TIMER_IDX].config.alarm_en = enable_alarm;

    // start the timer
    HW::TIMER_BASE->hw_timer[HW::TIMER_IDX].config.enable = enable_timer;
  }

  void reset_uart_fifo()
  {
    while(HW::UART_BASE->status.rxfifo_cnt != 0
      || (HW::UART_BASE->mem_rx_status.wr_addr !=
          HW::UART_BASE->mem_rx_status.rd_addr))
    {
      (void)HW::UART_BASE->fifo.rw_byte;
    }
  }

  uintptr_t railcomFeedbackKey_{0}; 
  dcc::RailcomHubFlow *railComHubFlow_;
  Buffer<dcc::RailcomHubData> *railComFeedback_{nullptr};
  RailComPhase railcomPhase_{RailComPhase::PRE_CUTOUT};
  bool enabled_{false};
};

template <class HW>
static void esp32_railcom_timer_tick(void *param)
{
  Esp32RailComDriver<HW> *driver =
    reinterpret_cast<Esp32RailComDriver<HW> *>(param);
  driver->timer_tick();
}

template <class HW>
static void esp32_railcom_uart_isr(void *param)
{
  Esp32RailComDriver<HW> *driver =
    reinterpret_cast<Esp32RailComDriver<HW> *>(param);
  Buffer<dcc::RailcomHubData> *fb = driver->buf();

  if (HW::UART_BASE->int_st.rxfifo_full  // RX fifo is full
   || HW::UART_BASE->int_st.rxfifo_tout) // RX data available
  {
    uint8_t rx_fifo_len = HW::UART_BASE->status.rxfifo_cnt;
    if (driver->railcom_phase() ==
        Esp32RailComDriver<HW>::RailComPhase::CUTOUT_PHASE1)
    {
      // this will flush the uart and process only the first two bytes
      for (uint8_t idx = 0; idx < rx_fifo_len; idx++)
      {
        fb->data()->add_ch1_data(HW::UART_BASE->fifo.rw_byte);
      }
    }
    else if (driver->railcom_phase() ==
             Esp32RailComDriver<HW>::RailComPhase::CUTOUT_PHASE2)
    {
      // this will flush the uart and process only the first six bytes
      for (uint8_t idx = 0; idx < rx_fifo_len; idx++)
      {
        fb->data()->add_ch2_data(HW::UART_BASE->fifo.rw_byte);
      }
    }

    // clear interrupt status
    HW::UART_BASE->int_clr.val = (UART_RXFIFO_FULL_INT_CLR
                                | UART_RXFIFO_TOUT_INT_CLR);
  }
  else if (HW::UART_BASE->int_st.txfifo_empty
        || HW::UART_BASE->int_st.tx_done)
  {
    // clear interrupt status
    HW::UART_BASE->int_clr.val = (UART_TXFIFO_EMPTY_INT_CLR
                                | UART_RXFIFO_TOUT_INT_CLR);
  }
  else
  {
    ets_printf("unexpected UART status %04x\n", HW::UART_BASE->int_st.val);
    HW::UART_BASE->int_clr.val = HW::UART_BASE->int_st.val;
  }
}

} // namespace esp32cs

#endif // ESP32_RAILCOM_DRIVER_H_
