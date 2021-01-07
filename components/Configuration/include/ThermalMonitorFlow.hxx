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

#include "sdkconfig.h"
#include "ThermalConfiguration.hxx"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <executor/StateFlow.hxx>
#include <openlcb/EventHandlerTemplates.hxx>
#include <utils/ConfigUpdateListener.hxx>

#ifndef CONFIG_THERMALMONITOR_ZERO_MV
#define CONFIG_THERMALMONITOR_ZERO_MV 0
#endif

#ifndef CONFIG_THERMALMONITOR_MV_PER_C
#define CONFIG_THERMALMONITOR_MV_PER_C 10
#endif

namespace esp32cs
{

/// StateFlow which will monitor an external temperature sensor connected to an
/// ADC pin and raise events based on thresholds.
class ThermalMonitorFlow : public StateFlowBase,
                           public DefaultConfigUpdateListener
{
public:
    /// Constructor.
    ///
    /// @param service is the @ref Service to attach this StateFlow to.
    /// @param node is the @ref Node to use for producing events from.
    /// @param cfg is the configuration for the temperature sensor.
    ThermalMonitorFlow(Service *service, openlcb::Node *node,
                       const ThermalConfiguration &cfg,
                       const adc1_channel_t channel)
        : StateFlowBase(service)
        , cfg_(cfg)
        , channel_(channel)
        , bitWarning_(node, 0, 0, &state_, (uint8_t)STATE_WARNING)
        , bitShutdown_(node, 0, 0, &state_, (uint8_t)STATE_SHUTDOWN)
    {
    }

    /// Updates the ThermalConfiguration settings used by this node.
    ///
    /// @param fd is the file descriptor used for the configuration settings.
    /// @param initial_load is set to true when this node loads the
    /// configuration for the first time, otherwise it is an update to the
    /// configuration and may require a restart.
    /// @param done is the control used by the caller to track when all config
    /// consumers have completed their updates.
    ///
    /// @return UPDATED when the configuration has been successfully updated,
    /// or REINIT_NEEDED if the node needs to reinitialize events.
    ConfigUpdateListener::UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) override
    {
        AutoNotify n(done);
        ConfigUpdateListener::UpdateAction res = UPDATED;

        // load the warning and shutdown temperature limits.
        warningTemperature_ =
            CDI_READ_TRIM_DEFAULT(cfg_.temperature_warning, fd);
        shutdownTemperature_ =
            CDI_READ_TRIM_DEFAULT(cfg_.temperature_shutdown, fd);

        // process the warning temperature events.
        openlcb::EventId warning_event_active =
            cfg_.event_warning().read(fd);
        openlcb::EventId warning_event_clear =
            cfg_.event_warning_clear().read(fd);
        if (warning_event_active != bitWarning_.event_off() ||
            warning_event_clear != bitWarning_.event_on())
        {
            auto saved_node = bitWarning_.node();
            bitWarning_.openlcb::MemoryBit<uint8_t>::~MemoryBit();
            new (&bitWarning_) openlcb::MemoryBit<uint8_t>(saved_node,
                warning_event_active, warning_event_clear, &state_,
                STATE_WARNING);
            res = REINIT_NEEDED;
        }

        // process the shutdown temperature events.
        openlcb::EventId shutdown_event_active =
            cfg_.event_shutdown().read(fd);
        openlcb::EventId shutdown_event_clear =
            cfg_.event_shutdown_clear().read(fd);
        if (shutdown_event_active != bitShutdown_.event_off() ||
            shutdown_event_clear != bitShutdown_.event_on())
        {
            auto saved_node = bitShutdown_.node();
            bitShutdown_.openlcb::MemoryBit<uint8_t>::~MemoryBit();
            new (&bitShutdown_) openlcb::MemoryBit<uint8_t>(saved_node,
                shutdown_event_active, shutdown_event_clear, &state_,
                STATE_SHUTDOWN);
            res = REINIT_NEEDED;
        }

        // if this is the initial load we need to calibrate the ADC and start
        // monitoring.
        if (initial_load)
        {
            adc1_config_channel_atten(channel_, ADC_ATTEN_DB_11);
            esp_adc_cal_value_t calibration_type =
                esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11
                                    , ADC_WIDTH_BIT_12, DEFAULT_ADC_VREF
                                    , &calibration_);
            LOG(INFO
              , "[ThermalMonitor] Using vRef: %s (%d mV), mV/C: %.2f, 0C: %d mV"
              , calibration_type == ESP_ADC_CAL_VAL_EFUSE_VREF ? "eFuse"
              : calibration_type == ESP_ADC_CAL_VAL_EFUSE_TP ? "two-point"
              : "default", calibration_.vref, MV_PER_C, MV_AT_ZERO_C);
            start_flow(STATE(check_temperature));
        }

        return res;
    }

    /// Resets the ThermalConfiguration settings used by this node.
    ///
    /// @param fd is the file descriptor used for the configuration settings.
    void factory_reset(int fd) override
    {
        LOG(VERBOSE, "[ThermalMonitor] factory_reset(%d)", fd);
        CDI_FACTORY_RESET(cfg_.temperature_warning);
        CDI_FACTORY_RESET(cfg_.temperature_shutdown);
    }

private:
    /// Bitmask for @ref state_.
    enum STATE : uint8_t
    {
        /// Temperature is below the warning temperature.
        STATE_NORMAL        = BIT(0)
        /// Temperature is above the warning temperature but below the shutdown
        /// temperature.
      , STATE_WARNING       = BIT(1)
        /// Temperature is above the shutdown temperature or there was a
        /// failure in reading the temperature.
      , STATE_SHUTDOWN      = BIT(2)
    };

    /// Default ADC voltage reference value.
    static constexpr uint32_t DEFAULT_ADC_VREF = 1100;

    /// Calculated millivolts per degree C.
    static constexpr float MV_PER_C = CONFIG_THERMALMONITOR_MV_PER_C / 10.0f;

    /// Millivolts for zero degrees C.
    static constexpr uint32_t MV_AT_ZERO_C = CONFIG_THERMALMONITOR_ZERO_MV;

    /// Configuration to use for thermal monitoring.
    const ThermalConfiguration cfg_;

    /// ADC channel used for thermal monitoring.
    const adc1_channel_t channel_;

    /// ADC Calibration settings.
    esp_adc_cal_characteristics_t calibration_;

    /// Current state of the temperature sensor.
    uint8_t state_{STATE_NORMAL};

    /// Temporary holder for secondary state when transitioning from one state
    /// to another that requires multiple events to be raised.
    uint8_t nextState_{STATE_NORMAL};

    /// Cache for the warning temperature limit.
    uint8_t warningTemperature_{50};

    /// Cache for the shutdown temperature limit.
    uint8_t shutdownTemperature_{80};

    /// Memory-mapped bit mask for the @ref state_ used for warning events.
    openlcb::MemoryBit<uint8_t> bitWarning_;

    /// Memory-mapped bit mask for the @ref state_ used for shutdown events.
    openlcb::MemoryBit<uint8_t> bitShutdown_;

    /// Event producer for the warning events.
    openlcb::BitEventProducer producerWarning_{&bitWarning_};

    /// Event producer for the shutdown events.
    openlcb::BitEventProducer producerShutdown_{&bitShutdown_};

    /// Timer used for allowing this stateflow to sleep between checks.
    StateFlowTimer timer_{this};

    /// Helper used for sending events to the OpenLCB bus.
    openlcb::WriteHelper helper_;

    /// Notifiable object used for signaling when an event has been dispatched
    /// to the OpenLCB bus.
    BarrierNotifiable n_;

    /// Reads the external temperature sensor of an ESP32-S2.
    float read_external_temperature()
    {
        uint32_t temperature =
            esp_adc_cal_raw_to_voltage(adc1_get_raw(channel_), &calibration_);
        return ((temperature - MV_AT_ZERO_C) / MV_PER_C);
    }

    /// Reads the current temperature from the selected sensor and checks
    /// for change with the warning/shutdown limits. If the temperature results
    /// in a state change it will produce at least one event.
    Action check_temperature()
    {
        uint8_t current_state = state_;
        float board_temp_c = read_external_temperature();
        float board_temp_f = (board_temp_c * (9.0f / 5.0f)) + 32.0f;
        LOG(INFO, "[ThermalMonitor] Current temperature: %.2fC (%.2fF)"
          , board_temp_c, board_temp_f);
        if (board_temp_c > shutdownTemperature_)
        {
            state_ = STATE_SHUTDOWN;
        }
        else if (board_temp_c > warningTemperature_)
        {
            state_ = STATE_WARNING;
        }
        else
        {
            state_ = STATE_NORMAL;
        }

        if (state_ != current_state)
        {
            if (current_state == STATE_SHUTDOWN &&
                (state_ == STATE_WARNING || state_ == STATE_NORMAL))
            {
                nextState_ = STATE_WARNING;
                // send shutdown cleared event
                producerShutdown_.SendEventReport(&helper_, n_.reset(this));
            }
            else if (current_state == STATE_WARNING && state_ == STATE_NORMAL)
            {
                nextState_ = STATE_NORMAL;
                // send warning cleared event
                producerWarning_.SendEventReport(&helper_, n_.reset(this));
            }
            else if (state_ == STATE_WARNING)
            {
                // send warning event
                producerWarning_.SendEventReport(&helper_, n_.reset(this));
            }
            else if (state_ == STATE_SHUTDOWN)
            {
                // send shutdown event
                producerShutdown_.SendEventReport(&helper_, n_.reset(this));
            }

            return wait_and_call(STATE(next_event));
        }
        return yield_and_call(STATE(sleep));
    }

    /// This method takes care of sending a second event for certain state
    /// transitions.
    Action next_event()
    {
        if (nextState_ == STATE_WARNING)
        {
            // clear the warning state flag
            nextState_ = STATE_NORMAL;

            // send warning cleared event
            producerWarning_.SendEventReport(&helper_, n_.reset(this));

            // wait for event to be sent and go back to sleep
            return wait_and_call(STATE(sleep));
        }
        return yield_and_call(STATE(sleep));
    }

    /// Delays the reading of the temperature sensor.
    Action sleep()
    {
        return sleep_and_call(&timer_, SEC_TO_NSEC(10)
                            , STATE(check_temperature));
    }
};

} // namespace esp32s2io