#ifndef _OPENLCB_SERVOCONSUMER_HXX_
#define _OPENLCB_SERVOCONSUMER_HXX_

#include "freertos_drivers/common/DummyGPIO.hxx"
#include "freertos_drivers/common/PWM.hxx"
#include "openlcb/ServoConsumerConfig.hxx"
#include "os/MmapGpio.hxx"
#include <memory>

namespace openlcb
{

/// Basically a specialized ConfiguredConsumer.
/// Can't subclass ConfiguredConsumer here because ServoConsumerConfig
/// isn't a subclass of ConsumerConfig,
class ServoConsumer : public DefaultConfigUpdateListener
{
public:
    ServoConsumer(Node *node, const ServoConsumerConfig &cfg,
        const uint32_t pwmCountPerMs, PWM *pwm)
        : DefaultConfigUpdateListener()
        , pwmCountPerMs_(pwmCountPerMs)
        , pwm_(pwm)        // save for apply_config, where we actually use it.
        , pwmGpo_(nullptr) // not initialized until apply_config
        , gpioImpl_(node, 0, 0, DummyPinWithRead())
        , consumer_(&gpioImpl_) // don't connect consumer to PWM yet
        , cfg_(cfg)
    {
    }

    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify n(done);

        const EventId cfg_event_min = cfg_.event_rotate_min().read(fd);
        const EventId cfg_event_max = cfg_.event_rotate_max().read(fd);
        const int16_t cfg_servo_min_pct = cfg_.servo_min_percent().read(fd);
        const int16_t cfg_servo_max_pct = cfg_.servo_max_percent().read(fd);

        // 1ms duty cycle
        const uint32_t servo_ticks_0 = pwmCountPerMs_ * 1;
        // 2ms duty cycle
        const uint32_t servo_ticks_180 = pwmCountPerMs_ * 2;

        // Use a weighted average to determine num ticks for max/min.
        const uint32_t cfg_srv_ticks_min =
            ((100 - cfg_servo_min_pct) * servo_ticks_0 +
                cfg_servo_min_pct * servo_ticks_180) /
            100;
        const uint32_t cfg_srv_ticks_max =
            ((100 - cfg_servo_max_pct) * servo_ticks_0 +
                cfg_servo_max_pct * servo_ticks_180) /
            100;

        // Defaults to CLR at startup.
        const bool was_set = pwmGpo_ && (pwmGpo_->read() == Gpio::SET);

        if (!pwmGpo_ || //
            cfg_event_min != gpioImpl_.event_off() ||
            cfg_event_max != gpioImpl_.event_on() ||
            cfg_srv_ticks_min != pwmGpo_->get_off_counts() ||
            cfg_srv_ticks_max != pwmGpo_->get_on_counts())
        {
            auto saved_node = gpioImpl_.node();

            consumer_.~BitEventConsumer();
            gpioImpl_.~GPIOBit();

            pwmGpo_.reset(new PWMGPO(pwm_,
                /*on_counts=*/cfg_srv_ticks_max,
                /*off_counts=*/cfg_srv_ticks_min));
            pwmGpo_->write(was_set ? Gpio::SET : Gpio::CLR);

            new (&gpioImpl_) GPIOBit(
                saved_node, cfg_event_min, cfg_event_max, pwmGpo_.get());
            new (&consumer_) BitEventConsumer(&gpioImpl_);

            return REINIT_NEEDED;
        }

        return UPDATED;
    }

    void factory_reset(int fd) OVERRIDE
    {
        cfg_.description().write(fd, "");
        CDI_FACTORY_RESET(cfg_.servo_min_percent);
        CDI_FACTORY_RESET(cfg_.servo_max_percent);
    }

private:
    /// Used to compute PWM ticks for max/min servo rotation.
    const uint32_t pwmCountPerMs_;

    /// timer channel. not owned; lives forever
    PWM *pwm_;

    /// all the rest are owned and must be reset on config change.
    /// pwmGpo_ heap-allocated because it's nullptr until first config.
    std::unique_ptr<PWMGPO> pwmGpo_; // has PWM* and on/off counts

    GPIOBit gpioImpl_;          /// has on/off events, Node*, and Gpio*
    BitEventConsumer consumer_; /// has GPIOBit*
    const ServoConsumerConfig cfg_;
};

} // namespace openlcb

#endif // _OPENLCB_SERVOCONSUMER_HXX_
