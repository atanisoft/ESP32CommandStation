/** \copyright
 * Copyright (c) 2020, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file DccOutput.hxx
 *
 * Common definitions (base classes) for DCC output drivers. The role of the
 * output driver is to control the output enable / disable and railcom cutout.
 *
 * @author Balazs Racz
 * @date 19 Apr 2020
 */

#ifndef _DCC_DCCOUTPUT_HXX_
#define _DCC_DCCOUTPUT_HXX_

#include <atomic>
#include <cstdint>

/// Virtual base class for controlling outputs. All of these functions are okay
/// to call from interrupts (including non-kernel-compatible interrupts under
/// FreeRTOS).
class DccOutput
{
public:
    /// Enumeration describing different outputs.
    enum Type : int
    {
        /// DCC output of the integrated booster.
        TRACK = 1,
        /// DCC output of the program track.
        PGM = 2,
        /// DCC output going towards the LCC cable.
        LCC = 3,
    };

    /// Values of a bit mask why we might want to disable a given DCC output.
    enum class DisableReason : uint8_t
    {
        /// Set as 1 during construction time, to be cleared by the application
        /// when the initialization is complete.
        INITIALIZATION_PENDING = 1,
        /// User decided via a persistent configuration that this output should
        /// not be enabled.
        CONFIG_SETTING = 2,
        /// A network message requested global emergency off.
        GLOBAL_EOFF = 4,
        /// Short detector says this output is shorted.
        SHORTED = 8,
        /// The system is in thermal shutdown.
        THERMAL = 16,
        /// This output should be off due to the conflict between program track
        /// and normal operation mode.
        PGM_TRACK_LOCKOUT = 32,
    };

    /// Disables the output, marking in a bitmask why.
    virtual void disable_output_for_reason(DisableReason bit) = 0;

    /// Removes a disable reason flag. All the flags need to be cleared in
    /// order to enable the output.
    virtual void clear_disable_output_for_reason(DisableReason bit) = 0;

    /// Sets or clears a disable reason.
    /// @param bit the disable reason
    /// @param value if true, bit set to disable output, if false, bit cleared
    /// to not disable output.
    void override_disable_bit_for_reason(DisableReason bit, bool value)
    {
        if (value)
        {
            disable_output_for_reason(bit);
        }
        else
        {
            clear_disable_output_for_reason(bit);
        }
    }

    /// @return Bitmask of all currently set disable reasons.
    virtual uint8_t get_disable_output_reasons() = 0;

    /// Defines the values for the railcom cutout enabled setting.
    enum class RailcomCutout
    {
        /// Generate no railcom cutout.
        DISABLED = 0,
        /// Generate short cutout (ch1 only).
        SHORT_CUTOUT = 1,
        /// Generate long cutout (standard size; ch1+ch2).
        LONG_CUTOUT = 2
    };

    /// Specifies whether there should be a railcom cutout on this output.
    virtual void set_railcom_cutout_enabled(RailcomCutout cutout) = 0;
};

/// Public API accessor for applications to get the object representing the
/// output hardware.
/// @param type which output reference to get.
/// @return an object to be used by the application to control the output.
DccOutput *get_dcc_output(DccOutput::Type type);

/// Default implementation class of the DccOutput that proxies the calls to a
/// hardware-specific static structure.
template <class HW> class DccOutputImpl : public DccOutput
{
public:
    constexpr DccOutputImpl()
    {
    }

    /// Disables the output, marking in a bitmask why.
    void disable_output_for_reason(DisableReason bit) override
    {
        HW::set_disable_reason(bit);
    }
    /// Removes a disable reason flag. All the flags need to be cleared in
    /// order to enable the output.
    void clear_disable_output_for_reason(DisableReason bit) override
    {
        HW::clear_disable_reason(bit);
    }

    /// Specifies whether there should be a railcom cutout on this output.
    void set_railcom_cutout_enabled(RailcomCutout cutout) override
    {
        HW::isRailcomCutoutEnabled_ = (uint8_t)cutout;
    }

    /// @return Bitmask of all currently set disable reasons.
    uint8_t get_disable_output_reasons() override
    {
        return HW::outputDisableReasons_;
    }

    /// @return the default instance created during initialization.
    static constexpr DccOutput *instance()
    {
        return const_cast<DccOutput *>(
            static_cast<const DccOutput *>(&instance_));
    }

private:
    /// Default instance to be used.
    static const DccOutputImpl<HW> instance_;
};

/// Allocates the storage by the linker for the static default instance.
template <class HW> const DccOutputImpl<HW> DccOutputImpl<HW>::instance_;

/// This structure represents a single output channel for a DCC command
/// station. The DCC driver uses these structures, with the business logic
/// filled in by the hardware implementor.
///
/// @param OUTPUT_NUMBER 0,1,... the number of outputs. Each output is
/// independently controlled.
template <int OUTPUT_NUMBER> struct DccOutputHw
{
public:
    /// Bitmask of why this output should be disabled. If zero, the output
    /// should be enabled. If non-zero, the output should be disabled. A
    /// variety of system components own one bit in this bitmask each; see {
    /// \link DccOutput::DisableReason } These bits are all set by the
    /// application. The DCC Driver will only read this variable, and enable
    /// the output if all bits are zero.
    static std::atomic_uint8_t outputDisableReasons_;

    /// 0 if we should not produce a railcom cutout; 1 for short cutout; 2 for
    /// regular cutout. Set by the application and read by the DCC driver.
    static std::atomic_uint8_t isRailcomCutoutEnabled_;

    /// 1 if we are in a railcom cutout currently. Set and cleared by the
    /// driver before calling the start/stop railcom cutout functions.
    static std::atomic_uint8_t isRailcomCutoutActive_;

    /// Called by the driver to decide whether to make this channel participate
    /// in the railcom cutout.
    static bool need_railcom_cutout()
    {
        return (outputDisableReasons_ == 0) && (isRailcomCutoutEnabled_ != 0);
    }

    /// Called once after the railcom cutout is done to decide whether this
    /// output should be reenabled.
    static bool should_be_enabled()
    {
        return outputDisableReasons_ == 0;
    }

    /// Clears a disable reason. If all disable reasons are clear, the output
    /// will be enabled by the DCC driver at the beginning of the next packet.
    static void clear_disable_reason(DccOutput::DisableReason bit)
    {
        outputDisableReasons_ &= ~((uint8_t)bit);
    }

protected:
    /// Set one bit in the disable reasons bit field. This is protected,
    /// because the implementation / child class should have a composite
    /// function that sets the bit and disables the output in one call.
    static void set_disable_reason_impl(DccOutput::DisableReason bit)
    {
        outputDisableReasons_ |= (uint8_t)bit;
    }

private:
    /// Private constructor. These objects cannot be initialized and must only
    /// have static members.
    DccOutputHw();
};

template <int N>
std::atomic_uint8_t DccOutputHw<N>::outputDisableReasons_ {
    (uint8_t)DccOutput::DisableReason::INITIALIZATION_PENDING};
template <int N>
std::atomic_uint8_t DccOutputHw<N>::isRailcomCutoutEnabled_ {
    (uint8_t)DccOutput::RailcomCutout::LONG_CUTOUT};
template <int N> std::atomic_uint8_t DccOutputHw<N>::isRailcomCutoutActive_ {0};

/// Interface that the actual outputs have to implement in their
/// hardware-specific classes.
template <int N> struct DccOutputHwDummy : public DccOutputHw<N>
{
public:
    /// Called once during hw_preinit boot state.
    static void hw_preinit(void)
    {
    }

    /// Invoked at the beginning of a railcom cutout. @return the number of usec
    /// to wait before invoking phase2.
    static unsigned start_railcom_cutout_phase1(void)
    {
        return 0;
    }

    /// Invoked at the beginning of a railcom cutout after the delay. @return
    /// number of usec to delay before enabling railcom UART receive.
    static unsigned start_railcom_cutout_phase2(void)
    {
        return 0;
    }

    /// Invoked at the end of a railcom cutout. @return the number of usec to
    /// wait before invoking phase2.
    static unsigned stop_railcom_cutout_phase1(void)
    {
        return 0;
    }

    /// Invoked at the end of a railcom cutout.
    static void stop_railcom_cutout_phase2(void)
    {
    }

    /// Called once every packet by the driver, typically before the preamble,
    /// if the output is supposed to be on.
    static void enable_output(void)
    {
    }

    /// A dummy output never needs a railcom cutout.
    static bool need_railcom_cutout()
    {
        return false;
    }

    static void set_disable_reason(DccOutput::DisableReason bit)
    {
        DccOutputHwDummy::set_disable_reason_impl(bit);
        // no actual output needs to be disabled.
    }
};

/// Generic implementation of the actual HW output with a booster enable and a
/// railcom enable GPIO.
/// @param N is the output number
/// @param BOOSTER_ENABLE is a GPIO structure that turns the output
/// on/off. set(true) is on.
/// @param RAILCOM_ENABLE is a GPIO structure that turns the RailCom FETs on.
/// set(true) starts the cutout.
/// @param DELAY_ON_1 is the number of usec to delay from BOOSTER_ENABLE off to
/// RAILCOM_ENABLE on.
/// @param DELAY_ON_2 is the number of usec to delay from RAILCOM_ENABLE on to
/// railcom UART on.
/// @param DELAY_OFF is the number of usec to delay from RAILCOM_ENABLE off to
/// booster enable on.
template <int N, class BOOSTER_ENABLE, class RAILCOM_ENABLE,
    unsigned DELAY_ON_1, unsigned DELAY_ON_2, unsigned DELAY_OFF>
struct DccOutputHwReal : public DccOutputHw<N>
{
public:
    /// Called once during hw_preinit boot state.
    static void hw_preinit(void)
    {
        BOOSTER_ENABLE::hw_init();
        BOOSTER_ENABLE::set(false);
        RAILCOM_ENABLE::hw_init();
        RAILCOM_ENABLE::set(false);
    }

    /// Invoked at the beginning of a railcom cutout. @return the number of usec
    /// to wait before invoking phase2.
    static unsigned start_railcom_cutout_phase1(void)
    {
        BOOSTER_ENABLE::set(false);
        return DELAY_ON_1;
    }

    /// Invoked at the beginning of a railcom cutout after the delay. @return
    /// number of usec to delay before enabling railcom UART receive.
    static unsigned start_railcom_cutout_phase2(void)
    {
        RAILCOM_ENABLE::set(true);
        return DELAY_ON_2;
    }

    /// Invoked at the end of a railcom cutout. @return the number of usec to
    /// wait before invoking phase2.
    static unsigned stop_railcom_cutout_phase1(void)
    {
        RAILCOM_ENABLE::set(false);
        return DELAY_OFF;
    }

    /// Invoked at the end of a railcom cutout.
    static void stop_railcom_cutout_phase2(void)
    {
        /// @todo consider checking whether output disable reason == 0 then
        /// enable the output?
    }

    /// Called once every packet by the driver, typically before the preamble,
    /// if the output is supposed to be on.
    static void enable_output(void)
    {
        BOOSTER_ENABLE::set(true);
    }

    static void set_disable_reason(DccOutput::DisableReason bit)
    {
        DccOutputHw<N>::set_disable_reason_impl(bit);
        BOOSTER_ENABLE::set(false);
    }
};

#endif // _DCC_DCCOUTPUT_HXX_
