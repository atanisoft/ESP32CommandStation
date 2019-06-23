/** \copyright
 *
 *    Copyright (C) 2019  Robert Heller D/B/A Deepwoods Software
 *			51 Locke Hill Road
 *			Wendell, MA 01379-9728
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
 *
 * \file LinuxPWM.hxx
 *
 * Defines PWM pins using the Linux sysfs ABI.
 *
 * \section HOWTOUSE How to use
 * TBD
 *
 * @author Robert Heller
 * @date 19 Feburary 2019
 */

#ifndef _OS_LINUXPWM_HXX_
#define _OS_LINUXPWM_HXX_

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

#include "freertos_drivers/common/PWM.hxx"
#include "utils/FileUtils.hxx"
#include "utils/StringPrintf.hxx"
#include "utils/format_utils.hxx"

/// Implementation of a single PWM channel using the linux sysfs API.
class LinuxPWM : public PWM
{
public:
    /// Constructor.
    /// @param chip is an integer, the index in the kernel where the PWM chip is
    /// registered.
    /// @param channel the index of the output channel on the specific chip to
    /// use.
    LinuxPWM(int chip, int channel)
        : chip_(chip)
        , channel_(channel)
        , pwmdir_(
              StringPrintf("/sys/class/pwm/pwmchip%d/pwm%d/", chip, channel))
    {
    }

    /// Call this once after instantiating the object but before using any of
    /// the PWM apis.
    void export_pin()
    {
        if (access((pwmdir_ + PERIOD_FILE).c_str(), F_OK) != -1)
        {
            set_period(1);
            return;
        }

        string export_file =
            StringPrintf("/sys/class/pwm/pwmchip%d/export", chip_);
        if (access(export_file.c_str(), W_OK) < 0) 
        {
            int err = errno;
            LOG_ERROR("Cannot write to %s: %s\n",export_file.c_str(),
                      strerror(errno));
            exit(err);
        }
        write_string_to_file(export_file, integer_to_string(channel_) + "\n");
        // 50ms delay IS needed while kernel changes ownership of created GPIO directory
        usleep(50000); 
        set_period(1);
    }

    void set_period(uint32_t counts) override
    {

        set_sysfs_file_value(PERIOD_FILE, counts);
        if (counts > 0)
        {


            enable();
        }
        else
        {
            disable();
        }
    }

    uint32_t get_period() override
    {

        return get_sysfs_file_value(PERIOD_FILE);

    }

    void set_duty(uint32_t counts) override
    {

        set_sysfs_file_value(DUTY_CYCLE_FILE, counts);
        if (counts > 0)
        {

            enable();
        }
        else
        {
            disable();
        }
    }

    uint32_t get_duty() override
    {

        return get_sysfs_file_value(DUTY_CYCLE_FILE);

    }

    uint32_t get_period_max() override
    {
        return 0xffffffff;
    }

    uint32_t get_period_min() override
    {
        return 1;
    }


    /// Enables the output PWM waveform.
    void enable()
    {
        set_sysfs_file_value(ENABLE_FILE, 1);
    }

    /// Disables the output PWM waveform.
    void disable()
    {
        set_sysfs_file_value(ENABLE_FILE, 0);
    }

    /// Returns whether the output PWM waveform is enabled or not.
    bool enabled()
    {
        return get_sysfs_file_value(ENABLE_FILE) == 1;


    }

private:
    /// Basename for sysfs file for enabling the channel.
    static constexpr const char *ENABLE_FILE = "enable";
    /// Basename for sysfs file for setting the period.
    static constexpr const char *PERIOD_FILE = "period";
    /// Basename for sysfs file for setting the duty cycle.
    static constexpr const char *DUTY_CYCLE_FILE = "duty_cycle";

    /// Kernel number of the chip.
    const uint32_t chip_;
    /// Number of the channel in the chip.
    const uint32_t channel_;
    /// Diretory path of the channel, with trailing /
    const string pwmdir_;

    /// Returns a single file's content from the channel directory on sysfs.
    /// @param basename is the name of the file (e.g. "period")
    /// @return the contents of the file converted to integer.
    uint32_t get_sysfs_file_value(const char *basename)
    {
        string filename(pwmdir_);
        filename += basename;
        uint32_t value = 0;
        FILE *fp = fopen(filename.c_str(), "r");
        if (fp == NULL)
        {
            int err = errno;
            LOG_ERROR("Cannot read from %s: %s\n",filename.c_str(),
                      strerror(errno));
            exit(err);
        }
        fscanf(fp, "%d", &value);
        fclose(fp);
        return value;
    }

    /// Writes a single integer into a sysfs file of the channel directory.
    /// @param basename is the name of the file (e.g. "period")
    /// @param value is the number to write.
    void set_sysfs_file_value(const char *basename, uint32_t value)
    {
        string filename(pwmdir_);
        filename += basename;
        FILE *fp = fopen(filename.c_str(), "w");
        if (fp == NULL)
        {
            int err = errno;
            LOG_ERROR("Cannot write to %s: %s\n",filename.c_str(),
                      strerror(errno));
            exit(err);
        }
        fprintf(fp, "%d\n", value);

        fclose(fp);
    }
};

#endif // _OS_LINUXPWM_HXX_
