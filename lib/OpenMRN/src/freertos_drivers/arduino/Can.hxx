/** \copyright
 * Copyright (c) 2018, Balazs Racz
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
 * \file Can.hxx
 *
 * Alternative base class for CAN device drivers that can be compiled in an
 * Arduino environment.
 *
 * @author Balazs Racz
 * @date 8 July 2018
 */

// This include is exclusive against freertos_drivers/common/Can.hxx
#ifndef _FREERTOS_DRIVERS_ARDUINO_CAN_HXX_
#define _FREERTOS_DRIVERS_ARDUINO_CAN_HXX_

#include "DeviceBuffer.hxx"
#include "can_frame.h"
#include "nmranet_config.h"
#include "os/OS.hxx"
#include "utils/Atomic.hxx"

/** Base class for a CAN device for the Arduino environment. */
class Can : protected Atomic
{
public:
    static unsigned numReceivedPackets_;
    static unsigned numTransmittedPackets_;

    /// @return number of CAN frames available for read (input frames).
    int available()
    {
        return rxBuf->pending();
    }

    /// @return number of CAN frames available for write (space in output
    /// buffer).
    int availableForWrite()
    {
        return txBuf->space();
    }

    /// Read a frame if there is one available.
    /// @param frame will be filled with the input CAN frame.
    /// @return 0 or 1 depending on whether a frame was read or not.
    int read(struct can_frame *frame)
    {
        AtomicHolder h(this);
        auto ret = rxBuf->get(frame, 1);
        return ret;
    }

    /// Send a frame if there is space available.
    /// @param frame the output CAN frame.
    /// @return 0 or 1 depending on whether the write happened or not.
    int write(const struct can_frame *frame)
    {
        AtomicHolder h(this);
        auto ret = txBuf->put(frame, 1);
        if (ret)
        {
            tx_msg();
        }
        return ret;
    }

    virtual void enable() = 0;  /**< function to enable device */
    virtual void disable() = 0; /**< function to disable device */

protected:
    /** Constructor
     * @param name ignored, may be null
     */
    Can(const char *ignored)
        : txBuf(DeviceBuffer<struct can_frame>::create(
              config_can_tx_buffer_size(), config_can_tx_buffer_size() / 2))
        , rxBuf(DeviceBuffer<struct can_frame>::create(
              config_can_rx_buffer_size()))
        , overrunCount(0)
        , busOffCount(0)
        , softErrorCount(0)
    {
    }

    /** Destructor.
     */
    ~Can()
    {
        txBuf->destroy();
        rxBuf->destroy();
    }

    virtual void tx_msg() = 0; /**< function to try and transmit a message */

    DeviceBuffer<struct can_frame> *txBuf; /**< transmit buffer */
    DeviceBuffer<struct can_frame> *rxBuf; /**< receive buffer */
    unsigned int overrunCount;             /**< overrun count */
    unsigned int busOffCount;              /**< bus-off count */
    unsigned int softErrorCount;           /**< soft error count */

private:
    DISALLOW_COPY_AND_ASSIGN(Can);
};

#endif /* _FREERTOS_DRIVERS_ARDUINO_CAN_HXX_ */
