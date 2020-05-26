/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file TractionTestTrain.hxx
 *
 * Train implementation plugins helpful for testing without a layout.
 *
 * @author Balazs Racz
 * @date 4 Aug 2014
 */

#ifndef _OPENLCB_TRACTIONTESTTRAIN_HXX_
#define _OPENLCB_TRACTIONTESTTRAIN_HXX_

#include <map>

#include "openlcb/TrainInterface.hxx"

namespace openlcb
{

/** Test train implementation that just logs every action to the info log. */
class LoggingTrain : public TrainImpl
{
public:
    LoggingTrain(uint32_t legacy_address,
        dcc::TrainAddressType address_type =
            dcc::TrainAddressType::DCC_LONG_ADDRESS);
    ~LoggingTrain();
    void set_speed(SpeedType speed) OVERRIDE;
    SpeedType get_speed() OVERRIDE;
    void set_emergencystop() OVERRIDE;
    bool get_emergencystop() OVERRIDE;
    void set_fn(uint32_t address, uint16_t value) OVERRIDE;
    uint16_t get_fn(uint32_t address) OVERRIDE;
    uint32_t legacy_address() OVERRIDE;
    dcc::TrainAddressType legacy_address_type() OVERRIDE;

private:
    uint32_t legacyAddress_;
    dcc::TrainAddressType legacyAddressType_;
    SpeedType currentSpeed_;
    bool estopActive_;
    std::map<uint32_t, uint16_t> fnValues_;
};

} // namespace openlcb

#endif  // _OPENLCB_TRACTIONTESTTRAIN_HXX_
