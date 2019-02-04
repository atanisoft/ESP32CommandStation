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

#include "openlcb/TractionTestTrain.hxx"

#include "utils/logging.h"

namespace openlcb
{

LoggingTrain::LoggingTrain(uint32_t legacy_address)
    : legacyAddress_(legacy_address)
{
    LOG(INFO, "Created train %" PRIu32 ".", legacyAddress_);
}

LoggingTrain::~LoggingTrain()
{
    LOG(INFO, "Destructed train %" PRIu32 ".", legacyAddress_);
}

void LoggingTrain::set_speed(SpeedType speed)
{
    LOG(INFO, "train %" PRIu32 " : set speed to %c %.0f mph.", legacyAddress_,
        speed.direction() == speed.FORWARD ? 'F' : 'R', speed.mph());
    currentSpeed_ = speed;
    estopActive_ = false;
}

SpeedType LoggingTrain::get_speed()
{
    LOG(INFO, "train %" PRIu32 " : get speed -> returns %c %.0f mph.",
        legacyAddress_,
        currentSpeed_.direction() == currentSpeed_.FORWARD ? 'F' : 'R',
        currentSpeed_.mph());
    return currentSpeed_;
}

void LoggingTrain::set_emergencystop()
{
    LOG(INFO, "train %" PRIu32 " : set emergency stop.", legacyAddress_);
    estopActive_ = 0;
}

bool LoggingTrain::get_emergencystop()
{
    LOG(INFO, "train %" PRIu32 " : get emergency stop.", legacyAddress_);
    return estopActive_;
}

void LoggingTrain::set_fn(uint32_t address, uint16_t value)
{
    LOG(INFO, "train %" PRIu32 " : set fn %" PRIu32 " to %u.", legacyAddress_,
        address, value);
    fnValues_[address] = value;
}

uint16_t LoggingTrain::get_fn(uint32_t address)
{
    uint16_t resp = fnValues_[address];
    LOG(INFO, "train %" PRIu32 " : get fn %" PRIu32 " -> current value is %u.",
        legacyAddress_, address, resp);
    return resp;
}

uint32_t LoggingTrain::legacy_address()
{
    return legacyAddress_;
}

dcc::TrainAddressType LoggingTrain::legacy_address_type()
{
    return dcc::TrainAddressType::DCC_LONG_ADDRESS;
}

} // namespace openlcb
