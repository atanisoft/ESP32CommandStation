/** \copyright
 * Copyright (c) 2016, Stuart W Baker
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
 * \file RailCom.h
 *
 * Defines a RailCom feedback structure.
 *
 * @author Stuart W Baker
 * @date 25 November 2016
 */

#ifndef _DCC_RAILCOM_H_
#define _DCC_RAILCOM_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dcc_feedback
{
    /// Number of bytes in channel one.
    uint8_t ch1Size;
    /// Payload of channel 1.
    uint8_t ch1Data[2];
    /// Number of bytes in channel two.
    uint8_t ch2Size;
    /// Payload of channel 2.
    uint8_t ch2Data[6];
    /// Used by multi-channel railcom receiver drivers. Specifies which
    /// hardware channel captured this data.
    uint8_t channel;
    /// Opaque identifier that allows linking outgoing dcc::Packet sent to the
    /// DCC waveform generator to the incoming dcc::Feedback structure read
    /// back from the railcom driver.
    uintptr_t feedbackKey;
} DCCFeedback;

#ifdef __cplusplus
}
#endif

#endif // _DCC_RAILCOM_H_
