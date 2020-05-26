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
 * \file UpdateLoop.hxx
 *
 * Proxy implementation to the global update loop.
 *
 * @author Balazs Racz
 * @date 10 May 2014
 */

#include "dcc/UpdateLoop.hxx"
#include "utils/Singleton.hxx"

namespace dcc {

void packet_processor_notify_update(PacketSource* source, unsigned code) {
  Singleton<UpdateLoopBase>::instance()->notify_update(source, code);
}

/** Adds a new refresh source to the background refresh loop. */
bool packet_processor_add_refresh_source(
    PacketSource *source, unsigned priority)
{
    return Singleton<UpdateLoopBase>::instance()->add_refresh_source(
        source, priority);
}

/** Removes a refresh source from the background refresh loop. */
void packet_processor_remove_refresh_source(PacketSource* source) {
  Singleton<UpdateLoopBase>::instance()->remove_refresh_source(source);
}

UpdateLoopBase::~UpdateLoopBase() {}

}

//DEFINE_SINGLETON_INSTANCE(dcc::UpdateLoopBase);
