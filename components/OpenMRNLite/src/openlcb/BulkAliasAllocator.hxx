/** \copyright
 * Copyright (c) 2020 Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file BulkAliasAllocator.hxx
 *
 * State flow for allocating many aliases at the same time.
 *
 * @author Balazs Racz
 * @date 14 Nov 2020
 */

#include "executor/CallableFlow.hxx"
#include "openlcb/AliasAllocator.hxx"
#include "openlcb/AliasCache.hxx"
#include "openlcb/CanDefs.hxx"

namespace openlcb
{

/// Message type to request allocating many aliases for an interface.
struct BulkAliasRequest : CallableFlowRequestBase
{
    /// @param count how many aliases to allocate.
    void reset(unsigned count)
    {
        reset_base();
        numAliases_ = count;
    }

    /// How many aliases to allocate.
    unsigned numAliases_;
};

using BulkAliasAllocatorInterface = FlowInterface<Buffer<BulkAliasRequest>>;

/// Creates a bulk alias allocator.
/// @param can_if the interface to bind it to.
std::unique_ptr<BulkAliasAllocatorInterface> create_bulk_alias_allocator(
    IfCan *can_if);

} // namespace openlcb