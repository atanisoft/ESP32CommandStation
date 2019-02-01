/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file GridConnectHub.hxx
 * Interface for creating gridconnect parser/renderer pipe components.
 *
 * @author Balazs Racz
 * @date 20 May 2013
 */

#ifndef _UTILS_GRIDCONNECTHUB_HXX_
#define _UTILS_GRIDCONNECTHUB_HXX_

#include <memory>

#include "utils/Hub.hxx"

class Pipe;
template <class T> class FlowInterface;
template <class T, int N> class DispatchFlow;

/// Publicly visible API for the gridconnect-to-CAN bridge.  This bridge links
/// two Hubs, one typed string, the other typed CanHubData, by
/// parsing/rendering the packets from the gridconnect protocol.
///
/// This public-facing API allows creating and managing the bridge as necessary
/// for production code and unittests.
class GCAdapterBase
{
public:
    virtual ~GCAdapterBase()
    {
    }

    /// Unregisters *this from the pipes.
    /// @return true if it is safe to destroy *this. It is OK to call this
    /// multiple times. It should be called on the executor of the CAN side
    /// service. */
    virtual bool shutdown() = 0;

    /**
       This function connects an ASCII (GridConnect-format) CAN adapter to a
       binary CAN adapter, performing the necessary format conversions
       inbetween.

       Specifically, it takes two Hub flows as input, one carrying CAN frames
       in GridConnect protocol, and the other carrying CAN frames in the binary
       protocol.

       @param gc_side is the Hub that has the ASCII GridConnect traffic.

       @param can_side is the Hub that has the binary CAN traffic.

       @param double_bytes if true, any frame rendered into the GC protocol
       will have their characters doubled.

       @return a pointer to the created object. It can be deleted, which will
       terminate the link and unregister the link members from both pipes.
    */
    static GCAdapterBase *CreateGridConnectAdapter(HubFlow *gc_side,
                                                   CanHubFlow *can_side,
                                                   bool double_bytes);

    /// Creates a gridconnect-CAN bridge with separate pipes for reading
    /// (parsing) from the GC side and writing (formatting) to the GC side. */
    ///
    /// @param gc_side_read is the Hub that the GridConnect traffic is read
    /// from, to be converted and sent to binary.
    /// @param gc_side_write is the Hub that the coverted GridConnect traffic is
    /// written to.
    /// @param can_side the hub (of type struct can_frame) that the binary IO
    /// is done via.
    /// @param double_bytes  if true, any frame rendered into the GC protocol
    ///   will have their characters doubled.
    ///
    /// @return a pointer to the created object. It can be deleted, which will
    ///   terminate the link and unregister the link members from both pipes.
    ///
    static GCAdapterBase *CreateGridConnectAdapter(HubFlow *gc_side_read,
        HubFlow *gc_side_write, CanHubFlow *can_side, bool double_bytes);
};

/** Create this port for a CAN hub and all packets will be written to stdout in
 * gridconnect format. */
class GcPacketPrinter
{
public:
    /// constructor
    ///
    /// @param can_hub Which hub's packets to write to stdout.
    /// @param timestamped Whether to put timestamps on the packets written.
    ///
    GcPacketPrinter(CanHubFlow *can_hub, bool timestamped);
    ~GcPacketPrinter();
private:
    /// pImpl class.
    struct Impl;
    /// pImpl object.
    std::unique_ptr<Impl> impl_;
};

/** Creates a new port on a CAN hub in gridconnect format for a
 * select-compatible file descriptor. The port will automatically be closed,
 * deleted and on_exit notified when the fd encounters an error.
 *
 * NOTE(balazs.racz): this could be expanded to return an object pointer via
 * which the port can be closed.
 *
 * @param can_hub the raw CAN packets are coming/going to this object.
 * @param fd the file descriptor of the port to send/receive the gridconnect
 * ascii data to/from.
 * @param on_exit is a notifiable (may be null) which will be called in case
 * an error is encountered on this port and the port is subsequently closed.
 * @param use_select when true, the FD will be used with select, when false,
 * separate threads will be started with blocking read and write calls. */
void create_gc_port_for_can_hub(CanHubFlow *can_hub, int fd,
    Notifiable *on_exit = nullptr, bool use_select = false);

#endif //_UTILS_GRIDCONNECTHUB_HXX_
