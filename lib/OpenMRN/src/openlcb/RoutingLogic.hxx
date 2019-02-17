/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file RoutingLogic.hxx
 *
 * Data structure for gateways and routers of OpenLCB, keeping track of the
 * necessary information to make routing decisions.
 *
 * @author Balazs Racz
 * @date 23 May 2016
 */

#ifndef _OPENLCB_ROUTNGLOGIC_HXX_
#define _OPENLCB_ROUTNGLOGIC_HXX_

#include <set>
#include <map>
#include <unordered_map>

#include "os/OS.hxx"
#include "openlcb/EventHandler.hxx"

namespace openlcb
{

/** Decodes an event range, encoded according to the Event Transport protocol
 * specification.
 *
 * @param event is a pointer to the variable holding the event range. This
 * value will be modified to hold only the base value of the event, without the
 * mask bits.
 *
 * @returns the number of mask bits that were there, in the range of 1..64.
 */
uint8_t event_range_to_bit_count(EventId *event);

/** Routing table for gateways and routers in OpenLCB.
 *
 * The routing table contains which direction to send addressed packets as well
 * as filters for the event IDs that have listeners in a given port.
 */
template <class Port, typename Address> class RoutingLogic
{
public:
    RoutingLogic()
    {
    }
    ~RoutingLogic()
    {
    }

    /** Clears all entries in the routing table related to a given port, as the
     * given port is being removed.
     *
     * @param port describes the target port to be removed.
     */
    void remove_port(Port *port)
    {
        OSMutexLock l(&lock_);
        eventRoutingTable_.erase(port);
        // Removing entries from a hashmap invalidates an iterator, thus it is
        // safer to null them out than actually remove. Having a null value
        // will cause address lookup to return null for a node that has not
        // been seen since then elsewhere, which is exactly the behavior we
        // want.
        for (auto &it : addressRoutingTable_)
        {
            if (it.second == port)
            {
                it.second = nullptr;
            }
        }
    }

    /** Declares that a given node ID is reachable via a specific port. Used
     * with the source node IDs of all the incoming packets.
     *
     * @param port is where the incoming packet came from (i.e. the port on
     * which source is reachable.
     * @param source is the node handle where the packet came from.
     */
    void add_node_id_to_route(Port *port, Address source)
    {
        OSMutexLock l(&lock_);
        addressRoutingTable_[source] = port;
    }

    /** Looks up which port an addressed packet should be sent to.
     *
     * @param dest is the address of the destination node that needs to be
     * contacted.
     * @returns a (live) port if the address is in the routing table, otherwise
     * nullptr.
     */
    Port *lookup_port_for_address(Address dest)
    {
        OSMutexLock l(&lock_);
        auto it = addressRoutingTable_.find(dest);
        if (it == addressRoutingTable_.end())
            return nullptr;
        return it->second;
    }

    /** Declares that there is a consumer for the given event ID on the given
     * port.
     *
     * @param port is where the consumer identified from has come from.
     * @param event is the event ID for which there is a consumer identified on
     * that port. */
    void register_consumer(Port *port, EventId event)
    {
        OSMutexLock l(&lock_);
        eventRoutingTable_[port].registeredConsumers_[0].insert(event);
    }

    /** Declares that there is a consumer for the given event ID range on the
     * given port.
     *
     * @param port is there the consumer range identified has come from.
     * @param encoded_range is the range of consumer encoded via the OpenLCB
     * method. */
    void register_consumer_range(Port *port, EventId encoded_range)
    {
        OSMutexLock l(&lock_);
        uint8_t bit_count = event_range_to_bit_count(&encoded_range);
        eventRoutingTable_[port].registeredConsumers_[bit_count].insert(
            encoded_range);
    }

    /** Declares that there is a producer for the given event ID on the given
     * port.
     *
     * @param port is where the producer identified from has come from.
     * @param event is the event ID for which there is a producer identified on
     * that port. */
    void register_producer(Port *port, EventId event)
    {
        // For the moment we do not keep separate routing tables for producers
        // and consumers.
        register_consumer(port, event);
    }

    /** Declares that there is a producer for the given event ID range on the
     * given port.
     *
     * @param port is there the producer range identified has come from.
     * @param encoded_range is the range of producer encoded via the OpenLCB
     * method. */
    void register_producer_range(Port *port, EventId encoded_range)
    {
        // For the moment we do not keep separate routing tables for producers
        // and consumers.
        register_consumer_range(port, encoded_range);
    }

    /** Checks if a given PCER message should be forwarded to the given port.
     *
     * @param port is the port to query.
     * @param event is the event ID from the PCER message.
     *
     * @return true if the given event has a consumer on the given port. */
    bool check_pcer(Port *port, EventId event)
    {
        OSMutexLock l(&lock_);
        auto ip = eventRoutingTable_.find(port);
        if (ip == eventRoutingTable_.end())
        {
            return false;
        }
        for (auto im = ip->second.registeredConsumers_.begin();
             im != ip->second.registeredConsumers_.end(); ++im)
        {
            if (im->first == 0)
            {
                if (im->second.find(event) != im->second.end())
                    return true;
            }
            else if (im->first == 64)
            {
                if (!im->second.empty())
                    return true;
            }
            else
            {
                EventId masked_range =
                    event & ~((UINT64_C(1) << im->first) - 1);
                if (im->second.find(masked_range) != im->second.end())
                    return true;
            }
        }
        return false;
    }

private:
    /// Protects all internal data structures.
    OSMutex lock_;

    /// Stores all known addresses and which port they route to.
    std::unordered_map<Address, Port *> addressRoutingTable_;

    /// The per-port event information.
    struct EventSet
    {
        /// key: number of bits set in the mask part. Valid values:
        /// 0..64. Value of 0 means individual event.
        std::map<uint8_t, std::set<EventId>> registeredConsumers_;
    };

    /// Stores per-port event information.
    std::map<Port *, EventSet> eventRoutingTable_;
};

} // namespace openlcb

#endif // _OPENLCB_ROUTNGLOGIC_HXX_
