/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file If.hxx
 *
 * Asynchronous NMRAnet interface.
 *
 * @author Balazs Racz
 * @date 3 Dec 2013
 */

#ifndef _OPENLCB_IF_HXX_
#define _OPENLCB_IF_HXX_

/// @todo(balazs.racz) remove this dep
#include <string>

#include "openlcb/Node.hxx"
#include "openlcb/Defs.hxx"
#include "executor/Dispatcher.hxx"
#include "executor/Service.hxx"
#include "executor/Executor.hxx"
#include "utils/Buffer.hxx"
#include "utils/Queue.hxx"
#include "utils/Map.hxx"

namespace openlcb
{

class Node;

/// Container that carries the data bytes in an NMRAnet message.
typedef string Payload;

/** Convenience function to render a 48-bit NMRAnet node ID into a new buffer.
 *
 * @param id is the 48-bit ID to render.
 * @returns a new buffer (from the main pool) with 6 bytes of used space, a
 * big-endian representation of the node ID.
 */
extern string node_id_to_buffer(NodeID id);
/** Convenience function to render a 48-bit NMRAnet node ID into an existing
 * buffer.
 *
 * @param id is the 48-bit ID to render.
 * @param data is the memory space to write the rendered ID into. There must be
 * at least 6 bytes available at this address.
 */
extern void node_id_to_data(NodeID id, void* data);

/** Converts a 6-byte-long buffer to a node ID.
 *
 * @param buf is a buffer that has to have exactly 6 bytes used, filled with a
 * big-endian node id.
 * @returns the node id (in host endian).
 */
extern NodeID buffer_to_node_id(const string& buf);
/** Converts 6 bytes of big-endian data to a node ID.
 *
 * @param d is a pointer to at least 6 valid bytes.
 * @returns the node ID represented by the first 6 bytes of d.
 */
extern NodeID data_to_node_id(const void* d);

/** Converts an Event ID to a Payload suitable to be sent as an event report. */
extern Payload eventid_to_buffer(uint64_t eventid);

/** Takes 8 bytes (big-endian) from *data, and returns the event id they
 * represent. */
inline uint64_t data_to_eventid(const void* data) {
    uint64_t ret = 0;
    memcpy(&ret, data, 8);
    return be64toh(ret);
}

/** Formats a payload for response of error response messages such as OPtioanl
 * Interaction Rejected or Terminate Due To Error. */
extern string error_to_buffer(uint16_t error_code, uint16_t mti);

/** Formats a payload for response of error response messages such as Datagram
 * Rejected. */
extern string error_to_buffer(uint16_t error_code);

/** Writes an error code into a payload object at a given pointer. */
extern void error_to_data(uint16_t error_code, void* data);

/** Parses an error code from a payload object at a given pointer. */
extern uint16_t data_to_error(const void *data);

/** Appends an error to the end of an existing buffer. */
extern void append_error_to_buffer(uint16_t error_code, Payload* p);

/** Parses the payload of an Optional Interaction Rejected or Terminate Due To
 * Error message.
 * @param payload is the contents of the incoming addressed message.
 * @param error_code will hold the 2-byte error code, or ERROR_PERMANENT if not
 * specified
 * @param mti will hold the MTI value, or 0 if not specified
 * @param error_message will hold all remaining bytes that came with the error
 * message.
 */
extern void buffer_to_error(const Payload& payload, uint16_t* error_code, uint16_t* mti, string* error_message);

/** A global class / variable for empty or not-yet-initialized payloads. */
extern string EMPTY_PAYLOAD;

/// @return the high 4 bytes of a node ID. @param id is the node ID.
inline unsigned node_high(NodeID id) {
    return id >> 32;
}
/// @return the low 4 bytes of a node ID. @param id is the node ID.
inline unsigned node_low(NodeID id) {
    return id & 0xffffffffU;
}

/// Helper function to send an event report to the bus. Performs
/// synchronous (dynamic) memory allocation so use it sparingly and when
/// there is sufficient amount of RAM available.
/// @param event_id is the event to send off.
extern void send_event(Node* src_node, uint64_t event_id);

/** This class is used in the dispatching of incoming or outgoing NMRAnet
 * messages to the message handlers at the protocol-agnostic level (i.e. not
 * CAN or TCP-specific).
 *
 * TODO(balazs.racz) There shall be one instance of this class that will be
 * sent to all handlers that expressed interest in that MTI. When all those
 * handlers are done, the instance should be freed. Currently the instance is
 * copied by the dispatcher separately for each handler. */
struct GenMessage
{
    GenMessage()
        : src({0, 0}), dst({0, 0}), flagsSrc(0), flagsDst(0) {}

    void clear()
    {
        reset((Defs::MTI)0, 0, EMPTY_PAYLOAD);
    }

    void reset(Defs::MTI mti, NodeID src, NodeHandle dst, string payload)
    {
        this->mti = mti;
        this->src = {src, 0};
        this->dst = dst;
        this->payload = std::move(payload);
        this->dstNode = nullptr;
        this->flagsSrc = 0;
        this->flagsDst = 0;
    }

    void reset(Defs::MTI mti, NodeID src, string payload)
    {
        this->mti = mti;
        this->src = {src, 0};
        this->dst = {0, 0};
        this->payload = std::move(payload);
        this->dstNode = nullptr;
        this->flagsSrc = 0;
        this->flagsDst = 0;
    }

    /// Source node.
    NodeHandle src;
    /// Destination node.
    NodeHandle dst;
    /// OpenLCB MTI of the incoming message.
    Defs::MTI mti;
    /// If the destination node is local, this value is non-NULL.
    Node *dstNode;
    /// Data content in the message body. Owned by the dispatcher.
    /// @todo(balazs.racz) figure out a better container.
    string payload;

    unsigned flagsSrc : 4;
    unsigned flagsDst : 4;
    unsigned get_flags_src() {
        return flagsSrc;
    }
    unsigned get_flags_dst() {
        return flagsDst;
    }
    void set_flag_src(unsigned flags) {
        flagsSrc |= flags;
    }
    void clear_flag_src(unsigned flags) {
        flagsSrc &= ~flags;
    }
    /** Returns true if src flags has all the specified flags set. */
    bool has_flag_src(unsigned flags) {
        return ((flagsSrc & flags) == flags);
    }
    void set_flag_dst(unsigned flags) {
        flagsDst |= flags;
    }
    void clear_flag_dst(unsigned flags) {
        flagsDst &= ~flags;
    }
    /** Returns true if src flags has all the specified flags set. */
    bool has_flag_dst(unsigned flags) {
        return ((flagsDst & flags) == flags);
    }

    typedef uint32_t id_type;
    id_type id() const
    {
        return static_cast<uint32_t>(mti);
    }

    /** Returns the NMRAnet-defined priority band, in the range of 0..3. */
    unsigned priority()
    {
        return Defs::mti_priority(mti);
    }

    enum DstFlags {
        /** Specifies that the stack should wait for the local loopback
         * processing before invoking the done notifiable. */
        WAIT_FOR_LOCAL_LOOPBACK = 1,
        /** Signals to the stack that we need to set the continuation bits in
         * the outgoing message to indicate that this is not the first frame of
         * a message. */
        DSTFLAG_NOT_FIRST_MESSAGE = 2,
        /** Signals to the stack that we need to set the continuation bits in
         * the outgoing message to indicate that this is not the last frame of
         * a message. */
        DSTFLAG_NOT_LAST_MESSAGE  = 4,
        // 8: free
    };
    enum SrcFlags {
        // 1, 2, 4, 8: free
    };
};

/// Interface class for all handlers that can be registered in the dispatcher
/// to receive incoming NMRAnet messages.
typedef FlowInterface<Buffer<GenMessage>> MessageHandler;

/// Abstract class representing an OpenLCB Interface. All interaction between
/// the local software stack and the physical bus has to go through this
/// class. The API that's not specific to the wire protocol appears here. The
/// implementations of this class would be specific to the wire protocol
/// (e.g. IfCan for CAN, and a not-yet-implemented class for TCP).
class If : public Service
{
public:
    /** Constructs an NMRAnet interface.
     * @param executor is the thread that will be used for all processing on
     * this interface.
     * @param local_nodes_count is the maximum number of virtual nodes that
     * this interface will support. */
    If(ExecutorBase *executor, int local_nodes_count);

    /** Destructor */
    virtual ~If()
    {
    }

    /** @return Flow to send global messages to the NMRAnet bus. */
    MessageHandler *global_message_write_flow()
    {
        HASSERT(globalWriteFlow_);
        return globalWriteFlow_;
    }
    /** @return Flow to send addressed messages to the NMRAnet bus. */
    MessageHandler *addressed_message_write_flow()
    {
        HASSERT(addressedWriteFlow_);
        return addressedWriteFlow_;
    }

    /** Type of the dispatcher of incoming NMRAnet messages. */
    typedef DispatchFlow<Buffer<GenMessage>, 4> MessageDispatchFlow;

    /** @return Dispatcher of incoming NMRAnet messages. */
    MessageDispatchFlow *dispatcher()
    {
        return &dispatcher_;
    }

    /** Transfers ownership of a module to the interface. It will be brought
     * down in the destructor. The destruction order is guaranteed such that
     * all supporting structures are still available when the flow is destryed,
     * but incoming messages can not come in anymore.
     *
     * @todo(balazs.racz) revise whether this needs to be virtual. */
    virtual void add_owned_flow(Executable *e) = 0;

    /** Registers a new local node on this interface. This function must be
     * called from the interface's executor.
     *
     * @param node is the node to register.
     */
    void add_local_node(Node *node)
    {
        NodeID id = node->node_id();
        HASSERT(localNodes_.find(id) == localNodes_.end());
        localNodes_[id] = node;
    }

    /** Removes a local node from this interface. This function must be called
     * from the interface's executor.
     *
     * @param node is the node to delete. The node will not be freed, just
     * removed from the data structures.
     */
    virtual void delete_local_node(Node *node) = 0;

    /** Looks up a node ID in the local nodes' registry. This function must be
     * called from the interface's executor.
     *
     * @param id is the 48-bit NMRAnet node ID to look up.
     * @returns the node pointer or NULL if the node is not registered.
     */
    Node *lookup_local_node(NodeID id)
    {
        auto it = localNodes_.find(id);
        if (it == localNodes_.end())
        {
            return nullptr;
        }
        return it->second;
    }

    /** Looks up a node ID in the local nodes' registry. This function must be
     * called from the interface's executor.
     *
     * @param handle is the NodeHandle representing a target node.
     * @returns the node pointer or NULL if the node is not local registered.
     */
    virtual Node *lookup_local_node_handle(NodeHandle handle)
    {
        return lookup_local_node(handle.id);
    }

    /**
     * @returns the first node (by nodeID order) that is registered in this
     * interface as a local node, or nullptr if this interface has no local
     * nodes.
     */
    Node* first_local_node() {
        auto it = localNodes_.begin();
        if (it == localNodes_.end()) return nullptr;
        return it->second;
    }

    /**
     * Iterator helper on the local nodes map.
     *
     * @param previous is the node ID of a valid local node.
     *
     * @returns the node pointer of the next local node (in node ID order) or
     * null if this was the last node or an invalid argument (not the node ID
     * of a local node).
     */
    Node* next_local_node(NodeID previous) {
        auto it = localNodes_.find(previous);
        if (it == localNodes_.end())
        {
            return nullptr;
        }
        ++it;
        if (it == localNodes_.end())
        {
            return nullptr;
        }
        return it->second;
    }

    /** @returns true if the two node handles match as far as we can tell
     * without doing any network traffic. */
    virtual bool matching_node(NodeHandle expected,
                               NodeHandle actual) = 0;


    /** Canonicalizes the node handle: fills in id and/or alias from the maps
     * the interface holds internally. Noop for TCP interface. Must be called
     * on the interface executor. */
    virtual void canonicalize_handle(NodeHandle *h) {}
    
protected:
    void remove_local_node_from_map(Node *node) {
        auto it = localNodes_.find(node->node_id());
        HASSERT(it != localNodes_.end());
        localNodes_.erase(it);
    }

    /// Allocator containing the global write flows.
    MessageHandler *globalWriteFlow_;
    /// Allocator containing the addressed write flows.
    MessageHandler *addressedWriteFlow_;

private:
    /// Flow responsible for routing incoming messages to handlers.
    MessageDispatchFlow dispatcher_;

    typedef Map<NodeID, Node *> VNodeMap;

    /// Local virtual nodes registered on this interface.
    VNodeMap localNodes_;

    friend class VerifyNodeIdHandler;

    DISALLOW_COPY_AND_ASSIGN(If);
};

/// Message handlers that are implemented as state flows should derive from
/// this class.
typedef StateFlow<Buffer<GenMessage>, QList<4>> MessageStateFlowBase;

/** Base class for incoming message handler flows. */
class IncomingMessageStateFlow
    : public MessageStateFlowBase
{
public:
    IncomingMessageStateFlow(If *iface)
        : MessageStateFlowBase(iface)
    {
    }

    If *iface()
    {
        return static_cast<If *>(service());
    }

    /// Returns the NMRAnet message we received.
    GenMessage *nmsg()
    {
        return message()->data();
    }
};

} // namespace openlcb

#endif // _OPENLCB_IF_HXX_
