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
 * \file HubDevice.hxx
 * Components for Hubs to connect to physical devices.
 *
 * @author Balazs Racz
 * @date 26 Apr 2014
 */

#ifndef _UTILS_HUBDEVICESELECT_HXX_
#define _UTILS_HUBDEVICESELECT_HXX_

#include "openmrn_features.h"

#ifndef OPENMRN_FEATURE_EXECUTOR_SELECT
#error OS does not have implementation for Executor::select, cannot compile HubDeviceSelect.
#endif

#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>

#include "executor/StateFlow.hxx"
#include "utils/Hub.hxx"

/// Generic template for the buffer traits. HubDeviceSelect will not compile on
/// this default template because it lacks the necessary definitions. For each
/// hub type there must be a partial template specialization of this class.
template <class BType> struct SelectBufferInfo
{
};

/// Partial template specialization of buffer traits for string-typed hubs.
template <> struct SelectBufferInfo<HubFlow::buffer_type>
{
    /// Preps a buffer for receiving data. @param b is the buffer to prep.
    static void resize_target(HubFlow::buffer_type *b)
    {
        b->data()->resize(64);
    }
    /// Clears out all potential empty space left after a buffer has been
    /// partially filled. @param b is the buffer, @param remaining is how many
    /// bytes we did not fill.
    static void check_target_size(HubFlow::buffer_type *b, int remaining)
    {
        HASSERT(remaining >= 0);
        HASSERT(remaining <= 64);
        b->data()->resize(64 - remaining);
    }
    /// @return false because we can deal with a partial read.
    static bool needs_read_fully()
    {
        return false;
    }
};

/// Partial template specialization of buffer traits for struct-typed hubs.
template <class T>
struct SelectBufferInfo<Buffer<HubContainer<StructContainer<T>>>>
{
    /// Helper type for declaring the payload buffer type.
    typedef Buffer<HubContainer<StructContainer<T>>> buffer_type;

    /// struct buffers do not need to be resized.
    static void resize_target(buffer_type *b)
    {
    }
    /// a struct buffer is only okay if the entire buffer was read in in one
    /// go. @param b is the filled buffer, @param remaining tells how many
    /// bytes are empty.
    static void check_target_size(buffer_type *b, int remaining)
    {
        HASSERT(remaining == 0);
    }
    /// @return true because struct buffers always need to be completely read
    /// before forwarding to lower levels.
    static bool needs_read_fully()
    {
        return true;
    }
};

/// Partial template specialization of buffer traits for CAN frame-typed
/// hubs. The implementation here is equivalent to
/// SelectBufferInfo<Hubcontainer<StructContainer<T>>> but the c++ template
/// inference cannot figure this out.
template <>
struct SelectBufferInfo<Buffer<CanHubData>> {
    /// Helper type for declaring the payload buffer type.
    typedef Buffer<CanHubData> buffer_type;
    
    /// CAN buffers do not need to be resized.
    static void resize_target(buffer_type *b)
    {
    }
    /// a CAN buffer is only okay if the entire buffer was read in in one
    /// go. @param b is the filled buffer, @param remaining tells how many
    /// bytes are empty.
    static void check_target_size(buffer_type *b, int remaining)
    {
        HASSERT(remaining == 0);
    }
    /// @return true because CAN buffers always need to be completely read
    /// before forwarding to lower levels.
    static bool needs_read_fully()
    {
        return true;
    }
};

/// State flow implementing select-aware fd reads.
template <class HFlow> class HubDeviceSelectReadFlow : public StateFlowBase
{
public:
    /// Buffer type.
    typedef typename HFlow::buffer_type buffer_type;

    /// Constructor.
    ///
    /// @param device parent object.
    HubDeviceSelectReadFlow(FdHubPortService *device,
        typename HFlow::port_type *dst, typename HFlow::port_type *skip_member)
        : StateFlowBase(device)
        , b_(nullptr)
        , dst_(dst)
        , skipMember_(skip_member)
    {
        this->start_flow(STATE(allocate_buffer));
    }

    /// Unregisters the current flow from the hub.
    void shutdown()
    {
        auto *e = this->service()->executor();
        if (e->is_selected(&selectHelper_))
        {
            e->unselect(&selectHelper_);
        }
        set_terminated();
        notify_barrier();
    }

    /// @return the parent object.
    FdHubPortService *device()
    {
        return static_cast<FdHubPortService *>(this->service());
    }

    /// Allocates a new buffer for incoming data. @return next state.
    Action allocate_buffer()
    {
        return this->allocate_and_call(dst_, STATE(try_read));
    }

    /// Attempts to read into the current buffer from the target
    /// fd. @return next state.
    Action try_read()
    {
        b_ = this->get_allocation_result(dst_);
        b_->data()->skipMember_ = skipMember_;
        SelectBufferInfo<buffer_type>::resize_target(b_);
        if (SelectBufferInfo<buffer_type>::needs_read_fully())
        {
            return this->read_repeated(&selectHelper_, device()->fd(),
                (void *)b_->data()->data(), b_->data()->size(),
                STATE(read_done), 0);
        }
        else
        {
            return this->read_single(&selectHelper_, device()->fd(),
                (void *)b_->data()->data(), b_->data()->size(),
                STATE(read_done), 0);
        }
    }

    /// Called when the stateflow read call(s) are completed.
    /// @return next state.
    Action read_done()
    {
        if (selectHelper_.hasError_)
        {
            /// Error reading the socket.
            b_->unref();
            notify_barrier();
            set_terminated();
            device()->report_read_error();
            return exit();
        }
        SelectBufferInfo<buffer_type>::check_target_size(
            b_, selectHelper_.remaining_);
        dst_->send(b_, 0);
        b_ = nullptr;
        return this->call_immediately(STATE(allocate_buffer));
    }

private:
    /** Calls into the parent flow's barrier notify, but makes sure to
     * only do this once in the lifetime of *this. */
    void notify_barrier()
    {
        if (barrierOwned_)
        {
            barrierOwned_ = false;
            device()->barrier_.notify();
        }
    }

    /// true iff pending parent->barrier_.notify()
    bool barrierOwned_{true};
    /// Helper object for read/write FD asynchronously.
    StateFlowSelectHelper selectHelper_{this};
    /// Buffer that we are currently filling.
    buffer_type *b_;
    /// Where do we forward the messages we created.
    typename HFlow::port_type *dst_;
    /// What should be the source port designation.
    typename HFlow::port_type *skipMember_;
};

/// HubPort that connects a select-aware device to a strongly typed Hub.
///
/// The device is given by either the path to the device or the fd to an opened
/// device instance or socket. The device will be put to nonblocking mode and
/// all processing will be performed in the executor of the hub, by using
/// ExecutorBase::select(). No additional threads are started.
///
/// Reads and writes will be performed in the units defined by the type of the
/// hub: for string-typed hubs in 64 bytes units; for hubs of specific
/// structures (such as CAN frame, dcc Packets or dcc Feedback structures) in
/// the units ofthe size of the structure.
template <class HFlow, class ReadFlow = HubDeviceSelectReadFlow<HFlow>>
class HubDeviceSelect : public FdHubPortService, private Atomic
{
public:
#ifndef __WINNT__
    /// Creates a select-aware hub port for the device specified by `path'.
    HubDeviceSelect(
        HFlow *hub, const char *path, Notifiable *on_error = nullptr)
        : FdHubPortService(
              hub->service()->executor(), ::open(path, O_RDWR | O_NONBLOCK))
        , hub_(hub)
        , readFlow_(this, hub, &writeFlow_)
        , writeFlow_(this)
    {
        HASSERT(fd_ >= 0);
        barrier_.reset(
            on_error ? on_error : EmptyNotifiable::DefaultInstance());
        barrier_.new_child();
        hub_->register_port(write_port());
    }
#endif

    /// Creates a select-aware hub port for the opened device specified by
    /// `fd'. It can be a hardware device, socket or pipe.
    ///
    /// @param hub the hub to open the port on
    /// @param fd the filedes to read/write data from/to.
    /// @param on_error notifiable that will be called when a write or read
    /// error is encountered.
    HubDeviceSelect(HFlow *hub, int fd, Notifiable *on_error = nullptr)
        : FdHubPortService(hub->service()->executor(), fd)
        , hub_(hub)
        , readFlow_(this, hub, &writeFlow_)
        , writeFlow_(this)
    {
        HASSERT(fd_ >= 0);
        barrier_.reset(
            on_error ? on_error : EmptyNotifiable::DefaultInstance());
        barrier_.new_child();
#ifdef __WINNT__
        unsigned long par = 1;
        ioctlsocket(fd_, FIONBIO, &par);
#else
        ::fcntl(fd, F_SETFL, O_RDWR | O_NONBLOCK);
#endif
        hub_->register_port(write_port());
    }

    /// If the barrier has not been called yet, will notify it inline.
    virtual ~HubDeviceSelect()
    {
        if (fd_ >= 0) {
            unregister_write_port();
            int fd = -1;
            executor()->sync_run([this, &fd]()
                                 {
                                     fd = fd_;
                                     fd_ = -1;
                                     readFlow_.shutdown();
                                     writeFlow_.shutdown();
                                 });
            ::close(fd);
            bool completed = false;
            while (!completed) {
                executor()->sync_run([this, &completed]()
                                 {
                                     if (barrier_.is_done()) completed = true;
                                 });
            }
        }
    }

    /// @return parent hub flow.
    HFlow *hub()
    {
        return hub_;
    }

    /// @return the write flow belonging to this device.
    typename HFlow::port_type *write_port()
    {
        return &writeFlow_;
    }

    /// Removes the current write port from the registry of the source hub.
    void unregister_write_port()
    {
        LOG(VERBOSE, "HubDeviceSelect::unregister write port %p %p",
            write_port(), &writeFlow_);
        hub_->unregister_port(&writeFlow_);
        /* We put an empty message at the end of the queue. This will cause
         * wait until all pending messages are dealt with, and then ping the
         * barrier notifiable, commencing the shutdown. */
        auto *b = writeFlow_.alloc();
        b->set_done(&barrier_);
        writeFlow_.send(b);
    }

    /// @return true if there is no pending data to write. Can be used to check
    /// safe destruction.
    bool write_done()
    {
        return writeFlow_.is_waiting();
    }

protected:
    /// Base stateflow for the WriteFlow.
    typedef StateFlow<typename HFlow::buffer_type, QList<1>> WriteFlowBase;
    /// State flow implementing select-aware fd writes.
    class WriteFlow : public WriteFlowBase
    {
    public:
        /// Constructor. @param dev is the parent object.
        WriteFlow(HubDeviceSelect *dev)
            : WriteFlowBase(dev)
        {
        }

        /// Destructor.
        ~WriteFlow()
        {
            HASSERT(this->is_waiting());
        }

        /// Unregisters this object from the flows.
        void shutdown()
        {
            // The fd must be set to negative already to ensure the shutdown
            // completes successfully.
            HASSERT(device()->fd() < 0);
            auto* e = this->service()->executor();
            if (!selectHelper_.is_empty() && e->is_selected(&selectHelper_)) {
                e->unselect(&selectHelper_);
                // will make the internal_try_write exit immediately
                selectHelper_.remaining_ = 0;
                // actually wake up the flow
                this->notify();
            }
        }

        /// @return parent object.
        HubDeviceSelect *device()
        {
            return static_cast<HubDeviceSelect *>(this->service());
        }

        StateFlowBase::Action entry() OVERRIDE
        {
            if (device()->fd() < 0) {
                return this->release_and_exit();
            }
            return this->write_repeated(&selectHelper_, device()->fd(),
                this->message()->data()->data(),
                this->message()->data()->size(), STATE(write_done),
                this->priority());
        }

        /// State flow call. @return next state.
        StateFlowBase::Action write_done()
        {
            if (selectHelper_.hasError_) {
                device()->report_write_error();
            }
            return this->release_and_exit();
        }

    private:
        /// Helper class for asynchronous writes.
        StateFlowBase::StateFlowSelectHelper selectHelper_{this};
    };

protected:

    /** The assumption here is that the write flow still has entries in its
     * queue that need to be removed. */
    void report_write_error() override
    {
        readFlow_.shutdown();
        unregister_write_port();
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    /** Callback from the ReadFlow when the read call has seen an error. The
     * read count will already have been taken out of the barrier, and the read
     * flow in terminated state. */
    void report_read_error() override
    {
        unregister_write_port();
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    /// Hub whose data we are trying to send.
    HFlow *hub_;
    /// StateFlow for reading data from the fd. Woken when data arrives.
    ReadFlow readFlow_;
    /// StateFlow for writing data to the fd. Woken by data to send or the fd
    /// being writeable.
    WriteFlow writeFlow_;
};

#endif // _UTILS_HUBDEVICESELECT_HXX_
