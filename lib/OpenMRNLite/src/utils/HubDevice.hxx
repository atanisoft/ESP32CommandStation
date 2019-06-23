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

#ifndef _UTILS_HUBDEVICE_HXX_
#define _UTILS_HUBDEVICE_HXX_

#include <unistd.h>

#include "openmrn_features.h"
#include "utils/Hub.hxx"
#include "executor/SemaphoreNotifiableBlock.hxx"

template <class Data> class FdHubWriteFlow;

/// Template-nonspecific base class for @ref FdHubPort. The purpose of this
/// class is to avoid compiling this code multiple times for differently typed
/// devices (and thus saving flash space).
class FdHubPortBase : public FdHubPortInterface, private Atomic
{
public:
#ifdef ESP32 // TODO: shrink these if possible
    /// How many bytes of stack should we allocate to the write thread's stack.
    static const int kWriteThreadStackSize = 2048;
    /// How many bytes of stack should we allocate to the read thread's stack.
    static const int kReadThreadStackSize = 2048;
#else
    /// How many bytes of stack should we allocate to the write thread's stack.
    static const int kWriteThreadStackSize = 1000;
    /// How many bytes of stack should we allocate to the read thread's stack.
    static const int kReadThreadStackSize = 1000;
#endif // ESP32

    /// Constructor.
    /// @param fd is the filedes to read/write
    /// @param done will be called when this file is closed and removed from
    /// the hub (usually due to an error).
    FdHubPortBase(int fd, Notifiable *done)
        : FdHubPortInterface(fd)
        , writeThread_(fill_thread_name('W', fd), 3, kWriteThreadStackSize)
        , writeService_(&writeThread_)
        , barrier_(done)
        , hasError_(0)
        , writeExitEnqueued_(0)
    {
        barrier_.new_child();
    }

    virtual ~FdHubPortBase()
    {
    }

    /// Puts the desired thread name for the read or write thread.
    ///
    /// @param buf where to put ocmputed thread name. Must be at least 15 chars
    /// long.
    /// @param mode a character describing read ('R') or write ('W')
    /// @param fd filedes number that will be rendered into the thread name.
    ///
    static void fill_thread_name(char *buf, char mode, int fd);

protected:
    /// Puts the desired thread name for the read or write thread.
    ///
    /// @param mode a character describing read ('R') or write ('W')
    /// @param fd filedes number that will be rendered into the thread name.
    /// @return the thread name buffer.
    const char *fill_thread_name(char mode, int fd) {
        fill_thread_name(threadName_, mode, fd);
        return threadName_;
    }

    /** Removes the write flow from the hub's registration. Triggers the write
     * flow to call barrier_ when all the pending queue entries are
     * released. */
    virtual void unregister_write_port() = 0;

    /// Call when an IO error is encountered. Closes the FD, unregisters the
    /// port from the hub and causes the threads to exit.
    void report_error()
    {
        {
            AtomicHolder h(this);
            if (hasError_)
            {
                return;
            }
            else
            {
                hasError_ = 1;
            }
        }
        ::close(fd_);
        unregister_write_port();
    }

    /// Read thread implementation with template-inspecific methods.
    class ReadThreadBase : public OSThread
    {
    public:
        /// Constructor. @param port is the parent flow.
        ReadThreadBase(FdHubPortBase *port) : port_(port)
        {
        }

        /** @return the minimum number of bytes that we will send. */
        virtual int unit() = 0;
        /** @return we will allocate this many bytes for read buffer. This is
         * the maximum number of bytes that we'll send. */
        virtual int buf_size() = 0;
        /** Sends off a buffer */
        virtual void send_message(const void *buf, int size) = 0;
        void *entry() OVERRIDE
        {
            AutoNotify bn(&port_->barrier_);
            uint8_t buf[buf_size()];
            while (true)
            {
                int done = 0;
                while (done < unit())
                {
                    {
                        AtomicHolder h(port_);
                        if (port_->hasError_)
                        {
                            return NULL;
                        }
                    }
                    ssize_t ret =
                        ::read(port_->fd_, &buf[done], buf_size() - done);
                    if (ret > 0)
                    {
                        done += ret;
                        continue;
                    }
                    if ((ret < 0) && (errno == EINTR || errno == EAGAIN))
                    {
                        continue;
                    }
// Now: we have an error.
#if OPENMRN_FEATURE_BSD_SOCKETS_REPORT_EOF_ERROR
                    if (!ret)
                    {
                        LOG_ERROR("EOF reading fd %d", port_->fd_);
                    }
                    else
                    {
                        LOG_ERROR("Error reading fd %d: (%d) %s", port_->fd_,
                            errno, strerror(errno));
                    }
#endif // OPENMRN_FEATURE_BSD_SOCKETS_REPORT_EOF_ERROR
                    port_->report_error();
                    return NULL;
                }
                send_message(buf, done);
            }
        }

    protected:
        /// Parent port.
        FdHubPortBase *port_;
    };

protected:
    template <class Data> friend class FdHubWriteFlow;

    /** Temporary buffer used for rendering thread names. */
    char threadName_[30];
    /** This executor is running the writes. */
    Executor<1> writeThread_;
    /** Service for the write flow. */
    Service writeService_;
    /** This barrier will be notified when both read and write thread has
     * exited. */
    BarrierNotifiable barrier_;
    /** If this is 1, the fd has been closed. */
    unsigned hasError_ : 1;
    /** If this is 1, we have already enqueued the request to exit the write
     * flow. */
    unsigned writeExitEnqueued_ : 1;
};

/// State flow for writing data to an fd. This flow performs synchronous
/// writes, thus must be run on its own executor (and must never be run on the
/// shared executor used by the stack).
template <class Data>
class FdHubWriteFlow : public StateFlow<Buffer<Data>, QList<1>>
{
public:
    /// Constructor. @param parent is the owning port.
    FdHubWriteFlow(FdHubPortBase *parent)
        : StateFlow<Buffer<Data>, QList<1>>(&parent->writeService_)
        , port_(parent)
    {
    }

    /// Handles the next incoming entry. @return next action
    StateFlowBase::Action entry() OVERRIDE
    {
        const uint8_t *buf =
            reinterpret_cast<const uint8_t *>(this->message()->data()->data());
        size_t size = this->message()->data()->size();
        while (size > 0)
        {
            {
                AtomicHolder h(port_);
                if (port_->hasError_)
                {
                    if (port_->writeExitEnqueued_ && this->queue_empty())
                    {
                        StateFlowBase::Action a = this->set_terminated();
                        this->release();
                        return a;
                    }
                    return this->release_and_exit();
                }
            }
            ssize_t ret = ::write(port_->fd_, buf, size);
            if (ret > 0)
            {
                size -= ret;
                buf += ret;
                continue;
            }
// now: we have an error.
#if OPENMRN_FEATURE_BSD_SOCKETS_REPORT_EOF_ERROR
            if (!ret)
            {
                LOG_ERROR("EOF writing fd %d", port_->fd_);
            }
            else
            {
                LOG_ERROR("Error writing fd %d: (%d) %s", port_->fd_, errno,
                    strerror(errno));
            }
#endif // OPENMRN_FEATURE_BSD_SOCKETS_REPORT_EOF_ERROR
            port_->report_error();
            break;
        }
        return this->release_and_exit();
    }

    /// The owning port.
    FdHubPortBase *port_;
};

/// HubPort that connects a raw device to a strongly typed Hub.
///
/// The device is given by the fd to an opened device instance (or socket,
/// pipe, etc). Starts two additional threads: one for reading, one for
/// writing.
///
/// Reads and writes will be performed in the units defined by the type of the
/// hub: for string-typed hubs in 64 bytes units; for hubs of specific
/// structures (such as CAN frame, dcc Packets or dcc Feedback structures) in
/// the units of the size of the structure.
template <class HFlow> class FdHubPort : public FdHubPortBase
{
public:
    /// Constructor.
    ///
    /// @param hub Parent hub where to register *this.
    /// @param fd file descriptor to read/write
    /// @param done will be notified when the termination of the port is
    /// completed.
    FdHubPort(HFlow *hub, int fd, Notifiable *done)
        : FdHubPortBase(fd, done)
        , hub_(hub)
        , writeFlow_(this)
        , readThread_(this)
    {
        hub_->register_port(&writeFlow_);
    }

    ~FdHubPort() OVERRIDE
    {
        writeThread_.shutdown();
    }

    void unregister_write_port() OVERRIDE
    {
        hub_->unregister_port(&writeFlow_);
        /* We put an empty message at the end of the queue. This will cause
         * wait until all pending messages are dealt with, and then ping the
         * barrier notifiable, commencing the shutdown. */
        auto *b = writeFlow_.alloc();
        b->set_done(&barrier_);
        writeFlow_.send(b);
    }

    /// Thread performing the read operations on the device.
    class ReadThread : public ReadThreadBase
    {
    public:
        /// Constructor. @param port is the parent flow.
        ReadThread(FdHubPort<HFlow> *port) : ReadThreadBase(port)
        {
            init();
            start(port->fill_thread_name('R', port->fd_), 0,
                  port->kReadThreadStackSize);
        }
        
        ~ReadThread() {
            delete semaphores_;
        }

        /// @return the parent flow
        FdHubPort<HFlow> *port()
        {
            return static_cast<FdHubPort<HFlow> *>(port_);
        }

        /** @param this is the minimum number of bytes that we will send. */
        int unit() OVERRIDE
        {
            return kUnit;
        }
        /** @return We will allocate this many bytes for read buffer. This is
         * the maximum number of bytes that we'll send. */
        int buf_size() OVERRIDE
        {
            return kBufSize;
        }
        /** Sends off a buffer */
        void send_message(const void *buf, int size) OVERRIDE;

    private:
        /// Initializes the semaphore notifiables.
        void init();
        /// If non-null, one slot will be acquired for each incoming message.
        SemaphoreNotifiableBlock* semaphores_{nullptr};

        /** This is the minimum number of bytes that we will send. */
        static const int kUnit;
        /** We will allocate this many bytes for read buffer. This is the
         * maximum number of bytes that we'll send. */
        static const int kBufSize;
    };

private:
    friend class ReadThread;

    /// Parent hub to send the data to / read the data from.
    HFlow *hub_;
    /// StateFlow that is performing the actual writes.
    FdHubWriteFlow<typename HFlow::value_type> writeFlow_;
    /// An OSThread child that is performing the reads.
    ReadThread readThread_;
};

#endif // _UTILS_HUBDEVICE_HXX_
