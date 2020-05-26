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
 * \file ClientConnection.hxx
 *
 * Utilities for managing can-hub connections as a client application.
 *
 * @author Balazs Racz
 * @date 30 Mar 2016
 */

#ifndef _UTILS_CLIENTCONNECTION_HXX_
#define _UTILS_CLIENTCONNECTION_HXX_

#include "utils/GridConnectHub.hxx"
#include <stdio.h>
#include <termios.h> /* tc* functions */
#include <unistd.h>

/// Abstract base class for the Hub's connections.
class ConnectionClient
{
public:
    /** Test the connection whether it is alive; establish the connection if it
     * is dead. @returns true if there is a live connection. */
    virtual bool ping() = 0;

    virtual ~ConnectionClient()
    {
    }
};

/// Notification implementation that sets an external variable to -1 when
/// notified. Supplied as an on-error callback to the gridconnect port creation
/// method.
class DeviceClosedNotify : public Notifiable
{
public:
    /// Constructor.
    ///
    /// @param fd pointer to file descriptor variable that will be set to -1
    /// upon error notification.
    /// @param name Info string to print to stderr when the device error causes
    /// a close event.
    DeviceClosedNotify(int *fd, string name)
        : fd_(fd)
        , name_(name)
    {
    }
    /// Callback from the application when an error is encountered.
    void notify() override
    {
        LOG_ERROR("Connection to %s closed.", name_.c_str());
        *fd_ = -1;
    }

private:
    int *fd_; ///< pointer to file descriptor variable.
    string name_; ///< info to print to stderr upon error.
};

/// Base class for FD-based GridConnect connection clients.
class GCFdConnectionClient : public ConnectionClient
{
public:
    /// Constructor.
    ///
    /// @param name user-readable name for this port.
    /// @param hub CAN packet hub to connect this port to
    GCFdConnectionClient(const string &name, CanHubFlow *hub)
        : closedNotify_(&fd_, name)
        , hub_(hub)
    {
    }

    virtual ~GCFdConnectionClient()
    {
    }

    /// Tests if the device is alive or encountered an error. Retries
    /// connection if an error happens. @return true if the connection is live.
    bool ping() OVERRIDE
    {
        if (fd_ < 0)
        {
            try_connect();
        }
        return fd_ >= 0;
    }

protected:
    /// Abstrct base function to attempt to connect (or open device) to the
    /// destination.
    virtual void try_connect() = 0;

    /** Callback from try_connect to donate the file descriptor. @param fd is
     * the file destriptor of the connection freshly opened.  */
    void connection_complete(int fd)
    {
        fd_ = fd;
        create_gc_port_for_can_hub(hub_, fd, &closedNotify_);
    }

private:
    /// Will be called when the descriptor experiences an error (typivcally
    /// upon device closed or connection lost).
    DeviceClosedNotify closedNotify_;
    /// Filedes of the currently open device/socket.
    int fd_{-1};
    /// CAN hub to read-write data to.
    CanHubFlow *hub_;
};

/// Connection client that opens a character device (such as an usb-serial) and
/// sets the termios attributes as appropriate for linux and mac on a
/// serial-to-can or usb-to-can controller.
class DeviceConnectionClient : public GCFdConnectionClient
{
public:
    /// Constructor.
    ///
    /// @param name user-readable name of this device
    /// @param hub CAN packet hub to connect this device to
    /// @param dev filename of the device node to open
    DeviceConnectionClient(
        const string &name, CanHubFlow *hub, const string &dev)
        : GCFdConnectionClient(name, hub)
        , dev_(dev)
    {
    }

    virtual ~DeviceConnectionClient()
    {
    }

private:
    /// Attempts to open the device.
    void try_connect() OVERRIDE
    {
        int fd = ::open(dev_.c_str(), O_RDWR);
        if (fd >= 0)
        {
            // Sets up the terminal in raw mode. Otherwise linux might echo
            // characters coming in from the device and that will make
            // packets go back to where they came from.
            HASSERT(!tcflush(fd, TCIOFLUSH));
            struct termios settings;
            HASSERT(!tcgetattr(fd, &settings));
            cfmakeraw(&settings);
            cfsetspeed(&settings, B115200);
            HASSERT(!tcsetattr(fd, TCSANOW, &settings));
            LOG(INFO, "Opened device %s.\n", dev_.c_str());
            connection_complete(fd);
            //
        }
        else
        {
            LOG_ERROR("Failed to open device %s: %s\n", dev_.c_str(),
                strerror(errno));
        }
    }

    string dev_; ///< filename of device to open
};

/// Connection client that connects to an upstream GridConnect-TCP hub via TCP
/// client socket.
class UpstreamConnectionClient : public GCFdConnectionClient
{
public:
    /// Constructor.
    ///
    /// @param name user-readable name that will be printed upon an error.
    /// @param hub CAN hub to connect device to
    /// @param host where to connect to
    /// @param port where to connect to
    UpstreamConnectionClient(
        const string &name, CanHubFlow *hub, const string &host, int port)
        : GCFdConnectionClient(name, hub)
        , host_(host)
        , port_(port)
    {
    }

    virtual ~UpstreamConnectionClient()
    {
    }

private:
    /// Implementation of connection method.
    void try_connect() OVERRIDE
    {
        int fd = ConnectSocket(host_.c_str(), port_);
        if (fd >= 0)
        {
            LOG_ERROR("Connected to %s:%d\n", host_.c_str(), port_);
            connection_complete(fd);
            // create_gc_port_for_can_hub(&can_hub0, fd, &closedNotify_);
        }
        else
        {
            LOG_ERROR("Failed to connect to %s:%d: (%d) %s\n", host_.c_str(),
                port_, errno, strerror(errno));
        }
    }

    /// where to connect to (host name)
    string host_;
    /// where to connect to (TCP port number)
    int port_;
};

#endif // _UTILS_CLIENTCONNECTION_HXX_
