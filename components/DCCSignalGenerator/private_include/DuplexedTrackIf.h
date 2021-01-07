/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019-2021 Mike Dunston

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses
**********************************************************************/

#ifndef DUPLEXED_TRACK_IF_H_
#define DUPLEXED_TRACK_IF_H_

#include <dcc/Packet.hxx>
#include <executor/Executor.hxx>
#include <executor/Service.hxx>
#include <executor/StateFlow.hxx>
#include <utils/Buffer.hxx>
#include <utils/Queue.hxx>

namespace esp32cs
{

/// StateFlow that accepts dcc::Packet structures and sends them to a local
/// device driver for producing the track signal.
///
/// The device driver must support the notifiable-based asynchronous write
/// model.
class DuplexedTrackIf : public StateFlow<Buffer<dcc::Packet>, QList<1>>
{
public:
    /// Constructor.
    ///
    /// Creates a TrackInterface from an fd to the mainline and an fd for prog.
    ///
    /// This class currently does synchronous writes to the device. In order not
    /// to block the executor, you have to create a new threadexecutor.
    ///
    /// @param service THE EXECUTOR OF THIS SERVICE WILL BE BLOCKED.
    /// @param pool_size will determine how many packets the current flow's
    /// alloc() will have.
    /// @param ops is the name of the OPS track.
    /// @param prog is the name of the PROG track.
    /// @param track_base_path is the base path for track drivers.
    DuplexedTrackIf(Service *service, size_t pool_size, const char *ops
                  , const char *prog, const char *track_base_path);

    /// Destructor.
    ~DuplexedTrackIf();

    /// @return the @ref FixedPool for dcc::Packet objects to send to the track.
    FixedPool *pool() override;

protected:
    /// Sends a queued packet to either the OPS or PROG track.
    ///
    /// Track selection is made based on the DCC header flag for a longer
    /// preamble which is only used for PROG track packets.
    ///
    /// If the packet can not be written to the file descriptor it will be held
    /// until the device driver alerts that it is ready for another packet.
    ///
    /// Note: Both OPS and PROG packets will be processed by this method and if
    /// either device driver prevents the write operation both tracks will be
    /// blocked.
    Action entry() override;

    /// File descriptor for the OPS track output.
    int fd_ops_;

    /// File descriptor for the PROG track output.
    int fd_prog_;

    /// Packet pool from which to allocate packets.
    FixedPool pool_;
};

} // namespace esp32cs

#endif // DUPLEXED_TRACK_IF_H_