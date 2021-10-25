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

#include "StringUtils.hxx"

#include <Httpd.h>
#include <HttpStringUtils.h>
#include <openlcb/MemoryConfigClient.hxx>

#ifndef CDI_DOWNLOADER_HXX_
#define CDI_DOWNLOADER_HXX_

struct CDIDownloadRequest : public CallableFlowRequestBase
{
    /// Sets up a request to read the SNIP from a target node
    /// @param id is the destination node to query
    /// @param field is the field in the webpage to respond with.
    void reset(openlcb::NodeID id, string field, http::WebSocketFlow *socket)
    {
        reset_base();
        this->target = openlcb::NodeHandle(id);
        this->field = field;
        this->socket = socket;
        this->attempts = 0;
        this->segment = 1;
        this->offs = 0;
    }
    /// Node to send the request to.
    openlcb::NodeHandle target;
    string field;
    http::WebSocketFlow *socket;
    uint8_t attempts;
    size_t segment;
    unsigned offs;
};

class CDIDownloadHandler : public CallableFlow<CDIDownloadRequest>
{
public:
    CDIDownloadHandler(Service *service, openlcb::Node *node,
                       openlcb::MemoryConfigHandler *memcfg);

private:
    openlcb::MemoryConfigClient client_;
    string tgt_;
    static constexpr unsigned CHUNK_SIZE = 128;
    static constexpr uint8_t MAX_ATTEMPTS = 5;

    static constexpr const char * const DOWNLOAD_START =
        R"!^!({"res":"cdi","reset":true})!^!";

    static constexpr const char * const STREAM_SEGMENT_PART =
        R"!^!({"res":"cdi","seg":%zu,"part":"%s"})!^!";

    static constexpr const char * const DOWNLOAD_FAILED_ERR =
        R"!^!({"res":"cdi","error":"Failed to download CDI: %d"})!^!";

    static constexpr const char * const DOWNLOAD_COMPLETE =
        R"!^!({"res":"cdi","done":true,"tgt":"%s",%s})!^!";

    Action entry() override;

    Action download_chunk();
    Action chunk_complete();
};

#endif // CDI_DOWNLOADER_HXX_