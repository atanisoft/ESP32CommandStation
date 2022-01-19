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

/// CDI XML Download request metadata used for @ref CDIDownloadHandler.
struct CDIDownloadRequest : public CallableFlowRequestBase
{
    /// Sets up a request to read the CDI from a target node
    /// @param id Destination node to send the request(s) to.
    /// @param field Field in the webpage to respond with.
    /// @param socket WebSocketFlow to send response data to.
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

    /// Webpage field/unique ID to include in the response.
    string field;

    /// Where to send response payload(s) to.
    http::WebSocketFlow *socket;

    /// Number of attempts in loading the CDI data.
    uint8_t attempts;

    /// Unique segment ID for multi-part CDI segment transmission.
    size_t segment;

    /// Current offset within the CDI data.
    unsigned offs;
};

/// Callable flow used to download the CDI XML data from a target node in
/// chunks which are streamed back to the client as a base64 encoded chunk in
/// a json payload.
///
/// Maximum chunk size is 128 bytes and will be truncated at the first null
/// byte which is used as an "end-of-file" marker byte in the CDI XML data.
///
/// Payload format:
///
/// CDI XML payload starting:
/// `{"res":"cdi","reset":true}`
///
/// This indicates that the client should discard any cached or partially
/// downloaded CDI data.
///
/// CDI XML payload chunk:
/// `{"res":"cdi","seg":%zu,"part":"%s"}`
///
/// The `seg` field is an auto-incremented field which can be used by the
/// client to re-assemble the chunks if they arrive out of sequence.
/// The `part` field is a base64 encoded chunk of the CDI XML data.
///
/// CDI XML download failure:
/// `{"res":"cdi","error":"Failed to download CDI: %d"}`
///
/// The `error` field will contain the numeric error code which was received as
/// part of the CDI XML download. This typically will indicate the node
/// rejected the request or a timeout occurred.
///
/// CDI XML download completed:
/// `{"res":"cdi","done":true,"tgt":"%s",%s}`
///
/// The `tgt` field is an identifier provided by the client and is passed back
/// as-is.
/// Additional fields provided in the response: 
/// * node_id - Node ID that the CDI belongs to.
/// * has_snip - Indicates that the node responded to a SNIP request.
/// * has_cdi - Indicates that the node advertised support for CDI requests.
/// * has_fdi - Indicates that the node advertised support for FDI requests.
/// * is_train - Indicates that this node is a train or not.
/// NOTE: The `has_fdi` and `is_train` fields will always be false for the
/// ESP32CS generated payloads.
class CDIDownloadHandler : public CallableFlow<CDIDownloadRequest>
{
public:
    /// Constructor.
    ///
    /// @param service @ref Service to attach this flow to.
    /// @param node @ref Node that is running this flow.
    /// @param memcfg @ref MemoryConfigHandler to use for all requests.
    CDIDownloadHandler(Service *service, openlcb::Node *node,
                       openlcb::MemoryConfigHandler *memcfg);

private:
    /// Memory config client to use for all CDI requests.
    openlcb::MemoryConfigClient client_;

    /// Displayable representation of the target node ID.
    string targetNodeId_;

    /// Maximum segment size used for downloading a CDI chunk.
    static constexpr unsigned CDI_DOWNLOAD_SEGMENT_SIZE = 128;

    /// Maximum number of attempts for download a CDI chunk.
    static constexpr uint8_t MAX_ATTEMPTS = 5;

    /// Payload sent to the client when the download has started.
    static constexpr const char * const DOWNLOAD_START =
        R"!^!({"res":"cdi","reset":true})!^!";

    /// Payload template sent to the client for a segment of the CDI payload.
    static constexpr const char * const STREAM_SEGMENT_PART =
        R"!^!({"res":"cdi","seg":%zu,"part":"%s"})!^!";

    /// Payload template sent to the client when a download fails.
    static constexpr const char * const DOWNLOAD_FAILED_ERR =
        R"!^!({"res":"cdi","error":"Failed to download CDI: %d"})!^!";

    /// Payload template sent to the client when the download is complete.
    static constexpr const char * const DOWNLOAD_COMPLETE =
        R"!^!({"res":"cdi","done":true,"tgt":"%s",%s})!^!";

    /// Main entry point that initiates the download process.
    Action entry() override;

    /// State flow step that initiates the download of a single segment.
    Action download_segment();

    /// State flow step called when a segment download completes.
    Action segment_complete();
};

#endif // CDI_DOWNLOADER_HXX_