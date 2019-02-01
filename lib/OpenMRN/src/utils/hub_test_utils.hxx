#include "utils/test_main.hxx"
#include "utils/socket_listener.hxx"
#include "utils/Hub.hxx"
#include "utils/HubDeviceSelect.hxx"
#include "utils/HubDevice.hxx"

/// Test payload for the hub unittest.
struct TestData
{
    /// Where this messsage was coming from (identifier or hub source)
    int from;
    /// Something like a TTL, will be decremented by the default test-hub
    /// handlers.
    int payload;
};

typedef HubContainer<StructContainer<TestData>> TestHubData;
typedef FlowInterface<Buffer<TestHubData>> TestHubPortInterface;
typedef StateFlow<Buffer<TestHubData>, QList<1>> TestHubPort;
typedef GenericHubFlow<TestHubData> TestHubFlow;
typedef HubDeviceSelect<TestHubFlow> TestHubDeviceAsync;
