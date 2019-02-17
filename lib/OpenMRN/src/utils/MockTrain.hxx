#ifndef _UTILS_MOCKTRAIN_HXX_
#define _UTILS_MOCKTRAIN_HXX_

#include "gmock/gmock.h"
#include "openlcb/TractionDefs.hxx"
#include "openlcb/TractionTrain.hxx"
#include "dcc/Defs.hxx"

namespace openlcb {

/// Test helper class for TrainImpl using GoogleMock. Allows creating a train
/// node without an implementation. All calls have to have an explicit
/// expectation using the GoogleMock framework.
class MockTrain : public TrainImpl
{
public:
    MOCK_METHOD1(set_speed, void(SpeedType speed));
    MOCK_METHOD0(get_speed, SpeedType());
    MOCK_METHOD0(get_commanded_speed, SpeedType());
    MOCK_METHOD0(get_actual_speed, SpeedType());
    MOCK_METHOD0(set_emergencystop, void());
    MOCK_METHOD0(get_emergencystop, bool());
    MOCK_METHOD2(set_fn, void(uint32_t address, uint16_t value));
    MOCK_METHOD1(get_fn, uint16_t(uint32_t address));
    MOCK_METHOD0(legacy_address, uint32_t());
    MOCK_METHOD0(legacy_address_type, dcc::TrainAddressType());
};

}  // namespace openlcb

#endif // _UTILS_MOCKTRAIN_HXX_
