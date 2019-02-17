#ifndef _UTILS_ASYNC_TRACTION_TEST_HELPER_HXX_
#define _UTILS_ASYNC_TRACTION_TEST_HELPER_HXX_

#include "utils/async_if_test_helper.hxx"

#include "openlcb/TractionDefs.hxx"
#include "openlcb/TractionTrain.hxx"
#include "utils/MockTrain.hxx"

namespace openlcb
{

/// Test fixture base for traction tests.
class TractionTest : public AsyncNodeTest
{
protected:
    TractionTest()
        : trainService_(ifCan_.get())
    {
    }

    TrainService trainService_;
    StrictMock<MockTrain> m1_, m2_;
};


} // namespace openlcb

#endif // _UTILS_ASYNC_TRACTION_TEST_HELPER_HXX_
