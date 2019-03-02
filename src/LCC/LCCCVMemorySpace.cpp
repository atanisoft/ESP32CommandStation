#if 0
#include "DCCppESP32.h"
#include <openlcb/MemoryConfig.hxx>

using openlcb::Defs;
using openlcb::MemoryConfigDefs;
using openlcb::MemorySpace;
using openlcb::Node;
using openlcb::NodeID;
using openlcb::TractionDefs;

class CVMemorySpace : public MemorySpace {
public:
    bool set_node(Node* node) override {
        NodeID id = node->node_id();
        if ((id & 0xFFFF00000000ULL) == TractionDefs::NODE_ID_DCC) {
            uint16_t new_address = id & 0xFFFFU;
            if (dccAddress != new_address) {
                dccAddress = new_address;
            }
            return true;
        } else {
            return false;
        }
    }

    bool read_only() override {
        return false;
    }

    address_t max_address() {
        return OFFSET_CV_VALUE;
    }

    size_t write(address_t destination, const uint8_t *data, size_t len,
        errorcode_t *error, Notifiable *again) override {
        if (destination == OFFSET_CV_INDEX) {
            lastDccAddress = dccAddress;
            uint8_t* cv = (uint8_t*)&cvNumber;
            if (len > 0) cv[3] = data[0];
            if (len > 1) cv[2] = data[1];
            if (len > 2) cv[1] = data[2];
            if (len > 3) cv[0] = data[3];
            return std::min(len, size_t(4));
        } else if (destination == OFFSET_CV_VERIFY_VALUE) {
            cvVerifyValue = data[0];
            return 1;
        } else if (destination == OFFSET_CV_VALUE) {
            if(lastDccAddress != dccAddress) {
                *error = Defs::ERROR_TEMPORARY;
                return 0;
            }
            if ((cvNumber - 1) > MAX_CV) {
                // CV is outside the expected range
                *error = MemoryConfigDefs::ERROR_OUT_OF_BOUNDS;
                return 0;
            }
            if(!enterProgrammingMode()) {
                // we failed to activate programming track, reject the request
                *error = Defs::ERROR_REJECTED;
                return 0;
            }
            bool success = writeProgCVByte(cvNumber, data[0]);
            leaveProgrammingMode();
            if(success) {
                return 1;
            }
            
        }
    }

    size_t read(address_t source, uint8_t *dst, size_t len, errorcode_t *error,
        Notifiable *again) override {
        // tbd
    }
private:
    uint16_t dccAddress{0};
    uint16_t lastDccAddress{0};
    uint16_t cvNumber{0};
    uint8_t cvData{0};
    uint8_t cvVerifyValue{0};
    static constexpr uint16_t MAX_CV = 1023;
    enum {
        OFFSET_CV_INDEX = 0x7F000000,
        OFFSET_CV_VALUE = 0x7F000004,
        OFFSET_CV_VERIFY_VALUE = 0x7F000005,
        OFFSET_CV_VERIFY_RESULT = 0x7F000006,
    };
};

#endif