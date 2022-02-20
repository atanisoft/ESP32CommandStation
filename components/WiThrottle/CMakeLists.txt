
set(IDF_DEPS
)

set(CUSTOM_DEPS
    AccessoryDecoderDB
    Config
    HttpServer
    OpenMRNIDF
    TrainDatabase
    TrainSearchProtocol
)

set(SOURCE_FILES
    WiThrottleServer.cpp
    WiThrottleClientFlow.cpp
    WiThrottleConstants.cpp
    WiThrottleCmdBase.cpp
    WiThrottleCmdPacket.cpp
    WiThrottleCmdPanel.cpp
    WiThrottleCmdRoster.cpp
    WiThrottleCmdThrottle.cpp
    WiThrottleCmdThrottleLegacy.cpp
)

idf_component_register(SRCS ${SOURCE_FILES}
                       INCLUDE_DIRS include
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")
