set(CUSTOM_DEPS
    OpenMRNIDF
    TrainSearchProtocol
    Utils
    WiThrottle
)

set(IDF_DEPS
    json
)

idf_component_register(SRCS Esp32TrainDatabase.cpp Esp32TrainDbEntry.cpp
                       INCLUDE_DIRS include
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")

