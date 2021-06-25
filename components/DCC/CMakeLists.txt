set(IDF_DEPS
    driver
    esp_adc_cal
    esp_timer
    ulp
    vfs
)

set(CUSTOM_DEPS
    Config
    AccessoryDecoderDB
    OpenMRNIDF
    StatusDisplay
    TrainSearchProtocol
    Utils
)

set(DCCVFS_SOURCE_FILES
    DccConstants.cpp
    DCCSignalVFS.cpp
    PrioritizedUpdateLoop.cpp
)

idf_component_register(SRCS "${DCCVFS_SOURCE_FILES}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "private_include"
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")

ulp_embed_binary(ulp_current_sense current_sense.S DCCSignalVFS.cpp)

set_source_files_properties(DCCSignalVFS.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(DuplexedTrackIf.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(PrioritizedUpdateLoop.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
