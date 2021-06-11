set(IDF_DEPS
    app_update
    nvs_flash
)

set(CUSTOM_DEPS
    Config
    OpenMRNIDF
    StatusLED
    Utils
)

set(NVS_SOURCE_FILES
    NvsManager.cpp
)

idf_component_register(SRCS "${NVS_SOURCE_FILES}"
                       INCLUDE_DIRS "include"
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")