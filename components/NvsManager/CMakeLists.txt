set(IDF_DEPS
    app_update
    nvs_flash
    esp_wifi
)

set(CUSTOM_DEPS
    Config
    OpenMRNIDF
    OpenMRNExtensions
    StatusLED
    Utils
)

idf_component_register(SRCS NvsManager.cpp
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "private_include"
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")