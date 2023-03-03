set(IDF_DEPS
    driver
    esp_timer
    ulp
    vfs
)

if (IDF_VERSION_MAJOR LESS 5)
    # esp_adc_cal component was combined with esp_adc in ESP-IDF v5.0
    list(APPEND IDF_DEPS esp_adc_cal)
endif()

set(CUSTOM_DEPS
    Config
    AccessoryDecoderDB
    OpenMRNIDF
    OpenMRNExtensions
    StatusDisplay
    ULPADC
    Utils
)

idf_component_register(SRCS DccConstants.cpp DCCSignalVFS.cpp PrioritizedUpdateLoop.cpp
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "private_include"
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")
