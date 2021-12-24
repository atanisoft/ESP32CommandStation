set(CUSTOM_DEPS
    Config
    OpenMRNIDF
    NeoPixelBus
)

idf_build_get_property(target IDF_TARGET)

if({target} STREQUAL "esp32s3")
idf_component_register(SRCS StatusLED.cpp StatusLEDGpio.cpp
                       INCLUDE_DIRS "include"
                       REQUIRES "${CUSTOM_DEPS}")
else()
idf_component_register(SRCS StatusLED.cpp StatusLEDNeoPixel.cpp
                       INCLUDE_DIRS "include"
                       REQUIRES "${CUSTOM_DEPS}")
endif()