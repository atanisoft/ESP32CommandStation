set(CUSTOM_DEPS
    Config
    OpenMRNIDF
    NeoPixelBus
)

set(LED_SOURCE_FILES
    StatusLED.cpp
)

idf_component_register(SRCS "${LED_SOURCE_FILES}"
                       INCLUDE_DIRS "include"
                       REQUIRES "${CUSTOM_DEPS}")

set_source_files_properties(StatusLED.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)