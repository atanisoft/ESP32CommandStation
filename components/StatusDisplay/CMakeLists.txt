set(IDF_DEPS
    driver
)

set(CUSTOM_DEPS
    Config
    NvsManager
    OpenMRNIDF
    TrainSearchProtocol
)

set(LED_SOURCE_FILES
    StatusDisplay.cpp
    StatusDisplayLCD.cpp
    StatusDisplayOLED.cpp
)

idf_component_register(SRCS "${LED_SOURCE_FILES}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "private_include"
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")

set_source_files_properties(StatusLED.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)