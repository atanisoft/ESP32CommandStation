set(IDF_DEPS
    driver
)

set(CUSTOM_DEPS
    Config
    NvsManager
    OpenMRNExtensions
)

idf_component_register(SRCS StatusDisplay.cpp StatusDisplayLCD.cpp StatusDisplayOLED.cpp
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "private_include"
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")
