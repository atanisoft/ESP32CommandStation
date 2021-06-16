
set(IDF_DEPS
    fatfs
    spi_flash
    spiffs
    vfs
)

set(CUSTOM_DEPS
    Config
    StatusDisplay
    StatusLED
)

set(UTILS_SOURCE_FILES
    FileSystem.cpp
)

idf_component_register(SRCS "${UTILS_SOURCE_FILES}"
                       INCLUDE_DIRS include
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")