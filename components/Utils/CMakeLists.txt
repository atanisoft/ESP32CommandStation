
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
    HttpServer
)

idf_component_register(SRCS FileSystem.cpp CDIClient.cpp CDIDownloader.cpp
                       INCLUDE_DIRS include
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")