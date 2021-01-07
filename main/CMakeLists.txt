set(esp_idf_deps
    "json"
    "driver"
    "efuse"
    "soc"
    "nvs_flash"
    "spi_flash"
    "vfs"
    "esp_common"
    "esp_adc_cal"
    "bootloader_support"
    "app_update"
    "esp_wifi"
)

set(required_deps
    "Configuration"
    "DCCppProtocol"
    "DCCSignalGenerator"
    "DCCTurnoutManager"
    "HttpServer"
    "LCCTrainSearchProtocol"
    "nlohmann_json"
    "OpenMRNLite"
    "TaskMonitor"
)

# These are all optional dependencies but due to the way that idf.py operates
# they must be specified here in full otherwise they will not be present in the
# include path when enabled.

set(optional_deps
    "GPIO"
    "HC12"
    "JmriInterface"
    "StatusLED"
    "StatusDisplay"
)

set(COMPONENT_REQUIRES "${esp_idf_deps} ${required_deps} ${optional_deps}")

set(COMPONENT_SRCS
    "ESP32Bootloader.cpp"
    "ESP32CommandStation.cpp"
    "ESP32TrainDatabase.cpp"
    "OpenMRNEsp32Overrides.cpp"
    "WebServer.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()

idf_build_get_property(project_ver PROJECT_VER)

set_source_files_properties(ESP32Bootloader.cpp PROPERTIES COMPILE_FLAGS "-Wno-implicit-fallthrough -Wno-ignored-qualifiers")
set_source_files_properties(ESP32CommandStation.cpp PROPERTIES COMPILE_FLAGS "-Wno-implicit-fallthrough -Wno-ignored-qualifiers")
set_source_files_properties(ESP32CommandStation.cpp PROPERTIES COMPILE_DEFINITIONS "SNIP_SW_VERSION=\"v1.5.0\"; CDI_SW_VERSION=\"${PROJECT_VER}\"; CDI_HW_VERSION=\"ESP32-v1\";")
set_source_files_properties(ESP32TrainDatabase.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(OpenMRNEsp32Overrides.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(WebServer.cpp PROPERTIES COMPILE_FLAGS "-Wno-implicit-fallthrough -Wno-ignored-qualifiers")