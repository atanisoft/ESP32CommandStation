set(COMPONENT_SRCS
    "FileSystemManager.cpp"
    "LCCStackManager.cpp"
    "LCCWiFiManager.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_REQUIRES
    "DCCSignalGenerator"
    "esp_adc_cal"
    "HttpServer"
    "spiffs"
    "vfs"
    "fatfs"
    "OpenMRNLite"
)

register_component()

set_source_files_properties(FileSystemManager.cpp PROPERTIES COMPILE_FLAGS "-Wno-implicit-fallthrough -Wno-ignored-qualifiers")
set_source_files_properties(LCCStackManager.cpp PROPERTIES COMPILE_FLAGS "-Wno-implicit-fallthrough -Wno-ignored-qualifiers")
set_source_files_properties(LCCWiFiManager.cpp PROPERTIES COMPILE_FLAGS "-Wno-implicit-fallthrough -Wno-ignored-qualifiers")