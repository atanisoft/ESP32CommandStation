set(COMPONENT_SRCS
    "Outputs.cpp"
    "Sensors.cpp"
    "RemoteSensors.cpp"
    "S88Sensors.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_REQUIRES
    "Configuration"
    "DCCppProtocol"
    "nlohmann_json"
    "OpenMRNLite"
    "StatusDisplay"
    "driver"
)

register_component()

set_source_files_properties(Outputs.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(RemoteSensors.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(Sensors.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(S88Sensors.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)