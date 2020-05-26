
set(COMPONENT_SRCS
    "DCCppProtocol.cpp"
    "DCCProgrammer.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_REQUIRES
    "Configuration"
    "DCCSignalGenerator"
    "DCCTurnoutManager"
    "Esp32HttpServer"
    "GPIO"
)

register_component()

set_source_files_properties(DCCppProtocol.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(DCCProgrammer.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)