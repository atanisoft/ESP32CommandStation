set(COMPONENT_SRCS
    "HC12Radio.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_REQUIRES
    "OpenMRNLite"
    "driver"
    "DCCppProtocol"
)

register_component()

set_source_files_properties(HC12Radio.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)