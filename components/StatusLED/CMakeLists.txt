set(COMPONENT_SRCS
    "StatusLED.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_REQUIRES
    "OpenMRNLite"
    "NeoPixelBus"
)

register_component()

set_source_files_properties(StatusLED.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)