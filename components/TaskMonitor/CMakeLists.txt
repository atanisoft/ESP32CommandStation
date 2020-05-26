set(COMPONENT_SRCS
    "FreeRTOSTaskMonitor.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_REQUIRES
    "OpenMRNLite"
    "freertos"
    "soc"
)

register_component()

set_source_files_properties(FreeRTOSTaskMonitor.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)