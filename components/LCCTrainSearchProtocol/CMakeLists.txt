set(COMPONENT_SRCS
    "AllTrainNodes.cpp"
    "FdiXmlGenerator.cpp"
    "FindProtocolDefs.cpp"
    "XmlGenerator.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS
    "include"
)

set(COMPONENT_REQUIRES
    "OpenMRNLite"
)

register_component()

set_source_files_properties(AllTrainNodes.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(FindProtocolDefs.cpp PROPERTIES COMPILE_FLAGS "-Wno-type-limits -Wno-ignored-qualifiers")
set_source_files_properties(FdiXmlGenerator.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
