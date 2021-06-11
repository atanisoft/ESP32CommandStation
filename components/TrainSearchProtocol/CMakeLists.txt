set(CUSTOM_DEPS
    OpenMRNIDF
    Utils
)

set(TSP_SOURCE_FILES
    AllTrainNodes.cpp
    FdiXmlGenerator.cpp
    FindProtocolDefs.cpp
    XmlGenerator.cpp
)

idf_component_register(SRCS "${TSP_SOURCE_FILES}"
                       INCLUDE_DIRS include
                       PRIV_INCLUDE_DIRS private_include
                       REQUIRES "${CUSTOM_DEPS}")

set_source_files_properties(AllTrainNodes.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(FindProtocolDefs.cpp PROPERTIES COMPILE_FLAGS "-Wno-type-limits -Wno-ignored-qualifiers")
set_source_files_properties(FdiXmlGenerator.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
