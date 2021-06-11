set(CUSTOM_DEPS
    OpenMRNIDF
    TrainSearchProtocol
    nlohmann_json
    Utils
)

idf_component_register(SRCS TrainDatabase.cpp
                       INCLUDE_DIRS include
                       REQUIRES "${CUSTOM_DEPS}")

idf_build_get_property(project_ver PROJECT_VER)
string(SUBSTRING "${project_ver}" 0 20 SNIP_SW_VERSION)
set(SNIP_HW_VERSION "1.0.0")

set_source_files_properties(TrainDatabase.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(TrainDatabase.cpp PROPERTIES COMPILE_DEFINITIONS "SNIP_HW_VERSION=\"${SNIP_HW_VERSION}\"; SNIP_SW_VERSION=\"${SNIP_SW_VERSION}\"")
