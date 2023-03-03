
set(IDF_DEPS
)

set(CUSTOM_DEPS
    OpenMRNIDF
    Utils
)

idf_component_register(SRC_DIRS src/locodb src/locomgr src/trainsearch
                       INCLUDE_DIRS "src"
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")

set_source_files_properties(src/trainsearch/Defs.cpp PROPERTIES COMPILE_FLAGS "-Wno-type-limits")