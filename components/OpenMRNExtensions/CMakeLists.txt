
idf_component_register(SRC_DIRS src/locodb src/locomgr src/trainsearch
                       INCLUDE_DIRS "src"
                       REQUIRES OpenMRNIDF)

set_source_files_properties(src/trainsearch/Defs.cpp PROPERTIES COMPILE_FLAGS "-Wno-type-limits")