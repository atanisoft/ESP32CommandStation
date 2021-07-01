set(CUSTOM_DEPS
    OpenMRNIDF
    TrainSearchProtocol
    Utils
)

set(IDF_DEPS
    json
)

set(DB_SOURCE_FILES
    Esp32TrainDatabase.cpp
    Esp32TrainDbEntry.cpp
)

idf_component_register(SRCS "${DB_SOURCE_FILES}"
                       INCLUDE_DIRS include
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")

set_source_files_properties(Esp32TrainDatabase.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
