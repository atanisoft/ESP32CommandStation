set(IDF_DEPS
    json
)

set(CUSTOM_DEPS
    Config
    HttpServer
    OpenMRNIDF
    Utils
)

set(TURNOUT_SOURCE_FILES
    turnout_constants.cpp
    GenericTurnout.cpp
    OpenLCBTurnout.cpp
    TurnoutManager.cpp
)

idf_component_register(SRCS "${TURNOUT_SOURCE_FILES}"
                       INCLUDE_DIRS "include"
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")

set_source_files_properties(TurnoutManager.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)