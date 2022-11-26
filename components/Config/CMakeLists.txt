set(IDF_DEPS
    hal
)

set(CUSTOM_DEPS
)

idf_component_register(INCLUDE_DIRS include
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")
