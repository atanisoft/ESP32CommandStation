set(IDF_DEPS
    json
)

set(CUSTOM_DEPS
    Config
    Utils
)

idf_component_register(SRCS AccessoryDecoderConstants.cpp AccessoryDecoderDB.cpp DccAccessoryDecoder.cpp OpenLCBAccessoryDecoder.cpp
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "private_include"
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")