set(IDF_DEPS
    driver
    ulp
)

set(CUSTOM_DEPS
    Config
)

if(CONFIG_IDF_TARGET_ESP32)
    idf_component_register(SRCS UlpAdc.cpp
                           INCLUDE_DIRS "include"
                           REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")

    ulp_embed_binary(ulp_adc_ops adc_ops.S UlpAdc.cpp)
elseif(CONFIG_IDF_TARGET_ESP32S3)
    idf_component_register(SRCS UlpAdcEsp32S3.cpp
                           INCLUDE_DIRS "include"
                           REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")
endif()

