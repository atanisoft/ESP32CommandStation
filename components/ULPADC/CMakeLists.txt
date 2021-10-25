set(IDF_DEPS
    driver
    ulp
)

set(CUSTOM_DEPS
    Config
)
  
idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "esp32s3")
    idf_component_register(SRCS UlpAdcEsp32S3.cpp
                           INCLUDE_DIRS "include"
                           REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")
else()
    idf_component_register(SRCS UlpAdc.cpp
                           INCLUDE_DIRS "include"
                           REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")

    ulp_embed_binary(ulp_adc_ops adc_ops.S UlpAdc.cpp)
endif()

