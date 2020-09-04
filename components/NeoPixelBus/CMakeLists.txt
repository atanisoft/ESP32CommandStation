cmake_minimum_required(VERSION 3.10)

file(GLOB SOURCES_CPP src/internal/*.cpp)
file(GLOB SOURCES_C src/internal/*.c)
idf_component_register(
    SRCS ${SOURCES_CPP} ${SOURCES_C}
    INCLUDE_DIRS src
    REQUIRES driver)

target_compile_options(${COMPONENT_LIB} PRIVATE -DESP32)