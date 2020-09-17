cmake_minimum_required(VERSION 3.10)

file(GLOB SOURCES src/*.cpp)
set(INCLUDE_PATH include)
set(DEPS lwip mbedtls)

if(CONFIG_HTTP_DNS_LIGHT_OPENMRN_LIB)
    file(GLOB OPENMRN_EXECUTOR src/OpenMRNUltraLite/executor/*.cpp)
    list(APPEND SOURCES ${OPENMRN_EXECUTOR})

    file(GLOB OPENMRN_OS src/OpenMRNUltraLite/os/*.cpp)
    file(GLOB OPENMRN_OS_C src/OpenMRNUltraLite/os/*.c)
    list(APPEND SOURCES ${OPENMRN_OS})
    list(APPEND SOURCES ${OPENMRN_OS_C})

    file(GLOB OPENMRN_UTILS src/OpenMRNUltraLite/utils/*.cpp)
    list(APPEND SOURCES ${OPENMRN_UTILS})

    list(APPEND INCLUDE_PATH src/OpenMRNUltraLite)
else()
    list(APPEND DEPS OpenMRNLite)
endif()

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS ${INCLUDE_PATH}
    REQUIRES ${DEPS})

if(CONFIG_HTTP_DNS_LIGHT_OPENMRN_LIB)
target_compile_options(${COMPONENT_LIB} PRIVATE -DESP32)
endif()

set_source_files_properties(src/HttpRequest.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/HttpRequestWebSocket.cpp PROPERTIES COMPILE_FLAGS "-Wno-implicit-fallthrough -Wno-ignored-qualifiers")
set_source_files_properties(src/HttpRequestFlow.cpp PROPERTIES COMPILE_FLAGS "-Wno-implicit-fallthrough -Wno-ignored-qualifiers")
set_source_files_properties(src/HttpResponse.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/HttpServer.cpp PROPERTIES COMPILE_FLAGS "-Wno-implicit-fallthrough -Wno-ignored-qualifiers")