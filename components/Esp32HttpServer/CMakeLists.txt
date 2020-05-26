set(COMPONENT_SRCS
    "Dnsd.cpp"
    "HttpdConstants.cpp"
    "HttpRequest.cpp"
    "HttpRequestFlow.cpp"
    "HttpRequestWebSocket.cpp"
    "HttpResponse.cpp"
    "HttpServer.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_REQUIRES "OpenMRNLite" "lwip" "mbedtls")

register_component()
set_source_files_properties(HttpRequest.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(HttpRequestWebSocket.cpp PROPERTIES COMPILE_FLAGS "-Wno-implicit-fallthrough -Wno-ignored-qualifiers")
set_source_files_properties(HttpRequestFlow.cpp PROPERTIES COMPILE_FLAGS "-Wno-implicit-fallthrough -Wno-ignored-qualifiers")
set_source_files_properties(HttpResponse.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(HttpServer.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
