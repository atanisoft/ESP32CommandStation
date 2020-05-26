set(src_dirs
    "src/dcc"
    "src/executor"
    "src/freertos_drivers/esp32"
    "src/freertos_drivers/arduino"
    "src/openlcb"
    "src/os"
    "src/utils"
)

if(CONFIG_ENABLE_ARDUINO_DEPENDS)
    list(APPEND src_dirs "src")
endif()

set(COMPONENT_SRCDIRS "${src_dirs}" )

set(COMPONENT_ADD_INCLUDEDIRS "src" )

set(COMPONENT_REQUIRES "mdns" )

register_component()

###############################################################################
# Add required compilation flags for customization of OpenMRNLite
###############################################################################

target_compile_options(${COMPONENT_LIB} PUBLIC -DESP32 -DLOCKED_LOGGING)

set_source_files_properties(src/utils/FileUtils.cpp PROPERTIES COMPILE_FLAGS -Wno-type-limits)
set_source_files_properties(src/dcc/Loco.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/dcc/SimpleUpdateLoop.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/dcc/UpdateLoop.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/executor/Executor.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/executor/StateFlow.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/executor/Service.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/executor/Timer.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/freertos_drivers/arduino/CpuLoad.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/freertos_drivers/esp32/Esp32WiFiManager.cpp PROPERTIES COMPILE_FLAGS
    "-Wno-ignored-qualifiers -DESP32_WIFIMGR_SOCKETPARAMS_LOG_LEVEL=VERBOSE -DESP32_WIFIMGR_MDNS_LOOKUP_LOG_LEVEL=VERBOSE")
set_source_files_properties(src/openlcb/AliasAllocator.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/BroadcastTime.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/BroadcastTimeClient.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/BroadcastTimeServer.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/ConfigEntry.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/ConfigUpdateFlow.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/DccAccyProducer.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/Datagram.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/DatagramCan.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/DatagramTcp.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/DefaultNode.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/EventHandler.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/EventHandlerContainer.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/EventHandlerTemplates.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/EventService.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/If.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/IfCan.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/IfImpl.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/IfTcp.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/MemoryConfig.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/NodeBrowser.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/NodeInitializeFlow.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/NonAuthoritativeEventProducer.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/PIPClient.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/RoutingLogic.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/SimpleNodeInfo.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/SimpleNodeInfoMockUserFile.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/SimpleStack.cpp PROPERTIES COMPILE_FLAGS "-Wno-ignored-qualifiers -Wno-implicit-fallthrough")
set_source_files_properties(src/openlcb/TractionCvSpace.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/TractionDefs.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/TractionProxy.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/TractionTestTrain.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/TractionThrottle.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/TractionTrain.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/openlcb/WriteHelper.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/utils/CanIf.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/utils/GcTcpHub.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/utils/GridConnectHub.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/utils/HubDevice.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/utils/HubDeviceSelect.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/utils/SocketClient.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(src/utils/socket_listener.cpp PROPERTIES COMPILE_FLAGS -DSOCKET_LISTENER_CONNECT_LOG_LEVEL=VERBOSE)
