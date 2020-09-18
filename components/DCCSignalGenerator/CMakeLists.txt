set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_PRIV_INCLUDEDIRS "private_include" )

set(COMPONENT_SRCS
    "DCCSignalVFS.cpp"
    "DuplexedTrackIf.cpp"
    "EStopHandler.cpp"
    "MonitoredHBridge.cpp"
    "RMTTrackDevice.cpp"
)

set(COMPONENT_REQUIRES
    "OpenMRNLite"
    "driver"
    "esp_adc_cal"
    "LCCTrainSearchProtocol"
    "nlohmann_json"
    "StatusDisplay"
    "StatusLED"
    "vfs"
)

register_component()

set_source_files_properties(DCCSignalVFS.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(DuplexedTrackIf.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(EStopHandler.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(LocalTrackIf.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(MonitoredHBridge.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(RMTTrackDevice.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)

if (CONFIG_OPS_RAILCOM)
    if (CONFIG_OPS_RAILCOM_FEEDBACK_QUEUE LESS CONFIG_OPS_PACKET_QUEUE_SIZE)
        message(FATAL_ERROR "OPS RailCom feedback queue must be larger than the OPS DCC packet queue.")
    endif()
endif()