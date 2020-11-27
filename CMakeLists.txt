# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

if (IDF_VERSION_MAJOR LESS 4)
    message(FATAL_ERROR "ESP32CommandStation requires IDF v4.0 (or newer)")
endif()

# ESP-IDF v4.1 has a few known issues:
# 1) RMT APIs will truncate the bit stream at 64 bits regardless of the number
# of memory blocks allocated to the RMT channel.
# 2) RailCom detector UART code may cause a ISR WDT timeout during init of the
# StatusLED module even though they should have zero interactions.
# 3) DCC Signal may be corrupted under certain circumstances (unconfirmed).
if ("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" STREQUAL "4.1")
    message(WARNING "ESP-IDF v${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH} has known stability issues, it is recommended to use ESP-IDF v4.0.x or v4.2+")
    # if the StatusLED module is enabled break the build since it will not work at runtime.
    if (CONFIG_STATUS_LED STREQUAL "y")
        message(FATAL_ERROR "ESP-IDF v${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH} includes a breaking change in the RMT API which will result in the StatusLED module malfunctioning.")
    endif()
endif()

#if (NOT CONFIG_FREERTOS_SUPPORT_STATIC_ALLOCATION STREQUAL "y")
#    message(FATAL_ERROR "CONFIG_FREERTOS_SUPPORT_STATIC_ALLOCATION must be enabled in sdkconfig")
#endif()

set(SUPPORTED_TARGETS esp32)
project(ESP32CommandStation)

###############################################################################
# Switch from GNU++11 to GNU++14
#
# NOTE: C++14 would be preferred but IDF has dependencies on GNU extensions.
###############################################################################

string(REPLACE "-std=gnu++11" "-std=gnu++14" CXX_OPTIONS "${CXX_COMPILE_OPTIONS}")
idf_build_set_property(CXX_COMPILE_OPTIONS "${CXX_OPTIONS}" REPLACE)

###############################################################################
# Add required compilation flags for customization of OpenMRNLite
###############################################################################

# none currently

###############################################################################
# Enable usage of std::stoi/stol/etc
###############################################################################

idf_build_set_property(COMPILE_DEFINITIONS "-D_GLIBCXX_USE_C99" APPEND)

###############################################################################
# Search for GZIP application
###############################################################################

FIND_PROGRAM(GZIP
  NAMES gzip
  PATHS /bin
        /usr/bin
        /usr/local/bin
)

if (NOT GZIP)
  message(FATAL_ERROR "Unable to find 'gzip' program")
endif()

###############################################################################
# Generate a compressed version of web content on-demand
###############################################################################

add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/data/index.html.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/data/index.html
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/data/index.html.gz")

add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/data/jqClock-lite.min.js.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/data/jqClock-lite.min.js
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/data/jqClock-lite.min.js.gz")

add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.min.js.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.min.js
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.min.js.gz")

add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.js.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.js
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.js.gz")

add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.css.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.css
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.css.gz")

add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.simple.websocket.min.js.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.simple.websocket.min.js
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.simple.websocket.min.js.gz")

###############################################################################
# Add web content to the binary
###############################################################################

target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/index.html.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/jqClock-lite.min.js.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.min.js.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.js.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.css.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.simple.websocket.min.js.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/ajax-loader.gif" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/loco-32x32.png" BINARY)

###############################################################################
# Configuration validations
###############################################################################

if (NOT CONFIG_FREERTOS_HZ EQUAL 1000)
    message(FATAL_ERROR "FreeRTOS tick rate (hz) is required to be 1000.")
endif()

if (NOT CONFIG_PARTITION_TABLE_FILENAME STREQUAL "ESP32CS-partitions.csv")
    message(FATAL_ERROR "The custom partition table option is not enabled in menuconfig and is required for compilation.")
endif()

if (NOT CONFIG_PARTITION_TABLE_CUSTOM_FILENAME STREQUAL "ESP32CS-partitions.csv")
    message(FATAL_ERROR "The custom partition table option is not enabled in menuconfig and is required for compilation.")
endif()

if (NOT CONFIG_PARTITION_TABLE_CUSTOM)
    message(FATAL_ERROR "The custom partition table option is not enabled in menuconfig and is required for compilation.")
endif()

if (NOT CONFIG_LWIP_SO_RCVBUF)
    message(FATAL_ERROR "LwIP SO_RCVBUF is a required option in menuconfig.")
endif()

if (CONFIG_ESP_MAIN_TASK_STACK_SIZE LESS 8192)
    message(FATAL_ERROR "Main task stack size must be at least 8192 bytes.")
endif()

###############################################################################
# Ensure SSID and PASSWORD are provided.
###############################################################################
if (CONFIG_WIFI_MODE_STATION OR CONFIG_WIFI_MODE_SOFTAP_STATION)
    if (NOT CONFIG_WIFI_SSID OR NOT CONFIG_WIFI_PASSWORD)
        message(FATAL_ERROR "WiFi SSID & Password are required when WiFi mode is STATION or STATION+SoftAP.")
    endif()
endif()

###############################################################################
# LCC interface configuration validations
###############################################################################
if (CONFIG_LCC_NODE_ID EQUAL "")
    message(FATAL_ERROR "LCC Node ID must be defined")
endif()

if (CONFIG_LCC_CAN_RX_PIN GREATER_EQUAL 0 AND CONFIG_LCC_CAN_TX_PIN GREATER_EQUAL 0)
    if (CONFIG_LCC_CAN_RX_PIN EQUAL CONFIG_LCC_CAN_TX_PIN)
        message(FATAL_ERROR "LCC CAN RX and LCC CAN TX pin must be unique.")
    endif()

    if (CONFIG_LCC_CAN_RX_PIN GREATER_EQUAL 6 AND CONFIG_LCC_CAN_RX_PIN LESS_EQUAL 11)
        message(FATAL_ERROR "LCC CAN RX pin can not be set to pin 6-11 (used by onboard flash).")
    endif()

    if (CONFIG_LCC_CAN_TX_PIN GREATER_EQUAL 6 AND CONFIG_LCC_CAN_TX_PIN LESS_EQUAL 11)
        message(FATAL_ERROR "LCC CAN TX pin can not be set to pin 6-11 (used by onboard flash).")
    endif()

    if (CONFIG_STATUS_LED)
        if (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_LCC_CAN_RX_PIN)
            message(FATAL_ERROR "Status LED data pin and LCC CAN RX pin must be unique.")
        endif()
        if (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_LCC_CAN_TX_PIN)
            message(FATAL_ERROR "Status LED data pin and LCC CAN TX pin must be unique.")
        endif()
    endif()
    if (CONFIG_GPIO_S88)
        if (CONFIG_GPIO_S88_CLOCK_PIN EQUAL CONFIG_LCC_CAN_RX_PIN)
            message(FATAL_ERROR "S88 Clock pin and LCC CAN RX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_LOAD_PIN EQUAL CONFIG_LCC_CAN_RX_PIN)
            message(FATAL_ERROR "S88 Load pin and LCC CAN RX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_RESET_PIN EQUAL CONFIG_LCC_CAN_RX_PIN)
            message(FATAL_ERROR "S88 Reset pin and LCC CAN RX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_CLOCK_PIN EQUAL CONFIG_LCC_CAN_TX_PIN)
            message(FATAL_ERROR "S88 Clock pin and LCC CAN TX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_LOAD_PIN EQUAL CONFIG_LCC_CAN_TX_PIN)
            message(FATAL_ERROR "S88 Load pin and LCC CAN TX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_RESET_PIN EQUAL CONFIG_LCC_CAN_TX_PIN)
            message(FATAL_ERROR "S88 Reset pin and LCC CAN TX pin must be unique.")
        endif()
    endif()
    if (CONFIG_NEXTION)
        if (CONFIG_NEXTION_RX_PIN EQUAL CONFIG_LCC_CAN_RX_PIN)
            message(FATAL_ERROR "Nextion RX pin and LCC CAN RX pin must be unique.")
        endif()
        if (CONFIG_NEXTION_RX_PIN EQUAL CONFIG_LCC_CAN_TX_PIN)
            message(FATAL_ERROR "Nextion RX pin and LCC CAN TX pin must be unique.")
        endif()
        if (CONFIG_NEXTION_TX_PIN EQUAL CONFIG_LCC_CAN_RX_PIN)
            message(FATAL_ERROR "Nextion TX pin and LCC CAN RX pin must be unique.")
        endif()
        if (CONFIG_NEXTION_TX_PIN EQUAL CONFIG_LCC_CAN_TX_PIN)
            message(FATAL_ERROR "Nextion TX pin and LCC CAN TX pin must be unique.")
        endif()
    endif()
    if (CONFIG_LOCONET)
        if (CONFIG_LOCONET_RX_PIN EQUAL CONFIG_LCC_CAN_RX_PIN)
            message(FATAL_ERROR "LocoNet RX pin and LCC CAN RX pin must be unique.")
        endif()
        if (CONFIG_LOCONET_RX_PIN EQUAL CONFIG_LCC_CAN_TX_PIN)
            message(FATAL_ERROR "LocoNet RX pin and LCC CAN TX pin must be unique.")
        endif()
        if (CONFIG_LOCONET_TX_PIN EQUAL CONFIG_LCC_CAN_RX_PIN)
            message(FATAL_ERROR "LocoNet TX pin and LCC CAN RX pin must be unique.")
        endif()
        if (CONFIG_LOCONET_TX_PIN EQUAL CONFIG_LCC_CAN_TX_PIN)
            message(FATAL_ERROR "LocoNet TX pin and LCC CAN TX pin must be unique.")
        endif()
    endif()
endif()

###############################################################################
# OPS H-Bridge validations
###############################################################################

if (NOT CONFIG_OPS_HBRIDGE_ENABLE_PIN OR NOT CONFIG_OPS_HBRIDGE_SIGNAL_PIN OR NOT CONFIG_OPS_DCC_PREAMBLE_BITS OR CONFIG_OPS_ADC STREQUAL "")
    message(FATAL_ERROR "One (or more) OPS H-Bridge required parameters is not defined.")
endif()

if (CONFIG_OPS_HBRIDGE_ENABLE_PIN GREATER_EQUAL 6 AND CONFIG_OPS_HBRIDGE_ENABLE_PIN LESS_EQUAL 11)
    message(FATAL_ERROR "OPS H-Bridge enable pin can not be set to pin 6-11 (used by onboard flash).")
endif()

if (CONFIG_OPS_HBRIDGE_SIGNAL_PIN GREATER_EQUAL 6 AND CONFIG_OPS_HBRIDGE_SIGNAL_PIN LESS_EQUAL 11)
    message(FATAL_ERROR "OPS H-Bridge signal pin can not be set to pin 6-11 (used by onboard flash).")
endif()

if (CONFIG_OPS_HBRIDGE_ENABLE_PIN EQUAL 0 OR CONFIG_OPS_HBRIDGE_ENABLE_PIN EQUAL 5 OR CONFIG_OPS_HBRIDGE_ENABLE_PIN EQUAL 15)
    message(FATAL_ERROR "OPS H-Bridge enable pin should not use GPIO 0, 5, 15. These pins typically have a pull-up resistor on them.")
endif()

if (CONFIG_OPS_HBRIDGE_ENABLE_PIN EQUAL 1 OR CONFIG_OPS_HBRIDGE_ENABLE_PIN EQUAL 3)
    message(FATAL_ERROR "OPS H-Bridge enable pin should not use GPIO 1 or 3 as these pins are reserved for UART0.")
endif()

if (CONFIG_OPS_HBRIDGE_SIGNAL_PIN EQUAL 1 OR CONFIG_OPS_HBRIDGE_SIGNAL_PIN EQUAL 3)
    message(FATAL_ERROR "OPS H-Bridge signal pin should not use GPIO 1 or 3 as these pins are reserved for UART0.")
endif()

if (CONFIG_OPS_HBRIDGE_ENABLE_PIN EQUAL CONFIG_HBRIDGE_OPS_BRAKE_PIN)
    message(FATAL_ERROR "OPS H-Bridge brake pin and OPS H-Bridge enable pin must be unique.")
endif()

if (CONFIG_OPS_HBRIDGE_BRAKE_PIN EQUAL 1 OR CONFIG_OPS_HBRIDGE_ENABLE_PIN EQUAL 3)
    message(FATAL_ERROR "OPS H-Bridge brake pin should not use GPIO 1 or 3 as these pins are reserved for UART0.")
endif()

if (CONFIG_OPS_HBRIDGE_BRAKE_PIN EQUAL 3)
    message(FATAL_ERROR "OPS H-Bridge brake pin should not use GPIO 3 as this pin is reserved for UART0.")
endif()

if (CONFIG_OPS_HBRIDGE_BRAKE_PIN GREATER_EQUAL 6 AND CONFIG_OPS_HBRIDGE_BRAKE_PIN LESS_EQUAL 11)
    message(FATAL_ERROR "OPS H-Bridge brake pin can not be set to pin 6-11 (used by onboard flash).")
endif()

if (CONFIG_OPS_HBRIDGE_SIGNAL_PIN EQUAL CONFIG_OPS_HBRIDGE_BRAKE_PIN)
    message(FATAL_ERROR "OPS H-Bridge brake pin and OPS H-Bridge signal pin must be unique.")
endif()

if (CONFIG_OPS_RAILCOM)
    if(CONFIG_OPS_HBRIDGE_ENABLE_PIN EQUAL CONFIG_OPS_RAILCOM_ENABLE_PIN)
        message(FATAL_ERROR "OPS RailCom enable pin and OPS H-Bridge enable pin must be unique.")
    endif()

    if (CONFIG_OPS_HBRIDGE_SIGNAL_PIN EQUAL CONFIG_OPS_RAILCOM_ENABLE_PIN)
        message(FATAL_ERROR "OPS RailCom enable pin and OPS H-Bridge signal pin must be unique.")
    endif()

    if (CONFIG_OPS_RAILCOM_ENABLE_PIN GREATER_EQUAL 6 AND CONFIG_OPS_RAILCOM_ENABLE_PIN LESS_EQUAL 11)
        message(FATAL_ERROR "OPS RailCom enable pin can not be set to pin 6-11 (used by onboard flash).")
    endif()

    if (CONFIG_OPS_RAILCOM_ENABLE_PIN EQUAL 1 OR CONFIG_OPS_RAILCOM_ENABLE_PIN EQUAL 3)
        message(FATAL_ERROR "OPS RailCom enable pin should not use GPIO 1 or 3 as these pins are reserved for UART0.")
    endif()

    if (CONFIG_OPS_RAILCOM_ENABLE_PIN EQUAL 0 OR CONFIG_OPS_RAILCOM_ENABLE_PIN EQUAL 5 OR CONFIG_OPS_RAILCOM_ENABLE_PIN EQUAL 15)
        message(FATAL_ERROR "OPS RailCom enable pin should not use GPIO 0, 5 or 15 as these pins typically have a pull-up resistor on them.")
    endif()

    if (CONFIG_OPS_HBRIDGE_ENABLE_PIN EQUAL CONFIG_OPS_RAILCOM_UART_RX_PIN)
        message(FATAL_ERROR "OPS RailCom UART RX pin and OPS H-Bridge enable pin must be unique.")
    endif()

    if (CONFIG_OPS_RAILCOM_UART_RX_PIN GREATER_EQUAL 6 AND CONFIG_OPS_RAILCOM_UART_RX_PIN LESS_EQUAL 11)
        message(FATAL_ERROR "OPS RailCom RX pin can not be set to pin 6-11 (used by onboard flash).")
    endif()

    if (CONFIG_OPS_RAILCOM_UART_RX_PIN EQUAL 1 OR CONFIG_OPS_RAILCOM_UART_RX_PIN EQUAL 3)
        message(FATAL_ERROR "OPS RailCom RX pin should not use GPIO 1 or 3 as these pins are reserved for UART0.")
    endif()

    if (CONFIG_OPS_HBRIDGE_SIGNAL_PIN EQUAL CONFIG_OPS_RAILCOM_UART_RX_PIN)
        message(FATAL_ERROR "OPS RailCom UART RX pin and OPS H-Bridge signal pin must be unique.")
    endif()
endif()

###############################################################################
# PROG H-Bridge validations
###############################################################################

if (NOT CONFIG_PROG_HBRIDGE_ENABLE_PIN OR NOT CONFIG_PROG_HBRIDGE_SIGNAL_PIN OR NOT CONFIG_PROG_DCC_PREAMBLE_BITS OR CONFIG_PROG_ADC STREQUAL "")
    message(FATAL_ERROR "One (or more) PROG H-Bridge required parameters is not defined.")
endif()

if (CONFIG_PROG_HBRIDGE_ENABLE_PIN GREATER_EQUAL 6 AND CONFIG_PROG_HBRIDGE_ENABLE_PIN LESS_EQUAL 11)
    message(FATAL_ERROR "PROG H-Bridge enable pin can not be set to pin 6-11 (used by onboard flash).")
endif()

if (CONFIG_PROG_SIGNAL_PIN GREATER_EQUAL 6 AND CONFIG_PROG_HBRIDGE_SIGNAL_PIN LESS_EQUAL 11)
    message(FATAL_ERROR "PROG H-Bridge signal pin can not be set to pin 6-11 (used by onboard flash).")
endif()

if (CONFIG_PROG_HBRIDGE_ENABLE_PIN EQUAL 0 OR CONFIG_PROG_HBRIDGE_ENABLE_PIN EQUAL 5 OR CONFIG_PROG_HBRIDGE_ENABLE_PIN EQUAL 15)
    message(FATAL_ERROR "PROG H-Bridge enable pin should not use GPIO 0, 5, 15. These pins typically have a pull-up resistor on them.")
endif()

if (CONFIG_PROG_HBRIDGE_ENABLE_PIN EQUAL 1 OR CONFIG_PROG_HBRIDGE_ENABLE_PIN EQUAL 3)
    message(FATAL_ERROR "PROG H-Bridge enable pin should not use GPIO 1 or 3 as these pins are reserved for UART0.")
endif()

if (CONFIG_PROG_HBRIDGE_SIGNAL_PIN EQUAL 1 OR CONFIG_PROG_HBRIDGE_SIGNAL_PIN EQUAL 3)
    message(FATAL_ERROR "PROG H-Bridge signal pin should not use GPIO 1 or 3 as these pins are reserved for UART0.")
endif()

if (CONFIG_PROG_DCC_PREAMBLE_BITS LESS 22)
    message(FATAL_ERROR "PROG track preamble bits is too low, at least 22 bits are required.")
endif()

if (CONFIG_PROG_DCC_PREAMBLE_BITS GREATER 75)
    message(FATAL_ERROR "PROG track preamble bits is too high, a maximum of 75 bits is supported.")
endif()

if (CONFIG_OPS_HBRIDGE_ENABLE_PIN EQUAL CONFIG_PROG_HBRIDGE_ENABLE_PIN)
    message(FATAL_ERROR "OPS and PROG H-Bridge enable pin must be unique.")
endif()

if (CONFIG_OPS_HBRIDGE_SIGNAL_PIN EQUAL CONFIG_PROG_HBRIDGE_SIGNAL_PIN)
    message(FATAL_ERROR "OPS and PROG H-Bridge signal pin must be unique.")
endif()

if (CONFIG_OPS_ADC EQUAL CONFIG_PROG_ADC)
    message(FATAL_ERROR "OPS and PROG H-Bridge current sense pins must be unique.")
endif()

if (CONFIG_STATUS_LED)
    if (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_OPS_HBRIDGE_ENABLE_PIN)
        message(FATAL_ERROR "Status LED data pin and OPS H-Bridge enable pin must be unique.")
    endif()

    if (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_PROG_HBRIDGE_ENABLE_PIN)
        message(FATAL_ERROR "Status LED data pin and PROG H-Bridge enable pin must be unique.")
    endif()

    if (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_OPS_HBRIDGE_SIGNAL_PIN)
        message(FATAL_ERROR "Status LED data pin and OPS H-Bridge signal pin must be unique.")
    endif()

    if (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_PROG_HBRIDGE_SIGNAL_PIN)
        message(FATAL_ERROR "Status LED data pin and PROG H-Bridge signal pin must be unique.")
    endif()

    if (CONFIG_STATUS_LED_DATA_PIN GREATER_EQUAL 6 AND CONFIG_STATUS_LED_DATA_PIN LESS_EQUAL 11)
        message(FATAL_ERROR "Status LED data pin can not be set to pin 6-11 (used by onboard flash).")
    endif()

    if (NOT CONFIG_ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS)
        if (CONFIG_STATUS_LED_DATA_PIN LESS 4 OR CONFIG_STATUS_LED_DATA_PIN EQUAL 5 OR CONFIG_STATUS_LED_DATA_PIN EQUAL 12 OR CONFIG_STATUS_LED_DATA_PIN EQUAL 15)
            message(FATAL_ERROR "Status LED data pin should not use GPIO 0-3, 5, 12, 15. These are reserved pins.")
        endif()
    endif()
endif()

###############################################################################
# Nextion interface validations
###############################################################################

if (CONFIG_NEXTION)
    if (CONFIG_NEXTION_RX_PIN EQUAL CONFIG_NEXTION_TX_PIN)
        message(FATAL_ERROR "Nextion RX and TX pin must be unique.")
    endif()

    if (CONFIG_NEXTION_RX_PIN GREATER_EQUAL 6 AND CONFIG_NEXTION_RX_PIN LESS_EQUAL 11)
        message(FATAL_ERROR "Nextion RX pin can not be set to pin 6-11 (used by onboard flash).")
    endif()

    if (CONFIG_NEXTION_TX_PIN GREATER_EQUAL 6 AND CONFIG_NEXTION_TX_PIN LESS_EQUAL 11)
        message(FATAL_ERROR "Nextion TX pin can not be set to pin 6-11 (used by onboard flash).")
    endif()

    if (NOT CONFIG_ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS)
        if (CONFIG_NEXTION_RX_PIN LESS 4 OR CONFIG_NEXTION_RX_PIN EQUAL 5 OR CONFIG_NEXTION_RX_PIN EQUAL 12 OR CONFIG_NEXTION_RX_PIN EQUAL 15)
            message(FATAL_ERROR "Nextion RX pin should not use GPIO 0-3, 5, 12, 15. These are reserved pins.")
        endif()

        if (CONFIG_NEXTION_TX_PIN LESS 4 OR CONFIG_NEXTION_TX_PIN EQUAL 5 OR CONFIG_NEXTION_TX_PIN EQUAL 12 OR CONFIG_NEXTION_TX_PIN EQUAL 15)
            message(FATAL_ERROR "Nextion TX pin should not use GPIO 0-3, 5, 12, 15. These are reserved pins.")
        endif()
    endif()

    if (CONFIG_STATUS_LED)
        if (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_NEXTION_RX_PIN)
            message(FATAL_ERROR "Status LED data pin and Nextion RX pin must be unique.")
        endif()

        if (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_NEXTION_TX_PIN)
            message(FATAL_ERROR "Status LED data pin and Nextion TX pin must be unique.")
        endif()
    endif()
    if (CONFIG_GPIO_S88)
        if (CONFIG_GPIO_S88_CLOCK_PIN EQUAL CONFIG_NEXTION_RX_PIN)
            message(FATAL_ERROR "S88 Clock pin and Nextion RX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_LOAD_PIN EQUAL CONFIG_NEXTION_RX_PIN)
            message(FATAL_ERROR "S88 Load pin and Nextion RX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_RESET_PIN EQUAL CONFIG_NEXTION_RX_PIN)
            message(FATAL_ERROR "S88 Reset pin and Nextion RX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_CLOCK_PIN EQUAL CONFIG_NEXTION_TX_PIN)
            message(FATAL_ERROR "S88 Clock pin and Nextion TX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_LOAD_PIN EQUAL CONFIG_NEXTION_TX_PIN)
            message(FATAL_ERROR "S88 Load pin and Nextion TX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_RESET_PIN EQUAL CONFIG_NEXTION_TX_PIN)
            message(FATAL_ERROR "S88 Reset pin and Nextion TX pin must be unique.")
        endif()
    endif()
    if (CONFIG_LOCONET)
        if (CONFIG_LOCONET_RX_PIN EQUAL CONFIG_NEXTION_RX_PIN)
            message(FATAL_ERROR "LocoNet RX pin and Nextion RX pin must be unique.")
        endif()
        if (CONFIG_LOCONET_RX_PIN EQUAL CONFIG_NEXTION_TX_PIN)
            message(FATAL_ERROR "LocoNet RX pin and Nextion TX pin must be unique.")
        endif()
        if (CONFIG_LOCONET_TX_PIN EQUAL CONFIG_NEXTION_RX_PIN)
            message(FATAL_ERROR "LocoNet TX pin and Nextion RX pin must be unique.")
        endif()
        if (CONFIG_LOCONET_TX_PIN EQUAL CONFIG_NEXTION_TX_PIN)
            message(FATAL_ERROR "LocoNet TX pin and Nextion TX pin must be unique.")
        endif()
    endif()
    if (CONFIG_HC12)
        if (CONFIG_HC12_RX_PIN EQUAL CONFIG_NEXTION_RX_PIN)
            message(FATAL_ERROR "HC12 RX pin and Nextion RX pin must be unique.")
        endif()
        if (CONFIG_HC12_RX_PIN EQUAL CONFIG_NEXTION_TX_PIN)
            message(FATAL_ERROR "HC12 RX pin and Nextion TX pin must be unique.")
        endif()
        if (CONFIG_HC12_TX_PIN EQUAL CONFIG_NEXTION_RX_PIN)
            message(FATAL_ERROR "HC12 TX pin and Nextion RX pin must be unique.")
        endif()
        if (CONFIG_HC12_TX_PIN EQUAL CONFIG_NEXTION_TX_PIN)
            message(FATAL_ERROR "HC12 TX pin and Nextion TX pin must be unique.")
        endif()
    endif()
endif()

###############################################################################
# HC12 radio interface validations
###############################################################################

if (CONFIG_HC12)
    if (CONFIG_HC12_RX_PIN EQUAL CONFIG_HC12_TX_PIN)
        message(FATAL_ERROR "HC12 RX pin and HC12 TX pin must be unique.")
    endif()

    if (CONFIG_HC12_RX_PIN GREATER_EQUAL 6 AND CONFIG_HC12_RX_PIN LESS_EQUAL 11)
        message(FATAL_ERROR "HC12 RX pin can not be set to pin 6-11 (used by onboard flash).")
    endif()

    if (CONFIG_HC12_TX_PIN GREATER_EQUAL 6 AND CONFIG_HC12_TX_PIN LESS_EQUAL 11)
        message(FATAL_ERROR "HC12 TX pin can not be set to pin 6-11 (used by onboard flash).")
    endif()

    if (NOT CONFIG_ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS)
        if (CONFIG_HC12_RX_PIN LESS 4 OR CONFIG_HC12_RX_PIN EQUAL 5 OR CONFIG_HC12_RX_PIN EQUAL 12 OR CONFIG_HC12_RX_PIN EQUAL 15)
            message(FATAL_ERROR "HC12 RX pin should not use GPIO 0-3, 5, 12, 15. These are reserved pins.")
        endif()

        if (CONFIG_HC12_TX_PIN LESS 4 OR CONFIG_HC12_TX_PIN EQUAL 5 OR CONFIG_HC12_TX_PIN EQUAL 12 OR CONFIG_HC12_TX_PIN EQUAL 15)
            message(FATAL_ERROR "HC12 TX pin should not use GPIO 0-3, 5, 12, 15. These are reserved pins.")
        endif()
    endif()

    if (CONFIG_STATUS_LED)
        if (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_HC12_RX_PIN)
            message(FATAL_ERROR "Status LED data pin and HC12 RX pin must be unique.")
        endif()

        if (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_HC12_TX_PIN)
            message(FATAL_ERROR "Status LED data pin and HC12 TX pin must be unique.")
        endif()
    endif()
    if (CONFIG_GPIO_S88)
        if (CONFIG_GPIO_S88_CLOCK_PIN EQUAL CONFIG_HC12_RX_PIN)
            message(FATAL_ERROR "S88 Clock pin and HC12 RX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_LOAD_PIN EQUAL CONFIG_HC12_RX_PIN)
            message(FATAL_ERROR "S88 Load pin and HC12 RX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_RESET_PIN EQUAL CONFIG_HC12_RX_PIN)
            message(FATAL_ERROR "S88 Reset pin and HC12 RX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_CLOCK_PIN EQUAL CONFIG_HC12_TX_PIN)
            message(FATAL_ERROR "S88 Clock pin and HC12 TX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_LOAD_PIN EQUAL CONFIG_HC12_TX_PIN)
            message(FATAL_ERROR "S88 Load pin and HC12 TX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_RESET_PIN EQUAL CONFIG_HC12_TX_PIN)
            message(FATAL_ERROR "S88 Reset pin and HC12 TX pin must be unique.")
        endif()
    endif()
    if (CONFIG_LOCONET)
        if (CONFIG_LOCONET_RX_PIN EQUAL CONFIG_HC12_RX_PIN)
            message(FATAL_ERROR "LocoNet RX pin and HC12 RX pin must be unique.")
        endif()
        if (CONFIG_LOCONET_RX_PIN EQUAL CONFIG_HC12_TX_PIN)
            message(FATAL_ERROR "LocoNet RX pin and HC12 TX pin must be unique.")
        endif()
        if (CONFIG_LOCONET_TX_PIN EQUAL CONFIG_HC12_RX_PIN)
            message(FATAL_ERROR "LocoNet TX pin and HC12 RX pin must be unique.")
        endif()
        if (CONFIG_LOCONET_TX_PIN EQUAL CONFIG_HC12_TX_PIN)
            message(FATAL_ERROR "LocoNet TX pin and HC12 TX pin must be unique.")
        endif()
    endif()
endif()

###############################################################################
# LocoNet interface validations
###############################################################################

if (CONFIG_LOCONET)
    if (CONFIG_LOCONET_RX_PIN GREATER_EQUAL 6 AND CONFIG_LOCONET_RX_PIN LESS_EQUAL 11)
        message(FATAL_ERROR "LocoNet RX pin can not be set to pin 6-11 (used by onboard flash).")
    endif()

    if (CONFIG_LOCONET_TX_PIN GREATER_EQUAL 6 AND CONFIG_LOCONET_TX_PIN LESS_EQUAL 11)
        message(FATAL_ERROR "LocoNet TX pin can not be set to pin 6-11 (used by onboard flash).")
    endif()

    if (NOT CONFIG_ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS)
        if (CONFIG_LOCONET_RX_PIN LESS 4 OR CONFIG_LOCONET_RX_PIN EQUAL 5 OR CONFIG_LOCONET_RX_PIN EQUAL 12 OR CONFIG_LOCONET_RX_PIN EQUAL 15)
            message(FATAL_ERROR "LocoNet RX pin should not use GPIO 0-3, 5, 12, 15. These are reserved pins.")
        endif()

        if (CONFIG_LOCONET_TX_PIN LESS 4 OR CONFIG_LOCONET_TX_PIN EQUAL 5 OR CONFIG_LOCONET_TX_PIN EQUAL 12 OR CONFIG_LOCONET_TX_PIN EQUAL 15)
            message(FATAL_ERROR "LocoNet TX pin should not use GPIO 0-3, 5, 12, 15. These are reserved pins.")
        endif()
    endif()

    if (CONFIG_STATUS_LED)
        if (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_LOCONET_RX_PIN)
            message(FATAL_ERROR "Status LED data pin and LocoNet RX pin must be unique.")
        endif()

        if (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_LOCONET_TX_PIN)
            message(FATAL_ERROR "Status LED data pin and LocoNet TX pin must be unique.")
        endif()
    endif()
    if (CONFIG_GPIO_S88)
        if (CONFIG_GPIO_S88_CLOCK_PIN EQUAL CONFIG_LOCONET_RX_PIN)
            message(FATAL_ERROR "S88 Clock pin and LocoNet RX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_LOAD_PIN EQUAL CONFIG_LOCONET_RX_PIN)
            message(FATAL_ERROR "S88 Load pin and LocoNet RX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_RESET_PIN EQUAL CONFIG_LOCONET_RX_PIN)
            message(FATAL_ERROR "S88 Reset pin and LocoNet RX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_CLOCK_PIN EQUAL CONFIG_LOCONET_TX_PIN)
            message(FATAL_ERROR "S88 Clock pin and LocoNet TX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_LOAD_PIN EQUAL CONFIG_LOCONET_TX_PIN)
            message(FATAL_ERROR "S88 Load pin and LocoNet TX pin must be unique.")
        endif()
        if (CONFIG_GPIO_S88_RESET_PIN EQUAL CONFIG_LOCONET_TX_PIN)
            message(FATAL_ERROR "S88 Reset pin and LocoNet TX pin must be unique.")
        endif()
    endif()
endif()

###############################################################################
# S88 validations
###############################################################################

if (CONFIG_GPIO_S88)

    if (CONFIG_GPIO_S88_CLOCK_PIN EQUAL CONFIG_GPIO_S88_LOAD_PIN)
        message(FATAL_ERROR "S88 Clock pin and S88 Load pin must be unique.")
    endif()

    if (CONFIG_GPIO_S88_CLOCK_PIN EQUAL CONFIG_GPIO_S88_RESET_PIN)
        message(FATAL_ERROR "S88 Clock pin and S88 Reset pin must be unique.")
    endif()

    if (CONFIG_GPIO_S88_CLOCK_PIN GREATER_EQUAL 6 AND CONFIG_GPIO_S88_CLOCK_PIN LESS_EQUAL 11)
        message(FATAL_ERROR "S88 Clock pin can not be set to pin 6-11 (used by onboard flash).")
    endif()

    if (CONFIG_GPIO_S88_LOAD_PIN EQUAL CONFIG_GPIO_S88_RESET_PIN)
        message(FATAL_ERROR "S88 Load pin and S88 Reset pin must be unique.")
    endif()

    if (CONFIG_GPIO_S88_LOAD_PIN GREATER_EQUAL 6 AND CONFIG_GPIO_S88_LOAD_PIN LESS_EQUAL 11)
        message(FATAL_ERROR "S88 Load pin can not be set to pin 6-11 (used by onboard flash).")
    endif()

    if (CONFIG_GPIO_S88_RESET_PIN GREATER_EQUAL 6 AND CONFIG_GPIO_S88_RESET_PIN LESS_EQUAL 11)
        message(FATAL_ERROR "S88 Reset pin can not be set to pin 6-11 (used by onboard flash).")
    endif()

    if (NOT CONFIG_ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS)
        if (CONFIG_GPIO_S88_CLOCK_PIN LESS 4 OR CONFIG_GPIO_S88_CLOCK_PIN EQUAL 5 OR CONFIG_GPIO_S88_CLOCK_PIN EQUAL 12 OR CONFIG_GPIO_S88_CLOCK_PIN EQUAL 15)
            message(FATAL_ERROR "S88 Clock pin should not use GPIO 0-3, 5, 12, 15. These are reserved pins.")
        endif()

        if (CONFIG_GPIO_S88_LOAD_PIN LESS 4 OR CONFIG_GPIO_S88_LOAD_PIN EQUAL 5 OR CONFIG_GPIO_S88_LOAD_PIN EQUAL 12 OR CONFIG_GPIO_S88_LOAD_PIN EQUAL 15)
            message(FATAL_ERROR "S88 Load pin should not use GPIO 0-3, 5, 12, 15. These are reserved pins.")
        endif()

        if (CONFIG_GPIO_S88_RESET_PIN GREATER_EQUAL 0 AND CONFIG_GPIO_S88_RESET_PIN LESS 4 OR CONFIG_GPIO_S88_RESET_PIN EQUAL 5 OR CONFIG_GPIO_S88_RESET_PIN EQUAL 12 OR CONFIG_GPIO_S88_RESET_PIN EQUAL 15)
            message(FATAL_ERROR "S88 Reset pin should not use GPIO 0-3, 5, 12, 15. These are reserved pins.")
        endif()
    endif()
endif()

###############################################################################
# Thermal Monitor validations
###############################################################################

if (CONFIG_THERMALMONITOR)
    if (CONFIG_OPS_ADC EQUAL CONFIG_THERMALMONITOR_ADC)
        message(FATAL_ERROR "OPS H-Bridge current sense pin and thermal monitor pin must be unique.")
    endif()
    if (CONFIG_PROG_ADC EQUAL CONFIG_THERMALMONITOR_ADC)
        message(FATAL_ERROR "PROG H-Bridge current sense pin and thermal monitor pin must be unique.")
    endif()
endif()
