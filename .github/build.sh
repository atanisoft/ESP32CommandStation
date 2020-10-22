#!bin/bash

ESP32CS_TARGET=$1
RUN_DIR=$PWD

export IDF_PATH=${GITHUB_WORKSPACE}/../esp-idf
TOOLCHAIN_DIR=${GITHUB_WORKSPACE}/toolchain
BUILD_DIR=${RUN_DIR}/build
BINARIES_DIR=${RUN_DIR}/binaries

# install GCC 8.2.0 toolchain
if [ ! -f "${TOOLCHAIN_DIR}/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc" ]; then
    echo "Toolchain not found in ${TOOLCHAIN_DIR}!"
    exit 1
fi

echo "Adding ${TOOLCHAIN_DIR}/xtensa-esp32-elf/bin to the path"
# add toolchain to the path
export PATH=${TOOLCHAIN_DIR}/xtensa-esp32-elf/bin:${PATH}

# clone ESP-IDF
if [ ! -f "${IDF_PATH}/export.sh" ]; then
    echo "ESP-IDF not found under ${IDF_PATH}!"
    exit 1
fi

python -m pip install -r "${IDF_PATH}/requirements.txt"

if [ -d "${BUILD_DIR}" ]; then
    echo "Cleaning up ${BUILD_DIR}"
    rm -rf "${BUILD_DIR}"
fi

mkdir -p "${BUILD_DIR}" "${BUILD_DIR}/config"

# generate config.env file for confgen.py and cmake
echo "Generating ${BUILD_DIR}/config.env"
cat > "${BUILD_DIR}/config.env" <<CONFIG_ENV_EOF
{
    "COMPONENT_KCONFIGS": "$(find ${IDF_PATH}/components -name Kconfig -printf '%p ')",
    "COMPONENT_KCONFIGS_PROJBUILD": "${RUN_DIR}/main/Kconfig.projbuild $(find ${IDF_PATH} -name Kconfig.profjbuild -printf '%p ')",
    "COMPONENT_SDKCONFIG_RENAMES": "$(find ${IDF_PATH}/components -name sdkconfig.rename -printf '%p ')",
    "IDF_CMAKE": "y",
    "IDF_TARGET": "esp32",
    "IDF_PATH": "${IDF_PATH}"
}
CONFIG_ENV_EOF

# create default sdkconfig
export IDF_TARGET=esp32
SDKCONFIG_DEFAULTS="--defaults ${RUN_DIR}/sdkconfig.defaults"
if [ "${ESP32CS_TARGET}" == "ESP32CommandStation.pcb" ]; then
    SDKCONFIG_DEFAULTS="${SDKCONFIG_DEFAULTS} --defaults ${RUN_DIR}/sdkconfig.defaults.pcb"
fi
echo "Generating default sdkconfig"
python "${IDF_PATH}/tools/kconfig_new/confgen.py" \
    --kconfig "${IDF_PATH}/Kconfig" \
    --config "${RUN_DIR}/sdkconfig" \
    --sdkconfig-rename "${IDF_PATH}/sdkconfig.rename" \
    ${SDKCONFIG_DEFAULTS} \
    --env-file "${BUILD_DIR}/config.env" \
    --output header "${BUILD_DIR}/config/sdkconfig.h" \
    --output cmake "${BUILD_DIR}/config/sdkconfig.cmake" \
    --output json "${BUILD_DIR}/config/sdkconfig.json" \
    --output json_menus "${BUILD_DIR}/config/kconfig_menus.json" \
    --output config "${RUN_DIR}/sdkconfig"
if [ $? -ne 0 ]; then
    echo "sdkconfig generation failed!"
    exit 1
fi

cat "${RUN_DIR}/sdkconfig"

echo "Starting build..."

# build via cmake/ninja
cd "${BUILD_DIR}" && cmake "${RUN_DIR}" -G Ninja && ninja

# print size information
python "${IDF_PATH}/tools/idf_size.py" "${BUILD_DIR}/ESP32CommandStation.map"

mkdir -p "${BINARIES_DIR}"
cat > "${BINARIES_DIR}/readme.txt" << README_EOF
The binaries can be sent to the ESP32 via esptool.py similar to the following:
python esptool.py -p (PORT) -b 460800 --before default_reset --after hard_reset
    write_flash 0x1000 bootloader.bin
    write_flash 0x8000 partition-table.bin
    write_flash 0xe000 ota_data_initial.bin
    write_flash 0x10000 ESP32CommandStation.bin

Note: The esptool.py command above should be all on one line.

If you prefer a graphical utility for flashing you can use the Flash Download Tool
available from https://www.espressif.com/en/support/download/other-tools to write
the four binary files listed above at the listed offsets.
README_EOF

cp "${BUILD_DIR}/partition_table/partition-table.bin" \
    "${BUILD_DIR}/ota_data_initial.bin" \
    "${BUILD_DIR}/bootloader/bootloader.bin" \
    "${BUILD_DIR}/ESP32CommandStation.bin" \
    "${BINARIES_DIR}"
