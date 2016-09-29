#!/usr/bin/sh

# Set env vars
TI_ROOT_DIR=C:/ti

export BLE_SDK_VER=ble_sdk_2_02_00_31
export TIRTOS_VER=tirtos_cc13xx_cc26xx_2_18_00_03
export BIOS_VER=bios_6_45_02_31
export TIDRIVERS_VER=tidrivers_cc13xx_cc26xx_2_16_01_13
export CC26XXWARE_VER=cc26xxware_2_23_03_17162
export CC13XXWARE_VER=cc13xxware_2_03_03_17162
export UIA_VER=uia_2_00_05_50

export BLE_SDK_PATH=${TI_ROOT_DIR}/simplelink/${BLE_SDK_VER}
export TIRTOS_PATH=${TI_ROOT_DIR}/${TIRTOS_VER}
export BIOS_PATH=${TIRTOS_PATH}/products/${BIOS_VER}
export TIDRIVERS_PATH=${TIRTOS_PATH}/products/${TIDRIVERS_VER}
export CC26XXWARE_PATH=${TIRTOS_PATH}/products/${CC26XXWARE_VER}
export CC13XXWARE_PATH=${TIRTOS_PATH}/products/${CC13XXWARE_VER}
