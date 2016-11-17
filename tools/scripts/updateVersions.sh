#!/usr/bin/sh

source ./setEnv.sh

: ${BLE_SDK_VER:?We need a BLE SDK version}
: ${TIRTOS_VER:?We need a TI-RTOS version}
: ${BIOS_VER:?We need a BIOS version}
: ${TIDRIVERS_VER:?We need a TI Drivers version}
: ${CC26XXWARE_VER:?We need a CC26XXWare version}
: ${CC13XXWARE_VER:?We need a CC26XXWare version}
: ${UIA_VER:?We need a UIA version}

re='([0-9])_([0-9]{2})_([0-9]{2})_([0-9]{2})([_a-z]*)'
[[ ${TIRTOS_VER} =~ ${re} ]]
TIRTOS_VER_DOT=${BASH_REMATCH[1]}.${BASH_REMATCH[2]}.${BASH_REMATCH[3]}.${BASH_REMATCH[4]}${BASH_REMATCH[5]}

#
#  ======== sedFile ========
#  Function to search and replace versions
#  $1 is the file path
#
sedFile()
{
    workFile=$1
    echo "Updating $(basename $workFile) ..."
    sed -i \
        -e "s|ble_sdk_[0-9_]*\(eng\)\?|${BLE_SDK_VER}|g" \
        -e "s|tirtos_cc13xx_cc26xx_[0-9_]*\(eng\)\?|${TIRTOS_VER}|g" \
        -e "s|bios_[0-9_]*\(eng\)\?|${BIOS_VER}|g" \
        -e "s|cc13xxware_[0-9_]*\(eng\)\?|${CC13XXWARE_VER}|g" \
        -e "s|cc26xxware_[0-9_]*\(eng\)\?|${CC26XXWARE_VER}|g" \
        -e "s|tidrivers_cc13xx_cc26xx_[0-9_]*\(eng\)\?|${TIDRIVERS_VER}|g" \
        -e "s|uia_[0-9_]*\(eng\)\?|${UIA_VER}|g" \
        -e "s|com.ti.rtsc.TIRTOSCC13XX_CC26XX:[0-9_\.]*\(eng\)\?|com.ti.rtsc.TIRTOSCC13XX_CC26XX:${TIRTOS_VER_DOT}|g" \
        $workFile
}

printVersions()
{

    echo "/*****************************"
    echo " *       VERSIONS USED       *"
    echo " ****************************/"
    echo "BLE SDK:          ${BLE_SDK_VER}"
    echo "TI-RTOS:          ${TIRTOS_VER}"
    echo "TI-DRIVERS:       ${TIDRIVERS_VER}"
    echo "CC26XXWARE:       ${CC26XXWARE_VER}"
    echo "CC13XXWARE:       ${CC13XXWARE_VER}"
    echo "UIA:              ${UIA_VER}"
}

#
#  ======== main ========
#  Main function
#
main()
{
    files=$(find ../../ -name *.custom_argvars)
    for fil in $files; do sedFile $fil ; done

    files=$(find ../../ -name *.project)
    for fil in $files; do sedFile $fil ; done

    files=$(find ../../ -name *.projectspec)
    for fil in $files; do sedFile $fil ; done

    files=$(find ../../ -name *.md)
    for fil in $files; do sedFile $fil ; done

    printVersions
}

main
