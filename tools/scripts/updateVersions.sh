#!/usr/bin/sh

source ./setEnv.sh

: ${SIMPLELINK_SDK_VER:?We need a Simplelink SDK version}
: ${XDCTOOLS_VER:?We need a XDCTools version}

re='([0-9])_([0-9]{2})_([0-9]{2})_([0-9]{2})([_a-z]*)'
[[ ${SIMPLELINK_SDK_VER} =~ ${re} ]]
SIMPLELINK_SDK_VER_DOT=${BASH_REMATCH[1]}.${BASH_REMATCH[2]}.${BASH_REMATCH[3]}.${BASH_REMATCH[4]}${BASH_REMATCH[5]}
[[ ${XDCTOOLS_VER} =~ ${re} ]]
XDCTOOLS_VER_DOT=${BASH_REMATCH[1]}.${BASH_REMATCH[2]}.${BASH_REMATCH[3]}.${BASH_REMATCH[4]}${BASH_REMATCH[5]}

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
        -e "s|simplelink_cc26x2_sdk[0-9_]*\(eng\)\?|${SIMPLELINK_SDK_VER}|g" \
        -e "s|\(SDK\s\+\)[0-9\.]\+|\1${SIMPLELINK_SDK_VER_DOT}|g" \
        -e "s|\(name=\"xdcToolsVersion\".value=\"\)[0-9_]*\(eng\)\?(\")|\1${XDCTOOLS_VER_DOT}\s|g" \
        -e "s|com.ti.SIMPLELINK_CC26X2_SDK:[0-9_\.]*\(eng\)\?|com.ti.SIMPLELINK_CC26X2_SDK:${SIMPLELINK_SDK_VER_DOT}|g" \
        $workFile
}

printVersions()
{

    echo "/*****************************"
    echo " *       VERSIONS USED       *"
    echo " ****************************/"
    echo "SIMPLELINK SDK: ${SIMPLELINK_SDK_VER}"
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
