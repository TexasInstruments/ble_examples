#!/usr/bin/sh

VERSIONFILE=versions.txt

#
#  ======== sedFile ========
#  Function to search and replace versions
#  $1 is the file path
#
sedFile()
{
    workFile=$1
    echo "Updating $(basename $workFile) ..."
    sed -i -f $VERSIONFILE $workFile
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
}

main

