#!/usr/bin/sh

VERSIONFILE=versions.txt
files=$(find ../../ -name *.custom_argvars)

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
    for fil in $files; do sedFile $fil ; done
}

main

