## Table of Contents

* [Introduction](#Introduction)
* [System](#System)
* [Setup](#Setup)
* [Create Custom BIN file](#create-custom-bin-file)
* [Run Bootloader](#Run)
* [Runtime](#Runtime)
* [How to work with TI Serial bootloader](#How-to-work-with-TI-Serial-bootloader)

# Introduction

This is a TI provided tool to flash CC2340R5 devices using a Linux system.
The tool comes in a pre-compiled .OUT file as well as a set of C-Files that can be used to customize the Tool. The Linux serial bootloader can be used in production to flash the device as well as for updating CC2340R5 running as a co-processor. Information on CC2340 can be found on <a href = "https://www.ti.com/">ti.com</a>. More detailed information to understand how the CC2340R5 bootloader works can be found in the <a href = " (https://www.ti.com/lit/ug/swcu193/swcu193.pdf)">TRM</a>. 

# System 
The SBL is developed and tested with the system mentioned below:
<table>
  <tbody>
    <tr>
      <th width = 50%>System</th>
      <th>Version</th>
    </tr>
    <tr>
      <td>
      Ubuntu
      </td>
      <td>
        <ul>
         20.4.6 LTS
        </ul>
      </td>
    </tr>
    <td>
      GCC
      </td>
      <td>
        <ul>
         9.4.0
        </ul>
      </td>
    
  </tbody>
</table>

# Setup

The Host is running the SBL is connected to the UART of the CC2340. 

The bootloader will be able to flash the factory image as well as updating the firmware of an already flashed device. For updating the firmware there need to be a mimic implementat that will pull the Bootloader backdoor pin and the device's Reset signal(or Power-Cycle it).

To be able to update or exchange an existing firmware image, it is necessary to use an image that is supporting the Bootloader backdoor pin. This can be implemented using SysConfig.

The TI XDS110 can but must not  be used as USB to UART bridge for host computers.

[SETUP](./TI_CC2340_Linux_SBL/images/Setup.png)

# Create Custom BIN file
In order to run the bootloader with CRC check it is required to have a firmware binary file that has  a pre-calculated CRC. 

In order to create this  several steps have to be performed. 

These steps are described with the gpiostandby example below: 

Flash the device with the the gpiostandby example
Run Uniflash and open a session for CC2340 
Go to:  >>Memory and press Read memory
To create CCFG file binary:
<ul>
          <li>Press export memory</li>
          <li>Enter 0x4E020000 as start and 0x4E020800 as end address</li>
          <li>Set destination file by pressing browse</li>
          <li>Press generate</li>
</ul>
To create the firmware binary file: 
<ul>
          <li>Press export memory </li>
          <li>Enter start address: 0x00000000 and end address which is aligned with sector size (2048 Bytes) </li>
          <li>set destination file by pressing browse</li>
          <li>press generate</li>
</ul>

You the binary files you can use to test the bootloader afterwards.

# Run
The SBL.out function will take a total number of 3 arguments. 

The function can be called using: 

**SBL.out (SerialPort) (Firmware) (CCFG)**

In Input values are: 

**SerialPort**: Name of the Serial Port that is connected to CC2340

**Firmware**: Path to the firmware binary file

**CCFG:** Path to the CCFG file

A further way to call the function is

**SBL.out -help**

This will display a short help function in the terminal. The function will open a UART Port with a Baud Rate of 1152000 Baud and start check the input files first.
The program will calculate and update CCFG  CRC 32 section.

Afterwards the device will start flashing the firmware image starting from flash address 0x0000000 and the CCFG.bin file to the address: 0x4E020000.

To data corruption and further issues the device will compare the CRC 32 from the file as well as the CRC that is calculated and stored by CC2340 itself.

As the CRC32 operation is sector aligned it is urgently necessary that the created BIN file is sector aligned (2048Byte).

# Runtime
We offer a bootloader with some possibilities of error detection.

This will offer a certain runtime however it is possible to optimize the Runtime.

This can be be done with different measures.

The provided code will check the status of each flash command using the BLDR_CMD_GET_STATUS command. This provides a way to stop writing to the flash after a step failed.

Removing this step will take away the opportunity to react to any issue occuring turing the write process. 

This can be done by setting the 

pvTransfer[x].bExpectAck = false

Corrupted flash content will be detect by the CRC anyways. There might be a time loss while waiting until flash is complete after a failing command.

A further point we used to reduce the bootloader runtime is removing the

setProgress() as this takes some computation steps on the PC and delays sending the next section.

Some Runtimes measured by TI are shown in the table below.

<table>
  <tbody>
    <tr>
      <th width = 25%>Image size</th>
      <th width = 25%>Setup</th>
      <th width = 25%>Baudrate</th>
      <th width = 25%>Runtime</th>
    </tr>
    <tr>
      <td>
      12 KB
      </td>
      <td>
      Standard (as published)
      </td>
      <td>
      1152000 Baud
      </td>
      <td>
         6s
      </td>
    </tr>
        <tr>
      <td>
      512 KB
      </td>
      <td>
      Standard (as published)
      </td>
      <td>
      1152000 Baud
      </td>
      <td>
         126s
      </td>
          <tr>
      <td>
      12 KB
      </td>
      <td>
      fully optimized
      </td>
      <td>
      1152000 Baud
      </td>
      <td>
         64 s
      </td>
    </tr>
    </tr>
    
    
  </tbody>
</table>

# How to work with TI Serial bootloader
The Serial Boot loader is offering a prebuilt SBL.out file which can be used to run the pre-compiled image.

In order to optimize the bootloader for your application it is possible to change the C-Code provided within the Source folder. 

The SBL consists mainly of the a set of .c and .h files: 
<ul>
          <li>Linux_Serial.c | Linux_Serial.h </li>
          <li>main.c</li>
          <li>myFile.c|myFile.h</li>
          <li>sbl_device.c|sbl_device.h</li>
          <li>sbl_device_cc2340.c|sbl_device_cc2340.h</li>
          <li>sbl_device.c|sbl_device.h</li>
</ul>
These files can be used to adapt the bootloader to custom applications.

An example change is: 

Adapt baudrate: 
This  can be done following the following steps: 
<ul>
          <li>Add the desired Baudrate to the baudSet_t enumeration in Linux_Serial.h</li>
          <li>Add case in setBaudRate() function in Linux_Serial.c</li>
          <li>Change setBaudRate() in configPort function</li>
</ul>
To built the project after changing it is possible to use the makefile which is provided in the
TI_CC2340_Linux_SBL folder. This can be done used by calling "make" from the Linux terminal. 
