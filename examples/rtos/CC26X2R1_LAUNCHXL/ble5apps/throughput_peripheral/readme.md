# Bluetooth 5 Throughput Peripheral

## Functional Overview

This project is meant to be used on a SimpleLink CC26x2R1 LaunchPad, connected to 
a CC26x2R1 LaunchPad running the Bluetooth 5 Throughput Central example, or a 
phone running the SimpleLink Starter app. The purpose of this demo is to 
demonstrate what data throughput can be achieved using TI SimpleLink Bluetooth 5 
devices.

This project is based on the Simple Peripheral sample application from the 
BLE5-Stack component of the SimpleLink CC13x2 / CC26x2 software development kit (SDK). 
Once a connection has been established with a peer device, the user can adjust 
desired physical layer (PHY), packet length (PDU) and toggle throughput.

In addition to CC26x2-to-CC26x2 communication with the Throughput Central,
the Throughput Peripheral can be connected to a phone using the SimpleLink
Starter app. When connected, the user can run the throughput demo, transferring
data to the phone. The PHY, PDU size and connection interval used can be 
controlled through the SimpleLink Starter app interface. 

The Peripheral generates data and sends it to the Central where an average and an
instant throughput rate are calculated and shown.

The following changes have been made to the Simple Peripheral example:
- Added support for the custom profiles Throughput Service and Connection Control Service (CCService)
- Changed default MTU size
- Added function to generate data and send the data as GATT_Notifications (ThroughputPeripheral_sendData())

## Project Hardware
- 1 CC26x2R1 Launchpad (http://www.ti.com/tool/LAUNCHXL-CC26X2R1)
- 1 UART Terminal on PC or 1 BOOSTXL-SHARP128 LCD boosterpack (http://www.ti.com/tool/BOOSTXL-SHARP128)

To use the Launchpad with the LCD boosterpack, add the following predefined 
symbols:
	
	BOARD_DISPLAY_USE_LCD=1
	BOARD_DISPLAY_USE_UART=0
  BOARD_DISPLAY_USE_UART_ANSI=0

## Two Button Menu

The Two Button Menu system is designed to utilize the two push buttons available
on the CC26X2R1 LaunchPad. The left button (BTN1) is always used to cycle
between options. The right button (BTN2) is used to activate the selected action
or item.

The Two Button Menu is set up by default to utilize the back channel UART
present on the LaunchPad as its display.

## Running the Demo on a LaunchPad

1.	Compile and load the project. Build and load `ble5_throughput_peripheral_cc26x2r1lp_app`.
	
2.  Connect to the Launchpad via PuTTY (or some other serial console emulator).
  - Alternatively follow the steps to use an LCD boosterpack.

3.	After connecting to the device with the serial console, the Two Button Menu
    will appear. By default, when not connected to a peer, no menu options are 
	  available and "Initialized" will be displayed.

4. 	After a connection has been formed between the Throughput Peripheral and a 
    Throughput Central, "Link Param Updated" will appear indicating that the
	  connection interval has been updated. No menu options are enabled at this 
	  point.

5.	After "Work With" has been selected on the Throughput Central, the PHY and 
	  MTU will be displayed along with the current connection interval.
	  The menu option for "Work With" will appear, allowing for the PHY to be 
	  changed from the Peripheral along with a local Throughput Toggle. Note that
	  this Throughput Toggle does not stop the instant and average rate displayed
	  on the Central from updating. Any PHY, PDU and connection interval and 
	  throughput toggle sent from the Central will be displayed on the Peripheral.
	
6.	The peripheral device can not initiate a disconnect, but will process a 
    GAP_LINK_TERMINATED event sent by the Central.
	

## Running the Demo on a Phone using the SimpleLink Starter App

1. 	Compile and load the project
    - First build `ble5_throughput_peripheral_cc26x2r1lp_stack`
    - Next build and load `ble5_throughput_peripheral_cc26x2r1lp_app`

2.	Connect to the device using the SimpleLink Starter app. The advertised name
	  of the Peripheral is "Throughput Periph". To connect, tap on the entry
	  for the peripheral and select Sensor View.
	
3.	The phone will attempt to connect to the Peripheral. The devices will 
	  exchange MTU and attempt to update the connection interval to the lowest 
    permitted value. For iOS, the lowest connection interval possible is 15 ms, 
    while for Android-devices the minimum is 7.5 ms. **Note that the allowed 
    connection interval depends on OS-version and the BLE-stack of the phone.**
	
4.	Once connected to the Peripheral, the SimpleLink Starter app will display 
    the Throughput Service and the Connection Control Service. In the Throughput 
    Service, the PHY and PDU can be adjusted, and the throughput test can be 
    toggled on and off. Under the Connection Control Service, the connection 
    interval can be adjusted. Note that this is not supported on some devices. 
	
5. 	To start the Throughput Demo, press the button "Enable Throughput Demo". 
	  A graph will appear which displays the throughput. In addition, the 
	  "Current Speed" and "Packets/second" fields will update.
	
6. 	To disconnect from the Peripheral, tap the "SimpleLink Starter" text at the
	  top left.
	
## Purpose / Scope

This example will demonstrate the maximum BLE throughput that can be achieved 
with a Texas Instruments SimpleLink Bluetooth 5 MCU to either a CC26X2R1 LaunchPad
or a phone. The following assumptions have been made to achieve the throughput 
displayed in this test:

- Minimal overhead associated with sending notifications. The operations 
  performed for each packet on both the Central and Peripheral are minimal and 
  don't affect performance substantially. Additional overhead can result in 
  decreased throughput.
- The order of the received packets are in the same order as they were sent.
- No encryption is being used. The additional computation required from 
  encrypting/decrypting might reduce the throughput.
  
The throughput being measured is actual usable payload GATT data, e.g. 244 B 
when PDU length = 251 B. Any LL / L2CAP headers are not considered part of the 
data throughput. The GATT architecture used is such that the GATT server 
(throughput peripheral) is sending GATT notifications to the GATT client.

# Parameters

The basic idea is to be constantly sending GATT notifications, with as
little overhead as possible, and as little downtime as possible. The
following parameters must be considered. 

## ATT_MTU Size

In order to minimize L2CAP header overhead, the maximum ATT_MTU size is being
used: 251 bytes. This will result in 244 byte notifications being sent. 

**Note: Sending 244 byte notification packets requires that the Bluetooth 
4.2 Data Length Extensions feature is supported by both devices.**

This means that there is a 7 byte L2CAP overhead for every 244 bytes sent. In order
to achieve this, both the throughput peripheral (TP) and the throughput central 
(TC) projects must set the following defines in the .opt-file of the currently 
active build configuration. (This file can be found in the in TOOLS/defines 
folder):


```	
	-DMAX_NUM_PDU=6
	-DMAX_PDU_SIZE=255
```

This will allocate 6 Tx/Rx buffers of 251 bytes in the heap. A custom application 
will need to be profiled to verify that there is enough heap for the desired
PDU / PDU size combination.

If there is not, then the MAX_NUM_PDU define can be decreased, possibly
causing a loss of throughput. Since the actual (worst case) memory used is a
product of the MAX_NUM_PDU & MAX_PDU_SIZE, the system designer will need to
balance these parameters according to available memory and capability of the
peer device. For example, when interfacing to an iOS8+ device, a
MAX_PDU_SIZE=162 and MAX_NUM_PDU=6 would match the iOS device's ATT_MTU of
158 and up to 6 packets per connection event. These parameters may vary from a
given iOS release or device.

Note that the max PDU size is not selected to be 255 (max supported by host).
This has to do with the maximum data payload supported by the data length
extension feature in the controller. Data length extension's role in the
throughput study is covered in depth in the next section.
	
## LE 2M, Coded S=2 and Coded S=8 PHY Support

This demo supports use of the Bluetooth 5 LE 2M PHY, LE Coded PHY S=2 and 
LE Coded PHY S=8. LE 2M PHY effectively doubles the number of symbols per 
seconds, and thus bits per second sent from the device during each connection 
event. The Coded PHYs allow for higher throughput over greater distances than 
LE 1M and LE 2M PHYs by using encoding and Forward Error Correction. This comes 
at the cost of the maximum PHY speed, which is reduced to 500 kbps and 125 kbps 
for Coded S=2 and Coded S=8 respectively.

**CC26X2R1 supports all Bluetooth 5 PHYs (LE 1M, LE 2M and LE Coded). PHY support 
in smartphones varies, and you should check your specific device before trying to 
update to a Bluetooth 5 PHY.**

## LE Controller Data Payload / LE Data Length Extension

The Bluetooth 4.2 specification allows the controller to send data packets
containing up to 251 bytes of application data in a single packet. This is a
drastic increase when compared to the previous specifications which limited all
controller data payloads to be 27 bytes. This feature is called data length
extension. Please refer to the chapter on "LE Data Length Extension" in the 
BLE5-Stack User's Guide for more information:
(http://dev.ti.com/tirex/explore/node?node=AOimuSWjap.4RuDbcp7OqA__pTTHBmu__LATEST)

Data length extensions (DLE) is supported by SimpleLink CC26X2R1 and CC13x2 
devices, and CC2650/CC2640R2 devices running the TI BLE5-Stack as well as TI 
BLE-Stack. This throughput example uses Data Length Extensions.

With DLE enabled, the LE controller can spend more time sending application
data and less time processing packet overhead, thus increasing the data 
throughput.

In order to optimize the payload transmit between the two devices, the ATT_MTU 
is limited to be 251 bytes (instead of 255). As mentioned above, the largest data payload
supported by the controller is 251 bytes. Setting ATT_MTU to be 255 would cause
the controller to have to fragment and recombine host data packets at the link
layer. This will negatively affect throughput. This is because there is overhead
for each data packet sent. See the packet overhead section for more details.

Note that not all 4.2 devices will support DLE. Throughput to mobile devices
may be limited based on the capabilities of the device's BLE stack.

### Connection Interval

Depending on the amount of post/pre-processing, the controller needs 2-3 ms to
prepare for the next connection event. Therefore, longer connection intervals
allow for higher throughput as there is less downtime where notifications are
not being sent. This example will use a connection interval of 200 ms. Note that
there is a significant downside to using higher intervals in a real world
scenario: missed connection events due to RF interference will drastically
decrease the throughput. It is up to the user to decide what throughput / latency 
trade-off is desired. For LaunchPads, the throughput increase is not significant 
after 100 ms.

When communicating with phones, the connection interval, along with DLE, is a 
major deciding factor when it comes to achieving high throughput. In general,
iPhones require lower connection interval as they only have a specific amount 
of BLE packets that can be received per connection event. On Android devices, 
longer connection interval usually results in higher throughput.

Apple-devices have requirements when it comes to connection parameters, defined 
in the  "Accessory Design Guidelines for Apple Devices, chapter 10" 
(https://developer.apple.com/accessories/Accessory-Design-Guidelines.pdf).

### Packet Overhead

The host and controller data payloads have been optimized to be 251 bytes.
This is the maximum value that makes sense for optimizing throughput. The 
following packet formats are further explained in the 
[Bluetooth Core Spec](https://www.bluetooth.com/specifications/bluetooth-core-specification).

A GATT-Notification packet can be illustrated as follows:

|	Preamble  | Access Address  |		  PDU		        |		CRC		    |
|-----------|-----------------|-------------------|-------------|
|	1 octet   | 	4 octets      |	2 to 257 octets   |	  3 octets  |


And the PDU Format is as follows:
		
|	LL Header  |       Payload		 	|	  MIC	    |
|------------|--------------------|-----------|
|	2 octets   | 	up to 251 octets 	|	4 octets  |


However, not all 251 bytes can be used as application data due to overhead at
the ATT and L2CAP levels. These headers are required by the Bluetooth spec and
cannot be changed. A brief description of this is shown below, and therefore
the PDU Format can be further divided into:

|	L2CAP Header   	|  ATT Header   | 	  Payload Data 	  |
|-----------------|---------------|---------------------|
|	  4 octets     	|	3 octets	    |	up to 244 octets    |


#### ATT Notification Header

All ATT notification packets have a 3 byte header required to identify the
opcode and the handle of the attribute sending the notification.

There is a 3 byte overhead to sending an ATT notification.

#### L2CAP Header

At the L2CAP layer, similar overhead is required to set the length of the packet
and the proper channel identifier (CID). The L2CAP header is included as a part of
the PDU Data payload.

Each of these fields are 16-bits (2 bytes) resulting in 4 bytes of L2CAP
overhead.

The L2CAP Header illustrated as follows:

|	 Length   |  Destination CID  |
|-----------|-------------------|
|	2 octets  | 	2 octets        |

Combining the L2CAP and ATT packet overhead yields:

    TOTAL_PACKET_OVERHEAD = 7 bytes


## Maximum achievable throughput

Even though the PHYs are named 1 Mbps and 2 Mbps, the maximum achievable 
throughput is limited by overhead introduced by the BLE Core specification, 
such as interframe spacing. See
http://blog.bluetooth.com/exploring-bluetooth-5-how-fast-can-it-be for more info 
on calculating throughput and the limitations reducing the maximum achievable throughput. 

Using PDU Size of 251 B and the transmission overhead as explained in the the Bluetooth blog 
post, the maximum achievable throughput can be calculated. For 1M and 2M PHY, 
the maximum achievable throughput is 803 kbps and 1434 kbps. However, this does 
not take into account that 7 B of the PDU size are used for the total packet 
overhead. Therefore, using the actual payload, 244 B, the maximum achievable 
throughput can be calculated to be 780.8kbps and 1394kbps for 1M and 2M PHY, 
respectively.

## Achieved throughput

Using the methods listed in this document, the following throughput was achieved
between a Throughput Central and Throughput Peripheral:

|	PHY     	| Achieved Throughput [kbps] | Theoretical Max [kbps] |
|-----------|----------------------------|------------------------|
|    1M	  	|  	  	780.80	             |  		803               |
|    2M		  |	   1385.92	    	         | 1394                   |
|  Coded S2	|		351.36	                 | 436                    |
|  Coded S8	|		 58.56	      	         | 113                    |

The parameters used were MTU = 251, PDU = 251, Connection Interval = 200 ms
with an RSSI of around -31 dBm when using 2M PHY. 

Increasing the distance, and therefore the RSSI, between the devices will
eventually lead to decreased throughput because of packet retransmission.

## Packet retransmission

If a packet is missed (not received by the peer device), it will be retransmitted 
according to the Bluetooth Core Spec. Missed packets occur due to RF issues, 
either caused by interference or due to the distance between the devices. If the 
CRC check on the Central fails, a request for a retransmission is sent to the 
Peripheral (and vice versa).

### Notification Queuing

The case considered here assumes that the application is able to queue up
notifications quickly enough that there is always a notification
ready to be sent when a slot opens. This is achieved by the application
RTOS task running in an infinite loop in ``throughput_peripheral.c``:

```
/*********************************************************************
 * @fn      ThroughputPeripheral_sendData
 *
 * @brief   Sends ATT notifications in a tight while loop
 *
 * @param   connHandle - connection handle to transmit over
 *
 * @return  none
 */
static void ThroughputPeripheral_sendData(uint16_t connHandle)
{
  uint16_t len = 0;
  attHandleValueNoti_t noti;
  bStatus_t status;
  msg_counter = 0;

  // Subtract the total packet overhead of ATT and L2CAP layer from notification payload
  len = currentMTU - TOTAL_PACKET_OVERHEAD;

  // Get the handle of the throughput service value characteristic
  noti.handle = getThroughput_Service_Data_Handle();
  noti.len = len;
  Util_constructClock(&clkTimeout, ThroughputPeripheral_clockHandler,
                                (connInfo.connTimeout * 10), 0, true,
                                (UArg) &argTimeout);
  while(throughputOn)
  {
    noti.pValue = (uint8 *)GATT_bm_alloc( connHandle, ATT_HANDLE_VALUE_NOTI, GATT_MAX_MTU, &len );

    if ( noti.pValue != NULL ) // if allocated
    {
      // Fill the notification with data
      noti.pValue[0] = (msg_counter >> 24) & 0xFF;
      noti.pValue[1] = (msg_counter >> 16) & 0xFF;
      noti.pValue[2] = (msg_counter >> 8) & 0xFF;
      noti.pValue[3] = msg_counter & 0xFF;

      // Attempt to send the notification w/ no authentication
      status = GATT_Notification( connHandle, &noti, 0x00);
      if ( status != SUCCESS)
      {
        // If noti not sent, free the message. (If it is sent, the BLE-Stack will free the message.)
        GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
      }
      else
      {
        // Notification is successfully sent, increment counters
        msg_counter++;
      }
    }
    else
    {
      // bleNoResources was returned, do nothing
      asm(" NOP ");
    }
    // If RTOS queue is not empty, process app message.
    // We need to process the app message here in the case of a button press
     while (!Queue_empty(appMsgQueueHandle))
    {
      tpEvt_t *pMsg = (tpEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
      if (pMsg)
      {
        // Process message.
        ThroughputPeripheral_processAppMsg(pMsg);

        // Free the space from the message.
        ICall_free(pMsg);
      }
    }
  }
  if (Util_isActive(&clkTimeout))
  {
    Util_stopClock(&clkTimeout);
  }
  // Destruct the clock object
  Clock_destruct(&clkTimeout);
}
```

Due to other processing needs, a custom application may not be able to 
replicate or sustain this throughput (e.g., having to wait for payload data to 
arrive over serial interface). In addition, the sendData function maximizes 
enqueuing of data (Notifications in this example), so it is expected to see
GATT_Notification return a non-SUCCESS status, such as blePending when the
queue is full. When this status is returned, the BLE5-Stack will gradually 
clear the Tx queue as channel conditions permit, thus allowing the application 
to enqueue more GATT Notifications once the queue clears. The depth of the Tx 
queue is determined by the MAX_NUM_PDU define listed above. Under maximum 
throughput conditions, you may expect to see a high number of blePending 
(non-SUCCESS) status results from calling GATT_Notification. 

In this while loop, clkTimeout is used as a replacement for checking whether the 
connection is still running. This is usually handled by the GAP layer where a 
GAP_LINK_TERMINATED event is posted when the timer runs out, but the Peripheral
doesn't react to this as it is busy transmitting notifications in the while-loop.