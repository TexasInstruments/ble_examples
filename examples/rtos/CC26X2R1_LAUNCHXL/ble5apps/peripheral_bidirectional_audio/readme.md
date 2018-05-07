Purpose / Scope
===============

This page will document how to demonstrate an end to end full duplex BLE
voice/audio solution using two CC26x2 devices.

In this demo, data flows bidirectionally between a streamer (GATT server) and a
receiver (GATT client) device. This means both devices must act as both a GATT
client and a GATT server.

The `peripheral_bidirectional_audio` project is a modified version of
`simple_peripheral` from the BLE5-Stack component of the CC26x2 SDK.

Purpose / Scope
===============

The theory and steps required to run the demo are described in the readme
inside the `central_bidirectional_audio` folder of this repo.

Changing Audio IO Config
========================

In the bidirectional audio demo, the peripheral device is intended to be
headless and run on a battery. However, there is display and menu support
built in for debugging purposes.

You can use the menu system to change the audio input/output configuration
on the fly. However, these must be configured before audio streaming starts.
Once audio streaming starts, the configuration is locked until streaming stops.

The support options are:

### Input


 - MIC_ONBOARD: use the microphone on the CC3200_AUDBOOST
 - LINE_IN: Use an input device connected to LINE_IN connector on
   CC3200_AUDBOOST

### Output

 - LINE_OUT: Sound is routed to LINE_OUT connector on the CC3200_AUDBOOST,
             line level
 - HEADPHONE: Sound is routed to LINE_OUT connector, headphone level
