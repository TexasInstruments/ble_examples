Purpose / Scope
===============

This page will document how to demonstrate an end to end full duplex BLE
voice/audio solution using two CC26xx devices.
Emphasis will be placed on the central device which is responsible for
establishing the connection.

Voice data is transferred over BLE using the TI audio\_profile\_dle which is a
Voice Over GATT Profile (VoGP) design. This profile has been augmented to use
data length extension and large MTU.

This profile has been update to add an additional start byte. This new start
byte indicates that the following stream uses mSBC compression.

In this demo, data flows bidirectionally between a streamer (GATT server) and a
receiver (GATT client) device. This means both devices must act as both a GATT
client and a GATT server.

The `simple_peripheral_bidirectional_audio` project is a modified version of
`simple_peripheral` from the BLE-SDK.

Purpose / Scope
===============

The theory and steps required to run the demo are described in the readme
inside the `simple_central_bidirectional_audio` folder of this repo.
