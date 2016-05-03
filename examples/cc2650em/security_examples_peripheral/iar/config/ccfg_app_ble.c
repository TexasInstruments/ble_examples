/******************************************************************************

 @file  ccfg_app_ble.c

 @brief Customer Configuration CC26xx PG2 device family.

 Group: WCS, BTS
 $Target Device: DEVICES $

 ******************************************************************************
 $License: BSD3 2014 $
 ******************************************************************************
 $Release Name: PACKAGE NAME $
 $Release Date: PACKAGE RELEASE DATE $
 *****************************************************************************/

//
//       ===> READ THIS BEFORE MODIFYING THIS FILE
//
//
//       ===> READ THIS BEFORE MODIFYING THIS FILE
//
//
//       ===> READ THIS BEFORE MODIFYING THIS FILE
//

// The customer configuration area (ccfg section) is located at the end of the 
// flash and reflect the hw configuration of the device. it is very important 
// that it remains align with the version of driverlib you are using.
// all BLE project except sensor tag use the same configuration.
// Keeping the "#include <startup_files/ccfg.c>" guarantee that your project using 
// driverlib and the ccfg area will be align.

// you can modify it if you want, the recommend way will be to remove the 
// bellow include, copy the content of the <startup_files/ccfg.c> file in this
// file and rebuild.

// ==> KEEP IN MIND that if you do so, be sure that any further update of the 
// driverlib must be align with your modified version of ccfg area.
#include <startup_files/ccfg.c>
