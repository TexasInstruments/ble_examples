#include <bcomdef.h>
#include <ti/display/Display.h>
#include <menu/two_btn_menu.h>
#include "central_bidirectional_audio_menu.h"
#include "central_bidirectional_audio.h"
#include <ti/audiocodec/audiocodec_config_menu.h>

/*
 * Menu Lists Initializations
 */

// Menu: Main
// upper: none
MENU_OBJ(menuMain, "Central Bidirectional Audio", 4, NULL)
  MENU_ITEM_ACTION("Start scanning",    CentralAudio_doScanAction)
  MENU_ITEM_SUBMENU(&caMenuAudioStream)
  MENU_ITEM_SUBMENU(&audiocodecConfigMenu)
  MENU_ITEM_SUBMENU(&caMenuPhySelect)
MENU_OBJ_END

// Menu: Audio Stream Options
// upper: Main
MENU_OBJ(caMenuAudioStream, "Audio Stream Options", 3, &menuMain)
  MENU_ITEM_ACTION("Start streaming: ADPCM",    CentralAudio_doStartStream)
  MENU_ITEM_ACTION("Start streaming: mSBC",     CentralAudio_doStartStream)
  MENU_ITEM_ACTION("Stop streaming",            CentralAudio_doStopStream)
MENU_OBJ_END

// Menu: PHY Preferences
// upper: Main
MENU_OBJ(caMenuPhySelect, "Select PHY (scan, connect, stream)", 3, &menuMain)
  MENU_ITEM_ACTION("Select PHY: 1M",            CentralAudio_doSetPhy)
  MENU_ITEM_ACTION("Select PHY: 2M",            CentralAudio_doSetPhy)
  MENU_ITEM_ACTION("Select PHY: Coded S=2",     CentralAudio_doSetPhy)
MENU_OBJ_END
