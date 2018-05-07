#include <bcomdef.h>
#include <ti/display/Display.h>

#include <menu/two_btn_menu.h>
#include <ti/audiocodec/audiocodec_config_menu.h>
#include "peripheral_bidirectional_audio_menu.h"


/*
 * Menu Lists Initializations
 */

// Menu: Main
// upper: none
MENU_OBJ(menuMain, NULL, 1, NULL)
  MENU_ITEM_SUBMENU(&audiocodecConfigMenu)
MENU_OBJ_END


