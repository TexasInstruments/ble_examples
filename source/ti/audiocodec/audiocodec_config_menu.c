#include <bcomdef.h>
#include <ti/display/Display.h>

#include <menu/two_btn_menu.h>
#include "audiocodec_config_menu.h"
#include <ti/audiocodec/audiocodec.h>
#include <profiles/audio_dle/audio_duplex.h>


extern Display_Handle dispHandle;

/*
 * Menu Lists Initializations
 */

// Menu: Audio Stream Options
// upper: Main
MENU_OBJ(audiocodecConfigMenu, "Audio Codec Config", 4, &menuMain)
  MENU_ITEM_ACTION("Set input: MIC_ONBOARD",    AudioCodecMenu_doSetIO)
  MENU_ITEM_ACTION("Set input: LINE_IN",        AudioCodecMenu_doSetIO)
  MENU_ITEM_ACTION("Set output: LINE_OUT",      AudioCodecMenu_doSetIO)
  MENU_ITEM_ACTION("Set output: HEADPHONE",     AudioCodecMenu_doSetIO)
MENU_OBJ_END



/*********************************************************************
 * @fn      AudioCodecMenu_doSetIO
 *
 * @brief   Set audio input/output options
 *
 * @param   index - The index of the option the user selected
 *
 * @return  always true
 */
bool AudioCodecMenu_doSetIO(uint8 index)
{
  // Open the display if the app has not already done so
  if ( !dispHandle )
  {
    dispHandle = Display_open(Display_Type_ANY, NULL);
  }

  /*
   * Note the prints in this function will use the 10th App row, the app
   * must be coordinated with this
   */
  switch(index)
  {
    case 0:
      // Set input to onboard MIC
      AudioDuplex_configureIO(AUDIO_CODEC_MIC_ONBOARD,
                              AUDIO_DUPLEX_OUTPUT_OPTION);

      Display_printf(dispHandle, (TBM_ROW_APP + 10), 0,
                      "Codec input switched to onboard mic");
      break;
    case 1:
      // Set input to LINE IN
      AudioDuplex_configureIO(AUDIO_CODEC_MIC_LINE_IN,
                              AUDIO_DUPLEX_OUTPUT_OPTION);

      Display_printf(dispHandle, (TBM_ROW_APP + 10), 0,
                      "Codec input switched to line in");
      break;
    case 2:
      // Set output to LINE OUT
      AudioDuplex_configureIO(AUDIO_DUPLEX_INPUT_OPTION,
                              AUDIO_CODEC_SPEAKER_LO);

      Display_printf(dispHandle, (TBM_ROW_APP + 10), 0,
                      "Codec output switched to line out");
      break;
    case 3:
      // Set output to Headphone
      AudioDuplex_configureIO(AUDIO_DUPLEX_INPUT_OPTION,
                              AUDIO_CODEC_SPEAKER_HP);

      Display_printf(dispHandle, (TBM_ROW_APP + 10), 0,
                      "Codec input switched to heaphone");
      break;

    default:
      break;
  }

  // Since configuring is a 1 shot, go back to home
  tbm_goTo(&menuMain);
  return (true);
}
