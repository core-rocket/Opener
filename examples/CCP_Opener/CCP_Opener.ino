#include <string.h>
#include <CCP_MCP2515.h>
#include "myOpener.h"

#define CAN0_INT 0
#define CAN0_CS 1
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);

MY_OPENER opener(OPENER::FM);

// Other
char msgString[128];

void setup()
{
  pinMode(CAN0_CS, OUTPUT);
  pinMode(CAN0_INT, INPUT);
  digitalWrite(CAN0_CS, HIGH);

  // CAN
  CCP.begin();

  opener.init();
}

void loop()
{
  static uint32_t last_tlm_time_ms = 0;
  if (millis() - last_tlm_time_ms > 100)
  {
    last_tlm_time_ms = millis();
    /*
        if (opener.mode == OPENER::CHECK)
        {
          CCP.string_to_device(CCP_lift_off_judge, "CHECK");
          sprintf(open_judge, "------");
          open_judge[1] = (!prohibitOpen ? 'O' : 'C');
          CCP.string_to_device(CCP_open_judge, open_judge);
          CCP.string_to_device(CCP_opener_source, "SENSEN");
        }
        else
        {
          CCP.string_to_device(CCP_lift_off_judge, lift_off_judge);

          open_judge[0] = ((opener.mode == OPENER::OPENED) ? 'O' : 'C');
          open_judge[1] = (!prohibitOpen ? 'O' : 'C');
          open_judge[2] = (meco_time ? 'O' : 'C');
          open_judge[3] = (meco_ac ? 'O' : 'C');
          open_judge[4] = (vertex_time ? 'O' : 'C');
          open_judge[5] = (vertex_descending ? 'O' : 'C');
          CCP.string_to_device(CCP_open_judge, open_judge);

          opener_source[0] = ((ALTsource == Sensor) ? 'S' : ((ALTsource == FastLogger) ? 'L' : '-'));
          opener_source[1] = ((ALTsource == Sensor) ? 'E' : ((ALTsource == FastLogger) ? 'O' : '-'));
          opener_source[2] = ((ALTsource == Sensor) ? 'N' : ((ALTsource == FastLogger) ? 'G' : '-'));
          opener_source[3] = ((ACCsource == Sensor) ? 'S' : ((ALTsource == FastLogger) ? 'L' : '-'));
          opener_source[4] = ((ACCsource == Sensor) ? 'E' : ((ALTsource == FastLogger) ? 'O' : '-'));
          opener_source[5] = ((ACCsource == Sensor) ? 'N' : ((ALTsource == FastLogger) ? 'G' : '-'));
          CCP.string_to_device(CCP_opener_source, opener_source);
        }
        */
  }

  static float acceleration_mss = 0;
  static float altitude_m = 0;
  static bool get_acc_data = false;
  static bool get_alt_data = false;
  if (!digitalRead(CAN0_INT)) // データ受信確認
  {
    CCP.read_device();
    switch (CCP.id)
    {
    case CCP_parachute_fuse:
      if (CCP.str_match("CLEAR", 5)){
        opener.clear_prohibitOpen();
      }
      else{
        opener.prohibitOpen();
      }
      break;
    case CCP_parachute_control:
      {
        if (opener.mode == OPENER::CHECK)
        {
          if (CCP.str_match("OPEN", 4))
            opener.manualOpen();
          if (CCP.str_match("CLOSE", 5))
            opener.manualClose();
        }
      }
      break;
    case CCP_opener_control:
      {
        if (CCP.str_match("CHECK", 5))
          opener.goCHECK();
        if (CCP.str_match("READY", 5))
          opener.goREADY();
      }
      break;
    case CCP_A_accel_mss:
      acceleration_mss = CCP.data_fp16_2();
      get_acc_data = true;
      break;
    case CCP_A_pressure_altitude_m:
      altitude_m = CCP.data_float();
      get_alt_data = true;
      break;
    case CCP_open_time_s:
      opener.set_open_threshold_time_ms(CCP.data_float() * 1000);
      delay(200);
      CCP.float_to_device(CCP_open_time_repeat_s, (float)opener.get_open_threshold_time_ms() / 1000.0);
      break;
    default:
      break;
    }
  }

  if (get_acc_data && get_alt_data)
  {
    get_acc_data = false;
    get_alt_data = false;
    opener.opener_100Hz(acceleration_mss, altitude_m);
  }
}
