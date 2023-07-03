#include <string.h>
#include <CCP_MCP2515.h>
#include "myOpener.h"

#define CAN0_INT 0
#define CAN0_CS 1
CCP_MCP2515 ccp_CAN(CAN0_CS, CAN0_INT);

MY_OPENER opener(OPENER::FM);

void setup()
{
  pinMode(CAN0_CS, OUTPUT);
  pinMode(CAN0_INT, INPUT);
  digitalWrite(CAN0_CS, HIGH);

  // CAN
  ccp_CAN.begin();

  opener.init();
}

void loop()
{
  static float acceleration_mss = 0;
  static float altitude_m = 0;
  static bool get_acc_data = false;
  static bool get_alt_data = false;
  if (!digitalRead(CAN0_INT)) // データ受信確認
  {
    ccp_CAN.read_device();
    switch (ccp_CAN.id)
    {
    case CCP_parachute_fuse:
      if (ccp_CAN.str_match("CLEAR", 5)){
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
          if (ccp_CAN.str_match("OPEN", 4))
            opener.manualOpen();
          if (ccp_CAN.str_match("CLOSE", 5))
            opener.manualClose();
        }
      }
      break;
    case CCP_opener_control:
      {
        if (ccp_CAN.str_match("CHECK", 5))
          opener.goCHECK();
        if (ccp_CAN.str_match("READY", 5))
          opener.goREADY();
      }
      break;
    case CCP_A_accel_mss:
      acceleration_mss = ccp_CAN.data_fp16_2();
      get_acc_data = true;
      break;
    case CCP_A_pressure_altitude_m:
      altitude_m = ccp_CAN.data_float();
      get_alt_data = true;
      break;
    case CCP_open_time_command_s:
      opener.set_open_threshold_time_ms(ccp_CAN.data_float() * 1000);
      delay(200);
      ccp_CAN.float_to_device(CCP_open_time_response_s, (float)opener.get_open_threshold_time_ms() / 1000.0);
      break;
    default:
      break;
    }
  }

  if (get_acc_data && get_alt_data)
  {
    get_acc_data = false;
    get_alt_data = false;

    bool new_judge = opener.opener_100Hz(acceleration_mss, altitude_m);

    if (new_judge)
    {
      if (opener.mode == OPENER::CHECK)
      {
          ccp_CAN.string_to_device(CCP_opener_state, "CHECK");
      }
      else if (opener.mode == OPENER::READY)
      {
          ccp_CAN.string_to_device(CCP_opener_state, "READY");
      }
      else if (opener.mode == OPENER::FLIGHT)
      {
          ccp_CAN.string_to_device(CCP_opener_state, "FLIGHT");
          ccp_CAN.uint32_to_device(CCP_lift_off_time_ms, opener.lift_off_time_ms);
      }
      else if (opener.mode == OPENER::OPENED)
      {
          ccp_CAN.string_to_device(CCP_opener_state, "OPENED");
          ccp_CAN.uint32_to_device(CCP_open_time_ms, opener.open_time_ms);
      }

      if (opener.lift_off_judge == OPENER::NONE)
      {
          ccp_CAN.string_to_device(CCP_lift_off_judge, "------");
      }
      else if (opener.lift_off_judge == OPENER::ACCSEN)
      {
          ccp_CAN.string_to_device(CCP_lift_off_judge, "ACCSEN");
      }
      else if (opener.lift_off_judge == OPENER::ALTSEN)
      {
          ccp_CAN.string_to_device(CCP_lift_off_judge, "ALTSEN");
      }

      char open_judge[6];
      open_judge[0] = (opener.open_judge.isOpend ? 'O' : 'C');
      open_judge[1] = (!opener.open_judge.prohibitOpen ? 'O' : 'C');
      open_judge[2] = (opener.open_judge.meco_time ? 'O' : 'C');
      open_judge[3] = (opener.open_judge.meco_acc ? 'O' : 'C');
      open_judge[4] = (opener.open_judge.apogee_time ? 'O' : 'C');
      open_judge[5] = (opener.open_judge.apogee_descending ? 'O' : 'C');
      ccp_CAN.string_to_device(CCP_open_judge, open_judge);

      ccp_CAN.float_to_device(CCP_open_time_response_s, opener.get_open_threshold_time_ms() / 1000.0);
    }
  }
}
