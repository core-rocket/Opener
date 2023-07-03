#include "myOpener.h"

MY_OPENER opener(OPENER::FM);

void setup()
{
  // 開放機構のハードウェアの初期設定(Option)
  opener.init();
}

void loop()
{
  // コマンド受信に関するブロック
  {
    // 開放禁止コマンドを受信したとき，
    opener.prohibitOpen();
    // 開放禁止解除コマンドを受信したとき，
    opener.clear_prohibitOpen();

    // 手動開放コマンドを受信したとき，
    opener.manualOpen();
    // 手動閉鎖コマンドを受信したとき，
    opener.manualClose();

    // フェーズ移行コマンドを受信したとき，
    opener.goCHECK();
    opener.goREADY();

    // 開放閾値時間を受信したとき，
    uint32_t open_threshold_time_ms = 0;
    opener.set_open_threshold_time_ms(open_threshold_time_ms);
    // opener.get_open_threshold_time_ms() で設定値を取得可
  }

  // 100Hzで実行するブロック
  {
    // 加速度センサーにより測定した機軸方向の加速度[m/s^2]
    // 地上でロケットを立ち上げて重力加速度がかかったとき，正の値となる向き
    float acceleration_mss = 0;
    
    // 気圧センサーにより測定される高度[m]
    // 高度差を利用するため，射点で0mである必要はない．
    float altitude_m = 0;

    // 100Hzのセンサーデータの中央値を取り，10Hzで判定を行う．
    // 判定が行われたときtrueを返す．
    // 開放判定を行うと内部で open() を呼び出して開放機構を開く．
    bool new_judge = opener.opener_100Hz(acceleration_mss, altitude_m);

    if (new_judge){
      // テレメトリ送信に関するブロック
      {
        // 以下の構造体から状態を読み出し可能
        //   opener.mode
        //   opener.lift_off_judge
        //   opener.open_judge
      }
    }
  }
}
