#ifndef MY_OPENER_H
#define MY_OPENER_H

#include <Arduino.h>
#include <Opener.h>

class MY_OPENER : public OPENER
{
protected:
    //! 頂点判定に使われる，0.1秒間の高度変化の閾値
    const float open_threshold_altitude_m = 0.5;
    //! 燃焼終了に使われる，加速度の閾値
    const float open_threshold_ac_mss = 5;
    //! opener_10Hz()が適切なタイミングで呼ばれているかの判定に使われる，10Hzの周期
    const uint32_t period_10Hz_ms = 150; // 10Hz+50ms
    //! opener_100Hz()が適切なタイミングで呼ばれているかの判定に使われる，100Hzの周期
    const uint32_t period_100Hz_ms = 20; // 100Hz+10ms
    //! 閾値以上の加速度が何回連続したときに離床判定・燃焼終了判定を行うかという回数
    const int ACC_threshold_count = 5;
    //! 離床判定にかかる時間
    const int flight_judgement_duration_ms = ACC_threshold_count * 100;

    float fm_lift_off_threshold_altitude_m = 1.0;
    float fm_lift_off_threshold_ac_mss = 25.0;
    int fm_ALT_oversampling_count = 1;
    int fm_ALT_threshold_count = ACC_threshold_count;

    float shinsasyo_lift_off_threshold_altitude_m = 0.5;
    float shinsasyo_lift_off_threshold_ac_mss = 9.0;
    int shinsasyo_ALT_oversampling_count = 5.0;
    int shinsasyo_ALT_threshold_count = 2;

    //! 離床判定後，燃焼中と判断し開放判定を行わない時間[ms]
    const uint32_t meco_threshold_time_ms = 10000;
    //! 開放機構の動作にかかる時間を引いた，離床から開放までの時間のシム値[ms]の初期値
    uint32_t open_threshold_time_ms = 19000;

    //! 開放機構を開く
    void open();
    //! 開放機構を閉じる
    void close();

public:
    MY_OPENER(OPENER::SETTING _setting) : OPENER(_setting)
    {
    }

    void init();
};

#endif // MY_OPENER_H