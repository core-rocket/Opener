/**
 * @file Opener/Opener.h
 * @subpage opener_class
 * @brief ハイブリッドロケットの開放機構の制御を行うライブラリ
 * @details ハイブリッドロケットの開放機構の制御を行うライブラリ
 **/

#ifndef OPENER_H
#define OPENER_H

#include <Arduino.h>
//#include "pico/stdlib.h"

int arraycmp(const void *p1, const void *p2);
bool is_odd_number(int n);
float get_median(int n, float a[]);

/*!
 *   @brief  ハイブリッドロケットの開放機構の制御を行うクラス
 */
class OPENER
{
private:
    /**
     * @brief 開放判定の判定結果が格納される
     **/
    typedef struct
    {
        //! 開放機構を開くべきか
        bool isOpend = false;
        //! 手動で開放禁止されているか
        bool prohibitOpen = true;
        //! 離床判定からの時間により燃焼終了したと判断したか
        bool meco_time = false;
        //! 加速度がかかっていないことにより燃焼終了したと判断したか
        bool meco_acc = false;
        //! 離床判定からの時間により頂点到達したと判断したか
        bool apogee_time = false;
        //! 下降を検知し頂点到達したと判断したか
        bool apogee_descending = false;
    } OPEN_JUDGE;

    void goFLIGHT();
    void goOPENED();
    uint32_t get_time_ms();

    // timer
    uint32_t before_10Hz_time = 0;
    uint32_t before_100Hz_time = 0;
    uint32_t count_100Hz = 0;
    void opener_10Hz();

    // get median
    float ALT_buf_mss[10];
    float ACC_buf_mss[10];

    // oversampling
    int ALT_count = 0;

    // altitude->velocity
    float before_altitude_m = 0;

    // sensor_judgement
    int lift_off_altitude_count = 0;
    int lift_off_acc_count = 0;
    int apogee_altitude_count = 0;
    int meco_acc_count = 0;

    // threshold
    float lift_off_threshold_altitude_m = 0;
    float lift_off_threshold_ac_mss = 0;
    int ALT_oversampling_count = 0;
    int ALT_threshold_count = 0;

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
    virtual void open() = 0;
    //! 開放機構を閉じる
    virtual void close() = 0;

public:
    enum SETTING
    {
        //! 閾値をフライト用に設定
        FM,
        //! 閾値を審査書用に設定
        SHINSASYO
    };
    /*! OPENERのコンストラクタ
     *   @param  _setting    閾値をフライト用に設定するか，審査書用に設定するか
     */
    OPENER(SETTING _setting)
    {
        if (_setting == FM)
        {
            lift_off_threshold_altitude_m = fm_lift_off_threshold_altitude_m;
            lift_off_threshold_ac_mss = fm_lift_off_threshold_ac_mss;
            ALT_oversampling_count = fm_ALT_oversampling_count;
            ALT_threshold_count = fm_ALT_threshold_count;
        }
        else if (_setting == SHINSASYO)
        {
            lift_off_threshold_altitude_m = shinsasyo_lift_off_threshold_altitude_m;
            lift_off_threshold_ac_mss = shinsasyo_lift_off_threshold_ac_mss;
            ALT_oversampling_count = shinsasyo_ALT_oversampling_count;
            ALT_threshold_count = shinsasyo_ALT_threshold_count;
        }
    }

    enum MODE
    {
        //! 離床判定禁止
        CHECK,
        //! 離床判定許可
        READY,
        //! 離床判定後
        FLIGHT,
        //! 開放判定後
        OPENED
    } mode = CHECK;
    enum LIFT_OFF_JUDGE
    {
        //! 離床判定前
        NONE,
        //! 加速度により離床判定
        ACCSEN,
        //! 高度上昇により離床判定
        ALTSEN
    } lift_off_judge = NONE;

    //! 初期化（myOpenerクラスでオーバーライドするときは必ず OPENER::init() を呼び出すこと）
    void init();

    void goCHECK();
    void goREADY();
    void clear_prohibitOpen();
    void prohibitOpen();

    void manualOpen();
    void manualClose();

    OPEN_JUDGE open_judge;
    bool opener_100Hz(float acceleration_mss, float altitude_m);

    void set_open_threshold_time_ms(uint32_t _open_threshold_time_ms);
    uint32_t get_open_threshold_time_ms();

    uint32_t get_meco_time_ms();

    //! 離床したと推定される時刻(離床判定が行われた時刻ではない)
    uint32_t lift_off_time_ms;
    //! 実際に開放した時刻
    uint32_t open_time_ms;
};

#endif