/**
 * @file opener/opener.cpp
 * @brief ハイブリッドロケットの開放判定を行うライブラリ
 * @details ハイブリッドロケットの開放判定を行うライブラリ
 **/

#include "opener.h"
#include <stdlib.h>

int arraycmp(const void *p1, const void *p2)
{
    return *(const float *)p1 > *(const float *)p2;
}

bool is_odd_number(int n)
{
    return n & 0x1;
}

float get_median(int n, float a[])
{
    int k = n / 2;

    qsort(a, n, sizeof(float), arraycmp);

    if (is_odd_number(n))
    {
        return a[k];
    }
    else
    {
        return (float)(a[k - 1] + a[k]) / 2;
    }
}

uint32_t OPENER::get_time_ms()
{
    // for raspberry pi pico SDK
    // return to_ms_since_boot(get_absolute_time());

    // for Arduino
    return millis();
}

/*!
 *   @brief  100Hzで呼び出す必要がある関数
 *   @param acceleration_mss 機軸方向の加速度[m/s^2](ランチャ上で重力加速度がかかって正の値をとる方向)
 *   @param altitude_m 高度[m](基準高度は任意)
 *  @return 判定が更新されたか場合trueを返す．更新がない場合はfalseを返す．
 */
bool OPENER::opener_100Hz(float acceleration_mss, float altitude_m)
{
    before_100Hz_time = get_time_ms();
    ACC_buf_mss[count_100Hz] = acceleration_mss;
    ALT_buf_mss[count_100Hz] = altitude_m;

    // 10Hz
    count_100Hz++;
    if (count_100Hz >= 10)
    {
        count_100Hz = 0;

        opener_10Hz();
        return true;
    }
    return false;
}
/*!
 * @brief 加速度・高度の中央値を計算し，離床判定・開放判定を行う
 */
void OPENER::opener_10Hz()
{
    bool normal_datarate = get_time_ms() - before_10Hz_time < period_10Hz_ms;
    before_10Hz_time = get_time_ms();

    float acceleration_median_mss = get_median(10, ACC_buf_mss);
    float altitude_median_m = get_median(10, ALT_buf_mss);

    ALT_count++;
    if (ALT_count == 1)
    {
        bool climbing = altitude_median_m > before_altitude_m + lift_off_threshold_altitude_m;
        if (climbing && normal_datarate)
        {
            if (lift_off_altitude_count < ALT_threshold_count){
                lift_off_altitude_count++;
            }
        }
        else
        {
            lift_off_altitude_count = 0;
        }

        bool descending = altitude_median_m < before_altitude_m - open_threshold_altitude_m;
        if (descending && normal_datarate)
        {
            if (apogee_altitude_count < ALT_threshold_count){
                apogee_altitude_count++;
            }
        }
        else
        {
            apogee_altitude_count = 0;
        }

        before_altitude_m = altitude_median_m;
    }
    if (ALT_count >= ALT_oversampling_count)
    {
        ALT_count = 0;
    }

    bool accelerating = acceleration_median_mss > lift_off_threshold_ac_mss;
    if (accelerating)
    {
        if (lift_off_acc_count <= ACC_threshold_count){
            lift_off_acc_count++;
        }
    }
    else
    {
        lift_off_acc_count = 0;
    }

    bool freefall = acceleration_median_mss < open_threshold_ac_mss;
    if (freefall)
    {
        if (meco_acc_count <= ACC_threshold_count){
            meco_acc_count++;
        }
    }
    else
    {
        meco_acc_count = 0;
    }

    if (mode == READY)
    {
        if (lift_off_altitude_count >= ALT_threshold_count)
        {
            goFLIGHT();
            lift_off_judge = ALTSEN;
        }
        if (lift_off_acc_count >= ACC_threshold_count)
        {
            goFLIGHT();
            lift_off_judge = ACCSEN;
        }
    }

    if (mode == FLIGHT)
    {
        open_judge.meco_time = get_time_ms() - lift_off_time_ms >= meco_threshold_time_ms;
        open_judge.meco_acc = meco_acc_count >= ACC_threshold_count;
        bool meco = open_judge.meco_time && open_judge.meco_acc;

        open_judge.apogee_time = get_time_ms() - lift_off_time_ms >= open_threshold_time_ms;
        open_judge.apogee_descending = apogee_altitude_count >= ALT_threshold_count;
        bool apogee = open_judge.apogee_time || open_judge.apogee_descending;

        if (!open_judge.prohibitOpen && meco && apogee)
        {
            goOPENED();
        }
    }
}

/*!
 * @brief ::CHECKモードへ移行する．変数の初期化し，::prohibitOpenを有効化する．
 */
void OPENER::goCHECK()
{
    mode = CHECK;
    open_judge.isOpend = false;
    open_judge.prohibitOpen = true;
    open_judge.meco_time = false;
    open_judge.meco_acc = false;
    open_judge.apogee_time = false;
    open_judge.apogee_descending = false;
    lift_off_judge = NONE;
    lift_off_time_ms = 0;

    count_100Hz = 0;

    // oversampling
    ALT_count = 0;
    // altitude->velocity
    before_altitude_m = 0;
    // sensor_judgement
    lift_off_altitude_count = 0;
    lift_off_acc_count = 0;
    apogee_altitude_count = 0;
    meco_acc_count = 0;
    close();
}
/*!
 * @brief goCHECK()を実行して変数を初期化してから， prohibitOpen を無効化したうえで READY モードへ移行する．
 */
void OPENER::goREADY()
{
    goCHECK();
    open_judge.prohibitOpen = false;
    mode = READY;
}
/*!
 * @brief FLIGHT モードに移行し，離床時刻を記録する．
 */
void OPENER::goFLIGHT()
{
    mode = FLIGHT;
    lift_off_time_ms = get_time_ms() - flight_judgement_duration_ms;
}
/*!
 * @brief OPENED モードに移行し，開放時刻を記録する．
 */
void OPENER::goOPENED()
{
    mode = OPENED;
    open_judge.isOpend = true;
    open_time_ms = get_time_ms();
    open();
}
/*!
 * @brief prohibitOpen を無効化する．
 */
void OPENER::clear_prohibitOpen()
{
    open_judge.prohibitOpen = false;
}
/*!
 * @brief prohibitOpen を有効化する．
 */
void OPENER::prohibitOpen()
{
    open_judge.prohibitOpen = true;
}

/*!
 * @brief prohibitOpen が無効になっていることを確認し，手動で開放機構を開く．
 */
void OPENER::manualOpen()
{
    if (!open_judge.prohibitOpen)
    {
        open();
    }
}
/*!
 * @brief 手動で開放機構を閉じ， prohibitOpen を有効化する．
 */
void OPENER::manualClose()
{
    close();
    open_judge.prohibitOpen = true;
}

/*!
 * @brief 開放までのタイマーの秒数を設定する．
 * @param _open_threshold_time_ms 開放機構の動作にかかる時間を引いた，離床から開放までの時間のシム値[ms]
 */
void OPENER::set_open_threshold_time_ms(uint32_t _open_threshold_time_ms)
{
    open_threshold_time_ms = _open_threshold_time_ms;
}

/*!
 * @brief  開放までのタイマーの秒数を取得する．
 * @return 開放機構の動作にかかる時間を引いた，離床から開放までの時間のシム値[ms]
 */
uint32_t OPENER::get_open_threshold_time_ms(){
    return open_threshold_time_ms;
}