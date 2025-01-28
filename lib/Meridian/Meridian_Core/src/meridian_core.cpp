/**
 * @file meridian_core.cpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#include "meridian_core.hpp"
// TODO: Arduinoは削除したい
#include <Arduino.h>
#include <string.h>

namespace meridian {
namespace core {
namespace execution {

void meridim_clear(Meridim90 &a_meridim) {
  a_meridim.master_command = (int16_t)MasterCommand::MCMD_TORQUE_ALL_OFF; //!	マスターコマンド

  a_meridim.sequential = 0; //! シーケンス番号

  a_meridim.acc_x = 0;       //! 加速度センサX値
  a_meridim.acc_y = 0;       //! 加速度センサY値
  a_meridim.acc_z = 0;       //! 加速度センサZ値
  a_meridim.gyro_x = 0;      //! ジャイロセンサX値
  a_meridim.gyro_y = 0;      //! ジャイロセンサY値
  a_meridim.gyro_z = 0;      //! ジャイロセンサZ値
  a_meridim.mag_x = 0;       //! 磁気コンパスX値
  a_meridim.mag_y = 0;       //! 磁気コンパスY値
  a_meridim.mag_z = 0;       //! 磁気コンパスZ値
  a_meridim.temperature = 0; //! 温度センサ値
  a_meridim.dmp_roll = 0;    //! DMP推定ロール方向値
  a_meridim.dmp_pitch = 0;   //! DMP推定ピッチ方向値
  a_meridim.dmp_yaw = 0;     //! DMP推定ヨー方向値

  a_meridim.control_buttons = 0;    //! リモコンの基本ボタン値
  a_meridim.control_stick_l = 0;    //! リモコンの左スティックアナログ値
  a_meridim.control_stick_r = 0;    //! リモコンの右スティックアナログ値
  a_meridim.control_l2r2analog = 0; //! リモコンのL2R2ボタンアナログ値

  a_meridim.motion_frames = 0;  //! モーション設定のフレーム数
  a_meridim.stop_frames_ms = 0; //! ボード停止時のフレーム数

  a_meridim.head_y_cmd = 0; //! 頭ヨーのコマンド
  a_meridim.head_y_val = 0; //! 頭ヨーの値

  a_meridim.l_shoulder_p_cmd = 0; //! 左肩ピッチのコマンド
  a_meridim.l_shoulder_p_val = 0; //! 左肩ピッチの値
  a_meridim.l_shoulder_r_cmd = 0; //! 左肩ロールのコマンド
  a_meridim.l_shoulder_r_val = 0; //! 左肩ロールの値
  a_meridim.l_elbow_y_cmd = 0;    //! 左肘ヨーのコマンド
  a_meridim.l_elbow_y_val = 0;    //! 左肘ヨーの値
  a_meridim.l_elbow_p_cmd = 0;    //! 左肘ピッチのコマンド
  a_meridim.l_elbow_p_val = 0;    //! 左肘ピッチの値
  a_meridim.l_hipjoint_y_cmd = 0; //! 左股ヨーのコマンド
  a_meridim.l_hipjoint_y_val = 0; //! 左股ヨーの値
  a_meridim.l_hipjoint_r_cmd = 0; //! 左股ロールのコマンド
  a_meridim.l_hipjoint_r_val = 0; //! 左股ロールの値
  a_meridim.l_hipjoint_p_cmd = 0; //! 左股ピッチのコマンド
  a_meridim.l_hipjoint_p_val = 0; //! 左股ピッチの値
  a_meridim.l_knee_p_cmd = 0;     //! 左膝ピッチのコマンド
  a_meridim.l_knee_p_val = 0;     //! 左膝ピッチの値
  a_meridim.l_ankle_p_cmd = 0;    //! 左足首ピッチのコマンド
  a_meridim.l_ankle_p_val = 0;    //! 左足首ピッチの値
  a_meridim.l_ankle_r_cmd = 0;    //! 左足首ロールのコマンド
  a_meridim.l_ankle_r_val = 0;    //! 左足首ロールの値
  a_meridim.l_servo_id11_cmd = 0; //! 追加サーボ用のコマンド
  a_meridim.l_servo_id11_val = 0; //! 追加サーボ用の値
  a_meridim.l_servo_id12_cmd = 0; //! 追加サーボ用のコマンド
  a_meridim.l_servo_id12_val = 0; //! 追加サーボ用の値
  a_meridim.l_servo_id13_cmd = 0; //! 追加サーボ用のコマンド
  a_meridim.l_servo_id13_val = 0; //! 追加サーボ用の値
  a_meridim.l_servo_id14_cmd = 0; //! 追加サーボ用のコマンド
  a_meridim.l_servo_id14_val = 0; //! 追加サーボ用の値

  a_meridim.waist_y_cmd = 0;      //! 腰ヨーのコマンド
  a_meridim.waist_y_val = 0;      //! 腰ヨーの値
  a_meridim.r_shoulder_p_cmd = 0; //! 右肩ピッチのコマンド
  a_meridim.r_shoulder_p_val = 0; //! 右肩ピッチの値
  a_meridim.r_shoulder_r_cmd = 0; //! 右肩ロールのコマンド
  a_meridim.r_shoulder_r_val = 0; //! 右肩ロールの値
  a_meridim.r_elbow_y_cmd = 0;    //! 右肘ヨーのコマンド
  a_meridim.r_elbow_y_val = 0;    //! 右肘ヨーの値
  a_meridim.r_elbow_p_cmd = 0;    //! 右肘ピッチのコマンド
  a_meridim.r_elbow_p_val = 0;    //! 右肘ピッチの値
  a_meridim.r_hipjoint_y_cmd = 0; //! 右股ヨーのコマンド
  a_meridim.r_hipjoint_y_val = 0; //! 右股ヨーの値
  a_meridim.r_hipjoint_r_cmd = 0; //! 右股ロールのコマンド
  a_meridim.r_hipjoint_r_val = 0; //! 右股ロールの値
  a_meridim.r_hipjoint_p_cmd = 0; //! 右股ピッチのコマンド
  a_meridim.r_hipjoint_p_val = 0; //! 右股ピッチの値
  a_meridim.r_knee_p_cmd = 0;     //! 右膝ピッチのコマンド
  a_meridim.r_knee_p_val = 0;     //! 右膝ピッチの値
  a_meridim.r_ankle_p_cmd = 0;    //! 右足首ピッチのコマンド
  a_meridim.r_ankle_p_val = 0;    //! 右足首ピッチの値
  a_meridim.r_ankle_r_cmd = 0;    //! 右足首ロールのコマンド
  a_meridim.r_ankle_r_val = 0;    //! 右足首ロールの値
  a_meridim.r_servo_id11_cmd = 0; //! 追加テスト用のコマンド
  a_meridim.r_servo_id11_val = 0; //! 追加テスト用の値
  a_meridim.r_servo_id12_cmd = 0; //! 追加テスト用のコマンド
  a_meridim.r_servo_id12_val = 0; //! 追加テスト用の値
  a_meridim.r_servo_id13_cmd = 0; //! 追加テスト用のコマンド
  a_meridim.r_servo_id13_val = 0; //! 追加テスト用の値
  a_meridim.r_servo_id14_cmd = 0; //! 追加テスト用のコマンド
  a_meridim.r_servo_id14_val = 0; //! 追加テスト用の値

  for (int i = 0; i < MERIDIM90_USER_DATA_SIZE; i++) {
    a_meridim.user_data[i] = 0; //! ユーザー定義用
  }

  a_meridim.err = 0;      //! ERROR CODE
  a_meridim.checksum = 0; //! CHECK SUM
  mrd_set_checksum(a_meridim);
}

void mrd_convert_array(const uint8_t *data, int len, Meridim90 &a_meridim) {
  mrd_set_checksum(a_meridim);
  memcpy(&data, &a_meridim, len);
}
void mrd_convert_Meridim90(Meridim90 &a_meridim, const uint8_t *data, int len) {
  if (len <= MERIDIM90_SIZE) {
    len = MERIDIM90_SIZE;
  }
  memcpy(&a_meridim, data, size_t(len));
  return;
}
void meridim_countup(Meridim90 &a_meridim) {
  a_meridim.sequential = (uint16_t)((uint32_t)a_meridim.sequential + 1) % 0xFFFF;
}

//------------------------------------------------------------------------------------
//  meridimへのデータ書き込み
//------------------------------------------------------------------------------------

/// @brief meridim配列のチェックサムを算出して[len-1]に書き込む.
/// @param a_meridim Meridim配列の共用体. 参照渡し.
/// @return 常にtrueを返す.
void mrd_set_checksum(Meridim90 &a_meridim) {
  uint16_t data[MERIDIM90_SIZE] = {0};
  memcpy(data, &a_meridim, MERIDIM90_SIZE);
  uint32_t a_checksum = 0;
  for (int i = 0; i < MERIDIM90_DATA_SIZE; i++) {
    a_checksum = (a_checksum + data[i]) & 0xFFFF;
  }
  a_meridim.checksum = (uint16_t)(~a_checksum & 0xFFFF);
  return;
}

/// @brief 指定された位置のビットをセットする(16ビット変数用).
/// @param a_byte ビットをセットする16ビットの変数.参照渡し.
/// @param a_bit_pos セットするビットの位置(0から15).
/// @return なし.
inline void mrd_setBit16(uint16_t &a_byte, uint16_t a_bit_pos) { a_byte |= (1 << a_bit_pos); }

/// @brief 指定された位置のビットをクリアする(16ビット変数用).
/// @param a_byte ビットをクリアする16ビットの変数.参照渡し.
/// @param a_bit_pos クリアするビットの位置(0から15).
/// @return なし.
inline void mrd_clearBit16(uint16_t &a_byte, uint16_t a_bit_pos) { a_byte &= ~(1 << a_bit_pos); }

/// @brief 指定された位置のビットをセットする(8ビット変数用).
/// @param value ビットをセットする8ビットの変数.参照渡し.
/// @param a_bit_pos セットするビットの位置(0から7).
/// @return なし.
inline void mrd_setBit8(uint8_t &value, uint8_t a_bit_pos) { value |= (1 << a_bit_pos); }

/// @brief 指定された位置のビットをクリアする(8ビット変数用).
/// @param value ビットをクリアする8ビットの変数.参照渡し.
/// @param a_bit_pos クリアするビットの位置(0から7).
/// @return なし.
inline void mrd_clearBit8(uint8_t &value, uint8_t a_bit_pos) { value &= ~(1 << a_bit_pos); }

//------------------------------------------------------------------------------------
//  タイムアウト監視用タイマー
//------------------------------------------------------------------------------------
// タイマーの開始状態を保持するための静的変数
static unsigned long timeout_start = 0;
static bool flg_timer_started = false; // タイマーが開始されたかどうかのフラグ

/// @brief 指定されたミリ秒のタイムアウトを監視する. mrd_timeout_resetとセットで使う.
/// @param a_limit タイムアウトまでの時間(ms)
/// @return タイムアウトでtrueを返す.
bool mrd_timeout_check(unsigned long a_limit) {
  // タイマーが開始されていない場合は現在の時間を記録してタイマーを開始
  if (!flg_timer_started) {
    timeout_start = millis();
    flg_timer_started = true; // タイムアウト監視開始フラグをアゲる
  }

  unsigned long current_time = millis(); // 現在の時間を取得

  if (current_time - timeout_start >= a_limit) { // 指定された時間が経過しているかチェック
    flg_timer_started = false;                   // タイムアウト監視開始フラグをサゲる
    return true;                                 // 指定された時間が経過していれば true を返す
  }

  return false; // まだ時間が経過していなければ false を返す
}

/// @brief タイムアウト監視開始フラグをリセットする. mrd_timeout_checkとセットで使う.
void mrd_timeout_reset() {
  flg_timer_started = false; // タイマーをリセットして次回の呼び出しに備える
}

} // namespace execution
} // namespace core
} // namespace meridian
