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
#include <string.h>

namespace meridian {
namespace core {
namespace execution {

void meridim_clear(Meridim90 &a_meridim) {
  a_meridim.master_command = (int16_t)MasterCommand::MCMD_TORQUE_ALL_OFF; ///! マスターコマンド

  a_meridim.sequential = 0; ///! シーケンス番号

  a_meridim.input_data.accelerator.x = 0;  ///! 加速度センサX値
  a_meridim.input_data.accelerator.y = 0;  ///! 加速度センサY値
  a_meridim.input_data.accelerator.z = 0;  ///! 加速度センサZ値
  a_meridim.input_data.gyroscope.x = 0;    ///! ジャイロセンサX値
  a_meridim.input_data.gyroscope.y = 0;    ///! ジャイロセンサY値
  a_meridim.input_data.gyroscope.z = 0;    ///! ジャイロセンサZ値
  a_meridim.input_data.magnetometer.x = 0; ///! 磁気コンパスX値
  a_meridim.input_data.magnetometer.y = 0; ///! 磁気コンパスY値
  a_meridim.input_data.magnetometer.z = 0; ///! 磁気コンパスZ値
  a_meridim.input_data.temperature = 0;    ///! 温度センサ値
  a_meridim.input_data.dmp.roll = 0;       ///! DMP推定ロール方向値
  a_meridim.input_data.dmp.pitch = 0;      ///! DMP推定ピッチ方向値
  a_meridim.input_data.dmp.yaw = 0;        ///! DMP推定ヨー方向値

  a_meridim.input_data.control.buttons = 0;  ///! リモコンの基本ボタン値
  a_meridim.input_data.control.stick_l = 0;  ///! リモコンの左スティックアナログ値
  a_meridim.input_data.control.stick_r = 0;  ///! リモコンの右スティックアナログ値
  a_meridim.input_data.control.analog_l = 0; ///! リモコンのL2R2ボタンアナログ値
  a_meridim.input_data.control.analog_r = 0; ///! リモコンのL2R2ボタンアナログ値

  a_meridim.userdata.motion_frames = 0;  ///! モーション設定のフレーム数
  a_meridim.userdata.stop_frames_ms = 0; ///! ボード停止時のフレーム数

  for (int i = 0; i < MERIDIM90_SERVO_NUM; i++) {
    a_meridim.userdata.servo[i].id = 0;
    a_meridim.userdata.servo[i].cmd = 0;
    a_meridim.userdata.servo[i].value = 0;
  }

  for (int i = 0; i < MERIDIM90_USER_DATA_SIZE; i++) {
    a_meridim.userdata.options[i] = 0; ///! ユーザー定義用
  }

  a_meridim.err = (int16_t)ErrorBit::ERRBIT_COMMON; ///! ERROR CODE
  a_meridim.checksum = 0xFFFF;                      ///! CHECK SUM
  mrd_set_checksum(a_meridim);
}

void mrd_convert_array(uint8_t *data, int len, Meridim90 &a_meridim) {
  mrd_set_checksum(a_meridim);
  memcpy(data, &a_meridim, len);
}
void mrd_convert_Meridim90(Meridim90 &a_meridim, const uint8_t *data, int len) {
  if (MERIDIM90_BYTE_LEN < len) {
    len = MERIDIM90_BYTE_LEN;
  }
  memcpy(&a_meridim, data, size_t(len));
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
  uint16_t data[MERIDIM90_BYTE_LEN] = {0};
  memcpy(data, &a_meridim, MERIDIM90_BYTE_LEN);
  uint32_t a_checksum = 0;
  for (int i = 0; i < MERIDIM90_DATA_LEN; i++) {
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

} // namespace execution
} // namespace core
} // namespace meridian
