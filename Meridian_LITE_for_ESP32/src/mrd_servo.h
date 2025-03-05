#ifndef __MERIDIAN_SERVO_DISTRIBUTOR_H__
#define __MERIDIAN_SERVO_DISTRIBUTOR_H__

// ライブラリ導入
#include "mrd_module/sv_ftbrx.h"
#include "mrd_module/sv_ics.h"
#include <Meridim90.hpp> // Meridim90のライブラリ導入

enum ServoType { // サーボプロトコルのタイプ
  NOSERVO = 0,   // サーボなし
  PWM_S = 1,     // Single PWM (WIP)
  PCA9685 = 11,  // I2C_PCA9685 to PWM (WIP)
  FTBRSX = 21,   // FUTABA_RSxTTL (WIP)
  DXL1 = 31,     // DYNAMIXEL 1.0 (WIP)
  DXL2 = 32,     // DYNAMIXEL 2.0 (WIP)
  KOICS3 = 43,   // KONDO_ICS 3.5 / 3.6
  KOPMX = 44,    // KONDO_PMX (WIP)
  JRXBUS = 51,   // JRPROPO_XBUS (WIP)
  FTCSTS = 61,   // FEETECH_STS (WIP)
  FTCSCS = 62    // FEETECH_SCS (WIP)
};

namespace meridian {
namespace modules {
namespace plugin {

class MrdServoNone {
public:
};

//==================================================================================================
//  Servo 関連の処理
//==================================================================================================

//------------------------------------------------------------------------------------
//  各UARTの開始
//------------------------------------------------------------------------------------

bool mrd_servo_begin(IcsHardSerialClass &a_ics) {
  if (nullptr != &a_ics) {
    a_ics.begin(); // サーボモータの通信初期設定.
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------------
//  各種オペレーション
//------------------------------------------------------------------------------------

/// @brief 第一引数のMeridim配列のすべてのサーボモーターをオフ（フリー状態）に設定する.
/// @param a_meridim サーボの動作パラメータを含むMeridim配列.
/// @return 設定完了時にtrueを返す.
bool mrd_servo_all_off(Meridim90Union &a_meridim) {
  for (int i = 0; i < 15; i++) {    // 15はサーボ数
    a_meridim.sval[i * 2 + 20] = 0; // サーボのコマンドをオフに設定
    a_meridim.sval[i * 2 + 50] = 0; //
  }
  Serial.println("All servos torque off.");
  return true;
}

/// @brief サーボパラメータからエラーのあるサーボのインデックス番号を作る.
/// @param a_sv サーボパラメータの構造体.
/// @return uint8_tで番号を返す.
///         100-149(L系統 0-49),200-249(R系統 0-49)
uint8_t mrd_servos_make_errcode_lite(ServoParam a_sv) {
  uint8_t servo_ix_tmp = 0;
  for (int i = 0; i < 15; i++) {
    if (a_sv.ixl_stat[i]) {
      servo_ix_tmp = uint8_t(i + 100);
    }
    if (a_sv.ixr_stat[i]) {
      servo_ix_tmp = uint8_t(i + 200);
    }
  }
  return servo_ix_tmp;
}

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MERIDIAN_SERVO_DISTRIBUTOR_H__
