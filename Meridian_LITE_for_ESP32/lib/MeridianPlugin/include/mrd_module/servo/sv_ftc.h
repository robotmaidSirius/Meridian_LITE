#ifndef __MERIDIAN_SERVO_FEETECH_H__
#define __MERIDIAN_SERVO_FEETECH_H__

// ヘッダファイルの読み込み
#include "sv_common.hpp" // サーボ用定義

// ライブラリ導入
#include <Meridim90.hpp> // Meridim90のライブラリ導入

namespace meridian {
namespace modules {
namespace plugin {

class MrdServoFeeTechTTL {
public:
};

//==================================================================================================
//  FEETECH TTLサーボ関連の処理  --------------------------------------------------------------------
//==================================================================================================

/// @brief スタブ関数.
/// @return 常にfalseを返す.
bool mrd_servo_ftc_x() {
  return false;
}

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MERIDIAN_SERVO_FUTABA_RSxTTL_H__
