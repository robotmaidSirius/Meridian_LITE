#ifndef __MERIDIAN_SERVO_FUTABA_RSxTTL_H__
#define __MERIDIAN_SERVO_FUTABA_RSxTTL_H__

namespace meridian {
namespace modules {
namespace plugin {

class MrdServoFutabaRSxTTL {
public:
  MrdServoFutabaRSxTTL() {
  }
};

//==================================================================================================
//  FUTABA RSxTTLサーボ関連の処理  ------------------------------------------------------------------
//==================================================================================================

/// @brief スタブ関数.
/// @return 常にfalseを返す.
bool mrd_servo_ftbrs_x() {
  return false;
}

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MERIDIAN_SERVO_FUTABA_RSxTTL_H__
