/**
 * @file i_mrd_plugin_servo.hpp
 * @brief MeridianCoreで使用するサーボのインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __I_MRD_PLUGIN_SERVO_HPP__
#define __I_MRD_PLUGIN_SERVO_HPP__

#include "i_mrd_plugin.hpp"

namespace meridian {
namespace modules {
namespace plugin {

class IMeridianServo : public IMeridianPlugin {
protected:
  static const int ERR_CODE = 0x80; ///! エラー発生
  uint8_t COMMAND_READ_ONLY = 0x8;  ///! サーボのストレッチ設定
  enum COMMAND {
    COMMAND_FREE = 0,        ///! サーボのトルクOFF
    COMMAND_SET_POS = 1,     ///! サーボのトルクON
    COMMAND_STRETCH = 3,     ///! サーボのストレッチ設定
    COMMAND_SPEED = 4,       ///! サーボのスピード設定
    COMMAND_CURRENT = 5,     ///! サーボの電流制限設定
    COMMAND_TEMPERATURE = 6, ///! サーボの温度設定
    COMMAND_ID = 7,          ///! サーボのID設定
  };

public:
  virtual bool setup() { return true; }
  virtual bool input(Meridim90 &a_meridim) { return true; }
  virtual bool output(Meridim90 &a_meridim) { return true; }
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __I_MRD_PLUGIN_SERVO_HPP__
