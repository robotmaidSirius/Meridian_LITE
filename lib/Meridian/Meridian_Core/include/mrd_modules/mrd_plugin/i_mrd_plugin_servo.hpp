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

class IMeridianServoStatus {
public:
  bool initalized = false;
  bool setup = false;
  bool happened_error = false;

public:
  void all_ok() {
    this->initalized = true;
    this->setup = true;
    this->happened_error = false;
  }
};

class IMeridianServo : public IMeridianPlugin {
public:
  virtual const char *get_name() { return "Unknow"; };
  virtual bool setup() { return true; };
  virtual bool input(Meridim90 &a_meridim) { return true; };
  virtual bool output(Meridim90 &a_meridim) { return true; };

public:
  void get_status(IMeridianServoStatus &state) {
    state.initalized = this->a_state.initalized;
    state.setup = this->a_state.setup;
    state.happened_error = this->a_state.happened_error;
  }

protected:
  IMeridianServoStatus a_state;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __I_MRD_PLUGIN_SERVO_HPP__
