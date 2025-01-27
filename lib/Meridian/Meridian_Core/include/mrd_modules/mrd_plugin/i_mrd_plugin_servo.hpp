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
public:
  virtual const char *get_name() { return "Unknow"; };
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __I_MRD_PLUGIN_SERVO_HPP__
