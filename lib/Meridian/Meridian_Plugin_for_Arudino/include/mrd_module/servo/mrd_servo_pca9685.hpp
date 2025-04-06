/**
 * @file mrd_servo_pca9685.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-17
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_SERVO_PCA9685_HPP
#define MRD_SERVO_PCA9685_HPP

// TODO: (未実装)
// ライブラリ導入
#include <mrd_module/mrd_plugin/i_mrd_plugin_servo.hpp>

namespace meridian {
namespace modules {
namespace plugin {

class MrdServoPCA9685 : public IMeridianServo {
public:
  MrdServoPCA9685() {}
  ~MrdServoPCA9685() {}

public:
  const char *get_name() { return "I2C_PCA9685 to PWM"; }
  bool setup() override { return true; }
  bool input(Meridim90 &a_meridim) override { return true; }
  bool output(Meridim90 &a_meridim) override { return true; }
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // MRD_SERVO_PCA9685_HPP
