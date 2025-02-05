/**
 * @file mrd_servo_feetech_sts.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-17
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_SERVO_FEETECH_STS_HPP
#define MRD_SERVO_FEETECH_STS_HPP

// TODO: (未実装)
// ライブラリ導入
#include <mrd_module/mrd_plugin/i_mrd_plugin_servo.hpp>

namespace meridian {
namespace modules {
namespace plugin {

class MrdServoFeetechSTS : public IMeridianServo {
public:
  MrdServoFeetechSTS() {}
  ~MrdServoFeetechSTS() {}

public:
  const char *get_name() { return "STS(FEETECH)"; }
  bool setup() override { return true; }
  bool input(Meridim90 &a_meridim) override { return true; }
  bool output(Meridim90 &a_meridim) override { return true; }
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // MRD_SERVO_FEETECH_STS_HPP
