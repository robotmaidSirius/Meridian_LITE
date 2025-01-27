/**
 * @file mrd_module_servo_ics.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MRD_MODULE_SERVO_ICS_HPP__
#define __MRD_MODULE_SERVO_ICS_HPP__

// ヘッダーファイルの読み込み
#include <mrd_modules/mrd_plugin/i_mrd_plugin_servo.hpp>

// ライブラリ導入

namespace meridian {
namespace modules {
namespace plugin {

class MrdServoICS : public IMeridianServo {
public:
  MrdServoICS() {}
  ~MrdServoICS() {}
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MRD_MODULE_SERVO_ICS_HPP__
