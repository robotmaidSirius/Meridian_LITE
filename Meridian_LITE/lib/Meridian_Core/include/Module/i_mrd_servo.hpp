/**
 * @file i_mrd_servo.hpp
 * @brief MeridianCoreで使用するサーボのインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_SERVO_HPP
#define I_MRD_SERVO_HPP

#include <stdint.h>

class I_Meridian_Servo {
public:
  virtual bool is_enabled() { return false; }
};

#endif // I_MRD_SERVO_HPP
