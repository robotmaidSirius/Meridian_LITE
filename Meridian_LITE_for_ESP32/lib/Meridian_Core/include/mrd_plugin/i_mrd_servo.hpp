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

#include "Meridim90.hpp"

class I_Meridian_Servo {
public:
public:
  virtual bool begin() = 0;
  virtual bool is_enabled() { return false; }
  ServoType getServoType() {
    return _servo_type;
  }

  virtual bool refresh(Meridim90Union &a_meridim) = 0;

protected:
  void setServoType(ServoType type) {
    _servo_type = type;
  }

private:
  ServoType _servo_type = ServoType::NOSERVO;
};

#endif // I_MRD_SERVO_HPP
