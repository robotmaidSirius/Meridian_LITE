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
  enum ServoType { // サーボプロトコルのタイプ
    NOSERVO = 0,   // サーボなし
    PWM_S = 1,     // Single PWM (WIP)
    PCA9685 = 11,  // I2C_PCA9685 to PWM (WIP)
    FTBRSX = 21,   // FUTABA_RSxTTL (WIP)
    DXL1 = 31,     // DYNAMIXEL 1.0 (WIP)
    DXL2 = 32,     // DYNAMIXEL 2.0 (WIP)
    KOICS3 = 43,   // KONDO_ICS 3.5 / 3.6
    KOPMX = 44,    // KONDO_PMX (WIP)
    JRXBUS = 51,   // JRPROPO_XBUS (WIP)
    FTCSTS = 61,   // FEETECH_STS (WIP)
    FTCSCS = 62    // FEETECH_SCS (WIP)
  };

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
