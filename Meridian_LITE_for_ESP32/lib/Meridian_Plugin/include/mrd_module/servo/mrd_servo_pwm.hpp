/**
 * @file mrd_servo_pwm.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-17
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_SERVO_PWM_HPP
#define MRD_SERVO_PWM_HPP

#include <mrd_plugin/i_mrd_servo.hpp>

class MrdServoPwm : public I_Meridian_Servo {
public:
  MrdServoPwm() {
    this->setServoType(ServoType::PWM_S);
  }
  ~MrdServoPwm() {
  }
};

#endif // MRD_SERVO_PWM_HPP
