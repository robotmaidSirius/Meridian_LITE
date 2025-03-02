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

#include <mrd_plugin/i_mrd_servo.hpp>

class MrdServoPca9685 : public I_Meridian_Servo {

public:
  MrdServoPca9685() {
    this->setServoType(ServoType::PCA9685);
  }
  ~MrdServoPca9685() {
  }
};

#endif // MRD_SERVO_PCA9685_HPP
