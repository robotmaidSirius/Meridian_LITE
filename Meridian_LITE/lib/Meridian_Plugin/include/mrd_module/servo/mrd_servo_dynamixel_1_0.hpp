/**
 * @file mrd_servo_dynamixel_1_0.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-17
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_SERVO_DYNAMIXEL_1_0_HPP
#define MRD_SERVO_DYNAMIXEL_1_0_HPP

#include <mrd_plugin/i_mrd_servo.hpp>

class MrdServoDynamixel_1_0 : public I_Meridian_Servo {
public:
  MrdServoDynamixel_1_0() {
    this->setServoType(ServoType::DXL1);
  }
  ~MrdServoDynamixel_1_0() {
  }
};

#endif // MRD_SERVO_DYNAMIXEL_1_0_HPP
