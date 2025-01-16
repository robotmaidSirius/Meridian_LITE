/**
 * @file i_mrd_servo.hpp
 * @brief
 * @version 0.25.1
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
  virtual ~I_Meridian_Servo() = default;
  virtual void write(uint8_t angle) = 0;
};

#endif // I_MRD_SERVO_HPP
