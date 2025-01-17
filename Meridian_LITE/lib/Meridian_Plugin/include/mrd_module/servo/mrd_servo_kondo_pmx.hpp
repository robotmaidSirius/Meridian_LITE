/**
 * @file mrd_servo_kondo_pmx.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-17
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_SERVO_KONDO_PMX_HPP
#define MRD_SERVO_KONDO_PMX_HPP

#include "Module/i_mrd_servo.hpp"

class MrdServoKondoPmx : public I_Meridian_Servo {
public:
  MrdServoKondoPmx() {
    this->setServoType(ServoType::KOPMX);
  }
  ~MrdServoKondoPmx() {
  }
};

#endif // MRD_SERVO_KONDO_PMX_HPP
