/**
 * @file mrd_servo_kondo_ics_3_5.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-17
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_SERVO_KONDO_ICS_3_5_HPP
#define MRD_SERVO_KONDO_ICS_3_5_HPP

#include <mrd_plugin/i_mrd_servo.hpp>

class MrdServoKondoIcs35 : public I_Meridian_Servo {

public:
  MrdServoKondoIcs35() {
    this->setServoType(ServoType::KOICS3);
  }
  ~MrdServoKondoIcs35() {
  }
};

#endif // MRD_SERVO_KONDO_ICS_3_5_HPP
