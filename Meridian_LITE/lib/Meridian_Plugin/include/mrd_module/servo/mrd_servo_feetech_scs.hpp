/**
 * @file mrd_servo_feetech_scs.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-17
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_SERVO_FEETECH_SCS_HPP
#define MRD_SERVO_FEETECH_SCS_HPP

#include <Module/i_mrd_servo.hpp>

class MrdServoFeetech_SCS : public I_Meridian_Servo {

public:
  MrdServoFeetech_SCS() {
    this->setServoType(ServoType::FTCSCS);
  }
  ~MrdServoFeetech_SCS() {
  }
};

#endif // MRD_SERVO_FEETECH_SCS_HPP
