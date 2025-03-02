/**
 * @file mrd_servo_jrpropo_xbus.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-17
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_SERVO_JRPROPO_XBUS_HPP
#define MRD_SERVO_JRPROPO_XBUS_HPP

#include <mrd_plugin/i_mrd_servo.hpp>

class MrdServoJrpropoXbus : public I_Meridian_Servo {
public:
  MrdServoJrpropoXbus() {

    this->setServoType(ServoType::JRXBUS);
  }
  ~MrdServoJrpropoXbus() {
  }
};

#endif // MRD_SERVO_JRPROPO_XBUS_HPP
