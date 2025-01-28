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

// TODO: (未実装)
// ライブラリ導入
#include <mrd_plugin/i_mrd_servo.hpp>

class MrdServoPCA9685 : public I_Meridian_Servo {
public:
  MrdServoPCA9685() {}
  ~MrdServoPCA9685() {}

public:
  const char *get_name() { return "I2C_PCA9685 to PWM"; };
  bool setup() override { return true; }

  bool write(int a_id, int value) override {};
  int read(int a_id) override {};

  bool refresh(Meridim90Union &a_meridim) override { return true; }
};

#endif // MRD_SERVO_PCA9685_HPP
