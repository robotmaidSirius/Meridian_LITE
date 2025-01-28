/**
 * @file mrd_servo_dynamixel_2_0.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-17
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_SERVO_DYNAMIXEL_2_0_HPP
#define MRD_SERVO_DYNAMIXEL_2_0_HPP

// TODO: (未実装)
// ライブラリ導入
#include <mrd_plugin/i_mrd_servo.hpp>

class MrdServoDynamixel_2_0 : public I_Meridian_Servo {
public:
  MrdServoDynamixel_2_0() {}
  ~MrdServoDynamixel_2_0() {}

public:
  const char *get_name() { return "DYNAMIXEL Protocol 2.0"; };
  bool setup() override { return true; }

  bool write(int a_id, int value) override {};
  int read(int a_id) override {};

  bool refresh(Meridim90Union &a_meridim) override { return true; }
};

#endif // MRD_SERVO_DYNAMIXEL_2_0_HPP
