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

// TODO: (未実装)
// ライブラリ導入
#include <mrd_plugin/i_mrd_servo.hpp>

class MrdServoFeetechSCS : public I_Meridian_Servo {
public:
  MrdServoFeetechSCS() {}
  ~MrdServoFeetechSCS() {}

public:
  const char *get_name() { return "SCS(FEETECH)"; };
  bool setup() override { return true; }

  bool write(int a_id, int value) override {};
  int read(int a_id) override {};

  bool refresh(Meridim90Union &a_meridim) override { return true; }

#endif // MRD_SERVO_FEETECH_SCS_HPP
