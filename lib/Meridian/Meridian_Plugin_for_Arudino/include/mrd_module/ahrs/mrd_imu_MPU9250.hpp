/**
 * @file mrd_imu_MPU9250.hpp
 * @brief デバイス[MPU9250]制御クラス
 * @version 1.2.0
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_IMU_MPU9250_HPP
#define MRD_IMU_MPU9250_HPP

// ライブラリ導入
#include "mrd_plugin/i_mrd_plugin_ahrs.hpp"

class MrdImuMPU9250 : public I_Meridian_AHRS<float, float, float> {

public:
  MrdImuMPU9250() {
  }
  ~MrdImuMPU9250() {
  }

  bool setup() override {
    return true;
  }

  bool reset() override {
    return true;
  }

  bool refresh(Meridim90Union &a_meridim) override {
    return true;
  }
};

#endif // MRD_IMU_MPU9250_HPP
