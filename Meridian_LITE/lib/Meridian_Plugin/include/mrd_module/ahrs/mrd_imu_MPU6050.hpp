/**
 * @file mrd_imu_MPU6050.hpp
 * @brief デバイス[MPU6050]制御クラス
 * @version 1.2.0
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_IMU_MPU6050_HPP
#define MRD_IMU_MPU6050_HPP

#include "mrd_plugin/i_mrd_imu.hpp"

class MrdImuMPU6050 : public I_Meridian_IMU {
public:
  MrdImuMPU6050() {
  }
  ~MrdImuMPU6050() {
  }
  void update() override {
  }
  void getAccel(float &x, float &y, float &z) override {
  }
  void getGyro(float &x, float &y, float &z) override {
  }
  void getMag(float &x, float &y, float &z) override {
  }

  bool refresh(Meridim90Union &a_meridim) override {
    return true;
  }
};

#endif // MRD_IMU_MPU6050_HPP
