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

#include "Module/i_mrd_imu.hpp"

class MrdImuMPU9250 : public I_Meridian_IMU {

public:
  MrdImuMPU9250() {
  }
  ~MrdImuMPU9250() {
  }
  void update() override {
  }
  void getAccel(float &x, float &y, float &z) override {
  }
  void getGyro(float &x, float &y, float &z) override {
  }
  void getMag(float &x, float &y, float &z) override {
  }
};

#endif // MRD_IMU_MPU9250_HPP
