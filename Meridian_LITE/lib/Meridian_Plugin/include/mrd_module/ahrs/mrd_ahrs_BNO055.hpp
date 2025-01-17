/**
 * @file mrd_ahrs_BNO055.hpp
 * @brief デバイス[BNO055]制御クラス
 * @version 1.2.0
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_AHRS_BNO055_HPP
#define MRD_AHRS_BNO055_HPP

#include "Module/i_mrd_ahrs.hpp"

class MrdAhrsBNO055 : public I_Meridian_AHRS {

public:
  MrdAhrsBNO055() {
  }
  ~MrdAhrsBNO055() {
  }
  void update() override {
  }
  void getEuler(float &roll, float &pitch, float &yaw) override {
  }
  void getQuaternion(float &w, float &x, float &y, float &z) override {
  }
};

#endif // MRD_AHRS_BNO055_HPP
