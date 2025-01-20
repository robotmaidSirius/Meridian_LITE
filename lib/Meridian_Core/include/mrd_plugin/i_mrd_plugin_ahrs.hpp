/**
 * @file i_mrd_plugin_ahrs.hpp
 * @brief MeridianCoreで使用するAttitude and heading reference systemのインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_PLUGIN_AHRS_HPP
#define I_MRD_PLUGIN_AHRS_HPP

#include "i_mrd_plugin.hpp"

template <typename TYPE_ACC, typename TYPE_GYRO, typename TYPE_MAG>
class I_Meridian_AHRS : public I_Meridian_Plugin {
public:
  // Acceleration
  struct ACC_vector3 {
    TYPE_ACC x;
    TYPE_ACC y;
    TYPE_ACC z;
  };
  // Gyroscope
  struct GYRO_vector3 {
    TYPE_GYRO x;
    TYPE_GYRO y;
    TYPE_GYRO z;
  };
  // Magnetometer
  struct MAG_vector3 {
    TYPE_MAG x;
    TYPE_MAG y;
    TYPE_MAG z;
  };

public:
  ACC_vector3 acc;
  GYRO_vector3 gyro;
  MAG_vector3 mag;

public:
  virtual bool reset() { return true; };
};

#endif // I_MRD_PLUGIN_AHRS_HPP
