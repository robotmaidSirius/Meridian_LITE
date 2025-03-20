/**
 * @file mrd_module_imu.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-03-08
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef __MERIDIAN_MODULE_IMU_H__
#define __MERIDIAN_MODULE_IMU_H__

namespace meridian {
namespace modules {
namespace plugin {

/// @brief 6軸or9軸センサーの種類を示す列挙型です.
enum ImuAhrsType {
  NO_IMU = 0,      /// IMU/AHRS なし.
  MPU6050_IMU = 1, /// MPU6050
  MPU9250_IMU = 2, /// MPU9250(未設定)
  BNO055_AHRS = 3  /// BNO055
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MERIDIAN_MODULE_IMU_H__
