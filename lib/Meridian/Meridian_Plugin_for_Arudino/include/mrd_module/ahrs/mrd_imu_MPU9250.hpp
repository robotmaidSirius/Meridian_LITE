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
#include <meridian_core.hpp>

#define IMUAHRS_INTERVAL 10 // IMU/AHRSのセンサの読み取り間隔(ms)

class MrdImuMPU9250 : public I_Meridian_AHRS<float, float, float> {

public:
  MrdImuMPU9250() {
  }
  ~MrdImuMPU9250() {
  }

  bool refresh(Meridim90Union &a_meridim) override {
    return true;
  }
  bool setup() override {
    return false;
  }
  bool begin() override {
    return false;
  }

  bool reset() { return true; };

  //////////////////////////////////////////////////////

  //================================================================================================================
  //  I2C wire0 関連の処理
  //================================================================================================================

  //------------------------------------------------------------------------------------
  //  初期設定
  //------------------------------------------------------------------------------------

  /// @brief Wire0 I2C通信を初期化し, 指定されたクロック速度で設定する.
  /// @param a_i2c0_speed I2C通信のクロック速度です.
  /// @param a_pinSDA SDAのピン番号. 下記と合わせて省略可.
  /// @param a_pinSCL SCLのピン番号. 上記と合わせて省略可.
  bool mrd_wire0_init_i2c(int a_i2c0_speed, int a_pinSDA = -1, int a_pinSCL = -1) {
    Serial.print("Initializing wire0 I2C... ");
    if (a_pinSDA == -1 && a_pinSCL == -1) {
      Wire.begin();
    } else {
      Wire.begin(a_pinSDA, a_pinSCL);
    }
    Wire.setClock(a_i2c0_speed);
    return true;
  }

  /// @brief 指定されたIMU/AHRSタイプに応じて適切なセンサの初期化を行います.
  /// @param a_imuahrs_type 使用するセンサのタイプを示す列挙型です（MPU6050, MPU9250, BNO055）.
  /// @param a_i2c0_speed I2C通信のクロック速度です.
  /// @param a_ahrs AHRSの値を保持する構造体.
  /// @param a_pinSDA SDAのピン番号.下記と合わせて省略可.
  /// @param a_pinSCL SCLのピン番号.上記と合わせて省略可.
  /// @return センサが正しく初期化された場合はtrueを, そうでない場合はfalseを返す.
  bool mrd_wire0_setup(ImuAhrsType a_imuahrs_type, int a_i2c0_speed, AhrsValue &a_ahrs, int a_pinSDA = -1, int a_pinSCL = -1) {
    if (a_imuahrs_type > 0) // 何らかのセンサを搭載
    {
      if (a_pinSDA == -1 && a_pinSCL == -1) {
        mrd_wire0_init_i2c(a_i2c0_speed);
      } else {
        mrd_wire0_init_i2c(a_i2c0_speed, a_pinSDA, a_pinSCL);
      }
    }

    if (a_imuahrs_type == MPU6050_IMU) // MPU6050
    {
      return mrd_wire0_init_mpu6050_dmp(a_ahrs);
    } else if (a_imuahrs_type == MPU9250_IMU) // MPU9250の場合
    {
      // mrd_wire_init_mpu9250_dmp(a_ahrs)
      return false;
    } else if (a_imuahrs_type == BNO055_AHRS) // BNO055の場合
    {
      return mrd_wire0_init_bno055(a_ahrs);
    }

    Serial.println("No IMU/AHRS sensor mounted.");
    return false;
  }

  //------------------------------------------------------------------------------------
  //  センサデータの取得処理
  //------------------------------------------------------------------------------------

  bool mrd_ahrs_setup(TaskHandle_t &pvCreatedTask) {

    if (MOUNT_IMUAHRS == BNO055_AHRS) {
      xTaskCreatePinnedToCore(mrd_wire0_Core0_bno055_r, "Core0_bno055_r", 4096, NULL, 2, &pvCreatedTask, 0);
      return true;
    }
    return false;
  }

  /// @brief AHRSセンサーからI2C経由でデータを読み取る関数.
  /// MPU6050, MPU9250を想定していますが, MPU9250は未実装.
  /// 各データは`ahrs.read`配列に格納され, 利用可能な場合は`ahrs.result`にコピーされる.
  bool mrd_wire0_read_ahrs_i2c(AhrsValue &a_ahrs, bool imuahrs_available) { // ※wireTimer0.beginの引数のためvoid必須
    if (MOUNT_IMUAHRS == MPU6050_IMU) {                                     // MPU6050
      if (a_ahrs.mpu6050.dmpGetCurrentFIFOPacket(a_ahrs.fifoBuffer)) {      // Get new data
        a_ahrs.mpu6050.dmpGetQuaternion(&a_ahrs.q, a_ahrs.fifoBuffer);
        a_ahrs.mpu6050.dmpGetGravity(&a_ahrs.gravity, &a_ahrs.q);
        a_ahrs.mpu6050.dmpGetYawPitchRoll(a_ahrs.ypr, &a_ahrs.q, &a_ahrs.gravity);

        // acceleration values
        a_ahrs.mpu6050.dmpGetAccel(&a_ahrs.aa, a_ahrs.fifoBuffer);
        a_ahrs.read[0] = (float)a_ahrs.aa.x;
        a_ahrs.read[1] = (float)a_ahrs.aa.y;
        a_ahrs.read[2] = (float)a_ahrs.aa.z;

        // gyro values
        a_ahrs.mpu6050.dmpGetGyro(&a_ahrs.gyro, a_ahrs.fifoBuffer);
        a_ahrs.read[3] = (float)a_ahrs.gyro.x;
        a_ahrs.read[4] = (float)a_ahrs.gyro.y;
        a_ahrs.read[5] = (float)a_ahrs.gyro.z;

        // magnetic field values
        a_ahrs.read[6] = (float)a_ahrs.mag.x;
        a_ahrs.read[7] = (float)a_ahrs.mag.y;
        a_ahrs.read[8] = (float)a_ahrs.mag.z;

        // Estimated gravity DMP value.
        a_ahrs.read[9] = a_ahrs.gravity.x;
        a_ahrs.read[10] = a_ahrs.gravity.y;
        a_ahrs.read[11] = a_ahrs.gravity.z;

        // Estimated heading value using DMP.
        a_ahrs.read[12] = a_ahrs.ypr[2] * 180 / M_PI;                       // Estimated DMP_ROLL
        a_ahrs.read[13] = a_ahrs.ypr[1] * 180 / M_PI;                       // Estimated DMP_PITCH
        a_ahrs.read[14] = (a_ahrs.ypr[0] * 180 / M_PI) - a_ahrs.yaw_origin; // Estimated DMP_YAW

        // Temperature
        a_ahrs.read[15] = 0; // Not implemented.

        if (imuahrs_available) {
          memcpy(a_ahrs.result, a_ahrs.read, sizeof(float) * 16);
        }
        return true;
      } else {
        return false;
      }
    } else if (MOUNT_IMUAHRS == MPU9250_IMU) { // MPU9250
      return false;
    } else {
      return false;
    }
  }

  //------------------------------------------------------------------------------------
  //  meriput
  //------------------------------------------------------------------------------------
};

#endif // MRD_IMU_MPU9250_HPP
