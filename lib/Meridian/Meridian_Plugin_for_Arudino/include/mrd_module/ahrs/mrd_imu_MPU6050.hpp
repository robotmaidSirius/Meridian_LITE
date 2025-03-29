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

// ライブラリ導入
#include "mrd_plugin/i_mrd_plugin_ahrs.hpp"
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050用
#include <meridian_core.hpp>

#define PIN_I2C0_SDA     22 // I2CのSDAピン
#define PIN_I2C0_SCL     21 // I2CのSCLピン
#define IMUAHRS_INTERVAL 10 // IMU/AHRSのセンサの読み取り間隔(ms)

class MrdImuMPU6050 : public I_Meridian_AHRS {
private:
  MPU6050 mpu6050; // MPU6050のインスタンス
  bool _available = false;
  uint8_t fifoBuffer[64]; // FIFO storage buffer

  Quaternion quaternion_container; // [w, x, y, z]         quaternion container
  VectorFloat gravity;             // [x, y, z]            gravity vector
  VectorInt16 aa;                  // [x, y, z]            加速度センサの測定値
  VectorInt16 gyro;                // [x, y, z]            角速度センサの測定値
  VectorInt16 mag;                 // [x, y, z]            磁力センサの測定値
  float mpu_result[16];            // 加工後の最新のmpuデータ（二次データ）
  float ypr[3];                    // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector

public:
  MrdImuMPU6050() {
  }
  ~MrdImuMPU6050() {
  }

  bool refresh(Meridim90Union &a_meridim) override {
    mrd_wire0_read_ahrs_i2c(this->_available);
    return true;
  }
  bool setup() override {
    this->_available = false;
    int a_i2c0_speed;
    int a_pinSDA = PIN_I2C0_SDA;
    int a_pinSCL = PIN_I2C0_SCL;

    mpu6050.initialize();
    ahrs.devStatus = mpu6050.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu6050.setXAccelOffset(-1745);
    mpu6050.setYAccelOffset(-1034);
    mpu6050.setZAccelOffset(966);
    mpu6050.setXGyroOffset(176);
    mpu6050.setYGyroOffset(-6);
    mpu6050.setZGyroOffset(-25);

    // make sure it worked (returns 0 if so)
    if (ahrs.devStatus == 0) {
      mpu6050.CalibrateAccel(6);
      mpu6050.CalibrateGyro(6);
      mpu6050.setDMPEnabled(true);
      ahrs.packetSize = mpu6050.dmpGetFIFOPacketSize();
      this->_available = true;
    }
    return this->_available;
  }
  bool begin() override {
    return true;
  }

  bool reset() { return true; };

  //================================================================================================================
  //  I2C wire0 関連の処理
  //================================================================================================================

  //------------------------------------------------------------------------------------
  //  初期設定
  //------------------------------------------------------------------------------------

  //------------------------------------------------------------------------------------
  //  センサデータの取得処理
  //------------------------------------------------------------------------------------

  /// @brief AHRSセンサーからI2C経由でデータを読み取る関数.
  /// MPU6050, MPU9250を想定していますが, MPU9250は未実装.
  /// 各データは`ahrs.read`配列に格納され, 利用可能な場合は`ahrs.result`にコピーされる.
  bool mrd_wire0_read_ahrs_i2c(bool imuahrs_available) { // ※wireTimer0.beginの引数のためvoid必須
    if (mpu6050.dmpGetCurrentFIFOPacket(fifoBuffer)) {   // Get new data
      mpu6050.dmpGetQuaternion(&quaternion_container, fifoBuffer);
      mpu6050.dmpGetGravity(&gravity, &quaternion_container);
      mpu6050.dmpGetYawPitchRoll(ypr, &quaternion_container, &gravity);

      // acceleration values
      mpu6050.dmpGetAccel(&aa, fifoBuffer);
      ahrs.read[0] = (float)aa.x;
      ahrs.read[1] = (float)aa.y;
      ahrs.read[2] = (float)aa.z;

      // gyro values
      mpu6050.dmpGetGyro(&gyro, fifoBuffer);
      ahrs.read[3] = (float)gyro.x;
      ahrs.read[4] = (float)gyro.y;
      ahrs.read[5] = (float)gyro.z;

      // magnetic field values
      ahrs.read[6] = (float)mag.x;
      ahrs.read[7] = (float)mag.y;
      ahrs.read[8] = (float)mag.z;

      // Estimated gravity DMP value.
      ahrs.read[9] = gravity.x;
      ahrs.read[10] = gravity.y;
      ahrs.read[11] = gravity.z;

      // Estimated heading value using DMP.
      ahrs.read[12] = ypr[2] * 180 / M_PI;                     // Estimated DMP_ROLL
      ahrs.read[13] = ypr[1] * 180 / M_PI;                     // Estimated DMP_PITCH
      ahrs.read[14] = (ypr[0] * 180 / M_PI) - ahrs.yaw_origin; // Estimated DMP_YAW

      // Temperature
      ahrs.read[15] = 0; // Not implemented.

      if (imuahrs_available) {
        memcpy(mpu_result, ahrs.read, sizeof(float) * 16);
      }
      return true;
    } else {
      return false;
    }
  }
};

#endif // MRD_IMU_MPU6050_HPP
