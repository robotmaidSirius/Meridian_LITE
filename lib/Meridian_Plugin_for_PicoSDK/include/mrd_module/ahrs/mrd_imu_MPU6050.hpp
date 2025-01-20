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

class MrdImuMPU6050 : public I_Meridian_AHRS<float, float, float> {
private:
  MPU6050 mpu6050;   // MPU6050のインスタンス
  uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
  bool _available = false;
  uint16_t _packetSize;   // expected DMP packet size (default is 42 bytes)
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float ypr[3];           // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector
  VectorInt16 aa;         // [x, y, z]            加速度センサの測定値
  VectorInt16 gyro;       // [x, y, z]            角速度センサの測定値
  VectorInt16 mag;        // [x, y, z]            磁力センサの測定値

  VectorInt16 gyro_origin; // [x, y, z]            角速度センサの測定値
  float yaw_origin;        // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector
public:
  TaskHandle_t pvCreatedTask;

public:
  MrdImuMPU6050() {
    this->_available = false;
    this->gyro_origin.x = 0;
    this->gyro_origin.y = 0;
    this->gyro_origin.z = 0;
    this->yaw_origin = 0;
  }
  ~MrdImuMPU6050() {
  }

  bool setup() override {
    this->_available = false;
    /// @brief MPU6050 センサーのDMP（デジタルモーションプロセッサ）を初期化し,
    ///        ジャイロスコープと加速度センサーのオフセットを設定する.
    /// @param a_ahrs AHRSの値を保持する構造体.
    /// @return DMPの初期化が成功した場合はtrue, 失敗した場合はfalseを返す.
    this->mpu6050.initialize();
    this->devStatus = this->mpu6050.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    this->mpu6050.setXAccelOffset(-1745);
    this->mpu6050.setYAccelOffset(-1034);
    this->mpu6050.setZAccelOffset(966);
    this->mpu6050.setXGyroOffset(176);
    this->mpu6050.setYGyroOffset(-6);
    this->mpu6050.setZGyroOffset(-25);
    // make sure it worked (returns 0 if so)
    if (0 == this->devStatus) {

      this->mpu6050.CalibrateAccel(6);
      this->mpu6050.CalibrateGyro(6);
      this->mpu6050.setDMPEnabled(true);
      this->_packetSize = this->mpu6050.dmpGetFIFOPacketSize();
      this->_available = true;
    }
    return this->_available;
  }

  bool reset() override {
    this->gyro_origin.x = this->gyro.x;
    this->gyro_origin.y = this->gyro.y;
    this->gyro_origin.z = this->gyro.z;
    return true;
  }

  bool refresh(Meridim90Union &a_meridim) override {
    if (this->_available) {
      if (this->mpu6050.dmpGetCurrentFIFOPacket(this->fifoBuffer)) { // Get new data
        this->mpu6050.dmpGetQuaternion(&this->q, this->fifoBuffer);
        this->mpu6050.dmpGetGravity(&this->gravity, &this->q);
        this->mpu6050.dmpGetYawPitchRoll(this->ypr, &this->q, &this->gravity);

        // acceleration values
        this->mpu6050.dmpGetAccel(&this->aa, this->fifoBuffer);
        a_meridim.sval[2] = (float)this->aa.x; // IMU/AHRS_acc_x
        a_meridim.sval[3] = (float)this->aa.y; // IMU/AHRS_acc_y
        a_meridim.sval[4] = (float)this->aa.z; // IMU/AHRS_acc_z

        // gyro values
        this->mpu6050.dmpGetGyro(&this->gyro, this->fifoBuffer);
        a_meridim.sval[5] = (float)(this->gyro.x - this->gyro_origin.x); // IMU/AHRS_gyro_x
        a_meridim.sval[6] = (float)(this->gyro.y - this->gyro_origin.y); // IMU/AHRS_gyro_y
        a_meridim.sval[7] = (float)(this->gyro.z - this->gyro_origin.z); // IMU/AHRS_gyro_z

        // magnetic field values
        a_meridim.sval[8] = 0;  // IMU/AHRS_mag_x
        a_meridim.sval[9] = 0;  // IMU/AHRS_mag_y
        a_meridim.sval[10] = 0; // IMU/AHRS_mag_z

        // Estimated heading value using DMP.

        a_meridim.sval[12] = ((this->ypr[2] * 180) / M_PI);                    // Estimated DMP_ROLL
        a_meridim.sval[13] = ((this->ypr[1] * 180) / M_PI);                    // Estimated DMP_PITCH
        a_meridim.sval[14] = ((this->ypr[0] * 180 / M_PI) - this->yaw_origin); // Estimated DMP_YAW

        // Temperature
        a_meridim.sval[11] = 0; // Not implemented.
        return true;
      }
    }
    return false
  }
};

#endif // MRD_IMU_MPU6050_HPP
