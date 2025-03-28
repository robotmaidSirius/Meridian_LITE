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

// ライブラリ導入
#include "mrd_plugin/i_mrd_plugin_ahrs.hpp"
#include <Adafruit_BNO055.h> // 9軸センサBNO055用
Adafruit_BNO055 *bno;
#define IMUAHRS_INTERVAL 10 // IMU/AHRSのセンサの読み取り間隔(ms)
volatile bool imuahrs_available = false;
imu::Vector<3> bno_accelerator;
imu::Vector<3> bno_gyroscope;
imu::Vector<3> bno_magnetmeter;
imu::Vector<3> bno_euler;

/// @brief bno055からI2C経由でデータを読み取るスレッド用関数. IMUAHRS_INTERVALの間隔で実行する.
void mrd_wire0_Core0_bno055_r(void *args) {
  imuahrs_available = true;
  if (true == bno->begin()) {
    bno->setExtCrystalUse(false);
    while (true == imuahrs_available) {

      // 加速度センサ値の取得と表示 - VECTOR_ACCELEROMETER - m/s^2
      bno_accelerator = bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      // ジャイロセンサ値の取得 - VECTOR_GYROSCOPE - rad/s
      bno_gyroscope = bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      // 磁力センサ値の取得と表示  - VECTOR_MAGNETOMETER - uT
      bno_magnetmeter = bno->getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      // センサフュージョンによる方向推定値の取得と表示 - VECTOR_EULER - degrees
      bno_euler = bno->getVector(Adafruit_BNO055::VECTOR_EULER);

      delay(IMUAHRS_INTERVAL);
    }
  }
}

class MrdAhrsBNO055 : public I_Meridian_AHRS<float, float, float> {
private:
  int32_t _sensorID = 55;
  uint8_t _address = BNO055_ADDRESS_A;
  float yaw_origin; // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector

public:
  TaskHandle_t task_handler;

public:
  MrdAhrsBNO055(int32_t sensorID = 55, uint8_t address = BNO055_ADDRESS_A) {
    this->_sensorID = sensorID;
    this->_address = address;
    bno = new Adafruit_BNO055(55, 0x28, &Wire); // BNO055のインスタンス
  }
  ~MrdAhrsBNO055() {
    imuahrs_available = false;
    vTaskDelete(task_handler);
  }

  bool setup() override {
    if (false == imuahrs_available) {
      xTaskCreatePinnedToCore(mrd_wire0_Core0_bno055_r, "Core0_bno055_r", 4096, NULL, 2, &task_handler, 0);
    }
    return true;
  };

  bool reset() override {
    this->yaw_origin = bno_euler.x();
    return true;
  };
  short degree_range(double value) {
    if (value >= 180) {
      return value - 360;
    } else if (value < -180) {
      return value + 360;
    } else {
      return value;
    }
  }
  short to_short(double val) {
    int _x = round(val * 100);
    if (_x > 32766) {
      _x = 32767;
    } else if (_x < -32766) {
      _x = -32767;
    }
    return static_cast<short>(_x);
  }

  bool refresh(Meridim90Union &a_meridim) override {
    a_meridim.sval[2] = this->to_short(bno_accelerator.x());                                   // IMU/AHRS_acc_x
    a_meridim.sval[3] = this->to_short(bno_accelerator.y());                                   // IMU/AHRS_acc_y
    a_meridim.sval[4] = this->to_short(bno_accelerator.z());                                   // IMU/AHRS_acc_z
    a_meridim.sval[5] = this->to_short(bno_gyroscope.x());                                     // IMU/AHRS_gyro_x
    a_meridim.sval[6] = this->to_short(bno_gyroscope.y());                                     // IMU/AHRS_gyro_y
    a_meridim.sval[7] = this->to_short(bno_gyroscope.z());                                     // IMU/AHRS_gyro_z
    a_meridim.sval[8] = this->to_short(bno_magnetmeter.x());                                   // IMU/AHRS_mag_x
    a_meridim.sval[9] = this->to_short(bno_magnetmeter.y());                                   // IMU/AHRS_mag_y
    a_meridim.sval[10] = this->to_short(bno_magnetmeter.z());                                  // IMU/AHRS_mag_z
    a_meridim.sval[11] = this->to_short(0);                                                    // temperature
    a_meridim.sval[12] = this->to_short(bno_euler.y());                                        // DMP_ROLL推定値
    a_meridim.sval[13] = this->to_short(bno_euler.z());                                        // DMP_PITCH推定値
    a_meridim.sval[14] = this->to_short(this->degree_range(bno_euler.x() - this->yaw_origin)); // DMP_YAW推定値
    return true;
  };
};

#endif // MRD_AHRS_BNO055_HPP
