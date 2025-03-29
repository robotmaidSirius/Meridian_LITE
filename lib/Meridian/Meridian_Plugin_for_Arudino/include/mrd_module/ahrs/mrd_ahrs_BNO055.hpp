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
#include <meridian_core.hpp>

#define PIN_I2C0_SDA     22 // I2CのSDAピン
#define PIN_I2C0_SCL     21 // I2CのSCLピン
#define IMUAHRS_INTERVAL 10 // IMU/AHRSのセンサの読み取り間隔(ms)

void mrd_wire0_Core0_bno055_r(void *args) {
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // BNO055のインスタンス
  while (1) {
    if (true == bno.begin()) {
      // BNO055 mounted.
      delay(50);
      bno.setExtCrystalUse(false);
      delay(10);
      while (1) {
        // 加速度センサ値の取得と表示 - VECTOR_ACCELEROMETER - m/s^2
        imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        ahrs.read[0] = (float)accelerometer.x();
        ahrs.read[1] = (float)accelerometer.y();
        ahrs.read[2] = (float)accelerometer.z();

        // ジャイロセンサ値の取得 - VECTOR_GYROSCOPE - rad/s
        imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        ahrs.read[3] = gyroscope.x();
        ahrs.read[4] = gyroscope.y();
        ahrs.read[5] = gyroscope.z();

        // 磁力センサ値の取得と表示  - VECTOR_MAGNETOMETER - uT
        imu::Vector<3> magnetmeter = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        ahrs.read[6] = magnetmeter.x();
        ahrs.read[7] = magnetmeter.y();
        ahrs.read[8] = magnetmeter.z();

        // センサフュージョンによる方向推定値の取得と表示 - VECTOR_EULER - degrees
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        ahrs.read[12] = euler.y();                   // DMP_ROLL推定値
        ahrs.read[13] = euler.z();                   // DMP_PITCH推定値
        ahrs.yaw_source = euler.x();                 // ヨー軸のソースデータ保持
        float yaw_tmp = euler.x() - ahrs.yaw_origin; // DMP_YAW推定値
        if (yaw_tmp >= 180) {
          yaw_tmp = yaw_tmp - 360;
        } else if (yaw_tmp < -180) {
          yaw_tmp = yaw_tmp + 360;
        }
        ahrs.read[14] = yaw_tmp; // DMP_YAW推定値
        ahrs.ypr[0] = ahrs.read[14];
        ahrs.ypr[1] = ahrs.read[13];
        ahrs.ypr[2] = ahrs.read[12];

        delay(IMUAHRS_INTERVAL);
      }
    }
  }
}

class MrdAhrsBNO055 : public I_Meridian_AHRS {

public:
  TaskHandle_t pvCreatedTask;
  MrdAhrsBNO055() {
  }
  ~MrdAhrsBNO055() {
  }
  bool begin() override {
    xTaskCreatePinnedToCore(mrd_wire0_Core0_bno055_r, "Core0_bno055_r", 4096, NULL, 2, &pvCreatedTask, 0);
    return true;
  }

  bool setup() override {
    int a_i2c0_speed;
    int a_pinSDA = PIN_I2C0_SDA;
    int a_pinSCL = PIN_I2C0_SCL;
    if (a_pinSDA == -1 && a_pinSCL == -1) {
      return mrd_wire0_init_i2c(a_i2c0_speed);
    } else {
      return mrd_wire0_init_i2c(a_i2c0_speed, a_pinSDA, a_pinSCL);
    }
  }

  bool refresh(Meridim90Union &a_meridim) override {
    a_meridim.sval[2] = mrd.float2HfShort(ahrs.read[0]);   // IMU/AHRS_acc_x
    a_meridim.sval[3] = mrd.float2HfShort(ahrs.read[1]);   // IMU/AHRS_acc_y
    a_meridim.sval[4] = mrd.float2HfShort(ahrs.read[2]);   // IMU/AHRS_acc_z
    a_meridim.sval[5] = mrd.float2HfShort(ahrs.read[3]);   // IMU/AHRS_gyro_x
    a_meridim.sval[6] = mrd.float2HfShort(ahrs.read[4]);   // IMU/AHRS_gyro_y
    a_meridim.sval[7] = mrd.float2HfShort(ahrs.read[5]);   // IMU/AHRS_gyro_z
    a_meridim.sval[8] = mrd.float2HfShort(ahrs.read[6]);   // IMU/AHRS_mag_x
    a_meridim.sval[9] = mrd.float2HfShort(ahrs.read[7]);   // IMU/AHRS_mag_y
    a_meridim.sval[10] = mrd.float2HfShort(ahrs.read[8]);  // IMU/AHRS_mag_z
    a_meridim.sval[11] = mrd.float2HfShort(ahrs.read[15]); // temperature
    a_meridim.sval[12] = mrd.float2HfShort(ahrs.read[12]); // DMP_ROLL推定値
    a_meridim.sval[13] = mrd.float2HfShort(ahrs.read[13]); // DMP_PITCH推定値
    a_meridim.sval[14] = mrd.float2HfShort(ahrs.read[14]); // DMP_YAW推定値
    return true;
  }
  bool reset() override {
    ahrs.yaw_origin = ahrs.yaw_source;
    return true;
  };

  /////////////

  //------------------------------------------------------------------------------------
  //  初期設定
  //------------------------------------------------------------------------------------

  /// @brief Wire0 I2C通信を初期化し, 指定されたクロック速度で設定する.
  /// @param a_i2c0_speed I2C通信のクロック速度です.
  /// @param a_pinSDA SDAのピン番号. 下記と合わせて省略可.
  /// @param a_pinSCL SCLのピン番号. 上記と合わせて省略可.
  bool mrd_wire0_init_i2c(int a_i2c0_speed, int a_pinSDA = -1, int a_pinSCL = -1) {
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

  //------------------------------------------------------------------------------------
  //  センサデータの取得処理
  //------------------------------------------------------------------------------------
};

#endif // MRD_AHRS_BNO055_HPP
