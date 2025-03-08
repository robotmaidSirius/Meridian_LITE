#ifndef __MERIDIAN_WIRE0_H__
#define __MERIDIAN_WIRE0_H__

#include "config.h"

// ライブラリ導入
#include <Adafruit_BNO055.h>            // 9軸センサBNO055用
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050用
#include <Meridim90.hpp>                // Meridim90のライブラリ導入
#include <Wire.h>

// 6軸or9軸センサーの値
struct AhrsValue {

  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float ypr[3];           // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector
  float yaw_origin = 0;   // ヨー軸の補正センター値
  float yaw_source = 0;   // ヨー軸のソースデータ保持用

  float read[16]; // mpuからの読み込んだ一次データacc_x,y,z,gyro_x,y,z,mag_x,y,z,gr_x,y,z,rpy_r,p,y,temp

  float zeros[16] = {0};               // リセット用
  float ave_data[16];                  // 上記の移動平均値を入れる
  float result[16];                    // 加工後の最新のmpuデータ（二次データ）
  float stock_data[IMUAHRS_STOCK][16]; // 上記の移動平均値計算用のデータストック
  int stock_count = 0;                 // 上記の移動平均値計算用のデータストックを輪番させる時の変数
  VectorInt16 aa;                      // [x, y, z]            加速度センサの測定値
  VectorInt16 gyro;                    // [x, y, z]            角速度センサの測定値
  VectorInt16 mag;                     // [x, y, z]            磁力センサの測定値
  long temperature;                    // センサの温度測定値
};
extern AhrsValue ahrs;

namespace meridian {
namespace modules {
namespace plugin {

namespace module_MPU6050 {
void mrd_wire0_Core0_mpu6050_r(void *args) {
}
} // namespace module_MPU6050

namespace module_BNO055 {
/// @brief bno055からI2C経由でデータを読み取るスレッド用関数. IMUAHRS_INTERVALの間隔で実行する.
void mrd_wire0_Core0_bno055_r(void *args) {
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // BNO055のインスタンス
  bool result = false;
  while (!result) {
    if (!bno.begin()) {
      Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    } else {
      Serial.println("BNO055 mounted.");
      delay(50);
      bno.setExtCrystalUse(false);
      delay(10);
      result = true;
    }
  }
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
    imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    ahrs.read[6] = magnetometer.x();
    ahrs.read[7] = magnetometer.y();
    ahrs.read[8] = magnetometer.z();

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

    // センサフュージョンの方向推定値のクオータニオン
    // imu::Quaternion quat = bno.getQuat();

    // Serial.print("qW: ");
    // Serial.print(quat.w(), 4);
    // Serial.print(" qX: ");
    // Serial.print(quat.x(), 4);
    // Serial.print(" qY: ");
    // Serial.print(quat.y(), 4);
    // Serial.print(" qZ: ");
    // Serial.println(quat.z(), 4);

    // キャリブレーションのステータスの取得と表示
    // uint8_t system, gyro, accel, mag = 0;
    // bno.getCalibration(&system, &gyro, &accel, &mag);
    // Serial.print("CALIB Sys:");
    // Serial.print(system, DEC);
    // Serial.print(", Gy");
    // Serial.print(gyro, DEC);
    // Serial.print(", Ac");
    // Serial.print(accelerometer.x(), DEC);
    // Serial.print(",");
    // Serial.print(accelerometer.y(), DEC);
    // Serial.print(",");
    // Serial.print(accelerometer.y(), DEC);
    // Serial.println("");
    // Serial.print(", Mg");
    // Serial.println(mag, DEC);

    delay(IMUAHRS_INTERVAL);
  }
}
} // namespace module_BNO055

volatile bool imuahrs_available = true; // メインセンサ値を読み取る間, サブスレッドによる書き込みを待機

class IMrdModuleImu {
public:
  virtual bool begin(AhrsValue &a_ahrs) { return false; }
  virtual bool read(AhrsValue &a_ahrs) { return true; }
  virtual bool read(Meridim90Union &a_meridim, float a_ahrs_result[]) { return true; }

protected:
  void print_data(Meridim90Union &a_meridim) {
    for (int i = 2; i < 15; i++) {
      Serial.print(a_meridim.sval[i]);
      Serial.print(",");
    }
    Serial.println("");
  }
};
class MrdImuMPU6050 : public IMrdModuleImu {
private:
  MPU6050 _mpu6050; // MPU6050のインスタンス
  /// @brief MPU6050センサーのDMP（デジタルモーションプロセッサ）を初期化し,ャイロスコープと加速度センサーのオフセットを設定する.
  /// @param a_ahrs AHRSの値を保持する構造体.
  /// @return DMPの初期化が成功した場合はtrue, 失敗した場合はfalseを返す.
  bool init_mpu6050_dmp(AhrsValue &a_ahrs) {
    this->_mpu6050.initialize();
    a_ahrs.devStatus = this->_mpu6050.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    this->_mpu6050.setXAccelOffset(-1745);
    this->_mpu6050.setYAccelOffset(-1034);
    this->_mpu6050.setZAccelOffset(966);
    this->_mpu6050.setXGyroOffset(176);
    this->_mpu6050.setYGyroOffset(-6);
    this->_mpu6050.setZGyroOffset(-25);

    // make sure it worked (returns 0 if so)
    if (a_ahrs.devStatus == 0) {
      this->_mpu6050.CalibrateAccel(6);
      this->_mpu6050.CalibrateGyro(6);
      this->_mpu6050.setDMPEnabled(true);
      a_ahrs.packetSize = this->_mpu6050.dmpGetFIFOPacketSize();
      Serial.println("MPU6050 OK.");
      return true;
    } else {
      Serial.println("IMU/AHRS DMP Initialization FAILED!");
      return false;
    }
  }
  /**
   * @brief Evaluate checksum of Meridim.
   *
   * @param[in] arr[] Meridim array
   * @param[in] len Length of array
   * @return true Check OK
   * @return false Check NG
   */
  short float2HfShort(float val) {
    int _x = round(val * 100);
    if (_x > 32766) {
      _x = 32767;
    } else if (_x < -32766) {
      _x = -32767;
    }
    return static_cast<short>(_x);
  }

public:
  bool begin(AhrsValue &a_ahrs) {
    return this->init_mpu6050_dmp(a_ahrs);
  }
  /// @brief MPU6050センサーからデータを読み取ります.
  bool read(AhrsValue &a_ahrs) {
    if (this->_mpu6050.dmpGetCurrentFIFOPacket(a_ahrs.fifoBuffer)) { // Get new data
      this->_mpu6050.dmpGetQuaternion(&a_ahrs.q, a_ahrs.fifoBuffer);
      this->_mpu6050.dmpGetGravity(&a_ahrs.gravity, &a_ahrs.q);
      this->_mpu6050.dmpGetYawPitchRoll(a_ahrs.ypr, &a_ahrs.q, &a_ahrs.gravity);

      // acceleration values
      this->_mpu6050.dmpGetAccel(&a_ahrs.aa, a_ahrs.fifoBuffer);
      a_ahrs.read[0] = (float)a_ahrs.aa.x;
      a_ahrs.read[1] = (float)a_ahrs.aa.y;
      a_ahrs.read[2] = (float)a_ahrs.aa.z;

      // gyro values
      this->_mpu6050.dmpGetGyro(&a_ahrs.gyro, a_ahrs.fifoBuffer);
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
  }
  bool read(Meridim90Union &a_meridim, float a_ahrs_result[]) {
    imuahrs_available = false;
    a_meridim.sval[2] = this->float2HfShort(a_ahrs_result[0]);   // IMU/AHRS_acc_x
    a_meridim.sval[3] = this->float2HfShort(a_ahrs_result[1]);   // IMU/AHRS_acc_y
    a_meridim.sval[4] = this->float2HfShort(a_ahrs_result[2]);   // IMU/AHRS_acc_z
    a_meridim.sval[5] = this->float2HfShort(a_ahrs_result[3]);   // IMU/AHRS_gyro_x
    a_meridim.sval[6] = this->float2HfShort(a_ahrs_result[4]);   // IMU/AHRS_gyro_y
    a_meridim.sval[7] = this->float2HfShort(a_ahrs_result[5]);   // IMU/AHRS_gyro_z
    a_meridim.sval[8] = this->float2HfShort(a_ahrs_result[6]);   // IMU/AHRS_mag_x
    a_meridim.sval[9] = this->float2HfShort(a_ahrs_result[7]);   // IMU/AHRS_mag_y
    a_meridim.sval[10] = this->float2HfShort(a_ahrs_result[8]);  // IMU/AHRS_mag_z
    a_meridim.sval[11] = this->float2HfShort(a_ahrs_result[15]); // temperature
    a_meridim.sval[12] = this->float2HfShort(a_ahrs_result[12]); // DMP_ROLL推定値
    a_meridim.sval[13] = this->float2HfShort(a_ahrs_result[13]); // DMP_PITCH推定値
    a_meridim.sval[14] = this->float2HfShort(a_ahrs_result[14]); // DMP_YAW推定値
    imuahrs_available = true;
    // this->print_data(a_meridim);
    return true;
  }
};
class MrdImuBNO055 : public IMrdModuleImu {
private:
  // システム用の変数
  TaskHandle_t thp; // マルチスレッドのタスクハンドル格納用
  /// @brief BNO055センサーの初期化を試みます.
  /// @param a_ahrs AHRSの値を保持する構造体.
  /// @return BNO055センサーの初期化が成功した場合はtrue, それ以外の場合はfalseを返す.
  ///         現在, この関数は常にfalseを返すように設定されています.
  bool init_bno055(AhrsValue &a_ahrs) {
    // データの取得はセンサー用スレッドで実行
    // I2Cスレッドの開始
    xTaskCreatePinnedToCore(module_BNO055::mrd_wire0_Core0_bno055_r, "Core0_bno055_r", 8 * 1024, NULL, 2, &thp, 0);
    Serial.println("Core0 thread for BNO055 start.");
    delay(10);
    return true;
  }
  /**
   * @brief Evaluate checksum of Meridim.
   *
   * @param[in] arr[] Meridim array
   * @param[in] len Length of array
   * @return true Check OK
   * @return false Check NG
   */
  short float2HfShort(float val) {
    int _x = round(val * 100);
    if (_x > 32766) {
      _x = 32767;
    } else if (_x < -32766) {
      _x = -32767;
    }
    return static_cast<short>(_x);
  }

public:
  bool begin(AhrsValue &a_ahrs) {
    return this->init_bno055(a_ahrs);
  }
  /// @brief BNO055センサーからデータを読み取ります.
  bool read(AhrsValue &a_ahrs) {
    return true;
  }
  bool read(Meridim90Union &a_meridim, float a_ahrs_result[]) {
    imuahrs_available = false;
    a_meridim.sval[2] = this->float2HfShort(a_ahrs_result[0]);   // IMU/AHRS_acc_x
    a_meridim.sval[3] = this->float2HfShort(a_ahrs_result[1]);   // IMU/AHRS_acc_y
    a_meridim.sval[4] = this->float2HfShort(a_ahrs_result[2]);   // IMU/AHRS_acc_z
    a_meridim.sval[5] = this->float2HfShort(a_ahrs_result[3]);   // IMU/AHRS_gyro_x
    a_meridim.sval[6] = this->float2HfShort(a_ahrs_result[4]);   // IMU/AHRS_gyro_y
    a_meridim.sval[7] = this->float2HfShort(a_ahrs_result[5]);   // IMU/AHRS_gyro_z
    a_meridim.sval[8] = this->float2HfShort(a_ahrs_result[6]);   // IMU/AHRS_mag_x
    a_meridim.sval[9] = this->float2HfShort(a_ahrs_result[7]);   // IMU/AHRS_mag_y
    a_meridim.sval[10] = this->float2HfShort(a_ahrs_result[8]);  // IMU/AHRS_mag_z
    a_meridim.sval[11] = this->float2HfShort(a_ahrs_result[15]); // temperature
    a_meridim.sval[12] = this->float2HfShort(a_ahrs_result[12]); // DMP_ROLL推定値
    a_meridim.sval[13] = this->float2HfShort(a_ahrs_result[13]); // DMP_PITCH推定値
    a_meridim.sval[14] = this->float2HfShort(a_ahrs_result[14]); // DMP_YAW推定値
    imuahrs_available = true;
    // this->print_data(a_meridim);
    return true;
  }
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MERIDIAN_WIRE0_H__
