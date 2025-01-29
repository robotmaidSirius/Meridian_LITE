/**
 * @file mrd_module_ahrs_BNO055.hpp
 * @brief デバイス[BNO055]制御クラス
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MRD_MODULE_AHRS_BNO055_HPP__
#define __MRD_MODULE_AHRS_BNO055_HPP__

// ヘッダーファイルの読み込み
#include <mrd_module/mrd_plugin/i_mrd_plugin_i2c.hpp>

// ライブラリ導入
#include <Adafruit_BNO055.h> // 9軸センサBNO055用ライブラリ
#include <Arduino.h>
#include <Wire.h>

namespace meridian {
namespace modules {
namespace plugin {
namespace ahrs_bno055 {
struct st_data {
  bool initalized = false;     ///< 初期化フラグ
  imu::Vector<3> accelerator;  ///<  加速度センサ値の取得と表示 - VECTOR_ACCELEROMETER - m/s^2
  imu::Vector<3> gyroscope;    ///< ジャイロセンサ値の取得 - VECTOR_GYROSCOPE - rad/s
  imu::Vector<3> magnetometer; ///< 磁力センサ値の取得と表示  - VECTOR_MAGNETOMETER - uT
  imu::Vector<3> euler;        ///< センサフュージョンによる方向推定値 - VECTOR_EULER - degrees
  unsigned long timestamp = 0;
};

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
volatile bool flag_mrd_ahrs_bno055_loop = false;
st_data a_data;
inline float deg2rad(float deg) { return (deg * M_PI) / 180.0; }
inline float rad2deg(float rad) { return (rad * 180.0) / M_PI; }
inline float deg_correction(float deg) {
  if (deg >= 180.0f) {
    return deg - 360.0f;
  } else if (deg < -180.0f) {
    return deg + 360.0f;
  } else {
    return deg;
  }
}
struct thread_args {
  int delay_ms;
  int32_t sensorID;
  uint8_t address;
  TwoWire *theWire;
};

void thread_mrd_ahrs_bno055(void *args) {
  flag_mrd_ahrs_bno055_loop = true;
  st_data a_ahrs;
  thread_args param = *(thread_args *)args;
  int delay_ms = param.delay_ms;

  Adafruit_BNO055 bno = Adafruit_BNO055(param.sensorID, param.address, param.theWire);
  while (true == flag_mrd_ahrs_bno055_loop) {
    if (!bno.begin()) {
    } else {
      bno.setExtCrystalUse(false);
      a_ahrs.initalized = true;
      break;
    }
    delay(1000);
  }
  while (true == flag_mrd_ahrs_bno055_loop) {
    {
      // 加速度センサ値の取得と表示 - VECTOR_ACCELEROMETER - m/s^2
      a_ahrs.accelerator = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      // ジャイロセンサ値の取得 - VECTOR_GYROSCOPE - rad/s
      a_ahrs.gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

      // 磁力センサ値の取得と表示  - VECTOR_MAGNETOMETER - uT
      a_ahrs.magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

      // センサフュージョンによる方向推定値の取得と表示 - VECTOR_EULER - degrees
      a_ahrs.euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

      // タイムスタンプの更新
      a_ahrs.timestamp = (a_ahrs.timestamp + 1) % 0xFFFFu;
      // データの更新
      if (0 == pthread_mutex_lock(&mutex)) {
        memcpy(&a_data, &a_ahrs, sizeof(st_data));
      }
      pthread_mutex_unlock(&mutex);
    }
    delay(delay_ms);
  }
}

} // namespace ahrs_bno055

class MrdAhrsBNO055 : public IMeridianI2C {
public:
  MrdAhrsBNO055(int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A, TwoWire *theWire = &Wire) {
    if (nullptr == theWire) {
      theWire = &Wire;
    }
    if (nullptr == this->param) {
      this->param = new ahrs_bno055::thread_args();
    }
    this->param->sensorID = sensorID;
    this->param->address = address;
    this->param->theWire = theWire;
    this->param->delay_ms = 10;
  }
  ~MrdAhrsBNO055() {
    ahrs_bno055::flag_mrd_ahrs_bno055_loop = false;
    vTaskDelete(this->task_handle);
    pthread_mutex_destroy(&ahrs_bno055::mutex);
  }

public:
  void write(uint8_t address, uint8_t data) {}
  uint8_t read(uint8_t address) { return 0; }
  bool setup() override {
    this->a_diag->log_info("Task '%s' created on core %d", this->m_task_name, this->core_id);
    xTaskCreatePinnedToCore(ahrs_bno055::thread_mrd_ahrs_bno055,
                            this->m_task_name,
                            this->m_stack_depth,
                            (void *)this->param,
                            this->m_priority,
                            &this->task_handle,
                            this->core_id);
    return true;
  }
  bool input(Meridim90 &a_meridim) override {
    if (0 == pthread_mutex_lock(&ahrs_bno055::mutex)) {
      memcpy(&this->a_ahrs, &ahrs_bno055::a_data, sizeof(ahrs_bno055::st_data));
    }
    if (0 == pthread_mutex_unlock(&ahrs_bno055::mutex)) {
      // コマンド:MCMD_SENSOR_YAW_CALIB(10002) IMU/AHRSのヨー軸リセット
      if (MCMD_SENSOR_YAW_CALIB == a_meridim.master_command) {
        this->reset();
      }

      a_meridim.accelerator.x = this->float2HfShort(this->a_ahrs.accelerator.x());   //! 加速度センサX値
      a_meridim.accelerator.y = this->float2HfShort(this->a_ahrs.accelerator.y());   //! 加速度センサY値
      a_meridim.accelerator.z = this->float2HfShort(this->a_ahrs.accelerator.z());   //! 加速度センサZ値
      a_meridim.gyroscope.x = this->float2HfShort(this->a_ahrs.gyroscope.x());       //! ジャイロセンサX値
      a_meridim.gyroscope.y = this->float2HfShort(this->a_ahrs.gyroscope.y());       //! ジャイロセンサY値
      a_meridim.gyroscope.z = this->float2HfShort(this->a_ahrs.gyroscope.z());       //! ジャイロセンサZ値
      a_meridim.magnetometer.x = this->float2HfShort(this->a_ahrs.magnetometer.x()); //! 磁力センサX値
      a_meridim.magnetometer.y = this->float2HfShort(this->a_ahrs.magnetometer.y()); //! 磁力センサY値
      a_meridim.magnetometer.z = this->float2HfShort(this->a_ahrs.magnetometer.z()); //! 磁力センサZ値

      // a_meridim.temperature = this->float2HfShort(0); //! 温度センサ値

      // Estimated heading value using DMP.
      if (true == this->m_rest_flag) {
        this->yaw_origin = this->float2HfShort(this->a_ahrs.euler.x());
        this->m_rest_flag = false;
      }
      a_meridim.dmp.roll = this->float2HfShort(this->a_ahrs.euler.z());                                                //! DMP推定ロール方向値
      a_meridim.dmp.pitch = this->float2HfShort(this->a_ahrs.euler.y());                                               //! DMP推定ピッチ方向値
      a_meridim.dmp.yaw = this->float2HfShort(ahrs_bno055::deg_correction(this->a_ahrs.euler.x() - this->yaw_origin)); //! DMP推定ヨー方向値
    }
    return true;
  }

  bool output(Meridim90 &a_meridim) override {
    return true;
  }
  bool reset() {
    this->m_rest_flag = true;
    return this->m_rest_flag;
  }

private:
  short float2HfShort(float val) {
    int _x = round(val * 100);
    if (_x > 32766) {
      _x = 32767;
    } else if (_x < -32766) {
      _x = -32767;
    }
    return static_cast<short>(_x);
  }

private:
  // Task Information
  const char *m_task_name = "mrd_ahrs_BNO055"; ///! 表示用タスク名
  const uint32_t m_stack_depth = 4096;         ///! スタックメモリ量
  UBaseType_t m_priority = 2;                  ///! 優先度
  TaskHandle_t task_handle;                    ///! タスクハンドル
  BaseType_t core_id = 0;                      ///! 実行するコア
  const int delay_ms = 10;                     ///! 処理間隔

  ahrs_bno055::st_data a_ahrs; ///! MPU6050のAHRSデータ
  ahrs_bno055::thread_args *param = new ahrs_bno055::thread_args();

  uint16_t m_address;
  bool m_rest_flag = false;
  float yaw_origin = 0.0;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MRD_MODULE_AHRS_BNO055_HPP__
