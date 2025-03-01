/**
 * @file mrd_module_ahrs_BNO055.hpp
 * @brief デバイス[BNO055]制御クラス
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 * @todo
 *  - 初期設定-動作モード
 *    - setMode()の実装
 *    - OFFSETの実装
 *      - setSensorOffsets()の実装
 *      - setSensorOffsets()の実装
 *    - 電源モードの設定
 *      - enterSuspendMode()の実装
 *      - enterNormalMode()の実装
 *
 * - 関数の機能の調査
 *  - setAxisRemap()
 *  - setAxisSign()
 * - キャリブレーションの実装
 *
 *  スレッドじゃなくて、定期的なタイマーに変更する？
 *
 * [ISSUE]
 *  タイムアウト(i2cWriteReadNonStop returned Error 263)時に値を取得しなおすようにライブラリから手を加えないといけないかも
 *    - エラーハンドリング出来ておらず、エラー検知時の対応が出来ずにいるため
 *
 * @note
 *  ENABLE_SEMAPHORE_BNO055はoutput()関数の最後にセマフォを与える処理を行うため、
 *  最大１サイクル遅れで更新する仕組みにしている
 *  理由は、他デバイスと通信するためinput()関数時に更新を行うと、更新時間が発生するため
 */
#ifndef __MRD_MODULE_AHRS_BNO055_HPP__
#define __MRD_MODULE_AHRS_BNO055_HPP__

// ヘッダーファイルの読み込み
#include <mrd_module/mrd_plugin/i_mrd_plugin_i2c.hpp>

// ライブラリ導入
#include <Adafruit_BNO055.h> // 9軸センサBNO055用ライブラリ
#include <Arduino.h>
#include <Wire.h>

#define DEBUG_OUTPUT_BNO055     0
#define ENABLE_PIN_BNO055       0
#define ENABLE_SEMAPHORE_BNO055 1

namespace meridian {
namespace modules {
namespace plugin {
namespace ahrs_bno055 {
struct st_data {
  bool initalized = false; ///< 初期化フラグ

  // sensors_event_t data;
  imu::Vector<3> acceleration; ///<  加速度センサ値の取得と表示 - VECTOR_ACCELEROMETER - m/s^2
  imu::Vector<3> gyro;         ///< ジャイロセンサ値の取得 - VECTOR_GYROSCOPE - rad/s
  imu::Vector<3> magnetic;     ///< 磁力センサ値の取得と表示  - VECTOR_MAGNETOMETER - uT
  imu::Vector<3> orientation;  ///< センサフュージョンによる方向推定値 - VECTOR_EULER - degrees
  int temperature = 0;
  unsigned long timestamp = 0;
};
struct thread_args {
  int start_delay_ms;
  int search_ms;
  uint8_t reset_pin;
  uint8_t int_pin;
  int32_t sensorID;
  uint8_t address;
  TwoWire *theWire;
};

#if ENABLE_SEMAPHORE_BNO055
volatile SemaphoreHandle_t semaphore_bno055; ///! ハードウェアタイマー用のセマフォ
#endif
pthread_mutex_t mutex_bno055 = PTHREAD_MUTEX_INITIALIZER;
volatile bool flag_mrd_ahrs_bno055_loop = false;
st_data a_data;

void thread_mrd_ahrs_bno055(void *args) {
  flag_mrd_ahrs_bno055_loop = true;
  st_data a_ahrs;
  thread_args param = *(thread_args *)args;
  int delay_ms = max(1, param.search_ms);
  int LONG_SPAN_MS_MAX = (5 * 1000) / delay_ms;
  int long_span_count = LONG_SPAN_MS_MAX;
  // param.theWire->end();
  delay(param.start_delay_ms);
  Adafruit_BNO055 bno = Adafruit_BNO055(param.sensorID, param.address, param.theWire);
  while (true == flag_mrd_ahrs_bno055_loop) {
    if (!bno.begin()) {
    } else {
#if ENABLE_PIN_BNO055
      if (0xFF != param.reset_pin) {
        pinMode(param.reset_pin, OUTPUT);
        digitalWrite(param.reset_pin, LOW);
        delay(100);
        digitalWrite(param.reset_pin, HIGH);
        delay(100);
      }
#endif
      while (param.theWire->available()) {
        param.theWire->read();
      }
      bno.setExtCrystalUse(false);
      a_ahrs.initalized = true;
      break;
    }
    delay(1000);
  }
  sensors_event_t sensors_data;
  while (true == flag_mrd_ahrs_bno055_loop) {
#if ENABLE_SEMAPHORE_BNO055
    if (xSemaphoreTake(semaphore_bno055, portMAX_DELAY) == pdTRUE)
#else
    delay(delay_ms);
#endif
    {
      // 加速度センサ値の取得と表示 - VECTOR_ACCELEROMETER - m/s^2
      a_ahrs.acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      // ジャイロセンサ値の取得 - VECTOR_GYROSCOPE - rad/s
      a_ahrs.gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      // 磁力センサ値の取得と表示  - VECTOR_MAGNETOMETER - uT
      a_ahrs.magnetic = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      // センサフュージョンによる方向推定値の取得と表示 - VECTOR_EULER - degrees
      a_ahrs.orientation = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

      long_span_count++;
      if (long_span_count >= LONG_SPAN_MS_MAX) {
        long_span_count -= LONG_SPAN_MS_MAX;
        // 温度センサ値
        a_ahrs.temperature = bno.getTemp();
      }

      // タイムスタンプの更新
      a_ahrs.timestamp = (a_ahrs.timestamp + 1) % 0xFFFFu;

      // データの更新
      if (0 == pthread_mutex_lock(&mutex_bno055)) {
        memcpy(&a_data, &a_ahrs, sizeof(st_data));
      }
      pthread_mutex_unlock(&mutex_bno055);
    }
  }
}

} // namespace ahrs_bno055

class MrdAhrsBNO055 : public IMeridianI2C {
public:
  virtual const char *get_name() { return "BNO055"; }
  void set_pin(uint8_t reset_pin, uint8_t int_pin = 0xFF) {
    if (nullptr == this->param) {
      this->param = new ahrs_bno055::thread_args();
    }
    this->param->reset_pin = reset_pin;
    this->param->int_pin = int_pin;
  }
  MrdAhrsBNO055(int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A, TwoWire *theWire = &Wire) : IMeridianI2C(address) {
    if (nullptr == theWire) {
      theWire = &Wire;
    }
    if (nullptr == this->param) {
      this->param = new ahrs_bno055::thread_args();
    }
    this->param->sensorID = sensorID;
    this->param->address = address;
    this->param->theWire = theWire;
    this->param->search_ms = 10;
    this->param->start_delay_ms = 100;
#if ENABLE_SEMAPHORE_BNO055
    ahrs_bno055::semaphore_bno055 = xSemaphoreCreateBinary();
#endif
  }
  ~MrdAhrsBNO055() {
    ahrs_bno055::flag_mrd_ahrs_bno055_loop = false;
    vTaskDelete(this->task_handle);
    pthread_mutex_destroy(&ahrs_bno055::mutex_bno055);
  }

public:
  bool setup() override {
    bool result = true;
    if (true == ahrs_bno055::a_data.initalized) {
      return result;
    }
    this->m_diag->log_info("Task '%s' created on core %d", this->m_task_name, this->core_id);
    xTaskCreatePinnedToCore(ahrs_bno055::thread_mrd_ahrs_bno055,
                            this->m_task_name,
                            this->m_stack_depth,
                            (void *)this->param,
                            this->m_priority,
                            &this->task_handle,
                            this->core_id);
    return result;
  }
  bool input(Meridim90 &a_meridim) override {
    if (0 == pthread_mutex_lock(&ahrs_bno055::mutex_bno055)) {
      memcpy(&this->a_ahrs, &ahrs_bno055::a_data, sizeof(ahrs_bno055::st_data));
    }
    if (0 == pthread_mutex_unlock(&ahrs_bno055::mutex_bno055)) {
      // コマンド:MCMD_SENSOR_YAW_CALIB(10002) IMU/AHRSのヨー軸リセット
      if (MCMD_SENSOR_YAW_CALIB == a_meridim.master_command) {
        this->reset();
      }

      if (this->is_zero_range(this->a_ahrs.acceleration.x()) || this->is_zero_range(this->a_ahrs.acceleration.y()) || this->is_zero_range(this->a_ahrs.acceleration.z())) {
        a_meridim.input_data.accelerator.x = this->float2HfShort(this->a_ahrs.acceleration.x()); ///! 加速度センサX値
        a_meridim.input_data.accelerator.y = this->float2HfShort(this->a_ahrs.acceleration.y()); ///! 加速度センサY値
        a_meridim.input_data.accelerator.z = this->float2HfShort(this->a_ahrs.acceleration.z()); ///! 加速度センサZ値
      }
      if (this->is_zero_range(this->a_ahrs.gyro.x()) || this->is_zero_range(this->a_ahrs.gyro.y()) || this->is_zero_range(this->a_ahrs.gyro.z())) {
        a_meridim.input_data.gyroscope.x = this->float2HfShort(this->a_ahrs.gyro.x()); ///! ジャイロセンサX値
        a_meridim.input_data.gyroscope.y = this->float2HfShort(this->a_ahrs.gyro.y()); ///! ジャイロセンサY値
        a_meridim.input_data.gyroscope.z = this->float2HfShort(this->a_ahrs.gyro.z()); ///! ジャイロセンサZ値
      }
      if (this->is_zero_range(this->a_ahrs.magnetic.x()) || this->is_zero_range(this->a_ahrs.magnetic.y()) || this->is_zero_range(this->a_ahrs.magnetic.z())) {
        a_meridim.input_data.magnetometer.x = this->float2HfShort(this->a_ahrs.magnetic.x()); ///! 磁力センサX値
        a_meridim.input_data.magnetometer.y = this->float2HfShort(this->a_ahrs.magnetic.y()); ///! 磁力センサY値
        a_meridim.input_data.magnetometer.z = this->float2HfShort(this->a_ahrs.magnetic.z()); ///! 磁力センサZ値
      }

      a_meridim.input_data.temperature = this->float2HfShort(this->a_ahrs.temperature); ///! 温度センサ値

      // Estimated heading value using DMP.
      if (this->is_zero_range(this->a_ahrs.orientation.x()) || this->is_zero_range(this->a_ahrs.orientation.y()) || this->is_zero_range(this->a_ahrs.orientation.z())) {
        if (this->m_rest_flag) {
          this->yaw_origin = this->float2HfShort(this->a_ahrs.orientation.x());
          this->m_rest_flag = false;
        }
        a_meridim.input_data.dmp.roll = this->float2HfShort(this->a_ahrs.orientation.z());                                         ///! DMP推定ロール方向値
        a_meridim.input_data.dmp.pitch = this->float2HfShort(this->a_ahrs.orientation.y());                                        ///! DMP推定ピッチ方向値
        a_meridim.input_data.dmp.yaw = this->float2HfShort(this->deg_correction(this->a_ahrs.orientation.x() - this->yaw_origin)); ///! DMP推定ヨー方向値
      }
    }
    return true;
  }

  bool output(Meridim90 &a_meridim) override {
#if DEBUG_OUTPUT_BNO055
    if (true == ahrs_bno055::a_data.initalized) {
      this->m_diag->log_trace("  BN0055-ACC[%7.2f,%7.2f,%7.2f]GYR[%7.2f,%7.2f,%7.2f]MAG[%7.2f,%7.2f,%7.2f]TMP[%3d] %lu",
                              this->a_ahrs.acceleration.x(), this->a_ahrs.acceleration.y(), this->a_ahrs.acceleration.z(),
                              this->a_ahrs.gyro.x(), this->a_ahrs.gyro.y(), this->a_ahrs.gyro.z(),
                              this->a_ahrs.magnetic.x(), this->a_ahrs.magnetic.y(), this->a_ahrs.magnetic.z(),
                              this->a_ahrs.temperature, a_ahrs.timestamp);
    }
#endif
#if ENABLE_SEMAPHORE_BNO055
    xSemaphoreGiveFromISR(ahrs_bno055::semaphore_bno055, NULL); // セマフォを与える
#endif
    return true;
  }
  bool reset() {
    this->m_rest_flag = true;
    return this->m_rest_flag;
  }

private:
  inline bool is_zero_range(float value) {
    return !(value < 0.0001f && value > -0.0001f);
  }
  inline float deg_correction(float deg) {
    if (deg >= 180.0f) {
      return deg - 360.0f;
    } else if (deg < -180.0f) {
      return deg + 360.0f;
    } else {
      return deg;
    }
  }
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
  const uint32_t m_stack_depth = (1 * 4096);   ///! スタックメモリ量
  UBaseType_t m_priority = 10;                 ///! 優先度
  TaskHandle_t task_handle;                    ///! タスクハンドル
  BaseType_t core_id = 1;                      ///! 実行するコア

  ahrs_bno055::st_data a_ahrs; ///! MPU6050のAHRSデータ
  ahrs_bno055::thread_args *param = new ahrs_bno055::thread_args();

  bool m_rest_flag = false;
  float yaw_origin = 0.0;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#undef DEBUG_OUTPUT_BNO055
#undef ENABLE_PIN_BNO055

#endif // __MRD_MODULE_AHRS_BNO055_HPP__
