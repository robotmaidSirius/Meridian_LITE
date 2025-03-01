/**
 * @file mrd_module_imu_MPU6050.hpp
 * @brief デバイス[MPU6050]制御クラス
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 *
 * @todo
 *  - Meridim90 の詰めるデータに疑問あり
 *
 */
#ifndef __MRD_MODULE_IMU_MPU6050_HPP__
#define __MRD_MODULE_IMU_MPU6050_HPP__

// ヘッダーファイルの読み込み
#include <mrd_module/mrd_plugin/i_mrd_plugin_i2c.hpp>

// ライブラリ導入
#include <Arduino.h>
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050用ライブラリ
#include <Wire.h>
#define MPU6050_UPDATE_TYPE_TIME_SPAN 0
#define MPU6050_UPDATE_TYPE_INIT_PIN  1
#define MPU6050_UPDATE_TYPE_SEMAPHORE 2

#define DEBUG_OUTPUT_MPU6050  0
#define ENABLE_UPDATE_MPU6050 MPU6050_UPDATE_TYPE_SEMAPHORE

namespace meridian {
namespace modules {
namespace plugin {

namespace ahrs_mpu6050 {
struct st_data {
  VectorInt16 accelerator; ///! 加速度センサの測定値
  VectorInt16 gyroscope;   ///! 角速度センサの測定値
  VectorFloat gravity;     ///! gravity vector
  Quaternion quaternion;   ///! quaternion container
  VectorFloat ypr_deg;     ///! roll/pitch/yaw container and gravity vector
  bool initalized = false; ///< 初期化フラグ
  unsigned long timestamp = 0;
};
struct thread_args {
  int start_delay_ms;
  int search_ms;

  int32_t sensorID;
  uint8_t address;
  int16_t accel_offset_x;
  int16_t accel_offset_y;
  int16_t accel_offset_z;
  int16_t gyro_offset_x;
  int16_t gyro_offset_y;
  int16_t gyro_offset_z;

  uint8_t accel_calibrate;
  uint8_t gyro_calibrate;
};

pthread_mutex_t mutex_mpu6050 = PTHREAD_MUTEX_INITIALIZER;
volatile bool flag_mrd_ahrs_mpu6050_loop = false;
st_data a_data;
inline float deg2rad(float deg) { return (deg * M_PI) / 180.0; }
inline float rad2deg(float rad) { return (rad * 180.0) / M_PI; }

#if ENABLE_UPDATE_MPU6050 != MPU6050_UPDATE_TYPE_TIME_SPAN
volatile SemaphoreHandle_t semaphore_mpu6050; ///! ハードウェアタイマー用のセマフォ
#endif
#if ENABLE_UPDATE_MPU6050 == MPU6050_UPDATE_TYPE_INIT_PIN
void interrupt_pin() {
  xSemaphoreGiveFromISR(semaphore_mpu6050, NULL); // セマフォを与える
}
#endif
void thread_mrd_ahrs_mpu6050(void *args) {
  flag_mrd_ahrs_mpu6050_loop = true;
  st_data local_data;
  thread_args param = *(thread_args *)args;
  int delay_ms = max(1, param.search_ms);
  delay(param.start_delay_ms);
  MPU6050 mpu6050(param.address);
  // データの更新
  local_data.initalized = false;
  if (0 == pthread_mutex_lock(&mutex_mpu6050)) {
    memcpy(&a_data, &local_data, sizeof(st_data));
  }
  pthread_mutex_unlock(&mutex_mpu6050);
  while (flag_mrd_ahrs_mpu6050_loop && (false == local_data.initalized)) {
    mpu6050.initialize();
    if (mpu6050.testConnection()) {
      uint8_t devStatus = mpu6050.dmpInitialize();

      // Accelerometer のオフセット
      mpu6050.setXAccelOffset(param.accel_offset_x);
      mpu6050.setYAccelOffset(param.accel_offset_y);
      mpu6050.setZAccelOffset(param.accel_offset_z);
      // Gyroscope のオフセット
      mpu6050.setXGyroOffset(param.gyro_offset_x);
      mpu6050.setYGyroOffset(param.gyro_offset_y);
      mpu6050.setZGyroOffset(param.gyro_offset_z);

      // make sure it worked (returns 0 if so)
      if (devStatus == 0) {
        mpu6050.CalibrateAccel(param.accel_calibrate);
        mpu6050.CalibrateGyro(param.gyro_calibrate);
        mpu6050.setDMPEnabled(true);
        local_data.initalized = true;
        break;
      }
    }
    delay(5000);
  }

  // uint8_t fifoBuffer[mpu6050.dmpGetFIFOPacketSize()]; ///! FIFO storage buffer
  uint8_t fifoBuffer[64]; ///! FIFO storage buffer
  float ypr[3];           ///! roll/pitch/yaw container and gravity vector

  while (true == flag_mrd_ahrs_mpu6050_loop) {
#if ENABLE_UPDATE_MPU6050 != MPU6050_UPDATE_TYPE_TIME_SPAN
    if (xSemaphoreTake(semaphore_mpu6050, portMAX_DELAY) == pdTRUE)
#else
    delay(delay_ms);
#endif
    {
      if (mpu6050.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get new data
        mpu6050.dmpGetQuaternion(&local_data.quaternion, fifoBuffer);
        mpu6050.dmpGetGravity(&local_data.gravity, &local_data.quaternion);
        mpu6050.dmpGetYawPitchRoll(ypr, &local_data.quaternion, &local_data.gravity);
        // acceleration values
        mpu6050.dmpGetAccel(&local_data.accelerator, fifoBuffer);
        // gyro values
        mpu6050.dmpGetGyro(&local_data.gyroscope, fifoBuffer);
        // Estimated heading value using DMP.
        local_data.ypr_deg.x = rad2deg(ypr[2]); // Estimated DMP_ROLL
        local_data.ypr_deg.y = rad2deg(ypr[1]); // Estimated DMP_PITCH
        local_data.ypr_deg.z = rad2deg(ypr[0]); // Estimated DMP_YAW

        // タイムスタンプの更新
        local_data.timestamp = (local_data.timestamp + 1) % 0xFFFFu;

        // データの更新
        if (0 == pthread_mutex_lock(&mutex_mpu6050)) {
          memcpy(&a_data, &local_data, sizeof(st_data));
        }
        pthread_mutex_unlock(&mutex_mpu6050);
      }
    }
  }
}

} // namespace ahrs_mpu6050

class MrdAhrsMPU6050 : public IMeridianI2C {
public:
  virtual const char *get_name() { return "MPU6050"; }
  void set_pin(uint8_t int_pin) {
    if (nullptr == this->param) {
      this->param = new ahrs_mpu6050::thread_args();
    }
#if ENABLE_UPDATE_MPU6050 == MPU6050_UPDATE_TYPE_INIT_PIN
    pinMode(int_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(int_pin), ahrs_mpu6050::interrupt_pin, RISING);
#endif
  }
  void set_offset_accel(int16_t x, int16_t y, int16_t z) {
    if (nullptr == this->param) {
      this->param = new ahrs_mpu6050::thread_args();
    }
    this->param->accel_offset_x = x;
    this->param->accel_offset_y = y;
    this->param->accel_offset_z = z;
  }
  void set_offset_gyro(int16_t x, int16_t y, int16_t z) {
    if (nullptr == this->param) {
      this->param = new ahrs_mpu6050::thread_args();
    }
    this->param->gyro_offset_x = x;
    this->param->gyro_offset_y = y;
    this->param->gyro_offset_z = z;
  }
  void set_calibrate_accel(uint8_t loops) {
    if (nullptr == this->param) {
      this->param = new ahrs_mpu6050::thread_args();
    }
    this->param->accel_calibrate = loops;
  }
  void set_calibrate_gyro(uint8_t loops) {
    if (nullptr == this->param) {
      this->param = new ahrs_mpu6050::thread_args();
    }
    this->param->gyro_calibrate = loops;
  }
  MrdAhrsMPU6050(bool is_AD0_low = true) : IMeridianI2C(is_AD0_low ? MPU6050_ADDRESS_AD0_LOW : MPU6050_ADDRESS_AD0_HIGH) {
    if (nullptr == this->param) {
      this->param = new ahrs_mpu6050::thread_args();
    }
    this->param->address = this->m_address;

    // デフォルトの設定

#if 0
// TODO:移植元の設定-設定値の根拠を確認する
    this->param->accel_offset_x = -1745;
    this->param->accel_offset_y = -1034;
    this->param->accel_offset_z = 966;
    this->param->gyro_offset_x = 176;
    this->param->gyro_offset_y = -6;
    this->param->gyro_offset_z = -25;
    this->param->accel_calibrate = 6;
    this->param->gyro_calibrate = 6;
#else
    this->param->accel_offset_x = 0;
    this->param->accel_offset_y = 0;
    this->param->accel_offset_z = 0;
    this->param->gyro_offset_x = 0;
    this->param->gyro_offset_y = 0;
    this->param->gyro_offset_z = 0;
    this->param->accel_calibrate = 6;
    this->param->gyro_calibrate = 6;
#endif

    this->param->search_ms = 10;
    this->param->start_delay_ms = 100;
#if ENABLE_UPDATE_MPU6050 != MPU6050_UPDATE_TYPE_TIME_SPAN
    ahrs_mpu6050::semaphore_mpu6050 = xSemaphoreCreateBinary();
#endif
  }
  ~MrdAhrsMPU6050() {
    ahrs_mpu6050::flag_mrd_ahrs_mpu6050_loop = false;
    vTaskDelete(this->task_handle);
    pthread_mutex_destroy(&ahrs_mpu6050::mutex_mpu6050);
  }

public:
  bool setup() override {
    bool result = true;
    if (true == ahrs_mpu6050::a_data.initalized) {
      return result;
    }
    this->m_diag->log_info("Task '%s' created on core %d", this->m_task_name, this->core_id);
    xTaskCreatePinnedToCore(ahrs_mpu6050::thread_mrd_ahrs_mpu6050,
                            this->m_task_name,
                            this->m_stack_depth,
                            (void *)this->param,
                            this->m_priority,
                            &this->task_handle,
                            this->core_id);
    return result;
  }
  bool input(Meridim90 &a_meridim) override {
    if (0 == pthread_mutex_lock(&ahrs_mpu6050::mutex_mpu6050)) {
      memcpy(&this->a_ahrs, &ahrs_mpu6050::a_data, sizeof(ahrs_mpu6050::st_data));
    }
    if (0 == pthread_mutex_unlock(&ahrs_mpu6050::mutex_mpu6050)) {
      // コマンド:MCMD_SENSOR_YAW_CALIB(10002) IMU/AHRSのヨー軸リセット
      if (MCMD_SENSOR_YAW_CALIB == a_meridim.master_command) {
        this->reset();
      }

      if (this->is_zero_range(this->a_ahrs.accelerator.x) || this->is_zero_range(this->a_ahrs.accelerator.y) || this->is_zero_range(this->a_ahrs.accelerator.z)) {
        a_meridim.input_data.accelerator.x = this->float2HfShort(this->a_ahrs.accelerator.x); ///! 加速度センサX値
        a_meridim.input_data.accelerator.y = this->float2HfShort(this->a_ahrs.accelerator.y); ///! 加速度センサY値
        a_meridim.input_data.accelerator.z = this->float2HfShort(this->a_ahrs.accelerator.z); ///! 加速度センサZ値
      }
      if (this->is_zero_range(this->a_ahrs.gyroscope.x) || this->is_zero_range(this->a_ahrs.gyroscope.y) || this->is_zero_range(this->a_ahrs.gyroscope.z)) {
        a_meridim.input_data.gyroscope.x = this->float2HfShort(this->a_ahrs.gyroscope.x); ///! ジャイロセンサX値
        a_meridim.input_data.gyroscope.y = this->float2HfShort(this->a_ahrs.gyroscope.y); ///! ジャイロセンサY値
        a_meridim.input_data.gyroscope.z = this->float2HfShort(this->a_ahrs.gyroscope.z); ///! ジャイロセンサZ値
      }

      // TODO: magnetometer に gravityをいれている？
      if (this->is_zero_range(this->a_ahrs.gravity.x) || this->is_zero_range(this->a_ahrs.gravity.y) || this->is_zero_range(this->a_ahrs.gravity.z)) {
        a_meridim.input_data.magnetometer.x = this->float2HfShort(this->a_ahrs.gravity.x); ///! 重力センサX値
        a_meridim.input_data.magnetometer.y = this->float2HfShort(this->a_ahrs.gravity.y); ///! 重力センサY値
        a_meridim.input_data.magnetometer.z = this->float2HfShort(this->a_ahrs.gravity.z); ///! 重力センサZ値
      }

      // a_meridim.temperature = this->float2HfShort(0); ///! 温度センサ値

      // Estimated heading value using DMP.
      if (this->is_zero_range(this->a_ahrs.ypr_deg.x) || this->is_zero_range(this->a_ahrs.ypr_deg.y) || this->is_zero_range(this->a_ahrs.ypr_deg.z)) {
        if (this->m_rest_flag) {
          this->yaw_origin = this->float2HfShort(this->a_ahrs.ypr_deg.z);
          this->m_rest_flag = false;
        }
        a_meridim.input_data.dmp.roll = this->float2HfShort(this->a_ahrs.ypr_deg.x);                                         ///! DMP推定ロール方向値
        a_meridim.input_data.dmp.pitch = this->float2HfShort(this->a_ahrs.ypr_deg.y);                                        ///! DMP推定ピッチ方向値
        a_meridim.input_data.dmp.yaw = this->float2HfShort(this->deg_correction(this->a_ahrs.ypr_deg.z) - this->yaw_origin); ///! DMP推定ヨー方向値
      }
    }
    return true;
  }

  bool output(Meridim90 &a_meridim) override {
#if DEBUG_OUTPUT_MPU6050
    if (true == ahrs_mpu6050::a_data.initalized) {
      this->m_diag->log_trace("  MPU6050-ACC[%5d,%5d,%5d]GYR[%5d,%5d,%5d]G[%7.2f,%7.2f,%7.2f]YPR[%7.2f,%7.2f,%7.2f] %lu",
                              this->a_ahrs.accelerator.x, this->a_ahrs.accelerator.y, this->a_ahrs.accelerator.z,
                              this->a_ahrs.gyroscope.x, this->a_ahrs.gyroscope.y, this->a_ahrs.gyroscope.z,
                              this->a_ahrs.gravity.x, this->a_ahrs.gravity.y, this->a_ahrs.gravity.z,
                              this->a_ahrs.ypr_deg.x, this->a_ahrs.ypr_deg.y, this->a_ahrs.ypr_deg.z, this->a_ahrs.timestamp);
    }
#endif
#if ENABLE_UPDATE_MPU6050 == MPU6050_UPDATE_TYPE_SEMAPHORE
    xSemaphoreGiveFromISR(ahrs_mpu6050::semaphore_mpu6050, NULL); // セマフォを与える
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
  const char *m_task_name = "mrd_ahrs_MPU6050"; ///! 表示用タスク名
  const uint32_t m_stack_depth = (1 * 4096);    ///! スタックメモリ量
  UBaseType_t m_priority = 10;                  ///! 優先度
  TaskHandle_t task_handle;                     ///! タスクハンドル
  BaseType_t core_id = 1;                       ///! 実行するコア

  ahrs_mpu6050::st_data a_ahrs; ///! MPU6050のAHRSデータ
  ahrs_mpu6050::thread_args *param = new ahrs_mpu6050::thread_args();

  bool m_rest_flag = false;
  float yaw_origin = 0.0;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#undef MPU6050_UPDATE_TYPE_TIME_SPAN
#undef MPU6050_UPDATE_TYPE_INIT_PIN
#undef MPU6050_UPDATE_TYPE_SEMAPHORE

#undef DEBUG_OUTPUT_MPU6050
#undef ENABLE_UPDATE_MPU6050

#endif // __MRD_MODULE_IMU_MPU6050_HPP__
