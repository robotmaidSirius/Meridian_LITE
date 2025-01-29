/**
 * @file mrd_module_imu_MPU6050.hpp
 * @brief デバイス[MPU6050]制御クラス
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
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
  bool initalized = false;
  unsigned long timestamp = 0;
};
struct thread_args {
  int delay_ms;
  int32_t sensorID;
  uint8_t address;
  TwoWire *theWire;
};

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
volatile bool flag_mrd_ahrs_mpu6050_loop = false;
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

void thread_mrd_ahrs_mpu6050(void *args) {
  flag_mrd_ahrs_mpu6050_loop = true;
  thread_args param = *(thread_args *)args;
  int delay_ms = param.delay_ms;

  st_data a_ahrs;
  MPU6050 mpu6050;
  while (true == flag_mrd_ahrs_mpu6050_loop) {
    mpu6050.initialize();
    uint8_t devStatus = mpu6050.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu6050.setXAccelOffset(-1745);
    mpu6050.setYAccelOffset(-1034);
    mpu6050.setZAccelOffset(966);
    mpu6050.setXGyroOffset(176);
    mpu6050.setYGyroOffset(-6);
    mpu6050.setZGyroOffset(-25);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      mpu6050.CalibrateAccel(6);
      mpu6050.CalibrateGyro(6);
      mpu6050.setDMPEnabled(true);
      // expected DMP packet size (default is 42 bytes)
      // uint16_t packetSize = a_ahrs.mpu6050.dmpGetFIFOPacketSize();
      a_ahrs.initalized = true;
      break;
    }
    delay(1000);
  }

  uint8_t fifoBuffer[64]; ///! FIFO storage buffer
  float ypr[3];           ///! roll/pitch/yaw container and gravity vector

  while (true == flag_mrd_ahrs_mpu6050_loop) {
    if (mpu6050.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get new data
      mpu6050.dmpGetQuaternion(&a_ahrs.quaternion, fifoBuffer);
      mpu6050.dmpGetGravity(&a_ahrs.gravity, &a_ahrs.quaternion);
      mpu6050.dmpGetYawPitchRoll(ypr, &a_ahrs.quaternion, &a_ahrs.gravity);
      // acceleration values
      mpu6050.dmpGetAccel(&a_ahrs.accelerator, fifoBuffer);
      // gyro values
      mpu6050.dmpGetGyro(&a_ahrs.gyroscope, fifoBuffer);
      // Estimated heading value using DMP.
      a_ahrs.ypr_deg.x = rad2deg(ypr[2]); // Estimated DMP_ROLL
      a_ahrs.ypr_deg.y = rad2deg(ypr[1]); // Estimated DMP_PITCH
      a_ahrs.ypr_deg.z = rad2deg(ypr[0]); // Estimated DMP_YAW

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
} // namespace ahrs_mpu6050

class MrdAhrsMPU6050 : public IMeridianI2C {
public:
  MrdAhrsMPU6050(uint16_t address, int core = 0) {
    this->m_address = address;
    if (nullptr == this->param) {
      this->param = new ahrs_mpu6050::thread_args();
    }
    this->param->address = address;
    this->param->delay_ms = 10;
  }
  ~MrdAhrsMPU6050() {
    ahrs_mpu6050::flag_mrd_ahrs_mpu6050_loop = false;
    vTaskDelete(this->task_handle);
    pthread_mutex_destroy(&ahrs_mpu6050::mutex);
  }

public:
  void write(uint8_t address, uint8_t data) {}
  uint8_t read(uint8_t address) { return 0; }
  bool setup() override {
    this->a_diag->log_info("Task '%s' created on core %d", this->m_task_name, this->core_id);
    xTaskCreatePinnedToCore(ahrs_mpu6050::thread_mrd_ahrs_mpu6050,
                            this->m_task_name,
                            this->m_stack_depth,
                            (void *)this->param,
                            this->m_priority,
                            &this->task_handle,
                            this->core_id);
    return true;
  }
  bool input(Meridim90 &a_meridim) override {
    if (0 == pthread_mutex_lock(&ahrs_mpu6050::mutex)) {
      memcpy(&this->a_ahrs, &ahrs_mpu6050::a_data, sizeof(ahrs_mpu6050::st_data));
    }
    if (0 == pthread_mutex_unlock(&ahrs_mpu6050::mutex)) {
      // コマンド:MCMD_SENSOR_YAW_CALIB(10002) IMU/AHRSのヨー軸リセット
      if (MCMD_SENSOR_YAW_CALIB == a_meridim.master_command) {
        this->reset();
      }

      a_meridim.accelerator.x = this->float2HfShort(this->a_ahrs.accelerator.x); //! 加速度センサX値
      a_meridim.accelerator.y = this->float2HfShort(this->a_ahrs.accelerator.y); //! 加速度センサY値
      a_meridim.accelerator.z = this->float2HfShort(this->a_ahrs.accelerator.z); //! 加速度センサZ値
      a_meridim.gyroscope.x = this->float2HfShort(this->a_ahrs.gyroscope.x);     //! ジャイロセンサX値
      a_meridim.gyroscope.y = this->float2HfShort(this->a_ahrs.gyroscope.y);     //! ジャイロセンサY値
      a_meridim.gyroscope.z = this->float2HfShort(this->a_ahrs.gyroscope.z);     //! ジャイロセンサZ値

      // TODO: magnetometer に gravityをいれている？
      a_meridim.magnetometer.x = this->float2HfShort(this->a_ahrs.gravity.x);
      a_meridim.magnetometer.y = this->float2HfShort(this->a_ahrs.gravity.y);
      a_meridim.magnetometer.z = this->float2HfShort(this->a_ahrs.gravity.z);

      // a_meridim.temperature = this->float2HfShort(0); //! 温度センサ値

      // Estimated heading value using DMP.
      if (true == this->m_rest_flag) {
        this->yaw_origin = this->float2HfShort(this->a_ahrs.ypr_deg.z);
        this->m_rest_flag = false;
      }
      a_meridim.dmp.roll = this->float2HfShort(this->a_ahrs.ypr_deg.x);                                                 //! DMP推定ロール方向値
      a_meridim.dmp.pitch = this->float2HfShort(this->a_ahrs.ypr_deg.y);                                                //! DMP推定ピッチ方向値
      a_meridim.dmp.yaw = this->float2HfShort(ahrs_mpu6050::deg_correction(this->a_ahrs.ypr_deg.z) - this->yaw_origin); //! DMP推定ヨー方向値
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
  const char *m_task_name = "mrd_ahrs_MPU6050"; ///! 表示用タスク名
  const uint32_t m_stack_depth = 4096;          ///! スタックメモリ量
  UBaseType_t m_priority = 2;                   ///! 優先度
  TaskHandle_t task_handle;                     ///! タスクハンドル
  BaseType_t core_id = 0;                       ///! 実行するコア

  ahrs_mpu6050::thread_args *param = new ahrs_mpu6050::thread_args();
  ahrs_mpu6050::st_data a_ahrs; ///! MPU6050のAHRSデータ

  uint16_t m_address;
  bool m_rest_flag = false;
  float yaw_origin = 0.0;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MRD_MODULE_IMU_MPU6050_HPP__
