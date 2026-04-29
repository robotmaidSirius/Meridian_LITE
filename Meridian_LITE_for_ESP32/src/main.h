#ifndef __MERIDIAN_MAIN_FUNC__
#define __MERIDIAN_MAIN_FUNC__

// ヘッダファイルの読み込み
#include "mrd_common.h"

// ライブラリ導入
#include <Adafruit_BNO055.h>            // 9軸センサBNO055用
#include <IcsHardSerialClass.h>         // ICSサーボのインスタンス設定
#include <MPU6050_6Axis_MotionApps20.h> // MPU6050用
#include <Meridian.h>                   // Meridianのライブラリ導入

// 6軸or9軸センサーの値
struct AhrsValue {
  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // BNO055のインスタンス

  MPU6050 mpu6050;        // MPU6050のインスタンス
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
  float result[16];                    // 加工後の最新のmpuデータ(二次データ)
  float stock_data[IMUAHRS_STOCK][16]; // 上記の移動平均値計算用のデータストック
  int stock_count = 0;                 // 上記の移動平均値計算用のデータストックを輪番させる時の変数
  VectorInt16 aa;                      // [x, y, z]            加速度センサの測定値
  VectorInt16 gyro;                    // [x, y, z]            角速度センサの測定値
  VectorInt16 mag;                     // [x, y, z]            磁力センサの測定値
  long temperature;                    // センサの温度測定値
};

#endif //__MERIDIAN_MAIN_FUNC__
