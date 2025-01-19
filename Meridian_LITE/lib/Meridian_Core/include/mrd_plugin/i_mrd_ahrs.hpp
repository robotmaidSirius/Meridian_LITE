/**
 * @file i_mrd_ahrs.hpp
 * @brief MeridianCoreで使用するAttitude and heading reference systemのインターフェースクラス
 * @version 1.2.0
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_AHRS_HPP
#define I_MRD_AHRS_HPP

#include "Meridim90.hpp"

#define IMUAHRS_STOCK 4 // MPUで移動平均を取る際の元にする時系列データの個数

class I_Meridian_AHRS {
public:
  // 6軸or9軸センサーの値
  struct AhrsValue {
    uint8_t mpuIntStatus;                // holds actual interrupt status byte from MPU
    uint8_t devStatus;                   // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;                 // expected DMP packet size (default is 42 bytes)
    float ypr[3];                        // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector
    float yaw_origin = 0;                // ヨー軸の補正センター値
    float yaw_source = 0;                // ヨー軸のソースデータ保持用
    float read[16];                      // mpuからの読み込んだ一次データacc_x,y,z,gyro_x,y,z,mag_x,y,z,gr_x,y,z,rpy_r,p,y,temp
    float zeros[16] = {0};               // リセット用
    float ave_data[16];                  // 上記の移動平均値を入れる
    float result[16];                    // 加工後の最新のmpuデータ（二次データ）
    float stock_data[IMUAHRS_STOCK][16]; // 上記の移動平均値計算用のデータストック
    int stock_count = 0;                 // 上記の移動平均値計算用のデータストックを輪番させる時の変数
    long temperature;                    // センサの温度測定値
  };

public:
  virtual ~I_Meridian_AHRS() = default;
  virtual bool setup() = 0;
  virtual bool begin() = 0;

  virtual bool reset() { return true; };

  virtual bool refresh(Meridim90Union &a_meridim) = 0;
};
I_Meridian_AHRS::AhrsValue ahrs;

#endif // I_MRD_AHRS_HPP
