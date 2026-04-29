#ifndef __MERIDIAN_WIRE0_H__
#define __MERIDIAN_WIRE0_H__

// ヘッダファイルの読み込み
#include "mrd_common.h"

// ライブラリ導入
#include <Meridian.h> // Meridianのライブラリ導入

//------------------------------------------------------------------------------------
//  初期設定
//------------------------------------------------------------------------------------

/// @brief Wire0 I2C通信を初期化し, 指定されたクロック速度で設定する.
/// @param a_i2c0_speed I2C通信のクロック速度です.
/// @param a_pinSDA SDAのピン番号. 下記と合わせて省略可.
/// @param a_pinSCL SCLのピン番号. 上記と合わせて省略可.
bool mrd_wire0_init_i2c(int a_i2c0_speed, int a_pinSDA = -1, int a_pinSCL = -1);

/// @brief MPU6050センサーのDMP(デジタルモーションプロセッサ)を初期化し,
///        ジャイロスコープと加速度センサーのオフセットを設定する.
/// @return DMPの初期化が成功した場合はtrue, 失敗した場合はfalseを返す.
bool mrd_wire0_init_mpu6050_dmp();

/// @brief BNO055センサーの初期化を試みます.
/// @return BNO055センサーの初期化が成功した場合はtrue, それ以外の場合はfalseを返す.
///         現在, この関数は常にfalseを返すように設定されています.
bool mrd_wire0_init_bno055();

/// @brief 指定されたIMU/AHRSタイプに応じて適切なセンサの初期化を行います.
/// @param a_imuahrs_type 使用するセンサのタイプを示す列挙型です(MPU6050, MPU9250, BNO055).
/// @param a_i2c0_speed I2C通信のクロック速度です.
/// @param a_pinSDA SDAのピン番号.下記と合わせて省略可.
/// @param a_pinSCL SCLのピン番号.上記と合わせて省略可.
/// @return センサが正しく初期化された場合はtrueを, そうでない場合はfalseを返す.
bool mrd_wire0_setup(ImuAhrsType a_imuahrs_type, int a_i2c0_speed, int a_pinSDA = -1, int a_pinSCL = -1);

//------------------------------------------------------------------------------------
//  センサデータの取得処理
//------------------------------------------------------------------------------------

/// @brief bno055からI2C経由でデータを読み取るスレッド用関数. IMUAHRS_INTERVALの間隔で実行する.
void mrd_wire0_Core0_bno055_r(void *args);

/// @brief AHRSセンサーからI2C経由でデータを読み取る関数.
/// MPU6050, MPU9250を想定していますが, MPU9250は未実装.
/// 各データは`ahrs.read`配列に格納され, 利用可能な場合は`ahrs.result`にコピーされる.
bool mrd_wire0_read_ahrs_i2c(MrdFlags &a_flg);

//------------------------------------------------------------------------------------
//  meriput
//------------------------------------------------------------------------------------

/// @brief 指定されたIMU/AHRSタイプに基づいて, 計測したAHRSデータを読み込む.
/// @param a_type 使用するセンサのタイプを示す列挙(MPU6050, MPU9250, BNO055).
/// @param a_ahrs_result AHRSから読み取った結果を格納した配列.
/// @return データの書き込みが成功した場合はtrue, それ以外の場合はfalseを返す.
bool meriput90_ahrs(Meridim90Union &a_meridim, int a_type, MERIDIANFLOW::Meridian &mrd, MrdFlags &a_flg);

/// @brief AHRSセンサーのyaw_originを現在のyaw_sourceにキャリブレートする関数.
void mrd_wire0_calibrate_yaw_origin();

#endif // __MERIDIAN_WIRE0_H__
