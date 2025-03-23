/**
 * @file sv_common.hpp
 * @brief サーボ用定義
 * @version 1.2.0
 * @date 2025-03-03
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef SV_COMMON_HPP
#define SV_COMMON_HPP

#include <stdint.h>

// 各サーボ系統の最大サーボマウント数
#define IXL_MAX 15 // L系統の最大サーボ数. 標準は15.
#define IXR_MAX 15 // R系統の最大サーボ数. 標準は15.

enum ServoType { // サーボプロトコルのタイプ
  NOSERVO = 0,   // サーボなし
  PWM_S = 1,     // Single PWM (WIP)
  PCA9685 = 11,  // I2C_PCA9685 to PWM (WIP)
  FTBRSX = 21,   // FUTABA_RSxTTL (WIP)
  DXL1 = 31,     // DYNAMIXEL 1.0 (WIP)
  DXL2 = 32,     // DYNAMIXEL 2.0 (WIP)
  KOICS3 = 43,   // KONDO_ICS 3.5 / 3.6
  KOPMX = 44,    // KONDO_PMX (WIP)
  JRXBUS = 51,   // JRPROPO_XBUS (WIP)
  FTCSTS = 61,   // FEETECH_STS (WIP)
  FTCSCS = 62    // FEETECH_SCS (WIP)
};

// サーボ用変数
struct ServoParam {
  struct ServoData {
    int num_max;                   // サーボの最大接続 (サーボ送受信のループ処理数）
    int mount[IXL_MAX];            // 各サーボのマウントありなし(config.hで設定)
    int id[IXL_MAX];               // 各サーボのコード上のインデックスに対し, 実際に呼び出すハードウェアのID番号(config.hで設定)
    int cw[IXL_MAX];               // 各サーボの正逆方向補正用配列(config.hで設定)
    float trim[IXL_MAX];           // 各サーボの直立ポーズトリム値(config.hで設定)
    float tgt[IXL_MAX] = {0};      // 各サーボのポジション値(degree)の目標値
    float tgt_past[IXL_MAX] = {0}; // 各サーボのポジション値(degree)の前回の値
    int err[IXL_MAX] = {0};        // サーボのエラーカウンタ配列
    uint16_t stat[IXL_MAX] = {0};  // サーボのコンディションステータス配列
  };

  ServoData ixl; // L系統
  ServoData ixr; // R系統
};

#endif // SV_COMMON_HPP
