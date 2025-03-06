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

// 各サーボ系統の最大サーボマウント数
#define IXL_MAX 15 // L系統の最大サーボ数. 標準は15.
#define IXR_MAX 15 // R系統の最大サーボ数. 標準は15.

// サーボ用変数
struct ServoParam {
  // サーボの最大接続 (サーボ送受信のループ処理数）
  int num_max;

  // 各サーボのマウントありなし(config.hで設定)
  int ixl_mount[IXL_MAX]; // L系統
  int ixr_mount[IXR_MAX]; // R系統

  // 各サーボのコード上のインデックスに対し, 実際に呼び出すハードウェアのID番号(config.hで設定)
  int ixl_id[IXL_MAX]; // L系統の実サーボ呼び出しID番号
  int ixr_id[IXR_MAX]; // R系統の実サーボ呼び出しID番号

  // 各サーボの正逆方向補正用配列(config.hで設定)
  int ixl_cw[IXL_MAX]; // L系統
  int ixr_cw[IXR_MAX]; // R系統

  // 各サーボの直立ポーズトリム値(config.hで設定)
  float ixl_trim[IXL_MAX]; // L系統
  float ixr_trim[IXR_MAX]; // R系統

  // 各サーボのポジション値(degree)
  float ixl_tgt[IXL_MAX] = {0};      // L系統の目標値
  float ixr_tgt[IXR_MAX] = {0};      // R系統の目標値
  float ixl_tgt_past[IXL_MAX] = {0}; // L系統の前回の値
  float ixr_tgt_past[IXR_MAX] = {0}; // R系統の前回の値

  // サーボのエラーカウンタ配列
  int ixl_err[IXL_MAX] = {0}; // L系統
  int ixr_err[IXR_MAX] = {0}; // R系統

  // サーボのコンディションステータス配列
  uint16_t ixl_stat[IXL_MAX] = {0}; // L系統サーボのコンディションステータス配列
  uint16_t ixr_stat[IXR_MAX] = {0}; // R系統サーボのコンディションステータス配列
};

#endif // SV_COMMON_HPP
