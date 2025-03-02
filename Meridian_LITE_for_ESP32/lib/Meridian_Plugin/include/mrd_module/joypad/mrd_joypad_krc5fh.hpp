/**
 * @file mrd_joypad_krc5fh.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_JOYPAD_KRR5FH_HPP
#define MRD_JOYPAD_KRR5FH_HPP

#define PAD_INTERVAL 4 // JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨)

// リモコンのアナログ入力データ
struct PadValue {
  unsigned short stick_R = 0;
  int stick_R_x = 0;
  int stick_R_y = 0;
  unsigned short stick_L = 0;
  int stick_L_x = 0;
  int stick_L_y = 0;
  unsigned short stick_L2R2V = 0;
  int R2_val = 0;
  int L2_val = 0;
};
PadValue pad_analog;

// ライブラリ導入
#include <IcsHardSerialClass.h> // ICSサーボのインスタンス設定
#include <mrd_plugin/i_mrd_joypad.hpp>

// リモコン受信ボタンデータの変換テーブル
constexpr unsigned short PAD_TABLE_KRC5FH_TO_COMMON[16] = { //
    0, 64, 32, 128, 1, 4, 2, 8, 1024, 4096, 512, 2048, 16, 64, 32, 256};

//==================================================================================================
//  タイプ別のJOYPAD読み込み処理
//==================================================================================================

//----------------------------------------------------------------------
// KRC-5FHの読み込み
//----------------------------------------------------------------------

/// @brief KRC-5FHジョイパッドからデータを読み取り, 指定された間隔でデータを更新する.
/// @param a_interval 読み取り間隔（ミリ秒）.
/// @return 更新されたジョイパッドの状態を64ビット整数で返す.
uint64_t mrd_pad_read_krc(IcsHardSerialClass &a_ics) {
  static uint64_t pre_val_tmp = 0; // 前回の値を保持する静的変数
  int8_t pad_analog_tmp[4] = {0};  // アナログ入力のデータ組み立て用
  static int calib[4] = {0};       // アナログスティックのキャリブレーション値

  static unsigned long last_time_tmp = 0; // 最後に関数が呼ばれた時間を記録
  unsigned long current_time_tmp = millis();

  if (current_time_tmp - last_time_tmp >= PAD_INTERVAL) {
    unsigned short krr_button_tmp;     // krrからのボタン入力データ
    int krr_analog_tmp[4];             // krrからのアナログ入力データ
    unsigned short pad_common_tmp = 0; // PS準拠に変換後のボタンデータ
    bool rcvd_tmp;                     // 受信機がデータを受信成功したか
    rcvd_tmp = a_ics.getKrrAllData(&krr_button_tmp, krr_analog_tmp);
    delayMicroseconds(2);

    if (rcvd_tmp) // リモコンデータが受信できていたら
    {
      // ボタンデータの処理
      int button_tmp = krr_button_tmp; // 受信ボタンデータの読み込み用

      if (PAD_GENERALIZE) {            // ボタンデータの一般化処理
        if ((button_tmp & 15) == 15) { // 左側十字ボタン全部押しなら select押下とみなす
          pad_common_tmp += 1;
          button_tmp &= 0b1111111111110000; // 左十字ボタンのクリア
        }

        if ((button_tmp & 368) == 368) {
          pad_common_tmp += 8;              // 右側十字ボタン全部押しなら start押下とみなす
          button_tmp &= 0b1111111010001111; // 右十字ボタンのクリア
        }

        // ボタン値の変換(一般化)
        for (int i = 0; i < 16; i++) {
          uint16_t mask_tmp = 1 << i;
          if (PAD_TABLE_KRC5FH_TO_COMMON[i] & button_tmp) {
            pad_common_tmp |= mask_tmp;
          }
        }
        pad_common_tmp &= 0b11111111111111001; // 2と4のビットのクリア(謎のデータ調整)

        // アナログ入力データの処理
        if (krr_analog_tmp[0] + krr_analog_tmp[1] + krr_analog_tmp[2] + krr_analog_tmp[3]) {
          for (int i = 0; i < 4; i++) {
            pad_analog_tmp[i] = (krr_analog_tmp[i] - 62) << 2;
            pad_analog_tmp[i] = (pad_analog_tmp[i] < -127) ? -127 : pad_analog_tmp[i];
            pad_analog_tmp[i] = (pad_analog_tmp[i] > 127) ? 127 : pad_analog_tmp[i];
          }
        } else
          for (int i = 0; i < 4; i++) {
            pad_analog_tmp[i] = 0;
          }
      } else {
        pad_common_tmp = button_tmp; // ボタンの変換なし生値を使用
      }
    }

    // アナログスティックのキャリブレーション
    // [WIP]

    // データの組み立て
    uint64_t updated_val_tmp = 0;
    updated_val_tmp = static_cast<uint64_t>(pad_common_tmp);
    updated_val_tmp |= ((uint64_t)pad_analog_tmp[0] & 0xFF) << 16;
    updated_val_tmp |= ((uint64_t)pad_analog_tmp[1] & 0xFF) << 24;
    //   updated_val_tmp |= ((uint64_t)pad_analog_tmp[2] & 0xFF) << 32;
    //   updated_val_tmp |= ((uint64_t)pad_analog_tmp[3] & 0xFF) << 40;

    last_time_tmp = current_time_tmp; // 最後の実行時間を更新
    pre_val_tmp = updated_val_tmp;
    return updated_val_tmp;
  }
  return pre_val_tmp;
}

//==================================================================================================
//  各種パッドへの分岐
//==================================================================================================

#undef PAD_INTERVAL
#undef PAD_LEN

#endif
