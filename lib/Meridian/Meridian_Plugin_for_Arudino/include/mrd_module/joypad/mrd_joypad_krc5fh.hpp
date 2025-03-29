/**
 * @file mrd_joypad_krc5fh.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_JOYPAD_KRC5FH_HPP
#define MRD_JOYPAD_KRC5FH_HPP
// ライブラリ導入

//================================================================================================================
//  サーボIDとロボット部位、軸との対応表 (KHR-3HVの例)
//================================================================================================================
//
// ID    Parts/Axis ＜ICS_Left_Upper SIO1,SIO2＞
// [L00] 頭/ヨー
// [L01] 左肩/ピッチ
// [L02] 左肩/ロール
// [L03] 左肘/ヨー
// [L04] 左肘/ピッチ
// [L05] -
// ID    Parts/Axis ＜ICS_Left_Lower SIO3,SIO4＞
// [L06] 左股/ロール
// [L07] 左股/ピッチ
// [L08] 左膝/ピッチ
// [L09] 左足首/ピッチ
// [L10] 左足首/ロール
// ID    Parts/Axis  ＜ICS_Right_Upper SIO5,SIO6＞
// [R00] 腰/ヨー
// [R01] 右肩/ピッチ
// [R02] 右肩/ロール
// [R03] 右肘/ヨー
// [R04] 右肘/ピッチ
// [R05] -
// ID    Parts/Axis  ＜ICS_Right_Lower SIO7,SIO8＞
// [R06] 右股/ロール
// [R07] 右股/ピッチ
// [R08] 右膝/ピッチ
// [R09] 右足首/ピッチ
// [R10] 右足首/ロール
#include <mrd_plugin/i_mrd_plugin_joypad.hpp>

constexpr unsigned short PAD_TABLE_KRC5FH_TO_COMMON[16] = { //
    0, 64, 32, 128, 1, 4, 2, 8, 1024, 4096, 512, 2048, 16, 64, 32, 256};

class MrdJoypadKrc5fh : public I_Meridian_Joypad {

public:
  MrdJoypadKrc5fh() {
  }
  ~MrdJoypadKrc5fh() {
  }
  bool setup() override {
    return true;
  }
  const char *get_name() override {
    return "KRC-5FH";
  }

  void set(uint8_t data) override {
  }

  uint8_t get() override {
    return 0;
  }

  bool refresh(Meridim90Union &a_meridim) override {
    return true;
  }

  //----------------------------------------------------------------------
  // KRC-5FHの読み込み
  //----------------------------------------------------------------------

  /// @brief 指定されたジョイパッドタイプに応じて最新データを読み取り, 64ビット整数で返す.
  /// @param a_pad_type ジョイパッドのタイプを示す列挙型（MERIMOTE, BLUERETRO, SBDBT, KRR5FH）.
  /// @param a_pad_data 64ビットのボタンデータ
  /// @return 64ビット整数に変換された受信データ
  /// @note WIIMOTEの場合は, スレッドがpad_array.ui64valを自動更新.
  uint64_t mrd_pad_read(unsigned short krr_button, int krr_analog_tmp[4]) {

    static uint64_t pre_val_tmp = 0; // 前回の値を保持する静的変数
    int8_t pad_analog_tmp[4] = {0};  // アナログ入力のデータ組み立て用
    static int calib[4] = {0};       // アナログスティックのキャリブレーション値

    static unsigned long last_time_tmp = 0; // 最後に関数が呼ばれた時間を記録
    unsigned long current_time_tmp = millis();

    if (current_time_tmp - last_time_tmp >= this->pad_interval) {
      unsigned short krr_button_tmp = krr_button; // krrからのボタン入力データ
                                                  // int krr_analog_tmp = krr_analog;            // krrからのアナログ入力データ
      unsigned short pad_common_tmp = 0;          // PS準拠に変換後のボタンデータ
      delayMicroseconds(2);

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

  bool mrd_joypad_setup(TaskHandle_t &pvCreatedTask, HardwareSerial &a_serial) {
    return true;
  }
  bool meriput90_pad(Meridim90Union &a_meridim, PadUnion a_pad_array, bool a_marge) {

    // ボタンデータの処理 (マージ or 上書き)
    if (a_marge) {
      a_meridim.usval[MRD_PAD_BUTTONS] |= a_pad_array.usval[0];
    } else {
      a_meridim.usval[MRD_PAD_BUTTONS] = a_pad_array.usval[0];
    }

    // アナログ入力データの処理 (上書きのみ)
    for (int i = 1; i < 4; i++) {
      a_meridim.usval[MRD_PAD_BUTTONS + i] = a_pad_array.usval[i];
    }
    return true;
  }

public:
  const int pad_interval = 4; // JOYPADのデータを読みに行くフレーム間隔
  PadUnion pad_array = {0};   // pad値の格納用配列
};

#endif // MRD_JOYPAD_KRC5FH_HPP
