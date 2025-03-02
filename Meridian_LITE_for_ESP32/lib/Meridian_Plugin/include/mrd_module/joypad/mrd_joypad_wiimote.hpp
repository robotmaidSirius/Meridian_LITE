/**
 * @file mrd_joypad_wiimote.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_PAD_WIIMOTE_HPP
#define MRD_PAD_WIIMOTE_HPP

// システム用の変数
#define PAD_LEN      5  // リモコン用配列の長さ
#define PAD_INTERVAL 10 // JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨)

typedef union // リモコン値格納用
{
  short sval[PAD_LEN];        // short型で4個の配列データを持つ
  uint16_t usval[PAD_LEN];    // 上記のunsigned short型
  int8_t bval[PAD_LEN * 2];   // 上記のbyte型
  uint8_t ubval[PAD_LEN * 2]; // 上記のunsigned byte型
  uint64_t ui64val;           // 上記のunsigned int16型
                              // [0]button, [1]pad.stick_L_x:pad.stick_L_y,
                              // [2]pad.stick_R_x:pad.stick_R_y, [3]pad.L2_val:pad.R2_val
} PadUnion;
PadUnion pad_array = {0}; // pad値の格納用配列

// ライブラリ導入
#include <ESP32Wiimote.h>       // Wiiコントローラー
#include <IcsHardSerialClass.h> // ICSサーボのインスタンス設定
#include <mrd_plugin/i_mrd_joypad.hpp>

// リモコン受信ボタンデータの変換テーブル
constexpr unsigned short PAD_TABLE_WIIMOTE_SOLO[16] = {
    0x1000, 0x0080, 0x0000, 0x0010, 0x0200, 0x0400, 0x0100, 0x0800,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0008, 0x0001, 0x0002, 0x0004};
constexpr unsigned short PAD_TABLE_WIIMOTE_ORIG[16] = {
    0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x0000, 0x0000, 0x0000,
    0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0000, 0x0000, 0x0080};

ESP32Wiimote wiimote;

//----------------------------------------------------------------------
// WIIMOTEの読み込み
//----------------------------------------------------------------------

/// @brief Wiiリモコンからの入力データを受信し, 処理する.
/// @return 更新されたジョイパッドの状態を64ビット整数で返す.
/// @note ESP32Wiimoteインスタンス wiimote, 定数PAD_GENERALIZE を関数内で使用.
uint64_t mrd_bt_read_wiimote() {
  static uint64_t pre_val_tmp = 0; // 前回の値を保持する静的変数
  static int calib_l1x = 0;
  static int calib_l1y = 0;

  // 受信データの問い合わせ
  wiimote.task();
  ButtonState rcvd_button_tmp;
  NunchukState nunchuk_tmp;
  // AccelState accel_tmp;

  if (wiimote.available() > 0) {

    // リモコンデータの取得
    rcvd_button_tmp = wiimote.getButtonState();
    nunchuk_tmp = wiimote.getNunchukState();

    uint16_t new_pad_tmp[4] = {0}; // アナログ入力のデータ組み立て用

    // ボタン値の変換(一般化)
    for (int i = 0; i < 16; i++) {
      uint16_t mask_tmp = 1 << i;
      if ((PAD_GENERALIZE && (PAD_TABLE_WIIMOTE_SOLO[i] & rcvd_button_tmp)) ||
          (!PAD_GENERALIZE && (PAD_TABLE_WIIMOTE_ORIG[i] & rcvd_button_tmp))) {
        new_pad_tmp[0] |= mask_tmp;
      }
    }

    if (rcvd_button_tmp & BUTTON_C) { // ヌンチャクCボタンの処理
      if (PAD_GENERALIZE) {
        new_pad_tmp[0] |= 1024;
      } else {
        new_pad_tmp[0] |= 8192;
      }
    }

    if (rcvd_button_tmp & BUTTON_Z) { // ヌンチャクZボタンの処理
      if (PAD_GENERALIZE) {
        new_pad_tmp[0] |= 2048;
      } else {
        new_pad_tmp[0] |= 16384;
      }
    }

    if (rcvd_button_tmp & BUTTON_HOME) { // ホームボタンでスティックのキャリブレーション
      calib_l1x = nunchuk_tmp.xStick - 127;
      calib_l1y = nunchuk_tmp.yStick - 127;
    }

    // ヌンチャクの値を組み入れ
    new_pad_tmp[1] = ((nunchuk_tmp.xStick - calib_l1x - 127) * 256 //
                      + (nunchuk_tmp.yStick - calib_l1y - 127));

    // データの組み立て
    uint64_t new_val_tmp = 0; // 戻り値格納用
    new_val_tmp = static_cast<uint64_t>(new_pad_tmp[0]);
    new_val_tmp |= ((uint64_t)new_pad_tmp[1] << 16);
    //  new_val_tmp |= ((uint64_t)new_analog_tmp[2]) << 32;
    //  new_val_tmp |= ((uint64_t)new_analog_tmp[3]) << 40;

    pre_val_tmp = new_val_tmp;
    return new_val_tmp;
  }
  return pre_val_tmp;
}

//----------------------------------------------------------------------
// WIIMOTE用スレッド
//----------------------------------------------------------------------

/// @brief サブCPU (Core0) で実行されるBluetooth通信用のルーチン.
/// @param args この関数に渡される引数. 現在は不使用.
/// @note PadUnion型の pad_array.ui64val, 定数PAD_INTERVAL, WIIMOTE を関数内で使用.
void Core0_BT_r(void *args) { // サブCPU(Core0)で実行するプログラム
  while (true) {              // Bluetooth待受用の無限ループ
    pad_array.ui64val = mrd_bt_read_wiimote();
    vTaskDelay(PAD_INTERVAL); // 他のタスクにCPU時間を譲る
  }
}

uint64_t mrd_pad_read_wiimote() {
  return pad_array.ui64val;
}

//==================================================================================================
//  初期化と準備
//==================================================================================================

//----------------------------------------------------------------------
// Bluetooth, WIIMOTEの初期化
//----------------------------------------------------------------------

/// @brief Bluetoothの設定を行い, Wiiコントローラの接続を開始する.
bool mrd_bt_settings(int a_mount_pad,
                     int a_timeout,
                     ESP32Wiimote &a_wiimote,
                     int a_led,
                     HardwareSerial &a_serial) {
  // Wiiコントローラの接続開始
  if (a_mount_pad == 5) {
    a_serial.println("Try to connect Wiimote...");
    a_wiimote.init();
    a_wiimote.addFilter(ACTION_IGNORE, FILTER_ACCEL);

    uint16_t count_tmp = 0;
    unsigned long start_time = millis();
    while (!a_wiimote.available()) {

      // リモコンへの問い合わせ
      a_wiimote.task();

      // タイムアウトチェック
      if (millis() - start_time >= a_timeout) {
        digitalWrite(a_led, LOW);
        a_serial.println("Wiimote connection timed out.");
        return false;
      }

      // LEDの点滅
      count_tmp++;
      if (count_tmp < 500) {
        digitalWrite(a_led, HIGH);
      } else {
        digitalWrite(a_led, LOW);
      }
      if (count_tmp > 1000) {
        a_serial.print(".");
        count_tmp = 0;
      }

      delay(1); // 1ms秒待機して再チェック
    }
    digitalWrite(a_led, HIGH);
    a_serial.println("Wiimote successfully connected. ");
    return true;
  }
  digitalWrite(a_led, LOW);
  return false;
}

#undef PAD_INTERVAL
#undef PAD_LEN

#endif
