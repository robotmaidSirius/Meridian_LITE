/**
 * @file mrd_joypad_wiimote.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_JOYPAD_WIIMOTE_HPP
#define MRD_JOYPAD_WIIMOTE_HPP

// ライブラリ導入
#include <ESP32Wiimote.h> // Wiiコントローラー
#include <mrd_plugin/i_mrd_plugin_joypad.hpp>
#define PAD_INTERVAL 10 // JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨)

////

ESP32Wiimote wiimote;

// リモコン受信ボタンデータの変換テーブル
constexpr unsigned short PAD_TABLE_WIIMOTE_SOLO[16] = {
    0x1000, 0x0080, 0x0000, 0x0010, 0x0200, 0x0400, 0x0100, 0x0800,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0008, 0x0001, 0x0002, 0x0004};
constexpr unsigned short PAD_TABLE_WIIMOTE_ORIG[16] = {
    0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x0000, 0x0000, 0x0000,
    0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0000, 0x0000, 0x0080};

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

class MrdPadWiimote : public I_Meridian_Joypad {
public:
  const int pad_interval = PAD_INTERVAL; // JOYPADのデータを読みに行くフレーム間隔
  PadUnion pad_array = {0};              // pad値の格納用配列

public:
  MrdPadWiimote() {
  }
  ~MrdPadWiimote() {
  }
  const char *get_name() { return "Wiimote"; }
  void set(uint8_t data) override {
  }
  uint8_t get() override {
    return 0;
  }
  bool refresh(Meridim90Union &a_meridim) override {
    return false;
  };

  ///////////////////////////////////////////////////////////

#if 0
  enum PadType {   // リモコン種の列挙型(NONE, PC, MERIMOTE, BLUERETRO, SBDBT, KRR5FH)
    NONE = 0,      // リモコンなし
    PC = 0,        // PCからのPD入力情報を使用
    MERIMOTE = 1,  // MERIMOTE(未導入)
    BLUERETRO = 2, // BLUERETRO(未導入)
    SBDBT = 3,     // SBDBT(未導入)
    KRR5FH = 4,    // KRR5FH
    WIIMOTE = 5,   // WIIMOTE / WIIMOTE + Nunchuk
    WIIMOTE_C = 6, // WIIMOTE+Classic
  };
#endif

  enum PadButton {  // リモコンボタンの列挙型
    PAD_SELECT = 1, // Select
    PAD_HOME = 2,   // HOME
    PAD_L3 = 2,     // L3
    PAD_R3 = 4,     // L4
    PAD_START = 8,  // Start
    PAD_UP = 16,    // 十字上
    PAD_RIGHT = 32, // 十字右
    PAD_DOWN = 64,  // 十字下
    PAD_LEFT = 128, // 十字左
    PAD_L2 = 256,   // L2
    PAD_R2 = 512,   // R2
    PAD_L1 = 1024,  // L1
    PAD_R1 = 2048,  // R1
    PAD_bU = 4096,  // △ 上
    PAD_bR = 8192,  // o 右
    PAD_bD = 16384, // x 下
    PAD_bL = 32768  // ◻︎ 左
  };

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

  //================================================================================================================
  //  タイプ別のJOYPAD読み込み処理
  //================================================================================================================

  //----------------------------------------------------------------------
  // WIIMOTEの読み込み
  //----------------------------------------------------------------------

  //================================================================================================================
  //  各種パッドへの分岐
  //================================================================================================================

  /// @brief 指定されたジョイパッドタイプに応じて最新データを読み取り, 64ビット整数で返す.
  /// @param a_pad_type ジョイパッドのタイプを示す列挙型（MERIMOTE, BLUERETRO, SBDBT, KRR5FH）.
  /// @param a_pad_data 64ビットのボタンデータ
  /// @return 64ビット整数に変換された受信データ
  /// @note WIIMOTEの場合は, スレッドがpad_array.ui64valを自動更新.
  uint64_t mrd_pad_read(uint64_t a_pad_data) {

    return a_pad_data;
  }

  //================================================================================================================
  //  初期化と準備
  //================================================================================================================

  //----------------------------------------------------------------------
  // Bluetooth, WIIMOTEの初期化
  //----------------------------------------------------------------------

  /// @brief Bluetoothの設定を行い, Wiiコントローラの接続を開始する.
  bool mrd_bt_settings(int a_timeout, ESP32Wiimote &a_wiimote, int a_led, HardwareSerial &a_serial) {
    // Wiiコントローラの接続開始
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
      digitalWrite(a_led, HIGH);
      a_serial.println("Wiimote successfully connected. ");
      return true;
    }
    digitalWrite(a_led, LOW);
    return false;
  }

  bool mrd_joypad_setup(TaskHandle_t &pvCreatedTask, HardwareSerial &a_serial) {
    // WIIMOTEの初期化
    mrd_bt_settings(PAD_INIT_TIMEOUT, wiimote, PIN_LED_BT, a_serial);
    xTaskCreatePinnedToCore(Core0_BT_r, "Core0_BT_r", 2048, NULL, 5, &pvCreatedTask, 0);
    return true;
  }

  //------------------------------------------------------------------------------------
  //  meridimへのデータ書き込み
  //------------------------------------------------------------------------------------

  /// @brief meridim配列にPADデータを書き込む.
  /// @param a_meridim Meridim配列の共用体. 参照渡し.
  /// @param a_pad_array PAD受信値の格納用配列.
  /// @param a_marge PADボタンデータをマージするかどうかのブール値.
  /// trueの場合は既存のデータにビット単位でOR演算を行い, falseの場合は新しいデータで上書きする.
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
};
#undef PAD_INTERVAL

#endif // MRD_JOYPAD_WIIMOTE_HPP
