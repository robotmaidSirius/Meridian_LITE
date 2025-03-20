#ifndef __MERIDIAN_BT_PAD_H__
#define __MERIDIAN_BT_PAD_H__

// ライブラリ導入
#include <ESP32Wiimote.h>       // Wiiコントローラー
#include <IcsHardSerialClass.h> // ICSサーボのインスタンス設定
#include <Meridim90.hpp>        // Meridim90のライブラリ導入

const int PAD_LEN = 5; // リモコン用配列の長さ

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

extern PadUnion pad_array;
namespace meridian {
namespace modules {
namespace plugin {
namespace joypad {
uint64_t joypad_data = 0; // ジョイパッドのデータを格納する変数

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
uint64_t mrd_bt_read_wiimote(ESP32Wiimote &a_wiimote) {
  static uint64_t pre_val_tmp = 0; // 前回の値を保持する静的変数
  static int calib_l1x = 0;
  static int calib_l1y = 0;

  // 受信データの問い合わせ
  a_wiimote.task();
  ButtonState rcvd_button_tmp;
  NunchukState nunchuk_tmp;
  // AccelState accel_tmp;

  if (a_wiimote.available() > 0) {

    // リモコンデータの取得
    rcvd_button_tmp = a_wiimote.getButtonState();
    nunchuk_tmp = a_wiimote.getNunchukState();

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

/// @brief Bluetoothの設定を行い, Wiiコントローラの接続を開始する.
bool mrd_bt_settings(ESP32Wiimote &a_wiimote, int a_timeout, int a_led) {
  // Wiiコントローラの接続開始
  Serial.println("Try to connect Wiimote...");
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
      Serial.println("Wiimote connection timed out.");
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
      Serial.print(".");
      count_tmp = 0;
    }

    delay(1); // 1ms秒待機して再チェック
  }
  digitalWrite(a_led, HIGH);
  Serial.println("Wiimote successfully connected. ");
  return true;
}

/// @brief サブCPU (Core0) で実行されるBluetooth通信用のルーチン.
/// @param args この関数に渡される引数. 現在は不使用.
/// @note PadUnion型の pad_array.ui64val, 定数PAD_INTERVAL, WIIMOTE を関数内で使用.
void task_core_wiimote(void *args) { // サブCPU(Core0)で実行するプログラム
  ESP32Wiimote wiimote;
  // Bluetoothの開始と表示(WIIMOTE)
  while (false == mrd_bt_settings(wiimote, PAD_INIT_TIMEOUT, PIN_LED_BT)) {
    /* code */
    delay(1000);
  }

  while (true) { // Bluetooth待受用の無限ループ
    joypad_data = mrd_bt_read_wiimote(wiimote);
    vTaskDelay(PAD_INTERVAL); // 他のタスクにCPU時間を譲る
  }
}
} // namespace joypad

class MrdJoypadNone {
public:
public:
  bool begin() {
    return true;
  }
};

class MrdJoypadWiimote {
private:
  //----------------------------------------------------------------------
  // Bluetooth, WIIMOTEの初期化
  //----------------------------------------------------------------------

  // システム用の変数
  TaskHandle_t thp; // マルチスレッドのタスクハンドル格納用
public:
  bool begin() {
    //----------------------------------------------------------------------
    // WIIMOTE用スレッド
    //----------------------------------------------------------------------
    xTaskCreatePinnedToCore(joypad::task_core_wiimote,
                            "Core0_WIIMOTE_r", 8 * (1024), NULL, 5, &this->thp, 0);
    return true;
  }

  /// @brief 指定されたジョイパッドタイプに応じて最新データを読み取り, 64ビット整数で返す.
  /// @return 64ビット整数に変換された受信データ
  /// @note WIIMOTEの場合は, スレッドがpad_array.ui64valを自動更新.
  uint64_t read() {
    // @[2-3] UDP受信配列から UDP送信配列にデータを転写
    return joypad::joypad_data;
  }
};

class MrdJoypadKRR5FH {
private:
  // リモコン受信ボタンデータの変換テーブル
  const unsigned short PAD_TABLE_KRC5FH_TO_COMMON[16] = { //
      0, 64, 32, 128, 1, 4, 2, 8, 1024, 4096, 512, 2048, 16, 64, 32, 256};

public:
  bool begin() {
    return true;
  }
  //----------------------------------------------------------------------
  // KRC-5FHの読み込み
  //----------------------------------------------------------------------

  /// @brief KRC-5FHジョイパッドからデータを読み取り, 指定された間隔でデータを更新する.
  /// @param a_interval 読み取り間隔（ミリ秒）.
  /// @return 更新されたジョイパッドの状態を64ビット整数で返す.
  uint64_t read(uint a_interval, IcsHardSerialClass &a_ics) {
    static uint64_t pre_val_tmp = 0; // 前回の値を保持する静的変数
    int8_t pad_analog_tmp[4] = {0};  // アナログ入力のデータ組み立て用
    static int calib[4] = {0};       // アナログスティックのキャリブレーション値

    static unsigned long last_time_tmp = 0; // 最後に関数が呼ばれた時間を記録
    unsigned long current_time_tmp = millis();

    if (current_time_tmp - last_time_tmp >= a_interval) {
      unsigned short krr_button_tmp;     // krrからのボタン入力データ
      int krr_analog_tmp[4];             // krrからのアナログ入力データ
      unsigned short pad_common_tmp = 0; // PS準拠に変換後のボタンデータ
      bool rcvd_tmp;                     // 受信機がデータを受信成功したか
      rcvd_tmp = a_ics.getKrrAllData(&krr_button_tmp, krr_analog_tmp);
      delayMicroseconds(2);

      if (rcvd_tmp) { // リモコンデータが受信できていたら
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
          } else {
            for (int i = 0; i < 4; i++) {
              pad_analog_tmp[i] = 0;
            }
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
};

//==================================================================================================
//  タイプ別のJOYPAD読み込み処理
//==================================================================================================

//==================================================================================================
//  各種パッドへの分岐
//==================================================================================================

//==================================================================================================
//  初期化と準備
//==================================================================================================

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

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MERIDIAN_BT_PAD_H__
