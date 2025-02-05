/**
 * @file mrd_module_pad_wiimote.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MRD_MODULE_PAD_WIIMOTE_HPP__
#define __MRD_MODULE_PAD_WIIMOTE_HPP__

// ヘッダーファイルの読み込み
#include "mrd_module/gpio/mrd_module_gpio_out.hpp"
#include <mrd_module/mrd_plugin/i_mrd_plugin_pad.hpp>

// ライブラリ導入
#include <ESP32Wiimote.h> // Wiiコントローラー

#define PAD_GENERALIZE 1 // ジョイパッドの入力値をPS系に一般化する

namespace meridian {
namespace modules {
namespace plugin {

// リモコン受信ボタンデータの変換テーブル
constexpr unsigned short PAD_TABLE_WIIMOTE_SOLO[16] = {
    0x1000, 0x0080, 0x0000, 0x0010, 0x0200, 0x0400, 0x0100, 0x0800,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0008, 0x0001, 0x0002, 0x0004};
constexpr unsigned short PAD_TABLE_WIIMOTE_ORIG[16] = {
    0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x0000, 0x0000, 0x0000,
    0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0000, 0x0000, 0x0080};

class MrdPadWiimote : public IMeridianPad {
public:
  /// @brief コンストラクタ
  MrdPadWiimote(MrdGpioOut *connect = nullptr) {
    // GPIO接続の初期設定と状態の初期化
    this->m_gpio_connect = connect;
    if (nullptr != this->m_gpio_connect) {
      this->m_gpio_connect->setup();
      this->connect_led(false);
    }
  }

  /// @brief デストラクタ
  ~MrdPadWiimote() {
  }

public:
  const char *get_name() override { return "Wiimote"; }
  bool setup() override {
    bool result = true;
    if (true == this->m_state.initalized) {
      return result;
    }
    wiimote.init();
    wiimote.addFilter(ACTION_IGNORE, FILTER_ACCEL);
    int delay_ms = 50;
    int timeout = 3000 / delay_ms;
    this->wiimote.task();
    while (false == this->wiimote.available()) {
      if (0 >= timeout) {
        break;
      }
      timeout--;
    }

    this->m_state.initalized = (0 >= timeout) ? false : true;
    return this->m_state.initalized;
  }
  ButtonState rcvd_button_tmp;
  NunchukState nunchuk_tmp;

  /// @brief データの入力
  /// @details Thread処理からデータをコピーして、Meridim90に格納する
  bool input(Meridim90 &a_meridim) override {
    static uint64_t pre_val_tmp = 0; // 前回の値を保持する静的変数
    static int calib_l1x = 0;
    static int calib_l1y = 0;

    this->wiimote.task();
    if (0 < this->wiimote.available()) {
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
          new_pad_tmp[0] |= 0x0400;
        } else {
          new_pad_tmp[0] |= 0x2000;
        }
      }

      if (rcvd_button_tmp & BUTTON_Z) { // ヌンチャクZボタンの処理
        if (PAD_GENERALIZE) {
          new_pad_tmp[0] |= 0x0800;
        } else {
          new_pad_tmp[0] |= 0x4000;
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
      a_meridim.input_data.control.buttons = new_pad_tmp[0];
      a_meridim.input_data.control.stick_l = new_pad_tmp[1];
      a_meridim.input_data.control.stick_r = new_pad_tmp[2];
      a_meridim.input_data.control.analog_l = (new_pad_tmp[3] >> 16) & 0xFF;
      a_meridim.input_data.control.analog_r = (new_pad_tmp[3]) & 0xFF;
    }
    return true;
  }

  /// @brief データの出力
  /// @details 制御可能なコントローラー(振動/LED点灯 など）ならば、操作すｒ
  bool output(Meridim90 &a_meridim) override {
    static bool flag_connect = true;
    static int count_connect = 0;
    count_connect++;
    if (count_connect >= this->M_CONNECT_PAIRING_SWITCH_TIMES) {
      count_connect = 0;
      flag_connect != flag_connect;
    }
    // Bluetooth接続確認用ピン(点滅はペアリング,点灯でリンク確立)
    if (0 == this->m_state.option[OPTION_INDEX_LINKED]) {
      this->connect_led(true);
    } else if (1 == this->m_state.option[OPTION_INDEX_PAIRING]) {
      this->connect_led(flag_connect);
    } else {
      this->connect_led(false);
    }

    return true;
  }

private:
  void connect_led(bool is_on) {
    static int previous_value = -1;
    int value = is_on ? 1 : 0;
    if (previous_value != is_on) {
      previous_value = value;
      if (nullptr != this->m_gpio_connect) {
        this->m_gpio_connect->write(value, true);
      }
    }
  }

private:
  ESP32Wiimote wiimote;

  int M_CONNECT_PAIRING_SWITCH_TIMES = 100;
  MrdGpioOut *m_gpio_connect = nullptr;
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#undef PAD_GENERALIZE

#endif // __MRD_MODULE_PAD_WIIMOTE_HPP__
