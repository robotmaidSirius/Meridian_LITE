/**
 * @file mrd_module_servo_ics.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-29
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MRD_MODULE_SERVO_ICS_HPP__
#define __MRD_MODULE_SERVO_ICS_HPP__

// ヘッダーファイルの読み込み
#include <Meridim90.hpp> // メリディアン90のヘッダファイル
#include <mrd_module/mrd_plugin/i_mrd_plugin_servo.hpp>

// ライブラリ導入
#include <IcsHardSerialClass.h> // ICSサーボのインスタンス設定

#define PAD_GENERALIZE 1 // ジョイパッドの入力値をPS系に一般化する

//================================================================================================================
//  サーボIDとロボット部位、軸との対応表 (KHR-3HVの例)
//================================================================================================================
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

namespace meridian {
namespace modules {
namespace plugin {
namespace krc5fh {

// リモコン受信ボタンデータの変換テーブル
constexpr unsigned short PAD_TABLE_KRC5FH_TO_COMMON[16] = { //
    0, 64, 32, 128, 1, 4, 2, 8, 1024, 4096, 512, 2048, 16, 64, 32, 256};
} // namespace krc5fh

class MrdServoICS : public IMeridianServo {
public:
  IcsHardSerialClass ics;                    ///! ICSサーボのインスタンス
  Meridim90Servo servo[MERIDIM90_SERVO_NUM]; ///! サーボのコマンドと値

public:
  MrdServoICS() {
    for (int i = 0; i < MERIDIM90_SERVO_NUM; i++) {
      this->servo[i].id = 0xFF;
      this->servo[i].cmd = COMMAND_FREE;
      this->servo[i].value = 0;
      this->a_err_cnt[i] = 0;
    }
  }
  ~MrdServoICS() {}

public:
  void set_meridim90_start_index(int a_index, int a_max) {
    assert(0 <= a_index && a_index < MERIDIM90_SERVO_NUM);
    assert(0 < a_max && a_max <= MERIDIM90_SERVO_NUM);
    assert((a_index + a_max) <= MERIDIM90_SERVO_NUM);
    this->_start_index = a_index;
    this->_servo_max = a_max;
  }
  void setup(HardwareSerial *icsSerial, byte en_pin, long baudrate = 1250000, int timeout = 2) {
    this->_icsSerial = icsSerial;
    this->_en_pin = en_pin;
    this->_baudrate = baudrate;
    this->_timeout = timeout;
    for (int i = 0; i < this->_servo_max; i++) {
      this->servo[i].id = 0xFF;
      this->servo[i].cmd = COMMAND_FREE;
      this->servo[i].value = 0;
      this->a_err_cnt[i] = 0;
    }
  }

public:
  const char *get_name() override { return "ICS3.5/3.6(KONDO,KRS)"; }
  bool setup() override {
    this->_initialzed = ics.begin(this->_icsSerial, this->_en_pin, this->_baudrate, this->_timeout);
    this->set_enable(this->_initialzed);
    return this->_enable;
  }
  bool input(Meridim90 &a_meridim) override {
    int a_interval = 10;

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
      rcvd_tmp = this->ics.getKrrAllData(&krr_button_tmp, krr_analog_tmp);
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
            if (krc5fh::PAD_TABLE_KRC5FH_TO_COMMON[i] & button_tmp) {
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
      uint64_t updated_val_tmp = static_cast<uint64_t>(pad_common_tmp);
      updated_val_tmp |= ((uint64_t)pad_analog_tmp[0] & 0xFF) << 16;
      updated_val_tmp |= ((uint64_t)pad_analog_tmp[1] & 0xFF) << 24;

      last_time_tmp = current_time_tmp; // 最後の実行時間を更新
      pre_val_tmp = updated_val_tmp;
      a_meridim.input_data.control.buttons = pad_analog_tmp[0];
      a_meridim.input_data.control.stick_l = pad_analog_tmp[1];
      a_meridim.input_data.control.stick_r = pad_analog_tmp[2];
      a_meridim.input_data.control.analog_l = (updated_val_tmp >> 16) & 0xFF;
      a_meridim.input_data.control.analog_r = (updated_val_tmp) & 0xFF;
    }
    return true;
  }

  bool output(Meridim90 &a_meridim) override {
    if (this->_enable) {
      this->mrd_servos_drive_lite();
      for (int i = 0; i < this->_servo_max; i++) {
        a_meridim.userdata.servo[this->_start_index + i].cmd = this->servo[i].cmd;
        a_meridim.userdata.servo[this->_start_index + i].id = this->servo[i].id;
        a_meridim.userdata.servo[this->_start_index + i].value = this->servo[i].value;
      }
      a_meridim.err = this->make_errcode_lite();
    }
    return true;
  }

public:
  bool get_enable() { return this->_enable; }
  void set_enable(bool flag) { this->_enable = flag; }

  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  bool mrd_servos_drive_lite() {
    for (int i = 0; i < this->_servo_max; i++) {
      if (this->check_id(this->servo[i].id)) {
        int a_tmp = 0;
        int cmd = ~(COMMAND_READ_ONLY) & this->servo[i].cmd;
        bool read_only = (0 < (this->servo[i].cmd & COMMAND_READ_ONLY)) ? true : false;
        switch (cmd) {
        case COMMAND_SET_POS: ///! サーボのトルクON
          if (false == read_only) {
            a_tmp = this->ics.setPos(this->servo[i].id, this->servo[i].value);
          } else {
            a_tmp = this->ics.getPos(this->servo[i].id);
          }
          break;
        case COMMAND_STRETCH: ///! サーボのストレッチ設定
          if (false == read_only) {
            a_tmp = this->ics.setStrc(this->servo[i].id, this->servo[i].value);
          } else {
            a_tmp = this->ics.getStrc(this->servo[i].id);
          }
          break;
        case COMMAND_SPEED: ///! サーボのスピード設定
          if (false == read_only) {
            a_tmp = this->ics.setSpd(this->servo[i].id, this->servo[i].value);
          } else {
            a_tmp = this->ics.getSpd(this->servo[i].id);
          }
          break;
        case COMMAND_CURRENT: ///! サーボの電流制限設定
          if (false == read_only) {
            a_tmp = this->ics.setCur(this->servo[i].id, this->servo[i].value);
          } else {
            a_tmp = this->ics.getCur(this->servo[i].id);
          }
          break;
        case COMMAND_TEMPERATURE: ///! サーボの温度設定
          if (false == read_only) {
            a_tmp = this->ics.setTmp(this->servo[i].id, this->servo[i].value);
          } else {
            a_tmp = this->ics.getTmp(this->servo[i].id);
          }
          break;
        case COMMAND_ID: ///! サーボのID設定
          if (false == read_only) {
            a_tmp = this->ics.setID(this->servo[i].id);
          } else {
            a_tmp = this->ics.getID();
          }
          break;
        case COMMAND_FREE: ///! サーボのトルクOFF
        default:
          a_tmp = this->ics.setFree(this->servo[i].id);
          break;
        }

        // サーボからの返信信号を受け取れなかった場合
        if (a_tmp == -1) { // サーボからの返信信号を受け取れなかった場合
          this->a_err_cnt[i]++;
          if (this->a_err_cnt[i] >= LOST_COUNT_MAX) { // 一定以上の連続エラーで通信不能とみなす
            this->a_err_cnt[i] &= ERR_CODE;
            this->servo[i].value = 0;
          }
        } else {
          this->servo[i].value = a_tmp;
          if (0 <= a_err_cnt[i]) {
            this->a_err_cnt[i] = ~(ERR_CODE) & this->a_err_cnt[i];
          }
        }
      }
    }
    return true;
  }

  bool all_off() {
    for (int i = 0; i < this->_servo_max; i++) {
      if (this->check_id(this->servo[i].id)) {
        this->servo[i].cmd = COMMAND_FREE;
      }
    }
    return true;
  }
  uint8_t make_errcode_lite() {
    uint8_t servo_ix_tmp = 0;
    for (int i = 0; i < this->_servo_max; i++) {
      if (this->check_id(this->servo[i].id)) {
        if (0 != this->a_err_cnt[i]) {
          servo_ix_tmp = uint8_t(i + 200);
        }
      }
    }
    return servo_ix_tmp;
  }
  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////

private:
  HardwareSerial *_icsSerial;
  byte _en_pin;
  long _baudrate;
  int _timeout;
  bool _initialzed = false;
  bool _enable = false;
  int _index = 0;
  int _start_index = 0;
  int _servo_max = 0;
  int LOST_COUNT_MAX = 6;
  int a_err_cnt[MERIDIM90_SERVO_NUM]; ///! エラーカウント

#if 1

public:
  bool check_id(byte id) {
    if (IcsHardSerialClass::MIN_ID <= id) {
      if (id < this->_servo_max) {
        if (id <= IcsHardSerialClass::MAX_ID) {
          return true;
        }
      }
    }
    return false;
  } // IDチェック

public:
  inline unsigned int range(unsigned int val, unsigned int min, unsigned int max) {
    if (val < min) {
      return min;
    } else if (val > max) {
      return max;
    }
    return val;

  } // 範囲指定

  // IcsBaseClassのメソッドを呼び出し

  // サーボ位置決め設定
  int setFree(byte id) { return this->ics.setFree(id); } // サーボ脱力＋現在値読込

  // 各種パラメータ書込み
  int setID(byte id) { return this->ics.setFree(id); }
  int setPos(byte id, unsigned int pos) { return this->ics.setPos(id, pos); }                                         // 目標値設定
  int setStrc(byte id, unsigned int strc) { return this->ics.setStrc(id, this->range(strc, 1, 127)); }                // ストレッチ書込 1～127  1(弱）  <=>    127(強）
  int setSpd(byte id, unsigned int spd) { return this->ics.setSpd(id, this->range(spd, 1, 127)); }                    // スピード書込   1～127  1(遅い) <=>    127(速い)
  int setCur(byte id, unsigned int current_limit) { return this->ics.setCur(id, this->range(current_limit, 1, 63)); } // 電流制限値書込 1～63   1(低い) <=>    63 (高い)
  int setTmp(byte id, unsigned int tmp_limit) { return this->ics.setTmp(id, this->range(tmp_limit, 1, 127)); }        // 温度上限書込   1～127  127(低温） <=> 1(高温)

  // 各種パラメータ読込み
  int getID() { return this->ics.getID(); }
  int getPos(byte id) { return this->ics.getPos(id); }   // 現在位置読込     ※ICS3.6以降で有効
  int getStrc(byte id) { return this->ics.getStrc(id); } // ストレッチ読込    1～127  1(弱） <=>     127(強）
  int getSpd(byte id) { return this->ics.getSpd(id); }   // スピード読込      1～127  1(遅い)<=>     127(速い)
  int getCur(byte id) { return this->ics.getCur(id); }   // 電流値読込        63←0 | 64→127
  int getTmp(byte id) { return this->ics.getTmp(id); }   // 現在温度読込      127(低温）<=> 0(高温)

  // KRRからボタンデータ受信
  unsigned short getKrrButton() { return this->ics.getKrrButton(); }

  // KRRからPAアナログデータ受信
  int getKrrAnalog(int paCh) { return this->ics.getKrrAnalog(paCh); }

  // KRRから全データ受信
  bool getKrrAllData(unsigned short *button, int adData[4]) { return this->ics.getKrrAllData(button, adData); }

public:
  static int degPos(float deg) { return IcsHardSerialClass::degPos(deg); } // 角度変換 POSから角度へ変換
  static float posDeg(int pos) { return IcsHardSerialClass::degPos(pos); } // 角度変換 角度からPOSへ変換

  // 角度変換 x100 POSから角度へ変換
  static int degPos100(int deg) { return IcsHardSerialClass::degPos100(deg); }
  // 角度変換 x100 角度からPOSへ変換
  static int posDeg100(int pos) { return IcsHardSerialClass::posDeg100(pos); }
#endif
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#undef PAD_GENERALIZE

#endif // __MRD_MODULE_SERVO_ICS_HPP__
