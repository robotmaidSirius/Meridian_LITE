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

namespace meridian {
namespace modules {
namespace plugin {

class MrdServoICS : public IMeridianServo {
private:
  static const int ERR_CODE = 0x80;     //! エラーコード
  static const int COMMAND_FREE = 0x00; //! エラーコード
  static const int COMMAND_SET = 0x01;  //! エラーコード
public:
  IcsHardSerialClass ics;
  Meridim90Servo servo[MERIDIM90_SERVO_NUM]; //! サーボのコマンドと値

public:
  MrdServoICS() {
    for (int i = 0; i < MERIDIM90_SERVO_NUM; i++) {
      this->servo[i].cmd = 0xFF;
      this->servo[i].id = 0xFF;
      this->servo[i].option = 0;
      this->servo[i].value = 0;
      this->a_err_cnt[i] = 0;
    }
  }
  ~MrdServoICS() {}

public:
  void set_meridim90_start_index(int index) {
  }
  void setup(HardwareSerial *icsSerial, byte en_pin, long baudrate = 1250000, int timeout = 2) {
    this->_icsSerial = icsSerial;
    this->_en_pin = en_pin;
    this->_baudrate = baudrate;
    this->_timeout = timeout;
    for (int i = 0; i < MERIDIM90_SERVO_NUM; i++) {
      this->servo[i].cmd = 0xFF;
      this->servo[i].id = 0xFF;
      this->servo[i].option = 0;
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
    if (true == this->_enable) {
    }
    return true;
  }
  bool output(Meridim90 &a_meridim) override {
    if (true == this->_enable) {
      for (int i = 0; i < MERIDIM90_SERVO_NUM; i++) {
        a_meridim.servo[i].cmd = this->servo[i].cmd;
        a_meridim.servo[i].id = this->servo[i].id;
        a_meridim.servo[i].option = this->servo[i].option;
        a_meridim.servo[i].value = this->servo[i].value;
      }
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
    for (int i = 0; i < MERIDIM90_SERVO_NUM; i++) {
      if (0 != this->servo[i].cmd) {
        if (true == this->check_id(this->servo[i].id)) {
          int a_pos = 0;
          switch (this->servo[i].cmd) {
          case COMMAND_SET:
            a_pos = this->ics.setPos(this->servo[i].id, this->servo[i].value);
            break;
          case COMMAND_FREE:
          default:
            a_pos = this->ics.setFree(this->servo[i].id);
            break;
          }

          // サーボからの返信信号を受け取れなかった場合
          if (a_pos == -1) { // サーボからの返信信号を受け取れなかった場合
            this->a_err_cnt[i]++;
            if (this->a_err_cnt[i] >= LOST_COUNT_MAX) { // 一定以上の連続エラーで通信不能とみなす
              this->a_err_cnt[i] = LOST_COUNT_MAX;
              this->servo[i].option &= ERR_CODE;
            }
          } else {
            if (0 <= a_err_cnt[i]) {
              this->a_err_cnt[i] = 0;
              this->servo[i].option = ~(ERR_CODE) & this->servo[i].option;
            }
          }
        }
      }
    }
    return true;
  }

  bool mrd_servo_all_off() {
    for (int i = 0; i < MERIDIM90_SERVO_NUM; i++) {
      this->servo[i].cmd = COMMAND_FREE;
    }
    return true;
  }
  uint8_t mrd_servos_make_errcode_lite() {
    uint8_t servo_ix_tmp = 0;
    for (int i = 0; i < MERIDIM90_SERVO_NUM; i++) {
      if (0 != this->servo[i].option) {
        servo_ix_tmp = uint8_t(i + 200);
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
  int LOST_COUNT_MAX = 6;
  int a_err_cnt[MERIDIM90_SERVO_NUM]; //! エラーカウント

#if 1

public:
  // IcsBaseClassのメソッドを呼び出し
  bool
  check_id(byte id) {
    if (IcsHardSerialClass::MIN_ID <= id) {
      if (id < MERIDIM90_SERVO_NUM) {
        if (id <= IcsHardSerialClass::MAX_ID) {
          return true;
        }
      }
    }
    return false;
  } // IDチェック

  // サーボ位置決め設定
  int setFree(byte id) { return this->ics.setFree(id); } // サーボ脱力＋現在値読込

  // 各種パラメータ書込み
  int setID(byte id) { return this->ics.setFree(id); }
  int setPos(byte id, unsigned int pos) { return this->ics.setPos(id, pos); }       // 目標値設定
  int setStrc(byte id, unsigned int strc) { return this->ics.setStrc(id, strc); }   // ストレッチ書込 1～127  1(弱）  <=>    127(強）
  int setSpd(byte id, unsigned int spd) { return this->ics.setSpd(id, spd); }       // スピード書込   1～127  1(遅い) <=>    127(速い)
  int setCur(byte id, unsigned int curlim) { return this->ics.setCur(id, curlim); } // 電流制限値書込 1～63   1(低い) <=>    63 (高い)
  int setTmp(byte id, unsigned int tmplim) { return this->ics.setTmp(id, tmplim); } // 温度上限書込   1～127  127(低温） <=> 1(高温)

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

#endif // __MRD_MODULE_SERVO_ICS_HPP__
