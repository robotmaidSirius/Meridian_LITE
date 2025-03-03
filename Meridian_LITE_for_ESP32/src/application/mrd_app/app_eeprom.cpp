/**
 * @file app_eeprom.cpp
 * @brief EEPROMの関数群
 * @version 1.2.0
 * @date 2025-03-03
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "application/mrd_app.hpp"
#include "mrd_eeprom.h"

meridian::modules::plugin::MrdFsEEPROM mrd_eeprom(EEPROM_SIZE);

void app_eeprom_setup() {
  if (mrd_eeprom.init()) {              // EEPROMの初期化
    mrd_eeprom.protect(EEPROM_PROTECT); // EEPROMの書き込み保護の設定を行う
    ///////////////////////////////////////////////
    // Boot時のEEPROMダンプ表示
    ///////////////////////////////////////////////
    if (EEPROM_DUMP) {
      Serial.println("[EEPROM] Dump at boot");
      mrd_eeprom.print_dump(mrd_eeprom.data, EEPROM_STYLE);
    }
    ///////////////////////////////////////////////
    // 書き込みチェック
    ///////////////////////////////////////////////
    if (EEPROM_CHECK_RW) {
      bool result = mrd_set_eeprom(); // EEPROMに設定値を書き込み
      if (result) {
        mrd_eeprom.print_dump(mrd_eeprom.read(), EEPROM_STYLE); // ダンプ表示
        Serial.println("... Write OK.");
      } else {
        Serial.println("... Write failed.");
      }
      // EEPROM読み込みを実行
      Serial.println("[EEPROM] Read EEPROM");
      mrd_eeprom.print_dump(mrd_eeprom.read(), EEPROM_STYLE); // ダンプ表示
      Serial.println("... Read completed.");
    }
    if (EEPROM_LOAD) {
      mrd_get_eeprom();
    }
  }
}

bool mrd_set_eeprom() {
  Serial.println("[EEPROM] Try to write EEPROM");
  // TODO: 書き込みデータを作成する
  std::vector<short> data;
  data.resize(EEPROM_SIZE);

  int servo_num = 15;
  for (int i = 0; i < servo_num; i++) {
    // 各サーボのマウントありなし（0:サーボなし, +:サーボあり順転, -:サーボあり逆転）
    // 例: IXL_MT[20] = -21; → FUTABA_RSxTTLサーボを逆転設定でマウント
    data[i + (servo_num * 0)] = short(sv.ixl_mount[i] * sv.ixl_cw[i]);
    data[i + (servo_num * 1)] = short(sv.ixr_mount[i] * sv.ixr_cw[i]);
    // 各サーボの直立デフォルト値 degree
    data[i + (servo_num * 2)] = mrd.float2HfShort(sv.ixl_trim[i]);
    data[i + (servo_num * 3)] = mrd.float2HfShort(sv.ixr_trim[i]);
  }

  mrd_eeprom.print_dump(data, EEPROM_STYLE); // ダンプ表示
  // EEPROMに設定値を書き込み
  return mrd_eeprom.write(data);
}

void mrd_get_eeprom() {
  std::vector<short> data = mrd_eeprom.read(); // EEPROMの読み込み

  int servo_num = 15;
  short tmp_servo = 0;
  for (int i = 0; i < servo_num; i++) {
    // 各サーボのマウントありなし（0:サーボなし, +:サーボあり順転, -:サーボあり逆転）
    // 例: IXL_MT[20] = -21; → FUTABA_RSxTTLサーボを逆転設定でマウント
    tmp_servo = data[i + (servo_num * 0)];
    if (tmp_servo == 0) {
      sv.ixl_mount[i] = 0;
      sv.ixl_cw[i] = 0;
    } else if (tmp_servo > 0) {
      sv.ixl_mount[i] = tmp_servo;
      sv.ixl_cw[i] = 1;
    } else {
      sv.ixl_mount[i] = tmp_servo * -1;
      sv.ixl_cw[i] = -1;
    }
    tmp_servo = data[i + (servo_num * 1)];
    if (tmp_servo == 0) {
      sv.ixr_mount[i] = 0;
      sv.ixr_cw[i] = 0;
    } else if (tmp_servo > 0) {
      sv.ixr_mount[i] = tmp_servo;
      sv.ixr_cw[i] = 1;
    } else {
      sv.ixr_mount[i] = tmp_servo * -1;
      sv.ixr_cw[i] = -1;
    }
    // 各サーボの直立デフォルト値 degree
    sv.ixl_trim[i] = mrd.HfShort2float(data[i + (servo_num * 2)]);
    sv.ixr_trim[i] = mrd.HfShort2float(data[i + (servo_num * 3)]);
  }
}
