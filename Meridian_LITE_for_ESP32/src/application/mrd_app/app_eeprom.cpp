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
#include "mrd_module/filesystem/mrd_eeprom.h"

meridian::modules::plugin::MrdFsEEPROM mrd_eeprom(EEPROM_SIZE);

void app_eeprom_setup() {
  if (mrd_eeprom.init(EEPROM_PROTECT)) { // EEPROMの初期化
    // mrd_eeprom.protect(EEPROM_PROTECT); // EEPROMの書き込み保護の設定を行う
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

  for (int i = 0; i < sv.ixl.num_max; i++) {
    // 各サーボのマウントありなし（0:サーボなし, +:サーボあり順転, -:サーボあり逆転）
    // 例: IXL_MT[20] = -21; → FUTABA_RSxTTLサーボを逆転設定でマウント
    data[i + (sv.ixl.num_max * 0)] = short(sv.ixl.mount[i] * sv.ixl.cw[i]);
    // 各サーボの直立デフォルト値 degree
    data[i + (sv.ixl.num_max * 2)] = mrd.float2HfShort(sv.ixl.trim[i]);
  }
  for (int i = 0; i < sv.ixr.num_max; i++) {
    // 各サーボのマウントありなし（0:サーボなし, +:サーボあり順転, -:サーボあり逆転）
    // 例: IXL_MT[20] = -21; → FUTABA_RSxTTLサーボを逆転設定でマウント
    data[i + (sv.ixr.num_max * 1)] = short(sv.ixr.mount[i] * sv.ixr.cw[i]);
    // 各サーボの直立デフォルト値 degree
    data[i + (sv.ixr.num_max * 3)] = mrd.float2HfShort(sv.ixr.trim[i]);
  }

  mrd_eeprom.print_dump(data, EEPROM_STYLE); // ダンプ表示
  // EEPROMに設定値を書き込み
  return mrd_eeprom.write(data);
}

void mrd_get_eeprom() {
  std::vector<short> data = mrd_eeprom.read(); // EEPROMの読み込み

  short tmp_servo = 0;
  for (int i = 0; i < sv.ixl.num_max; i++) {
    // 各サーボのマウントありなし（0:サーボなし, +:サーボあり順転, -:サーボあり逆転）
    // 例: IXL_MT[20] = -21; → FUTABA_RSxTTLサーボを逆転設定でマウント
    tmp_servo = data[i + (sv.ixl.num_max * 0)];
    if (tmp_servo == 0) {
      sv.ixl.mount[i] = 0;
      sv.ixl.cw[i] = 0;
    } else if (tmp_servo > 0) {
      sv.ixl.mount[i] = tmp_servo;
      sv.ixl.cw[i] = 1;
    } else {
      sv.ixl.mount[i] = tmp_servo * -1;
      sv.ixl.cw[i] = -1;
    }
    // 各サーボの直立デフォルト値 degree
    sv.ixl.trim[i] = mrd.HfShort2float(data[i + (sv.ixl.num_max * 2)]);
  }

  for (int i = 0; i < sv.ixr.num_max; i++) {
    // 各サーボのマウントありなし（0:サーボなし, +:サーボあり順転, -:サーボあり逆転）
    // 例: IXL_MT[20] = -21; → FUTABA_RSxTTLサーボを逆転設定でマウント
    tmp_servo = data[i + (sv.ixr.num_max * 1)];
    if (tmp_servo == 0) {
      sv.ixr.mount[i] = 0;
      sv.ixr.cw[i] = 0;
    } else if (tmp_servo > 0) {
      sv.ixr.mount[i] = tmp_servo;
      sv.ixr.cw[i] = 1;
    } else {
      sv.ixr.mount[i] = tmp_servo * -1;
      sv.ixr.cw[i] = -1;
    }
    // 各サーボの直立デフォルト値 degree
    sv.ixr.trim[i] = mrd.HfShort2float(data[i + (sv.ixr.num_max * 3)]);
  }
}
