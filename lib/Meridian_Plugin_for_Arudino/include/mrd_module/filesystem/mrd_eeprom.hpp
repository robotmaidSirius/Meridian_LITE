/**
 * @file mrd_eeprom.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-20
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_EEPROM_HPP
#define MRD_EEPROM_HPP

// ライブラリ導入
#include "mrd_plugin/i_mrd_plugin_eeprom.hpp"
#include <EEPROM.h>

// EEPROMの設定
#define EEPROM_SIZE    540 // 使用するEEPROMのサイズ(バイト)
#define EEPROM_SET     0   // 起動時にEEPROMにconfig.hの内容をセット(mrd_set_eeprom)
#define EEPROM_PROTECT 0   // EEPROMの書き込み保護(0:保護しない, 1:書き込み禁止)
#define EEPROM_LOAD    0   // 起動時にEEPROMの内容を諸設定にロードする(未導入)
#define EEPROM_DUMP    0   // 起動時のEEPROM内容のダンプ表示
#define EEPROM_STYLE   Dec // 起動時のEEPROM内容のダンプ表示の書式(Bin,Hex,Dec)

// 動作チェックモード
#define CHECK_EEPROM_RW 0 // 起動時のEEPROMの動作チェック

class MrdEEPROM : public I_Meridian_EEPROM {
public:
  bool protect = false;
  // EEPROM読み書き用共用体
  typedef union {
    uint8_t bval[EEPROM_SIZE];            // 1バイト単位で540個のデータを持つ
    short saval[3][int(EEPROM_SIZE / 4)]; // short型で3*90個の配列データを持つ
    short sval[int(EEPROM_SIZE / 2)];     // short型で270個のデータを持つ
  } UnionEEPROM;

private:
  const int ROM_SIZE = EEPROM_SIZE;

  bool _protect = false;
  int _size = 0;

  UnionEEPROM eeprom_write_data; // EEPROM書き込み用
  UnionEEPROM eeprom_read_data;  // EEPROM読み込み用

public:
  MrdEEPROM(int size) {
    this->_size = size;
  }
  ~MrdEEPROM() {}

public:
  bool refresh(Meridim90Union &a_meridim) override {
    return true;
  }
  bool setup() override {
    if (EEPROM.begin(this->ROM_SIZE)) {
      return true;
    }
    return false;
  }
  bool write(uint16_t address, uint8_t data) override {
    return true;
  }
  uint8_t read(uint16_t address) override { return EEPROM.read(address); }
  /// @brief EEPROMの内容を読み込んで返す.
  /// @return UnionEEPROM のフォーマットで配列を返す.
  UnionEEPROM mrd_eeprom_read() {
    UnionEEPROM read_data_tmp;
    for (int i = 0; i < EEPROM_SIZE; i++) // データを読み込む時はbyte型
    {
      read_data_tmp.bval[i] = this->read(i);
    }
    return read_data_tmp;
  }

  //================================================================================================================
  //  EEPROM関連の処理
  //================================================================================================================

  /// @brief EEPROMにEEPROM格納用の配列データを書き込む.
  /// @param a_write_data EEPROM書き込み用の配列データ.
  /// @param a_flg_protect EEPROMの書き込み許可があるかどうかのブール値.
  /// @return EEPROMの書き込みと読み込みが成功した場合はtrueを, 書き込まなかった場合はfalseを返す.
  bool mrd_eeprom_write(UnionEEPROM a_write_data, bool a_flg_protect, bool eeprom_protect) {
    if (a_flg_protect) { // EEPROM書き込み実施フラグをチェック
      return false;
    }
    if (eeprom_protect) // config.hのEEPROM書き込みプロテクトをチェック
    {
      return false;
    }

    // EEPROM書き込み
    byte old_value_tmp;                   // EEPROMにすでに書き込んであるデータ
    bool flg_renew_tmp = false;           // 書き込みコミットを実施するかのフラグ
    for (int i = 0; i < EEPROM_SIZE; i++) // データを書き込む時はbyte型
    {
      if (i >= EEPROM.length()) // EEPROMのサイズを超えないようチェック
      {
        Serial.println("Error: EEPROM address out of range.");
        return false;
      }
      old_value_tmp = EEPROM.read(i);
      // 書き込みデータがEEPROM内のデータと違う場合のみ書き込みをセット
      if (old_value_tmp != a_write_data.bval[i]) {
        EEPROM.write(i, a_write_data.bval[i]);
        flg_renew_tmp = true;
      }
    }
    if (flg_renew_tmp) // 変更箇所があれば書き込みを実施
    {
      EEPROM.commit(); // 書き込みを確定する
      Serial.print("Value updated ");
      return true;
    } else {
      Serial.print("Same value ");
    }
    return false;
  }

  /// @brief EEPROMに設定値を書き込み, その後で読み込んで内容を確認し, シリアルポートに出力する.
  /// @param a_write_data EEPROM書き込み用の配列データ.
  /// @param a_do EEPROMの読み書きチェックを実施するかのブール値.
  /// @param a_protect EEPROMの書き込み許可があるかどうかのブール値.
  /// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
  /// @return EEPROMの書き込みと読み込みが成功した場合はtrueを, それ以外はfalseを返す.
  bool write_read_check(UnionEEPROM a_write_data, bool a_do, bool a_protect, int a_bhd) {
    if (!a_do) // EEPROMの読み書きチェックを実施するか
    {
      return false;
    }

    // EEPROM書き込みを実行
    mrd_eeprom_dump_to_serial(a_write_data, a_bhd); // 書き込み内容をダンプ表示

    if (false == mrd_eeprom_write(a_write_data, a_protect)) {
      return false;
    }

    // EEPROM読み込みを実行
    UnionEEPROM read_data_tmp = mrd_eeprom_read();
    mrd_eeprom_dump_to_serial(read_data_tmp, a_bhd); // 読み込み内容をダンプ表示

    return true;
  }

  //------------------------------------------------------------------------------------
  //  各種オペレーション
  //------------------------------------------------------------------------------------

  /// @brief EEPROMから任意のshort型データを読み込む.
  /// @param index_y 配列の一次元目(0~2).
  /// @param index_x 配列の二次元目(0~89).
  /// @return short型データを返す.
  short read_short(int index_y, int index_x) {
    return short(EEPROM.read(index_y * 90 + index_x));
  }

  /// @brief EEPROMから任意のbyte型データを読み込む.
  /// @param index_y 配列の一次元目(0~2).
  /// @param index_x 配列の二次元目(0~179).
  /// @param low_high 下位ビットか上位ビットか. (0:low_bit, 1:high_bit)
  /// @return byte型データを返す.
  int8_t read_byte(int index_y, int index_x, int low_high) //
  {
    return int8_t(EEPROM.read(index_y * 180 + index_x * 2 + low_high));
  }
};

#endif // MRD_EEPROM_HPP
