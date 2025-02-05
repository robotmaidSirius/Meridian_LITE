/**
 * @file mrd_module_eeprom.hpp
 * @brief EEPROMの読み書き処理
 * @version 1.2.0
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef __MRD_MODULE_EEPROM_HPP__
#define __MRD_MODULE_EEPROM_HPP__

// ヘッダーファイルの読み込み
#include <mrd_module/mrd_plugin/i_mrd_plugin_eeprom.hpp>

// ライブラリ導入
#include <EEPROM.h>

enum BinHexDec { // 数値表示タイプの列挙型(Bin, Hex, Dec)
  Bin = 0,       // BIN
  Hex = 1,       // HEX
  Dec = 2,       // DEC
};

#define EEPROM_DEFAULT_SIZE 540 // 使用するEEPROMのサイズ(バイト)
#define EEPROM_DUMP         0   // 起動時のEEPROM内容のダンプ表示
#define EEPROM_STYLE        Dec // 起動時のEEPROM内容のダンプ表示の書式(Bin,Hex,Dec)
#define CHECK_EEPROM_RW     0   // 起動時のEEPROMの動作チェック
#define EEPROM_PROTECT      0   // EEPROMの書き込み保護(0:保護しない, 1:書き込み禁止)
#define EEPROM_SET          0   // 起動時にEEPROMにconfig.hの内容をセット(mrd_set_eeprom)
#define EEPROM_LOAD         0   // 起動時にEEPROMの内容を諸設定にロードする(未導入)

namespace meridian {
namespace modules {
namespace plugin {

namespace eeprom {

} // namespace eeprom

class MrdEEPROM : public IMeridianEEPROM {
public:
  MrdEEPROM(int size, bool protect = true) {
    this->_size = size;
    this->_protect = protect;
  }
  ~MrdEEPROM() {}

private:
  bool _protect = false; // EEPROMの書き込み保護フラグ
  int _size = 0;

public:
  bool input(Meridim90 &a_meridim) override { return true; }
  bool output(Meridim90 &a_meridim) override { return true; }
  bool write(uint16_t address, uint8_t data) override { return true; }
  uint8_t read(uint16_t address) override { return EEPROM.read(address); }
  bool setup() override {
    if (EEPROM.begin(this->_size)) {                 // EEPROMの初期化
      this->dump_at_boot(EEPROM_DUMP, EEPROM_STYLE); // 内容のダンプ表示
      this->write_read_check(this->eeprom_setup,     // EEPROMのリードライトテスト
                             CHECK_EEPROM_RW, EEPROM_PROTECT, EEPROM_STYLE);
      return true;
    } else {
      return false;
    }
  }

public:
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  short float2HfShort(float val) {
    int _x = round(val * 100);
    if (_x > 32766) {
      _x = 32767;
    } else if (_x < -32766) {
      _x = -32767;
    }
    return static_cast<short>(_x);
  }
  //////////////////////////////////////////////////////////////////////////////
  // EEPROM読み書き用共用体
  typedef union {
    uint8_t bval[EEPROM_DEFAULT_SIZE];            // 1バイト単位で540個のデータを持つ
    short saval[3][int(EEPROM_DEFAULT_SIZE / 4)]; // short型で3*90個の配列データを持つ
    short sval[int(EEPROM_DEFAULT_SIZE / 2)];     // short型で270個のデータを持つ
  } UnionEEPROM;
  UnionEEPROM eeprom_write_data; // EEPROM書き込み用
  UnionEEPROM eeprom_read_data;  // EEPROM読み込み用
  UnionEEPROM eeprom_setup;      // EEPROM読み込み用

  /// @brief EEPROMの内容を読み込んで返す.
  /// @return UnionEEPROM のフォーマットで配列を返す.
  UnionEEPROM eeprom_read() {
    UnionEEPROM read_data_tmp;
    for (int i = 0; i < EEPROM_DEFAULT_SIZE; i++) // データを読み込む時はbyte型
    {
      read_data_tmp.bval[i] = this->read(i);
    }
    return read_data_tmp;
  }

  /// @brief EEPROM格納用の配列データをシリアルにダンプ出力する.
  /// @param a_data EEPROM用の配列データ.
  /// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
  /// @return 終了時にtrueを返す.
  bool dump_to_serial(UnionEEPROM a_data, int a_bhd) {
    int len_tmp = EEPROM.length(); // EEPROMの長さ
    this->m_diag->log_info("EEPROM Length %d byte, 16bit Dump;", len_tmp);
    for (int i = 0; i < 270; i++) // 読み込むデータはshort型で作成
    {
      if (a_bhd == 1) {
        this->m_diag->log("%d", a_data.sval[i]);
      } else {
        this->m_diag->log("%X", a_data.sval[i]);
      }
      if (0 == (i % 89)) {
        this->m_diag->log("\n");
      } else {
        this->m_diag->log("/");
      }
    }
    this->m_diag->log("\n");
    return true;
  }

  /// @brief EEPROM格納用の配列データをシリアルにダンプ出力する.(起動時用)
  /// @param a_do_dump 実施するか否か.
  /// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
  /// @return 終了時にtrueを返す.
  bool dump_at_boot(bool a_do_dump, int a_bhd) {
    if (a_do_dump) {
      dump_to_serial(eeprom_read(), a_bhd);
      return true;
    }
    return false;
  }

  /// @brief EEPROMにEEPROM格納用の配列データを書き込む.
  /// @param a_write_data EEPROM書き込み用の配列データ.
  /// @param a_flg_protect EEPROMの書き込み許可があるかどうかのブール値.
  /// @return EEPROMの書き込みと読み込みが成功した場合はtrueを, 書き込まなかった場合はfalseを返す.
  bool eeprom_write(UnionEEPROM a_write_data) {
    if (this->_protect) { // EEPROM書き込み実施フラグをチェック
      return false;
    }
    if (this->_protect) // config.hのEEPROM書き込みプロテクトをチェック
    {
      return false;
    }

    // EEPROM書き込み
    byte old_value_tmp;                           // EEPROMにすでに書き込んであるデータ
    bool flg_renew_tmp = false;                   // 書き込みコミットを実施するかのフラグ
    for (int i = 0; i < EEPROM_DEFAULT_SIZE; i++) // データを書き込む時はbyte型
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
    dump_to_serial(a_write_data, a_bhd); // 書き込み内容をダンプ表示

    if (false == eeprom_write(a_write_data)) {
      return false;
    }

    // EEPROM読み込みを実行
    UnionEEPROM read_data_tmp = eeprom_read();
    dump_to_serial(read_data_tmp, a_bhd); // 読み込み内容をダンプ表示

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

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MRD_MODULE_EEPROM_HPP__
