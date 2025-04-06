/**
 * @file mrd_eeprom.h
 * @brief EEPROMを操作するクラス
 * @version 1.2.0
 * @date 2025-03-03
 *
 * @todo データの先頭は書き込み済みかどうかのフラグをもたせたほうがよいかもしれない
 * 理由は、起動時のデータはデフォルト値かEEPROMか判定させるようにするため
 * 工場出荷時のEEPROMの値は不定のため、どうするか考える
 * @copyright Copyright (c) 2025
 *
 */
#ifndef __MERIDIAN_EEPROM_H__
#define __MERIDIAN_EEPROM_H__

// ライブラリ導入
#include <EEPROM.h>
#include <vector>

namespace meridian {
namespace modules {
namespace plugin {

class MrdFsEEPROM {
public:
  enum Hexadecimal { // 数値表示タイプの列挙型(Bin, Hex, Dec)
    Bin = BIN,       // BIN
    Dec = DEC,       // DEC
    Hex = HEX,       // HEX
  };
  std::vector<short> data; // EEPROM書き込み用
#if 0
// TODO: flagとして定義されていたが、利用を確認できなかったのでコメントアウト
  bool eeprom_write_mode = false;       // EEPROMへの書き込みモード.
  bool eeprom_read_mode = false;        // EEPROMからの読み込みモード.
  bool eeprom_protect = EEPROM_PROTECT; // EEPROMの書き込みプロテクト.
  bool eeprom_load = EEPROM_LOAD;       // 起動時にEEPROMの内容を読み込む
  bool eeprom_set = EEPROM_SET;         // 起動時にEEPROMに規定値をセット
#endif

private:
  bool _initialed = false;       // EEPROMの初期化
  size_t _size = 0;              // EEPROMのバイト長
  bool _protected = true;        // EEPROMの書き込み保護
  int _aligned = 10;             // ダンプ表示時の１行あたりの表示個数
  std::vector<short> _read_data; // EEPROM書き込み用

public:
  explicit MrdFsEEPROM(size_t a_size) {
    this->_initialed = false;
    if (0 < a_size) {
      this->_size = a_size * 2;
      this->data.resize(this->_size);
      this->_read_data.resize(this->_size);
    }
  }
  ~MrdFsEEPROM() {
    if (this->_initialed) {
      EEPROM.end();
    }
  }

public:
  /// @brief EEPROMのプロテクト状態を設定する.
  /// @param a_protect EEPROMの書き込み許可があるかどうか
  void protect(bool a_protect) {
    this->_protected = a_protect;
  }

  /// @brief EEPROMの初期化
  /// @param a_protect EEPROMの書き込み許可があるかどうか
  /// @return 初期化が成功すればtrue, 失敗ならfalseを返す.
  bool init(bool a_protect = true) {
    bool result = false;
    Serial.print("Initializing EEPROM... ");
    if (0 < this->_size) {

#if defined(ARDUINO_TEENSY40)
      EEPROM.begin();
      result = true;
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
      EEPROM.begin(this->_size);
      result = true;
#else
      result = EEPROM.begin(this->_size);
#endif
      if (result) {
        this->read();
      }
    }
    if (result) {
      Serial.println("OK.");
    } else {
      Serial.println("Failed.");
    }
    this->_initialed = result;
    this->_protected = result ? a_protect : true; // 初期化が失敗していたら、保護状態にする
    return result;
  }

  /// @brief EEPROMの内容を読み込んで返す.
  /// @return UnionEEPROM のフォーマットで配列を返す.
  std::vector<short> read() {
    int data_size = this->_read_data.size();
    for (int i = 0; i < data_size; i++) {
      this->_read_data[i] = this->readShort(i * 2);
    }
    return this->_read_data;
  }
  /// @brief EEPROMにEEPROM格納用の配列データを書き込む.
  /// @param a_write_data EEPROM書き込み用の配列データ.
  /// @return EEPROMの書き込みと読み込みが成功した場合はtrueを, 書き込まなかった場合はfalseを返す.
  bool write(std::vector<short> a_write_data) {
    bool result = false;
    if (!this->_initialed) { // EEPROMの初期化フラグをチェック
      Serial.println("EEPROM is NOT initialized.");
      return false;
    } else if (this->_protected) { // EEPROM書き込み実施フラグをチェック
      Serial.println("EEPROM is protected. To unprotect, please set 'EEPROM_PROTECT' to false.");
      return false;
    } else {
      // EEPROM書き込み
      // bool flg_renew_tmp = false;           // 書き込みコミットを実施するかのフラグ
      short old_value_tmp; // EEPROMにすでに書き込んであるデータ
      int update_cnt = 0;
      uint16_t eeprom_length = EEPROM.length();
      int data_size = a_write_data.size();
      for (int i = 0; i < data_size && i < eeprom_length; i++) { // データを書き込む時はbyte型
        // 書き込みデータがEEPROM内のデータと違う場合のみ書き込みをセット
        old_value_tmp = this->readShort(i * 2);
        if (old_value_tmp != a_write_data.at(i)) {
          this->writeShort(i * 2, a_write_data.at(i));
          update_cnt++;
          Serial.printf("  EEPROM[%04X] != WRITE[%04X]\n", old_value_tmp, a_write_data.at(i));
        }
      }
      if (0 < update_cnt) { // 変更箇所があれば書き込みを実施
#if defined(ARDUINO_TEENSY40)
        result = true;
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
        EEPROM.commit();
        result = true;
#else
        result = EEPROM.commit();
#endif
        if (result) { // 書き込みを確定する
          this->read();
          Serial.printf("Value updated : Number of overwrites[%d]\n", update_cnt);
        } else {
          Serial.println("Error: EEPROM commit failed.");
        }
      } else {
        Serial.print("Same value ");
      }
    }
    return result;
  }
  /// @brief EEPROM格納用の配列データをシリアルにダンプ出力する.
  /// @param a_data EEPROM用の配列データ.
  /// @param a_bhd ダンプリストの表示形式.(0:Bin, 1:Hex, 2:Dec)
  /// @return 終了時にtrueを返す.
  bool print_dump(std::vector<short> a_data, Hexadecimal a_bhd = Hexadecimal::Hex) {
    int len_tmp = EEPROM.length(); // EEPROMの長さ
    Serial.printf("# EEPROM Length %d byte, %d bit Dump;\n", len_tmp, a_bhd);
    int data_size = a_data.size();
    for (int i = 0; i < data_size; i++) { // 読み込むデータはshort型で作成
      if (a_bhd == Hexadecimal::Hex) {
        Serial.printf("0x%04X", a_data.at(i) & 0xFFFF);
      } else {
        Serial.print(a_data.at(i), a_bhd);
      }
      if (0 == ((i + 1) % this->_aligned)) {
        Serial.println();
      } else {
        if (a_bhd == Hexadecimal::Hex) {
          Serial.print(" ");
        } else {
          Serial.print("/");
        }
      }
    }
    return true;
  }

  //------------------------------------------------------------------------------------
  //  各種オペレーション
  //------------------------------------------------------------------------------------

  /// @brief EEPROMから任意のshort型データを読み込む.
  /// @param index 配列のインデックス.
  /// @return short型データを返す.
  short readShort(int index) {
#if defined(ARDUINO_ESP32_DEV)
    return EEPROM.readShort(index);
#else
    return short((EEPROM.read(index) & 0xFF) | (EEPROM.read(index + 1) & 0xFF) << 8);
#endif
  }
  void writeShort(int index, short value) {
#if defined(ARDUINO_ESP32_DEV)
    EEPROM.writeShort(index, value);
#else
    EEPROM.write(index, byte(value & 0xFF));
    EEPROM.write(index + 1, byte((value >> 8) & 0xFF));
#endif
  }

  /// @brief EEPROMから任意のbyte型データを読み込む.
  /// @param index_y 配列の一次元目(0~2).
  /// @param index_x 配列の二次元目(0~179).
  /// @param low_high 下位ビットか上位ビットか. (0:low_bit, 1:high_bit)
  /// @return byte型データを返す.
  int8_t read_byte(int index_y, int index_x, int low_high) {
    return int8_t(EEPROM.read(index_y * 180 + index_x * 2 + low_high));
  }
};

} // namespace plugin
} // namespace modules
} // namespace meridian

#endif // __MERIDIAN_EEPROM_H__
