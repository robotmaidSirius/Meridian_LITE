/**
 * @file mrd_sd.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-20
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef MRD_SD_HPP
#define MRD_SD_HPP

#include <SD.h> // SDカード用

// ライブラリ導入
#include "mrd_plugin/i_mrd_plugin_sd.hpp"

#define MOUNT_SD    1 // SDカードリーダーの有無 (0:なし, 1:あり)
#define CHECK_SD_RW 1 // 起動時のSDカードリーダーの読み書きチェック

class MrdSD : public I_Meridian_SD {
private:
  bool _mount_sd = false;
  int _chipselect_pin = -1;
  int _test_pin = A0;
  const char *_test_file = "/test.txt";

public:
  MrdSD(int chipselect_pin) { this->_chipselect_pin = chipselect_pin; }
  ~MrdSD() {}

public:
  bool mount(bool mount_sd) {
    this->_mount_sd = mount_sd;
    return this->_mount_sd;
  }

  bool setup() override {
    if (true == this->_mount_sd) {
      if (!SD.begin(this->_chipselect_pin)) {
        return false;
      } else {
        if (1 == CHECK_SD_RW) {
          return this->sd_check();
        }
      }
    }
    return true;
  }
  bool write(uint16_t address, uint8_t data) override {
    return true;
  }
  uint8_t read(uint16_t address) override {}

  bool refresh(Meridim90Union &a_meridim) override {
    return true;
  }

  //------------------------------------------------------------------------------------
  //  リードライトテスト
  //------------------------------------------------------------------------------------

  /// @brief SDカードの読み書き機能をテストする. SDカードがマウントされ,
  /// 読み書きのチェックが要求された場合のみテストを実行する.
  /// @return SDカードの読み書きが成功した場合はtrueを, 失敗した場合はfalseを返す.
  bool sd_check() {
    bool result = false;
    if ((true == this->mount_sd)) {
      bool flag_write = false;
      File sd_file;                             // SDカード用
      uint8_t rand_number = random(0x10, 0xFF); // SD書き込みテスト用のランダムな4桁の数字を生成
      sd_file = SD.open(this->_test_file, FILE_WRITE);
      if (sd_file) {
        delay(1);                                      // SPI安定化検証用
        randomSeed(long(analogRead(this->_test_pin))); // 未接続ピンのノイズを利用
        {
          // ファイルへの書き込みを実行
          sd_file.println(rand_number);
          delayMicroseconds(1); // SPI安定化検証用
          sd_file.close();
          flag_write = true;
          delayMicroseconds(10); // SPI安定化検証用
        }
      }
      if (true == flag_write) {
        uint8_t rand_number_tmp;
        // ファイルからの読み込みを実行
        sd_file = SD.open(this->_test_file, FILE_READ);
        if (sd_file) {
          rand_number_tmp = sd_file.read();

          sd_file.close();
          if (rand_number == rand_number_tmp) {
            result = true;
          }
        }
      }
      if (true == flag_write) {
        result &= SD.remove(this->_test_file);
      }
    } else {
      result = true;
    }

    return result;
  }
};

#endif // MRD_SD_HPP
